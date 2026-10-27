//! Splitting closed, non-periodic NURBS faces into [`PIECES`] faces along generators.
//!
//! A B-spline surface that closes on itself without being periodic (an OCC `ThruSections`
//! loft, a closed extrusion of a NURBS curve) has two parameter lines, `u = lo` and `u = hi`,
//! that map to the same curve. A face bounded by that seam uses the seam edge twice, and no
//! point-to-parameter projection can tell the two uses apart. The face integrals of
//! `sm-topo-api` (`volume_moments`, `face_moments`) take parameters from projection, so such a
//! face gets a wrong, silently too small volume. Their composite Gauss rule also puts a single
//! 5-point panel across the whole parameter span of a non-periodic direction.
//!
//! The reader therefore cuts the surface (exactly, by knot insertion) into [`PIECES`] patches,
//! each open in the closed direction, and the face into one face per patch. The cuts are
//! generators (iso-curves) at equal parameter fractions, shared by two neighbouring faces, and
//! every edge that crosses a generator is split there. The solid is geometrically unchanged;
//! with four pieces the one-panel rule is accurate to about 1e-8 on a degree-13 loft.
//!
//! Only the plain configuration is handled: a face with one loop that uses one edge twice
//! (the seam), with the surface closed in exactly one direction. Anything else is left as read
//! and a `step.split_closed_face` warning says so. The solid is checked for closure once after
//! all splits, and every split is undone if it fails. Projections onto the surface are Newton
//! steps from a neighbouring point, with at most [`MAX_GLOBAL`] grid searches per face, so the
//! work is linear in the face whatever its control net. Remove this module when `sample_face`
//! honours seam pcurves (ACR 1) and the quadrature is panelled (ACR 2).

use sm_core::Point3;
use sm_geom_api::{CurveDef, Interval, NurbsCurve, NurbsSurface, ParametricSurface, SurfaceDef, Uv};
use sm_topo_api::Sense;

use std::collections::BTreeMap;

use crate::decode::check_closed;
use crate::model::{Coedge, CurveRec, Edge, Face, Note, NoteCode, Solid, SurfaceRec, Vertex};

/// Number of faces a closed face is split into.
pub(crate) const PIECES: usize = 4;
/// Samples along an edge when looking for generator crossings.
const SAMPLES: usize = 48;

type Hom = [f64; 4];
type Rows = Vec<Vec<Hom>>;
/// A generator crossing of an edge: curve parameter, cut index, new vertex.
type Cut = (f64, usize, usize);

/// Split every closed non-periodic NURBS face of `solid` that has the plain seam configuration.
///
/// One pass over the faces: each split mutates the solid in place (a failed attempt is rolled
/// back by truncating what it appended), cut edges become dead and are replaced by their pieces
/// in every loop at the end, the faces are reordered and the solid compacted once, and closure
/// is checked once. If that check fails the solid is restored and every split face is reported
/// as left as is. Work is linear in the solid plus the split faces' own size.
pub(crate) fn split_closed_nurbs(solid: &mut Solid, tol: f64, notes: &mut Vec<Note>) {
    let n0 = solid.faces.len();
    let mut snapshot: Option<Solid> = None;
    // Dead edge -> its pieces in traversal order (pieces may die in turn).
    let mut children: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    // Face -> its pieces after the first, which stays in its slot.
    let mut extra: BTreeMap<usize, Vec<Face>> = BTreeMap::new();
    let mut split_notes: Vec<usize> = Vec::new();
    for fi in 0..n0 {
        if !children.is_empty() {
            relink(&mut solid.faces[fi], &children);
        }
        let Some(dir) = candidate(solid, fi, tol) else { continue };
        let id = solid.faces[fi].id;
        if snapshot.is_none() {
            snapshot = Some(solid.clone());
        }
        match try_split(solid, fi, dir, tol, &mut children) {
            Ok(more) => {
                split_notes.push(notes.len());
                notes.push(Note::info(NoteCode::SplitClosedFace, format!("face #{id} on a closed NURBS surface split into {PIECES} faces along generators (volume and area integrals need an open parameter span)"), vec![id]));
                extra.insert(fi, more);
            }
            Err(why) => notes.push(left_as_is(id, &why)),
        }
    }
    if extra.is_empty() {
        return;
    }
    // ---- every loop on live edges, faces in order with their pieces, dead edges dropped
    for f in &mut solid.faces {
        relink(f, &children);
    }
    let mut faces = Vec::with_capacity(n0 + extra.len() * (PIECES - 1));
    let mut first = Vec::with_capacity(n0);
    for (fi, f) in std::mem::take(&mut solid.faces).into_iter().enumerate() {
        first.push(faces.len());
        faces.push(f);
        if let Some(more) = extra.remove(&fi) {
            for mut g in more {
                relink(&mut g, &children);
                faces.push(g);
            }
        }
    }
    first.push(faces.len());
    solid.faces = faces;
    for sh in &mut solid.shells {
        let mut out = Vec::with_capacity(sh.faces.len());
        for &f in &sh.faces {
            out.extend(first[f]..first[f + 1]);
        }
        sh.faces = out;
    }
    let mut new_ix = vec![usize::MAX; solid.edges.len()];
    let mut kept = Vec::with_capacity(solid.edges.len());
    for (i, e) in std::mem::take(&mut solid.edges).into_iter().enumerate() {
        if !children.contains_key(&i) {
            new_ix[i] = kept.len();
            kept.push(e);
        }
    }
    solid.edges = kept;
    for f in &mut solid.faces {
        for l in &mut f.loops {
            for c in l.iter_mut() {
                c.edge = new_ix[c.edge];
            }
        }
    }
    if let Err(e) = check_closed(solid.id, &solid.shells, &solid.faces, &solid.edges) {
        let why = format!("the split solid is not closed: {e}");
        if let Some(orig) = snapshot {
            *solid = orig;
        }
        for k in split_notes {
            let id = notes[k].step_ids.first().copied().unwrap_or(0);
            notes[k] = left_as_is(id, &why);
        }
    }
}

fn left_as_is(id: u64, why: &str) -> Note {
    Note::warn(NoteCode::SplitClosedFace, format!("face #{id} lies on a closed NURBS surface that was left as is ({why}); its area and volume integrals may be wrong"), vec![id])
}

/// Replace every coedge on a dead edge by the coedges of its live pieces.
fn relink(f: &mut Face, children: &BTreeMap<usize, Vec<usize>>) {
    for l in &mut f.loops {
        if l.iter().any(|c| children.contains_key(&c.edge)) {
            *l = l.iter().flat_map(|&c| expand(children, c)).collect();
        }
    }
}

/// Which parameter direction closes.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Dir {
    U,
    V,
}

fn seam_pair(face: &Face) -> Option<usize> {
    if face.loops.len() != 1 {
        return None;
    }
    let lp = &face.loops[0];
    lp.iter().find(|c| lp.iter().any(|d| d.edge == c.edge && d.sense != c.sense)).map(|c| c.edge)
}

fn closed_dir(s: &NurbsSurface, tol: f64) -> Option<Dir> {
    if s.periodic != [false, false] {
        return None;
    }
    let (du, dv) = (Interval::new(s.knots_u[s.degree_u], s.knots_u[s.nu]), Interval::new(s.knots_v[s.degree_v], s.knots_v[s.nv]));
    let along = |a: Interval, b: Interval, flip: bool| {
        (0..=8).all(|k| {
            let x = b.lo + b.len() * k as f64 / 8.0;
            let (p0, p1) = if flip { (s.jet(Uv::new(x, a.lo)).p, s.jet(Uv::new(x, a.hi)).p) } else { (s.jet(Uv::new(a.lo, x)).p, s.jet(Uv::new(a.hi, x)).p) };
            (p0 - p1).norm() <= tol
        })
    };
    match (along(du, dv, false), along(dv, du, true)) {
        (true, false) => Some(Dir::U),
        (false, true) => Some(Dir::V),
        _ => None,
    }
}

fn candidate(solid: &Solid, fi: usize, tol: f64) -> Option<Dir> {
    let f = &solid.faces[fi];
    let SurfaceDef::Nurbs(s) = &solid.surfaces[f.surface].def else { return None };
    seam_pair(f)?;
    closed_dir(s, tol)
}

fn transpose(s: &NurbsSurface) -> Result<NurbsSurface, String> {
    let (nu, nv) = (s.nu, s.nv);
    let mut control = vec![Point3::new(0.0, 0.0, 0.0); nu * nv];
    let mut weights = s.weights.as_ref().map(|_| vec![0.0; nu * nv]);
    for i in 0..nu {
        for j in 0..nv {
            control[j * nu + i] = s.control[i * nv + j];
            if let (Some(w), Some(src)) = (&mut weights, &s.weights) {
                w[j * nu + i] = src[i * nv + j];
            }
        }
    }
    NurbsSurface::new(s.degree_v, s.degree_u, s.knots_v.clone(), s.knots_u.clone(), nv, nu, control, weights).map_err(|e| e.to_string())
}

fn rows_of(s: &NurbsSurface) -> Rows {
    (0..s.nu)
        .map(|i| {
            (0..s.nv)
                .map(|j| {
                    let w = s.weights.as_ref().map_or(1.0, |w| w[i * s.nv + j]);
                    let p = s.control[i * s.nv + j];
                    [p.x * w, p.y * w, p.z * w, w]
                })
                .collect()
        })
        .collect()
}

/// A5.1 of The NURBS Book: insert the knot `t` once into the `u` direction.
fn insert_knot(deg: usize, knots: &mut Vec<f64>, rows: &mut Rows, t: f64) {
    let n = rows.len() - 1;
    let mut k = deg;
    while k < n && knots[k + 1] <= t {
        k += 1;
    }
    let s = knots.iter().filter(|&&x| x == t).count();
    let mut q: Rows = Vec::with_capacity(n + 2);
    for (i, r) in rows.iter().enumerate().take(k - deg + 1) {
        debug_assert_eq!(i, q.len());
        q.push(r.clone());
    }
    for i in (k - deg + 1)..=(k - s) {
        let a = (t - knots[i]) / (knots[i + deg] - knots[i]);
        let row: Vec<Hom> = rows[i].iter().zip(&rows[i - 1]).map(|(p, m)| [0, 1, 2, 3].map(|c| a * p[c] + (1.0 - a) * m[c])).collect();
        q.push(row);
    }
    for r in rows.iter().skip(k - s) {
        q.push(r.clone());
    }
    knots.insert(k + 1, t);
    *rows = q;
}

fn surface_of(du: usize, dv: usize, knots_u: Vec<f64>, knots_v: &[f64], rows: &[Vec<Hom>], rational: bool) -> Result<NurbsSurface, String> {
    let (nu, nv) = (rows.len(), rows[0].len());
    let mut control = Vec::with_capacity(nu * nv);
    let mut weights = Vec::with_capacity(nu * nv);
    for r in rows {
        for h in r {
            control.push(Point3::new(h[0] / h[3], h[1] / h[3], h[2] / h[3]));
            weights.push(h[3]);
        }
    }
    NurbsSurface::new(du, dv, knots_u, knots_v.to_vec(), nu, nv, control, rational.then_some(weights)).map_err(|e| e.to_string())
}

/// Cut `s` (closed in `u`) at `params` (ascending, interior). Returns the patches and, for each
/// cut, the control polygon of its generator as a `v`-direction curve.
fn cut(s: &NurbsSurface, params: &[f64]) -> Result<(Vec<NurbsSurface>, Vec<NurbsCurve>), String> {
    let deg = s.degree_u;
    let rational = s.weights.is_some();
    let mut knots = s.knots_u.clone();
    let mut rows = rows_of(s);
    let mut pieces = Vec::new();
    let mut gens = Vec::new();
    for &t in params {
        let have = knots.iter().filter(|&&x| x == t).count();
        for _ in have..=deg {
            insert_knot(deg, &mut knots, &mut rows, t);
        }
        let a = knots.iter().position(|&x| x == t).ok_or("knot insertion failed")?;
        pieces.push(surface_of(deg, s.degree_v, knots[..=a + deg].to_vec(), &s.knots_v, &rows[..a], rational)?);
        let g = &rows[a];
        let control: Vec<Point3> = g.iter().map(|h| Point3::new(h[0] / h[3], h[1] / h[3], h[2] / h[3])).collect();
        let weights: Vec<f64> = g.iter().map(|h| h[3]).collect();
        gens.push(NurbsCurve::new(s.degree_v, s.knots_v.clone(), control, rational.then_some(weights)).map_err(|e| e.to_string())?);
        knots = knots[a..].to_vec();
        rows = rows[a..].to_vec();
    }
    pieces.push(surface_of(deg, s.degree_v, knots, &s.knots_v, &rows, rational)?);
    Ok((pieces, gens))
}

/// Gauss-Newton projection of `p` onto `surf` from `uv0` (no global search).
fn local_project(surf: &dyn ParametricSurface, p: Point3, uv0: Uv, tol: f64) -> Option<Uv> {
    let dom = surf.domain();
    let (mut u, mut v) = (uv0.u, uv0.v);
    for _ in 0..50 {
        let j = surf.jet(Uv::new(u, v));
        let r = j.p - p;
        let (a, b, c) = (j.du.dot(j.du), j.du.dot(j.dv), j.dv.dot(j.dv));
        let (g0, g1) = (r.dot(j.du), r.dot(j.dv));
        let det = a * c - b * b;
        if det.abs() < 1e-300 {
            return None;
        }
        let (su, sv) = ((c * g0 - b * g1) / det, (a * g1 - b * g0) / det);
        u = dom.u.range.clamp(u - su);
        v = dom.v.range.clamp(v - sv);
        if su.abs() + sv.abs() < 1e-14 {
            break;
        }
    }
    ((surf.jet(Uv::new(u, v)).p - p).norm() <= tol).then_some(Uv::new(u, v))
}

/// Global (grid search) projections allowed per split face. Each costs about `64 nu nv` surface
/// evaluations (degree at most `MAX_DEGREE`), so a face costs a constant times its surface size
/// however many edges bound it; every other projection is Newton from a nearby projected point.
const MAX_GLOBAL: usize = 8;

/// Projection onto the surface of one split face: local Newton from a seed, with a capped number
/// of global searches as the fallback.
struct Proj<'a> {
    surf: &'a NurbsSurface,
    tol: f64,
    global_left: usize,
}

impl Proj<'_> {
    /// Project `p` from `seed` when there is one, else (or when Newton fails) by a global search.
    fn project(&mut self, p: Point3, seed: Option<Uv>) -> Result<Uv, String> {
        if let Some(uv) = seed.and_then(|s| local_project(self.surf, p, s, self.tol)) {
            return Ok(uv);
        }
        if self.global_left == 0 {
            return Err("too many boundary points needed a global projection".into());
        }
        self.global_left -= 1;
        Ok(self.surf.project(p, seed).map_err(|e| e.to_string())?.uv)
    }
}

/// The samples of one edge: curve parameters and their surface parameters.
struct Track {
    ts: Vec<f64>,
    uvs: Vec<Uv>,
}

impl Track {
    /// Surface parameters of the curve point at `t`, seeded from the nearest sample.
    fn uv_at(&self, proj: &mut Proj, curve: &dyn sm_geom_api::ParametricCurve, t: f64) -> Result<Uv, String> {
        let near = self.ts.iter().zip(&self.uvs).min_by(|a, b| (a.0 - t).abs().total_cmp(&(b.0 - t).abs())).map(|(_, &uv)| uv);
        proj.project(curve.eval(t), near)
    }
}

/// Parameters `t` on the edge (with the cut index) where the edge curve crosses a generator,
/// and the edge's samples. Samples are projected in traversal order (`backward`: from the high
/// end), the first from `seed` (the previous edge's last sample), each next from its neighbour.
fn crossings(proj: &mut Proj, curve: &dyn sm_geom_api::ParametricCurve, range: Interval, us: &[f64], seed: Option<Uv>, backward: bool) -> Result<(Vec<(f64, usize)>, Track), String> {
    let ts: Vec<f64> = (0..SAMPLES).map(|i| range.lo + range.len() * (i as f64 + 0.5) / SAMPLES as f64).collect();
    let mut uvs: Vec<Uv> = vec![Uv::new(0.0, 0.0); SAMPLES];
    let mut prev = seed;
    for k in 0..SAMPLES {
        let i = if backward { SAMPLES - 1 - k } else { k };
        let p = curve.eval(ts[i]);
        let uv = proj.project(p, prev)?;
        if (proj.surf.jet(uv).p - p).norm() > proj.tol {
            return Err("an edge does not lie on the surface".into());
        }
        uvs[i] = uv;
        prev = Some(uv);
    }
    let (surf, tol) = (proj.surf, proj.tol);
    let mut out = Vec::new();
    for (k, &uk) in us.iter().enumerate() {
        for i in 0..SAMPLES - 1 {
            let (da, db) = (uvs[i].u - uk, uvs[i + 1].u - uk);
            if da * db >= 0.0 {
                continue;
            }
            let (mut ta, mut tb, mut uva) = (ts[i], ts[i + 1], uvs[i]);
            for _ in 0..80 {
                let tm = 0.5 * (ta + tb);
                let uvm = local_project(surf, curve.eval(tm), uva, tol).ok_or("projection failed while locating a generator crossing")?;
                if (uvm.u - uk) * da > 0.0 {
                    (ta, uva) = (tm, uvm);
                } else {
                    tb = tm;
                }
                if tb - ta <= 1e-14 * (1.0 + tb.abs()) {
                    break;
                }
            }
            out.push((0.5 * (ta + tb), k));
        }
    }
    Ok((out, Track { ts, uvs }))
}

/// One use of a (sub-)edge in the expanded loop.
#[derive(Clone, Copy)]
struct Item {
    c: Coedge,
    piece: usize,
    from: usize,
    to: usize,
}

/// The coedges a coedge becomes when its edge was split (itself if not), in traversal order,
/// through every level of splitting.
fn expand(sub: &BTreeMap<usize, Vec<usize>>, c: Coedge) -> Vec<Coedge> {
    let mut v = Vec::new();
    let mut stack = vec![c.edge];
    while let Some(e) = stack.pop() {
        match sub.get(&e) {
            Some(t) => stack.extend(t.iter().rev()),
            None => v.push(Coedge { edge: e, sense: c.sense }),
        }
    }
    if c.sense == Sense::Reversed {
        v.reverse();
    }
    v
}

/// Split face `fi` of `s` in place. Returns the pieces after the first (which replaces the face
/// in its slot) and records the cut edges, now dead, in `children`. On failure everything the
/// attempt appended is truncated away and nothing else has changed.
fn try_split(s: &mut Solid, fi: usize, dir: Dir, tol: f64, children: &mut BTreeMap<usize, Vec<usize>>) -> Result<Vec<Face>, String> {
    let lens = (s.vertices.len(), s.curves.len(), s.edges.len(), s.surfaces.len());
    let r = split_face(s, fi, dir, tol, children);
    if r.is_err() {
        s.vertices.truncate(lens.0);
        s.curves.truncate(lens.1);
        s.edges.truncate(lens.2);
        s.surfaces.truncate(lens.3);
    }
    r
}

fn split_face(s: &mut Solid, fi: usize, dir: Dir, tol: f64, children: &mut BTreeMap<usize, Vec<usize>>) -> Result<Vec<Face>, String> {
    let face = s.faces[fi].clone();
    let SurfaceDef::Nurbs(orig) = &s.surfaces[face.surface].def else { return Err("not a NURBS surface".into()) };
    let orig = orig.clone();
    let base = if dir == Dir::U { orig.clone() } else { transpose(&orig)? };
    let (ulo, uhi) = (base.knots_u[base.degree_u], base.knots_u[base.nu]);
    let us: Vec<f64> = (1..PIECES).map(|k| ulo + (uhi - ulo) * k as f64 / PIECES as f64).collect();
    let (base_pieces, gen_curves) = cut(&base, &us)?;
    let piece_defs: Vec<SurfaceDef> = base_pieces.into_iter().map(|bp| Ok(SurfaceDef::Nurbs(if dir == Dir::U { bp } else { transpose(&bp)? }))).collect::<Result<_, String>>()?;
    let lp = face.loops[0].clone();
    let seam = seam_pair(&face).ok_or("no seam pair")?;
    if lp.iter().filter(|c| c.edge == seam).count() != 2 {
        return Err("the seam edge is used more than twice".into());
    }
    let mut ids: Vec<usize> = lp.iter().map(|c| c.edge).filter(|&e| e != seam).collect();
    ids.sort_unstable();
    if ids.windows(2).any(|w| w[0] == w[1]) {
        return Err("another edge appears twice in the loop".into());
    }
    let etol = tol.max(1e-6);
    let mut proj = Proj { surf: &base, tol: etol * 10.0, global_left: MAX_GLOBAL };
    // ---- project the edges in loop order, each seeded from the previous one
    let mut tracks: BTreeMap<usize, (Vec<(f64, usize)>, Track)> = BTreeMap::new();
    let mut seed: Option<Uv> = None;
    for c in lp.iter().filter(|c| c.edge != seam) {
        let e = &s.edges[c.edge];
        let backward = (c.sense == Sense::Forward) != (e.sense == Sense::Forward);
        let (cs, tr) = crossings(&mut proj, s.curves[e.curve].def.as_curve(), e.range, &us, seed, backward)?;
        seed = Some(if backward { tr.uvs[0] } else { tr.uvs[SAMPLES - 1] });
        tracks.insert(c.edge, (cs, tr));
    }
    // ---- cut the edges that cross a generator (new vertices in edge order)
    let mut cuts: Vec<(usize, Vec<Cut>)> = Vec::new();
    let mut cut_vertices: Vec<Vec<usize>> = vec![Vec::new(); PIECES - 1];
    for &ei in &ids {
        let e = s.edges[ei].clone();
        let curve = s.curves[e.curve].def.as_curve();
        let mut cs = tracks.get(&ei).map(|t| t.0.clone()).unwrap_or_default();
        cs.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut with_v = Vec::new();
        for (t, k) in cs {
            let vi = s.vertices.len();
            s.vertices.push(Vertex { id: e.id, point: curve.eval(t) });
            cut_vertices[k].push(vi);
            with_v.push((t, k, vi));
        }
        if !with_v.is_empty() {
            cuts.push((ei, with_v));
        }
    }
    if cut_vertices.iter().any(|v| v.len() != 2) {
        return Err("a generator does not cross the boundary exactly twice".into());
    }
    // ---- generator edges, bottom (v = lo) to top
    let vdom = Interval::new(base.knots_v[base.degree_v], base.knots_v[base.nv]);
    let mut gens: Vec<(usize, usize, usize)> = Vec::new(); // (bottom, top, edge)
    for (k, g) in gen_curves.into_iter().enumerate() {
        let (pb, pt) = (base.jet(Uv::new(us[k], vdom.lo)).p, base.jet(Uv::new(us[k], vdom.hi)).p);
        let (v0, v1) = (cut_vertices[k][0], cut_vertices[k][1]);
        let d = |vi: usize, p: Point3| (s.vertices[vi].point - p).norm();
        let (bot, top) = if d(v0, pb) + d(v1, pt) <= d(v1, pb) + d(v0, pt) { (v0, v1) } else { (v1, v0) };
        if d(bot, pb) > etol * 10.0 || d(top, pt) > etol * 10.0 {
            return Err("generator ends do not meet the cut vertices".into());
        }
        let ci = s.curves.len();
        s.curves.push(CurveRec { id: s.surfaces[face.surface].id, def: CurveDef::Nurbs(g) });
        let ei = s.edges.len();
        s.edges.push(Edge { id: face.id, start: bot, end: top, curve: ci, range: vdom, sense: Sense::Forward, merged: Vec::new() });
        gens.push((bot, top, ei));
    }
    // ---- split the cut edges (all pieces new; the cut edge dies on success); `sub[e]` lists
    // the sub-edges in traversal order
    let mut sub: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (ei, list) in &cuts {
        let e = s.edges[*ei].clone();
        let m = list.len();
        let mut bounds = vec![e.range.lo];
        bounds.extend(list.iter().map(|c| c.0));
        bounds.push(e.range.hi);
        let vtx = |i: usize| list[i - 1].2;
        let mut asc: Vec<usize> = Vec::with_capacity(m + 1);
        for i in 0..=m {
            let (start, end) = if e.sense == Sense::Forward {
                (if i == 0 { e.start } else { vtx(i) }, if i == m { e.end } else { vtx(i + 1) })
            } else {
                (if i == m { e.start } else { vtx(i + 1) }, if i == 0 { e.end } else { vtx(i) })
            };
            let ne = Edge { id: e.id, start, end, curve: e.curve, range: Interval::new(bounds[i], bounds[i + 1]), sense: e.sense, merged: if i == 0 { e.merged.clone() } else { Vec::new() } };
            asc.push(s.edges.len());
            s.edges.push(ne);
        }
        if e.sense == Sense::Reversed {
            asc.reverse();
        }
        sub.insert(*ei, asc);
    }
    // ---- the expanded boundary of the lateral face with its pieces
    // The original edge of each sub-edge, whose samples seed the midpoint projection.
    let mut parent: BTreeMap<usize, usize> = BTreeMap::new();
    for (o, list) in &sub {
        for &x in list {
            parent.insert(x, *o);
        }
    }
    let mut piece_of_edge = |s: &Solid, ei: usize| -> Result<usize, String> {
        let e = &s.edges[ei];
        let curve = s.curves[e.curve].def.as_curve();
        let t = 0.5 * (e.range.lo + e.range.hi);
        let uv = match tracks.get(&parent.get(&ei).copied().unwrap_or(ei)) {
            Some((_, tr)) => tr.uv_at(&mut proj, curve, t)?,
            None => proj.project(curve.eval(t), None)?,
        };
        Ok((((uv.u - ulo) / (uhi - ulo) * PIECES as f64).floor().max(0.0) as usize).min(PIECES - 1))
    };
    let ends = |s: &Solid, c: Coedge| {
        let e = &s.edges[c.edge];
        if c.sense == Sense::Forward { (e.start, e.end) } else { (e.end, e.start) }
    };
    let mut items: Vec<Item> = Vec::new();
    for &c in &lp {
        for sc in expand(&sub, c) {
            let (from, to) = ends(s, sc);
            let piece = if sc.edge == seam { usize::MAX } else { piece_of_edge(s, sc.edge)? };
            items.push(Item { c: sc, piece, from, to });
        }
    }
    let n = items.len();
    for i in 0..n {
        if items[i].piece == usize::MAX {
            let (p, q) = (items[(i + n - 1) % n].piece, items[(i + 1) % n].piece);
            if p != q || p == usize::MAX || (p != 0 && p != PIECES - 1) {
                return Err("the seam is not between edges of the first or last piece".into());
            }
            items[i].piece = p;
        }
    }
    let start = (0..n).find(|&i| items[i].piece != items[(i + n - 1) % n].piece).ok_or("the boundary stays inside one piece")?;
    let mut runs: Vec<Vec<Item>> = Vec::new();
    for k in 0..n {
        let it = items[(start + k) % n];
        match runs.last_mut() {
            Some(r) if r[0].piece == it.piece => r.push(it),
            _ => runs.push(vec![it]),
        }
    }
    let connector = |a: usize, b: usize| -> Result<Coedge, String> {
        gens.iter()
            .find_map(|&(bot, top, e)| match (a == bot && b == top, a == top && b == bot) {
                (true, _) => Some(Coedge { edge: e, sense: Sense::Forward }),
                (_, true) => Some(Coedge { edge: e, sense: Sense::Reversed }),
                _ => None,
            })
            .ok_or_else(|| "no generator joins two runs of a piece".to_owned())
    };
    let mut piece_loops: Vec<Vec<Coedge>> = Vec::new();
    for p in 0..PIECES {
        let mine: Vec<&Vec<Item>> = runs.iter().filter(|r| r[0].piece == p).collect();
        if mine.is_empty() {
            return Err("a piece has no boundary".into());
        }
        let mut lp_p = Vec::new();
        for (ri, r) in mine.iter().enumerate() {
            lp_p.extend(r.iter().map(|i| i.c));
            let next = mine[(ri + 1) % mine.len()];
            let (a, b) = (r[r.len() - 1].to, next[0].from);
            if a != b {
                lp_p.push(connector(a, b)?);
            }
        }
        piece_loops.push(lp_p);
    }
    // ---- commit: no failure past this point
    children.extend(sub);
    let sid = s.surfaces[face.surface].id;
    let mut surf_ix = Vec::with_capacity(PIECES);
    for (p, def) in piece_defs.into_iter().enumerate() {
        if p == 0 {
            s.surfaces[face.surface] = SurfaceRec { id: sid, def };
            surf_ix.push(face.surface);
        } else {
            surf_ix.push(s.surfaces.len());
            s.surfaces.push(SurfaceRec { id: sid, def });
        }
    }
    let mut pieces = piece_loops.into_iter().enumerate().map(|(p, l)| Face { id: face.id, surface: surf_ix[p], sense: face.sense, loops: vec![l], shell: face.shell });
    if let Some(f0) = pieces.next() {
        s.faces[fi] = f0;
    }
    Ok(pieces.collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A closed (in `u`) degree-3 rational cylinder-like patch: nine control columns of a
    /// quarter-circle based circle, two rows.
    fn cylinder() -> NurbsSurface {
        let w = core::f64::consts::FRAC_1_SQRT_2;
        let ring = [(1.0, 0.0, 1.0), (1.0, 1.0, w), (0.0, 1.0, 1.0), (-1.0, 1.0, w), (-1.0, 0.0, 1.0), (-1.0, -1.0, w), (0.0, -1.0, 1.0), (1.0, -1.0, w), (1.0, 0.0, 1.0)];
        let mut control = Vec::new();
        let mut weights = Vec::new();
        for (x, y, wt) in ring {
            for z in [0.0, 2.0] {
                control.push(Point3::new(x, y, z));
                weights.push(wt);
            }
        }
        let knots_u = vec![0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0];
        NurbsSurface::new(2, 1, knots_u, vec![0.0, 0.0, 1.0, 1.0], 9, 2, control, Some(weights)).expect("surface")
    }

    #[test]
    fn closed_direction_is_found_in_either_orientation() {
        let s = cylinder();
        assert!(matches!(closed_dir(&s, 1e-9), Some(Dir::U)));
        let t = transpose(&s).expect("transpose");
        assert!(matches!(closed_dir(&t, 1e-9), Some(Dir::V)));
    }

    #[test]
    fn knot_insertion_cuts_exactly() {
        let s = cylinder();
        let (pieces, gens) = cut(&s, &[0.2, 0.5, 0.8]).expect("cut");
        assert_eq!((pieces.len(), gens.len()), (4, 3));
        let bounds = [0.0, 0.2, 0.5, 0.8, 1.0];
        for (k, p) in pieces.iter().enumerate() {
            for i in 0..=10 {
                for j in 0..=2 {
                    let u = bounds[k] + (bounds[k + 1] - bounds[k]) * i as f64 / 10.0;
                    let v = 0.5 * j as f64;
                    let d = (p.jet(Uv::new(u, v)).p - s.jet(Uv::new(u, v)).p).norm();
                    assert!(d < 1e-13, "piece {k} u={u}: {d:e}");
                }
            }
        }
        // The generators are the iso-curves.
        for (g, u) in gens.iter().zip([0.2, 0.5, 0.8]) {
            use sm_geom_api::ParametricCurve;
            for j in 0..=4 {
                let v = j as f64 / 4.0;
                assert!((g.eval(v) - s.jet(Uv::new(u, v)).p).norm() < 1e-13);
            }
        }
    }
}
