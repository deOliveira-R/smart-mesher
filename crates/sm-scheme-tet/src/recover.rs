//! Boundary recovery without boundary Steiner points.
//!
//! Works on the Delaunay triangulation of the input points plus eight far auxiliary points
//! (see `DESIGN.md`). Constraint edges are recovered first, by flips that remove the edges
//! crossing the segment (n-to-m edge removal and 2-3 flips); then constraint triangles by
//! removing the tets they cross and re-tetrahedralizing the two cavities (gift wrapping with an
//! empty-sphere preference). A cavity that cannot be filled gets interior Steiner points chosen
//! where the fill gets stuck (see `DESIGN.md`); a triangle cavity that stays unfillable is
//! starred from a kernel point.
//!
//! All crossing decisions use symbolic perturbation (generic position); every tet created has
//! strictly positive exact orientation.

use crate::delaunay::Dt;
use crate::error::TetError;
use crate::fill::Seeds;
use crate::flips::FlipScratch;
use crate::geom::{Pts, V3, add, dot, ori, sanitize, scale, sub};
use crate::guard::Guard;
use crate::tds::{FACE, NONE, sorted3};

/// Constraint edges and triangles with their owners.
#[derive(Clone)]
pub(crate) struct Constraints {
    pub edges: Vec<([u32; 2], u32)>,
    pub faces: Vec<([u32; 3], u32)>,
    /// The input triangles as listed (outward oriented).
    pub tris: Vec<[u32; 3]>,
    /// `(edge, triangle)` for every triangle edge, sorted: the triangles around an edge.
    edge_tris: Vec<([u32; 2], u32)>,
    /// `(vertex, triangle)` for every triangle corner, sorted: the triangles around a vertex.
    vert_tris: Vec<(u32, u32)>,
}

impl Constraints {
    pub fn new(tris: &[[u32; 3]]) -> Self {
        let mut edges: Vec<([u32; 2], u32)> = Vec::with_capacity(tris.len() * 3 / 2);
        let mut faces: Vec<([u32; 3], u32)> = Vec::with_capacity(tris.len());
        for (i, t) in tris.iter().enumerate() {
            faces.push((sorted3(t[0], t[1], t[2]), i as u32));
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                edges.push(([a.min(b), a.max(b)], i as u32));
            }
        }
        let mut edge_tris = edges.clone();
        edge_tris.sort_unstable();
        edges.sort_unstable();
        edges.dedup_by_key(|e| e.0);
        faces.sort_unstable();
        let mut vert_tris: Vec<(u32, u32)> = tris.iter().enumerate().flat_map(|(i, t)| t.map(|v| (v, i as u32))).collect();
        vert_tris.sort_unstable();
        Self { edges, faces, tris: tris.to_vec(), edge_tris, vert_tris }
    }
    /// The triangles around vertex `v`.
    pub fn tris_of_vertex(&self, v: u32) -> impl Iterator<Item = u32> + '_ {
        let lo = self.vert_tris.partition_point(|e| e.0 < v);
        let hi = self.vert_tris.partition_point(|e| e.0 <= v);
        self.vert_tris[lo..hi].iter().map(|e| e.1)
    }
    /// The (up to two) triangles that have edge `(a,b)`.
    pub fn tris_of_edge(&self, a: u32, b: u32) -> &[([u32; 2], u32)] {
        let k = [a.min(b), a.max(b)];
        let lo = self.edge_tris.partition_point(|e| e.0 < k);
        let hi = self.edge_tris.partition_point(|e| e.0 <= k);
        &self.edge_tris[lo..hi]
    }
    pub fn edge_owner(&self, a: u32, b: u32) -> Option<u32> {
        let k = [a.min(b), a.max(b)];
        self.edges.binary_search_by(|e| e.0.cmp(&k)).ok().map(|i| self.edges[i].1)
    }
    pub fn face_owner(&self, k: [u32; 3]) -> Option<u32> {
        self.faces.binary_search_by(|e| e.0.cmp(&k)).ok().map(|i| self.faces[i].1)
    }
}

/// Recovery driver.
pub(crate) struct Recover<'a> {
    pub dt: &'a mut Dt,
    /// The input constraints (lookups go through [`Recover::edge_owner`] and
    /// [`Recover::face_owner`], which in complete mode read the current subdivision instead).
    pub con: &'a Constraints,
    /// The input constraints.
    pub orig: &'a Constraints,
    /// Complete mode: missing edges and triangles get temporary boundary Steiner points
    /// instead of interior-point searches (see [`crate::steiner`]).
    pub complete_mode: bool,
    /// The surface subdivision of complete mode.
    pub bnd: crate::steiner::Bnd,
    /// Temporary boundary Steiner points created (and later suppressed) in complete mode.
    pub n_boundary_steiner: usize,
    pub flips: usize,
    pub max_steiner: usize,
    pub n_steiner: usize,
    /// Points below this index are input points (the far points follow them).
    pub n_real: u32,
    /// Interior points suggested by failed recoveries (for the caller's restart).
    pub hints: Vec<V3>,
    /// Restart number (selects which hint candidates are produced).
    pub attempt: usize,
    pub(crate) guard: &'a Guard<'a>,
    /// Edges that failed in this pass; after [`SEARCH_EDGES`] of them the expensive searches
    /// with interior points are skipped (the pass only collects hints).
    failed_edges: usize,
    marks: Marks,
    sc: FlipScratch,
    buf: Vec<u32>,
    tabu: Vec<[u32; 2]>,
}

/// Epoch-marked tet sets: O(1) membership without clearing.
#[derive(Default)]
struct Marks {
    a: Vec<u32>,
    b: Vec<u32>,
    ea: u32,
    eb: u32,
}

impl Marks {
    /// Start fresh sets covering `slots` tet slots.
    fn reset(&mut self, slots: usize) {
        if self.a.len() < slots {
            self.a.resize(slots, 0);
            self.b.resize(slots, 0);
        }
        self.ea += 1;
        self.eb += 1;
    }
    fn in_a(&self, t: u32) -> bool {
        self.a.get(t as usize) == Some(&self.ea)
    }
    fn in_b(&self, t: u32) -> bool {
        self.b.get(t as usize) == Some(&self.eb)
    }
    /// Slots created after `reset` (flips during triangle recovery) grow the arrays on demand.
    fn grow(&mut self, t: u32) {
        let i = t as usize;
        if i >= self.a.len() {
            self.a.resize(i + 64, 0);
            self.b.resize(i + 64, 0);
        }
    }
    fn add_a(&mut self, t: u32) {
        self.grow(t);
        self.a[t as usize] = self.ea;
    }
    fn add_b(&mut self, t: u32) {
        self.grow(t);
        self.b[t as usize] = self.eb;
    }
}

/// How many times an edge cavity may grow across the faces that block its refill.
const MAX_GROW: usize = 8;

/// Cavities above this many tets are not grown or searched with interior points any further.
const MAX_CAVITY: usize = 80;

/// Layers of neighbours (around the grown cavity) over which interior points are tried.
const STEINER_LAYERS: usize = 3;

/// Surface components with at most this many vertices count as one pocket.
const POCKET_VERTS: usize = 64;

/// Failed edges per pass after which the searches (cavity growth, interior points) stop.
const SEARCH_EDGES: usize = 3;

/// Fill predicate work (see [`Dt::work`]) per pass after which the interior-point searches stop
/// and what is still missing is left to complete recovery ([`crate::steiner`]). Successful
/// searches on the test families used up to about 2.2 million units; stuck ones on the hard
/// relief cases used tens to hundreds of millions (seconds to minutes).
const SEARCH_WORK: u64 = 2_500_000;

/// A cavity around a missing edge: boundary faces (outward), the tet beyond each (sorted by
/// face key; `NONE` for a kept constraint face), and enclosed vertices.
struct Cavity {
    faces: Vec<[u32; 3]>,
    outside: Vec<([u32; 3], u32)>,
    inner: Vec<u32>,
}


/// Tets crossed by a segment and the crossed faces as `(tet, slot)`.
type Corridor = (Vec<u32>, Vec<(u32, usize)>);

fn fail(tri: u32, edge: Option<[u32; 2]>, reason: impl Into<String>) -> TetError {
    TetError::RecoveryFailed { tri, edge, reason: reason.into() }
}

impl<'a> Recover<'a> {
    pub fn new(dt: &'a mut Dt, con: &'a Constraints, guard: &'a Guard<'a>, max_steiner: usize, n_real: u32, attempt: usize) -> Self {
        dt.search_limit = dt.work.load(std::sync::atomic::Ordering::Relaxed).saturating_add(SEARCH_WORK);
        Self { dt, con, orig: con, complete_mode: false, bnd: crate::steiner::Bnd::default(), n_boundary_steiner: 0, guard, marks: Marks::default(), flips: 0, max_steiner, n_steiner: 0, n_real, hints: Vec::new(), attempt, failed_edges: 0, sc: FlipScratch::default(), buf: Vec::new(), tabu: Vec::new() }
    }

    /// Input triangle owning constraint edge `(a,b)`: an input edge, or in complete mode a
    /// subsegment of one or an edge of a recovered tile (the smallest owner).
    #[inline]
    pub(crate) fn edge_owner(&self, a: u32, b: u32) -> Option<u32> {
        if self.complete_mode { self.bnd.edge_owner(self.orig, a, b) } else { self.con.edge_owner(a, b) }
    }

    /// Input triangle owning constraint face `k` (sorted): an input triangle, or in complete
    /// mode a recovered tile of one.
    #[inline]
    pub(crate) fn face_owner(&self, k: [u32; 3]) -> Option<u32> {
        if self.complete_mode { self.bnd.face_owner(k) } else { self.con.face_owner(k) }
    }

    /// Recover every constraint edge and triangle. A recovery failure does not stop the pass:
    /// every failing constraint contributes interior point hints ([`Recover::hints`]) and the
    /// first failure is returned at the end.
    pub fn run(&mut self) -> Result<(), TetError> {
        let mut first: Option<TetError> = None;
        let note = |first: &mut Option<TetError>, e: TetError| -> Result<(), TetError> {
            match e {
                e @ TetError::RecoveryFailed { .. } => {
                    first.get_or_insert(e);
                    Ok(())
                }
                other => Err(other),
            }
        };
        for ei in 0..self.con.edges.len() {
            let (e, owner) = self.con.edges[ei];
            if let Err(err) = self.ensure_edge(e[0], e[1], owner) {
                note(&mut first, err)?;
            }
        }
        if first.is_some() {
            return Err(first.unwrap_or_else(|| fail(0, None, "edge recovery failed")));
        }
        for _round in 0..8 {
            let mut changed = false;
            for fi in 0..self.con.faces.len() {
                let (k, tri) = self.con.faces[fi];
                if !self.face_present(k) {
                    if let Err(err) = self.recover_face(tri, k) {
                        note(&mut first, err)?;
                    }
                    changed = true;
                }
            }
            if first.is_some() {
                break;
            }
            for ei in 0..self.con.edges.len() {
                let (e, owner) = self.con.edges[ei];
                if self.dt.tds.find_edge(e[0], e[1]).is_none() {
                    if let Err(err) = self.ensure_edge(e[0], e[1], owner) {
                        note(&mut first, err)?;
                    }
                    changed = true;
                }
            }
            if first.is_some() || !changed {
                break;
            }
        }
        if let Some(e) = first {
            return Err(e);
        }
        let con = self.orig;
        for &(k, tri) in &con.faces {
            if !self.face_present(k) {
                return Err(fail(tri, None, "triangle still missing after recovery rounds"));
            }
        }
        Ok(())
    }

    pub(crate) fn face_present(&mut self, k: [u32; 3]) -> bool {
        let mut buf = std::mem::take(&mut self.buf);
        self.dt.tds.star(k[0], &mut buf);
        let r = buf.iter().any(|&t| self.dt.tds.has(t, k[1]) && self.dt.tds.has(t, k[2]));
        self.buf = buf;
        r
    }

    // ---------------------------------------------------------------- edges

    fn seg_crosses_tet(&self, v: [u32; 4], u: u32, w: u32) -> bool {
        let s = self.dt.sos();
        let hu = v.iter().position(|&x| x == u);
        let hw = v.iter().position(|&x| x == w);
        let cone = |apex: usize, target: u32| -> bool {
            (0..4).filter(|&k| k != apex).all(|k| {
                let f = FACE[k];
                s.ori_s(v[f[0]], v[f[1]], v[f[2]], target) < 0
            })
        };
        match (hu, hw) {
            (Some(_), Some(_)) => false,
            (Some(a), None) => cone(a, w),
            (None, Some(b)) => cone(b, u),
            (None, None) => (0..4).any(|k| {
                let f = FACE[k];
                let (a, b, c) = (v[f[0]], v[f[1]], v[f[2]]);
                self.pierces(u, w, a, b, c)
            }),
        }
    }

    /// Does segment `(p,q)` cross triangle `(a,b,c)` (generic position)?
    pub(crate) fn pierces(&self, p: u32, q: u32, a: u32, b: u32, c: u32) -> bool {
        let s = self.dt.sos();
        if s.ori_s(a, b, c, p) == s.ori_s(a, b, c, q) {
            return false;
        }
        let o1 = s.ori_s(p, q, a, b);
        o1 == s.ori_s(p, q, b, c) && o1 == s.ori_s(p, q, c, a)
    }

    /// The tets crossed by segment `(u,w)` from `u` to `w`, and the crossed faces
    /// `(tet, slot)` (face of `tet` towards the next tet).
    pub(crate) fn corridor(&mut self, u: u32, w: u32) -> Option<Corridor> {
        let mut star = Vec::new();
        self.dt.tds.star(u, &mut star);
        let s = self.dt.sos();
        let mut t = NONE;
        for &c in &star {
            if self.dt.tds.ghost_slot(c).is_some() {
                continue;
            }
            let v = self.dt.tds.verts[c as usize];
            let su = v.iter().position(|&x| x == u)?;
            let inside = (0..4).filter(|&k| k != su).all(|k| {
                let f = FACE[k];
                s.ori_s(v[f[0]], v[f[1]], v[f[2]], w) < 0
            });
            if inside {
                t = c;
                break;
            }
        }
        if t == NONE {
            return None;
        }
        let mut tets = vec![t];
        let mut faces = Vec::new();
        let mut entry: Option<u32> = None; // tet we came from
        let limit = 4 * self.dt.tds.slots() + 16;
        for _ in 0..limit {
            let v = self.dt.tds.verts[t as usize];
            if v.contains(&w) {
                return Some((tets, faces));
            }
            let mut exit = None;
            for (k, f) in FACE.iter().enumerate() {
                let n = self.dt.tds.nbr[t as usize][k];
                if n == NONE || Some(n) == entry {
                    continue;
                }
                let (a, b, c) = (v[f[0]], v[f[1]], v[f[2]]);
                if a == u || b == u || c == u {
                    continue;
                }
                if self.pierces(u, w, a, b, c) {
                    exit = Some((k, n));
                    break;
                }
            }
            let (k, n) = exit?;
            faces.push((t, k));
            entry = Some(t);
            t = n;
            tets.push(t);
        }
        None
    }

    pub(crate) fn ensure_edge(&mut self, u: u32, w: u32, owner: u32) -> Result<(), TetError> {
        if self.complete_mode {
            return self.recover_seg(u, w, owner);
        }
        match self.ensure_edge_once(u, w, owner) {
            Err(e @ TetError::RecoveryFailed { .. }) => {
                self.failed_edges += 1;
                let hints = self.edge_hints(u, w);
                self.hints.extend(hints);
                Err(e)
            }
            other => other,
        }
    }

    /// Distance along the ray `o + t d` (unit `d`) to the nearest input triangle that does not
    /// contain any of the vertices `skip`; infinite if the ray hits none. A heuristic bound on how
    /// far a hint point can sit from the surface before it crosses to the other side.
    fn ray_cap(&self, o: V3, d: V3, skip: &[u32]) -> f64 {
        let mut best = f64::INFINITY;
        for tri in &self.con.tris {
            if tri.iter().any(|x| skip.contains(x)) {
                continue;
            }
            let [a, b, c] = tri.map(|i| self.dt.pts[i as usize]);
            let (e1, e2) = (sub(b, a), sub(c, a));
            let pv = crate::geom::cross(d, e2);
            let det = dot(e1, pv);
            if det.abs() < 1e-300 {
                continue;
            }
            let inv = 1.0 / det;
            let tv = sub(o, a);
            let u = dot(tv, pv) * inv;
            if !(0.0..=1.0).contains(&u) {
                continue;
            }
            let qv = crate::geom::cross(tv, e1);
            let v = dot(d, qv) * inv;
            if v < 0.0 || u + v > 1.0 {
                continue;
            }
            let t = dot(e2, qv) * inv;
            if t > 0.0 && t < best {
                best = t;
            }
        }
        best
    }

    /// Points next to input vertex `v`, inside the solid and outside it: `v` plus a direction on
    /// the inner (outer) side of every incident triangle, at `rho * h` for each `rho`. A pit
    /// (peak) vertex whose edges all bend the wrong way is recovered by the fan of tets from `v`
    /// to a single point below (above) it. Empty if the incident triangles have no common side.
    fn vertex_points(&self, v: u32, rhos: &[f64]) -> (Vec<V3>, Vec<V3>) {
        let vp = self.dt.pts[v as usize];
        let mut normals: Vec<V3> = Vec::new();
        let mut hsum = 0.0;
        let mut hcnt = 0.0;
        for t in self.con.tris_of_vertex(v) {
            let tri = self.con.tris[t as usize];
            let [a, b, c] = tri.map(|i| self.dt.pts[i as usize]);
            let n = crate::geom::cross(sub(b, a), sub(c, a));
            let nl = dot(n, n).sqrt();
            if nl == 0.0 {
                return (Vec::new(), Vec::new());
            }
            normals.push(scale(n, 1.0 / nl));
            for &o in &tri {
                if o != v {
                    hsum += crate::geom::dist(vp, self.dt.pts[o as usize]);
                    hcnt += 1.0;
                }
            }
        }
        if normals.is_empty() {
            return (Vec::new(), Vec::new());
        }
        let h = hsum / hcnt;
        // A unit direction `d` with `dot(d, m_i) >= margin` for all inner normals `m_i = -sgn n_i`
        // (the solid side for sgn = 1), by alternating projection from the mean.
        let dir = |sgn: f64| -> Option<V3> {
            let mut d = [0.0; 3];
            for n in &normals {
                d = add(d, scale(*n, -sgn));
            }
            let l = dot(d, d).sqrt();
            if l == 0.0 {
                return None;
            }
            d = scale(d, 1.0 / l);
            for margin in [0.5, 0.35, 0.2, 0.1] {
                for _ in 0..80 {
                    let mut moved = false;
                    for n in &normals {
                        let m = scale(*n, -sgn);
                        let g = dot(d, m);
                        if g < margin {
                            d = add(d, scale(m, margin - g + 1e-3));
                            let l = dot(d, d).sqrt();
                            d = scale(d, 1.0 / l);
                            moved = true;
                        }
                    }
                    if !moved {
                        return Some(d);
                    }
                }
            }
            None
        };
        let side = |sgn: f64| -> Vec<V3> {
            match dir(sgn) {
                Some(d) => {
                    // Stay on this side of the surface: at most half way to the next triangle.
                    let cap = 0.5 * self.ray_cap(vp, d, &[v]);
                    rhos.iter().map(|&r| add(vp, scale(d, (r * h).min(cap)))).collect()
                }
                None => Vec::new(),
            }
        };
        (side(1.0), side(-1.0))
    }

    /// Points in the wedges around constraint edge `(u,w)`: the bisector of the solid-side wedge
    /// and of the exterior wedge, at `rho * h` from the points `t * |uw|` along the edge. The
    /// ring of tets around a recovered edge needs a vertex on every side whose dihedral angle
    /// exceeds 180 degrees (the solid side of a reflex edge, the exterior side of a convex one).
    fn wedge_points(&self, u: u32, w: u32, rhos: &[f64], ts: &[f64]) -> (Vec<V3>, Vec<V3>) {
        let (mut win, mut wout) = (Vec::new(), Vec::new());
        let (up, wp) = (self.dt.pts[u as usize], self.dt.pts[w as usize]);
        let tris: Vec<[u32; 3]> = self.con.tris_of_edge(u, w).iter().map(|&(_, t)| self.con.tris[t as usize]).collect();
        let [ta, tb] = tris[..] else { return (win, wout) };
        let third = |t: [u32; 3]| t.iter().copied().find(|&x| x != u && x != w);
        let (Some(a), Some(b)) = (third(ta), third(tb)) else { return (win, wout) };
        let [p0, p1, p2] = ta.map(|i| self.dt.pts[i as usize]);
        let nrm = crate::geom::cross(sub(p1, p0), sub(p2, p0));
        let nl = dot(nrm, nrm).sqrt();
        let el = crate::geom::dist(up, wp);
        if nl == 0.0 || el == 0.0 {
            return (win, wout);
        }
        let ev = scale(sub(wp, up), 1.0 / el);
        let perp = |q: V3| {
            let d = sub(q, up);
            let d = sub(d, scale(ev, dot(d, ev)));
            let l = dot(d, d).sqrt();
            if l > 0.0 { Some(scale(d, 1.0 / l)) } else { None }
        };
        let (Some(da), Some(db)) = (perp(self.dt.pts[a as usize]), perp(self.dt.pts[b as usize])) else { return (win, wout) };
        // Rotating about the edge from `da` by +sigma goes into the solid.
        let ta_dir = crate::geom::cross(ev, da);
        let sigma = if dot(ta_dir, nrm) < 0.0 { 1.0 } else { -1.0 };
        let ang = f64::atan2(dot(ev, crate::geom::cross(da, db)), dot(da, db));
        let mut theta = sigma * ang;
        if theta <= 0.0 {
            theta += std::f64::consts::TAU;
        }
        let (sn, cs) = (sigma * theta * 0.5).sin_cos();
        let dir = add(scale(da, cs), scale(ta_dir, sn));
        let h = (el + crate::geom::dist(up, self.dt.pts[a as usize]) + crate::geom::dist(wp, self.dt.pts[b as usize])) / 3.0;
        for &t in ts {
            let m = add(up, scale(ev, t * el));
            let cap_in = 0.5 * self.ray_cap(m, dir, &[u, w]);
            let cap_out = 0.5 * self.ray_cap(m, scale(dir, -1.0), &[u, w]);
            for &rho in rhos {
                win.push(add(m, scale(dir, (rho * h).min(cap_in))));
                wout.push(add(m, scale(dir, -(rho * h).min(cap_out))));
            }
        }
        (win, wout)
    }

    /// Points worth adding when edge `(u,w)` cannot be recovered: the centroid of the input
    /// vertices around the tets that touch the segment, and a point between it and the segment
    /// midpoint (at most two). They may lie outside the solid: the triangulation outside the
    /// surface must be conforming too, and Steiner points that end up in the outside region are
    /// dropped with it.
    fn edge_hints(&mut self, u: u32, w: u32) -> Vec<V3> {
        let (up, wp) = (self.dt.pts[u as usize], self.dt.pts[w as usize]);
        let mut star = Vec::new();
        self.dt.tds.star(u, &mut star);
        let mut touching: Vec<u32> = Vec::new();
        let mut marks = std::mem::take(&mut self.marks);
        marks.reset(self.dt.tds.slots());
        let mut stack: Vec<u32> = star.into_iter().filter(|&t| self.touches(t, u, w, up, wp)).collect();
        while let Some(t) = stack.pop() {
            if marks.in_a(t) {
                continue;
            }
            marks.add_a(t);
            touching.push(t);
            for k in 0..4 {
                let n = self.dt.tds.nbr[t as usize][k];
                if n != NONE && !marks.in_a(n) && self.touches(n, u, w, up, wp) {
                    stack.push(n);
                }
            }
            if touching.len() > 512 {
                break;
            }
        }
        self.marks = marks;
        let mut vs: Vec<u32> = touching.iter().flat_map(|&t| self.dt.tds.verts[t as usize]).filter(|&x| x < self.n_real).collect();
        vs.sort_unstable();
        vs.dedup();
        if vs.is_empty() {
            return Vec::new();
        }
        let mut c = [0.0; 3];
        for &v in &vs {
            c = add(c, self.dt.pts[v as usize]);
        }
        let c = scale(c, 1.0 / vs.len() as f64);
        let mid = scale(add(up, wp), 0.5);
        // Points in the wedges around the edge. The ring of tets around a recovered edge needs a
        // vertex on every side whose dihedral angle exceeds 180 degrees (the interior side of a
        // reflex edge, the exterior side of a convex one): the bisector of each wedge at growing
        // distances from the edge, then the chambers next to the two triangles.
        let rho = [0.45, 0.7, 0.3, 1.0, 0.55][self.attempt % 5];
        let tpos = [0.5, 0.35, 0.65][(self.attempt / 5) % 3];
        let (wedge_in, wedge_out) = self.wedge_points(u, w, &[rho], &[tpos]);
        let mut chamber: Vec<V3> = Vec::new();
        let frac = [0.5, 0.35, 0.7][self.attempt % 3];
        for &(_, ti) in self.con.tris_of_edge(u, w) {
            let [a, b, d] = self.con.tris[ti as usize].map(|i| self.dt.pts[i as usize]);
            let n = crate::geom::cross(sub(b, a), sub(d, a));
            let nl = dot(n, n).sqrt();
            if nl == 0.0 {
                continue;
            }
            let h = (crate::geom::dist(a, b) + crate::geom::dist(b, d) + crate::geom::dist(d, a)) / 3.0;
            let fc = scale(add(add(a, b), d), 1.0 / 3.0);
            for sgn in [-1.0, 1.0] {
                chamber.push(add(fc, scale(n, sgn * frac * h / nl)));
            }
        }
        if self.attempt < 6 && (!wedge_in.is_empty() || !chamber.is_empty()) {
            // Exterior points are free (dropped with the exterior); interior ones go first only
            // as the wedge bisector, the chambers only after a few failed restarts.
            let mut out = vec![c];
            let mut ext: Vec<V3> = Vec::new();
            for v in [u, w] {
                let (i, o) = self.vertex_points(v, &[0.6]);
                out.extend(i);
                ext.extend(o);
            }
            out.extend(wedge_in);
            out.extend(ext);
            out.extend(wedge_out);
            if self.attempt >= 2 {
                out.extend(chamber);
            }
            return out.into_iter().filter_map(sanitize).collect();
        }
        let mut cands: Vec<V3> = vec![c, add(scale(c, 0.5), scale(mid, 0.5))];
        for &v in &vs {
            if v != u && v != w {
                cands.push(add(mid, scale(sub(self.dt.pts[v as usize], mid), 0.4)));
            }
        }
        // Successive restarts use different candidates: the fixed ones first, then low-discrepancy
        // points spread over the neighbourhood (fresh ones for every restart).
        if self.attempt >= 6 {
            let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
            for &v in &vs {
                for a in 0..3 {
                    lo[a] = lo[a].min(self.dt.pts[v as usize][a]);
                    hi[a] = hi[a].max(self.dt.pts[v as usize][a]);
                }
            }
            let radical = |mut n: usize, b: usize| {
                let (mut f, mut r) = (1.0, 0.0);
                while n > 0 {
                    f /= b as f64;
                    r += f * (n % b) as f64;
                    n /= b;
                }
                r
            };
            let mut fresh = Vec::new();
            for j in 0..2 {
                let n = 2 * self.attempt + j + 1;
                fresh.push([
                    lo[0] + (hi[0] - lo[0]) * radical(n, 2),
                    lo[1] + (hi[1] - lo[1]) * radical(n, 3),
                    lo[2] + (hi[2] - lo[2]) * radical(n, 5),
                ]);
            }
            return fresh.into_iter().filter_map(sanitize).collect();
        }
        let skip = (2 * self.attempt) % cands.len();
        cands.rotate_left(skip);
        cands.into_iter().filter_map(sanitize).take(2).collect()
    }

    /// Exact test: does segment `(u,w)` meet the vertices-`f` triangle other than at a common vertex?
    pub(crate) fn edge_hits_face(&self, u: u32, w: u32, f: [u32; 3]) -> bool {
        let q = |i: u32| (i, self.dt.pts[i as usize]);
        crate::exact::seg_tri_improper(q(u), q(w), [q(f[0]), q(f[1]), q(f[2])])
    }

    /// Exact test: do the constraint triangles `g` and `f` meet beyond their common sub-simplex?
    pub(crate) fn faces_conflict(&self, g: [u32; 3], f: [u32; 3]) -> bool {
        let q = |i: u32| (i, self.dt.pts[i as usize]);
        let h = f.map(|i| self.dt.pts[i as usize]);
        // Reference point off the plane of `f` (only used for coplanar triangles sharing an edge).
        let r = g.iter().map(|&i| self.dt.pts[i as usize]).find(|&p| ori(h[0], h[1], h[2], p) != 0).unwrap_or(h[0]);
        crate::exact::tris_conflict([q(g[0]), q(g[1]), q(g[2])], [q(f[0]), q(f[1]), q(f[2])], r)
    }

    fn touches(&self, t: u32, u: u32, w: u32, up: V3, wp: V3) -> bool {
        let v = self.dt.tds.verts[t as usize];
        if v.contains(&crate::tds::GHOST) {
            return false;
        }
        let p = |i: usize| self.dt.pts[v[i] as usize];
        crate::exact::seg_touches_tet(v, [p(0), p(1), p(2), p(3)], u, w, up, wp)
    }

    fn ensure_edge_once(&mut self, u: u32, w: u32, owner: u32) -> Result<(), TetError> {
        if self.edge_by_flips(u, w, owner)? {
            return Ok(());
        }
        self.edge_by_cavity(u, w, owner)
    }

    /// Try to recover edge `(u,w)` by flips that reduce the number of tets crossing it; `true`
    /// if the edge is present afterwards.
    pub(crate) fn edge_by_flips(&mut self, u: u32, w: u32, owner: u32) -> Result<bool, TetError> {
        self.tabu.clear();
        for _iter in 0..120 {
            if self.dt.tds.find_edge(u, w).is_some() {
                return Ok(true);
            }
            let Some((tets, faces)) = self.corridor(u, w) else {
                if self.complete_mode {
                    return Ok(false);
                }
                return Err(fail(owner, Some([u, w]), "could not trace the edge through the triangulation"));
            };
            // A constraint face in the corridor is only a proof of intersection if the exact test
            // agrees (the corridor walk is perturbed); otherwise the cavity path decides.
            let mut blocked = false;
            for &(t, k) in &faces {
                let f = self.dt.tds.face(t, k);
                if let Some(other) = self.face_owner(sorted3(f[0], f[1], f[2])) {
                    if self.edge_hits_face(u, w, f) {
                        return Err(TetError::SelfIntersection { tri_a: owner.min(other), tri_b: owner.max(other) });
                    }
                    blocked = true;
                }
            }
            if blocked {
                break;
            }
            let before = tets.len();
            let mut sorted_c = tets.clone();
            sorted_c.sort_unstable();
            let after_of = |this: &Self, old: &[u32], new: &[[u32; 4]]| -> usize {
                if new.iter().any(|v| v.contains(&u) && v.contains(&w)) {
                    return 0;
                }
                let removed = old.iter().filter(|t| sorted_c.binary_search(t).is_ok()).count();
                let added = new.iter().filter(|&&v| this.seg_crosses_tet(v, u, w)).count();
                before - removed + added
            };
            // Edge removals of the edges of crossed faces.
            let mut best: Option<(usize, usize, [u32; 2], crate::flips::Plan)> = None;
            let mut seen: Vec<[u32; 2]> = Vec::new();
            for &(t, k) in &faces {
                let f = self.dt.tds.face(t, k);
                for e in 0..3 {
                    let (x, y) = (f[e], f[(e + 1) % 3]);
                    let key = [x.min(y), x.max(y)];
                    if seen.contains(&key) || self.tabu.contains(&key) || self.edge_owner(x, y).is_some() {
                        continue;
                    }
                    seen.push(key);
                    if let Some(plan) = self.dt.plan_edge_removal(x, y, t, 16, &mut self.sc)
                        && !self.makes_shortcut(&plan.old, &plan.new)
                    {
                        let after = after_of(self, &plan.old, &plan.new);
                        let cand = (after, plan.new.len(), key);
                        if best.as_ref().is_none_or(|b| cand < (b.0, b.1, b.2)) {
                            best = Some((cand.0, cand.1, cand.2, plan));
                        }
                    }
                }
            }
            if let Some((after, _, _, plan)) = best
                && after < before {
                    self.dt.apply(&plan)?;
                    self.flips += 1;
                    continue;
                }
            // 2-3 flips of crossed faces (reshape), best-first, never undone by edge removal.
            let mut best23: Option<(usize, [u32; 2], crate::flips::Plan)> = None;
            for &(t, k) in &faces {
                if let Some(plan) = self.dt.plan_flip23(t, k)
                    && !self.makes_shortcut(&plan.old, &plan.new)
                {
                    let after = after_of(self, &plan.old, &plan.new);
                    let ne = [plan.new[0][0].min(plan.new[0][1]), plan.new[0][0].max(plan.new[0][1])];
                    if self.tabu.contains(&ne) {
                        continue;
                    }
                    if best23.as_ref().is_none_or(|b| (after, ne) < (b.0, b.1)) {
                        best23 = Some((after, ne, plan));
                    }
                }
            }
            if let Some((after, ne, plan)) = best23
                && after <= before {
                    self.dt.apply(&plan)?;
                    self.flips += 1;
                    self.tabu.push(ne);
                    continue;
                }
            break;
        }
        Ok(self.dt.tds.find_edge(u, w).is_some())
    }

    // ---------------------------------------------------------------- faces

    /// Do triangles `g` and `f` intersect improperly (generic position)?
    pub(crate) fn tri_pierce(&self, g: [u32; 3], f: [u32; 3]) -> bool {
        for e in 0..3 {
            let (p, q) = (g[e], g[(e + 1) % 3]);
            if !f.contains(&p) && !f.contains(&q) && self.pierces(p, q, f[0], f[1], f[2]) {
                return true;
            }
            let (p, q) = (f[e], f[(e + 1) % 3]);
            if !g.contains(&p) && !g.contains(&q) && self.pierces(p, q, g[0], g[1], g[2]) {
                return true;
            }
        }
        false
    }

    pub(crate) fn recover_face(&mut self, tri: u32, key: [u32; 3]) -> Result<(), TetError> {
        let mut marks = std::mem::take(&mut self.marks);
        marks.reset(self.dt.tds.slots());
        let res = self.recover_face_in(tri, key, &mut marks);
        self.marks = marks;
        res
    }

    /// [`Recover::recover_face`] with the epoch set `a` marking the crossing tets.
    fn recover_face_in(&mut self, tri: u32, key: [u32; 3], marks: &mut Marks) -> Result<(), TetError> {
        // The oriented triangle is not needed here, only its vertices.
        let [a, b, c] = key;
        for &(x, y) in &[(a, b), (b, c), (a, c)] {
            if self.dt.tds.find_edge(x, y).is_none() {
                self.ensure_edge(x, y, tri)?;
            }
        }
        if self.face_present(key) {
            return Ok(());
        }
        // Seed: the tet around edge (a,b) whose wedge contains the direction to c.
        let Some(t0) = self.dt.tds.find_edge(a, b) else { return Err(fail(tri, Some([a, b]), "edge vanished")) };
        let s = Pts(&self.dt.pts);
        let mut ring = Vec::new();
        let mut poly = Vec::new();
        if !self.dt.tds.ring(a, b, t0, 64, &mut ring, &mut poly) {
            return Err(fail(tri, Some([a, b]), "open ring around a constraint edge"));
        }
        let mut seed = NONE;
        for &t in &ring {
            let v = self.dt.tds.verts[t as usize];
            let mut o = v.iter().copied().filter(|&x| x != a && x != b);
            let (Some(mut x), Some(mut y)) = (o.next(), o.next()) else { continue };
            if s.ori_s(a, b, x, y) < 0 {
                std::mem::swap(&mut x, &mut y);
            }
            if s.ori_s(a, b, x, c) > 0 && s.ori_s(a, b, c, y) > 0 {
                seed = t;
                break;
            }
        }
        if seed == NONE {
            return Err(fail(tri, None, "no tet around the edge contains the triangle direction"));
        }
        // BFS over crossing tets.
        let mut rset: Vec<u32> = vec![seed];
        marks.add_a(seed);
        let mut head = 0;
        while head < rset.len() {
            let t = rset[head];
            head += 1;
            for k in 0..4 {
                let n = self.dt.tds.nbr[t as usize][k];
                if n == NONE || marks.in_a(n) {
                    continue;
                }
                let g = self.dt.tds.face(t, k);
                if g.iter().all(|x| key.contains(x)) {
                    continue;
                }
                if self.tri_pierce(g, key) {
                    if let Some(other) = self.face_owner(sorted3(g[0], g[1], g[2])) {
                        if self.faces_conflict(g, key) {
                            return Err(TetError::SelfIntersection { tri_a: tri.min(other), tri_b: tri.max(other) });
                        }
                        // Perturbed crossing only (exact contact is proper): do not cross it.
                        continue;
                    }
                    if self.dt.tds.ghost_slot(n).is_some() {
                        return Err(fail(tri, None, "triangle reaches the auxiliary hull"));
                    }
                    marks.add_a(n);
                    rset.push(n);
                }
            }
            if rset.len() > 4096 {
                return Err(fail(tri, None, "crossing region too large"));
            }
        }
        // Constraint edges that pierce the triangle, and internal constraint faces.
        for &t in &rset {
            let v = self.dt.tds.verts[t as usize];
            for i in 0..4 {
                for j in i + 1..4 {
                    let (p, q) = (v[i], v[j]);
                    if !key.contains(&p) && !key.contains(&q)
                        && let Some(other) = self.edge_owner(p, q)
                            && self.edge_hits_face(p, q, key) {
                                return Err(TetError::SelfIntersection { tri_a: tri.min(other), tri_b: tri.max(other) });
                            }
                }
            }
            for k in 0..4 {
                let n = self.dt.tds.nbr[t as usize][k];
                if n != NONE && marks.in_a(n) {
                    let g = self.dt.tds.face(t, k);
                    if let Some(other) = self.face_owner(sorted3(g[0], g[1], g[2])) {
                        return Err(fail(tri, None, format!("triangle {other} lies inside the cavity of this triangle")));
                    }
                }
            }
        }
        // Split the cavity boundary by the side of the triangle's plane.
        let mut top: Vec<[u32; 3]> = Vec::new();
        let mut bot: Vec<[u32; 3]> = Vec::new();
        let mut bverts: Vec<u32> = Vec::new();
        for &t in &rset {
            for k in 0..4 {
                let n = self.dt.tds.nbr[t as usize][k];
                if n != NONE && marks.in_a(n) {
                    continue;
                }
                let g = self.dt.tds.face(t, k);
                let mut up = false;
                let mut down = false;
                for &w in &g {
                    bverts.push(w);
                    if key.contains(&w) {
                        continue;
                    }
                    if s.ori_s(a, b, c, w) > 0 {
                        up = true;
                    } else {
                        down = true;
                    }
                }
                match (up, down) {
                    (true, false) => top.push(g),
                    (false, true) => bot.push(g),
                    (false, false) => return Err(fail(tri, None, "cavity boundary face made of triangle vertices only")),
                    (true, true) => return Err(fail(tri, None, "cavity boundary face straddles the triangle plane")),
                }
            }
        }
        bverts.sort_unstable();
        bverts.dedup();
        for &t in &rset {
            for &x in &self.dt.tds.verts[t as usize] {
                if bverts.binary_search(&x).is_err() && !self.dt.droppable(x) {
                    return Err(fail(tri, None, "a vertex is enclosed by the cavity"));
                }
            }
        }
        if self.complete_mode {
            // Complete recovery never gives up a recovered constraint edge: refuse a cavity that
            // has one inside (not on a boundary face); the caller splits instead.
            let mut bedges: Vec<[u32; 2]> = Vec::new();
            for g in top.iter().chain(bot.iter()) {
                for e in 0..3 {
                    let (x, y) = (g[e], g[(e + 1) % 3]);
                    bedges.push([x.min(y), x.max(y)]);
                }
            }
            for e in 0..3 {
                let (x, y) = (key[e], key[(e + 1) % 3]);
                bedges.push([x.min(y), x.max(y)]);
            }
            bedges.sort_unstable();
            for &t in &rset {
                let v = self.dt.tds.verts[t as usize];
                for i in 0..4 {
                    for j in i + 1..4 {
                        let k = [v[i].min(v[j]), v[i].max(v[j])];
                        if self.edge_owner(k[0], k[1]).is_some() && bedges.binary_search(&k).is_err() {
                            return Err(fail(tri, None, "a constraint edge lies inside the cavity"));
                        }
                    }
                }
            }
        }
        // Caps: outward from the top polyhedron the triangle faces down (a,c,b).
        let mut new_tets: Vec<[u32; 4]> = Vec::new();
        for (side, faces) in [(true, &mut top), (false, &mut bot)] {
            let cap = if side { [a, c, b] } else { [a, b, c] };
            faces.insert(0, cap);
            match self.fill_steiner(faces, None, &[], &Seeds::default()) {
                Some(t) => new_tets.extend(t),
                None => {
                    let t = self.star_fill(tri, faces)?;
                    new_tets.extend(t);
                }
            }
        }
        if self.makes_shortcut(&rset, &new_tets) {
            return Err(fail(tri, None, "the refill would join two points of a split edge"));
        }
        let mut ids = Vec::new();
        self.dt.tds.replace(&rset, &new_tets, &mut ids).map_err(|e| fail(tri, None, format!("cavity commit failed: {e}")))?;
        self.flips += 1;
        Ok(())
    }

    // ---------------------------------------------------------------- cavity filling

    /// Edge recovery by cavity re-tetrahedralization: remove every tet touching the open segment
    /// and refill with the segment as a mandatory edge.
    fn edge_by_cavity(&mut self, u: u32, w: u32, owner: u32) -> Result<(), TetError> {
        let mut marks = std::mem::take(&mut self.marks);
        marks.reset(self.dt.tds.slots());
        let res = self.edge_by_cavity_in(u, w, owner, &mut marks);
        self.marks = marks;
        res
    }

    /// [`Recover::edge_by_cavity`] with the epoch sets (`a`: cavity, `b`: tested).
    fn edge_by_cavity_in(&mut self, u: u32, w: u32, owner: u32, marks: &mut Marks) -> Result<(), TetError> {
        let (up, wp) = (self.dt.pts[u as usize], self.dt.pts[w as usize]);
        let touches = |this: &Self, t: u32| -> bool {
            let v = this.dt.tds.verts[t as usize];
            if v.contains(&crate::tds::GHOST) {
                return false;
            }
            let p = |i: usize| this.dt.pts[v[i] as usize];
            crate::exact::seg_touches_tet(v, [p(0), p(1), p(2), p(3)], u, w, up, wp)
        };
        let mut star = Vec::new();
        self.dt.tds.star(u, &mut star);
        let mut r: Vec<u32> = Vec::new();
        for t in star {
            marks.add_b(t);
            if touches(self, t) {
                marks.add_a(t);
                r.push(t);
            }
        }
        if r.is_empty() {
            return Err(fail(owner, Some([u, w]), "no tet touches the edge"));
        }
        let mut head = 0;
        while head < r.len() {
            let t = r[head];
            head += 1;
            for k in 0..4 {
                let n = self.dt.tds.nbr[t as usize][k];
                if n == NONE || marks.in_b(n) {
                    continue;
                }
                marks.add_b(n);
                if touches(self, n) {
                    marks.add_a(n);
                    r.push(n);
                }
            }
            if r.len() > MAX_CAVITY {
                // Too large for a gift-wrapping refill; complete recovery splits the edge.
                return Err(fail(owner, Some([u, w]), "edge cavity too large"));
            }
        }
        // Phase 0: the smallest cavity, with interior points if it needs them.
        let search = self.failed_edges < SEARCH_EDGES && !self.dt.searched_out();
        let seeds = if search { self.edge_seeds(u, w) } else { Seeds::default() };
        if search {
            let cav = self.edge_cavity_boundary(&r, marks, u, w, owner)?;
            if let Some(tets) = self.fill_steiner(&cav.faces, Some((u, w)), &cav.inner, &seeds) {
                return self.commit_edge_cavity(&r, &tets, u, w, owner);
            }
        }
        // Phase A: refill with the vertices at hand, growing the cavity across the boundary faces
        // that made candidate tets invalid until the fill works or nothing more can be absorbed.
        for round in 0..if search { MAX_GROW } else { 1 } {
            if round > 0 && self.dt.searched_out() {
                break;
            }
            let cav = self.edge_cavity_boundary(&r, marks, u, w, owner)?;
            let budget = 2000 + 40 * cav.faces.len();
            match self.dt.fill_report(&cav.faces, Some((u, w)), &cav.inner, budget) {
                Ok(tets) => return self.commit_edge_cavity(&r, &tets, u, w, owner),
                Err(why) => {
                    let mut added = 0;
                    for g in &why.blockers {
                        added += self.absorb_across(g, &cav, &mut r, marks);
                    }
                    if added == 0 || r.len() > MAX_CAVITY {
                        break;
                    }
                }
            }
        }
        // Phase B: interior points may help (once on the cavity so far, then on layers around it).
        for layer in 0..if search { STEINER_LAYERS } else { 0 } {
            if self.dt.searched_out() {
                break;
            }
            if layer > 0 {
                self.grow_layer(&mut r, marks);
            }
            if r.len() > MAX_CAVITY {
                break;
            }
            let cav = self.edge_cavity_boundary(&r, marks, u, w, owner)?;
            if let Some(tets) = self.fill_steiner(&cav.faces, Some((u, w)), &cav.inner, &seeds) {
                return self.commit_edge_cavity(&r, &tets, u, w, owner);
            }
        }
        Err(fail(owner, Some([u, w]), "edge cavity cannot be tetrahedralized with the edge"))
    }

    /// Steiner candidates for the cavity around constraint edge `(u,w)`: points next to its
    /// endpoints and in the wedges around it, inside the solid and outside it.
    fn edge_seeds(&self, u: u32, w: u32) -> Seeds {
        let (mut inside, mut outside) = (Vec::new(), Vec::new());
        for v in [u, w] {
            let (i, o) = self.vertex_points(v, &[0.6, 0.35, 0.9]);
            inside.extend(i);
            outside.extend(o);
        }
        let (i, o) = self.wedge_points(u, w, &[0.4, 0.6, 0.9], &[0.5, 0.3, 0.7]);
        inside.extend(i);
        outside.extend(o);
        Seeds { pocket: self.pocket_centroid(u), inside, outside }
    }

    /// The vertex centroid of the connected surface component holding vertex `v`, if that
    /// component is tiny (a few dozen vertices): such a body is one pocket, and a single point
    /// that sees all of it is what a Schönhardt-like prism and its affine images need.
    fn pocket_centroid(&self, v: u32) -> Option<V3> {
        let mut seen: Vec<u32> = vec![v];
        let mut head = 0;
        while head < seen.len() {
            let x = seen[head];
            head += 1;
            for t in self.con.tris_of_vertex(x) {
                for &y in &self.con.tris[t as usize] {
                    if !seen.contains(&y) {
                        if seen.len() >= POCKET_VERTS {
                            return None;
                        }
                        seen.push(y);
                    }
                }
            }
        }
        let mut c = [0.0; 3];
        for &x in &seen {
            c = add(c, self.dt.pts[x as usize]);
        }
        sanitize(scale(c, 1.0 / seen.len() as f64))
    }

    /// The boundary of the cavity formed by tets `r` (marked in set `a`): its faces (outward),
    /// the tet beyond each boundary face, and the vertices the cavity encloses.
    fn edge_cavity_boundary(&self, r: &[u32], marks: &Marks, u: u32, w: u32, owner: u32) -> Result<Cavity, TetError> {
        let mut faces: Vec<[u32; 3]> = Vec::new();
        let mut outside: Vec<([u32; 3], u32)> = Vec::new();
        for &t in r {
            for k in 0..4 {
                let n = self.dt.tds.nbr[t as usize][k];
                let g = self.dt.tds.face(t, k);
                if n != NONE && marks.in_a(n) {
                    if let Some(other) = self.face_owner(sorted3(g[0], g[1], g[2])) {
                        if self.edge_hits_face(u, w, g) {
                            return Err(TetError::SelfIntersection { tri_a: owner.min(other), tri_b: owner.max(other) });
                        }
                        // The segment passes around this constraint face: keep it as a slit
                        // (both sides are cavity boundary, so it survives the refill).
                        faces.push(g);
                        outside.push((sorted3(g[0], g[1], g[2]), NONE));
                    }
                    continue;
                }
                faces.push(g);
                outside.push((sorted3(g[0], g[1], g[2]), n));
            }
        }
        outside.sort_unstable();
        let mut bverts: Vec<u32> = faces.iter().flatten().copied().collect();
        bverts.sort_unstable();
        bverts.dedup();
        // Interior points added earlier that the cavity encloses must be used by the refill
        // unless they are droppable Steiner points; both kinds are candidates.
        let mut inner: Vec<u32> = r.iter().flat_map(|&t| self.dt.tds.verts[t as usize]).filter(|x| bverts.binary_search(x).is_err()).collect();
        inner.sort_unstable();
        inner.dedup();
        Ok(Cavity { faces, outside, inner })
    }

    /// Add the tet beyond cavity boundary face `g` to the cavity `r`, unless `g` is a constraint
    /// face, there is no such tet, or taking it would swallow a constraint face. Returns the
    /// number of tets added.
    fn absorb_across(&self, g: &[u32; 3], cav: &Cavity, r: &mut Vec<u32>, marks: &mut Marks) -> usize {
        let key = sorted3(g[0], g[1], g[2]);
        if self.face_owner(key).is_some() {
            return 0;
        }
        let Ok(i) = cav.outside.binary_search_by(|e| e.0.cmp(&key)) else { return 0 };
        let n = cav.outside[i].1;
        if n == NONE || marks.in_a(n) || self.dt.tds.ghost_slot(n).is_some() {
            return 0;
        }
        // Do not swallow a constraint face between two cavity tets.
        let swallows = (0..4).any(|j| {
            let m = self.dt.tds.nbr[n as usize][j];
            m != NONE && marks.in_a(m) && {
                let h = self.dt.tds.face(n, j);
                self.face_owner(sorted3(h[0], h[1], h[2])).is_some()
            }
        });
        if swallows {
            return 0;
        }
        marks.add_a(n);
        r.push(n);
        1
    }

    /// Grow the cavity `r` by one layer of neighbours across non-constraint faces.
    fn grow_layer(&self, r: &mut Vec<u32>, marks: &mut Marks) {
        let snapshot = r.clone();
        for &t in &snapshot {
            for k in 0..4 {
                let n = self.dt.tds.nbr[t as usize][k];
                if n == NONE || marks.in_a(n) || self.dt.tds.ghost_slot(n).is_some() {
                    continue;
                }
                let g = self.dt.tds.face(t, k);
                if self.face_owner(sorted3(g[0], g[1], g[2])).is_some() {
                    continue;
                }
                let swallows = (0..4).any(|j| {
                    let m = self.dt.tds.nbr[n as usize][j];
                    m != NONE && m != t && marks.in_a(m) && {
                        let h = self.dt.tds.face(n, j);
                        self.face_owner(sorted3(h[0], h[1], h[2])).is_some()
                    }
                });
                if swallows {
                    continue;
                }
                marks.add_a(n);
                r.push(n);
            }
        }
    }

    /// Replace the cavity tets `r` by `tets` and check that the edge is there.
    fn commit_edge_cavity(&mut self, r: &[u32], tets: &[[u32; 4]], u: u32, w: u32, owner: u32) -> Result<(), TetError> {
        let mut ids = Vec::new();
        self.dt.tds.replace(r, tets, &mut ids).map_err(|e| fail(owner, Some([u, w]), format!("edge cavity commit failed: {e}")))?;
        self.flips += 1;
        if self.dt.tds.find_edge(u, w).is_none() {
            return Err(fail(owner, Some([u, w]), "edge missing after cavity refill"));
        }
        Ok(())
    }

    /// Fill a cavity (see [`Dt::fill_with_steiner`]); counts the Steiner points it adds.
    fn fill_steiner(&mut self, faces: &[[u32; 3]], seg: Option<(u32, u32)>, inner: &[u32], seeds: &Seeds) -> Option<Vec<[u32; 4]>> {
        let budget = 2000 + 40 * faces.len();
        if self.n_steiner + 1 > self.max_steiner {
            return self.dt.fill_budgeted(faces, seg, inner, budget);
        }
        let (t, used) = self.dt.fill_with_steiner(faces, seg, inner, seeds, budget, self.guard)?;
        self.n_steiner += used;
        Some(t)
    }

    /// Interior Steiner fallback: find a point that sees every face (strictly) and star it.
    fn star_fill(&mut self, tri: u32, faces: &[[u32; 3]]) -> Result<Vec<[u32; 4]>, TetError> {
        let Some(c) = self.dt.kernel_point(faces, self.guard) else {
            // Suggest the centroid of the cavity (if inside the solid) to the caller's restart.
            let mut vs: Vec<u32> = faces.iter().flatten().copied().collect();
            vs.sort_unstable();
            vs.dedup();
            let mut c = [0.0; 3];
            for &v in &vs {
                c = add(c, self.dt.pts[v as usize]);
            }
            let c = scale(c, 1.0 / vs.len() as f64);
            let k = self.attempt % vs.len();
            let toward = self.dt.pts[vs[k] as usize];
            let blend = if self.attempt == 0 { c } else { add(scale(c, 0.6), scale(toward, 0.4)) };
            if let Some(c) = sanitize(blend) {
                self.hints.push(c);
            }
            return Err(fail(tri, None, "cavity cannot be tetrahedralized and has no interior kernel point (Steiner fallback failed)"));
        };
        if self.n_steiner + 1 > self.max_steiner {
            return Err(TetError::SteinerBudget { limit: self.max_steiner });
        }
        let id = self.dt.push_point(c);
        self.n_steiner += 1;
        Ok(faces.iter().map(|f| [id, f[0], f[1], f[2]]).collect())
    }
}

impl Dt {
    /// A point strictly on the inner side of every face of the polyhedron bounded by `faces`
    /// (outward orientation), found from the vertex centroid, the face centroid and an
    /// alternating-projection kernel search with decreasing margins; `None` if the polyhedron is
    /// not star-shaped enough. The candidate with the best S4 score wins (first on ties), so a
    /// compliant point is used whenever one is found.
    pub(crate) fn kernel_point(&self, faces: &[[u32; 3]], guard: &Guard<'_>) -> Option<V3> {
        let pts = &self.pts;
        let visible = |p: V3| faces.iter().all(|f| ori(pts[f[0] as usize], pts[f[1] as usize], pts[f[2] as usize], p) < 0);
        let mut best: Option<(f64, V3)> = None;
        for c in self.kernel_candidates(faces).into_iter().filter(|&c| visible(c)) {
            let sc = guard.score(c);
            if best.is_none_or(|b| sc > b.0) {
                best = Some((sc, c));
            }
        }
        best.map(|b| b.1)
    }

    /// Candidate kernel points of the polyhedron bounded by `faces` (see [`Dt::kernel_point`]),
    /// finite but not checked for visibility: the vertex and face centroids and the results of
    /// the alternating projection with margins 0.4 .. 0.05 of the mean edge length.
    pub(crate) fn kernel_candidates(&self, faces: &[[u32; 3]]) -> Vec<V3> {
        let pts = &self.pts;
        if faces.is_empty() {
            return Vec::new();
        }
        let mut verts: Vec<u32> = faces.iter().flatten().copied().collect();
        verts.sort_unstable();
        verts.dedup();
        let mut cen = [0.0; 3];
        for &v in &verts {
            cen = add(cen, pts[v as usize]);
        }
        let cen = scale(cen, 1.0 / verts.len() as f64);
        let mut fc = [0.0; 3];
        for f in faces {
            fc = add(fc, scale(add(add(pts[f[0] as usize], pts[f[1] as usize]), pts[f[2] as usize]), 1.0 / 3.0));
        }
        let fcen = scale(fc, 1.0 / faces.len() as f64);
        let h = {
            let mut m = 0.0;
            for f in faces {
                m += crate::geom::dist(pts[f[0] as usize], pts[f[1] as usize]);
            }
            m / faces.len() as f64
        };
        let mut cands: Vec<V3> = vec![cen, fcen];
        for margin in [0.4, 0.3, 0.2, 0.12, 0.05] {
            let mut p = cen;
            for _ in 0..60 {
                let mut moved = false;
                for f in faces {
                    let (a, b, c) = (pts[f[0] as usize], pts[f[1] as usize], pts[f[2] as usize]);
                    let n = crate::geom::cross(sub(b, a), sub(c, a));
                    let nn = dot(n, n).sqrt();
                    if nn == 0.0 {
                        continue;
                    }
                    let n = scale(n, 1.0 / nn);
                    let d = dot(sub(p, a), n);
                    // The inner side is negative; keep at least `margin h` inside.
                    if d > -margin * h {
                        p = sub(p, scale(n, d + 1.2 * margin * h));
                        moved = true;
                    }
                }
                if !moved {
                    break;
                }
            }
            cands.push(p);
        }
        cands.into_iter().filter_map(sanitize).collect()
    }
}
