//! Face triangulation.
//!
//! Two paths, both built on the shared edge samples (a face never re-samples its boundary):
//!
//! 1. **Grid** (curved faces whose parameter-space loop is an axis-aligned rectangle: cylinder
//!    lateral, cone, sphere, torus, ruled or revolved patches). The four sides come from the
//!    edge samples; interior rows sit at the left/right side's sample parameters, evaluated on
//!    the surface. A row's point count is the larger of (a) the count interpolated between the
//!    bottom and top counts and (b) the count its own iso-`v` curve needs (circle rule at the
//!    row's radius for revolved surfaces, adaptive sampling otherwise), so bulging patches
//!    (torus seamed at the inner equator, sphere bands) are resolved. On doubly curved
//!    surfaces the chord budget is split: edges next to such a face are sampled with
//!    `chord_tol / 2` (see `tess.rs`) and rows use `chord_tol / 2` too, so a triangle's sag,
//!    about `sag_u + sag_v`, stays within `chord_tol`.
//!    Adjacent rows are joined by a merge ("zipper") triangulation, so unequal counts work. Cone
//!    rings are given equal counts by the edge sampler (`tess::edge_rule`); with unequal counts
//!    the zipper's cross-ring edges are not generators and the cone sags far beyond `chord_tol`.
//!    Pole rows collapse to the single pole vertex and the wedges that degenerate are dropped.
//! 2. **Polygon** (planar faces, and any other face): ear clipping of the loops in a 2-D frame
//!    (`Plane` frame, pcurves, or a best-fit Newell frame for faces without a surface). Only
//!    boundary vertices are used, so a curved face that is not a parameter rectangle is
//!    resolved at its boundary only.

use sm_core::EntityId;
use sm_geom_api::{Domain1, PCurve, ParametricSurface, SurfaceDef, Uv};
use sm_topo_api::{Carrier, CellFlags, CoedgeKey, Incidence, Sense, Side, TopologyView, TopologyViewExt};

use crate::earcut::{self, RingPt};
use crate::options::{TessError, TessOptions};
use crate::sample::{self, P3};

/// Read-only data shared by all face jobs.
pub(crate) struct Shared<'a> {
    pub view: &'a dyn TopologyView,
    pub opts: &'a TessOptions,
    /// Live non-degenerate edges, ascending.
    pub edges: &'a [EntityId],
    /// CSR offsets into `edge_g` / `edge_pos` / `edge_t`, one range per entry of `edges`.
    pub edge_off: &'a [usize],
    pub edge_g: &'a [u32],
    pub edge_pos: &'a [P3],
    pub edge_t: &'a [f64],
    /// Vertices, ascending; a vertex's global index is its position here.
    pub vertices: &'a [EntityId],
    /// Number of global (vertex + edge-interior) mesh vertices; face-local points are `>= g_count`.
    pub g_count: u32,
    /// Face-interior points reserved so far by all face jobs (size budget, see `MAX_VERTICES`).
    pub budget: &'a std::sync::atomic::AtomicUsize,
}

/// Face-local result: extra points and triangles. Indices `>= g_count` address `steiner`.
pub(crate) struct FaceOut {
    pub steiner: Vec<P3>,
    pub tris: Vec<[u32; 3]>,
    /// A curved face resolved at its boundary points only (no interior sampling).
    pub boundary_only: bool,
}

#[derive(Clone, Copy)]
struct Bs {
    g: u32,
    p: P3,
    t: f64,
}

struct Chain<'a> {
    /// `Some(vertex)` for a degenerate (pole) coedge.
    pole: Option<u32>,
    pcurve: Option<&'a PCurve>,
    /// Pole coedge traversed against its pcurve.
    reversed: bool,
    s: Vec<Bs>,
}

fn build_chain<'a>(sh: &Shared<'a>, face: EntityId, e: &Incidence) -> Result<Chain<'a>, TessError> {
    let view = sh.view;
    let pcurve = view.pcurve(CoedgeKey { face, edge: e.cell, side: e.side });
    if view.flags(e.cell).contains(CellFlags::DEGENERATE) {
        let (v0, _) = view.edge_ends(e.cell).ok_or_else(|| TessError::face(face, format!("degenerate edge {:?} has no end vertex", e.cell)))?;
        let g = sh.vertices.binary_search(&v0).map_err(|_| TessError::face(face, format!("vertex {v0:?} is not tessellated")))?;
        return Ok(Chain { pole: Some(g as u32), pcurve, reversed: e.side == Side::Right, s: Vec::new() });
    }
    let idx = sh.edges.binary_search(&e.cell).map_err(|_| TessError::face(face, format!("edge {:?} is not tessellated", e.cell)))?;
    let r = sh.edge_off[idx]..sh.edge_off[idx + 1];
    let mut s: Vec<Bs> = r.map(|k| Bs { g: sh.edge_g[k], p: sh.edge_pos[k], t: sh.edge_t[k] }).collect();
    if e.side == Side::Right {
        s.reverse();
    }
    Ok(Chain { pole: None, pcurve, reversed: false, s })
}

/// Message of the placeholder error a face returns when the shared budget is exceeded. Which face
/// crosses the limit first depends on the scheduler, so `tess::pick_error` replaces it with a
/// message built from the (order-independent) total.
pub(crate) const BUDGET_TRIPPED: &str = "shared face-point budget exceeded";

fn too_large(rows: usize, per_row: usize) -> TessError {
    TessError::TooLarge(format!("a grid face needs about {rows} rows of {per_row} points; the limit is {} vertices in total (raise chord_tol)", sample::MAX_VERTICES))
}

pub(crate) fn tessellate_face(sh: &Shared<'_>, face: EntityId) -> Result<FaceOut, TessError> {
    let view = sh.view;
    let mut rings: Vec<Vec<Chain<'_>>> = Vec::new();
    for l in view.loops(face) {
        let chains = l.entries.iter().map(|e| build_chain(sh, face, e)).collect::<Result<Vec<_>, _>>()?;
        rings.push(chains);
    }
    if rings.is_empty() || rings[0].is_empty() {
        return Err(TessError::face(face, "face has no boundary loop"));
    }
    let mut out = match view.carrier(face) {
        Carrier::Surface(sc) => {
            let shapes = view.shapes();
            let def = shapes.surface_def(sc.surface).ok_or_else(|| TessError::face(face, format!("surface {:?} is not in the geometry store", sc.surface)))?;
            let surf = shapes.surface(sc.surface);
            let mut out = None;
            if !matches!(def, SurfaceDef::Plane(_))
                && rings.len() == 1
                && let Some(s) = surf
            {
                out = grid_face(sh, face, s, def, &rings[0])?;
            }
            let mut out = match out {
                Some(o) => o,
                None => {
                    let mut o = polygon_face(face, Some((surf, def)), &rings)?;
                    o.boundary_only = !matches!(def, SurfaceDef::Plane(_));
                    o
                }
            };
            if sc.sense == Sense::Reversed {
                out.tris.iter_mut().for_each(|t| t.swap(1, 2));
            }
            out
        }
        _ => polygon_face(face, None, &rings)?,
    };
    out.tris.retain(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2]);
    Ok(out)
}

// ----------------------------------------------------------------------------------- grid path

/// One chain along a side: pole vertex (if any) and `(vertex, coordinate)` samples.
type Piece = (Option<u32>, Vec<(u32, f64)>);

struct SideSamples {
    pole: Option<u32>,
    g: Vec<u32>,
    c: Vec<f64>,
}

fn grid_face(sh: &Shared<'_>, face: EntityId, surf: &dyn ParametricSurface, def: &SurfaceDef, chains: &[Chain<'_>]) -> Result<Option<FaceOut>, TessError> {
    // Parameter-space points of every chain: (g, u, v).
    let mut uvc: Vec<Vec<(u32, f64, f64)>> = Vec::with_capacity(chains.len());
    for c in chains {
        let Some(pc) = c.pcurve else { return Ok(None) };
        if let Some(g) = c.pole {
            let r = pc.range();
            let (a, b) = (pc.eval(r.lo), pc.eval(r.hi));
            let (a, b) = if c.reversed { (b, a) } else { (a, b) };
            uvc.push(vec![(g, a.u, a.v), (g, b.u, b.v)]);
        } else {
            if c.s.iter().any(|s| !s.t.is_finite()) {
                return Ok(None);
            }
            uvc.push(c.s.iter().map(|s| {
                let q = pc.eval(s.t);
                (s.g, q.u, q.v)
            }).collect());
        }
    }
    let (mut ulo, mut uhi, mut vlo, mut vhi) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    for &(_, u, v) in uvc.iter().flatten() {
        if !(u.is_finite() && v.is_finite()) {
            return Ok(None);
        }
        ulo = ulo.min(u);
        uhi = uhi.max(u);
        vlo = vlo.min(v);
        vhi = vhi.max(v);
    }
    let (du, dv) = (uhi - ulo, vhi - vlo);
    // Row and column budget (halved chord on doubly curved surfaces).
    let o = sample::grid_opts(def, sh.opts);
    let o = &o;
    if du <= 0.0 || dv <= 0.0 {
        return Ok(None);
    }
    let eps = 1e-9 * du.max(dv);

    // Assign each chain to a side; collect per side.
    let mut per_side: [Vec<usize>; 4] = [vec![], vec![], vec![], vec![]];
    for (i, pts) in uvc.iter().enumerate() {
        let all = |f: &dyn Fn(&(u32, f64, f64)) -> bool| pts.iter().all(f);
        let s = all(&|p| (p.2 - vlo).abs() <= eps);
        let n = all(&|p| (p.2 - vhi).abs() <= eps);
        let w = all(&|p| (p.1 - ulo).abs() <= eps);
        let e = all(&|p| (p.1 - uhi).abs() <= eps);
        match (s, n, w, e) {
            (true, false, false, false) => per_side[0].push(i),
            (false, true, false, false) => per_side[1].push(i),
            (false, false, true, false) => per_side[2].push(i),
            (false, false, false, true) => per_side[3].push(i),
            _ => return Ok(None),
        }
    }
    if per_side.iter().any(|s| s.is_empty()) {
        return Ok(None);
    }
    let mut sides: Vec<SideSamples> = Vec::with_capacity(4);
    for (k, idxs) in per_side.iter().enumerate() {
        let horizontal = k < 2;
        let coord = |p: &(u32, f64, f64)| if horizontal { p.1 } else { p.2 };
        let mut pieces: Vec<Piece> = idxs
            .iter()
            .map(|&i| {
                let mut v: Vec<(u32, f64)> = uvc[i].iter().map(|p| (p.0, coord(p))).collect();
                if v[0].1 > v[v.len() - 1].1 {
                    v.reverse();
                }
                (chains[i].pole, v)
            })
            .collect();
        pieces.sort_by(|a, b| a.1[0].1.total_cmp(&b.1[0].1));
        if pieces.iter().any(|p| p.0.is_some()) {
            if pieces.len() != 1 || k >= 2 {
                return Ok(None);
            }
            sides.push(SideSamples { pole: pieces[0].0, g: Vec::new(), c: Vec::new() });
            continue;
        }
        let (mut g, mut c): (Vec<u32>, Vec<f64>) = (Vec::new(), Vec::new());
        for (_, v) in pieces {
            let skip = usize::from(!g.is_empty());
            if skip == 1 && (g[g.len() - 1] != v[0].0 || (c[c.len() - 1] - v[0].1).abs() > eps) {
                return Ok(None);
            }
            for &(gg, cc) in &v[skip..] {
                g.push(gg);
                c.push(cc);
            }
        }
        sides.push(SideSamples { pole: None, g, c });
    }
    let [mut s, mut n, w, e]: [SideSamples; 4] = sides.try_into().map_err(|_| TessError::face(face, "internal: side count"))?;

    // Counts: a pole side copies the opposite side's count, or the surface estimate if both are poles.
    let count_u = |s: &SideSamples, n: &SideSamples| -> Result<usize, TessError> {
        match (s.pole, n.pole) {
            (None, _) => Ok(s.g.len() - 1),
            (Some(_), None) => Ok(n.g.len() - 1),
            (Some(_), Some(_)) => row_u_count(surf, def, (ulo, uhi), 0.5 * (vlo + vhi), o).map_err(|r| TessError::face(face, r)),
        }
    };
    let cu = count_u(&s, &n)?;
    for side in [&mut s, &mut n] {
        if let Some(pg) = side.pole {
            side.g = vec![pg; cu + 1];
            side.c = sample::uniform(ulo, uhi, cu);
        }
    }
    if w.g.len() != e.g.len() || w.g.len() < 2 {
        return Ok(None);
    }
    let corners = [(s.g[0], w.g[0]), (s.g[s.g.len() - 1], e.g[0]), (n.g[0], w.g[w.g.len() - 1]), (n.g[n.g.len() - 1], e.g[e.g.len() - 1])];
    if corners.iter().any(|(a, b)| a != b) {
        return Ok(None);
    }
    if w.c.iter().zip(&e.c).any(|(a, b)| (a - b).abs() > eps) {
        return Ok(None);
    }

    let rows = w.g.len() - 1;
    let (a, b) = ((s.g.len() - 1) as f64, (n.g.len() - 1) as f64);
    let mut steiner: Vec<P3> = Vec::new();
    let mut row_g: Vec<Vec<u32>> = Vec::with_capacity(rows + 1);
    let mut row_u: Vec<Vec<f64>> = Vec::with_capacity(rows + 1);
    row_g.push(s.g.clone());
    row_u.push(s.c.clone());
    // Row counts first, with the size budget checked before anything is allocated.
    let mut counts: Vec<usize> = Vec::with_capacity(rows.saturating_sub(1));
    let mut total = 0usize;
    for j in 1..rows {
        let interp = ((a + (b - a) * (j as f64 / rows as f64)).round() as usize).max(1);
        // The row's own iso-v curve may need more points than the linear blend (bulging patches).
        let own = row_u_count(surf, def, (ulo, uhi), w.c[j], o).map_err(|r| TessError::face(face, r))?;
        let k = interp.max(own).max(1);
        total += k - 1;
        if total > sample::MAX_VERTICES {
            return Err(too_large(rows, k));
        }
        counts.push(k);
    }
    if sh.budget.fetch_add(total, std::sync::atomic::Ordering::Relaxed) + total > sample::MAX_VERTICES {
        return Err(TessError::TooLarge(BUDGET_TRIPPED.into()));
    }
    let mut uvs: Vec<Uv> = Vec::new();
    for j in 1..rows {
        let v = w.c[j];
        let k = counts[j - 1];
        let mut g = Vec::with_capacity(k + 1);
        let mut u = Vec::with_capacity(k + 1);
        g.push(w.g[j]);
        u.push(ulo);
        uvs.clear();
        for i in 1..k {
            let uu = ulo + du * (i as f64 / k as f64);
            uvs.push(Uv::new(uu, v));
            u.push(uu);
        }
        let mut pts = vec![sm_core::Point3::default(); uvs.len()];
        surf.eval_many(&uvs, &mut pts);
        for p in pts {
            g.push(sh.g_count + steiner.len() as u32);
            steiner.push([p.x, p.y, p.z]);
        }
        g.push(e.g[j]);
        u.push(uhi);
        row_g.push(g);
        row_u.push(u);
    }
    row_g.push(n.g.clone());
    row_u.push(n.c.clone());

    let mut tris = Vec::new();
    for j in 0..rows {
        zipper(&row_g[j], &row_u[j], &row_g[j + 1], &row_u[j + 1], &mut tris);
    }
    Ok(Some(FaceOut { steiner, tris, boundary_only: false }))
}

/// Merge-triangulate the strip between a lower row `a` and an upper row `b` (both left to
/// right, sharing their first and last parameter). Triangles are counter-clockwise in `(u, v)`.
fn zipper(ag: &[u32], au: &[f64], bg: &[u32], bu: &[f64], tris: &mut Vec<[u32; 3]>) {
    let (na, nb) = (ag.len() - 1, bg.len() - 1);
    let (mut i, mut j) = (0, 0);
    while i < na || j < nb {
        let advance_a = if i == na {
            false
        } else if j == nb {
            true
        } else {
            au[i + 1] <= bu[j + 1]
        };
        if advance_a {
            tris.push([ag[i], ag[i + 1], bg[j]]);
            i += 1;
        } else {
            tris.push([ag[i], bg[j + 1], bg[j]]);
            j += 1;
        }
    }
}

/// Segments along `u` on the iso-line `v = vm`: the circle rule at the line's radius for
/// revolved analytic surfaces (sphere, torus, cone, cylinder), adaptive sampling otherwise.
fn row_u_count(surf: &dyn ParametricSurface, def: &SurfaceDef, (ulo, uhi): (f64, f64), vm: f64, o: &TessOptions) -> Result<usize, String> {
    let span = uhi - ulo;
    let full = (span - core::f64::consts::TAU).abs() < 1e-9;
    let frame = match def {
        SurfaceDef::Sphere(s) => Some(&s.frame),
        SurfaceDef::Torus(t) => Some(&t.frame),
        SurfaceDef::Cone(c) => Some(&c.frame),
        SurfaceDef::Cylinder(c) => Some(&c.frame),
        _ => None,
    };
    if let Some(fr) = frame {
        // Distance of the iso-line from the axis.
        let p = surf.eval(Uv::new(ulo, vm));
        let d = [p.x - fr.origin.x, p.y - fr.origin.y, p.z - fr.origin.z];
        let along = d[0] * fr.z.x + d[1] * fr.z.y + d[2] * fr.z.z;
        let radius = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2] - along * along).max(0.0).sqrt();
        return sample::circle_segments(span, radius, full, o);
    }
    let f = |u: f64| {
        let p = surf.eval(Uv::new(u, vm));
        [p.x, p.y, p.z]
    };
    Ok(sample::adaptive_params(&f, ulo, uhi, o)?.len() - 1)
}

// -------------------------------------------------------------------------------- polygon path

struct Frame2 {
    origin: P3,
    x: P3,
    y: P3,
}

fn dot(a: P3, b: P3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: P3, b: P3) -> P3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn unit(a: P3) -> Option<P3> {
    let l = dot(a, a).sqrt();
    (l > 0.0 && l.is_finite()).then(|| [a[0] / l, a[1] / l, a[2] / l])
}

/// Best-fit plane of the outer loop (Newell normal; the loop runs counter-clockwise about it).
fn newell_frame(pts: &[P3]) -> Option<Frame2> {
    let mut n = [0.0; 3];
    let k = pts.len();
    let mut c = [0.0; 3];
    for i in 0..k {
        let (p, q) = (pts[i], pts[(i + 1) % k]);
        n[0] += (p[1] - q[1]) * (p[2] + q[2]);
        n[1] += (p[2] - q[2]) * (p[0] + q[0]);
        n[2] += (p[0] - q[0]) * (p[1] + q[1]);
        for d in 0..3 {
            c[d] += p[d] / k as f64;
        }
    }
    let n = unit(n)?;
    let axis = if n[0].abs() <= n[1].abs() && n[0].abs() <= n[2].abs() {
        [1.0, 0.0, 0.0]
    } else if n[1].abs() <= n[2].abs() {
        [0.0, 1.0, 0.0]
    } else {
        [0.0, 0.0, 1.0]
    };
    let x = unit(cross(axis, n))?;
    let y = cross(n, x);
    Some(Frame2 { origin: c, x, y })
}

fn polygon_face(face: EntityId, surface: Option<(Option<&dyn ParametricSurface>, &SurfaceDef)>, rings: &[Vec<Chain<'_>>]) -> Result<FaceOut, TessError> {
    // Boundary points of each ring: all samples but the last of every chain (it is the next
    // chain's first); pole coedges contribute nothing.
    let ring_samples: Vec<Vec<(Bs, Option<&PCurve>)>> = rings
        .iter()
        .map(|r| r.iter().filter(|c| c.pole.is_none()).flat_map(|c| c.s[..c.s.len() - 1].iter().map(move |s| (*s, c.pcurve))).collect())
        .collect();
    if ring_samples[0].len() < 3 {
        return Err(TessError::face(face, "outer loop has fewer than 3 boundary points"));
    }
    let planar: Option<Frame2> = match surface {
        Some((_, SurfaceDef::Plane(p))) => {
            let f = &p.frame;
            Some(Frame2 { origin: [f.origin.x, f.origin.y, f.origin.z], x: [f.x.x, f.x.y, f.x.z], y: [f.y.x, f.y.y, f.y.z] })
        }
        Some(_) => None,
        None => {
            let outer: Vec<P3> = ring_samples[0].iter().map(|(s, _)| s.p).collect();
            Some(newell_frame(&outer).ok_or_else(|| TessError::face(face, "face has no surface and its outer loop is degenerate"))?)
        }
    };
    let mut prev_uv: Option<Uv> = None;
    let mut rp: Vec<Vec<RingPt>> = Vec::with_capacity(rings.len());
    for ring in &ring_samples {
        let mut pts = Vec::with_capacity(ring.len());
        for (s, pc) in ring {
            let uv = if let Some(f) = &planar {
                let l = [s.p[0] - f.origin[0], s.p[1] - f.origin[1], s.p[2] - f.origin[2]];
                [dot(l, f.x), dot(l, f.y)]
            } else {
                let (surf, _) = surface.ok_or_else(|| TessError::face(face, "internal: no surface"))?;
                if let (Some(pc), true) = (pc, s.t.is_finite()) {
                    let q = pc.eval(s.t);
                    [q.u, q.v]
                } else {
                    let surf = surf.ok_or_else(|| TessError::face(face, "implicit-only surfaces are not supported"))?;
                    let d = surf.domain();
                    let periodic = |a: &Domain1| a.is_periodic();
                    if periodic(&d.u) || periodic(&d.v) {
                        return Err(TessError::face(face, "a face on a periodic surface needs pcurves on its edges"));
                    }
                    let pr = surf.project(sm_core::Point3::new(s.p[0], s.p[1], s.p[2]), prev_uv).map_err(|e| TessError::face(face, format!("surface projection failed: {e}")))?;
                    prev_uv = Some(pr.uv);
                    [pr.uv.u, pr.uv.v]
                }
            };
            pts.push((uv, s.g));
        }
        rp.push(pts);
    }
    let tris = earcut::triangulate(&rp).map_err(|e| TessError::face(face, e))?;
    Ok(FaceOut { steiner: Vec::new(), tris, boundary_only: false })
}
