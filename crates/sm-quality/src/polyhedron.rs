//! Metrics of general polyhedra, computed from the mesh's face loops.
//!
//! Generalizations (see `DESIGN.md`): volume under the mesh geometry rule, edge ratio over all
//! face edges, min/max interior dihedral angle over edges shared by two faces, Wadell
//! sphericity and star-shapedness about the vertex mean.

use crate::faces;
use crate::metric::Metric;
use crate::vec::{DBL_MAX, TINY, V, cap, norm, sub};
use sm_mesh_api::{CellIx, MeshRead, Side};
use smallvec::SmallVec;

/// Evaluate a polyhedron metric for cell `c` (`None` if the metric is undefined for polyhedra).
pub(crate) fn eval(mesh: &dyn MeshRead, c: CellIx, metric: Metric) -> Option<f64> {
    if !metric.applies(sm_mesh_api::CellShape::Polyhedron) {
        return None;
    }
    let co = mesh.coords();
    let sfs = mesh.cell_faces(c);
    // Outward loops in global vertex ids.
    let mut ids: SmallVec<[u32; 96]> = SmallVec::new();
    let mut spans: SmallVec<[(u32, u32); 16]> = SmallVec::new();
    for sf in sfs {
        let l = mesh.face_vertices(sf.face());
        let start = ids.len() as u32;
        match sf.side() {
            Side::Left => ids.extend(l.iter().map(|v| v.0)),
            Side::Right => ids.extend(l.iter().rev().map(|v| v.0)),
        }
        spans.push((start, l.len() as u32));
    }
    let mut uniq: SmallVec<[u32; 32]> = ids.iter().copied().collect();
    uniq.sort_unstable();
    uniq.dedup();
    let pts: SmallVec<[V; 32]> = uniq.iter().map(|&v| [co.x[v as usize], co.y[v as usize], co.z[v as usize]]).collect();
    if pts.iter().any(|q| q.iter().any(|x| !x.is_finite())) {
        return Some(f64::NAN);
    }
    let local: SmallVec<[u32; 96]> = ids.iter().map(|v| uniq.binary_search(v).unwrap_or(0) as u32).collect();
    let loops: SmallVec<[&[u32]; 16]> = spans.iter().map(|&(s, n)| &local[s as usize..(s + n) as usize]).collect();

    let v = match metric {
        Metric::Volume => faces::fan_volume(&pts, &loops),
        Metric::EdgeRatio => {
            let (mut lo, mut hi) = (f64::INFINITY, 0.0f64);
            for l in &loops {
                for i in 0..l.len() {
                    let d = norm(sub(pts[l[i] as usize], pts[l[(i + 1) % l.len()] as usize]));
                    lo = lo.min(d);
                    hi = hi.max(d);
                }
            }
            if lo <= TINY { DBL_MAX } else { cap(hi / lo) }
        }
        Metric::MinAngle => faces::dihedral_extrema(&pts, &loops)?.0,
        Metric::MaxAngle => faces::dihedral_extrema(&pts, &loops)?.1,
        Metric::Sphericity => faces::sphericity(&pts, &loops),
        Metric::Starness => faces::starness(&pts, &loops),
        _ => return None,
    };
    Some(v)
}

/// `(volume, surface area, starness)` for validity checks.
pub(crate) fn validity_inputs(mesh: &dyn MeshRead, c: CellIx) -> (f64, f64, f64) {
    let g = |m| eval(mesh, c, m).unwrap_or(f64::NAN);
    let v = g(Metric::Volume);
    let star = g(Metric::Starness);
    // Surface area from the mesh geometry rule, allocation free.
    let s: f64 = mesh
        .cell_faces(c)
        .iter()
        .map(|sf| {
            let a = sm_mesh_api::geometry::face_area_vector(mesh, sf.face());
            (a.x * a.x + a.y * a.y + a.z * a.z).sqrt()
        })
        .sum();
    (v, s, star)
}
