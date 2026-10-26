//! Statistics of a polyhedral mesh (`mesh.poly_info`).

use crate::geom::{self, V3};
use rayon::prelude::*;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_mesh_api::{CellIx, FaceIx, MeshEntityKind, MeshRead, cell_volume, face_area_vector};

/// Fixed chunk size of every reduction, so sums do not depend on the thread count.
pub(crate) const CHUNK: usize = 2048;

/// Minimum, mean and maximum of a count.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Spread {
    /// Smallest value.
    pub min: usize,
    /// Mean value.
    pub mean: f64,
    /// Largest value.
    pub max: usize,
}

/// Statistics of a polyhedral mesh, as returned by [`poly_stats`] and `mesh.poly_info`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[non_exhaustive]
pub struct PolyStats {
    /// Number of cells.
    pub cells: usize,
    /// Number of faces.
    pub faces: usize,
    /// Number of vertices.
    pub vertices: usize,
    /// Faces per cell.
    pub faces_per_cell: Spread,
    /// Vertices per face.
    pub vertices_per_face: Spread,
    /// Largest planarity measure of any face (the `GeomTol::planarity` metric).
    pub max_nonplanarity: f64,
    /// Smallest cell volume.
    pub volume_min: f64,
    /// Largest cell volume.
    pub volume_max: f64,
    /// Sum of cell volumes.
    pub volume_sum: f64,
    /// Total area of the boundary faces.
    pub boundary_area: f64,
}

fn spread(values: impl Iterator<Item = usize>) -> Spread {
    let (mut n, mut sum, mut min, mut max) = (0usize, 0usize, usize::MAX, 0usize);
    for v in values {
        n += 1;
        sum += v;
        min = min.min(v);
        max = max.max(v);
    }
    if n == 0 {
        return Spread::default();
    }
    Spread { min, mean: sum as f64 / n as f64, max }
}

/// The `GeomTol::planarity` metric of face `f` of a finished mesh (0 for triangles).
pub fn face_nonplanarity(m: &dyn MeshRead, f: FaceIx) -> f64 {
    let l = m.face_vertices(f);
    if l.len() <= 3 {
        return 0.0;
    }
    let co = m.coords();
    let pts: smallvec::SmallVec<[V3; 16]> = l
        .iter()
        .map(|v| [co.x[v.get()], co.y[v.get()], co.z[v.get()]])
        .collect();
    geom::nonplanarity(&pts)
}

/// Largest planarity measure over all faces, a fixed-order reduction.
pub(crate) fn max_nonplanarity(m: &dyn MeshRead) -> f64 {
    let nf = m.count(MeshEntityKind::Face);
    (0..nf.div_ceil(CHUNK))
        .into_par_iter()
        .map(|c| {
            (c * CHUNK..((c + 1) * CHUNK).min(nf))
                .map(|f| face_nonplanarity(m, FaceIx::new(f)))
                .fold(0.0, f64::max)
        })
        .collect::<Vec<_>>()
        .into_iter()
        .fold(0.0, f64::max)
}

/// `(min, max, sum)` of the cell volumes; chunks are summed in index order.
pub(crate) fn volume_range(m: &dyn MeshRead) -> (f64, f64, f64) {
    let nc = m.count(MeshEntityKind::Cell);
    if nc == 0 {
        return (0.0, 0.0, 0.0);
    }
    let parts: Vec<(f64, f64, f64)> = (0..nc.div_ceil(CHUNK))
        .into_par_iter()
        .map(|c| {
            let (mut lo, mut hi, mut sum) = (f64::INFINITY, f64::NEG_INFINITY, 0.0);
            for i in c * CHUNK..((c + 1) * CHUNK).min(nc) {
                let v = cell_volume(m, CellIx::new(i));
                lo = lo.min(v);
                hi = hi.max(v);
                sum += v;
            }
            (lo, hi, sum)
        })
        .collect();
    let mut acc = (f64::INFINITY, f64::NEG_INFINITY, 0.0);
    for (lo, hi, s) in parts {
        acc = (acc.0.min(lo), acc.1.max(hi), acc.2 + s);
    }
    acc
}

/// Statistics of any mesh (meant for polyhedral ones).
pub fn poly_stats(m: &dyn MeshRead) -> PolyStats {
    let (nc, nf, nv) = (
        m.count(MeshEntityKind::Cell),
        m.count(MeshEntityKind::Face),
        m.count(MeshEntityKind::Vertex),
    );
    let (volume_min, volume_max, volume_sum) = volume_range(m);
    let parts: Vec<f64> = (0..nf.div_ceil(CHUNK))
        .into_par_iter()
        .map(|c| {
            (c * CHUNK..((c + 1) * CHUNK).min(nf))
                .filter(|&f| m.is_boundary_face(FaceIx::new(f)) && m.face_active(FaceIx::new(f)))
                .map(|f| face_area_vector(m, FaceIx::new(f)).norm())
                .sum::<f64>()
        })
        .collect();
    PolyStats {
        cells: nc,
        faces: nf,
        vertices: nv,
        faces_per_cell: spread((0..nc).map(|c| m.cell_faces(CellIx::new(c)).len())),
        vertices_per_face: spread((0..nf).map(|f| m.face_vertices(FaceIx::new(f)).len())),
        max_nonplanarity: max_nonplanarity(m),
        volume_min,
        volume_max,
        volume_sum,
        boundary_area: parts.into_iter().sum(),
    }
}
