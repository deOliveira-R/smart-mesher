//! Statistics of a polyhedral mesh (`mesh.poly_info`).

use crate::geom::{self, V3};
use crate::measure::{self, DualEligibility};
use rayon::prelude::*;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_mesh_api::{CellIx, FaceIx, MeshEntityKind, MeshRead, TagValues, cell_volume, face_area_vector};
use std::collections::BTreeMap;

/// Fixed chunk size of every reduction, so sums do not depend on the thread count.
pub(crate) const CHUNK: usize = 2048;

/// Name of the cell tag column `dualize` adds when it split cells: per output cell, the number of
/// wedges of the dual cell it came from (0 if that cell was not split). Absent when nothing was
/// split.
pub const WEDGE_TAG: &str = "poly.wedges";

/// Names of the three `f64` cell tag columns (x, y, z) that a circumcentric dual adds: the
/// generator point of each cell, the primal vertex whose Voronoi cell it is. The face between two
/// cells lies in the bisector plane of their generators.
pub const GENERATOR_TAGS: [&str; 3] = ["poly.generator.x", "poly.generator.y", "poly.generator.z"];

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
    /// Output cells that came from split dual cells (see `DualReport::wedges`); read from the
    /// [`WEDGE_TAG`] column, 0 if there is none. Cells merged by agglomeration are counted once.
    pub wedges: usize,
    /// Dual cells that were split (see `DualReport::split_cells`); with `wedges`, the poly cell
    /// count is `(vertex, region) pairs + wedges - split_cells - merges`.
    pub split_cells: usize,
    /// Convex cells in the `sm-rt` sense (all faces planar, no vertex outside a face plane),
    /// whatever their face count. Planarity tolerance `sm_quality::RT_PLANARITY_TOL`.
    #[serde(default)]
    pub convex_cells: usize,
    /// Cells the `sm-rt` fast path (Cyrus-Beck walk) takes: convex and at most
    /// `sm_quality::CONVEX_CHECK_MAX` (64) faces.
    #[serde(default)]
    pub fast_path_cells: usize,
    /// Cells with more than 64 faces.
    #[serde(default)]
    pub over_cap_cells: usize,
    /// Convex cells with more than 64 faces (what raising `CONVEX_CHECK_MAX` would recover).
    #[serde(default)]
    pub convex_over_cap_cells: usize,
    /// Largest `cell_convexity` of any cell (0 if all are convex).
    #[serde(default)]
    pub max_cell_convexity: f64,
    /// Pairs of cells that share more than one interior face (hub splitting makes these).
    #[serde(default)]
    pub multi_face_pairs: usize,
    /// Share of interior faces that belong to a multi-face pair (0 without interior faces).
    /// Hub-split faces always do, so this is the hub-split share up to coincidences.
    #[serde(default)]
    pub hub_split_share: f64,
    /// Dual-eligibility counts, only for an all-tet mesh (see [`crate::dual_eligibility`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dual_eligibility: Option<DualEligibility>,
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

/// `(wedges, split_cells)` from the [`WEDGE_TAG`] column: cells with tag `n > 0` are wedges, and
/// each split cell contributes `n` of them, so `split_cells = sum(1 / n)`.
fn wedge_counts(m: &dyn MeshRead) -> (usize, usize) {
    let Some(TagValues::I64(v)) = m.tag(MeshEntityKind::Cell, WEDGE_TAG).map(|t| &t.values) else {
        return (0, 0);
    };
    let mut per_n: BTreeMap<i64, usize> = BTreeMap::new();
    for &n in v.iter().filter(|&&n| n > 0) {
        *per_n.entry(n).or_insert(0) += 1;
    }
    let wedges = per_n.values().sum();
    // after agglomeration a count need not be a multiple of n: round up
    let split = per_n.iter().map(|(&n, &k)| k.div_ceil(n as usize)).sum();
    (wedges, split)
}

/// Statistics of any mesh (meant for polyhedral ones).
pub fn poly_stats(m: &dyn MeshRead) -> PolyStats {
    let (wedges, split_cells) = wedge_counts(m);
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
    let cv = sm_quality::convexity_report(m, sm_quality::RT_PLANARITY_TOL);
    let (multi_face_pairs, in_multi, interior) = measure::multi_face_pairs(m);
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
        wedges,
        split_cells,
        convex_cells: cv.convex,
        fast_path_cells: cv.eligible,
        over_cap_cells: cv.over_cap,
        convex_over_cap_cells: cv.convex_over_cap,
        max_cell_convexity: cv.max_convexity,
        multi_face_pairs,
        hub_split_share: if interior > 0 { in_multi as f64 / interior as f64 } else { 0.0 },
        dual_eligibility: measure::dual_eligibility(m),
    }
}
