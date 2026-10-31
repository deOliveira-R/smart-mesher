//! Tetrahedral volume scheme: Delaunay insertion, boundary recovery, refinement, optimization.
//!
//! See `DESIGN.md`. The pure core is [`tetrahedralize`] (closed oriented triangle surface in,
//! tets out; boundary points and triangles frozen) and [`delaunay`] (convex hull
//! tetrahedralization). The scheme adapter is [`TetVolume`] and [`register`].

mod canon;
mod delaunay;
mod error;
mod exact;
mod fill;
mod flips;
mod geom;
mod guard;
mod optimize;
mod pipeline;
mod quality;
mod refine;
mod relocate;
mod scheme;
mod recover;
mod spacing;
mod steiner;
mod tds;
mod validate;

pub use error::TetError;
pub use geom::{in_sphere, ori};
pub use pipeline::tetrahedralize;
pub use scheme::{NAME, TetParams, TetVolume, register};
pub use quality::{TetQ, tet_quality};
pub use spacing::{MAX_DIAGNOSTICS, MIN_DIHEDRAL_DEG, MIN_MEAN_RATIO, TARGET_R};

use geom::V3;

/// A closed, oriented triangle surface to fill with tets.
///
/// Triangles are wound so that the right-hand normal points **out of** the region to mesh.
/// Shells (edge-connected components) are derived: one outer shell (positive enclosed volume)
/// and any number of cavity shells (normals into the cavity). Points and triangles are frozen.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SurfaceInput {
    /// Boundary points.
    pub points: Vec<[f64; 3]>,
    /// Boundary triangles (indices into `points`).
    pub tris: Vec<[u32; 3]>,
}

/// Element size control.
#[derive(Clone, Copy)]
pub enum SizeSpec<'a> {
    /// Size from the boundary mesh: the mean incident boundary edge length at each boundary
    /// vertex, graded into the interior.
    FromBoundary,
    /// A constant target edge length (combined with the boundary size by `min`, but never
    /// finer than the graded floor from the frozen boundary: next to a coarse boundary the
    /// target is `max_c (h_c - (gradation - 1)|p - c|)` at least, so a value far below the
    /// boundary edge length is only partly honoured; see `DESIGN.md`, deviation 6).
    Constant(f64),
    /// A user size field `h(x)` (combined with the boundary size by `min`, with the same floor as
    /// [`SizeSpec::Constant`]).
    Field(&'a (dyn Fn([f64; 3]) -> f64 + Sync)),
}

impl core::fmt::Debug for SizeSpec<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            SizeSpec::FromBoundary => write!(f, "FromBoundary"),
            SizeSpec::Constant(c) => write!(f, "Constant({c})"),
            SizeSpec::Field(_) => write!(f, "Field(..)"),
        }
    }
}

/// Options of [`tetrahedralize`].
#[derive(Clone, Debug)]
pub struct TetOptions<'a> {
    /// Size control.
    pub size: SizeSpec<'a>,
    /// Refine tets whose circumradius / shortest edge exceeds this (default 2.0).
    pub radius_edge: f64,
    /// Size gradation factor `>= 1` (default 1.3).
    pub gradation: f64,
    /// Optimization level: 0 none, 1 flips, 2 flips and smoothing (default 2), 3 reserved.
    pub optimize: u8,
    /// Maximum number of interior Steiner points (default `8 * n_input + 100_000`; 0 selects
    /// the default).
    pub max_steiner: usize,
    /// Dihedral angle goal in degrees for the (reserved) level 3 (default 15).
    pub min_dihedral_goal_deg: f64,
}

impl Default for TetOptions<'_> {
    fn default() -> Self {
        Self {
            size: SizeSpec::FromBoundary,
            radius_edge: 2.0,
            gradation: 1.3,
            optimize: 2,
            max_steiner: 0,
            min_dihedral_goal_deg: 15.0,
        }
    }
}

/// Distribution of the normalized spacing `r = d / h_loc` of the interior Steiner points
/// (ruling INT-7): `d` the distance to the nearest input triangle (or the boundary vertex on
/// it), `h_loc` the size field at the point. `r` is clipped at `cap`. All zero without points.
#[derive(Clone, Copy, Debug, Default)]
pub struct SpacingStats {
    /// Number of interior Steiner points measured.
    pub count: usize,
    /// Mean of `r`.
    pub mean: f64,
    /// Standard deviation of `r` (population).
    pub std_dev: f64,
    /// 1st percentile of `r` (lowest-rank method: the value at index `floor(0.01 (count - 1))` of the sorted list).
    pub p01: f64,
    /// Smallest `r`.
    pub min: f64,
    /// The objective: the target value of `r` ([`TARGET_R`]).
    pub target: f64,
    /// Root mean square deviation of `r` from `target` (the objective's value).
    pub rms_dev: f64,
    /// Clip value of `r` (a point farther than the search reach has at least this).
    pub cap: f64,
}

impl PartialEq for SpacingStats {
    fn eq(&self, o: &Self) -> bool {
        self.count == o.count
            && [self.mean, self.std_dev, self.p01, self.min, self.target, self.rms_dev, self.cap].map(f64::to_bits)
                == [o.mean, o.std_dev, o.p01, o.min, o.target, o.rms_dev, o.cap].map(f64::to_bits)
    }
}

/// The measure a [`QualityDiagnostic`] is about.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum QualityMetric {
    /// Smallest dihedral angle of a tet, degrees (floor [`MIN_DIHEDRAL_DEG`]).
    MinDihedral,
    /// Mean ratio of a tet (floor [`MIN_MEAN_RATIO`]); a proxy for cell quality.
    MeanRatio,
}

/// What would probably cure a [`QualityDiagnostic`].
#[derive(Clone, Copy, Debug)]
pub enum SuggestedFix {
    /// Refine the size field near the position to `h` (the tet has a Steiner vertex).
    RefineSizeNear {
        /// Suggested local size.
        h: f64,
    },
    /// The tet spans input vertices only, which are frozen: remesh the listed input triangles
    /// finer (about `h`) or smooth the surface there.
    RefineInputSurface {
        /// Suggested local surface size.
        h: f64,
    },
}

impl PartialEq for SuggestedFix {
    fn eq(&self, o: &Self) -> bool {
        match (self, o) {
            (Self::RefineSizeNear { h: a }, Self::RefineSizeNear { h: b }) | (Self::RefineInputSurface { h: a }, Self::RefineInputSurface { h: b }) => a.to_bits() == b.to_bits(),
            _ => false,
        }
    }
}

/// A tet below the quality floor, located (ruling INT-7). Never hidden, never an error.
#[derive(Clone, Debug)]
pub struct QualityDiagnostic {
    /// Centroid of the tet.
    pub position: [f64; 3],
    /// Nearby input triangles (indices into `SurfaceInput::tris` as given by the caller, the
    /// owners of the surface there), sorted ascending, at most 6.
    pub triangles: Vec<u32>,
    /// The measure that is below its floor.
    pub metric: QualityMetric,
    /// Measured value.
    pub value: f64,
    /// The floor.
    pub threshold: f64,
    /// Suggested fix.
    pub fix: SuggestedFix,
}

impl PartialEq for QualityDiagnostic {
    fn eq(&self, o: &Self) -> bool {
        self.position.map(f64::to_bits) == o.position.map(f64::to_bits)
            && self.triangles == o.triangles
            && self.metric == o.metric
            && self.value.to_bits() == o.value.to_bits()
            && self.threshold.to_bits() == o.threshold.to_bits()
            && self.fix == o.fix
    }
}

/// Statistics of a run. `stage_ms` is excluded from equality and from every hash.
#[derive(Clone, Debug, Default)]
pub struct TetStats {
    /// Interior Steiner points in the output (`n_steiner_recovery + n_steiner_refine`).
    pub n_steiner: usize,
    /// Interior Steiner points in the output that boundary recovery added (including the
    /// zero-volume repair before refinement).
    pub n_steiner_recovery: usize,
    /// Interior Steiner points in the output that refinement added.
    pub n_steiner_refine: usize,
    /// Temporary boundary Steiner points that complete recovery inserted on input edges and
    /// triangles and then moved off the surface (none remain in the output).
    pub n_boundary_steiner: usize,
    /// Constraint-lookup entries (subsegments, tiles and their edges) inserted or removed by
    /// complete recovery: its work measure, linear in the surface size plus the temporary
    /// points (0 when complete recovery did not run).
    pub n_constraint_updates: u64,
    /// Interior Steiner points inside the solid closer than ruling S4 allows (`0.3 h_T` to an
    /// input triangle `T`, `h_T` its mean edge length), after every attempt to move or remove
    /// them; 0 when S4 holds everywhere. Thin parts can make S4 infeasible.
    pub n_s4_violations: usize,
    /// Smallest `dist(p, T) / h_T` over those points, capped at 0.3 (exactly 0.3 when
    /// `n_s4_violations == 0`; 0 only in a `Default` value).
    pub worst_s4_margin: f64,
    /// Distribution of the normalized spacing of the interior Steiner points (ruling INT-7).
    pub spacing: SpacingStats,
    /// Tets below the quality floor (a tet counts once per metric): the exact count.
    pub n_quality_violations: usize,
    /// The worst [`MAX_DIAGNOSTICS`] of them, located, worst first by `value / threshold` (ruling INT-7).
    pub quality_diagnostics: Vec<QualityDiagnostic>,
    /// Times the constraint lookup of complete recovery was rebuilt from scratch (0: it is
    /// always updated incrementally).
    pub n_lookup_rebuilds: u64,
    /// Points the post-classification removal pass did not try or finish because its work cap
    /// was reached (0 when it ran to completion).
    pub n_removal_deferred: usize,
    /// Flips performed (recovery and optimization).
    pub flips: usize,
    /// Smoothing passes performed.
    pub smooth_passes: usize,
    /// Tets that stayed bad because no admissible refinement point existed.
    pub unrefinable: usize,
    /// Smallest dihedral angle (degrees).
    pub min_dihedral_deg: f64,
    /// Largest dihedral angle (degrees).
    pub max_dihedral_deg: f64,
    /// Largest circumradius / shortest-edge ratio.
    pub max_radius_edge: f64,
    /// Smallest mean ratio `12 (3V)^(2/3) / sum l^2`.
    pub min_mean_ratio: f64,
    /// Mean of the mean ratio.
    pub mean_mean_ratio: f64,
    /// Wall time per stage in ms: validate, delaunay, recover+classify, refine, optimize.
    pub stage_ms: [f64; 5],
}

impl PartialEq for TetStats {
    fn eq(&self, o: &Self) -> bool {
        self.n_steiner == o.n_steiner
            && self.n_steiner_recovery == o.n_steiner_recovery
            && self.n_steiner_refine == o.n_steiner_refine
            && self.n_boundary_steiner == o.n_boundary_steiner
            && self.n_constraint_updates == o.n_constraint_updates
            && self.n_s4_violations == o.n_s4_violations
            && self.worst_s4_margin.to_bits() == o.worst_s4_margin.to_bits()
            && self.spacing == o.spacing
            && self.n_quality_violations == o.n_quality_violations
            && self.quality_diagnostics == o.quality_diagnostics
            && self.n_lookup_rebuilds == o.n_lookup_rebuilds
            && self.n_removal_deferred == o.n_removal_deferred
            && self.flips == o.flips
            && self.smooth_passes == o.smooth_passes
            && self.unrefinable == o.unrefinable
            && self.min_dihedral_deg.to_bits() == o.min_dihedral_deg.to_bits()
            && self.max_dihedral_deg.to_bits() == o.max_dihedral_deg.to_bits()
            && self.max_radius_edge.to_bits() == o.max_radius_edge.to_bits()
            && self.min_mean_ratio.to_bits() == o.min_mean_ratio.to_bits()
            && self.mean_mean_ratio.to_bits() == o.mean_mean_ratio.to_bits()
    }
}

/// A tetrahedral mesh.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TetMesh {
    /// Points: the first `n_input` are the input points (bit-identical), then Steiner points in
    /// creation order.
    pub points: Vec<[f64; 3]>,
    /// Number of input points.
    pub n_input: usize,
    /// Tets in `CellShape::Tet` template order (positive volume), canonically rotated and sorted.
    pub tets: Vec<[u32; 4]>,
    /// Statistics.
    pub stats: TetStats,
}

/// Delaunay tetrahedralization of the convex hull of `points` (stage 2 alone).
///
/// Exact predicates with symbolic perturbation make the result unique. Tets of exactly zero
/// volume (degenerate cospherical/coplanar input) are omitted.
pub fn delaunay(points: &[[f64; 3]]) -> Result<TetMesh, TetError> {
    validate::check_points(points)?;
    if points.len() < 4 {
        return Err(TetError::BadOption("delaunay needs at least 4 points".into()));
    }
    // Perturbation ids are the lexicographic coordinate ranks, so the triangulation of a point
    // set does not depend on the order it is presented in.
    let order = validate::lex_order(points);
    let sorted: Vec<V3> = order.iter().map(|&i| points[i as usize]).collect();
    let dt = delaunay::build(sorted.clone(), None, None)?;
    let mut tets: Vec<[u32; 4]> = Vec::with_capacity(dt.tds.len());
    for t in 0..dt.tds.slots() as u32 {
        if !dt.tds.alive[t as usize] || dt.tds.ghost_slot(t).is_some() {
            continue;
        }
        let v = dt.tds.verts[t as usize];
        if dt.sos().ori(v[0], v[1], v[2], v[3]) > 0 {
            tets.push(v.map(|x| order[x as usize]));
        }
    }
    canon::canonical_tets(&mut tets);
    Ok(TetMesh { points: points.to_vec(), n_input: points.len(), tets, stats: TetStats { worst_s4_margin: guard::S4, ..TetStats::default() } })
}
