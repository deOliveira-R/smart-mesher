//! Tetrahedral volume scheme: Delaunay insertion, boundary recovery, refinement, optimization.
//!
//! See `DESIGN.md`. The pure core is [`tetrahedralize`] (closed oriented triangle surface in,
//! tets out; boundary points and triangles frozen) and [`delaunay`] (convex hull
//! tetrahedralization). The scheme adapter is [`TetVolume`] and [`register`].

mod canon;
mod delaunay;
mod error;
mod geom;
mod tds;
mod validate;

pub use error::TetError;
pub use geom::{in_sphere, ori};

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
    /// A constant target edge length (combined with the boundary size by `min`).
    Constant(f64),
    /// A user size field `h(x)` (combined with the boundary size by `min`).
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

/// Statistics of a run. `stage_ms` is excluded from equality and from every hash.
#[derive(Clone, Debug, Default)]
pub struct TetStats {
    /// Interior Steiner points in the output.
    pub n_steiner: usize,
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
    Ok(TetMesh { points: points.to_vec(), n_input: points.len(), tets, stats: TetStats::default() })
}
