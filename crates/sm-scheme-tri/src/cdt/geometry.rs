//! The geometry seam of the kernel and the option/report types of refinement.

use sm_core::Point3;

/// How chart points map to the surface, and how big triangles should be there.
///
/// The kernel is exact in the chart and measures quality in 3-D through this trait.
pub trait Geometry2 {
    /// The surface point of chart point `q`.
    fn lift(&self, q: [f64; 2]) -> Point3;

    /// The target edge length at the surface point `p` (positive).
    fn size(&self, p: Point3) -> f64;

    /// True if the chart is an isometry of the surface (plane, cylinder): chart-Delaunay is
    /// then exactly the right criterion and flips use the exact in-circle test.
    fn is_isometric(&self) -> bool;

    /// The first fundamental form of the chart at `q` as `[g11, g12, g22]` (the squared length of
    /// a chart displacement `d` on the surface is `g11 dx^2 + 2 g12 dx dy + g22 dy^2`). `None`
    /// (the default) means the identity: circumcenters are then the plain chart circumcenters.
    /// Non-isometric charts return the form so circumcenters are those of the local surface
    /// metric, which makes refinement converge where the chart is distorted.
    fn chart_metric(&self, q: [f64; 2]) -> Option<[f64; 3]> {
        let _ = q;
        None
    }

    /// A matrix `A` (row-major) with `|A e|^2` the squared *metric length* of the 3-D vector `e`
    /// at surface point `p`. `None` (the default) means the isotropic length with target
    /// [`Geometry2::size`]; `Some` means quality is measured in the metric and the target length
    /// is 1 (anisotropic size fields).
    fn metric_map(&self, p: Point3) -> Option<[[f64; 3]; 3]> {
        let _ = p;
        None
    }
}

/// Options of [`crate::cdt::Cdt::refine`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RefineOptions {
    /// A triangle whose 3-D minimum angle is below this (degrees) is bad.
    pub min_angle: f64,
    /// A triangle whose 3-D circumradius exceeds `size_factor * h * 0.67` is bad.
    pub size_factor: f64,
    /// Stop when the mesh would exceed this many points.
    pub max_points: usize,
}

impl Default for RefineOptions {
    fn default() -> Self {
        Self {
            min_angle: 20.0,
            size_factor: 1.0,
            max_points: 2_000_000,
        }
    }
}

/// What [`crate::cdt::Cdt::refine`] did.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct RefineReport {
    /// Steiner points inserted.
    pub inserted: usize,
    /// Bad triangles (vertex ids) whose circumcenter would encroach a constrained segment or
    /// leave the domain, or whose insertion failed: left as they are (Triangle's `-Y`).
    pub unfixable: Vec<[u32; 3]>,
    /// True if refinement stopped at `max_points`.
    pub truncated: bool,
}

/// Options of [`crate::cdt::Cdt::optimize`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OptimizeOptions {
    /// Maximum number of Jacobi smoothing sweeps (default 4).
    pub sweeps: u32,
}

impl Default for OptimizeOptions {
    fn default() -> Self {
        Self { sweeps: 4 }
    }
}
