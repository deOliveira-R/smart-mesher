//! The metric catalogue and the element-level dispatcher.

use crate::vec::V;
use crate::{planar, solid};
use sm_mesh_api::CellShape;

/// A quality metric. Names follow the Verdict library; the formulas are in `DESIGN.md`.
///
/// Not every metric is defined for every shape: see [`Metric::applies`]. In a 2-D mesh
/// "volume" means area.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum Metric {
    /// Cell volume (area in 2-D) under the mesh geometry rule `smartmesh.fan_vertex_mean.v1`.
    Volume,
    /// Minimum corner Jacobian (unnormalised determinant of the corner edge vectors).
    Jacobian,
    /// Minimum corner Jacobian normalised by edge lengths (sine of the corner angle for a quad).
    ScaledJacobian,
    /// Verdict aspect ratio: tri/quad/polygon perimeter form, tet `beta` form.
    AspectRatio,
    /// Frobenius aspect (triangle and tetrahedron; equals the condition number for a triangle).
    AspectFrobenius,
    /// Maximum over corners of the Frobenius aspect.
    MaxAspectFrobenius,
    /// Mean over corners of the Frobenius aspect (Verdict's "med").
    MeanAspectFrobenius,
    /// Tetrahedron aspect `gamma` (rms edge cubed over volume).
    AspectGamma,
    /// Circumradius over (dimension times) inradius (triangle, tetrahedron).
    RadiusRatio,
    /// Tetrahedron collapse ratio (smallest height over the longest edge of the opposite face).
    CollapseRatio,
    /// Longest edge over shortest edge.
    EdgeRatio,
    /// Ratio of the longer to the shorter principal axis (quad, hex).
    MaxEdgeRatio,
    /// Largest `|cos|` between principal axes (quad, hex).
    Skew,
    /// Largest cross-derivative over the shorter principal axis (quad, hex).
    Taper,
    /// Quad warpage `min cos^3` of opposite corner normals (1 = flat).
    Warpage,
    /// Shortest edge over longest diagonal, scaled so the ideal element is 1 (quad, hex).
    Stretch,
    /// Scaled Jacobian floored at zero (quad, hex).
    Shear,
    /// Oddy metric (quad, hex); 0 for a perfect cube/square.
    Oddy,
    /// Shortest over longest diagonal (hex).
    Diagonal,
    /// Cube root of the volume (hex characteristic length).
    Dimension,
    /// Condition number of the (weighted) corner Jacobian matrices, maximum over corners.
    Condition,
    /// Minimum Jacobian over Gauss points over the mean Jacobian.
    Distortion,
    /// Knupp shape metric (minimum over corners).
    Shape,
    /// `min(V/Vref, Vref/V)^2`; `Vref` is the reference size of the mesh.
    RelativeSizeSquared,
    /// `RelativeSizeSquared * Shape`.
    ShapeAndSize,
    /// `RelativeSizeSquared * Shear`.
    ShearAndSize,
    /// Smallest interior angle in degrees (2-D) or smallest interior dihedral angle (3-D).
    MinAngle,
    /// Largest interior angle in degrees (2-D) or largest interior dihedral angle (3-D).
    MaxAngle,
    /// Wadell sphericity `pi^(1/3) (6V)^(2/3) / S` of a 3-D cell (1 for a sphere).
    Sphericity,
    /// Smallest cosine between an outward face normal and the direction from the vertex mean to
    /// the face mean; positive if and only if the cell is star-shaped about its vertex mean.
    Starness,
}

impl Metric {
    /// Every metric, in declaration order.
    pub const ALL: &'static [Metric] = &[
        Metric::Volume,
        Metric::Jacobian,
        Metric::ScaledJacobian,
        Metric::AspectRatio,
        Metric::AspectFrobenius,
        Metric::MaxAspectFrobenius,
        Metric::MeanAspectFrobenius,
        Metric::AspectGamma,
        Metric::RadiusRatio,
        Metric::CollapseRatio,
        Metric::EdgeRatio,
        Metric::MaxEdgeRatio,
        Metric::Skew,
        Metric::Taper,
        Metric::Warpage,
        Metric::Stretch,
        Metric::Shear,
        Metric::Oddy,
        Metric::Diagonal,
        Metric::Dimension,
        Metric::Condition,
        Metric::Distortion,
        Metric::Shape,
        Metric::RelativeSizeSquared,
        Metric::ShapeAndSize,
        Metric::ShearAndSize,
        Metric::MinAngle,
        Metric::MaxAngle,
        Metric::Sphericity,
        Metric::Starness,
    ];

    /// True if the metric is defined for `shape`.
    pub fn applies(self, shape: CellShape) -> bool {
        use CellShape as S;
        use Metric as M;
        let common = matches!(self, M::Volume | M::Jacobian | M::ScaledJacobian | M::EdgeRatio | M::MinAngle | M::MaxAngle);
        let sized = matches!(self, M::RelativeSizeSquared | M::ShapeAndSize | M::Shape | M::Condition);
        match shape {
            S::Tri => {
                common
                    || sized
                    || matches!(self, M::AspectRatio | M::AspectFrobenius | M::Distortion | M::RadiusRatio)
            }
            S::Quad => {
                common
                    || sized
                    || matches!(
                        self,
                        M::AspectRatio
                            | M::MaxAspectFrobenius
                            | M::MeanAspectFrobenius
                            | M::Distortion
                            | M::MaxEdgeRatio
                            | M::Skew
                            | M::Taper
                            | M::Warpage
                            | M::Stretch
                            | M::Shear
                            | M::ShearAndSize
                            | M::Oddy
                    )
            }
            S::Polygon => common || matches!(self, M::AspectRatio),
            S::Tet => {
                common
                    || sized
                    || matches!(
                        self,
                        M::AspectRatio
                            | M::AspectFrobenius
                            | M::AspectGamma
                            | M::RadiusRatio
                            | M::CollapseRatio
                            | M::Distortion
                            | M::Sphericity
                            | M::Starness
                    )
            }
            S::Hex => {
                common
                    || sized
                    || matches!(
                        self,
                        M::MaxAspectFrobenius
                            | M::MeanAspectFrobenius
                            | M::Distortion
                            | M::MaxEdgeRatio
                            | M::Skew
                            | M::Taper
                            | M::Stretch
                            | M::Shear
                            | M::ShearAndSize
                            | M::Oddy
                            | M::Diagonal
                            | M::Dimension
                            | M::Sphericity
                            | M::Starness
                    )
            }
            S::Wedge => {
                common
                    || sized
                    || matches!(
                        self,
                        M::MaxAspectFrobenius | M::MeanAspectFrobenius | M::Distortion | M::Sphericity | M::Starness
                    )
            }
            S::Pyramid => {
                common
                    || sized
                    || matches!(self, M::MaxAspectFrobenius | M::MeanAspectFrobenius | M::Sphericity | M::Starness)
            }
            S::Polyhedron => matches!(
                self,
                M::Volume | M::EdgeRatio | M::MinAngle | M::MaxAngle | M::Sphericity | M::Starness
            ),
            _ => false,
        }
    }

    /// Value of the metric on the ideal element of `shape` (unit square/cube, equilateral
    /// triangle, regular tetrahedron, regular wedge with unit edges, regular square pyramid),
    /// or `None` if the metric is not defined for the shape or has no scale-free ideal.
    pub fn ideal(self, shape: CellShape) -> Option<f64> {
        use CellShape as S;
        use Metric as M;
        if !self.applies(shape) {
            return None;
        }
        if shape == S::Polygon {
            return matches!(self, M::AspectRatio | M::EdgeRatio).then_some(1.0);
        }
        let v = match self {
            M::Volume | M::Jacobian | M::Dimension => return None,
            M::ScaledJacobian
            | M::AspectRatio
            | M::AspectFrobenius
            | M::MaxAspectFrobenius
            | M::MeanAspectFrobenius
            | M::AspectGamma
            | M::RadiusRatio
            | M::EdgeRatio
            | M::MaxEdgeRatio
            | M::Warpage
            | M::Stretch
            | M::Shear
            | M::Diagonal
            | M::Condition
            | M::Distortion
            | M::Shape
            | M::RelativeSizeSquared
            | M::ShapeAndSize
            | M::ShearAndSize => 1.0,
            M::CollapseRatio => (2.0f64 / 3.0).sqrt(),
            M::Skew | M::Taper | M::Oddy => 0.0,
            M::MinAngle | M::MaxAngle => match (shape, self) {
                (S::Tri, _) => 60.0,
                (S::Quad, _) => 90.0,
                (S::Tet, _) => (1.0f64 / 3.0).acos().to_degrees(),
                (S::Wedge, M::MinAngle) => 60.0,
                (S::Hex | S::Wedge, _) => 90.0,
                _ => return None,
            },
            M::Sphericity | M::Starness => return None,
        };
        Some(v)
    }
}

/// Per-evaluation context shared by the kernels.
#[derive(Clone, Copy, Debug)]
pub struct Context {
    /// Reference normal of a 2-D cell (`None`: the cell's own area normal, so a planar element is
    /// never inverted). Need not be normalised.
    pub normal: Option<V>,
    /// Reference size (area/volume) for the relative-size metrics; `NaN` makes them `NaN`.
    pub ref_size: f64,
}

impl Default for Context {
    fn default() -> Self {
        Self { normal: None, ref_size: f64::NAN }
    }
}

/// Evaluate `metric` on one element whose corner coordinates `p` are in the template order of
/// `shape` (the cell's vertex cycle for 2-D cells). Returns `None` if the metric is not defined
/// for the shape (see [`Metric::applies`]) or `p` has the wrong length.
///
/// Degenerate elements follow Verdict's conventions: metrics where large is bad return at most
/// [`crate::DBL_MAX`], metrics where small is bad return 0 (or a negative value for an inverted
/// element, where the metric admits one). Non-finite coordinates give `NaN`.
pub fn eval(metric: Metric, shape: CellShape, p: &[V], ctx: &Context) -> Option<f64> {
    if !metric.applies(shape) {
        return None;
    }
    if let Some(n) = shape.vertex_count()
        && p.len() != n
    {
        return None;
    }
    if p.len() < 3 {
        return None;
    }
    if p.iter().any(|q| q.iter().any(|x| !x.is_finite())) {
        return Some(f64::NAN);
    }
    match shape {
        CellShape::Tri | CellShape::Quad | CellShape::Polygon => planar::eval(metric, shape, p, ctx),
        _ => solid::eval(metric, shape, p, ctx),
    }
}

/// `min(t, 1/t)^2` with `t = v / v_ref`; 0 for a non-positive size.
pub(crate) fn relative_size_squared(v: f64, v_ref: f64) -> f64 {
    if v_ref.is_nan() {
        return f64::NAN;
    }
    if v <= crate::vec::TINY || v_ref <= crate::vec::TINY {
        return 0.0;
    }
    let t = v / v_ref;
    let m = t.min(1.0 / t);
    m * m
}
