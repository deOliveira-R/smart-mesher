//! Boolean test cases as data: the operand pair and the closed-form expectations of each
//! result. No boolean code lives here; the `sm-boolean` tests build the operands from [`Operand`]
//! and compare their results against [`Expected`]. All numbers are derived by hand in the case
//! docs and cross-checked by inclusion-exclusion in `tests/w3_geom.rs`.
//!
//! Counts `[V, E, F]` are the cell-complex counts of the result with degenerate pole edges
//! excluded (so a sphere contributes two vertices, one seam edge and one face); `euler` is
//! `V - E + F`.

use core::f64::consts::PI;

/// A primitive operand of a boolean case.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Operand {
    /// Axis-aligned brick `[min, min + size]`.
    Brick {
        /// Minimum corner.
        min: [f64; 3],
        /// Edge lengths.
        size: [f64; 3],
    },
    /// Solid cylinder: the disk of `radius` about the line through `base` along the unit `axis`,
    /// from `base` to `base + height * axis`.
    Cylinder {
        /// Centre of the base disk.
        base: [f64; 3],
        /// Unit axis.
        axis: [f64; 3],
        /// Radius.
        radius: f64,
        /// Height along the axis.
        height: f64,
    },
    /// The same solid cylinder as [`Operand::Cylinder`], but its lateral surface is given as a
    /// rational quadratic NURBS (nine-point circle, weights `1, sqrt(2)/2` alternating) instead of
    /// an analytic quadric. Geometrically identical, so volume and area equal the analytic ones.
    NurbsCylinder {
        /// Centre of the base disk.
        base: [f64; 3],
        /// Unit axis.
        axis: [f64; 3],
        /// Radius.
        radius: f64,
        /// Height along the axis.
        height: f64,
    },
    /// Solid sphere.
    Sphere {
        /// Centre.
        center: [f64; 3],
        /// Radius.
        radius: f64,
    },
}

impl Operand {
    /// Exact volume of the operand.
    pub fn volume(&self) -> f64 {
        match *self {
            Operand::Brick { size, .. } => size[0] * size[1] * size[2],
            Operand::Cylinder { radius, height, .. } | Operand::NurbsCylinder { radius, height, .. } => PI * radius * radius * height,
            Operand::Sphere { radius, .. } => 4.0 / 3.0 * PI * radius.powi(3),
        }
    }

    /// Exact boundary area of the operand.
    pub fn area(&self) -> f64 {
        match *self {
            Operand::Brick { size, .. } => 2.0 * (size[0] * size[1] + size[1] * size[2] + size[0] * size[2]),
            Operand::Cylinder { radius, height, .. } | Operand::NurbsCylinder { radius, height, .. } => 2.0 * PI * radius * (radius + height),
            Operand::Sphere { radius, .. } => 4.0 * PI * radius * radius,
        }
    }
}

/// The operation a result belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rule {
    /// `A union B`.
    Unite,
    /// `A intersect B`.
    Intersect,
    /// `A minus B`.
    Subtract,
    /// Imprint `A` and `B` on each other and merge the coincident faces (two volumes remain).
    ImprintMerge,
}

/// The analytic expectation of one operation.
#[derive(Clone, Debug, PartialEq)]
pub struct Expected {
    /// The operation.
    pub rule: Rule,
    /// Volumes of the resulting volumes, ascending.
    pub volumes: Vec<f64>,
    /// Sum of the areas of all resulting faces (a merged shared face counts once).
    pub total_area: f64,
    /// Cell counts `[V, E, F]` if known.
    pub counts: Option<[usize; 3]>,
    /// `V - E + F` if known.
    pub euler: Option<i64>,
}

/// A boolean case: two operands and the expectations of the operations on them.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanCase {
    /// Case name.
    pub name: &'static str,
    /// First operand (the tool body of `Subtract` is `b`).
    pub a: Operand,
    /// Second operand.
    pub b: Operand,
    /// Expectations (possibly empty if the result has no closed form).
    pub expected: Vec<Expected>,
}

impl BooleanCase {
    /// The expectation of `rule`, if the case has one.
    pub fn expect(&self, rule: Rule) -> Option<&Expected> {
        self.expected.iter().find(|e| e.rule == rule)
    }
}

fn exp(rule: Rule, volumes: &[f64], total_area: f64, counts: Option<[usize; 3]>, euler: Option<i64>) -> Expected {
    Expected { rule, volumes: volumes.to_vec(), total_area, counts, euler }
}

/// `A = [0,2]^3`, `B = [1,3] x [0,2]^2`. Union `[0,3] x [0,2]^2`: volume 12, area 32, 8/12/6.
/// Intersection `[1,2] x [0,2]^2`: volume 4, area 16. `A - B = [0,1] x [0,2]^2`: volume 4,
/// area 16. All bricks: Euler 2.
pub fn overlapping_bricks() -> BooleanCase {
    let brick = |min: [f64; 3]| Operand::Brick { min, size: [2.0; 3] };
    BooleanCase {
        name: "overlapping_bricks",
        a: brick([0.0; 3]),
        b: brick([1.0, 0.0, 0.0]),
        expected: vec![
            exp(Rule::Unite, &[12.0], 32.0, Some([8, 12, 6]), Some(2)),
            exp(Rule::Intersect, &[4.0], 16.0, Some([8, 12, 6]), Some(2)),
            exp(Rule::Subtract, &[4.0], 16.0, Some([8, 12, 6]), Some(2)),
        ],
    }
}

/// A `4^3` brick at the origin minus the cylinder of radius 1 about the line `x = y = 2`
/// through the brick (`z` from `-1` to `5`): a through hole. Volume `64 - 4 pi`, area
/// `96 + 2 pi 1 * 4 - 2 pi = 96 + 6 pi` (the hole's lateral face minus the two end disks),
/// counts 10/15/7 (8 + 2 vertices, 12 + 3 edges, 6 + 1 faces), `V - E + F = 2`.
pub fn brick_minus_cylinder() -> BooleanCase {
    BooleanCase {
        name: "brick_minus_cylinder",
        a: Operand::Brick { min: [0.0; 3], size: [4.0; 3] },
        b: Operand::Cylinder { base: [2.0, 2.0, -1.0], axis: [0.0, 0.0, 1.0], radius: 1.0, height: 6.0 },
        expected: vec![exp(Rule::Subtract, &[64.0 - 4.0 * PI], 96.0 + 6.0 * PI, Some([10, 15, 7]), Some(2))],
    }
}

/// A `10^3` brick minus the sphere of radius 2 at its centre `(5, 5, 5)`: a closed cavity.
/// Volume `1000 - 32 pi / 3`, area `600 + 16 pi`, an `Outer` shell and a `Void` shell, counts
/// 10/13/7 (the sphere adds two pole vertices, one seam edge and one face), `V - E + F = 4`.
pub fn brick_with_cavity() -> BooleanCase {
    BooleanCase {
        name: "brick_with_cavity",
        a: Operand::Brick { min: [0.0; 3], size: [10.0; 3] },
        b: Operand::Sphere { center: [5.0; 3], radius: 2.0 },
        expected: vec![exp(Rule::Subtract, &[1000.0 - 32.0 * PI / 3.0], 600.0 + 16.0 * PI, Some([10, 13, 7]), Some(4))],
    }
}

/// `A = [0,2]^3`, `B = [2,4] x [1,3] x [0,2]`: they touch along the plane `x = 2`, overlapping
/// in `y in [1,2], z in [0,2]` (area 2). Imprint then merge keeps two volumes of volume 8,
/// splits each `x = 2` face in two and shares the overlap: 13 faces (6 + 6 + 2 - 1), 16
/// vertices, 26 edges, `V - E + F = 3`; total face area `24 + 24 - 2 = 46`.
pub fn partial_overlap_bricks() -> BooleanCase {
    BooleanCase {
        name: "partial_overlap_bricks",
        a: Operand::Brick { min: [0.0; 3], size: [2.0; 3] },
        b: Operand::Brick { min: [2.0, 1.0, 0.0], size: [2.0; 3] },
        expected: vec![exp(Rule::ImprintMerge, &[8.0, 8.0], 46.0, Some([16, 26, 13]), Some(3))],
    }
}

/// The brick `[0,2]^3` and the sphere of radius 1 centred at `(2, 1, 1)`, the middle of the
/// brick's `x = 2` face: exactly a hemisphere is inside. This is a deliberate tangency stress
/// case: the sphere is tangent to the four edges of the `x = 2` face and touches the faces
/// `y = 0`, `y = 2`, `z = 0`, `z = 2` at single points. Union: volume `8 + 2 pi / 3`, area
/// `24 - pi + 2 pi = 24 + pi`. Intersection (the hemisphere): volume `2 pi / 3`, area
/// `pi + 2 pi = 3 pi`. `A - B`: volume `8 - 2 pi / 3`, area `24 + pi`.
pub fn brick_sphere_half() -> BooleanCase {
    BooleanCase {
        name: "brick_sphere_half",
        a: Operand::Brick { min: [0.0; 3], size: [2.0; 3] },
        b: Operand::Sphere { center: [2.0, 1.0, 1.0], radius: 1.0 },
        expected: vec![
            exp(Rule::Unite, &[8.0 + 2.0 * PI / 3.0], 24.0 + PI, None, None),
            exp(Rule::Intersect, &[2.0 * PI / 3.0], 3.0 * PI, None, None),
            exp(Rule::Subtract, &[8.0 - 2.0 * PI / 3.0], 24.0 + PI, None, None),
        ],
    }
}

/// [`brick_minus_cylinder`] with the tool given as a [`Operand::NurbsCylinder`]: the rational
/// NURBS form of the same cylinder must give the same result (volume `64 - 4 pi`, area
/// `96 + 6 pi`, counts 10/15/7, Euler 2).
pub fn nurbs_cylinder_equiv() -> BooleanCase {
    let mut c = brick_minus_cylinder();
    c.name = "nurbs_cylinder_equiv";
    c.b = Operand::NurbsCylinder { base: [2.0, 2.0, -1.0], axis: [0.0, 0.0, 1.0], radius: 1.0, height: 6.0 };
    c
}

/// Two perpendicular cylinders for surface-surface intersection tests: `A` of radius 1 about the
/// `z` axis (`z in [-2, 2]`) and `B` of radius 0.5 about the `x` axis (`x in [-2, 2]`). The
/// volumes involve elliptic integrals, so no [`Expected`] is given; the intersection curve
/// is known in closed form, see [`crossing_cylinders_curve_point`].
pub fn crossing_cylinders() -> BooleanCase {
    BooleanCase {
        name: "crossing_cylinders",
        a: Operand::Cylinder { base: [0.0, 0.0, -2.0], axis: [0.0, 0.0, 1.0], radius: 1.0, height: 4.0 },
        b: Operand::Cylinder { base: [-2.0, 0.0, 0.0], axis: [1.0, 0.0, 0.0], radius: 0.5, height: 4.0 },
        expected: vec![],
    }
}

/// A point of the intersection curve of the lateral surfaces of [`crossing_cylinders`]: the point
/// of `B` at angle `phi` (`y = 0.5 cos phi`, `z = 0.5 sin phi`) on the branch `x = sign *
/// sqrt(1 - y^2)` (`sign` is `+1.0` or `-1.0`). It lies on both surfaces: `x^2 + y^2 = 1` and
/// `y^2 + z^2 = 0.25`.
pub fn crossing_cylinders_curve_point(phi: f64, sign: f64) -> [f64; 3] {
    let (y, z) = (0.5 * phi.cos(), 0.5 * phi.sin());
    [sign * (1.0 - y * y).sqrt(), y, z]
}

/// All cases, in a fixed order.
pub fn cases() -> Vec<BooleanCase> {
    vec![overlapping_bricks(), brick_minus_cylinder(), brick_with_cavity(), partial_overlap_bricks(), brick_sphere_half(), crossing_cylinders(), nurbs_cylinder_equiv()]
}
