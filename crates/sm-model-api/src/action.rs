//! The shared group-action machinery.
//!
//! Symmetry (deck group acting on a fundamental domain) and instancing (lattices placing
//! prototypes) both produce a list of [`Placement`]s: a transform, whether it reverses
//! orientation, and the path that led to it. Everything else (unfold, flatten) is a
//! deterministic enumeration of placements.

use crate::error::ModelError;
use sm_core::{EntityId, Point3, Transform, Vector3};
use smallvec::SmallVec;
use std::collections::BTreeMap;

/// Compose two transforms: `compose(a, b)(p) = a(b(p))`.
pub fn compose(a: &Transform, b: &Transform) -> Transform {
    let mut m = [[0.0; 3]; 3];
    for (i, row) in m.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
        }
    }
    Transform { m, t: a.apply_vector(b.t) + a.t }
}

/// Inverse of an affine transform. Fails on a singular linear part.
pub fn inverse(a: &Transform) -> Result<Transform, ModelError> {
    let d = a.det();
    if d.abs() < 1e-14 {
        return Err(ModelError::SingularTransform);
    }
    let m = &a.m;
    let c = |r0: usize, c0: usize, r1: usize, c1: usize| m[r0][c0] * m[r1][c1] - m[r0][c1] * m[r1][c0];
    // Adjugate (transposed cofactor matrix) divided by det.
    let inv = [
        [c(1, 1, 2, 2) / d, -c(0, 1, 2, 2) / d, c(0, 1, 1, 2) / d],
        [-c(1, 0, 2, 2) / d, c(0, 0, 2, 2) / d, -c(0, 0, 1, 2) / d],
        [c(1, 0, 2, 1) / d, -c(0, 0, 2, 1) / d, c(0, 0, 1, 1) / d],
    ];
    let lin = Transform { m: inv, t: Vector3::new(0.0, 0.0, 0.0) };
    let t = -lin.apply_vector(a.t);
    Ok(Transform { m: inv, t })
}

/// Pure translation.
pub fn translation(v: Vector3) -> Transform {
    Transform { t: v, ..Transform::IDENTITY }
}

/// Reflection in the plane through `p` with unit normal `n` (improper, det = -1).
pub fn reflection(p: Point3, n: Vector3) -> Transform {
    let mut m = [[0.0; 3]; 3];
    let nv = [n.x, n.y, n.z];
    for (i, row) in m.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = f64::from(u8::from(i == j)) - 2.0 * nv[i] * nv[j];
        }
    }
    let lin = Transform { m, t: Vector3::new(0.0, 0.0, 0.0) };
    let pv = Vector3::new(p.x, p.y, p.z);
    Transform { m, t: pv - lin.apply_vector(pv) }
}

/// Rotation by `angle` (radians, right-handed) about the line through `p` with unit direction `k`.
pub fn rotation(p: Point3, k: Vector3, angle: f64) -> Transform {
    let (s, c) = angle.sin_cos();
    let kv = [k.x, k.y, k.z];
    let cross = [[0.0, -k.z, k.y], [k.z, 0.0, -k.x], [-k.y, k.x, 0.0]];
    let mut m = [[0.0; 3]; 3];
    for (i, row) in m.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = c * f64::from(u8::from(i == j)) + s * cross[i][j] + (1.0 - c) * kv[i] * kv[j];
        }
    }
    let lin = Transform { m, t: Vector3::new(0.0, 0.0, 0.0) };
    let pv = Vector3::new(p.x, p.y, p.z);
    Transform { m, t: pv - lin.apply_vector(pv) }
}

/// Normalize a vector; `None` if it is (numerically) zero.
pub fn unit(v: Vector3) -> Option<Vector3> {
    let n = v.norm();
    (n > 1e-14).then(|| v * (1.0 / n))
}

/// Largest absolute entry-wise difference between two transforms.
pub fn max_diff(a: &Transform, b: &Transform) -> f64 {
    let mut d = (a.t.x - b.t.x).abs().max((a.t.y - b.t.y).abs()).max((a.t.z - b.t.z).abs());
    for i in 0..3 {
        for j in 0..3 {
            d = d.max((a.m[i][j] - b.m[i][j]).abs());
        }
    }
    d
}

/// Quantized transform, usable as a map key for deduplicating group elements.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TransformKey([i64; 12]);

impl TransformKey {
    /// Quantize to a grid of size `q` (use ~1e-7).
    pub fn new(t: &Transform, q: f64) -> Self {
        let r = |x: f64| ((x + 0.0) / q).round() as i64;
        let mut k = [0i64; 12];
        for i in 0..3 {
            for j in 0..3 {
                k[i * 3 + j] = r(t.m[i][j]);
            }
        }
        k[9] = r(t.t.x);
        k[10] = r(t.t.y);
        k[11] = r(t.t.z);
        Self(k)
    }
}

/// An oriented plane. `normal` is a unit vector; the positive side is where it points.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Plane {
    /// A point on the plane.
    pub point: Point3,
    /// Unit normal.
    pub normal: Vector3,
}

impl Plane {
    /// Build a plane; the normal is normalized. Fails on a zero normal.
    pub fn new(point: Point3, normal: Vector3) -> Result<Self, ModelError> {
        let normal = unit(normal).ok_or_else(|| ModelError::BadGenerator("zero plane normal".into()))?;
        Ok(Self { point, normal })
    }
    /// Signed distance of `p` (positive on the normal side).
    pub fn signed_distance(&self, p: Point3) -> f64 {
        (p - self.point).dot(self.normal)
    }
    /// The same plane facing the other way.
    pub fn flipped(&self) -> Self {
        Self { point: self.point, normal: -self.normal }
    }
}

/// Kind of a deck transformation (ir.md §5.1).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum DeckKind {
    /// Reflection; self-paired, improper.
    Mirror,
    /// Rigid translation; faces A and B = T(A).
    Translation,
    /// Rotation by 2 pi / n; faces A and B = R(A).
    Rotation,
}

/// A generator of the deck group.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Generator {
    /// Reflection in a plane.
    Mirror {
        /// The mirror plane (its normal side is the side that is *kept* when cutting).
        plane: Plane,
    },
    /// Translation. `origin` anchors the slab `[origin, origin + vector]` used when cutting.
    Translation {
        /// Anchor of the first slab face.
        origin: Point3,
        /// Translation vector (A to B).
        vector: Vector3,
    },
    /// Rotation by `2 pi / order` about the axis line.
    Rotation {
        /// A point on the axis.
        origin: Point3,
        /// Unit axis direction.
        axis: Vector3,
        /// Order `n >= 2`.
        order: u32,
    },
}

impl Generator {
    /// Reject degenerate generators.
    pub fn validate(&self) -> Result<(), ModelError> {
        match self {
            Generator::Mirror { plane } if (plane.normal.norm() - 1.0).abs() > 1e-9 => {
                Err(ModelError::BadGenerator("mirror normal is not unit".into()))
            }
            Generator::Translation { vector, .. } if vector.norm() < 1e-14 => {
                Err(ModelError::BadGenerator("zero translation".into()))
            }
            Generator::Rotation { axis, order, .. } if (axis.norm() - 1.0).abs() > 1e-9 || *order < 2 => {
                Err(ModelError::BadGenerator("rotation needs a unit axis and order >= 2".into()))
            }
            _ => Ok(()),
        }
    }

    /// The affine map this generator applies to the fundamental domain.
    pub fn transform(&self) -> Transform {
        match *self {
            Generator::Mirror { plane } => reflection(plane.point, plane.normal),
            Generator::Translation { vector, .. } => translation(vector),
            Generator::Rotation { origin, axis, order } => {
                rotation(origin, axis, std::f64::consts::TAU / f64::from(order))
            }
        }
    }

    /// Deck kind of this generator.
    pub fn kind(&self) -> DeckKind {
        match self {
            Generator::Mirror { .. } => DeckKind::Mirror,
            Generator::Translation { .. } => DeckKind::Translation,
            Generator::Rotation { .. } => DeckKind::Rotation,
        }
    }
}

/// A discrete 3-index slot of a lattice, ordered lexicographically `(a, b, c)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Slot {
    /// First index (cartesian `i`, hex `q`).
    pub a: i32,
    /// Second index (cartesian `j`, hex `r`).
    pub b: i32,
    /// Third index (layer `k`).
    pub c: i32,
}

impl Slot {
    /// Construct a slot.
    pub const fn new(a: i32, b: i32, c: i32) -> Self {
        Self { a, b, c }
    }
}

/// One step of the path that led to a placement.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PathStep {
    /// Element `index` of the symmetry group.
    GroupElement(u32),
    /// A top-level instance.
    Instance(EntityId),
    /// A slot of a lattice.
    Slot {
        /// The lattice.
        lattice: EntityId,
        /// The slot in it.
        slot: Slot,
    },
}

/// Path of a placement (inline up to four levels of nesting).
pub type InstancePath = SmallVec<[PathStep; 4]>;

/// One application of the group action: where a copy of the base object sits.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Placement {
    /// Maps base coordinates to placed coordinates.
    pub transform: Transform,
    /// True if the transform is improper: face loops must be reversed and left/right re-applied.
    pub orientation_reversed: bool,
    /// How we got here.
    pub path: InstancePath,
}

impl Placement {
    /// The identity placement with an empty path.
    pub fn root() -> Self {
        Self { transform: Transform::IDENTITY, orientation_reversed: false, path: InstancePath::new() }
    }
    /// Place a child with local transform `local` (applied first), recording `step`.
    pub fn then(&self, local: &Transform, step: PathStep) -> Self {
        let transform = compose(&self.transform, local);
        let mut path = self.path.clone();
        path.push(step);
        Self { orientation_reversed: transform.is_improper(), transform, path }
    }
}

/// Configuration for enumerating group elements.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EnumerateOptions {
    /// Cap on the number of elements.
    pub max_elements: usize,
}

impl Default for EnumerateOptions {
    fn default() -> Self {
        Self { max_elements: 4096 }
    }
}

/// Enumerate the finite set of group elements generated by `gens`.
///
/// Mirrors and rotations generate a point group by right-multiplication closure (breadth
/// first, generators in the given order). Each translation `(generator, repeat)` then
/// contributes the powers `0..repeat` (placed after the point operation, so the set stays
/// closed under right-multiplication by that translation, which unfold gluing relies on).
/// Element 0 is always the identity. Output order is deterministic.
pub fn enumerate_elements(
    point_gens: &[Transform],
    translations: &[(Transform, u32)],
    opts: &EnumerateOptions,
) -> Result<Vec<Placement>, ModelError> {
    const Q: f64 = 1e-7;
    let mut elems: Vec<Transform> = vec![Transform::IDENTITY];
    let mut seen: BTreeMap<TransformKey, usize> = BTreeMap::new();
    seen.insert(TransformKey::new(&Transform::IDENTITY, Q), 0);
    let mut i = 0;
    while i < elems.len() {
        for g in point_gens {
            let cand = compose(&elems[i], g);
            let key = TransformKey::new(&cand, Q);
            if let std::collections::btree_map::Entry::Vacant(e) = seen.entry(key) {
                if elems.len() >= opts.max_elements {
                    return Err(ModelError::GroupTooLarge { limit: opts.max_elements });
                }
                e.insert(elems.len());
                elems.push(cand);
            }
        }
        i += 1;
    }
    for (t, repeat) in translations {
        let mut next = Vec::with_capacity(elems.len() * *repeat as usize);
        let mut power = Transform::IDENTITY;
        for _ in 0..*repeat {
            for e in &elems {
                next.push(compose(e, &power));
            }
            power = compose(&power, t);
        }
        if next.len() > opts.max_elements {
            return Err(ModelError::GroupTooLarge { limit: opts.max_elements });
        }
        elems = next;
    }
    Ok(elems
        .into_iter()
        .enumerate()
        .map(|(i, transform)| Placement {
            orientation_reversed: transform.is_improper(),
            transform,
            path: smallvec::smallvec![PathStep::GroupElement(i as u32)],
        })
        .collect())
}
