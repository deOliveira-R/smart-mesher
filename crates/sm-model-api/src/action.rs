//! Group-action machinery shared by symmetry (unfold) and instancing (flatten).
//!
//! Both are "a fundamental piece placed many times": a list of [`Placed`] copies, each a
//! transform plus the [`Path`] that names it. Symmetry builds the list from deck generators,
//! instancing from lattice slots and instance transforms. Improper copies (mirrors) record
//! `improper = true` so consumers flip face loops and sides ([`crate::topology::Sense::under`]).

use crate::error::{ModelError, Result};
use crate::topology::Tolerance;
use sm_core::{EntityId, Point3, Transform, Vector3};
use smallvec::SmallVec;

/// An oriented plane (unit normal).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Plane {
    /// A point on the plane.
    pub point: Point3,
    /// Unit normal.
    pub normal: Vector3,
}

impl Plane {
    /// Build a plane; the normal is normalized. `None` if the normal is (near) zero.
    pub fn new(point: Point3, normal: Vector3) -> Option<Self> {
        let n = normal.norm();
        (n > 1e-300).then(|| Self { point, normal: normal * (1.0 / n) })
    }

    /// Signed distance of `p` from the plane (positive on the normal side).
    pub fn signed_distance(&self, p: Point3) -> f64 {
        (p - self.point).dot(self.normal)
    }
}

/// A directed axis (unit direction) for rotations.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Axis {
    /// A point on the axis.
    pub point: Point3,
    /// Unit direction.
    pub dir: Vector3,
}

impl Axis {
    /// Build an axis; the direction is normalized. `None` if the direction is (near) zero.
    pub fn new(point: Point3, dir: Vector3) -> Option<Self> {
        let n = dir.norm();
        (n > 1e-300).then(|| Self { point, dir: dir * (1.0 / n) })
    }

    /// Distance of `p` from the axis line.
    pub fn distance(&self, p: Point3) -> f64 {
        (p - self.point).cross(self.dir).norm()
    }
}

/// Pure translation by `v`.
pub fn translation(v: Vector3) -> Transform {
    Transform { t: v, ..Transform::IDENTITY }
}

/// Rotation by `angle` radians (right-hand rule) about `axis`.
pub fn rotation(axis: &Axis, angle: f64) -> Transform {
    let (s, c) = angle.sin_cos();
    let k = axis.dir;
    let kk = [k.x, k.y, k.z];
    let mut m = [[0.0; 3]; 3];
    // Rodrigues: c I + s [k]x + (1-c) k k^T
    let cross = [[0.0, -k.z, k.y], [k.z, 0.0, -k.x], [-k.y, k.x, 0.0]];
    for (i, row) in m.iter_mut().enumerate() {
        for (j, e) in row.iter_mut().enumerate() {
            *e = s * cross[i][j] + (1.0 - c) * kk[i] * kk[j] + if i == j { c } else { 0.0 };
        }
    }
    let rc = Transform { m, t: Vector3::default() }.apply_point(axis.point);
    Transform { m, t: axis.point - rc }
}

/// Reflection in `plane` (improper, `det = -1`).
pub fn reflection(plane: &Plane) -> Transform {
    let n = plane.normal;
    let nn = [n.x, n.y, n.z];
    let mut m = [[0.0; 3]; 3];
    for (i, row) in m.iter_mut().enumerate() {
        for (j, e) in row.iter_mut().enumerate() {
            *e = if i == j { 1.0 } else { 0.0 } - 2.0 * nn[i] * nn[j];
        }
    }
    let d = 2.0 * Vector3::new(plane.point.x, plane.point.y, plane.point.z).dot(n);
    Transform { m, t: n * d }
}

/// Composition `a ∘ b`: first `b`, then `a`.
pub fn compose(a: &Transform, b: &Transform) -> Transform {
    let mut m = [[0.0; 3]; 3];
    for (i, row) in m.iter_mut().enumerate() {
        for (j, e) in row.iter_mut().enumerate() {
            *e = (0..3).map(|k| a.m[i][k] * b.m[k][j]).sum();
        }
    }
    Transform { m, t: a.apply_vector(b.t) + a.t }
}

/// Inverse transform; [`ModelError::Singular`] if the linear part is not invertible.
pub fn inverse(t: &Transform) -> Result<Transform> {
    let m = &t.m;
    let det = t.det();
    if det.abs() < 1e-14 {
        return Err(ModelError::Singular);
    }
    let c = |r0: usize, c0: usize, r1: usize, c1: usize| m[r0][c0] * m[r1][c1] - m[r0][c1] * m[r1][c0];
    let inv = [
        [c(1, 1, 2, 2), -c(0, 1, 2, 2), c(0, 1, 1, 2)],
        [-c(1, 0, 2, 2), c(0, 0, 2, 2), -c(0, 0, 1, 2)],
        [c(1, 0, 2, 1), -c(0, 0, 2, 1), c(0, 0, 1, 1)],
    ];
    let mut mi = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            mi[i][j] = inv[i][j] / det;
        }
    }
    let ti = Transform { m: mi, t: Vector3::default() }.apply_vector(t.t);
    Ok(Transform { m: mi, t: -ti })
}

/// True if every entry of `a` and `b` agrees within `tol` (linear part: angular, translation: linear).
pub fn approx_eq(a: &Transform, b: &Transform, tol: &Tolerance) -> bool {
    let lin = (0..3).all(|i| (0..3).all(|j| (a.m[i][j] - b.m[i][j]).abs() <= tol.angular));
    lin && (a.t - b.t).norm() <= tol.linear
}

/// Index of a lattice slot: cartesian `[i, j, k]`, or hex axial `[q, r, k]`.
pub type SlotIndex = [i32; 3];

/// One step of the path that names a placed copy.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PathStep {
    /// Group element number `n` of an unfold (0 is the identity, the fundamental domain itself).
    Element(u32),
    /// An `Instance` record.
    Instance(EntityId),
    /// A slot of a lattice.
    Slot {
        /// The lattice record.
        lattice: EntityId,
        /// Slot index within the lattice.
        index: SlotIndex,
    },
}

/// Path from the root to a copy. Nesting depth is small, so no heap allocation in practice.
pub type Path = SmallVec<[PathStep; 4]>;

/// A copy of the fundamental piece: where it is and how it is named.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Placed {
    /// Name of the copy.
    pub path: Path,
    /// Transform from the fundamental piece's frame to the model frame.
    pub transform: Transform,
    /// True if the transform reverses orientation; consumers must flip face loops and sides.
    pub improper: bool,
}

impl Placed {
    /// The root copy: identity, empty path.
    pub fn root() -> Self {
        Self { path: Path::new(), transform: Transform::IDENTITY, improper: false }
    }

    /// A child placed with `local` (in this copy's frame) and named by `step`.
    pub fn child(&self, step: PathStep, local: &Transform) -> Self {
        let transform = compose(&self.transform, local);
        let mut path = self.path.clone();
        path.push(step);
        Self { path, improper: transform.is_improper(), transform }
    }
}

/// A finite group represented as an explicit transform list (element 0 is the identity).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GroupAction {
    elements: Vec<Transform>,
}

impl GroupAction {
    /// The trivial group.
    pub fn trivial() -> Self {
        Self { elements: vec![Transform::IDENTITY] }
    }

    /// Closure of `generators` under composition, breadth-first (deterministic order).
    /// Fails with [`ModelError::GroupTooLarge`] if more than `max` elements appear (this is
    /// also how infinite groups, e.g. a bare translation, are rejected).
    pub fn closure(generators: &[Transform], max: usize, tol: &Tolerance) -> Result<Self> {
        let mut elements = vec![Transform::IDENTITY];
        let mut head = 0;
        while head < elements.len() {
            let e = elements[head];
            head += 1;
            for g in generators {
                let n = compose(&e, g);
                if !elements.iter().any(|x| approx_eq(x, &n, tol)) {
                    if elements.len() >= max {
                        return Err(ModelError::GroupTooLarge { limit: max });
                    }
                    elements.push(n);
                }
            }
        }
        Ok(Self { elements })
    }

    /// Left-multiply by powers of translations: elements become `T1^i T2^j ... · g`, in
    /// lexicographic order of `(i, j, ...)` then original order, so element 0 stays the
    /// identity. `(T, n)` means exponents `0..n`. Valid when the translations commute with
    /// the point group elements they are combined with (documented limitation).
    pub fn tiled(&self, translations: &[(Transform, u32)], max: usize) -> Result<Self> {
        let mut shifts = vec![Transform::IDENTITY];
        for (t, n) in translations {
            let mut next = Vec::with_capacity(shifts.len() * (*n as usize).max(1));
            for s in &shifts {
                let mut cur = *s;
                for _ in 0..(*n).max(1) {
                    next.push(cur);
                    cur = compose(t, &cur);
                }
            }
            shifts = next;
            if shifts.len() * self.elements.len() > max {
                return Err(ModelError::GroupTooLarge { limit: max });
            }
        }
        let elements = shifts.iter().flat_map(|s| self.elements.iter().map(move |g| compose(s, g))).collect();
        Ok(Self { elements })
    }

    /// The elements; index 0 is the identity.
    pub fn elements(&self) -> &[Transform] {
        &self.elements
    }

    /// Number of elements.
    pub fn len(&self) -> usize {
        self.elements.len()
    }

    /// Always false: a group has at least the identity.
    pub fn is_empty(&self) -> bool {
        false
    }

    /// Position of `t` in the list, within `tol`.
    pub fn index_of(&self, t: &Transform, tol: &Tolerance) -> Option<usize> {
        self.elements.iter().position(|x| approx_eq(x, t, tol))
    }

    /// The elements as named copies (`Element(n)` paths).
    pub fn placements(&self) -> Vec<Placed> {
        self.elements
            .iter()
            .enumerate()
            .map(|(i, t)| Placed { path: SmallVec::from_slice(&[PathStep::Element(i as u32)]), transform: *t, improper: t.is_improper() })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn z_axis() -> Axis {
        Axis::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap()
    }

    #[test]
    fn rotation_by_60_has_order_6() {
        let tol = Tolerance::default();
        let r = rotation(&z_axis(), core::f64::consts::PI / 3.0);
        let g = GroupAction::closure(&[r], 100, &tol).unwrap();
        assert_eq!(g.len(), 6);
        let p = r.apply_point(Point3::new(1.0, 0.0, 0.0));
        assert!((p.x - 0.5).abs() < 1e-12 && (p.y - 3f64.sqrt() / 2.0).abs() < 1e-12);
    }

    #[test]
    fn rotation_about_offset_axis_fixes_axis() {
        let ax = Axis::new(Point3::new(2.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let r = rotation(&ax, 1.0);
        let p = r.apply_point(Point3::new(2.0, 1.0, 5.0));
        assert!((p.x - 2.0).abs() < 1e-12 && (p.y - 1.0).abs() < 1e-12 && (p.z - 5.0).abs() < 1e-12);
    }

    #[test]
    fn reflection_is_improper_involution_fixing_plane() {
        let tol = Tolerance::default();
        let pl = Plane::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(2.0, 0.0, 0.0)).unwrap();
        let s = reflection(&pl);
        assert!(s.is_improper());
        assert!(approx_eq(&compose(&s, &s), &Transform::IDENTITY, &tol));
        let q = s.apply_point(Point3::new(1.0, 3.0, 4.0));
        assert!((q.x - 1.0).abs() < 1e-12);
        let q = s.apply_point(Point3::new(3.0, 0.0, 0.0));
        assert!((q.x + 1.0).abs() < 1e-12);
    }

    #[test]
    fn inverse_roundtrip() {
        let tol = Tolerance::default();
        let t = compose(&translation(Vector3::new(1.0, 2.0, 3.0)), &rotation(&z_axis(), 0.7));
        let i = inverse(&t).unwrap();
        assert!(approx_eq(&compose(&t, &i), &Transform::IDENTITY, &tol));
    }

    #[test]
    fn bare_translation_is_infinite() {
        let tol = Tolerance::default();
        let t = translation(Vector3::new(1.0, 0.0, 0.0));
        assert!(matches!(GroupAction::closure(&[t], 50, &tol), Err(ModelError::GroupTooLarge { .. })));
        let g = GroupAction::trivial().tiled(&[(t, 3)], 50).unwrap();
        assert_eq!(g.len(), 3);
    }
}
