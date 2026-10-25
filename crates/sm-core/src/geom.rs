//! Basic geometric value types. Deliberately small; richer math lives in `sm-math`.

use core::ops::{Add, Mul, Neg, Sub};

/// A 3D vector.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Vector3 {
    /// x component.
    pub x: f64,
    /// y component.
    pub y: f64,
    /// z component.
    pub z: f64,
}

/// A 3D point.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Point3 {
    /// x coordinate.
    pub x: f64,
    /// y coordinate.
    pub y: f64,
    /// z coordinate.
    pub z: f64,
}

impl Vector3 {
    /// Construct a vector.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
    /// Dot product.
    pub fn dot(self, o: Self) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    /// Cross product.
    pub fn cross(self, o: Self) -> Self {
        Self::new(self.y * o.z - self.z * o.y, self.z * o.x - self.x * o.z, self.x * o.y - self.y * o.x)
    }
    /// Euclidean norm.
    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }
}

impl Point3 {
    /// Construct a point.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

impl Add for Vector3 {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl Sub for Vector3 {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl Neg for Vector3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}
impl Mul<f64> for Vector3 {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}
impl Sub for Point3 {
    type Output = Vector3;
    fn sub(self, o: Self) -> Vector3 {
        Vector3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl Add<Vector3> for Point3 {
    type Output = Self;
    fn add(self, v: Vector3) -> Self {
        Self::new(self.x + v.x, self.y + v.y, self.z + v.z)
    }
}

/// An affine transform `p -> M p + t`. May be improper (`det(M) < 0`, e.g. a mirror),
/// which reverses orientation. See docs/design/ir.md §5.1.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Transform {
    /// Row-major 3x3 linear part.
    pub m: [[f64; 3]; 3],
    /// Translation.
    pub t: Vector3,
}

/// Largest component magnitude of `v` when it is finite and outside `[2^-500, 2^500]`, else `None`.
///
/// Inside that range `dot`/`sqrt` neither overflow nor underflow, so callers run their plain formula
/// unchanged (bit-identical to the unscaled code); outside it they divide by this value first.
fn out_of_range_scale(v: Vector3) -> Option<f64> {
    let m = v.x.abs().max(v.y.abs()).max(v.z.abs());
    let (lo, hi) = (2f64.powi(-500), 2f64.powi(500));
    (m.is_finite() && m > 0.0 && (m < lo || m > hi)).then_some(m)
}

impl Transform {
    /// The identity.
    pub const IDENTITY: Self = Self { m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t: Vector3::new(0.0, 0.0, 0.0) };

    /// Apply to a point.
    pub fn apply_point(&self, p: Point3) -> Point3 {
        let m = &self.m;
        Point3::new(
            m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + self.t.x,
            m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + self.t.y,
            m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + self.t.z,
        )
    }

    /// Apply the linear part to a vector.
    pub fn apply_vector(&self, v: Vector3) -> Vector3 {
        let m = &self.m;
        Vector3::new(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
        )
    }

    /// Determinant of the linear part.
    pub fn det(&self) -> f64 {
        let m = &self.m;
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }

    /// True if the transform reverses orientation (mirror-like).
    pub fn is_improper(&self) -> bool {
        self.det() < 0.0
    }

    /// Pure translation by `v`.
    pub const fn translation(v: Vector3) -> Self {
        Self { m: Self::IDENTITY.m, t: v }
    }

    /// Reflection in the plane `{x : n·x = d}`. `n` need not be unit length.
    /// Returns `None` if `n` is (near) zero.
    pub fn reflection(n: Vector3, d: f64) -> Option<Self> {
        let (n, d) = match out_of_range_scale(n) {
            Some(m) => (Vector3::new(n.x / m, n.y / m, n.z / m), d / m),
            None => (n, d),
        };
        let l2 = n.dot(n);
        if l2 <= f64::MIN_POSITIVE {
            return None;
        }
        let u = n * (1.0 / l2.sqrt());
        let dd = d / l2.sqrt();
        let a = [u.x, u.y, u.z];
        let mut m = [[0.0; 3]; 3];
        for (i, row) in m.iter_mut().enumerate() {
            for (j, e) in row.iter_mut().enumerate() {
                *e = if i == j { 1.0 } else { 0.0 } - 2.0 * a[i] * a[j];
            }
        }
        Some(Self { m, t: u * (2.0 * dd) })
    }

    /// Rotation by `angle` radians (right-hand rule) about the axis through `p` with direction `axis`.
    /// Returns `None` if `axis` is (near) zero.
    pub fn rotation(p: Point3, axis: Vector3, angle: f64) -> Option<Self> {
        let axis = match out_of_range_scale(axis) {
            Some(m) => Vector3::new(axis.x / m, axis.y / m, axis.z / m),
            None => axis,
        };
        let l = axis.norm();
        if l <= f64::MIN_POSITIVE {
            return None;
        }
        let k = axis * (1.0 / l);
        let (s, c) = angle.sin_cos();
        let v = 1.0 - c;
        let m = [
            [c + k.x * k.x * v, k.x * k.y * v - k.z * s, k.x * k.z * v + k.y * s],
            [k.y * k.x * v + k.z * s, c + k.y * k.y * v, k.y * k.z * v - k.x * s],
            [k.z * k.x * v - k.y * s, k.z * k.y * v + k.x * s, c + k.z * k.z * v],
        ];
        let lin = Self { m, t: Vector3::new(0.0, 0.0, 0.0) };
        let pv = Vector3::new(p.x, p.y, p.z);
        // x -> R(x - p) + p
        Some(Self { m, t: pv - lin.apply_vector(pv) })
    }

    /// Composition `self ∘ other`: apply `other` first, then `self`.
    pub fn compose(&self, other: &Self) -> Self {
        let mut m = [[0.0; 3]; 3];
        for (i, row) in m.iter_mut().enumerate() {
            for (j, e) in row.iter_mut().enumerate() {
                *e = (0..3).map(|k| self.m[i][k] * other.m[k][j]).sum();
            }
        }
        Self { m, t: self.apply_vector(other.t) + self.t }
    }

    /// Inverse transform. Returns `None` if the linear part is singular.
    pub fn inverse(&self) -> Option<Self> {
        let d = self.det();
        if d.abs() <= f64::MIN_POSITIVE {
            return None;
        }
        let m = &self.m;
        let inv_d = 1.0 / d;
        let c = |r0: usize, c0: usize, r1: usize, c1: usize| m[r0][c0] * m[r1][c1] - m[r0][c1] * m[r1][c0];
        let mi = [
            [c(1, 1, 2, 2) * inv_d, -c(0, 1, 2, 2) * inv_d, c(0, 1, 1, 2) * inv_d],
            [-c(1, 0, 2, 2) * inv_d, c(0, 0, 2, 2) * inv_d, -c(0, 0, 1, 2) * inv_d],
            [c(1, 0, 2, 1) * inv_d, -c(0, 0, 2, 1) * inv_d, c(0, 0, 1, 1) * inv_d],
        ];
        let lin = Self { m: mi, t: Vector3::new(0.0, 0.0, 0.0) };
        Some(Self { m: mi, t: -lin.apply_vector(self.t) })
    }
}

/// Axis-aligned bounding box. The empty box has `lo = +inf`, `hi = -inf`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Aabb {
    /// Minimum corner.
    pub lo: Point3,
    /// Maximum corner.
    pub hi: Point3,
}

impl Aabb {
    /// The empty box (contains nothing; identity for [`Aabb::union`]).
    pub const fn empty() -> Self {
        Self { lo: Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), hi: Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY) }
    }

    /// Degenerate box holding one point.
    pub const fn from_point(p: Point3) -> Self {
        Self { lo: p, hi: p }
    }

    /// True if the box contains no point (some `lo > hi`, or a NaN bound).
    pub fn is_empty(&self) -> bool {
        !(self.lo.x <= self.hi.x && self.lo.y <= self.hi.y && self.lo.z <= self.hi.z)
    }

    /// Grow the box to include `p`.
    pub fn extend(&mut self, p: Point3) {
        self.lo = Point3::new(self.lo.x.min(p.x), self.lo.y.min(p.y), self.lo.z.min(p.z));
        self.hi = Point3::new(self.hi.x.max(p.x), self.hi.y.max(p.y), self.hi.z.max(p.z));
    }

    /// Smallest box containing both.
    pub fn union(&self, o: &Self) -> Self {
        Self {
            lo: Point3::new(self.lo.x.min(o.lo.x), self.lo.y.min(o.lo.y), self.lo.z.min(o.lo.z)),
            hi: Point3::new(self.hi.x.max(o.hi.x), self.hi.y.max(o.hi.y), self.hi.z.max(o.hi.z)),
        }
    }

    /// True if `p` lies in the closed box.
    pub fn contains(&self, p: Point3) -> bool {
        self.lo.x <= p.x && p.x <= self.hi.x && self.lo.y <= p.y && p.y <= self.hi.y && self.lo.z <= p.z && p.z <= self.hi.z
    }
}

impl Default for Aabb {
    fn default() -> Self {
        Self::empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Point3, b: Point3) -> bool {
        (a - b).norm() < 1e-12
    }

    #[test]
    fn reflection_is_involutive_and_improper() {
        let r = Transform::reflection(Vector3::new(1.0, 0.0, 0.0), 2.0).unwrap();
        assert!(r.is_improper());
        assert!(close(r.apply_point(Point3::new(0.0, 1.0, 1.0)), Point3::new(4.0, 1.0, 1.0)));
        let rr = r.compose(&r);
        assert!(close(rr.apply_point(Point3::new(0.3, -2.0, 5.0)), Point3::new(0.3, -2.0, 5.0)));
    }

    #[test]
    fn rotation_about_offset_axis() {
        let r = Transform::rotation(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0), core::f64::consts::FRAC_PI_2).unwrap();
        assert!(!r.is_improper());
        assert!(close(r.apply_point(Point3::new(2.0, 0.0, 0.0)), Point3::new(1.0, 1.0, 0.0)));
        let six = (0..6).fold(Transform::IDENTITY, |acc, _| {
            Transform::rotation(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0), core::f64::consts::PI / 3.0).unwrap().compose(&acc)
        });
        assert!(close(six.apply_point(Point3::new(1.0, 2.0, 3.0)), Point3::new(1.0, 2.0, 3.0)));
    }

    const ROT_BITS_A: &[u64] = &[4605215599329968137, 13825743337350415434, 4600782619949761238, 4603130235055696891, 4605669480746133353, 13813180829258347041, 13822339131139876180, 4598588764182329229, 4606425949773075380, 13831289571320271488, 13822452362520667160, 4603470598323170740];
    const ROT_BITS_B: &[u64] = &[4599624750068189093, 13830048415072142402, 0, 4606676378217366594, 4599624750068189093, 0, 0, 0, 4607182418800017408, 0, 0, 0];
    const REF_BITS: &[u64] = &[4603751104798211316, 4605037847548888601, 13819402685148923417, 4605037847548888601, 13826265313152535602, 4600534247921518105, 13819402685148923417, 4600534247921518105, 4606324590299565885, 4607825790175356051, 13835701426657502355, 4603322190547985555];

    #[test]
    fn rotation_in_range_bits_unchanged() {
        // Bits computed with the unscaled formula at 53ffa58.
        let r = Transform::rotation(Point3::new(0.5, -1.0, 2.0), Vector3::new(1.0, 2.0, 3.0), 0.7).unwrap();
        let bits: Vec<u64> = r.m.iter().flatten().chain([r.t.x, r.t.y, r.t.z].iter()).map(|x| x.to_bits()).collect();
        assert_eq!(bits, ROT_BITS_A);
        let r = Transform::rotation(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0), 1.234).unwrap();
        let bits: Vec<u64> = r.m.iter().flatten().chain([r.t.x, r.t.y, r.t.z].iter()).map(|x| x.to_bits()).collect();
        assert_eq!(bits, ROT_BITS_B);
        let r = Transform::reflection(Vector3::new(1.0, -2.0, 0.5), 3.0).unwrap();
        let bits: Vec<u64> = r.m.iter().flatten().chain([r.t.x, r.t.y, r.t.z].iter()).map(|x| x.to_bits()).collect();
        assert_eq!(bits, REF_BITS);
    }

    #[test]
    fn huge_and_tiny_axes() {
        let p = Point3::new(1.0, 2.0, 3.0);
        let base = Transform::rotation(p, Vector3::new(1.0, 2.0, 3.0), 0.7).unwrap();
        for s in [1e200, 1e300, 1e-200, 1e-300, 1e-320] {
            let r = Transform::rotation(p, Vector3::new(s, 2.0 * s, 3.0 * s), 0.7).unwrap();
            for i in 0..3 {
                for j in 0..3 {
                    assert!((r.m[i][j] - base.m[i][j]).abs() < 1e-14, "scale {s}");
                }
            }
            assert!(close(r.apply_point(Point3::new(0.1, 0.2, 0.3)), base.apply_point(Point3::new(0.1, 0.2, 0.3))), "scale {s}");
        }
        let b = Transform::reflection(Vector3::new(1.0, 0.0, 0.0), 2.0).unwrap();
        for s in [1e200, 1e-200] {
            let r = Transform::reflection(Vector3::new(s, 0.0, 0.0), 2.0 * s).unwrap();
            assert!(close(r.apply_point(Point3::new(0.0, 1.0, 1.0)), b.apply_point(Point3::new(0.0, 1.0, 1.0))), "scale {s}");
        }
        assert!(Transform::rotation(p, Vector3::new(0.0, 0.0, 0.0), 1.0).is_none());
        assert!(Transform::reflection(Vector3::new(0.0, 0.0, 0.0), 1.0).is_none());
    }

    #[test]
    fn aabb_ops() {
        let mut b = Aabb::empty();
        assert!(b.is_empty());
        assert!(!b.contains(Point3::new(0.0, 0.0, 0.0)));
        b.extend(Point3::new(1.0, -1.0, 2.0));
        assert!(!b.is_empty());
        assert_eq!(b, Aabb::from_point(Point3::new(1.0, -1.0, 2.0)));
        b.extend(Point3::new(-1.0, 3.0, 0.0));
        assert_eq!(b.lo, Point3::new(-1.0, -1.0, 0.0));
        assert_eq!(b.hi, Point3::new(1.0, 3.0, 2.0));
        assert!(b.contains(Point3::new(0.0, 0.0, 1.0)));
        assert!(!b.contains(Point3::new(0.0, 0.0, 2.5)));
        assert_eq!(b.union(&Aabb::empty()), b);
        let u = b.union(&Aabb::from_point(Point3::new(5.0, 0.0, 0.0)));
        assert_eq!(u.hi.x, 5.0);
    }

    #[test]
    fn inverse_roundtrip() {
        let t = Transform::rotation(Point3::new(1.0, 2.0, 3.0), Vector3::new(1.0, 1.0, 0.0), 0.7)
            .unwrap()
            .compose(&Transform::translation(Vector3::new(-3.0, 0.5, 2.0)));
        let p = Point3::new(0.1, 0.2, 0.3);
        assert!(close(t.inverse().unwrap().apply_point(t.apply_point(p)), p));
    }
}
