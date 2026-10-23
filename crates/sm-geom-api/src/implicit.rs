//! CSG shape providers: signed implicit surfaces (quadrics, torus) and half-spaces.
//!
//! The convention everywhere: `value(p) > 0` is the *positive* side, the side the gradient (and
//! the analytic surface normal) points to; `value(p) < 0` is the negative side (for closed
//! shapes the inside).

use crate::{Frame, GeomError, Interval, Ray, RayHit, util};
use sm_core::{EntityId, Point3, Vector3};

/// A signed implicit surface `f(p) = 0`. Object-safe; the CSG front end and tracker use
/// `&dyn SignedSurface`.
pub trait SignedSurface: Send + Sync + core::fmt::Debug {
    /// The signed function value.
    fn value(&self, p: Point3) -> f64;
    /// The gradient of `value` (not normalized).
    fn gradient(&self, p: Point3) -> Vector3;
    /// Append all crossings with `range.lo <= t <= range.hi` to `out`, ascending in `t`.
    /// Tangential (double-root) contacts are reported once. `RayHit::normal` is the unit
    /// gradient (zero at a singular point of the surface).
    fn ray_hits(&self, ray: &Ray, range: Interval, out: &mut Vec<RayHit>) -> Result<(), GeomError>;
}

/// Which side of a signed surface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Sign {
    /// `f <= 0` (inside for closed shapes).
    Negative,
    /// `f >= 0`.
    Positive,
}

/// A half-space `sign * f >= 0` of a surface held in a [`crate::GeometryStore`] (kind
/// `Surface`). CSG cells are intersections/unions of these (built in the CSG front end).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct HalfSpace {
    /// Handle of the bounding surface (an `EntityKind::Surface` id).
    pub surface: EntityId,
    /// Which side is inside the half-space.
    pub sign: Sign,
}

impl HalfSpace {
    /// True if `p` is in the half-space, allowing `slack` in function-value units.
    pub fn contains(&self, f: &dyn SignedSurface, p: Point3, slack: f64) -> bool {
        let v = f.value(p);
        match self.sign {
            Sign::Negative => v <= slack,
            Sign::Positive => v >= -slack,
        }
    }
}

/// General quadric `A x^2 + B y^2 + C z^2 + D xy + E yz + F zx + G x + H y + J z + K` (MCNP "GQ"
/// ordering).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Quadric {
    /// Coefficients `[A, B, C, D, E, F, G, H, J, K]`.
    pub coeffs: [f64; 10],
}

impl Quadric {
    /// From the local form `sum d_i l_i^2 + sum lin_i l_i + k` with `l` the coordinates in
    /// `frame`. Covers planes, spheres, cylinders, cones, ellipsoids, paraboloids.
    pub fn from_local(frame: &Frame, d: [f64; 3], lin: [f64; 3], k: f64) -> Self {
        let e = [frame.x, frame.y, frame.z];
        let o = util::pos(frame.origin);
        let (mut a, mut b, mut c, mut dd, mut ee, mut ff) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        for i in 0..3 {
            a += d[i] * e[i].x * e[i].x;
            b += d[i] * e[i].y * e[i].y;
            c += d[i] * e[i].z * e[i].z;
            dd += 2.0 * d[i] * e[i].x * e[i].y;
            ee += 2.0 * d[i] * e[i].y * e[i].z;
            ff += 2.0 * d[i] * e[i].z * e[i].x;
        }
        let l = util::comb3(lin[0], e[0], lin[1], e[1], lin[2], e[2]);
        // A*o with the symmetric matrix [[a, dd/2, ff/2], [dd/2, b, ee/2], [ff/2, ee/2, c]].
        let ao = Vector3::new(a * o.x + 0.5 * dd * o.y + 0.5 * ff * o.z, 0.5 * dd * o.x + b * o.y + 0.5 * ee * o.z, 0.5 * ff * o.x + 0.5 * ee * o.y + c * o.z);
        let g = l.x - 2.0 * ao.x;
        let h = l.y - 2.0 * ao.y;
        let j = l.z - 2.0 * ao.z;
        let kk = k + o.dot(ao) - l.dot(o);
        Self { coeffs: [a, b, c, dd, ee, ff, g, h, j, kk] }
    }

    /// The plane through `frame.origin` with normal `frame.z`; positive side along `z`.
    pub fn plane(frame: &Frame) -> Self {
        Self::from_local(frame, [0.0; 3], [0.0, 0.0, 1.0], 0.0)
    }

    /// Sphere of radius `r` about the frame origin (negative inside).
    pub fn sphere(frame: &Frame, r: f64) -> Self {
        Self::from_local(frame, [1.0; 3], [0.0; 3], -r * r)
    }

    /// Infinite cylinder of radius `r` about the frame's `z` axis (negative inside).
    pub fn cylinder(frame: &Frame, r: f64) -> Self {
        Self::from_local(frame, [1.0, 1.0, 0.0], [0.0; 3], -r * r)
    }

    /// Double cone with apex at the frame origin, axis `z`, half-angle `half_angle` (negative
    /// inside).
    pub fn cone(frame: &Frame, half_angle: f64) -> Self {
        let t2 = half_angle.tan().powi(2);
        Self::from_local(frame, [1.0, 1.0, -t2], [0.0; 3], 0.0)
    }
}

impl SignedSurface for Quadric {
    fn value(&self, p: Point3) -> f64 {
        let [a, b, c, d, e, f, g, h, j, k] = self.coeffs;
        a * p.x * p.x + b * p.y * p.y + c * p.z * p.z + d * p.x * p.y + e * p.y * p.z + f * p.z * p.x + g * p.x + h * p.y + j * p.z + k
    }

    fn gradient(&self, p: Point3) -> Vector3 {
        let [a, b, c, d, e, f, g, h, j, _] = self.coeffs;
        Vector3::new(2.0 * a * p.x + d * p.y + f * p.z + g, 2.0 * b * p.y + d * p.x + e * p.z + h, 2.0 * c * p.z + e * p.y + f * p.x + j)
    }

    fn ray_hits(&self, ray: &Ray, range: Interval, out: &mut Vec<RayHit>) -> Result<(), GeomError> {
        let [a, b, c, d, e, f, g, h, j, _] = self.coeffs;
        let (o, v) = (ray.origin, ray.dir);
        let qa = a * v.x * v.x + b * v.y * v.y + c * v.z * v.z + d * v.x * v.y + e * v.y * v.z + f * v.z * v.x;
        let qb = 2.0 * (a * o.x * v.x + b * o.y * v.y + c * o.z * v.z)
            + d * (o.x * v.y + o.y * v.x)
            + e * (o.y * v.z + o.z * v.y)
            + f * (o.z * v.x + o.x * v.z)
            + g * v.x
            + h * v.y
            + j * v.z;
        let qc = self.value(o);
        let mut roots = [0.0f64; 2];
        let n = solve_quadratic(qa, qb, qc, &mut roots);
        push_hits(self, ray, range, &roots[..n], out);
        Ok(())
    }
}

/// Torus about the frame's `z` axis as the quartic `(|l|^2 + R^2 - r^2)^2 - 4 R^2 (lx^2 + ly^2)`
/// in local coordinates `l` (negative inside the tube). Requires `major > minor > 0`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TorusImplicit {
    /// Placement; `z` is the symmetry axis.
    pub frame: Frame,
    /// Major radius `R`.
    pub major: f64,
    /// Minor (tube) radius `r`.
    pub minor: f64,
}

impl SignedSurface for TorusImplicit {
    fn value(&self, p: Point3) -> f64 {
        let l = self.frame.to_local(p);
        let (rr, k) = (self.major * self.major, self.major * self.major - self.minor * self.minor);
        let s = l.dot(l) + k;
        s * s - 4.0 * rr * (l.x * l.x + l.y * l.y)
    }

    fn gradient(&self, p: Point3) -> Vector3 {
        let l = self.frame.to_local(p);
        let (rr, k) = (self.major * self.major, self.major * self.major - self.minor * self.minor);
        let s = l.dot(l) + k;
        let g = Vector3::new(4.0 * s * l.x - 8.0 * rr * l.x, 4.0 * s * l.y - 8.0 * rr * l.y, 4.0 * s * l.z);
        self.frame.vec_from_local(g)
    }

    fn ray_hits(&self, ray: &Ray, range: Interval, out: &mut Vec<RayHit>) -> Result<(), GeomError> {
        let o = self.frame.to_local(ray.origin);
        let d = self.frame.to_local_vec(ray.dir);
        let rr = self.major * self.major;
        let k = rr - self.minor * self.minor;
        let (a, b, c) = (d.dot(d), o.dot(d), o.dot(o));
        let (e, g, h) = (d.x * d.x + d.y * d.y, o.x * d.x + o.y * d.y, o.x * o.x + o.y * o.y);
        let ck = c + k;
        let coef = [ck * ck - 4.0 * rr * h, 4.0 * b * ck - 8.0 * rr * g, 4.0 * b * b + 2.0 * a * ck - 4.0 * rr * e, 4.0 * a * b, a * a];
        let roots = real_roots(&coef, range.lo, range.hi);
        push_hits(self, ray, range, &roots, out);
        Ok(())
    }
}

/// Either implicit form of an analytic surface. Cheap `Copy` value returned by
/// [`crate::ParametricSurface::implicit`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ImplicitForm {
    /// A (degree <= 2) quadric.
    Quadric(Quadric),
    /// A torus (degree 4).
    Torus(TorusImplicit),
}

impl SignedSurface for ImplicitForm {
    fn value(&self, p: Point3) -> f64 {
        match self {
            Self::Quadric(q) => q.value(p),
            Self::Torus(t) => t.value(p),
        }
    }
    fn gradient(&self, p: Point3) -> Vector3 {
        match self {
            Self::Quadric(q) => q.gradient(p),
            Self::Torus(t) => t.gradient(p),
        }
    }
    fn ray_hits(&self, ray: &Ray, range: Interval, out: &mut Vec<RayHit>) -> Result<(), GeomError> {
        match self {
            Self::Quadric(q) => q.ray_hits(ray, range, out),
            Self::Torus(t) => t.ray_hits(ray, range, out),
        }
    }
}

fn push_hits<S: SignedSurface + ?Sized>(s: &S, ray: &Ray, range: Interval, roots: &[f64], out: &mut Vec<RayHit>) {
    let mut sorted = [0.0f64; 4];
    let n = roots.len().min(4);
    sorted[..n].copy_from_slice(&roots[..n]);
    sorted[..n].sort_by(f64::total_cmp);
    for &t in &sorted[..n] {
        if t >= range.lo && t <= range.hi {
            let point = ray.at(t);
            let normal = util::unit(s.gradient(point)).unwrap_or_default();
            out.push(RayHit { t, point, normal });
        }
    }
}

/// Real roots of `a t^2 + b t + c` (stable form). Returns the count written to `out`
/// (a double root counts once). Degenerates to the linear case when `a` is negligible.
pub(crate) fn solve_quadratic(a: f64, b: f64, c: f64, out: &mut [f64; 2]) -> usize {
    let scale = a.abs().max(b.abs()).max(c.abs());
    if scale == 0.0 {
        return 0;
    }
    if a.abs() <= 1e-14 * scale {
        if b.abs() <= 1e-14 * scale {
            return 0;
        }
        out[0] = -c / b;
        return 1;
    }
    let disc = b * b - 4.0 * a * c;
    if disc < -1e-14 * b * b {
        return 0;
    }
    if disc <= 1e-14 * b * b {
        out[0] = -b / (2.0 * a);
        return 1;
    }
    let sq = disc.sqrt();
    let q = -0.5 * (b + b.signum() * sq);
    let (r1, r2) = (q / a, if q != 0.0 { c / q } else { q / a });
    out[0] = r1.min(r2);
    out[1] = r1.max(r2);
    2
}

/// Real roots of the polynomial with ascending coefficients `c` inside `[lo, hi]` (either may
/// be infinite). Isolates monotone pieces with the roots of the derivative, then bisects;
/// double roots are detected from near-zero critical values. Sorted, deduplicated.
pub(crate) fn real_roots(c: &[f64], lo: f64, hi: f64) -> Vec<f64> {
    let maxabs = c.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    let mut n = c.len();
    while n > 1 && c[n - 1].abs() <= 1e-14 * maxabs {
        n -= 1;
    }
    let c = &c[..n];
    if n <= 1 {
        return Vec::new();
    }
    if n == 2 {
        let r = -c[0] / c[1];
        return if r >= lo && r <= hi { vec![r] } else { Vec::new() };
    }
    let lead = c[n - 1].abs();
    let bound = 1.0 + c[..n - 1].iter().fold(0.0f64, |m, v| m.max(v.abs())) / lead;
    let (lo, hi) = (lo.max(-bound), hi.min(bound));
    if lo > hi {
        return Vec::new();
    }
    let deriv: Vec<f64> = c.iter().enumerate().skip(1).map(|(i, v)| v * i as f64).collect();
    let crit = real_roots(&deriv, lo, hi);
    let eval = |t: f64| c.iter().rev().fold(0.0, |acc, v| acc * t + v);
    let abs_eval = |t: f64| c.iter().rev().fold(0.0, |acc, v| acc * t.abs() + v.abs());
    let mut pts = Vec::with_capacity(crit.len() + 2);
    pts.push(lo);
    pts.extend(crit.iter().copied());
    pts.push(hi);
    let mut roots = Vec::new();
    for w in pts.windows(2) {
        let (mut a, mut b) = (w[0], w[1]);
        let (mut fa, fb) = (eval(a), eval(b));
        if fa == 0.0 {
            roots.push(a);
        } else if fa * fb < 0.0 {
            for _ in 0..200 {
                let m = 0.5 * (a + b);
                if m <= a || m >= b {
                    break;
                }
                let fm = eval(m);
                if fm == 0.0 {
                    a = m;
                    b = m;
                    break;
                }
                if fa * fm < 0.0 {
                    b = m;
                } else {
                    a = m;
                    fa = fm;
                }
            }
            roots.push(0.5 * (a + b));
        }
        if fb == 0.0 {
            roots.push(b);
        }
    }
    for &t in &crit {
        if eval(t).abs() <= 1e-12 * abs_eval(t) {
            roots.push(t);
        }
    }
    roots.sort_by(f64::total_cmp);
    roots.dedup_by(|a, b| (*a - *b).abs() <= 1e-9 * (1.0 + b.abs()));
    roots
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ray_x(x0: f64) -> Ray {
        Ray { origin: Point3::new(x0, 0.0, 0.0), dir: Vector3::new(1.0, 0.0, 0.0) }
    }

    #[test]
    fn sphere_ray_hits_analytic() {
        let q = Quadric::sphere(&Frame::at(Point3::new(0.0, 0.0, 0.0)), 2.0);
        let mut out = Vec::new();
        q.ray_hits(&ray_x(-5.0), Interval::new(0.0, 100.0), &mut out).unwrap();
        assert_eq!(out.len(), 2);
        assert!((out[0].t - 3.0).abs() < 1e-12 && (out[1].t - 7.0).abs() < 1e-12);
        assert!((out[0].normal.x + 1.0).abs() < 1e-12);
        assert!(out[1].leaving(&ray_x(-5.0)));
    }

    #[test]
    fn tangent_and_miss() {
        let q = Quadric::sphere(&Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0);
        let mut out = Vec::new();
        let grazing = Ray { origin: Point3::new(-3.0, 1.0, 0.0), dir: Vector3::new(1.0, 0.0, 0.0) };
        q.ray_hits(&grazing, Interval::new(0.0, 10.0), &mut out).unwrap();
        assert_eq!(out.len(), 1);
        out.clear();
        let miss = Ray { origin: Point3::new(-3.0, 1.5, 0.0), dir: Vector3::new(1.0, 0.0, 0.0) };
        q.ray_hits(&miss, Interval::new(0.0, 10.0), &mut out).unwrap();
        assert!(out.is_empty());
    }

    #[test]
    fn plane_and_offset_frame() {
        let f = Frame::from_z(Point3::new(1.0, 2.0, 3.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let q = Quadric::plane(&f);
        assert!((q.value(Point3::new(9.0, 9.0, 5.0)) - 2.0).abs() < 1e-12);
        let r = Ray { origin: Point3::new(0.0, 0.0, 0.0), dir: Vector3::new(0.0, 0.0, 1.0) };
        let mut out = Vec::new();
        q.ray_hits(&r, Interval::new(0.0, 10.0), &mut out).unwrap();
        assert_eq!(out.len(), 1);
        assert!((out[0].t - 3.0).abs() < 1e-12);
    }

    #[test]
    fn torus_ray_through_axis_plane() {
        let t = TorusImplicit { frame: Frame::at(Point3::new(0.0, 0.0, 0.0)), major: 3.0, minor: 1.0 };
        let mut out = Vec::new();
        t.ray_hits(&ray_x(-10.0), Interval::new(0.0, 100.0), &mut out).unwrap();
        let ts: Vec<f64> = out.iter().map(|h| h.t).collect();
        assert_eq!(ts.len(), 4);
        for (got, want) in ts.iter().zip([6.0, 8.0, 12.0, 14.0]) {
            assert!((got - want).abs() < 1e-9, "{got} vs {want}");
        }
        // Tube top grazes at z = r: a double root.
        out.clear();
        let graze = Ray { origin: Point3::new(-10.0, 0.0, 1.0), dir: Vector3::new(1.0, 0.0, 0.0) };
        t.ray_hits(&graze, Interval::new(0.0, 100.0), &mut out).unwrap();
        assert_eq!(out.len(), 2, "two tangent contacts at x=-3 and x=3");
    }

    #[test]
    fn halfspace_membership() {
        let q = Quadric::sphere(&Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0);
        let id = EntityId::new(sm_core::EntityKind::Surface, 0).unwrap();
        let inside = HalfSpace { surface: id, sign: Sign::Negative };
        assert!(inside.contains(&q, Point3::new(0.5, 0.0, 0.0), 0.0));
        assert!(!inside.contains(&q, Point3::new(2.0, 0.0, 0.0), 0.0));
    }

    #[test]
    fn polynomial_roots() {
        // (t-1)(t-2)(t+3) = t^3 - 7t + 6
        let r = real_roots(&[6.0, -7.0, 0.0, 1.0], f64::NEG_INFINITY, f64::INFINITY);
        assert_eq!(r.len(), 3);
        for (g, w) in r.iter().zip([-3.0, 1.0, 2.0]) {
            assert!((g - w).abs() < 1e-12);
        }
    }
}
