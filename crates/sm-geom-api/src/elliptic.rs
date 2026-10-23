//! Elliptic quadrics (W0f gaps 2): the exact images of circular cylinders and cones under a
//! non-uniform affine map, and the elliptic frustum primitive (PARITY B).
//!
//! Parametrizations (outward normals `du x dv` in a right-handed frame):
//! - [`EllipticCylinder`] `o + a cos u x + b sin u y + v z`.
//! - [`EllipticCone`] `o + k(v)(a cos u x + b sin u y) + v cos h z` with `k(v) = 1 + v sin h / a`:
//!   the section at `v = 0` is the ellipse `(a, b)`, sections scale about the axis, and `h` is
//!   the half-angle in the `x`-`z` plane (with `a = b = r` this is [`crate::Cone`]). The apex is at
//!   `v = -a / sin h`; the half-angle in the `y`-`z` plane is `atan(b tan h / a)`.

use crate::implicit::{ImplicitDef, Quadric};
use crate::util::{comb2, unit, wrap_tau};
use crate::{Domain1, Domain2, Frame, GeomError, ImplicitSurface, Interval, ParamAxis, ParametricSurface, Ray, RayHit, Singularity, SurfaceJet, SurfaceProjection, Uv};
use core::f64::consts::{FRAC_PI_2, TAU};
use sm_core::{Point3, Vector3};
use smallvec::smallvec;

fn positive(v: f64, what: &'static str) -> Result<f64, GeomError> {
    if v.is_finite() && v > 0.0 { Ok(v) } else { Err(GeomError::Invalid(what)) }
}

/// Closest point on the ellipse `(x/a)^2 + (y/b)^2 = 1` to `(px, py)`, as the angle `t` of
/// `(a cos t, b sin t)` (bisection on the Lagrange root, Eberly's robust method).
fn ellipse_closest_angle(a: f64, b: f64, px: f64, py: f64) -> f64 {
    // Work in the first quadrant with e0 >= e1.
    let swap = b > a;
    let (e0, e1, y0, y1) = if swap { (b, a, py.abs(), px.abs()) } else { (a, b, px.abs(), py.abs()) };
    let (x0, x1) = if y1 > 0.0 {
        if y0 > 0.0 {
            let z0 = y0 / e0;
            let z1 = y1 / e1;
            let g = z0 * z0 + z1 * z1 - 1.0;
            if g != 0.0 {
                let r0 = (e0 / e1) * (e0 / e1);
                let n0 = r0 * z0;
                let (mut s0, mut s1) = (z1 - 1.0, if g < 0.0 { 0.0 } else { n0.hypot(z1) - 1.0 });
                let mut s = 0.0;
                for _ in 0..160 {
                    s = 0.5 * (s0 + s1);
                    if s == s0 || s == s1 {
                        break;
                    }
                    let ratio0 = n0 / (s + r0);
                    let ratio1 = z1 / (s + 1.0);
                    let gs = ratio0 * ratio0 + ratio1 * ratio1 - 1.0;
                    if gs > 0.0 {
                        s0 = s;
                    } else if gs < 0.0 {
                        s1 = s;
                    } else {
                        break;
                    }
                }
                (r0 * y0 / (s + r0), y1 / (s + 1.0))
            } else {
                (y0, y1)
            }
        } else {
            (0.0, e1)
        }
    } else {
        let numer0 = e0 * y0;
        let denom0 = e0 * e0 - e1 * e1;
        if numer0 < denom0 {
            let xde0 = numer0 / denom0;
            (e0 * xde0, e1 * (1.0 - xde0 * xde0).max(0.0).sqrt())
        } else {
            (e0, 0.0)
        }
    };
    // Back to the original quadrant and axes.
    let (qx, qy) = if swap { (x1, x0) } else { (x0, x1) };
    let (qx, qy) = (qx.copysign(px), qy.copysign(py));
    wrap_tau((qy / b).atan2(qx / a))
}

/// Infinite elliptic cylinder about `frame.z`: semi-axis `a` along `frame.x`, `b` along `frame.y`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct EllipticCylinder {
    /// Placement (right-handed); `u` is the eccentric angle, `v` the height along `z`.
    pub frame: Frame,
    /// Semi-axis along `frame.x`.
    pub a: f64,
    /// Semi-axis along `frame.y`.
    pub b: f64,
}

impl EllipticCylinder {
    /// Build an elliptic cylinder (both semi-axes positive and finite).
    pub fn new(frame: Frame, a: f64, b: f64) -> Result<Self, GeomError> {
        Ok(Self { frame, a: positive(a, "elliptic cylinder a")?, b: positive(b, "elliptic cylinder b")? })
    }
}

impl ParametricSurface for EllipticCylinder {
    fn domain(&self) -> Domain2 {
        Domain2::regular(Domain1::periodic(0.0, TAU), Domain1::open(f64::NEG_INFINITY, f64::INFINITY))
    }
    fn jet(&self, uv: Uv) -> SurfaceJet {
        let f = &self.frame;
        let (s, c) = uv.u.sin_cos();
        let radial = comb2(self.a * c, f.x, self.b * s, f.y);
        let z = Vector3::default();
        SurfaceJet { p: f.origin + radial + f.z * uv.v, du: comb2(-self.a * s, f.x, self.b * c, f.y), dv: f.z, duu: -radial, duv: z, dvv: z }
    }
    fn normal(&self, uv: Uv) -> Result<Vector3, GeomError> {
        let (s, c) = uv.u.sin_cos();
        unit(comb2(self.b * c, self.frame.x, self.a * s, self.frame.y)).ok_or(GeomError::Singular("elliptic cylinder normal"))
    }
    /// Exact closest point (the closest point on the cross-section ellipse, by bisection).
    fn project(&self, p: Point3, _hint: Option<Uv>) -> Result<SurfaceProjection, GeomError> {
        let l = self.frame.to_local(p);
        let u = ellipse_closest_angle(self.a, self.b, l.x, l.y);
        let uv = Uv::new(u, l.z);
        let point = self.eval(uv);
        Ok(SurfaceProjection { uv, point, distance: (p - point).norm() })
    }
    fn implicit(&self) -> Option<ImplicitDef> {
        Some(ImplicitDef::Quadric(Quadric::from_local(&self.frame, [1.0 / (self.a * self.a), 1.0 / (self.b * self.b), 0.0], [0.0; 3], -1.0)))
    }
}

/// One nappe of a right elliptic cone about `frame.z`, widening toward `+z` (see the module docs).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct EllipticCone {
    /// Placement (right-handed); the ellipse `(a, b)` at `v = 0` is centred on `frame.origin`.
    pub frame: Frame,
    /// Semi-axis along `frame.x` at `v = 0`.
    pub a: f64,
    /// Semi-axis along `frame.y` at `v = 0`.
    pub b: f64,
    /// Half-angle between the axis and the generator in the `x`-`z` plane, in `(0, pi/2)`.
    pub half_angle: f64,
}

impl EllipticCone {
    /// Build an elliptic cone (`a`, `b` positive and finite, half-angle in `(0, pi/2)`).
    pub fn new(frame: Frame, a: f64, b: f64, half_angle: f64) -> Result<Self, GeomError> {
        if !(half_angle > 0.0 && half_angle < FRAC_PI_2) {
            return Err(GeomError::Invalid("elliptic cone half-angle"));
        }
        Ok(Self { frame, a: positive(a, "elliptic cone a")?, b: positive(b, "elliptic cone b")?, half_angle })
    }
    /// The `v` value of the apex.
    pub fn apex_v(&self) -> f64 {
        -self.a / self.half_angle.sin()
    }
    /// The apex point.
    pub fn apex(&self) -> Point3 {
        self.frame.origin + self.frame.z * (self.apex_v() * self.half_angle.cos())
    }
    fn k(&self, v: f64) -> f64 {
        1.0 + v * self.half_angle.sin() / self.a
    }
}

impl ParametricSurface for EllipticCone {
    fn domain(&self) -> Domain2 {
        let mut d = Domain2::regular(Domain1::periodic(0.0, TAU), Domain1::open(self.apex_v(), f64::INFINITY));
        d.singularities = smallvec![Singularity { along: ParamAxis::V, at: self.apex_v(), point: self.apex() }];
        d
    }
    fn jet(&self, uv: Uv) -> SurfaceJet {
        let f = &self.frame;
        let (sh, ch) = self.half_angle.sin_cos();
        let (s, c) = uv.u.sin_cos();
        let k = self.k(uv.v);
        let kv = sh / self.a;
        let e = comb2(self.a * c, f.x, self.b * s, f.y);
        let de = comb2(-self.a * s, f.x, self.b * c, f.y);
        SurfaceJet { p: f.origin + e * k + f.z * (uv.v * ch), du: de * k, dv: e * kv + f.z * ch, duu: e * -k, duv: de * kv, dvv: Vector3::default() }
    }
    /// The normal is constant along a generator, so it is exact at the apex too.
    fn normal(&self, uv: Uv) -> Result<Vector3, GeomError> {
        let f = &self.frame;
        let (sh, ch) = self.half_angle.sin_cos();
        let (s, c) = uv.u.sin_cos();
        let e = comb2(self.a * c, f.x, self.b * s, f.y);
        let de = comb2(-self.a * s, f.x, self.b * c, f.y);
        unit(de.cross(e * (sh / self.a) + f.z * ch)).ok_or(GeomError::Singular("elliptic cone normal"))
    }
    /// Not provided (no closed form); `sm-geom` supplies an iterative version.
    fn project(&self, _p: Point3, _hint: Option<Uv>) -> Result<SurfaceProjection, GeomError> {
        Err(GeomError::Unsupported("elliptic cone projection"))
    }
    /// The double cone `(x/a)^2 + (y/b)^2 - (1 + z tan h / a)^2` (negative inside), like
    /// [`crate::Cone`]'s.
    fn implicit(&self) -> Option<ImplicitDef> {
        let t = self.half_angle.tan() / self.a;
        // (x/a)^2 + (y/b)^2 - (1 + t z)^2 = x^2/a^2 + y^2/b^2 - t^2 z^2 - 2 t z - 1.
        Some(ImplicitDef::Quadric(Quadric::from_local(&self.frame, [1.0 / (self.a * self.a), 1.0 / (self.b * self.b), -t * t], [0.0, 0.0, -2.0 * t], -1.0)))
    }
    /// Intersect the double-cone quadric, keeping only the nappe `v >= apex_v`.
    fn ray_hits(&self, ray: &Ray, range: Interval, out: &mut Vec<RayHit>) -> Result<(), GeomError> {
        let Some(ImplicitDef::Quadric(q)) = self.implicit() else { return Err(GeomError::Unsupported("elliptic cone")) };
        let first = out.len();
        q.ray_hits(ray, range, out)?;
        let apex = self.apex();
        let mut keep = first;
        for i in first..out.len() {
            if (out[i].point - apex).dot(self.frame.z) >= -1e-12 {
                out.swap(keep, i);
                keep += 1;
            }
        }
        out.truncate(keep);
        for h in &mut out[first..] {
            // u from the local position; v from the height (exact on the surface).
            let l = self.frame.to_local(h.point);
            let v = l.z / self.half_angle.cos();
            let k = self.k(v);
            h.uv = (k > 0.0).then(|| Uv::new(wrap_tau((l.y / (self.b * k)).atan2(l.x / (self.a * k))), v));
        }
        Ok(())
    }
}

/// Eigen-decomposition of a symmetric 3x3 matrix by cyclic Jacobi (deterministic): eigenvalues
/// and unit eigenvectors (columns), unsorted.
fn sym_eigen(mut a: [[f64; 3]; 3]) -> ([f64; 3], [Vector3; 3]) {
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..64 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        let scale = a[0][0].abs() + a[1][1].abs() + a[2][2].abs();
        if off <= 1e-300 || off <= 1e-17 * scale {
            break;
        }
        for (p, q) in [(0usize, 1usize), (0, 2), (1, 2)] {
            if a[p][q] == 0.0 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let t = if theta == 0.0 { 1.0 } else { t };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for k in 0..3 {
                let (akp, akq) = (a[k][p], a[k][q]);
                a[k][p] = c * akp - s * akq;
                a[k][q] = s * akp + c * akq;
            }
            for k in 0..3 {
                let (apk, aqk) = (a[p][k], a[q][k]);
                a[p][k] = c * apk - s * aqk;
                a[q][k] = s * apk + c * aqk;
            }
            for row in &mut v {
                let (vp, vq) = (row[p], row[q]);
                row[p] = c * vp - s * vq;
                row[q] = s * vp + c * vq;
            }
        }
    }
    let col = |j: usize| Vector3::new(v[0][j], v[1][j], v[2][j]);
    ([a[0][0], a[1][1], a[2][2]], [col(0), col(1), col(2)])
}

/// The right elliptic cone of a quadric cone `q` (signature `(+, +, -)`, negative inside), the
/// nappe that `toward` points into from the apex, placed with its `v = 0` section through
/// `section` (projected on the axis; a point of that nappe away from the apex).
pub(crate) fn cone_from_quadric(q: &Quadric, toward: Vector3, section: Point3) -> Result<EllipticCone, GeomError> {
    let [a, b, c, d, e, f, g, h, j, _] = q.coeffs;
    let m = [[a, d / 2.0, f / 2.0], [d / 2.0, b, e / 2.0], [f / 2.0, e / 2.0, c]];
    // Apex: grad = 2 M x + l = 0.
    let t = sm_core::Transform { m, t: Vector3::default() };
    let inv = t.inverse().ok_or(GeomError::Singular("degenerate cone image"))?;
    let apex = Point3::default() + inv.apply_vector(Vector3::new(g, h, j)) * -0.5;
    let (vals, vecs) = sym_eigen(m);
    let neg: Vec<usize> = (0..3).filter(|i| vals[*i] < 0.0).collect();
    let [ni] = neg.as_slice() else { return Err(GeomError::Singular("cone image is not a cone")) };
    let (i1, i2) = match ni {
        0 => (1, 2),
        1 => (0, 2),
        _ => (0, 1),
    };
    // x along the smaller semi-axis slope? Keep the larger positive eigenvalue second so the
    // choice is deterministic: x = eigenvector with the smaller eigenvalue (larger semi-axis).
    let (ix, iy) = if vals[i1] <= vals[i2] { (i1, i2) } else { (i2, i1) };
    let mut z = vecs[*ni];
    if z.dot(toward) < 0.0 {
        z = -z;
    }
    let x = vecs[ix];
    let y = z.cross(x);
    let h0 = (section - apex).dot(z);
    if !(h0.is_finite() && h0 > 0.0) {
        return Err(GeomError::Singular("cone section at the apex"));
    }
    let lz = -vals[*ni];
    let (sa, sb) = ((lz / vals[ix]).sqrt(), (lz / vals[iy]).sqrt());
    let half_angle = sa.atan();
    let frame = Frame { origin: apex + z * h0, x, y, z };
    EllipticCone::new(frame, h0 * sa, h0 * sb, half_angle)
}

/// The elliptic cylinder through `o + cos u p + sin u w + v z` (any `p`, `w` spanning a plane not
/// containing `z`): the perpendicular section in principal axes.
pub(crate) fn cylinder_from_section(o: Point3, p: Vector3, w: Vector3, z: Vector3) -> Result<EllipticCylinder, GeomError> {
    let z = unit(z).ok_or(GeomError::Singular("degenerate cylinder image"))?;
    let pp = p - z * p.dot(z);
    let ww = w - z * w.dot(z);
    let (fr, a, b, _) = crate::transform::principal_ellipse(o, pp, ww)?;
    // principal_ellipse gives z' = x x y = +-z; make the frame right-handed about +z.
    let y = if fr.z.dot(z) < 0.0 { -fr.y } else { fr.y };
    EllipticCylinder::new(Frame { origin: fr.origin, x: fr.x, y, z }, a, b)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tilted() -> Frame {
        Frame::from_zx(Point3::new(0.3, -0.7, 1.1), Vector3::new(1.0, 2.0, -0.5), Vector3::new(0.0, 0.0, 1.0)).unwrap()
    }

    fn check_param(s: &dyn ParametricSurface, uvs: &[Uv]) {
        let q = s.implicit().unwrap();
        for &uv in uvs {
            let j = s.jet(uv);
            assert!(q.value(j.p).abs() < 1e-12, "{s:?} {uv:?}: residual {}", q.value(j.p));
            // Outward: the normal agrees with the implicit gradient (negative inside).
            let n = s.normal(uv).unwrap();
            assert!(n.dot(q.gradient(j.p)) > 0.0);
            assert!((n - unit(j.du.cross(j.dv)).unwrap()).norm() < 1e-12);
            // Derivatives by central differences.
            let hh = 1e-6;
            let fd = |a: Uv, b: Uv| (s.eval(a) - s.eval(b)) * (0.5 / hh);
            let du = fd(Uv::new(uv.u + hh, uv.v), Uv::new(uv.u - hh, uv.v));
            let dv = fd(Uv::new(uv.u, uv.v + hh), Uv::new(uv.u, uv.v - hh));
            assert!((du - j.du).norm() < 1e-7 && (dv - j.dv).norm() < 1e-7);
            let duu = (s.jet(Uv::new(uv.u + hh, uv.v)).du - s.jet(Uv::new(uv.u - hh, uv.v)).du) * (0.5 / hh);
            let duv = (s.jet(Uv::new(uv.u, uv.v + hh)).du - s.jet(Uv::new(uv.u, uv.v - hh)).du) * (0.5 / hh);
            assert!((duu - j.duu).norm() < 1e-6 && (duv - j.duv).norm() < 1e-6);
        }
    }

    #[test]
    fn parametrizations_lie_on_their_quadrics_with_outward_normals() {
        let uvs = [Uv::new(0.3, 0.2), Uv::new(2.0, -0.5), Uv::new(4.0, 1.4)];
        check_param(&EllipticCylinder::new(tilted(), 2.0, 0.5).unwrap(), &uvs);
        check_param(&EllipticCone::new(tilted(), 2.0, 0.5, 0.4).unwrap(), &uvs);
        // a = b reduces to the circular kinds.
        let cy = EllipticCylinder::new(tilted(), 1.5, 1.5).unwrap();
        let cc = crate::Cylinder::new(tilted(), 1.5).unwrap();
        let co = EllipticCone::new(tilted(), 1.5, 1.5, 0.4).unwrap();
        let cr = crate::Cone::new(tilted(), 1.5, 0.4).unwrap();
        for uv in uvs {
            assert!((cy.eval(uv) - cc.eval(uv)).norm() < 1e-12);
            assert!((co.eval(uv) - cr.eval(uv)).norm() < 1e-12);
        }
        assert!((co.apex() - cr.apex()).norm() < 1e-12);
    }

    #[test]
    fn analytic_values() {
        // Elliptic cone a = 2, b = 1, h = 45 deg: apex 2 below the origin; the y-z half-angle is
        // atan(1/2).
        let c = EllipticCone::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 2.0, 1.0, core::f64::consts::FRAC_PI_4).unwrap();
        assert!((c.apex() - Point3::new(0.0, 0.0, -2.0)).norm() < 1e-12);
        let p = c.eval(Uv::new(FRAC_PI_2, 0.0));
        assert!((p - Point3::new(0.0, 1.0, 0.0)).norm() < 1e-12);
        assert!(c.normal(Uv::new(0.0, c.apex_v())).is_ok(), "exact normal at the apex");
        assert!(EllipticCone::new(Frame::at(Point3::default()), 0.0, 1.0, 0.3).is_err());
        assert!(EllipticCylinder::new(Frame::at(Point3::default()), 1.0, -1.0).is_err());
        // Ray along x through the origin hits the cylinder (a = 3, b = 1) at x = +-3.
        let cy = EllipticCylinder::new(Frame::at(Point3::default()), 3.0, 1.0).unwrap();
        let mut out = Vec::new();
        cy.ray_hits(&Ray { origin: Point3::new(-10.0, 0.0, 0.0), dir: Vector3::new(1.0, 0.0, 0.0) }, Interval::new(0.0, 100.0), &mut out).unwrap();
        assert_eq!(out.len(), 2);
        assert!((out[0].t - 7.0).abs() < 1e-12 && (out[1].t - 13.0).abs() < 1e-12);
        assert!((out[1].uv.unwrap().u).abs() < 1e-12);
        // Only the +z nappe of the cone is kept.
        out.clear();
        c.ray_hits(&Ray { origin: Point3::new(0.0, 0.0, -10.0), dir: Vector3::new(0.0, 0.0, 1.0) }, Interval::new(0.0, 100.0), &mut out).unwrap();
        assert_eq!(out.len(), 1);
    }

    #[test]
    fn cylinder_projection_is_the_closest_point() {
        let cy = EllipticCylinder::new(tilted(), 2.0, 0.5).unwrap();
        for (x, y, z) in [(3.0, 1.0, 0.2), (-0.1, 0.05, 1.0), (0.0, -2.0, -3.0), (1.9, 0.0, 0.0), (0.0, 0.0, 0.0), (-5.0, -5.0, 2.0)] {
            let p = cy.frame.from_local(Vector3::new(x, y, z));
            let pr = cy.project(p, None).unwrap();
            // Brute force over the section.
            let best = (0..20000).map(|i| (cy.eval(Uv::new(TAU * i as f64 / 20000.0, z)) - p).norm()).fold(f64::INFINITY, f64::min);
            assert!(pr.distance <= best + 1e-9, "({x}, {y}, {z}): {} vs {best}", pr.distance);
            assert!((pr.point - p).norm() - pr.distance < 1e-12);
        }
    }

    #[test]
    fn jacobi_diagonalizes() {
        let m = [[4.0, 1.0, -2.0], [1.0, 2.0, 0.5], [-2.0, 0.5, -3.0]];
        let (vals, vecs) = sym_eigen(m);
        for i in 0..3 {
            let v = vecs[i];
            let mv = Vector3::new(m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z, m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z, m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
            assert!((mv - v * vals[i]).norm() < 1e-12);
        }
    }
}
