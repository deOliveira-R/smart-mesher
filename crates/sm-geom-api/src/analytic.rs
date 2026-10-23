//! Analytic curves and surfaces (the exact kinds every CAD import and CSG front end produces).
//!
//! Parametrizations (all outward normals `du x dv`):
//! - [`Line`] `o + t d`; [`Circle`] `c + r(cos t x + sin t y)`; [`Ellipse`] `c + a cos t x + b sin t y`.
//! - [`Plane`] `o + u x + v y`, normal `z`.
//! - [`Cylinder`] `o + r(cos u x + sin u y) + v z`.
//! - [`Cone`] `o + (r0 + v sin a)(cos u x + sin u y) + v cos a z`, apex at `v = -r0 / sin a`.
//! - [`Sphere`] longitude `u`, latitude `v` in `[-pi/2, pi/2]`.
//! - [`Torus`] `(R + r cos v)(cos u x + sin u y) + r sin v z`, requires `R > r > 0`.

use crate::implicit::{ImplicitDef, Quadric, ImplicitTorus};
use crate::util::{comb2, unit, wrap_tau};
use crate::{
    CurveJet, CurveProjection, Domain1, Domain2, Frame, GeomError, Interval, ParamAxis, ParametricCurve, ParametricSurface, Ray, RayHit, ImplicitSurface,
    Singularity, SurfaceJet, SurfaceProjection, Uv,
};
use core::f64::consts::{FRAC_PI_2, TAU};
use sm_core::{Point3, Vector3};
use smallvec::smallvec;

fn positive(v: f64, what: &'static str) -> Result<f64, GeomError> {
    if v.is_finite() && v > 0.0 { Ok(v) } else { Err(GeomError::Invalid(what)) }
}

fn unit_dir(f: &Frame, u: f64) -> (Vector3, Vector3) {
    let (s, c) = u.sin_cos();
    (comb2(c, f.x, s, f.y), comb2(-s, f.x, c, f.y))
}

// ---------------------------------------------------------------- curves

/// Infinite line, unit direction, parameter = arc length.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Line {
    /// Point at `t = 0`.
    pub origin: Point3,
    /// Unit direction.
    pub dir: Vector3,
}

impl Line {
    /// Build a line; the direction is normalized.
    pub fn new(origin: Point3, dir: Vector3) -> Result<Self, GeomError> {
        Ok(Self { origin, dir: unit(dir).ok_or(GeomError::Invalid("zero direction"))? })
    }
}

impl ParametricCurve for Line {
    fn domain(&self) -> Domain1 {
        Domain1::open(f64::NEG_INFINITY, f64::INFINITY)
    }
    fn jet(&self, t: f64) -> CurveJet {
        CurveJet { p: self.origin + self.dir * t, d1: self.dir, d2: Vector3::default() }
    }
    fn project(&self, p: Point3, _hint: Option<f64>) -> Result<CurveProjection, GeomError> {
        let t = (p - self.origin).dot(self.dir);
        let point = self.origin + self.dir * t;
        Ok(CurveProjection { t, point, distance: (p - point).norm() })
    }
    fn length(&self, range: Interval, _tol: f64) -> Result<f64, GeomError> {
        if range.is_valid() && range.is_bounded() { Ok(range.len()) } else { Err(GeomError::Invalid("arc length needs a bounded, ordered range")) }
    }
}

/// Circle in the `x`-`y` plane of its frame.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Circle {
    /// Placement; the circle lies in the `x`-`y` plane about the origin.
    pub frame: Frame,
    /// Radius.
    pub radius: f64,
}

impl Circle {
    /// Build a circle (radius must be positive and finite).
    pub fn new(frame: Frame, radius: f64) -> Result<Self, GeomError> {
        Ok(Self { frame, radius: positive(radius, "circle radius")? })
    }
}

impl ParametricCurve for Circle {
    fn domain(&self) -> Domain1 {
        Domain1::periodic(0.0, TAU)
    }
    fn jet(&self, t: f64) -> CurveJet {
        let (e, n) = unit_dir(&self.frame, t);
        CurveJet { p: self.frame.origin + e * self.radius, d1: n * self.radius, d2: e * -self.radius }
    }
    /// Closest point; for a query on the axis every point is equally close and `t = 0` is returned.
    fn project(&self, p: Point3, _hint: Option<f64>) -> Result<CurveProjection, GeomError> {
        let l = self.frame.to_local(p);
        let t = if l.x == 0.0 && l.y == 0.0 { 0.0 } else { wrap_tau(l.y.atan2(l.x)) };
        let rho = l.x.hypot(l.y);
        Ok(CurveProjection { t, point: self.eval(t), distance: (rho - self.radius).hypot(l.z) })
    }
    fn length(&self, range: Interval, _tol: f64) -> Result<f64, GeomError> {
        if range.is_valid() && range.is_bounded() { Ok(self.radius * range.len()) } else { Err(GeomError::Invalid("arc length needs a bounded, ordered range")) }
    }
}

/// Ellipse in the `x`-`y` plane of its frame, semi-axis `a` along `x`, `b` along `y`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Ellipse {
    /// Placement.
    pub frame: Frame,
    /// Semi-axis along `frame.x`.
    pub a: f64,
    /// Semi-axis along `frame.y`.
    pub b: f64,
}

impl Ellipse {
    /// Build an ellipse (both semi-axes positive and finite).
    pub fn new(frame: Frame, a: f64, b: f64) -> Result<Self, GeomError> {
        Ok(Self { frame, a: positive(a, "ellipse a")?, b: positive(b, "ellipse b")? })
    }
}

impl ParametricCurve for Ellipse {
    fn domain(&self) -> Domain1 {
        Domain1::periodic(0.0, TAU)
    }
    fn jet(&self, t: f64) -> CurveJet {
        let (s, c) = t.sin_cos();
        let f = &self.frame;
        CurveJet {
            p: f.origin + comb2(self.a * c, f.x, self.b * s, f.y),
            d1: comb2(-self.a * s, f.x, self.b * c, f.y),
            d2: comb2(-self.a * c, f.x, -self.b * s, f.y),
        }
    }
    /// Not provided (needs a quartic/Newton solve); `sm-geom` supplies a robust version.
    fn project(&self, _p: Point3, _hint: Option<f64>) -> Result<CurveProjection, GeomError> {
        Err(GeomError::Unsupported("ellipse projection"))
    }
}

// ---------------------------------------------------------------- surfaces

/// Infinite plane. Normal is `frame.z`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Plane {
    /// Placement; `(u, v)` are coordinates along `frame.x`, `frame.y`.
    pub frame: Frame,
}

impl ParametricSurface for Plane {
    fn domain(&self) -> Domain2 {
        Domain2::regular(Domain1::open(f64::NEG_INFINITY, f64::INFINITY), Domain1::open(f64::NEG_INFINITY, f64::INFINITY))
    }
    fn jet(&self, uv: Uv) -> SurfaceJet {
        let f = &self.frame;
        let z = Vector3::default();
        SurfaceJet { p: f.origin + comb2(uv.u, f.x, uv.v, f.y), du: f.x, dv: f.y, duu: z, duv: z, dvv: z }
    }
    fn normal(&self, _uv: Uv) -> Result<Vector3, GeomError> {
        Ok(self.frame.z)
    }
    fn project(&self, p: Point3, _hint: Option<Uv>) -> Result<SurfaceProjection, GeomError> {
        let l = self.frame.to_local(p);
        let uv = Uv::new(l.x, l.y);
        Ok(SurfaceProjection { uv, point: self.eval(uv), distance: l.z.abs() })
    }
    fn implicit(&self) -> Option<ImplicitDef> {
        Some(ImplicitDef::Quadric(Quadric::plane(&self.frame)))
    }
}

/// Infinite circular cylinder about `frame.z`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Cylinder {
    /// Placement; `u` is the angle from `frame.x` about `frame.z`, `v` the height along `z`.
    pub frame: Frame,
    /// Radius.
    pub radius: f64,
}

impl Cylinder {
    /// Build a cylinder (radius positive, finite).
    pub fn new(frame: Frame, radius: f64) -> Result<Self, GeomError> {
        Ok(Self { frame, radius: positive(radius, "cylinder radius")? })
    }
}

impl ParametricSurface for Cylinder {
    fn domain(&self) -> Domain2 {
        Domain2::regular(Domain1::periodic(0.0, TAU), Domain1::open(f64::NEG_INFINITY, f64::INFINITY))
    }
    fn jet(&self, uv: Uv) -> SurfaceJet {
        let f = &self.frame;
        let (e, n) = unit_dir(f, uv.u);
        let z = Vector3::default();
        SurfaceJet { p: f.origin + e * self.radius + f.z * uv.v, du: n * self.radius, dv: f.z, duu: e * -self.radius, duv: z, dvv: z }
    }
    fn normal(&self, uv: Uv) -> Result<Vector3, GeomError> {
        Ok(unit_dir(&self.frame, uv.u).0)
    }
    fn project(&self, p: Point3, _hint: Option<Uv>) -> Result<SurfaceProjection, GeomError> {
        let l = self.frame.to_local(p);
        let rho = l.x.hypot(l.y);
        let u = if rho == 0.0 { 0.0 } else { wrap_tau(l.y.atan2(l.x)) };
        let uv = Uv::new(u, l.z);
        Ok(SurfaceProjection { uv, point: self.eval(uv), distance: (rho - self.radius).abs() })
    }
    fn implicit(&self) -> Option<ImplicitDef> {
        Some(ImplicitDef::Quadric(Quadric::cylinder(&self.frame, self.radius)))
    }
}

/// One nappe of a circular cone about `frame.z`, widening toward `+z`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Cone {
    /// Placement; the circle of radius `radius` at `v = 0` is centred on `frame.origin`.
    pub frame: Frame,
    /// Radius at `v = 0` (may be 0: apex at the origin).
    pub radius: f64,
    /// Half-angle between axis and generator, in `(0, pi/2)`.
    pub half_angle: f64,
}

impl Cone {
    /// Build a cone.
    pub fn new(frame: Frame, radius: f64, half_angle: f64) -> Result<Self, GeomError> {
        if !(radius.is_finite() && radius >= 0.0) {
            return Err(GeomError::Invalid("cone radius"));
        }
        if !(half_angle > 0.0 && half_angle < FRAC_PI_2) {
            return Err(GeomError::Invalid("cone half-angle"));
        }
        Ok(Self { frame, radius, half_angle })
    }
    /// The `v` value of the apex.
    pub fn apex_v(&self) -> f64 {
        -self.radius / self.half_angle.sin()
    }
    /// The apex point.
    pub fn apex(&self) -> Point3 {
        self.frame.origin + self.frame.z * (self.apex_v() * self.half_angle.cos())
    }
}

impl ParametricSurface for Cone {
    fn domain(&self) -> Domain2 {
        let mut d = Domain2::regular(Domain1::periodic(0.0, TAU), Domain1::open(self.apex_v(), f64::INFINITY));
        d.singularities = smallvec![Singularity { along: ParamAxis::V, at: self.apex_v(), point: self.apex() }];
        d
    }
    fn jet(&self, uv: Uv) -> SurfaceJet {
        let f = &self.frame;
        let (s, c) = self.half_angle.sin_cos();
        let (e, n) = unit_dir(f, uv.u);
        let rho = self.radius + uv.v * s;
        SurfaceJet {
            p: f.origin + e * rho + f.z * (uv.v * c),
            du: n * rho,
            dv: comb2(s, e, c, f.z),
            duu: e * -rho,
            duv: n * s,
            dvv: Vector3::default(),
        }
    }
    /// Exact limit normal `cos a e_r - sin a z`, valid at the apex too.
    fn normal(&self, uv: Uv) -> Result<Vector3, GeomError> {
        let (s, c) = self.half_angle.sin_cos();
        Ok(comb2(c, unit_dir(&self.frame, uv.u).0, -s, self.frame.z))
    }
    fn project(&self, p: Point3, _hint: Option<Uv>) -> Result<SurfaceProjection, GeomError> {
        let (s, c) = self.half_angle.sin_cos();
        let l = self.frame.to_local(p);
        let rho = l.x.hypot(l.y);
        let u = if rho == 0.0 { 0.0 } else { wrap_tau(l.y.atan2(l.x)) };
        let va = self.apex_v();
        let h = l.z - va * c;
        let along = rho * s + h * c;
        if along <= 0.0 {
            let uv = Uv::new(u, va);
            return Ok(SurfaceProjection { uv, point: self.apex(), distance: (p - self.apex()).norm() });
        }
        let uv = Uv::new(u, va + along);
        Ok(SurfaceProjection { uv, point: self.eval(uv), distance: (rho * c - h * s).abs() })
    }
    fn implicit(&self) -> Option<ImplicitDef> {
        let apex_frame = Frame { origin: self.apex(), ..self.frame };
        Some(ImplicitDef::Quadric(Quadric::cone(&apex_frame, self.half_angle)))
    }
    /// Intersect the double-cone quadric, keeping only the nappe `v >= apex_v`.
    fn ray_hits(&self, ray: &Ray, range: Interval, out: &mut Vec<RayHit>) -> Result<(), GeomError> {
        let Some(ImplicitDef::Quadric(q)) = self.implicit() else { return Err(GeomError::Unsupported("cone")) };
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
            h.uv = self.project(h.point, None).ok().map(|p| p.uv);
        }
        Ok(())
    }
}

/// Sphere. `u` = longitude from `frame.x`, `v` = latitude from the `x`-`y` plane.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Sphere {
    /// Placement; the centre is `frame.origin`, poles lie on `+-z`.
    pub frame: Frame,
    /// Radius.
    pub radius: f64,
}

impl Sphere {
    /// Build a sphere (radius positive, finite).
    pub fn new(frame: Frame, radius: f64) -> Result<Self, GeomError> {
        Ok(Self { frame, radius: positive(radius, "sphere radius")? })
    }
}

impl ParametricSurface for Sphere {
    fn domain(&self) -> Domain2 {
        let mut d = Domain2::regular(Domain1::periodic(0.0, TAU), Domain1::open(-FRAC_PI_2, FRAC_PI_2));
        let f = &self.frame;
        d.singularities = smallvec![
            Singularity { along: ParamAxis::V, at: -FRAC_PI_2, point: f.origin + f.z * (-self.radius) },
            Singularity { along: ParamAxis::V, at: FRAC_PI_2, point: f.origin + f.z * self.radius },
        ];
        d
    }
    fn jet(&self, uv: Uv) -> SurfaceJet {
        let f = &self.frame;
        let (e, n) = unit_dir(f, uv.u);
        let (sv, cv) = uv.v.sin_cos();
        let r = self.radius;
        let radial = comb2(cv, e, sv, f.z);
        SurfaceJet {
            p: f.origin + radial * r,
            du: n * (r * cv),
            dv: comb2(-sv * r, e, cv * r, f.z),
            duu: e * (-r * cv),
            duv: n * (-r * sv),
            dvv: radial * -r,
        }
    }
    /// Exact radial normal, valid at the poles.
    fn normal(&self, uv: Uv) -> Result<Vector3, GeomError> {
        let (sv, cv) = uv.v.sin_cos();
        Ok(comb2(cv, unit_dir(&self.frame, uv.u).0, sv, self.frame.z))
    }
    /// Closest point; a query at the centre is equidistant from all points ([`GeomError::Singular`]).
    fn project(&self, p: Point3, _hint: Option<Uv>) -> Result<SurfaceProjection, GeomError> {
        let l = self.frame.to_local(p);
        let d = l.norm();
        if d == 0.0 {
            return Err(GeomError::Singular("projection from sphere centre"));
        }
        let rho = l.x.hypot(l.y);
        let u = if rho == 0.0 { 0.0 } else { wrap_tau(l.y.atan2(l.x)) };
        let uv = Uv::new(u, (l.z / d).clamp(-1.0, 1.0).asin());
        Ok(SurfaceProjection { uv, point: self.eval(uv), distance: (d - self.radius).abs() })
    }
    fn implicit(&self) -> Option<ImplicitDef> {
        Some(ImplicitDef::Quadric(Quadric::sphere(&self.frame, self.radius)))
    }
}

/// Torus about `frame.z` with `R > r > 0` (ring torus; spindle/horn tori are rejected).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Torus {
    /// Placement; the ring of radius `major` lies in the `x`-`y` plane.
    pub frame: Frame,
    /// Major radius `R`.
    pub major: f64,
    /// Minor (tube) radius `r`.
    pub minor: f64,
}

impl Torus {
    /// Build a ring torus.
    pub fn new(frame: Frame, major: f64, minor: f64) -> Result<Self, GeomError> {
        positive(minor, "torus minor radius")?;
        if !(major.is_finite() && major > minor) {
            return Err(GeomError::Invalid("torus requires major > minor"));
        }
        Ok(Self { frame, major, minor })
    }
}

impl ParametricSurface for Torus {
    fn domain(&self) -> Domain2 {
        Domain2::regular(Domain1::periodic(0.0, TAU), Domain1::periodic(0.0, TAU))
    }
    fn jet(&self, uv: Uv) -> SurfaceJet {
        let f = &self.frame;
        let (e, n) = unit_dir(f, uv.u);
        let (sv, cv) = uv.v.sin_cos();
        let (rr, r) = (self.major + self.minor * cv, self.minor);
        SurfaceJet {
            p: f.origin + e * rr + f.z * (r * sv),
            du: n * rr,
            dv: comb2(-r * sv, e, r * cv, f.z),
            duu: e * -rr,
            duv: n * (-r * sv),
            dvv: comb2(-r * cv, e, -r * sv, f.z),
        }
    }
    /// Exact outward normal `cos v e_r + sin v z`.
    fn normal(&self, uv: Uv) -> Result<Vector3, GeomError> {
        let (sv, cv) = uv.v.sin_cos();
        Ok(comb2(cv, unit_dir(&self.frame, uv.u).0, sv, self.frame.z))
    }
    fn project(&self, p: Point3, _hint: Option<Uv>) -> Result<SurfaceProjection, GeomError> {
        let l = self.frame.to_local(p);
        let rho = l.x.hypot(l.y);
        let u = if rho == 0.0 { 0.0 } else { wrap_tau(l.y.atan2(l.x)) };
        let (wr, wz) = (rho - self.major, l.z);
        let v = if wr == 0.0 && wz == 0.0 { 0.0 } else { wrap_tau(wz.atan2(wr)) };
        let uv = Uv::new(u, v);
        Ok(SurfaceProjection { uv, point: self.eval(uv), distance: (wr.hypot(wz) - self.minor).abs() })
    }
    fn implicit(&self) -> Option<ImplicitDef> {
        Some(ImplicitDef::Torus(ImplicitTorus { frame: self.frame, major: self.major, minor: self.minor }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::util::{Lcg, pos};

    fn world() -> Frame {
        Frame::at(Point3::new(0.0, 0.0, 0.0))
    }
    fn tilted() -> Frame {
        Frame::from_zx(Point3::new(0.3, -0.7, 1.1), Vector3::new(1.0, 2.0, -0.5), Vector3::new(0.0, 0.0, 1.0)).unwrap()
    }

    fn close(a: Vector3, b: Vector3, tol: f64) -> bool {
        (a - b).norm() <= tol
    }

    fn fd_curve(c: &dyn ParametricCurve, t: f64) {
        let h = 1e-6;
        let j = c.jet(t);
        let d1 = (c.eval(t + h) - c.eval(t - h)) * (0.5 / h);
        let d2 = (c.d1(t + h) - c.d1(t - h)) * (0.5 / h);
        assert!(close(j.d1, d1, 1e-6), "d1 {:?} vs {:?}", j.d1, d1);
        assert!(close(j.d2, d2, 1e-6), "d2 {:?} vs {:?}", j.d2, d2);
    }

    fn fd_surface(s: &dyn ParametricSurface, uv: Uv) {
        let h = 1e-6;
        let j = s.jet(uv);
        let at = |du: f64, dv: f64| s.jet(Uv::new(uv.u + du, uv.v + dv));
        let du = (s.eval(Uv::new(uv.u + h, uv.v)) - s.eval(Uv::new(uv.u - h, uv.v))) * (0.5 / h);
        let dv = (s.eval(Uv::new(uv.u, uv.v + h)) - s.eval(Uv::new(uv.u, uv.v - h))) * (0.5 / h);
        let duu = (at(h, 0.0).du - at(-h, 0.0).du) * (0.5 / h);
        let duv = (at(0.0, h).du - at(0.0, -h).du) * (0.5 / h);
        let dvv = (at(0.0, h).dv - at(0.0, -h).dv) * (0.5 / h);
        assert!(close(j.du, du, 1e-5) && close(j.dv, dv, 1e-5), "first derivatives");
        assert!(close(j.duu, duu, 1e-5) && close(j.duv, duv, 1e-5) && close(j.dvv, dvv, 1e-5), "second derivatives");
        // Analytic normal agrees with du x dv away from poles.
        if let Ok(n) = s.normal(uv) {
            let m = unit(j.du.cross(j.dv)).unwrap();
            assert!(close(n, m, 1e-9), "normal {n:?} vs {m:?}");
        }
    }

    #[test]
    fn curve_derivatives_match_finite_differences() {
        let cs: Vec<Box<dyn ParametricCurve>> = vec![
            Box::new(Line::new(Point3::new(1.0, 2.0, 3.0), Vector3::new(1.0, -1.0, 2.0)).unwrap()),
            Box::new(Circle::new(tilted(), 2.5).unwrap()),
            Box::new(Ellipse::new(tilted(), 3.0, 1.5).unwrap()),
        ];
        for c in &cs {
            for t in [0.1, 1.3, 4.0] {
                fd_curve(c.as_ref(), t);
            }
        }
    }

    #[test]
    fn surface_derivatives_match_finite_differences() {
        let ss: Vec<Box<dyn ParametricSurface>> = vec![
            Box::new(Plane { frame: tilted() }),
            Box::new(Cylinder::new(tilted(), 1.7).unwrap()),
            Box::new(Cone::new(tilted(), 1.0, 0.4).unwrap()),
            Box::new(Sphere::new(tilted(), 2.0).unwrap()),
            Box::new(Torus::new(tilted(), 3.0, 1.0).unwrap()),
        ];
        for s in &ss {
            for uv in [Uv::new(0.2, 0.3), Uv::new(2.5, 1.1), Uv::new(5.0, -0.4)] {
                fd_surface(s.as_ref(), uv);
            }
        }
    }

    #[test]
    fn circle_analytic_values() {
        let c = Circle::new(world(), 2.0).unwrap();
        assert!(close(pos(c.eval(core::f64::consts::FRAC_PI_2)), Vector3::new(0.0, 2.0, 0.0), 1e-14));
        assert!((c.length(Interval::new(0.0, TAU), 1e-12).unwrap() - 4.0 * core::f64::consts::PI).abs() < 1e-12);
        assert!(c.is_closed(1e-9));
        let pr = c.project(Point3::new(3.0, 4.0, 0.0), None).unwrap();
        assert!((pr.distance - 3.0).abs() < 1e-12); // |(3,4)| = 5, radius 2
        assert!(close(pos(pr.point), Vector3::new(1.2, 1.6, 0.0), 1e-12));
    }

    #[test]
    fn ellipse_length_gauss_matches_series() {
        // Perimeter of a=2,b=1 via Ramanujan II is 9.688448220547672 (accurate to ~1e-9).
        let e = Ellipse::new(world(), 2.0, 1.0).unwrap();
        assert!((e.length(Interval::new(0.0, TAU), 1e-9).unwrap() - 9.688_448_220_547_672).abs() < 1e-8);
        assert!(e.length(Interval::new(1.0, 0.0), 1e-9).is_err() && e.length(Interval::REAL, 1e-9).is_err());
        assert!(e.project(Point3::default(), None).is_err());
    }

    #[test]
    fn cylinder_seam_and_values() {
        let cy = Cylinder::new(world(), 2.0).unwrap();
        let d = cy.domain();
        assert_eq!(d.u.period, Some(TAU));
        assert!(d.v.period.is_none() && d.singularities.is_empty());
        // The seam: u = 0 and u = 2pi give the same point.
        let a = cy.eval(Uv::new(0.0, 1.0));
        let b = cy.eval(Uv::new(TAU, 1.0));
        assert!((a - b).norm() < 1e-14);
        assert!(close(cy.normal(Uv::new(0.0, 0.0)).unwrap(), Vector3::new(1.0, 0.0, 0.0), 1e-15));
        let pr = cy.project(Point3::new(0.0, 5.0, 3.0), None).unwrap();
        assert!((pr.distance - 3.0).abs() < 1e-12);
        assert!((pr.uv.u - core::f64::consts::FRAC_PI_2).abs() < 1e-12 && (pr.uv.v - 3.0).abs() < 1e-12);
    }

    #[test]
    fn sphere_poles_and_projection() {
        let s = Sphere::new(world(), 2.0).unwrap();
        let d = s.domain();
        assert_eq!(d.singularities.len(), 2);
        assert!(close(pos(d.singularities[1].point), Vector3::new(0.0, 0.0, 2.0), 1e-14));
        // du vanishes at the pole, yet the analytic normal is exact.
        let pole = Uv::new(1.0, FRAC_PI_2);
        assert!(s.jet(pole).du.norm() < 1e-12);
        assert!(close(s.normal(pole).unwrap(), Vector3::new(0.0, 0.0, 1.0), 1e-12));
        let pr = s.project(Point3::new(0.0, 0.0, 5.0), None).unwrap();
        assert!((pr.distance - 3.0).abs() < 1e-12 && (pr.uv.v - FRAC_PI_2).abs() < 1e-12);
        assert!(s.project(Point3::default(), None).is_err());
        // Default normal() reports the singularity.
        struct NoNormal(Sphere);
        impl core::fmt::Debug for NoNormal {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                f.write_str("NoNormal")
            }
        }
        impl ParametricSurface for NoNormal {
            fn domain(&self) -> Domain2 {
                self.0.domain()
            }
            fn jet(&self, uv: Uv) -> SurfaceJet {
                self.0.jet(uv)
            }
            fn project(&self, p: Point3, h: Option<Uv>) -> Result<SurfaceProjection, GeomError> {
                self.0.project(p, h)
            }
        }
        assert!(matches!(NoNormal(s).normal(pole), Err(GeomError::Singular(_))));
        assert!(matches!(NoNormal(s).ray_hits(&Ray { origin: Point3::default(), dir: Vector3::new(1.0, 0.0, 0.0) }, Interval::new(0.0, 1.0), &mut Vec::new()), Err(GeomError::Unsupported(_))));
    }

    #[test]
    fn cone_apex_and_projection() {
        let c = Cone::new(world(), 1.0, 0.5).unwrap();
        let apex = c.apex();
        let d = c.domain();
        assert_eq!(d.singularities.len(), 1);
        assert!(close(pos(c.eval(Uv::new(0.3, c.apex_v()))), pos(apex), 1e-12));
        // Point on the surface projects to itself.
        let q = c.eval(Uv::new(1.0, 0.7));
        let pr = c.project(q, None).unwrap();
        assert!(pr.distance < 1e-12 && (pr.uv.v - 0.7).abs() < 1e-12);
        // Point behind the apex projects to the apex.
        let behind = apex + c.frame.z * (-2.0);
        assert!(close(pos(c.project(behind, None).unwrap().point), pos(apex), 1e-12));
    }

    #[test]
    fn torus_analytic_projection_and_validation() {
        let t = Torus::new(world(), 3.0, 1.0).unwrap();
        let pr = t.project(Point3::new(5.0, 0.0, 0.0), None).unwrap();
        assert!((pr.distance - 1.0).abs() < 1e-12 && close(pos(pr.point), Vector3::new(4.0, 0.0, 0.0), 1e-12));
        let top = t.project(Point3::new(3.0, 0.0, 2.5), None).unwrap();
        assert!((top.distance - 1.5).abs() < 1e-12);
        assert!(Torus::new(world(), 1.0, 2.0).is_err());
        assert!(Cone::new(world(), 1.0, 2.0).is_err());
    }

    /// Projection is the true minimum: compare with dense sampling and check the residual
    /// is normal to the surface.
    #[test]
    fn projections_are_minimal_and_orthogonal() {
        let ss: Vec<Box<dyn ParametricSurface>> = vec![
            Box::new(Cylinder::new(tilted(), 1.7).unwrap()),
            Box::new(Sphere::new(tilted(), 2.0).unwrap()),
            Box::new(Torus::new(tilted(), 3.0, 1.0).unwrap()),
        ];
        let mut rng = Lcg(7);
        for s in &ss {
            for _ in 0..8 {
                let p = Point3::new(rng.next() * 12.0 - 6.0, rng.next() * 12.0 - 6.0, rng.next() * 12.0 - 6.0);
                let pr = s.project(p, None).unwrap();
                let n = s.normal(pr.uv).unwrap();
                let r = p - pr.point;
                if r.norm() > 1e-9 {
                    assert!(n.cross(r).norm() <= 1e-8 * r.norm().max(1.0), "residual not along normal");
                }
                // Dense sampling must not beat it.
                let dom = s.domain();
                for i in 0..48 {
                    for j in 0..48 {
                        let u = dom.u.range.lo + TAU * i as f64 / 48.0;
                        let v = if dom.v.is_periodic() { TAU * j as f64 / 48.0 } else if dom.v.range.is_bounded() { dom.v.range.lo + dom.v.range.len() * j as f64 / 47.0 } else { -8.0 + 16.0 * j as f64 / 47.0 };
                        assert!((p - s.eval(Uv::new(u, v))).norm() >= pr.distance - 1e-9);
                    }
                }
            }
        }
    }

    #[test]
    fn ray_hooks_on_analytic_surfaces() {
        let ray = Ray { origin: Point3::new(-10.0, 0.0, 0.0), dir: Vector3::new(1.0, 0.0, 0.0) };
        let range = Interval::new(0.0, f64::INFINITY);
        let mut out = Vec::new();
        Sphere::new(world(), 2.0).unwrap().ray_hits(&ray, range, &mut out).unwrap();
        assert_eq!(out.iter().map(|h| h.t).collect::<Vec<_>>(), vec![8.0, 12.0]);
        out.clear();
        Cylinder::new(world(), 1.5).unwrap().ray_hits(&ray, range, &mut out).unwrap();
        assert!((out[0].t - 8.5).abs() < 1e-12 && (out[1].t - 11.5).abs() < 1e-12);
        out.clear();
        Torus::new(world(), 3.0, 1.0).unwrap().ray_hits(&ray, range, &mut out).unwrap();
        assert_eq!(out.len(), 4);
        out.clear();
        Plane { frame: Frame::at(Point3::new(2.0, 0.0, 0.0)) }.ray_hits(&ray, range, &mut out).unwrap();
        assert!(out.is_empty(), "ray parallel to plane z=0 in-plane origin: plane z-normal, ray in plane");
        // Cone: only the +z nappe is kept. Axis ray along z through the apex region.
        let cone = Cone::new(world(), 1.0, core::f64::consts::FRAC_PI_4).unwrap();
        out.clear();
        let across = Ray { origin: Point3::new(-5.0, 0.0, 3.0), dir: Vector3::new(1.0, 0.0, 0.0) };
        cone.ray_hits(&across, range, &mut out).unwrap();
        // radius at z=3 is 1+3*tan(pi/4)... generator: rho = r0 + v sin a, z = v cos a => rho = 1 + z.
        assert_eq!(out.len(), 2);
        assert!((out[0].point.x + 4.0).abs() < 1e-9 && (out[1].point.x - 4.0).abs() < 1e-9);
        // The same x-ray at z = -3 would hit the lower nappe of the double cone only.
        out.clear();
        let below = Ray { origin: Point3::new(-5.0, 0.0, -3.0), dir: Vector3::new(1.0, 0.0, 0.0) };
        cone.ray_hits(&below, range, &mut out).unwrap();
        assert!(out.is_empty());
    }

    #[test]
    fn implicit_and_parametric_agree() {
        let ss: Vec<Box<dyn ParametricSurface>> = vec![
            Box::new(Plane { frame: tilted() }),
            Box::new(Cylinder::new(tilted(), 1.7).unwrap()),
            Box::new(Cone::new(tilted(), 1.0, 0.4).unwrap()),
            Box::new(Sphere::new(tilted(), 2.0).unwrap()),
            Box::new(Torus::new(tilted(), 3.0, 1.0).unwrap()),
        ];
        for s in &ss {
            let f = s.implicit().unwrap();
            for uv in [Uv::new(0.2, 0.3), Uv::new(2.5, 1.1), Uv::new(5.0, 0.9)] {
                let p = s.eval(uv);
                let scale = 1.0 + f.gradient(p).norm();
                assert!(f.value(p).abs() < 1e-9 * scale * scale, "on-surface value {}", f.value(p));
                let g = unit(f.gradient(p)).unwrap();
                assert!(close(g, s.normal(uv).unwrap(), 1e-8), "gradient vs normal");
            }
        }
    }
}
