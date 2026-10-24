//! Analytic geometry fixtures with closed-form expectations.
//!
//! The shapes are the `sm-geom-api` implementors themselves (no duplicated types). This module
//! adds, through the extension traits [`SurfaceExpect`] and [`CurveExpect`], values that are
//! derived from closed formulas and *not* from the implementor's own `jet`/`project`, so they can
//! serve as oracles: unit normal, Gaussian and mean curvature, area of a parameter rectangle,
//! closest point, curve curvature and length.
//!
//! **Curvature sign convention.** `H = (k1 + k2) / 2` is positive when the surface bends away
//! from its normal `n = du x dv` (a sphere with outward normal has `H = +1/r`). `K = k1 k2`.

use core::f64::consts::{PI, TAU};
use sm_core::{Point3, Vector3};
use sm_geom_api::{
    Circle, Cone, Cylinder, Ellipse, Frame, Interval, Line, ParametricCurve, ParametricSurface, Plane, Sphere,
    SurfaceJet, Torus, Uv,
};

/// Dot product helper.
fn dot(a: Vector3, b: Vector3) -> f64 {
    a.dot(b)
}

/// A frame at the origin with the world axes.
pub fn world_frame() -> Frame {
    Frame::at(Point3::new(0.0, 0.0, 0.0))
}

/// A deterministic, arbitrarily placed and tilted frame (axis `(1, 2, -0.5)`), used to check that
/// no formula silently assumes the world frame.
pub fn tilted_frame() -> Frame {
    // The inputs are constants with a non-zero axis and a hint not parallel to it.
    Frame::from_zx(Point3::new(0.3, -0.7, 1.1), Vector3::new(1.0, 2.0, -0.5), Vector3::new(0.0, 0.0, 1.0))
        .unwrap_or_else(|_| Frame::at(Point3::new(0.3, -0.7, 1.1)))
}

/// Gaussian and mean curvature `(K, H)` of a surface point from its jet, using the first and
/// second fundamental forms. The normal is `n`, which must be the unit normal of the surface at
/// that point (so the sign convention of the module applies). Returns `None` at a degenerate
/// tangent plane.
pub fn curvatures_from_jet(j: &SurfaceJet, n: Vector3) -> Option<(f64, f64)> {
    let (e, f, g) = (dot(j.du, j.du), dot(j.du, j.dv), dot(j.dv, j.dv));
    let (l, m, nn) = (dot(j.duu, n), dot(j.duv, n), dot(j.dvv, n));
    let det = e * g - f * f;
    if det.abs() <= 1e-300 {
        return None;
    }
    let k = (l * nn - m * m) / det;
    // Standard mean curvature is positive toward the normal; the module convention is opposite.
    let h = -(e * nn - 2.0 * f * m + g * l) / (2.0 * det);
    Some((k, h))
}

/// Closed-form expectations for a parametric surface.
pub trait SurfaceExpect: ParametricSurface {
    /// A short name for failure messages.
    fn label(&self) -> &'static str;
    /// Exact unit normal `du x dv` at `uv`.
    fn exact_normal(&self, uv: Uv) -> Vector3;
    /// Exact Gaussian curvature `K` at `uv`.
    fn gaussian_curvature(&self, uv: Uv) -> f64;
    /// Exact mean curvature `H` at `uv` (module sign convention).
    fn mean_curvature(&self, uv: Uv) -> f64;
    /// Exact area of the parameter rectangle `u x v` (both intervals inside the domain).
    fn area(&self, u: Interval, v: Interval) -> f64;
    /// Exact closest point on the (full) surface to `p`, or `None` where it is not unique or the
    /// closed form does not apply (a point on the axis or at the centre).
    fn closest_point(&self, p: Point3) -> Option<Point3>;
    /// Parameter points away from every singularity, for pointwise checks.
    fn sample_uvs(&self) -> Vec<Uv>;
}

fn lerp_samples(a: f64, b: f64, n: usize) -> impl Iterator<Item = f64> {
    (0..n).map(move |i| a + (b - a) * (i as f64 + 0.5) / n as f64)
}

fn grid(u: (f64, f64), v: (f64, f64), n: usize) -> Vec<Uv> {
    let mut out = Vec::new();
    for a in lerp_samples(u.0, u.1, n) {
        for b in lerp_samples(v.0, v.1, n) {
            out.push(Uv::new(a, b));
        }
    }
    out
}

fn radial_unit(f: &Frame, u: f64) -> Vector3 {
    let (s, c) = u.sin_cos();
    f.x * c + f.y * s
}

impl SurfaceExpect for Plane {
    fn label(&self) -> &'static str {
        "plane"
    }
    fn exact_normal(&self, _uv: Uv) -> Vector3 {
        self.frame.z
    }
    fn gaussian_curvature(&self, _uv: Uv) -> f64 {
        0.0
    }
    fn mean_curvature(&self, _uv: Uv) -> f64 {
        0.0
    }
    fn area(&self, u: Interval, v: Interval) -> f64 {
        u.len() * v.len()
    }
    fn closest_point(&self, p: Point3) -> Option<Point3> {
        let d = dot(p - self.frame.origin, self.frame.z);
        Some(p + self.frame.z * (-d))
    }
    fn sample_uvs(&self) -> Vec<Uv> {
        grid((-2.0, 3.0), (-1.0, 4.0), 4)
    }
}

impl SurfaceExpect for Cylinder {
    fn label(&self) -> &'static str {
        "cylinder"
    }
    fn exact_normal(&self, uv: Uv) -> Vector3 {
        radial_unit(&self.frame, uv.u)
    }
    fn gaussian_curvature(&self, _uv: Uv) -> f64 {
        0.0
    }
    fn mean_curvature(&self, _uv: Uv) -> f64 {
        0.5 / self.radius
    }
    fn area(&self, u: Interval, v: Interval) -> f64 {
        self.radius * u.len() * v.len()
    }
    fn closest_point(&self, p: Point3) -> Option<Point3> {
        let l = self.frame.to_local(p);
        let rho = l.x.hypot(l.y);
        if rho == 0.0 {
            return None;
        }
        let k = self.radius / rho;
        Some(self.frame.from_local(Vector3::new(l.x * k, l.y * k, l.z)))
    }
    fn sample_uvs(&self) -> Vec<Uv> {
        grid((0.1, TAU - 0.1), (-1.0, 2.0), 5)
    }
}

impl SurfaceExpect for Cone {
    fn label(&self) -> &'static str {
        "cone"
    }
    fn exact_normal(&self, uv: Uv) -> Vector3 {
        let (s, c) = self.half_angle.sin_cos();
        radial_unit(&self.frame, uv.u) * c + self.frame.z * (-s)
    }
    fn gaussian_curvature(&self, _uv: Uv) -> f64 {
        0.0
    }
    fn mean_curvature(&self, uv: Uv) -> f64 {
        let rho = self.radius + uv.v * self.half_angle.sin();
        0.5 * self.half_angle.cos() / rho
    }
    fn area(&self, u: Interval, v: Interval) -> f64 {
        // |du x dv| = rho = radius + v sin(a) since |dv| = 1 and du is orthogonal to dv.
        let s = self.half_angle.sin();
        u.len() * (self.radius * v.len() + 0.5 * s * (v.hi * v.hi - v.lo * v.lo))
    }
    fn closest_point(&self, p: Point3) -> Option<Point3> {
        let (s, c) = self.half_angle.sin_cos();
        let l = self.frame.to_local(p);
        let rho = l.x.hypot(l.y);
        if rho == 0.0 {
            return None;
        }
        // Work in the half-plane (rho, z): the nappe is the ray from the apex along (s, c).
        let za = self.apex_v() * c;
        let t = rho * s + (l.z - za) * c;
        let (qr, qz) = if t <= 0.0 { (0.0, za) } else { (t * s, za + t * c) };
        Some(self.frame.from_local(Vector3::new(l.x / rho * qr, l.y / rho * qr, qz)))
    }
    fn sample_uvs(&self) -> Vec<Uv> {
        grid((0.1, TAU - 0.1), (0.5, 2.5), 5)
    }
}

impl SurfaceExpect for Sphere {
    fn label(&self) -> &'static str {
        "sphere"
    }
    fn exact_normal(&self, uv: Uv) -> Vector3 {
        let (sv, cv) = uv.v.sin_cos();
        radial_unit(&self.frame, uv.u) * cv + self.frame.z * sv
    }
    fn gaussian_curvature(&self, _uv: Uv) -> f64 {
        1.0 / (self.radius * self.radius)
    }
    fn mean_curvature(&self, _uv: Uv) -> f64 {
        1.0 / self.radius
    }
    fn area(&self, u: Interval, v: Interval) -> f64 {
        self.radius * self.radius * u.len() * (v.hi.sin() - v.lo.sin())
    }
    fn closest_point(&self, p: Point3) -> Option<Point3> {
        let l = self.frame.to_local(p);
        let d = l.norm();
        if d == 0.0 {
            return None;
        }
        let k = self.radius / d;
        Some(self.frame.from_local(Vector3::new(l.x * k, l.y * k, l.z * k)))
    }
    fn sample_uvs(&self) -> Vec<Uv> {
        grid((0.1, TAU - 0.1), (-1.4, 1.4), 5)
    }
}

impl SurfaceExpect for Torus {
    fn label(&self) -> &'static str {
        "torus"
    }
    fn exact_normal(&self, uv: Uv) -> Vector3 {
        let (sv, cv) = uv.v.sin_cos();
        radial_unit(&self.frame, uv.u) * cv + self.frame.z * sv
    }
    fn gaussian_curvature(&self, uv: Uv) -> f64 {
        let cv = uv.v.cos();
        cv / (self.minor * (self.major + self.minor * cv))
    }
    fn mean_curvature(&self, uv: Uv) -> f64 {
        let cv = uv.v.cos();
        (self.major + 2.0 * self.minor * cv) / (2.0 * self.minor * (self.major + self.minor * cv))
    }
    fn area(&self, u: Interval, v: Interval) -> f64 {
        self.minor * u.len() * (self.major * v.len() + self.minor * (v.hi.sin() - v.lo.sin()))
    }
    fn closest_point(&self, p: Point3) -> Option<Point3> {
        let l = self.frame.to_local(p);
        let rho = l.x.hypot(l.y);
        if rho == 0.0 {
            return None;
        }
        // Nearest point of the core circle, then `minor` toward p from it.
        let core = Vector3::new(l.x / rho * self.major, l.y / rho * self.major, 0.0);
        let d = Vector3::new(l.x - core.x, l.y - core.y, l.z);
        let dn = d.norm();
        if dn == 0.0 {
            return None;
        }
        let k = self.minor / dn;
        Some(self.frame.from_local(Vector3::new(core.x + d.x * k, core.y + d.y * k, d.z * k)))
    }
    fn sample_uvs(&self) -> Vec<Uv> {
        grid((0.1, TAU - 0.1), (0.1, TAU - 0.1), 5)
    }
}

/// Closed-form expectations for a parametric curve.
pub trait CurveExpect: ParametricCurve {
    /// A short name for failure messages.
    fn label(&self) -> &'static str;
    /// Exact unit tangent at `t`.
    fn exact_tangent(&self, t: f64) -> Vector3;
    /// Exact curvature `|r'' |` for arc length, i.e. `1/radius of curvature`, at `t`.
    fn curvature(&self, t: f64) -> f64;
    /// Exact arc length over `range`, where a closed form exists (`None` otherwise).
    fn arc_length(&self, range: Interval) -> Option<f64>;
    /// Exact closest point on the whole curve, or `None` if not unique or not available.
    fn closest_point(&self, p: Point3) -> Option<Point3>;
    /// Parameters for pointwise checks.
    fn sample_ts(&self) -> Vec<f64>;
}

impl CurveExpect for Line {
    fn label(&self) -> &'static str {
        "line"
    }
    fn exact_tangent(&self, _t: f64) -> Vector3 {
        self.dir
    }
    fn curvature(&self, _t: f64) -> f64 {
        0.0
    }
    fn arc_length(&self, range: Interval) -> Option<f64> {
        Some(range.len())
    }
    fn closest_point(&self, p: Point3) -> Option<Point3> {
        let t = dot(p - self.origin, self.dir);
        Some(self.origin + self.dir * t)
    }
    fn sample_ts(&self) -> Vec<f64> {
        lerp_samples(-2.0, 3.0, 6).collect()
    }
}

impl CurveExpect for Circle {
    fn label(&self) -> &'static str {
        "circle"
    }
    fn exact_tangent(&self, t: f64) -> Vector3 {
        let (s, c) = t.sin_cos();
        self.frame.x * (-s) + self.frame.y * c
    }
    fn curvature(&self, _t: f64) -> f64 {
        1.0 / self.radius
    }
    fn arc_length(&self, range: Interval) -> Option<f64> {
        Some(self.radius * range.len())
    }
    fn closest_point(&self, p: Point3) -> Option<Point3> {
        let l = self.frame.to_local(p);
        let rho = l.x.hypot(l.y);
        if rho == 0.0 {
            return None;
        }
        let k = self.radius / rho;
        Some(self.frame.from_local(Vector3::new(l.x * k, l.y * k, 0.0)))
    }
    fn sample_ts(&self) -> Vec<f64> {
        lerp_samples(0.0, TAU, 8).collect()
    }
}

/// Complete perimeter of an ellipse with semi-axes `a, b > 0`, to machine precision, from the
/// arithmetic-geometric mean: `L = 4 a E(m)`, `m = 1 - b^2/a^2` (axes swapped so `a >= b`).
/// Degenerate inputs are defined, not left to the series: one zero semi-axis gives the doubled
/// segment `4 max(a, b)` (both zero: 0); a negative or non-finite semi-axis gives NaN.
pub fn ellipse_perimeter(a: f64, b: f64) -> f64 {
    if !(a.is_finite() && b.is_finite()) || a < 0.0 || b < 0.0 {
        return f64::NAN;
    }
    let (a, b) = if a >= b { (a, b) } else { (b, a) };
    if b == 0.0 {
        return 4.0 * a;
    }
    let m = 1.0 - (b / a) * (b / a);
    let (mut an, mut bn) = (1.0_f64, (1.0 - m).sqrt());
    let mut cn2 = m; // c_0^2
    let mut sum = 0.5 * cn2; // 2^(n-1) c_n^2 with n = 0
    let mut w = 0.5;
    for _ in 0..64 {
        let next_a = 0.5 * (an + bn);
        let c = 0.5 * (an - bn);
        bn = (an * bn).sqrt();
        an = next_a;
        cn2 = c * c;
        w *= 2.0;
        sum += w * cn2;
        if cn2 < 1e-34 {
            break;
        }
    }
    let k = PI / (2.0 * an);
    4.0 * a * k * (1.0 - sum)
}

impl CurveExpect for Ellipse {
    fn label(&self) -> &'static str {
        "ellipse"
    }
    fn exact_tangent(&self, t: f64) -> Vector3 {
        let (s, c) = t.sin_cos();
        let d = self.frame.x * (-self.a * s) + self.frame.y * (self.b * c);
        d * (1.0 / d.norm())
    }
    fn curvature(&self, t: f64) -> f64 {
        let (s, c) = t.sin_cos();
        self.a * self.b / (self.a * self.a * s * s + self.b * self.b * c * c).powf(1.5)
    }
    /// Only the full period `[t0, t0 + 2 pi]` has a closed form (complete elliptic integral);
    /// partial arcs return `None`.
    fn arc_length(&self, range: Interval) -> Option<f64> {
        ((range.len() - TAU).abs() < 1e-12).then(|| ellipse_perimeter(self.a, self.b))
    }
    /// Not available: the closest point on an ellipse is a quartic root (and `Ellipse::project`
    /// is `Unsupported` in the contract).
    fn closest_point(&self, _p: Point3) -> Option<Point3> {
        None
    }
    fn sample_ts(&self) -> Vec<f64> {
        lerp_samples(0.0, TAU, 8).collect()
    }
}

/// The surface fixtures, one per analytic kind, each in the world frame and in a tilted frame.
///
/// | name | shape |
/// |---|---|
/// | plane, plane (tilted) | `z = 0`, and a plane with normal `(1, 2, -0.5)` |
/// | cylinder `r = 1.5` | about `z` / tilted |
/// | cone `r0 = 0.5`, half-angle 0.4 | single nappe |
/// | sphere `r = 2` | poles on `+-z` |
/// | ring torus `R = 3, r = 1` | |
pub fn surface_fixtures() -> Vec<Box<dyn SurfaceExpect>> {
    let mut out: Vec<Box<dyn SurfaceExpect>> = Vec::new();
    for fr in [world_frame(), tilted_frame()] {
        out.push(Box::new(Plane { frame: fr }));
        if let Ok(s) = Cylinder::new(fr, 1.5) {
            out.push(Box::new(s));
        }
        if let Ok(s) = Cone::new(fr, 0.5, 0.4) {
            out.push(Box::new(s));
        }
        if let Ok(s) = Sphere::new(fr, 2.0) {
            out.push(Box::new(s));
        }
        if let Ok(s) = Torus::new(fr, 3.0, 1.0) {
            out.push(Box::new(s));
        }
    }
    out
}

/// The curve fixtures: lines, circles and ellipses, world and tilted frames.
pub fn curve_fixtures() -> Vec<Box<dyn CurveExpect>> {
    let mut out: Vec<Box<dyn CurveExpect>> = Vec::new();
    for fr in [world_frame(), tilted_frame()] {
        if let Ok(l) = Line::new(fr.origin, Vector3::new(1.0, -2.0, 0.5)) {
            out.push(Box::new(l));
        }
        if let Ok(c) = Circle::new(fr, 1.75) {
            out.push(Box::new(c));
        }
        if let Ok(e) = Ellipse::new(fr, 2.0, 1.0) {
            out.push(Box::new(e));
        }
    }
    out
}

/// Surface area of a sphere, `4 pi r^2`.
pub fn sphere_area(r: f64) -> f64 {
    4.0 * PI * r * r
}

/// Surface area of a ring torus, `4 pi^2 R r`.
pub fn torus_area(major: f64, minor: f64) -> f64 {
    4.0 * PI * PI * major * minor
}

/// Lateral area of a cylinder, `2 pi r h`.
pub fn cylinder_lateral_area(r: f64, h: f64) -> f64 {
    TAU * r * h
}

