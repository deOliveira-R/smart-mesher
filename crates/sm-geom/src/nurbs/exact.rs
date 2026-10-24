//! Exact rational NURBS forms of the analytic kinds (Piegl and Tiller ch. 7 and 8).
//!
//! The point sets are exact (to rounding); the parametrization of a rational quadratic arc is
//! *not* the angle, except at the knots, which are placed at the arc ends so `t` equals the angle
//! there. Compare shapes through the implicit form, projection or the knot parameters.

use core::f64::consts::TAU;
use sm_core::Point3;
use sm_geom_api::{Circle, Cone, Cylinder, Ellipse, Frame, GeomError, Line, NurbsCurve, NurbsSurface, Sphere, Torus};

/// Poles of a unit-circle arc from angle `a0` to `a1` (`0 < a1 - a0 <= 2 pi`): local `(x, y)`,
/// weight, and the clamped knot vector with the knot values equal to the angles at arc ends.
/// Local `(x, y, weight)` poles of a unit-circle arc and its knot vector.
type ArcPoles = (Vec<(f64, f64, f64)>, Vec<f64>);

fn unit_arc(a0: f64, a1: f64) -> Result<ArcPoles, GeomError> {
    let theta = a1 - a0;
    if !(theta > 0.0 && theta <= TAU + 1e-12) {
        return Err(GeomError::Invalid("arc angle must be in (0, 2 pi]"));
    }
    let narcs = ((theta / core::f64::consts::FRAC_PI_2).ceil() as usize).max(1);
    let dtheta = theta / narcs as f64;
    let w1 = (dtheta / 2.0).cos();
    let mut pts = Vec::with_capacity(2 * narcs + 1);
    let (s0, c0) = a0.sin_cos();
    // Tangent intersection distance factor: 1 / cos(dtheta / 2).
    let k = 1.0 / w1;
    pts.push((c0, s0, 1.0));
    let mut ang = a0;
    for _ in 0..narcs {
        let mid = ang + 0.5 * dtheta;
        let (sm, cm) = mid.sin_cos();
        ang += dtheta;
        let (s2, c2) = ang.sin_cos();
        pts.push((cm * k, sm * k, w1));
        pts.push((c2, s2, 1.0));
    }
    let mut knots = vec![a0; 3];
    for i in 1..narcs {
        let t = a0 + dtheta * i as f64;
        knots.extend([t, t]);
    }
    knots.extend([a1; 3]);
    Ok((pts, knots))
}

/// Exact rational NURBS of the circular arc of `circle` from angle `a0` to `a1`
/// (`a1 - a0 <= 2 pi`). Knots equal the angles at the arc ends and joins.
pub fn circle_arc_to_nurbs(circle: &Circle, a0: f64, a1: f64) -> Result<NurbsCurve, GeomError> {
    let (pts, knots) = unit_arc(a0, a1)?;
    let f = &circle.frame;
    let control = pts.iter().map(|(x, y, _)| f.origin + f.x * (x * circle.radius) + f.y * (y * circle.radius)).collect();
    let weights = pts.iter().map(|p| p.2).collect();
    NurbsCurve::new(2, knots, control, Some(weights))
}

/// The full circle as a degree-2 rational NURBS with 9 poles (knots at the quarter angles).
pub fn circle_to_nurbs(circle: &Circle) -> Result<NurbsCurve, GeomError> {
    circle_arc_to_nurbs(circle, 0.0, TAU)
}

/// The ellipse arc `a0..a1` (eccentric angle) as a rational NURBS (affine image of the circle).
pub fn ellipse_arc_to_nurbs(e: &Ellipse, a0: f64, a1: f64) -> Result<NurbsCurve, GeomError> {
    let (pts, knots) = unit_arc(a0, a1)?;
    let f = &e.frame;
    let control = pts.iter().map(|(x, y, _)| f.origin + f.x * (x * e.a) + f.y * (y * e.b)).collect();
    let weights = pts.iter().map(|p| p.2).collect();
    NurbsCurve::new(2, knots, control, Some(weights))
}

/// The full ellipse as a rational NURBS.
pub fn ellipse_to_nurbs(e: &Ellipse) -> Result<NurbsCurve, GeomError> {
    ellipse_arc_to_nurbs(e, 0.0, TAU)
}

/// A line segment `t in [t0, t1]` as a degree-1 NURBS (parameter equals the line's).
pub fn line_to_nurbs(l: &Line, t0: f64, t1: f64) -> Result<NurbsCurve, GeomError> {
    if !(t1 > t0 && t0.is_finite() && t1.is_finite()) {
        return Err(GeomError::Invalid("line segment needs a finite increasing range"));
    }
    NurbsCurve::new(1, vec![t0, t0, t1, t1], vec![l.origin + l.dir * t0, l.origin + l.dir * t1], None)
}

/// A profile `(rho, z, weight)` with its knots: the generating curve of a surface of revolution.
struct Profile {
    degree: usize,
    knots: Vec<f64>,
    poles: Vec<(f64, f64, f64)>,
}

/// Surface of revolution of `profile` (in the `rho`-`z` half plane of `frame`) about `frame.z`,
/// sweeping the angle `a0..a1` (P&T A8.1). `u` is the sweep, `v` the profile parameter.
fn revolve(frame: &Frame, profile: &Profile, a0: f64, a1: f64) -> Result<NurbsSurface, GeomError> {
    let (circ, ku) = unit_arc(a0, a1)?;
    let (nu, nv) = (circ.len(), profile.poles.len());
    let mut control = Vec::with_capacity(nu * nv);
    let mut weights = Vec::with_capacity(nu * nv);
    for (cx, cy, cw) in &circ {
        for (rho, z, pw) in &profile.poles {
            // The circle pole at a tangent corner already carries the 1/cos factor; the
            // profile radius scales it.
            control.push(frame.origin + frame.x * (cx * rho) + frame.y * (cy * rho) + frame.z * *z);
            weights.push(cw * pw);
        }
    }
    NurbsSurface::new(2, profile.degree, ku, profile.knots.clone(), nu, nv, control, Some(weights))
}

fn line_profile(p0: (f64, f64), p1: (f64, f64), v0: f64, v1: f64) -> Profile {
    Profile { degree: 1, knots: vec![v0, v0, v1, v1], poles: vec![(p0.0, p0.1, 1.0), (p1.0, p1.1, 1.0)] }
}

/// Exact NURBS patch of the cylinder over `v in [v0, v1]` and the full turn
/// (`u` knots at the quarter angles, `v` equals the cylinder's `v`).
pub fn cylinder_to_nurbs(c: &Cylinder, v0: f64, v1: f64) -> Result<NurbsSurface, GeomError> {
    if !(v1 > v0 && v0.is_finite() && v1.is_finite()) {
        return Err(GeomError::Invalid("cylinder patch needs a finite increasing v range"));
    }
    revolve(&c.frame, &line_profile((c.radius, v0), (c.radius, v1), v0, v1), 0.0, TAU)
}

/// Exact NURBS patch of the cone over `v in [v0, v1]` (both on the parametric nappe).
pub fn cone_to_nurbs(c: &Cone, v0: f64, v1: f64) -> Result<NurbsSurface, GeomError> {
    if !(v1 > v0 && v0.is_finite() && v1.is_finite() && v0 >= c.apex_v()) {
        return Err(GeomError::Invalid("cone patch needs v0 < v1 on the parametric nappe"));
    }
    let (s, co) = c.half_angle.sin_cos();
    let at = |v: f64| (c.radius + v * s, v * co);
    revolve(&c.frame, &line_profile(at(v0), at(v1), v0, v1), 0.0, TAU)
}

/// Exact NURBS of the whole sphere: `u` the longitude knots at quarter turns, `v` the latitude
/// `-pi/2..pi/2` (poles are collapsed pole rows of the net).
pub fn sphere_to_nurbs(s: &Sphere) -> Result<NurbsSurface, GeomError> {
    let (arc, kv) = unit_arc(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2)?;
    // Latitude profile: (rho, z) = r (cos v, sin v); poles exactly on the axis.
    let mut poles: Vec<(f64, f64, f64)> = arc.iter().map(|(c, sn, w)| (c * s.radius, sn * s.radius, *w)).collect();
    poles[0].0 = 0.0;
    let last = poles.len() - 1;
    poles[last].0 = 0.0;
    revolve(&s.frame, &Profile { degree: 2, knots: kv, poles }, 0.0, TAU)
}

/// Exact NURBS of the whole ring torus (9 x 9 net; both parameters at quarter-turn knots).
pub fn torus_to_nurbs(t: &Torus) -> Result<NurbsSurface, GeomError> {
    let (arc, kv) = unit_arc(0.0, TAU)?;
    let poles = arc.iter().map(|(c, sn, w)| (t.major + t.minor * c, t.minor * sn, *w)).collect();
    revolve(&t.frame, &Profile { degree: 2, knots: kv, poles }, 0.0, TAU)
}

/// Bilinear NURBS patch of a plane over `[u0, u1] x [v0, v1]` (parameters equal the plane's).
pub fn plane_to_nurbs(p: &sm_geom_api::Plane, u: (f64, f64), v: (f64, f64)) -> Result<NurbsSurface, GeomError> {
    if !(u.1 > u.0 && v.1 > v.0 && u.0.is_finite() && u.1.is_finite() && v.0.is_finite() && v.1.is_finite()) {
        return Err(GeomError::Invalid("plane patch needs finite increasing ranges"));
    }
    let f = &p.frame;
    let pt = |a: f64, b: f64| f.origin + f.x * a + f.y * b;
    NurbsSurface::new(1, 1, vec![u.0, u.0, u.1, u.1], vec![v.0, v.0, v.1, v.1], 2, 2, vec![pt(u.0, v.0), pt(u.0, v.1), pt(u.1, v.0), pt(u.1, v.1)], None)
}

/// Rational quadratic Bezier arc of a conic through the end points `p0`, `p2` with tangent
/// intersection `p1` and the shoulder weight `w1` (exact for any conic section).
pub fn conic_arc(p0: Point3, p1: Point3, p2: Point3, w1: f64) -> Result<NurbsCurve, GeomError> {
    if !(w1 > 0.0 && w1.is_finite()) {
        return Err(GeomError::Invalid("conic arc weight must be positive"));
    }
    NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![p0, p1, p2], Some(vec![1.0, w1, 1.0]))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nurbs::{PreparedCurve, PreparedSurface};
    use sm_core::Vector3;
    use sm_geom_api::{ImplicitSurface, ParametricCurve, ParametricSurface, Uv};
    use sm_testkit::geom::{curvatures_from_jet, tilted_frame, world_frame};

    #[test]
    fn circle_is_exact() {
        for fr in [world_frame(), tilted_frame()] {
            let c = Circle::new(fr, 1.75).unwrap();
            let n = PreparedCurve::new(&circle_to_nurbs(&c).unwrap()).unwrap();
            for i in 0..=200 {
                let t = TAU * i as f64 / 200.0;
                let j = n.jet(t);
                let l = fr.to_local(j.p);
                assert!((l.norm() - 1.75).abs() < 1e-14 && l.z.abs() < 1e-14, "t={t}");
                // Curvature of the circle: |d1 x d2| / |d1|^3.
                let k = j.d1.cross(j.d2).norm() / j.d1.norm().powi(3);
                assert!((k - 1.0 / 1.75).abs() < 1e-11, "curvature at t={t}: {k}");
            }
            let len = n.length(n.range(), 1e-12).unwrap();
            assert!((len - TAU * 1.75).abs() < 1e-10, "{len}");
            // Knots sit at the analytic angles.
            for q in 0..4 {
                let a = core::f64::consts::FRAC_PI_2 * q as f64;
                assert!((n.eval(a) - c.eval(a)).norm() < 1e-13);
            }
            // Projection agrees with the analytic circle.
            let p = fr.from_local(Vector3::new(2.0, -3.0, 0.7));
            let (a, b) = (n.project(p, None).unwrap(), c.project(p, None).unwrap());
            assert!((a.distance - b.distance).abs() < 1e-12 && (a.point - b.point).norm() < 1e-10);
        }
    }

    #[test]
    fn ellipse_and_arcs_are_exact() {
        let e = Ellipse::new(tilted_frame(), 2.0, 1.0).unwrap();
        let n = PreparedCurve::new(&ellipse_to_nurbs(&e).unwrap()).unwrap();
        let len = n.length(n.range(), 1e-12).unwrap();
        assert!((len - sm_testkit::geom::ellipse_perimeter(2.0, 1.0)).abs() < 1e-9, "{len}");
        let c = Circle::new(world_frame(), 3.0).unwrap();
        let arc = PreparedCurve::new(&circle_arc_to_nurbs(&c, 0.3, 2.9).unwrap()).unwrap();
        assert!((arc.length(arc.range(), 1e-12).unwrap() - 3.0 * 2.6).abs() < 1e-10);
        assert!((arc.eval(0.3) - c.eval(0.3)).norm() < 1e-14 && (arc.eval(2.9) - c.eval(2.9)).norm() < 1e-14);
        assert!(circle_arc_to_nurbs(&c, 0.0, 7.0).is_err());
    }

    fn residual(f: &dyn ImplicitSurface, s: &PreparedSurface, n: usize) -> f64 {
        let (du, dv) = s.ranges();
        let mut m = 0.0f64;
        for i in 0..=n {
            for j in 0..=n {
                let uv = Uv::new(du.lo + du.len() * i as f64 / n as f64, dv.lo + dv.len() * j as f64 / n as f64);
                m = m.max(f.value(s.eval(uv)).abs());
            }
        }
        m
    }

    #[test]
    fn surfaces_lie_on_their_analytic_implicit_forms() {
        for fr in [world_frame(), tilted_frame()] {
            let cyl = Cylinder::new(fr, 1.5).unwrap();
            let sph = Sphere::new(fr, 2.0).unwrap();
            let tor = Torus::new(fr, 3.0, 1.0).unwrap();
            let cone = Cone::new(fr, 0.5, 0.4).unwrap();
            let cases: [(NurbsSurface, sm_geom_api::ImplicitDef); 4] = [
                (cylinder_to_nurbs(&cyl, -1.0, 2.0).unwrap(), cyl.implicit().unwrap()),
                (sphere_to_nurbs(&sph).unwrap(), sph.implicit().unwrap()),
                (torus_to_nurbs(&tor).unwrap(), tor.implicit().unwrap()),
                (cone_to_nurbs(&cone, 0.0, 3.0).unwrap(), cone.implicit().unwrap()),
            ];
            for (def, imp) in cases {
                let s = PreparedSurface::new(&def).unwrap();
                let r = residual(&imp, &s, 24);
                assert!(r < 1e-12, "implicit residual {r}");
            }
        }
    }

    #[test]
    fn cylinder_and_sphere_curvatures_and_normals() {
        let fr = tilted_frame();
        let cyl = Cylinder::new(fr, 1.5).unwrap();
        let s = PreparedSurface::new(&cylinder_to_nurbs(&cyl, -1.0, 2.0).unwrap()).unwrap();
        for i in 1..8 {
            for j in 1..8 {
                let uv = Uv::new(TAU * i as f64 / 8.0 + 0.01, -1.0 + 3.0 * j as f64 / 8.0);
                let j0 = s.jet(uv);
                let n = s.normal(uv).unwrap();
                // Outward normal: radial direction in the frame.
                let l = fr.to_local(j0.p);
                let radial = fr.vec_from_local(Vector3::new(l.x, l.y, 0.0)) * (1.0 / 1.5);
                assert!((n - radial).norm() < 1e-12);
                let (k, h) = curvatures_from_jet(&j0, n).unwrap();
                assert!(k.abs() < 1e-10, "K={k}");
                assert!((h - 1.0 / 3.0).abs() < 1e-10, "H={h}");
            }
        }
        let sph = Sphere::new(fr, 2.0).unwrap();
        let ps = PreparedSurface::new(&sphere_to_nurbs(&sph).unwrap()).unwrap();
        assert_eq!(ps.domain().singularities.len(), 2, "both poles are collapsed rows");
        let uv = Uv::new(1.3, 0.2);
        let j0 = ps.jet(uv);
        let n = ps.normal(uv).unwrap();
        let (k, h) = curvatures_from_jet(&j0, n).unwrap();
        assert!((k - 0.25).abs() < 1e-10 && (h - 0.5).abs() < 1e-10, "K={k} H={h}");
    }

    #[test]
    fn nurbs_areas_and_projection_match_analytic() {
        let fr = world_frame();
        let sph = Sphere::new(fr, 2.0).unwrap();
        let ps = PreparedSurface::new(&sphere_to_nurbs(&sph).unwrap()).unwrap();
        // Projection of an exterior point lands at the radial point.
        let q = Point3::new(3.0, 4.0, 5.0);
        let pr = ps.project(q, None).unwrap();
        let want = sph.project(q, None).unwrap();
        assert!((pr.distance - want.distance).abs() < 1e-10 && (pr.point - want.point).norm() < 1e-9, "{pr:?} vs {want:?}");
        // Area by midpoint quadrature in (u, v) of |du x dv|.
        let (n_u, n_v) = (160, 160);
        let (du, dv) = ps.ranges();
        let mut area = 0.0;
        for i in 0..n_u {
            for j in 0..n_v {
                let uv = Uv::new(du.lo + du.len() * (i as f64 + 0.5) / n_u as f64, dv.lo + dv.len() * (j as f64 + 0.5) / n_v as f64);
                let jt = ps.jet(uv);
                area += jt.du.cross(jt.dv).norm();
            }
        }
        area *= du.len() * dv.len() / (n_u * n_v) as f64;
        assert!((area - sm_testkit::geom::sphere_area(2.0)).abs() / sm_testkit::geom::sphere_area(2.0) < 1e-4, "{area}");
    }
}
