//! Exact images of shape definitions under an affine [`Transform`] (W0f).
//!
//! A similarity (rotation, translation, uniform scale, optionally a reflection) keeps every kind:
//! frames are mapped and lengths scaled. A non-uniform map keeps lines, planes and NURBS (affine
//! invariance: the control points are mapped); circles and ellipses become ellipses; cylinders
//! and elliptic cylinders become [`SurfaceDef::EllipticCylinder`], cones and elliptic cones
//! [`SurfaceDef::EllipticCone`] (the same nappe, its `v = 0` section through the image of the old
//! section centre); spheres become their implicit quadric ([`SurfaceDef::Implicit`]); a torus has
//! no exact representation here and returns [`GeomError::Unsupported`]. A singular map is
//! [`GeomError::Singular`].
//!
//! **Parametrization.** Curves keep their parameter where the kind allows: a line's parameter
//! is scaled by `|M dir|` (arc length), circle and ellipse angles are unchanged under a
//! similarity (under a non-uniform map the ellipse angle is shifted to its new principal axes),
//! NURBS keep their knots. Analytic surfaces of revolution keep their axis (`z' = R z`), so under
//! a reflection their angle `u` runs the other way; a plane keeps `(u, v)` scaled by `s`.
//!
//! **Sense.** [`SurfaceDef::transformed`] returns whether the new definition's natural normal
//! (`du x dv`, or the implicit gradient) is opposite to the image of the old one (`M^-T n`, the
//! normal that keeps pointing out of a mapped solid). It is `Reversed` only for planes and
//! NURBS surfaces under an improper map; surfaces of revolution and implicit forms keep their
//! outward normal.

use crate::implicit::{ImplicitDef, ImplicitTorus};
use crate::store::{CurveDef, SurfaceDef};
use crate::util::unit;
use crate::elliptic::{cone_from_quadric, cylinder_from_section};
use crate::{Circle, Cone, Cylinder, EllipticCone, EllipticCylinder, Ellipse, Frame, GeomError, Line, NurbsCurve, NurbsSurface, Plane, Sphere, Torus};
use sm_core::{Point3, Transform, Vector3};

/// Whether a transformed surface's natural normal agrees with the image of the original normal.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub enum Sense {
    /// Same orientation.
    Forward,
    /// Opposite orientation (compose it into the face's carrier sense).
    Reversed,
}

/// `(s, improper)` if `t.m = s R` with `R` orthogonal, `s > 0` (relative tolerance `1e-12`).
fn similarity(t: &Transform) -> Result<Option<(f64, bool)>, GeomError> {
    let det = t.det();
    if !(det.is_finite() && det.abs() > f64::MIN_POSITIVE) {
        return Err(GeomError::Singular("transform is singular"));
    }
    let m = &t.m;
    let col = |j: usize| Vector3::new(m[0][j], m[1][j], m[2][j]);
    let (a, b, c) = (col(0), col(1), col(2));
    let s2 = (a.dot(a) + b.dot(b) + c.dot(c)) / 3.0;
    let tol = 1e-12 * s2;
    let ok = (a.dot(a) - s2).abs() <= tol
        && (b.dot(b) - s2).abs() <= tol
        && (c.dot(c) - s2).abs() <= tol
        && a.dot(b).abs() <= tol
        && b.dot(c).abs() <= tol
        && c.dot(a).abs() <= tol;
    Ok(ok.then(|| (s2.sqrt(), det < 0.0)))
}

fn lin(t: &Transform, v: Vector3) -> Vector3 {
    t.apply_vector(v)
}

/// Frame image keeping `x` and `y` (so a planar parametrization is preserved); `z' = x' x y'`.
fn frame_xy(fr: &Frame, t: &Transform, s: f64) -> Frame {
    let k = 1.0 / s;
    let x = lin(t, fr.x) * k;
    let y = lin(t, fr.y) * k;
    Frame { origin: t.apply_point(fr.origin), x, y, z: x.cross(y) }
}

/// Frame image keeping the axis `z` and `x`; `y' = z' x x'` (flips under a reflection).
fn frame_zx(fr: &Frame, t: &Transform, s: f64) -> Frame {
    let k = 1.0 / s;
    let x = lin(t, fr.x) * k;
    let z = lin(t, fr.z) * k;
    Frame { origin: t.apply_point(fr.origin), x, y: z.cross(x), z }
}

/// The ellipse `o + cos t u + sin t w` written with principal axes: returns the frame, the
/// semi-axes and the phase `t0` such that the new angle is `t - t0`.
pub(crate) fn principal_ellipse(o: Point3, u: Vector3, w: Vector3) -> Result<(Frame, f64, f64, f64), GeomError> {
    let t0 = 0.5 * (2.0 * u.dot(w)).atan2(u.dot(u) - w.dot(w));
    let (s0, c0) = t0.sin_cos();
    let a = u * c0 + w * s0;
    let b = u * (-s0) + w * c0;
    let (na, nb) = (a.norm(), b.norm());
    let x = unit(a).ok_or(GeomError::Singular("degenerate ellipse image"))?;
    let y = unit(b).ok_or(GeomError::Singular("degenerate ellipse image"))?;
    // Re-orthogonalize against rounding.
    let y = unit(y - x * x.dot(y)).ok_or(GeomError::Singular("degenerate ellipse image"))?;
    Ok((Frame { origin: o, x, y, z: x.cross(y) }, na, nb, t0))
}

impl Line {
    /// Image under `t` (exact for any non-singular affine map). The parameter is scaled by
    /// `|M dir|`.
    pub fn transformed(&self, t: &Transform) -> Result<Line, GeomError> {
        similarity(t)?;
        Line::new(t.apply_point(self.origin), lin(t, self.dir))
    }
}

impl NurbsCurve {
    /// Image under `t`: control points mapped, knots and weights kept (exact, affine invariance).
    pub fn transformed(&self, t: &Transform) -> Result<NurbsCurve, GeomError> {
        similarity(t)?;
        Ok(NurbsCurve { control: self.control.iter().map(|p| t.apply_point(*p)).collect(), ..self.clone() })
    }
}

impl NurbsSurface {
    /// Image under `t`: control points mapped (exact, affine invariance). The natural normal
    /// flips under an improper map.
    pub fn transformed(&self, t: &Transform) -> Result<NurbsSurface, GeomError> {
        similarity(t)?;
        Ok(NurbsSurface { control: self.control.iter().map(|p| t.apply_point(*p)).collect(), ..self.clone() })
    }
}

impl CurveDef {
    /// Exact image under `t` (see the module docs for the parametrization).
    pub fn transformed(&self, t: &Transform) -> Result<CurveDef, GeomError> {
        let sim = similarity(t)?;
        Ok(match self {
            CurveDef::Line(l) => CurveDef::Line(l.transformed(t)?),
            CurveDef::Nurbs(n) => CurveDef::Nurbs(n.transformed(t)?),
            CurveDef::Circle(c) => match sim {
                Some((s, _)) => CurveDef::Circle(Circle::new(frame_xy(&c.frame, t, s), c.radius * s)?),
                None => {
                    let (fr, a, b, _) = principal_ellipse(t.apply_point(c.frame.origin), lin(t, c.frame.x) * c.radius, lin(t, c.frame.y) * c.radius)?;
                    CurveDef::Ellipse(Ellipse::new(fr, a, b)?)
                }
            },
            CurveDef::Ellipse(e) => match sim {
                Some((s, _)) => CurveDef::Ellipse(Ellipse::new(frame_xy(&e.frame, t, s), e.a * s, e.b * s)?),
                None => {
                    let (fr, a, b, _) = principal_ellipse(t.apply_point(e.frame.origin), lin(t, e.frame.x) * e.a, lin(t, e.frame.y) * e.b)?;
                    CurveDef::Ellipse(Ellipse::new(fr, a, b)?)
                }
            },
        })
    }
}

impl SurfaceDef {
    /// Exact image under `t` and whether its natural normal is opposite to the image of the old
    /// normal (see the module docs). Non-uniform maps of (elliptic) cylinders give an
    /// [`SurfaceDef::EllipticCylinder`], of (elliptic) cones an [`SurfaceDef::EllipticCone`] (same
    /// nappe), of spheres their implicit quadric; a non-uniformly mapped torus is
    /// [`GeomError::Unsupported`]. Surfaces of revolution and elliptic kinds keep their outward
    /// normal (`Forward`).
    pub fn transformed(&self, t: &Transform) -> Result<(SurfaceDef, Sense), GeomError> {
        use crate::surface::ParametricSurface;
        let sim = similarity(t)?;
        let improper = t.det() < 0.0;
        let flip = if improper { Sense::Reversed } else { Sense::Forward };
        let quadric = |q: Option<ImplicitDef>| -> Result<(SurfaceDef, Sense), GeomError> {
            match q {
                Some(d) => Ok((SurfaceDef::Implicit(d.transformed(t)?), Sense::Forward)),
                None => Err(GeomError::Unsupported("no implicit form")),
            }
        };
        Ok(match (self, sim) {
            (SurfaceDef::Plane(p), Some((s, _))) => (SurfaceDef::Plane(Plane { frame: frame_xy(&p.frame, t, s) }), flip),
            (SurfaceDef::Plane(p), None) => {
                let x = unit(lin(t, p.frame.x)).ok_or(GeomError::Singular("plane image"))?;
                let n = lin(t, p.frame.x).cross(lin(t, p.frame.y));
                let frame = Frame::from_zx(t.apply_point(p.frame.origin), n, x)?;
                (SurfaceDef::Plane(Plane { frame }), flip)
            }
            (SurfaceDef::Cylinder(c), Some((s, _))) => (SurfaceDef::Cylinder(Cylinder::new(frame_zx(&c.frame, t, s), c.radius * s)?), Sense::Forward),
            (SurfaceDef::Cone(c), Some((s, _))) => (SurfaceDef::Cone(Cone::new(frame_zx(&c.frame, t, s), c.radius * s, c.half_angle)?), Sense::Forward),
            (SurfaceDef::Sphere(c), Some((s, _))) => (SurfaceDef::Sphere(Sphere::new(frame_zx(&c.frame, t, s), c.radius * s)?), Sense::Forward),
            (SurfaceDef::Torus(c), Some((s, _))) => (SurfaceDef::Torus(Torus::new(frame_zx(&c.frame, t, s), c.major * s, c.minor * s)?), Sense::Forward),
            (SurfaceDef::Cylinder(c), None) => {
                let f = &c.frame;
                (SurfaceDef::EllipticCylinder(cylinder_from_section(t.apply_point(f.origin), lin(t, f.x) * c.radius, lin(t, f.y) * c.radius, lin(t, f.z))?), Sense::Forward)
            }
            (SurfaceDef::EllipticCylinder(c), Some((s, _))) => (SurfaceDef::EllipticCylinder(EllipticCylinder::new(frame_zx(&c.frame, t, s), c.a * s, c.b * s)?), Sense::Forward),
            (SurfaceDef::EllipticCylinder(c), None) => {
                let f = &c.frame;
                (SurfaceDef::EllipticCylinder(cylinder_from_section(t.apply_point(f.origin), lin(t, f.x) * c.a, lin(t, f.y) * c.b, lin(t, f.z))?), Sense::Forward)
            }
            (SurfaceDef::Cone(c), None) => {
                // The v = 0 section centre, or one unit up the axis when it is the apex.
                let up = if c.radius > 0.0 { 0.0 } else { 1.0 };
                (SurfaceDef::EllipticCone(cone_image(c.implicit(), t, lin(t, c.frame.z), c.frame.origin + c.frame.z * up)?), Sense::Forward)
            }
            (SurfaceDef::EllipticCone(c), Some((s, _))) => (SurfaceDef::EllipticCone(EllipticCone::new(frame_zx(&c.frame, t, s), c.a * s, c.b * s, c.half_angle)?), Sense::Forward),
            (SurfaceDef::EllipticCone(c), None) => (SurfaceDef::EllipticCone(cone_image(c.implicit(), t, lin(t, c.frame.z), c.frame.origin)?), Sense::Forward),
            (SurfaceDef::Sphere(c), None) => quadric(c.implicit())?,
            (SurfaceDef::Torus(_), None) => return Err(GeomError::Unsupported("non-uniformly scaled torus")),
            (SurfaceDef::Nurbs(n), _) => (SurfaceDef::Nurbs(n.transformed(t)?), flip),
            (SurfaceDef::Implicit(d), _) => (SurfaceDef::Implicit(d.transformed(t)?), Sense::Forward),
        })
    }
}

/// The right elliptic cone of the image of a cone's double-cone quadric: the nappe along
/// `toward`, its `v = 0` section through the image of `section` (a point on the old axis).
fn cone_image(q: Option<ImplicitDef>, t: &Transform, toward: Vector3, section: Point3) -> Result<EllipticCone, GeomError> {
    let Some(ImplicitDef::Quadric(q)) = q else { return Err(GeomError::Unsupported("cone without a quadric")) };
    let img = q.transformed(t).ok_or(GeomError::Singular("transform is singular"))?;
    cone_from_quadric(&img, toward, t.apply_point(section))
}

impl ImplicitTorus {
    /// Image under a similarity (frame mapped, radii scaled). A non-uniform map is
    /// [`GeomError::Unsupported`] (the image is not a torus).
    pub fn transformed(&self, t: &Transform) -> Result<ImplicitTorus, GeomError> {
        match similarity(t)? {
            Some((s, _)) => Ok(ImplicitTorus { frame: frame_zx(&self.frame, t, s), major: self.major * s, minor: self.minor * s }),
            None => Err(GeomError::Unsupported("non-uniformly scaled torus")),
        }
    }
}

impl ImplicitDef {
    /// Image under `t`, keeping the sign convention (`f'(p) = f(T^-1 p)` up to a positive
    /// factor, so the inside stays negative). Quadrics accept any non-singular map; a torus
    /// needs a similarity.
    pub fn transformed(&self, t: &Transform) -> Result<ImplicitDef, GeomError> {
        Ok(match self {
            ImplicitDef::Quadric(q) => ImplicitDef::Quadric(q.transformed(t).ok_or(GeomError::Singular("transform is singular"))?),
            ImplicitDef::Torus(tt) => ImplicitDef::Torus(tt.transformed(t)?),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::ParametricCurve;
    use crate::implicit::ImplicitSurface;
    use crate::surface::ParametricSurface;
    use crate::{Interval, Uv};
    use core::f64::consts::{PI, TAU};

    fn tilted() -> Frame {
        Frame::from_zx(Point3::new(0.3, -0.7, 1.1), Vector3::new(1.0, 2.0, -0.5), Vector3::new(0.0, 0.0, 1.0)).unwrap()
    }

    fn rigid() -> Transform {
        Transform::rotation(Point3::new(1.0, 0.0, -2.0), Vector3::new(0.3, -1.0, 0.4), 0.9)
            .unwrap()
            .compose(&Transform::translation(Vector3::new(2.0, -1.0, 0.5)))
    }

    fn scaled(s: f64) -> Transform {
        Transform { m: [[s, 0.0, 0.0], [0.0, s, 0.0], [0.0, 0.0, s]], t: Vector3::new(0.5, 0.0, -1.0) }.compose(&rigid())
    }

    fn mirror() -> Transform {
        Transform::reflection(Vector3::new(1.0, 1.0, 0.0), 0.7).unwrap().compose(&scaled(2.5))
    }

    fn stretch() -> Transform {
        Transform { m: [[2.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.5]], t: Vector3::new(1.0, 2.0, 3.0) }
    }

    fn all() -> [Transform; 4] {
        [rigid(), scaled(3.0), mirror(), stretch()]
    }

    fn close(a: Point3, b: Point3, tol: f64) -> bool {
        (a - b).norm() <= tol
    }

    /// Every image point lies on the image curve and vice versa (by projection).
    fn check_curve(c: &CurveDef, t: &Transform, samples: &[f64]) {
        let img = c.transformed(t).unwrap();
        let (src, dst): (&dyn ParametricCurve, &dyn ParametricCurve) = (curve(c), curve(&img));
        for &u in samples {
            let p = t.apply_point(src.eval(u));
            let pr = dst.project(p, None).map(|r| r.distance);
            let d = match pr {
                Ok(d) => d,
                // Ellipse projection is unsupported: check the implicit ellipse equation instead.
                Err(_) => ellipse_residual(&img, p),
            };
            assert!(d < 1e-9, "{c:?} under {t:?}: image point off the curve by {d}");
        }
    }

    fn ellipse_residual(c: &CurveDef, p: Point3) -> f64 {
        let CurveDef::Ellipse(e) = c else { panic!("expected an ellipse") };
        let l = e.frame.to_local(p);
        ((l.x / e.a).powi(2) + (l.y / e.b).powi(2) - 1.0).abs() + l.z.abs()
    }

    fn curve(c: &CurveDef) -> &dyn ParametricCurve {
        match c {
            CurveDef::Line(x) => x,
            CurveDef::Circle(x) => x,
            CurveDef::Ellipse(x) => x,
            CurveDef::Nurbs(x) => x,
        }
    }

    #[test]
    fn curves_map_exactly() {
        let circle = CurveDef::Circle(Circle::new(tilted(), 1.5).unwrap());
        let ellipse = CurveDef::Ellipse(Ellipse::new(tilted(), 2.0, 0.5).unwrap());
        let line = CurveDef::Line(Line::new(Point3::new(1.0, 2.0, 3.0), Vector3::new(1.0, -1.0, 2.0)).unwrap());
        let nurbs = CurveDef::Nurbs(
            NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0), Point3::new(0.0, 1.0, 0.0)], Some(vec![1.0, core::f64::consts::FRAC_1_SQRT_2, 1.0]))
                .unwrap(),
        );
        let samples = [0.0, 0.3, 1.1, 2.0, 4.0, 5.9];
        for t in all() {
            check_curve(&circle, &t, &samples);
            check_curve(&ellipse, &t, &samples);
            check_curve(&line, &t, &samples);
            check_curve(&nurbs, &t, &[0.0, 0.25, 0.5, 1.0]);
        }
        // Under a similarity the parameter is kept exactly (circle, ellipse, nurbs) or scaled
        // (line, by s).
        for t in [rigid(), scaled(3.0), mirror()] {
            for c in [&circle, &ellipse, &nurbs] {
                let img = c.transformed(&t).unwrap();
                for u in [0.0, 0.4, 0.9] {
                    assert!(close(curve(&img).eval(u), t.apply_point(curve(c).eval(u)), 1e-12));
                }
            }
        }
        let CurveDef::Line(l) = line.transformed(&scaled(3.0)).unwrap() else { panic!() };
        let CurveDef::Line(l0) = &line else { panic!() };
        assert!(close(l.eval(3.0 * 1.7), scaled(3.0).apply_point(l0.eval(1.7)), 1e-12));
        // Analytic: a unit circle stretched by (2, 1) in its plane is an ellipse with a = 2, b = 1.
        let unit = CurveDef::Circle(Circle::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0).unwrap());
        let CurveDef::Ellipse(e) = unit.transformed(&stretch()).unwrap() else { panic!("ellipse expected") };
        assert!((e.a.max(e.b) - 2.0).abs() < 1e-14 && (e.a.min(e.b) - 1.0).abs() < 1e-14);
        assert!(close(e.frame.origin, Point3::new(1.0, 2.0, 3.0), 1e-15));
        // A circle under a uniform scale stays a circle with the radius scaled.
        let CurveDef::Circle(c) = unit.transformed(&scaled(3.0)).unwrap() else { panic!() };
        assert!((c.radius - 3.0).abs() < 1e-14);
        assert!((curve(&CurveDef::Circle(c)).length(Interval::new(0.0, TAU), 1e-12).unwrap() - 6.0 * PI).abs() < 1e-12);
    }

    fn sample_uvs(d: &SurfaceDef) -> Vec<Uv> {
        match d {
            SurfaceDef::Plane(_) => vec![Uv::new(0.3, -1.2), Uv::new(2.0, 0.5)],
            SurfaceDef::Sphere(_) => vec![Uv::new(0.3, -1.2), Uv::new(2.0, 0.5), Uv::new(4.0, 1.0)],
            _ => vec![Uv::new(0.3, 0.2), Uv::new(2.0, 0.5), Uv::new(4.0, 1.4)],
        }
    }

    fn psurf(d: &SurfaceDef) -> Option<&dyn ParametricSurface> {
        Some(match d {
            SurfaceDef::Plane(x) => x,
            SurfaceDef::Cylinder(x) => x,
            SurfaceDef::Cone(x) => x,
            SurfaceDef::Sphere(x) => x,
            SurfaceDef::Torus(x) => x,
            SurfaceDef::Nurbs(x) => x,
            SurfaceDef::EllipticCylinder(x) => x,
            SurfaceDef::EllipticCone(x) => x,
            _ => return None,
        })
    }

    fn cofactor_normal(t: &Transform, n: Vector3) -> Vector3 {
        // M^-T n.
        let inv = t.inverse().unwrap();
        let m = inv.m;
        Vector3::new(m[0][0] * n.x + m[1][0] * n.y + m[2][0] * n.z, m[0][1] * n.x + m[1][1] * n.y + m[2][1] * n.z, m[0][2] * n.x + m[1][2] * n.y + m[2][2] * n.z)
    }

    #[test]
    fn surfaces_map_exactly_with_the_right_sense() {
        let defs = [
            SurfaceDef::Plane(Plane { frame: tilted() }),
            SurfaceDef::Cylinder(Cylinder::new(tilted(), 1.5).unwrap()),
            SurfaceDef::Cone(Cone::new(tilted(), 0.5, 0.4).unwrap()),
            SurfaceDef::Sphere(Sphere::new(tilted(), 2.0).unwrap()),
            SurfaceDef::Torus(Torus::new(tilted(), 3.0, 1.0).unwrap()),
            SurfaceDef::Cone(Cone::new(tilted(), 0.0, 0.7).unwrap()),
            SurfaceDef::EllipticCylinder(EllipticCylinder::new(tilted(), 2.0, 0.7).unwrap()),
            SurfaceDef::EllipticCone(EllipticCone::new(tilted(), 1.2, 0.5, 0.4).unwrap()),
            SurfaceDef::Nurbs(NurbsSurface {
                degree_u: 1,
                degree_v: 1,
                knots_u: vec![0.0, 0.0, 1.0, 1.0],
                knots_v: vec![0.0, 0.0, 1.0, 1.0],
                nu: 2,
                nv: 2,
                control: vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.2), Point3::new(1.0, 0.0, 0.1), Point3::new(1.0, 1.0, 0.0)],
                weights: None,
                periodic: [false; 2],
            }),
        ];
        for d in &defs {
            for t in all() {
                let r = d.transformed(&t);
                if matches!(d, SurfaceDef::Torus(_)) && similarity(&t).unwrap().is_none() {
                    assert!(matches!(r, Err(GeomError::Unsupported(_))));
                    continue;
                }
                let (img, sense) = r.unwrap();
                let src = psurf(d).unwrap();
                let uvs = if matches!(d, SurfaceDef::Nurbs(_)) { vec![Uv::new(0.2, 0.7), Uv::new(0.6, 0.3)] } else { sample_uvs(d) };
                for uv in uvs {
                    let p = t.apply_point(src.eval(uv));
                    let n_img = cofactor_normal(&t, src.normal(uv).unwrap());
                    let n_new = match psurf(&img).map(|s| (s, s.project(p, None))) {
                        Some((s, Ok(pr))) => {
                            assert!(pr.distance < 1e-9, "{d:?} under {t:?}: off by {}", pr.distance);
                            s.normal(pr.uv).unwrap()
                        }
                        Some((s, Err(GeomError::Unsupported(_)))) => {
                            let q = s.implicit().unwrap();
                            assert!(q.value(p).abs() < 1e-9 * q.gradient(p).norm().max(1.0), "{d:?} under {t:?}: residual {}", q.value(p));
                            q.gradient(p)
                        }
                        Some((_, Err(e))) => panic!("{e:?}"),
                        None => {
                            let SurfaceDef::Implicit(id) = &img else { panic!() };
                            let v = id.value(p);
                            assert!(v.abs() < 1e-9, "implicit image residual {v}");
                            id.gradient(p)
                        }
                    };
                    let agree = n_new.dot(n_img) > 0.0;
                    assert_eq!(agree, sense == Sense::Forward, "{d:?} under {t:?}");
                }
            }
        }
    }

    #[test]
    fn non_uniform_cylinders_and_cones_become_elliptic_kinds() {
        // Unit cylinder about z stretched by (2, 1, 0.5): elliptic, a = 2, b = 1 (in some order).
        let cyl = SurfaceDef::Cylinder(Cylinder::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0).unwrap());
        let (SurfaceDef::EllipticCylinder(e), Sense::Forward) = cyl.transformed(&stretch()).unwrap() else { panic!("elliptic cylinder expected") };
        assert!((e.a.max(e.b) - 2.0).abs() < 1e-14 && (e.a.min(e.b) - 1.0).abs() < 1e-14);
        assert!((e.frame.z.cross(Vector3::new(0.0, 0.0, 1.0))).norm() < 1e-14);
        // A shear tilts the axis: the section is taken perpendicular to it.
        let shear = Transform { m: [[1.0, 0.0, 1.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t: Vector3::default() };
        let (SurfaceDef::EllipticCylinder(e), _) = cyl.transformed(&shear).unwrap() else { panic!() };
        assert!((e.frame.z - Vector3::new(1.0, 0.0, 1.0) * core::f64::consts::FRAC_1_SQRT_2).norm() < 1e-14);
        assert!((e.a.min(e.b) - core::f64::consts::FRAC_1_SQRT_2).abs() < 1e-14 && (e.a.max(e.b) - 1.0).abs() < 1e-14);
        // A 45 degree cone stretched by 2 along x: section semi-axes (2, 1) per unit height, so
        // the half-angles are atan 2 (x) and 45 degrees (y); the nappe is kept.
        let cone = SurfaceDef::Cone(Cone::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0, core::f64::consts::FRAC_PI_4).unwrap());
        let sx = Transform { m: [[2.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t: Vector3::default() };
        let (SurfaceDef::EllipticCone(c), Sense::Forward) = cone.transformed(&sx).unwrap() else { panic!("elliptic cone expected") };
        assert!((c.apex() - Point3::new(0.0, 0.0, -1.0)).norm() < 1e-12);
        assert!(c.frame.z.z > 0.0, "same nappe");
        let (ta, tb) = (c.half_angle.tan(), c.b * c.half_angle.tan() / c.a);
        assert!((ta.max(tb) - 2.0).abs() < 1e-12 && (ta.min(tb) - 1.0).abs() < 1e-12, "{ta} {tb}");
        // Similarities keep the elliptic kinds; the hash visits them.
        let (SurfaceDef::EllipticCone(c2), _) = SurfaceDef::EllipticCone(c).transformed(&scaled(2.0)).unwrap() else { panic!() };
        assert!((c2.a - 2.0 * c.a).abs() < 1e-12 && c2.half_angle == c.half_angle);
        let mut n = 0;
        SurfaceDef::EllipticCone(c).for_each_value(|_| n += 1);
        assert_eq!(n, 1 + 12 + 3);
        assert!(SurfaceDef::EllipticCylinder(e).validate().is_ok());
        assert!(SurfaceDef::EllipticCylinder(EllipticCylinder { a: -1.0, ..e }).validate().is_err());
    }

    #[cfg(feature = "schemars")]
    #[test]
    fn value_types_have_json_schemas() {
        for s in [
            schemars::schema_for!(CurveDef),
            schemars::schema_for!(SurfaceDef),
            schemars::schema_for!(crate::CsgCell),
            schemars::schema_for!(crate::Domain2),
            schemars::schema_for!(crate::PCurve),
            schemars::schema_for!(crate::ModelTolerance),
            schemars::schema_for!(Sense),
        ] {
            assert!(s.as_object().is_some());
        }
    }

    #[test]
    fn analytic_values_after_transform() {
        // Sphere of radius 2 scaled by 3: radius 6, centre mapped.
        let s = SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::new(1.0, 0.0, 0.0)), 2.0).unwrap());
        let t = Transform { m: [[3.0, 0.0, 0.0], [0.0, 3.0, 0.0], [0.0, 0.0, 3.0]], t: Vector3::new(0.0, 1.0, 0.0) };
        let (SurfaceDef::Sphere(img), Sense::Forward) = s.transformed(&t).unwrap() else { panic!() };
        assert_eq!(img.radius, 6.0);
        assert!(close(img.frame.origin, Point3::new(3.0, 1.0, 0.0), 0.0));
        // Unit sphere stretched by (2, 1, 0.5): the ellipsoid x'^2/4 + y'^2 + 4 z'^2 = 1 (shifted).
        let unit = SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0).unwrap());
        let (SurfaceDef::Implicit(ImplicitDef::Quadric(q)), Sense::Forward) = unit.transformed(&stretch()).unwrap() else { panic!() };
        for p in [Point3::new(3.0, 2.0, 3.0), Point3::new(1.0, 3.0, 3.0), Point3::new(1.0, 2.0, 3.5)] {
            assert!(q.value(p).abs() < 1e-14, "{p:?}");
        }
        assert!(q.value(Point3::new(1.0, 2.0, 3.0)) < 0.0, "centre inside");
        // A plane under a reflection through itself is the same plane (same natural normal), but
        // the image of its normal is -z, so the sense is reversed.
        let pl = SurfaceDef::Plane(Plane { frame: Frame::at(Point3::new(0.0, 0.0, 0.0)) });
        let m = Transform::reflection(Vector3::new(0.0, 0.0, 1.0), 0.0).unwrap();
        let (SurfaceDef::Plane(p2), sense) = pl.transformed(&m).unwrap() else { panic!() };
        assert_eq!(sense, Sense::Reversed);
        assert!((p2.frame.z - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-15);
        // Torus radii scale; the implicit torus agrees with the parametric one.
        let tor = Torus::new(tilted(), 3.0, 1.0).unwrap();
        let (SurfaceDef::Torus(t2), _) = SurfaceDef::Torus(tor).transformed(&mirror()).unwrap() else { panic!() };
        assert!((t2.major - 7.5).abs() < 1e-12 && (t2.minor - 2.5).abs() < 1e-12);
        let it = ImplicitTorus { frame: tor.frame, major: 3.0, minor: 1.0 }.transformed(&mirror()).unwrap();
        let p = mirror().apply_point(tor.eval(Uv::new(1.0, 2.0)));
        assert!(it.value(p).abs() < 1e-9 * it.major.powi(4));
        assert!(ImplicitTorus { frame: tor.frame, major: 3.0, minor: 1.0 }.transformed(&stretch()).is_err());
        // Singular maps are rejected.
        let flat = Transform { m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]], t: Vector3::default() };
        assert!(matches!(pl.transformed(&flat), Err(GeomError::Singular(_))));
        assert!(matches!(CurveDef::Line(Line::new(Point3::default(), Vector3::new(0.0, 0.0, 1.0)).unwrap()).transformed(&flat), Err(GeomError::Singular(_))));
    }
}
