//! Frame independence: closed-form and marched intersections survive any rigid motion (and
//! mirror) of the input surfaces, using the testkit's transform strategies.

use proptest::prelude::*;
use sm_core::{Point3, Transform, Vector3};
use sm_geom::{CurveGeometry, IntersectionCurve, SsiOptions, SsiResult, intersect_surfaces};
use sm_geom_api::{Cone, Cylinder, Frame, ImplicitSurface, ModelTolerance, Plane, Sphere, SurfaceDef, Torus};
use sm_testkit::strategies::{any_transform, config, proper_transform};

fn dist(s: &SurfaceDef, p: Point3) -> f64 {
    let f = s.implicit_def().expect("implicit");
    f.value(p).abs() / f.gradient(p).norm().max(1e-300)
}

fn points(c: &IntersectionCurve) -> Vec<Point3> {
    match &c.geometry {
        CurveGeometry::Analytic { curve, range } => {
            let (lo, hi) = (range.lo.max(-8.0), range.hi.min(8.0));
            let (lo, hi) = if curve.as_curve().domain().is_periodic() { (range.lo, range.hi) } else { (lo, hi) };
            (0..=24).map(|i| curve.as_curve().eval(lo + (hi - lo) * i as f64 / 24.0)).collect()
        }
        CurveGeometry::Marched(m) => m.points.clone(),
        _ => unreachable!(),
    }
}

fn world() -> Frame {
    Frame::at(Point3::default())
}

fn base_pair(kind: u8, size: f64, offset: f64, tilt: f64) -> (SurfaceDef, SurfaceDef) {
    let plane = SurfaceDef::Plane(Plane { frame: Frame::from_z(Point3::new(0.0, 0.0, offset), Vector3::new(tilt, 0.2 * tilt, 1.0)).unwrap() });
    let other = match kind {
        0 => SurfaceDef::Sphere(Sphere::new(world(), size).unwrap()),
        1 => SurfaceDef::Cylinder(Cylinder::new(world(), size).unwrap()),
        2 => SurfaceDef::Cone(Cone::new(world(), 0.2 * size, 0.5).unwrap()),
        _ => SurfaceDef::Torus(Torus::new(world(), 2.0 * size, size).unwrap()),
    };
    (plane, other)
}

proptest! {
    #![proptest_config(config(48))]

    #[test]
    fn plane_quadric_sections_are_frame_independent(kind in 0u8..4, size in 0.5f64..2.0, offset in -0.4f64..0.4, tilt in -0.6f64..0.6, t in any_transform()) {
        let (pa, sa) = base_pair(kind, size, offset, tilt);
        let move_surface = |s: &SurfaceDef| s.transformed(&t).unwrap().0;
        let (pb, sb) = (move_surface(&pa), move_surface(&sa));
        let opts = SsiOptions::new(ModelTolerance::DEFAULT).with_window({
            // A cube around the moved origin that contains the interesting part.
            let c = t.apply_point(Point3::default());
            [c.x - 12.0, c.y - 12.0, c.z - 12.0, c.x + 12.0, c.y + 12.0, c.z + 12.0]
        }).with_seed_lines(32);
        let before = intersect_surfaces(&pa, &sa, &SsiOptions::new(ModelTolerance::DEFAULT).with_window([-12.0, -12.0, -12.0, 12.0, 12.0, 12.0]).with_seed_lines(32)).unwrap();
        let after = intersect_surfaces(&pb, &sb, &opts).unwrap();
        match (&before, &after) {
            (SsiResult::Curves(b), SsiResult::Curves(a)) => {
                prop_assert_eq!(b.len(), a.len(), "curve count is frame independent");
                for c in a {
                    for p in points(c) {
                        prop_assert!(dist(&pb, p) < 1e-7 && dist(&sb, p) < 1e-7, "{p:?}: {} {}", dist(&pb, p), dist(&sb, p));
                    }
                }
            }
            (SsiResult::Empty, SsiResult::Empty) | (SsiResult::Point(_), SsiResult::Point(_)) => {}
            (b, a) => prop_assert!(false, "different result kinds: {b:?} vs {a:?}"),
        }
    }

    /// Re-anchoring the plane's frame origin inside the plane does not change the section.
    #[test]
    fn plane_quadric_sections_do_not_depend_on_the_plane_origin(kind in 0u8..4, size in 0.5f64..2.0, offset in -0.4f64..0.4, tilt in -0.6f64..0.6, a in -1e6f64..1e6, b in -1e6f64..1e6) {
        let (pa, sa) = base_pair(kind, size, offset, tilt);
        let SurfaceDef::Plane(plane) = &pa else { unreachable!() };
        let f = plane.frame;
        let moved = SurfaceDef::Plane(Plane { frame: Frame { origin: f.origin + f.x * a + f.y * b, ..f } });
        let o = SsiOptions::new(ModelTolerance::DEFAULT).with_window([-12.0, -12.0, -12.0, 12.0, 12.0, 12.0]).with_seed_lines(32);
        let before = intersect_surfaces(&pa, &sa, &o).unwrap();
        let after = intersect_surfaces(&moved, &sa, &o).unwrap();
        match (&before, &after) {
            (SsiResult::Curves(x), SsiResult::Curves(y)) => {
                prop_assert_eq!(x.len(), y.len());
                for c in y {
                    for p in points(c) {
                        prop_assert!(dist(&moved, p) < 1e-6 && dist(&sa, p) < 1e-6, "{p:?}: {} {}", dist(&moved, p), dist(&sa, p));
                    }
                }
            }
            (SsiResult::Empty, SsiResult::Empty) | (SsiResult::Point(_), SsiResult::Point(_)) => {}
            (x, y) => prop_assert!(false, "different result kinds: {x:?} vs {y:?}"),
        }
    }

    #[test]
    fn marched_cylinder_pair_is_frame_independent(r2 in 0.3f64..0.8, t in proper_transform()) {
        let a = SurfaceDef::Cylinder(Cylinder::new(world(), 1.0).unwrap());
        let b = SurfaceDef::Cylinder(Cylinder::new(Frame::from_z(Point3::default(), Vector3::new(1.0, 0.0, 0.0)).unwrap(), r2).unwrap());
        let (a2, b2) = (a.transformed(&t).unwrap().0, b.transformed(&t).unwrap().0);
        let c = t.apply_point(Point3::default());
        let o = SsiOptions::new(ModelTolerance::DEFAULT).with_window([c.x - 3.0, c.y - 3.0, c.z - 3.0, c.x + 3.0, c.y + 3.0, c.z + 3.0]);
        let SsiResult::Curves(cs) = intersect_surfaces(&a2, &b2, &o).unwrap() else { return Err(TestCaseError::fail("expected curves")) };
        prop_assert_eq!(cs.len(), 2);
        for cv in &cs {
            prop_assert!(cv.closed);
            for p in points(cv) {
                prop_assert!(dist(&a2, p) < 1e-7 && dist(&b2, p) < 1e-7);
            }
        }
    }
}

#[test]
fn transform_helper_is_used() {
    // Keeps the import of `Transform` meaningful for readers: identity leaves points alone.
    assert_eq!(Transform::IDENTITY.apply_point(Point3::new(1.0, 2.0, 3.0)), Point3::new(1.0, 2.0, 3.0));
}
