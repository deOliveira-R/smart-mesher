//! Near-tangent surface pairs: the marcher must not jump between branches, and the closed
//! forms must treat a gap within `tol.linear` as a tangent contact.

use sm_core::{Point3, Vector3};
use sm_geom::{CurveGeometry, MarchedCurve, SsiOptions, SsiResult, intersect_surfaces};
use sm_geom_api::{Cylinder, Frame, ImplicitSurface, ModelTolerance, Plane, Sphere, SurfaceDef};

fn world() -> Frame {
    Frame::at(Point3::new(0.0, 0.0, 0.0))
}

fn dist(s: &SurfaceDef, p: Point3) -> f64 {
    let f = s.implicit_def().expect("implicit");
    f.value(p).abs() / f.gradient(p).norm().max(1e-300)
}

fn cyl(frame: Frame, r: f64) -> SurfaceDef {
    SurfaceDef::Cylinder(Cylinder::new(frame, r).unwrap())
}

fn marched(r: SsiResult) -> Vec<MarchedCurve> {
    match r {
        SsiResult::Curves(cs) => cs
            .into_iter()
            .map(|c| match c.geometry {
                CurveGeometry::Marched(m) => m,
                other => panic!("expected marched, got {other:?}"),
            })
            .collect(),
        other => panic!("expected curves, got {other:?}"),
    }
}

/// Near-equal-diameter pipe tee: the two loops (z > 0 and z < 0) are only `2 sqrt(2 dr)` apart
/// at y = +-1. The marcher must not bridge the gap.
#[test]
fn near_equal_perpendicular_cylinders_keep_two_separate_loops() {
    for dr in [1e-2, 1e-3, 1e-4] {
        let a = cyl(world(), 1.0);
        let x_axis = Frame::from_z(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let b = cyl(x_axis, 1.0 + dr);
        let o = SsiOptions::new(ModelTolerance::DEFAULT).with_window([-3.0, -3.0, -3.0, 3.0, 3.0, 3.0]);
        let chord = 1000.0 * o.tol.linear;
        let ms = marched(intersect_surfaces(&a, &b, &o).unwrap());
        assert_eq!(ms.len(), 2, "dr={dr}: two loops, got {}", ms.len());
        for m in &ms {
            assert!(m.closed, "dr={dr}: loop closes");
            for p in &m.points {
                assert!(dist(&a, *p) < 1e-8 && dist(&b, *p) < 1e-8, "dr={dr}: point off the surfaces");
            }
            let (pos, neg) = (m.points.iter().any(|p| p.z > 1e-9), m.points.iter().any(|p| p.z < -1e-9));
            assert!(pos != neg, "dr={dr}: a loop must stay on one side of z = 0");
            // Every chord stays within chord_tol of both surfaces (no branch jumping).
            for w in m.points.windows(2) {
                let mid = Point3::new(0.5 * (w[0].x + w[1].x), 0.5 * (w[0].y + w[1].y), 0.5 * (w[0].z + w[1].z));
                assert!(dist(&a, mid) <= chord && dist(&b, mid) <= chord, "dr={dr}: chord midpoint off the curve");
            }
        }
    }
}

/// Branch gap `2 sqrt(2 dr)` of a few `chord_tol` and below: the step control follows the local
/// branch separation, so the default options still give two loops (previously one bridged
/// branch or a runaway at the point cap for dr = 1e-6 .. 3e-6).
#[test]
fn perpendicular_cylinders_with_a_gap_of_a_few_chord_tol_keep_two_loops() {
    let x_axis = Frame::from_z(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap();
    let o = SsiOptions::new(ModelTolerance::DEFAULT).with_window([-3.0, -3.0, -3.0, 3.0, 3.0, 3.0]);
    let chord = 1000.0 * o.tol.linear;
    for dr in [1e-5, 5e-6, 3e-6, 2e-6, 1e-6, 5e-7, 1e-7, 1e-8, 1e-9, -1e-6, -3e-6] {
        for swap in [false, true] {
            let (a, b) = (cyl(world(), 1.0), cyl(x_axis, 1.0 + dr));
            let (a, b) = if swap { (b, a) } else { (a, b) };
            let ms = marched(intersect_surfaces(&a, &b, &o).unwrap());
            assert_eq!(ms.len(), 2, "dr={dr} swap={swap}: two loops, got {} ({:?})", ms.len(), ms.iter().map(|m| m.points.len()).collect::<Vec<_>>());
            for m in &ms {
                assert!(m.closed, "dr={dr} swap={swap}: loop closes");
                assert!(m.points.len() < 20_000, "dr={dr} swap={swap}: runaway");
                // With the x-cylinder larger the loops are the z > 0 and z < 0 halves; with it
                // smaller they are the x > 0 and x < 0 halves.
                let side = |f: fn(&Point3) -> f64| (m.points.iter().any(|p| f(p) > 1e-9), m.points.iter().any(|p| f(p) < -1e-9));
                let (pos, neg) = if dr > 0.0 { side(|p| p.z) } else { side(|p| p.x) };
                assert!(pos != neg, "dr={dr} swap={swap}: a loop must stay on one side of the splitting plane");
                for w in m.points.windows(2) {
                    let mid = Point3::new(0.5 * (w[0].x + w[1].x), 0.5 * (w[0].y + w[1].y), 0.5 * (w[0].z + w[1].z));
                    assert!(dist(&a, mid) <= chord && dist(&b, mid) <= chord, "dr={dr} swap={swap}: chord midpoint off the curve");
                }
            }
        }
    }
}

/// The same holds for other windows and step caps.
#[test]
fn near_tangent_loops_are_robust_to_window_and_max_step() {
    let x_axis = Frame::from_z(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap();
    for w in [3.0, 10.0, 30.0] {
        for max_step in [0.5, 5.0] {
            for dr in [1e-4, 1e-6] {
                let o = SsiOptions::new(ModelTolerance::DEFAULT).with_window([-w, -w, -w, w, w, w]).with_max_step(max_step);
                let ms = marched(intersect_surfaces(&cyl(world(), 1.0), &cyl(x_axis, 1.0 + dr), &o).unwrap());
                assert_eq!(ms.len(), 2, "w={w} max_step={max_step} dr={dr}: {} curves", ms.len());
                for m in &ms {
                    let (pos, neg) = (m.points.iter().any(|p| p.z > 1e-9), m.points.iter().any(|p| p.z < -1e-9));
                    assert!(m.closed && pos != neg, "w={w} max_step={max_step} dr={dr}");
                }
            }
        }
    }
}

/// A sphere touched by a cylinder at one point: the marcher reports the point (and nothing when
/// the gap is beyond the tolerance).
#[test]
fn sphere_touching_cylinder_is_a_point() {
    let sphere = SurfaceDef::Sphere(Sphere::new(world(), 1.0).unwrap());
    let at = |x: f64| cyl(Frame::at(Point3::new(x, 0.0, 0.0)), 0.5);
    let o = SsiOptions::new(ModelTolerance::DEFAULT).with_window([-3.0, -3.0, -3.0, 3.0, 3.0, 3.0]);
    for (a, b) in [(sphere.clone(), at(1.5)), (at(1.5), sphere.clone())] {
        match intersect_surfaces(&a, &b, &o).unwrap() {
            SsiResult::Point(p) => assert!((p - Point3::new(1.0, 0.0, 0.0)).norm() < 1e-6, "{p:?}"),
            other => panic!("expected the tangent point, got {other:?}"),
        }
        assert_eq!(intersect_surfaces(&a, &b, &o).unwrap(), intersect_surfaces(&a, &b, &o).unwrap());
    }
    assert_eq!(intersect_surfaces(&sphere, &at(1.5 + 1e-4), &o).unwrap(), SsiResult::Empty);
}

/// A shallow penetration is continuous: contact within `tol.linear` is a point, a deeper one is
/// a small closed loop even when it is smaller than the seed-line spacing, and a gap beyond the
/// tolerance is empty.
#[test]
fn shallow_sphere_cylinder_penetration_is_continuous() {
    let cylinder = cyl(world(), 1.0);
    let o = SsiOptions::new(ModelTolerance::DEFAULT).with_window([-3.0, -3.0, -3.0, 3.0, 3.0, 3.0]);
    for swap in [false, true] {
        let run = |g: f64| {
            let sphere = SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::new(1.5 + g, 0.0, 0.0)), 0.5).unwrap());
            if swap { intersect_surfaces(&sphere, &cylinder, &o).unwrap() } else { intersect_surfaces(&cylinder, &sphere, &o).unwrap() }
        };
        for g in [0.0, -1e-7, 5e-7] {
            assert!(matches!(run(g), SsiResult::Point(_)), "g={g} swap={swap}: {:?}", run(g));
        }
        assert_eq!(run(1e-4), SsiResult::Empty);
        for g in [-1e-5, -1e-4, -1e-3, -1e-2, -3e-2] {
            let ms = marched(run(g));
            assert_eq!(ms.len(), 1, "g={g} swap={swap}");
            assert!(ms[0].closed, "g={g} swap={swap}");
            // The loop lies on both surfaces and has the size of the cap: half-width about
            // sqrt(-g) in z and about sqrt(-g/3)-ish in y.
            let sphere_c = Point3::new(1.5 + g, 0.0, 0.0);
            for p in &ms[0].points {
                assert!(((p.x * p.x + p.y * p.y).sqrt() - 1.0).abs() < 1e-6 && ((*p - sphere_c).norm() - 0.5).abs() < 1e-6, "g={g}: {p:?}");
            }
        }
    }
}

/// A parabola that enters the window twice gives two curves, each inside the window, also when
/// the window is tiny and far from the vertex.
#[test]
fn open_conics_are_clipped_to_each_stretch_inside_the_window() {
    // 45 degree cone cut by x + z = 2 (parallel to a generator): z = 1 + y^2 / 4, x = 2 - z.
    let cone = SurfaceDef::Cone(sm_geom_api::Cone::new(world(), 0.0, std::f64::consts::FRAC_PI_4).unwrap());
    let s2 = std::f64::consts::FRAC_1_SQRT_2;
    let plane = SurfaceDef::Plane(Plane { frame: Frame::from_z(Point3::new(0.0, 0.0, 2.0), Vector3::new(s2, 0.0, s2)).unwrap() });
    for window in [[-10.0, -10.0, 5.0, 10.0, 10.0, 8.0], [-224.5, -100.0, 225.5, -223.5, 100.0, 226.5]] {
        let o = SsiOptions::new(ModelTolerance::DEFAULT).with_window(window);
        let SsiResult::Curves(cs) = intersect_surfaces(&plane, &cone, &o).unwrap() else { panic!("expected curves") };
        assert_eq!(cs.len(), 2, "one stretch per side of y = 0");
        let mut sides = Vec::new();
        for c in &cs {
            let CurveGeometry::Analytic { curve, range } = &c.geometry else { panic!("analytic expected") };
            for i in 0..=40 {
                let p = curve.as_curve().eval(range.lo + (range.hi - range.lo) * i as f64 / 40.0);
                let pad = 2e-6;
                assert!(p.x >= window[0] - pad && p.x <= window[3] + pad && p.y >= window[1] - pad && p.y <= window[4] + pad && p.z >= window[2] - pad && p.z <= window[5] + pad, "{p:?} outside {window:?}");
                assert!((p.x + p.z - 2.0).abs() < 1e-9 && (p.x * p.x + p.y * p.y - p.z * p.z).abs() < 1e-6 * (1.0 + p.z * p.z));
            }
            sides.push(curve.as_curve().eval(0.5 * (range.lo + range.hi)).y.signum());
        }
        sides.sort_by(|a, b| a.total_cmp(b));
        assert_eq!(sides, vec![-1.0, 1.0]);
    }
}

fn plane_at_z(z: f64) -> SurfaceDef {
    SurfaceDef::Plane(Plane { frame: Frame::at(Point3::new(0.0, 0.0, z)) })
}

fn unit_sphere() -> SurfaceDef {
    SurfaceDef::Sphere(Sphere::new(world(), 1.0).unwrap())
}

fn opts() -> SsiOptions {
    SsiOptions::new(ModelTolerance::DEFAULT).with_window([-4.0, -4.0, -4.0, 4.0, 4.0, 4.0])
}

/// A gap within `tol.linear` is a tangent contact: sphere against plane, from either side.
#[test]
fn plane_sphere_gap_within_tolerance_is_a_point() {
    for gap in [1e-13, 1e-10, 1e-8, 1e-7, 5e-7] {
        for sgn in [1.0, -1.0] {
            let r = intersect_surfaces(&unit_sphere(), &plane_at_z(sgn * (1.0 + gap)), &opts()).unwrap();
            match r {
                SsiResult::Point(p) => assert!((p - Point3::new(0.0, 0.0, sgn)).norm() <= 2e-6, "gap {gap}: {p:?}"),
                other => panic!("gap {gap}: expected a point, got {other:?}"),
            }
        }
    }
    // Beyond the tolerance it is empty, and below the surface it is a circle.
    assert_eq!(intersect_surfaces(&unit_sphere(), &plane_at_z(1.0 + 1e-5), &opts()).unwrap(), SsiResult::Empty);
    assert!(matches!(intersect_surfaces(&unit_sphere(), &plane_at_z(0.5), &opts()).unwrap(), SsiResult::Curves(_)));
}

/// Same for a cylinder: a plane parallel to the axis at distance `1 + gap` gives the tangent line.
#[test]
fn plane_cylinder_gap_within_tolerance_is_a_line() {
    let x_plane = |x: f64| SurfaceDef::Plane(Plane { frame: Frame::from_z(Point3::new(x, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap() });
    for gap in [1e-13, 1e-10, 1e-8, 5e-7] {
        let r = intersect_surfaces(&cyl(world(), 1.0), &x_plane(1.0 + gap), &opts()).unwrap();
        let SsiResult::Curves(cs) = r else { panic!("gap {gap}: expected a line, got {r:?}") };
        assert_eq!(cs.len(), 1, "gap {gap}");
    }
    assert_eq!(intersect_surfaces(&cyl(world(), 1.0), &x_plane(1.0 + 1e-5), &opts()).unwrap(), SsiResult::Empty);
}

#[test]
fn sphere_sphere_gap_within_tolerance_is_a_point() {
    let at = |x: f64, r: f64| SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::new(x, 0.0, 0.0)), r).unwrap());
    for gap in [1e-13, 1e-10, 1e-8, 5e-7] {
        // External.
        let r = intersect_surfaces(&at(0.0, 1.0), &at(2.0 + gap, 1.0), &opts()).unwrap();
        assert!(matches!(r, SsiResult::Point(_)), "external gap {gap}: {r:?}");
        // Internal: a unit sphere inside a sphere of radius 2 whose surface is `gap` away.
        let r = intersect_surfaces(&at(0.0, 2.0), &at(1.0 - gap, 1.0), &opts()).unwrap();
        assert!(matches!(r, SsiResult::Point(_)), "internal gap {gap}: {r:?}");
    }
    assert_eq!(intersect_surfaces(&at(0.0, 1.0), &at(2.0 + 1e-5, 1.0), &opts()).unwrap(), SsiResult::Empty);
    assert_eq!(intersect_surfaces(&at(0.0, 2.0), &at(1.0 - 1e-5, 1.0), &opts()).unwrap(), SsiResult::Empty);
}

/// The plane-quadric result may not depend on where the plane's frame origin sits.
#[test]
fn plane_quadric_is_independent_of_the_plane_origin() {
    for off in [0.0, 1e2, 1e4, 1e5, 1e6, -3e5] {
        let n = Vector3::new(0.0, 0.0, 1.0);
        let fr = Frame::from_z(Point3::new(off, 0.7 * off, 0.5), n).unwrap();
        let r = intersect_surfaces(&unit_sphere(), &SurfaceDef::Plane(Plane { frame: fr }), &opts()).unwrap();
        let SsiResult::Curves(cs) = r else { panic!("offset {off}: expected a circle, got {r:?}") };
        let CurveGeometry::Analytic { curve, .. } = &cs[0].geometry else { panic!("analytic expected") };
        let sphere = unit_sphere();
        for i in 0..8 {
            let p = curve.as_curve().eval(i as f64 * 0.8);
            assert!(dist(&sphere, p) < 1e-9 && (p.z - 0.5).abs() < 1e-9, "offset {off}");
            assert!(((p.x * p.x + p.y * p.y).sqrt() - 0.75f64.sqrt()).abs() < 1e-8, "offset {off}: radius");
        }
    }
}
