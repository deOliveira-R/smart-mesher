//! Polish round 0: the marcher through a point where the implicit surface's gradient vanishes,
//! and boundary pins for the fixed thresholds that are not tolerances.

use sm_core::{Point3, Vector3};
use sm_geom::{CurveGeometry, SsiOptions, SsiResult, intersect_surfaces};
use sm_geom_api::{Cone, Cylinder, Frame, ModelTolerance, SurfaceDef};

fn opts() -> SsiOptions {
    SsiOptions::new(ModelTolerance::DEFAULT).with_window([-6.0, -6.0, -6.0, 6.0, 6.0, 6.0])
}

fn marched_points(a: &SurfaceDef, b: &SurfaceDef) -> Vec<Point3> {
    let SsiResult::Curves(cs) = intersect_surfaces(a, b, &opts()).unwrap() else { panic!("expected curves") };
    cs.iter()
        .flat_map(|c| match &c.geometry {
            CurveGeometry::Marched(m) => m.points.clone(),
            other => panic!("expected marched, got {other:?}"),
        })
        .collect()
}

/// A cylinder through the apex of a cone: `T`'s gradient is zero where the branch passes. The
/// march must reach the apex region and cost about what a nearby regular configuration costs.
#[test]
fn march_through_a_vanishing_gradient_terminates_in_reasonable_steps() {
    let cone = SurfaceDef::Cone(Cone::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 0.0, 0.5).unwrap());
    let cylinder_at = |x: f64| SurfaceDef::Cylinder(Cylinder::new(Frame::from_z(Point3::new(x, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap(), 1.0).unwrap());
    let through = cylinder_at(1.0);
    let regular = cylinder_at(1.3);
    let reference = marched_points(&regular, &cone).len();
    for (a, b, name) in [(&through, &cone, "cylinder, cone"), (&cone, &through, "cone, cylinder")] {
        let pts = marched_points(a, b);
        let nearest = pts.iter().map(|p| p.x.hypot(p.y).hypot(p.z)).fold(f64::INFINITY, f64::min);
        eprintln!("{name}: {} points (regular pair {reference}), nearest to the apex {nearest}", pts.len());
        assert!(nearest < 0.05, "{name}: the branch stops {nearest} short of the apex");
        assert!(pts.len() < 4 * reference + 100, "{name}: {} points against {reference} for a regular pair", pts.len());
    }
}

// ------------------------------------------------------------------ conic shape classification

fn conic_kind(delta: f64) -> &'static str {
    use sm_geom_api::{CurveDef, Plane};
    let h = 0.5f64;
    let cone = SurfaceDef::Cone(Cone::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 0.0, h).unwrap());
    // Normal perpendicular to the generator (sin h, 0, cos h), turned by `delta` about y.
    let a = h + delta;
    let n = Vector3::new(a.cos(), 0.0, -a.sin());
    let plane = SurfaceDef::Plane(Plane { frame: Frame::from_z(Point3::new(0.0, 0.0, 1.0), n).unwrap() });
    let w = SsiOptions::new(ModelTolerance::DEFAULT).with_window([-1e7, -1e7, -1e7, 1e7, 1e7, 1e7]);
    match intersect_surfaces(&plane, &cone, &w) {
        Ok(SsiResult::Curves(cs)) => match &cs[0].geometry {
            CurveGeometry::Analytic { curve: CurveDef::Ellipse(_), .. } => "ellipse",
            CurveGeometry::Analytic { curve: CurveDef::Nurbs(_), .. } => "open",
            _ => "other",
        },
        _ => "none",
    }
}

/// The conic type threshold (`1e-10` relative size of the smaller quadratic eigenvalue) sits
/// where it is documented: the smaller eigenvalue grows linearly with the angle by which a plane
/// is turned off the parallel-to-a-generator position, so the type flips at about 1e-10 rad. A tiny
/// angle is still a parabola, by a visible one an ellipse (or a hyperbola on the other side),
/// and the type changes once in between.
#[test]
fn conic_shape_threshold_is_where_it_is_documented() {
    // The sign of `delta` that opens an ellipse.
    let s = if conic_kind(1e-2) == "ellipse" { 1.0 } else { -1.0 };
    assert_eq!(conic_kind(s * 1e-2), "ellipse");
    assert_eq!(conic_kind(0.0), "open", "exactly parallel: a parabola");
    assert_eq!(conic_kind(s * 1e-13), "open");
    // Bisect (in log delta) for the flip.
    let (mut lo, mut hi) = (1e-13f64, 1e-2f64);
    for _ in 0..40 {
        let m = (lo * hi).sqrt();
        if conic_kind(s * m) == "ellipse" {
            hi = m;
        } else {
            lo = m;
        }
    }
    eprintln!("conic type flips at {hi:e} rad");
    assert!(hi > 5e-11 && hi < 2e-10, "flip at {hi}");
}

// ------------------------------------------------------------------ box leaf of pair_hits

/// A circle cutting the NURBS unit sphere by depth `-g` crosses it twice, about `2 sqrt(|g|)`
/// apart (combined curvature 2). `pair_hits` stops its box search at 1e-3 of the larger
/// bounding diagonal (3.5e-3 here): crossings farther apart than a leaf are found separately
/// (documented limit: closer ones in one box may come out as one contact).
#[test]
fn pair_hits_leaf_separates_crossings_farther_apart_than_a_leaf() {
    use sm_geom::{CurveSurface, curve_surface, exact};
    use sm_geom_api::{Circle, CurveDef, Sphere};
    let sphere = Sphere::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0).unwrap();
    let ns = SurfaceDef::Nurbs(exact::sphere_to_nurbs(&sphere).unwrap());
    let hits = |g: f64| {
        let c = CurveDef::Circle(Circle::new(Frame::at(Point3::new(2.0 + g, 0.0, 0.0)), 1.0).unwrap());
        match curve_surface(&c, None, &ns, &ModelTolerance::DEFAULT).unwrap() {
            CurveSurface::Points(p) => p,
            other => panic!("{other:?}"),
        }
    };
    // Separation 2 sqrt(1e-5) = 6.3e-3 and 2 sqrt(4e-5) = 1.3e-2: both above the leaf (3.5e-3).
    for g in [-1e-5, -4e-5, -1e-3] {
        let h = hits(g);
        assert_eq!(h.len(), 2, "g={g}: {h:?}");
        assert!(h.iter().all(|x| !x.tangent), "g={g}: {h:?}");
    }
    // Below the leaf (2.8e-3 apart) the documented guarantee is at least one contact.
    assert!(!hits(-2e-6).is_empty());
}
