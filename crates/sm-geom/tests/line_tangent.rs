//! `curve_surface` for a line goes through the contract's `ray_hits_tol`: a line passing a
//! quadric within `tol.linear` is one tangent hit, a farther one is none.

use sm_core::{Point3, Vector3};
use sm_geom::{CurveSurface, curve_surface};
use sm_geom_api::{CurveDef, Cylinder, Frame, Line, ModelTolerance, Sphere, SurfaceDef, Torus};

fn hits(surface: &SurfaceDef, y: f64, z: f64) -> Vec<(f64, bool)> {
    let line = CurveDef::Line(Line::new(Point3::new(-5.0, y, z), Vector3::new(1.0, 0.0, 0.0)).unwrap());
    match curve_surface(&line, None, surface, &ModelTolerance::DEFAULT).unwrap() {
        CurveSurface::Points(h) => h.iter().map(|h| (h.t, h.tangent)).collect(),
        _ => panic!("unexpected result"),
    }
}

#[test]
fn near_tangent_lines_are_one_tangent_hit() {
    let f = Frame::at(Point3::new(0.0, 0.0, 0.0));
    let sphere = SurfaceDef::Sphere(Sphere::new(f, 1.0).unwrap());
    let cyl = SurfaceDef::Cylinder(Cylinder::new(f, 1.0).unwrap());
    let torus = SurfaceDef::Torus(Torus::new(f, 3.0, 1.0).unwrap());
    for (s, y, z, n) in [(&sphere, 0.0, 1.0, 1), (&cyl, 1.0, 0.0, 1), (&torus, 0.0, 1.0, 2)] {
        let h = hits(s, y + 1e-9 * y, z + 1e-9 * z);
        assert_eq!(h.len(), n, "{h:?}");
        assert!(h.iter().all(|h| h.1), "{h:?}");
        assert!(hits(s, y + 1e-3 * y, z + 1e-3 * z).is_empty());
    }
}
