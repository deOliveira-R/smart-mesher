//! Determinism: the same inputs give bit-identical outputs on repeated runs and across threads,
//! and the geometry store hands out the same ids for the same insertion order.

use sm_core::{Point3, Vector3};
use sm_geom_api::*;

/// Build a store, evaluate a battery of queries and return everything as raw bits / ids.
fn run() -> (Vec<u64>, Vec<u64>) {
    let mut s = MemoryStore::new();
    let f = Frame::from_z(Point3::new(0.3, -0.2, 0.1), Vector3::new(1.0, 2.0, 3.0)).unwrap();
    let ids = [
        s.add_surface(SurfaceDef::Plane(Plane { frame: f })).unwrap(),
        s.add_surface(SurfaceDef::Cylinder(Cylinder::new(f, 1.5).unwrap())).unwrap(),
        s.add_surface(SurfaceDef::Cone(Cone::new(f, 0.5, 0.4).unwrap())).unwrap(),
        s.add_surface(SurfaceDef::Sphere(Sphere::new(f, 2.0).unwrap())).unwrap(),
        s.add_surface(SurfaceDef::Torus(Torus::new(f, 3.0, 1.0).unwrap())).unwrap(),
        s.add_surface(SurfaceDef::Implicit(ImplicitDef::Quadric(Quadric { coeffs: [0.25, 1.0, 1.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, -1.5] }))).unwrap(),
    ];
    let curve = s
        .add_curve(CurveDef::Nurbs(
            NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0), Point3::new(0.0, 1.0, 0.0)], Some(vec![1.0, 0.7, 1.0])).unwrap(),
        ))
        .unwrap();
    let mut out = Vec::new();
    for id in ids {
        let ray = Ray { origin: Point3::new(-9.0, 0.3, 0.2), dir: Vector3::new(1.0, 0.01, 0.02) };
        let mut hits = Vec::new();
        s.implicit(id).unwrap().ray_hits(&ray, Interval::new(0.0, 100.0), &mut hits).unwrap();
        for h in hits {
            out.extend([h.t.to_bits(), h.normal.x.to_bits(), h.point.z.to_bits(), h.crossing as u64]);
        }
    }
    let c = s.curve(curve).unwrap();
    for k in 0..=20 {
        out.push(c.jet(k as f64 / 20.0).d2.x.to_bits());
    }
    out.push(c.length(Interval::new(0.0, 1.0), 1e-12).unwrap().to_bits());
    (ids.iter().map(|i| i.to_raw()).collect(), out)
}

#[test]
fn repeated_runs_and_threads_are_bit_identical() {
    let reference = run();
    assert_eq!(run(), reference);
    let results: Vec<_> = std::thread::scope(|s| {
        let hs: Vec<_> = (0..4).map(|_| s.spawn(run)).collect();
        hs.into_iter().map(|h| h.join().unwrap()).collect()
    });
    for r in results {
        assert_eq!(r, reference);
    }
    // Quantization is platform independent integer arithmetic on the rounded value.
    let t = ModelTolerance::DEFAULT;
    assert_eq!(t.quantize(0.123_456_789), t.quantize(0.123_456_789 + 0.2 * t.quantum()));
}
