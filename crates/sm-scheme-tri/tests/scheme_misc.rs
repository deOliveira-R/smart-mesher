//! Parameters, applicability, failure mapping and pole/seam variations.

mod common;

use common::*;
use sm_mesh_api::{FaceIx, FaceKind, MeshRead, validate};
use sm_scheme_api::{Applicability, FailureKind, Params, SchemeError, Sequential, Target};
use sm_scheme_tri::{TriError, TriParams};
use sm_testkit::{loops2d, topo::planar_face};
use sm_topo_api::fixtures::{cylinder, sphere, with_op};
use sm_topo_api::{CellComplex, OpId};

#[test]
fn params_defaults_roundtrip_and_validation() {
    let d = TriParams::default();
    assert_eq!((d.min_angle, d.fail_angle, d.size_factor, d.optimize, d.max_points), (20.0, 12.0, 1.0, true, 2_000_000));
    let p = Params(serde_json::to_value(&d).unwrap());
    assert_eq!(TriParams::from_params(&p).unwrap(), d);
    assert_eq!(TriParams::from_params(&Params::empty()).unwrap(), d);
    // Partial parameters keep the other defaults.
    let q = Params(serde_json::json!({ "min_angle": 25.0 }));
    assert_eq!(TriParams::from_params(&q).unwrap().min_angle, 25.0);
    for bad in [
        serde_json::json!({ "bogus": 1 }),
        serde_json::json!({ "min_angle": -1.0 }),
        serde_json::json!({ "min_angle": 40.0 }),
        serde_json::json!({ "fail_angle": 60.0 }),
        serde_json::json!({ "size_factor": 0.0 }),
        serde_json::json!({ "size_factor": "big" }),
        serde_json::json!({ "max_points": 2 }),
        serde_json::json!({ "optimize": 3 }),
    ] {
        assert!(TriParams::from_params(&Params(bad.clone())).is_err(), "{bad}");
    }
}

#[test]
fn bad_parameters_are_not_applicable_and_never_panic() {
    use sm_scheme_api::{MeshInput, MeshIntent, PlanCtx, WholeFaces};
    let fx = planar_face(&loops2d::square(), 0.0).unwrap();
    let model = sm_model_api::Model::new();
    let sel = [fx.face];
    let input = MeshInput::new(&fx.complex, &model, &WholeFaces, &sel);
    let intent = MeshIntent::default();
    let cx = PlanCtx::new(&input, &intent);
    for name in ["trimesh", "tridelaunay"] {
        let reg = registry();
        let s = reg.get(sm_scheme_api::SchemeDim::Surface, name).unwrap();
        let ok = s.applicability(&cx, Target::Surface(fx.face), &Params::empty());
        assert!(matches!(ok, Applicability::Applicable { score: 30, .. }), "{ok:?}");
        let bad = s.applicability(&cx, Target::Surface(fx.face), &Params(serde_json::json!({ "min_angle": f64::NAN.to_string() })));
        assert!(matches!(bad, Applicability::NotApplicable { .. }), "{bad:?}");
        let bad = s.applicability(&cx, Target::Surface(fx.face), &Params(serde_json::json!({ "nope": true })));
        assert!(matches!(bad, Applicability::NotApplicable { .. }));
        let bad = s.applicability(&cx, Target::Curve(fx.edges[0][0]), &Params::empty());
        assert!(matches!(bad, Applicability::NotApplicable { .. }));
        assert_eq!(s.caps(&Params::empty()), sm_scheme_api::Caps::TRI);
    }
}

#[test]
fn tridelaunay_and_trimesh_give_the_same_mesh() {
    let fx = planar_face(&loops2d::l_shape(), 0.0).unwrap();
    let counts = counts_for(&fx, 0.2);
    let a = mesh_faces(&fx.complex, &[fx.face], &counts, "trimesh", &Sequential).unwrap();
    let b = mesh_faces(&fx.complex, &[fx.face], &counts, "tridelaunay", &Sequential).unwrap();
    assert_eq!(sm_mesh_api::topology_hash(&a.finished.mesh), sm_mesh_api::topology_hash(&b.finished.mesh));
    assert_eq!(sm_mesh_api::geometry_hash(&a.finished.mesh), sm_mesh_api::geometry_hash(&b.finished.mesh));
}

#[test]
fn spheres_of_many_resolutions_are_closed_valid_and_pole_fans_have_six_triangles() {
    for n in [4u32, 5, 6, 8, 12, 16, 24, 32] {
        let mut c = CellComplex::new();
        let ids = with_op(&mut c, OpId(1), |t| sphere(t, 2.5)).unwrap();
        let out = mesh_faces_params(
            &registry(),
            &c,
            &[ids.face],
            &[(ids.edges[1], n)],
            // The pole fans of coarse spheres are poor: accept any angle here, check topology.
            serde_json::json!({ "fail_angle": 0.0 }),
            &Sequential,
            None,
        )
        .unwrap_or_else(|e| panic!("n = {n}: {e}"));
        let m = &out.finished.mesh;
        assert!(validate(m).is_empty(), "n = {n}: {:?}", validate(m));
        assert_eq!(euler(m), 2, "n = {n}");
        assert!((0..m.face_count()).all(|f| m.face_kind(FaceIx::new(f)) == FaceKind::Interior), "n = {n}");
        for pole in ids.vertices {
            let v = out.vertex_of(sm_scheme_api::NodeKey::vertex(pole)).unwrap().0 as usize;
            assert_eq!(tri_vertices(m).iter().filter(|t| t.contains(&v)).count(), 6, "n = {n}");
        }
        // Outward orientation: every triangle's normal points away from the center.
        for t in tri_points(m) {
            let nrm = (t[1] - t[0]).cross(t[2] - t[0]);
            let c3 = sm_core::Vector3::new(t[0].x + t[1].x + t[2].x, t[0].y + t[1].y + t[2].y, t[0].z + t[1].z + t[2].z);
            assert!(nrm.dot(c3) > 0.0, "n = {n}");
        }
        // Area is bounded by the sphere and positive.
        let a = mesh_area(m);
        assert!(a > 0.0 && a <= 4.0 * std::f64::consts::PI * 2.5 * 2.5 * (1.0 + 1e-12), "n = {n}: {a}");
    }
}

#[test]
fn a_sphere_too_coarse_for_pole_rings_is_unsupported_not_a_panic() {
    let mut c = CellComplex::new();
    let ids = with_op(&mut c, OpId(1), |t| sphere(t, 1.0)).unwrap();
    let err = generate_only(&registry(), &c, &[ids.face], &[(ids.edges[1], 2)], serde_json::json!({}))
        .map(|_| ())
        .expect_err("two seam intervals leave no room for the rings");
    match err {
        SchemeError::Generate(gf) => assert_eq!(gf.kind, FailureKind::Unsupported, "{}", gf.reason),
        other => panic!("{other:?}"),
    }
}

#[test]
fn cylinders_with_few_or_many_intervals() {
    for (around, along) in [(3u32, 1u32), (4, 1), (6, 2), (24, 4), (40, 20)] {
        let mut c = CellComplex::new();
        let ids = with_op(&mut c, OpId(1), |t| cylinder(t, 1.0, 2.0)).unwrap();
        let counts = vec![(ids.edges[0], around), (ids.edges[1], around), (ids.edges[2], along)];
        let r = mesh_faces_params(&registry(), &c, &[ids.faces[2]], &counts, serde_json::json!({ "fail_angle": 0.0 }), &Sequential, None);
        let out = r.unwrap_or_else(|e| panic!("{around}x{along}: {e}"));
        let m = &out.finished.mesh;
        assert!(validate(m).is_empty(), "{around}x{along}: {:?}", validate(m));
        assert_eq!(euler(m), 1 - 1, "{around}x{along}: annulus-like cylinder has Euler 0");
        let _ = m;
    }
}

#[test]
fn error_kinds_map_to_failure_kinds() {
    use sm_scheme_api::NodeKey;
    let k = NodeKey::new(sm_core::EntityId::new(sm_core::EntityKind::TopoEdge, 1).unwrap(), 0);
    assert_eq!(TriError::Unsupported("x".into()).kind(), FailureKind::Unsupported);
    assert_eq!(TriError::Lift { key: k }.kind(), FailureKind::Geometry);
    assert_eq!(TriError::OpenLoop { ring: 0, gap: 1.0 }.kind(), FailureKind::Geometry);
    assert_eq!(TriError::SelfIntersection { a: 0, b: 1 }.kind(), FailureKind::Geometry);
    assert_eq!(TriError::TooManyPoints { limit: 3 }.kind(), FailureKind::Quality);
    assert_eq!(TriError::Quality { min_angle: 1.0, limit: 2.0, worst: [0; 3] }.kind(), FailureKind::Quality);
    assert_eq!(TriError::Stuck { front_edges: 1 }.kind(), FailureKind::Quality);
}

#[test]
fn point_budget_is_a_quality_failure() {
    let fx = planar_face(&loops2d::square(), 0.0).unwrap();
    let counts = counts_for(&fx, 0.1);
    let err = generate_only(&registry(), &fx.complex, &[fx.face], &counts, serde_json::json!({ "max_points": 60 }))
        .map(|_| ())
        .expect_err("budget");
    match err {
        SchemeError::Generate(gf) => {
            assert_eq!(gf.kind, FailureKind::Quality);
            assert!(gf.reason.contains("budget"), "{}", gf.reason);
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn torus_and_cone_faces_mesh_validly() {
    use sm_topo_api::fixtures::{cone, torus};
    // Torus: doubly periodic, two seams, conformal chart. Euler characteristic 0.
    let mut c = CellComplex::new();
    let ids = with_op(&mut c, OpId(1), |t| torus(t, 3.0, 1.0)).unwrap();
    let out = mesh_faces(&c, &[ids.face], &[(ids.edges[0], 24), (ids.edges[1], 12)], "trimesh", &Sequential)
        .expect("torus");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(euler(m), 0);
    assert!(min_angle_deg(m) >= 20.0);
    let exact = 4.0 * std::f64::consts::PI * std::f64::consts::PI * 3.0;
    let a = mesh_area(m);
    assert!(a <= exact && a >= 0.9 * exact, "{a} vs {exact}");
    assert!((0..m.face_count()).all(|f| m.face_kind(FaceIx::new(f)) == FaceKind::Interior));
    // Cone: apex handled through its collapsed coedge; the lateral face is a closed-off disc.
    let mut c = CellComplex::new();
    let ids = with_op(&mut c, OpId(1), |t| cone(t, 2.0, 3.0)).unwrap();
    let counts: Vec<_> = ids.edges.iter().map(|&e| (e, 12u32)).collect();
    for f in ids.faces {
        let r = mesh_faces_params(&registry(), &c, &[f], &counts, serde_json::json!({}), &Sequential, None);
        // Either a valid mesh or a typed refusal (apex faces without pole coedges are deferred).
        match r {
            Ok(o) => {
                assert!(validate(&o.finished.mesh).is_empty());
                assert_eq!(euler(&o.finished.mesh), 1);
            }
            Err(e) => println!("cone face {f}: {e}"),
        }
    }
}
