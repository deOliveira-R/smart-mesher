//! T1 acceptance: primitives (brick, cylinder, sphere).

mod common;

use common::*;
use core::f64::consts::PI;
use sm_core::EntityKind;
use sm_model::{CreateBrick, CreateCylinder, CreateSphere};
use sm_topo_api::{Dim, TopologyView, TopologyViewExt, euler_characteristic, validate, validate_geometry};

fn brick10() -> CreateBrick {
    CreateBrick { x: 10.0, y: None, z: None, center: None }
}

fn clean(s: &sm_op_api::Session) {
    let t = model(s).topo();
    assert_eq!(validate(t), vec![], "validate");
    assert_eq!(validate_geometry(t), vec![], "validate_geometry");
}

#[test]
fn brick_counts_vertices_volume_euler_and_validate() {
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = session(factory);
        run(&mut s, &brick10()).expect("brick");
        let t = model(&s).topo();
        let counts: Vec<usize> = Dim::ALL.iter().map(|&d| t.cells(d).len()).collect();
        assert_eq!(counts, vec![8, 12, 6, 1, 1], "V/E/F/volumes/bodies");
        assert_eq!(euler_characteristic(t), 2);
        let pts = vertex_points(&s);
        for (idx, p) in pts.iter().enumerate() {
            let want = [(idx & 1) as f64 * 10.0 - 5.0, ((idx >> 1) & 1) as f64 * 10.0 - 5.0, ((idx >> 2) & 1) as f64 * 10.0 - 5.0];
            assert_eq!(p.map(f64::to_bits), want.map(f64::to_bits), "vertex {idx} is exactly +-5");
        }
        let v = t.volume_moments(t.volumes()[0]).expect("moments").volume;
        assert!((v - 1000.0).abs() <= 1e-12 * 1000.0, "volume {v}");
        clean(&s);
    }
}

#[test]
fn brick_hash_equals_the_cell_complex_fixture() {
    let want = sm_testkit::topo::brick([-5.0; 3], [10.0; 3]).expect("fixture").complex.canonical_hash();
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = session(factory);
        run(&mut s, &brick10()).expect("brick");
        assert_eq!(model(&s).topo().canonical_hash(), want);
    }
}

#[test]
fn brick_delta_lists_the_28_ids_ascending_and_the_declared_roles() {
    // Production ids come from `TopoStore`, so both hosts are checked.
    for factory in [cell_complex as Factory, topo_store] {
        brick_delta_on(factory);
    }
}

fn brick_delta_on(factory: Factory) {
    let mut s = session(factory);
    let (_, d) = run(&mut s, &brick10()).expect("brick");
    let mut want = Vec::new();
    for (k, n) in [
        (EntityKind::TopoVertex, 8),
        (EntityKind::TopoEdge, 12),
        (EntityKind::TopoFace, 6),
        (EntityKind::TopoVolume, 1),
        (EntityKind::TopoBody, 1),
    ] {
        want.extend(range(k, n));
    }
    assert_eq!(d.created, want);
    assert!(d.removed.is_empty());
    let roles: Vec<&str> = d.outputs.keys().map(String::as_str).collect();
    assert_eq!(roles, vec!["body", "edges", "faces", "vertices", "volume"]);
    assert_eq!(d.outputs["volume"], range(EntityKind::TopoVolume, 1));
    assert_eq!(d.outputs["body"], range(EntityKind::TopoBody, 1));
    assert_eq!(d.outputs["faces"], range(EntityKind::TopoFace, 6));
    assert_eq!(d.outputs["edges"], range(EntityKind::TopoEdge, 12));
    assert_eq!(d.outputs["vertices"], range(EntityKind::TopoVertex, 8));
}

#[test]
fn cylinder_hash_equals_the_cell_complex_fixture() {
    // The fixture spans z = 0..h; the op is centred at the origin by default.
    let want = sm_testkit::topo::cylinder(1.5, 4.0).expect("fixture").complex.canonical_hash();
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = session(factory);
        run(&mut s, &CreateCylinder { height: 4.0, radius: 1.5, center: Some([0.0, 0.0, 2.0]) }).expect("cylinder");
        assert_eq!(model(&s).topo().canonical_hash(), want);
    }
}

#[test]
fn sphere_hash_equals_the_cell_complex_fixture() {
    let want = sm_testkit::topo::sphere(2.0).expect("fixture").complex.canonical_hash();
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = session(factory);
        run(&mut s, &CreateSphere { radius: 2.0, center: None }).expect("sphere");
        assert_eq!(model(&s).topo().canonical_hash(), want);
    }
}

#[test]
fn brick_extents_default_to_x_and_center_shifts() {
    let mut s = session(cell_complex);
    run(&mut s, &CreateBrick { x: 2.0, y: Some(4.0), z: Some(6.0), center: Some([10.0, 20.0, 30.0]) }).expect("brick");
    let pts = vertex_points(&s);
    assert_eq!(pts[0], [9.0, 18.0, 27.0]);
    assert_eq!(pts[7], [11.0, 22.0, 33.0]);
    let t = model(&s).topo();
    let v = t.volume_moments(t.volumes()[0]).expect("moments").volume;
    assert!((v - 48.0).abs() < 1e-9 * 48.0, "{v}");

    let mut s = session(cell_complex);
    run(&mut s, &CreateBrick { x: 3.0, y: None, z: None, center: None }).expect("cube");
    let t = model(&s).topo();
    assert!((t.volume_moments(t.volumes()[0]).expect("moments").volume - 27.0).abs() < 1e-12);
}

#[test]
fn invalid_parameters_are_refused_and_leave_the_state_alone() {
    let mut s = session(cell_complex);
    let before = s.state().hash();
    for bad in [
        CreateBrick { x: 0.0, y: None, z: None, center: None },
        CreateBrick { x: -1.0, y: None, z: None, center: None },
        CreateBrick { x: f64::NAN, y: None, z: None, center: None },
        CreateBrick { x: 1.0, y: Some(f64::INFINITY), z: None, center: None },
        CreateBrick { x: 1.0, y: None, z: None, center: Some([0.0, f64::NAN, 0.0]) },
    ] {
        assert!(run(&mut s, &bad).is_err(), "{bad:?}");
        assert_eq!(s.state().hash(), before);
    }
    assert!(run(&mut s, &CreateCylinder { height: 1.0, radius: 0.0, center: None }).is_err());
    assert!(run(&mut s, &CreateSphere { radius: -2.0, center: None }).is_err());
    assert_eq!(s.state().hash(), before);
}

#[test]
fn overflowing_or_tiny_primitives_are_invalid_and_leave_the_state_alone() {
    let mut s = session(cell_complex);
    let before = s.state().hash();
    let m = f64::MAX;
    let ops: Vec<Box<dyn sm_op_api::Op>> = vec![
        // Each input is finite, the derived corner (`min + x`) is not.
        Box::new(CreateBrick { x: m, y: None, z: None, center: Some([m, 0.0, 0.0]) }),
        Box::new(CreateBrick { x: m, y: Some(1.0), z: Some(1.0), center: Some([-m, 0.0, 0.0]) }),
        Box::new(CreateCylinder { height: 1.0, radius: m, center: Some([m, 0.0, 0.0]) }),
        Box::new(CreateCylinder { height: m, radius: 1.0, center: Some([0.0, 0.0, m]) }),
        Box::new(CreateSphere { radius: m, center: Some([0.0, m, 0.0]) }),
        // Below the linear tolerance (1e-6) or subnormal.
        Box::new(CreateBrick { x: 5e-324, y: None, z: None, center: None }),
        Box::new(CreateBrick { x: 1.0, y: Some(1e-7), z: None, center: None }),
        Box::new(CreateCylinder { height: 1.0, radius: 1e-7, center: None }),
        Box::new(CreateSphere { radius: 1e-7, center: None }),
        // Representable only as a degenerate range next to the centre.
        Box::new(CreateBrick { x: 1e-30, y: None, z: None, center: Some([1.0, 0.0, 0.0]) }),
    ];
    for op in &ops {
        let err = run(&mut s, op.as_ref()).expect_err("refused");
        assert!(format!("{err:?}").contains("Invalid"), "{op:?}: {err:?}");
        assert_eq!(s.state().hash(), before, "{op:?}");
    }
    // Just above the tolerance still works.
    run(&mut s, &CreateBrick { x: 1e-5, y: None, z: None, center: None }).expect("small brick");
}

#[test]
fn cylinder_volume_and_topology() {
    let (r, h) = (1.5, 4.0);
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = session(factory);
        let (_, d) = run(&mut s, &CreateCylinder { height: h, radius: r, center: None }).expect("cylinder");
        let t = model(&s).topo();
        let counts: Vec<usize> = Dim::ALL.iter().map(|&d| t.cells(d).len()).collect();
        assert_eq!(counts, vec![2, 3, 3, 1, 1]);
        let v = t.volume_moments(t.volumes()[0]).expect("moments").volume;
        assert!((v - PI * r * r * h).abs() < 1e-9 * PI * r * r * h, "volume {v}");
        // Spans z = +-h/2.
        let pts = vertex_points(&s);
        assert_eq!(pts[0][2], -h / 2.0);
        assert_eq!(pts[1][2], h / 2.0);
        assert_eq!(d.outputs["faces"].len(), 3);
        assert_eq!(d.outputs["edges"].len(), 3);
        clean(&s);
    }
}

#[test]
fn sphere_volume_and_topology() {
    let r = 2.0;
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = session(factory);
        run(&mut s, &CreateSphere { radius: r, center: Some([1.0, 2.0, 3.0]) }).expect("sphere");
        let t = model(&s).topo();
        let counts: Vec<usize> = Dim::ALL.iter().map(|&d| t.cells(d).len()).collect();
        assert_eq!(counts, vec![2, 3, 1, 1, 1]);
        clean(&s);
    }
}

#[test]
fn schemas_declare_entity_kinds_outputs_and_positional() {
    let reg = registry(cell_complex);
    for name in ["model.create_brick", "model.create_cylinder", "model.create_sphere"] {
        assert!(!reg.op_outputs(name).is_empty(), "{name} declares OUTPUTS");
    }
    let schema = reg.schema("model.create_brick").expect("schema");
    let x = &schema.as_value()["properties"]["x"];
    assert_eq!(x["x-positional"], serde_json::json!(true));
}
