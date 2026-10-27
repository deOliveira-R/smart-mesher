//! Stretch S2: the STEP writer. For every fixture X: `T1 = write(X)`, `Y = read(T1)`,
//! `T2 = write(Y)`; `T1 == T2` byte for byte, `Y` hashes like `read(T2)`, and counts, volume and
//! area agree with `X`.

mod common;

use std::sync::Arc;

use common::*;
use sm_cadio::{ImportStep, LengthUnit, MemoryInputStore, Source, StepSchema, WriteOptions, export_step, read_step, ReadOptions, write_step};
use sm_layer_api::TopoHost;
use sm_topo_api::{Dim, TopologyView};

fn written(h: &dyn TopoHost, o: &WriteOptions) -> (Vec<u8>, sm_cadio::WriteReport) {
    let mut buf = Vec::new();
    let rep = write_step(h.as_view(), None, o, &mut buf).expect("write_step");
    (buf, rep)
}

fn imported(bytes: &[u8]) -> Box<dyn TopoHost> {
    import_into(cell_complex, &read(bytes)).0
}

fn counts(v: &dyn TopologyView) -> [usize; 5] {
    [Dim::Vertex, Dim::Edge, Dim::Face, Dim::Volume, Dim::Body].map(|d| v.cells(d).len())
}

fn all_volumes(v: &dyn TopologyView) -> f64 {
    v.cells(Dim::Volume).iter().map(|&x| volume(v, x)).sum()
}

fn fixtures() -> Vec<(String, Vec<u8>)> {
    let mut v: Vec<(String, Vec<u8>)> = sm_testkit::step::ALL.iter().map(|f| (f.name.to_owned(), f.bytes.to_vec())).collect();
    for n in [
        "brick_2x3x4.stp",
        "brick_ap203.stp",
        "brick_inch.stp",
        "two_bricks.stp",
        "brick_with_void.stp",
        "brick_with_void_inward.stp",
        "brick_inverted.stp",
        "frustum_r2_r1_h3.stp",
        "sphere_seam_r3.stp",
        "block_fillet_nurbs.stp",
        "assembly_pins.stp",
        "assembly_mapped.stp",
    ] {
        v.push((n.to_owned(), data(n)));
    }
    v
}

#[test]
fn every_fixture_round_trips_to_a_byte_identical_fixed_point() {
    let o = WriteOptions::default();
    for (name, bytes) in fixtures() {
        let x = imported(&bytes);
        let (t1, rep) = written(&*x, &o);
        assert_eq!(rep.bytes as usize, t1.len(), "{name}");
        let y = imported(&t1);
        let (t2, _) = written(&*y, &o);
        assert_eq!(String::from_utf8_lossy(&t1), String::from_utf8_lossy(&t2), "{name}: T1 == T2");
        assert_eq!(t1, t2, "{name}");
        let z = imported(&t2);
        assert_eq!(y.as_view().canonical_hash(), z.as_view().canonical_hash(), "{name}: canonical hash of Y and read(T2)");
        // Counts, volume and area against X (the pole edges are re-created, so counts match).
        let (cx, cy) = (counts(x.as_view()), counts(y.as_view()));
        assert_eq!(cx, cy, "{name}: counts");
        // The degree-13 loft makes every quadrature slow in debug builds: release runs it all.
        if name == "loft_bspline" && cfg!(debug_assertions) {
            continue;
        }
        let (vx, vy) = (all_volumes(x.as_view()), all_volumes(y.as_view()));
        assert!(vx == vy || rel(vy, vx) <= 1e-12, "{name}: volume {vx} vs {vy}");
        let (ax, ay) = (total_area(x.as_view()), total_area(y.as_view()));
        assert!(rel(ay, ax) <= 1e-12, "{name}: area {ax} vs {ay}");
        assert!(issues(y.as_view()).is_empty(), "{name}: Y is valid");
    }
}

#[test]
fn the_nurbs_block_is_a_fixed_point_with_its_rational_forms() {
    let x = imported(&data("block_fillet_nurbs.stp"));
    let (t1, _) = written(&*x, &WriteOptions::default());
    let text = String::from_utf8(t1.clone()).expect("ascii");
    assert!(text.contains("RATIONAL_B_SPLINE_SURFACE") && text.contains("RATIONAL_B_SPLINE_CURVE"), "rational complex forms");
    let y = imported(&t1);
    let (t2, _) = written(&*y, &WriteOptions::default());
    assert_eq!(t1, t2);
    // The weights survive bit for bit.
    let m0 = read(&data("block_fillet_nurbs.stp"));
    let m1 = read(&t1);
    let w = |m: &sm_cadio::BrepModel| m.solids[0].surfaces.iter().find_map(|s| match &s.def {
        sm_geom_api::SurfaceDef::Nurbs(n) => n.weights.clone(),
        _ => None,
    });
    assert_eq!(w(&m0), w(&m1));
    assert!(w(&m0).is_some());
}

#[test]
fn output_is_reproducible_and_declares_what_it_is() {
    let x = imported(&data("brick_2x3x4.stp"));
    let (a, _) = written(&*x, &WriteOptions::default());
    let (b, _) = written(&*x, &WriteOptions::default());
    assert_eq!(a, b);
    let text = String::from_utf8(a).expect("ascii");
    assert!(text.starts_with("ISO-10303-21;\nHEADER;"));
    assert!(text.contains("'1970-01-01T00:00:00'"), "fixed default time stamp");
    assert!(text.contains("AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }"));
    assert!(text.contains("SI_UNIT(.MILLI.,.METRE.)"));
    assert!(text.trim_end().ends_with("END-ISO-10303-21;"));
    assert_eq!(text.matches("MANIFOLD_SOLID_BREP").count(), 1);
    // Every real has a point or an exponent after a digit run: no bare integers as reals.
    assert!(!text.contains(",1,0,0"), "directions are written with decimal points");
    let o = WriteOptions { schema: StepSchema::Ap242, unit: LengthUnit::In, timestamp: Some("2026-01-02T03:04:05".into()) };
    let (c, _) = written(&*x, &o);
    let t = String::from_utf8(c.clone()).expect("ascii");
    assert!(t.contains("AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF") && t.contains("CONVERSION_BASED_UNIT('INCH'") && t.contains("2026-01-02T03:04:05"));
    // AP242 and inches read back as the same coordinates, declared in inches (no scaling).
    let m = read_step(&c, &ReadOptions::default()).expect("read");
    assert_eq!(m.solids[0].units.length_name, "inch");
    assert_eq!(m.solids[0].vertices.iter().map(|v| v.point).collect::<Vec<_>>(), read(&data("brick_2x3x4.stp")).solids[0].vertices.iter().map(|v| v.point).collect::<Vec<_>>());
}

#[test]
fn shared_faces_are_written_once_per_volume_and_reported() {
    // Two touching bricks merged by hand would need sm-boolean; the stage-0 fixture builds them.
    let mut cc = sm_topo_api::CellComplex::new();
    sm_topo_api::fixtures::with_op(&mut cc, sm_topo_api::OpId(1), |t| {
        let (a, b) = sm_topo_api::fixtures::two_bricks(t)?;
        sm_topo_api::fixtures::merge_touching_bricks(t, &a, &b)
    })
    .expect("merged bricks");
    let mut buf = Vec::new();
    let rep = write_step(&cc, None, &WriteOptions::default(), &mut buf).expect("write");
    assert_eq!((rep.solids, rep.faces), (2, 12), "11 faces, the shared one twice");
    assert_eq!(rep.losses.iter().filter(|l| l.what.contains("shared by several volumes")).count(), 2);
    // Reading it back gives two valid solids of volume 1 each.
    let m = read(&buf);
    assert_eq!(m.solids.len(), 2);
    let (h, r) = import_into(cell_complex, &m);
    assert!(issues(h.as_view()).is_empty());
    for v in &r.volumes {
        assert!(rel(volume(h.as_view(), *v), 1.0) <= 1e-9);
    }
}

#[test]
fn bodies_can_be_selected_and_unsupported_geometry_is_named() {
    let x = imported(&data("two_bricks.stp"));
    let bodies: Vec<_> = x.as_view().cells(Dim::Body).to_vec();
    let mut buf = Vec::new();
    let rep = write_step(x.as_view(), Some(&bodies[1..]), &WriteOptions::default(), &mut buf).expect("write one body");
    assert_eq!(rep.solids, 1);
    assert!(rel(volume(imported(&buf).as_view(), imported(&buf).as_view().cells(Dim::Volume)[0]), 6.0) <= 1e-9);
    let faces = x.as_view().cells(Dim::Face).to_vec();
    let e = write_step(x.as_view(), Some(&faces[..1]), &WriteOptions::default(), &mut Vec::new()).expect_err("a face is not a body");
    assert!(matches!(e, sm_cadio::StepError::Export(_)), "{e:?}");
    // An implicit surface cannot be written, and the error names the face.
    let mut cc = sm_topo_api::CellComplex::new();
    use sm_topo_api::TopologyEdit;
    cc.begin_op(sm_topo_api::OpId(1)).expect("begin");
    let ids = sm_topo_api::fixtures::brick(&mut cc, sm_core::Point3::new(0.0, 0.0, 0.0), [1.0; 3]).expect("brick");
    let q = sm_geom_api::SurfaceDef::Implicit(sm_geom_api::ImplicitDef::Quadric(sm_geom_api::Quadric::sphere(&sm_geom_api::Frame::at(sm_core::Point3::new(0.0, 0.0, 0.0)), 1.0)));
    let h = cc.shapes_mut().expect("shapes").add_surface(q).expect("add");
    cc.set_carrier(ids.faces[0], sm_topo_api::Carrier::Surface(sm_topo_api::SurfaceCarrier { surface: h, sense: sm_topo_api::Sense::Forward })).expect("carrier");
    cc.end_op().expect("end");
    let e = write_step(&cc, None, &WriteOptions::default(), &mut Vec::new()).expect_err("implicit");
    let sm_cadio::StepError::Unsupported { entity, .. } = &e else { panic!("{e:?}") };
    assert!(entity.contains(&ids.faces[0].to_string()), "{entity}");
    // Nothing to write.
    let empty = sm_topo_api::CellComplex::new();
    assert!(write_step(&empty, None, &WriteOptions::default(), &mut Vec::new()).is_err());
}

#[test]
fn a_session_state_exports_like_its_topology() {
    let st = Arc::new(MemoryInputStore::new());
    let mut s = session(topo_store, Arc::clone(&st));
    let op = ImportStep { source: Source::inline(&data("brick_with_void.stp")), name: None, to_unit: None, assume_unit: None, weld_tol: None, solids: None, sheets: None, check: None };
    run(&mut s, &op).expect("import");
    let mut a = Vec::new();
    export_step(s.state(), None, &WriteOptions::default(), &mut a).expect("export");
    let mut b = Vec::new();
    write_step(model(&s).topo(), None, &WriteOptions::default(), &mut b).expect("write");
    assert_eq!(a, b);
    // The exported file imports into a fresh session with the same model hash.
    let mut s2 = session(cell_complex, st);
    let op2 = ImportStep { source: Source::inline(&a), name: None, to_unit: None, assume_unit: None, weld_tol: None, solids: None, sheets: None, check: None };
    run(&mut s2, &op2).expect("re-import");
    assert_eq!(model(&s2).topo().canonical_hash(), model(&s).topo().canonical_hash());
}
