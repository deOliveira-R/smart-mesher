//! Acceptance 9 (op forms), 10 and 14: the op and the query through a `Session`
//! (`CoreLayers` + `CadioPlugin` + `StepInputs`).

mod common;

use std::sync::Arc;

use common::*;
use serde_json::json;
use sm_cadio::{CadioPlugin, ImportStep, LengthUnit, MAX_INLINE_BYTES, MemoryInputStore, Source, StepInfo, sha256_hex};
use sm_core::EntityKind;
use sm_layer_api::ModelLayer;
use sm_op_api::{ApplyError, DerivedCache, OpError, OpRecord, Plugin, QuietControl, Registry, Session, TimelineError};
use sm_topo_api::Dim;

fn op(source: Source) -> ImportStep {
    ImportStep { source, name: None, to_unit: None, assume_unit: None, weld_tol: None, solids: None, sheets: None, check: None }
}

fn store() -> Arc<MemoryInputStore> {
    Arc::new(MemoryInputStore::new())
}

fn op_error(e: TimelineError) -> OpError {
    match e {
        TimelineError::Apply(ApplyError::Op { source, .. }) => source,
        other => panic!("not an op error: {other:?}"),
    }
}

fn invalid_msg(e: TimelineError) -> String {
    match op_error(e) {
        OpError::Invalid(m) => m,
        other => panic!("not Invalid: {other:?}"),
    }
}

#[test]
fn plugin_registers_the_op_and_the_query_but_no_layers() {
    let mut reg = Registry::new();
    reg.install(&CadioPlugin).expect("install");
    assert_eq!(CadioPlugin.id(), "cadio");
    let ops: Vec<&str> = reg.op_names().into_iter().filter(|n| !n.starts_with("core.")).collect();
    assert_eq!(ops, ["model.import_step"]);
    assert_eq!(reg.query_names(), ["cad.step_info"]);
    assert!(reg.layer_roles().is_empty(), "layers belong to CoreLayers");
}

#[test]
fn inline_and_sha256_forms_give_the_same_state_hash() {
    for (bytes, name) in [(data("brick_2x3x4.stp"), "brick"), (sm_testkit::w3a::STEP.to_vec(), "w3a")] {
        let st = store();
        let r = st.put(&bytes);
        let mut hashes = Vec::new();
        for factory in [cell_complex as Factory, topo_store] {
            for source in [Source::inline(&bytes), Source::sha256(r.sha256.clone())] {
                let mut s = session(factory, Arc::clone(&st));
                run(&mut s, &op(source)).expect("import");
                hashes.push((s.state().hash(), s.state().layer_hash("model").expect("model layer")));
            }
        }
        assert!(hashes.windows(2).all(|w| w[0] == w[1]), "{name}: inline/sha256 and both hosts agree: {hashes:?}");
    }
}

#[test]
fn milestone_part_through_the_op() {
    let st = store();
    let r = st.put(sm_testkit::w3a::STEP);
    assert_eq!(r.sha256, sm_testkit::step::W3A_PART.sha256, "the pinned testkit hash");
    let mut s = session(cell_complex, st);
    let (_, delta) = run(&mut s, &op(Source::sha256(r.sha256))).expect("import");
    assert_eq!(delta.outputs["bodies"].len(), 3);
    assert_eq!(delta.outputs["faces"].len(), 19);
    let t = model(&s).topo();
    for (i, want) in sm_testkit::w3a::VOLUMES.iter().enumerate() {
        let v = delta.outputs["volumes"][i];
        assert!(rel(volume(t, v), *want) <= 1e-12, "volume {i}");
    }
    assert!(issues(t).is_empty());
    assert!(delta.diagnostics.iter().all(|d| d.code != "step.unsupported_entity"));
    // The three placements are identity: flattened, no warning.
    assert!(delta.diagnostics.iter().any(|d| d.code == "step.assembly_flattened"));
    assert!(delta.diagnostics.iter().all(|d| d.code != "step.assembly_transforms_ignored"));
}

#[test]
fn delta_lists_topology_ids_ascending_and_outputs_match_the_roles() {
    let mut s = session(topo_store, store());
    let (_, d) = run(&mut s, &op(Source::inline(&data("brick_2x3x4.stp")))).expect("import");
    assert!(d.created.windows(2).all(|w| w[0] < w[1]), "created ascending: {:?}", d.created);
    assert_eq!(d.created.len(), 8 + 12 + 6 + 1 + 1);
    let roles: Vec<&str> = d.outputs.keys().map(String::as_str).collect();
    assert_eq!(roles, ["bodies", "edges", "faces", "vertices", "volumes"]);
    let kinds = [("bodies", EntityKind::TopoBody), ("edges", EntityKind::TopoEdge), ("faces", EntityKind::TopoFace), ("vertices", EntityKind::TopoVertex), ("volumes", EntityKind::TopoVolume)];
    let mut union = Vec::new();
    for (role, kind) in kinds {
        let ids = &d.outputs[role];
        assert!(ids.iter().all(|i| i.kind() == kind), "{role}");
        assert!(ids.windows(2).all(|w| w[0] < w[1]), "{role} ascending");
        union.extend(ids.iter().copied());
    }
    union.sort_unstable();
    assert_eq!(union, d.created, "every created id sits in exactly one role");
    // Volume equals the role's id and the cells really are in the model.
    assert!(rel(volume(model(&s).topo(), d.outputs["volumes"][0]), 24.0) <= 1e-9);
}

#[test]
fn undo_redo_and_cold_replay_do_not_diverge() {
    let st = store();
    let r = st.put(&data("two_bricks.stp"));
    let mut s = Session::new(engine(cell_complex, Arc::clone(&st), DerivedCache::disabled()));
    let (h1, _) = run(&mut s, &op(Source::sha256(r.sha256.clone()))).expect("import");
    let mut second = op(Source::inline(&data("brick_inch.stp")));
    second.to_unit = Some(LengthUnit::Mm);
    let (h2, _) = run(&mut s, &second).expect("second import");
    let full = s.state().hash();
    s.undo().expect("undo");
    assert_ne!(s.state().hash(), full);
    s.undo().expect("undo");
    assert_eq!(s.state().entity_count(), 0);
    s.redo().expect("redo");
    s.redo().expect("redo");
    assert_eq!(s.state().hash(), full);
    let q = QuietControl::new();
    let cold = s.state_at_from_root(h2, &q.control()).expect("cold replay of both imports: no Diverged");
    assert_eq!(cold.hash(), full);
    let mid = s.state_at_from_root(h1, &q.control()).expect("cold replay of the first");
    assert_eq!(mid.entity_count(), 2 * (8 + 12 + 6 + 1 + 1));
}

#[test]
fn failures_are_invalid_and_leave_the_state_untouched() {
    let st = store();
    let mut s = session(cell_complex, Arc::clone(&st));
    run(&mut s, &op(Source::inline(&data("brick_2x3x4.stp")))).expect("seed");
    let before = (s.state().hash(), s.state().layer_hash("model"), s.timeline().len());

    // A blob nobody staged.
    let missing = "0".repeat(64);
    let m = invalid_msg(run(&mut s, &op(Source::sha256(missing.clone()))).expect_err("missing"));
    assert!(m.contains("step.input_missing") && m.contains(&missing) && m.contains("stage it first"), "{m}");

    // A blob whose content does not hash to its name.
    let fake = "ab".repeat(32);
    st.put_unchecked(&fake, b"not what the hash says");
    let m = invalid_msg(run(&mut s, &op(Source::sha256(fake.clone()))).expect_err("mismatch"));
    assert!(m.contains("step.input_mismatch") && m.contains(&sha256_hex(b"not what the hash says")), "{m}");

    // A malformed name.
    let m = invalid_msg(run(&mut s, &op(Source::sha256("XYZ"))).expect_err("bad hash"));
    assert!(m.contains("step.bad_sha256"), "{m}");

    // An inline file over the cap, and invalid base64.
    let big = vec![b' '; MAX_INLINE_BYTES + 1];
    let m = invalid_msg(run(&mut s, &op(Source::inline(&big))).expect_err("oversize"));
    assert!(m.contains("step.inline"), "{m}");
    let m = invalid_msg(run(&mut s, &op(Source::Inline(sm_cadio::InlineSource { inline: "***".into() }))).expect_err("base64"));
    assert!(m.contains("step.inline"), "{m}");

    // Not a STEP file, and a file with a broken shell.
    let m = invalid_msg(run(&mut s, &op(Source::inline(b"hello"))).expect_err("syntax"));
    assert!(m.contains("step.syntax"), "{m}");
    let m = invalid_msg(run(&mut s, &op(Source::inline(&bad("open_shell.stp")))).expect_err("open shell"));
    assert!(m.contains("step.open_shell"), "{m}");

    // Bad op parameters.
    let mut o = op(Source::inline(&data("brick_2x3x4.stp")));
    o.weld_tol = Some(1.0);
    assert!(matches!(op_error(run(&mut s, &o).expect_err("weld_tol above linear_max")), OpError::Invalid(_)));
    let mut o = op(Source::inline(&data("brick_2x3x4.stp")));
    o.solids = Some(vec![5]);
    assert!(matches!(op_error(run(&mut s, &o).expect_err("ordinal")), OpError::Invalid(_)));

    assert_eq!((s.state().hash(), s.state().layer_hash("model"), s.timeline().len()), before, "atomic: nothing changed");
}

#[test]
fn a_missing_input_service_is_reported_only_for_sha256_sources() {
    let engine = sm_op_api::Engine::new(Arc::new(registry(cell_complex)), sm_op_api::Services::new(), Arc::new(DerivedCache::default())).expect("engine");
    let mut s = Session::new(engine);
    run(&mut s, &op(Source::inline(&data("brick_2x3x4.stp")))).expect("inline needs no service");
    let e = run(&mut s, &op(Source::sha256("0".repeat(64)))).expect_err("no service");
    assert!(matches!(op_error(e), OpError::MissingService(_)));
}

#[test]
fn solid_ordinals_select_solids() {
    let mut s = session(cell_complex, store());
    let mut o = op(Source::inline(&data("two_bricks.stp")));
    o.solids = Some(vec![1]);
    let (_, d) = run(&mut s, &o).expect("import");
    assert_eq!(d.outputs["bodies"].len(), 1);
    let t = model(&s).topo();
    assert!(rel(volume(t, d.outputs["volumes"][0]), 6.0) <= 1e-9, "the second solid has volume 6");
    assert_eq!(t.cells(Dim::Face).len(), 6);
}

#[test]
fn units_are_converted_reported_and_defaulted() {
    let mut s = session(cell_complex, store());
    let mut o = op(Source::inline(&data("brick_inch.stp")));
    o.to_unit = Some(LengthUnit::Mm);
    let (_, d) = run(&mut s, &o).expect("import");
    assert!(rel(volume(model(&s).topo(), d.outputs["volumes"][0]), 98_322.384) <= 1e-9);
    let u = d.diagnostics.iter().find(|x| x.code == "step.units").expect("units info");
    assert!(u.message.contains("inch") && u.message.contains("mm"), "{}", u.message);

    // No to_unit: kept and reported.
    let mut s = session(cell_complex, store());
    let (_, d) = run(&mut s, &op(Source::inline(&data("brick_inch.stp")))).expect("import");
    assert!(rel(volume(model(&s).topo(), d.outputs["volumes"][0]), 6.0) <= 1e-9);

    // A file without units: assumed mm (the default) with a warning.
    let mut s = session(cell_complex, store());
    let (_, d) = run(&mut s, &op(Source::inline(&bad("no_units.stp")))).expect("assumed mm");
    assert!(d.diagnostics.iter().any(|x| x.code == "step.units" && x.severity == sm_op_api::Severity::Warning && x.message.contains("assuming mm")));
    let mut s = session(cell_complex, store());
    let mut o = op(Source::inline(&bad("no_units.stp")));
    o.assume_unit = Some(LengthUnit::In);
    o.to_unit = Some(LengthUnit::Mm);
    let (_, d) = run(&mut s, &o).expect("assumed inch");
    assert!(rel(volume(model(&s).topo(), d.outputs["volumes"][0]), 24.0 * 25.4f64.powi(3)) <= 1e-9);
}

#[test]
fn inverted_solids_are_flipped_with_a_warning() {
    let mut s = session(cell_complex, store());
    let (_, d) = run(&mut s, &op(Source::inline(&data("brick_inverted.stp")))).expect("import");
    let w = d.diagnostics.iter().find(|x| x.code == "step.inverted_solid").expect("inverted warning");
    assert_eq!(w.severity, sm_op_api::Severity::Warning);
    assert_eq!(w.entities, d.outputs["bodies"]);
    let t = model(&s).topo();
    assert!(rel(volume(t, d.outputs["volumes"][0]), 24.0) <= 1e-9, "positive after the flip");
    assert!(issues(t).is_empty());
    // The flipped solid is the plain brick again.
    let mut s2 = session(cell_complex, store());
    run(&mut s2, &op(Source::inline(&data("brick_2x3x4.stp")))).expect("plain");
    assert_eq!(model(&s2).topo().cells(Dim::Face).len(), t.cells(Dim::Face).len());
}

#[test]
fn check_fails_the_op_on_validator_errors_and_can_be_switched_off() {
    let bytes = bad("face_off_surface.stp");
    let mut s = session(cell_complex, store());
    let before = s.state().hash();
    let m = invalid_msg(run(&mut s, &op(Source::inline(&bytes))).expect_err("validate"));
    assert!(m.contains("step.validate"), "{m}");
    assert!(m.contains("GeometryMismatch"), "located issues are listed: {m}");
    assert_eq!(s.state().hash(), before);
    let mut o = op(Source::inline(&bytes));
    o.check = Some(false);
    run(&mut s, &o).expect("check: false imports it");
}

#[test]
fn the_query_reports_each_fixture() {
    let st = store();
    let q = |s: &Session, src: &Source| -> StepInfo {
        let v = common_query(s, json!({ "source": serde_json::to_value(src).expect("source") }));
        serde_json::from_value(v).expect("StepInfo")
    };
    let s = session(cell_complex, Arc::clone(&st));
    // (name, bytes, [(faces, shells, edges, vertices, kinds)])
    type Row = (usize, usize, usize, usize, &'static [(&'static str, usize)]);
    let cases: Vec<(&str, Vec<u8>, Vec<Row>)> = vec![
        ("brick_2x3x4", data("brick_2x3x4.stp"), vec![(6, 1, 12, 8, &[("plane", 6)])]),
        ("brick_ap203", data("brick_ap203.stp"), vec![(6, 1, 12, 8, &[("plane", 6)])]),
        ("brick_inch", data("brick_inch.stp"), vec![(6, 1, 12, 8, &[("plane", 6)])]),
        ("two_bricks", data("two_bricks.stp"), vec![(6, 1, 12, 8, &[("plane", 6)]), (6, 1, 12, 8, &[("plane", 6)])]),
        ("brick_with_void", data("brick_with_void.stp"), vec![(12, 2, 24, 16, &[("plane", 12)])]),
        ("frustum", data("frustum_r2_r1_h3.stp"), vec![(3, 1, 3, 2, &[("cone", 1), ("plane", 2)])]),
        ("sphere_seam", data("sphere_seam_r3.stp"), vec![(1, 1, 1, 2, &[("sphere", 1)])]),
        ("fillet_nurbs", data("block_fillet_nurbs.stp"), vec![(7, 1, 15, 10, &[("nurbs", 1), ("plane", 6)])]),
        ("cylinder", sm_testkit::step::CYLINDER_R2_H5.bytes.to_vec(), vec![(3, 1, 3, 2, &[("cylinder", 1), ("plane", 2)])]),
        ("torus", sm_testkit::step::TORUS_R5_R1.bytes.to_vec(), vec![(1, 1, 2, 1, &[("torus", 1)])]),
        ("cone", sm_testkit::step::CONE_R2_H3.bytes.to_vec(), vec![(2, 1, 2, 2, &[("cone", 1), ("plane", 1)])]),
        ("sphere_occ", sm_testkit::step::SPHERE_R3.bytes.to_vec(), vec![(1, 1, 0, 1, &[("sphere", 1)])]),
        ("box_fillet1", sm_testkit::step::BOX_FILLET1.bytes.to_vec(), vec![(7, 1, 15, 10, &[("cylinder", 1), ("plane", 6)])]),
        ("loft", sm_testkit::step::LOFT_BSPLINE.bytes.to_vec(), vec![(3, 1, 3, 2, &[("nurbs", 1), ("plane", 2)])]),
        ("w3a", sm_testkit::w3a::STEP.to_vec(), vec![(8, 1, 18, 12, &[("plane", 8)]), (5, 1, 9, 6, &[("plane", 5)]), (6, 1, 12, 8, &[("plane", 6)])]),
    ];
    for (name, bytes, rows) in cases {
        for src in [Source::inline(&bytes), Source::sha256(st.put(&bytes).sha256)] {
            let info = q(&s, &src);
            assert_eq!(info.sha256, sha256_hex(&bytes), "{name}");
            assert_eq!(info.bytes as usize, bytes.len());
            assert_eq!(info.solids.len(), rows.len(), "{name}");
            for (got, (faces, shells, edges, vertices, kinds)) in info.solids.iter().zip(&rows) {
                assert_eq!((got.faces, got.shells, got.edges, got.vertices), (*faces, *shells, *edges, *vertices), "{name}: {got:?}");
                let want: std::collections::BTreeMap<String, usize> = kinds.iter().map(|(k, c)| ((*k).to_owned(), *c)).collect();
                assert_eq!(got.surface_kinds, want, "{name}");
            }
            assert_eq!(info.length_unit.as_deref().map(|u| u.contains("metre") || u.contains("inch")), Some(true), "{name}: {:?}", info.length_unit);
            assert!(info.unsupported.iter().all(|u| u.entity != "PLANE"), "{name}");
        }
    }
    let w = q(&s, &Source::inline(sm_testkit::w3a::STEP));
    assert!(w.assemblies && w.schema == "AP214" && w.uncertainty == Some(1e-7));
    assert_eq!(w.length_unit.as_deref(), Some("millimetre"));
    let b = q(&s, &Source::inline(&data("brick_ap203.stp")));
    assert_eq!(b.schema, "AP203");
    assert!(!b.assemblies);
    let v = q(&s, &Source::inline(&data("brick_with_void.stp")));
    assert_eq!(v.solids[0].kind, "with_voids");
    let i = q(&s, &Source::inline(&data("brick_inch.stp")));
    assert_eq!(i.length_unit.as_deref(), Some("inch"));
    assert!((i.length_unit_um.expect("um") - 25_400.0).abs() < 1e-9);
    // Unsupported entities are listed instead of failing.
    let u = q(&s, &Source::inline(&bad("unsupported_offset_surface.stp")));
    assert_eq!(u.unsupported.iter().map(|x| (x.entity.as_str(), x.count)).collect::<Vec<_>>(), [("OFFSET_SURFACE", 1)]);
}

fn common_query(s: &Session, input: serde_json::Value) -> serde_json::Value {
    let q = QuietControl::new();
    s.engine().run_query(s.state(), "cad.step_info", &input, &q.control()).expect("query")
}

#[test]
fn the_query_fails_cleanly_and_never_changes_the_state() {
    let s = session(cell_complex, store());
    let before = s.state().hash();
    let q = QuietControl::new();
    let e = s.engine().run_query(s.state(), "cad.step_info", &json!({"source": {"sha256": "0".repeat(64)}}), &q.control()).expect_err("missing blob");
    assert!(e.to_string().contains("step.input_missing"), "{e}");
    let e = s.engine().run_query(s.state(), "cad.step_info", &json!({"source": {"inline": "aGVsbG8="}}), &q.control()).expect_err("not step");
    assert!(e.to_string().contains("step.syntax"), "{e}");
    assert_eq!(s.state().hash(), before);
}

#[test]
fn json_records_decode_and_reject_paths_and_unknown_fields() {
    let s = session(cell_complex, store());
    let reg = s.engine().registry();
    let good = OpRecord::new("model.import_step", 1, json!({"source": {"sha256": "a".repeat(64)}, "to_unit": "mm", "weld_tol": 1e-5, "solids": [0, 1], "check": false})).expect("record");
    let decoded = reg.decode(&good).expect("decodes");
    let enc = decoded.encode().expect("encode");
    assert_eq!(enc["to_unit"], "mm");
    assert_eq!(enc["source"], json!({"sha256": "a".repeat(64)}));
    for bad_params in [
        json!({"path": "/tmp/x.step"}),
        json!({"source": {"path": "/tmp/x.step"}}),
        json!({"source": {"sha256": "a".repeat(64), "inline": "AA=="}}),
        json!({"source": {"inline": "AA=="}, "bogus": 1}),
        json!({"source": {"inline": "AA=="}, "to_unit": "parsec"}),
        json!({}),
    ] {
        let rec = OpRecord::new("model.import_step", 1, bad_params.clone()).expect("record");
        assert!(reg.decode(&rec).is_err(), "must reject {bad_params}");
    }
    // The model layer is the one CoreLayers registered: cadio adds no layer.
    assert!(s.state().layer::<ModelLayer>().is_some());
}

#[test]
fn s4_sheets_and_faceted_solids_through_the_op_and_the_query() {
    // Faceted solid: imported like any other, reported by the query.
    let mut s = session(cell_complex, store());
    let (_, d) = run(&mut s, &op(Source::inline(&data("brick_faceted_dup.stp")))).expect("faceted");
    assert!(rel(volume(model(&s).topo(), d.outputs["volumes"][0]), 24.0) <= 1e-9);
    let info: StepInfo = serde_json::from_value(common_query(&s, json!({"source": serde_json::to_value(Source::inline(&data("brick_faceted.stp"))).expect("source")}))).expect("info");
    assert_eq!((info.solids[0].kind.as_str(), info.solids[0].faces, info.solids[0].edges, info.solids[0].vertices), ("faceted", 6, 12, 8));
    assert!(info.unsupported.is_empty());
    // Sheets: refused by default (nothing to import), imported on request without volumes.
    let mut s = session(cell_complex, store());
    let sheet = data("sheet_open_box.stp");
    let m = invalid_msg(run(&mut s, &op(Source::inline(&sheet))).expect_err("no solids"));
    assert!(m.contains("step.no_solids"), "{m}");
    let mut o = op(Source::inline(&sheet));
    o.sheets = Some(true);
    let (_, d) = run(&mut s, &o).expect("sheets");
    assert_eq!(d.outputs["bodies"].len(), 1);
    assert_eq!(d.outputs["faces"].len(), 5);
    assert!(d.outputs.get("volumes").is_none_or(|v| v.is_empty()));
    assert!(rel(total_area(model(&s).topo()), 46.0) <= 1e-9);
    assert!(issues(model(&s).topo()).is_empty());
}
