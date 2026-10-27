//! Reader rules: welding, edge repair and gaps, trimmed curves, option errors.

mod common;

use common::*;
use sm_cadio::{NoteCode, NoteSeverity, ReadOptions, StepError, read_step};

fn text(name: &str) -> String {
    String::from_utf8(data(name)).expect("ascii fixture")
}

fn opts() -> ReadOptions {
    ReadOptions::default()
}

#[test]
fn an_unstitched_face_soup_is_welded_into_a_brick() {
    let m = read_step(&data("brick_unstitched.stp"), &opts()).expect("read");
    let s = &m.solids[0];
    assert_eq!((s.vertices.len(), s.edges.len(), s.faces.len()), (8, 12, 6));
    // 24 vertices became 8; 24 edges became 12, each recording the merged duplicate.
    let note = m.notes.iter().find(|n| n.code == NoteCode::Welded).expect("step.welded");
    assert_eq!(note.severity, NoteSeverity::Info);
    assert!(note.message.contains("welded 16 vertices and 12 duplicate edges"), "{}", note.message);
    assert_eq!(s.edges.iter().map(|e| e.merged.len()).sum::<usize>(), 12);
    // The lowest STEP id wins.
    assert!(s.vertices.windows(2).all(|w| w[0].id < w[1].id));
    let (h, r) = import_into(cell_complex, &m);
    assert!(issues(h.as_view()).is_empty());
    assert!(rel(volume(h.as_view(), r.volumes[0]), 24.0) <= 1e-9);
    assert!(rel(total_area(h.as_view()), 52.0) <= 1e-9);
}

#[test]
fn without_welding_the_soup_is_an_open_shell() {
    let e = read_step(&data("brick_unstitched.stp"), &ReadOptions { weld_tol: 0.0, ..opts() }).expect_err("not welded");
    assert!(matches!(e, StepError::OpenShell { .. }), "{e:?}");
    // A plain brick does not need welding.
    read_step(&data("brick_2x3x4.stp"), &ReadOptions { weld_tol: 0.0, ..opts() }).expect("stitched file");
}

#[test]
fn a_vertex_off_its_curve_is_a_warning_inside_the_uncertainty_and_an_error_beyond() {
    let base = text("brick_2x3x4.stp").replace("LENGTH_MEASURE(1.E-07)", "LENGTH_MEASURE(1.E-05)");
    // 3e-5 is above the weld tolerance (1e-6) and inside 10 x the uncertainty (1e-4).
    let near = base.replace("#6 = CARTESIAN_POINT('',(1.,2.,3.));", "#6 = CARTESIAN_POINT('',(1.,2.,3.00003));");
    assert_ne!(near, base);
    let m = read_step(near.as_bytes(), &opts()).expect("within the uncertainty");
    let gaps: Vec<_> = m.notes.iter().filter(|n| n.code == NoteCode::EdgeGap).collect();
    assert_eq!(gaps.len(), 2, "the two edges that leave the displaced vertex sideways");
    assert!(gaps.iter().all(|g| g.severity == NoteSeverity::Warning && g.message.contains("edge #")));
    // 5e-4 is beyond it.
    let far = base.replace("#6 = CARTESIAN_POINT('',(1.,2.,3.));", "#6 = CARTESIAN_POINT('',(1.,2.,3.0005));");
    let e = read_step(far.as_bytes(), &opts()).expect_err("beyond the limit");
    assert!(matches!(e, StepError::GeometryMismatch { .. }), "{e:?}");
    assert!(e.to_string().contains('#'), "names the edge: {e}");
}

#[test]
fn a_wrong_same_sense_flag_is_repaired_from_the_vertices() {
    let t = text("brick_2x3x4.stp");
    let flipped = t.replacen("#26 = EDGE_CURVE('',#7,#9,#25,.T.);", "#26 = EDGE_CURVE('',#7,#9,#25,.F.);", 1);
    assert_ne!(flipped, t);
    let m = read_step(flipped.as_bytes(), &opts()).expect("repairable");
    let n = m.notes.iter().find(|n| n.code == NoteCode::EdgeSense).expect("step.edge_sense");
    assert!(n.message.contains("#26"), "{}", n.message);
    let (h, r) = import_into(cell_complex, &m);
    assert!(rel(volume(h.as_view(), r.volumes[0]), 24.0) <= 1e-9);
    // The repaired model is the same model.
    let plain = read(&data("brick_2x3x4.stp"));
    assert_eq!(m.solids[0].edges, plain.solids[0].edges);
}

#[test]
fn a_trimmed_curve_is_its_basis_curve() {
    let t = text("brick_2x3x4.stp");
    let with = t
        .replacen("#26 = EDGE_CURVE('',#7,#9,#25,.T.);", "#26 = EDGE_CURVE('',#7,#9,#900,.T.);", 1)
        .replacen("ENDSEC;\nEND-ISO", "#900 = TRIMMED_CURVE('',#25,(#6),(#8),.T.,.CARTESIAN.);\nENDSEC;\nEND-ISO", 1);
    let m = read_step(with.as_bytes(), &opts()).expect("trimmed curve");
    let plain = read(&data("brick_2x3x4.stp"));
    // The same edges and curves (the trimmed one is listed last: curves ascend by entity id).
    let view = |m: &sm_cadio::BrepModel| m.solids[0].edges.iter().map(|e| (e.id, e.start, e.end, e.range, e.sense, m.solids[0].curves[e.curve].def.clone())).collect::<Vec<_>>();
    assert_eq!(view(&m), view(&plain));
    // A trimmed curve that trims itself is a cycle, not a stack overflow.
    let cyc = t.replacen("#26 = EDGE_CURVE('',#7,#9,#25,.T.);", "#26 = EDGE_CURVE('',#7,#9,#900,.T.);", 1).replacen("ENDSEC;\nEND-ISO", "#900 = TRIMMED_CURVE('',#901,(#6),(#8),.T.,.CARTESIAN.);\n#901 = TRIMMED_CURVE('',#900,(#6),(#8),.T.,.CARTESIAN.);\nENDSEC;\nEND-ISO", 1);
    assert!(matches!(read_step(cyc.as_bytes(), &opts()), Err(StepError::Cycle { .. })));
}

#[test]
fn bad_options_are_typed_errors() {
    let b = data("two_bricks.stp");
    let e = read_step(&b, &ReadOptions { solids: Some(vec![2]), ..opts() }).expect_err("ordinal");
    assert!(matches!(e, StepError::BadOption(ref m) if m.contains("ordinal 2")), "{e:?}");
    let e = read_step(&b, &ReadOptions { weld_tol: f64::NAN, ..opts() }).expect_err("tolerance");
    assert!(matches!(e, StepError::BadOption(_)), "{e:?}");
    assert_eq!(e.code(), "step.bad_option");
    // Duplicate and unordered ordinals select each solid once, in file order.
    let m = read_step(&b, &ReadOptions { solids: Some(vec![1, 0, 1]), ..opts() }).expect("read");
    assert_eq!(m.solids.len(), 2);
    assert!(m.solids[0].id < m.solids[1].id);
}

#[test]
fn sheets_are_skipped_with_a_note_and_a_sheet_only_file_has_no_solids() {
    let t = text("brick_2x3x4.stp");
    let with_sheet = t.replacen("ENDSEC;\nEND-ISO", "#950 = SHELL_BASED_SURFACE_MODEL('',(#951));\n#951 = OPEN_SHELL('',());\nENDSEC;\nEND-ISO", 1);
    let m = read_step(with_sheet.as_bytes(), &opts()).expect("read");
    assert_eq!(m.solids.len(), 1);
    let n = m.notes.iter().find(|n| n.code == NoteCode::SheetsSkipped).expect("step.sheets_skipped");
    assert!(n.message.contains("SHELL_BASED_SURFACE_MODEL") && n.message.contains("OPEN_SHELL"), "{}", n.message);
}

#[test]
fn colors_and_pmi_are_counted_not_imported() {
    let t = text("brick_2x3x4.stp");
    let with = t.replacen(
        "ENDSEC;\nEND-ISO",
        "#960 = COLOUR_RGB('',1.,0.,0.);\n#961 = STYLED_ITEM('',(),#6);\n#962 = DIMENSIONAL_SIZE(#6,'x');\nENDSEC;\nEND-ISO",
        1,
    );
    let m = read_step(with.as_bytes(), &opts()).expect("read");
    let n = m.notes.iter().find(|n| n.code == NoteCode::Attributes).expect("step.attributes");
    assert_eq!(n.severity, NoteSeverity::Info);
    assert!(n.message.contains("1 colors") && n.message.contains("1 styled items") && n.message.contains("1 PMI"), "{}", n.message);
}

// ---- Stretch S4: sheets and faceted solids

#[test]
fn s4_faceted_brep_gets_planes_and_straight_edges() {
    for name in ["brick_faceted.stp", "brick_faceted_dup.stp"] {
        let m = read_step(&data(name), &opts()).expect(name);
        let s = &m.solids[0];
        assert_eq!(s.kind, sm_cadio::SolidKind::Faceted);
        assert_eq!((s.vertices.len(), s.edges.len(), s.faces.len()), (8, 12, 6), "{name}");
        if name.contains("dup") {
            let w = m.notes.iter().find(|n| n.code == NoteCode::Welded).expect("welded");
            assert!(w.message.contains("welded 16 vertices"), "{}", w.message);
        }
        assert!(m.notes.iter().all(|n| n.code != NoteCode::UnsupportedEntity), "{name}");
        let (h, r) = import_into(cell_complex, &m);
        assert!(issues(h.as_view()).is_empty(), "{name}: {:?}", issues(h.as_view()));
        assert!(rel(volume(h.as_view(), r.volumes[0]), 24.0) <= 1e-9, "{name}");
        assert!(rel(total_area(h.as_view()), 52.0) <= 1e-9, "{name}");
    }
    // Poly loops with the opposite winding are repaired by the orientation check.
    let t = text("brick_faceted.stp").replace(",.T.)", ",.F.)");
    let m = read_step(t.as_bytes(), &opts()).expect("reversed bounds");
    let (h, r) = import_into(cell_complex, &m);
    assert!(rel(volume(h.as_view(), r.volumes[0]), 24.0) <= 1e-9);
    assert_eq!(r.inverted.len(), 1, "the inward solid is flipped");
}

#[test]
fn s4_sheets_import_as_free_faces_when_asked() {
    let bytes = data("sheet_open_box.stp");
    let skipped = read_step(&bytes, &opts()).expect("read");
    assert!(skipped.solids.is_empty());
    assert!(skipped.notes.iter().any(|n| n.code == NoteCode::SheetsSkipped));
    let m = read_step(&bytes, &ReadOptions { sheets: true, ..opts() }).expect("sheets");
    assert_eq!(m.solids.len(), 1);
    let s = &m.solids[0];
    assert_eq!(s.kind, sm_cadio::SolidKind::Sheet);
    assert_eq!((s.name.as_str(), s.vertices.len(), s.edges.len(), s.faces.len()), ("open box", 8, 12, 5));
    assert!(m.notes.iter().all(|n| n.code != NoteCode::SheetsSkipped));
    let (h, r) = import_into(cell_complex, &m);
    assert!(r.volumes.is_empty(), "a sheet has no volume");
    assert_eq!((r.bodies.len(), r.faces.len()), (1, 5));
    assert!(issues(h.as_view()).is_empty(), "{:?}", issues(h.as_view()));
    assert!(rel(total_area(h.as_view()), 46.0) <= 1e-9, "52 minus the top face");
    for f in &r.faces {
        assert_eq!(sm_topo_api::TopologyViewExt::face_class(h.as_view(), *f), Some(sm_topo_api::FaceClass::Free));
    }
}
