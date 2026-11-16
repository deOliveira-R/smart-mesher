//! `repair: "tolerant"`: open shells closed by planar caps, analytic checks on the 2x3x4 brick.

mod common;

use std::sync::Arc;

use common::*;
use sm_cadio::{ImportStep, LengthUnit, MemoryInputStore, NoteCode, ReadOptions, RepairMode, SolidKind, Source, StepError, read_step};
use sm_topo_api::{Dim, TopologyView};

const BRICK_VOLUME: f64 = 24.0;
const BRICK_AREA: f64 = 2.0 * (6.0 + 8.0 + 12.0);

fn repair_opts() -> ReadOptions {
    ReadOptions { repair: true, assume_unit: Some(LengthUnit::Mm), ..ReadOptions::default() }
}

fn plain_opts() -> ReadOptions {
    ReadOptions { assume_unit: Some(LengthUnit::Mm), ..ReadOptions::default() }
}

/// The brick text with the listed `ADVANCED_FACE` ids dropped from its shell.
fn brick_without(drop: &[&str]) -> Vec<u8> {
    let text = String::from_utf8(data("brick_2x3x4.stp")).expect("utf8");
    let mut out = String::new();
    for line in text.lines() {
        if line.starts_with("#154 = CLOSED_SHELL") {
            let mut s = line.to_owned();
            for d in drop {
                s = s.replace(&format!("{d},"), "").replace(&format!(",{d})"), ")");
            }
            out.push_str(&s);
        } else {
            out.push_str(line);
        }
        out.push('\n');
    }
    out.into_bytes()
}

fn check_brick(m: &sm_cadio::BrepModel) {
    let (h, _) = import_into(cell_complex, m);
    let view: &dyn TopologyView = &*h;
    assert!(issues(view).is_empty(), "{:?}", issues(view));
    let vols = view.cells(Dim::Volume);
    assert_eq!(vols.len(), 1);
    assert!(rel(volume(view, vols[0]), BRICK_VOLUME) < 1e-12);
    assert!(rel(total_area(view), BRICK_AREA) < 1e-12);
    assert_eq!(count(view, Dim::Face), 6);
    assert_eq!(count(view, Dim::Edge), 12);
    assert_eq!(count(view, Dim::Vertex), 8);
}

#[test]
fn a_missing_face_is_capped_with_a_note() {
    let b = bad("open_shell.stp");
    assert!(matches!(read_step(&b, &plain_opts()), Err(StepError::OpenShell { .. })));
    let m = read_step(&b, &repair_opts()).expect("repaired");
    let fixes: Vec<_> = m.notes.iter().filter(|n| n.code == NoteCode::Repaired).collect();
    assert_eq!(fixes.len(), 1);
    assert_eq!(fixes[0].step_ids.len(), 4);
    check_brick(&m);
}

#[test]
fn two_opposite_faces_missing_give_the_brick_back_and_two_adjacent_do_not() {
    let ids = ["#93", "#105", "#117", "#129", "#141", "#153"];
    let (mut repaired, mut refused) = (0, 0);
    for i in 0..6 {
        for j in i + 1..6 {
            let bytes = brick_without(&[ids[i], ids[j]]);
            match read_step(&bytes, &repair_opts()) {
                Ok(m) => {
                    repaired += 1;
                    assert_eq!(m.notes.iter().filter(|n| n.code == NoteCode::Repaired).count(), 2);
                    check_brick(&m);
                }
                Err(StepError::OpenShell { issues, .. }) => {
                    refused += 1;
                    assert!(!issues.is_empty());
                }
                Err(e) => panic!("{e:?}"),
            }
        }
    }
    assert_eq!((repaired, refused), (3, 12));
}

#[test]
fn a_clean_file_is_unchanged_by_repair() {
    let b = data("brick_2x3x4.stp");
    let plain = read_step(&b, &plain_opts()).expect("plain");
    let rep = read_step(&b, &repair_opts()).expect("repair");
    assert_eq!(plain.solids, rep.solids);
    assert_eq!(plain.notes, rep.notes);
}

#[test]
fn an_open_sheet_becomes_a_solid_only_with_repair() {
    let b = data("sheet_open_box.stp");
    let plain = read_step(&b, &plain_opts()).expect("plain");
    assert!(plain.solids.is_empty());
    let m = read_step(&b, &repair_opts()).expect("repaired");
    assert_eq!(m.solids.len(), 1);
    assert_eq!(m.solids[0].kind, SolidKind::Manifold);
    assert_eq!(m.notes.iter().filter(|n| n.code == NoteCode::Repaired).count(), 1);
    check_brick(&m);
    // With `sheets` the sheet stays a sheet.
    let s = read_step(&b, &ReadOptions { sheets: true, ..repair_opts() }).expect("sheets");
    assert_eq!(s.solids[0].kind, SolidKind::Sheet);
}

#[test]
fn a_closed_surface_model_promoted_to_a_solid_is_reported() {
    let text = String::from_utf8(data("sheet_open_box.stp")).expect("utf8");
    let bytes = text.replace("OPEN_SHELL('',(#93,#105,#117,#129,#141))", "OPEN_SHELL('',(#93,#105,#117,#129,#141,#153))").into_bytes();
    let plain = read_step(&bytes, &plain_opts()).expect("plain");
    assert!(plain.solids.is_empty());
    assert!(plain.notes.iter().any(|n| n.code == NoteCode::SheetsSkipped));
    let m = read_step(&bytes, &repair_opts()).expect("repaired");
    assert_eq!(m.solids.len(), 1);
    assert_eq!(m.solids[0].kind, SolidKind::Manifold);
    // Promotion with no caps is still a finding.
    assert_eq!(m.notes.iter().filter(|n| n.code == NoteCode::Repaired).count(), 1);
    let n = m.notes.iter().find(|n| n.code == NoteCode::Repaired).expect("note");
    assert!(n.message.contains("closed surface model"), "{}", n.message);
    // With `sheets` it stays a sheet and nothing is reported.
    let s = read_step(&bytes, &ReadOptions { sheets: true, ..repair_opts() }).expect("sheets");
    assert_eq!(s.solids[0].kind, SolidKind::Sheet);
    assert_eq!(s.notes.iter().filter(|n| n.code == NoteCode::Repaired).count(), 0);
}

#[test]
fn an_unclosable_sheet_is_reported_not_imported() {
    let text = String::from_utf8(data("sheet_open_box.stp")).expect("utf8");
    let bytes = text.replace("OPEN_SHELL('',(#93,#105,#117,#129,#141))", "OPEN_SHELL('',(#93,#105,#117))").into_bytes();
    let m = read_step(&bytes, &repair_opts()).expect("read");
    assert!(m.solids.is_empty());
    assert!(m.notes.iter().any(|n| n.code == NoteCode::RepairFailed));
}

fn sheet_with(shells: &str, extra: &str) -> Vec<u8> {
    let text = String::from_utf8(data("sheet_open_box.stp")).expect("utf8");
    let text = text.replace("#155 = SHELL_BASED_SURFACE_MODEL('open box',(#154));", &format!("{extra}\n{shells}"));
    text.into_bytes()
}

fn repaired(m: &sm_cadio::BrepModel) -> usize {
    m.notes.iter().filter(|n| n.code == NoteCode::Repaired).count()
}

fn failed(m: &sm_cadio::BrepModel) -> usize {
    m.notes.iter().filter(|n| n.code == NoteCode::RepairFailed).count()
}

#[test]
fn a_single_face_sheet_is_not_capped_onto_itself() {
    let b = sheet_with("#155 = SHELL_BASED_SURFACE_MODEL('one face',(#900));", "#900 = OPEN_SHELL('',(#93));");
    let m = read_step(&b, &repair_opts()).expect("read");
    assert!(m.solids.is_empty(), "{:?}", m.solids.len());
    assert_eq!(repaired(&m), 0);
    assert_eq!(failed(&m), 1);
}

#[test]
fn a_brick_plus_a_stray_face_gives_one_brick_and_one_failure() {
    let b = sheet_with("#155 = SHELL_BASED_SURFACE_MODEL('brick',(#154));\n#901 = SHELL_BASED_SURFACE_MODEL('stray',(#900));", "#900 = OPEN_SHELL('',(#153));");
    let m = read_step(&b, &repair_opts()).expect("read");
    assert_eq!(m.solids.len(), 1);
    assert_eq!((repaired(&m), failed(&m)), (1, 1));
    check_brick(&m);
}

#[test]
fn a_multi_shell_surface_model_is_not_repaired() {
    let b = sheet_with("#155 = SHELL_BASED_SURFACE_MODEL('two shells',(#154,#900));", "#900 = OPEN_SHELL('',(#153));");
    let m = read_step(&b, &repair_opts()).expect("read");
    assert!(m.solids.is_empty());
    assert_eq!((repaired(&m), failed(&m)), (0, 1));
}

#[test]
fn the_op_refuses_without_repair_and_imports_with_it() {
    let store = Arc::new(MemoryInputStore::new());
    let mut s = session(cell_complex, store);
    let mut op = ImportStep { source: Source::inline(&bad("open_shell.stp")), name: None, to_unit: None, assume_unit: None, weld_tol: None, solids: None, sheets: None, check: None, attributes: None, repair: None };
    let e = format!("{:?}", run(&mut s, &op).expect_err("open shell"));
    assert!(e.contains("step.open_shell"), "{e}");
    op.repair = Some(RepairMode::Tolerant);
    let (_, d) = run(&mut s, &op).expect("repaired import");
    let view = model(&s).topo();
    assert!(rel(volume(view, d.outputs["volumes"][0]), BRICK_VOLUME) <= 1e-12);
    // A shell that cannot be closed still fails typed.
    let op = ImportStep { source: Source::inline(&brick_without(&["#93", "#117"])), ..op };
    let e = format!("{:?}", run(&mut s, &op).expect_err("unclosable"));
    assert!(e.contains("step.open_shell"), "{e}");
}

#[test]
fn an_undecodable_stray_sheet_does_not_fail_the_import_under_repair() {
    let text = String::from_utf8(data("brick_2x3x4.stp")).expect("utf8");
    let stray = "#9011 = OFFSET_SURFACE('',#152,1.,.F.);\n#9000 = ADVANCED_FACE('',(#152),#9011,.T.);\n#9001 = OPEN_SHELL('',(#9000));\n#9002 = SHELL_BASED_SURFACE_MODEL('stray',(#9001));\nENDSEC;";
    let at = text.rfind("ENDSEC;").expect("endsec");
    let text = format!("{}{}{}", &text[..at], stray, &text[at + "ENDSEC;".len()..]);
    let plain = read_step(text.as_bytes(), &plain_opts()).expect("plain read");
    assert_eq!(plain.solids.len(), 1);
    let m = read_step(text.as_bytes(), &repair_opts()).expect("repair read");
    assert_eq!(m.solids.len(), 1);
    assert_eq!(failed(&m), 1);
    assert!(m.notes.iter().any(|n| n.code == NoteCode::RepairFailed && n.message.contains("#9002")));
    check_brick(&m);

    // A face on a missing surface behaves the same.
    let missing = text.replace("#9011,.T.", "#99999,.T.");
    let m = read_step(missing.as_bytes(), &repair_opts()).expect("repair read");
    assert_eq!((m.solids.len(), failed(&m)), (1, 1));
}
