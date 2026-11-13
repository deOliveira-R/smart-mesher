//! W4-BOOL-HEAL-A T3: `model.heal_check` on analytic fixtures (exact finding sets) and on the
//! W4-REFCORPUS STEP files (codes equal the manifest's `expected`).

mod common;

use std::collections::BTreeMap;
use std::path::PathBuf;

use common::rig::*;
use serde_json::json;
use sm_boolean::ops::{Subtract, Unite};
use sm_boolean::{HealCode, HealOptions, HealReport, heal_check, regularize_volumes};
use sm_cadio::{ReadOptions, RealizeOptions, read_step, realize};
use sm_topo_api::{CellComplex, OpId, TopologyEdit, TopologyView};

fn check(r: &Rig) -> HealReport {
    heal_check(r.topo(), &HealOptions::default()).expect("heal_check")
}

fn count(rep: &HealReport, c: HealCode) -> usize {
    rep.with_code(c).count()
}

fn sub(r: &mut Rig, target: &Solid, tool: &Solid) {
    r.run(&Subtract { targets: vec![target.body], tools: vec![tool.body], keep: None, regularize: None, tol: None }).unwrap();
}

#[test]
fn notch_gives_nine_small_edges_and_three_small_faces() {
    const S: f64 = 1e-4;
    let mut r = Rig::cc();
    let b = r.brick([1.0; 3], [0.5; 3]);
    let c = r.brick([S; 3], [S / 2.0; 3]);
    sub(&mut r, &b, &c);
    let rep = check(&r);
    assert_eq!(rep.codes(), vec![HealCode::SmallEdge, HealCode::SmallFace], "{:#?}", rep.findings);
    assert_eq!(count(&rep, HealCode::SmallEdge), 9);
    assert_eq!(count(&rep, HealCode::SmallFace), 3);
    for f in rep.with_code(HealCode::SmallEdge) {
        assert!((f.value - S).abs() <= 1e-12, "edge length {}", f.value);
    }
    for f in rep.with_code(HealCode::SmallFace) {
        assert!((f.value - S * S).abs() <= 1e-12, "face area {}", f.value);
    }
}

#[test]
fn step_gives_four_small_edges_and_two_slivers() {
    const S: f64 = 1e-4;
    let mut r = Rig::cc();
    let a = r.brick([1.0; 3], [0.5; 3]);
    // B = [S, 1+S] x [0,1] x [1,2]: center x = S + 0.5.
    let b = r.brick([1.0; 3], [S + 0.5, 0.5, 1.5]);
    r.run(&Unite { bodies: vec![a.body, b.body], keep: None, regularize: None, tol: None }).unwrap();
    let rep = check(&r);
    assert_eq!(rep.codes(), vec![HealCode::SliverFace, HealCode::SmallEdge], "{:#?}", rep.findings);
    assert_eq!(count(&rep, HealCode::SmallEdge), 4);
    assert_eq!(count(&rep, HealCode::SliverFace), 2);
    for f in rep.with_code(HealCode::SliverFace) {
        assert!(((f.value - 1.0 / S) / (1.0 / S)).abs() <= 1e-9, "aspect {}", f.value);
    }
}

#[test]
fn plate_hole_is_small_below_the_threshold_only() {
    const RH: f64 = 1e-3;
    let mut r = Rig::cc();
    let plate = r.brick([1.0, 1.0, 0.1], [0.5, 0.5, 0.05]);
    let pin = r.cylinder(0.3, RH, [0.5, 0.5, 0.05]);
    sub(&mut r, &plate, &pin);
    let with = |d: f64| heal_check(r.topo(), &HealOptions { hole_max_diameter: Some(d), ..HealOptions::default() }).unwrap();
    let rep = with(2.5 * RH);
    assert_eq!(count(&rep, HealCode::SmallHole), 1, "{:#?}", rep.findings);
    assert!((rep.with_code(HealCode::SmallHole).next().unwrap().value - 2.0 * RH).abs() <= 1e-12);
    assert_eq!(count(&with(1.5 * RH), HealCode::SmallHole), 0);
    // The default threshold takes it too, and the circle is not a short edge.
    let rep = check(&r);
    assert_eq!(rep.codes(), vec![HealCode::SmallHole], "{:#?}", rep.findings);
}

#[test]
fn gap_near_exactly_between_the_tolerance_and_linear_max() {
    let lin = 1e-6;
    let max = 1e-3;
    for (g, near) in [(0.5 * lin, false), (1e-5, true), (5e-4, true), (max, true), (2.0 * max, false)] {
        let mut r = Rig::cc();
        r.brick([1.0; 3], [0.5; 3]);
        r.brick([1.0; 3], [1.5 + g, 0.5, 0.5]);
        let rep = check(&r);
        let n = count(&rep, HealCode::GapNear);
        // The merge_check statuses the finding is defined by.
        let mc = sm_boolean::merge_check(r.topo(), &[], &[], &sm_boolean::MergeOptions::default());
        let near_faces = mc.pairs.iter().filter(|p| p.kind == sm_boolean::MergeKind::Face && p.status == sm_boolean::MergeStatus::Near).count();
        assert_eq!(near_faces > 0, near, "gap {g:e}: merge_check face near pairs {near_faces}");
        assert_eq!(n > 0, near, "gap {g:e}: {:#?}", rep.findings);
        if near {
            let f = rep.with_code(HealCode::GapNear).find(|f| f.entities.len() == 2).unwrap();
            assert!((f.value - g).abs() <= 1e-9 * g.max(1e-12) + 1e-15, "gap {} vs {g:e}", f.value);
        }
        assert!(lin < g || !near);
    }
}

#[test]
fn fillet_is_one_blend_candidate_with_its_radius() {
    const RAD: f64 = 0.1;
    let mut r = Rig::cc();
    let b = r.brick([1.0; 3], [0.5; 3]);
    // The tool: the corner block outside the quarter round, taller than the brick.
    let blk = r.brick([1.0 + RAD, 1.0 + RAD, 3.0], [(RAD - 1.0) / 2.0, (RAD - 1.0) / 2.0, 0.5]);
    let cyl = r.cylinder(5.0, RAD, [RAD, RAD, 0.5]);
    sub(&mut r, &blk, &cyl);
    let tool = r.topo().cells(sm_topo_api::Dim::Body).iter().copied().find(|&x| x != b.body).unwrap();
    r.run(&Subtract { targets: vec![b.body], tools: vec![tool], keep: None, regularize: None, tol: None }).unwrap();
    let rep = check(&r);
    assert_eq!(rep.codes(), vec![HealCode::BlendCandidate], "{:#?}", rep.findings);
    let f = rep.with_code(HealCode::BlendCandidate).next().unwrap();
    assert!((f.value - RAD).abs() <= 1e-9);
    assert!((f.extra.unwrap() - std::f64::consts::FRAC_PI_2).abs() <= 1e-9);
}

#[test]
fn clean_bodies_have_no_findings() {
    let mut r = Rig::cc();
    r.brick([2.0, 3.0, 4.0], [0.0; 3]);
    r.cylinder(2.0, 0.5, [5.0, 0.0, 0.0]);
    r.sphere(0.7, [-5.0, 0.0, 0.0]);
    let rep = check(&r);
    assert!(rep.findings.is_empty(), "{:#?}", rep.findings);
}

#[test]
fn the_report_is_sorted_and_identical_for_one_and_many_threads() {
    const S: f64 = 1e-4;
    let mut r = Rig::cc();
    let a = r.brick([1.0; 3], [0.5; 3]);
    let b = r.brick([1.0; 3], [S + 0.5, 0.5, 1.5]);
    r.run(&Unite { bodies: vec![a.body, b.body], keep: None, regularize: None, tol: None }).unwrap();
    r.brick([1.0; 3], [5.0, 0.5, 0.5]);
    r.brick([1.0; 3], [6.0 + 1e-5, 0.5, 0.5]);
    let many = check(&r);
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().unwrap().install(|| check(&r));
    assert_eq!(one, many);
    assert_eq!(check(&r), many, "twice");
    let keys: Vec<_> = many.findings.iter().map(|f| (f.code, f.entity)).collect();
    let mut sorted = keys.clone();
    sorted.sort();
    assert_eq!(keys, sorted);
    assert!(many.codes().len() >= 3, "{:#?}", many.findings);
}

#[test]
fn the_query_returns_the_same_report_and_refuses_bad_input() {
    let mut r = Rig::cc();
    let b = r.brick([1.0; 3], [0.5; 3]);
    let c = r.brick([1e-4; 3], [0.5e-4; 3]);
    sub(&mut r, &b, &c);
    let v = r.query("model.heal_check", json!({})).unwrap();
    let rep: HealReport = serde_json::from_value(v).unwrap();
    assert_eq!(rep, check(&r));
    assert_eq!(count(&rep, HealCode::SmallEdge), 9);
    assert!(r.query("model.heal_check", json!({"small_edge": -1.0})).is_err());
    assert!(r.query("model.heal_check", json!({"entities": ["TopoFace#999"]})).is_err());
    assert!(r.query("model.heal_check", json!({"bogus": 1})).is_err());
    // Scoped to one face: its own small edges only.
    let scoped = r.query("model.heal_check", json!({"entities": [r.topo().cells(sm_topo_api::Dim::Volume)[0].to_string()]})).unwrap();
    let scoped: HealReport = serde_json::from_value(scoped).unwrap();
    assert_eq!(scoped.findings.len(), rep.findings.len());
}

// ---- the W4-REFCORPUS STEP files

fn corpus_dir() -> Option<PathBuf> {
    let d = std::env::var_os("SM_REFCORPUS_DIR").map(PathBuf::from).or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".cache/smart-mesh/refcorpus")))?;
    d.is_dir().then_some(d)
}

/// `id -> expected codes` of the `step-*` entries of the manifest.
fn manifest_expected() -> BTreeMap<String, Vec<String>> {
    let text = std::fs::read_to_string(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tools/refcorpus/manifest.toml")).expect("manifest");
    let mut out = BTreeMap::new();
    let mut id = String::new();
    for line in text.lines() {
        if let Some(v) = line.strip_prefix("id = \"") {
            id = v.trim_end_matches('"').to_owned();
        } else if let Some(v) = line.strip_prefix("expected = [")
            && id.starts_with("step-")
        {
            let codes = v.trim_end_matches(']').split(',').map(|s| s.trim().trim_matches('"').to_owned()).filter(|s| !s.is_empty()).collect();
            out.insert(id.clone(), codes);
        }
    }
    out
}

fn import(id: &str, dir: &std::path::Path) -> CellComplex {
    let bytes = std::fs::read(dir.join(id).join(format!("{id}.step"))).unwrap_or_else(|e| panic!("{id}: {e}"));
    let m = read_step(&bytes, &ReadOptions { sheets: true, ..ReadOptions::default() }).expect("read_step");
    let mut h = CellComplex::new();
    h.begin_op(OpId(1)).unwrap();
    realize(&m, &mut h, &RealizeOptions::default()).expect("realize");
    h.end_op().unwrap();
    h
}

#[test]
fn corpus_step_files_give_the_manifest_codes() {
    let Some(dir) = corpus_dir() else {
        eprintln!("skipped: no W4-REFCORPUS cache (python3 -I tools/refcorpus/build.py build)");
        return;
    };
    let expected = manifest_expected();
    assert!(expected.len() >= 11, "{expected:?}");
    for (id, want) in expected {
        if !dir.join(&id).join(format!("{id}.step")).exists() {
            eprintln!("skipped {id}: not built");
            continue;
        }
        let h = import(&id, &dir);
        let rep = heal_check(&h, &HealOptions::default()).unwrap();
        let got: Vec<&str> = rep.codes().iter().map(|c| c.as_str()).collect();
        let mut want_sorted: Vec<&str> = want.iter().map(String::as_str).collect();
        want_sorted.sort_unstable();
        assert_eq!(got, want_sorted, "{id}: {:#?}", rep.findings);
    }
}

#[test]
fn regularize_changes_nothing_on_clean_step_imports() {
    let Some(dir) = corpus_dir() else {
        eprintln!("skipped: no W4-REFCORPUS cache");
        return;
    };
    for id in ["step-clean-pin", "step-clean-cladded-pin", "step-clean-assembly-can", "step-clean-grid-spacer"] {
        if !dir.join(id).join(format!("{id}.step")).exists() {
            continue;
        }
        let mut h = import(id, &dir);
        let words = |t: &dyn TopologyView| {
            let mut w = sm_topo_api::canonical_words(t);
            w.remove(4); // the revision word
            w
        };
        let before = words(&h);
        let vols: Vec<_> = h.cells(sm_topo_api::Dim::Volume).to_vec();
        h.begin_op(OpId(2)).unwrap();
        let rep = regularize_volumes(&mut h, &vols).unwrap();
        h.end_op().unwrap();
        assert!(rep.joined_faces.is_empty() && rep.joined_edges.is_empty(), "{id}: {rep:?}");
        assert_eq!(words(&h), before, "{id}");
    }
}
