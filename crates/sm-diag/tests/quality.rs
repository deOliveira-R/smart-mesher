//! `mesh.quality`: grading, worst entities, clusters, suggestions, skipped metrics.
#![allow(missing_docs)]

mod common;
use common::*;
use serde_json::json;
use sm_core::{EntityKind, Point3};
use sm_diag::{OpNames, QualityOptions, QualityReport, ThresholdOverride, Verdict, quality_report};
use sm_mesh_api::reference::ReferenceMesh;
use sm_mesh_api::{FaceIx, MeshRead, SerialAlloc};
use sm_quality::{FvMetric, Measure, Profile, fv_compute, fv_geometry};
use sm_testkit::broken::Tampered;
use sm_testkit::{lattice, meshes};
use std::collections::BTreeMap;

fn all_ops(_: &str) -> bool {
    true
}

fn report(m: &dyn MeshRead, o: &QualityOptions) -> QualityReport {
    quality_report(m, o, &all_ops).unwrap()
}

fn by_name<'a>(r: &'a QualityReport, name: &str) -> &'a sm_diag::MetricReport {
    r.metrics.iter().find(|m| m.metric == name).unwrap_or_else(|| panic!("no metric {name}: {:?}", r.metrics.iter().map(|m| &m.metric).collect::<Vec<_>>()))
}

fn pair(kind: &str) -> ReferenceMesh {
    pair_mesh(kind, &mut SerialAlloc::new())
}

#[test]
fn sheared_pair_passes_openfoam() {
    let m = pair("sheared");
    let r = report(&m, &QualityOptions { profile: Profile::OpenFoam, ..Default::default() });
    assert_eq!(r.verdict, Verdict::Pass);
    assert_eq!((r.cells, r.dim, r.profile.as_str()), (2, 3, "openfoam"));
    assert_eq!((r.validity.n_valid, r.validity.n_inverted, r.validity.n_degenerate), (2, 0, 0));
    let no = by_name(&r, "non_orthogonality");
    assert!((no.summary.max.unwrap() - 45.0).abs() < 1e-12, "{:?}", no.summary);
    assert_eq!((no.n_warn, no.n_fail, no.entity.as_str(), no.unit.as_str()), (0, 0, "face", "deg"));
    for mr in &r.metrics {
        let h = &mr.histogram;
        let in_bins: u64 = h.counts.iter().sum();
        assert_eq!(in_bins as usize + (h.below + h.above) as usize + mr.undefined + mr.n_sentinel, mr.count, "{}", mr.metric);
        assert_eq!(h.counts.len(), 20);
    }
    // Eleven faces, ten of them boundary: undefined for an internal-only metric.
    assert_eq!((no.count, no.undefined), (11, 10));
    // The only defined face is the shared one, centroid (1, 0.5, 0.5).
    let w = &no.worst[0];
    assert_eq!(w.location, Some([1.0, 0.5, 0.5]));
    assert!((w.value - 45.0).abs() < 1e-12);
    assert_eq!(w.grade.as_deref(), Some("ok"));
    assert_eq!(w.cells.as_ref().unwrap().len(), 2);
    assert_eq!(w.owner.unwrap().kind(), EntityKind::TopoVolume, "an interior face is owned by the volume");
    assert_eq!(no.worst.len(), 1, "undefined faces are not worst entries");
    // Skewness: the boundary face with 2.0 is the worst.
    let sk = by_name(&r, "skewness");
    assert!((sk.worst[0].value - 2.0).abs() < 1e-12);
    assert_eq!(sk.threshold.fail_boundary, Some(20.0));
    assert!(r.suggestions.is_empty());
    // Cell metric
    let fl = by_name(&r, "face_flatness");
    assert_eq!(fl.threshold.fail, None, "an infinite limit is null on the wire");
    assert_eq!(fl.threshold.warn, Some(0.8));
}

#[test]
fn threshold_override_makes_the_pair_fail() {
    let m = pair("sheared");
    let o = QualityOptions {
        profile: Profile::Generic,
        thresholds: BTreeMap::from([("non_orthogonality".to_string(), ThresholdOverride { warn: None, fail: Some(40.0) })]),
        ..Default::default()
    };
    let r = report(&m, &o);
    assert_eq!(r.verdict, Verdict::Fail);
    let no = by_name(&r, "non_orthogonality");
    assert_eq!((no.n_fail, no.n_warn), (1, 0));
    assert_eq!(no.threshold.fail, Some(40.0));
    assert_eq!(no.threshold.warn, Some(50.0), "the generic warning limit stays");
    assert_eq!(no.worst[0].grade.as_deref(), Some("fail"));
    // Skewness 2.0 on the boundary: generic warn limit 2.0 is not exceeded (equality passes).
    assert_eq!(by_name(&r, "skewness").n_warn, 0);
    // A failing hot spot with a single bad entity is spread, not clustered: it gets a smooth hint.
    assert!(r.suggestions.iter().any(|s| s.kind == "smooth" && s.metric == "non_orthogonality"), "{:?}", r.suggestions);
}

#[test]
fn bad_options_are_errors() {
    let m = pair("sheared");
    let run = |o: QualityOptions| quality_report(&m, &o, &all_ops).unwrap_err().to_string();
    assert!(run(QualityOptions { worst_n: 101, ..Default::default() }).contains("worst_n"));
    assert!(run(QualityOptions { bins: 1, ..Default::default() }).contains("bins"));
    assert!(run(QualityOptions { bins: 201, ..Default::default() }).contains("bins"));
    assert!(run(QualityOptions { thresholds: BTreeMap::from([("nope".into(), ThresholdOverride::default())]), ..Default::default() }).contains("unknown metric"));
    assert!(run(QualityOptions { thresholds: BTreeMap::from([("volume_ratio".into(), ThresholdOverride { warn: None, fail: Some(2.0) })]), profile: Profile::Fluent, ..Default::default() })
        .contains("not in the metric set"));
    assert!(run(QualityOptions { thresholds: BTreeMap::from([("non_orthogonality".into(), ThresholdOverride { warn: Some(f64::NAN), fail: None })]), ..Default::default() }).contains("finite"));
    let (rig, _) = Rig::with_pair("sheared");
    assert!(rig.query("mesh.quality", json!({"metrics": ["nope"]})).unwrap_err().contains("unknown metric"));
    assert!(rig.query("mesh.quality", json!({"profile": "nope"})).is_err());
    assert!(rig.query("mesh.quality", json!({"extra": 1})).is_err());
}

#[test]
fn ungraded_and_explicit_metrics() {
    let m = pair("sheared");
    let measures = vec![Measure::from_name("volume").unwrap(), Measure::from_name("non_orthogonality").unwrap(), Measure::from_name("volume").unwrap()];
    let r = report(&m, &QualityOptions { metrics: Some(measures), profile: Profile::Fluent, ..Default::default() });
    assert_eq!(r.metrics.len(), 2, "duplicates collapse");
    let v = by_name(&r, "volume");
    assert_eq!((v.n_warn, v.n_fail), (0, 0));
    assert!(v.threshold.warn.is_none() && v.threshold.fail.is_none());
    assert!(v.worst.iter().all(|w| w.grade.is_none()));
    assert_eq!(v.sense, "lower_is_worse");
    assert!((v.summary.min.unwrap() - 1.0).abs() < 1e-12 && v.entity == "cell");
    assert!(v.clusters.is_empty());
    // Fluent has no rule for non_orthogonality either: computed, not graded.
    assert_eq!(by_name(&r, "non_orthogonality").n_fail, 0);
    assert_eq!(r.verdict, Verdict::Pass);
}

#[test]
fn unsupported_metrics_are_skipped_not_errors() {
    // Verdict `Skew` on tets, finite-volume metrics on a 2-D mesh.
    let tet = meshes::single_tet().unwrap();
    let o = QualityOptions { metrics: Some(vec![Measure::from_name("skew").unwrap(), Measure::from_name("edge_ratio").unwrap()]), ..Default::default() };
    let r = report(&tet.mesh, &o);
    assert_eq!(r.skipped.len(), 1);
    assert_eq!(r.skipped[0].metric, "skew");
    assert!(r.skipped[0].reason.contains("cell shape"));
    assert_eq!(r.metrics.len(), 1);
    let quad = lattice::quad([3, 3], [3.0, 3.0]).unwrap().mesh;
    let r = report(&quad, &QualityOptions::default());
    assert_eq!(r.dim, 2);
    let skipped: Vec<&str> = r.skipped.iter().map(|s| s.metric.as_str()).collect();
    assert_eq!(skipped, ["non_orthogonality", "skewness", "volume_ratio", "interpolation_weight", "face_flatness", "face_concavity"]);
    assert!(r.skipped.iter().all(|s| s.reason.contains("dimension-3")));
    let names: Vec<&str> = r.metrics.iter().map(|m| m.metric.as_str()).collect();
    assert_eq!(names, ["scaled_jacobian", "edge_ratio"]);
    assert_eq!(r.verdict, Verdict::Pass);
}

#[test]
fn worst_entries_are_ordered_with_ties_by_index() {
    // A regular block: every cell has the same scaled Jacobian, so ties decide.
    let m = block([3, 3, 3], [1.0; 3], &[], &mut SerialAlloc::new());
    let r = report(&m, &QualityOptions { metrics: Some(vec![Measure::from_name("scaled_jacobian").unwrap()]), worst_n: 5, ..Default::default() });
    let idx: Vec<u32> = by_name(&r, "scaled_jacobian").worst.iter().map(|w| w.entity.index).collect();
    assert_eq!(idx, [0, 1, 2, 3, 4]);
    // HigherIsWorse: largest first.
    let t = Tampered::of(&m).move_vertex(5, Point3::new(2.0, 0.2, 0.1));
    let r = report(&t, &QualityOptions { metrics: Some(vec![Measure::from_name("edge_ratio").unwrap()]), worst_n: 3, ..Default::default() });
    let w = &by_name(&r, "edge_ratio").worst;
    assert!(w.windows(2).all(|p| p[0].value >= p[1].value));
    assert_eq!(w.len(), 3);
    // worst_n = 0 gives none.
    let r = report(&m, &QualityOptions { worst_n: 0, ..Default::default() });
    assert!(r.metrics.iter().all(|m| m.worst.is_empty()));
    // Cell entries carry no `cells`, faces do.
    let r = report(&m, &QualityOptions::default());
    assert!(by_name(&r, "scaled_jacobian").worst.iter().all(|w| w.cells.is_none()));
    assert!(by_name(&r, "skewness").worst.iter().all(|w| w.cells.as_ref().is_some_and(|c| (1..=2).contains(&c.len()))));
}

#[test]
fn sentinel_values_are_counted_and_kept_out_of_summary_and_histogram() {
    let m = block([1, 1, 1], [1.0; 3], &[], &mut SerialAlloc::new());
    // Collapse one edge: the edge ratio is the 1e30 sentinel.
    let t = Tampered::of(&m).move_vertex(1, Point3::new(0.0, 0.0, 0.0));
    let r = report(&t, &QualityOptions { metrics: Some(vec![Measure::from_name("edge_ratio").unwrap()]), ..Default::default() });
    let e = by_name(&r, "edge_ratio");
    assert_eq!(e.n_sentinel, 1);
    assert_eq!(e.summary.min, None, "the only value is the sentinel");
    assert_eq!(e.histogram.counts.iter().sum::<u64>(), 0);
    assert_eq!(e.worst[0].value, 1e30);
    assert_eq!(e.n_fail, 1, "the sentinel is graded");
    assert_eq!(r.verdict, Verdict::Fail);
    let v = serde_json::to_value(&r).unwrap();
    assert_finite(&v, "quality");
    assert_eq!(v["metrics"][0]["worst"][0]["value"].as_f64(), Some(1e30));
}

#[test]
fn invalid_cells_fail_the_verdict_and_suggest_a_fix() {
    let m = block([1, 1, 1], [1.0; 3], &[], &mut SerialAlloc::new());
    let t = Tampered::of(&m).move_vertex(0, Point3::new(2.0, 2.0, 2.0));
    let r = report(&t, &QualityOptions::default());
    assert_eq!(r.verdict, Verdict::Fail);
    assert_eq!(r.validity.n_inverted, 1);
    let s = r.suggestions.iter().find(|s| s.kind == "fix_geometry_or_scheme").expect("fix suggestion");
    assert_eq!(s.targets, vec![id(EntityKind::TopoVolume, 1)]);
    assert!(s.op.is_none() && s.args.is_none() && !s.available);
    assert_eq!(s.params["n_inverted"], 1);
    assert_eq!(s.metric, "validity");
}

/// Edge-vertex displaced block: the hot spot is on one model edge.
fn hot_edge_block() -> (ReferenceMesh, sm_core::EntityId) {
    let m0 = block([3, 3, 3], [1.0; 3], &[], &mut SerialAlloc::new());
    let edge = id(EntityKind::TopoEdge, 1);
    let moves: Vec<(usize, [f64; 3])> = m0
        .classified_on(edge)
        .unwrap()
        .vertices
        .iter()
        .map(|v| {
            let p = m0.point(v.get());
            (v.get(), [p.x + 0.45, p.y, p.z])
        })
        .collect();
    (block([3, 3, 3], [1.0; 3], &moves, &mut SerialAlloc::new()), edge)
}

fn hot_options(provided: bool) -> QualityOptions {
    QualityOptions {
        metrics: Some(vec![Measure::from_name("non_orthogonality").unwrap()]),
        thresholds: BTreeMap::from([("non_orthogonality".to_string(), ThresholdOverride { warn: Some(5.0), fail: Some(30.0) })]),
        registry_provided: provided,
        ..Default::default()
    }
}

/// Independent recount of `(n_bad, n_total)` per model entity from the reverse classification
/// index, grading the face column directly.
fn recount(m: &ReferenceMesh, model: sm_core::EntityId, warn: f64) -> (usize, usize, usize, usize) {
    let g = fv_geometry(m).unwrap();
    let col = fv_compute(m, &g, &[FvMetric::NonOrthogonality]).unwrap().remove(0).values;
    let touching = |model: sm_core::EntityId| -> Vec<usize> {
        let on: std::collections::BTreeSet<u32> = m.classified_on(model).map(|s| s.vertices.iter().map(|v| v.0).collect()).unwrap_or_default();
        (0..m.face_count()).filter(|&f| col[f].is_finite() && m.face_vertices(FaceIx::new(f)).iter().any(|v| on.contains(&v.0))).collect()
    };
    let t = touching(model);
    let n_bad = t.iter().filter(|&&f| col[f] > warn).count();
    let all: Vec<usize> = (0..m.face_count()).filter(|&f| col[f].is_finite()).collect();
    (n_bad, t.len(), all.iter().filter(|&&f| col[f] > warn).count(), all.len())
}

#[test]
fn hot_edge_ranks_first_with_the_recounted_fraction_and_enrichment() {
    let (m, edge) = hot_edge_block();
    let r = report(&m, &hot_options(true));
    let c = &by_name(&r, "non_orthogonality").clusters;
    assert_eq!(c[0].model, edge, "{c:?}");
    let (n_bad, n_total, all_bad, all) = recount(&m, edge, 5.0);
    assert_eq!((c[0].n_bad, c[0].n_total), (n_bad, n_total));
    let fraction = n_bad as f64 / n_total as f64;
    assert!((c[0].fraction - fraction).abs() < 1e-15);
    assert!((c[0].enrichment - fraction / (all_bad as f64 / all as f64)).abs() < 1e-12);
    assert!(c[0].fraction >= 0.5 && c[0].enrichment >= 2.0);
    assert!(c.len() <= 10);
    assert!(c.windows(2).all(|w| w[0].fraction >= w[1].fraction - 1e-15));
    // The suggestion.
    let s = r.suggestions.iter().find(|s| s.kind == "adjust_intervals").expect("adjust_intervals");
    assert_eq!(s.targets, vec![edge]);
    assert_eq!(s.op.as_deref(), Some("mesh.intervals"));
    assert!(s.args.is_none());
    assert_eq!(s.params["current_count"], 3, "two interior vertices on the edge, plus one");
    assert!(s.available);
    assert_eq!(s.metric, "non_orthogonality");
    // Every TopoFace cluster with the same thresholds would be a change_scheme, none here.
    assert!(r.suggestions.iter().all(|s| s.kind != "change_scheme" || s.targets[0].kind() == EntityKind::TopoFace));
}

#[test]
fn availability_follows_the_registry_service() {
    let (m, _) = hot_edge_block();
    // Pure core: a predicate that rejects everything, and the "not provided" flag.
    let r = quality_report(&m, &hot_options(true), &|_| false).unwrap();
    let s = r.suggestions.iter().find(|s| s.kind == "adjust_intervals").unwrap();
    assert!(!s.available && s.reason.contains("not registered"), "{}", s.reason);
    let r = quality_report(&m, &hot_options(false), &|_| true).unwrap();
    let s = r.suggestions.iter().find(|s| s.kind == "adjust_intervals").unwrap();
    assert!(!s.available && s.reason.contains("op registry not provided"), "{}", s.reason);

    // Through the query with and without the service.
    let moves: Vec<(usize, [f64; 3])> = {
        let m0 = block([3, 3, 3], [1.0; 3], &[], &mut SerialAlloc::new());
        m0.classified_on(id(EntityKind::TopoEdge, 1)).unwrap().vertices.iter().map(|v| (v.get(), [m0.point(v.get()).x + 0.45, m0.point(v.get()).y, m0.point(v.get()).z])).collect()
    };
    let input = json!({"metrics": ["non_orthogonality"], "thresholds": {"non_orthogonality": {"warn": 5.0, "fail": 30.0}}});
    let mk = |names: Option<OpNames>| {
        let mut rig = Rig::with_names(names);
        rig.apply(CreateBrick { size: [3.0; 3] }).unwrap();
        rig.apply(InsertBlock { n: [3, 3, 3], h: [1.0; 3], moves: moves.clone() }).unwrap();
        rig
    };
    let avail = |rig: &Rig| {
        let v = rig.query("mesh.quality", input.clone()).unwrap();
        v["suggestions"].as_array().unwrap().iter().find(|s| s["kind"] == "adjust_intervals").map(|s| (s["available"].as_bool().unwrap(), s["reason"].as_str().unwrap().to_owned()))
    };
    let (a, _) = avail(&mk(Some(OpNames::new(["mesh.intervals"])))).unwrap();
    assert!(a);
    let (a, why) = avail(&mk(Some(OpNames::new(["mesh.size"])))).unwrap();
    assert!(!a && why.contains("not registered"));
    let (a, why) = avail(&mk(None)).unwrap();
    assert!(!a && why.contains("op registry not provided"));
}

#[test]
fn smooth_becomes_available_when_the_op_is_registered() {
    // Everything is bad: no hot spot, so the suggestion is `smooth`.
    let mut m = block([4, 4, 4], [1.0; 3], &[], &mut SerialAlloc::new());
    let mut s: u64 = 12345;
    for v in 0..m.vertex_count() {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let p = m.point(v);
        if [p.x, p.y, p.z].iter().all(|c| *c > 0.0 && *c < 4.0) {
            let j = ((s >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * 0.3;
            sm_mesh_api::MeshEdit::set_point(&mut m, sm_mesh_api::PointIx::new(v), Point3::new(p.x + j, p.y - j, p.z + j * 0.5)).unwrap();
        }
    }
    let o = QualityOptions {
        metrics: Some(vec![Measure::from_name("skewness").unwrap()]),
        thresholds: BTreeMap::from([("skewness".to_string(), ThresholdOverride { warn: Some(0.0), fail: None })]),
        ..Default::default()
    };
    let without = quality_report(&m, &o, &|op| op != "mesh.smooth").unwrap();
    let with = quality_report(&m, &o, &|op| op == "mesh.smooth").unwrap();
    for (r, want) in [(&without, false), (&with, true)] {
        let s = r.suggestions.iter().find(|s| s.kind == "smooth").unwrap_or_else(|| panic!("{:?}", r.suggestions));
        assert_eq!(s.available, want);
        assert_eq!(s.op.as_deref(), Some("mesh.smooth"));
        assert_eq!(s.targets, vec![m.mesh_id()]);
        assert!(s.args.is_none());
    }
    assert!(by_name(&with, "skewness").clusters.iter().all(|c| c.enrichment < 1.5));
}

/// A 4x4x4 block whose nine interior vertices on `TopoFace#3` are displaced in all directions:
/// the faces around them are warped, and the surface is the hot spot.
fn warped_surface_block() -> ReferenceMesh {
    let m0 = block([4, 4, 4], [1.0; 3], &[], &mut SerialAlloc::new());
    let face = id(EntityKind::TopoFace, 3);
    let moves: Vec<(usize, [f64; 3])> = m0
        .classified_on(face)
        .unwrap()
        .vertices
        .iter()
        .enumerate()
        .map(|(i, v)| {
            let p = m0.point(v.get());
            let s = if i % 2 == 0 { 1.0 } else { -1.0 };
            let t = if i % 3 == 0 { 1.0 } else { -0.7 };
            (v.get(), [p.x + 0.1 * s, p.y + 0.12 * t, p.z - 0.08 * s * t])
        })
        .collect();
    block([4, 4, 4], [1.0; 3], &moves, &mut SerialAlloc::new())
}

#[test]
fn surface_clusters_suggest_a_scheme_change_or_refinement() {
    let m = warped_surface_block();
    let face = id(EntityKind::TopoFace, 3);
    let opts = |profile| QualityOptions {
        profile,
        metrics: Some(vec![Measure::from_name("face_flatness").unwrap()]),
        thresholds: BTreeMap::from([("face_flatness".to_string(), ThresholdOverride { warn: Some(0.9999), fail: Some(0.999) })]),
        ..Default::default()
    };
    let generic = report(&m, &opts(Profile::Generic));
    let c = &by_name(&generic, "face_flatness").clusters;
    assert_eq!(c[0].model, face, "{c:?}");
    assert!(c[0].fraction >= 0.5 && c[0].enrichment >= 2.0);
    let s = generic.suggestions.iter().find(|s| s.targets == vec![face]).expect("suggestion on the face");
    assert_eq!((s.kind.as_str(), s.op.as_deref()), ("change_scheme", Some("mesh.scheme")));
    assert!(s.args.is_none() && s.available);
    let transport = report(&m, &opts(Profile::Transport));
    let s = transport.suggestions.iter().find(|s| s.targets == vec![face]).expect("suggestion on the face");
    assert_eq!((s.kind.as_str(), s.op.as_deref()), ("refine_surface", Some("mesh.size")));
    let tf = by_name(&transport, "face_flatness");
    assert_eq!(tf.n_fail + tf.n_warn, 32, "the 32 bad faces of the recorded run");
    // A planar mesh gives no refinement and the transport rules are the five of the profile.
    let r = report(&pair("sheared"), &QualityOptions { profile: Profile::Transport, ..Default::default() });
    assert!(r.suggestions.iter().all(|s| s.kind != "refine_surface"));
    let names: Vec<&str> = r.metrics.iter().map(|m| m.metric.as_str()).collect();
    assert_eq!(names, ["face_flatness", "face_concavity", "starness", "edge_ratio", "volume_ratio"]);
}
