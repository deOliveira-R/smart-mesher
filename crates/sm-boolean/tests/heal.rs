//! W4-HEAL-TOL (T5): `model.heal` on analytic fixtures (exact volumes, areas, Euler number) and
//! on the W4-REFCORPUS dirty STEP files.

mod common;

use std::path::PathBuf;

use common::rig::*;
use proptest::prelude::*;
use sm_boolean::ops::{Heal, HealFixName};
use sm_boolean::{
    HealApply, HealCode, HealFixes, HealOptions, MergeKind, MergeOptions, MergeStatus, heal,
    heal_check, merge_check,
};
use sm_cadio::{ReadOptions, RealizeOptions, read_step, realize};
use sm_core::Point3;
use sm_geom_api::{CurveDef, Line};
use sm_testkit::strategies::config;
use sm_topo_api::fixtures::{BrickIds, brick, webcut_brick};
use sm_topo_api::{
    Carrier, CellComplex, Dim, LineageKind, OpId, Side, TopologyEdit, TopologyView,
    TopologyViewExt, canonical_words, validate,
};

/// The canonical words without the revision word (a commit with no edit still moves it).
fn words(t: &dyn TopologyView) -> Vec<sm_topo_api::HashWord> {
    let mut w = canonical_words(t);
    w.remove(4);
    w
}

fn volumes_of(t: &dyn TopologyView) -> Vec<f64> {
    t.cells(Dim::Volume)
        .iter()
        .map(|&v| t.volume_moments(v).unwrap().volume)
        .collect()
}

fn heal_op(tol: Option<f64>) -> Heal {
    Heal {
        entities: None,
        tol,
        fixes: None,
    }
}

fn two_gap_bricks(g: f64) -> Rig {
    let mut r = Rig::cc();
    r.brick([1.0; 3], [0.5; 3]);
    r.brick([1.0; 3], [1.5 + g, 0.5, 0.5]);
    r
}

fn near_faces(t: &dyn TopologyView) -> usize {
    merge_check(t, &[], &[], &MergeOptions::default())
        .pairs
        .iter()
        .filter(|p| p.status == MergeStatus::Near)
        .count()
}

#[test]
fn gap_bricks_heal_to_one_shared_face_with_the_exact_volume_change() {
    const G: f64 = 5e-5;
    let mut r = two_gap_bricks(G);
    assert_eq!(
        heal_check(r.topo(), &HealOptions::default())
            .unwrap()
            .codes(),
        vec![HealCode::GapNear]
    );
    let faces_before = r.topo().cells(Dim::Face).len();
    let d = r.run(&heal_op(None)).unwrap();
    assert!(!d.outputs.is_empty());
    let t = r.topo();
    // One shared face: 12 faces become 11, and the volumes are 1 and 1 + A g (A = 1) per brick:
    // the kept face is the lowest id, so the second brick's -x face moved by G.
    assert_eq!(t.cells(Dim::Face).len(), faces_before - 1);
    let v = volumes_of(t);
    assert!((v[0] - 1.0).abs() <= 1e-12, "{v:?}");
    assert!((v[1] - (1.0 + G)).abs() <= 1e-12, "{v:?}");
    // The kept face carries a tolerance of at least the gap.
    let kept = t
        .cells(Dim::Face)
        .iter()
        .copied()
        .find(|&f| t.face_uses_of(f).len() == 2)
        .expect("a shared face");
    assert!(t.tolerance_of(kept) >= G, "{:e}", t.tolerance_of(kept));
    // No near pair is left, and heal_check agrees.
    assert_eq!(near_faces(t), 0);
    assert!(
        heal_check(t, &HealOptions::default())
            .unwrap()
            .findings
            .is_empty()
    );
    assert!(validate(t).is_empty());
    // Every merge is lineage with its gap.
    let gaps: Vec<f64> = t
        .lineage()
        .records()
        .iter()
        .filter_map(|rec| match rec.kind {
            LineageKind::Merged { gap, .. } => Some(gap),
            _ => None,
        })
        .collect();
    assert_eq!(gaps.len(), 9, "4 vertices + 4 edges + 1 face: {gaps:?}");
    assert!(gaps.iter().all(|g| (g - G).abs() <= 1e-9 * G), "{gaps:?}");
}

#[test]
fn heal_twice_equals_heal_once() {
    let mut r = two_gap_bricks(2e-5);
    r.run(&heal_op(None)).unwrap();
    let once = words(r.topo());
    let lineage = r.topo().lineage().len();
    let d = r.run(&heal_op(None)).unwrap();
    assert_eq!(words(r.topo()), once);
    assert_eq!(r.topo().lineage().len(), lineage, "no new lineage");
    assert!(d.outputs.iter().all(|o| o.1.is_empty()), "{:?}", d.outputs);
}

#[test]
fn a_gap_above_the_tolerance_is_left_and_tol_is_validated() {
    let mut r = two_gap_bricks(5e-4);
    let before = words(r.topo());
    r.run(&heal_op(Some(1e-4))).unwrap();
    assert_eq!(words(r.topo()), before);
    assert!(near_faces(r.topo()) > 0);
    assert!(r.run(&heal_op(Some(1.0))).is_err());
    assert!(r.run(&heal_op(Some(0.0))).is_err());
    // fixes limit the run: orient and close leave the gap.
    r.run(&Heal {
        entities: None,
        tol: None,
        fixes: Some(vec![HealFixName::Orient, HealFixName::Close]),
    })
    .unwrap();
    assert_eq!(words(r.topo()), before);
    r.run(&Heal {
        entities: None,
        tol: None,
        fixes: Some(vec![HealFixName::Weld]),
    })
    .unwrap();
    assert_eq!(near_faces(r.topo()), 0);
}

#[test]
fn close_by_vertices_and_short_edges_are_not_welded() {
    // A 1e-4 notch: every gap-like pair is a corner of the notch, not a gap.
    use sm_boolean::ops::Subtract;
    let mut r = Rig::cc();
    let b = r.brick([1.0; 3], [0.5; 3]);
    let c = r.brick([1e-4; 3], [0.5e-4; 3]);
    r.run(&Subtract {
        targets: vec![b.body],
        tools: vec![c.body],
        keep: None,
        regularize: None,
        tol: None,
    })
    .unwrap();
    let before = words(r.topo());
    r.run(&heal_op(None)).unwrap();
    assert_eq!(words(r.topo()), before);
}

// ---- library level on a raw cell complex

fn raw_brick(min: [f64; 3]) -> (CellComplex, BrickIds) {
    let mut h = CellComplex::new();
    h.begin_op(OpId(1)).unwrap();
    let b = brick(&mut h, Point3::new(min[0], min[1], min[2]), [1.0; 3]).unwrap();
    h.end_op().unwrap();
    (h, b)
}

fn run_heal(h: &mut CellComplex, n: u64, fixes: HealFixes) -> sm_boolean::HealOutcome {
    h.begin_op(OpId(n)).unwrap();
    let o = heal(
        h,
        &HealApply {
            entities: None,
            tol: None,
            fixes,
        },
    )
    .unwrap();
    h.end_op().unwrap();
    o
}

/// Take the volume off a brick, reverse every face and put the uses back: inside out.
fn invert(h: &mut CellComplex, b: &BrickIds, n: u64) {
    h.begin_op(OpId(n)).unwrap();
    for &f in &b.faces {
        h.remove_face_use(b.volume, f, Side::Left).unwrap();
    }
    for &f in &b.faces {
        h.reverse_face(f).unwrap();
    }
    for &f in &b.faces {
        h.add_face_use(
            b.volume,
            0,
            sm_topo_api::FaceUseSpec {
                face: f,
                side: Side::Left,
            },
        )
        .unwrap();
    }
    h.end_op().unwrap();
}

#[test]
fn an_inside_out_volume_regains_its_positive_volume() {
    let (mut h, b) = raw_brick([0.0; 3]);
    let good = words(&h);
    invert(&mut h, &b, 2);
    let v = h.volume_moments(b.volume).unwrap().volume;
    assert!((v + 1.0).abs() <= 1e-12, "inverted volume {v}");
    assert!(validate(&h).is_empty(), "inside out is topologically valid");
    assert_ne!(words(&h), good);
    let out = run_heal(&mut h, 3, HealFixes::default());
    assert_eq!(out.reoriented, vec![b.volume]);
    let v = h.volume_moments(b.volume).unwrap().volume;
    assert!((v - 1.0).abs() <= 1e-12, "healed volume {v}");
    assert!(validate(&h).is_empty());
    // A second run changes nothing.
    let once = words(&h);
    let out = run_heal(&mut h, 4, HealFixes::default());
    assert!(out.is_empty(), "{out:?}");
    assert_eq!(words(&h), once);
}

/// A unit brick whose `+z` face is gone: five free faces and one open loop of four edges.
fn open_brick() -> (CellComplex, BrickIds) {
    let (mut h, b) = raw_brick([0.0; 3]);
    h.begin_op(OpId(2)).unwrap();
    h.kill(b.body).unwrap();
    h.kill(b.volume).unwrap();
    h.end_op().unwrap();
    // The volume went with the body; drop the top face and keep the rest as free faces.
    h.begin_op(OpId(3)).unwrap();
    h.kill(b.faces[5]).unwrap();
    h.end_op().unwrap();
    (h, b)
}

#[test]
fn a_planar_hole_is_closed_to_a_watertight_shell_of_euler_two() {
    let (mut h, _b) = open_brick();
    assert_eq!(h.cells(Dim::Volume).len(), 0);
    let rep = heal_check(&h, &HealOptions::default()).unwrap();
    assert_eq!(rep.codes(), vec![HealCode::OpenShell]);
    let out = run_heal(&mut h, 4, HealFixes::default());
    assert_eq!(out.closed.len(), 1, "{out:?}");
    let cap = out.closed[0].face;
    assert!(out.closed[0].flatness <= 1e-12);
    // Area of the cap is 1, its normal is +z.
    let a = h.face_moments(cap).unwrap().area;
    assert!((a - 1.0).abs() <= 1e-12, "cap area {a}");
    // Closed: no free edge, validate clean, V - E + F = 2.
    assert!(
        heal_check(&h, &HealOptions::default())
            .unwrap()
            .findings
            .is_empty()
    );
    assert!(validate(&h).is_empty());
    let (v, e, f) = (
        h.cells(Dim::Vertex).len() as i64,
        h.cells(Dim::Edge).len() as i64,
        h.cells(Dim::Face).len() as i64,
    );
    assert_eq!((v, e, f), (8, 12, 6));
    assert_eq!(v - e + f, 2);
    for &ed in h.cells(Dim::Edge) {
        let n = h
            .coboundary(ed)
            .iter()
            .filter(|i| h.dim_of(i.cell) == Some(Dim::Face))
            .count();
        assert_eq!(n, 2, "{ed}");
    }
    // Idempotent.
    let once = words(&h);
    assert!(run_heal(&mut h, 5, HealFixes::default()).is_empty());
    assert_eq!(words(&h), once);
}

#[test]
fn a_loop_off_the_plane_by_more_than_tol_is_left_open() {
    let (mut h, b) = open_brick();
    // Lift one corner of the open loop by 0.1 and re-run its two free edges to it.
    let top = Point3::new(0.0, 0.0, 1.1);
    h.begin_op(OpId(4)).unwrap();
    h.set_vertex_point(b.vertices[4], top).unwrap();
    for &e in &b.edges {
        let (s, t) = h.edge_ends(e).unwrap();
        if (s == b.vertices[4] || t == b.vertices[4]) && e != b.edges[8] {
            let pt = |v| match h.carrier(v) {
                Carrier::Point(p) => *p,
                _ => unreachable!(),
            };
            let (ps, pe) = (pt(s), pt(t));
            let Carrier::Curve(cc) = h.carrier(e).clone() else {
                unreachable!()
            };
            h.replace_curve(cc.curve, CurveDef::Line(Line::new(ps, pe - ps).unwrap()))
                .unwrap();
        }
    }
    h.end_op().unwrap();
    let out = run_heal(&mut h, 5, HealFixes::default());
    assert!(out.closed.is_empty(), "{out:?}");
    assert_eq!(out.skipped.len(), 1);
    assert_eq!(out.skipped[0].fix, "close");
    assert!(out.skipped[0].reason.contains("plane"), "{:?}", out.skipped);
}

#[test]
fn a_lone_sheet_is_not_a_hole() {
    // A free face alone: its boundary is not a hole in anything.
    let (mut h, b) = open_brick();
    h.begin_op(OpId(4)).unwrap();
    for &f in &b.faces[..5] {
        if f != b.faces[4] {
            h.kill(f).unwrap();
        }
    }
    h.end_op().unwrap();
    let before = words(&h);
    let out = run_heal(&mut h, 5, HealFixes::default());
    assert!(out.is_empty(), "{out:?}");
    assert_eq!(words(&h), before);
    assert!(
        out.skipped.iter().any(|s| s.reason.contains("lone face")),
        "{:?}",
        out.skipped
    );
}

#[test]
fn a_gap_welded_in_the_library_raises_every_merged_cell() {
    const G: f64 = 3e-5;
    let (mut h, a) = raw_brick([0.0; 3]);
    h.begin_op(OpId(2)).unwrap();
    let b = brick(&mut h, Point3::new(1.0 + G, 0.0, 0.0), [1.0; 3]).unwrap();
    h.end_op().unwrap();
    let out = run_heal(&mut h, 3, HealFixes::default());
    assert_eq!(out.merged.len(), 9);
    let kinds = |k: MergeKind| out.merged.iter().filter(|m| m.kind == k).count();
    assert_eq!(
        (
            kinds(MergeKind::Vertex),
            kinds(MergeKind::Edge),
            kinds(MergeKind::Face)
        ),
        (4, 4, 1)
    );
    for m in &out.merged {
        assert!(h.tolerance_of(m.keep) >= G, "{m:?}");
    }
    let v: Vec<f64> = [a.volume, b.volume]
        .iter()
        .map(|&v| h.volume_moments(v).unwrap().volume)
        .collect();
    assert!(
        (v[0] - 1.0).abs() <= 1e-12 && (v[1] - (1.0 + G)).abs() <= 1e-12,
        "{v:?}"
    );
    // The raised tolerances do not exceed linear_max.
    assert!(out.raised.iter().all(|r| r.tol <= h.tolerance().linear_max));
}

proptest! {
    #![proptest_config(config(24))]
    #[test]
    fn any_gap_up_to_linear_max_heals_with_the_exact_volume(e in -5.7f64..-3.1) {
        let g = 10f64.powf(e);
        let (mut h, a) = raw_brick([0.0; 3]);
        h.begin_op(OpId(2)).unwrap();
        let b = brick(&mut h, Point3::new(1.0 + g, 0.0, 0.0), [1.0; 3]).unwrap();
        h.end_op().unwrap();
        let out = run_heal(&mut h, 3, HealFixes::default());
        prop_assert_eq!(out.merged.len(), 9, "{:?}", out);
        let va = h.volume_moments(a.volume).unwrap().volume;
        let vb = h.volume_moments(b.volume).unwrap().volume;
        prop_assert!((va - 1.0).abs() <= 1e-12);
        prop_assert!((vb - (1.0 + g)).abs() <= 1e-12);
        prop_assert_eq!(near_faces(&h), 0);
        prop_assert!(validate(&h).is_empty());
        let once = words(&h);
        prop_assert!(run_heal(&mut h, 4, HealFixes::default()).is_empty());
        prop_assert_eq!(words(&h), once);
    }
}

#[test]
fn heal_is_bit_identical_for_one_and_many_threads() {
    let run = || {
        let mut r = two_gap_bricks(4e-5);
        r.brick([1.0; 3], [5.0, 0.5, 0.5]);
        r.brick([1.0; 3], [6.0 + 2e-5, 0.5, 0.5]);
        r.run(&heal_op(None)).unwrap();
        (r.hash(), words(r.topo()))
    };
    let many = run();
    let one = rayon::ThreadPoolBuilder::new()
        .num_threads(1)
        .build()
        .unwrap()
        .install(run);
    assert_eq!(one, many);
    assert_eq!(run(), many, "twice");
}

// ---- the W4-REFCORPUS STEP files

fn corpus_dir() -> Option<PathBuf> {
    let d = std::env::var_os("SM_REFCORPUS_DIR")
        .map(PathBuf::from)
        .or_else(|| {
            std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".cache/smart-mesh/refcorpus"))
        })?;
    d.is_dir().then_some(d)
}

fn import(id: &str, dir: &std::path::Path) -> Option<CellComplex> {
    let bytes = std::fs::read(dir.join(id).join(format!("{id}.step"))).ok()?;
    let m = read_step(
        &bytes,
        &ReadOptions {
            sheets: true,
            ..ReadOptions::default()
        },
    )
    .expect("read_step");
    let mut h = CellComplex::new();
    h.begin_op(OpId(1)).unwrap();
    realize(&m, &mut h, &RealizeOptions::default()).expect("realize");
    h.end_op().unwrap();
    Some(h)
}

fn codes(h: &CellComplex) -> Vec<&'static str> {
    heal_check(h, &HealOptions::default())
        .unwrap()
        .codes()
        .iter()
        .map(|c| c.as_str())
        .collect()
}

/// What heals, per file: `(before, after)`; the files with no tolerant fix are bit-identical.
#[test]
fn corpus_table_what_heals() {
    let Some(dir) = corpus_dir() else {
        eprintln!("skipped: no W4-REFCORPUS cache");
        return;
    };
    // (id, codes before, codes after, volumes after, free faces closed)
    let healed: [(&str, &[&str], &[&str]); 3] = [
        ("step-dirty-gap", &["gap_near"], &[]),
        ("step-dirty-open-shell", &["open_shell"], &[]),
        ("step-dirty-tol-edge", &["gap_near", "open_shell"], &[]),
    ];
    for (id, before, after) in healed {
        let Some(mut h) = import(id, &dir) else {
            eprintln!("skipped {id}: not built");
            continue;
        };
        assert_eq!(codes(&h), before, "{id}");
        let out = run_heal(&mut h, 2, HealFixes::default());
        assert_eq!(codes(&h), after, "{id}: {out:?}");
        assert!(validate(&h).is_empty(), "{id}");
        println!(
            "HEAL {id}: {before:?} -> {after:?}; merged {} raised {} reoriented {} closed {} skipped {}; V{} E{} F{} Vol{}",
            out.merged.len(),
            out.raised.len(),
            out.reoriented.len(),
            out.closed.len(),
            out.skipped.len(),
            h.cells(Dim::Vertex).len(),
            h.cells(Dim::Edge).len(),
            h.cells(Dim::Face).len(),
            h.cells(Dim::Volume).len()
        );
        // The volumes of the gap file: the second moved by the gap, 1e-5.
        if id == "step-dirty-gap" {
            let v = volumes_of(&h);
            assert!(
                (v[0] - 1.0).abs() <= 1e-9 && (v[1] - 1.00001).abs() <= 1e-9,
                "{v:?}"
            );
        }
        let once = words(&h);
        assert!(run_heal(&mut h, 3, HealFixes::default()).is_empty(), "{id}");
        assert_eq!(words(&h), once, "{id} twice");
    }
    // No tolerant fix: untouched (small features are collapses or defeaturing, not welds).
    for id in [
        "step-dirty-notch",
        "step-dirty-step",
        "step-dirty-small-hole",
        "step-dirty-fillet",
        "step-clean-pin",
        "step-clean-cladded-pin",
        "step-clean-assembly-can",
        "step-clean-grid-spacer",
    ] {
        let Some(mut h) = import(id, &dir) else {
            eprintln!("skipped {id}: not built");
            continue;
        };
        let before = (words(&h), codes(&h));
        let out = run_heal(&mut h, 2, HealFixes::default());
        assert!(out.is_empty(), "{id}: {out:?}");
        assert_eq!((words(&h), codes(&h)), before, "{id}");
        println!("HEAL {id}: unchanged, codes {:?}", before.1);
    }
}

#[test]
fn a_coplanar_two_face_sheet_is_not_capped() {
    let mut h = CellComplex::new();
    h.begin_op(OpId(1)).unwrap();
    let b = brick(&mut h, Point3::new(0.0, 0.0, 0.0), [1.0; 3]).unwrap();
    h.end_op().unwrap();
    h.begin_op(OpId(2)).unwrap();
    let w = webcut_brick(&mut h, &b, 0.5).unwrap();
    h.end_op().unwrap();
    // Keep only the two split pieces of the -x side: a planar sheet of two coplanar faces.
    h.begin_op(OpId(3)).unwrap();
    for &v in h.cells(Dim::Body).to_vec().iter() {
        h.kill(v).unwrap();
    }
    for &v in h.cells(Dim::Volume).to_vec().iter() {
        h.kill(v).unwrap();
    }
    let keep = w.split_faces[0];
    for f in h.cells(Dim::Face).to_vec() {
        if f != keep[0] && f != keep[1] {
            h.kill(f).unwrap();
        }
    }
    h.end_op().unwrap();
    let before = words(&h);
    let out = run_heal(&mut h, 4, HealFixes::default());
    assert!(out.closed.is_empty(), "{out:?}");
    assert!(
        out.skipped
            .iter()
            .any(|s| s.reason.contains("planar sheet")),
        "{out:?}"
    );
    assert_eq!(words(&h), before);
    let area: f64 = h
        .cells(Dim::Face)
        .iter()
        .map(|&f| h.face_moments(f).unwrap().area)
        .sum();
    assert!((area - 1.0).abs() <= 1e-12, "area {area}");
}
