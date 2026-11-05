//! The recorder: linear history, fork topologies, head, corpus, requirements, verification, determinism.

mod common;
mod script_support;

use std::collections::BTreeSet;
use std::hash::Hasher;

use serde_json::{Value as Json, json};
use sm_lang::{Check, Forks, RecordError, RecordNote, RecordOptions, Requirement, Style, record_script, verify_recording};
use sm_op_api::{Batch, Check as NodeCheck, Engine, Hash256, Head, Node, NodeBody, OpRecord, QuietControl, Rev, Session, Timeline};
use sm_testkit::determinism::{Fnv, check};

use script_support::*;

fn batch_one(e: &Engine, name_after: &str) -> OpRecord {
    let r = e.registry();
    OpRecord::of(&Batch {
        ops: vec![
            rec(r, "w2.make", json!({})),
            rec(r, "model.pick", json!({"id": "$0.body[0]"})),
            rec(r, "w2.use_volume", json!({"vol": "$0.volume[0]", "many": ["$0.body[*]", "TopoVolume#1"]})),
            rec(r, "model.note", json!({"text": "$$lit"})),
            rec(r, "model.note", json!({"text": name_after})),
        ],
    })
    .unwrap()
}

/// Replace the stored op of a head node by an older version of itself (a v1 record that migrates to the same state).
fn with_v1_tail(e: &Engine, s: &Session, v1: OpRecord) -> Session {
    let mut tl = s.timeline().clone();
    let mut st = s.state().clone();
    let q = QuietControl::new();
    e.apply_record_canonical(&mut st, &v1, &q.control()).unwrap();
    let current = OpRecord::of(e.registry().decode(&v1).unwrap().as_ref()).unwrap();
    let check = NodeCheck::of_with(e.registry(), e.services(), &st, &current);
    let h = tl.insert(Node { body: NodeBody { parent: s.position(), op: v1, meta: meta() }, check }).unwrap();
    tl.set_ref("main", h).unwrap();
    Session::open(e.clone(), tl, Head::Ref("main".into()), &q.control()).unwrap()
}

/// Twelve mixed ops: two batches and a v1 record that migrates.
fn linear(e: &Engine) -> Session {
    let r = e.registry();
    let mut s = Session::new(e.clone());
    apply(&mut s, "model.create_brick", json!({"x": 1.5, "y": 2, "z": 3, "name": "a b"}));
    note(&mut s, "hello \"q\"");
    apply_rec(&mut s, batch_one(e, "after"));
    apply(
        &mut s,
        "test.zoo",
        json!({"name": "z", "n": -3, "x": 1e-7, "flag": true, "mode": "beta", "nested": {"a": [1.0, 2.5], "b": "t"}, "list": ["a", "b"], "face": "TopoFace#2", "ids": ["TopoVolume#1"], "extra": {"k": [1, 2.0, "s"]}}),
    );
    apply(&mut s, "intent.set_size", json!({"ids": ["TopoFace#2", "TopoFace#3"], "size": 0.5}));
    apply(&mut s, "mesh.run", json!({"targets": ["TopoVolume#1"], "scheme": "map", "merge": true, "order": 2}));
    let b2 = OpRecord::of(&Batch { ops: vec![rec(r, "w2.make", json!({})), rec(r, "model.pick", json!({"id": "$0.volume[0]"}))] }).unwrap();
    apply_rec(&mut s, b2);
    let v1 = OpRecord { name: "test.legacy".into(), version: 1, params: json!({"name": "old", "count": 3}) };
    let mut s = with_v1_tail(e, &s, v1);
    apply(&mut s, "model.scale_brick", json!({"id": "TopoBody#1", "factor": 2.0}));
    note(&mut s, "tail");
    apply(&mut s, "test.tag", json!({"label": "x", "n": 3}));
    apply(&mut s, "intent.set_size", json!({"ids": ["TopoFace#4"], "size": 0.25}));
    s
}

fn hash_hex(s: &Session) -> String {
    s.state().hash().to_hex()
}

fn lines(text: &str) -> Vec<&str> {
    text.lines().collect()
}

/// Acceptance 4: linear history, `Forks::Head`.
#[test]
fn linear_history_replays_to_the_same_state() {
    let e = eng();
    let s = linear(&e);
    assert_eq!(s.timeline().len(), 12);
    let o = RecordOptions::default().with_check(Check::Every);
    let (rec, h) = record_and_replay(&e, &s, &o);
    let (_, rep) = run_text(&e, &rec.text);
    assert!(rep.diagnostics.is_empty(), "{:?}", rep.diagnostics);
    assert_eq!(h.s.state().hash(), s.state().hash());
    assert_eq!(h.s.timeline().len(), s.timeline().len());
    let last = *lines(&rec.text).last().unwrap();
    assert_eq!(last, format!("assert session.hash() == \"{}\"", hash_hex(&s)));
    assert!(rec.notes.iter().any(|n| matches!(n, RecordNote::Migrated { from: 1, to: 2, .. })), "{:?}", rec.notes);
    assert_eq!(rec.nodes, 12);
    // The two batches use distinct variables.
    assert!(rec.text.contains("b0m0") && rec.text.contains("b1m0"), "{}", rec.text);
    // verify_recording agrees.
    let v = verify_recording(&e, s.timeline(), s.head(), &rec).unwrap();
    assert!(v.all_equal() && v.clean, "{v:?}");
    assert_eq!(v.statements, rec.statements);
}

/// The default checks only the tip, and the header carries no time, author or hash of a node.
#[test]
fn default_check_is_the_tip_and_the_header_is_stable() {
    let e = eng();
    let s = linear(&e);
    let rec = record_script(e.registry(), s.timeline(), s.head(), &RecordOptions::default()).unwrap();
    assert_eq!(rec.text.lines().filter(|l| l.starts_with("assert")).count(), 1);
    assert!(rec.text.starts_with("// smart-mesh script record\n// source: head main, forks head, 1 ref tip(s), 12 node(s)\n// replay on an empty project\n\n"), "{}", rec.text);
    let node_hexes: Vec<String> = s.timeline().nodes().map(|(h, _)| h.to_hex()).collect();
    assert!(node_hexes.iter().all(|h| !rec.text.contains(h.as_str())), "node hashes must not appear");
    assert!(!rec.text.contains("undo") && !rec.text.contains("redo"));
    let none = record_script(e.registry(), s.timeline(), s.head(), &RecordOptions::default().with_check(Check::None).with_header(false)).unwrap();
    assert!(!none.text.contains("assert") && !none.text.contains("//"));
}

fn names(t: &Timeline) -> Vec<String> {
    t.refs().keys().cloned().collect()
}

fn refs_replay(e: &Engine, s: &Session) -> (sm_lang::Recording, SessionHost) {
    let o = RecordOptions::default().with_forks(Forks::Refs).with_check(Check::Every);
    let (rec, h) = record_and_replay(e, s, &o);
    assert_eq!(names(s.timeline()), names(h.s.timeline()), "{}", rec.text);
    for (n, tip) in s.timeline().refs() {
        assert_eq!(s.timeline().get(*tip).unwrap().check.state, h.s.timeline().get(h.s.timeline().refs()[n]).unwrap().check.state);
    }
    let v = verify_recording(e, s.timeline(), s.head(), &rec).unwrap();
    assert!(v.all_equal(), "{v:?}\n{}", rec.text);
    (rec, h)
}

fn base(e: &Engine, n: usize) -> (Session, Vec<Hash256>) {
    let mut s = Session::new(e.clone());
    let hs = (0..n).map(|i| note(&mut s, &format!("m{i}"))).collect();
    (s, hs)
}

/// Acceptance 5 (a): a fork in the middle of `main`.
#[test]
fn fork_in_the_middle() {
    let e = eng();
    let (mut s, hs) = base(&e, 4);
    checkout(&mut s, Rev::Node(hs[1]));
    s.branch("f").unwrap();
    note(&mut s, "f1");
    note(&mut s, "f2");
    let (rec, h) = refs_replay(&e, &s);
    assert_eq!(h.s.head(), s.head());
    assert_eq!(rec.refs.len(), 2);
}

/// (b): a ref at an interior node of another ref.
#[test]
fn ref_at_an_interior_node() {
    let e = eng();
    let (mut s, hs) = base(&e, 3);
    s.create_branch_at("tag", hs[1]).unwrap();
    refs_replay(&e, &s);
}

/// (c): two refs on one node.
#[test]
fn two_refs_on_one_node() {
    let e = eng();
    let (mut s, _) = base(&e, 3);
    s.branch("also").unwrap();
    let (rec, _) = refs_replay(&e, &s);
    assert_eq!(rec.refs.len(), 2);
}

/// (d): forks nested three deep.
#[test]
fn nested_forks_three_deep() {
    let e = eng();
    let (mut s, hs) = base(&e, 5);
    checkout(&mut s, Rev::Node(hs[1]));
    s.branch("f1").unwrap();
    let a = note(&mut s, "f1a");
    note(&mut s, "f1b");
    note(&mut s, "f1c");
    checkout(&mut s, Rev::Node(a));
    s.branch("f2").unwrap();
    let b = note(&mut s, "f2a");
    note(&mut s, "f2b");
    checkout(&mut s, Rev::Node(b));
    s.branch("f3").unwrap();
    note(&mut s, "f3a");
    let (rec, _) = refs_replay(&e, &s);
    assert_eq!(rec.refs.len(), 4);
}

/// (e): two roots.
#[test]
fn two_roots() {
    let e = eng();
    let (mut s, _) = base(&e, 2);
    checkout(&mut s, Rev::Empty);
    note(&mut s, "other root");
    s.branch("other").unwrap();
    note(&mut s, "other 2");
    let (rec, _) = refs_replay(&e, &s);
    assert_eq!(s.timeline().roots().len(), 2);
    assert_eq!(rec.text.matches("timeline.checkout(empty: true)").count(), 2, "{}", rec.text);
}

/// (f): a source with no `main`: the replay has none either.
#[test]
fn no_main() {
    let e = eng();
    let (mut s, _) = base(&e, 3);
    s.branch("x").unwrap();
    s.delete_branch("main").unwrap();
    let (_, h) = refs_replay(&e, &s);
    assert!(!h.s.timeline().refs().contains_key("main"));
    assert_eq!(names(h.s.timeline()), vec!["x"]);
}

/// (g): a detached head.
#[test]
fn detached_head() {
    let e = eng();
    let (mut s, hs) = base(&e, 3);
    checkout(&mut s, Rev::Node(hs[1]));
    let (_, h) = refs_replay(&e, &s);
    assert_eq!(names(h.s.timeline()), vec!["main"]);
    // Forks::Head from a detached head records the path to it.
    let rec = record_script(e.registry(), s.timeline(), s.head(), &RecordOptions::default()).unwrap();
    assert_eq!(rec.nodes, 2);
    assert!(rec.refs.is_empty());
    let (h2, rep) = run_text(&e, &rec.text);
    assert!(rep.ok());
    assert_eq!(h2.s.state().hash(), s.state().hash());
}

/// (h): an unborn head gives a header-only script.
#[test]
fn unborn_head() {
    let e = eng();
    let s = Session::new(e.clone());
    for forks in [Forks::Head, Forks::Refs] {
        let rec = record_script(e.registry(), s.timeline(), s.head(), &RecordOptions::default().with_forks(forks)).unwrap();
        assert_eq!((rec.nodes, rec.statements), (0, 0));
        assert!(rec.text.lines().all(|l| l.starts_with("//")), "{}", rec.text);
        let (h, rep) = run_text(&e, &rec.text);
        assert!(rep.diagnostics.is_empty() && h.s.timeline().is_empty());
        let v = verify_recording(&e, s.timeline(), s.head(), &rec).unwrap();
        assert!(v.all_equal());
    }
}

/// (i): nodes made orphan by `undo` are counted and absent.
#[test]
fn orphans_are_counted_and_absent() {
    let e = eng();
    let (mut s, _) = base(&e, 3);
    note(&mut s, "doomed");
    s.undo().unwrap();
    note(&mut s, "kept");
    let (rec, _) = refs_replay(&e, &s);
    assert!(rec.notes.contains(&RecordNote::Orphans { count: 1 }), "{:?}", rec.notes);
    assert!(!rec.text.contains("doomed") && rec.text.contains("kept"));
}

/// Acceptance 6: the head after replay.
#[test]
fn head_after_replay() {
    let e = eng();
    let (mut s, _) = base(&e, 2);
    s.branch("dev").unwrap();
    note(&mut s, "dev1");
    assert_eq!(s.head(), &Head::Ref("dev".into()));

    // Forks::Head on a non-main head ends on that ref; `main` also exists at the tip (documented).
    let rec = record_script(e.registry(), s.timeline(), s.head(), &RecordOptions::default()).unwrap();
    assert!(lines(&rec.text).contains(&"timeline.branch(\"dev\")"), "{}", rec.text);
    let (h, rep) = run_text(&e, &rec.text);
    assert!(rep.diagnostics.is_empty(), "{:?}", rep.diagnostics);
    assert_eq!(h.s.head(), &Head::Ref("dev".into()));
    assert_eq!(names(h.s.timeline()), vec!["dev", "main"]);
    assert_eq!(h.s.timeline().refs()["main"], h.s.timeline().refs()["dev"]);
    assert_eq!(h.s.state().hash(), s.state().hash());
    let v = verify_recording(&e, s.timeline(), s.head(), &rec).unwrap();
    assert!(v.all_equal() && v.head.as_deref() == Some("dev"), "{v:?}");

    // Forks::Refs reproduces exactly the source refs and the attached head.
    let (_, h) = refs_replay(&e, &s);
    assert_eq!(h.s.head(), &Head::Ref("dev".into()));

    // An attached main head leaves no extra ref.
    let (s, _) = base(&e, 2);
    let rec = record_script(e.registry(), s.timeline(), s.head(), &RecordOptions::default()).unwrap();
    let (h, _) = run_text(&e, &rec.text);
    assert_eq!(names(h.s.timeline()), vec!["main"]);
    assert_eq!(h.s.head(), &Head::Ref("main".into()));
}

fn golden_scripts() -> Vec<(String, String)> {
    let mut out = Vec::new();
    for dir in ["tests/golden/run", "tests/golden/parse", "tests/golden/w2"] {
        let mut paths: Vec<_> = std::fs::read_dir(format!("{}/{dir}", env!("CARGO_MANIFEST_DIR"))).unwrap().map(|e| e.unwrap().path()).filter(|p| p.extension().is_some_and(|x| x == "smesh")).collect();
        paths.sort();
        for p in paths {
            out.push((p.file_name().unwrap().to_string_lossy().into_owned(), std::fs::read_to_string(&p).unwrap()));
        }
    }
    // The golden scripts are mostly error cases; these valid ones exercise loops, batches and timeline commands.
    for (n, t) in [
        ("inline-loop", "for i in 1..3 { model.create_brick(x: i, y: 1, z: 1) }\nintent.set_size(ids: Face#[1..3], size: 0.5)\nmesh.run(targets: all volumes, scheme: \"pave\")\n"),
        ("inline-batch", "w2.make()\nbatch {\n  let a = w2.make()\n  w2.use_volume(vol: a.volume[0], many: [a.body, a.volume])\n  model.note(text: \"$x\")\n}\n"),
        (
            "inline-timeline",
            "model.note(text: \"a\")\nmodel.note(text: \"b\")\ntimeline.branch(\"x\")\nmodel.note(text: \"c\")\ntimeline.undo()\nmodel.note(text: \"d\")\ntimeline.checkout(\"main\")\nmodel.note(text: \"e\")\n",
        ),
    ] {
        out.push((n.to_owned(), t.to_owned()));
    }
    out
}

/// Acceptance 7: every golden script, run (a failing statement stays failed, earlier ones stay applied), recorded with
/// both policies and `Check::Every`, replays to the same state.
#[test]
fn the_golden_corpus_records_and_replays() {
    let e = eng();
    let (mut with_nodes, mut total) = (0, 0);
    for (name, text) in golden_scripts() {
        total += 1;
        let (h, _) = run_text(&e, &text);
        if h.s.timeline().is_empty() {
            continue;
        }
        with_nodes += 1;
        for forks in [Forks::Head, Forks::Refs] {
            let o = RecordOptions::default().with_forks(forks).with_check(Check::Every);
            let rec = record_script(e.registry(), h.s.timeline(), h.s.head(), &o).unwrap_or_else(|x| panic!("{name} {forks:?}: {x}"));
            let (r, rep) = run_text(&e, &rec.text);
            assert!(rep.ok(), "{name} {forks:?}: {:?}\n{}", rep.diagnostics.iter().map(|d| (d.code, d.message.clone())).collect::<Vec<_>>(), rec.text);
            for (rname, tip) in &rec.refs {
                let rt = r.s.timeline().refs()[rname];
                assert_eq!(chain(e.registry(), h.s.timeline(), *tip), chain(e.registry(), r.s.timeline(), rt), "{name} {forks:?} {rname}");
            }
            if forks == Forks::Head && matches!(h.s.head(), Head::Ref(_)) {
                assert_eq!(r.s.state().hash(), h.s.state().hash(), "{name}");
            }
        }
    }
    assert!(total >= 60 && with_nodes >= 5, "{with_nodes}/{total}");
}

/// Acceptance 8 (second half) and verification failures name the statement and the node.
#[test]
fn verify_names_the_failing_statement_and_node() {
    let e = eng();
    let mut s = Session::new(e.clone());
    let first = apply(&mut s, "model.create_brick", json!({"x": 1.5, "y": 2, "z": 3}));
    note(&mut s, "a");
    let third = note(&mut s, "b");
    note(&mut s, "c");
    let o = RecordOptions::default().with_check(Check::Every);
    let rec = record_script(e.registry(), s.timeline(), s.head(), &o).unwrap();
    assert_eq!(rec.statements, 8);

    // A literal changed to an invalid value: the op fails and had no effect.
    let bad = with_text(&rec, rec.text.replace("x: 1.5", "x: -1.5"));
    let err = verify_recording(&e, s.timeline(), s.head(), &bad).unwrap_err();
    match err {
        RecordError::Replay { statement, node, code, nodes_applied, .. } => {
            assert_eq!((statement, node, code.as_str(), nodes_applied), (1, Some(first), "op-failed", 0));
        }
        other => panic!("{other}"),
    }

    // A literal changed to another valid value: the next hash check fails and names its node.
    let bad = with_text(&rec, rec.text.replace("text: \"b\"", "text: \"B\""));
    let err = verify_recording(&e, s.timeline(), s.head(), &bad).unwrap_err();
    match err {
        RecordError::Replay { statement, node, code, nodes_applied, .. } => {
            assert_eq!((statement, node, code.as_str(), nodes_applied), (6, Some(third), "assert-failed", 3));
        }
        other => panic!("{other}"),
    }
    assert!(err_text(&e, &s, &rec.text.replace("text: \"b\"", "text: \"B\"")).contains("statement 6"));

    // Without hash checks the corrupted script runs, and the report says the tip differs.
    let o = RecordOptions::default().with_check(Check::None);
    let rec = record_script(e.registry(), s.timeline(), s.head(), &o).unwrap();
    let bad = with_text(&rec, rec.text.replace("text: \"b\"", "text: \"B\""));
    let v = verify_recording(&e, s.timeline(), s.head(), &bad).unwrap();
    assert!(!v.refs["main"].tip_state_equal && !v.refs["main"].records_equal && !v.all_equal());
}

fn with_text(rec: &sm_lang::Recording, text: String) -> sm_lang::Recording {
    let mut r = rec.clone();
    r.text = text;
    r
}

fn err_text(e: &Engine, s: &Session, text: &str) -> String {
    let o = RecordOptions::default().with_check(Check::Every);
    let rec = record_script(e.registry(), s.timeline(), s.head(), &o).unwrap();
    verify_recording(e, s.timeline(), s.head(), &with_text(&rec, text.to_owned())).unwrap_err().to_string()
}

/// A readable form that cannot be expressed falls back to Raw with a note, and the replay still matches (state hash).
#[test]
fn raw_fallback_is_noted_and_replays() {
    let e = eng();
    let mut s = Session::new(e.clone());
    apply(&mut s, "model.create_brick", json!({"x": 1, "y": 1, "z": 1}));
    let v1 = OpRecord {
        name: "test.zoo".into(),
        version: 1,
        // `Face#2` is a lenient spelling of `TopoFace#2`; the readable form would canonicalize it.
        params: json!({"name": "z", "n": 1, "x": 1.5, "flag": true, "mode": "alpha", "face": "Face#2"}),
    };
    let s = with_tail_unchanged(&e, &s, v1);
    let rec = record_script(e.registry(), s.timeline(), s.head(), &RecordOptions::default()).unwrap();
    assert!(rec.notes.iter().any(|n| matches!(n, RecordNote::RawFallback { .. })), "{:?}", rec.notes);
    assert!(rec.text.contains("test.zoo(...{ "), "{}", rec.text);
    let (h, rep) = run_text(&e, &rec.text);
    assert!(rep.diagnostics.is_empty(), "{:?}\n{}", rep.diagnostics, rec.text);
    assert_eq!(h.s.state().hash(), s.state().hash());
}

/// Like `with_v1_tail` for a record already at the current version (kept as written, not canonicalized).
fn with_tail_unchanged(e: &Engine, s: &Session, r: OpRecord) -> Session {
    with_v1_tail(e, s, r)
}

/// An id the op accepted but the language considers dead: the replay fails naming the statement and node. Raw spells
/// the id as a string, which the language checks the same way, so Raw does not help here.
#[test]
fn a_dead_literal_id_is_reported_with_its_node() {
    let e = eng();
    let mut s = Session::new(e.clone());
    let dead = apply(&mut s, "test.zoo", json!({"name": "z", "n": 1, "x": 1.5, "flag": true, "mode": "alpha", "face": "TopoFace#9"}));
    for style in [Style::Readable, Style::Raw] {
        let o = RecordOptions::default().with_style(style);
        let rec = record_script(e.registry(), s.timeline(), s.head(), &o).unwrap();
        match verify_recording(&e, s.timeline(), s.head(), &rec).unwrap_err() {
            RecordError::Replay { statement, node, code, hint, nodes_applied, .. } => {
                assert_eq!((statement, node, code.as_str(), nodes_applied), (1, Some(dead), "entity-not-found", 0));
                assert!(hint.is_some_and(|h| h.contains("Raw")));
            }
            other => panic!("{other}"),
        }
    }
}

/// Acceptance 9: requirements and limits.
#[test]
fn requirements_and_limits() {
    let e = eng();
    let sha = "ab".repeat(32);
    let mut s = Session::new(e.clone());
    apply(&mut s, "test.import", json!({"source": {"sha256": sha}, "label": "part"}));
    let rec = record_script(e.registry(), s.timeline(), s.head(), &RecordOptions::default()).unwrap();
    assert_eq!(rec.requires, vec![Requirement::Input { sha256: sha.clone() }]);
    assert!(rec.text.contains(&format!("// requires input sha256:{sha}\n")), "{}", rec.text);

    // A string over the language's value limit is noted.
    let mut s = Session::new(e.clone());
    let big = "a".repeat(1_000_001);
    let h = note(&mut s, &big);
    let rec = record_script(e.registry(), s.timeline(), s.head(), &RecordOptions::default()).unwrap();
    assert!(rec.notes.contains(&RecordNote::LargeValue { node: h, bytes: 1_000_001 }), "{:?}", rec.notes);
    assert!(rec.notes.iter().any(|n| n.to_string().contains("stage")));

    // A script longer than the parse limit is an error.
    let mut s = Session::new(e.clone());
    note(&mut s, &"a".repeat((16 << 20) + 1));
    match record_script(e.registry(), s.timeline(), s.head(), &RecordOptions::default()) {
        Err(RecordError::TooLarge { bytes, limit }) => assert!(bytes > limit && limit == 16 << 20),
        other => panic!("{other:?}"),
    }
}

fn text_hash(text: &str) -> u64 {
    let mut f = Fnv::default();
    f.write(text.as_bytes());
    f.finish()
}

/// Acceptance 10: determinism.
#[test]
fn recording_is_deterministic() {
    let e = eng();
    let s = linear(&e);
    let reg = e.registry();
    // A forked timeline as well.
    let (mut f, hs) = base(&e, 4);
    checkout(&mut f, Rev::Node(hs[1]));
    f.branch("f").unwrap();
    note(&mut f, "f1");
    for (tl, head, forks) in [(s.timeline(), s.head(), Forks::Head), (f.timeline(), f.head(), Forks::Refs)] {
        let o = RecordOptions::default().with_forks(forks).with_check(Check::Every);
        let go = |_: usize| text_hash(&record_script(reg, tl, head, &o).unwrap().text);
        let h = check(3, 4, go).unwrap();
        // A timeline rebuilt from its files records the same bytes.
        let rebuilt = Timeline::from_files(&tl.to_files().unwrap()).unwrap();
        assert_eq!(text_hash(&record_script(reg, &rebuilt, head, &o).unwrap().text), h);
        // The default pool against one thread.
        let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().unwrap();
        assert_eq!(one.install(|| go(0)), h);
    }
}

/// Child order depends on node hashes, not on insertion order.
#[test]
fn sibling_insertion_order_does_not_matter() {
    let e = eng();
    let (mut s, hs) = base(&e, 2);
    checkout(&mut s, Rev::Node(hs[0]));
    s.branch("b").unwrap();
    note(&mut s, "sib b");
    checkout(&mut s, Rev::Ref("main".into()));
    let src = s.timeline();
    let nodes: Vec<Node> = src.nodes().map(|(_, n)| n.clone()).collect();
    let depth = |n: &Node| src.depth(n.hash().unwrap()).unwrap();
    let mut fwd = nodes.clone();
    fwd.sort_by_key(|n| (depth(n), n.hash().unwrap()));
    let mut rev = nodes;
    rev.sort_by_key(|n| (depth(n), std::cmp::Reverse(n.hash().unwrap())));
    let build = |order: &[Node]| {
        let mut t = Timeline::new();
        for n in order {
            t.insert(n.clone()).unwrap();
        }
        for (name, h) in src.refs() {
            t.set_ref(name, *h).unwrap();
        }
        t
    };
    let (a, b) = (build(&fwd), build(&rev));
    let o = RecordOptions::default().with_forks(Forks::Refs);
    let (ra, rb) = (record_script(e.registry(), &a, s.head(), &o).unwrap(), record_script(e.registry(), &b, s.head(), &o).unwrap());
    assert_eq!(ra.text, rb.text);
    assert_eq!(ra.text, record_script(e.registry(), src, s.head(), &o).unwrap().text);
    let set: BTreeSet<_> = ra.refs.keys().collect();
    assert_eq!(set.len(), 2);
    let _ = Json::Null;
}
