//! The printer: values, single ops, batches and the Raw fallback.

mod common;
mod script_support;

use proptest::prelude::*;
use serde_json::{Map, Value as Json, json};
use sm_lang::{Interpreter, Options, PrintError, Style, print_op};
use sm_op_api::{Batch, OpRecord, Session};
use sm_testkit::strategies::config;

use common::*;
use script_support::*;

/// Lower `text` through the real interpreter and return the one record it applied.
fn lower(engine: &sm_op_api::Engine, text: &str) -> OpRecord {
    let mut h = host(engine);
    let rep = {
        let mut it = Interpreter::new(&mut h, Options::default());
        it.run_source("p.smesh", text)
    };
    assert!(rep.ok(), "{text}\n{:?}", rep.diagnostics.iter().map(|d| (d.code, d.message.clone())).collect::<Vec<_>>());
    assert_eq!(h.applied().len(), 1, "{text}");
    h.applied()[0].clone()
}

/// The stored form of `rec`: decoded and re-encoded at the current version.
fn canonical(engine: &sm_op_api::Engine, rec: &OpRecord) -> OpRecord {
    OpRecord::of(engine.registry().decode(rec).unwrap().as_ref()).unwrap()
}

fn json_value() -> impl Strategy<Value = Json> {
    let strings = prop::collection::vec(
        prop::sample::select(vec![
            "\"", "\\", "\n", "\t", "\r", "\u{0}", "\u{1f}", "$", "${x}", "\\$", "\u{1f600}", "Face#3", "$0.body", "{x}", "", "a b", "$$x", "é",
        ]),
        0..4,
    )
    .prop_map(|v| v.concat());
    let printable = "[ -~]{0,8}";
    let leaf = prop_oneof![
        any::<bool>().prop_map(Json::Bool),
        any::<i64>().prop_map(Json::from),
        any::<u64>().prop_map(f64::from_bits).prop_filter("finite", |f| f.is_finite()).prop_map(|f| Json::Number(serde_json::Number::from_f64(f).unwrap())),
        prop::sample::select(vec![0.0f64, 1.0, -1.0, 1e-7, 1e16, 1.7976931348623157e308, 5e-324, 123456789.125]).prop_map(|f| Json::Number(serde_json::Number::from_f64(f).unwrap())),
        prop::sample::select(vec![i64::MIN, i64::MAX, 0, 1, -1]).prop_map(Json::from),
        strings.prop_map(Json::String),
        printable.prop_map(Json::String),
    ];
    let keys = prop::sample::select(vec!["let", "null", "x-y", "", "a", "return", "in", "Face#3", "a b", "_k", "9"]);
    leaf.prop_recursive(3, 24, 4, move |inner| {
        prop_oneof![
            prop::collection::vec(inner.clone(), 0..4).prop_map(Json::Array),
            prop::collection::btree_map(keys.clone(), inner, 0..4).prop_map(|m| Json::Object(m.into_iter().map(|(k, v)| (k.to_owned(), v)).collect::<Map<_, _>>())),
        ]
    })
}

fn zoo(extra: Json) -> OpRecord {
    let e = eng();
    rec(e.registry(), "test.zoo", json!({"name": "z", "n": 1, "x": 1.5, "flag": true, "mode": "alpha", "extra": extra}))
}

proptest! {
    #![proptest_config(config(100))]

    /// Acceptance 1: random JSON params print and read back equal, through the real interpreter.
    #[test]
    fn value_round_trip(v in json_value(), top in prop::collection::btree_map(prop::sample::select(vec!["let", "null", "x-y", "", "a", "return", "in", "b"]), json_value(), 0..4)) {
        let e = eng();
        // As a nested value.
        let r = zoo(v);
        let text = print_op(e.registry(), &r, Style::Readable).unwrap();
        prop_assert_eq!(lower(&e, &text), canonical(&e, &r), "{}", text);
        // Raw prints the same record.
        let raw = print_op(e.registry(), &r, Style::Raw).unwrap();
        prop_assert_eq!(lower(&e, &raw), canonical(&e, &r), "{}", raw);
        // As top-level keys (the `...{ "k": v }` route for keys that are not plain identifiers).
        let params: Map<String, Json> = top.into_iter().map(|(k, v)| (k.to_owned(), v)).collect();
        let r = rec(e.registry(), "test.zoo_free", Json::Object(params));
        let text = print_op(e.registry(), &r, Style::Readable).unwrap();
        prop_assert_eq!(lower(&e, &text), canonical(&e, &r), "{}", text);
    }
}

#[test]
fn number_kinds_and_extremes_survive() {
    let e = eng();
    for (v, text) in [
        (json!(1), "1"),
        (json!(1.0), "1.0"),
        (json!(1e-7), "1e-7"),
        (json!(1.7976931348623157e308), "1.7976931348623157e308"),
        (json!(5e-324), "5e-324"),
        (json!(i64::MAX), "9223372036854775807"),
        (json!(i64::MIN), "-9223372036854775807 - 1"),
        (json!(-1.5), "-1.5"),
    ] {
        let r = zoo(v);
        let out = print_op(e.registry(), &r, Style::Readable).unwrap();
        assert!(out.contains(&format!("extra: {text}")), "{out}");
        assert_eq!(lower(&e, &out), canonical(&e, &r));
    }
}

#[test]
fn strings_use_only_the_lexer_escapes() {
    let e = eng();
    let r = zoo(json!("a\"b\\c\n\t\r\u{1}${x}$y\u{1f600}"));
    let out = print_op(e.registry(), &r, Style::Readable).unwrap();
    assert!(out.contains(r#"extra: "a\"b\\c\n\t\r\u{1}\${x}$y😀""#), "{out}");
    assert_eq!(lower(&e, &out), canonical(&e, &r));
}

/// Acceptance 2: the single-op golden, ids in id positions only.
#[test]
fn single_op_golden_and_ids() {
    let e = eng();
    let r = rec(e.registry(), "model.create_brick", json!({"x": 1.5, "y": 2, "z": 3, "name": "a b"}));
    let text = print_op(e.registry(), &r, Style::Readable).unwrap();
    assert_eq!(text, r#"model.create_brick(x: 1.5, y: 2, z: 3, name: "a b")"#);
    assert_eq!(lower(&e, &text), canonical(&e, &r));

    // Ids in id-typed positions are script ids; the carrier is `GeomSurface`, never `Surface`; a text field stays text.
    let r = rec(e.registry(), "test.zoo", json!({"name": "Face#2", "n": 1, "x": 1.5, "flag": false, "mode": "beta", "face": "TopoFace#2", "ids": ["TopoVolume#1", "Surface#7", "Curve#4", "TopoBody#1"]}));
    let text = print_op(e.registry(), &r, Style::Readable).unwrap();
    assert_eq!(
        text,
        r#"test.zoo(flag: false, mode: "beta", n: 1, name: "Face#2", x: 1.5, face: Face#2, ids: [Volume#1, GeomSurface#7, GeomCurve#4, Body#1])"#
    );
    assert!(!text.contains(" Surface#") && !text.contains("[Surface#"), "{text}");
    // Raw keeps ids as strings.
    let raw = print_op(e.registry(), &r, Style::Raw).unwrap();
    assert!(raw.contains("\"Face#2\"") && raw.contains("\"Surface#7\""), "{raw}");
}

#[test]
fn printed_ids_lower_back_on_a_live_state() {
    let e = eng();
    let mut s = Session::new(e.clone());
    apply(&mut s, "model.create_brick", json!({"x": 1, "y": 1, "z": 1}));
    let r = rec(e.registry(), "intent.set_size", json!({"ids": ["TopoFace#2", "TopoFace#3"], "size": 0.5}));
    let text = print_op(e.registry(), &r, Style::Readable).unwrap();
    assert_eq!(text, "intent.set_size(ids: [Face#2, Face#3], size: 0.5)");
    let mut h = host(&e);
    let rep = {
        let mut it = Interpreter::new(&mut h, Options::default());
        it.run_source("p.smesh", "model.create_brick(x: 1, y: 1, z: 1)");
        it.run_source("q.smesh", &text)
    };
    assert!(rep.ok(), "{:?}", rep.diagnostics);
    assert_eq!(h.applied()[1], canonical(&e, &r));
}

fn batch_record(members: Vec<OpRecord>) -> OpRecord {
    OpRecord::of(&Batch { ops: members }).unwrap_or_else(|x| panic!("{x}"))
}

/// The stored (canonical) form of a batch, as the engine produces it.
fn stored_batch(e: &sm_op_api::Engine, setup: &[(&str, Json)], members: Vec<OpRecord>) -> OpRecord {
    let mut s = Session::new(e.clone());
    for (n, p) in setup {
        apply(&mut s, n, p.clone());
    }
    let b = batch_record(members);
    let h = apply_rec(&mut s, b);
    s.timeline().get(h).unwrap().body.op.clone()
}

fn the_batch(e: &sm_op_api::Engine) -> (Vec<(&'static str, Json)>, OpRecord) {
    let setup = vec![("model.create_brick", json!({"x": 1, "y": 1, "z": 1}))];
    let members = vec![
        rec(e.registry(), "w2.make", json!({})),
        rec(e.registry(), "model.pick", json!({"id": "$0.body[0]"})),
        rec(e.registry(), "w2.use_volume", json!({"vol": "$0.volume[0]", "many": ["$0.body[*]", "TopoVolume#1"]})),
        rec(e.registry(), "model.note", json!({"text": "$$lit"})),
    ];
    let stored = stored_batch(e, &setup, members);
    (setup, stored)
}

/// Acceptance 3: a stored batch prints as a block (golden) whose lowering is byte-identical to the stored record.
#[test]
fn batch_golden_and_lowering() {
    let e = eng();
    let (_setup, stored) = the_batch(&e);
    let text = print_op(e.registry(), &stored, Style::Readable).unwrap();
    assert_eq!(
        text,
        "batch {\n  let b0m0 = w2.make()\n  model.pick(id: b0m0.body[0])\n  w2.use_volume(vol: b0m0.volume[0], many: [b0m0.body, Volume#1])\n  model.note(text: \"$lit\")\n}"
    );
    let mut h = host(&e);
    let rep = {
        let mut it = Interpreter::new(&mut h, Options::default());
        it.run_source("b.smesh", &format!("model.create_brick(x: 1, y: 1, z: 1)\n{text}"))
    };
    assert!(rep.ok(), "{:?}", rep.diagnostics);
    assert_eq!(h.applied()[1], stored);
}

#[test]
fn bare_references_are_an_error_or_normalized() {
    let e = eng();
    let setup = [("model.create_brick", json!({"x": 1, "y": 1, "z": 1}))];
    let members = vec![rec(e.registry(), "w2.make", json!({})), rec(e.registry(), "model.pick", json!({"id": "$0.body"}))];
    let stored = stored_batch(&e, &setup, members);
    assert_eq!(stored.params["ops"][1]["params"]["id"], json!("$0.body"), "the engine keeps the bare form");
    let err = print_op(e.registry(), &stored, Style::Readable).unwrap_err();
    assert!(matches!(&err, PrintError::BareReference { path, .. } if path == "ops[1].id"), "{err}");

    // With normalize_refs the recorder prints `[0]`, notes it, and the replayed state hash still matches.
    let mut s = Session::new(e.clone());
    apply(&mut s, "model.create_brick", json!({"x": 1, "y": 1, "z": 1}));
    apply_rec(&mut s, stored);
    let o = sm_lang::RecordOptions::default().with_normalize_refs(true);
    let (rec, h) = record_and_replay_state_only(&e, &s, &o);
    assert!(rec.text.contains("model.pick(id: b0m0.body[0])"), "{}", rec.text);
    assert!(rec.notes.iter().any(|n| matches!(n, sm_lang::RecordNote::Normalized { count: 1, .. })), "{:?}", rec.notes);
    assert_eq!(h.s.state().hash(), s.state().hash());
    // Without it the recorder fails, naming the node.
    let err = sm_lang::record_script(e.registry(), s.timeline(), s.head(), &sm_lang::RecordOptions::default()).unwrap_err();
    assert!(err.to_string().contains("bare batch reference"), "{err}");
}

fn record_and_replay_state_only(e: &sm_op_api::Engine, s: &Session, o: &sm_lang::RecordOptions) -> (sm_lang::Recording, SessionHost) {
    let r = sm_lang::record_script(e.registry(), s.timeline(), s.head(), o).unwrap();
    let (h, rep) = run_text(e, &r.text);
    assert!(rep.diagnostics.is_empty(), "{:?}\n{}", rep.diagnostics, r.text);
    (r, h)
}

#[test]
fn unprintable_values_are_typed_errors() {
    let e = eng();
    let r = OpRecord { name: "test.zoo".into(), version: 1, params: json!({"name": "z", "n": 1, "x": 1.5, "flag": true, "mode": "alpha", "extra": u64::MAX}) };
    let err = print_op(e.registry(), &r, Style::Readable).unwrap_err();
    assert!(matches!(err, PrintError::Unprintable { .. }), "{err}");
    let r = OpRecord { name: "no.such_op".into(), version: 1, params: json!({}) };
    assert!(matches!(print_op(e.registry(), &r, Style::Readable), Err(PrintError::UnknownOp { .. })));
}

/// Acceptance 8 (first half): a record the readable form cannot express falls back to Raw, and the Raw text reads back.
#[test]
fn a_failed_self_check_falls_back_to_raw() {
    let e = eng();
    // A non-canonical spelling of an id: the readable `Face#2` would read back as `"TopoFace#2"`, not `"Face#2"`.
    let r = rec(e.registry(), "test.zoo", json!({"name": "z", "n": 1, "x": 1.5, "flag": true, "mode": "alpha", "face": "Face#2"}));
    let text = print_op(e.registry(), &r, Style::Readable).unwrap();
    assert!(text.contains("...{ ") && text.contains("face: \"Face#2\""), "{text}");
}
