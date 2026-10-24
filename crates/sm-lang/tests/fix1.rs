//! Language decisions and review fixes of `fix1`: literal strings, ambiguous `Surface#`, the contract `Rev`,
//! reserved words, batch diagnostics, printing in source form.

mod common;

use sm_lang::diag::render_all;
use sm_lang::{Diagnostic, EngineHost, Event, Host, HostError, Interpreter, Options, PlanHost, RunReport, SourceMap, TimelineCommand};
use sm_op_api::Rev;

use common::*;

/// A node hash as the timeline prints it: 64 lowercase hex characters.
const HASH: &str = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";

fn run(src: &str) -> (RunReport, sm_op_api::State, SourceMap) {
    let engine = engine();
    let mut h = host(&engine);
    let (rep, map) = {
        let mut it = Interpreter::new(&mut h, Options::default());
        let rep = it.run_source("t.smesh", src);
        (rep, it.sources().clone())
    };
    (rep, h.into_state(), map)
}

fn ok(src: &str) -> (RunReport, sm_op_api::State) {
    let (rep, st, map) = run(src);
    assert!(rep.ok(), "{src}\n{}", render_all(&rep.diagnostics, &map));
    (rep, st)
}

fn err_map(src: &str) -> (Diagnostic, SourceMap) {
    let (rep, _, map) = run(src);
    (rep.diagnostics.into_iter().find(Diagnostic::is_error).unwrap_or_else(|| panic!("`{src}` ran without an error")), map)
}

fn err(src: &str) -> Diagnostic {
    err_map(src).0
}

fn text_of(map: &SourceMap, d: &Diagnostic) -> String {
    let s = map.get(d.span.file).expect("source");
    s.text[d.span.lo as usize..d.span.hi as usize].to_owned()
}

fn prints(rep: &RunReport) -> Vec<String> {
    rep.events.iter().filter_map(|e| if let Event::Print { text, .. } = e { Some(text.clone()) } else { None }).collect()
}

fn notes(st: &sm_op_api::State) -> Vec<String> {
    st.layer::<DemoModel>().unwrap().notes.clone()
}

// ---- strings are literal -----------------------------------------------------------------------

#[test]
fn braces_in_a_string_are_literal_text_forever() {
    let (rep, _) = ok("let i = 3\nprint \"fuel_{i}\"\nprint \"{\" + \"}\"\n");
    assert_eq!(prints(&rep), ["fuel_{i}", "{}"]);
    // A plain string that looks like an interpolation gets a warning that points at `+ str(...)`.
    let w = rep.diagnostics.iter().find(|d| d.code == "literal-braces").expect("warning");
    assert!(!w.is_error());
    assert!(w.help.as_deref().is_some_and(|h| h.contains("str(")), "{w:?}");
}

#[test]
fn dollar_brace_in_a_string_is_reserved_and_can_be_escaped() {
    let d = err("print \"pin_${i}\"");
    assert_eq!(d.code, "reserved-interpolation");
    assert!(d.help.as_deref().is_some_and(|h| h.contains("\\${") && h.contains("str(")), "{d:?}");
    // Nothing runs when a string is reserved.
    let (rep, _, _) = run("model.create_brick(x: 1, y: 1, z: 1)\nprint \"a${b}\"\n");
    assert!(rep.events.is_empty());
    // The escape and a lone `$` stay literal.
    let (rep, _) = ok("print \"a\\${b}\"\nprint \"$0.x\"\nprint \"cost $5\"\n");
    assert_eq!(prints(&rep), ["a${b}", "$0.x", "cost $5"]);
}

// ---- Surface#n in strings --------------------------------------------------------------------------

#[test]
fn surface_and_curve_prefixes_in_user_strings_are_ambiguous() {
    for src in [
        "model.create_brick(x: 1, y: 1, z: 1)\nmodel.pick(\"Surface#1\")",
        "model.create_brick(x: 1, y: 1, z: 1)\nmodel.pick(\"Curve#1\")",
        "model.create_brick(x: 1, y: 1, z: 1)\nlet s = ents(\"Surface#1\")",
        "model.create_brick(x: 1, y: 1, z: 1)\nlet s = ents([\"Face#1\", \"Curve#1\"])",
        "model.create_brick(x: 1, y: 1, z: 1)\nintent.set_size(ids: [\"Surface#1\"], size: 1)",
        "model.create_brick(x: 1, y: 1, z: 1)\nlet b = contains(all faces, \"Surface#1\")",
        "model.create_brick(x: 1, y: 1, z: 1)\nlet i = 1\nmodel.pick(\"Surface#\" + str(i))",
    ] {
        let d = err(src);
        assert_eq!(d.code, "ambiguous-kind", "{src}: {d:?}");
        let help = d.help.as_deref().unwrap_or("");
        assert!(help.contains("Face#n") && help.contains("GeomSurface#n"), "{src}: {d:?}");
    }
}

#[test]
fn unambiguous_spellings_still_work_in_strings() {
    let (_, st) = ok("model.create_brick(x: 1, y: 1, z: 1)\nmodel.pick(\"Face#1\")\nmodel.pick(\"TopoFace#2\")\nassert ents(\"Face#3\") == Face#3\n");
    assert_eq!(notes(&st), ["TopoFace#1", "TopoFace#2"]);
}

#[test]
fn ids_in_strings_are_checked_for_liveness_like_bare_ids() {
    assert_eq!(err("let s = ents(\"Volume#9\")").code, "entity-not-found");
    assert_eq!(err("model.pick(\"Face#99\")").code, "entity-not-found");
    assert_eq!(err("model.pick(\"GeomSurface#1\")").code, "entity-not-found");
    // Plan mode does not execute, so it cannot know.
    let engine = engine();
    let mut h = PlanHost::new(engine.registry(), engine.registry().new_state());
    let mut it = Interpreter::new(&mut h, Options::default());
    let rep = it.run_source("p", "model.pick(\"Face#99\")\n");
    assert!(rep.ok());
}

// ---- timeline: the contract Rev ----------------------------------------------------------------------

struct TlHost<'a> {
    inner: EngineHost<'a>,
    cmds: Vec<TimelineCommand>,
}

impl Host for TlHost<'_> {
    fn registry(&self) -> &sm_op_api::Registry {
        self.inner.registry()
    }
    fn state(&self) -> &sm_op_api::State {
        self.inner.state()
    }
    fn apply(&mut self, rec: &sm_op_api::OpRecord) -> Result<sm_op_api::Delta, HostError> {
        self.inner.apply(rec)
    }
    fn query(&mut self, name: &str, params: &serde_json::Value) -> Result<serde_json::Value, HostError> {
        self.inner.query(name, params)
    }
    fn timeline(&mut self, cmd: &TimelineCommand) -> Result<(), HostError> {
        self.cmds.push(cmd.clone());
        Ok(())
    }
}

fn timeline_cmds(src: &str) -> (RunReport, Vec<TimelineCommand>, SourceMap) {
    let engine = engine();
    let mut h = TlHost { inner: host(&engine), cmds: Vec::new() };
    let (rep, map) = {
        let mut it = Interpreter::new(&mut h, Options::default());
        let rep = it.run_source("t", src);
        (rep, it.sources().clone())
    };
    (rep, h.cmds, map)
}

#[test]
fn checkout_and_branch_carry_contract_revisions() {
    let (rep, cmds, _) = timeline_cmds(&format!("timeline.checkout(node: \"{HASH}\")\ntimeline.branch(\"x\", at: \"{HASH}\")\ntimeline.branch(\"y\")\ntimeline.checkout(\"main\")\n"));
    assert!(rep.ok(), "{:?}", rep.diagnostics);
    let h: sm_op_api::Hash256 = HASH.parse().unwrap();
    assert_eq!(
        cmds,
        [
            TimelineCommand::Checkout(Rev::Node(h)),
            TimelineCommand::Branch { name: "x".into(), at: Some(h) },
            TimelineCommand::Branch { name: "y".into(), at: None },
            TimelineCommand::Checkout(Rev::Ref("main".into())),
        ]
    );
}

#[test]
fn a_malformed_node_hash_is_diagnosed_at_the_argument_and_never_reaches_the_host() {
    let src = "timeline.checkout(node: \"zz\")\n";
    let (rep, cmds, map) = timeline_cmds(src);
    assert!(cmds.is_empty());
    let d = rep.diagnostics.iter().find(|d| d.is_error()).expect("error");
    assert_eq!(d.code, "invalid-argument");
    assert_eq!(text_of(&map, d), "\"zz\"");
    let (rep, cmds, map) = timeline_cmds("timeline.branch(\"b\", at: \"ABC\")\n");
    assert!(cmds.is_empty());
    let d = rep.diagnostics.iter().find(|d| d.is_error()).expect("error");
    assert_eq!((d.code, text_of(&map, d).as_str()), ("invalid-argument", "\"ABC\""));
}

#[test]
fn timeline_commands_have_no_value() {
    for src in ["let x = timeline.undo()", "let x = 1\nset x = timeline.branch(\"a\")", "let x = timeline.checkout(\"main\")"] {
        let d = err(src);
        assert_eq!(d.code, "op-in-expression", "{src}");
        assert!(d.help.as_deref().is_some_and(|h| h.contains("own statement")), "{d:?}");
    }
}

// ---- reserved words ------------------------------------------------------------------------------------

#[test]
fn future_keywords_cannot_be_names_but_stay_valid_segments_and_quoted_keys() {
    for w in ["fn", "def", "return", "while", "break", "continue", "import"] {
        let d = err(&format!("let {w} = 1"));
        assert_eq!(d.code, "reserved-word", "{w}");
    }
    assert_eq!(err("let x = return (1)").code, "reserved-word");
    let (rep, _) = ok("let o = { \"return\": 1 }\nprint o.return\nprint o[\"return\"]\n");
    assert_eq!(prints(&rep), ["1", "1"]);
}

#[test]
fn element_type_words_are_reserved_as_selectors_but_remain_names() {
    for src in ["let s = all hexes", "let s = tets in Volume#1", "let s = all tris"] {
        assert_eq!(err(src).code, "not-yet-supported", "{src}");
    }
    ok("let hexes = 3\nlet tets = hexes + 1\nassert tets == 4\n");
}

// ---- batches: failures, indexing, loops, rebinding -----------------------------------------------------

#[test]
fn a_misspelled_output_role_in_a_batch_points_at_the_argument() {
    let src = "batch {\n  let q = model.create_brick(x: 1, y: 1, z: 1)\n  model.pick(q.voluem[0])\n}\n";
    let (d, map) = err_map(src);
    assert_eq!(d.code, "op-failed");
    assert_eq!(text_of(&map, &d), "q.voluem[0]");
    assert!(d.message.contains("no such output"), "{d:?}");
    assert!(d.notes.iter().any(|(sp, _)| text_of_span(&map, *sp).starts_with("batch")), "{d:?}");
    // Inside a list literal the exact element is found.
    let src = "batch {\n  let q = model.create_brick(x: 1, y: 1, z: 1)\n  intent.set_size(ids: [q.body[0], q.voluem[0]], size: 1)\n}\n";
    let (d, map) = err_map(src);
    assert_eq!(text_of(&map, &d), "q.voluem[0]");
}

fn text_of_span(map: &SourceMap, sp: sm_lang::Span) -> String {
    map.get(sp.file).expect("source").text[sp.lo as usize..sp.hi as usize].to_owned()
}

#[test]
fn batch_limits_explain_why_they_differ_from_top_level() {
    let neg = err("batch {\n let q = model.create_brick(x: 1, y: 1, z: 1)\n model.pick(q.faces[-1])\n}");
    assert_eq!(neg.code, "type-mismatch");
    assert!(neg.message.contains("negative index") && neg.message.contains("explicit `[k]`"), "{neg:?}");
    let lp = err("batch {\n let q = model.create_brick(x: 1, y: 1, z: 1)\n for f in q.faces { model.pick(f) }\n}");
    assert_eq!(lp.code, "type-mismatch");
    let help = lp.help.as_deref().unwrap_or("");
    assert!(help.contains("cannot know") && help.contains("array parameter") && help.contains("after the batch"), "{lp:?}");
    // Top level, the same code works.
    ok("let q = model.create_brick(x: 1, y: 1, z: 1)\nmodel.pick(q.faces[-1])\nfor f in q.faces { model.pick(f) }\n");
}

#[test]
fn a_bare_role_message_names_the_written_variable() {
    let d = err("batch {\n let p = model.create_brick(x: 1, y: 1, z: 1)\n model.pick(p.faces)\n}");
    assert_eq!(d.code, "expected-one-entity");
    assert!(d.message.contains("p.faces[0]"), "{d:?}");
}

#[test]
fn references_inside_lists_and_objects_are_rebound_after_the_batch() {
    let (_, st) = ok("batch {\n let p = model.create_brick(x: 1, y: 1, z: 1)\n let l = [p.body[0], p.volume[0]]\n let o = { b: p.body[0], inner: [p.faces[1]] }\n}\nmodel.pick(l[0])\nmodel.pick(l[1])\nmodel.pick(o.b)\nmodel.pick(o.inner[0])\n");
    assert_eq!(notes(&st), ["TopoBody#1", "TopoVolume#1", "TopoBody#1", "TopoFace#2"]);
}

#[test]
fn a_selection_past_the_end_of_a_role_says_why_it_is_unusable() {
    let d = err("batch {\n let p = model.create_brick(x: 1, y: 1, z: 1)\n let z = p.body[5]\n}\nmodel.pick(z)\n");
    assert_eq!(d.code, "stale-reference");
    assert!(d.message.contains("produced no output"), "{d:?}");
    assert!(!d.message.contains("not the one being built"));
}

// ---- parse diagnostics -------------------------------------------------------------------------------------

#[test]
fn ambiguous_in_help_echoes_the_operator() {
    for (op, src) in [("&", "let s = faces in Volume#1 & faces in Volume#2"), ("-", "let s = faces in Volume#1 - faces in Volume#2"), ("except", "let s = faces in Volume#1 except Face#1")] {
        let d = err(src);
        assert_eq!(d.code, "ambiguous-in");
        let help = d.help.unwrap();
        assert!(help.contains(&format!("(A {op} B)")) && help.contains(&format!("(faces in A) {op} B")), "{help}");
        assert!(!help.contains(" | ") || op == "|");
    }
}

#[test]
fn line_start_operators_ranges_and_juxtaposition_get_targeted_help() {
    let d = err("print all faces\n  - Face#1");
    assert_eq!(d.code, "expected-statement");
    assert!(d.help.as_deref().is_some_and(|h| h.contains("end of the previous line") && h.contains("parentheses")), "{d:?}");
    let d = err("let n = 3\nlet s = Volume#1..n");
    assert!(d.help.as_deref().is_some_and(|h| h.contains("Volume#[1..n]")), "{d:?}");
    let d = err("print Volume#1 Volume#2");
    assert!(d.help.as_deref().is_some_and(|h| h.contains("Volume#1 | Volume#2") && h.contains("Volume#[1, 2]")), "{d:?}");
}

// ---- printing in source form --------------------------------------------------------------------------------

#[test]
fn printed_entity_sets_are_source_and_can_be_pasted_back() {
    let (rep, _) = ok("model.create_brick(x: 1, y: 1, z: 1)\nprint all faces\nprint all faces | Volume#1\nprint Face#[1, 3, 4, 5]\nprint Face#2\nprint all faces - all faces\n");
    let (rep2, _) = ok("let p = test.mixed()\nprint p.mix\n");
    let out = prints(&rep);
    assert_eq!(out[0], "Face#[1..6]");
    assert_eq!(out[1], "Face#[1..6] | Volume#1");
    assert_eq!(out[2], "Face#[1, 3..5]");
    assert_eq!(out[3], "Face#2");
    assert_eq!(out[4], "<no entities>");
    // An ordered op output keeps its order and prints as a list, not as a set.
    assert_eq!(prints(&rep2), ["[Face#1, Body#1, Volume#1]"]);
    // Pasting the printed text back gives the same set.
    let script = format!("model.create_brick(x: 1, y: 1, z: 1)\nassert all faces | Volume#1 == {}\nassert Face#[1, 3, 4, 5] == {}\n", out[1], out[2]);
    ok(&script);
}

#[test]
fn large_sets_print_compactly() {
    let (rep, _) = ok("for i in range(0, 50) { model.create_brick(x: 1, y: 1, z: 1) }\nprint all faces\n");
    assert_eq!(prints(&rep), ["Face#[1..300]"]);
}

// ---- op errors speak the script's language ------------------------------------------------------------------

#[test]
fn op_errors_use_the_source_spelling_of_ids() {
    let d = err("model.create_brick(x: 1, y: 1, z: 1)\nmodel.scale_brick(id: Volume#1, factor: 2)");
    assert_eq!(d.code, "op-failed");
    assert!(d.message.contains("Volume#1") && !d.message.contains("TopoVolume#1"), "{d:?}");
}
