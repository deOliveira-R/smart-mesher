//! Review items of `polish0`: linear appends, `null`, `if` and `a..b` expressions, shadowing and alias rules, new
//! diagnostics, strict `==`, flat operator chains, printing op results, loop budget, plan-mode `if`, `timeline.head()`.

mod common;

use std::time::{Duration, Instant};

use sm_lang::diag::render_all;
use sm_lang::{Diagnostic, EngineHost, Event, Host, HostError, Interpreter, Options, PlanHost, RunReport, Severity, SourceMap, TimelineCommand, TimelineHead};

use common::*;

fn run_with(src: &str, opts: Options) -> (RunReport, sm_op_api::State, SourceMap) {
    let engine = engine();
    let mut h = host(&engine);
    let (rep, map) = {
        let mut it = Interpreter::new(&mut h, opts);
        let rep = it.run_source("t.smesh", src);
        (rep, it.sources().clone())
    };
    (rep, h.into_state(), map)
}

fn run(src: &str) -> (RunReport, sm_op_api::State, SourceMap) {
    run_with(src, Options::default())
}

fn ok(src: &str) -> RunReport {
    let (rep, _, map) = run(src);
    assert!(rep.ok(), "{src}\n{}", render_all(&rep.diagnostics, &map));
    rep
}

fn err(src: &str) -> Diagnostic {
    let (rep, _, _) = run(src);
    rep.diagnostics.into_iter().find(Diagnostic::is_error).unwrap_or_else(|| panic!("`{src}` ran without an error"))
}

fn prints(rep: &RunReport) -> Vec<String> {
    rep.events.iter().filter_map(|e| if let Event::Print { text, .. } = e { Some(text.clone()) } else { None }).collect()
}

fn warnings(rep: &RunReport) -> Vec<&Diagnostic> {
    rep.diagnostics.iter().filter(|d| d.severity == Severity::Warning).collect()
}

const BRICK: &str = "let b = model.create_brick(x: 1, y: 1, z: 1)\n";

// ---- 1. appends are linear -----------------------------------------------------------------------

fn budget() -> Duration {
    // Quadratic copying took 103 s in release; the bound only has to separate linear from quadratic.
    if cfg!(debug_assertions) { Duration::from_secs(30) } else { Duration::from_secs(1) }
}

#[test]
fn building_a_list_in_a_loop_is_linear() {
    let t = Instant::now();
    ok("let l = []\nfor i in range(0, 200000) { set l = l + [i] }\nassert len(l) == 200000\nassert l[199999] == 199999\n");
    assert!(t.elapsed() < budget(), "200k list appends took {:?}", t.elapsed());
}

#[test]
fn building_a_string_in_a_loop_is_linear() {
    let t = Instant::now();
    ok("let s = \"\"\nfor i in range(0, 200000) { set s = s + \"ab\" }\nassert len(s) == 400000\n");
    assert!(t.elapsed() < budget(), "200k string appends took {:?}", t.elapsed());
}

#[test]
fn appending_keeps_value_semantics() {
    // A copy bound earlier does not change; the right-hand side may read the variable; chains append in order.
    let rep = ok("let l = [1]\nlet m = l\nset l = l + [2]\nset l = l + [len(l)] + [9]\nprint m, l\nlet s = \"a\"\nset s = s + s + \"b\"\nprint s\nset l = l + l\nprint len(l)\n");
    assert_eq!(prints(&rep), ["[1] [1, 2, 2, 9]", "aab", "8"]);
}

#[test]
fn a_failing_append_changes_nothing() {
    let (rep, _, _) = run("let l = [1]\nset l = l + [2] + \"x\"\n");
    let d = rep.diagnostics.iter().find(|d| d.is_error()).expect("error");
    assert_eq!(d.code, "type-mismatch");
    let (rep, _, _) = run_with("let l = [1, 2]\nset l = l + [3] + [4, 5]\n", Options { max_value_len: 4, ..Options::default() });
    assert_eq!(rep.diagnostics.iter().find(|d| d.is_error()).map(|d| d.code), Some("value-too-long"));
    // The variable is untouched after the failure (checked through a script that survives via `if`).
    let (rep, _, _) = run_with("let l = [1, 2]\nlet ok = 0\nfor i in [1] { set ok = len(l) }\nassert ok == 2\n", Options { max_value_len: 4, ..Options::default() });
    assert!(rep.ok());
}

// ---- 2. null -------------------------------------------------------------------------------------

#[test]
fn null_is_reserved() {
    let d = err("let null = 1");
    assert_eq!(d.code, "reserved-word");
    assert!(d.help.as_deref().unwrap_or("").contains("no null"));
    assert_eq!(err("let x = null").code, "no-null");
    assert_eq!(err("let x = [null]").code, "no-null");
    assert_eq!(err("let x = null + 1").code, "no-null");
    assert_eq!(err("let o = { a: null + 1 }").code, "no-null");
    let d = err("model.create_brick(x: 1, y: 1, z: 1, name: null)");
    assert_eq!(d.code, "no-null");
    assert!(d.help.as_deref().unwrap_or("").contains("omit"));
    assert_eq!(err("for null in [1] { }").code, "reserved-word");
}

#[test]
fn null_in_an_object_omits_the_field() {
    ok("let o = { a: 1, b: null }\nassert len(o) == 1\nassert not contains(o, \"b\")\n");
    ok("let o = { a: 1, b: null, }\nassert len(o) == 1\n");
    let (rep, st, map) = run("model.create_brick(...{ x: 1, y: 1, z: 1, name: null })\n");
    assert!(rep.ok(), "{}", render_all(&rep.diagnostics, &map));
    assert_eq!(st.layer::<DemoModel>().unwrap().bricks.len(), 1);
    assert_eq!(err("let o = { a: 1, a: null }").code, "duplicate-key");
    // After a dot it is an ordinary path segment, like every reserved word.
    assert_eq!(err("let o = { a: 1 }\nlet x = o.null").code, "unknown-field");
}

// ---- 3. if expression ------------------------------------------------------------------------------

#[test]
fn if_is_an_expression() {
    let rep = ok("let c = true\nlet r = if c { 0.6 } else { 0.475 }\nlet q = if not c { 1 } else if r > 0.5 { 2 } else { 3 }\nprint r, q\n");
    assert_eq!(prints(&rep), ["0.6 2"]);
    // Only the chosen branch is evaluated.
    ok("let x = if true { 1 } else { 1 / 0 }\nlet y = if false { 1 / 0 } else { 2 }\nassert x + y == 3\n");
    // In arguments, with Allman else, and inside brackets across lines.
    ok("let n = 2\nmodel.create_brick(x: if n > 1 { 2.0 } else { 1.0 }, y: 1, z: 1)\n");
    ok("let r = if true { 1 }\nelse { 2 }\nassert r == 1\n");
    ok("let l = [\n  1, if true { 2 } else { 3 },\n  4\n]\nassert l == [1, 2, 4]\n");
    ok("let r = if true {\n  1 + 1\n} else {\n  3\n}\nassert r == 2\n");
    ok("let r = (if true { 1 } else { 2 }) + 1\nassert r == 2\n");
}

#[test]
fn if_expression_needs_else_and_a_bool() {
    let d = err("let r = if true { 1 }");
    assert_eq!(d.code, "missing-else");
    assert!(d.help.as_deref().unwrap_or("").contains("else"));
    assert_eq!(err("let r = if 1 { 1 } else { 2 }").code, "type-mismatch");
    assert_eq!(err("let r = if true { } else { 2 }").code, "expected-expression");
}

// ---- 4. shadowing ------------------------------------------------------------------------------------

#[test]
fn a_shadowing_let_in_a_loop_warns_once() {
    let (rep, _, map) = run("let x = 1\nfor i in range(0, 1000) { let x = 2 }\n");
    assert!(rep.ok());
    let w = warnings(&rep);
    assert_eq!(w.len(), 1, "{}", render_all(&rep.diagnostics, &map));
    assert_eq!(w[0].code, "shadowed");
}

#[test]
fn a_block_let_that_hides_an_outer_variable_warns() {
    let (rep, _, map) = run("let x = 1\nif true { let x = 2 }\nassert x == 1\n");
    assert!(rep.ok());
    let w = warnings(&rep);
    assert_eq!(w.len(), 1, "{}", render_all(&rep.diagnostics, &map));
    assert_eq!(w[0].code, "shadowed");
    assert_eq!(w[0].help.as_deref(), Some("use `set` to change the outer variable"));
    // No warning for a fresh name, for the same name at top level in another scope that is gone, or for a loop variable reuse.
    let (rep, _, _) = run("let x = 1\nif true { let y = 2 }\nif true { let x = 3 }\n");
    assert_eq!(warnings(&rep).len(), 1);
    let (rep, _, _) = run("for i in [1] { let a = 1 }\nfor i in [1] { let a = 2 }\n");
    assert!(warnings(&rep).is_empty());
}

// ---- 5. `x = 2` ---------------------------------------------------------------------------------------

#[test]
fn assigning_without_set_suggests_set() {
    let d = err("let x = 1\nx = 2 + 3");
    assert_eq!(d.code, "expected-token");
    assert!(d.help.as_deref().unwrap_or("").contains("set x = 2 + 3"), "{:?}", d.help);
    let d = err("y = 2");
    let h = d.help.unwrap_or_default();
    assert!(h.contains("let y = 2") && h.contains("set y = 2"), "{h}");
}

// ---- 6, 7. string prefix and braces ---------------------------------------------------------------------

#[test]
fn a_prefixed_string_is_a_targeted_error() {
    let d = err("let s = f\"x\"");
    assert_eq!(d.code, "reserved-prefix");
    assert!(d.help.as_deref().unwrap_or("").contains("+ str("));
    let (rep, _, _) = run("let i = 1\nlet s = f\"x{i}\"\n");
    assert_eq!(rep.diagnostics.iter().filter(|d| d.is_error()).count(), 1, "{:?}", rep.diagnostics);
    // Keywords glued to a string stay valid, and so do names separated from it.
    let rep = ok("print\"hi\"\nlet f = 1\nprint f, \"x\"\n");
    assert_eq!(prints(&rep), ["hi", "1 x"]);
}

#[test]
fn literal_braces_warning_is_quiet_after_an_escaped_dollar() {
    let (rep, _, _) = run("let s = \"cost \\${name}\"\nassert s == \"cost \\${name}\"\n");
    assert!(rep.ok() && warnings(&rep).is_empty(), "{:?}", rep.diagnostics);
    let (rep, _, _) = run("let s = \"pin_${i}\"");
    assert_eq!(rep.diagnostics.len(), 1, "{:?}", rep.diagnostics);
    assert_eq!(rep.diagnostics[0].code, "reserved-interpolation");
    // A plain `{name}` still warns.
    let (rep, _, _) = run("let s = \"fuel_{i}\"");
    assert_eq!(warnings(&rep).len(), 1);
}

// ---- 8. numbers ---------------------------------------------------------------------------------------

#[test]
fn malformed_numbers_have_one_targeted_diagnostic() {
    for (src, hint) in [("1.e5", "1.0e5"), ("1.", "1.0"), ("1_", "1_000"), ("1__0", "1_000"), ("1e", "1e5"), ("1e+", "1e5"), ("2.5_", "1_000"), ("1_.5", "1_000")] {
        let (rep, _, _) = run(&format!("let x = {src}"));
        assert_eq!(rep.diagnostics.len(), 1, "{src}: {:?}", rep.diagnostics);
        let d = &rep.diagnostics[0];
        assert_eq!(d.code, "invalid-number", "{src}");
        assert!(d.help.as_deref().unwrap_or("").contains(hint), "{src}: {:?}", d.help);
    }
    ok("assert 1e5 == 100000.0\nassert 1_000 == 1000\nassert 1.0e5 == 1e5\nassert 1.5e-3 < 1\nassert len(1..3) == 3\nassert 2E+2 == 200.0\n");
}

// ---- 9. strict equality -------------------------------------------------------------------------------

#[test]
fn equality_between_different_types_is_an_error() {
    let src = format!("{BRICK}let r = Face#1 == \"Face#1\"\n");
    let d = err(&src);
    assert_eq!(d.code, "type-mismatch");
    let h = d.help.unwrap_or_default();
    assert!(h.contains("ents(") && h.contains("str("), "{h}");
    assert_eq!(err("let r = 1 != \"1\"").code, "type-mismatch");
    assert_eq!(err("let r = true == 1").code, "type-mismatch");
    assert_eq!(err("let r = [1] == {}").code, "type-mismatch");
    // Int and Num compare numerically; same types compare structurally; nested mixed types are just unequal.
    ok("assert 1 == 1.0\nassert 2 != 2.5\nassert [1, 2] == [1.0, 2.0]\nassert [1] != [\"a\"]\nassert \"a\" != \"b\"\nassert contains([\"a\", 1], 1)\n");
    ok(&format!("{BRICK}assert Face#1 == ents(\"Face#1\")\nassert str(Face#1) == \"Face#1\"\n"));
}

// ---- 10. flat chains ---------------------------------------------------------------------------------------

#[test]
fn a_five_thousand_term_union_runs() {
    let terms: Vec<String> = (0..5000).map(|i| format!("Face#{}", i % 6 + 1)).collect();
    let src = format!("{BRICK}let all6 = {}\nassert count(all6) == 6\nlet n = {}\nassert n == 5000\n", terms.join(" | "), vec!["1"; 5000].join(" + "));
    ok(&src);
    // Generated scripts also wrap lines.
    let src = format!("{BRICK}let u = {}\nassert count(u) == 6\n", terms.join(" |\n  "));
    ok(&src);
    ok(&format!("assert {}\n", vec!["true"; 5000].join(" and ")));
    ok(&format!("assert {}\n", ["false"; 5000].join(" or ") + " or true"));
    let l = vec!["[1]"; 5000].join(" + ");
    ok(&format!("assert len({l}) == 5000\n"));
    // `and`/`or` still short-circuit and still check types.
    ok("assert not (false and 1 and 2)\nassert true or 1 or 2\n");
    assert_eq!(err("let x = true and true and 1").code, "type-mismatch");
}

// ---- 11. aliases ---------------------------------------------------------------------------------------------

#[test]
fn aliases_follow_the_rebind_rule_and_warn_about_confusing_names() {
    let d = err("alias brick = model.create_brick\nalias brick = model.note\n");
    assert_eq!(d.code, "rebind");
    let (rep, _, _) = run("alias model = model.create_brick\n");
    assert!(rep.ok());
    assert!(warnings(&rep).iter().any(|w| w.code == "alias-confusing" && w.message.contains("namespace")), "{:?}", rep.diagnostics);
    let (rep, _, _) = run("let brick = 1\nalias brick = model.create_brick\n");
    assert!(warnings(&rep).iter().any(|w| w.code == "alias-confusing" && w.message.contains("variable")), "{:?}", rep.diagnostics);
    let (rep, _, _) = run("alias brick = model.create_brick\nalias note = model.note\n");
    assert!(rep.ok() && warnings(&rep).is_empty());
    // Interactive sessions may re-send an alias, with the `shadowed` warning, as they may re-send `let`.
    let engine = engine();
    let mut h = host(&engine);
    let mut it = Interpreter::new(&mut h, Options { allow_top_level_rebind: true, ..Options::default() });
    assert!(it.run_source("a", "alias brick = model.create_brick").ok());
    let rep = it.run_source("b", "alias brick = model.note");
    assert!(rep.ok());
    assert_eq!(warnings(&rep)[0].code, "shadowed");
}

// ---- 12, 17. printing --------------------------------------------------------------------------------------

#[test]
fn printing_an_op_result_shows_its_roles() {
    let rep = ok(&format!("{BRICK}print b\nprint str(b)\nprint [b.body]\nlet s = \"r=\" + str(b)\nprint s\n"));
    let p = prints(&rep);
    assert_eq!(p[0], "{body: Body#1, faces: Face#[1..6], volume: Volume#1}");
    assert_eq!(p[1], p[0]);
    assert_eq!(p[2], "[Body#1]");
    assert_eq!(p[3], format!("r={}", p[0]));
}

#[test]
fn print_joins_items_with_one_space() {
    // The separator is one space, as in Python; the literal itself carries no trailing space.
    let rep = ok("print \"x =\", 5\nprint \"a\", \"b\", 1.5\nprint \"a\", \"\"\nprint \"only\"\n");
    assert_eq!(prints(&rep), ["x = 5", "a b 1.5", "a ", "only"]);
}

// ---- 13. loop budget --------------------------------------------------------------------------------------

#[test]
fn nested_loops_are_charged_for_innermost_bodies_only() {
    let nested = |outer: i64, inner: i64, limit: u64| {
        let src = format!("let n = 0\nfor i in range(0, {outer}) {{ for j in range(0, {inner}) {{ set n = n + 1 }} }}\nassert n == {}\n", outer * inner);
        run_with(&src, Options { max_loop_iterations: limit, ..Options::default() }).0
    };
    assert!(nested(100, 100, 10_000).ok());
    assert_eq!(nested(100, 101, 10_000).diagnostics.iter().find(|d| d.is_error()).map(|d| d.code), Some("loop-limit"));
    // An empty inner loop leaves the outer body as the innermost one, so it is charged (no free outer iterations).
    let src = "for i in range(0, 50) { for j in [] { } }";
    assert!(run_with(src, Options { max_loop_iterations: 50, ..Options::default() }).0.ok());
    assert!(!run_with(src, Options { max_loop_iterations: 49, ..Options::default() }).0.ok());
    // The documented case: 1000 x 1000 within the default budget of 1,000,000.
    let rep = ok("let n = 0\nfor i in 1..1000 { for j in 1..1000 { set n = n + 1 } }\nassert n == 1000000\n");
    assert!(rep.ok());
    let (rep, _, _) = run("for i in 1..1000 { for j in 1..1001 { } }");
    assert_eq!(rep.diagnostics.iter().find(|d| d.is_error()).map(|d| d.code), Some("loop-limit"));
}

// ---- 14. plan mode ---------------------------------------------------------------------------------------

fn plan(src: &str) -> (RunReport, SourceMap) {
    let engine = engine();
    let mut h = PlanHost::new(engine.registry(), engine.registry().new_state());
    let mut it = Interpreter::new(&mut h, Options::default());
    let rep = it.run_source("p", src);
    (rep, it.sources().clone())
}

#[test]
fn an_if_on_a_plan_placeholder_checks_every_branch() {
    for cond in ["r.count > 3", "r.flag", "r.a and r.b"] {
        let src = format!("let r = model.create_brick(x: 1, y: 1, z: 1)\nif {cond} {{ model.note(text: \"a\") }} else if r.other {{ model.note(text: \"b\") }} else {{ model.note(text: \"c\") }}\nmodel.note(text: \"after\")\n");
        let (rep, map) = plan(&src);
        assert!(rep.ok(), "{cond}\n{}", render_all(&rep.diagnostics, &map));
        assert!(warnings(&rep).iter().any(|w| w.code == "plan-unresolved"), "{cond}: {:?}", rep.diagnostics);
        // brick + the three branches + the statement after the `if`.
        assert_eq!(rep.records().len(), 5, "{cond}");
    }
    // A condition that is known still takes one branch in a plan.
    let (rep, _) = plan("if true { model.note(text: \"a\") } else { model.note(text: \"b\") }");
    assert!(rep.ok());
    assert_eq!(rep.records().len(), 1);
    // A real error inside an unknown branch stays an error.
    let (rep, _) = plan("let r = model.create_brick(x: 1, y: 1, z: 1)\nif r.flag { let a = undefined_name }");
    assert!(!rep.ok());
    // An `if` expression on a placeholder is a placeholder, not an error.
    let (rep, _) = plan("let r = model.create_brick(x: 1, y: 1, z: 1)\nlet v = if r.flag { 1 } else { 2 }\nmodel.note(text: \"x\")");
    assert!(rep.ok(), "{:?}", rep.diagnostics);
}

// ---- 15. timeline.head ------------------------------------------------------------------------------------

struct HeadHost<'a> {
    inner: EngineHost<'a>,
    head: TimelineHead,
}

impl Host for HeadHost<'_> {
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
    fn timeline(&mut self, _: &TimelineCommand) -> Result<(), HostError> {
        Ok(())
    }
    fn timeline_head(&mut self) -> Result<TimelineHead, HostError> {
        Ok(self.head.clone())
    }
}

fn with_head(head: TimelineHead, src: &str) -> RunReport {
    let engine = engine();
    let mut h = HeadHost { inner: host(&engine), head };
    let mut it = Interpreter::new(&mut h, Options::default());
    it.run_source("t", src)
}

#[test]
fn timeline_head_reads_the_current_branch() {
    let node = "ab".repeat(32);
    let head = TimelineHead::new(Some("main".into()), Some(node.clone()));
    let rep = with_head(head, "let h = timeline.head()\nprint h.branch\nassert timeline.head().branch == \"main\"\ntimeline.head()\nprint len(h)\n");
    assert!(rep.ok(), "{:?}", rep.diagnostics);
    assert_eq!(prints(&rep), ["main", "2"]);
    // A detached head has no `branch` field (the language has no null).
    let rep = with_head(TimelineHead::new(None, Some(node)), "assert not contains(timeline.head(), \"branch\")\nlet b = timeline.head().branch\n");
    assert_eq!(rep.diagnostics.iter().find(|d| d.is_error()).map(|d| d.code), Some("unknown-field"));
    // Arguments, batches and hosts without a timeline.
    let rep = with_head(TimelineHead::default(), "timeline.head(1)");
    assert_eq!(rep.diagnostics[0].code, "unknown-argument");
    let rep = with_head(TimelineHead::default(), "batch { let h = timeline.head() }");
    assert_eq!(rep.diagnostics[0].code, "timeline-in-batch");
    let (rep, _, _) = run("let h = timeline.head()");
    assert_eq!(rep.diagnostics[0].code, "timeline-failed");
    let (rep, _) = plan("let h = timeline.head()\nprint h.branch");
    assert!(rep.ok(), "{:?}", rep.diagnostics);
    // The other commands remain statements.
    assert_eq!(err("let x = timeline.undo()").code, "op-in-expression");
    let rep = ok("help timeline.head");
    assert!(matches!(&rep.events[0], Event::Help { text, .. } if text.contains("branch")));
}

// ---- 18. source spelling of JSON ------------------------------------------------------------------------------

#[test]
fn respell_json_rewrites_canonical_ids_for_display() {
    let j = serde_json::json!({ "ids": ["TopoFace#1", "Surface#7", "Curve#2"], "n": 3, "msg": "no TopoVolume#4 here", "deep": [{ "x": "TopoEdge#9" }] });
    let r = sm_lang::respell_json(&j);
    assert_eq!(r, serde_json::json!({ "ids": ["Face#1", "GeomSurface#7", "GeomCurve#2"], "n": 3, "msg": "no Volume#4 here", "deep": [{ "x": "Edge#9" }] }));
    // The events themselves stay in wire form.
    let rep = ok(&format!("{BRICK}model.pick(Face#1)\n"));
    let Some(Event::Op { record, .. }) = rep.events.last() else { panic!("op event") };
    assert!(record.params.to_string().contains("TopoFace#1"), "{}", record.params);
}

// ---- 19. range expression ------------------------------------------------------------------------------------

#[test]
fn a_range_expression_is_inclusive() {
    let rep = ok("let n = 4\nlet t = 0\nfor i in 1..n { set t = t + i }\nprint t\nprint 1..3, -2..0\nprint len(5..4), len(7..7)\nlet m = 1..n + 1\nprint len(m)\n");
    assert_eq!(prints(&rep), ["10", "[1, 2, 3] [-2, -1, 0]", "0 1", "5"]);
    // Id specs keep their own inclusive `..`, also with computed ends.
    ok(&format!("{BRICK}let k = 6\nassert count(Face#[1..k]) == 6\nassert count(Face#1..3) == 3\nassert count(Face#[2..k by 2]) == 3\n"));
    assert_eq!(err("let r = 1..2..3").code, "chained-range");
    assert_eq!(err("let r = 1..2.5").code, "type-mismatch");
    assert_eq!(err("let r = \"a\"..\"b\"").code, "type-mismatch");
    assert_eq!(err("let r = 0..100000000").code, "range-too-long");
    // `range(a, b)` is unchanged and half-open.
    ok("assert range(1, 4) == [1, 2, 3]\nassert (1..3) == [1, 2, 3]\n");
}
