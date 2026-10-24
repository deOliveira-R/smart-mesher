//! Language decisions and review fixes of `fix0`: ordered outputs, computed ids, Cubit vocabulary, timeline
//! namespace, numerics, plan-mode loops, limits, printing, diagnostics.

mod common;

use serde_json::json;
use sm_core::{EntityId, EntityKind};
use sm_lang::diag::render_all;
use sm_lang::{EngineHost, Event, Host, HostError, Interpreter, Options, PlanHost, RunReport, SourceMap, TimelineCommand, TimelineRev};

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

fn ok(src: &str) -> (RunReport, sm_op_api::State) {
    let (rep, st, map) = run(src);
    assert!(rep.ok(), "{src}\n{}", render_all(&rep.diagnostics, &map));
    (rep, st)
}

fn err(src: &str) -> sm_lang::Diagnostic {
    let (rep, _, _) = run(src);
    rep.diagnostics.into_iter().find(|d| d.is_error()).unwrap_or_else(|| panic!("`{src}` ran without an error"))
}

fn notes(st: &sm_op_api::State) -> Vec<String> {
    st.layer::<DemoModel>().unwrap().notes.clone()
}

fn prints(rep: &RunReport) -> Vec<String> {
    rep.events.iter().filter_map(|e| if let Event::Print { text, .. } = e { Some(text.clone()) } else { None }).collect()
}

// ---- MUST 3: op output order -----------------------------------------------------------------

#[test]
fn op_outputs_keep_the_order_the_op_reported_them() {
    // `create_brick` allocates the body first; a sorted set would put the faces first.
    let (_, st) = ok("let p = model.create_brick(x: 1, y: 1, z: 1)\nmodel.pick(p.created[0])\nmodel.pick(p.created[1])\nmodel.pick(p.created[-1])\n");
    assert_eq!(notes(&st), ["TopoBody#1", "TopoVolume#1", "TopoFace#6"]);
}

#[test]
fn indexing_a_mixed_kind_role_is_the_same_inside_and_outside_a_batch() {
    let top = "let m = test.mixed()\nmodel.pick(m.mix[0])\nmodel.pick(m.mix[1])\nmodel.pick(m.mix[2])\nmodel.pick(m.mix[-1])\n";
    let bat = "batch {\nlet m = test.mixed()\nmodel.pick(m.mix[0])\nmodel.pick(m.mix[1])\nmodel.pick(m.mix[2])\n}\n";
    let (_, a) = ok(top);
    let (_, b) = ok(bat);
    // Face first, then body, then volume: the order the op allocated them.
    assert_eq!(notes(&a)[..3], ["TopoFace#1", "TopoBody#1", "TopoVolume#1"]);
    assert_eq!(notes(&a)[3], "TopoVolume#1");
    assert_eq!(notes(&b), ["TopoFace#1", "TopoBody#1", "TopoVolume#1"]);
    // After the batch the bound variable indexes identically.
    let (_, c) = ok("batch {\nlet m = test.mixed()\n}\nmodel.pick(m.mix[1])\n");
    assert_eq!(notes(&c), ["TopoBody#1"]);
}

#[test]
fn set_algebra_returns_sorted_sets_while_roles_stay_ordered() {
    let (rep, _) = ok("let m = test.mixed()\nprint m.mix\nprint m.mix | m.mix\nassert m.mix == (m.mix | m.mix)\nassert count(m.mix) == 3\n");
    assert_eq!(prints(&rep), ["{Face#1, Body#1, Volume#1}", "{Face#1, Volume#1, Body#1}"]);
}

// ---- MUST 4 / SHOULD 5: computed ids ---------------------------------------------------------

#[test]
fn ids_can_be_built_from_variables() {
    let (rep, st) = ok(
        "model.create_brick(x: 1, y: 1, z: 1)\nmodel.create_brick(x: 2, y: 2, z: 2)\nmodel.create_brick(x: 3, y: 3, z: 3)\n\
         for i in range(1, 4) { model.pick(Volume#[i]) }\n\
         let n = 2\n\
         model.pick(Body#(n + 1))\n\
         print Volume#[1, n + 1], Volume#[1..n], Volume#[1..n * 2 by 2]\n",
    );
    assert_eq!(notes(&st), ["TopoVolume#1", "TopoVolume#2", "TopoVolume#3", "TopoBody#3"]);
    assert_eq!(prints(&rep), ["{Volume#1, Volume#3} {Volume#1, Volume#2} {Volume#1, Volume#3}"]);
}

#[test]
fn computed_serials_are_checked_when_they_are_evaluated() {
    assert_eq!(err("let n = -1\nlet s = Volume#[n]").code, "bad-id-range");
    assert_eq!(err("let s = Volume#[\"a\"]").code, "type-mismatch");
    assert_eq!(err("let a = 3\nlet s = Volume#[a..1]").code, "bad-id-range");
    assert_eq!(err("let a = 0\nlet s = Volume#[1..5 by a]").code, "bad-id-range");
    assert_eq!(err("model.pick(Volume#[7])").code, "entity-not-found");
}

// ---- MUST 5 / SHOULD 6: Cubit vocabulary ------------------------------------------------------

#[test]
fn surface_and_curve_mean_faces_and_edges() {
    let (rep, _) = ok("model.create_brick(x: 1, y: 1, z: 1)\nassert Surface#1 == Face#1\nassert all surfaces == all faces\nassert surfaces in Volume#1 == faces in Volume#1\nassert all curves == all edges\nprint Surface#3\n");
    assert_eq!(prints(&rep), ["{Face#3}"]);
}

#[test]
fn geometry_carriers_have_explicit_names_and_a_pointer_for_cubit_users() {
    // The carrier kinds are `GeomSurface`/`GeomCurve`; none are live yet, and the empty-selection help says what they are.
    let (rep, _, _) = run("let s = all geom_surfaces\n");
    let d = &rep.diagnostics[0];
    assert_eq!(d.code, "empty-selection");
    assert!(d.help.as_deref().is_some_and(|h| h.contains("faces")), "{d:?}");
    let d = err("model.pick(GeomSurface#1)");
    assert_eq!(d.code, "entity-not-found");
    assert!(d.help.as_deref().is_some_and(|h| h.contains("geometry carriers")), "{d:?}");
    // Strings are wire form: the canonical carrier spelling stays the carrier.
    assert_eq!(sm_lang::kinds::parse_entity_text("Surface#7"), Ok(EntityId::new(EntityKind::Surface, 7).unwrap()));
    assert_eq!(sm_lang::kinds::parse_entity_text("GeomSurface#7"), Ok(EntityId::new(EntityKind::Surface, 7).unwrap()));
    assert_eq!(sm_lang::kinds::kind_from_singular("Surface"), Some(EntityKind::TopoFace));
    assert_eq!(sm_lang::kinds::kind_from_singular("GeomCurve"), Some(EntityKind::Curve));
}

// ---- MUST 7: timeline namespace --------------------------------------------------------------

/// An `EngineHost` that records timeline commands.
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

#[test]
fn timeline_statements_are_dispatched_to_the_host_outside_the_op_log() {
    let engine = engine();
    let mut h = TlHost { inner: host(&engine), cmds: Vec::new() };
    let rep = {
        let mut it = Interpreter::new(&mut h, Options::default());
        it.run_source(
            "t",
            "model.create_brick(x: 1, y: 1, z: 1)\ntimeline.branch(\"alt\")\ntimeline.branch(name: \"b2\")\ntimeline.checkout(\"main\")\ntimeline.checkout(node: \"abc\")\ntimeline.checkout(empty: true)\ntimeline.undo()\ntimeline.redo()\n",
        )
    };
    assert!(rep.ok(), "{:?}", rep.diagnostics);
    assert_eq!(
        h.cmds,
        [
            TimelineCommand::Branch { name: "alt".into() },
            TimelineCommand::Branch { name: "b2".into() },
            TimelineCommand::Checkout(TimelineRev::Ref("main".into())),
            TimelineCommand::Checkout(TimelineRev::Node("abc".into())),
            TimelineCommand::Checkout(TimelineRev::Empty),
            TimelineCommand::Undo,
            TimelineCommand::Redo,
        ]
    );
    assert_eq!(rep.records().len(), 1, "timeline commands are not op records");
    assert_eq!(rep.events.iter().filter(|e| matches!(e, Event::Timeline { applied: true, .. })).count(), 7);
}

#[test]
fn timeline_misuse_is_diagnosed() {
    assert_eq!(err("timeline.fork(\"x\")").code, "unknown-name");
    assert!(err("timeline.fork(\"x\")").help.unwrap().contains("branch"));
    assert_eq!(err("timeline.undo(1)").code, "positional-ambiguous");
    assert_eq!(err("timeline.branch()").code, "missing-argument");
    assert_eq!(err("timeline.checkout(ref: \"a\", node: \"b\")").code, "missing-argument");
    assert_eq!(err("timeline.branch(nme: \"a\")").code, "unknown-argument");
    assert_eq!(err("batch { timeline.undo() }").code, "timeline-in-batch");
    assert_eq!(err("let x = 1 + timeline.undo()").code, "op-in-expression");
    // A host without a timeline reports it instead of silently ignoring the statement.
    assert_eq!(err("timeline.undo()").code, "timeline-failed");
    // The demo registry has no op that the reserved namespace would hide.
    let engine = engine();
    assert!(sm_lang::help::callables(engine.registry()).iter().all(|c| !c.name.starts_with("timeline.")));
}

#[test]
fn a_plan_does_not_simulate_the_timeline() {
    let engine = engine();
    let mut h = PlanHost::new(engine.registry(), engine.registry().new_state());
    let mut it = Interpreter::new(&mut h, Options::default());
    let rep = it.run_source("p", "timeline.branch(\"x\")\n");
    assert!(rep.ok());
    assert_eq!(rep.diagnostics[0].code, "plan-timeline");
    assert!(matches!(rep.events[0], Event::Timeline { applied: false, .. }));
}

// ---- SHOULD 1 and 2: numerics ----------------------------------------------------------------

#[test]
fn integers_compare_and_order_exactly() {
    ok("assert max(9007199254740992, 9007199254740993) == 9007199254740993\n\
        assert min(9007199254740993, 9007199254740992) == 9007199254740992\n\
        assert max([3, 9, 4]) == 9\n\
        assert 9007199254740993 != 9007199254740992.0\n\
        assert 9007199254740993 > 9007199254740992.0\n\
        assert 9007199254740992 == 9007199254740992.0\n\
        assert 9007199254740992.0 < 9007199254740993\n\
        assert 1 < 1.5 and 2 > 1.5 and 3 == 3.0 and -1 < -0.5\n\
        assert 9223372036854775807 < 9.3e18\n\
        assert max(1, 2.5) == 2.5\n");
}

#[test]
fn large_integers_are_not_silently_rounded_into_number_parameters() {
    let d = err("model.create_brick(x: 9007199254740993, y: 1, z: 1)");
    assert_eq!(d.code, "integer-precision");
    ok("model.create_brick(x: 9007199254740992, y: 1, z: 1)\n");
}

#[test]
fn builtins_match_their_analytic_values() {
    ok("assert -7 % 3 == -1          // truncated remainder, like C and Rust\n\
        assert 7 % -3 == 1\n\
        assert -7.5 % 2 == -1.5\n\
        assert round(-2.5) == -3     // half away from zero\n\
        assert round(2.5) == 3 and round(-0.4) == 0\n\
        assert floor(-0.5) == -1 and ceil(-0.5) == 0\n\
        assert int(-0.5) == 0 and int(-1.9) == -1 and int(1.9) == 1\n\
        assert atan2(1, 1) > 0 and atan2(1, 1) < pi / 2 and abs(atan2(1, 1) - pi / 4) < 1e-12\n\
        assert atan2(1, -1) > pi / 2 and abs(atan2(1, -1) - 3 * pi / 4) < 1e-12\n\
        assert atan2(-1, -1) < -pi / 2 and abs(atan2(-1, -1) + 3 * pi / 4) < 1e-12\n\
        assert atan2(-1, 1) < 0 and atan2(-1, 1) > -pi / 2 and abs(atan2(-1, 1) + pi / 4) < 1e-12\n\
        assert abs(sqrt(2) ** 2 - 2) < 1e-12 and abs(ln(exp(3)) - 3) < 1e-12 and abs(log10(1000) - 3) < 1e-12\n\
        assert abs(sin(pi / 6) - 0.5) < 1e-12 and abs(cos(pi / 3) - 0.5) < 1e-12\n\
        assert range(5, 0, -2) == [5, 3, 1]\n\
        assert range(0, 5, 2) == [0, 2, 4]\n\
        assert range(3, 3) == [] and range(5, 0) == [] and range(0, 5, -1) == []\n\
        assert len(range(0, 10)) == 10 and range(-2, 1) == [-2, -1, 0]\n\
        assert 2 ** -1 == 0.5 and 2 ** 10 == 1024 and (-2) ** 3 == -8\n");
    assert_eq!(err("let x = sqrt(-1)").code, "non-finite");
    assert_eq!(err("let x = ln(0)").code, "non-finite");
    assert_eq!(err("let x = asin(2)").code, "non-finite");
    assert_eq!(err("let x = 2.0 ** 2000").code, "non-finite");
    assert_eq!(err("let x = 2 ** 63").code, "overflow");
    assert_eq!(err("let x = 9223372036854775807 + 1").code, "overflow");
    assert_eq!(err("let x = 1 / 0").code, "division-by-zero");
    assert_eq!(err("let x = range(0, 5, 0)").code, "bad-arguments");
    assert_eq!(err("let x = range(0, 10000000)").code, "range-too-long");
}

// ---- SHOULD 3: plan mode ---------------------------------------------------------------------

#[test]
fn plan_mode_checks_loops_over_unknown_results() {
    let engine = engine();
    let mut h = PlanHost::new(engine.registry(), engine.registry().new_state());
    let mut it = Interpreter::new(&mut h, Options::default());
    let rep = it.run_source("p", "let b = model.create_brick(x: 1, y: 1, z: 1)\nfor f in b.faces {\n  intent.set_size(ids: f, size: 0.5)\n  let n = f + 1\n}\nmodel.note(text: \"after\")\n");
    assert!(rep.ok(), "{}", render_all(&rep.diagnostics, it.sources()));
    let codes: Vec<&str> = rep.diagnostics.iter().map(|d| d.code).collect();
    assert!(codes.iter().all(|c| *c == "plan-unresolved"), "{codes:?}");
    assert_eq!(rep.records().last().unwrap().name, "model.note", "the statements after the loop still run");
    // A real error in the body is still an error.
    let mut it = Interpreter::new(&mut h, Options::default());
    let rep = it.run_source("q", "let b = model.create_brick(x: 1, y: 1, z: 1)\nfor f in b.faces {\n  model.nope(1)\n}\n");
    assert!(!rep.ok());
}

// ---- SHOULD 4: enum help ---------------------------------------------------------------------

#[test]
fn enum_values_must_be_strings_and_typos_get_suggestions() {
    let d = err("model.create_brick(x: 1, y: 1, z: 1)\nmesh.run(targets: Volume#1, scheme: map)");
    assert_eq!(d.code, "unknown-variable");
    assert!(d.help.as_deref().is_some_and(|h| h.contains("scheme: \"map\"")), "{d:?}");
    let d = err("model.create_brick(x: 1, y: 1, z: 1)\nmesh.run(targets: Volume#1, scheme: \"mapp\")");
    assert_eq!(d.code, "invalid-argument");
    assert!(d.help.as_deref().is_some_and(|h| h.contains("did you mean \"map\"")), "{d:?}");
    // A defined variable of that name wins: it is a value, not an enum word.
    ok("model.create_brick(x: 1, y: 1, z: 1)\nlet map = \"pave\"\nmesh.run(targets: Volume#1, scheme: map)\n");
}

// ---- SHOULD 9: value limits ------------------------------------------------------------------

#[test]
fn growing_values_hit_a_limit() {
    let opts = Options { max_value_len: 1000, ..Options::default() };
    let (rep, _, _) = run_with("let l = [1, 2]\nfor i in range(0, 40) { set l = l + l }\n", opts.clone());
    assert!(rep.diagnostics.iter().any(|d| d.code == "value-too-long"), "{:?}", rep.diagnostics);
    let (rep, _, _) = run_with("let s = \"abcdefghij\"\nfor i in range(0, 40) { set s = s + s }\n", opts.clone());
    assert!(rep.diagnostics.iter().any(|d| d.code == "value-too-long"));
    let (rep, _, _) = run_with("let s = str(range(0, 5000))\n", opts);
    assert!(rep.diagnostics.iter().any(|d| d.code == "value-too-long"), "{:?}", rep.diagnostics);
}

// ---- SHOULD 11 and 24: printing ----------------------------------------------------------------

#[test]
fn print_shows_floats_as_floats_and_ids_as_written() {
    let (rep, _) = ok("model.create_brick(x: 1, y: 1, z: 1)\nprint 10 / 2, 2.0, 1e300, 1.5e-7, 0.1 + 0.2, 7, Face#1, Volume#1, [Body#1]\nprint str(1e300)\n");
    assert_eq!(prints(&rep), ["5.0 2.0 1e300 1.5e-7 0.30000000000000004 7 {Face#1} {Volume#1} [{Body#1}]", "1e300"]);
}

// ---- SHOULD 12 and 19: batch snapshots ----------------------------------------------------------

#[test]
fn naming_an_entity_the_batch_will_create_explains_snapshot_semantics() {
    let d = err("batch { model.create_brick(x: 1, y: 1, z: 1)\nmodel.pick(Body#1) }");
    assert_eq!(d.code, "entity-not-found");
    assert!(d.help.as_deref().is_some_and(|h| h.contains("created inside the block cannot be named")), "{d:?}");
}

#[test]
fn a_selection_after_an_emitted_op_in_a_batch_warns() {
    let (rep, _, _) = run("model.create_brick(x: 1, y: 1, z: 1)\nbatch {\n  model.create_brick(x: 2, y: 2, z: 2)\n  mesh.run(targets: all volumes, scheme: \"map\")\n}\n");
    assert!(rep.ok());
    assert!(rep.diagnostics.iter().any(|d| d.code == "batch-snapshot"), "{:?}", rep.diagnostics);
    // Before any op there is nothing missing, so no warning.
    let (rep, _, _) = run("model.create_brick(x: 1, y: 1, z: 1)\nbatch {\n  mesh.run(targets: all volumes, scheme: \"map\")\n  model.create_brick(x: 2, y: 2, z: 2)\n}\n");
    assert!(rep.diagnostics.is_empty(), "{:?}", rep.diagnostics);
}

// ---- SHOULD 20, 21: membership and `+` ------------------------------------------------------------

#[test]
fn contains_tests_membership() {
    ok("model.create_brick(x: 1, y: 1, z: 1)\nlet s = faces in Volume#1\n\
        assert contains(s, Face#2)\nassert not contains(s, Volume#1)\nassert contains(s, \"Face#3\")\n\
        assert contains([1, 2, 3], 2) and not contains([1, 2], 5)\nassert contains(\"abc\", \"bc\")\nassert contains({a: 1}, \"a\")\n");
}

#[test]
fn plus_on_entity_sets_points_at_union() {
    let d = err("model.create_brick(x: 1, y: 1, z: 1)\nlet s = Face#1 + Face#2");
    assert!(d.message.contains('|'), "{}", d.message);
}

// ---- SHOULD 23: interactive rebinding ---------------------------------------------------------------

#[test]
fn interactive_sessions_may_rebind_top_level_names() {
    let engine = engine();
    let mut h = host(&engine);
    let mut it = Interpreter::new(&mut h, Options { allow_top_level_rebind: true, ..Options::default() });
    assert!(it.run_source("a", "let w = 10\n").ok());
    let rep = it.run_source("b", "let w = 12\nassert w == 12\n");
    assert!(rep.ok());
    assert_eq!(rep.diagnostics[0].code, "shadowed");
    // Inside a block the default rule still applies.
    let rep = it.run_source("c", "if true { let q = 1\nlet q = 2 }\n");
    assert!(!rep.ok());
    // And without the option a rebind is an error.
    let mut h2 = host(&engine);
    let mut it2 = Interpreter::new(&mut h2, Options::default());
    assert!(it2.run_source("a", "let w = 10\n").ok());
    assert!(!it2.run_source("b", "let w = 12\n").ok());
}

// ---- SHOULD 17: queries in expressions --------------------------------------------------------------

#[test]
fn queries_can_be_used_inside_expressions() {
    let (_, st) = ok(
        "model.create_brick(x: 1, y: 1, z: 1)\n\
         assert model.count(kind: \"TopoFace\").count == 6\n\
         let n = model.count().count + 1\n\
         assert n == 9\n\
         intent.set_size(ids: topo.related(kind: \"TopoFace\", within: [Volume#1]), size: 0.5)\n\
         let r = topo.related(kind: \"TopoFace\", within: [Volume#1])\n\
         assert count(r) == 6\n\
         model.pick(topo.related(kind: \"TopoFace\", within: [Volume#1])[2])\n",
    );
    assert_eq!(st.layer::<DemoModel>().unwrap().sizes.len(), 6);
    assert_eq!(notes(&st), ["TopoFace#3"]);
    // Ops are still statements.
    assert_eq!(err("let x = 1 + model.create_brick(x: 1, y: 1, z: 1)").code, "op-in-expression");
    // A query inside a batch is still an error.
    assert_eq!(err("batch { model.pick(model.count()) }").code, "query-in-batch");
}

// ---- SHOULD 25: op errors point at the argument -----------------------------------------------------

#[test]
fn op_failures_point_at_the_argument_that_named_the_entity() {
    let src = "model.create_brick(x: 1, y: 1, z: 1)\nmesh.run(targets: faces in Volume#1, scheme: \"map\")\n";
    let (rep, _, _) = run(src);
    let d = rep.diagnostics.iter().find(|d| d.is_error()).unwrap();
    assert_eq!(d.code, "op-failed");
    assert!(!d.message.contains("failed") || d.message.matches("failed").count() == 1, "{}", d.message);
    assert_eq!(&src[d.span.lo as usize..d.span.hi as usize], "faces in Volume#1");
    assert!(d.help.as_deref().is_some_and(|h| h.contains("targets")), "{d:?}");
}

// ---- SHOULD 26: machine-readable diagnostics ----------------------------------------------------------

#[test]
fn diagnostics_have_a_json_form() {
    let (rep, _, map) = run("let x = 1\nmodel.pick(Volume#9)\n");
    let j = rep.diagnostics_json(&map);
    let d = &j[0];
    assert_eq!(d["code"], "entity-not-found");
    assert_eq!(d["severity"], "error");
    assert_eq!(d["span"]["file"], "t.smesh");
    assert_eq!(d["span"]["line"], 2);
    assert_eq!(d["span"]["col"], 12);
    assert_eq!(d["span"]["lo"], 21);
    assert_eq!(d["span"]["hi"], 29);
    assert!(d["label"].is_string() && d["help"].is_string() && d["message"].is_string());
    assert_eq!(d["notes"], json!([]));
}

// ---- SHOULD 30: no panics on odd states -----------------------------------------------------------------

#[test]
fn rerunning_after_an_error_keeps_the_interpreter_usable() {
    let engine = engine();
    let mut h = host(&engine);
    let mut it = Interpreter::new(&mut h, Options::default());
    assert!(!it.run_source("a", "batch {\n let x = 1 / 0\n}\n").ok());
    assert!(it.run_source("b", "batch {\n model.create_brick(x: 1, y: 1, z: 1)\n}\n").ok());
}

#[test]
fn a_schema_marker_picks_the_positional_parameter() {
    // Two required parameters would normally make a positional argument ambiguous.
    let (_, st) = ok("test.tag(\"a\", n: 2)\n");
    assert_eq!(notes(&st), ["a:2"]);
    assert_eq!(err("test.tag(1, 2)").code, "positional-ambiguous");
}
