//! Diagnostics policy: verbosity, suppression, stop-on-warning.

mod common;

use common::*;
use serde_json::json;
use sm_engine::{ApplyOptions, DiagnosticPolicy, EngineError, Verbosity};
use sm_op_api::Severity;

fn setup(policy: DiagnosticPolicy) -> (sm_engine::Workspace, sm_core::EntityId) {
    let (b, _) = builder();
    let mut ws = b.policy(policy).memory().unwrap();
    let id = create(&mut ws, "a", 0);
    (ws, id)
}

#[test]
fn default_policy_reports_warnings_and_hides_info() {
    let (mut ws, id) = setup(DiagnosticPolicy::default());
    let r = ws.apply(&Warn { id }, &opts()).unwrap();
    assert_eq!(r.delta.diagnostics.len(), 1);
    assert_eq!(r.delta.diagnostics[0].code, "demo.note");
    assert_eq!(r.delta.diagnostics[0].severity, Severity::Warning);
    assert_eq!(r.delta.diagnostics[0].entities, vec![id]);
    assert_eq!(r.hidden_diagnostics, 1);
}

#[test]
fn verbosity_levels() {
    for (v, shown) in [
        (Verbosity::Quiet, 0),
        (Verbosity::Warnings, 1),
        (Verbosity::Info, 2),
    ] {
        let (mut ws, id) = setup(DiagnosticPolicy {
            verbosity: v,
            ..Default::default()
        });
        let r = ws.apply(&Warn { id }, &opts()).unwrap();
        assert_eq!(r.delta.diagnostics.len(), shown, "{v:?}");
        assert_eq!(r.hidden_diagnostics, 2 - shown);
    }
}

#[test]
fn suppressed_codes_are_neither_shown_nor_blocking() {
    let mut p = DiagnosticPolicy {
        verbosity: Verbosity::Info,
        stop_on_warning: true,
        ..Default::default()
    };
    p.suppress.insert("demo.note".into());
    let (mut ws, id) = setup(p);
    let r = ws.apply(&Warn { id }, &opts()).unwrap();
    assert_eq!(
        r.delta
            .diagnostics
            .iter()
            .map(|d| d.code.as_str())
            .collect::<Vec<_>>(),
        ["demo.fyi"]
    );
}

#[test]
fn stop_on_warning_records_nothing_and_leaves_state_untouched() {
    let (mut ws, id) = setup(DiagnosticPolicy {
        stop_on_warning: true,
        verbosity: Verbosity::Quiet,
        ..Default::default()
    });
    let before = ws.status();
    let e = ws.apply(&Warn { id }, &opts()).unwrap_err();
    match &e {
        EngineError::StoppedOnWarning { diagnostics } => {
            assert_eq!(diagnostics[0].code, "demo.note")
        }
        other => panic!("{other:?}"),
    }
    assert_eq!(e.code(), "stopped-on-warning");
    assert_eq!(ws.status(), before, "no node, no state change, no new ids");
    // Ops that do not warn are unaffected, and the id counter did not advance.
    let id2 = create(&mut ws, "b", 5);
    assert_ne!(id, id2);
    assert_eq!(total(&ws), 5);
}

#[test]
fn accept_warnings_overrides_stop_for_one_call() {
    let (mut ws, id) = setup(DiagnosticPolicy {
        stop_on_warning: true,
        ..Default::default()
    });
    let o = ApplyOptions {
        accept_warnings: true,
        message: Some("really".into()),
        ..Default::default()
    };
    let r = ws
        .apply_json("counter.warn", json!({"id": id}), &o)
        .unwrap();
    assert_eq!(r.delta.diagnostics.len(), 1, "still reported");
    assert_eq!(ws.log(None, 1).unwrap()[0].message, "really");
    assert!(ws.apply(&Warn { id }, &opts()).is_err());
}

#[test]
fn policy_changes_apply_to_later_calls_and_do_not_touch_hashes() {
    let (mut a, ida) = setup(DiagnosticPolicy::default());
    let (mut b, idb) = setup(DiagnosticPolicy {
        verbosity: Verbosity::Quiet,
        ..Default::default()
    });
    let ra = a.apply(&Warn { id: ida }, &opts()).unwrap();
    let rb = b.apply(&Warn { id: idb }, &opts()).unwrap();
    assert_eq!(
        ra.node, rb.node,
        "diagnostics are front-end policy, not part of any hash"
    );
    assert_eq!(ra.state, rb.state);
    b.set_policy(DiagnosticPolicy {
        verbosity: Verbosity::Info,
        ..Default::default()
    });
    assert_eq!(
        b.apply(&Warn { id: idb }, &opts())
            .unwrap()
            .delta
            .diagnostics
            .len(),
        2
    );
}
