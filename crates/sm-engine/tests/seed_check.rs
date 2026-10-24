//! Snapshot seeds are validated against the node's recorded check; cache damage is survivable.

mod common;

use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::Ordering;

use common::*;

use sm_engine::{EngineError, ProjectConfig};
use sm_op_api::{Hash256, ReplayPolicy};

fn cfg() -> ProjectConfig {
    ProjectConfig::new("demo")
}

/// Flip one nibble of a node's `check.state` line (not part of the node hash).
fn tamper_state(p: &Path, node: Hash256) {
    let f = p.join(format!("timeline/ops/{node}.toml"));
    let text = fs::read_to_string(&f).unwrap();
    let line = text
        .lines()
        .find(|l| l.starts_with("state = "))
        .unwrap()
        .to_owned();
    let i = line.len() - 3;
    let c = if line.as_bytes()[i] == b'0' { "1" } else { "0" };
    let flipped = format!("{}{}{}", &line[..i], c, &line[i + 1..]);
    fs::write(&f, text.replace(&line, &flipped)).unwrap();
}

fn tampered_tip_project(name: &str) -> (TempDir, PathBuf, Hash256) {
    let dir = TempDir::new(name);
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.create(&p, cfg()).unwrap();
    let id = create(&mut ws, "a", 0);
    for _ in 0..3 {
        add(&mut ws, id, 1);
    }
    assert!(ws.snapshot_now().unwrap());
    let tip = ws.status().position.unwrap();
    ws.close().unwrap();
    tamper_state(&p, tip);
    (dir, p, tip)
}

#[test]
fn snapshot_disagreeing_with_recorded_check_is_dropped_not_used() {
    let (_d, p, tip) = tampered_tip_project("snaptamper");
    // Strict: Diverged comes from the replay, not from a seed.
    let (b, counter) = builder();
    match b.open(&p) {
        Err(EngineError::Timeline(sm_op_api::TimelineError::Diverged { node, .. })) => {
            assert_eq!(node, tip)
        }
        other => panic!("expected Diverged, got {:?}", other.err()),
    }
    assert_eq!(counter.load(Ordering::SeqCst), 4, "full replay, no seed");
    assert!(!p.join(format!(".cache/snapshots/{tip}.snap")).exists());
}

#[test]
fn rebaseline_and_warn_survive_a_stale_snapshot() {
    for policy in [ReplayPolicy::Rebaseline, ReplayPolicy::Warn] {
        let (_d, p, tip) = tampered_tip_project("snaprebase");
        let (b, _) = builder();
        let mut ws = b.replay_policy(policy).open(&p).unwrap();
        assert_eq!(ws.open_report().seeded_from, None);
        assert!(
            ws.open_report()
                .warnings
                .iter()
                .any(|w| w.contains("snapshot"))
        );
        if policy == ReplayPolicy::Rebaseline {
            assert_eq!(ws.open_report().rebaselined, vec![tip]);
        }
        // Under Warn the state still disagrees with the node, so no snapshot is written.
        let wrote = ws.snapshot_now().unwrap();
        if policy == ReplayPolicy::Warn {
            assert!(!wrote);
        }
        ws.close().unwrap();
        let (b, _) = builder();
        b.replay_policy(policy).open(&p).unwrap();
    }
}

#[test]
fn snapshot_now_keeps_the_snapshot_it_just_wrote() {
    let dir = TempDir::new("snapprune");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.keep_snapshots(1).create(&p, cfg()).unwrap();
    let id = create(&mut ws, "a", 0);
    add(&mut ws, id, 1);
    add(&mut ws, id, 1);
    assert!(ws.snapshot_now().unwrap()); // deep
    ws.undo().unwrap();
    ws.undo().unwrap();
    let shallow = ws.status().position.unwrap();
    assert!(ws.snapshot_now().unwrap());
    assert!(p.join(format!(".cache/snapshots/{shallow}.snap")).exists());
    ws.close().unwrap();
}

#[test]
fn reflog_bad_line_does_not_drop_newer_entries() {
    let dir = TempDir::new("reflogskip");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.create(&p, cfg()).unwrap();
    let id = create(&mut ws, "a", 0);
    add(&mut ws, id, 1);
    add(&mut ws, id, 1);
    ws.close().unwrap();
    let f = p.join(".cache/reflog.jsonl");
    let text = fs::read_to_string(&f).unwrap();
    let n = text.lines().count();
    assert!(n >= 3, "{text}");
    let mut lines: Vec<&str> = text.lines().collect();
    lines[1] = "{garbage";
    fs::write(&f, lines.join("\n") + "\n").unwrap();
    let (b, _) = builder();
    let ws = b.open(&p).unwrap();
    ws.close().unwrap();
    assert_eq!(fs::read_to_string(&f).unwrap().lines().count(), n - 1);
}
