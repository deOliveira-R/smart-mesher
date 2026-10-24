//! Snapshot seeds are validated against the node's recorded check; cache damage is survivable.

mod common;

use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::Ordering;

use common::*;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_engine::{EngineError, FixedClock, ProjectConfig, Workspace};
use sm_op_api::{
    ApplyCtx, Hash256, OpError, Plugin, Registry, RegistryError, ReplayPolicy, Services, TypedOp,
};

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

/// Same name and params as `counter.add`, but a new implementation version.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct AddV2 {
    id: EntityId,
    delta: i64,
}

impl TypedOp for AddV2 {
    const NAME: &'static str = "counter.add";
    const IMPL_VERSION: u32 = 2;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let id = ctx.resolve_kind(self.id, EntityKind::TopoBody)?;
        let c = ctx
            .layer_mut::<CounterLayer>()?
            .counters
            .get_mut(id)
            .ok_or(OpError::UnknownEntity(id))?;
        c.value += self.delta;
        Ok(())
    }
}

struct V2Plugin;

impl Plugin for V2Plugin {
    fn id(&self) -> &str {
        "demo.counter.v2"
    }
    fn register(&self, r: &mut Registry) -> Result<(), RegistryError> {
        r.register_layer::<CounterLayer>()?;
        r.register::<Create>()?;
        r.register::<AddV2>()?;
        r.register_query::<Total>()?;
        Ok(())
    }
}

#[test]
fn snapshot_with_a_changed_op_impl_version_is_not_used() {
    let dir = TempDir::new("snapversion");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.create(&p, cfg()).unwrap();
    let id = create(&mut ws, "a", 0);
    let shallow = ws.status().position.unwrap(); // only counter.create on its path
    assert!(ws.snapshot_now().unwrap());
    for _ in 0..3 {
        add(&mut ws, id, 1);
    }
    let tip = ws.status().position.unwrap();
    assert!(ws.snapshot_now().unwrap());
    ws.close().unwrap();

    // Unchanged registry: the deepest snapshot seeds the replay.
    let (b, _) = builder();
    let ws = b.open(&p).unwrap();
    assert_eq!(ws.open_report().seeded_from, Some(tip));
    ws.close().unwrap();

    // `counter.add` changed implementation: the tip snapshot depends on it and is skipped,
    // the older one (create only) is still valid.
    let v2 = || {
        Workspace::builder()
            .plugin(V2Plugin)
            .services(
                Services::new()
                    .with(common::ApplyCount::default())
                    .with(common::Flaky::default()),
            )
            .clock(FixedClock(TIME.to_owned()))
            .author("tester")
    };
    let ws = v2().open(&p).unwrap();
    assert_eq!(ws.open_report().seeded_from, Some(shallow));
    assert!(
        ws.open_report()
            .warnings
            .iter()
            .any(|w| w.contains(&tip.to_string()) && w.contains("versions")),
        "{:?}",
        ws.open_report().warnings
    );
    assert_eq!(total(&ws), 3);
    ws.close().unwrap();
    // Without the older snapshot there is nothing valid left: full replay from the root.
    fs::remove_file(p.join(format!(".cache/snapshots/{shallow}.snap"))).unwrap();
    let ws = v2().open(&p).unwrap();
    assert_eq!(ws.open_report().seeded_from, None);
    assert_eq!(total(&ws), 3);
    ws.close().unwrap();

    // The stale file was kept, so restoring the old registry makes it usable again.
    let (b, _) = builder();
    let ws = b.open(&p).unwrap();
    assert_eq!(ws.open_report().seeded_from, Some(tip));
}
