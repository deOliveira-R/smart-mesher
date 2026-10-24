//! Project directory format, autosave, open, undo/redo and branch persistence, snapshots.

mod common;

use std::fs;
use std::sync::atomic::Ordering;

use common::*;
use serde_json::json;
use sm_engine::{EngineError, FaultInjector, ProjectConfig, Workspace};
use sm_op_api::{Head, ReplayPolicy, Rev};

fn cfg() -> ProjectConfig {
    ProjectConfig::new("demo")
}

fn populate(ws: &mut Workspace) -> sm_core::EntityId {
    let id = create(ws, "a", 1);
    for k in 1..=4 {
        add(ws, id, k);
    }
    id
}

#[test]
fn create_writes_the_documented_layout() {
    let dir = TempDir::new("layout");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.create(&p, cfg()).unwrap();
    let id = populate(&mut ws);
    ws.fork("try", None).unwrap();
    add(&mut ws, id, 100);

    let files = tree(&p, false);
    let names: Vec<&str> = files.keys().map(String::as_str).collect();
    assert!(names.contains(&"project.toml"));
    assert_eq!(files[".gitignore"], b"/.cache/\n");
    assert!(names.contains(&"timeline/refs/heads/main"));
    assert!(names.contains(&"timeline/refs/heads/try"));
    assert_eq!(
        names
            .iter()
            .filter(|n| n.starts_with("timeline/ops/") && n.ends_with(".toml"))
            .count(),
        6
    );
    assert!(names.contains(&".cache/HEAD"));
    assert!(names.contains(&".cache/reflog.jsonl"));
    // No temp files survive successful writes.
    assert!(
        !names
            .iter()
            .any(|n| n.ends_with(".lock") && !n.ends_with("/lock"))
    );
    // Ref files hold exactly the tip hash.
    let tip = ws.timeline().refs()["try"];
    assert_eq!(
        files["timeline/refs/heads/try"],
        format!("{tip}\n").into_bytes()
    );
    // The files are what Timeline::to_files produces.
    let expect = ws.timeline().to_files().unwrap();
    for (rel, text) in expect {
        assert_eq!(
            files[&format!("timeline/{rel}")],
            text.into_bytes(),
            "{rel}"
        );
    }
    let toml = String::from_utf8(files["project.toml"].clone()).unwrap();
    assert!(toml.contains("format = 1") && toml.contains("name = \"demo\""));
}

#[test]
fn open_replays_to_the_same_state_and_head() {
    let dir = TempDir::new("open");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.create(&p, cfg()).unwrap();
    populate(&mut ws);
    ws.fork("try", None).unwrap();
    let status = ws.status();
    ws.close().unwrap();

    let (b, _) = builder();
    let ws2 = b.open(&p).unwrap();
    assert_eq!(ws2.status(), status);
    assert!(!ws2.dirty());
    assert_eq!(total(&ws2), 11);
    assert_eq!(ws2.config().unwrap().name, "demo");
}

#[test]
fn autosave_writes_only_what_changed() {
    let dir = TempDir::new("incr");
    let p = dir.project("p.smesh");
    let counting = FaultInjector::counting();
    let (b, _) = builder();
    let mut ws = b.fault(counting.clone()).create(&p, cfg()).unwrap();
    let id = create(&mut ws, "a", 0);
    let mut per_op = Vec::new();
    for _ in 0..30 {
        let before = counting.points();
        add(&mut ws, id, 1);
        per_op.push(counting.points() - before);
    }
    // One node file + one ref file (3 points each); HEAD and the redo stack are unchanged.
    assert!(per_op.iter().all(|n| *n == 6), "{per_op:?}");
    // And the on-disk reflog gained exactly one line per op.
    let log = fs::read_to_string(p.join(".cache/reflog.jsonl")).unwrap();
    assert_eq!(log.lines().count(), 31);
}

#[test]
fn undo_redo_and_branch_ops_survive_a_restart() {
    let dir = TempDir::new("undo");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.create(&p, cfg()).unwrap();
    let id = populate(&mut ws);
    let tip5 = ws.status();
    ws.undo().unwrap();
    ws.undo().unwrap();
    let at3 = ws.status();
    assert_eq!(total(&ws), 1 + 1 + 2);
    ws.close().unwrap();

    let (b, _) = builder();
    let mut ws = b.open(&p).unwrap();
    assert_eq!(ws.status(), at3);
    assert_eq!(ws.status().redo, 2);
    ws.redo().unwrap();
    ws.redo().unwrap();
    assert_eq!(ws.status().state, tip5.state);
    assert_eq!(ws.status().position, tip5.position);
    // A new op after undo clears the redo stack, and that survives too.
    ws.undo().unwrap();
    add(&mut ws, id, 7);
    assert_eq!(ws.status().redo, 0);
    ws.close().unwrap();
    let (b, _) = builder();
    let ws = b.open(&p).unwrap();
    assert_eq!(ws.status().redo, 0);
    // The undone nodes are still in the timeline and on disk.
    assert_eq!(ws.timeline().len(), 6);
    assert!(ws.timeline().get(tip5.position.unwrap()).is_some());
}

#[test]
fn branch_create_delete_rename_checkout_fork_persist() {
    let dir = TempDir::new("branches");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.create(&p, cfg()).unwrap();
    let id = populate(&mut ws);
    ws.create_branch("keep", None).unwrap();
    ws.fork("feat/one", None).unwrap();
    add(&mut ws, id, 1);
    ws.checkout(Rev::Ref("main".into())).unwrap();
    ws.rename_branch("feat/one", "feat/two").unwrap();
    ws.delete_branch("keep").unwrap();
    ws.fork("a", None).unwrap();
    ws.checkout(Rev::Ref("main".into())).unwrap();
    ws.delete_branch("a").unwrap();
    let node3 = ws.log(None, 10).unwrap()[2].node;
    ws.checkout(Rev::Node(node3)).unwrap(); // detached
    let status = ws.status();
    ws.close().unwrap();

    let heads = p.join("timeline/refs/heads");
    assert!(heads.join("feat/two").is_file());
    assert!(!heads.join("feat/one").exists());
    assert!(!heads.join("keep").exists());
    assert!(!heads.join("a").exists());

    let (b, _) = builder();
    let ws = b.open(&p).unwrap();
    assert_eq!(ws.status(), status);
    assert_eq!(ws.status().head, Head::Detached(Some(node3)));
}

#[test]
fn branch_shape_changes_on_disk() {
    // File -> directory and case-only renames must not break the ref files.
    let dir = TempDir::new("shape");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.create(&p, cfg()).unwrap();
    populate(&mut ws);
    ws.create_branch("a", None).unwrap();
    ws.rename_branch("a", "a2").unwrap();
    ws.create_branch("Feat", None).unwrap();
    ws.rename_branch("Feat", "feat").unwrap();
    ws.create_branch("x", None).unwrap();
    ws.delete_branch("x").unwrap();
    ws.create_branch("x/y", None).unwrap();
    let st = ws.status();
    ws.close().unwrap();
    let (b, _) = builder();
    assert_eq!(b.open(&p).unwrap().status().branches, st.branches);
}

#[test]
fn deleting_the_cache_is_harmless() {
    let dir = TempDir::new("nocache");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.create(&p, cfg()).unwrap();
    populate(&mut ws);
    ws.fork("other", None).unwrap();
    let tip = ws.status().state;
    ws.close().unwrap();
    fs::remove_dir_all(p.join(".cache")).unwrap();
    let (b, _) = builder();
    let ws = b.open(&p).unwrap();
    // HEAD is lost (default head = main) but the state of main is intact.
    assert_eq!(ws.status().head, Head::Ref("main".into()));
    assert_eq!(ws.status().state, tip);
}

#[test]
fn damaged_cache_files_are_ignored() {
    let dir = TempDir::new("badcache");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.snapshot_every(Some(3)).create(&p, cfg()).unwrap();
    populate(&mut ws);
    let st = ws.status();
    ws.close().unwrap();
    fs::write(p.join(".cache/HEAD"), "{{{{").unwrap();
    fs::write(p.join(".cache/redo.json"), "nonsense").unwrap();
    fs::write(
        p.join(".cache/reflog.jsonl"),
        "{\"ref_name\":\"main\"}\nnot json",
    )
    .unwrap();
    for e in fs::read_dir(p.join(".cache/snapshots")).unwrap() {
        fs::write(e.unwrap().path(), b"garbage").unwrap();
    }
    let (b, _) = builder();
    let ws = b.open(&p).unwrap();
    assert_eq!(ws.status().state, st.state);
    assert!(
        ws.open_report()
            .warnings
            .iter()
            .any(|w| w.contains("snapshot")),
        "{:?}",
        ws.open_report()
    );
}

#[test]
fn snapshots_seed_open_and_skip_replay() {
    let dir = TempDir::new("snap");
    let p = dir.project("p.smesh");
    let (b, counter) = builder();
    let mut ws = b
        .snapshot_every(Some(10))
        .keep_snapshots(2)
        .create(&p, cfg())
        .unwrap();
    let id = create(&mut ws, "a", 0);
    for _ in 0..24 {
        add(&mut ws, id, 1);
    }
    let st = ws.status(); // depth 25
    assert!(counter.load(Ordering::SeqCst) >= 25);
    ws.close().unwrap();
    let snaps: Vec<_> = fs::read_dir(p.join(".cache/snapshots")).unwrap().collect();
    assert_eq!(
        snaps.len(),
        2,
        "pruned to the two deepest (depth 10 and 20)"
    );

    let (b, counter) = builder();
    let ws = b.open(&p).unwrap();
    assert_eq!(ws.status().state, st.state);
    let seeded = ws.open_report().seeded_from.expect("seeded");
    assert_eq!(ws.timeline().depth(seeded), Some(20));
    assert_eq!(
        counter.load(Ordering::SeqCst),
        5,
        "only depth 21..=25 replayed"
    );
}

#[test]
fn stale_snapshot_is_never_used() {
    // A snapshot whose state hash no longer matches the node's recorded check is rejected.
    let dir = TempDir::new("stale");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.create(&p, cfg()).unwrap();
    let id = create(&mut ws, "a", 0);
    add(&mut ws, id, 1);
    let n1 = ws
        .timeline()
        .path_to_root(ws.status().position.unwrap())
        .unwrap()[0];
    ws.snapshot_now().unwrap(); // snapshot of node 2
    let n2 = ws.status().position.unwrap();
    ws.close().unwrap();
    let snap = p.join(format!(".cache/snapshots/{n2}.snap"));
    // Re-label the snapshot of node 2 as node 1's: names the wrong node.
    fs::copy(&snap, p.join(format!(".cache/snapshots/{n1}.snap"))).unwrap();
    fs::remove_file(&snap).unwrap();
    let (b, counter) = builder();
    let ws = b.open(&p).unwrap();
    assert_eq!(ws.status().position, Some(n2));
    assert_eq!(ws.open_report().seeded_from, None);
    assert_eq!(counter.load(Ordering::SeqCst), 2, "full replay");
}

#[test]
fn autosave_off_defers_until_save() {
    let dir = TempDir::new("manual");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.autosave(false).create(&p, cfg()).unwrap();
    let id = create(&mut ws, "a", 0);
    add(&mut ws, id, 1);
    assert!(ws.dirty());
    assert!(fs::read_dir(p.join("timeline/ops")).map_or(true, |d| d.count() == 0));
    ws.save().unwrap();
    assert!(!ws.dirty());
    assert_eq!(fs::read_dir(p.join("timeline/ops")).unwrap().count(), 2);
}

#[test]
fn memory_workspace_saves_as_and_in_memory_save_fails() {
    let dir = TempDir::new("saveas");
    let (b, _) = builder();
    let mut ws = b.memory().unwrap();
    let id = populate(&mut ws);
    ws.fork("x", None).unwrap();
    assert!(matches!(ws.save(), Err(EngineError::NoProject)));
    assert!(!ws.dirty());
    let p = dir.project("p.smesh");
    ws.save_as(&p, cfg()).unwrap();
    add(&mut ws, id, 1);
    let st = ws.status();
    ws.close().unwrap();
    let (b, _) = builder();
    assert_eq!(b.open(&p).unwrap().status(), st);
}

#[test]
fn create_refuses_existing_content_and_open_refuses_non_projects() {
    let dir = TempDir::new("refuse");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let ws = b.create(&p, cfg()).unwrap();
    ws.close().unwrap();
    let (b, _) = builder();
    assert!(matches!(
        b.create(&p, cfg()),
        Err(EngineError::AlreadyExists(_))
    ));
    let (b, _) = builder();
    assert!(matches!(
        b.open(dir.path()),
        Err(EngineError::NotAProject(_))
    ));
    let (b, _) = builder();
    assert!(matches!(
        b.open(dir.project("missing")),
        Err(EngineError::NotAProject(_))
    ));
}

#[test]
fn project_lock_and_read_only() {
    let dir = TempDir::new("lock");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.create(&p, cfg()).unwrap();
    populate(&mut ws);
    let (b, _) = builder();
    assert!(matches!(b.open(&p), Err(EngineError::Locked(_))));
    // A read-only workspace takes no lock, writes nothing, and rejects mutation.
    let before = tree(&p, false);
    let (b, _) = builder();
    let mut ro = b.read_only(true).open(&p).unwrap();
    assert_eq!(ro.status().state, ws.status().state);
    assert!(matches!(
        ro.apply_json("counter.add_all", json!({"delta": 1}), &opts()),
        Err(EngineError::ReadOnly)
    ));
    assert!(matches!(ro.undo(), Err(EngineError::ReadOnly)));
    ro.checkout(Rev::Empty).unwrap();
    assert_eq!(total(&ro), 0);
    ro.close().unwrap();
    assert_eq!(tree(&p, false), before);
    ws.close().unwrap();
    let (b, _) = builder();
    b.open(&p).unwrap(); // lock released
}

#[test]
fn project_toml_round_trips_unknown_keys_and_rejects_newer_formats() {
    let dir = TempDir::new("toml");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut c = cfg();
    c.units = Some("cm".into());
    c.tolerances = Some(sm_engine::Tolerances {
        linear: 1e-6,
        angular: 1e-9,
    });
    c.metadata.insert("owner".into(), "inl".into());
    b.create(&p, c).unwrap().close().unwrap();
    let text = fs::read_to_string(p.join("project.toml")).unwrap();
    fs::write(
        p.join("project.toml"),
        format!("future_key = \"keep me\"\n{text}"),
    )
    .unwrap();
    let (b, _) = builder();
    let mut ws = b.open(&p).unwrap();
    let got = ws.config().unwrap().clone();
    assert_eq!(got.units.as_deref(), Some("cm"));
    assert!(got.extra.contains_key("future_key"));
    ws.set_config(got).unwrap();
    assert!(
        fs::read_to_string(p.join("project.toml"))
            .unwrap()
            .contains("future_key")
    );
    ws.close().unwrap();
    fs::write(p.join("project.toml"), "format = 99\nname = \"x\"\n").unwrap();
    let (b, _) = builder();
    assert!(matches!(
        b.open(&p),
        Err(EngineError::UnsupportedFormat { found: 99, .. })
    ));
    // Control characters would not survive a CRLF checkout.
    let mut bad = cfg();
    bad.description = Some("two\nlines".into());
    assert!(bad.to_toml().is_err());
}

#[test]
fn tampered_result_hash_is_diverged_and_rebaseline_rewrites_the_node() {
    let dir = TempDir::new("tamper");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.create(&p, cfg()).unwrap();
    let id = populate(&mut ws);
    let victim = ws.log(None, 10).unwrap()[2].node;
    let _ = id;
    ws.close().unwrap();
    // `check.state` is not part of the node hash: flip a nibble in it.
    let f = p.join(format!("timeline/ops/{victim}.toml"));
    let text = fs::read_to_string(&f).unwrap();
    let line = text
        .lines()
        .find(|l| l.starts_with("state = "))
        .unwrap()
        .to_owned();
    let flipped = line.replace(
        &line[line.len() - 3..line.len() - 2],
        if line.as_bytes()[line.len() - 3] == b'0' {
            "1"
        } else {
            "0"
        },
    );
    fs::write(&f, text.replace(&line, &flipped)).unwrap();

    let (b, _) = builder();
    match b.open(&p) {
        Err(EngineError::Timeline(sm_op_api::TimelineError::Diverged { node, .. })) => {
            assert_eq!(node, victim)
        }
        other => panic!("expected Diverged, got {:?}", other.err()),
    }
    let (b, _) = builder();
    let ws = b.replay_policy(ReplayPolicy::Rebaseline).open(&p).unwrap();
    assert_eq!(ws.open_report().rebaselined, vec![victim]);
    assert!(!ws.open_report().warnings.is_empty());
    ws.close().unwrap();
    assert_ne!(
        fs::read_to_string(&f).unwrap(),
        text.replace(&line, &flipped),
        "node file was rewritten"
    );
    let (b, _) = builder();
    b.open(&p).unwrap(); // strict open works again
}

#[test]
fn flaky_op_is_reported_as_diverged_on_reopen() {
    let dir = TempDir::new("flaky");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.create(&p, cfg()).unwrap();
    create(&mut ws, "a", 0);
    ws.apply_json("counter.flaky", json!({}), &opts()).unwrap();
    ws.apply_json("counter.flaky", json!({}), &opts()).unwrap();
    ws.close().unwrap();
    // A fresh process sees the Flaky service restart from 0: replay of the first flaky node
    // is fine (n = 0 as recorded), the second gets n = 1 as recorded; both agree. Replaying
    // the same nodes again in the same process does not.
    let (b, _) = builder();
    let ws = b.open(&p).unwrap();
    let tip = *ws.timeline().refs().get("main").unwrap();
    match ws.state_from_root(tip) {
        Err(EngineError::Timeline(sm_op_api::TimelineError::Diverged { .. })) => {}
        other => panic!("expected Diverged, got {:?}", other.map(|s| s.hash())),
    }
}

#[test]
fn stray_and_junk_files_do_not_break_open() {
    let dir = TempDir::new("junk");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.create(&p, cfg()).unwrap();
    populate(&mut ws);
    let st = ws.status();
    ws.close().unwrap();
    let ops = p.join("timeline/ops");
    let heads = p.join("timeline/refs/heads");
    fs::write(ops.join("0123.toml.77-1.lock"), "half a no").unwrap();
    fs::write(ops.join(".DS_Store"), "junk").unwrap();
    fs::write(heads.join("main.99-3.lock"), "deadbeef").unwrap();
    fs::write(heads.join(".DS_Store"), "junk").unwrap();
    let (b, _) = builder();
    let ws = b.open(&p).unwrap();
    assert_eq!(ws.status(), st);
    assert!(!ops.join("0123.toml.77-1.lock").exists(), "swept");
    assert!(!heads.join("main.99-3.lock").exists(), "swept");
    assert!(
        ws.open_report()
            .warnings
            .iter()
            .any(|w| w.contains(".DS_Store"))
    );
}

#[test]
fn corrupt_node_file_is_a_typed_error() {
    let dir = TempDir::new("corrupt");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.create(&p, cfg()).unwrap();
    populate(&mut ws);
    let victim = ws.log(None, 10).unwrap()[1].node;
    ws.close().unwrap();
    let f = p.join(format!("timeline/ops/{victim}.toml"));
    let text = fs::read_to_string(&f).unwrap();
    fs::write(&f, &text[..text.len() / 2]).unwrap(); // truncated by something other than us
    let (b, _) = builder();
    let e = b.open(&p).err().expect("must fail");
    assert!(matches!(e.code(), "format" | "hash-mismatch"), "{e}");
}

#[test]
fn head_on_a_branch_that_vanished_falls_back_but_unborn_after_undo_is_kept() {
    let dir = TempDir::new("head");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.create(&p, cfg()).unwrap();
    populate(&mut ws);
    ws.fork("gone", None).unwrap();
    ws.close().unwrap();
    // A pull deleted the branch HEAD was on.
    fs::remove_file(p.join("timeline/refs/heads/gone")).unwrap();
    let (b, _) = builder();
    let ws = b.open(&p).unwrap();
    assert_eq!(ws.status().head, Head::Ref("main".into()));
    assert_eq!(total(&ws), 11, "not an empty state");
    assert!(
        ws.open_report()
            .warnings
            .iter()
            .any(|w| w.contains("HEAD names"))
    );
    ws.close().unwrap();

    // Undoing a branch to its root leaves it unborn on purpose; that survives a restart.
    let dir = TempDir::new("head2");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.create(&p, cfg()).unwrap();
    let id = create(&mut ws, "a", 1);
    add(&mut ws, id, 1);
    ws.undo().unwrap();
    ws.undo().unwrap();
    assert_eq!(total(&ws), 0);
    ws.close().unwrap();
    let (b, _) = builder();
    let mut ws = b.open(&p).unwrap();
    assert_eq!(ws.status().head, Head::Ref("main".into()));
    assert_eq!(ws.status().position, None);
    ws.redo().unwrap();
    ws.redo().unwrap();
    assert_eq!(total(&ws), 2);

    // Same with another branch alive: the unborn HEAD must not be mistaken for a vanished one.
    let dir = TempDir::new("head3");
    let p = dir.project("p.smesh");
    let (b, _) = builder();
    let mut ws = b.create(&p, cfg()).unwrap();
    let id = create(&mut ws, "a", 1);
    ws.fork("work", None).unwrap();
    add(&mut ws, id, 1);
    ws.undo().unwrap();
    ws.undo().unwrap();
    let st = ws.status();
    assert_eq!((st.position, st.redo), (None, 2));
    ws.close().unwrap();
    let (b, _) = builder();
    assert_eq!(b.open(&p).unwrap().status(), st);
}
