//! Git round trip: save a project, commit it with real `git`, clone it (also with CRLF
//! conversion), reopen, and require replay to equal the original. Also: branches merge
//! without conflicts, and a project survives `git checkout` of an older commit under a stale
//! `.cache/`.

mod common;

use std::collections::BTreeMap;
use std::path::Path;
use std::process::Command;

use common::*;
use sm_engine::{ProjectConfig, Workspace};
use sm_op_api::{Hash256, Rev};

fn git_cmd(dir: &Path) -> Command {
    let mut c = Command::new("git");
    c.current_dir(dir)
        .env("GIT_CONFIG_NOSYSTEM", "1")
        .env("GIT_CONFIG_GLOBAL", "/dev/null")
        .env("GIT_TERMINAL_PROMPT", "0")
        .args([
            "-c",
            "user.name=t",
            "-c",
            "user.email=t@example.com",
            "-c",
            "commit.gpgsign=false",
        ]);
    c
}

fn git(dir: &Path, args: &[&str]) -> String {
    let out = git_cmd(dir)
        .args(args)
        .output()
        .expect("git must be installed to run the git tests");
    assert!(
        out.status.success(),
        "git {args:?} failed: {}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    String::from_utf8_lossy(&out.stdout).into_owned()
}

fn commit_all(dir: &Path, msg: &str) {
    git(dir, &["add", "-A"]);
    git(dir, &["commit", "-q", "-m", msg]);
}

fn clone(from: &Path, to: &Path, extra: &[&str]) {
    let parent = to.parent().unwrap();
    let mut args = vec!["clone", "-q"];
    args.extend_from_slice(extra);
    args.push(from.to_str().unwrap());
    args.push(to.to_str().unwrap());
    git(parent, &args);
}

/// Every branch tip with the state hash a checkout of it produces.
fn tips(ws: &mut Workspace) -> BTreeMap<String, (Hash256, Hash256)> {
    let mut out = BTreeMap::new();
    for (n, h) in ws.timeline().refs().clone() {
        out.insert(n.clone(), (h, ws.state_at(&Rev::Ref(n)).unwrap().hash()));
    }
    out
}

/// A project at `root` with `main` (5 nodes) and `feat/x` forked at node 3 (2 more nodes).
fn make_project(root: &Path) -> BTreeMap<String, (Hash256, Hash256)> {
    let (b, _) = builder();
    let mut ws = b.create(root, ProjectConfig::new("git-demo")).unwrap();
    let id = create(&mut ws, "a", 0);
    for k in 1..=4 {
        add(&mut ws, id, k);
    }
    let n3 = ws.log(None, 10).unwrap()[2].node;
    ws.fork("feat/x", Some(n3)).unwrap();
    add(&mut ws, id, 50);
    ws.apply(&ScaleGain { id, factor: 3.0 }, &opts()).unwrap();
    ws.checkout(Rev::Ref("main".into())).unwrap();
    let t = tips(&mut ws);
    ws.close().unwrap();
    t
}

fn open(root: &Path) -> Workspace {
    let (b, _) = builder();
    b.open(root).unwrap()
}

#[test]
fn save_commit_clone_reopen_replays_equal() {
    let dir = TempDir::new("git1");
    let src = dir.project("src");
    let want = make_project(&src);
    git(&src, &["init", "-q"]);
    commit_all(&src, "project");

    // The cache is not committed and the tree is clean afterwards.
    let tracked = git(&src, &["ls-files"]);
    assert!(!tracked.contains(".cache"), "{tracked}");
    assert!(tracked.contains("project.toml") && tracked.contains("timeline/refs/heads/feat/x"));
    assert_eq!(git(&src, &["status", "--porcelain"]).trim(), "");

    let dst = dir.project("dst");
    clone(&src, &dst, &[]);
    assert!(!dst.join(".cache").exists());
    let mut ws = open(&dst);
    assert_eq!(tips(&mut ws), want);
    // Cold replay from the root agrees too.
    for (tip, state) in want.values() {
        assert_eq!(ws.state_from_root(*tip).unwrap().hash(), *state);
    }
    // And the original, reopened in place, still agrees (its .cache is intact).
    let mut orig = open(&src);
    assert_eq!(tips(&mut orig), want);
}

#[test]
fn crlf_checkout_still_opens_and_replays() {
    let dir = TempDir::new("git2");
    let src = dir.project("src");
    let want = make_project(&src);
    git(&src, &["init", "-q"]);
    commit_all(&src, "project");
    let dst = dir.project("crlf");
    clone(&src, &dst, &["-c", "core.autocrlf=true"]);
    let some_node = std::fs::read_dir(dst.join("timeline/ops"))
        .unwrap()
        .next()
        .unwrap()
        .unwrap()
        .path();
    let raw = std::fs::read(&some_node).unwrap();
    assert!(
        raw.windows(2).any(|w| w == b"\r\n"),
        "the checkout must actually be CRLF for this test to mean anything"
    );
    let mut ws = open(&dst);
    assert_eq!(tips(&mut ws), want);
}

#[test]
fn independent_branches_merge_without_conflicts() {
    let dir = TempDir::new("git3");
    let src = dir.project("src");
    make_project(&src);
    git(&src, &["init", "-q"]);
    commit_all(&src, "base");

    let (a, b_) = (dir.project("a"), dir.project("b"));
    clone(&src, &a, &[]);
    clone(&src, &b_, &[]);
    let mut wa = open(&a);
    let mut wb = open(&b_);
    let id = wa
        .state()
        .entities_of(sm_core::EntityKind::TopoBody)
        .next()
        .unwrap();
    wa.fork("a-work", None).unwrap();
    add(&mut wa, id, 1000);
    add(&mut wa, id, 1);
    let ta = tips(&mut wa);
    wa.close().unwrap();
    wb.fork("b-work", None).unwrap();
    add(&mut wb, id, 7);
    let tb = tips(&mut wb);
    wb.close().unwrap();
    commit_all(&a, "a");
    commit_all(&b_, "b");

    git(&a, &["fetch", "-q", b_.to_str().unwrap(), "HEAD"]);
    git(&a, &["merge", "-q", "--no-edit", "FETCH_HEAD"]);
    assert_eq!(git(&a, &["status", "--porcelain"]).trim(), "");

    let mut merged = open(&a);
    let m = tips(&mut merged);
    assert_eq!(m["a-work"], ta["a-work"]);
    assert_eq!(m["b-work"], tb["b-work"]);
    assert_eq!(m["main"], ta["main"]);
    assert_eq!(merged.timeline().len(), 7 + 2 + 1);
}

#[test]
fn two_writers_on_one_branch_conflict_only_in_its_ref_file() {
    let dir = TempDir::new("git4");
    let src = dir.project("src");
    make_project(&src);
    git(&src, &["init", "-q"]);
    commit_all(&src, "base");
    let (a, b_) = (dir.project("a"), dir.project("b"));
    clone(&src, &a, &[]);
    clone(&src, &b_, &[]);
    for (p, delta) in [(&a, 11), (&b_, 22)] {
        let mut ws = open(p);
        let id = ws
            .state()
            .entities_of(sm_core::EntityKind::TopoBody)
            .next()
            .unwrap();
        add(&mut ws, id, delta);
        ws.close().unwrap();
        commit_all(p, "advance main");
    }
    git(&a, &["fetch", "-q", b_.to_str().unwrap(), "HEAD"]);
    let out = git_cmd(&a)
        .args(["merge", "--no-edit", "FETCH_HEAD"])
        .output()
        .unwrap();
    assert!(
        !out.status.success(),
        "both moved main: a real conflict is expected"
    );
    let conflicted = git(&a, &["diff", "--name-only", "--diff-filter=U"]);
    assert_eq!(
        conflicted.trim(),
        "timeline/refs/heads/main",
        "node files never conflict"
    );
    // Resolve by keeping the other writer's tip, and park ours on a branch from its node.
    git(&a, &["checkout", "--theirs", "timeline/refs/heads/main"]);
    commit_all(&a, "merge");
    let ws = open(&a);
    assert_eq!(
        ws.timeline().len(),
        7 + 1 + 1,
        "all nodes of both writers survive the merge"
    );
}

#[test]
fn checking_out_an_older_commit_under_a_stale_cache() {
    let dir = TempDir::new("git5");
    let p = dir.project("p");
    let (b, _) = builder();
    let mut ws = b
        .snapshot_every(Some(2))
        .create(&p, ProjectConfig::new("stale"))
        .unwrap();
    let id = create(&mut ws, "a", 0);
    add(&mut ws, id, 1);
    ws.close().unwrap();
    git(&p, &["init", "-q"]);
    commit_all(&p, "two nodes");
    let two = open(&p).status();

    let (b, _) = builder();
    let mut ws = b.snapshot_every(Some(2)).open(&p).unwrap();
    add(&mut ws, id, 1);
    add(&mut ws, id, 1);
    ws.undo().unwrap(); // leaves a redo stack and a snapshot at depth 4 in .cache
    ws.close().unwrap();
    commit_all(&p, "four nodes");

    git(&p, &["checkout", "-q", "HEAD~1"]);
    let ws = open(&p); // .cache still describes the newer history
    assert_eq!(ws.status().state, two.state);
    assert_eq!(ws.status().nodes, 2);
    assert_eq!(ws.status().redo, 0);
    assert!(
        ws.open_report().warnings.iter().any(|w| w.contains("redo")),
        "{:?}",
        ws.open_report()
    );
}
