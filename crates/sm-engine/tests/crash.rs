//! Crash safety: for every instrumented write/delete point of a scenario, kill the writer there
//! (leaving exactly what a dead process leaves: a half-written temp file, a complete temp file,
//! a renamed file) and check that the project still opens, every branch tip replays to its
//! recorded state, and the workspace is usable afterwards.

mod common;

use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

use common::*;
use sm_engine::{EngineError, FaultInjector, ProjectConfig, Workspace};
use sm_op_api::{Hash256, Rev};

fn copy_dir(from: &Path, to: &Path) {
    fs::create_dir_all(to).unwrap();
    for e in fs::read_dir(from).unwrap() {
        let e = e.unwrap();
        let dest = to.join(e.file_name());
        if e.file_type().unwrap().is_dir() {
            copy_dir(&e.path(), &dest);
        } else {
            fs::copy(e.path(), dest).unwrap();
        }
    }
}

/// A project with 5 nodes on main and a side branch at node 3.
fn base_project(path: &Path) -> BTreeMap<String, Hash256> {
    let (b, _) = builder();
    let mut ws = b.create(path, ProjectConfig::new("crash")).unwrap();
    let id = create(&mut ws, "a", 0);
    for k in 1..=4 {
        add(&mut ws, id, k);
    }
    let n3 = ws.log(None, 10).unwrap()[2].node;
    ws.create_branch("side", Some(n3)).unwrap();
    ws.create_branch("old", Some(n3)).unwrap();
    let tips = ws.timeline().refs().clone();
    ws.close().unwrap();
    tips
}

fn no_leftover_temp_files(root: &Path) {
    for k in tree(root, false).keys() {
        assert!(
            !(k.ends_with(".lock") && !k.ends_with("/lock")),
            "temp file left behind: {k}"
        );
    }
}

/// What a crash may legitimately leave: every tip a branch had at any step boundary of the
/// scenario, and the branches that exist throughout (those may never be lost).
struct Expect {
    tips: BTreeMap<String, Vec<Hash256>>,
    always: Vec<String>,
    heads: Vec<sm_op_api::Head>,
}

impl Expect {
    fn new(base: &BTreeMap<String, Hash256>) -> Self {
        let mut e = Self {
            tips: BTreeMap::new(),
            always: base.keys().cloned().collect(),
            heads: vec![sm_op_api::Head::Ref("main".into())],
        };
        e.record(base, false);
        e
    }
    fn record_head(&mut self, h: &sm_op_api::Head) {
        if !self.heads.contains(h) {
            self.heads.push(h.clone());
        }
    }
    fn record(&mut self, refs: &BTreeMap<String, Hash256>, prune: bool) {
        for (n, h) in refs {
            let v = self.tips.entry(n.clone()).or_default();
            if !v.contains(h) {
                v.push(*h);
            }
        }
        if prune {
            self.always.retain(|n| refs.contains_key(n));
        }
    }
}

/// Reopen after the crash and check every invariant.
fn verify(root: &Path, expect: &Expect, k: usize) {
    let (b, _) = builder();
    let mut ws = b
        .open(root)
        .unwrap_or_else(|e| panic!("point {k}: project does not open: {e}"));
    no_leftover_temp_files(root);
    let warned = ws
        .open_report()
        .warnings
        .iter()
        .any(|w| w.contains("HEAD names"));
    assert!(
        !warned,
        "point {k}: HEAD dangled and fell back: {:?}",
        ws.open_report().warnings
    );
    assert!(
        expect.heads.contains(ws.session().head()),
        "point {k}: HEAD is {:?}",
        ws.session().head()
    );
    for (name, tip) in ws.timeline().refs().clone() {
        let ok = expect.tips.get(&name).is_some_and(|v| v.contains(&tip));
        assert!(ok, "point {k}: branch {name} at unexpected node {tip}");
        let cold = ws
            .state_from_root(tip)
            .unwrap_or_else(|e| panic!("point {k}: replay of {name}: {e}"));
        ws.checkout(Rev::Ref(name.clone())).unwrap();
        assert_eq!(ws.state_hash(), cold.hash(), "point {k}: {name}");
    }
    for name in &expect.always {
        assert!(
            ws.timeline().refs().contains_key(name),
            "point {k}: branch {name} lost"
        );
    }
    // Usable afterwards: record another op and reopen.
    let first = ws.state().entities_of(sm_core::EntityKind::TopoBody).next();
    if let Some(id) = first {
        ws.apply(&Add { id, delta: 1 }, &opts())
            .unwrap_or_else(|e| panic!("point {k}: {e}"));
        let st = ws.status();
        ws.close().unwrap();
        let (b, _) = builder();
        assert_eq!(b.open(root).unwrap().status().state, st.state, "point {k}");
    }
}

type Act = dyn Fn(&mut Workspace, &mut dyn FnMut(&Workspace)) -> Result<(), EngineError>;

/// Run `act` on a copy of the base project once under a counting injector (collecting what is
/// acceptable at each step boundary), then once per point under a failing injector.
fn sweep(tag: &str, act: &Act) {
    let dir = TempDir::new(tag);
    let base_path = dir.project("base");
    let base = base_project(&base_path);

    let counted = dir.project("count");
    copy_dir(&base_path, &counted);
    let inj = FaultInjector::counting();
    let (b, _) = builder();
    let mut ws = b.fault(inj.clone()).open(&counted).unwrap();
    let mut expect = Expect::new(&base);
    act(&mut ws, &mut |w| {
        expect.record(w.timeline().refs(), true);
        expect.record_head(w.session().head());
    })
    .unwrap();
    expect.record(ws.timeline().refs(), true);
    expect.record_head(ws.session().head());
    ws.close().unwrap();
    let points = inj.points();
    assert!(points >= 4, "scenario should write something ({points})");

    for k in 0..points {
        let work = dir.project(&format!("work{k}"));
        copy_dir(&base_path, &work);
        let inj = FaultInjector::fail_at(k);
        let (b, _) = builder();
        let mut ws = b.fault(inj.clone()).open(&work).unwrap();
        let r = act(&mut ws, &mut |_| {});
        assert!(inj.tripped(), "point {k} of {points} never reached");
        assert!(
            r.is_err(),
            "point {k}: injected crash must surface as an error"
        );
        drop(ws); // the process is dead: nothing else gets written
        verify(&work, &expect, k);
    }
    // And with no crash everything is there.
    verify(&counted, &expect, points);
}

fn first_counter(ws: &Workspace) -> sm_core::EntityId {
    ws.state()
        .entities_of(sm_core::EntityKind::TopoBody)
        .next()
        .expect("a counter")
}

#[test]
fn crash_while_applying_an_op() {
    sweep("apply", &|ws, step| {
        let id = first_counter(ws);
        ws.apply(&Add { id, delta: 5 }, &opts())?;
        step(ws);
        Ok(())
    });
}

#[test]
fn crash_while_forking_and_applying() {
    sweep("fork", &|ws, step| {
        let id = first_counter(ws);
        let n3 = ws.log(None, 10)?[2].node;
        ws.fork("try", Some(n3))?;
        step(ws);
        ws.apply(&Add { id, delta: 100 }, &opts())?;
        step(ws);
        ws.apply(&Add { id, delta: 200 }, &opts())?;
        step(ws);
        Ok(())
    });
}

#[test]
fn crash_while_renaming_and_deleting_branches() {
    sweep("rename", &|ws, step| {
        ws.rename_branch("side", "renamed/x")?;
        step(ws);
        ws.delete_branch("old")?;
        step(ws);
        Ok(())
    });
}

#[test]
fn crash_while_renaming_the_attached_branch_never_orphans_head() {
    sweep("head", &|ws, step| {
        ws.checkout(Rev::Ref("side".into()))?;
        step(ws);
        ws.rename_branch("side", "side2")?;
        step(ws);
        Ok(())
    });
}

#[test]
fn crash_while_undoing_and_redoing() {
    sweep("undo", &|ws, step| {
        ws.undo()?;
        step(ws);
        ws.undo()?;
        step(ws);
        ws.redo()?;
        step(ws);
        Ok(())
    });
}

#[test]
fn crash_during_a_file_to_directory_branch_change_may_lose_that_branch_but_not_the_project() {
    // `old` -> `old/inner` needs the file removed before the directory can exist: the one case
    // where a crash can drop the branch name (its nodes stay in the timeline).
    sweep("shape", &|ws, step| {
        ws.rename_branch("side", "tmp")?;
        step(ws);
        ws.rename_branch("old", "old/inner")?;
        step(ws);
        Ok(())
    });
}

#[test]
fn crash_while_creating_a_project_leaves_no_half_project() {
    let dir = TempDir::new("create");
    let build = |inj: Option<std::sync::Arc<FaultInjector>>| {
        let (mut b, _) = builder();
        if let Some(i) = inj {
            b = b.fault(i);
        }
        let mut ws = b.memory().unwrap();
        let id = create(&mut ws, "a", 0);
        add(&mut ws, id, 1);
        add(&mut ws, id, 2);
        ws.create_branch("side", None).unwrap();
        ws
    };
    let inj = FaultInjector::counting();
    let mut ws = build(Some(inj.clone()));
    ws.save_as(dir.project("count"), ProjectConfig::new("c"))
        .unwrap();
    let points = inj.points();
    let tips = ws.timeline().refs().clone();
    for k in 0..points {
        let p = dir.project(&format!("c{k}"));
        let inj = FaultInjector::fail_at(k);
        let mut ws = build(Some(inj));
        assert!(
            ws.save_as(&p, ProjectConfig::new("c")).is_err(),
            "point {k}"
        );
        // The failed save cleaned up after itself: the path is absent, and a retry works.
        assert!(!p.exists(), "point {k}: leftovers after failed save_as");
        ws.save_as(&p, ProjectConfig::new("c"))
            .unwrap_or_else(|e| panic!("point {k}: retry failed: {e}"));
        drop(ws);
        let (b, _) = builder();
        b.open(&p).unwrap_or_else(|e| panic!("point {k}: {e}"));
        let p = dir.project(&format!("d{k}"));
        let inj = FaultInjector::fail_at(k);
        let mut ws = build(Some(inj));
        assert!(ws.save_as(&p, ProjectConfig::new("c")).is_err());
        drop(ws);
        let (b, _) = builder();
        match b.open(&p) {
            Err(EngineError::NotAProject(_)) => {} // project.toml is written last
            Ok(ws) => assert_eq!(
                ws.timeline().refs(),
                &tips,
                "point {k}: a project that opens is complete"
            ),
            Err(e) => panic!("point {k}: {e}"),
        }
    }
}

#[test]
fn a_failed_save_can_be_retried_and_nothing_is_written_twice_wrongly() {
    let dir = TempDir::new("retry");
    let p = dir.project("p");
    let base = base_project(&p);
    let (b, _) = builder();
    // Crash on the third point of the first save, then retry with the injector spent.
    let inj = FaultInjector::fail_at(4);
    let mut ws = b.fault(inj.clone()).open(&p).unwrap();
    let id = first_counter(&ws);
    let e = ws.apply(&Add { id, delta: 1 }, &opts()).unwrap_err();
    assert_eq!(e.code(), "not-persisted");
    assert!(ws.dirty());
    ws.save().unwrap(); // the injector only fails once
    assert!(!ws.dirty());
    let st = ws.status();
    ws.close().unwrap();
    let (b, _) = builder();
    let ws = b.open(&p).unwrap();
    assert_eq!(ws.status().state, st.state);
    assert_ne!(ws.timeline().refs()["main"], base["main"]);
}

#[test]
fn failed_save_as_into_an_existing_empty_directory_empties_it_again() {
    let dir = TempDir::new("emptydir");
    let p = dir.project("empty");
    std::fs::create_dir_all(&p).unwrap();
    let (b, _) = builder();
    let mut ws = b.fault(FaultInjector::fail_at(1)).memory().unwrap();
    create(&mut ws, "a", 0);
    assert!(ws.save_as(&p, ProjectConfig::new("c")).is_err());
    assert!(p.is_dir());
    assert_eq!(std::fs::read_dir(&p).unwrap().count(), 0);
    ws.save_as(&p, ProjectConfig::new("c")).unwrap();
}
