//! Property: any sequence of ops, forks, checkouts, undo/redo and branch edits leaves a project
//! on disk that reopens to exactly the in-memory status, and the same sequence on a twin
//! workspace without a project gives identical hashes.

mod common;

use std::sync::{Mutex, OnceLock};

use common::*;
use proptest::prelude::*;
use sm_engine::{ProjectConfig, Workspace};
use sm_op_api::Rev;

#[derive(Clone, Debug)]
enum Act {
    Add(i64),
    Scale(u8),
    Fork(u8),
    Checkout(u8),
    Undo,
    Redo,
    Delete(u8),
    Rename(u8, u8),
    Detach(u8),
}

fn act() -> impl Strategy<Value = Act> {
    prop_oneof![
        4 => (-50i64..50).prop_map(Act::Add),
        1 => (1u8..4).prop_map(Act::Scale),
        2 => (0u8..6).prop_map(Act::Fork),
        2 => (0u8..6).prop_map(Act::Checkout),
        2 => Just(Act::Undo),
        1 => Just(Act::Redo),
        1 => (0u8..6).prop_map(Act::Delete),
        1 => (0u8..6, 0u8..6).prop_map(|(a, b)| Act::Rename(a, b)),
        1 => (0u8..12).prop_map(Act::Detach),
    ]
}

fn pick(ws: &Workspace, i: u8) -> Option<String> {
    let names: Vec<&String> = ws.timeline().refs().keys().collect();
    (!names.is_empty()).then(|| names[i as usize % names.len()].clone())
}

/// Run one action; the result is the error code if it was refused. Refusals are legitimate
/// (undo of the creating op, duplicate branch names); disk and twin must agree on them.
fn run(ws: &mut Workspace, id: sm_core::EntityId, a: &Act) -> Option<&'static str> {
    let name = |i: u8| format!("b{i}");
    let code = |r: Result<(), sm_engine::EngineError>| r.err().map(|e| e.code());
    match a {
        Act::Add(d) => code(ws.apply(&Add { id, delta: *d }, &opts()).map(|_| ())),
        Act::Scale(f) => code(
            ws.apply(
                &ScaleGain {
                    id,
                    factor: f64::from(*f),
                },
                &opts(),
            )
            .map(|_| ()),
        ),
        Act::Fork(i) => code(ws.fork(&name(*i), None)),
        Act::Checkout(i) => pick(ws, *i).and_then(|n| code(ws.checkout(Rev::Ref(n)))),
        Act::Undo => code(ws.undo()),
        Act::Redo => code(ws.redo()),
        Act::Delete(i) => pick(ws, *i).and_then(|n| code(ws.delete_branch(&n))),
        Act::Rename(i, j) => pick(ws, *i).and_then(|n| code(ws.rename_branch(&n, &name(*j)))),
        Act::Detach(i) => {
            let nodes: Vec<_> = ws.timeline().nodes().map(|(h, _)| h).collect();
            if nodes.is_empty() {
                None
            } else {
                code(ws.checkout(Rev::Node(nodes[*i as usize % nodes.len()])))
            }
        }
    }
}

static GUARD: OnceLock<Mutex<()>> = OnceLock::new();

proptest! {
    #![proptest_config(sm_testkit::strategies::config(
        std::env::var("PROPTEST_CASES").ok().and_then(|v| v.parse().ok()).unwrap_or(40),
    ))]

    #[test]
    fn reopen_equals_memory_and_twin(acts in proptest::collection::vec(act(), 1..30), reopen_at in 0usize..30) {
        let _g = GUARD.get_or_init(Mutex::default).lock().unwrap_or_else(|e| e.into_inner());
        let dir = TempDir::new("prop");
        let p = dir.project("p");
        let (b, _) = builder();
        let mut disk = b.create(&p, ProjectConfig::new("prop")).unwrap();
        let (b, _) = builder();
        let mut twin = b.memory().unwrap();
        let id = create(&mut disk, "a", 0);
        let id2 = create(&mut twin, "a", 0);
        prop_assert_eq!(id, id2);
        for (k, a) in acts.iter().enumerate() {
            if k == reopen_at {
                // Mid-sequence restart.
                let want = disk.status();
                disk.close().unwrap();
                let (b, _) = builder();
                disk = b.open(&p).unwrap();
                prop_assert_eq!(disk.status(), want);
            }
            let (rd, rt) = (run(&mut disk, id, a), run(&mut twin, id, a));
            prop_assert_eq!(rd, rt, "{:?}", a);
            prop_assert_eq!(disk.status().state, twin.status().state);
            prop_assert_eq!(&disk.status().branches, &twin.status().branches);
        }
        let want = disk.status();
        prop_assert!(!want.dirty);
        disk.close().unwrap();
        let (b, _) = builder();
        let mut re = b.open(&p).unwrap();
        prop_assert_eq!(re.status(), want);
        // Every branch tip replays to the state a twin checkout gives.
        for (n, _) in twin.timeline().refs().clone() {
            let a = re.state_at(&Rev::Ref(n.clone())).unwrap().hash();
            let b = twin.state_at(&Rev::Ref(n)).unwrap().hash();
            prop_assert_eq!(a, b);
        }
    }
}
