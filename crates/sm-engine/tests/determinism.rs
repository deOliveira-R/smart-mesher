//! Determinism: the same session gives bit-identical state hashes and bit-identical files for
//! repeated runs, 1 vs N rayon threads, and concurrent runs; error reporting does not depend on
//! scheduling either.

mod common;

use std::fs;

use common::*;
use sm_engine::{Durability, ProjectConfig};
use sm_op_api::{Hash256, Rev};
use sm_testkit::determinism;

fn in_pool<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new()
        .num_threads(threads)
        .build()
        .expect("pool")
        .install(f)
}

/// 200 ops over three branches, saved in bulk (above the parallel-write threshold), reopened
/// (above the parallel-read threshold). Returns every state hash seen and the hash of the files.
fn scenario(threads: usize) -> (Vec<Hash256>, Hash256) {
    in_pool(threads, || {
        let dir = TempDir::new("det");
        let p = dir.project("p");
        let (b, _) = builder();
        let mut ws = b
            .autosave(false)
            .create(&p, ProjectConfig::new("det"))
            .unwrap();
        let id = create(&mut ws, "a", 0);
        let mut hashes = Vec::new();
        for k in 0..100 {
            hashes.push(ws.apply(&Add { id, delta: k }, &opts()).unwrap().state);
        }
        let n50 = ws.log(None, 100).unwrap()[50].node;
        ws.fork("side", Some(n50)).unwrap();
        for k in 0..60 {
            let r = if k % 7 == 0 {
                ws.apply(&ScaleGain { id, factor: 1.5 }, &opts()).unwrap()
            } else {
                ws.apply(&Add { id, delta: -k }, &opts()).unwrap()
            };
            hashes.push(r.state);
        }
        ws.checkout(Rev::Ref("main".into())).unwrap();
        ws.fork("third", None).unwrap();
        for k in 0..40 {
            hashes.push(ws.apply(&Add { id, delta: 2 * k }, &opts()).unwrap().state);
        }
        ws.save().unwrap();
        ws.close().unwrap();

        let (b, _) = builder();
        let mut ws = b.open(&p).unwrap();
        for name in ["main", "side", "third"] {
            hashes.push(ws.state_at(&Rev::Ref(name.into())).unwrap().hash());
        }
        (hashes, tree_hash(&p))
    })
}

#[test]
fn sessions_are_bit_identical_across_runs_and_thread_counts() {
    let (hashes, files) = determinism::check(2, 4, scenario).expect("deterministic");
    assert_eq!(hashes.len(), 100 + 60 + 40 + 3);
    assert_ne!(files, Hash256::default());
}

#[test]
fn full_durability_writes_the_same_bytes() {
    let run = |dur: Durability| {
        let dir = TempDir::new("dur");
        let p = dir.project("p");
        let (b, _) = builder();
        let mut ws = b
            .durability(dur)
            .create(&p, ProjectConfig::new("d"))
            .unwrap();
        let id = create(&mut ws, "a", 0);
        add(&mut ws, id, 1);
        ws.close().unwrap();
        tree_hash(&p)
    };
    assert_eq!(run(Durability::Fast), run(Durability::Full));
}

#[test]
fn read_errors_do_not_depend_on_scheduling() {
    let dir = TempDir::new("errs");
    let p = dir.project("p");
    let (b, _) = builder();
    let mut ws = b.create(&p, ProjectConfig::new("e")).unwrap();
    let id = create(&mut ws, "a", 0);
    for k in 0..150 {
        add(&mut ws, id, k);
    }
    ws.close().unwrap();
    // Make several node files unreadable as UTF-8.
    let mut names: Vec<_> = fs::read_dir(p.join("timeline/ops"))
        .unwrap()
        .map(|e| e.unwrap().path())
        .collect();
    names.sort();
    for f in [&names[10], &names[90], &names[140]] {
        fs::write(f, [0xff, 0xfe, 0xfd]).unwrap();
    }
    let msg = |threads: usize| {
        in_pool(threads, || {
            let (b, _) = builder();
            b.open(&p).err().expect("must fail").to_string()
        })
    };
    let first = msg(1);
    assert!(
        first.contains(names[10].file_name().unwrap().to_str().unwrap()),
        "{first}"
    );
    for _ in 0..3 {
        assert_eq!(msg(4), first);
        assert_eq!(msg(1), first);
    }
}
