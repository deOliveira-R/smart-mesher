//! Determinism: same text, registry and state give the same records and the same final state, on every run and for
//! every thread count, including inputs large enough to take the parallel paths.

mod common;

use std::hash::Hasher;

use sm_lang::entity::PAR_THRESHOLD;
use sm_lang::{Interpreter, Options, ParseLimits, parse_many};
use sm_op_api::canonical_bytes;
use sm_testkit::determinism::{Fnv, check};

use common::*;

const N: u64 = 200_000;

const SCRIPT: &str = r#"
test.alloc_many(kind: "TopoFace", n: 200000)
model.create_brick(x: 1, y: 1, z: 1)
let a = all faces | Face#1..150000
let b = all faces & Face#100000..400000 by 3
let c = all faces - Face#[1, 5, 7..100000]
let d = (Face#1..199999 by 2 | Face#2..200000 by 2) - c
assert count(a) == 200006
assert count(b) == 33336
assert count(c) == 100010
assert count(d) == 99996
intent.set_size(ids: b, size: 0.5)
intent.set_size(ids: c - Face#150001..180000, size: 0.25)
mesh.run(targets: volumes in all bodies, scheme: "map")
"#;

/// Run the script on a pool of `workers` threads; hash every record and the final state.
fn run_hashed(workers: usize) -> u64 {
    let pool = rayon::ThreadPoolBuilder::new().num_threads(workers).build().unwrap();
    pool.install(|| {
        let engine = engine();
        let mut h = host(&engine);
        let rep = {
            let mut it = Interpreter::new(&mut h, Options::default());
            it.run_source("det.smesh", SCRIPT)
        };
        assert!(rep.ok(), "{:?}", rep.diagnostics.iter().map(|d| (d.code, &d.message)).collect::<Vec<_>>());
        let mut f = Fnv::default();
        for r in rep.records() {
            f.write(r.name.as_bytes());
            f.write(&r.version.to_le_bytes());
            f.write(&canonical_bytes(&r.params));
        }
        f.write(h.into_state().hash().to_string().as_bytes());
        f.finish()
    })
}

#[test]
fn large_entity_lists_lower_identically_on_every_run_and_thread_count() {
    assert!(N as usize > PAR_THRESHOLD, "the script must exercise the parallel paths");
    let h = check(2, 4, run_hashed).unwrap();
    assert_ne!(h, 0);
    // And against the sequential oracle: the same script on a single worker is the baseline `check` already compared.
    assert_eq!(h, run_hashed(1));
}

#[test]
fn parsing_many_sources_in_parallel_matches_sequential() {
    let sources: Vec<String> = (0..64).map(|i| format!("let a{i} = {i} * 2\nmodel.f(x: [{i}, Volume#{i}..{}], y: faces in Volume#{i})\nlet = {i}\n", i + 5)).collect();
    let refs: Vec<&str> = sources.iter().map(String::as_str).collect();
    let digest = |workers: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(workers).build().unwrap();
        pool.install(|| {
            let outs = parse_many(&refs, ParseLimits::default());
            format!("{:?}", outs.iter().map(|o| (&o.script, &o.diagnostics)).collect::<Vec<_>>())
        })
    };
    let base = digest(1);
    assert_eq!(base, digest(8));
    let seq: Vec<_> = refs.iter().enumerate().map(|(i, t)| sm_lang::parse_with(i as u32, t, ParseLimits::default())).collect();
    assert_eq!(base, format!("{:?}", seq.iter().map(|o| (&o.script, &o.diagnostics)).collect::<Vec<_>>()));
    check(2, 4, digest).unwrap();
}

#[test]
fn repeated_runs_of_a_small_script_are_identical() {
    let small = "let b = model.create_brick(x: 1, y: 2, z: 3)\nintent.set_size(ids: faces in b.volume, size: 0.5)\nbatch { model.pick(b.body) }\n";
    let hash = |_: usize| {
        let engine = engine();
        let mut h = host(&engine);
        let rep = Interpreter::new(&mut h, Options::default()).run_source("s", small);
        assert!(rep.ok(), "{:?}", rep.diagnostics);
        let mut f = Fnv::default();
        for r in rep.records() {
            f.write(&canonical_bytes(&r.params));
        }
        f.finish()
    };
    check(3, 4, hash).unwrap();
}
