//! Determinism (CLAUDE.md): two runs, the sequential vs threaded executor, and concurrent runs
//! on four `std` threads give identical committed meshes and identical sm-mesh-api hashes. API
//! crates use no rayon (W0c decision 8); `Threads` stands in for `RAYON_NUM_THREADS` > 1.

mod common;
use common::*;
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{MeshBuild, SerialAlloc, geometry_hash, topology_hash};
use sm_scheme_api::*;

fn run<E: Executor>(exec: &E) -> (u64, u64, u64) {
    let (t, ids) = brick_complex();
    let shape = TopoShape::new(&t);
    let reg = registry();
    let intent = sweep_intent(&ids);
    let out = driver(&reg, &shape, &intent).run(&[key(ids.volume)], exec).expect("mesh");
    let mut b = ReferenceBuilder::new();
    emit(&out.mesh, &shape, &mut b).expect("emit");
    let m = b.finish(&mut SerialAlloc::new()).expect("finish").mesh;
    (out.mesh.fingerprint(), topology_hash(&m), geometry_hash(&m))
}

#[test]
fn repeated_runs_are_identical() {
    assert_eq!(run(&Sequential), run(&Sequential));
}

#[test]
fn thread_count_does_not_change_output() {
    let one = run(&Sequential);
    assert_eq!(one, run(&Threads(1)));
    assert_eq!(one, run(&Threads(4)));
    assert_eq!(one, run(&Threads(64)));
}

#[test]
fn concurrent_runs_agree() {
    let one = run(&Sequential);
    let handles: Vec<_> = (0..4).map(|_| std::thread::spawn(|| run(&Threads(3)))).collect();
    for h in handles {
        assert_eq!(h.join().expect("thread"), one);
    }
}
