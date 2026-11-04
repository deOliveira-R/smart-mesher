//! Record-only timing of `mesh.run` and `mesh.delete` at 10^6 cells (no wall-clock assertions).
//! Run: `cargo test --release -p sm-mesher --test bulk_perf -- --ignored --nocapture`.

mod common;

use std::time::Instant;

use common::*;
use serde_json::json;

#[test]
#[ignore = "perf record: release mode, ~10 s"]
fn mesh_run_and_delete_at_a_million_cells() {
    let n: f64 = std::env::var("SM_BULK_N")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(100.0);
    let mut s = brick_session([n; 3], 1.0);
    let t = Instant::now();
    let (h_run, _) = mesh_run(&mut s).expect("mesh.run");
    let run = t.elapsed();
    let t = Instant::now();
    run_json(&mut s, "mesh.delete", json!({"targets": ["TopoVolume#1"]})).expect("mesh.delete");
    let del = t.elapsed();
    println!(
        "n={n} cells={} mesh.run {run:?} mesh.delete {del:?} total {:?} state hash after run {h_run:?}",
        (n * n * n) as u64,
        run + del
    );
}
