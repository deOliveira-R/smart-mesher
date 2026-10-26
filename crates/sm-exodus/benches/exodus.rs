//! Export throughput: a 1e6-hex block to `io::sink` (record-only, not a gate: decision 12).

// criterion_group! generates an undocumented public function.
#![allow(missing_docs)]

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use sm_exodus::{ExodusOptions, write_exodus};
use sm_mesh_api::SerialAlloc;
use sm_mesh_api::reference::hex_block;
use std::hint::black_box;

fn export(c: &mut Criterion) {
    let mut g = c.benchmark_group("exodus_write");
    g.sample_size(10);
    for (name, n) in [("64k_hex", 40u32), ("1e6_hex", 100)] {
        let m = hex_block([n; 3], [1.0; 3], &mut SerialAlloc::new()).expect("block");
        let opts = ExodusOptions::default();
        let bytes = write_exodus(&m, &opts, &mut std::io::sink()).expect("write").bytes;
        g.throughput(Throughput::Bytes(bytes));
        g.bench_function(name, |b| b.iter(|| black_box(write_exodus(&m, &opts, &mut std::io::sink()).expect("write"))));
    }
    g.finish();
}

criterion_group!(benches, export);
criterion_main!(benches);
