//! Export throughput on a 40x40x40 hex block (64,000 cells).

// criterion_group! generates an undocumented public function.
#![allow(missing_docs)]

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use sm_mesh_api::SerialAlloc;
use sm_mesh_api::reference::hex_block;
use sm_meshio::{SmshFile, SmshWriter, VtuEncoding, VtuOptions, vtu_bytes};
use std::hint::black_box;

fn export(c: &mut Criterion) {
    let m = hex_block([40, 40, 40], [1.0; 3], &mut SerialAlloc::new()).expect("block");
    let smsh = SmshWriter::new(&m).to_bytes().expect("smsh");
    let mut g = c.benchmark_group("export_64k_hex");
    g.throughput(Throughput::Bytes(smsh.len() as u64));
    g.sample_size(20);
    g.bench_function("smsh_write", |b| b.iter(|| black_box(SmshWriter::new(&m).to_bytes().expect("smsh"))));
    g.bench_function("smsh_parse_verify_decode", |b| {
        b.iter(|| {
            let f = SmshFile::parse(&smsh).expect("parse");
            f.verify_checksums().expect("crc");
            black_box(f.decode().expect("decode"))
        })
    });
    g.bench_function("vtu_raw", |b| b.iter(|| black_box(vtu_bytes(&m, &VtuOptions::with_encoding(VtuEncoding::AppendedRaw)).expect("vtu"))));
    g.bench_function("vtu_base64", |b| b.iter(|| black_box(vtu_bytes(&m, &VtuOptions::with_encoding(VtuEncoding::AppendedBase64)).expect("vtu"))));
    g.finish();
}

criterion_group!(benches, export);
criterion_main!(benches);
