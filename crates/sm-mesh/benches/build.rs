//! Build-time benchmarks: structured hex blocks from 32^3 up to 10^6 cells.
//!
//! * `build/bulk`: the parallel bulk path (`MeshBuilder::add_cells_bulk` + `finish`).
//! * `build/incremental`: `add_cell_from_vertices` per cell + `finish` (the generic mesher path).
//! * `build/reference`: the `sm-mesh-api` reference builder, for scale (small sizes only).
//! * `query/*`: adjacency tables and edge table on a finished mesh.
//!
//! `cargo bench -p sm-mesh` runs everything; the 10^6 case is `build/bulk/100^3` and
//! `build/incremental/100^3`. Thread scaling: `build/bulk_threads/{1,2,4}` on 64^3.

use criterion::{BenchmarkId, Criterion};
use sm_mesh::MeshBuilder;
use sm_mesh::generate::{hex_block, hex_block_bulk};
use sm_mesh_api::reference::{ReferenceBuilder, hex_block as ref_hex_block};
use sm_mesh_api::{MeshEdit, MeshRead, SerialAlloc};
use std::time::Duration;

fn builds(c: &mut Criterion) {
    let mut g = c.benchmark_group("build");
    g.sample_size(10).measurement_time(Duration::from_secs(20)).warm_up_time(Duration::from_secs(2));
    for n in [32u32, 64, 100] {
        let id = format!("{n}^3");
        g.bench_function(BenchmarkId::new("bulk", &id), |b| {
            b.iter(|| hex_block_bulk([n; 3], [1.0; 3], &mut SerialAlloc::new()).map(|f| f.mesh.cell_count_hint()));
        });
        g.bench_function(BenchmarkId::new("incremental", &id), |b| {
            b.iter(|| {
                hex_block(MeshBuilder::new(), [n; 3], [1.0; 3], false, &mut SerialAlloc::new())
                    .map(|f| f.mesh.cell_count_hint())
            });
        });
        if n <= 64 {
            g.bench_function(BenchmarkId::new("reference", &id), |b| {
                b.iter(|| ref_hex_block([n; 3], [1.0; 3], &mut SerialAlloc::new()).map(|m| m.cell_count_hint()));
            });
            g.bench_function(BenchmarkId::new("reference_generic_builder", &id), |b| {
                b.iter(|| {
                    hex_block(ReferenceBuilder::new(), [n; 3], [1.0; 3], false, &mut SerialAlloc::new()).map(|f| f.mesh.cell_count_hint())
                });
            });
        }
    }
    g.finish();

    let mut g = c.benchmark_group("build/bulk_threads");
    g.sample_size(10).measurement_time(Duration::from_secs(10));
    for t in [1usize, 2, 4] {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(t).build().expect("pool");
        g.bench_function(BenchmarkId::from_parameter(t), |b| {
            b.iter(|| pool.install(|| hex_block_bulk([64; 3], [1.0; 3], &mut SerialAlloc::new()).map(|f| f.mesh.cell_count_hint())));
        });
    }
    g.finish();
}

fn queries(c: &mut Criterion) {
    let mut g = c.benchmark_group("query");
    g.sample_size(10).measurement_time(Duration::from_secs(10));
    let mesh = hex_block_bulk([64; 3], [1.0; 3], &mut SerialAlloc::new()).expect("block").mesh;
    g.bench_function("vertex_cells/64^3", |b| {
        b.iter(|| mesh.clone().vertex_cells().indices().len());
    });
    g.bench_function("cell_neighbors/64^3", |b| {
        b.iter(|| mesh.clone().cell_neighbors().indices().len());
    });
    let classified = hex_block(MeshBuilder::new(), [32; 3], [1.0; 3], true, &mut SerialAlloc::new()).expect("block").mesh;
    g.bench_function("edge_table_derive/32^3", |b| {
        b.iter(|| {
            let mut m = classified.clone();
            let mut a = SerialAlloc::new();
            // Re-derive from scratch: drop the existing table, give the edge classes back.
            let explicit: Vec<_> = m
                .edges()
                .map(|t| t.vertices.iter().zip(&t.class).map(|(v, c)| (v[0], v[1], *c)).collect())
                .unwrap_or_default();
            m.drop_edges().ok();
            m.ensure_edges(&mut a, &explicit).map(|()| m.edges().map(|t| t.len()))
        });
    });
    g.finish();
}

/// Keeps the optimizer from discarding the mesh.
trait Hint {
    fn cell_count_hint(&self) -> usize;
}
impl<M: sm_mesh_api::MeshRead> Hint for M {
    fn cell_count_hint(&self) -> usize {
        self.cell_count()
    }
}

mod harness {
    #![allow(missing_docs)]
    use super::{builds, queries};
    use criterion::criterion_group;
    criterion_group!(benches, builds, queries);
}
criterion::criterion_main!(harness::benches);
