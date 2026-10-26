//! Informational benchmark: dualize a Kuhn tet block (never a gate).
//!
//! `cargo bench -p sm-poly` times `tet_block([20,20,20])` (48k tets) and `tet_block([40,40,40])`
//! (384k tets) in both face modes with the production builder.

use criterion::Criterion;
use sm_mesh::MeshBuilder;
use sm_mesh_api::SerialAlloc;
use sm_poly::{FaceMode, PolyOptions, VertexClassResolver, dualize};
use sm_testkit::meshes;

fn bench(c: &mut Criterion) {
    let mut g = c.benchmark_group("dualize");
    g.sample_size(10);
    for n in [20u32, 40] {
        let f = meshes::tet_block([n; 3], [1.0; 3]).expect("fixture");
        for (name, mode) in [("planar", FaceMode::Planar), ("keep", FaceMode::Keep)] {
            let opts = PolyOptions::new().with_faces(mode);
            g.bench_function(format!("tet_block_{n}/{name}"), |b| {
                b.iter(|| {
                    let mut alloc = SerialAlloc::new();
                    dualize(&f.mesh, &opts, &VertexClassResolver, MeshBuilder::new(), &mut alloc).expect("dualize")
                });
            });
        }
    }
    g.finish();
}

#[allow(missing_docs)]
mod harness {
    use super::bench;
    criterion::criterion_group!(benches, bench);
}
criterion::criterion_main!(harness::benches);
