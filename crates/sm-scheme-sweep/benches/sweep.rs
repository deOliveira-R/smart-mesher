#![allow(missing_docs)]
//! `Driver::generate` of 1-to-1 sweeps on four rayon threads (the bulk mesh commit is not
//! measured here: it is W3b work). Run `cargo bench -p sm-scheme-sweep`.

#[path = "../tests/common/mod.rs"]
mod common;

use criterion::{Criterion, criterion_group, criterion_main};
use sm_model_api::Model;
use sm_scheme_api::{
    Driver, Firm, Firmness, IntervalRequest, MeshInput, MeshIntent, ReferenceCornerPicker,
    ReferenceIntervalSolver, SchemeChoice, Sequential, WholeFaces,
};

fn bench_generate(c: &mut Criterion) {
    let mut group = c.benchmark_group("generate");
    group.sample_size(10);
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(4)
        .build()
        .expect("pool");
    let reg = common::registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let model = Model::new();

    // Brick 100 x 100 x 100: translation fast path, 10^6 hexes.
    let (topo, ids, _) = sm_scheme_api::stubs::brick_setup([100.0, 100.0, 100.0]);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    for e in ids.edges {
        intent.set_intervals(e, Firm::hard(IntervalRequest::Count(100)));
    }
    intent.set_scheme(
        ids.volume,
        SchemeChoice::with_params(
            "sweep",
            serde_json::json!({ "source": [ids.faces[4]], "target": [ids.faces[5]] }),
            Firmness::Hard,
        ),
    );
    let plan = driver.plan(&input, &intent).expect("plan");
    group.bench_function("brick_100x100x100_4_threads", |b| {
        b.iter(|| {
            pool.install(|| driver.generate(&input, &plan, &Sequential))
                .expect("generate")
        });
    });

    // Twisted prism 40 x 40 x 40: affine fit plus Shepard residual on every layer.
    let fx = sm_testkit::topo::twisted_prism(2.0, 3.0, 1.0).expect("twisted");
    let sel = [fx.volume];
    let input = MeshInput::new(&fx.topo.complex, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    let mut edges = sm_topo_api::TopologyView::face_edges(&fx.topo.complex, fx.source).to_vec();
    edges.extend(fx.columns.iter().copied());
    for e in edges {
        intent.set_intervals(e, Firm::hard(IntervalRequest::Count(40)));
    }
    intent.set_scheme(
        fx.volume,
        SchemeChoice::with_params(
            "sweep",
            serde_json::json!({ "source": [fx.source], "target": [fx.target] }),
            Firmness::Hard,
        ),
    );
    let plan = driver.plan(&input, &intent).expect("plan");
    group.bench_function("twisted_40x40x40_4_threads", |b| {
        b.iter(|| {
            pool.install(|| driver.generate(&input, &plan, &Sequential))
                .expect("generate")
        });
    });
    group.finish();
}

criterion_group!(benches, bench_generate);
criterion_main!(benches);
