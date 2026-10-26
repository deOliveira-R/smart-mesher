//! Acceptance 14: record-only timing of a 100 x 100 source x 100 layers sweep (10^6 hexes).
//! Run: `cargo test -p sm-scheme-sweep --all-features --release --test perf -- --ignored --nocapture`.

mod common;

use common::registry;
use sm_mesh_api::SerialAlloc;
use sm_mesh_api::reference::ReferenceBuilder;
use sm_model_api::Model;
use sm_scheme_api::{
    Driver, Firm, Firmness, IntervalRequest, MeshInput, MeshIntent, ReferenceCornerPicker,
    ReferenceIntervalSolver, SchemeChoice, Sequential, WholeFaces,
};
use std::time::Instant;

fn run(
    label: &str,
    build: impl Fn() -> (
        sm_topo_api::CellComplex,
        sm_core::EntityId,
        sm_core::EntityId,
        sm_core::EntityId,
        Vec<sm_core::EntityId>,
    ),
) {
    let (topo, vol, src, tgt, edges) = build();
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let model = Model::new();
    let sel = [vol];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    for (i, e) in edges.iter().enumerate() {
        intent.set_intervals(*e, Firm::hard(IntervalRequest::Count(100)));
        let _ = i;
    }
    intent.set_scheme(
        vol,
        SchemeChoice::with_params(
            "sweep",
            serde_json::json!({ "source": [src], "target": [tgt] }),
            Firmness::Hard,
        ),
    );
    let t0 = Instant::now();
    let plan = driver.plan(&input, &intent).expect("plan");
    let t_plan = t0.elapsed();
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(4)
        .build()
        .expect("pool");
    let t1 = Instant::now();
    let ps = pool
        .install(|| driver.generate(&input, &plan, &Sequential))
        .expect("generate");
    let t_gen = t1.elapsed();
    let t2 = Instant::now();
    let out = driver
        .commit(
            &input,
            &ps,
            ReferenceBuilder::new(),
            &mut SerialAlloc::new(),
        )
        .expect("commit");
    let t_commit = t2.elapsed();
    let cells = sm_mesh_api::MeshRead::count(&out.mesh, sm_mesh_api::MeshEntityKind::Cell);
    let share = t_commit.as_secs_f64() / (t_commit + t_gen).as_secs_f64();
    println!(
        "PERF {label}: cells {cells}, plan {:.3}s, generate (4 threads) {:.3}s, commit {:.3}s, commit share {:.0}%",
        t_plan.as_secs_f64(),
        t_gen.as_secs_f64(),
        t_commit.as_secs_f64(),
        100.0 * share
    );
}

#[test]
#[ignore = "record-only performance run"]
fn brick_100_cubed() {
    run("brick 100x100x100 (translation path)", || {
        let (topo, ids, _) = sm_scheme_api::stubs::brick_setup([100.0, 100.0, 100.0]);
        let edges = ids.edges.to_vec();
        (topo, ids.volume, ids.faces[4], ids.faces[5], edges)
    });
}

#[test]
#[ignore = "record-only performance run"]
fn twisted_100_cubed() {
    run("twisted prism 100x100x100 (affine + Shepard)", || {
        let fx = sm_testkit::topo::twisted_prism(2.0, 3.0, 1.0).expect("twisted");
        let mut edges = sm_topo_api::TopologyView::face_edges(&fx.topo.complex, fx.source).to_vec();
        edges.extend(fx.columns.iter().copied());
        (fx.topo.complex, fx.volume, fx.source, fx.target, edges)
    });
}
