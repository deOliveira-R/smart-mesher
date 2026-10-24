//! Commit throughput at 10^6 cells (W2 plan section 5, risk 3).
//!
//! Meshes an `n`^3 brick (n = 100 gives 10^6 hexahedra and 1 030 301 vertices) with the stub
//! schemes (`sweep` + `map` + `equal` from `sm_scheme_api::stubs`), and times the driver phases
//! separately: plan, generate (rayon executor), commit into `sm_mesh::MeshBuilder`; then the whole
//! driver; then the whole `mesh.run` op (driver + id adoption + layer insert + state hash).
//! The stub schemes are reference implementations: the generate numbers bound, they do not
//! predict, the production schemes.
#![allow(missing_docs)]

use std::sync::Arc;

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, Point3};
use sm_layer_api::{CoreLayers, LayerResultExt, commit_topology, topo_txn};
use sm_mesh::MeshBuilder;
use sm_mesher::{MeshPlugin, MesherConfig, RayonExec};
use sm_op_api::{
    ApplyCtx, DerivedCache, Engine, Meta, OpError, OpRecord, QuietControl, Registry, Services,
    Session, TypedOp,
};
use sm_scheme_api::{
    Driver, Firm, Firmness, IntervalRequest, MeshInput, MeshIntent, ReferenceCornerPicker,
    ReferenceIntervalSolver, SchemeChoice, WholeFaces, stubs,
};
use sm_mesh_api::SerialAlloc;
use sm_topo_api::fixtures::brick;

fn sizes() -> Vec<usize> {
    match std::env::var("SM_BENCH_N") {
        Ok(v) => v.split(',').filter_map(|s| s.trim().parse().ok()).collect(),
        Err(_) => vec![50, 100],
    }
}

fn sweep_intent(ids: &sm_topo_api::fixtures::BrickIds) -> MeshIntent {
    let mut intent = MeshIntent::default();
    intent.set_intervals(ids.volume, Firm::hard(IntervalRequest::Size(1.0)));
    intent.set_scheme(
        ids.volume,
        SchemeChoice::with_params(
            "sweep",
            serde_json::json!({ "source": [ids.faces[4]], "target": [ids.faces[5]] }),
            Firmness::Hard,
        ),
    );
    intent
}

fn driver_phases(c: &mut Criterion) {
    let mut g = c.benchmark_group("driver");
    g.sample_size(10);
    for n in sizes() {
        let (topo, ids, model) = stubs::brick_setup([n as f64; 3]);
        let reg = stubs::registry();
        let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
        let driver = Driver::new(&reg, &solver, &picker);
        let sel = [ids.volume];
        let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
        let intent = sweep_intent(&ids);
        let cells = (n * n * n) as u64;
        g.throughput(Throughput::Elements(cells));

        g.bench_with_input(BenchmarkId::new("plan", n), &n, |b, _| {
            b.iter(|| driver.plan(&input, &intent).expect("plan"));
        });
        let plan = driver.plan(&input, &intent).expect("plan");
        g.bench_with_input(BenchmarkId::new("generate", n), &n, |b, _| {
            b.iter(|| driver.generate(&input, &plan, &RayonExec).expect("generate"));
        });
        let patches = driver.generate(&input, &plan, &RayonExec).expect("generate");
        g.bench_with_input(BenchmarkId::new("commit", n), &n, |b, _| {
            b.iter(|| {
                let mut alloc = SerialAlloc::new();
                driver
                    .commit(&input, &patches, MeshBuilder::new(), &mut alloc)
                    .expect("commit")
            });
        });
        g.bench_with_input(BenchmarkId::new("plan+generate+commit", n), &n, |b, _| {
            b.iter(|| {
                let mut alloc = SerialAlloc::new();
                driver
                    .mesh(&input, &intent, &RayonExec, MeshBuilder::new(), &mut alloc)
                    .expect("mesh")
            });
        });
    }
    g.finish();
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct CreateBrick {
    size: f64,
}

impl TypedOp for CreateBrick {
    const NAME: &'static str = "bench.create_brick";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let (_, txn) = topo_txn(ctx, |topo, _| {
            brick(topo, Point3::new(0.0, 0.0, 0.0), [self.size; 3]).or_op()
        })?;
        commit_topology(ctx, &txn, &[])
    }
}

fn id(s: &str) -> EntityId {
    s.parse().expect("id")
}

fn mesh_run_op(c: &mut Criterion) {
    let mut g = c.benchmark_group("op");
    g.sample_size(10);
    for n in sizes() {
        let mut reg = Registry::new();
        reg.install(&CoreLayers::default()).expect("core");
        reg.install(&MeshPlugin).expect("mesher");
        reg.register::<CreateBrick>().expect("brick");
        let eng = Engine::new(
            Arc::new(reg),
            Services::new().with_versioned("mesher", 1, MesherConfig::new(stubs::registry())),
            Arc::new(DerivedCache::disabled()),
        )
        .expect("engine");
        let mut s = Session::new(eng);
        let q = QuietControl::new();
        let ctl = q.control();
        let apply = |s: &mut Session, name: &str, p: serde_json::Value| {
            let rec = OpRecord::new(name, 1, p).expect("record");
            let op = s.engine().registry().decode(&rec).expect("decode");
            s.apply(op.as_ref(), Meta::new("bench", "", ""), &ctl).expect(name)
        };
        apply(&mut s, "bench.create_brick", serde_json::json!({"size": n as f64}));
        apply(&mut s, "mesh.size", serde_json::json!({"targets": ["TopoVolume#1"], "size": 1.0, "firmness": "hard"}));
        apply(
            &mut s,
            "mesh.scheme",
            serde_json::json!({"targets": ["TopoVolume#1"], "scheme": "sweep",
                "params": {"source": [id("TopoFace#5").to_string()], "target": [id("TopoFace#6").to_string()]}}),
        );
        g.throughput(Throughput::Elements((n * n * n) as u64));
        g.bench_with_input(BenchmarkId::new("mesh.run+delete", n), &n, |b, _| {
            b.iter(|| {
                apply(&mut s, "mesh.run", serde_json::json!({"targets": ["TopoVolume#1"]}));
                apply(&mut s, "mesh.delete", serde_json::json!({"targets": ["TopoVolume#1"]}));
            });
        });
    }
    g.finish();
}

criterion_group!(benches, driver_phases, mesh_run_op);
criterion_main!(benches);
