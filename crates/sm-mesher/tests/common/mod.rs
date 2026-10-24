//! Shared test harness: a tiny model plugin (brick, webcut, delete) and an engine with
//! `CoreLayers`, `MeshPlugin` and the stub schemes.
#![allow(dead_code)]

use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind, Point3};
use sm_layer_api::{CoreLayers, LayerResultExt, TopoHost, commit_topology, topo_txn};
use sm_mesher::{MesherConfig, MeshPlugin};
use sm_op_api::{
    ApplyCtx, DerivedCache, Delta, Engine, Hash256, Meta, Op, OpError, QuietControl, Registry,
    Services, Session, TypedOp,
};
use sm_scheme_api::{SchemeRegistry, stubs};
use sm_topo_api::CellComplex;
use sm_topo_api::fixtures::brick;

/// `model.create_brick`-shaped test op.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CreateBrick {
    pub origin: [f64; 3],
    pub size: [f64; 3],
}

impl TypedOp for CreateBrick {
    const NAME: &'static str = "test.create_brick";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let [x, y, z] = self.origin;
        let (ids, txn) = topo_txn(ctx, |topo, _| {
            brick(topo, Point3::new(x, y, z), self.size).or_op()
        })?;
        commit_topology(
            ctx,
            &txn,
            &[("volume", &[ids.volume]), ("faces", &ids.faces), ("edges", &ids.edges)],
        )
    }
}

/// Cuts the first brick (ids 1..) with a plane: two volumes sharing a cap face.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Webcut {
    pub frac: f64,
}

impl TypedOp for Webcut {
    const NAME: &'static str = "test.webcut";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let ids = |k, n: u64| -> Vec<EntityId> { (1..=n).map(|s| id(k, s)).collect() };
        let b = sm_topo_api::fixtures::BrickIds {
            min: Point3::new(0.0, 0.0, 0.0),
            size: [1.0, 2.0, 3.0],
            vertices: ids(EntityKind::TopoVertex, 8).try_into().expect("8"),
            edges: ids(EntityKind::TopoEdge, 12).try_into().expect("12"),
            faces: ids(EntityKind::TopoFace, 6).try_into().expect("6"),
            volume: id(EntityKind::TopoVolume, 1),
            body: id(EntityKind::TopoBody, 1),
        };
        let (cut, txn) = topo_txn(ctx, |topo, _| {
            sm_topo_api::fixtures::webcut_brick(topo, &b, self.frac).or_op()
        })?;
        commit_topology(ctx, &txn, &[("volumes", &cut.volumes), ("cap", &[cut.cap])])
    }
}

/// Deletes a volume and everything only it uses.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct DeleteVolume {
    pub volume: EntityId,
}

impl TypedOp for DeleteVolume {
    const NAME: &'static str = "test.delete_volume";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        ctx.resolve_kind(self.volume, EntityKind::TopoVolume)?;
        let (_, txn) = topo_txn(ctx, |topo, _| topo.kill_cascade(self.volume).or_op())?;
        commit_topology(ctx, &txn, &[])
    }
}

pub fn id(kind: EntityKind, serial: u64) -> EntityId {
    EntityId::new(kind, serial).expect("id")
}

pub fn vol(n: u64) -> EntityId {
    id(EntityKind::TopoVolume, n)
}

pub fn face(n: u64) -> EntityId {
    id(EntityKind::TopoFace, n)
}

pub fn edge(n: u64) -> EntityId {
    id(EntityKind::TopoEdge, n)
}

pub fn cell_complex() -> Box<dyn TopoHost> {
    Box::new(CellComplex::new())
}

/// An engine over `schemes` with the given cache.
pub fn engine_with(schemes: SchemeRegistry, cache: DerivedCache) -> Engine {
    engine_cfg(MesherConfig::new(schemes), cache)
}

pub fn engine_cfg(cfg: MesherConfig, cache: DerivedCache) -> Engine {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::new(cell_complex)).expect("core layers");
    reg.install(&MeshPlugin).expect("mesher");
    reg.register::<CreateBrick>().expect("brick");
    reg.register::<Webcut>().expect("webcut");
    reg.register::<DeleteVolume>().expect("delete");
    Engine::new(
        Arc::new(reg),
        Services::new().with_versioned("mesher", 1, cfg),
        Arc::new(cache),
    )
    .expect("engine")
}

pub fn engine() -> Engine {
    engine_with(stubs::registry(), DerivedCache::default())
}

pub fn session() -> Session {
    Session::new(engine())
}

pub fn meta() -> Meta {
    Meta::new("test", "", "")
}

pub fn run(s: &mut Session, op: &dyn Op) -> Result<(Hash256, Delta), sm_op_api::TimelineError> {
    let q = QuietControl::new();
    s.apply(op, meta(), &q.control())
}

/// Apply a JSON op through the registry (as a front end would).
pub fn run_json(
    s: &mut Session,
    name: &str,
    params: serde_json::Value,
) -> Result<(Hash256, Delta), sm_op_api::TimelineError> {
    let rec = sm_op_api::OpRecord::new(name, 1, params).expect("record");
    let op = s.engine().registry().decode(&rec).expect("decode");
    run(s, op.as_ref())
}

pub fn brick_op(size: [f64; 3]) -> CreateBrick {
    CreateBrick {
        origin: [0.0; 3],
        size,
    }
}

/// A session with a brick of `size`, the sweep scheme chosen on its volume (z source/target) and
/// size `h` requested.
pub fn brick_session(size: [f64; 3], h: f64) -> Session {
    let mut s = session();
    run(&mut s, &brick_op(size)).expect("brick");
    run_json(
        &mut s,
        "mesh.size",
        serde_json::json!({"targets": ["TopoVolume#1"], "size": h}),
    )
    .expect("size");
    run_json(
        &mut s,
        "mesh.scheme",
        serde_json::json!({
            "targets": ["TopoVolume#1"], "scheme": "sweep",
            "params": {"source": ["TopoFace#5"], "target": ["TopoFace#6"]}
        }),
    )
    .expect("scheme");
    s
}

pub fn mesh_run(s: &mut Session) -> Result<(Hash256, Delta), sm_op_api::TimelineError> {
    run_json(s, "mesh.run", serde_json::json!({"targets": ["TopoVolume#1"]}))
}
