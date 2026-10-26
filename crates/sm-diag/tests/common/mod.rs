//! Shared rig: test ops that insert prepared meshes and bricks, and a session over `CoreLayers`,
//! `MeshPlugin` and `DiagPlugin`.
#![allow(dead_code, missing_docs)]

use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sm_core::{EntityId, EntityKind, Point3};
use sm_diag::{DiagPlugin, OpNames};
use sm_layer_api::{CoreLayers, LayerResultExt, MeshEntry, MeshLayer, ModelLayer, commit_topology, insert_mesh, topo_txn};
use sm_mesh_api::reference::{ReferenceMesh, hex_block};
use sm_mesh_api::{CellShape, MeshEdit, PointIx, SerialAlloc};
use sm_mesher::{MeshPlugin, MesherConfig};
use sm_op_api::{
    ApplyCtx, Delta, DerivedCache, Engine, Hash256, Meta, Op, OpError, OpRecord, QuietControl, Registry, Services, Session, State, TimelineError, TypedOp,
};
use sm_scheme_api::stubs;
use sm_testkit::meshes::{CellSpec, build_mesh};
use sm_topo_api::fixtures::brick;

/// A unit-ish brick `size` at the origin: gives the model entities (TopoVolume#1, faces 1..6,
/// edges 1..12, vertices 1..8) that the block meshes below are classified on.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CreateBrick {
    pub size: [f64; 3],
}

impl TypedOp for CreateBrick {
    const NAME: &'static str = "test.create_brick";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let (ids, txn) = topo_txn(ctx, |topo, _| brick(topo, Point3::new(0.0, 0.0, 0.0), self.size).or_op())?;
        commit_topology(ctx, &txn, &[("volume", &[ids.volume]), ("faces", &ids.faces), ("edges", &ids.edges)])
    }
}

/// Insert a `hex_block` of `n` cells per axis with spacing `h`; vertices listed in `moves`
/// (`(vertex index, new point)`) are moved. Role `mesh`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct InsertBlock {
    pub n: [u32; 3],
    pub h: [f64; 3],
    #[serde(default)]
    pub moves: Vec<(usize, [f64; 3])>,
}

pub fn block(n: [u32; 3], h: [f64; 3], moves: &[(usize, [f64; 3])], alloc: &mut SerialAlloc) -> ReferenceMesh {
    let mut m = hex_block(n, h, alloc).expect("block");
    for (v, p) in moves {
        m.set_point(PointIx::new(*v), Point3::new(p[0], p[1], p[2])).expect("move");
    }
    m
}

impl TypedOp for InsertBlock {
    const NAME: &'static str = "test.insert_block";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let before = ctx.layer::<MeshLayer>()?.alloc().clone();
        let mut after = before.clone();
        let mesh = block(self.n, self.h, &self.moves, &mut after);
        let quanta = ctx.layer::<ModelLayer>()?.hash_quanta();
        let entry = MeshEntry::new(Box::new(mesh), Vec::new(), None, quanta);
        insert_mesh(ctx, entry, &before, after, "mesh")
    }
}

/// Insert the sheared (`kind = "sheared"`) or stretched (`"stretched"`) hex pair.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct InsertPair {
    pub kind: String,
}

pub const CUBE: [[f64; 3]; 8] = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];

pub fn pair_mesh(kind: &str, alloc: &mut SerialAlloc) -> ReferenceMesh {
    let mut p = CUBE.to_vec();
    match kind {
        "sheared" => p.extend([[2., 2., 0.], [2., 3., 0.], [2., 2., 1.], [2., 3., 1.]]),
        _ => p.extend([[3., 0., 0.], [3., 1., 0.], [3., 0., 1.], [3., 1., 1.]]),
    }
    let cells = [CellSpec::Template(CellShape::Hex, (0..8).collect()), CellSpec::Template(CellShape::Hex, vec![1, 8, 9, 2, 5, 10, 11, 6])];
    build_mesh(&p, &cells, alloc).expect("pair")
}

impl TypedOp for InsertPair {
    const NAME: &'static str = "test.insert_pair";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let before = ctx.layer::<MeshLayer>()?.alloc().clone();
        let mut after = before.clone();
        let mesh = pair_mesh(&self.kind, &mut after);
        let quanta = ctx.layer::<ModelLayer>()?.hash_quanta();
        insert_mesh(ctx, MeshEntry::new(Box::new(mesh), Vec::new(), None, quanta), &before, after, "mesh")
    }
}

/// Delete a volume and what only it uses.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct KillVolume {
    pub volume: EntityId,
}

impl TypedOp for KillVolume {
    const NAME: &'static str = "test.kill_volume";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let (_, txn) = topo_txn(ctx, |topo, _| topo.kill_cascade(self.volume).or_op())?;
        commit_topology(ctx, &txn, &[])
    }
}

pub fn id(kind: EntityKind, serial: u64) -> EntityId {
    EntityId::new(kind, serial).expect("id")
}

pub fn registry() -> Registry {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::default()).expect("layers");
    reg.install(&MeshPlugin).expect("mesher");
    reg.install(&DiagPlugin).expect("diag");
    reg.register::<CreateBrick>().expect("brick");
    reg.register::<InsertBlock>().expect("block");
    reg.register::<InsertPair>().expect("pair");
    reg.register::<KillVolume>().expect("kill");
    reg
}

pub fn engine(op_names: Option<OpNames>, cold: bool) -> Engine {
    let reg = registry();
    let mut services = Services::new().with_versioned("mesher", 1, MesherConfig::new(stubs::registry()));
    if let Some(n) = op_names {
        services = services.with(n);
    }
    let cache = if cold { DerivedCache::disabled() } else { DerivedCache::default() };
    Engine::new(Arc::new(reg), services, Arc::new(cache)).expect("engine")
}

/// The real op names of the rig's registry.
pub fn rig_op_names() -> OpNames {
    OpNames::new(registry().op_names().into_iter().map(str::to_owned))
}

pub struct Rig {
    pub s: Session,
}

impl Rig {
    pub fn new() -> Self {
        Self { s: Session::new(engine(Some(rig_op_names()), false)) }
    }
    pub fn with_names(names: Option<OpNames>) -> Self {
        Self { s: Session::new(engine(names, false)) }
    }
    pub fn cold() -> Self {
        Self { s: Session::new(engine(Some(rig_op_names()), true)) }
    }

    pub fn apply<T: TypedOp>(&mut self, op: T) -> Result<(Hash256, Delta), TimelineError> {
        self.apply_dyn(&op)
    }

    pub fn apply_dyn(&mut self, op: &dyn Op) -> Result<(Hash256, Delta), TimelineError> {
        let q = QuietControl::new();
        self.s.apply(op, Meta::new("test", "", ""), &q.control())
    }

    /// Apply an op given as JSON through the registry, as a front end would.
    pub fn json(&mut self, name: &str, params: Value) -> Result<(Hash256, Delta), String> {
        let rec = OpRecord::new(name, 1, params).map_err(|e| e.to_string())?;
        let op = self.s.engine().registry().decode(&rec).map_err(|e| e.to_string())?;
        self.apply_dyn(op.as_ref()).map_err(|e| e.to_string())
    }

    pub fn state(&self) -> &State {
        self.s.state()
    }

    pub fn query(&self, name: &str, input: Value) -> Result<Value, String> {
        let q = QuietControl::new();
        self.s.engine().run_query(self.s.state(), name, &input, &q.control()).map_err(|e| e.to_string())
    }

    /// Brick plus an inserted block; returns the mesh id.
    pub fn with_block(n: [u32; 3], moves: &[(usize, [f64; 3])]) -> (Self, EntityId) {
        let mut r = Self::new();
        r.apply(CreateBrick { size: [n[0] as f64, n[1] as f64, n[2] as f64] }).expect("brick");
        let (_, d) = r.apply(InsertBlock { n, h: [1.0; 3], moves: moves.to_vec() }).expect("block");
        let mesh = d.outputs["mesh"][0];
        (r, mesh)
    }

    pub fn with_pair(kind: &str) -> (Self, EntityId) {
        let mut r = Self::new();
        r.apply(CreateBrick { size: [1.0, 1.0, 1.0] }).expect("brick");
        let (_, d) = r.apply(InsertPair { kind: kind.into() }).expect("pair");
        let mesh = d.outputs["mesh"][0];
        (r, mesh)
    }
}

pub fn assert_finite(v: &Value, path: &str) {
    match v {
        Value::Number(n) => assert!(n.as_f64().is_some_and(f64::is_finite), "non-finite number at {path}"),
        Value::Array(a) => a.iter().enumerate().for_each(|(i, x)| assert_finite(x, &format!("{path}[{i}]"))),
        Value::Object(o) => o.iter().for_each(|(k, x)| assert_finite(x, &format!("{path}.{k}"))),
        _ => {}
    }
}

/// Structure exact, floats to `rel` relative.
pub fn same(a: &Value, b: &Value, rel: f64, path: &str) {
    match (a, b) {
        (Value::Number(x), Value::Number(y)) if x.is_f64() || y.is_f64() => {
            let (x, y) = (x.as_f64().unwrap(), y.as_f64().unwrap());
            assert!((x - y).abs() <= rel * x.abs().max(y.abs()).max(1e-300) + 1e-300, "{path}: {x} vs {y}");
        }
        (Value::Array(x), Value::Array(y)) => {
            assert_eq!(x.len(), y.len(), "{path}: array length");
            for (i, (p, q)) in x.iter().zip(y).enumerate() {
                same(p, q, rel, &format!("{path}[{i}]"));
            }
        }
        (Value::Object(x), Value::Object(y)) => {
            assert_eq!(x.keys().collect::<Vec<_>>(), y.keys().collect::<Vec<_>>(), "{path}: keys");
            for (k, p) in x {
                same(p, &y[k], rel, &format!("{path}.{k}"));
            }
        }
        _ => assert_eq!(a, b, "{path}"),
    }
}

pub fn golden_path(name: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden").join(name)
}

/// Compare `actual` with the golden file (bless with `SM_DIAG_BLESS=1`).
pub fn golden(name: &str, actual: &Value, rel: f64) {
    let p = golden_path(name);
    if std::env::var_os("SM_DIAG_BLESS").is_some() {
        std::fs::write(&p, serde_json::to_string_pretty(actual).unwrap() + "\n").unwrap();
    }
    let want: Value = serde_json::from_str(&std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{name}: {e} (SM_DIAG_BLESS=1 creates it)"))).unwrap();
    same(&want, actual, rel, name);
}

pub fn jv<T: Serialize>(t: &T) -> Value {
    serde_json::to_value(t).unwrap()
}

pub fn q(v: Value) -> Value {
    json!(v)
}
