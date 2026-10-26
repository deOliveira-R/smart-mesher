//! Shared helpers for the acceptance tests.
#![allow(dead_code)]

use sm_mesh_api::{MeshEdit, MeshRead, PointIx};
use sm_core::Point3;
use sm_smooth::{Method, NodeSelect, Schedule, SmoothInput, SmoothOutcome, SmoothRequest, apply_outcome, smooth};
use sm_testkit::lattice::LatticeFixture;

pub fn input(f: &LatticeFixture) -> SmoothInput<'_> {
    SmoothInput { mesh: &f.mesh, topo: &f.topo, model: &f.model }
}

pub fn req(method: Method, schedule: Schedule, iterations: u32, fix_boundary: bool) -> SmoothRequest {
    SmoothRequest { nodes: NodeSelect::All, method, iterations, tolerance: 1e-12, schedule, fix_boundary, ..SmoothRequest::default() }
}

pub fn run(f: &LatticeFixture, r: &SmoothRequest) -> SmoothOutcome {
    smooth(&input(f), r).expect("smooth")
}

/// Largest distance between `pts` and `exact`.
pub fn max_dev(pts: &[[f64; 3]], exact: &[[f64; 3]]) -> f64 {
    pts.iter()
        .zip(exact)
        .map(|(p, e)| ((p[0] - e[0]).powi(2) + (p[1] - e[1]).powi(2) + (p[2] - e[2]).powi(2)).sqrt())
        .fold(0.0, f64::max)
}

/// A copy of the fixture mesh with the outcome applied.
pub fn applied(f: &LatticeFixture, o: &SmoothOutcome) -> sm_mesh_api::reference::ReferenceMesh {
    let mut m = f.mesh.clone();
    apply_outcome(&mut m, o).expect("apply");
    m
}

pub fn set_points(m: &mut dyn MeshEdit, pts: &[[f64; 3]]) {
    for (i, p) in pts.iter().enumerate() {
        m.set_point(PointIx::new(i), Point3::new(p[0], p[1], p[2])).expect("set_point");
    }
}

pub fn pts_of(m: &dyn MeshRead) -> Vec<[f64; 3]> {
    (0..m.vertex_count()).map(|i| {
        let p = m.point(i);
        [p.x, p.y, p.z]
    }).collect()
}

use sm_core::{EntityId, EntityKind};
use sm_mesh_api::{Classification, MeshEntity, MeshEntityKind, Param, SerialAlloc, VertexIx};
use sm_model_api::Model;
use sm_scheme_api::{
    Driver, Firmness, MeshInput, MeshIntent, ReferenceCornerPicker, ReferenceIntervalSolver, SchemeChoice, Sequential,
    SizingSpec, WholeFaces, stubs,
};
use sm_testkit::surf3::{SplitMix64, mix64};
use sm_topo_api::{Carrier, TopologyView};

/// Mesh one volume of `topo` with the stub `mapvol` scheme at element size `size`.
pub fn stub_mesh(topo: &dyn TopologyView, model: &Model, volume: EntityId, size: f64) -> sm_mesh::Mesh {
    let registry = stubs::registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&registry, &solver, &picker);
    let targets = [volume];
    let input = MeshInput::new(topo, model, &WholeFaces, &targets);
    let mut intent = MeshIntent::default();
    intent.defaults.sizing = Some(SizingSpec::Constant { size });
    intent.set_scheme(volume, SchemeChoice::new("mapvol", Firmness::Soft));
    let mut alloc = SerialAlloc::new();
    let out = driver
        .mesh(&input, &intent, &Sequential, sm_mesh::MeshBuilder::new(), &mut alloc)
        .expect("stub mesh");
    out.finished.mesh
}

/// Jitter every non-vertex node like `lattice::perturb` does (SplitMix64 of `(seed, index)`),
/// then put it back on its entity; works on any editable mesh.
pub fn jitter(mesh: &mut dyn MeshEdit, topo: &dyn TopologyView, seed: u64, amp: f64, volume_only: bool) {
    let mut items: Vec<(MeshEntity, Classification)> = Vec::new();
    for i in 0..mesh.vertex_count() {
        let c = mesh.classification(MeshEntityKind::Vertex, i as u32);
        if !matches!(c.entity.kind(), EntityKind::TopoEdge | EntityKind::TopoFace | EntityKind::TopoVolume) {
            continue;
        }
        if volume_only && c.entity.kind() != EntityKind::TopoVolume {
            continue;
        }
        let p = mesh.point(i);
        let mut rng = SplitMix64(mix64(seed) ^ mix64(i as u64));
        let q = Point3::new(p.x + amp * rng.unit(), p.y + amp * rng.unit(), p.z + amp * rng.unit());
        let (moved, param) = match (c.entity.kind(), topo.carrier(c.entity)) {
            (EntityKind::TopoEdge, Carrier::Curve(cc)) => {
                let r = topo.shapes().curve(cc.curve).expect("curve").project(q, None).expect("project");
                (r.point, Param::Curve(r.t))
            }
            (EntityKind::TopoFace, Carrier::Surface(sc)) => {
                let r = topo.shapes().surface(sc.surface).expect("surface").project(q, None).expect("project");
                (r.point, Param::Surface([r.uv.u, r.uv.v]))
            }
            (EntityKind::TopoVolume, _) => (q, Param::None),
            _ => panic!("no carrier for {:?}", c.entity),
        };
        mesh.set_point(PointIx::new(i), moved).expect("set_point");
        items.push((MeshEntity::Vertex(VertexIx::new(i)), Classification { entity: c.entity, param }));
    }
    mesh.set_classes(&items).expect("set_classes");
}

/// Mean position of the nodes classified on `entity`.
pub fn centroid_of(mesh: &dyn MeshRead, entity: EntityId) -> [f64; 3] {
    let set = mesh.classified_on(entity).expect("entity has nodes");
    let mut s = [0.0; 3];
    for v in &set.vertices {
        let p = mesh.point(v.get());
        s = [s[0] + p.x, s[1] + p.y, s[2] + p.z];
    }
    let k = set.vertices.len().max(1) as f64;
    [s[0] / k, s[1] / k, s[2] / k]
}

// ---- op-level harness ---------------------------------------------------------------------------

use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_layer_api::{CoreLayers, LayerResultExt, MeshLayer, ModelLayer, TopoHost, commit_topology, model_txn, topo_txn};
use sm_mesher::{MeshPlugin, MesherConfig};
use sm_model_api::{BoundaryKind, Deck};
use sm_op_api::{ApplyCtx, Delta, DerivedCache, Engine, Hash256, Meta, Op, OpError, QuietControl, Registry, Services, Session, TypedOp};
use sm_smooth::SmoothPlugin;
use sm_topo_api::CellComplex;
use sm_topo_api::fixtures::brick;

/// `test.create_brick`.
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
        let (ids, txn) = topo_txn(ctx, |topo, _| brick(topo, Point3::new(x, y, z), self.size).or_op())?;
        commit_topology(ctx, &txn, &[("volume", &[ids.volume]), ("faces", &ids.faces), ("edges", &ids.edges)])
    }
}

/// `test.webcut`: cut the first brick (ids from 1) with the plane `z = frac * size[2]`; two volumes
/// share the cap face (roles `volumes`, `cap`).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Webcut {
    pub size: [f64; 3],
    pub frac: f64,
}

impl TypedOp for Webcut {
    const NAME: &'static str = "test.webcut";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let ids = |k, n: u64| -> Vec<EntityId> { (1..=n).map(|s| id(k, s)).collect() };
        let b = sm_topo_api::fixtures::BrickIds {
            min: Point3::new(0.0, 0.0, 0.0),
            size: self.size,
            vertices: ids(EntityKind::TopoVertex, 8).try_into().expect("8"),
            edges: ids(EntityKind::TopoEdge, 12).try_into().expect("12"),
            faces: ids(EntityKind::TopoFace, 6).try_into().expect("6"),
            volume: id(EntityKind::TopoVolume, 1),
            body: id(EntityKind::TopoBody, 1),
        };
        let (cut, txn) = topo_txn(ctx, |topo, _| sm_topo_api::fixtures::webcut_brick(topo, &b, self.frac).or_op())?;
        commit_topology(ctx, &txn, &[("volumes", &cut.volumes), ("cap", &[cut.cap])])
    }
}

/// `test.jitter`: jitter the nodes of a mesh in place (replace_mesh, no ids).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct JitterMesh {
    pub mesh: EntityId,
    pub seed: u64,
    pub amp: f64,
    pub volume_only: bool,
}

impl TypedOp for JitterMesh {
    const NAME: &'static str = "test.jitter";
    const NO_OUTPUTS: bool = true;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let model = ctx.layer::<ModelLayer>()?.clone();
        let (mut host, alloc) = {
            let meshes = ctx.layer::<MeshLayer>()?;
            let e = meshes.get(self.mesh).ok_or(OpError::UnknownEntity(self.mesh))?;
            (e.mesh.clone_host(), meshes.alloc().clone())
        };
        jitter(&mut *host, model.topo(), self.seed, self.amp, self.volume_only);
        ctx.layer_mut::<MeshLayer>()?.replace_mesh(self.mesh, Arc::from(host), alloc).or_op()
    }
}

/// `test.add_extra_node`: append one extra node (a stand-in for a high-order mesh).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddExtraNode {
    pub mesh: EntityId,
}

impl TypedOp for AddExtraNode {
    const NAME: &'static str = "test.add_extra_node";
    const NO_OUTPUTS: bool = true;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let (mut host, alloc) = {
            let meshes = ctx.layer::<MeshLayer>()?;
            let e = meshes.get(self.mesh).ok_or(OpError::UnknownEntity(self.mesh))?;
            (e.mesh.clone_host(), meshes.alloc().clone())
        };
        host.add_extra_node(Point3::new(0.5, 0.5, 0.5)).map_err(|e| OpError::Failed(e.to_string()))?;
        ctx.layer_mut::<MeshLayer>()?.replace_mesh(self.mesh, Arc::from(host), alloc).or_op()
    }
}

/// `test.pair_faces`: record a translation deck pair on two faces (model layer only).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PairFaces {
    pub a: EntityId,
    pub b: EntityId,
    pub vector: [f64; 3],
}

impl TypedOp for PairFaces {
    const NAME: &'static str = "test.pair_faces";
    const NO_OUTPUTS: bool = true;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        model_txn(
            ctx,
            |model, _| {
                model.unassign(&[self.a, self.b]);
                let [x, y, z] = self.vector;
                let id = model
                    .add_boundary("pair", BoundaryKind::Deck(Deck::Translation { vector: sm_core::Vector3::new(x, y, z) }))
                    .map_err(|e| OpError::Failed(e.to_string()))?;
                model.assign_pairs(id, &[(self.a, self.b)]).map_err(|e| OpError::Failed(e.to_string()))
            },
            |_, _| Vec::new(),
        )
    }
}

pub fn engine_with(cache: DerivedCache) -> Engine {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::new(|| -> Box<dyn TopoHost> { Box::new(CellComplex::new()) })).expect("core layers");
    reg.install(&MeshPlugin).expect("mesher");
    reg.install(&SmoothPlugin).expect("smooth");
    reg.register::<CreateBrick>().expect("brick");
    reg.register::<JitterMesh>().expect("jitter");
    reg.register::<Webcut>().expect("webcut");
    reg.register::<AddExtraNode>().expect("extra");
    reg.register::<PairFaces>().expect("pair");
    Engine::new(Arc::new(reg), Services::new().with_versioned("mesher", 1, MesherConfig::new(stubs::registry())), Arc::new(cache))
        .expect("engine")
}

pub fn session() -> Session {
    Session::new(engine_with(DerivedCache::default()))
}

pub fn meta() -> Meta {
    Meta::new("test", "", "")
}

pub fn apply(s: &mut Session, op: &dyn Op) -> Result<(Hash256, Delta), sm_op_api::TimelineError> {
    let q = QuietControl::new();
    s.apply(op, meta(), &q.control())
}

pub fn run_json(s: &mut Session, name: &str, params: serde_json::Value) -> Result<(Hash256, Delta), sm_op_api::TimelineError> {
    let rec = sm_op_api::OpRecord::new(name, 1, params).expect("record");
    let op = s.engine().registry().decode(&rec).expect("decode");
    apply(s, op.as_ref())
}

pub fn query(s: &Session, name: &str, input: serde_json::Value) -> Result<serde_json::Value, sm_op_api::RegistryError> {
    let q = QuietControl::new();
    s.engine().run_query(s.state(), name, &input, &q.control())
}

pub fn id(kind: EntityKind, serial: u64) -> EntityId {
    EntityId::new(kind, serial).expect("id")
}

/// A session with a brick of `size`, meshed with the stub sweep at element size `h`; returns the mesh id.
pub fn meshed_brick(size: [f64; 3], h: f64) -> (Session, EntityId) {
    let mut s = session();
    apply(&mut s, &CreateBrick { origin: [0.0; 3], size }).expect("brick");
    run_json(&mut s, "mesh.size", serde_json::json!({"targets": ["TopoVolume#1"], "size": h})).expect("size");
    run_json(
        &mut s,
        "mesh.scheme",
        serde_json::json!({"targets": ["TopoVolume#1"], "scheme": "sweep", "params": {"source": ["TopoFace#5"], "target": ["TopoFace#6"]}}),
    )
    .expect("scheme");
    let (_, d) = run_json(&mut s, "mesh.run", serde_json::json!({"targets": ["TopoVolume#1"]})).expect("mesh.run");
    let mesh = d.outputs["mesh"][0];
    (s, mesh)
}

pub fn mesh_points(s: &Session, mesh: EntityId) -> Vec<[f64; 3]> {
    let layer = s.state().layer::<MeshLayer>().expect("layer");
    pts_of(layer.get(mesh).expect("mesh").mesh())
}
