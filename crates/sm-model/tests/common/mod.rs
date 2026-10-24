//! Shared test rig: engines over both topology hosts, an op runner, a query runner and a
//! test-only op that inserts a mesh on a volume (to exercise the meshed-volume rule).
#![allow(dead_code)]

use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{CoreLayers, MeshEntry, MeshLayer, ModelLayer, TopoHost, insert_mesh};
use sm_model::ModelPlugin;
use sm_op_api::{
    ApplyCtx, DerivedCache, Engine, Hash256, Meta, Op, OpError, QuietControl, Registry, Services,
    Session, TimelineError, TypedOp,
};
use sm_topo_api::CellComplex;

pub type Factory = fn() -> Box<dyn TopoHost>;

pub fn cell_complex() -> Box<dyn TopoHost> {
    Box::new(CellComplex::new())
}

pub fn topo_store() -> Box<dyn TopoHost> {
    Box::new(sm_topo::TopoStore::new())
}

const UNIT_CUBE: [[f64; 3]; 8] = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];

/// Inserts the unit hex mesh with `selection = [volume]` (test only).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FakeMesh {
    pub volume: EntityId,
}

impl TypedOp for FakeMesh {
    const NAME: &'static str = "test.fake_mesh";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let q = ctx.layer::<ModelLayer>()?.hash_quanta();
        let before = ctx.layer::<MeshLayer>()?.alloc().clone();
        let mut alloc = before.clone();
        let fx = sm_testkit::meshes::build_mesh(
            &UNIT_CUBE,
            &[sm_testkit::meshes::CellSpec::Template(sm_mesh_api::CellShape::Hex, (0..8).collect())],
            &mut alloc,
        )
        .map_err(|e| OpError::Failed(e.to_string()))?;
        insert_mesh(ctx, MeshEntry::new(Box::new(fx), vec![self.volume], None, q), &before, alloc, "mesh")
    }
}

pub fn registry(factory: Factory) -> Registry {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::new(factory)).expect("core layers");
    reg.install(&ModelPlugin).expect("model plugin");
    reg.register::<FakeMesh>().expect("fake mesh");
    reg.register::<MirrorDeck>().expect("mirror deck");
    reg.register::<MergedBricks>().expect("merged bricks");
    reg
}

pub fn engine(factory: Factory, cache: DerivedCache) -> Engine {
    Engine::new(Arc::new(registry(factory)), Services::new(), Arc::new(cache)).expect("engine")
}

pub fn session(factory: Factory) -> Session {
    Session::new(engine(factory, DerivedCache::default()))
}

pub fn meta() -> Meta {
    Meta::new("test", "", "")
}

pub fn id(kind: EntityKind, serial: u64) -> EntityId {
    EntityId::new(kind, serial).expect("id")
}

pub fn range(kind: EntityKind, n: u64) -> Vec<EntityId> {
    (1..=n).map(|s| id(kind, s)).collect()
}

pub fn run(s: &mut Session, op: &dyn Op) -> Result<(Hash256, sm_op_api::Delta), TimelineError> {
    let q = QuietControl::new();
    s.apply(op, meta(), &q.control())
}

/// Run a registered query by name with a JSON input.
pub fn query(s: &Session, name: &str, input: serde_json::Value) -> Result<serde_json::Value, sm_op_api::RegistryError> {
    let q = QuietControl::new();
    s.engine().run_query(s.state(), name, &input, &q.control())
}

pub fn model(s: &Session) -> &ModelLayer {
    s.state().layer::<ModelLayer>().expect("model layer")
}

/// All vertex points of the model layer, in ascending id order.
pub fn vertex_points(s: &Session) -> Vec<[f64; 3]> {
    use sm_topo_api::{Carrier, Dim};
    let t = model(s).topo();
    t.cells(Dim::Vertex)
        .iter()
        .map(|&v| match t.carrier(v) {
            Carrier::Point(p) => [p.x, p.y, p.z],
            other => panic!("vertex carrier {other:?}"),
        })
        .collect()
}

/// Face orientation oracle: for every volume use, the loop-derived vector area of the face must
/// point away from the volume centroid. Returns the worst `out * n . (c_face - c_vol) / |n|`
/// (positive means every face points outward).
pub fn worst_outwardness(s: &Session) -> f64 {
    use sm_topo_api::{Dim, Side};
    let t = model(s).topo();
    let mut worst = f64::INFINITY;
    for &v in t.cells(Dim::Volume) {
        let m = t.volume_moments(v).expect("volume moments");
        let vc = m.centroid().expect("centroid");
        for inc in t.boundary(v) {
            let fm = t.face_moments(inc.cell).expect("face moments");
            let fc = fm.centroid().expect("face centroid");
            let out = if inc.side == Side::Left { 1.0 } else { -1.0 };
            assert!(fm.area > 0.0, "face {} has non-positive area {}", inc.cell, fm.area);
            let n = fm.vector_area;
            if n.norm() < 1e-9 * fm.area {
                // A closed curved face (a cylinder side) has no net vector area.
                continue;
            }
            let d = sm_core::Vector3::new(fc.x - vc.x, fc.y - vc.y, fc.z - vc.z);
            let len = n.norm().max(1e-300);
            worst = worst.min(out * n.dot(d) / len);
        }
    }
    worst
}

/// `validate` and `validate_geometry` issues of the model layer's topology.
pub fn issues(s: &Session) -> Vec<String> {
    let t = model(s).topo();
    sm_topo_api::validate(t).into_iter().chain(sm_topo_api::validate_geometry(t)).map(|i| format!("{i:?}")).collect()
}

/// Declares a self-paired mirror deck on `face` (plane `normal . x = offset`) with a
/// model-only transaction (test only).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MirrorDeck {
    pub face: EntityId,
    pub normal: [f64; 3],
    pub offset: f64,
}

impl TypedOp for MirrorDeck {
    const NAME: &'static str = "test.mirror_deck";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        use sm_layer_api::{LayerResultExt, model_txn};
        use sm_model_api::{BoundaryKind, Deck, Plane};
        let plane = Plane::new(sm_core::Vector3::new(self.normal[0], self.normal[1], self.normal[2]), self.offset).or_op()?;
        model_txn(
            ctx,
            |m, t| {
                use sm_model_api::Domain;
                let vols: Vec<EntityId> = sm_topo_api::TopologyView::cells(t, sm_topo_api::Dim::Volume).to_vec();
                let mat = m.add_color(1, "mat", Domain::Volume).or_op()?;
                m.color_volumes(mat, &vols).or_op()?;
                let cap = m.add_color(9, "cap", Domain::Face).or_op()?;
                m.color_faces(cap, &[self.face]).or_op()?;
                let b = m.add_boundary("mirror", BoundaryKind::Deck(Deck::Mirror { plane })).or_op()?;
                m.assign_self_paired(b, &[self.face]).or_op()?;
                sm_testkit::topo::finish_boundaries(m, t, 7).or_op()?;
                Ok(b)
            },
            |b, _| vec![("boundary", vec![*b])],
        )?;
        Ok(())
    }
}

/// Two touching bricks, merged along their shared face (test only).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MergedBricks {}

impl TypedOp for MergedBricks {
    const NAME: &'static str = "test.merged_bricks";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        use sm_layer_api::{LayerResultExt, commit_topology, topo_txn};
        use sm_topo_api::fixtures;
        let ((a, b), txn) = topo_txn(ctx, |topo, _| {
            let (a, b) = fixtures::two_bricks(topo).or_op()?;
            fixtures::merge_touching_bricks(topo, &a, &b).or_op()?;
            Ok((a, b))
        })?;
        commit_topology(ctx, &txn, &[("volumes", &[a.volume, b.volume])])
    }
}

/// Model validation errors against the model layer's topology (planarity tolerance 1e-6: the
/// second-moment planarity test loses digits at coordinates of order 10).
pub fn model_errors(s: &Session) -> Vec<String> {
    let l = model(s);
    sm_model_api::validate(l.model(), l.topo(), sm_model_api::Tolerance::default().with_linear(1e-6)).errors().map(|i| format!("{:?}: {}", i.code, i.detail)).collect()
}
