//! Shared rig: test-only ops that build bricks and meshes, and a session over `CoreLayers` +
//! `FemPlugin`.
#![allow(dead_code, missing_docs)]

use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_fem::FemPlugin;
use sm_layer_api::{
    CoreLayers, LayerResultExt, MeshEntry, MeshLayer, ModelLayer, commit_topology, insert_mesh,
    retire_mesh, topo_txn,
};
use sm_model_api::{CutSpec, Deck, Domain, Keep, Plane, plan_cut};
use sm_op_api::{
    ApplyCtx, Delta, DerivedCache, Engine, Hash256, Meta, OpError, QuietControl, Registry,
    Services, Session, State, TypedOp,
};
use sm_scheme_api::stubs as stub_schemes;
use sm_scheme_api::{
    Driver, Firmness, MeshInput, MeshIntent, ReferenceCornerPicker, ReferenceIntervalSolver,
    SchemeChoice, Sequential, SizingSpec, WholeFaces,
};
use sm_topo_api::fixtures::{brick, merge_touching_bricks, webcut_brick};
use sm_topo_api::{Coincidence, OpId};

/// Bricks `size` at each origin; roles `volume` and `faces` (six per brick, `-x +x -y +y -z +z`).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Bricks {
    pub origins: Vec<[f64; 3]>,
    pub size: [f64; 3],
}

impl TypedOp for Bricks {
    const NAME: &'static str = "test.bricks";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let (ids, txn) = topo_txn(ctx, |topo, _| {
            let mut v = Vec::new();
            for o in &self.origins {
                v.push(brick(topo, Point3::new(o[0], o[1], o[2]), self.size).or_op()?);
            }
            Ok(v)
        })?;
        let vols: Vec<EntityId> = ids.iter().map(|b| b.volume).collect();
        let faces: Vec<EntityId> = ids.iter().flat_map(|b| b.faces).collect();
        commit_topology(ctx, &txn, &[("volume", &vols), ("faces", &faces)])
    }
}

/// `n` unit bricks in a row along x merged face to face; role `volume` (ascending by x).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Row {
    pub n: usize,
}

impl TypedOp for Row {
    const NAME: &'static str = "test.row";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let (ids, txn) = topo_txn(ctx, |topo, _| {
            let mut v = Vec::new();
            for i in 0..self.n {
                v.push(brick(topo, Point3::new(i as f64, 0.0, 0.0), [1.0; 3]).or_op()?);
            }
            for w in v.windows(2) {
                topo.record_coincidence(
                    w[0].faces[1],
                    w[1].faces[0],
                    Coincidence::CoincidentUnimprinted,
                )
                .or_op()?;
            }
            for w in v.windows(2) {
                merge_touching_bricks(topo, &w[0], &w[1]).or_op()?;
            }
            Ok(v)
        })?;
        let vols: Vec<EntityId> = ids.iter().map(|b| b.volume).collect();
        commit_topology(ctx, &txn, &[("volume", &vols)])
    }
}

/// A unit brick cut in half by the plane `z = 1/2` with the upper half deleted, the cap becoming
/// a self-paired mirror deck face (the `sm_testkit::topo::mirror_chamber` model, built inside a
/// state). The volume gets block color 1 "mat", the cap face color 9 "cap".
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MirrorChamber {}

impl TypedOp for MirrorChamber {
    const NAME: &'static str = "test.mirror_chamber";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let b = {
            let (b, txn) = topo_txn(ctx, |topo, model| {
                let b = brick(topo, Point3::new(0.0, 0.0, 0.0), [1.0; 3]).or_op()?;
                let mat = model.add_color(1, "mat", Domain::Volume).or_op()?;
                model.color_volumes(mat, &[b.volume]).or_op()?;
                Ok(b)
            })?;
            commit_topology(ctx, &txn, &[("volume", &[b.volume])])?;
            b
        };
        let op = OpId(ctx.layer::<ModelLayer>()?.ops_run() + 1);
        let (_, txn) = topo_txn(ctx, |topo, model| {
            let cap_color = model.add_color(9, "cap", Domain::Face).or_op()?;
            let w = webcut_brick(topo, &b, 0.5).or_op()?;
            topo.kill_cascade(w.volumes[1]).or_op()?;
            let plane = Plane::new(Vector3::new(0.0, 0.0, 1.0), 0.5).or_op()?;
            let plan = plan_cut(&CutSpec::new(
                Deck::Mirror { plane },
                Keep::Negative,
                Point3::new(0.0, 0.0, 0.5),
            ))
            .or_op()?;
            let out = topo.lineage().cut_outcome(op);
            model
                .apply_cut(&plan, &out, Some(cap_color), topo.as_view())
                .or_op()?;
            Ok(())
        })?;
        commit_topology(ctx, &txn, &[])
    }
}

/// Mesh `volumes` with the stub sweep and a constant `size`; role `mesh`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshIt {
    pub volumes: Vec<EntityId>,
    pub size: f64,
}

impl TypedOp for MeshIt {
    const NAME: &'static str = "test.mesh_it";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let model = ctx.layer::<ModelLayer>()?.clone();
        let alloc0 = ctx.layer::<MeshLayer>()?.alloc().clone();
        let registry = stub_schemes::registry();
        let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
        let driver = Driver::new(&registry, &solver, &picker);
        let targets = self.volumes.clone();
        let input = MeshInput::new(model.topo(), model.model(), &WholeFaces, &targets);
        let mut intent = MeshIntent::default();
        intent.defaults.sizing = Some(SizingSpec::Constant { size: self.size });
        for v in &targets {
            intent.set_scheme(*v, SchemeChoice::new("sweep", Firmness::Soft));
        }
        let mut alloc = alloc0.clone();
        let out = driver
            .mesh(
                &input,
                &intent,
                &Sequential,
                sm_mesh::MeshBuilder::new(),
                &mut alloc,
            )
            .map_err(|e| OpError::Failed(e.to_string()))?;
        let entry = MeshEntry::new(
            Box::new(out.finished.mesh),
            targets,
            Some(out.plan),
            model.hash_quanta(),
        );
        insert_mesh(ctx, entry, &alloc0, alloc, "mesh")
    }
}

/// Delete a mesh.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct DropMesh {
    pub mesh: EntityId,
}

impl TypedOp for DropMesh {
    const NAME: &'static str = "test.drop_mesh";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        retire_mesh(ctx, self.mesh)
    }
}

/// Delete a volume (and what only it uses).
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

pub fn registry(fem: bool) -> Registry {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::default()).expect("layers");
    if fem {
        reg.install(&FemPlugin).expect("fem");
    }
    reg.register::<Row>().expect("row");
    reg.register::<Bricks>().expect("bricks");
    reg.register::<MirrorChamber>().expect("mirror");
    reg.register::<MeshIt>().expect("mesh_it");
    reg.register::<DropMesh>().expect("drop_mesh");
    reg.register::<KillVolume>().expect("kill");
    reg
}

pub fn engine_with(fem: bool, cold: bool) -> Engine {
    let cache = if cold {
        DerivedCache::disabled()
    } else {
        DerivedCache::default()
    };
    Engine::new(Arc::new(registry(fem)), Services::new(), Arc::new(cache)).expect("engine")
}

pub fn engine(cold: bool) -> Engine {
    engine_with(true, cold)
}

/// A session with helpers.
pub struct Rig {
    pub s: Session,
}

impl Rig {
    pub fn new() -> Self {
        Self {
            s: Session::new(engine(false)),
        }
    }

    /// A session without the FEM plugin (no `fem` layer).
    pub fn without_fem() -> Self {
        Self {
            s: Session::new(engine_with(false, false)),
        }
    }

    pub fn try_apply<T: TypedOp>(&mut self, op: T) -> Result<(Hash256, Delta), String> {
        let q = QuietControl::new();
        self.s
            .apply(&op, Meta::new("test", "", ""), &q.control())
            .map_err(|e| e.to_string())
    }

    pub fn apply<T: TypedOp>(&mut self, op: T) -> Delta {
        self.try_apply(op).expect("op applies").1
    }

    pub fn state(&self) -> &State {
        self.s.state()
    }
}

pub fn role(d: &Delta, name: &str) -> Vec<EntityId> {
    d.outputs.get(name).cloned().unwrap_or_default()
}

pub fn kind_count(s: &State, k: EntityKind) -> usize {
    s.count_of(k)
}
