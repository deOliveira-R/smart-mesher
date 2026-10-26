//! Op-level test rig: an engine with the core layers, the model plugin and the boolean plugin.
#![allow(dead_code)]

use std::sync::Arc;

use sm_boolean::BooleanPlugin;
use sm_core::EntityId;
use sm_layer_api::{CoreLayers, ModelLayer, TopoHost};
use sm_model::{CreateBrick, CreateCylinder, CreateSphere, ModelPlugin};
use sm_op_api::{DerivedCache, Delta, Engine, Meta, Op, QuietControl, Registry, Services, Session, TimelineError};
use sm_topo_api::{TopologyView};

pub type Factory = fn() -> Box<dyn TopoHost>;

pub fn cell_complex() -> Box<dyn TopoHost> {
    Box::new(sm_topo_api::CellComplex::new())
}

pub fn topo_store() -> Box<dyn TopoHost> {
    Box::new(sm_topo::TopoStore::new())
}

pub fn registry(factory: Factory) -> Registry {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::new(factory)).expect("core layers");
    reg.install(&ModelPlugin).expect("model plugin");
    reg.install(&BooleanPlugin).expect("boolean plugin");
    reg.register::<Tag>().expect("tag");
    reg.register::<FakeMesh>().expect("fake mesh");
    reg
}

pub struct Rig {
    pub s: Session,
}

/// A created solid.
#[derive(Clone, Debug)]
pub struct Solid {
    pub body: EntityId,
    pub volume: EntityId,
    pub faces: Vec<EntityId>,
}

impl Rig {
    pub fn new(factory: Factory) -> Self {
        let engine = Engine::new(Arc::new(registry(factory)), Services::new(), Arc::new(DerivedCache::default())).expect("engine");
        Self { s: Session::new(engine) }
    }

    pub fn cc() -> Self {
        Self::new(cell_complex)
    }

    /// A rig without the derived-data cache (cold replay checks).
    pub fn uncached(factory: Factory) -> Self {
        let engine = Engine::new(Arc::new(registry(factory)), Services::new(), Arc::new(DerivedCache::disabled())).expect("engine");
        Self { s: Session::new(engine) }
    }

    pub fn run(&mut self, op: &dyn Op) -> Result<Delta, TimelineError> {
        let q = QuietControl::new();
        self.s.apply(op, Meta::new("test", "", ""), &q.control()).map(|(_, d)| d)
    }

    pub fn topo(&self) -> &dyn TopologyView {
        self.s.state().layer::<ModelLayer>().expect("model layer").topo()
    }

    pub fn layer(&self) -> &ModelLayer {
        self.s.state().layer::<ModelLayer>().expect("model layer")
    }

    pub fn hash(&self) -> sm_op_api::Hash256 {
        self.s.state().hash()
    }

    fn solid(d: &Delta) -> Solid {
        Solid { body: d.outputs["body"][0], volume: d.outputs["volume"][0], faces: d.outputs["faces"].clone() }
    }

    pub fn brick(&mut self, size: [f64; 3], center: [f64; 3]) -> Solid {
        let d = self.run(&CreateBrick { x: size[0], y: Some(size[1]), z: Some(size[2]), center: Some(center) }).expect("brick");
        Self::solid(&d)
    }

    pub fn cylinder(&mut self, height: f64, radius: f64, center: [f64; 3]) -> Solid {
        let d = self.run(&CreateCylinder { height, radius, center: Some(center) }).expect("cylinder");
        Self::solid(&d)
    }

    pub fn sphere(&mut self, radius: f64, center: [f64; 3]) -> Solid {
        let d = self.run(&CreateSphere { radius, center: Some(center) }).expect("sphere");
        Self::solid(&d)
    }
}

impl Rig {
    /// Run a library function on a clone of the topology host inside one operation; returns the
    /// edited clone, or the error with the `end_op` issues spelled out.
    pub fn lib<T>(&self, f: impl FnOnce(&mut dyn TopoHost) -> Result<T, sm_boolean::BooleanError>) -> Result<(Box<dyn TopoHost>, T), String> {
        let mut host = self.layer().topo_host().clone_host();
        host.begin_op(sm_topo_api::OpId(self.layer().ops_run() + 1)).map_err(|e| e.to_string())?;
        let r = match f(&mut *host) {
            Ok(r) => r,
            Err(e) => {
                host.abort_op();
                return Err(format!("{e}"));
            }
        };
        match host.end_op() {
            Ok(_) => Ok((host, r)),
            Err(e) => Err(format!("end_op: {e} {:?}", match &e { sm_topo_api::TopoError::OpInvalid(v) => v.clone(), _ => vec![] })),
        }
    }
}

/// Mesh the given volumes of the model layer with the stub schemes in one `Driver::mesh` call,
/// every edge at `size`.
pub fn mesh_volumes(rig: &Rig, volumes: &[EntityId], size: f64) -> sm_scheme_api::Meshed<sm_mesh_api::reference::ReferenceMesh> {
    use sm_mesh_api::SerialAlloc;
    use sm_mesh_api::reference::ReferenceBuilder;
    use sm_scheme_api::{Driver, Firm, IntervalRequest, MeshInput, MeshIntent, ReferenceCornerPicker, ReferenceIntervalSolver, Sequential, WholeFaces};
    use sm_topo_api::Dim;
    let layer = rig.layer();
    let topo = layer.topo();
    let reg = sm_scheme_api::stubs::registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let mut intent = MeshIntent::default();
    for &e in topo.cells(Dim::Edge) {
        intent.set_intervals(e, Firm::hard(IntervalRequest::Size(size)));
    }
    let input = MeshInput::new(topo, layer.model(), &WholeFaces, volumes);
    driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::new(), &mut SerialAlloc::new()).expect("mesh")
}

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_op_api::{ApplyCtx, OpError, TypedOp};

/// Colors volumes and creates a group of the same members (test only).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Tag {
    pub volumes: Vec<EntityId>,
    pub label: u32,
}

impl TypedOp for Tag {
    const NAME: &'static str = "test.tag";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        use sm_layer_api::{LayerResultExt, model_txn};
        use sm_model_api::Domain;
        model_txn(
            ctx,
            |m, _t| {
                let c = match m.color_by_label(Domain::Volume, self.label) {
                    Some(c) => c,
                    None => m.add_color(self.label, "tag", Domain::Volume).or_op()?,
                };
                m.color_volumes(c, &self.volumes).or_op()?;
                m.add_group(&format!("g{}", self.label), &self.volumes).or_op()?;
                Ok(c)
            },
            |_, _| Vec::new(),
        )?;
        Ok(())
    }
}

/// Inserts a one-hex mesh on a volume (test only), to exercise the meshed-volume guard.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FakeMesh {
    pub volume: EntityId,
}

const UNIT_CUBE: [[f64; 3]; 8] = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];

impl TypedOp for FakeMesh {
    const NAME: &'static str = "test.fake_mesh";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        use sm_layer_api::{MeshEntry, MeshLayer, insert_mesh};
        let q = ctx.layer::<ModelLayer>()?.hash_quanta();
        let before = ctx.layer::<MeshLayer>()?.alloc().clone();
        let mut alloc = before.clone();
        let fx = sm_testkit::meshes::build_mesh(&UNIT_CUBE, &[sm_testkit::meshes::CellSpec::Template(sm_mesh_api::CellShape::Hex, (0..8).collect())], &mut alloc).map_err(|e| OpError::Failed(e.to_string()))?;
        insert_mesh(ctx, MeshEntry::new(Box::new(fx), vec![self.volume], None, q), &before, alloc, "mesh")
    }
}
