//! Op-level test rig: an engine with the core layers, the model plugin and the boolean plugin.
#![allow(dead_code)]

use std::sync::Arc;

use sm_boolean::BooleanPlugin;
use sm_core::EntityId;
use sm_layer_api::{CoreLayers, ModelLayer, TopoHost};
use sm_model::{CreateBrick, CreateCylinder, CreateSphere, ModelPlugin};
use sm_op_api::{DerivedCache, Delta, Engine, Meta, Op, QuietControl, Registry, Services, Session, TimelineError};
use sm_topo_api::{TopologyEdit, TopologyView};

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
