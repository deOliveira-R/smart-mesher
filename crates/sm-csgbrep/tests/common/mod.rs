//! Test rig: an engine with the core layers and the model, boolean, fem and csgbrep plugins, and
//! inline decks (nothing touches `$TMPDIR`).
#![allow(dead_code)]

pub mod decks;

use std::sync::Arc;

use sm_cadio::Source;
use sm_core::{EntityId, Point3};
use sm_csgbrep::{ImportOpenmc, OpenmcOptions};
use sm_layer_api::{CoreLayers, ModelLayer, TopoHost};
use sm_model_api::{Model, RegionTag};
use sm_op_api::{DerivedCache, Delta, Engine, Meta, Op, QuietControl, Registry, Services, Session, TimelineError};
use sm_topo_api::{Dim, TopologyView};

pub fn topo_store() -> Box<dyn TopoHost> {
    Box::new(sm_topo::TopoStore::new())
}

pub fn registry() -> Registry {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::new(topo_store)).expect("core layers");
    reg.install(&sm_model::ModelPlugin).expect("model");
    reg.install(&sm_fem::FemPlugin).expect("fem");
    reg.install(&sm_boolean::BooleanPlugin).expect("boolean");
    reg.install(&sm_csgbrep::CsgbrepPlugin).expect("csgbrep");
    reg
}

pub struct Rig {
    pub s: Session,
}

impl Rig {
    pub fn new() -> Self {
        let engine = Engine::new(Arc::new(registry()), Services::new(), Arc::new(DerivedCache::default())).expect("engine");
        Self { s: Session::new(engine) }
    }

    pub fn run(&mut self, op: &dyn Op) -> Result<Delta, TimelineError> {
        let q = QuietControl::new();
        self.s.apply(op, Meta::new("test", "", ""), &q.control()).map(|(_, d)| d)
    }

    pub fn import(&mut self, geometry: &str, materials: Option<&str>) -> Result<Delta, TimelineError> {
        self.import_with(geometry, materials, None)
    }

    pub fn import_with(&mut self, geometry: &str, materials: Option<&str>, options: Option<OpenmcOptions>) -> Result<Delta, TimelineError> {
        self.run(&ImportOpenmc { geometry: Source::inline(geometry.as_bytes()), materials: materials.map(|m| Source::inline(m.as_bytes())), options })
    }

    pub fn layer(&self) -> &ModelLayer {
        self.s.state().layer::<ModelLayer>().expect("model layer")
    }

    pub fn topo(&self) -> &dyn TopologyView {
        self.layer().topo()
    }

    pub fn model(&self) -> &Model {
        self.layer().model()
    }

    pub fn hash(&self) -> sm_op_api::Hash256 {
        self.s.state().hash()
    }

    pub fn volume(&self, v: EntityId) -> f64 {
        self.topo().volume_moments(v).expect("moments").volume
    }

    pub fn centroid(&self, v: EntityId) -> Point3 {
        self.topo().volume_moments(v).expect("moments").centroid().expect("centroid")
    }

    pub fn live_volumes(&self) -> Vec<EntityId> {
        self.topo().cells(Dim::Volume).to_vec()
    }

    pub fn tag(&self, v: EntityId) -> Option<&RegionTag> {
        self.model().region_tag(v)
    }
}

pub fn diag<'a>(d: &'a Delta, code: &str) -> Vec<&'a str> {
    d.diagnostics.iter().filter(|x| x.code == code).map(|x| x.message.as_str()).collect()
}

/// The materials file of a deck with `n` numbered materials named `m1`, `m2`, ...
pub fn materials(n: u32) -> String {
    let mut s = String::from("<?xml version='1.0' encoding='UTF-8'?>\n<materials>\n");
    for i in 1..=n {
        s.push_str(&format!("  <material id=\"{i}\" name=\"m{i}\"><density value=\"1.0\" units=\"g/cc\"/><nuclide name=\"H1\" ao=\"1.0\"/></material>\n"));
    }
    s.push_str("</materials>\n");
    s
}

pub fn fixture(name: &str, file: &str) -> String {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../sm-csgio/tests/fixtures/openmc").join(name).join(file);
    std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{}: {e}", p.display()))
}
