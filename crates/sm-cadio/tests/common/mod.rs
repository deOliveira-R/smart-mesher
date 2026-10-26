//! Shared helpers of the sm-cadio integration tests.
#![allow(dead_code)]

use std::path::PathBuf;
use std::sync::Arc;

use sm_cadio::{BrepModel, CadioPlugin, MemoryInputStore, ReadOptions, RealizeOptions, Realized, StepInputs, read_step, realize};
use sm_layer_api::{CoreLayers, ModelLayer, TopoHost};
use sm_op_api::{DerivedCache, Delta, Engine, Hash256, Op, QuietControl, Registry, Services, Session, TimelineError};
use sm_topo_api::{CellComplex, Dim, OpId, TopologyView, validate, validate_geometry};

pub fn data_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name)
}

/// A hand-written fixture of `tests/data`.
pub fn data(name: &str) -> Vec<u8> {
    std::fs::read(data_path(name)).unwrap_or_else(|e| panic!("fixture {name}: {e}"))
}

/// A malformed fixture of `tests/data/bad`.
pub fn bad(name: &str) -> Vec<u8> {
    data(&format!("bad/{name}"))
}

pub fn read(bytes: &[u8]) -> BrepModel {
    read_step(bytes, &ReadOptions::default()).expect("read_step")
}

pub fn cell_complex() -> Box<dyn TopoHost> {
    Box::new(CellComplex::new())
}

pub fn topo_store() -> Box<dyn TopoHost> {
    Box::new(sm_topo::TopoStore::new())
}

pub type Factory = fn() -> Box<dyn TopoHost>;

/// Realize `m` into a fresh host inside operation 1 and close the operation.
pub fn import_into(factory: Factory, m: &BrepModel) -> (Box<dyn TopoHost>, Realized) {
    let mut host = factory();
    host.begin_op(OpId(1)).expect("begin");
    let r = realize(m, &mut *host, &RealizeOptions::default()).expect("realize");
    host.end_op().expect("end_op");
    (host, r)
}

pub fn count(view: &dyn TopologyView, dim: Dim) -> usize {
    view.cells(dim).len()
}

/// Every issue of both validators (empty means valid).
pub fn issues(view: &dyn TopologyView) -> Vec<String> {
    validate(view).into_iter().chain(validate_geometry(view)).map(|i| format!("{} {:?}: {}", i.cell, i.rule, i.message)).collect()
}

pub fn volume(view: &dyn TopologyView, v: sm_core::EntityId) -> f64 {
    view.volume_moments(v).expect("volume moments").volume
}

pub fn total_area(view: &dyn TopologyView) -> f64 {
    view.cells(Dim::Face).iter().map(|&f| view.face_moments(f).expect("face moments").area).sum()
}

pub fn rel(got: f64, want: f64) -> f64 {
    ((got - want) / want).abs()
}

// ---- op level

pub fn registry(factory: Factory) -> Registry {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::new(factory)).expect("core layers");
    reg.install(&CadioPlugin).expect("cadio plugin");
    reg
}

pub fn engine(factory: Factory, store: Arc<MemoryInputStore>, cache: DerivedCache) -> Engine {
    let services = Services::new().with(StepInputs(store));
    Engine::new(Arc::new(registry(factory)), services, Arc::new(cache)).expect("engine")
}

pub fn session(factory: Factory, store: Arc<MemoryInputStore>) -> Session {
    Session::new(engine(factory, store, DerivedCache::default()))
}

pub fn meta() -> sm_op_api::Meta {
    sm_op_api::Meta::new("test", "", "")
}

pub fn run(s: &mut Session, op: &dyn Op) -> Result<(Hash256, Delta), TimelineError> {
    let q = QuietControl::new();
    s.apply(op, meta(), &q.control())
}

pub fn model(s: &Session) -> &ModelLayer {
    s.state().layer::<ModelLayer>().expect("model layer")
}

/// Gauss-Legendre nodes and weights on [-1, 1] (Newton on the Legendre polynomial).
pub fn gauss_legendre(n: usize) -> Vec<(f64, f64)> {
    (0..n)
        .map(|i| {
            let mut x = (std::f64::consts::PI * (i as f64 + 0.75) / (n as f64 + 0.5)).cos();
            let mut dp = 1.0;
            for _ in 0..100 {
                let (mut p0, mut p1) = (1.0, x);
                for k in 2..=n {
                    let p2 = ((2 * k - 1) as f64 * x * p1 - (k - 1) as f64 * p0) / k as f64;
                    p0 = p1;
                    p1 = p2;
                }
                dp = n as f64 * (x * p1 - p0) / (x * x - 1.0);
                let dx = p1 / dp;
                x -= dx;
                if dx.abs() < 1e-16 {
                    break;
                }
            }
            (x, 2.0 / ((1.0 - x * x) * dp * dp))
        })
        .collect()
}

/// `(1/3) integral p . (du x dv) du dv` over the whole parameter domain of a parametric
/// surface, tensor Gauss-Legendre of the given order (the volume contribution of a patch that
/// is a complete face, up to the carrier sign).
pub fn patch_volume_term(s: &dyn sm_geom_api::ParametricSurface, order: usize) -> f64 {
    use sm_geom_api::Uv;
    let dom = s.domain();
    let (ur, vr) = (dom.u.range, dom.v.range);
    let g = gauss_legendre(order);
    let mut sum = 0.0;
    for (xu, wu) in &g {
        for (xv, wv) in &g {
            let u = ur.lo + 0.5 * (xu + 1.0) * ur.len();
            let v = vr.lo + 0.5 * (xv + 1.0) * vr.len();
            let j = s.jet(Uv::new(u, v));
            let n = j.du.cross(j.dv);
            sum += wu * wv * (j.p.x * n.x + j.p.y * n.y + j.p.z * n.z);
        }
    }
    sum * 0.25 * ur.len() * vr.len() / 3.0
}
