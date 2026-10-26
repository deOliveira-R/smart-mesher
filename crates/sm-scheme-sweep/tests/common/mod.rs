//! Shared test helpers: registries built from individual stubs plus the real sweep.
#![allow(dead_code)]

pub mod rng;
pub mod surf;

use sm_scheme_api::SchemeRegistry;
use sm_scheme_api::stubs::{
    BadReq, Boom, Equal, Lift, MSweep, Map, MapVol, Noop, Pave, PolyVol, Submap, TetVol, Tri,
};

/// Every stub except the stub `Sweep`, plus the real `"sweep"` scheme. Never call
/// `stubs::registry()` together with `sm_scheme_sweep::register` (duplicate name).
pub fn registry() -> SchemeRegistry {
    let mut r = SchemeRegistry::new();
    r.register(Box::new(Equal)).expect("equal");
    r.register(Box::new(Map)).expect("map");
    r.register(Box::new(Submap)).expect("submap");
    r.register(Box::new(Pave)).expect("pave");
    r.register(Box::new(Tri)).expect("tri");
    sm_scheme_sweep::register(&mut r).expect("sweep");
    r.register(Box::new(MSweep)).expect("msweep");
    r.register(Box::new(MapVol)).expect("mapvol");
    r.register(Box::new(TetVol)).expect("tetvol");
    r.register(Box::new(PolyVol)).expect("polyvol");
    r.register(Box::new(BadReq)).expect("badreq");
    r.register_transform(Box::new(Noop)).expect("noop");
    r.register_transform(Box::new(Lift)).expect("lift");
    r.register_transform(Box::new(Boom)).expect("boom");
    r
}

use sm_core::EntityId;
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::{CellIx, MeshRead, SerialAlloc, cell_volume};
use sm_model_api::Model;
use sm_scheme_api::{
    Applicability, Caps, Driver, Executor, GenCtx, GenerateFailure, Meshed, MeshInput, MeshIntent,
    Params, Patch, PlanCtx, ReferenceCornerPicker, ReferenceIntervalSolver, Scheme, SchemeDim,
    Target, WholeFaces,
};
use sm_topo_api::CellComplex;

/// Plan, generate and commit `sel` with the reference solver and builder.
pub fn mesh_with(
    reg: &SchemeRegistry,
    topo: &CellComplex,
    model: &Model,
    sel: &[EntityId],
    intent: &MeshIntent,
    exec: &dyn Executor,
) -> sm_scheme_api::Result<Meshed<ReferenceMesh>> {
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(reg, &solver, &picker);
    let input = MeshInput::new(topo, model, &WholeFaces, sel);
    driver.mesh(
        &input,
        intent,
        exec,
        ReferenceBuilder::new(),
        &mut SerialAlloc::new(),
    )
}

/// Total volume of a mesh.
pub fn total_volume(m: &dyn MeshRead) -> f64 {
    (0..m.count(sm_mesh_api::MeshEntityKind::Cell))
        .map(|c| cell_volume(m, CellIx(c as u32)))
        .sum()
}

/// Minimum scaled Jacobian over all cells (sm-quality).
pub fn min_scaled_jacobian(m: &dyn MeshRead) -> f64 {
    let q = sm_quality::compute(m, sm_quality::Metric::ScaledJacobian, &sm_quality::Options::default())
        .expect("quality");
    q.into_iter().fold(f64::INFINITY, f64::min)
}

/// A test-local geometric-bias curve scheme: consecutive interval lengths grow by `ratio`
/// from the edge's start vertex.
pub struct Bias;

impl Scheme for Bias {
    fn name(&self) -> &str {
        "bias"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Curve
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::NONE
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(80, "bias")
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, p: &Params) -> Result<Patch, GenerateFailure> {
        let Target::Curve(e) = target else {
            return Err(GenerateFailure::fatal(target, "bias", "not a curve"));
        };
        let ratio = p.get::<f64>("bias", "ratio").ok().flatten().unwrap_or(1.0);
        let n = cx.intervals(e).unwrap_or(1) as usize;
        let h0 = if (ratio - 1.0).abs() < 1e-14 {
            1.0 / n as f64
        } else {
            (ratio - 1.0) / (ratio.powi(n as i32) - 1.0)
        };
        let mut acc = 0.0;
        let fr: Vec<f64> = (0..n - 1)
            .map(|i| {
                acc += h0 * ratio.powi(i as i32);
                acc
            })
            .collect();
        cx.curve_patch(e, &fr)
    }
}

/// [`registry`] plus the test-local `bias` curve scheme.
pub fn registry_with_bias() -> SchemeRegistry {
    let mut r = registry();
    r.register(Box::new(Bias)).expect("bias");
    r
}

/// Plan only (no re-plans): the first planning error is returned as is.
pub fn plan_only(
    reg: &SchemeRegistry,
    topo: &CellComplex,
    model: &Model,
    sel: &[EntityId],
    intent: &MeshIntent,
) -> sm_scheme_api::Result<sm_scheme_api::Plan> {
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(reg, &solver, &picker);
    let input = MeshInput::new(topo, model, &WholeFaces, sel);
    driver.plan(&input, intent)
}

/// [`registry`] plus the test-local source schemes.
pub fn registry_with_sources() -> SchemeRegistry {
    let mut r = registry();
    r.register(Box::new(surf::GridFace)).expect("gridface");
    r.register(Box::new(surf::Fan)).expect("fan");
    r.register(Box::new(surf::Ring)).expect("ring");
    r.register(Box::new(surf::OnePolygon)).expect("onepolygon");
    r
}
