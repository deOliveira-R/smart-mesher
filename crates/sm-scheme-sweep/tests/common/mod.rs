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
    Applicability, Caps, Driver, Executor, GenCtx, GenerateFailure, MeshInput, MeshIntent, Meshed,
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
    let q = sm_quality::compute(
        m,
        sm_quality::Metric::ScaledJacobian,
        &sm_quality::Options::default(),
    )
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
    fn generate(
        &self,
        cx: &GenCtx<'_>,
        target: Target,
        p: &Params,
    ) -> Result<Patch, GenerateFailure> {
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

/// Determinant of the trilinear map of a hex (template order) at `(s, t, u)` in `[0, 1]^3`.
pub fn jacobian_det(p: &[[f64; 3]], s: f64, t: f64, u: f64) -> f64 {
    // Corner (i, j, k) ordering: 0 (0,0,0) 1 (1,0,0) 2 (1,1,0) 3 (0,1,0), then z = 1.
    let ijk = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [0.0, 1.0, 1.0],
    ];
    let mut jac = [[0.0; 3]; 3];
    for (c, q) in p.iter().enumerate() {
        let (a, b, d) = (ijk[c][0], ijk[c][1], ijk[c][2]);
        let (fs, ft, fu) = (
            if a > 0.5 { s } else { 1.0 - s },
            if b > 0.5 { t } else { 1.0 - t },
            if d > 0.5 { u } else { 1.0 - u },
        );
        let (ds, dt, du) = (
            if a > 0.5 { 1.0 } else { -1.0 },
            if b > 0.5 { 1.0 } else { -1.0 },
            if d > 0.5 { 1.0 } else { -1.0 },
        );
        for k in 0..3 {
            jac[k][0] += q[k] * ds * ft * fu;
            jac[k][1] += q[k] * fs * dt * fu;
            jac[k][2] += q[k] * fs * ft * du;
        }
    }
    jac[0][0] * (jac[1][1] * jac[2][2] - jac[1][2] * jac[2][1])
        - jac[0][1] * (jac[1][0] * jac[2][2] - jac[1][2] * jac[2][0])
        + jac[0][2] * (jac[1][0] * jac[2][1] - jac[1][1] * jac[2][0])
}

/// Exact (2x2x2 Gauss) volume of a hex mesh and the smallest Jacobian determinant at the
/// Gauss points.
pub fn hex_gauss_volume(m: &dyn MeshRead) -> (f64, f64) {
    let c = m.coords();
    let g = 0.5 / 3f64.sqrt();
    let (mut vol, mut least) = (0.0, f64::INFINITY);
    for ci in 0..m.count(sm_mesh_api::MeshEntityKind::Cell) {
        let corners = m.cell_corners(CellIx(ci as u32)).expect("hex corners");
        let p: Vec<[f64; 3]> = corners
            .iter()
            .map(|v| [c.x[v.0 as usize], c.y[v.0 as usize], c.z[v.0 as usize]])
            .collect();
        for &sx in &[0.5 - g, 0.5 + g] {
            for &sy in &[0.5 - g, 0.5 + g] {
                for &sz in &[0.5 - g, 0.5 + g] {
                    let d = jacobian_det(&p, sx, sy, sz);
                    vol += 0.125 * d;
                    least = least.min(d);
                }
            }
        }
    }
    (vol, least)
}
