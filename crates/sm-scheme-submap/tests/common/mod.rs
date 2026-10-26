//! Fixtures: bare planar faces from `sm_testkit::topo::planar_face`, registries with the stub
//! `equal` curve scheme and the real `submap`, and a one-call mesher.
#![allow(dead_code, missing_docs)]

use sm_core::EntityId;
use sm_mesh::{Mesh, MeshBuilder};
use sm_mesh_api::{MeshRead, SerialAlloc};
use sm_model_api::Model;
use sm_scheme_api::stubs::{Equal, Pave};
use sm_scheme_api::{
    CornerPicker, Driver, Executor, Firm, IntervalRequest, IntervalSolver, MeshInput, MeshIntent,
    Meshed, ReferenceCornerPicker, ReferenceIntervalSolver, SchemeDim, SchemeRegistry, WholeFaces,
};
use sm_testkit::loops2d::Region2;
use sm_topo_api::CellComplex;

/// A bare planar face with the ids a test needs.
pub struct BareFace {
    pub topo: CellComplex,
    pub model: Model,
    pub face: EntityId,
    /// Edges in loop order (edge `i` runs from vertex `i` to vertex `i + 1`).
    pub edges: Vec<EntityId>,
}

/// The planar face of `region` at `z = 0`.
pub fn face_of(region: &Region2) -> BareFace {
    let fx = sm_testkit::topo::planar_face(region, 0.0).expect("planar face");
    BareFace {
        topo: fx.complex,
        model: Model::new(),
        face: fx.face,
        edges: fx.edges[0].clone(),
    }
}

/// `equal` (stub curve scheme) plus the real `submap`.
pub fn registry() -> SchemeRegistry {
    let mut reg = SchemeRegistry::new();
    reg.register(Box::new(Equal)).expect("equal");
    sm_scheme_submap::register(&mut reg).expect("submap");
    reg
}

/// `equal`, the stub `pave` and the real `submap`, with the registry chain `submap -> pave`.
pub fn registry_with_pave_fallback() -> SchemeRegistry {
    let mut reg = registry();
    reg.register(Box::new(Pave)).expect("pave");
    reg.set_fallback(SchemeDim::Surface, "submap", &["pave"])
        .expect("chain");
    reg
}

/// Hard interval counts per edge.
pub fn counts(pairs: &[(EntityId, u32)]) -> MeshIntent {
    let mut intent = MeshIntent::default();
    for (e, c) in pairs {
        intent.set_intervals(*e, Firm::hard(IntervalRequest::Count(*c)));
    }
    intent
}

/// A constant default element size.
pub fn sized(size: f64) -> MeshIntent {
    let mut intent = MeshIntent::default();
    intent.defaults.sizing = Some(sm_scheme_api::SizingSpec::Constant { size });
    intent
}

/// Mesh the faces `sel` into a dimension-2 mesh with the given registry and solvers.
pub fn mesh_with(
    reg: &SchemeRegistry,
    solver: &dyn IntervalSolver,
    picker: &dyn CornerPicker,
    f: &BareFace,
    intent: &MeshIntent,
    exec: &dyn Executor,
) -> Result<Meshed<Mesh>, sm_scheme_api::SchemeError> {
    let driver = Driver::new(reg, solver, picker);
    let sel = [f.face];
    let input = MeshInput::new(&f.topo, &f.model, &WholeFaces, &sel);
    driver.mesh(
        &input,
        intent,
        exec,
        MeshBuilder::surface(),
        &mut SerialAlloc::new(),
    )
}

/// Mesh with the reference solver and picker.
pub fn mesh_face(
    f: &BareFace,
    intent: &MeshIntent,
    exec: &dyn Executor,
) -> Result<Meshed<Mesh>, sm_scheme_api::SchemeError> {
    mesh_with(
        &registry(),
        &ReferenceIntervalSolver::default(),
        &ReferenceCornerPicker,
        f,
        intent,
        exec,
    )
}

/// Every vertex position of a mesh.
pub fn points(m: &dyn MeshRead) -> Vec<[f64; 3]> {
    (0..m.vertex_count())
        .map(|v| {
            let p = m.point(v);
            [p.x, p.y, p.z]
        })
        .collect()
}

/// Total area of a dimension-2 mesh through `sm_quality`.
pub fn area(m: &dyn MeshRead) -> f64 {
    sm_quality::compute(
        m,
        sm_quality::Metric::Volume,
        &sm_quality::Options::default(),
    )
    .expect("quality")
    .iter()
    .sum()
}

/// `(min, max)` scaled Jacobian over the cells.
pub fn scaled_jacobian_range(m: &dyn MeshRead) -> (f64, f64) {
    let v = sm_quality::compute(
        m,
        sm_quality::Metric::ScaledJacobian,
        &sm_quality::Options::default(),
    )
    .expect("quality");
    v.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |a, x| {
        (a.0.min(*x), a.1.max(*x))
    })
}
