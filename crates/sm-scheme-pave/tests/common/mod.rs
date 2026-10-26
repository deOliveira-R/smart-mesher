//! Shared helpers of the driver-level tests: a registry with the real `pave` plus the stub
//! `equal` (curves) and `tri` (the fallback target), counts, one-call meshing, measures.
#![allow(dead_code, missing_docs)]

use sm_core::{EntityId, Point3};
use sm_mesh::{Mesh, MeshBuilder};
use sm_mesh_api::{CellIx, MeshRead, SerialAlloc};
use sm_model_api::Model;
use sm_scheme_api::stubs::{Equal, Tri};
use sm_scheme_api::{
    Driver, Executor, Firm, IntervalRequest, MeshInput, MeshIntent, Meshed, ReferenceCornerPicker,
    ReferenceIntervalSolver, SchemeError, SchemeRegistry, WholeFaces,
};
use sm_topo_api::fixtures::{CylinderIds, with_op};
use sm_topo_api::{CellComplex, OpId, TopologyView, TopologyViewExt};

/// The real `pave`, the stub `equal` and the stub `tri` (never `stubs::registry()`: it holds
/// the stub `pave` too).
pub fn registry() -> SchemeRegistry {
    let mut reg = SchemeRegistry::new();
    reg.register(Box::new(Equal)).expect("equal");
    reg.register(Box::new(Tri)).expect("tri");
    sm_scheme_pave::register(&mut reg).expect("pave");
    reg
}

/// Hard interval counts per edge.
pub fn hard(pairs: &[(EntityId, u32)]) -> MeshIntent {
    let mut intent = MeshIntent::default();
    for (e, c) in pairs {
        intent.set_intervals(*e, Firm::hard(IntervalRequest::Count(*c)));
    }
    intent
}

/// Mesh the faces `sel` into a dimension-2 mesh with `reg`.
pub fn mesh_with(
    reg: &SchemeRegistry,
    topo: &CellComplex,
    model: &Model,
    sel: &[EntityId],
    intent: &MeshIntent,
    exec: &dyn Executor,
) -> Result<Meshed<Mesh>, SchemeError> {
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(reg, &solver, &picker);
    let input = MeshInput::new(topo, model, &WholeFaces, sel);
    driver.mesh(&input, intent, exec, MeshBuilder::surface(), &mut SerialAlloc::new())
}

/// [`mesh_with`] over [`registry`].
pub fn mesh_faces(
    topo: &CellComplex,
    model: &Model,
    sel: &[EntityId],
    intent: &MeshIntent,
    exec: &dyn Executor,
) -> Result<Meshed<Mesh>, SchemeError> {
    mesh_with(&registry(), topo, model, sel, intent, exec)
}

pub fn points(m: &dyn MeshRead) -> Vec<Point3> {
    (0..m.vertex_count()).map(|v| m.point(v)).collect()
}

/// Total area through `sm_quality`.
pub fn area(m: &dyn MeshRead) -> f64 {
    sm_quality::compute(m, sm_quality::Metric::Volume, &sm_quality::Options::default())
        .expect("quality")
        .iter()
        .sum()
}

/// `(min, mean)` of the scaled Jacobian over all cells.
pub fn scaled_jacobian(m: &dyn MeshRead) -> (f64, f64) {
    let s = sm_quality::report(m, &[sm_quality::Metric::ScaledJacobian], &sm_quality::Options::default())
        .expect("quality")
        .remove(0)
        .1;
    assert_eq!(s.undefined, 0, "every cell has a scaled Jacobian");
    (s.min, s.mean)
}

/// The corner points of cell `c`.
pub fn corners(m: &dyn MeshRead, c: usize) -> Vec<Point3> {
    m.cell_corners(CellIx::new(c)).expect("corners").iter().map(|v| m.point(v.get())).collect()
}

/// The full cylinder fixture (bottom cap, top cap, lateral face with its seam used twice).
pub fn cylinder(r: f64, h: f64) -> (CellComplex, CylinderIds, Model) {
    let mut c = CellComplex::new();
    let ids = with_op(&mut c, OpId(1), |t| sm_topo_api::fixtures::cylinder(t, r, h)).expect("cylinder");
    (c, ids, Model::new())
}

/// Length of an edge (for telling the x edges of a brick face from the y edges).
pub fn edge_len(t: &dyn TopologyView, e: EntityId) -> f64 {
    sm_scheme_api::geom::edge_length(t, e).expect("edge length")
}

/// Edges of a face.
pub fn face_edges(t: &CellComplex, f: EntityId) -> Vec<EntityId> {
    t.face_edges(f).to_vec()
}
