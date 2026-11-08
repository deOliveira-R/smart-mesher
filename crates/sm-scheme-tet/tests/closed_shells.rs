//! Applicability is decided by the closed-shell check, not by a face count: a cylinder (three
//! faces, seam used twice) and a sphere (one face) are applicable.

use sm_scheme_api::{
    Applicability, MeshInput, MeshIntent, Params, PlanCtx, Scheme, Target, WholeFaces,
};
use sm_scheme_tet::TetVolume;
use sm_topo_api::fixtures::{cylinder, sphere, torus, with_op};
use sm_topo_api::{CellComplex, OpId};

fn applicable(topo: &CellComplex, v: sm_core::EntityId) -> Applicability {
    let model = sm_model_api::Model::new();
    let intent = MeshIntent::default();
    let sel = [v];
    let input = MeshInput::new(topo, &model, &WholeFaces, &sel);
    let cx = PlanCtx::new(&input, &intent);
    TetVolume.applicability(&cx, Target::Volume(v), &Params::empty())
}

#[test]
fn cylinder_with_three_faces_is_applicable() {
    let mut topo = CellComplex::new();
    let c = with_op(&mut topo, OpId(1), |t| cylinder(t, 0.4, 1.0)).unwrap();
    assert!(matches!(
        applicable(&topo, c.volume),
        Applicability::Applicable { score: 10, .. }
    ));
}

#[test]
fn sphere_and_torus_single_face_shells_are_applicable() {
    let mut topo = CellComplex::new();
    let s = with_op(&mut topo, OpId(1), |t| sphere(t, 1.0)).unwrap();
    assert!(matches!(
        applicable(&topo, s.volume),
        Applicability::Applicable { score: 10, .. }
    ));
    let mut topo = CellComplex::new();
    let o = with_op(&mut topo, OpId(1), |t| torus(t, 2.0, 0.5)).unwrap();
    assert!(matches!(
        applicable(&topo, o.volume),
        Applicability::Applicable { score: 10, .. }
    ));
}
