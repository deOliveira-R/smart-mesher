//! Conformance scenarios: Shell rings, internal shells and their compaction.
#![allow(clippy::too_many_arguments, clippy::type_complexity, clippy::needless_range_loop)]


use sm_core::EntityId;
use sm_topo_api::fixtures::{self, with_op};
use super::{Mk, Scenario, Sys};
use sm_topo_api::*;

type Ring = (u16, Option<ShellKind>, Vec<EntityId>);

fn rings<E: Sys>(t: &E, v: EntityId) -> Vec<Ring> {
    t.shells(v).map(|s| (s.ring, t.shell_kind(v, s.ring), s.entries.iter().map(|e| e.cell).collect())).collect()
}

/// Brick with the baffle in ring 1 and a second sheet on the same edges in ring 2.
fn two_internal_shells<E: Sys>(make: &Mk<E>) -> (E, EntityId, EntityId, EntityId) {
    let mut t = make(None);
    let (b, f) = with_op(&mut t, OpId(1), fixtures::brick_with_baffle).unwrap();
    t.begin_op(OpId(2)).unwrap();
    let lp: Vec<CoedgeSpec> = f.edges.iter().map(|&edge| CoedgeSpec { edge, sense: Sense::Forward }).collect();
    let Carrier::Surface(sc) = *t.carrier(f.face) else { panic!("baffle has a surface") };
    let sheet = t.make_face(&[&lp], Some(sc)).unwrap();
    let r = t.add_shell(b.volume, ShellSpec { kind: ShellKind::Internal, uses: &[FaceUseSpec { face: sheet, side: Side::Both }] }).unwrap();
    assert_eq!(r, 2);
    t.end_op().unwrap();
    (t, b.volume, f.face, sheet)
}

pub(super) fn emptied_shell_loses_its_kind_and_higher_rings_shift_down<E: Sys>(make: &Mk<E>) {
    let (mut t, v, baffle, sheet) = two_internal_shells(make);
    assert_eq!(rings(&t, v).len(), 3);
    t.begin_op(OpId(3)).unwrap();
    t.remove_face_use(v, baffle, Side::Both).unwrap();
    t.end_op().unwrap();
    let r = rings(&t, v);
    assert_eq!(r.len(), 2);
    assert_eq!(r[1], (1, Some(ShellKind::Internal), vec![sheet]));
    assert_eq!(t.shell_kind(v, 2), None, "no stale kind entry");
    assert!(validate(&t).is_empty());
    // The next shell takes ring 2 again.
    t.begin_op(OpId(4)).unwrap();
    let r = t.add_shell(v, ShellSpec { kind: ShellKind::Internal, uses: &[FaceUseSpec { face: baffle, side: Side::Both }] }).unwrap();
    assert_eq!(r, 2);
    t.end_op().unwrap();
}

pub(super) fn removing_the_last_use_of_the_only_inner_shell_drops_it<E: Sys>(make: &Mk<E>) {
    let mut t = make(None);
    let (b, f) = with_op(&mut t, OpId(1), fixtures::brick_with_baffle).unwrap();
    t.begin_op(OpId(2)).unwrap();
    t.remove_face_use(b.volume, f.face, Side::Both).unwrap();
    t.end_op().unwrap();
    assert_eq!(rings(&t, b.volume).len(), 1);
    assert_eq!(t.shell_kind(b.volume, 1), None);
    t.begin_op(OpId(3)).unwrap();
    let r = t.add_shell(b.volume, ShellSpec { kind: ShellKind::Internal, uses: &[FaceUseSpec { face: f.face, side: Side::Both }] }).unwrap();
    assert_eq!(r, 1);
    t.end_op().unwrap();
}

pub(super) fn remove_shell_matches_removing_its_uses<E: Sys>(make: &Mk<E>) {
    let (mut a, v, baffle, _) = two_internal_shells(make);
    let (mut b, _, _, _) = two_internal_shells(make);
    a.begin_op(OpId(3)).unwrap();
    a.remove_face_use(v, baffle, Side::Both).unwrap();
    a.end_op().unwrap();
    b.begin_op(OpId(3)).unwrap();
    assert!(matches!(b.remove_shell(v, 0), Err(TopoError::InvalidParameter(_))));
    assert!(matches!(b.remove_shell(v, 7), Err(TopoError::InvalidParameter(_))));
    b.remove_shell(v, 1).unwrap();
    b.end_op().unwrap();
    assert_eq!(rings(&a, v), rings(&b, v));
    assert_eq!(a.canonical_hash(), b.canonical_hash(), "equal shapes hash equal");
}

pub(super) fn an_aborted_op_restores_shell_kinds<E: Sys>(make: &Mk<E>) {
    let (mut t, v, _, _) = two_internal_shells(make);
    let before = t.canonical_hash();
    t.begin_op(OpId(3)).unwrap();
    t.remove_shell(v, 1).unwrap();
    t.abort_op();
    assert_eq!(t.canonical_hash(), before);
    assert_eq!(rings(&t, v).len(), 3);
}

pub(super) fn split_and_loop_assignments_stay_constructible<E: Sys>(_make: &Mk<E>) {
    // `#[non_exhaustive]` only forbids exhaustive matches outside the crate.
    let a = ShellAssign::Explicit(&[(1, 0)]);
    assert_ne!(a, ShellAssign::None);
    assert_ne!(InnerLoopAssign::ByPointTest, InnerLoopAssign::None);
}

pub(super) fn virtual_op_created_lists_the_ids_remove_retires<E: Sys>(make: &Mk<E>) {
    use sm_core::{EntityKind, Point3, Vector3};
    use sm_geom_api::{CurveDef, Interval, Line};
    use std::sync::Arc;
    let mut t = make(None);
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3])).unwrap();
    let [f0, f1] = with_op(&mut t, OpId(2), |t| {
        let line = t.shapes_mut().unwrap().add_curve(CurveDef::Line(Line::new(Point3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 1.0, 0.0)).unwrap())).unwrap();
        let chord = t.make_edge(b.vertices[4], b.vertices[7], Some(CurveCarrier { curve: line, range: Interval::new(0.0, 2f64.sqrt()), sense: Sense::Forward }))?;
        Ok(t.split_face(b.faces[5], FaceCut::Chord { edge: chord }, InnerLoopAssign::None)?)
    })
    .unwrap();
    let host: Arc<dyn TopologyView> = Arc::new(t);
    let mut ov = Overlay::new(host);
    let vf = ov.composite_faces(&[f0, f1]).unwrap();
    let op = ov.ops().iter().find(|o| o.created().contains(&vf)).unwrap();
    let ids = op.created();
    assert_eq!(ids.as_slice(), &[vf]);
    assert!(ids.iter().all(|i| i.kind() == EntityKind::PartitionFace));
    ov.remove(ids[0]).unwrap();
    assert!(ids.iter().all(|i| !ov.is_alive(*i)));
    assert!(ov.ops().iter().all(|o| !o.created().contains(&vf)));
}

pub(super) fn scenarios<E: Sys>() -> Vec<Scenario<E>> {
    vec![
        ("emptied_shell_loses_its_kind_and_higher_rings_shift_down", emptied_shell_loses_its_kind_and_higher_rings_shift_down::<E>),
        ("removing_the_last_use_of_the_only_inner_shell_drops_it", removing_the_last_use_of_the_only_inner_shell_drops_it::<E>),
        ("remove_shell_matches_removing_its_uses", remove_shell_matches_removing_its_uses::<E>),
        ("an_aborted_op_restores_shell_kinds", an_aborted_op_restores_shell_kinds::<E>),
        ("split_and_loop_assignments_stay_constructible", split_and_loop_assignments_stay_constructible::<E>),
        ("virtual_op_created_lists_the_ids_remove_retires", virtual_op_created_lists_the_ids_remove_retires::<E>),
    ]
}
