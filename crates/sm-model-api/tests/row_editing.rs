//! W0f gaps 2: row editing (boundary kind, removals refused while referenced, instance
//! replacement), `Model::apply_transform`, and instance/slot clip volumes.

mod common;

use common::*;
use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_model_api::*;
use sm_topo_api::fixtures::{self, BrickIds, with_op};
use sm_topo_api::{CellComplex, OpId};
use std::collections::BTreeMap;

fn brick(t: &mut CellComplex, op: u64, min: [f64; 3]) -> BrickIds {
    with_op(t, OpId(op), |t| fixtures::brick(t, Point3::new(min[0], min[1], min[2]), [1.0, 1.0, 1.0])).unwrap()
}

/// Two unit bricks, A at the origin and B at x = 5, both colored.
fn two() -> (CellComplex, Model, BrickIds, BrickIds) {
    let mut t = CellComplex::new();
    let a = brick(&mut t, 1, [0.0; 3]);
    let b = brick(&mut t, 2, [5.0, 0.0, 0.0]);
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[a.volume, b.volume]).unwrap();
    (t, m, a, b)
}

fn close(a: Vector3, b: Vector3) -> bool {
    (a - b).norm() < 1e-12
}

// ------------------------------------------------------------------ boundary rows

#[test]
fn set_boundary_kind_keeps_compatible_roles_only() {
    let (_t, mut m, a, _) = two();
    let px = m.add_boundary("px", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(1.0, 0.0, 0.0) })).unwrap();
    m.assign_pairs(px, &[(a.faces[0], a.faces[1])]).unwrap();
    // Translation -> rotation keeps A/B pairs.
    let axis = Axis::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
    m.set_boundary_kind(px, BoundaryKind::Deck(Deck::rotation(axis, Turns::new(1, 4).unwrap()))).unwrap();
    assert!(matches!(m.boundary(px).unwrap().kind, BoundaryKind::Deck(Deck::Rotation { .. })));
    // Pairs cannot become a mirror or a response.
    let mirror = BoundaryKind::Deck(Deck::Mirror { plane: Plane::new(Vector3::new(1.0, 0.0, 0.0), 0.0).unwrap() });
    assert!(matches!(m.set_boundary_kind(px, mirror), Err(ModelError::Invalid(_))));
    assert!(matches!(m.set_boundary_kind(px, BoundaryKind::Response), Err(ModelError::Invalid(_))));
    // Degenerate parameters and unknown records are refused.
    assert!(m.set_boundary_kind(px, BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(0.0, 0.0, 0.0) })).is_err());
    let ghost = EntityId::new(sm_core::EntityKind::Boundary, 99).unwrap();
    assert!(matches!(m.set_boundary_kind(ghost, BoundaryKind::Response), Err(ModelError::Unknown(_))));
    // With no faces any kind goes.
    m.unassign(&[a.faces[0]]);
    m.set_boundary_kind(px, BoundaryKind::Response).unwrap();
    m.check_integrity().unwrap();
}

#[test]
fn set_boundary_kind_keeps_symmetry_generators_decks() {
    let (t, mut m, a, b) = two();
    let _ = b;
    let px = m.add_boundary("px", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(1.0, 0.0, 0.0) })).unwrap();
    m.assign_pairs(px, &[(a.faces[0], a.faces[1])]).unwrap();
    m.declare_symmetry(&[px], &t).unwrap();
    m.unassign(&[a.faces[0]]);
    assert!(matches!(m.set_boundary_kind(px, BoundaryKind::Response), Err(ModelError::Invalid(_))));
    assert!(matches!(m.remove_boundary(px), Err(ModelError::InUse(x)) if x == px));
}

#[test]
fn remove_boundary_is_refused_while_faces_or_groups_refer_to_it() {
    let (_t, mut m, a, _) = two();
    let r = m.add_boundary("vac", BoundaryKind::Response).unwrap();
    m.assign_response(r, &[a.faces[4]]).unwrap();
    assert!(matches!(m.remove_boundary(r), Err(ModelError::InUse(x)) if x == r));
    m.unassign(&[a.faces[4]]);
    let g = m.add_group("bcs", &[r]).unwrap();
    assert!(matches!(m.remove_boundary(r), Err(ModelError::InUse(_))));
    m.remove_group(g).unwrap();
    m.remove_boundary(r).unwrap();
    assert!(m.boundary(r).is_none());
    assert!(matches!(m.remove_boundary(r), Err(ModelError::Unknown(_))));
    // The id is not reused.
    let r2 = m.add_boundary("vac", BoundaryKind::Response).unwrap();
    assert!(r2 > r);
    m.check_integrity().unwrap();
}

// ------------------------------------------------------------------ instancing rows

fn inst_row(of: Fill) -> InstanceRow {
    InstanceRow::new(of, Transform::IDENTITY)
}

#[test]
fn instance_prototype_and_lattice_rows_can_be_replaced_and_removed() {
    let (_t, mut m, a, b) = two();
    let pa = m.add_prototype(&[a.volume]).unwrap();
    let pb = m.add_prototype(&[b.volume]).unwrap();
    let lat = m
        .add_lattice(Lattice::new(Layout::Cartesian { n: [2, 1, 1], pitch: Vector3::new(2.0, 0.0, 0.0) }, Transform::IDENTITY, Fill::Prototype(pa), BTreeMap::new()))
        .unwrap();
    let i = m.add_instance(inst_row(Fill::Lattice(lat))).unwrap();
    // Referenced rows cannot go.
    assert!(matches!(m.remove_prototype(pa), Err(ModelError::InUse(x)) if x == pa));
    assert!(matches!(m.remove_lattice(lat), Err(ModelError::InUse(x)) if x == lat));
    // Replace the instance row: same id, new fill.
    m.set_instance(i, inst_row(Fill::Prototype(pb))).unwrap();
    assert_eq!(m.instancing().instances[&i].of, Fill::Prototype(pb));
    assert!(m.set_instance(i, inst_row(Fill::Lattice(EntityId::new(sm_core::EntityKind::Lattice, 77).unwrap()))).is_err());
    assert!(matches!(m.set_instance(EntityId::new(sm_core::EntityKind::Instance, 77).unwrap(), inst_row(Fill::Void)), Err(ModelError::Unknown(_))));
    // Now the lattice is free; its outer fill still holds pa.
    m.remove_lattice(lat).unwrap();
    m.remove_prototype(pa).unwrap();
    assert!(matches!(m.remove_prototype(pb), Err(ModelError::InUse(_))));
    // A path override names the instance.
    m.add_path_override(PathOverride::new([PathStep::Instance(i)].into_iter().collect(), ColorMap::default())).unwrap();
    assert!(matches!(m.remove_instance(i), Err(ModelError::InUse(x)) if x == i));
    m.check_integrity().unwrap();
    let mut m2 = Model::new();
    let p = m2.add_prototype(&[a.volume]).unwrap();
    let i2 = m2.add_instance(inst_row(Fill::Prototype(p))).unwrap();
    m2.remove_instance(i2).unwrap();
    m2.remove_prototype(p).unwrap();
    assert!(m2.instancing().instances.is_empty() && m2.instancing().prototypes.is_empty());
    m2.check_integrity().unwrap();
}

#[test]
fn groups_and_slot_overrides_pin_lattices_and_prototypes() {
    let (_t, mut m, a, _) = two();
    let pa = m.add_prototype(&[a.volume]).unwrap();
    let lat = m
        .add_lattice(Lattice::new(Layout::Cartesian { n: [1, 1, 1], pitch: Vector3::new(1.0, 1.0, 1.0) }, Transform::IDENTITY, Fill::Void, BTreeMap::new()))
        .unwrap();
    let g = m.add_group("protos", &[pa]).unwrap();
    assert!(matches!(m.remove_prototype(pa), Err(ModelError::InUse(_))));
    m.remove_group(g).unwrap();
    m.add_path_override(PathOverride::new([PathStep::Slot { lattice: lat, ix: SlotIx::new(0, 0, 0) }].into_iter().collect(), ColorMap::default())).unwrap();
    assert!(matches!(m.remove_lattice(lat), Err(ModelError::InUse(_))));
    m.remove_prototype(pa).unwrap();
}

// ------------------------------------------------------------------ clips

#[test]
fn clips_are_kind_checked_validated_and_flattened() {
    let mut t = CellComplex::new();
    let pin = brick(&mut t, 1, [0.0; 3]);
    let barrel = brick(&mut t, 2, [-10.0, -10.0, 0.0]);
    let cell = brick(&mut t, 3, [-0.5, -0.5, 0.0]);
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[pin.volume, barrel.volume, cell.volume]).unwrap();
    let p = m.add_prototype(&[pin.volume]).unwrap();
    // A clip must be a volume.
    assert!(matches!(m.add_instance(inst_row(Fill::Prototype(p)).with_clip(pin.faces[0])), Err(ModelError::WrongKind { .. })));
    let slot_ix = SlotIx::new(1, 0, 0);
    let slots = BTreeMap::from([(slot_ix, Slot::new(Fill::Prototype(p)).with_clip(cell.volume))]);
    let origin = Transform::translation(Vector3::new(0.0, 0.0, 3.0));
    let lat = m.add_lattice(Lattice::new(Layout::Cartesian { n: [2, 1, 1], pitch: Vector3::new(2.0, 2.0, 1.0) }, origin, Fill::Prototype(p), slots)).unwrap();
    let bad_slot = Slot::new(Fill::Prototype(p)).with_clip(pin.edges[0]);
    assert!(m.set_lattice_slot(lat, slot_ix, bad_slot).is_err());
    let at = Transform::translation(Vector3::new(100.0, 0.0, 0.0));
    let i = m.add_instance(InstanceRow::new(Fill::Lattice(lat), at).with_clip(barrel.volume)).unwrap();
    m.check_integrity().unwrap();
    let flat = flatten(&m).unwrap();
    assert_eq!(flat.entries.len(), 2);
    // The instance clip sits in the model frame; the slot clip in the lattice frame, nested in it.
    assert_eq!(flat.clips.len(), 2);
    assert_eq!(flat.clips[0], FlatClip::new(barrel.volume, Placement::identity(), None));
    assert_eq!(flat.clips[1].volume, cell.volume);
    assert_eq!(flat.clips[1].outer, Some(0));
    assert_eq!(flat.clips[1].placement, Placement::of(at.compose(&origin)));
    let plain = flat.find(&[PathStep::Instance(i), PathStep::Slot { lattice: lat, ix: SlotIx::new(0, 0, 0) }]).unwrap();
    let clipped = flat.find(&[PathStep::Instance(i), PathStep::Slot { lattice: lat, ix: slot_ix }]).unwrap();
    assert_eq!(flat.entries[plain.start].clip, Some(0));
    assert_eq!(flat.entries[clipped.start].clip, Some(1));
    // Clips enter the fingerprint.
    let mut unclipped = m.clone();
    unclipped.set_instance(i, InstanceRow::new(Fill::Lattice(lat), at)).unwrap();
    assert_ne!(flatten(&unclipped).unwrap().fingerprint(), flat.fingerprint());
    // validate: a dead clip volume is reported.
    assert!(validate(&m, &t, Tolerance::default()).with(IssueCode::UnknownEntity).next().is_none());
    with_op(&mut t, OpId(4), |t| {
        sm_topo_api::TopologyEdit::kill_cascade(t, barrel.volume)?;
        Ok(())
    })
    .unwrap();
    let r = validate(&m, &t, Tolerance::default());
    assert!(r.with(IssueCode::UnknownEntity).any(|x| x.entities.contains(&barrel.volume)), "{r:?}");
}

#[cfg(feature = "serde")]
#[test]
fn clip_fields_default_when_absent() {
    let json = serde_json::json!({ "of": "Void", "at": Transform::IDENTITY, "recolor": ColorMap::default() });
    let row: InstanceRow = serde_json::from_value(json).unwrap();
    assert_eq!(row.clip, None);
    let s: Slot = serde_json::from_value(serde_json::json!({ "fill": "Void", "recolor": ColorMap::default() })).unwrap();
    assert_eq!(s.clip, None);
}

// ------------------------------------------------------------------ apply_transform

#[test]
fn apply_transform_maps_decks_of_moved_volumes_and_refuses_straddles() {
    let (t, mut m, a, b) = two();
    let px = m.add_boundary("px", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(1.0, 0.0, 0.0) })).unwrap();
    m.assign_pairs(px, &[(a.faces[0], a.faces[1])]).unwrap();
    let mz = m.add_boundary("mz", BoundaryKind::Deck(Deck::Mirror { plane: Plane::new(Vector3::new(0.0, 0.0, 1.0), 0.0).unwrap() })).unwrap();
    m.assign_self_paired(mz, &[a.faces[4]]).unwrap();
    let rb = m.add_boundary("rb", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(0.0, 1.0, 0.0) })).unwrap();
    m.assign_pairs(rb, &[(b.faces[2], b.faces[3])]).unwrap();
    let slide = m.add_boundary("slide", BoundaryKind::Interface(InterfaceIntent::non_conformal(Some(Transform::translation(Vector3::new(0.0, 0.0, 1.0)))))).unwrap();
    m.assign_interface(slide, &[a.faces[2]], &[a.faces[3]]).unwrap();
    // A quarter turn about z through the origin, then up by 2.
    let x = Transform::translation(Vector3::new(0.0, 0.0, 2.0)).compose(&Transform::rotation(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_2).unwrap());
    m.apply_transform(&x, &[a.volume], &t).unwrap();
    let BoundaryKind::Deck(Deck::Translation { vector }) = m.boundary(px).unwrap().kind else { panic!() };
    assert!(close(vector, Vector3::new(0.0, 1.0, 0.0)), "{vector:?}");
    let BoundaryKind::Deck(Deck::Mirror { plane }) = m.boundary(mz).unwrap().kind else { panic!() };
    assert!(close(plane.normal, Vector3::new(0.0, 0.0, 1.0)) && (plane.offset - 2.0).abs() < 1e-12, "{plane:?}");
    let BoundaryKind::Deck(Deck::Translation { vector }) = m.boundary(rb).unwrap().kind else { panic!() };
    assert_eq!(vector, Vector3::new(0.0, 1.0, 0.0), "B did not move");
    let BoundaryKind::Interface(InterfaceIntent::NonConformal { transform: Some(tr), .. }) = m.boundary(slide).unwrap().kind else { panic!() };
    assert!(close(tr.t, Vector3::new(0.0, 0.0, 1.0)), "a pure z translation commutes with the motion");
    // A record with faces on both bricks straddles; nothing changes.
    let both = m.add_boundary("both", BoundaryKind::Response).unwrap();
    m.assign_response(both, &[a.faces[5], b.faces[5]]).unwrap();
    let before = m.clone();
    assert!(matches!(m.apply_transform(&x, &[a.volume], &t), Err(ModelError::Straddles(id)) if id == both));
    assert_eq!(m, before);
    // Singular maps and non-volumes are refused.
    let zero = Transform { m: [[0.0; 3]; 3], t: Vector3::new(0.0, 0.0, 0.0) };
    assert!(matches!(m.apply_transform(&zero, &[a.volume], &t), Err(ModelError::Singular)));
    assert!(matches!(m.apply_transform(&x, &[a.faces[0]], &t), Err(ModelError::WrongKind { .. })));
}

#[test]
fn mirror_decks_need_a_similarity_and_rotations_flip_under_reflection() {
    let (t, mut m, a, _) = two();
    let axis = Axis::new(Point3::new(0.5, 0.5, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
    let r = m.add_boundary("r", BoundaryKind::Deck(Deck::rotation(axis, Turns::new(1, 4).unwrap()))).unwrap();
    m.assign_pairs(r, &[(a.faces[0], a.faces[2])]).unwrap();
    let stretch = Transform { m: [[2.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t: Vector3::new(0.0, 0.0, 0.0) };
    assert!(matches!(m.apply_transform(&stretch, &[a.volume], &t), Err(ModelError::Invalid(_))));
    let flip = Transform::reflection(Vector3::new(1.0, 0.0, 0.0), 0.0).unwrap();
    m.apply_transform(&flip, &[a.volume], &t).unwrap();
    let BoundaryKind::Deck(Deck::Rotation { axis, angle, .. }) = m.boundary(r).unwrap().kind else { panic!() };
    assert!(close(axis.point - Point3::new(-0.5, 0.5, 0.0), Vector3::new(0.0, 0.0, 0.0)));
    assert!(close(axis.dir, Vector3::new(0.0, 0.0, -1.0)), "conjugation by a reflection reverses the sense");
    assert_eq!(angle, Turns::new(1, 4).unwrap());
}

#[test]
fn apply_transform_maps_cut_planes_of_moved_volumes() {
    let (t, mut m, a, b) = two();
    let faces: Vec<EntityId> = a.faces.to_vec();
    let cut = CutGeometry::Plane(Plane::new(Vector3::new(1.0, 0.0, 0.0), 0.5).unwrap());
    m.split(PieceRef::Whole(a.volume), cut, &faces, &faces, &t).unwrap();
    let bf: Vec<EntityId> = b.faces.to_vec();
    m.split(PieceRef::Whole(b.volume), CutGeometry::Plane(Plane::new(Vector3::new(1.0, 0.0, 0.0), 5.5).unwrap()), &bf, &bf, &t).unwrap();
    m.apply_transform(&Transform::translation(Vector3::new(3.0, 0.0, 0.0)), &[a.volume], &t).unwrap();
    let CutGeometry::Plane(pa) = &m.decomposition().split_of(PieceRef::Whole(a.volume)).unwrap().geometry else { panic!() };
    assert!((pa.offset - 3.5).abs() < 1e-12);
    let CutGeometry::Plane(pb) = &m.decomposition().split_of(PieceRef::Whole(b.volume)).unwrap().geometry else { panic!() };
    assert_eq!(pb.offset, 5.5, "B's cut stays");
}

#[test]
fn symmetry_moves_only_with_the_whole_model() {
    let mut t = CellComplex::new();
    let a = brick(&mut t, 1, [0.0; 3]);
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[a.volume]).unwrap();
    let px = m.add_boundary("px", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(1.0, 0.0, 0.0) })).unwrap();
    m.assign_pairs(px, &[(a.faces[0], a.faces[1])]).unwrap();
    m.declare_symmetry(&[px], &t).unwrap();
    // Whole model: the generator follows.
    m.apply_transform(&Transform::rotation(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_2).unwrap(), &[a.volume], &t).unwrap();
    let BoundaryKind::Deck(Deck::Translation { vector }) = m.boundary(px).unwrap().kind else { panic!() };
    assert!(close(vector, Vector3::new(0.0, 1.0, 0.0)));
    // Add a second volume: moving only the first touches the symmetry.
    let b = brick(&mut t, 2, [5.0, 0.0, 0.0]);
    m.color_volumes(mat, &[b.volume]).unwrap();
    let before = m.clone();
    assert!(matches!(m.apply_transform(&Transform::translation(Vector3::new(0.0, 0.0, 1.0)), &[a.volume], &t), Err(ModelError::SymmetryStraddles)));
    assert_eq!(m, before);
    // Moving only the other one is fine.
    m.apply_transform(&Transform::translation(Vector3::new(0.0, 0.0, 1.0)), &[b.volume], &t).unwrap();
    finish_boundaries(&mut m, &t, 9, true);
}
