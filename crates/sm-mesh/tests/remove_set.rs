//! `MeshEdit::remove_set`: add then remove restores content, set list and hashes.

use sm_core::EntityKind;
use sm_mesh::generate::hex_block_bulk;
use sm_mesh_api::*;

#[test]
fn add_then_remove_restores_the_mesh() {
    let mut alloc = SerialAlloc::new();
    let mut m = hex_block_bulk([2, 2, 2], [1.0; 3], &mut alloc).unwrap().mesh;
    let before = m.clone();
    let (ct, co) = (connectivity_hash(&m), coordinate_hash(&m));
    let gh = geometry_hash(&m);
    let alloc_before: Vec<_> = alloc.state().collect();

    let id = MeshEdit::add_set(&mut m, "s", SetMembers::Cells(vec![CellIx(1), CellIx(3)]), &mut alloc).unwrap();
    assert_eq!(id.kind(), EntityKind::MeshSet);
    assert_eq!(m.sets().len(), 1);
    MeshEdit::remove_set(&mut m, id).unwrap();

    assert_eq!(m, before, "content incl. set list");
    assert!(m.sets().is_empty());
    assert_eq!((connectivity_hash(&m), coordinate_hash(&m), geometry_hash(&m)), (ct, co, gh));
    // The allocator keeps its counter (serials are never reused); the mesh holds no trace of it.
    assert_ne!(alloc.state().collect::<Vec<_>>(), alloc_before);
    // The name is free again, and the removed id is unknown.
    MeshEdit::add_set(&mut m, "s", SetMembers::Cells(vec![CellIx(0)]), &mut alloc).unwrap();
    assert!(MeshEdit::remove_set(&mut m, id).is_err());
}

#[test]
fn remove_keeps_other_sets() {
    let mut alloc = SerialAlloc::new();
    let mut m = hex_block_bulk([2, 1, 1], [1.0; 3], &mut alloc).unwrap().mesh;
    let a = MeshEdit::add_set(&mut m, "a", SetMembers::Cells(vec![CellIx(0)]), &mut alloc).unwrap();
    let with_a = m.clone();
    let b = MeshEdit::add_set(&mut m, "b", SetMembers::Cells(vec![CellIx(1)]), &mut alloc).unwrap();
    MeshEdit::remove_set(&mut m, b).unwrap();
    assert_eq!(m, with_a);
    MeshEdit::remove_set(&mut m, a).unwrap();
    assert!(m.sets().is_empty());
}
