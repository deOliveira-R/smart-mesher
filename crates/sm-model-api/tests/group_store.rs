//! The model is the single group store (W0f): edits, rename/remove, lookups, group algebra,
//! wider member kinds, lineage and successor re-keying, rank expansion, integrity and serde.

use sm_core::{EntityId, EntityKind, Point3};
use sm_model_api::*;
use sm_topo_api::fixtures::{self, with_op};
use sm_topo_api::{CellComplex, Dim, OpId, TopologyView};
use std::collections::BTreeMap;

fn brick() -> (CellComplex, fixtures::BrickIds) {
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])).unwrap();
    (t, b)
}

#[test]
fn edit_rename_remove_and_lookup() {
    let (_, b) = brick();
    let mut m = Model::new();
    let g = m.add_group("g", &[b.faces[0]]).unwrap();
    assert_eq!(g.kind(), EntityKind::Group);
    m.group_edit(g, GroupEdit::Add, &[b.faces[1], b.edges[0], b.vertices[0]]).unwrap();
    assert_eq!(m.group(g).unwrap().members.len(), 4);
    m.group_edit(g, GroupEdit::Remove, &[b.faces[1], b.faces[5]]).unwrap();
    assert_eq!(m.group(g).unwrap().members.len(), 3);
    m.group_edit(g, GroupEdit::Xor, &[b.faces[0], b.faces[2]]).unwrap();
    assert!(!m.group(g).unwrap().members.contains(&b.faces[0]));
    assert!(m.group(g).unwrap().members.contains(&b.faces[2]));
    m.group_edit(g, GroupEdit::Equals, &[b.volume]).unwrap();
    assert_eq!(m.group(g).unwrap().members.iter().copied().collect::<Vec<_>>(), vec![b.volume]);
    // Rejected members leave the group unchanged: a curve handle, a mesh entity, an unknown color.
    let before = m.group(g).unwrap().clone();
    let curve = EntityId::new(EntityKind::Curve, 1).unwrap();
    assert!(m.group_edit(g, GroupEdit::Add, &[b.faces[3], curve]).is_err());
    assert!(m.group_edit(g, GroupEdit::Add, &[EntityId::new(EntityKind::MeshCell, 1).unwrap()]).is_err());
    assert!(matches!(m.group_edit(g, GroupEdit::Add, &[EntityId::new(EntityKind::Color, 9).unwrap()]), Err(ModelError::Unknown(_))));
    assert_eq!(m.group(g).unwrap(), &before);
    // Partition entities are allowed (minted pieces must exist; overlay cells are not checked here).
    m.group_edit(g, GroupEdit::Add, &[EntityId::new(EntityKind::PartitionFace, 3).unwrap()]).unwrap();
    assert!(matches!(m.group_edit(g, GroupEdit::Add, &[EntityId::new(EntityKind::PartitionCell, 3).unwrap()]), Err(ModelError::Unknown(_))));
    // Names.
    let h = m.add_group("h", &[g]).unwrap();
    assert_eq!(m.group_by_name("h"), Some(h));
    assert!(matches!(m.rename_group(h, "g"), Err(ModelError::NameTaken(_))));
    m.rename_group(h, "outer").unwrap();
    assert_eq!(m.group_by_name("outer"), Some(h));
    assert_eq!(m.group_by_name("h"), None);
    // Cycles are refused.
    assert!(matches!(m.group_edit(g, GroupEdit::Add, &[h]), Err(ModelError::Cycle(_))));
    assert!(matches!(m.group_edit(g, GroupEdit::Add, &[g]), Err(ModelError::Cycle(_))));
    // A nested group cannot be removed while referenced.
    assert!(m.remove_group(g).is_err());
    m.remove_group(h).unwrap();
    m.remove_group(g).unwrap();
    assert_eq!(m.groups().count(), 0);
    assert!(m.remove_group(g).is_err());
}

#[test]
fn unite_intersect_subtract_mint_new_groups() {
    let (_, b) = brick();
    let mut m = Model::new();
    let a = m.add_group("a", &[b.faces[0], b.faces[1], b.faces[2]]).unwrap();
    let c = m.add_group("c", &[b.faces[2], b.faces[3]]).unwrap();
    let u = m.group_unite("u", a, c).unwrap();
    let i = m.group_intersect("i", a, c).unwrap();
    let s = m.group_subtract("s", a, c).unwrap();
    let mem = |m: &Model, g| m.group(g).unwrap().members.iter().copied().collect::<Vec<_>>();
    assert_eq!(mem(&m, u), vec![b.faces[0], b.faces[1], b.faces[2], b.faces[3]]);
    assert_eq!(mem(&m, i), vec![b.faces[2]]);
    assert_eq!(mem(&m, s), vec![b.faces[0], b.faces[1]]);
    assert!(u > c && i > u && s > i, "new ids are minted in order");
    assert!(matches!(m.group_unite("u", a, c), Err(ModelError::NameTaken(_))));
}

#[test]
fn group_entities_expands_by_closure() {
    let (t, b) = brick();
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[b.volume]).unwrap();
    let g = m.add_group("g", &[b.faces[0]]).unwrap();
    // A face brings its 4 edges and 4 vertices.
    assert_eq!(m.group_entities(g, &t, Dim::Face).unwrap(), vec![b.faces[0]]);
    assert_eq!(m.group_entities(g, &t, Dim::Edge).unwrap().len(), 4);
    assert_eq!(m.group_entities(g, &t, Dim::Vertex).unwrap().len(), 4);
    assert!(m.group_entities(g, &t, Dim::Volume).unwrap().is_empty(), "no upward expansion");
    // A nested color group brings the whole brick; a body too.
    let h = m.add_group("h", &[mat]).unwrap();
    let outer = m.add_group("outer", &[h, g]).unwrap();
    assert_eq!(m.group_entities(outer, &t, Dim::Volume).unwrap(), vec![b.volume]);
    assert_eq!(m.group_entities(outer, &t, Dim::Face).unwrap().len(), 6);
    assert_eq!(m.group_entities(outer, &t, Dim::Edge).unwrap().len(), 12);
    let body = m.add_group("body", &[b.body]).unwrap();
    assert_eq!(m.group_entities(body, &t, Dim::Vertex).unwrap().len(), 8);
    // Edge and vertex members of their own rank.
    let ev = m.add_group("ev", &[b.edges[3], b.vertices[7]]).unwrap();
    assert_eq!(m.group_entities(ev, &t, Dim::Edge).unwrap(), vec![b.edges[3]]);
    // Edge 3 runs from corner 6 to corner 7, so vertex 7 is counted once.
    assert_eq!(m.group_entities(ev, &t, Dim::Vertex).unwrap(), vec![b.vertices[6], b.vertices[7]]);
}

#[test]
fn lineage_and_successors_rekey_every_topological_member() {
    let (mut t, b) = brick();
    let mut m = Model::new();
    let g = m.add_group("g", &[b.edges[8], b.faces[0], b.vertices[0]]).unwrap();
    let w = with_op(&mut t, OpId(2), |t| fixtures::webcut_brick(t, &b, 0.5)).unwrap();
    let lineage = t.lineage().cut_outcome(OpId(2)).lineage;
    m.apply_lineage(&lineage, &t).unwrap();
    let mem = m.group(g).unwrap().members.clone();
    // The vertical edge split in two; the -x face split in two; the vertex is untouched.
    assert!(mem.contains(&w.split_edges[0][0]) && mem.contains(&w.split_edges[0][1]), "{mem:?}");
    assert!(!mem.contains(&b.edges[8]) && !mem.contains(&b.faces[0]));
    assert!(mem.contains(&w.split_faces[0][0]) && mem.contains(&w.split_faces[0][1]));
    assert!(mem.contains(&b.vertices[0]));
    // The succession hook path: a partition face copied, another removed, a color untouched.
    let pf = |s| EntityId::new(EntityKind::PartitionFace, s).unwrap();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    let h = m.add_group("h", &[pf(1), pf(2), mat]).unwrap();
    assert!(m.groups_reference_any(&[pf(1)]));
    assert!(!m.groups_reference_any(&[pf(9)]));
    let succ = BTreeMap::from([(pf(1), vec![pf(1), pf(5)]), (pf(2), vec![])]);
    m.apply_group_successors(&succ);
    assert_eq!(m.group(h).unwrap().members.iter().copied().collect::<Vec<_>>(), vec![mat, pf(1), pf(5)]);
    m.check_integrity().unwrap();
}

#[cfg(feature = "serde")]
#[test]
fn groups_round_trip_and_integrity_rejects_bad_members() {
    let (_, b) = brick();
    let mut m = Model::new();
    let g = m.add_group("g", &[b.faces[0], b.edges[1]]).unwrap();
    m.add_group("h", &[g]).unwrap();
    let s = serde_json::to_string(&m).unwrap();
    let back: Model = serde_json::from_str(&s).unwrap();
    assert_eq!(back, m);
    // A tampered file with a cycle or a forbidden member does not load.
    let cyc = s.replace(&format!("\"members\":[\"{}\",\"{}\"]", b.edges[1], b.faces[0]), &format!("\"members\":[\"{}\"]", EntityId::new(EntityKind::Group, 2).unwrap()));
    assert_ne!(cyc, s);
    assert!(serde_json::from_str::<Model>(&cyc).is_err());
    let bad = s.replace(&b.edges[1].to_string(), "MeshCell#1");
    assert!(serde_json::from_str::<Model>(&bad).is_err());
}

#[cfg(feature = "schemars")]
#[test]
fn model_input_types_have_schemas() {
    let _ = schemars::schema_for!(GroupEdit);
    let _ = schemars::schema_for!(Deck);
    let _ = schemars::schema_for!(InterfaceIntent);
    let _ = schemars::schema_for!(ColorRow);
    let _ = schemars::schema_for!(CutSpec);
    let _ = schemars::schema_for!(Lattice);
    let _ = schemars::schema_for!(PieceShape);
    let _ = schemars::schema_for!(Plane);
}
