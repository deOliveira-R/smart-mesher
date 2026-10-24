//! W0g freeze: constructors of the non-exhaustive structs and golden serde shapes.

use sm_core::{EntityId, EntityKind};
use sm_mesh_api::*;

fn id(kind: EntityKind, s: u64) -> EntityId {
    EntityId::new(kind, s).unwrap()
}

#[test]
fn constructors_fill_every_field() {
    let s = MeshSet::new(id(EntityKind::MeshSet, 1), "inlet", SetMembers::Faces(vec![FaceIx(2)])).with_label(7);
    assert_eq!((s.name.as_str(), s.label), ("inlet", Some(7)));
    let t = TagColumn::new("mat", MeshEntityKind::Cell, TagValues::I64(vec![1, 2]));
    assert_eq!((t.name.as_str(), t.target, t.values.len()), ("mat", MeshEntityKind::Cell, 2));
    let (x, y, z) = ([0.0, 1.0], [2.0, 3.0], [4.0, 5.0]);
    let c = Coords::new(&x, &y, &z);
    assert_eq!(c.len(), 2);
    assert_eq!(c.get(1).x, 1.0);
    let g = GeometryCache::new("r", 1, 2);
    assert!(g.rule == "r" && g.connectivity_hash == 1 && g.coordinate_hash == 2 && g.face_area.is_empty());
    let o = FlattenOptions::new().with_weld_tol(1e-9);
    assert_eq!(o.weld_tol, Some(1e-9));
    assert!(o.remap_class.is_none() && o.on_merge.is_none());
    let n = LossNote::new(id(EntityKind::MeshInstance, 1), LossKind::EdgeTable);
    assert_eq!(n.what, LossKind::EdgeTable);
    let f = Finished::new(3u8, vec![FaceIx(0)]);
    assert_eq!((f.mesh, f.face_remap.len()), (3, 1));
}

#[cfg(feature = "serde")]
#[test]
fn golden_serde_shapes() {
    use serde_json::json;
    let s = MeshSet::new(id(EntityKind::MeshSet, 1), "s", SetMembers::Cells(vec![CellIx(0)]));
    let v = serde_json::to_value(&s).unwrap();
    assert_eq!(v["name"], json!("s"));
    assert_eq!(v["label"], json!(null));
    assert_eq!(v["members"], json!({"Cells": [0]}));
    // `label` is optional on read (serde(default)).
    let mut old = v.clone();
    old.as_object_mut().unwrap().remove("label");
    assert_eq!(serde_json::from_value::<MeshSet>(old).unwrap(), s);
    assert_eq!(serde_json::to_value(CellShape::Hex).unwrap(), json!("Hex"));
    assert_eq!(serde_json::to_value(FaceKind::Boundary).unwrap(), json!("Boundary"));
    assert_eq!(serde_json::to_value(TagValues::I64(vec![1])).unwrap(), json!({"I64": [1]}));
}
