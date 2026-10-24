//! W0g freeze: constructors of the non-exhaustive rows and variants, golden serde shapes.
#![cfg(feature = "serde")]

use serde_json::json;
use sm_core::{EntityId, EntityKind, Point3, Transform, Vector3};
use sm_model_api::*;
use std::collections::BTreeSet;

fn id(k: EntityKind, n: u64) -> EntityId {
    EntityId::new(k, n).unwrap()
}

fn v<T: serde::Serialize>(x: &T) -> serde_json::Value {
    serde_json::to_value(x).unwrap()
}

#[test]
fn variant_constructors_match_the_variants() {
    let axis = Axis::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
    let angle = Turns::new(1, 4).unwrap();
    let d = Deck::rotation(axis, angle);
    assert!(matches!(d, Deck::Rotation { axis: a, angle: t, .. } if a == axis && t == angle));
    assert_eq!(d.order(), Some(4));
    let i = InterfaceIntent::non_conformal(None);
    assert!(matches!(i, InterfaceIntent::NonConformal { transform: None, .. }));
    let f = id(EntityKind::TopoFace, 1);
    let s = PieceShape::sweep(vec![PieceFace::Whole(f)], vec![]);
    assert!(matches!(&s, PieceShape::Sweep { source, target, .. } if source.len() == 1 && target.is_empty()));
}

#[test]
fn golden_serde_shapes() {
    let axis = Axis::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
    let d = Deck::rotation(axis, Turns::new(1, 6).unwrap());
    let val = v(&d);
    assert_eq!(val["Rotation"]["angle"], json!({"num": 1, "den": 6}));
    assert_eq!(serde_json::from_value::<Deck>(val).unwrap(), d);
    assert_eq!(v(&InterfaceIntent::non_conformal(None)), json!({"NonConformal": {"transform": null}}));
    assert_eq!(v(&InterfaceIntent::Conformal), json!("Conformal"));
    assert_eq!(v(&PieceShape::Map), json!("Map"));
    let f = id(EntityKind::TopoFace, 1);
    assert_eq!(v(&PieceShape::sweep(vec![PieceFace::Whole(f)], vec![])), json!({"Sweep": {"source": [{"Whole": v(&f)}], "target": []}}));
}

#[test]
fn row_constructors_fill_defaults() {
    let p = id(EntityKind::Prototype, 1);
    let r = InstanceRow::new(Fill::Prototype(p), Transform::IDENTITY);
    assert_eq!((r.of, r.clip), (Fill::Prototype(p), None));
    assert_eq!(r.recolor, ColorMap::default());
    let c = id(EntityKind::TopoVolume, 9);
    assert_eq!(Slot::new(Fill::Void).with_clip(c).clip, Some(c));
    // Optional row fields stay optional on read.
    let mut val = v(&r);
    val.as_object_mut().unwrap().remove("clip");
    assert_eq!(serde_json::from_value::<InstanceRow>(val).unwrap(), r);
    let pr = Prototype::new(BTreeSet::from([c]));
    assert_eq!(pr.volumes.len(), 1);
    let b = BoundaryRow::new(id(EntityKind::Boundary, 1), "inlet", BoundaryKind::Response);
    assert_eq!(b.name, "inlet");
    let rep = Report::new(Vec::new());
    assert!(rep.is_ok());
}

#[test]
fn color_domain_and_color_map_golden() {
    use std::collections::BTreeMap;
    assert_eq!(v(&Domain::Volume), json!("Volume"));
    assert_eq!(v(&Domain::Face), json!("Face"));
    let row = ColorRow::new(id(EntityKind::Color, 1), 3, "fuel", Domain::Face);
    assert_eq!(serde_json::from_value::<ColorRow>(v(&row)).unwrap(), row);
    let (vol, c1, c2, f, fc) = (id(EntityKind::TopoVolume, 1), id(EntityKind::Color, 1), id(EntityKind::Color, 2), id(EntityKind::TopoFace, 1), id(EntityKind::Color, 3));
    assert_eq!(ColorMap::new(), ColorMap::default());
    assert!(ColorMap::new().is_empty());
    let m = ColorMap::new().with_volume(BTreeMap::from([(vol, c1), (c1, c2)])).with_face(BTreeMap::from([(f, fc)]));
    assert_eq!((m.volume.len(), m.face.len()), (2, 1));
    assert_eq!(m.apply_volume(vol, c2), c1);
    // Serde shape (persisted in every recolor map) is unchanged by the attribute.
    let mv = v(&m);
    assert_eq!(mv.as_object().unwrap().keys().collect::<Vec<_>>(), ["face", "volume"]);
    assert_eq!(serde_json::from_value::<ColorMap>(mv).unwrap(), m);
    assert_eq!(v(&ColorMap::new()), json!({"volume": {}, "face": {}}));
}
