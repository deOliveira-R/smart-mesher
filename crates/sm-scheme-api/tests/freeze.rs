//! W0g freeze: constructors of the non-exhaustive types and golden serde shapes.

use serde_json::json;
use sm_core::{EntityId, EntityKind};
use sm_scheme_api::*;

fn id(k: EntityKind, n: u64) -> EntityId {
    EntityId::new(k, n).expect("id")
}

fn v<T: serde::Serialize>(x: &T) -> serde_json::Value {
    serde_json::to_value(x).unwrap()
}

#[test]
fn intent_constructors_and_golden_json() {
    let e = id(EntityKind::TopoEdge, 2);
    let g = MeshGroup::new(GroupId(1), vec![id(EntityKind::TopoVolume, 1)]).with_scheme(SchemeChoice::new("msweep", Firmness::Hard));
    let mut i = MeshIntent::new();
    i.entities.insert(e, EntityIntent::new().with_intervals(Firm::hard(IntervalRequest::Count(3))));
    i.groups.push(g);
    i.defaults = Defaults::new().with_sizing(SizingSpec::Constant { size: 0.5 });
    let val = v(&i);
    let ek = v(&e).as_str().map(str::to_owned).unwrap_or_else(|| v(&e).to_string());
    assert_eq!(val["entities"][&ek], json!({"intervals": {"value": {"Count": 3}, "firmness": "Hard"}}));
    // Optional group fields are omitted when empty.
    assert_eq!(val["groups"][0], json!({"id": 1, "members": [v(&id(EntityKind::TopoVolume, 1))], "scheme": {"name": "msweep", "firmness": "Hard"}}));
    assert_eq!(val["defaults"], json!({"sizing": {"Constant": {"size": 0.5}}, "auto": {"curve": [], "surface": [], "volume": []}}));
    let back: MeshIntent = serde_json::from_value(val).unwrap();
    assert_eq!(back, i);
    // Every intent field is optional on read.
    assert_eq!(serde_json::from_value::<MeshIntent>(json!({})).unwrap(), MeshIntent::new());
    assert_eq!(serde_json::from_value::<EntityIntent>(json!({})).unwrap(), EntityIntent::new());
}

#[test]
fn enum_shapes_are_unchanged() {
    assert_eq!(v(&FailureKind::Quality), json!("Quality"));
    assert_eq!(v(&Phase::Generate), json!("Generate"));
    let f = id(EntityKind::TopoFace, 3);
    assert_eq!(v(&Target::Surface(f)), json!({"Surface": v(&f)}));
    assert_eq!(v(&Job::Generate(Target::Surface(f))), json!({"Generate": {"Surface": v(&f)}}));
    assert_eq!(v(&VertexRule::Ends(3)), json!({"Ends": 3}));
    assert_eq!(v(&IntervalRequest::Size(0.25)), json!({"Size": 0.25}));
}

#[test]
fn implementer_results_have_constructors() {
    let t = Target::Surface(id(EntityKind::TopoFace, 1));
    let a = Attempt::new(t, SchemeName::new("map"), Phase::Resolve, FailureKind::NotApplicable, "3 sides");
    assert_eq!((a.target, a.scheme.0.as_str(), a.phase, a.kind, a.message.as_str()), (t, "map", Phase::Resolve, FailureKind::NotApplicable, "3 sides"));
    let e1 = id(EntityKind::TopoEdge, 1);
    let e2 = id(EntityKind::TopoEdge, 2);
    let inf = Infeasible::new("odd", vec![ConstraintId(2), ConstraintId(1), ConstraintId(2)], vec![e2, e1]);
    assert_eq!(inf.constraints, vec![ConstraintId(1), ConstraintId(2)]);
    assert_eq!(inf.entities, vec![e1, e2]);
    assert!(inf.blame.is_none());
    let o = DriverOptions::new().with_max_replans(3).with_classify_edges(true).with_copy_tolerance(1e-6);
    assert_eq!((o.max_replans, o.classify_edges, o.copy_tolerance), (3, true, Some(1e-6)));
    let d = DriverOptions::default();
    assert_eq!((d.max_replans, d.classify_edges, d.copy_tolerance), (16, false, None));
    assert!(matches!(Applicability::yes(10, "ok"), Applicability::Applicable { score: 10, .. }));
}

/// W0g r2: implementer-built `Patch`, `PatchNode`, `FaceLayout`, `LogicalSide`.
#[test]
fn layout_and_patch_constructors_and_golden_json() {
    use sm_topo_api::{CoedgeKey, CornerKey, Side};
    let f = id(EntityKind::TopoFace, 1);
    let e = id(EntityKind::TopoEdge, 2);
    let ce = CoedgeKey { face: f, edge: e, side: Side::Left };
    let side = LogicalSide::new(vec![ce], SideDir::PlusI);
    assert_eq!((side.coedges.as_slice(), side.dir), (&[ce][..], SideDir::PlusI));
    let corner = CornerAssignment { corner: CornerKey { face: f, incoming: ce }, vertex_type: VertexType::End, firmness: Firmness::Default, angle: 1.5 };
    let layout = FaceLayout::new(f, VertexRule::Map, vec![corner], vec![side.clone()]);
    assert_eq!((layout.face, layout.rule, layout.corners.len(), layout.sides.len()), (f, VertexRule::Map, 1, 1));
    // Serde shapes are unchanged by the attribute.
    assert_eq!(v(&side), json!({"coedges": [v(&ce)], "dir": "PlusI"}));
    let lv = v(&layout);
    assert_eq!(lv.as_object().unwrap().keys().collect::<Vec<_>>(), ["corners", "face", "rule", "sides"]);
    assert_eq!(serde_json::from_value::<FaceLayout>(lv).unwrap(), layout);
    let key = NodeKey::new(f, 0);
    let n = PatchNode::new(key, sm_core::Point3::new(1.0, 2.0, 3.0), sm_mesh_api::Param::None);
    assert_eq!((n.key, n.point), (key, sm_core::Point3::new(1.0, 2.0, 3.0)));
    let mut p = Patch::new(Target::Surface(f));
    p.nodes.push(n);
    assert_eq!(p.local_len(), 1);
}
