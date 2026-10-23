//! Intent and plan are serde data (they live in an Op and in git).

mod stub_schemes;

use sm_core::{EntityId, EntityKind};
use sm_scheme_api::{
    CornerIntent, Driver, Firm, Firmness, GroupId, IntervalRequest, MeshGroup, MeshInput, MeshIntent, Plan, ReferenceCornerPicker, ReferenceIntervalSolver, SchemeChoice, SizingSpec, VertexType, WholeFaces,
};
use sm_topo_api::{CoedgeKey, CornerKey, Side};
use stub_schemes::{brick_setup, registry};

fn id(k: EntityKind, n: u64) -> EntityId {
    EntityId::new(k, n).expect("id")
}

#[test]
fn intent_round_trips_and_canonicalizes() {
    let f = id(EntityKind::TopoFace, 3);
    let corner = CornerKey { face: f, incoming: CoedgeKey { face: f, edge: id(EntityKind::TopoEdge, 2), side: Side::Left } };
    let mut a = MeshIntent::default();
    a.set_scheme(f, SchemeChoice::new("pave", Firmness::Soft));
    a.entity_mut(f).fallbacks = vec![SchemeChoice::new("tridelaunay", Firmness::Default), SchemeChoice::new("qtri", Firmness::Default)];
    a.set_intervals(id(EntityKind::TopoEdge, 2), Firm::hard(IntervalRequest::Count(7)));
    a.corners.push(CornerIntent { corner, vertex_type: Firm::soft(VertexType::Side) });
    a.corners.push(CornerIntent { corner, vertex_type: Firm::hard(VertexType::End) });
    a.groups.push(MeshGroup { id: GroupId(1), members: vec![id(EntityKind::TopoVolume, 2), id(EntityKind::TopoVolume, 1)], scheme: Some(SchemeChoice::new("sweep", Firmness::Hard)), conversion: Some(SchemeChoice::new("thex", Firmness::Hard)) });
    a.defaults.sizing = Some(SizingSpec::Constant { size: 0.5 });
    a.canonicalize();
    assert_eq!(a.corners.len(), 1);
    assert_eq!(a.corners[0].vertex_type, Firm::hard(VertexType::End));
    assert_eq!(a.groups[0].members[0], id(EntityKind::TopoVolume, 1));
    let text = serde_json::to_string_pretty(&a).expect("json");
    assert!(text.contains("\"TopoFace#3\""));
    let b: MeshIntent = serde_json::from_str(&text).expect("parse");
    assert_eq!(a, b);
    assert_eq!(text, serde_json::to_string_pretty(&b).expect("json"));
}

#[test]
fn plan_round_trips() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput { topo: &topo, model: &model, pieces: &WholeFaces, selection: &sel };
    let mut intent = MeshIntent::default();
    intent.set_intervals(ids.volume, Firm::soft(IntervalRequest::Count(3)));
    let plan = driver.plan(&input, &intent).expect("plan");
    let text = serde_json::to_string(&plan).expect("json");
    let back: Plan = serde_json::from_str(&text).expect("parse");
    assert_eq!(plan, back);
    assert!(plan.intervals.iter().all(|x| x.1 == 3));
}
