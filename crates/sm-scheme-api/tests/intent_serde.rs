//! Intent and plan are serde data (they live in an Op and in git), and TOML-safe through the
//! Op codec.

use sm_scheme_api::stubs as stub_schemes;

use sm_core::{EntityId, EntityKind};
use sm_scheme_api::{
    CornerIntent, Driver, Firm, Firmness, GroupId, IntervalRequest, MeshGroup, MeshInput,
    MeshIntent, Params, Plan, ReferenceCornerPicker, ReferenceIntervalSolver, SchemeChoice,
    SizingSpec, TransformId, TransformIntent, VertexType, WholeFaces,
};
use sm_topo_api::{CoedgeKey, CornerKey, Side};
use stub_schemes::{brick_setup, registry};

fn id(k: EntityKind, n: u64) -> EntityId {
    EntityId::new(k, n).expect("id")
}

fn sample_intent() -> MeshIntent {
    let f = id(EntityKind::TopoFace, 3);
    let corner = CornerKey {
        face: f,
        incoming: CoedgeKey {
            face: f,
            edge: id(EntityKind::TopoEdge, 2),
            side: Side::Left,
        },
    };
    let mut a = MeshIntent::default();
    a.set_scheme(f, SchemeChoice::new("pave", Firmness::Soft));
    a.entity_mut(f).fallbacks = vec![
        SchemeChoice::new("tridelaunay", Firmness::Default),
        SchemeChoice::new("qtri", Firmness::Default),
    ];
    a.set_intervals(
        id(EntityKind::TopoEdge, 2),
        Firm::hard(IntervalRequest::Count(7)),
    );
    a.corners
        .push(CornerIntent::new(corner, Firm::soft(VertexType::Side)));
    a.corners
        .push(CornerIntent::new(corner, Firm::hard(VertexType::End)));
    let mut g = MeshGroup::new(
        GroupId(1),
        vec![id(EntityKind::TopoVolume, 2), id(EntityKind::TopoVolume, 1)],
    )
    .with_scheme(SchemeChoice::new("sweep", Firmness::Hard))
    .with_fallbacks(vec![SchemeChoice::with_params(
        "tetmesh",
        serde_json::json!({ "growth": 1.3 }),
        Firmness::Default,
    )]);
    g.conversion = Some(SchemeChoice::new("thex", Firmness::Hard));
    a.groups.push(g);
    a.transforms.push(TransformIntent::new(
        TransformId(2),
        vec![id(EntityKind::TopoVolume, 3), id(EntityKind::TopoVolume, 3)],
        SchemeChoice::new("htet", Firmness::Hard),
    ));
    a.defaults.sizing = Some(SizingSpec::Constant { size: 0.5 });
    a
}

#[test]
fn intent_round_trips_and_canonicalizes() {
    let mut a = sample_intent();
    a.canonicalize();
    assert_eq!(a.corners.len(), 1);
    assert_eq!(a.corners[0].vertex_type, Firm::hard(VertexType::End));
    assert_eq!(a.groups[0].members[0], id(EntityKind::TopoVolume, 1));
    assert_eq!(a.transforms[0].targets.len(), 1);
    a.check().expect("valid");
    let text = serde_json::to_string_pretty(&a).expect("json");
    assert!(text.contains("\"TopoFace#3\""));
    let b: MeshIntent = serde_json::from_str(&text).expect("parse");
    assert_eq!(a, b);
    assert_eq!(text, serde_json::to_string_pretty(&b).expect("json"));
}

#[test]
fn intent_crosses_the_op_codec_and_rejects_non_toml_params() {
    let mut a = sample_intent();
    a.canonicalize();
    // The Op payload path: TOML-representable value in, the same intent out.
    let v = sm_op_api::codec::to_value(&a).expect("encode");
    let back: MeshIntent = serde_json::from_value(v).expect("decode");
    assert_eq!(a, back);
    // Empty parameters are `{}`, never null, and are not serialized.
    assert_eq!(Params::empty().0, serde_json::json!({}));
    let c = SchemeChoice::new("map", Firmness::Default);
    assert!(!serde_json::to_string(&c).expect("json").contains("params"));
    // Parameters the codec cannot hold are refused when the intent is checked.
    let mut bad = sample_intent();
    bad.set_scheme(
        id(EntityKind::TopoFace, 4),
        SchemeChoice::with_params("x", serde_json::json!({ "a": null }), Firmness::Soft),
    );
    assert!(bad.check().is_err());
    let mut bad = sample_intent();
    bad.set_scheme(
        id(EntityKind::TopoFace, 4),
        SchemeChoice::with_params("x", serde_json::json!([1, 2]), Firmness::Soft),
    );
    assert!(bad.check().is_err());
    // Reserved and duplicate ids.
    let mut bad = sample_intent();
    bad.transforms.push(TransformIntent::new(
        TransformId(TransformId::GROUP_BASE),
        vec![id(EntityKind::TopoVolume, 1)],
        SchemeChoice::new("htet", Firmness::Hard),
    ));
    assert!(bad.check().is_err());
    let mut bad = sample_intent();
    bad.groups.push(bad.groups[0].clone());
    assert!(bad.check().is_err());
}

#[test]
fn plan_round_trips_through_json_and_the_op_codec() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    intent.set_intervals(ids.volume, Firm::soft(IntervalRequest::Count(3)));
    let plan = driver.plan(&input, &intent).expect("plan");
    let text = serde_json::to_string(&plan).expect("json");
    let back: Plan = serde_json::from_str(&text).expect("parse");
    assert_eq!(plan, back);
    assert!(plan.intervals.iter().all(|x| x.1 == 3));
    // TOML-safe: every parameter value in the plan is `{}` or an object, never null.
    let v = sm_op_api::codec::to_value(&plan).expect("the plan encodes through the Op codec");
    let via: Plan = serde_json::from_value(v).expect("decode");
    assert_eq!(plan, via);
}
