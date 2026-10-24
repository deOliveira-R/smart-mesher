//! W0f contract fixes: firm sizes keep their firmness, size-field goals integrate `ds / h`,
//! boundary layers off faces are refused, `BoundaryLayerIntent` is extensible, and the intent
//! has a JSON schema.

mod stub_schemes;

use sm_core::{EntityId, Point3};
use sm_scheme_api::{
    BoundaryLayerIntent, Driver, Firm, Firmness, IntervalConstraint, IntervalRequest, MeshInput,
    MeshIntent, Origin, Plan, ReferenceCornerPicker, ReferenceIntervalSolver, SchemeChoice,
    SchemeError, SizeField, WholeFaces, geom,
};
use stub_schemes::{brick_setup, registry};

const X: usize = 0; // x-edge at y = z = 0, from (0,0,0)
const Y: usize = 4;
const Z: usize = 8;

fn sweep_z(ids: &sm_topo_api::fixtures::BrickIds) -> SchemeChoice {
    SchemeChoice::with_params("sweep", serde_json::json!({ "source": [ids.faces[4]], "target": [ids.faces[5]] }), Firmness::Hard)
}

fn interval_records(plan: &Plan, e: EntityId) -> Vec<(Origin, IntervalConstraint)> {
    plan.problem
        .intervals
        .iter()
        .filter(|r| matches!(&r.item, IntervalConstraint::Fixed { edge, .. } | IntervalConstraint::Goal { edge, .. } if *edge == e))
        .map(|r| (r.origin.clone(), r.item.clone()))
        .collect()
}

fn plan_with(size: [f64; 3], f: impl FnOnce(&sm_topo_api::fixtures::BrickIds, &mut MeshIntent), field: Option<&dyn SizeField>) -> (sm_topo_api::fixtures::BrickIds, Result<Plan, SchemeError>) {
    let (topo, ids, model) = brick_setup(size);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let mut driver = Driver::new(&reg, &solver, &picker);
    driver.size = field;
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    intent.set_scheme(ids.volume, sweep_z(&ids));
    f(&ids, &mut intent);
    let r = driver.plan(&input, &intent);
    (ids, r)
}

#[test]
fn firm_sizes_become_fixed_counts_with_their_firmness() {
    for (firmness, want) in [(Firmness::Hard, Some(Firmness::Hard)), (Firmness::Soft, Some(Firmness::Soft)), (Firmness::Default, None)] {
        let (ids, plan) = plan_with(
            [2.0, 1.0, 1.0],
            |ids, i| {
                i.set_intervals(ids.edges[X], Firm { value: IntervalRequest::Size(0.5), firmness });
                i.set_intervals(ids.edges[Y], Firm::hard(IntervalRequest::Count(1)));
                i.set_intervals(ids.edges[Z], Firm::hard(IntervalRequest::Count(1)));
            },
            None,
        );
        let plan = plan.expect("plan");
        let recs = interval_records(&plan, ids.edges[X]);
        match want {
            Some(fm) => assert!(
                recs.iter().any(|(o, c)| *o == Origin::Intent && *c == IntervalConstraint::Fixed { edge: ids.edges[X], count: 4, firmness: fm }),
                "{recs:?}"
            ),
            None => assert!(recs.iter().any(|(_, c)| matches!(c, IntervalConstraint::Goal { goal, .. } if (goal - 4.0).abs() < 1e-12)), "{recs:?}"),
        }
        if want.is_some() {
            // A firm count beats the default goals of the parallel x-edges.
            assert_eq!(plan.intervals(ids.edges[X]), Some(4));
        }
    }
    // round(L / h), at least one: 2 / 0.75 = 2.67 -> 3; 2 / 10 -> 1.
    for (h, n) in [(0.75, 3), (10.0, 1)] {
        let (ids, plan) = plan_with(
            [2.0, 1.0, 1.0],
            |ids, i| {
                i.set_intervals(ids.edges[X], Firm::hard(IntervalRequest::Size(h)));
                i.set_intervals(ids.edges[Y], Firm::hard(IntervalRequest::Count(1)));
                i.set_intervals(ids.edges[Z], Firm::hard(IntervalRequest::Count(1)));
            },
            None,
        );
        assert_eq!(plan.expect("plan").intervals(ids.edges[X]), Some(n));
    }
}

/// `h(p) = 0.01 + x`.
struct Linear;
impl SizeField for Linear {
    fn size(&self, p: Point3) -> f64 {
        0.01 + p.x
    }
}

#[test]
fn size_field_goal_integrates_ds_over_h() {
    let want = 101f64.ln();
    // Directly on the unit x-edge: integral_0^1 dx / (0.01 + x) = ln 101 (the midpoint rule
    // gave 1 / 0.51 = 1.96).
    let (topo, ids, _) = brick_setup([1.0, 1.0, 1.0]);
    let g = geom::edge_size_goal(&topo, ids.edges[X], &Linear).unwrap();
    assert!((g - want).abs() < 1e-9, "{g} vs {want}");
    // A constant field gives length / h exactly; a y-edge at x = 0 sees h = 0.01.
    let gy = geom::edge_size_goal(&topo, ids.edges[Y], &Linear).unwrap();
    assert!((gy - 100.0).abs() < 1e-9, "{gy}");
    struct Bad;
    impl SizeField for Bad {
        fn size(&self, _: Point3) -> f64 {
            0.0
        }
    }
    assert!(geom::edge_size_goal(&topo, ids.edges[X], &Bad).is_err());
    // Through the planner: the sizing goal of the x-edge is ln 101.
    let (ids, plan) = plan_with([1.0, 1.0, 1.0], |_, _| {}, Some(&Linear));
    let plan = plan.expect("plan");
    let recs = interval_records(&plan, ids.edges[X]);
    assert!(
        recs.iter().any(|(o, c)| *o == Origin::Sizing && matches!(c, IntervalConstraint::Goal { goal, .. } if (goal - want).abs() < 1e-9)),
        "{recs:?}"
    );
}

#[test]
fn boundary_layers_off_faces_are_refused_and_into_is_checked() {
    // On an edge: refused (it used to be dropped silently).
    let (_, r) = plan_with(
        [1.0, 1.0, 1.0],
        |ids, i| {
            i.entity_mut(ids.edges[X]).boundary_layer = Some(BoundaryLayerIntent::new(0.01, 1.2, 3));
        },
        None,
    );
    assert!(matches!(r, Err(SchemeError::Input(ref m)) if m.contains("only supported on faces")), "{:?}", r.map(|_| ()));
    // `into` must list adjacent volume pieces.
    let (_, r) = plan_with(
        [1.0, 1.0, 1.0],
        |ids, i| {
            let mut bl = BoundaryLayerIntent::new(0.01, 1.2, 3);
            bl.into = vec![EntityId::new(sm_core::EntityKind::TopoVolume, 99).unwrap()];
            i.entity_mut(ids.faces[2]).boundary_layer = Some(bl);
        },
        None,
    );
    assert!(matches!(r, Err(SchemeError::Input(ref m)) if m.contains("not adjacent")), "{:?}", r.map(|_| ()));
    let (_, r) = plan_with(
        [1.0, 1.0, 1.0],
        |ids, i| {
            let mut bl = BoundaryLayerIntent::new(0.01, 1.2, 3);
            bl.into = vec![ids.faces[0]];
            i.entity_mut(ids.faces[2]).boundary_layer = Some(bl);
        },
        None,
    );
    assert!(matches!(r, Err(SchemeError::Input(ref m)) if m.contains("not a volume piece")), "{:?}", r.map(|_| ()));
}

#[test]
fn boundary_layer_intent_is_extensible_on_the_wire() {
    // Old records (no `into`, no `params`) still load; empty fields are not written.
    let old: BoundaryLayerIntent = serde_json::from_value(serde_json::json!({"first": 0.01, "growth": 1.2, "layers": 3})).unwrap();
    assert_eq!(old, BoundaryLayerIntent::new(0.01, 1.2, 3));
    let v = serde_json::to_value(&old).unwrap();
    assert!(v.get("into").is_none() && v.get("params").is_none(), "{v}");
    let mut full = old.clone();
    full.into = vec![EntityId::new(sm_core::EntityKind::TopoVolume, 1).unwrap()];
    full.params = sm_scheme_api::Params(serde_json::json!({"aspect": 2.0}));
    let back: BoundaryLayerIntent = serde_json::from_value(serde_json::to_value(&full).unwrap()).unwrap();
    assert_eq!(back, full);
}

#[cfg(feature = "schemars")]
#[test]
fn mesh_intent_has_a_json_schema() {
    let s = serde_json::to_value(schemars::schema_for!(MeshIntent)).unwrap();
    assert!(s["properties"].get("entities").is_some(), "{s}");
    let _ = schemars::schema_for!(sm_scheme_api::TransformIntent);
    let _ = schemars::schema_for!(sm_scheme_api::MeshGroup);
    let p = serde_json::to_value(schemars::schema_for!(sm_scheme_api::Params)).unwrap();
    assert_eq!(p["type"], "object");
}
