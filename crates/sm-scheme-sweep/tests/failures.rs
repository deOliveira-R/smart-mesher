//! Acceptance 11 and the parameter checks: typed failures, the quality gate and the driver
//! fallback.

mod common;

use common::{mesh_with, plan_only, registry, registry_with_sources};
use sm_core::{EntityId, Point3};
use sm_mesh_api::{MeshEntityKind, MeshRead, validate};
use sm_model_api::Model;
use sm_scheme_api::{
    Driver, FailureKind, Firm, Firmness, IntervalRequest, MeshInput, MeshIntent, NodeKey, Params,
    Phase, ReferenceCornerPicker, ReferenceIntervalSolver, SchemeChoice, SchemeError,
    SchemeRegistry, Sequential, Target, WholeFaces,
};
use sm_scheme_sweep::{SweepParams, TargetMode};
use sm_topo_api::TopologyEdit;
use sm_topo_api::fixtures::BrickIds;

fn brick() -> (sm_topo_api::CellComplex, BrickIds, Model) {
    sm_scheme_api::stubs::brick_setup([1.0, 1.0, 1.0])
}

fn sweep_with(ids: &BrickIds, extra: serde_json::Value) -> SchemeChoice {
    let mut p = serde_json::json!({ "source": [ids.faces[4]], "target": [ids.faces[5]] });
    if let (Some(o), Some(e)) = (p.as_object_mut(), extra.as_object()) {
        for (k, v) in e {
            o.insert(k.clone(), v.clone());
        }
    }
    SchemeChoice::with_params("sweep", p, Firmness::Hard)
}

#[test]
fn faces_that_touch_are_not_applicable() {
    let (topo, ids, model) = brick();
    let mut intent = MeshIntent::default();
    intent.set_scheme(
        ids.volume,
        SchemeChoice::with_params(
            "sweep",
            serde_json::json!({ "source": [ids.faces[0]], "target": [ids.faces[2]] }),
            Firmness::Hard,
        ),
    );
    match plan_only(&registry(), &topo, &model, &[ids.volume], &intent) {
        Err(SchemeError::NotApplicable { reason, .. }) => {
            assert!(reason.contains("share a vertex"), "{reason}");
        }
        other => panic!("expected NotApplicable, got {:?}", other.map(|_| ())),
    }
}

#[test]
fn a_volume_without_any_sweepable_face_pair_is_not_applicable_with_a_reason() {
    // A sphere has a single face: no pair can give columns.
    let fx = sm_testkit::topo::sphere(1.0).expect("sphere");
    let vol = sm_topo_api::TopologyView::cells(&fx.complex, sm_topo_api::Dim::Volume)
        .first()
        .copied()
        .expect("volume");
    let mut intent = MeshIntent::default();
    intent.set_scheme(vol, SchemeChoice::new("sweep", Firmness::Hard));
    match plan_only(&registry(), &fx.complex, &Model::new(), &[vol], &intent) {
        Err(SchemeError::NotApplicable { reason, .. }) => {
            assert!(
                reason.contains("no pair of faces gives columns"),
                "{reason}"
            );
        }
        other => panic!("expected NotApplicable, got {:?}", other.map(|_| ())),
    }
}

#[test]
fn a_non_affine_target_in_auto_mode_names_the_residual() {
    let (mut topo, ids, model) = brick();
    // Move one top vertex: the top square becomes a quadrilateral that is no affine image.
    let top_edges: Vec<EntityId> =
        sm_topo_api::TopologyView::face_edges(&topo, ids.faces[5]).to_vec();
    let (a, _) = sm_topo_api::TopologyViewExt::edge_ends(&topo, top_edges[0]).expect("ends");
    let p = sm_scheme_api::geom::vertex_point(&topo, a).expect("p");
    sm_topo_api::fixtures::with_op(&mut topo, sm_topo_api::OpId(2), |t| {
        Ok(t.set_vertex_point(a, Point3::new(p.x + 0.2, p.y + 0.1, p.z))?)
    })
    .expect("move");
    let mut intent = MeshIntent::default();
    intent.set_scheme(ids.volume, sweep_with(&ids, serde_json::json!({})));
    match plan_only(&registry(), &topo, &model, &[ids.volume], &intent) {
        Err(SchemeError::NotApplicable { reason, .. }) => {
            assert!(
                reason.contains("not an affine image") && reason.contains("residual"),
                "{reason}"
            );
        }
        other => panic!("expected NotApplicable, got {:?}", other.map(|_| ())),
    }
    // target_mode copy registers the copy anyway (the user takes responsibility).
    let mut intent = MeshIntent::default();
    intent.set_scheme(
        ids.volume,
        sweep_with(&ids, serde_json::json!({ "target_mode": "copy" })),
    );
    let plan = plan_only(&registry(), &topo, &model, &[ids.volume], &intent).expect("copy plans");
    assert!(plan.copy_of(Target::Surface(ids.faces[5])).is_some());
}

fn fallback_registry() -> SchemeRegistry {
    registry()
}

#[test]
fn an_inverted_layer_is_a_quality_failure_with_nodes_and_the_driver_falls_back() {
    let (topo, ids, model) = brick();
    let reg = fallback_registry();
    let mut intent = MeshIntent::default();
    for e in [0usize, 4, 8] {
        intent.set_intervals(ids.edges[e], Firm::hard(IntervalRequest::Count(1)));
    }
    // 170 degrees of bend about the x axis folds the top layer back through the bottom.
    intent.set_scheme(
        ids.volume,
        SchemeChoice::with_params(
            "sweep",
            serde_json::json!({
                "source": [ids.faces[4]], "target": [ids.faces[5]],
                "bend": 170.0_f64.to_radians(),
            }),
            Firmness::Hard,
        ),
    );
    intent.entity_mut(ids.volume).fallbacks = vec![SchemeChoice::new("polyvol", Firmness::Default)];
    // Direct generation: the failure carries the layer, element and node keys.
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let plan = driver.plan(&input, &intent).expect("plan");
    let g = match driver.generate(&input, &plan, &Sequential) {
        Err(SchemeError::Generate(g)) => g,
        other => panic!("expected a generation failure, got {:?}", other.map(|_| ())),
    };
    assert_eq!(g.kind, FailureKind::Quality);
    assert!(
        g.reason.contains("layer 0") && g.reason.contains("source element 0"),
        "{}",
        g.reason
    );
    assert_eq!(g.nodes.len(), 8, "{:?}", g.nodes);
    for k in &g.nodes {
        assert!(
            g.reason.contains(&format!("{k:?}")),
            "{k:?} missing from {}",
            g.reason
        );
    }
    assert!(g.reason.contains("Jacobian"));
    // Through the driver: the attempt is recorded and the fallback meshes the brick.
    let out =
        mesh_with(&reg, &topo, &model, &[ids.volume], &intent, &Sequential).expect("fallback");
    let first = out.attempts.first().expect("an attempt");
    assert_eq!(
        (first.target, first.scheme.as_str(), first.phase, first.kind),
        (
            Target::Volume(ids.volume),
            "sweep",
            Phase::Generate,
            FailureKind::Quality
        )
    );
    assert!(out.plan.rejections.contains(
        Target::Volume(ids.volume),
        &sm_scheme_api::SchemeName::new("sweep")
    ));
    let tp = out
        .plan
        .target(Target::Volume(ids.volume))
        .expect("plan row");
    assert_eq!(tp.scheme.as_str(), "polyvol");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 1);
    let _ = NodeKey::new(ids.volume, 0);
}

#[test]
fn a_min_scaled_jacobian_of_one_or_more_is_bad_params() {
    let (topo, ids, model) = brick();
    let mut intent = MeshIntent::default();
    intent.set_scheme(
        ids.volume,
        sweep_with(&ids, serde_json::json!({ "min_scaled_jacobian": 0.5 })),
    );
    // Unit cube hexes have scaled Jacobian 1: fine.
    mesh_with(
        &registry(),
        &topo,
        &model,
        &[ids.volume],
        &intent,
        &Sequential,
    )
    .expect("mesh");
    // Nothing exceeds 1: rejected at parameter parsing.
    let mut intent = MeshIntent::default();
    intent.set_scheme(
        ids.volume,
        sweep_with(&ids, serde_json::json!({ "min_scaled_jacobian": 1.0 })),
    );
    assert!(matches!(
        plan_only(&registry(), &topo, &model, &[ids.volume], &intent),
        Err(SchemeError::BadParams { .. })
    ));
}

#[test]
fn bad_parameters_are_bad_params() {
    let p = |v: serde_json::Value| SweepParams::from_params(&Params(v));
    let a = EntityId::new(sm_core::EntityKind::TopoFace, 1).expect("id");
    let b = EntityId::new(sm_core::EntityKind::TopoFace, 2).expect("id");
    assert!(p(serde_json::json!({})).is_ok());
    assert!(
        p(serde_json::json!({ "source": [a], "target": [b], "layers": 4, "support": 0 })).is_ok()
    );
    for bad in [
        serde_json::json!({ "source": [a] }),
        serde_json::json!({ "target": [b] }),
        serde_json::json!({ "source": [a, b], "target": [b] }),
        serde_json::json!({ "colour": 1 }),
        serde_json::json!({ "shepard_power": 0.0 }),
        serde_json::json!({ "shepard_power": -1.0 }),
        serde_json::json!({ "layers": 0 }),
        serde_json::json!({ "min_scaled_jacobian": 1.5 }),
        serde_json::json!({ "target_mode": "sideways" }),
        serde_json::json!({ "placement": 3 }),
        serde_json::json!({ "support": -2 }),
    ] {
        assert!(
            matches!(p(bad.clone()), Err(SchemeError::BadParams { .. })),
            "{bad}"
        );
    }
    match p(serde_json::json!({ "target_mode": "project" })) {
        Err(SchemeError::BadParams { reason, .. }) => {
            assert_eq!(reason, "target_mode project is not implemented");
        }
        other => panic!("{other:?}"),
    }
    assert_eq!(
        p(serde_json::json!({ "target_mode": "copy" }))
            .expect("copy")
            .target_mode,
        TargetMode::Copy
    );
}

#[test]
fn bad_parameters_surface_from_the_driver_as_bad_params() {
    let (topo, ids, model) = brick();
    let mut intent = MeshIntent::default();
    intent.set_scheme(
        ids.volume,
        sweep_with(&ids, serde_json::json!({ "target_mode": "project" })),
    );
    assert!(matches!(
        plan_only(&registry(), &topo, &model, &[ids.volume], &intent),
        Err(SchemeError::BadParams { .. })
    ));
}

#[test]
fn sources_registry_has_the_real_sweep_only_once() {
    // Registering the real sweep twice is the duplicate-name error the stubs registry would give.
    let mut r = registry_with_sources();
    assert!(sm_scheme_sweep::register(&mut r).is_err());
}

#[test]
fn a_min_scaled_jacobian_above_the_measured_value_is_a_quality_failure() {
    // The twisted prism (2, 3, pi/2), 4x4 x 6, has minimum scaled Jacobian 0.83205.
    let fx =
        sm_testkit::topo::twisted_prism(2.0, 3.0, core::f64::consts::FRAC_PI_2).expect("twisted");
    let topo = &fx.topo.complex;
    let build = |min: f64| {
        let mut intent = MeshIntent::default();
        for e in sm_topo_api::TopologyView::face_edges(topo, fx.source).to_vec() {
            intent.set_intervals(e, Firm::hard(IntervalRequest::Count(4)));
        }
        for e in &fx.columns {
            intent.set_intervals(*e, Firm::hard(IntervalRequest::Count(6)));
        }
        intent.set_scheme(
            fx.volume,
            SchemeChoice::with_params(
                "sweep",
                serde_json::json!({
                    "source": [fx.source], "target": [fx.target], "min_scaled_jacobian": min
                }),
                Firmness::Hard,
            ),
        );
        intent.set_scheme(fx.source, SchemeChoice::new("map", Firmness::Hard));
        intent
    };
    let reg = registry();
    let model = Model::new();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [fx.volume];
    let input = MeshInput::new(topo, &model, &WholeFaces, &sel);
    // 0.8 is below the measured minimum: fine.
    let intent = build(0.8);
    let plan = driver.plan(&input, &intent).expect("plan");
    driver
        .generate(&input, &plan, &Sequential)
        .expect("0.8 passes");
    // 0.9 is above it: a Quality failure naming the cell and its eight nodes.
    let intent = build(0.9);
    let plan = driver.plan(&input, &intent).expect("plan");
    match driver.generate(&input, &plan, &Sequential) {
        Err(SchemeError::Generate(g)) => {
            assert_eq!(g.kind, FailureKind::Quality, "{g}");
            assert!(
                g.reason.contains("Jacobian") && g.reason.contains("0.9"),
                "{}",
                g.reason
            );
            assert!(
                g.reason.contains("layer") && g.reason.contains("source element"),
                "{}",
                g.reason
            );
            assert_eq!(g.nodes.len(), 8);
        }
        other => panic!("expected a Quality failure, got {:?}", other.map(|_| ())),
    }
}
