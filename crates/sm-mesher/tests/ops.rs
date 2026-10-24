//! Acceptance tests of T7: the intent and mesh ops end to end with the stub schemes.

mod common;

use common::*;
use serde_json::json;
use sm_core::EntityKind;
use sm_layer_api::{MeshLayer, ModelLayer};
use sm_op_api::{DerivedCache, LayerType, QuietControl, Session};
use sm_scheme_api::stubs;

#[test]
fn stub_sweep_brick_10_size_1_gives_1000_cells_and_1331_vertices() {
    let mut s = brick_session([10.0; 3], 1.0);
    let before = s.state().entity_count();
    let (_, d) = mesh_run(&mut s).expect("mesh.run");
    let st = s.state();
    assert_eq!(st.count_of(EntityKind::MeshCell), 1000);
    assert_eq!(st.count_of(EntityKind::MeshVertex), 1331);
    assert_eq!(st.count_of(EntityKind::Mesh), 1);
    assert_eq!(d.outputs["mesh"], vec![id(EntityKind::Mesh, 1)]);
    assert_eq!(d.touched_layers, vec!["meshes".to_owned()]);
    // Delta.created is exactly what the mesh holds, ascending by (kind, serial), Mesh first.
    let layer = st.layer::<MeshLayer>().expect("layer");
    let (mid, e) = layer.meshes().next().expect("one mesh");
    let m = e.mesh();
    assert_eq!(d.created.first(), Some(&mid));
    assert!(d.created.windows(2).all(|w| w[0] < w[1]));
    assert_eq!(st.entity_count() - before, d.created.len());
    let of = |k| d.created.iter().filter(|i| i.kind() == k).count();
    assert_eq!(of(EntityKind::MeshCell), m.cell_count());
    assert_eq!(of(EntityKind::MeshFace), m.face_count());
    assert_eq!(of(EntityKind::MeshVertex), m.vertex_count());
    assert_eq!(e.selection, vec![vol(1)]);
    assert!(!e.stale);
    assert!(e.plan.is_some());
}

fn err_text<T>(r: Result<T, sm_op_api::TimelineError>) -> String {
    match r {
        Ok(_) => panic!("expected an error"),
        Err(e) => format!("{e:?} {e}"),
    }
}

// ------------------------------------------------------------------ re-plan and fallback

#[test]
fn failing_scheme_replans_and_is_recorded_in_the_plan_attempts() {
    let mut reg = stubs::registry();
    reg.register(Box::new(stubs::AlwaysFail))
        .expect("always_fail");
    let mut s = sm_op_api::Session::new(engine_with(reg, DerivedCache::default()));
    run(&mut s, &brick_op([2.0; 3])).expect("brick");
    run_json(
        &mut s,
        "mesh.size",
        json!({"targets": ["TopoVolume#1"], "size": 1.0}),
    )
    .expect("size");
    run_json(
        &mut s,
        "mesh.scheme",
        json!({"targets": ["TopoVolume#1"], "scheme": "sweep",
               "params": {"source": ["TopoFace#5"], "target": ["TopoFace#6"]}}),
    )
    .expect("scheme");
    mesh_run(&mut s).expect("the fallback chain recovers");
    let layer = s.state().layer::<MeshLayer>().expect("layer");
    let (_, e) = layer.meshes().next().expect("mesh");
    let plan = e.plan.as_ref().expect("plan kept");
    assert_eq!(plan.rejections.len(), 1, "always_fail was rejected once");
    // The failure that caused the re-plan is in `rejections`; the final plan lists the
    // passed-over candidate in `attempts`.
    let f = plan.rejections.attempts().next().expect("one");
    assert_eq!(
        (f.scheme.as_str(), f.phase),
        ("always_fail", sm_scheme_api::Phase::Generate)
    );
    assert!(
        plan.attempts
            .iter()
            .any(|a| a.scheme.as_str() == "always_fail"),
        "the failed candidate is recorded in plan.attempts"
    );
    assert_eq!(e.mesh().cell_count(), 8);
}

// ------------------------------------------------------------------ atomic failure

#[test]
fn infeasible_hard_intent_fails_atomically() {
    let mut s = brick_session([10.0; 3], 1.0);
    run_json(
        &mut s,
        "mesh.intervals",
        json!({"targets": ["TopoEdge#1"], "count": 3}),
    )
    .expect("3");
    run_json(
        &mut s,
        "mesh.intervals",
        json!({"targets": ["TopoEdge#3"], "count": 5}),
    )
    .expect("5");
    let hash = s.state().hash();
    let n = s.state().entity_count();
    let layers = s.state().layer_hashes();
    let e = err_text(mesh_run(&mut s));
    assert!(e.contains("infeasible") || e.contains("Infeasible"), "{e}");
    assert_eq!(s.state().hash(), hash);
    assert_eq!(s.state().entity_count(), n);
    assert_eq!(s.state().layer_hashes(), layers);
    assert!(s.state().layer::<MeshLayer>().expect("layer").is_empty());
    assert_eq!(
        s.state().peek_next(EntityKind::MeshCell),
        Some(id(EntityKind::MeshCell, 1)),
        "no serial was consumed"
    );
}

// ------------------------------------------------------------------ delete

#[test]
fn mesh_delete_restores_the_live_counts() {
    let mut s = brick_session([4.0; 3], 1.0);
    let (base_count, base_hash) = {
        let st = s.state();
        (st.entity_count(), st.layer_hash("meshes").ok())
    };
    let (_, d) = mesh_run(&mut s).expect("run");
    let mid = d.outputs["mesh"][0];
    assert_eq!(s.state().count_of(EntityKind::MeshCell), 64);
    let (_, dd) =
        run_json(&mut s, "mesh.delete", json!({"meshes": [mid.to_string()]})).expect("delete");
    assert_eq!(dd.removed.len(), d.created.len());
    assert!(dd.removed.windows(2).all(|w| w[0] < w[1]));
    for k in [
        EntityKind::Mesh,
        EntityKind::MeshCell,
        EntityKind::MeshFace,
        EntityKind::MeshVertex,
    ] {
        assert_eq!(s.state().count_of(k), 0, "{k:?}");
    }
    assert_eq!(s.state().entity_count(), base_count);
    assert!(!s.state().contains(mid));
    // The layer is empty again, but its hash is not the pre-run one: the allocator's serial
    // counters advanced and are never rewound (ids are never reused).
    assert!(s.state().layer::<MeshLayer>().expect("layer").is_empty());
    assert_ne!(s.state().layer_hash("meshes").ok(), base_hash);
    // Deleting again fails; so does naming both or neither selector.
    assert!(run_json(&mut s, "mesh.delete", json!({"meshes": [mid.to_string()]})).is_err());
    assert!(run_json(&mut s, "mesh.delete", json!({})).is_err());
    assert!(
        run_json(
            &mut s,
            "mesh.delete",
            json!({"meshes": [mid.to_string()], "targets": ["TopoVolume#1"]})
        )
        .is_err()
    );
    // The volume can be meshed again, with fresh serials (never reused).
    let (_, d2) = mesh_run(&mut s).expect("again");
    assert_ne!(d2.outputs["mesh"][0], mid);
    // Delete by target.
    run_json(&mut s, "mesh.delete", json!({"targets": ["TopoVolume#1"]})).expect("by target");
    assert_eq!(s.state().count_of(EntityKind::MeshCell), 0);
    assert!(run_json(&mut s, "mesh.delete", json!({"targets": ["TopoVolume#1"]})).is_err());
}

#[test]
fn an_already_meshed_volume_is_refused() {
    let mut s = brick_session([2.0; 3], 1.0);
    mesh_run(&mut s).expect("run");
    let hash = s.state().hash();
    let e = err_text(mesh_run(&mut s));
    assert!(e.contains("mesh.delete first"), "{e}");
    assert_eq!(s.state().hash(), hash);
}

#[test]
fn a_neighbour_of_a_meshed_volume_is_refused() {
    let mut s = session();
    run(&mut s, &brick_op([1.0, 2.0, 3.0])).expect("brick");
    run(&mut s, &Webcut { frac: 0.5 }).expect("webcut");
    // The cut leaves two volumes sharing the cap face.
    let vols: Vec<_> = s
        .state()
        .layer::<ModelLayer>()
        .expect("model")
        .topo()
        .cells(sm_topo_api::Dim::Volume)
        .to_vec();
    assert_eq!(vols.len(), 2);
    run_json(
        &mut s,
        "mesh.size",
        json!({"targets": [vols[0].to_string(), vols[1].to_string()], "size": 1.0}),
    )
    .expect("size");
    run_json(
        &mut s,
        "mesh.run",
        json!({"targets": [vols[0].to_string()]}),
    )
    .expect("first half");
    let e = err_text(run_json(
        &mut s,
        "mesh.run",
        json!({"targets": [vols[1].to_string()]}),
    ));
    assert!(
        e.contains("shares") && e.contains("mesh.delete first"),
        "{e}"
    );
    // Meshing the two together is a separate run: delete, then mesh both.
    run_json(
        &mut s,
        "mesh.delete",
        json!({"targets": [vols[0].to_string()]}),
    )
    .expect("delete");
    run_json(
        &mut s,
        "mesh.run",
        json!({"targets": [vols[0].to_string(), vols[1].to_string()]}),
    )
    .expect("together");
}

// ------------------------------------------------------------------ intent layer

fn intent_hash(s: &sm_op_api::Session) -> sm_op_api::Hash256 {
    s.state()
        .layer_hash("intent")
        .expect("intent layer is non-empty")
}

#[test]
fn intent_is_independent_of_op_order_and_replaces_repeats() {
    let two_bricks = || {
        let mut s = session();
        run(&mut s, &brick_op([4.0; 3])).expect("a");
        run(
            &mut s,
            &CreateBrick {
                origin: [10.0, 0.0, 0.0],
                size: [2.0; 3],
            },
        )
        .expect("b");
        s
    };
    let size = |s: &mut sm_op_api::Session, t: &str, h: f64| {
        run_json(s, "mesh.size", json!({"targets": [t], "size": h})).expect("size");
    };
    let mut a = two_bricks();
    size(&mut a, "TopoVolume#1", 1.0);
    size(&mut a, "TopoVolume#2", 0.5);
    run_json(
        &mut a,
        "mesh.intervals",
        json!({"targets": ["TopoEdge#1"], "count": 3}),
    )
    .expect("n");
    let mut b = two_bricks();
    run_json(
        &mut b,
        "mesh.intervals",
        json!({"targets": ["TopoEdge#1"], "count": 3}),
    )
    .expect("n");
    size(&mut b, "TopoVolume#2", 0.5);
    size(&mut b, "TopoVolume#1", 1.0);
    assert_eq!(
        a.state().hash(),
        b.state().hash(),
        "request order does not matter"
    );

    // Repeated mesh.size replaces; it does not accumulate.
    let mut c = two_bricks();
    size(&mut c, "TopoVolume#1", 2.0);
    let once = intent_hash(&c);
    size(&mut c, "TopoVolume#1", 1.0);
    assert_ne!(intent_hash(&c), once, "the digest sees the value");
    size(&mut c, "TopoVolume#1", 2.0);
    assert_eq!(intent_hash(&c), once);
    let intent = c
        .state()
        .layer::<sm_mesher::IntentLayer>()
        .expect("layer")
        .intent();
    assert_eq!(intent.entities.len(), 1);
    // A target list in another order or with a duplicate is the same request.
    let mut d = two_bricks();
    run_json(
        &mut d,
        "mesh.size",
        json!({"targets": ["TopoVolume#2", "TopoVolume#1", "TopoVolume#2"], "size": 2.0}),
    )
    .expect("both");
    let mut e = two_bricks();
    run_json(
        &mut e,
        "mesh.size",
        json!({"targets": ["TopoVolume#1", "TopoVolume#2"], "size": 2.0}),
    )
    .expect("both");
    assert_eq!(d.state().hash(), e.state().hash());
}

#[test]
fn firmness_ladder_keeps_the_firmer_request_and_warns() {
    let mut s = session();
    run(&mut s, &brick_op([4.0; 3])).expect("brick");
    run_json(
        &mut s,
        "mesh.intervals",
        json!({"targets": ["TopoEdge#1"], "count": 5}),
    )
    .expect("hard");
    let h = s.state().hash();
    let (_, d) = run_json(
        &mut s,
        "mesh.size",
        json!({"targets": ["TopoEdge#1"], "size": 1.0}),
    )
    .expect("size");
    assert_eq!(s.state().hash(), h, "the hard count stays");
    assert_eq!(d.diagnostics.len(), 1);
    assert_eq!(d.diagnostics[0].code, "mesh.firmness_kept");
    // An equal or higher firmness replaces it.
    run_json(
        &mut s,
        "mesh.size",
        json!({"targets": ["TopoEdge#1"], "size": 1.0, "firmness": "hard"}),
    )
    .expect("hard size");
    assert_ne!(s.state().hash(), h);
}

#[test]
fn intent_requests_on_deleted_entities_are_dropped() {
    let mut s = session();
    run(&mut s, &brick_op([4.0; 3])).expect("a");
    run(
        &mut s,
        &CreateBrick {
            origin: [10.0, 0.0, 0.0],
            size: [2.0; 3],
        },
    )
    .expect("b");
    run_json(
        &mut s,
        "mesh.size",
        json!({"targets": ["TopoVolume#1", "TopoVolume#2"], "size": 1.0}),
    )
    .expect("size");
    run(&mut s, &DeleteVolume { volume: vol(1) }).expect("delete");
    let intent = s
        .state()
        .layer::<sm_mesher::IntentLayer>()
        .expect("layer")
        .intent();
    assert_eq!(
        intent.entities.keys().copied().collect::<Vec<_>>(),
        vec![vol(2)]
    );
    run(&mut s, &DeleteVolume { volume: vol(2) }).expect("delete");
    assert!(LayerType::is_empty(
        s.state().layer::<sm_mesher::IntentLayer>().expect("layer")
    ));
}

#[test]
fn bad_intent_requests_are_rejected_and_change_nothing() {
    let mut s = session();
    run(&mut s, &brick_op([4.0; 3])).expect("brick");
    let h = s.state().hash();
    let bad = [
        ("mesh.size", json!({"targets": ["TopoVolume#1"]})),
        (
            "mesh.size",
            json!({"targets": ["TopoVolume#1"], "size": 1.0, "auto": 3}),
        ),
        (
            "mesh.size",
            json!({"targets": ["TopoVolume#1"], "size": -1.0}),
        ),
        ("mesh.size", json!({"targets": ["TopoVolume#1"], "auto": 3})), // no auto hook
        (
            "mesh.size",
            json!({"targets": ["TopoVolume#1"], "auto": 11}),
        ),
        ("mesh.size", json!({"targets": [], "size": 1.0})),
        (
            "mesh.size",
            json!({"targets": ["TopoVolume#9"], "size": 1.0}),
        ),
        (
            "mesh.size",
            json!({"targets": ["TopoVertex#1"], "size": 1.0}),
        ),
        (
            "mesh.intervals",
            json!({"targets": ["TopoEdge#1"], "count": 0}),
        ),
        (
            "mesh.scheme",
            json!({"targets": ["TopoVolume#1"], "scheme": "nope"}),
        ),
        (
            "mesh.scheme",
            json!({"targets": ["TopoVolume#1"], "scheme": "equal"}),
        ), // a curve scheme
        (
            "mesh.scheme",
            json!({"targets": ["TopoEdge#1"], "scheme": "equal", "fallbacks": ["nope"]}),
        ),
        ("mesh.default_size", json!({})),
        ("mesh.default_size", json!({"function": "linear"})), // no builder
        ("mesh.run", json!({"targets": ["TopoFace#1"]})),
        ("mesh.run", json!({"targets": []})),
    ];
    for (op, p) in bad {
        assert!(run_json(&mut s, op, p.clone()).is_err(), "{op} {p}");
        assert_eq!(s.state().hash(), h, "{op} {p}");
    }
    run_json(
        &mut s,
        "mesh.scheme",
        json!({"targets": ["TopoEdge#1"], "scheme": "equal", "fallbacks": []}),
    )
    .expect("a curve scheme on a curve");
}

#[test]
fn auto_size_and_size_fields_come_from_the_injected_hooks() {
    use std::sync::Arc;
    let cfg = sm_mesher::MesherConfig::new(stubs::registry())
        .with_auto_size(Arc::new(|_, t, k| {
            assert_eq!(t.len(), 1);
            Ok(f64::from(k) * 0.5)
        }))
        .with_size_fields(Arc::new(|spec| match spec {
            sm_scheme_api::SizingSpec::Function { name, params } if name == "const" => {
                let h = params.get("const", "size")?.unwrap_or(1.0);
                Ok(Some(Box::new(sm_scheme_api::ConstantSize(h))))
            }
            _ => Err(sm_scheme_api::SchemeError::Input(
                "unknown sizing function".into(),
            )),
        }));
    let mut s = sm_op_api::Session::new(engine_cfg(cfg, DerivedCache::default()));
    run(&mut s, &brick_op([10.0; 3])).expect("brick");
    // auto is resolved to a plain size when applied.
    run_json(
        &mut s,
        "mesh.size",
        json!({"targets": ["TopoVolume#1"], "auto": 4}),
    )
    .expect("auto");
    let intent = s
        .state()
        .layer::<sm_mesher::IntentLayer>()
        .expect("layer")
        .intent();
    assert_eq!(
        intent.entities[&vol(1)].intervals.expect("set").value,
        sm_scheme_api::IntervalRequest::Size(2.0)
    );
    // A named function is validated when set and used by mesh.run: size 2.5 -> 4 per edge. A
    // fresh model carries no explicit size, so the injected field alone decides the count.
    assert!(run_json(&mut s, "mesh.default_size", json!({"function": "bogus"})).is_err());
    let cfg2 =
        sm_mesher::MesherConfig::new(stubs::registry()).with_size_fields(Arc::new(
            |spec| match spec {
                sm_scheme_api::SizingSpec::Function { name, params } if name == "const" => {
                    let h = params.get("const", "size")?.unwrap_or(1.0);
                    Ok(Some(Box::new(sm_scheme_api::ConstantSize(h))))
                }
                _ => Err(sm_scheme_api::SchemeError::Input(
                    "unknown sizing function".into(),
                )),
            },
        ));
    let mut s = sm_op_api::Session::new(engine_cfg(cfg2, DerivedCache::default()));
    run(&mut s, &brick_op([10.0; 3])).expect("brick");
    run_json(
        &mut s,
        "mesh.default_size",
        json!({"function": "const", "params": {"size": 2.5}}),
    )
    .expect("fn");
    run_json(
        &mut s,
        "mesh.scheme",
        json!({"targets": ["TopoVolume#1"], "scheme": "sweep",
               "params": {"source": ["TopoFace#5"], "target": ["TopoFace#6"]}}),
    )
    .expect("scheme");
    mesh_run(&mut s).expect("run");
    assert_eq!(s.state().count_of(EntityKind::MeshCell), 64);
}

#[test]
fn default_size_applies_where_no_request_reaches() {
    let mut s = session();
    run(&mut s, &brick_op([10.0; 3])).expect("brick");
    run_json(&mut s, "mesh.default_size", json!({"size": 2.0})).expect("default");
    run_json(
        &mut s,
        "mesh.scheme",
        json!({"targets": ["TopoVolume#1"], "scheme": "sweep",
               "params": {"source": ["TopoFace#5"], "target": ["TopoFace#6"]}}),
    )
    .expect("scheme");
    mesh_run(&mut s).expect("run");
    assert_eq!(s.state().count_of(EntityKind::MeshCell), 125);
}

#[test]
fn the_intent_of_other_volumes_does_not_disturb_a_run() {
    let mut s = session();
    run(&mut s, &brick_op([4.0; 3])).expect("a");
    run(
        &mut s,
        &CreateBrick {
            origin: [10.0, 0.0, 0.0],
            size: [2.0; 3],
        },
    )
    .expect("b");
    run_json(
        &mut s,
        "mesh.size",
        json!({"targets": ["TopoVolume#1"], "size": 1.0}),
    )
    .expect("a size");
    run_json(
        &mut s,
        "mesh.size",
        json!({"targets": ["TopoVolume#2"], "size": 0.5}),
    )
    .expect("b size");
    run_json(
        &mut s,
        "mesh.intervals",
        json!({"targets": ["TopoEdge#13"], "count": 7}),
    )
    .expect("b edge");
    run_json(
        &mut s,
        "mesh.scheme",
        json!({"targets": ["TopoVolume#1"], "scheme": "sweep",
               "params": {"source": ["TopoFace#5"], "target": ["TopoFace#6"]}}),
    )
    .expect("a scheme");
    run_json(
        &mut s,
        "mesh.scheme",
        json!({"targets": ["TopoVolume#2"], "scheme": "sweep",
               "params": {"source": ["TopoFace#11"], "target": ["TopoFace#12"]}}),
    )
    .expect("b scheme");
    run_json(&mut s, "mesh.run", json!({"targets": ["TopoVolume#1"]})).expect("a only");
    assert_eq!(s.state().count_of(EntityKind::MeshCell), 64);
    run_json(&mut s, "mesh.run", json!({"targets": ["TopoVolume#2"]})).expect("then b");
    assert_eq!(s.state().count_of(EntityKind::Mesh), 2);
}

// ------------------------------------------------------------------ queries

#[test]
fn mesh_info_reports_counts_hashes_and_volume() {
    let mut s = brick_session([10.0; 3], 1.0);
    let (_, d) = mesh_run(&mut s).expect("run");
    let mid = d.outputs["mesh"][0];
    let q = QuietControl::new();
    let info = s
        .engine()
        .run_query(
            s.state(),
            "mesh.info",
            &json!({"mesh": mid.to_string()}),
            &q.control(),
        )
        .expect("query");
    assert_eq!(info["cells"], 1000);
    assert_eq!(info["vertices"], 1331);
    assert_eq!(info["cell_shapes"], json!({"Hex": 1000}));
    assert_eq!(info["stale"], false);
    assert_eq!(info["selection"], json!(["TopoVolume#1"]));
    assert!((info["volume"].as_f64().expect("f64") - 1000.0).abs() < 1e-9);
    assert_eq!(info["topology_hash"].as_str().expect("str").len(), 16);
    assert_eq!(info["geometry_hash"].as_str().expect("str").len(), 16);
    // Deleting the volume marks the mesh stale; mesh.info says so.
    run(&mut s, &DeleteVolume { volume: vol(1) }).expect("delete");
    let info2 = s
        .engine()
        .run_query(
            s.state(),
            "mesh.info",
            &json!({"mesh": mid.to_string()}),
            &q.control(),
        )
        .expect("query");
    assert_eq!(info2["stale"], true);
    assert_eq!(info2["topology_hash"], info["topology_hash"]);
    assert!(
        s.engine()
            .run_query(
                s.state(),
                "mesh.info",
                &json!({"mesh": "Mesh#9"}),
                &q.control()
            )
            .is_err()
    );
}

#[test]
fn the_mesh_is_a_valid_unit_lattice() {
    let mut s = brick_session([10.0; 3], 1.0);
    mesh_run(&mut s).expect("run");
    let layer = s.state().layer::<MeshLayer>().expect("layer");
    let (_, e) = layer.meshes().next().expect("mesh");
    let m = e.mesh();
    assert!(sm_mesh_api::validate(m).is_empty());
    let co = m.coords();
    for i in 0..co.len() {
        for c in [co.x[i], co.y[i], co.z[i]] {
            assert!(
                (c - c.round()).abs() < 1e-12 && (0.0..=10.0).contains(&c),
                "{c}"
            );
        }
    }
}

#[test]
fn registered_schemas_carry_entity_kinds_and_declared_outputs() {
    let eng = engine();
    let reg = eng.registry();
    for name in [
        "mesh.size",
        "mesh.intervals",
        "mesh.scheme",
        "mesh.run",
        "mesh.delete",
        "mesh.default_size",
    ] {
        let schema = serde_json::to_value(reg.schema(name).expect("schema")).expect("json");
        let props = schema["properties"].as_object().expect("properties");
        for key in ["targets", "meshes"] {
            if let Some(p) = props.get(key) {
                assert!(p.get("x-entity-kind").is_some(), "{name}.{key}: {p}");
            }
        }
    }
    let outs = reg.op_outputs("mesh.run");
    assert_eq!(outs.len(), 1);
    assert_eq!(outs[0].name, "mesh");
    assert!(reg.query_names().contains(&"mesh.info"));
    let q = serde_json::to_value(
        reg.query_catalog()
            .into_iter()
            .next()
            .expect("query")
            .expect("info")
            .schema,
    )
    .expect("json");
    assert!(q["properties"]["mesh"].get("x-entity-kind").is_some());
}

#[test]
fn a_missing_service_is_a_typed_error_and_changes_nothing() {
    use std::sync::Arc;
    let mut reg = sm_op_api::Registry::new();
    reg.install(&sm_layer_api::CoreLayers::new(cell_complex))
        .expect("core");
    reg.install(&sm_mesher::MeshPlugin).expect("mesher");
    reg.register::<CreateBrick>().expect("brick");
    let eng = sm_op_api::Engine::new(
        Arc::new(reg),
        sm_op_api::Services::new(),
        Arc::new(DerivedCache::default()),
    )
    .expect("engine");
    let mut s = sm_op_api::Session::new(eng);
    run(&mut s, &brick_op([1.0; 3])).expect("brick");
    let h = s.state().hash();
    assert!(err_text(mesh_run(&mut s)).contains("MissingService"));
    assert_eq!(s.state().hash(), h);
}

// ------------------------------------------------------------------ replay and determinism

#[test]
fn undo_redo_and_cold_replay_do_not_diverge() {
    let eng = engine_with(stubs::registry(), DerivedCache::disabled());
    let mut s = sm_op_api::Session::new(eng);
    run(&mut s, &brick_op([4.0; 3])).expect("brick");
    run_json(
        &mut s,
        "mesh.size",
        json!({"targets": ["TopoVolume#1"], "size": 1.0}),
    )
    .expect("size");
    run_json(
        &mut s,
        "mesh.scheme",
        json!({"targets": ["TopoVolume#1"], "scheme": "sweep",
               "params": {"source": ["TopoFace#5"], "target": ["TopoFace#6"]}}),
    )
    .expect("scheme");
    let (h_run, _) = mesh_run(&mut s).expect("run");
    let after_run = s.state().hash();
    let (h_del, _) =
        run_json(&mut s, "mesh.delete", json!({"targets": ["TopoVolume#1"]})).expect("delete");
    let full = s.state().hash();
    let q = QuietControl::new();
    assert_eq!(
        s.state_at_from_root(h_run, &q.control())
            .expect("cold replay")
            .hash(),
        after_run
    );
    assert_eq!(
        s.state_at_from_root(h_del, &q.control())
            .expect("cold replay")
            .hash(),
        full
    );
    s.undo().expect("undo");
    assert_eq!(s.state().hash(), after_run);
    assert_eq!(s.state().count_of(EntityKind::MeshCell), 64);
    s.undo().expect("undo");
    assert_eq!(s.state().count_of(EntityKind::MeshCell), 0);
    s.redo().expect("redo");
    s.redo().expect("redo");
    assert_eq!(s.state().hash(), full);
}

/// Mesh a brick of `n` cells per side on a rayon pool of `threads`; state hash plus the exact
/// (bit-for-bit) hash of the meshes layer.
fn flow(n: u32, threads: usize) -> (sm_op_api::Hash256, sm_op_api::Hash256) {
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(threads)
        .build()
        .expect("pool");
    pool.install(|| {
        let mut s = brick_session([f64::from(n); 3], 1.0);
        mesh_run(&mut s).expect("run");
        let exact = sm_op_api::Layer::exact_hash(s.state().layer::<MeshLayer>().expect("layer"));
        (s.state().hash(), exact)
    })
}

#[test]
fn state_and_mesh_are_bit_identical_across_threads_for_a_20_cubed_brick() {
    let got = sm_testkit::determinism::check(2, 4, |t| flow(20, t)).expect("deterministic");
    let (state, _) = got;
    // And equal to a default-pool run (RAYON_NUM_THREADS is honoured when set in the environment).
    let mut s = brick_session([20.0; 3], 1.0);
    mesh_run(&mut s).expect("run");
    assert_eq!(s.state().hash(), state);
    assert_eq!(s.state().count_of(EntityKind::MeshCell), 8000);
}

// ------------------------------------------------------------------ review fixes

#[test]
fn a_later_size_replaces_an_earlier_soft_interval_count() {
    let mut s = session();
    run(&mut s, &brick_op([10.0; 3])).expect("brick");
    run_json(
        &mut s,
        "mesh.scheme",
        json!({"targets": ["TopoVolume#1"], "scheme": "sweep",
               "params": {"source": ["TopoFace#5"], "target": ["TopoFace#6"]}}),
    )
    .expect("scheme");
    run_json(
        &mut s,
        "mesh.intervals",
        json!({"targets": ["TopoVolume#1"], "count": 5}),
    )
    .expect("count");
    let (_, d) = run_json(
        &mut s,
        "mesh.size",
        json!({"targets": ["TopoVolume#1"], "size": 1.0}),
    )
    .expect("size");
    assert!(d.diagnostics.is_empty(), "{:?}", d.diagnostics);
    mesh_run(&mut s).expect("run");
    assert_eq!(s.state().count_of(EntityKind::MeshCell), 1000);
}

#[test]
fn an_absurd_size_is_refused_before_generating() {
    let mut s = brick_session([10.0; 3], 1e-6);
    let hash = s.state().hash();
    let e = err_text(mesh_run(&mut s));
    assert!(e.contains("budget"), "{e}");
    assert_eq!(s.state().hash(), hash);
    // The budget is a config matter: 1000 cells pass under the default, fail under 100.
    let cfg = sm_mesher::MesherConfig::new(stubs::registry()).with_max_cells(100);
    let mut s = Session::new(engine_cfg(cfg, DerivedCache::default()));
    run(&mut s, &brick_op([10.0; 3])).expect("brick");
    run_json(
        &mut s,
        "mesh.size",
        json!({"targets": ["TopoVolume#1"], "size": 1.0}),
    )
    .expect("size");
    assert!(err_text(mesh_run(&mut s)).contains("budget"));
}

#[test]
fn a_stale_mesh_can_be_deleted_by_target_and_by_id() {
    for by_target in [true, false] {
        let mut s = brick_session([2.0; 3], 1.0);
        let (_, d) = mesh_run(&mut s).expect("run");
        let mid = d.outputs["mesh"][0];
        run(&mut s, &DeleteVolume { volume: vol(1) }).expect("delete volume");
        assert_eq!(
            s.state().count_of(EntityKind::MeshCell),
            8,
            "mesh outlives it"
        );
        let params = if by_target {
            json!({"targets": ["TopoVolume#1"]})
        } else {
            json!({"meshes": [mid.to_string()]})
        };
        run_json(&mut s, "mesh.delete", params).expect("delete stale mesh");
        for k in [
            EntityKind::Mesh,
            EntityKind::MeshCell,
            EntityKind::MeshFace,
            EntityKind::MeshVertex,
        ] {
            assert_eq!(s.state().count_of(k), 0, "{k:?} (by_target {by_target})");
        }
    }
}

#[test]
fn user_fallbacks_are_tried_through_mesh_run() {
    let mut reg = stubs::registry();
    reg.register(Box::new(stubs::AlwaysFail))
        .expect("always_fail");
    let mut s = Session::new(engine_with(reg, DerivedCache::default()));
    run(&mut s, &brick_op([2.0; 3])).expect("brick");
    run_json(
        &mut s,
        "mesh.size",
        json!({"targets": ["TopoVolume#1"], "size": 1.0}),
    )
    .expect("size");
    run_json(
        &mut s,
        "mesh.scheme",
        json!({"targets": ["TopoVolume#1"], "scheme": "sweep",
               "params": {"source": ["TopoFace#5"], "target": ["TopoFace#6"]}}),
    )
    .expect("volume scheme");
    run_json(
        &mut s,
        "mesh.scheme",
        json!({"targets": ["TopoFace#5"], "scheme": "always_fail", "fallbacks": ["map"]}),
    )
    .expect("face scheme");
    mesh_run(&mut s).expect("the fallback recovers");
    let layer = s.state().layer::<MeshLayer>().expect("layer");
    let (_, e) = layer.meshes().next().expect("mesh");
    let plan = e.plan.as_ref().expect("plan");
    assert_eq!(plan.rejections.len(), 1, "always_fail was rejected once");
    assert_eq!(
        plan.rejections
            .attempts()
            .next()
            .expect("one")
            .scheme
            .as_str(),
        "always_fail"
    );
    assert_eq!(e.mesh().cell_count(), 8);
}

#[test]
fn an_infeasible_hard_request_on_another_volume_does_not_break_a_run() {
    let mut s = session();
    run(&mut s, &brick_op([4.0; 3])).expect("a");
    run(
        &mut s,
        &CreateBrick {
            origin: [10.0, 0.0, 0.0],
            size: [4.0; 3],
        },
    )
    .expect("b");
    for (e, n) in [("TopoEdge#13", 3), ("TopoEdge#15", 5)] {
        run_json(
            &mut s,
            "mesh.intervals",
            json!({"targets": [e], "count": n}),
        )
        .expect("b edge");
    }
    run_json(
        &mut s,
        "mesh.size",
        json!({"targets": ["TopoVolume#1"], "size": 1.0}),
    )
    .expect("a size");
    run_json(
        &mut s,
        "mesh.scheme",
        json!({"targets": ["TopoVolume#1"], "scheme": "sweep",
               "params": {"source": ["TopoFace#5"], "target": ["TopoFace#6"]}}),
    )
    .expect("a scheme");
    run_json(&mut s, "mesh.run", json!({"targets": ["TopoVolume#1"]})).expect("a only");
    assert_eq!(s.state().count_of(EntityKind::MeshCell), 64);
}

#[test]
fn elongated_bricks_are_not_refused_by_the_budget() {
    // Estimate is per volume from the geometric mean of its edge counts: exact for a brick.
    for (dims, h, cells) in [
        ([1.0, 1.0, 1000.0], 1.0, 1000),
        ([1.0, 1.0, 100.0], 0.1, 100_000),
        ([100.0, 100.0, 1.0], 1.0, 10_000),
    ] {
        let mut s = brick_session(dims, h);
        let r = mesh_run(&mut s);
        assert!(
            r.is_ok(),
            "{dims:?}: {}",
            r.err().map(|e| e.to_string()).unwrap_or_default()
        );
        assert_eq!(s.state().count_of(EntityKind::MeshCell), cells, "{dims:?}");
    }
}

#[test]
fn a_subnormal_size_is_rejected_when_set() {
    let mut s = session();
    run(&mut s, &brick_op([1.0; 3])).expect("brick");
    for size in [5e-324, 0.0, -1.0] {
        let r = run_json(
            &mut s,
            "mesh.size",
            json!({"targets": ["TopoVolume#1"], "size": size}),
        );
        assert!(r.is_err(), "mesh.size {size}");
    }
    assert!(run_json(&mut s, "mesh.default_size", json!({"size": 5e-324})).is_err());
}
