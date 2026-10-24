//! The `view.tessellate` query through a real `Session`: schema, wire encoding, equality with
//! the library call, no state change, thread-count and cold-replay determinism.

mod common;

use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sm_core::{EntityId, EntityKind, Point3};
use sm_layer_api::{CoreLayers, LayerResultExt, ModelLayer, commit_topology, topo_txn};
use sm_op_api::{ApplyCtx, DerivedCache, Engine, Meta, OpError, QuietControl, Registry, Services, Session, TypedOp};
use sm_tess::{TessOptions, TessPlugin, TessellateReply, base64_decode, tessellate_entities};
use sm_topo_api::fixtures::{brick, cylinder, sphere};

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct MakeShape {
    kind: String,
}

impl TypedOp for MakeShape {
    const NAME: &'static str = "test.make_shape";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let kind = self.kind.clone();
        let (faces, txn) = topo_txn(ctx, |topo, _| match kind.as_str() {
            "brick" => Ok(brick(topo, Point3::new(-5.0, -5.0, -5.0), [10.0; 3]).or_op()?.faces.to_vec()),
            "cylinder" => Ok(cylinder(topo, 2.0, 3.0).or_op()?.faces.to_vec()),
            "sphere" => Ok(vec![sphere(topo, 1.5).or_op()?.face]),
            other => Err(OpError::Invalid(format!("unknown shape {other}"))),
        })?;
        commit_topology(ctx, &txn, &[("faces", &faces)])
    }
}

fn engine(with_core: bool, cache: DerivedCache) -> Engine {
    let mut reg = Registry::new();
    if with_core {
        reg.install(&CoreLayers::default()).expect("core layers");
        reg.register::<MakeShape>().expect("op");
    }
    reg.install(&TessPlugin).expect("tess");
    Engine::new(Arc::new(reg), Services::new(), Arc::new(cache)).expect("engine")
}

fn session_with(shapes: &[&str], cache: DerivedCache) -> Session {
    let mut s = Session::new(engine(true, cache));
    let q = QuietControl::new();
    for k in shapes {
        let op = MakeShape { kind: (*k).to_owned() };
        s.apply(&op, Meta::new("test", "", ""), &q.control()).expect("apply");
    }
    s
}

fn head_of(s: &mut Session, kind: &str) -> sm_op_api::Hash256 {
    let q = QuietControl::new();
    s.apply(&MakeShape { kind: kind.to_owned() }, Meta::new("test", "", ""), &q.control()).expect("apply").0
}

fn query(s: &Session, input: &Value) -> Result<Value, sm_op_api::RegistryError> {
    let q = QuietControl::new();
    s.engine().run_query(s.state(), "view.tessellate", input, &q.control())
}

fn bytes_f32(s: &str) -> Vec<f32> {
    base64_decode(s).unwrap().chunks(4).map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]])).collect()
}

fn bytes_u32(s: &str) -> Vec<u32> {
    base64_decode(s).unwrap().chunks(4).map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]])).collect()
}

fn id(kind: EntityKind, n: u64) -> EntityId {
    EntityId::new(kind, n).unwrap()
}

#[test]
fn plugin_registers_the_query_with_entity_kinds_in_its_schema() {
    let e = engine(true, DerivedCache::default());
    let cat = e.registry().query_catalog();
    let info = cat.into_iter().map(|r| r.unwrap()).find(|q| q.name == "view.tessellate").expect("registered");
    assert_eq!(info.tool_name, "view__tessellate");
    let schema = serde_json::to_value(&info.schema).unwrap();
    let kinds = &schema["properties"]["entities"]["x-entity-kind"];
    assert_eq!(kinds, &json!(["TopoFace", "TopoVolume", "TopoBody", "TopoEdge"]));
}

#[test]
fn plugin_alone_registers_exactly_one_query_and_no_ops() {
    let mut reg = Registry::new();
    reg.install(&TessPlugin).expect("tess");
    // A fresh registry already holds the built-in ops (e.g. `core.batch`); the plugin adds none.
    let builtin: Vec<String> = Registry::new().op_names().iter().map(|s| (*s).to_owned()).collect();
    let with_plugin: Vec<String> = reg.op_names().iter().map(|s| (*s).to_owned()).collect();
    assert_eq!(with_plugin, builtin, "the plugin registers no ops");
    assert!(with_plugin.iter().all(|n| !n.starts_with("view.")));
    assert_eq!(reg.query_names(), vec!["view.tessellate"]);
}

#[test]
fn reply_buffers_equal_the_library_result() {
    let s = session_with(&["brick", "cylinder", "sphere"], DerivedCache::default());
    let faces: Vec<EntityId> = s.state().entities_of(EntityKind::TopoFace).collect();
    assert_eq!(faces.len(), 6 + 3 + 1);
    let reply: TessellateReply = serde_json::from_value(query(&s, &json!({ "entities": faces, "chord_tol": 0.002 })).unwrap()).unwrap();
    let view = s.state().layer::<ModelLayer>().unwrap().topo();
    let o = TessOptions { chord_tol: 0.002, ..TessOptions::default() };
    let t = tessellate_entities(view, &faces, &o).unwrap();
    assert_eq!(reply.vertex_count as usize, t.vertex_count());
    assert_eq!(reply.triangle_count as usize, t.triangle_count());
    assert_eq!(bytes_f32(&reply.positions), t.positions);
    assert_eq!(bytes_u32(&reply.indices), t.indices);
    assert_eq!(bytes_u32(&reply.edge_indices), t.edge_indices);
    assert_eq!(reply.faces.len(), faces.len());
    assert_eq!(reply.edges.len(), t.edges.len());
    assert_eq!(reply.faces[0].first_index, 0);
    assert!(reply.faces.iter().all(|f| !f.boundary_only), "brick, cylinder and sphere faces are all fully resolved");
    assert_eq!((reply.chord_tol, reply.angle_tol, reply.max_edge), (0.002, o.angle_tol, None));
}

#[test]
fn query_does_not_change_state_and_accepts_volumes_and_edges() {
    let s = session_with(&["brick"], DerivedCache::default());
    let before = s.state().hash();
    let vol = id(EntityKind::TopoVolume, 1);
    let r: TessellateReply = serde_json::from_value(query(&s, &json!({ "entities": [vol] })).unwrap()).unwrap();
    assert_eq!((r.triangle_count, r.vertex_count), (12, 8));
    let edge = id(EntityKind::TopoEdge, 1);
    let r: TessellateReply = serde_json::from_value(query(&s, &json!({ "entities": [edge] })).unwrap()).unwrap();
    assert_eq!((r.triangle_count, r.vertex_count, r.edges.len()), (0, 2, 1));
    assert_eq!(s.state().hash(), before);
}

#[test]
fn default_chord_tolerance_is_relative_to_the_scene() {
    let s = session_with(&["sphere"], DerivedCache::default());
    let face = id(EntityKind::TopoFace, 1);
    let r: TessellateReply = serde_json::from_value(query(&s, &json!({ "entities": [face] })).unwrap()).unwrap();
    // The scene box comes from vertices and edge samples: the sphere's two poles and its meridian
    // span x in [0, 1.5] and z in [-1.5, 1.5], so the diagonal is sqrt(1.5^2 + 3^2).
    let diag = (1.5f64 * 1.5 + 3.0 * 3.0).sqrt();
    assert!((r.chord_tol - 1e-3 * diag).abs() < 1e-12, "{}", r.chord_tol);
}

#[test]
fn bad_input_is_an_error_not_a_panic() {
    let s = session_with(&["brick"], DerivedCache::default());
    let ghost = id(EntityKind::TopoFace, 999);
    assert!(query(&s, &json!({ "entities": [ghost] })).is_err());
    assert!(query(&s, &json!({ "entities": [id(EntityKind::TopoVertex, 1)] })).is_err());
    assert!(query(&s, &json!({ "entities": [id(EntityKind::TopoFace, 1)], "chord_tol": -1.0 })).is_err());
    assert!(query(&s, &json!({ "entities": [], "bogus": 1 })).is_err());
    // Empty selection is fine.
    let r: TessellateReply = serde_json::from_value(query(&s, &json!({ "entities": [] })).unwrap()).unwrap();
    assert_eq!(r.triangle_count, 0);
}

#[test]
fn missing_model_layer_is_reported() {
    let e = engine(false, DerivedCache::default());
    let state = e.registry().new_state();
    let q = QuietControl::new();
    let err = e.run_query(&state, "view.tessellate", &json!({ "entities": [] }), &q.control()).unwrap_err();
    assert!(err.to_string().contains("model"), "{err}");
}

#[test]
fn query_bytes_are_identical_across_threads_runs_and_cold_replay() {
    let mut s = session_with(&["brick", "cylinder"], DerivedCache::disabled());
    let h = head_of(&mut s, "sphere");
    let input = json!({ "entities": [id(EntityKind::TopoBody, 1), id(EntityKind::TopoBody, 2), id(EntityKind::TopoBody, 3)], "chord_tol": 0.001 });
    let head = s.state().clone();
    let run = |state: &sm_op_api::State, threads: usize| -> String {
        common::pool(threads, || {
            let q = QuietControl::new();
            s.engine().run_query(state, "view.tessellate", &input, &q.control()).unwrap().to_string()
        })
    };
    let base = run(&head, 1);
    assert_eq!(base, run(&head, 1));
    assert_eq!(base, run(&head, 4));
    // Cold replay from the root reproduces the state and therefore the reply.
    let q = QuietControl::new();
    let replayed = s.state_at_from_root(h, &q.control()).expect("cold replay");
    assert_eq!(replayed.hash(), head.hash());
    assert_eq!(base, run(&replayed, 4));
}
