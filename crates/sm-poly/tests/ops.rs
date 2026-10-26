//! Acceptance bullet 8: the op end to end through a session.
#![allow(missing_docs)]

mod common;

use common::id;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::json;
use sm_core::{EntityId, EntityKind, Point3};
use sm_layer_api::{CoreLayers, LayerResultExt, MeshEntry, MeshLayer, ModelLayer, commit_topology, insert_mesh, topo_txn};
use sm_mesh_api::reference::{ReferenceMesh, hex_block};
use sm_mesh_api::{
    GeomTol, MeshEntityKind, MeshRead, SerialAlloc, validate,
    validate_geometry,
};
use sm_op_api::{
    ApplyCtx, Delta, DerivedCache, Engine, Hash256, Meta, Op, OpError, OpRecord, QuietControl, Registry, Services,
    Session, TimelineError, TypedOp,
};
use sm_poly::{PolyPlugin, poly_stats};
use sm_testkit::meshes;
use sm_topo_api::fixtures::brick;

/// Test-only op: a brick model of `n * h` plus a mesh of the given kind classified on it.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct InsertMesh {
    kind: String,
    n: [u32; 3],
    h: [f64; 3],
}

/// The allocator state a fresh build of `m` leaves behind.
fn alloc_after(m: &dyn MeshRead) -> SerialAlloc {
    let mut state = vec![
        (EntityKind::Mesh, 2),
        (EntityKind::MeshVertex, m.count(MeshEntityKind::Vertex) as u64 + 1),
        (EntityKind::MeshFace, m.count(MeshEntityKind::Face) as u64 + 1),
        (EntityKind::MeshCell, m.count(MeshEntityKind::Cell) as u64 + 1),
    ];
    if m.count(MeshEntityKind::Edge) > 0 {
        state.push((EntityKind::MeshEdge, m.count(MeshEntityKind::Edge) as u64 + 1));
    }
    SerialAlloc::from_state(state).expect("alloc")
}

fn build(kind: &str, n: [u32; 3], h: [f64; 3]) -> ReferenceMesh {
    match kind {
        "tet" => meshes::tet_block(n, h).expect("tet_block").mesh,
        "hex" => hex_block(n, h, &mut SerialAlloc::new()).expect("hex_block"),
        "mixed" => meshes::mixed_transition().expect("mixed_transition").mesh,
        other => panic!("unknown kind {other}"),
    }
}

impl TypedOp for InsertMesh {
    const NAME: &'static str = "test.insert_mesh";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let size = [self.n[0] as f64 * self.h[0], self.n[1] as f64 * self.h[1], self.n[2] as f64 * self.h[2]];
        let (ids, txn) = topo_txn(ctx, |topo, _| brick(topo, Point3::new(0.0, 0.0, 0.0), size).or_op())?;
        commit_topology(ctx, &txn, &[("volume", &[ids.volume]), ("faces", &ids.faces), ("edges", &ids.edges)])?;
        let quanta = ctx.layer::<ModelLayer>()?.hash_quanta();
        let before = ctx.layer::<MeshLayer>()?.alloc().clone();
        let mesh = build(&self.kind, self.n, self.h);
        let after = alloc_after(&mesh);
        let entry = MeshEntry::new(Box::new(mesh), vec![ids.volume], None, quanta);
        insert_mesh(ctx, entry, &before, after, "mesh")
    }
}

fn engine() -> Engine {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::default()).expect("layers");
    reg.install(&PolyPlugin).expect("poly");
    reg.register::<InsertMesh>().expect("test op");
    Engine::new(std::sync::Arc::new(reg), Services::new(), std::sync::Arc::new(DerivedCache::default())).expect("engine")
}

fn run(s: &mut Session, op: &dyn Op) -> Result<(Hash256, Delta), TimelineError> {
    let q = QuietControl::new();
    s.apply(op, Meta::new("test", "", ""), &q.control())
}

fn run_json(s: &mut Session, name: &str, params: serde_json::Value) -> Result<(Hash256, Delta), TimelineError> {
    let rec = OpRecord::new(name, 1, params).expect("record");
    let q = QuietControl::new();
    s.apply_record(rec, Meta::new("test", "", ""), &q.control())
}

fn session_with(kind: &str) -> Session {
    let mut s = Session::new(engine());
    run(&mut s, &InsertMesh { kind: kind.into(), n: [3, 3, 3], h: [1.0; 3] }).expect("insert");
    s
}

fn mesh1() -> EntityId {
    id(EntityKind::Mesh, 1)
}

#[test]
fn acc8_default_keeps_the_source() {
    let mut s = session_with("tet");
    let before = s.state().clone();
    let (_, d) = run_json(&mut s, "mesh.to_poly", json!({"mesh": "Mesh#1"})).expect("to_poly");
    let st = s.state();
    assert_eq!(d.outputs["mesh"], vec![id(EntityKind::Mesh, 2)]);
    // both live; id counts add up
    assert!(st.contains(id(EntityKind::Mesh, 1)) && st.contains(id(EntityKind::Mesh, 2)));
    let layer = st.layer::<MeshLayer>().unwrap();
    assert_eq!(layer.len(), 2);
    let (src, poly) = (layer.get(mesh1()).unwrap(), layer.get(id(EntityKind::Mesh, 2)).unwrap());
    assert_eq!(src.mesh().count(MeshEntityKind::Cell), 162);
    assert_eq!(poly.mesh().count(MeshEntityKind::Cell), 64);
    for (kind, mk) in [
        (EntityKind::MeshCell, MeshEntityKind::Cell),
        (EntityKind::MeshFace, MeshEntityKind::Face),
        (EntityKind::MeshVertex, MeshEntityKind::Vertex),
    ] {
        assert_eq!(st.count_of(kind), src.mesh().count(mk) + poly.mesh().count(mk));
        assert_eq!(st.count_of(kind) - before.count_of(kind), poly.mesh().count(mk));
    }
    assert_eq!(st.count_of(EntityKind::Mesh), 2);
    assert_eq!(d.created.iter().filter(|i| i.kind() == EntityKind::MeshCell).count(), 64);
    assert!(d.removed.is_empty());
    // copied entry data
    assert_eq!(poly.selection, src.selection);
    assert_eq!(poly.quanta.length.to_bits(), src.quanta.length.to_bits());
    assert_eq!(poly.quanta.param.to_bits(), src.quanta.param.to_bits());
    assert_eq!(poly.plan.is_some(), src.plan.is_some());
    assert!(!poly.stale);
    // valid, planar, and the volume is conserved
    assert!(validate(poly.mesh()).is_empty());
    assert!(validate_geometry(poly.mesh(), &GeomTol::default()).is_empty());
    assert!((poly_stats(poly.mesh()).volume_sum - 27.0).abs() < 1e-11);
    // the source is untouched
    assert_eq!(src.mesh().count(MeshEntityKind::Cell), 162);
    assert_eq!(d.touched_layers, vec!["meshes".to_owned()]);
    // the new mesh is marked as derived from the source: a copy, `A -> [A, A']`
    assert_eq!(d.successors[&mesh1()], vec![mesh1(), id(EntityKind::Mesh, 2)]);
    assert_eq!(d.successors.len(), 1);
}

#[test]
fn acc8_keep_source_false_retires_the_source() {
    let mut s = session_with("tet");
    let (_, d) = run_json(&mut s, "mesh.to_poly", json!({"mesh": "Mesh#1", "keep_source": false})).expect("to_poly");
    let st = s.state();
    let layer = st.layer::<MeshLayer>().unwrap();
    assert_eq!(layer.len(), 1);
    assert!(layer.get(mesh1()).is_none());
    let poly_id = d.outputs["mesh"][0];
    assert!(layer.get(poly_id).is_some());
    assert!(!st.contains(mesh1()));
    // the source's Mesh* ids are all retired; only the poly mesh's are live
    let poly = layer.get(poly_id).unwrap().mesh();
    assert_eq!(st.count_of(EntityKind::MeshCell), poly.count(MeshEntityKind::Cell));
    assert_eq!(st.count_of(EntityKind::MeshFace), poly.count(MeshEntityKind::Face));
    assert_eq!(st.count_of(EntityKind::MeshVertex), poly.count(MeshEntityKind::Vertex));
    assert_eq!(st.count_of(EntityKind::Mesh), 1);
    assert_eq!(d.removed.iter().filter(|i| i.kind() == EntityKind::MeshCell).count(), 162);
    // the model is untouched and the selection still names the volume
    assert_eq!(layer.get(poly_id).unwrap().selection, vec![id(EntityKind::TopoVolume, 1)]);
    // a replacement: `source -> [new]`
    assert_eq!(d.successors[&mesh1()], vec![poly_id]);
}

fn assert_refused(s: &mut Session, params: serde_json::Value) -> TimelineError {
    let h = s.state().hash();
    let e = run_json(s, "mesh.to_poly", params).expect_err("must be refused");
    assert_eq!(s.state().hash(), h, "a refusal must not change the state");
    e
}

fn is_invalid(e: &TimelineError) -> bool {
    format!("{e:?}").contains("Invalid")
}

#[test]
fn acc8_refusals_leave_the_state_untouched() {
    // a second conversion of the poly mesh
    let mut s = session_with("tet");
    run_json(&mut s, "mesh.to_poly", json!({"mesh": "Mesh#1"})).expect("first");
    let e = assert_refused(&mut s, json!({"mesh": "Mesh#2"}));
    assert!(is_invalid(&e), "{e:?}");
    // a hex mesh
    let mut s = session_with("hex");
    let e = assert_refused(&mut s, json!({"mesh": "Mesh#1"}));
    assert!(is_invalid(&e), "{e:?}");
    // a mesh with one non-tet cell
    let mut s = session_with("mixed");
    let e = assert_refused(&mut s, json!({"mesh": "Mesh#1"}));
    assert!(is_invalid(&e), "{e:?}");
    // an unknown mesh
    let mut s = session_with("tet");
    let _ = assert_refused(&mut s, json!({"mesh": "Mesh#9"}));
    // a mesh id of the wrong kind
    let _ = assert_refused(&mut s, json!({"mesh": "TopoVolume#1"}));
    // decode-time refusals: an unknown field, `targets`, a missing `mesh`
    for p in [json!({"mesh": "Mesh#1", "bogus": 1}), json!({"mesh": "Mesh#1", "targets": ["TopoVolume#1"]}), json!({"keep_source": true})] {
        let h = s.state().hash();
        let e = run_json(&mut s, "mesh.to_poly", p.clone()).expect_err("decode must fail");
        assert_eq!(s.state().hash(), h, "{p}");
        let t = format!("{e:?}");
        assert!(t.contains("Decode") || t.contains("Invalid"), "{t}");
    }
}

#[test]
fn acc8_poly_info_equals_poly_stats() {
    let mut s = session_with("tet");
    let (_, d) = run_json(&mut s, "mesh.to_poly", json!({"mesh": "Mesh#1"})).expect("to_poly");
    let pm = d.outputs["mesh"][0];
    let q = QuietControl::new();
    let v = s
        .engine()
        .run_query(s.state(), "mesh.poly_info", &json!({"mesh": pm.to_string()}), &q.control())
        .expect("query");
    let layer = s.state().layer::<MeshLayer>().unwrap();
    let want = serde_json::to_value(poly_stats(layer.get(pm).unwrap().mesh())).unwrap();
    assert_eq!(v, want);
    assert_eq!(v["cells"], 64);
    assert!(s.engine().run_query(s.state(), "mesh.poly_info", &json!({"mesh": "Mesh#9"}), &q.control()).is_err());
}

#[test]
fn acc8_cold_replay_has_no_divergence() {
    let mut s = session_with("tet");
    run_json(&mut s, "mesh.to_poly", json!({"mesh": "Mesh#1", "keep_source": false})).expect("to_poly");
    run_json(&mut s, "mesh.to_poly", json!({"mesh": "Mesh#2"})).expect_err("poly of poly is refused");
    let head = s.position().expect("head");
    let q = QuietControl::new();
    let cold = s.state_at_from_root(head, &q.control()).expect("cold replay (no Diverged)");
    assert_eq!(cold.hash(), s.state().hash());
    // and a fresh engine replays the same timeline to the same state
    let s2 = Session::open(engine(), s.timeline().clone(), s.head().clone(), &q.control()).expect("reopen");
    assert_eq!(s2.state().hash(), s.state().hash());
}
