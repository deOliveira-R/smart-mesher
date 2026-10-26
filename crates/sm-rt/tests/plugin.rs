//! The `rt` plugin: `rt.info`, `rt.check`, `rt.locate` over a session with `CoreLayers`.
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::json;
use sm_core::{EntityId, Point3};
use sm_layer_api::{CoreLayers, LayerResultExt, MeshEntry, MeshLayer, ModelLayer, commit_topology, insert_mesh, topo_txn};
use sm_mesh_api::reference::hex_block;
use sm_op_api::{ApplyCtx, DerivedCache, Engine, Meta, OpError, QuietControl, Registry, Services, Session, TypedOp};
use sm_rt::RtPlugin;
use sm_topo_api::fixtures::brick;
use std::sync::Arc;

/// A 4x4x4 brick with a matching hex mesh inserted by classification.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct HexModel {
    n: u32,
    /// Also tag the first brick face as a response boundary named `wall`.
    #[serde(default)]
    wall: bool,
}

impl TypedOp for HexModel {
    const NAME: &'static str = "test.hex_model";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let n = f64::from(self.n);
        let (b, txn) = topo_txn(ctx, |topo, _| brick(topo, Point3::new(0.0, 0.0, 0.0), [n, n, n]).or_op())?;
        commit_topology(ctx, &txn, &[("volume", &[b.volume])])?;
        if self.wall {
            let face = b.faces[0];
            sm_layer_api::model_txn(
                ctx,
                |model, _| {
                    let w = model.add_boundary("wall", sm_model_api::BoundaryKind::Response).map_err(|e| OpError::Failed(e.to_string()))?;
                    model.assign_response(w, &[face]).map_err(|e| OpError::Failed(e.to_string()))
                },
                |_, _| Vec::new(),
            )?;
        }
        let alloc0 = ctx.layer::<MeshLayer>()?.alloc().clone();
        let mut alloc = alloc0.clone();
        let mesh = hex_block([self.n; 3], [1.0; 3], &mut alloc).map_err(|e| OpError::Failed(e.to_string()))?;
        let quanta = ctx.layer::<ModelLayer>()?.hash_quanta();
        let entry = MeshEntry::new(Box::new(mesh), vec![b.volume], None, quanta);
        insert_mesh(ctx, entry, &alloc0, alloc, "mesh")
    }
}

fn session() -> Session {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::default()).unwrap();
    reg.install(&RtPlugin).unwrap();
    reg.register::<HexModel>().unwrap();
    Session::new(Engine::new(Arc::new(reg), Services::new(), Arc::new(DerivedCache::default())).unwrap())
}

fn mesh_id(s: &Session) -> EntityId {
    s.state().layer::<MeshLayer>().unwrap().meshes().next().unwrap().0
}

#[test]
fn r2_rt_queries_report_info_check_and_locate() {
    let mut s = session();
    let q = QuietControl::new();
    s.apply(&HexModel { n: 4, wall: false }, Meta::new("test", "", ""), &q.control()).expect("model + mesh");
    let mesh = mesh_id(&s);
    let info = s.engine().run_query(s.state(), "rt.info", &json!({ "mesh": mesh.to_string() }), &q.control()).expect("rt.info");
    assert_eq!(info["cells"], 64);
    assert_eq!(info["faces"], 3 * 4 * 4 * 5);
    assert_eq!(info["faceted_faces"], 0);
    assert_eq!(info["caps"], json!(["volume_mesh", "exact_planar"]));
    assert_eq!(info["bounds"], json!([[0.0, 0.0, 0.0], [4.0, 4.0, 4.0]]));
    assert_eq!(info["regions"].as_array().unwrap().len(), 1);
    assert_eq!(info["surfaces"], 6);
    assert!(info["bytes_per_face"].as_f64().unwrap() > 0.0);
    // A second call hits the derived cache and gives the same answer.
    let again = s.engine().run_query(s.state(), "rt.info", &json!({ "mesh": mesh.to_string() }), &q.control()).unwrap();
    assert_eq!(info, again);
    let check = s.engine().run_query(s.state(), "rt.check", &json!({ "mesh": mesh.to_string() }), &q.control()).unwrap();
    assert_eq!(check["ok"], true);
    assert_eq!(check["issues"], json!([]));
    let loc = s
        .engine()
        .run_query(s.state(), "rt.locate", &json!({ "mesh": mesh.to_string(), "points": [[0.5, 0.5, 0.5], [3.5, 3.5, 3.5], [9.0, 0.0, 0.0], [1.0, 0.5, 0.5]] }), &q.control())
        .unwrap();
    let loc = loc.as_array().unwrap();
    assert_eq!(loc[0]["cell_index"], 0);
    assert_eq!(loc[1]["cell_index"], 63);
    assert!(loc[2]["cell"].is_null() && loc[2]["region"].is_null());
    assert!(loc[3]["on_face"].is_string(), "a point on an interior face is flagged");
    assert!(loc[0]["region"].is_string());
    // Errors are typed, not panics.
    let bad = s.engine().run_query(s.state(), "rt.info", &json!({ "mesh": "Mesh#999" }), &q.control());
    assert!(bad.is_err());
    let bad_dir = s.engine().run_query(s.state(), "rt.locate", &json!({ "mesh": mesh.to_string(), "points": [[0.5, 0.5, 0.5]], "dir": [2.0, 0.0, 0.0] }), &q.control());
    assert!(bad_dir.is_err());
}

#[test]
fn r2_rt_queries_are_listed_with_schemas() {
    let s = session();
    let reg = s.engine().registry();
    for n in ["rt.info", "rt.check", "rt.locate"] {
        assert!(reg.query_names().contains(&n), "{n} registered");
    }
    // Every schema generates and the entity-kind hint is present on the mesh argument.
    for q in reg.query_catalog() {
        let q = q.expect("schema");
        if q.name.starts_with("rt.") {
            assert!(serde_json::to_string(&q.schema).unwrap().contains("x-entity-kind"), "{}", q.name);
        }
    }
}

#[test]
fn r2_rt_check_reports_an_invalid_boundary_as_a_located_issue() {
    let mut s = session();
    let q = QuietControl::new();
    s.apply(&HexModel { n: 2, wall: true }, Meta::new("test", "", ""), &q.control()).expect("model + mesh + wall");
    let mesh = mesh_id(&s);
    let args = |a: f64| json!({ "mesh": mesh.to_string(), "responses": [{ "boundary": "wall", "response": { "albedo": a } }] });
    let ok = s.engine().run_query(s.state(), "rt.check", &args(0.5), &q.control()).unwrap();
    assert_eq!(ok["ok"], true, "{ok}");
    let bad = s.engine().run_query(s.state(), "rt.check", &args(2.0), &q.control()).unwrap();
    assert_eq!(bad["ok"], false, "{bad}");
    let issues = bad["issues"].as_array().unwrap();
    assert_eq!(issues.len(), 1);
    assert_eq!(issues[0]["code"], "boundary.bad_albedo");
    assert!(bad["error"].is_string());
    // The tracker-building queries refuse the same input with a typed error.
    assert!(s.engine().run_query(s.state(), "rt.info", &args(2.0), &q.control()).is_err());
}
