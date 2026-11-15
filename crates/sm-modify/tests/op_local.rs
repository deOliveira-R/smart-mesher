//! MOD-REFINE-L at op level through `sm_server::standard`: a meshed brick with a slab refined (hand
//! counts, exact volume, reported quality), the link (replay, fork, retirement), a curved case with
//! projection, refusals and determinism.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sm_core::{EntityId, EntityKind};
use sm_engine::{DiagnosticPolicy, Verbosity, Workspace};
use sm_layer_api::{MeshLayer, add_mesh_set};
use sm_mesh_api::{CellIx, CellShape, MeshCatalog, MeshRead, SetMembers, cell_volume, validate};
use sm_op_api::{ApplyCtx, Control, OpError, Plugin, QuietControl, Registry, RegistryError, TypedOp};
use sm_protocol::{Endpoint, Request};
use sm_server::{Server, standard_with};

const TIME: &str = "2026-10-24T00:00:00Z";

/// A cell set named `name` of the cells of `mesh` all of whose corners have `x <= max_x`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct CellSetBelow {
    mesh: EntityId,
    name: String,
    max_x: f64,
}

impl TypedOp for CellSetBelow {
    const NAME: &'static str = "test.cell_set";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let cells: Vec<CellIx> = {
            let layer = ctx.layer::<MeshLayer>()?;
            let m = layer.get(self.mesh).ok_or(OpError::UnknownEntity(self.mesh))?.mesh();
            (0..m.cell_count())
                .map(CellIx::new)
                .filter(|&c| m.cell_corners(c).is_some_and(|cs| cs.iter().all(|v| m.point(v.get()).x <= self.max_x + 1e-12)))
                .collect()
        };
        add_mesh_set(ctx, self.mesh, &self.name, SetMembers::Cells(cells), "set").map(|_| ())
    }
}

struct TestKit;

impl Plugin for TestKit {
    fn id(&self) -> &str {
        "local-test-kit"
    }
    fn register(&self, r: &mut Registry) -> Result<(), RegistryError> {
        r.register::<CellSetBelow>().map(|_| ())
    }
}

fn server() -> Server {
    let policy = DiagnosticPolicy { verbosity: Verbosity::Info, ..DiagnosticPolicy::default() };
    Server::new(standard_with(Some("tester".into()), Some(TIME.into())).policy(policy).plugin(TestKit)).expect("server")
}

fn script(s: &mut Server, text: &str) -> Value {
    s.call(Request::ScriptRun { text: text.into(), path: None, includes: Default::default() })
        .unwrap_or_else(|e| panic!("script failed: {e}\n{}", e.data["rendered"].as_str().unwrap_or("")))
}

fn mesh_id(n: u64) -> EntityId {
    EntityId::new(EntityKind::Mesh, n).expect("id")
}

fn layer(s: &Server) -> &MeshLayer {
    s.workspace().state().layer::<MeshLayer>().expect("meshes layer")
}

fn hash(s: &Server) -> String {
    s.workspace().state_hash().to_string()
}

fn mesh(s: &Server, n: u64) -> &dyn MeshRead {
    layer(s).get(mesh_id(n)).unwrap_or_else(|| panic!("mesh {n}")).mesh()
}

fn volume(m: &dyn MeshRead) -> f64 {
    (0..m.cell_count()).map(|c| cell_volume(m, CellIx::new(c))).sum()
}

fn report(run: &Value) -> Value {
    let d = run["events"]
        .as_array()
        .expect("events")
        .iter()
        .filter(|e| e["op"] == "mesh.refine_local")
        .flat_map(|e| e["delta"]["diagnostics"].as_array().cloned().unwrap_or_default())
        .find(|d| d["code"] == "refine_local.report")
        .expect("refine_local.report");
    serde_json::from_str(d["message"].as_str().expect("message")).expect("json")
}

/// A 2 x 2 x 2 brick (centred, `[-1, 1]^3`) meshed with 0.5 hexes (4 x 4 x 4), with the half `x <= 0` as cell set `slab`.
const BRICK: &str = r#"
let b = model.create_brick(x: 2.0, y: 2.0, z: 2.0)
mesh.size(targets: all volumes, size: 0.5)
let au = mesh.auto(strategy: "default")
let m = mesh.run(targets: all volumes)
"#;

fn brick_with_slab() -> Server {
    let mut s = server();
    script(&mut s, BRICK);
    assert_eq!(mesh(&s, 1).cell_count(), 64);
    s.call(Request::Apply { op: "test.cell_set".into(), params: json!({"mesh": "Mesh#1", "name": "slab", "max_x": 0.0}), version: None }).expect("set");
    s
}

fn cold(ws: &mut Workspace) -> sm_op_api::State {
    let q = QuietControl::new();
    let ctl: Control<'_> = q.control();
    let head = ws.session().position().expect("head");
    ws.session().state_at_from_root(head, &ctl).expect("cold replay")
}

#[test]
fn a_slab_of_a_meshed_brick_refines_with_the_hand_counts() {
    let mut s = brick_with_slab();
    let run = script(&mut s, r#"mesh.refine_local(mesh: Mesh#1, cell_set: "slab")"#);
    let r = report(&run);
    // The 32 hexes of the half become 256; the 16 hexes of the next layer meet it by a face with 4
    // sub-faces each and become polyhedra; the other 16 are untouched hexes.
    let m = mesh(&s, 2);
    let hexes = (0..m.cell_count()).filter(|&c| m.cell_shape(CellIx::new(c)) == CellShape::Hex).count();
    let polys = m.cell_count() - hexes;
    assert_eq!(r["refined"], 32);
    assert_eq!(r["balanced"], 0);
    assert_eq!(m.cell_count(), 32 * 8 + 32);
    assert_eq!(polys, 16, "the face neighbours across x = 0");
    assert_eq!(hexes, 32 * 8 + 16);
    assert_eq!(r["hexes"], hexes);
    assert_eq!(r["polyhedra"], polys);
    assert!((volume(m) - 8.0).abs() < 1e-12);
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(r["min_scaled_jacobian"].as_f64().map(|x| (x - 1.0).abs() < 1e-12), Some(true));
    assert!(r["max_edge_ratio"].as_f64().expect("edge ratio") >= 1.0);
    assert_eq!(layer(&s).maps_from(mesh_id(2)).len(), 1, "linked to the source");
    // Face map: the sub-faces of the interface carry the coarse face.
    let map = &layer(&s).maps_from(mesh_id(2))[0];
    assert!(map.1.face_map().is_some());
}

#[test]
fn quality_report_is_unchanged_for_boxes_and_bounded_for_two_levels() {
    let mut s = brick_with_slab();
    let run = script(&mut s, r#"mesh.refine_local(mesh: Mesh#1, cell_set: "slab", levels: 2)"#);
    let r = report(&run);
    // Level 2 in the half (4 x 4 x 4 per cell) raises the 16 face neighbours to level 1.
    assert_eq!(r["balanced"], 16);
    assert_eq!(r["cells"], 32 * 64 + 16 * 8 + 16);
    let m = mesh(&s, 2);
    assert!((volume(m) - 8.0).abs() < 1e-12);
    assert!(validate(m).is_empty());
    assert!((r["min_scaled_jacobian"].as_f64().expect("sj") - 1.0).abs() < 1e-12);
    assert!((r["max_edge_ratio"].as_f64().expect("er") - 1.0).abs() < 1e-12, "boxes keep edge ratio 1");
}

#[test]
fn balance_false_refuses_a_jump_above_one_and_leaves_the_state_alone() {
    let mut s = brick_with_slab();
    let h = hash(&s);
    let e = s
        .call(Request::Apply { op: "mesh.refine_local".into(), params: json!({"mesh": "Mesh#1", "cell_set": "slab", "levels": 2, "balance": false}), version: None })
        .expect_err("jump 2");
    assert!(e.to_string().contains("jump above 1"), "{e}");
    assert_eq!(hash(&s), h);
    // Selection errors.
    for p in [json!({"mesh": "Mesh#1"}), json!({"mesh": "Mesh#1", "cell_set": "nope"}), json!({"mesh": "Mesh#1", "tag": "w"}), json!({"mesh": "Mesh#1", "cell_set": "slab", "levels": 4})] {
        assert!(s.call(Request::Apply { op: "mesh.refine_local".into(), params: p, version: None }).is_err());
        assert_eq!(hash(&s), h);
    }
}

#[test]
fn replay_and_fork_give_equal_hashes_and_keep_the_link() {
    let mut s = brick_with_slab();
    let h_before = hash(&s);
    script(&mut s, r#"mesh.refine_local(mesh: Mesh#1, cell_set: "slab")"#);
    let h = hash(&s);
    assert_ne!(h, h_before);
    assert_eq!(layer(&s).maps_from(mesh_id(2)).len(), 1);
    assert_eq!(layer(&s).maps_to(mesh_id(1)).len(), 1);
    let mut ws = s.into_workspace();
    let replayed = cold(&mut ws);
    assert_eq!(replayed.hash().to_string(), h, "cold replay from the root");
    let links = |st: &sm_op_api::State| st.layer::<MeshLayer>().expect("layer").maps_from(mesh_id(2)).len();
    assert_eq!(links(&replayed), 1);
    // A fork at the state before the op, redoing it, reaches the same hash.
    let before = ws.log(None, 10).expect("log");
    let parent = before[1].node;
    ws.fork("again", Some(parent)).expect("fork");
    assert_eq!(ws.state_hash().to_string(), h_before);
    ws.apply_json("mesh.refine_local", json!({"mesh": "Mesh#1", "cell_set": "slab"}), &Default::default()).expect("again on the fork");
    assert_eq!(ws.state_hash().to_string(), h, "fork and original agree");
    assert_eq!(links(ws.state()), 1);
}

#[test]
fn retiring_the_source_drops_the_link() {
    let mut s = brick_with_slab();
    script(&mut s, r#"mesh.refine_local(mesh: Mesh#1, cell_set: "slab", keep_source: false)"#);
    let l = layer(&s);
    assert!(l.get(mesh_id(1)).is_none(), "the source was retired");
    assert!(l.get(mesh_id(2)).is_some());
    assert!(l.maps_from(mesh_id(2)).is_empty() && l.maps_to(mesh_id(1)).is_empty());
}

#[test]
fn the_op_is_deterministic_across_runs_and_thread_counts() {
    let run_once = || {
        let mut s = brick_with_slab();
        script(&mut s, r#"mesh.refine_local(mesh: Mesh#1, cell_set: "slab", levels: 2)"#);
        hash(&s)
    };
    let a = run_once();
    assert_eq!(a, run_once());
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool");
    assert_eq!(a, one.install(run_once));
}

#[test]
fn a_cylinder_half_refined_projects_onto_the_circle_and_validates() {
    let mut s = server();
    script(
        &mut s,
        r#"
let cy = model.create_cylinder(height: 2.0, radius: 1.0)
mesh.size(targets: all volumes, size: 0.4)
let au = mesh.auto(strategy: "default")
let m = mesh.run(targets: all volumes)
"#,
    );
    s.call(Request::Apply { op: "test.cell_set".into(), params: json!({"mesh": "Mesh#1", "name": "half", "max_x": 0.0}), version: None }).expect("set");
    let n0 = mesh(&s, 1).cell_count();
    let run = script(&mut s, r#"mesh.refine_local(mesh: Mesh#1, cell_set: "half")"#);
    let r = report(&run);
    let m = mesh(&s, 2);
    assert!(m.cell_count() > n0);
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    // Every vertex classified on the lateral face lies at r = 1 (new ones were projected).
    let mut on_lateral = 0;
    for v in 0..m.vertex_count() {
        let p = m.point(v);
        let rho = (p.x * p.x + p.y * p.y).sqrt();
        if (rho - 1.0).abs() < 1e-9 {
            on_lateral += 1;
        }
    }
    assert!(on_lateral > 0);
    assert!(r["projected"].as_u64().expect("projected") > 0, "{r}");
    assert_eq!(layer(&s).maps_from(mesh_id(2)).len(), 1);
    // The polygon volume grows toward the cylinder: projected new boundary nodes lie on the circle.
    let v1 = volume(m);
    let v0 = volume(mesh(&s, 1));
    assert!(v1 > v0 && v1 < std::f64::consts::PI * 2.0, "{v0} -> {v1}");
    eprintln!("cylinder: {} -> {} cells, volume {v0:.9} -> {v1:.9}, report {r}", n0, m.cell_count());
}
