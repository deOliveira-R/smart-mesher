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

/// A cell set named `name` of the cells of `mesh` whose corner mean lies within radius `max_r` of the z axis.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct CellSetInside {
    mesh: EntityId,
    name: String,
    max_r: f64,
}

impl TypedOp for CellSetInside {
    const NAME: &'static str = "test.cell_set_r";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let cells: Vec<CellIx> = {
            let layer = ctx.layer::<MeshLayer>()?;
            let m = layer.get(self.mesh).ok_or(OpError::UnknownEntity(self.mesh))?.mesh();
            (0..m.cell_count())
                .map(CellIx::new)
                .filter(|&c| {
                    m.cell_corners(c).is_some_and(|cs| {
                        let k = cs.len() as f64;
                        let (x, y) = cs.iter().fold((0.0, 0.0), |a, v| (a.0 + m.point(v.get()).x / k, a.1 + m.point(v.get()).y / k));
                        (x * x + y * y).sqrt() <= self.max_r
                    })
                })
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
        r.register::<CellSetBelow>()?;
        r.register::<CellSetInside>().map(|_| ())
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

const CYLINDER: &str = r#"
let cy = model.create_cylinder(height: 2.0, radius: 1.0)
mesh.size(targets: all volumes, size: 0.4)
let au = mesh.auto(strategy: "default")
let m = mesh.run(targets: all volumes)
"#;

/// Vertices classified on a model edge or on the lateral face (any model face away from the caps).
fn rim_radii(m: &dyn MeshRead) -> Vec<f64> {
    (0..m.vertex_count())
        .filter(|&v| {
            let k = m.vertex_class(sm_mesh_api::VertexIx::new(v)).entity.kind();
            let z = m.point(v).z.abs();
            k == EntityKind::TopoEdge || (k == EntityKind::TopoFace && z < 1.0 - 1e-9)
        })
        .map(|v| (m.point(v).x.powi(2) + m.point(v).y.powi(2)).sqrt())
        .collect()
}

#[test]
fn a_cylinder_half_refined_projects_onto_the_circle_with_the_analytic_volume() {
    let mut s = server();
    script(&mut s, CYLINDER);
    s.call(Request::Apply { op: "test.cell_set".into(), params: json!({"mesh": "Mesh#1", "name": "half", "max_x": 0.0}), version: None }).expect("set");
    let n0 = mesh(&s, 1).cell_count();
    let run = script(&mut s, r#"mesh.refine_local(mesh: Mesh#1, cell_set: "half")"#);
    let r = report(&run);
    let m = mesh(&s, 2);
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert!(r["projected"].as_u64().expect("projected") > 0, "{r}");
    assert_eq!(r["projection_rejected"], 0);
    let radii = rim_radii(m);
    assert!(radii.len() > rim_radii(mesh(&s, 1)).len());
    assert!(radii.iter().all(|x| (x - 1.0).abs() < 1e-12), "every rim and wall node is at r = 1");
    // The left half circle has 16 chords of angle pi/16 (8 refined to 16), the right half 8 of pi/8.
    let pi = std::f64::consts::PI;
    let exact = 2.0 * (8.0 * (pi / 16.0).sin() + 4.0 * (pi / 8.0).sin());
    assert!((volume(m) - exact).abs() < 1e-12, "{} vs inscribed polygon {exact}", volume(m));
    assert_eq!(layer(&s).maps_from(mesh_id(2)).len(), 1);
    eprintln!(
        "cylinder half: {n0} -> {} cells ({} hexes, {} polyhedra), volume {:.9} -> {:.9} (polygon {exact:.9}, 2 pi = {:.9}), min SJ {} -> {}, hex edge ratio {} -> {} (polyhedra {})",
        m.cell_count(), r["hexes"], r["polyhedra"], volume(mesh(&s, 1)), volume(m), 2.0 * pi,
        r["min_scaled_jacobian_parent"], r["min_scaled_jacobian"], r["max_edge_ratio_parent"], r["max_edge_ratio"], r["max_edge_ratio_polyhedra"]
    );
}

#[test]
fn refining_only_the_fuel_of_the_pin_cell_keeps_every_cell_positive_and_the_total_exact() {
    let mut s = server();
    script(
        &mut s,
        r#"
let br = model.create_brick(x: 1.26, y: 1.26, z: 1.0)
let fu = model.create_cylinder(height: 1.0, radius: 0.4)
let tl = model.create_cylinder(height: 1.0, radius: 0.4)
let mo = model.subtract(targets: br.body, tools: tl.body)
model.imprint(bodies: all bodies)
model.merge()
mesh.size(targets: all volumes, size: 0.3)
let au = mesh.auto(strategy: "default")
let m = mesh.run(targets: all volumes)
"#,
    );
    s.call(Request::Apply { op: "test.cell_set_r".into(), params: json!({"mesh": "Mesh#1", "name": "fuel", "max_r": 0.39}), version: None }).expect("set");
    let n0 = mesh(&s, 1).cell_count();
    let run = script(&mut s, r#"mesh.refine_local(mesh: Mesh#1, cell_set: "fuel")"#);
    let r = report(&run);
    let m = mesh(&s, 2);
    assert!(r["refined"].as_u64().expect("refined") > 0 && m.cell_count() > n0);
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert!((0..m.cell_count()).all(|c| cell_volume(m, CellIx::new(c)) > 0.0), "every cell has positive volume");
    assert!((volume(m) - 1.26 * 1.26).abs() < 1e-12, "the moderator's straight outer brick keeps the total exact: {}", volume(m));
    assert!(r["polyhedra"].as_u64().expect("polyhedra") > 0, "the moderator ring cells at the fuel interface became polyhedra");
    assert!(r["min_scaled_jacobian"].as_f64().expect("sj") > 0.5);
    eprintln!(
        "pin fuel only: {n0} -> {} cells ({} hexes, {} polyhedra, {} refined, {} balanced), volume {:.9} -> {:.9}, min SJ {} -> {}, hex edge ratio {} -> {} (polyhedra {}), projected {} rejected {}",
        m.cell_count(), r["hexes"], r["polyhedra"], r["refined"], r["balanced"], volume(mesh(&s, 1)), volume(m),
        r["min_scaled_jacobian_parent"], r["min_scaled_jacobian"], r["max_edge_ratio_parent"], r["max_edge_ratio"], r["max_edge_ratio_polyhedra"],
        r["projected"], r["projection_rejected"]
    );
}

#[test]
fn rt_check_and_locate_accept_the_refined_mesh() {
    let mut s = brick_with_slab();
    script(&mut s, r#"mesh.refine_local(mesh: Mesh#1, cell_set: "slab")"#);
    let chk = s.call(Request::Query { name: "rt.check".into(), input: json!({"mesh": "Mesh#2"}), at: None }).expect("rt.check");
    assert_eq!(chk["ok"], true, "{chk}");
    assert_eq!(chk["issues"].as_array().map(Vec::len), Some(0), "{chk}");
    let (pts, ncells): (Vec<[f64; 3]>, usize) = {
        let m = mesh(&s, 2);
        let pts = (0..m.cell_count())
            .map(|c| {
                let (_, g) = sm_mesh_api::cell_volume_centroid(m, CellIx::new(c));
                [g.x, g.y, g.z]
            })
            .collect();
        (pts, m.cell_count())
    };
    let loc = s.call(Request::Query { name: "rt.locate".into(), input: json!({"mesh": "Mesh#2", "points": pts}), at: None }).expect("rt.locate");
    let found: Vec<u64> = loc.as_array().expect("array").iter().map(|o| o["cell_index"].as_u64().expect("inside")).collect();
    assert_eq!(found, (0..ncells as u64).collect::<Vec<_>>(), "every centroid is located in its own cell");
}

#[test]
fn tag_selection_errors_are_typed_and_leave_the_state_alone() {
    let mut s = brick_with_slab();
    let h = hash(&s);
    for p in [json!({"mesh": "Mesh#1", "tag": "w", "threshold": 1.0}), json!({"mesh": "Mesh#1", "tag": "w"}), json!({"mesh": "Mesh#1", "cell_set": "slab", "tag": "w", "threshold": 1.0})] {
        assert!(s.call(Request::Apply { op: "mesh.refine_local".into(), params: p, version: None }).is_err());
        assert_eq!(hash(&s), h);
    }
}
