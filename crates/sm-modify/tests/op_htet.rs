//! MOD-HTET and MOD-CONV2 at op level through `sm_server::standard`: the converted meshes validate,
//! pass through `mesh.to_poly`, link to their source (replay and fork give equal hashes, retirement
//! drops the link), the deck pair of a model survives, and `refine.set_ambiguous` arrives at info.

use std::collections::BTreeSet;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sm_core::{EntityId, EntityKind, Vector3};
use sm_engine::{DiagnosticPolicy, Verbosity, Workspace};
use sm_layer_api::{LayerResultExt, MeshEntry, MeshHost, MeshLayer, add_mesh_set, insert_mesh, model_txn};
use sm_mesh::MeshBuilder;
use sm_mesh_api::{
    CellIx, CellShape, Classification, FaceIx, FaceKind, MeshBuild, MeshCatalog, MeshEdit, MeshEntityKind, MeshEntityRef, MeshRead, SerialAlloc,
    SetMembers, VertexIx, cell_volume, validate,
};
use sm_model_api::{BoundaryKind, Deck};
use sm_op_api::{ApplyCtx, Control, OpError, Plugin, QuietControl, Registry, RegistryError, TypedOp};
use sm_protocol::{Endpoint, Request};
use sm_server::{Server, standard_with};

const TIME: &str = "2026-10-24T00:00:00Z";

// ---- test-only ops --------------------------------------------------------------------------------

/// Insert a fixture mesh (`blocks_2x1x1`, `ends_block`, `tet_block`, `tri_patch`, `quad_patch`) matching no model entity.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct PutMesh {
    name: String,
}

fn on(k: EntityKind, n: u64) -> Classification {
    Classification::on(EntityId::new(k, n).expect("id"))
}

/// A 2D patch of `quads` unit quads (or two triangles per unit square), classified.
fn patch(quads: usize, tris: bool, alloc: &mut SerialAlloc) -> sm_mesh::Mesh {
    let mut b = MeshBuilder::surface();
    let vs: Vec<VertexIx> = (0..=quads)
        .flat_map(|i| [(i as f64, 0.0), (i as f64, 1.0)])
        .map(|(x, y)| b.add_vertex(sm_core::Point3::new(x, y, 0.0)).expect("v"))
        .collect();
    for (i, v) in vs.iter().enumerate() {
        b.set_class(MeshEntityRef::Vertex(*v), on(EntityKind::TopoVertex, i as u64 + 1)).expect("c");
    }
    let mut en = 0;
    let mut cls = |b: &mut MeshBuilder, a: sm_mesh_api::CellAdded| {
        b.set_class(MeshEntityRef::Cell(a.cell), on(EntityKind::TopoFace, 1)).expect("c");
        for (f, r) in a.faces.iter().zip(&a.reused) {
            if !r {
                en += 1;
                b.set_class(MeshEntityRef::Face(f.face()), on(EntityKind::TopoEdge, en)).expect("c");
            }
        }
    };
    for i in 0..quads {
        let (a, bb, c, d) = (vs[2 * i], vs[2 * i + 2], vs[2 * i + 3], vs[2 * i + 1]);
        if tris {
            for t in [[a, bb, c], [a, c, d]] {
                let added = b.add_cell_from_vertices(CellShape::Tri, &t).expect("tri");
                cls(&mut b, added);
            }
        } else {
            let added = b.add_cell_from_vertices(CellShape::Quad, &[a, bb, c, d]).expect("quad");
            cls(&mut b, added);
        }
    }
    b.finish(alloc).expect("finish").mesh
}

impl TypedOp for PutMesh {
    const NAME: &'static str = "test.put_mesh";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        use sm_mesh_api::reference::hex_block_with_edges;
        let mut alloc = ctx.layer::<MeshLayer>()?.alloc().clone();
        let boxed: Box<dyn MeshHost> = match self.name.as_str() {
            "blocks_2x1x1" => Box::new(hex_block_with_edges([2, 1, 1], [1.0; 3], &mut alloc).map_err(|e| OpError::Failed(e.to_string()))?),
            "ends_block" => {
                let mut m = hex_block_with_edges([1, 2, 2], [1.0; 3], &mut alloc).map_err(|e| OpError::Failed(e.to_string()))?;
                let ends: Vec<VertexIx> =
                    (0..m.vertex_count()).map(VertexIx::new).filter(|v| m.point(v.get()).y == 0.0 && m.point(v.get()).z == 0.0).collect();
                m.add_set("ends", SetMembers::Vertices(ends), &mut alloc).map_err(|e| OpError::Failed(e.to_string()))?;
                Box::new(m)
            }
            "tet_block" => Box::new(sm_testkit::meshes::tet_block([2, 2, 2], [1.0; 3]).map_err(|e| OpError::Failed(e.to_string()))?.mesh),
            "tri_patch" => Box::new(patch(2, true, &mut alloc)),
            "quad_patch" => Box::new(patch(2, false, &mut alloc)),
            n => return Err(OpError::Invalid(format!("no fixture {n}"))),
        };
        let mesh = boxed;
        let alloc0 = ctx.layer::<MeshLayer>()?.alloc().clone();
        let mut next: std::collections::BTreeMap<EntityKind, u64> = alloc0.state().collect();
        let mut raise = |id: EntityId| {
            let e = next.entry(id.kind()).or_insert(1);
            *e = (*e).max(id.serial() + 1);
        };
        raise(mesh.mesh_id());
        for kind in [MeshEntityKind::Vertex, MeshEntityKind::Edge, MeshEntityKind::Face, MeshEntityKind::Cell] {
            for i in 0..mesh.count(kind) {
                raise(mesh.id(kind, i as u32));
            }
        }
        for s in mesh.sets() {
            raise(s.id);
        }
        let after = SerialAlloc::from_state(next).map_err(|e| OpError::Failed(e.to_string()))?;
        let quanta = ctx.layer::<sm_layer_api::ModelLayer>()?.hash_quanta();
        let entry = MeshEntry::new(mesh, vec![], None, quanta);
        insert_mesh(ctx, entry, &alloc0, after, "mesh")
    }
}

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

/// A translation deck pairing model faces `a` and `b` by `vector`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct PairDeck {
    a: EntityId,
    b: EntityId,
    vector: [f64; 3],
}

impl TypedOp for PairDeck {
    const NAME: &'static str = "test.pair_deck";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let (a, b, v) = (self.a, self.b, self.vector);
        model_txn(
            ctx,
            |model, _| {
                let id = model.add_boundary("pair", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(v[0], v[1], v[2]) })).or_op()?;
                model.assign_pairs(id, &[(a, b)]).or_op()?;
                Ok(())
            },
            |_, _| vec![],
        )
    }
}

struct TestKit;

impl Plugin for TestKit {
    fn id(&self) -> &str {
        "htet-test-kit"
    }
    fn register(&self, r: &mut Registry) -> Result<(), RegistryError> {
        r.register::<PutMesh>()?;
        r.register::<CellSetBelow>()?;
        r.register::<PairDeck>()?;
        Ok(())
    }
}

// ---- helpers ------------------------------------------------------------------------------------------

fn server() -> Server {
    let policy = DiagnosticPolicy { verbosity: Verbosity::Info, ..DiagnosticPolicy::default() };
    Server::new(standard_with(Some("tester".into()), Some(TIME.into())).policy(policy).plugin(TestKit)).expect("server")
}

fn script(s: &mut Server, text: &str) -> Value {
    s.call(Request::ScriptRun { text: text.into(), path: None, includes: Default::default() })
        .unwrap_or_else(|e| panic!("script failed: {e}\n{}", e.data["rendered"].as_str().unwrap_or("")))
}

/// A script literal of a JSON value (entity ids such as `Mesh#1` are written bare).
fn lit(v: &Value) -> String {
    match v {
        Value::String(t) if t.split_once('#').is_some_and(|(k, n)| k.chars().all(|c| c.is_ascii_alphabetic()) && n.chars().all(|c| c.is_ascii_digit())) => t.clone(),
        Value::Array(a) => format!("[{}]", a.iter().map(lit).collect::<Vec<_>>().join(", ")),
        v => v.to_string(),
    }
}

/// Run one op as a one-line script; the error is the rendered failure text.
fn apply(s: &mut Server, op: &str, params: Value) -> Result<Value, String> {
    let args: Vec<String> = params.as_object().expect("object").iter().map(|(k, v)| format!("{k}: {}", lit(v))).collect();
    let text = format!("{op}({})", args.join(", "));
    s.call(Request::ScriptRun { text, path: None, includes: Default::default() })
        .map_err(|e| format!("{e} {}", e.data["rendered"].as_str().unwrap_or("")))
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

/// Diagnostics `(code, severity, message)` of the events of op `op` in a script or apply result.
fn diags(run: &Value, op: &str) -> Vec<(String, String, String)> {
    let events = run["events"].as_array().cloned().unwrap_or_else(|| vec![run.clone()]);
    events
        .iter()
        .filter(|e| e["op"] == op)
        .flat_map(|e| e["delta"]["diagnostics"].as_array().cloned().unwrap_or_default())
        .map(|d| (d["code"].as_str().unwrap_or("").to_string(), d["severity"].as_str().unwrap_or("?").to_string(), d["message"].as_str().unwrap_or("").to_string()))
        .collect()
}

fn report(run: &Value, op: &str, code: &str) -> Value {
    let d = diags(run, op).into_iter().find(|d| d.0 == code).unwrap_or_else(|| panic!("no {code} diagnostic in {:?}", diags(run, op)));
    serde_json::from_str(&d.2).expect("report json")
}

const BRICK: &str = r#"
let b = model.create_brick(x: 2.0, y: 2.0, z: 2.0)
mesh.size(targets: all volumes, size: 1.0)
let au = mesh.auto(strategy: "default")
let m = mesh.run(targets: all volumes)
"#;

fn cold(ws: &mut Workspace) -> sm_op_api::State {
    let q = QuietControl::new();
    let ctl: Control<'_> = q.control();
    let head = ws.session().position().expect("head");
    ws.session().state_at_from_root(head, &ctl).expect("cold replay")
}

// ---- tests ----------------------------------------------------------------------------------------------

#[test]
fn htet_of_a_meshed_brick_is_all_tets_conformal_linked_and_dualizes() {
    let mut s = server();
    script(&mut s, BRICK);
    let src = mesh(&s, 1);
    assert_eq!(src.cell_count(), 8);
    let run = script(&mut s, "mesh.htet(mesh: Mesh#1)");
    let m = mesh(&s, 2);
    assert!((0..m.cell_count()).all(|c| m.cell_shape(CellIx::new(c)) == CellShape::Tet));
    assert!((40..=48).contains(&m.cell_count()), "8 hexes give 5 or 6 tets each: {}", m.cell_count());
    assert!(validate(m).is_empty());
    assert!((volume(m) - 8.0).abs() < 1e-12);
    let rep = report(&run, "mesh.htet", "htet.report");
    eprintln!(
        "htet brick: {} hexes -> {} tets ({} cut in 5, {} in 6); volume {} -> {}; min scaled Jacobian {:.4} -> {:.4}",
        rep["parent_cells"], rep["cells"], rep["hex_5_tets"], rep["hex_6_tets"], rep["volume_parent"], rep["volume"],
        rep["min_scaled_jacobian_parent"].as_f64().unwrap(), rep["min_scaled_jacobian"].as_f64().unwrap()
    );
    assert_eq!(rep["cut_quads"], 36, "all 36 quads of a 2x2x2 block are cut: 24 on the boundary, 12 inside");
    assert_eq!(layer(&s).maps_from(mesh_id(2)).len(), 1, "the tet mesh is linked to its source");
    // The all-tet result goes through mesh.to_poly and the dual cells fill the volume.
    script(&mut s, "mesh.to_poly(mesh: Mesh#2)");
    let p = mesh(&s, 3);
    assert!(validate(p).is_empty());
    assert!((volume(p) - 8.0).abs() < 1e-9, "dual volume {}", volume(p));
}

#[test]
fn closure_gives_a_polyhedron_and_whole_refuses_with_the_cell() {
    let mut s = server();
    apply(&mut s, "test.put_mesh", json!({"name": "blocks_2x1x1"})).expect("put");
    apply(&mut s, "test.cell_set", json!({"mesh": "Mesh#1", "name": "left", "max_x": 1.0})).expect("set");
    let h = hash(&s);
    let e = apply(&mut s, "mesh.htet", json!({"mesh": "Mesh#1", "cell_set": "left", "closure": "whole"})).expect_err("whole refuses");
    assert!(e.contains("closure `whole`") && e.contains("cell"), "{e}");
    assert_eq!(hash(&s), h, "atomic");
    let e = apply(&mut s, "mesh.htet", json!({"mesh": "Mesh#1", "cell_set": "nope"})).expect_err("unknown set");
    assert!(e.contains("no set `nope`"), "{e}");
    let run = apply(&mut s, "mesh.htet", json!({"mesh": "Mesh#1", "cell_set": "left"})).expect("closure");
    let m = mesh(&s, 2);
    let shapes: BTreeSet<String> = (0..m.cell_count()).map(|c| format!("{:?}", m.cell_shape(CellIx::new(c)))).collect();
    assert_eq!(shapes, ["Polyhedron", "Tet"].map(String::from).into_iter().collect());
    assert!(validate(m).is_empty());
    assert!((volume(m) - 2.0).abs() < 1e-14);
    let rep = report(&run, "mesh.htet", "htet.report");
    assert_eq!(rep["polyhedra"], 1);
    // `mesh.to_poly` takes mixed meshes since W4-POLY-MIXED (the NotTet refusal of the draft is gone). Its
    // stitch of this closure result currently fails validation of its own output (reported to the
    // orchestrator, see the task notes), so only the all-tet route is pinned (the brick test above).
}

#[test]
fn replay_and_fork_give_equal_hashes_and_keep_the_link() {
    let mut s = server();
    script(&mut s, BRICK);
    let h_before = hash(&s);
    script(&mut s, "mesh.htet(mesh: Mesh#1)");
    let h = hash(&s);
    assert_ne!(h, h_before);
    assert_eq!(layer(&s).maps_from(mesh_id(2)).len(), 1);
    let mut ws = s.into_workspace();
    let replayed = cold(&mut ws);
    assert_eq!(replayed.hash().to_string(), h, "cold replay from the root");
    let links = |st: &sm_op_api::State| st.layer::<MeshLayer>().expect("layer").maps_from(mesh_id(2)).len();
    assert_eq!(links(&replayed), 1);
    let before = ws.log(None, 10).expect("log");
    ws.fork("again", Some(before[1].node)).expect("fork");
    assert_eq!(ws.state_hash().to_string(), h_before);
    ws.apply_json("mesh.htet", json!({"mesh": "Mesh#1"}), &Default::default()).expect("htet on the fork");
    assert_eq!(ws.state_hash().to_string(), h, "fork and original agree");
    assert_eq!(links(ws.state()), 1);
}

#[test]
fn keep_source_false_retires_the_source_and_drops_the_link() {
    let mut s = server();
    script(&mut s, BRICK);
    script(&mut s, "mesh.htet(mesh: Mesh#1, keep_source: false)");
    let l = layer(&s);
    assert!(l.get(mesh_id(1)).is_none(), "the source was retired");
    assert!(l.get(mesh_id(2)).is_some());
    assert!(l.maps_from(mesh_id(2)).is_empty() && l.maps_to(mesh_id(1)).is_empty());
}

#[test]
fn the_ops_are_deterministic_across_runs_and_thread_counts() {
    let run_once = || {
        let mut s = server();
        script(&mut s, BRICK);
        script(&mut s, "mesh.htet(mesh: Mesh#1)");
        apply(&mut s, "test.put_mesh", json!({"name": "quad_patch"})).expect("put");
        apply(&mut s, "mesh.qtri", json!({"mesh": "Mesh#3", "mode": "four"})).expect("qtri");
        hash(&s)
    };
    let a = run_once();
    assert_eq!(a, run_once());
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool");
    assert_eq!(a, one.install(run_once));
}

/// A translation deck pairs the x = 0 and x = 2 faces of the meshed brick. After `mesh.htet` of the
/// whole mesh the pair is intact (no warning); a selection that cuts only the A side leaves the pair
/// mismatched and warns (`htet.deck_mismatch`).
#[test]
fn deck_pair_survives_a_full_htet_and_a_partial_one_warns() {
    let mut s = server();
    script(&mut s, BRICK);
    let (a, b, x0, x1) = {
        let m = mesh(&s, 1);
        let xs: Vec<f64> = (0..m.vertex_count()).map(|v| m.point(v).x).collect();
        let (x0, x1) = (xs.iter().copied().fold(f64::INFINITY, f64::min), xs.iter().copied().fold(f64::NEG_INFINITY, f64::max));
        let at = |x: f64| {
            (0..m.face_count())
                .map(FaceIx::new)
                .find(|&f| m.face_kind(f) == FaceKind::Boundary && m.face_vertices(f).iter().all(|v| m.point(v.get()).x == x))
                .map(|f| m.face_class(f).entity)
                .expect("wall")
        };
        (at(x0), at(x1), x0, x1)
    };
    apply(&mut s, "test.pair_deck", json!({"a": a.to_string(), "b": b.to_string(), "vector": [x1 - x0, 0.0, 0.0]})).expect("deck");
    let run = apply(&mut s, "mesh.htet", json!({"mesh": "Mesh#1"})).expect("htet");
    let rep = report(&run, "mesh.htet", "htet.report");
    assert_eq!(rep["deck_pairs"], 1);
    assert_eq!(rep["deck_mismatch"], 0, "{rep}");
    assert!(!diags(&run, "mesh.htet").iter().any(|d| d.0 == "htet.deck_mismatch"));
    apply(&mut s, "test.cell_set", json!({"mesh": "Mesh#1", "name": "a_side", "max_x": x0 + 1.0})).expect("set");
    let run = apply(&mut s, "mesh.htet", json!({"mesh": "Mesh#1", "cell_set": "a_side"})).expect("partial htet");
    let rep = report(&run, "mesh.htet", "htet.report");
    assert!(rep["deck_mismatch"].as_u64().unwrap() > 0, "{rep}");
    assert!(diags(&run, "mesh.htet").iter().any(|d| d.0 == "htet.deck_mismatch" && d.1 == "warning"), "{:?}", diags(&run, "mesh.htet"));
}

#[test]
fn qtri_splits_a_quad_patch_in_two_and_in_four() {
    let mut s = server();
    apply(&mut s, "test.put_mesh", json!({"name": "quad_patch"})).expect("put");
    let run = apply(&mut s, "mesh.qtri", json!({"mesh": "Mesh#1"})).expect("two");
    assert_eq!(mesh(&s, 2).cell_count(), 4);
    assert!((volume(mesh(&s, 2)) - 2.0).abs() < 1e-15);
    assert!(validate(mesh(&s, 2)).is_empty());
    assert_eq!(report(&run, "mesh.qtri", "qtri.report")["cells"], 4);
    apply(&mut s, "mesh.qtri", json!({"mesh": "Mesh#1", "mode": "four"})).expect("four");
    let m = mesh(&s, 3);
    assert_eq!(m.cell_count(), 8);
    assert!((0..8).all(|c| (cell_volume(m, CellIx::new(c)) - 0.25).abs() < 1e-15), "each triangle is A/4 of a unit square");
    assert!(validate(m).is_empty());
    assert_eq!(layer(&s).maps_from(mesh_id(3)).len(), 1);
    // A 3D mesh is refused with a pointer to the other op.
    let mut s = server();
    script(&mut s, BRICK);
    let e = apply(&mut s, "mesh.qtri", json!({"mesh": "Mesh#1"})).expect_err("3D");
    assert!(e.contains("surface mesh"), "{e}");
    let e = apply(&mut s, "mesh.htet", json!({"mesh": "Mesh#1", "cell_set": "x"})).expect_err("no set");
    assert!(e.contains("no set"), "{e}");
}

#[test]
fn thex_and_tquad_ops_convert_and_link() {
    let mut s = server();
    apply(&mut s, "test.put_mesh", json!({"name": "tet_block"})).expect("put");
    let run = apply(&mut s, "mesh.thex", json!({"mesh": "Mesh#1"})).expect("thex");
    let m = mesh(&s, 2);
    assert_eq!(m.cell_count(), 4 * 48);
    assert!((0..m.cell_count()).all(|c| m.cell_shape(CellIx::new(c)) == CellShape::Hex));
    assert!(validate(m).is_empty());
    assert!((volume(m) - 8.0).abs() < 1e-12);
    let rep = report(&run, "mesh.thex", "thex.report");
    eprintln!("thex 2x2x2 tet block: {} tets -> {} hexes, min scaled Jacobian {:.4}", rep["parent_cells"], rep["cells"], rep["min_scaled_jacobian"].as_f64().unwrap());
    assert_eq!(layer(&s).maps_from(mesh_id(2)).len(), 1);
    apply(&mut s, "test.put_mesh", json!({"name": "tri_patch"})).expect("put");
    let run = apply(&mut s, "mesh.tquad", json!({"mesh": "Mesh#3"})).expect("tquad");
    let q = mesh(&s, 4);
    assert_eq!(q.cell_count(), 12, "4 triangles give 12 quads");
    assert!((volume(q) - 2.0).abs() < 1e-14);
    assert!(validate(q).is_empty());
    assert_eq!(report(&run, "mesh.tquad", "tquad.report")["cells"], 12);
    let e = apply(&mut s, "mesh.thex", json!({"mesh": "Mesh#2"})).expect_err("hexes");
    assert!(e.contains("tet"), "{e}");
}

/// `refine.set_ambiguous` is informational: it arrives at info severity (never a warning), naming the
/// node set and the one-interval model edge whose midpoint joined.
#[test]
fn refine_set_ambiguous_arrives_at_info_severity() {
    let mut s = server();
    apply(&mut s, "test.put_mesh", json!({"name": "ends_block"})).expect("put");
    let run = apply(&mut s, "mesh.refine", json!({"mesh": "Mesh#1"})).expect("refine");
    let amb: Vec<_> = diags(&run, "mesh.refine").into_iter().filter(|d| d.0 == "refine.set_ambiguous").collect();
    assert_eq!(amb.len(), 1, "{:?}", diags(&run, "mesh.refine"));
    assert_eq!(amb[0].1, "info", "{:?}", amb[0]);
    assert!(amb[0].2.contains("`ends`") && amb[0].2.contains("one interval"), "{}", amb[0].2);
    assert!(!diags(&run, "mesh.refine").iter().any(|d| d.1 == "warning" && d.0 == "refine.set_ambiguous"));
}
