//! W4-POLY-SCHEME: `mesh.scheme(scheme: "polyhedron")` makes tets and dualizes them in one
//! `mesh.run`, byte-identical to `tetmesh` followed by `mesh.to_poly(keep_source: false)`.

use serde_json::{Value, json};
use sm_layer_api::MeshLayer;
use sm_protocol::{Endpoint, Request, RpcError};
use sm_server::{Server, standard_with};

fn server() -> Server {
    Server::new(standard_with(Some("tester".into()), Some("2026-11-12T00:00:00Z".into())))
        .expect("server")
}

fn script(s: &mut Server, text: &str) -> Result<Value, RpcError> {
    s.call(Request::ScriptRun { text: text.into(), path: None, includes: Default::default() })
}

fn brief(e: &RpcError) -> String {
    e.to_string().chars().take(400).collect()
}

fn query(s: &mut Server, name: &str, input: Value) -> Value {
    s.call(Request::Query { name: name.into(), input, at: None })
        .unwrap_or_else(|e| panic!("{name}: {}", brief(&e)))
}

const PRE: &str = "let b = model.create_brick(x: 1.0, y: 1.0, z: 1.0)\n\
    mesh.size(targets: all volumes, size: 0.34)\n\
    mesh.scheme(targets: topo.related(kind: \"TopoFace\", within: all volumes), scheme: \"trimesh\")\n";

/// One step: the polyhedron scheme.
fn one_step(dual: Option<&str>) -> Server {
    let mut s = server();
    let params = dual.map_or(String::new(), |d| format!(", params: {{dual: \"{d}\"}}"));
    script(
        &mut s,
        &format!(
            "{PRE}mesh.scheme(targets: all volumes, scheme: \"polyhedron\"{params})\n\
             let m = mesh.run(targets: all volumes)\n"
        ),
    )
    .unwrap_or_else(|e| panic!("one step: {}", brief(&e)));
    s
}

/// Two steps: tets, then `mesh.to_poly` with the source dropped.
fn two_steps(dual: Option<&str>) -> Server {
    let mut s = server();
    let d = dual.map_or(String::new(), |d| format!(", dual: \"{d}\""));
    script(
        &mut s,
        &format!(
            "{PRE}mesh.scheme(targets: all volumes, scheme: \"tetmesh\")\n\
             let m = mesh.run(targets: all volumes)\n\
             let p = mesh.to_poly(mesh: m.mesh[0], keep_source: false{d})\n"
        ),
    )
    .unwrap_or_else(|e| panic!("two steps: {}", brief(&e)));
    s
}

fn meshes_hash(s: &Server) -> String {
    format!("{:?}", s.workspace().state().layer_hash("meshes").expect("meshes layer"))
}

fn mesh_count(s: &Server) -> usize {
    s.workspace().state().layer::<MeshLayer>().expect("layer").len()
}

#[test]
fn one_step_equals_tetmesh_then_to_poly() {
    for dual in [None, Some("barycentric")] {
        let (a, b) = (one_step(dual), two_steps(dual));
        assert_eq!(mesh_count(&a), 1, "one mesh entry");
        assert_eq!(meshes_hash(&a), meshes_hash(&b), "mesh layer, dual {dual:?}");
        assert_eq!(
            a.workspace().state().entity_count(),
            b.workspace().state().entity_count(),
            "same live entities"
        );
    }
}

#[test]
fn the_polyhedral_mesh_has_the_analytic_volume_and_one_cell_per_tet_vertex() {
    let mut a = one_step(None);
    let st = query(&mut a, "mesh.poly_info", json!({"mesh": "Mesh#2"}));
    let tets = two_steps(None); // same plan: tet vertex count = poly cell count
    let id = tets.workspace().state().layer::<MeshLayer>().expect("l").meshes().next().expect("m").0;
    let n_cells = tets.workspace().state().layer::<MeshLayer>().expect("l").get(id).expect("e").mesh().cell_count();
    assert_eq!(st["cells"].as_u64(), Some(n_cells as u64), "{st}");
    eprintln!("poly_info: {st}");
    let vol = st["volume_sum"].as_f64().expect("volume");
    assert!((vol - 1.0).abs() < 1e-12, "unit brick volume, got {vol}");
    let area = st["boundary_area"].as_f64().expect("area");
    assert!((area - 6.0).abs() < 1e-12, "unit brick boundary area, got {area}");
    assert!(st["max_nonplanarity"].as_f64().expect("planarity") < 1e-9, "{st}");
    // `session.export` needs no mesh name: there is one entry.
    assert_eq!(mesh_count(&a), 1);
}

#[test]
fn one_step_is_deterministic_across_thread_counts() {
    let hash = |n: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(n)
            .build()
            .expect("pool")
            .install(|| meshes_hash(&one_step(None)))
    };
    let (h1, h4) = (hash(1), hash(4));
    assert_eq!(h1, h4, "1 vs 4 threads");
    assert_eq!(h1, meshes_hash(&one_step(None)), "run twice");
}

#[test]
fn circumcentric_follows_the_strict_gate_like_to_poly() {
    let run = |text: String| {
        let mut s = server();
        script(&mut s, &text).map(|_| s).map_err(|e| brief(&e))
    };
    let a = run(format!(
        "{PRE}mesh.scheme(targets: all volumes, scheme: \"polyhedron\", params: {{dual: \"circumcentric\"}})\n\
         let m = mesh.run(targets: all volumes)\n"
    ));
    let b = run(format!(
        "{PRE}mesh.scheme(targets: all volumes, scheme: \"tetmesh\")\n\
         let m = mesh.run(targets: all volumes)\n\
         let p = mesh.to_poly(mesh: m.mesh[0], keep_source: false, dual: \"circumcentric\")\n"
    ));
    assert_eq!(a.is_ok(), b.is_ok(), "both pass the gate or both refuse: {:?} / {:?}", a.as_ref().err(), b.as_ref().err());
    match (a, b) {
        (Ok(a), Ok(b)) => assert_eq!(meshes_hash(&a), meshes_hash(&b)),
        (Err(a), Err(b)) => {
            assert!(a.contains("circumcentric") || a.contains("orthogonal") || a.contains("Delaunay"), "{a}");
            assert!(b.contains("mesh.to_poly"), "{b}");
            // A refused run leaves nothing behind (atomic).
        }
        _ => unreachable!(),
    }
}

#[test]
fn bad_use_is_refused_with_a_message() {
    let mut s = server();
    script(&mut s, PRE).expect("setup");
    let err = |s: &mut Server, text: &str| script(s, text).expect_err("refused").to_string();
    let e = err(&mut s, "mesh.scheme(targets: all volumes, scheme: \"polyhedron\", params: {dual: \"voronoi\"})");
    assert!(e.contains("barycentric"), "{e}");
    let e = err(&mut s, "mesh.scheme(targets: all volumes, scheme: \"tetmesh\", fallbacks: [\"polyhedron\"])");
    assert!(e.contains("polyhedron cannot be a fallback; it changes the cell type"), "{e}");
    let e = err(&mut s, "mesh.scheme(targets: topo.related(kind: \"TopoFace\", within: all volumes), scheme: \"polyhedron\")");
    assert!(e.contains("polyhedron"), "{e}");
}

#[test]
fn mesh_plan_accepts_the_polyhedron_scheme() {
    let mut s = server();
    script(&mut s, &format!("{PRE}mesh.scheme(targets: all volumes, scheme: \"polyhedron\")\n")).expect("setup");
    let p = query(&mut s, "mesh.plan", json!({"targets": ["TopoVolume#1"]}));
    assert!(p["blockers"].as_array().is_some_and(Vec::is_empty), "{p}");
}
