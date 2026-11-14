//! MOD-REFINE-U at op level through `sm_server::standard`: the axial cylinder and the W3c pin cell
//! (analytic volumes and radii with projection), the level-map link (replay, fork, retirement,
//! smoothing), refusals and determinism.

use std::collections::BTreeMap;

use serde_json::{Value, json};
use sm_core::{EntityId, EntityKind};
use sm_engine::{DiagnosticPolicy, Verbosity, Workspace};
use sm_layer_api::MeshLayer;
use sm_mesh_api::{CellIx, FaceIx, FaceKind, MeshCatalog, MeshRead, cell_volume, validate};
use sm_op_api::{Control, QuietControl};
use sm_protocol::{Endpoint, Request};
use sm_server::{Server, standard_with};

const TIME: &str = "2026-10-24T00:00:00Z";

fn server() -> Server {
    let policy = DiagnosticPolicy { verbosity: Verbosity::Info, ..DiagnosticPolicy::default() };
    Server::new(standard_with(Some("tester".into()), Some(TIME.into())).policy(policy)).expect("server")
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

/// The `refine.report` diagnostics of all events of a script run, in order.
fn reports(run: &Value) -> Vec<Value> {
    run["events"]
        .as_array()
        .expect("events")
        .iter()
        .filter(|e| e["op"] == "mesh.refine")
        .flat_map(|e| e["delta"]["diagnostics"].as_array().cloned().unwrap_or_default())
        .filter(|d| d["code"] == "refine.report")
        .map(|d| serde_json::from_str(d["message"].as_str().expect("message")).expect("report json"))
        .collect()
}

const CYLINDER: &str = r#"
let cy = model.create_cylinder(height: 2.0, radius: 1.0)
mesh.size(targets: all volumes, size: 0.4)
let au = mesh.auto(strategy: "default")
let m = mesh.run(targets: all volumes)
"#;

fn volume(m: &dyn MeshRead) -> f64 {
    (0..m.cell_count()).map(|c| cell_volume(m, CellIx::new(c))).sum()
}

/// Inscribed-prism volume of a unit-radius, height-2 cylinder with `n` circle chords.
fn prism(n: f64) -> f64 {
    n / 2.0 * (std::f64::consts::TAU / n).sin() * 2.0
}

#[test]
fn a5_cylinder_projection_is_analytic_for_two_levels() {
    let mut s = server();
    script(&mut s, CYLINDER);
    let src = layer(&s).get(mesh_id(1)).expect("mesh").mesh();
    assert_eq!(src.cell_count(), 240);
    assert!((volume(src) - prism(16.0)).abs() < 1e-12, "source is the 16-gon prism");
    let run = script(&mut s, "mesh.refine(mesh: Mesh#1, levels: 2)");
    let l = layer(&s);
    assert_eq!(l.mesh_ids().len(), 3, "source plus two levels");
    let rep = reports(&run);
    assert_eq!(rep.len(), 2);
    for (k, rep) in rep.iter().enumerate() {
        let level = k + 1;
        let m = l.get(mesh_id(1 + level as u64)).expect("level").mesh();
        assert_eq!(m.cell_count(), 240 * 8usize.pow(level as u32), "cells at level {level}");
        // Lateral nodes (not on the cap planes z = 0, 2, or on the axis cap-face) lie on r = 1.
        let mut lateral = 0;
        for v in 0..m.vertex_count() {
            let p = m.point(v);
            let on_boundary = matches!(
                m.classification(sm_mesh_api::MeshEntityKind::Vertex, v as u32).entity.kind(),
                EntityKind::TopoFace | EntityKind::TopoEdge | EntityKind::TopoVertex
            );
            if on_boundary && p.z.abs() < 1.0 - 1e-9 {
                lateral += 1;
                assert!((p.x.hypot(p.y) - 1.0).abs() < 1e-12, "level {level}: node {v} at r = {} z = {} class {:?}", p.x.hypot(p.y), p.z, m.classification(sm_mesh_api::MeshEntityKind::Vertex, v as u32));
            }
        }
        assert!(lateral > 0);
        // Circle nodes on the caps: those classified on a model edge are on r = 1 too.
        for v in 0..m.vertex_count() {
            let c = m.classification(sm_mesh_api::MeshEntityKind::Vertex, v as u32);
            if c.entity.kind() == EntityKind::TopoEdge {
                let p = m.point(v);
                assert!((p.x.hypot(p.y) - 1.0).abs() < 1e-12 || p.x.hypot(p.y) < 1e-12, "circle node {v}");
            }
        }
        let n = 16.0 * 2f64.powi(level as i32);
        // The lateral boundary (faces whose nodes all lie on r = 1) is the inscribed n-gon strip:
        // n chords of length 2 sin(pi/n) over the height 2.
        let lateral_area: f64 = (0..m.face_count())
            .map(FaceIx::new)
            .filter(|&f| m.face_kind(f) == FaceKind::Boundary && m.face_vertices(f).iter().all(|v| (m.point(v.get()).x.hypot(m.point(v.get()).y) - 1.0).abs() < 1e-12))
            .map(|f| sm_mesh_api::face_area_vector(m, f).norm())
            .sum();
        let want_area = n * 2.0 * (std::f64::consts::PI / n).sin() * 2.0;
        assert!((lateral_area - want_area).abs() <= 1e-12 * want_area, "level {level}: lateral area {lateral_area} vs {want_area}");
        let want = prism(n);
        assert!((volume(m) - want).abs() <= 1e-12 * want, "level {level}: volume {} vs inscribed prism {want}", volume(m));
        assert!(validate(m).is_empty());
        eprintln!(
            "a5 cylinder level {level}: cells {} volume {:.12} (analytic cylinder {:.12}, prism {want:.12}) min SJ parent {:.6} child {:.6} projected {} rejected {}",
            m.cell_count(),
            volume(m),
            2.0 * std::f64::consts::PI,
            rep["min_scaled_jacobian_parent"].as_f64().unwrap(),
            rep["min_scaled_jacobian"].as_f64().unwrap(),
            rep["projected"],
            rep["projection_rejected"]
        );
        assert_eq!(rep["projection_rejected"], 0);
    }
    // Straight midpoints (project: false) lose volume: the polygon stays the 16-gon.
    let mut s2 = server();
    script(&mut s2, CYLINDER);
    script(&mut s2, "mesh.refine(mesh: Mesh#1, project: false)");
    let m = layer(&s2).get(mesh_id(2)).expect("level").mesh();
    assert!((volume(m) - prism(16.0)).abs() < 1e-12, "without projection the 16-gon prism is kept");
}

/// A cylinder meshed with a handful of intervals: the guard decides what happens to the projection
/// of its coarse circle; whatever it decides is reported (`refine.projection` warning when a node
/// kept its straight position), and the result validates with the exact inscribed volume of the
/// polygon actually kept.
#[test]
fn coarse_cylinder_reports_what_the_projection_left_alone() {
    let mut s = server();
    script(
        &mut s,
        r#"
let cy = model.create_cylinder(height: 2.0, radius: 1.0)
mesh.size(targets: all volumes, size: 1.5)
let au = mesh.auto(strategy: "default")
let m = mesh.run(targets: all volumes)
"#,
    );
    let run = script(&mut s, "mesh.refine(mesh: Mesh#1)");
    let rep = &reports(&run)[0];
    let m = layer(&s).get(mesh_id(2)).expect("level").mesh();
    assert!(validate(m).is_empty());
    eprintln!(
        "coarse cylinder: cells {} -> {}, projected {}, failed {}, rejected {}, min SJ {} -> {}",
        rep["parent_cells"], rep["cells"], rep["projected"], rep["projection_failed"], rep["projection_rejected"],
        rep["min_scaled_jacobian_parent"], rep["min_scaled_jacobian"]
    );
    let warned = run["events"]
        .as_array()
        .expect("events")
        .iter()
        .filter(|e| e["op"] == "mesh.refine")
        .flat_map(|e| e["delta"]["diagnostics"].as_array().cloned().unwrap_or_default())
        .any(|d| d["code"] == "refine.projection");
    let kept = rep["projection_failed"].as_u64().unwrap() + rep["projection_rejected"].as_u64().unwrap();
    assert_eq!(warned, kept > 0, "a node left at its straight position is reported as a warning");
    // No refined cell is inverted whatever the guard did.
    let sj = sm_quality::compute(m, sm_quality::Metric::ScaledJacobian, &sm_quality::Options::default()).expect("sj");
    assert!(sj.iter().all(|x| *x > 0.0), "no inverted or collapsed cell");
}

/// One through pin cell (the `tools/w3c_pin.smesh` model for a single cell).
const PIN: &str = r#"
let br = model.create_brick(x: 1.26, y: 1.26, z: 1.0)
let fu = model.create_cylinder(height: 1.0, radius: 0.4)
let tl = model.create_cylinder(height: 1.0, radius: 0.4)
let mo = model.subtract(targets: br.body, tools: tl.body)
model.imprint(bodies: all bodies)
model.merge()
assert count(all volumes) == 2, "pin cell: fuel and moderator"
mesh.size(targets: all volumes, size: 0.3)
let au = mesh.auto(strategy: "default")
let m = mesh.run(targets: all volumes)
"#;

fn volumes_by_class(m: &dyn MeshRead) -> BTreeMap<String, f64> {
    let mut out: BTreeMap<String, f64> = BTreeMap::new();
    for c in 0..m.cell_count() {
        let ci = CellIx::new(c);
        *out.entry(m.cell_class(ci).entity.to_string()).or_default() += cell_volume(m, ci);
    }
    out
}

#[test]
fn w3c_pin_volume_error_before_and_after() {
    let mut s = server();
    script(&mut s, PIN);
    let src = layer(&s).get(mesh_id(1)).expect("mesh").mesh();
    let fuel_exact = std::f64::consts::PI * 0.4 * 0.4 * 1.0;
    let total_exact = 1.26 * 1.26;
    let before = volumes_by_class(src);
    let (cells0, vol0) = (src.cell_count(), volume(src));
    assert_eq!(before.len(), 2);
    let run = script(&mut s, "mesh.refine(mesh: Mesh#1)");
    let m = layer(&s).get(mesh_id(2)).expect("level").mesh();
    let after = volumes_by_class(m);
    // The fuel is the smaller volume.
    let fuel = |v: &BTreeMap<String, f64>| v.values().copied().fold(f64::INFINITY, f64::min);
    let (fb, fa) = (fuel(&before), fuel(&after));
    let (eb, ea) = ((fb - fuel_exact).abs() / fuel_exact, (fa - fuel_exact).abs() / fuel_exact);
    let rep = &reports(&run)[0];
    eprintln!(
        "w3c pin cell: cells {} -> {}; fuel volume {fb:.9} -> {fa:.9} (pi r^2 h = {fuel_exact:.9}); relative error {eb:.3e} -> {ea:.3e}; \
         total {:.9} -> {:.9} (exact {total_exact}); min SJ {:.6} -> {:.6}; projected {} failed {} rejected {}",
        cells0,
        m.cell_count(),
        vol0,
        volume(m),
        rep["min_scaled_jacobian_parent"].as_f64().unwrap(),
        rep["min_scaled_jacobian"].as_f64().unwrap(),
        rep["projected"],
        rep["projection_failed"],
        rep["projection_rejected"],
    );
    assert!(fa > fb && fa < fuel_exact, "the fuel volume grows toward the cylinder");
    assert!(ea < eb / 3.0, "the circle polygon deficit falls about fourfold: {eb:e} -> {ea:e}");
    // The moderator's outer brick is straight-sided: the total stays exact.
    assert!((volume(m) - total_exact).abs() < 1e-12, "total volume {}", volume(m));
    assert!(validate(m).is_empty());
    let _ = m;
    // The same refinement without projection: straight midpoints.
    let mut s2 = server();
    script(&mut s2, PIN);
    let run2 = script(&mut s2, "mesh.refine(mesh: Mesh#1, project: false)");
    let rep2 = &reports(&run2)[0];
    eprintln!(
        "w3c pin cell without projection: min SJ {:.6}, volume {:.9}",
        rep2["min_scaled_jacobian"].as_f64().unwrap(),
        rep2["volume"].as_f64().unwrap()
    );
    // Quality: the worst cell is the corner cell with two boundary arcs; its angle is fixed by the
    // circle nodes, so the smart-Laplacian smoother cannot improve it (it never worsens it).
    let sj_refined = rep["min_scaled_jacobian"].as_f64().unwrap();
    script(&mut s, "mesh.smooth(mesh: Mesh#2, method: {kind: \"smart_laplacian\", relax: 0.5}, iterations: 20)");
    let sm = layer(&s).get(mesh_id(2)).expect("level").mesh();
    let sj_smooth = sm_quality::compute(sm, sm_quality::Metric::ScaledJacobian, &sm_quality::Options::default())
        .expect("sj")
        .into_iter()
        .fold(f64::INFINITY, f64::min);
    eprintln!("w3c pin cell: min SJ of the refined mesh {sj_refined:.6}, after 20 smart-Laplacian sweeps {sj_smooth:.6}");
    assert!((sj_smooth - sj_refined).abs() < 1e-9, "the corner cell's angle is fixed by the boundary nodes");
    assert!(sj_smooth >= sj_refined - 1e-12, "smart smoothing never worsens the minimum");
    assert!((volume(sm) - total_exact).abs() < 1e-9, "smoothing keeps the total volume");
}

/// `tools/w3c_pin.smesh` up to (not including) its exports: the 3x3 pin lattice with interface
/// faces between merged volumes, whose edges are not decided by the boundary faces alone.
fn lattice_script() -> String {
    let full = include_str!("../../../tools/w3c_pin.smesh");
    let cut = full.find("// ---------------------------------------------------------------- exports").expect("exports marker");
    full[..cut].to_owned()
}

#[test]
fn w3c_lattice_refines_with_interface_faces() {
    let mut s = server();
    script(&mut s, &lattice_script());
    let src = layer(&s).get(mesh_id(1)).expect("mesh").mesh();
    let n0 = src.cell_count();
    let per_volume = |m: &dyn MeshRead| {
        let mut out: BTreeMap<String, usize> = BTreeMap::new();
        for c in 0..m.cell_count() {
            *out.entry(m.cell_class(CellIx::new(c)).entity.to_string()).or_default() += 1;
        }
        out
    };
    let before = per_volume(src);
    assert_eq!(before.len(), 18);
    script(&mut s, "mesh.refine(mesh: Mesh#1)");
    let m = layer(&s).get(mesh_id(2)).expect("level").mesh();
    assert_eq!(m.cell_count(), 8 * n0);
    for (k, v) in per_volume(m) {
        assert_eq!(v, 8 * before[&k], "volume {k}");
    }
    let viol = validate(m);
    assert!(viol.is_empty(), "{viol:?}");
    assert!((volume(m) - 9.0 * 1.26 * 1.26).abs() < 1e-12);
    let map = &layer(&s).maps_from(mesh_id(2))[0].1;
    assert_eq!(map.fine_len(), 8 * n0);
    assert!(map.face_map().is_some());
}

/// A brick meshed with one interval per side: every edge of the mesh lies on a model edge between
/// two model vertices, so the edge classes must come from the topology.
#[test]
fn coarse_brick_with_one_interval_refines() {
    let mut s = server();
    script(
        &mut s,
        r#"
let b = model.create_brick(x: 2.0, y: 3.0, z: 4.0)
mesh.size(targets: all volumes, size: 10.0)
let au = mesh.auto(strategy: "default")
let m = mesh.run(targets: all volumes)
"#,
    );
    let n0 = layer(&s).get(mesh_id(1)).expect("mesh").mesh().cell_count();
    script(&mut s, "mesh.refine(mesh: Mesh#1, levels: 2)");
    let m = layer(&s).get(mesh_id(3)).expect("level 2").mesh();
    assert_eq!(m.cell_count(), 64 * n0);
    assert!(validate(m).is_empty());
    assert!((volume(m) - 24.0).abs() < 1e-12);
}

fn cold(ws: &mut Workspace) -> sm_op_api::State {
    let q = QuietControl::new();
    let ctl: Control<'_> = q.control();
    let head = ws.session().position().expect("head");
    ws.session().state_at_from_root(head, &ctl).expect("cold replay")
}

#[test]
fn replay_and_fork_give_equal_hashes_and_keep_the_link() {
    let mut s = server();
    script(&mut s, CYLINDER);
    let h_before = ws_hash(&s);
    script(&mut s, "mesh.refine(mesh: Mesh#1, levels: 2)");
    let h = ws_hash(&s);
    assert_ne!(h, h_before);
    {
        let l = layer(&s);
        assert_eq!(l.maps_from(mesh_id(2)).len(), 1, "level 1 links to the source");
        assert_eq!(l.maps_from(mesh_id(3)).len(), 1, "level 2 links to level 1");
        assert_eq!(l.maps_to(mesh_id(1)).len(), 1);
    }
    let mut ws = s.into_workspace();
    // Cold replay from the root (no cached states) reproduces the hash, links included.
    let replayed = cold(&mut ws);
    assert_eq!(replayed.hash().to_string(), h);
    let links = |st: &sm_op_api::State| st.layer::<MeshLayer>().expect("layer").maps_from(mesh_id(3)).len();
    assert_eq!(links(&replayed), 1);
    // A fork at the state before the refine, redoing the same op, reaches the same hash.
    let before = ws.log(None, 10).expect("log");
    assert!(before.len() >= 2);
    let parent = before[1].node;
    ws.fork("again", Some(parent)).expect("fork");
    assert_eq!(ws.state_hash().to_string(), h_before);
    ws.apply_json("mesh.refine", json!({"mesh": "Mesh#1", "levels": 2}), &Default::default()).expect("refine on the fork");
    assert_eq!(ws.state_hash().to_string(), h, "fork and original agree");
    assert_eq!(links(ws.state()), 1);
}

fn ws_hash(s: &Server) -> String {
    hash(s)
}

#[test]
fn link_survives_smoothing_and_drops_on_replace_or_retire() {
    let mut s = server();
    script(&mut s, CYLINDER);
    script(&mut s, "mesh.refine(mesh: Mesh#1)");
    assert_eq!(layer(&s).maps_from(mesh_id(2)).len(), 1);
    // A smooth keeps connectivity: the link stays (either mesh).
    script(&mut s, "mesh.smooth(mesh: Mesh#2, method: {kind: \"laplacian\", relax: 0.5}, iterations: 2)");
    assert_eq!(layer(&s).maps_from(mesh_id(2)).len(), 1, "link survives smoothing the fine mesh");
    script(&mut s, "mesh.smooth(mesh: Mesh#1, method: {kind: \"laplacian\", relax: 0.5}, iterations: 2)");
    assert_eq!(layer(&s).maps_from(mesh_id(2)).len(), 1, "link survives smoothing the coarse mesh");
    // Refining the fine mesh again with keep_source false retires it: its links drop.
    script(&mut s, "mesh.refine(mesh: Mesh#2, keep_source: false)");
    let l = layer(&s);
    assert!(l.get(mesh_id(2)).is_none(), "the source was retired");
    assert!(l.get(mesh_id(3)).is_some());
    assert!(l.maps_from(mesh_id(2)).is_empty() && l.maps_to(mesh_id(2)).is_empty());
    assert!(l.maps_from(mesh_id(3)).is_empty(), "no link is recorded for a replaced source");
    assert_eq!(l.get(mesh_id(3)).expect("mesh").mesh().cell_count(), 240 * 64);
}

#[test]
fn refusals_leave_the_state_unchanged() {
    let mut s = server();
    // A polyhedral mesh: dualize a tet mesh.
    script(
        &mut s,
        r#"
let b = model.create_brick(x: 1.0, y: 1.0, z: 1.0)
mesh.size(targets: all volumes, size: 0.5)
mesh.scheme(targets: all volumes, scheme: "tetmesh")
let m = mesh.run(targets: all volumes)
let p = mesh.to_poly(mesh: m.mesh[0])
"#,
    );
    let h = hash(&s);
    let n = layer(&s).mesh_ids().len();
    let e = s
        .call(Request::Apply { op: "mesh.refine".into(), params: json!({"mesh": "Mesh#2"}), version: None })
        .expect_err("polyhedral mesh is refused");
    assert!(e.to_string().contains("dualize again"), "{e}");
    assert_eq!(hash(&s), h);
    assert_eq!(layer(&s).mesh_ids().len(), n);
    // levels out of range.
    let e = s.call(Request::Apply { op: "mesh.refine".into(), params: json!({"mesh": "Mesh#1", "levels": 7}), version: None }).expect_err("levels");
    assert!(e.to_string().contains("levels"), "{e}");
    assert_eq!(hash(&s), h);
    // The tet mesh itself refines, and its boundary stays valid.
    script(&mut s, "mesh.refine(mesh: Mesh#1)");
    let t = layer(&s).get(mesh_id(3)).expect("tet level").mesh();
    assert!(validate(t).is_empty());
    assert!((volume(t) - 1.0).abs() < 1e-12);
    let _ = (FaceIx::new(0), FaceKind::Boundary);
}

#[test]
fn the_op_is_deterministic_across_runs_and_thread_counts() {
    let run_once = || {
        let mut s = server();
        script(&mut s, CYLINDER);
        script(&mut s, "mesh.refine(mesh: Mesh#1, levels: 2)");
        hash(&s)
    };
    let a = run_once();
    assert_eq!(a, run_once());
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool");
    assert_eq!(a, one.install(run_once));
}

/// Diagnostic codes of the `mesh.refine` events of a script run.
fn diag_codes(run: &Value) -> Vec<String> {
    run["events"]
        .as_array()
        .expect("events")
        .iter()
        .filter(|e| e["op"] == "mesh.refine")
        .flat_map(|e| e["delta"]["diagnostics"].as_array().cloned().unwrap_or_default())
        .filter_map(|d| d["code"].as_str().map(str::to_string))
        .collect()
}

/// The pin's worst cell is the corner cell with two boundary arcs: its minimum scaled Jacobian is
/// `sin(2 pi / n)` for `n` circle intervals (0.7071, 0.3827, 0.1951, 0.0980 at n = 8, 16, 32, 64),
/// independent of node placement; every level below 0.6 of its parent warns. Without projection
/// the minimum stays 0.7071 and nothing warns.
#[test]
fn quality_drop_per_level_is_the_boundary_corner_angle_and_warns() {
    let mut s = server();
    script(&mut s, PIN);
    let run = script(&mut s, "mesh.refine(mesh: Mesh#1, levels: 3)");
    let reps = reports(&run);
    assert_eq!(reps.len(), 3);
    for (k, rep) in reps.iter().enumerate() {
        let n = 8.0 * 2f64.powi(k as i32 + 1);
        let sj = rep["min_scaled_jacobian"].as_f64().expect("sj");
        assert!((sj - (std::f64::consts::TAU / n).sin()).abs() < 1e-6, "level {} min SJ {sj} vs sin(2pi/{n})", k + 1);
        eprintln!(
            "pin level {}: cells {} min SJ {sj:.6} (parent {:.6})",
            k + 1,
            rep["cells"],
            rep["min_scaled_jacobian_parent"].as_f64().expect("parent sj")
        );
    }
    let n_warn = diag_codes(&run).iter().filter(|c| *c == "refine.quality").count();
    assert_eq!(n_warn, 3, "every level halves the minimum");
    let mut s2 = server();
    script(&mut s2, PIN);
    let run2 = script(&mut s2, "mesh.refine(mesh: Mesh#1, levels: 3, project: false)");
    for rep in reports(&run2) {
        assert!((rep["min_scaled_jacobian"].as_f64().expect("sj") - 0.5f64.sqrt()).abs() < 1e-6);
    }
    assert!(!diag_codes(&run2).iter().any(|c| c == "refine.quality"));
}
