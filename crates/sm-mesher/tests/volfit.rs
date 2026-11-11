//! `mesh.preserve_volume` (W4-HEX-VOLFIT, ruling D4) on the real stack: analytic region volumes,
//! the refusals, determinism.

use std::f64::consts::PI;

use serde_json::{Value, json};
use sm_core::EntityId;
use sm_layer_api::ModelLayer;
use sm_protocol::{Endpoint, Request, RpcError};
use sm_scheme_api::geom::{edge_curve, edge_length, edge_point};
use sm_server::{Server, standard_with};
use sm_topo_api::Dim;

fn server() -> Server {
    Server::new(standard_with(Some("tester".into()), Some("2026-11-10T00:00:00Z".into()))).expect("server")
}

fn script(s: &mut Server, text: &str) -> Result<Value, RpcError> {
    s.call(Request::ScriptRun { text: text.into(), path: None, includes: Default::default() })
}

/// The first line of an error, cut short (a script error carries a long rendering).
fn brief(e: &RpcError) -> String {
    e.to_string().chars().take(300).collect()
}

fn apply(s: &mut Server, op: &str, params: Value) -> Result<Value, RpcError> {
    s.call(Request::Apply { op: op.into(), params, version: None })
}

/// Circular edges, and the cylindrical (non-planar) faces that bound a volume, ascending.
fn circles_and_cylinders(s: &Server) -> (Vec<EntityId>, Vec<EntityId>) {
    let st = s.workspace().state();
    let l = st.layer::<ModelLayer>().expect("model");
    let t = l.topo();
    let circles = t
        .cells(Dim::Edge)
        .iter()
        .copied()
        .filter(|&e| edge_curve(t, e).is_ok_and(|(c, _)| c.domain().period.is_some()))
        .collect();
    let cyl = t
        .cells(Dim::Face)
        .iter()
        .copied()
        .filter(|&f| t.face_plane(f, 1e-9).ok().flatten().is_none())
        .collect();
    (circles, cyl)
}

fn info(s: &mut Server) -> Value {
    s.call(Request::Query { name: "mesh.info".into(), input: json!({"mesh": "Mesh#1"}), at: None }).expect("info")
}


const RING_SECTOR: &str = "let fu = model.create_cylinder(height: 1.0, radius: 0.5)\n\
    model.subdivide_pin(volumes: fu.volume, axis: [0.0, 0.0, 1.0], center: [0.0, 0.0, 0.0], rings: {count: 3}, sectors: {count: 4})\n";

/// Intervals of a circular edge: one per 22.5 degrees (n = 16 per circle), at least 1. The seam of a
/// ring circle splits a quadrant into two edges, so the counts follow the angle, not the edge.
fn counts(s: &Server, circles: &[EntityId]) -> Vec<(EntityId, u32)> {
    let st = s.workspace().state();
    let l = st.layer::<ModelLayer>().expect("model");
    let t = l.topo();
    circles
        .iter()
        .map(|&e| {
            let p = edge_point(t, e, 0.5).expect("point");
            let r = p.x.hypot(p.y);
            let angle = edge_length(t, e).expect("length") / r;
            (e, ((angle / (2.0 * PI / 16.0)).round() as u32).max(1))
        })
        .collect()
}

/// 3 equal-area rings x 4 sectors, n = 16 on every circle, hex mesh.
fn ring_sector_mesh() -> Server {
    let mut s = server();
    script(&mut s, RING_SECTOR).expect("pin");
    let (circles, _) = circles_and_cylinders(&s);
    for (e, n) in counts(&s, &circles) {
        apply(&mut s, "mesh.intervals", json!({"targets": [e.to_string()], "count": n})).expect("intervals");
    }
    script(
        &mut s,
        "mesh.size(targets: all volumes, size: 0.2)\nmesh.auto(strategy: \"hex_only\")\nlet m = mesh.run(targets: all volumes)\n",
    )
    .unwrap_or_else(|e| panic!("mesh: {}", brief(&e)));
    s
}

fn volumes_by_owner(i: &Value) -> Vec<f64> {
    i["volume_by_owner"].as_object().expect("by owner").values().map(|v| v.as_f64().expect("f64")).collect()
}

#[test]
fn rings_and_sectors_hold_pi_r2_h_over_12_to_1e_12_after_the_op() {
    let mut s = ring_sector_mesh();
    let before = info(&mut s);
    assert_eq!(before["cell_shapes"]["Hex"], before["cells"], "all hex: {before}");
    let regions = volumes_by_owner(&before);
    assert_eq!(regions.len(), 12, "3 rings x 4 sectors: {before}");
    let want = PI * 0.25 / 12.0;
    // The linear mesh under-fills every region (about 2.5 percent at n = 16).
    for v in &regions {
        assert!(*v < want * 0.995 && *v > want * 0.95, "before: {v} vs {want}");
    }

    let (_, cyl) = circles_and_cylinders(&s);
    assert!(cyl.len() >= 12, "at least one cylindrical face per ring and sector: {}", cyl.len());
    let ids: Vec<String> = cyl.iter().map(|e| e.to_string()).collect();
    apply(&mut s, "mesh.preserve_volume", json!({"mesh": "Mesh#1", "surfaces": ids})).expect("preserve");
    let after = info(&mut s);
    let regions = volumes_by_owner(&after);
    assert_eq!(regions.len(), 12);
    for v in &regions {
        assert!((v / want - 1.0).abs() < 1e-12, "after: region volume {v} vs pi r^2 h / 12 = {want}");
    }
    assert!((after["volume"].as_f64().expect("vol") / (PI * 0.25) - 1.0).abs() < 1e-12, "{after}");
    assert_eq!(after["cells"], before["cells"], "no cell is added or removed");
    // The mesh is still conformal and valid.
    let val = s.call(Request::Query { name: "mesh.validate".into(), input: json!({"mesh": "Mesh#1"}), at: None }).expect("validate");
    assert_eq!(val["total"], 0, "{val}");
}

#[test]
fn a_single_layer_cylinder_is_exact_through_its_circles() {
    let mut s = server();
    script(&mut s, "let fu = model.create_cylinder(height: 2.0, radius: 0.4)\n").expect("cyl");
    let (circles, _) = circles_and_cylinders(&s);
    assert_eq!(circles.len(), 2);
    let ids: Vec<String> = circles.iter().map(|e| e.to_string()).collect();
    apply(&mut s, "mesh.intervals", json!({"targets": ids, "count": 8})).expect("n = 8");
    script(&mut s, "mesh.size(targets: all volumes, size: 2.0)\nmesh.auto(strategy: \"hex_only\")\nlet m = mesh.run(targets: all volumes)\n").expect("mesh");
    let before = info(&mut s)["volume"].as_f64().expect("vol");
    let exact = PI * 0.16 * 2.0;
    assert!((before / exact - (2.0 * PI / 8.0).sin() / (2.0 * PI / 8.0)).abs() < 1e-12, "{before}");
    // Two circles, one axial layer: moving the circle nodes alone is the whole boundary.
    let ids: Vec<String> = circles.iter().map(|e| e.to_string()).collect();
    apply(&mut s, "mesh.preserve_volume", json!({"mesh": "Mesh#1", "curves": ids})).expect("preserve");
    let after = info(&mut s)["volume"].as_f64().expect("vol");
    assert!((after / exact - 1.0).abs() < 1e-12, "{after} vs {exact}");
}

#[test]
fn refusals_leave_the_state_alone() {
    let mut s = ring_sector_mesh();
    let (circles, cyl) = circles_and_cylinders(&s);
    let h0 = s.workspace().state_hash();
    let c: Vec<String> = circles.iter().map(|e| e.to_string()).collect();
    let f: Vec<String> = cyl.iter().map(|e| e.to_string()).collect();
    // Both lists, neither list, an empty list.
    for params in [
        json!({"mesh": "Mesh#1", "curves": c, "surfaces": f}),
        json!({"mesh": "Mesh#1"}),
        json!({"mesh": "Mesh#1", "curves": []}),
    ] {
        let e = apply(&mut s, "mesh.preserve_volume", params).expect_err("must refuse");
        assert!(e.to_string().contains("exactly one non-empty list"), "{e}");
    }
    // One arc of a circle split in four is not a whole circle.
    let e = apply(&mut s, "mesh.preserve_volume", json!({"mesh": "Mesh#1", "curves": [c[0]]})).expect_err("partial circle");
    assert!(e.to_string().contains("select every arc"), "{e}");
    // A planar face has no cylinder to preserve.
    let st = s.workspace().state();
    let l = st.layer::<ModelLayer>().expect("model");
    let planar = l.topo().cells(Dim::Face).iter().copied().find(|&f| l.topo().face_plane(f, 1e-9).ok().flatten().is_some()).expect("planar");
    let e = apply(&mut s, "mesh.preserve_volume", json!({"mesh": "Mesh#1", "surfaces": [planar.to_string()]})).expect_err("planar");
    assert!(e.to_string().contains("mesh.preserve_volume"), "{e}");
    assert_eq!(s.workspace().state_hash(), h0, "a refused op leaves the state alone");

    // The op is not idempotent: a second application finds the nodes off the model circle.
    apply(&mut s, "mesh.preserve_volume", json!({"mesh": "Mesh#1", "surfaces": f})).expect("first");
    let h1 = s.workspace().state_hash();
    assert_ne!(h1, h0);
    let e = apply(&mut s, "mesh.preserve_volume", json!({"mesh": "Mesh#1", "surfaces": f})).expect_err("second");
    assert!(e.to_string().contains("off the model circle"), "{e}");
    assert_eq!(s.workspace().state_hash(), h1);
}

#[test]
fn same_input_same_state_hash_and_node_bits() {
    let run = || {
        let mut s = ring_sector_mesh();
        let (_, cyl) = circles_and_cylinders(&s);
        let f: Vec<String> = cyl.iter().map(|e| e.to_string()).collect();
        apply(&mut s, "mesh.preserve_volume", json!({"mesh": "Mesh#1", "surfaces": f})).expect("preserve");
        (s.workspace().state_hash(), info(&mut s)["geometry_hash"].clone())
    };
    assert_eq!(run(), run());
    // Bit-identical for 1 thread and N threads.
    let in_pool = |threads: usize| {
        rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool").install(run)
    };
    assert_eq!(in_pool(1), in_pool(4), "1 vs 4 threads");
}

/// VOLFIT acceptance on the W3c through pin (default auto, size 0.3): fuel and moderator hold their
/// analytic volumes to 1e-12 after the op on the cylinder face, the total is unchanged, the mesh is valid.
#[test]
fn the_w3c_through_pin_holds_fuel_and_moderator_volumes() {
    let mut s = server();
    script(
        &mut s,
        "let br = model.create_brick(x: 1.26, y: 1.26, z: 1.0)\n\
         let fu = model.create_cylinder(height: 1.0, radius: 0.4)\n\
         let tl = model.create_cylinder(height: 1.0, radius: 0.4)\n\
         let mo = model.subtract(targets: br.body, tools: tl.body)\n\
         model.imprint(bodies: all bodies)\n\
         model.merge()\n\
         mesh.size(targets: all volumes, size: 0.3)\n\
         mesh.auto(strategy: \"default\")\n\
         let m = mesh.run(targets: all volumes)\n",
    )
    .unwrap_or_else(|e| panic!("pin: {}", brief(&e)));
    let before = info(&mut s);
    let (_, cyl) = circles_and_cylinders(&s);
    assert!(!cyl.is_empty(), "the pin has a cylindrical face");
    let ids: Vec<String> = cyl.iter().map(|e| e.to_string()).collect();
    apply(&mut s, "mesh.preserve_volume", json!({"mesh": "Mesh#1", "surfaces": ids})).expect("preserve");
    let after = info(&mut s);
    let mut vols = volumes_by_owner(&after);
    vols.sort_by(f64::total_cmp);
    assert_eq!(vols.len(), 2, "fuel and moderator: {after}");
    let fuel = PI * 0.16;
    let moderator = 1.26 * 1.26 - fuel;
    assert!((vols[0] / fuel - 1.0).abs() < 1e-12, "fuel {} vs {fuel}", vols[0]);
    assert!((vols[1] / moderator - 1.0).abs() < 1e-12, "moderator {} vs {moderator}", vols[1]);
    let total = after["volume"].as_f64().expect("vol");
    assert!((total / (1.26 * 1.26) - 1.0).abs() < 1e-12, "total {total}");
    assert!((total / before["volume"].as_f64().expect("vol") - 1.0).abs() < 1e-12, "the brick total is unchanged");
    assert_eq!(after["cells"], before["cells"]);
    let val = s.call(Request::Query { name: "mesh.validate".into(), input: json!({"mesh": "Mesh#1"}), at: None }).expect("validate");
    assert_eq!(val["total"], 0, "{val}");
}
