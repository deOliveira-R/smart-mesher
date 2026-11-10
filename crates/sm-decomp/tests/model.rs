//! The planner against real models: the plan read from the live topology, applied through the ordinary
//! ops, then meshed with `default` (all hex, no violations, volume preserved to 1e-12).

use serde_json::{Value, json};
use sm_core::EntityId;
use sm_decomp::{CutStep, DecompError, plan_cuts, render_script};
use sm_layer_api::ModelLayer;
use sm_protocol::{Endpoint, Request};
use sm_server::{Server, standard_with};
use sm_topo_api::Dim;

fn server() -> Server {
    Server::new(standard_with(
        Some("tester".into()),
        Some("2026-11-09T00:00:00Z".into()),
    ))
    .expect("server")
}

fn run(s: &mut Server, text: &str) -> String {
    let v = s
        .call(Request::ScriptRun {
            text: text.into(),
            path: None,
            includes: Default::default(),
        })
        .unwrap_or_else(|e| {
            panic!(
                "script failed: {e}\n{}\n{text}",
                e.data["rendered"].as_str().unwrap_or("")
            )
        });
    v["output"].as_str().unwrap_or_default().to_owned()
}

fn volumes(s: &Server) -> Vec<EntityId> {
    let layer = s
        .workspace()
        .state()
        .layer::<ModelLayer>()
        .expect("model layer");
    layer.topo().cells(Dim::Volume).to_vec()
}

fn plan(s: &Server, v: EntityId) -> Result<Vec<CutStep>, DecompError> {
    let layer = s
        .workspace()
        .state()
        .layer::<ModelLayer>()
        .expect("model layer");
    plan_cuts(layer.topo(), v)
}

fn volume_of(s: &mut Server, v: EntityId) -> f64 {
    let r: Value = s
        .call(Request::Query {
            name: "model.measure".into(),
            input: json!({"entity": v.to_string()}),
            at: None,
        })
        .expect("measure");
    r["volume"].as_f64().expect("volume")
}

fn total_volume(s: &mut Server) -> f64 {
    volumes(s).into_iter().map(|v| volume_of(s, v)).sum()
}

fn print_of(out: &str, key: &str) -> String {
    out.lines()
        .find_map(|l| l.strip_prefix(key).map(|r| r.trim().to_owned()))
        .unwrap_or_else(|| panic!("no `{key}` in output:\n{out}"))
}

/// The host (the one volume with a non-empty plan) and its steps; every other volume plans empty.
fn host_plan(s: &Server) -> (EntityId, Vec<CutStep>) {
    let mut found = None;
    for v in volumes(s) {
        let p = plan(s, v).expect("plan");
        if !p.is_empty() {
            assert!(found.is_none(), "two volumes with cuts");
            found = Some((v, p));
        }
    }
    found.expect("a volume with cuts")
}

/// Mesh every volume with `default`, then check the result.
const MESH: &str = "model.imprint(bodies: all bodies)\nmodel.merge()\nprint \"nvol\", count(all volumes)\nfem.block_add(1, volumes: all volumes, name: \"all\")\nmesh.size(targets: all volumes, size: 0.3)\nlet au = mesh.auto(strategy: \"default\")\nlet m = mesh.run(targets: all volumes)\nlet mi = mesh.info(mesh: m.mesh[0])\nprint \"cells\", mi.cells\nprint \"shapes\", mi.cell_shapes\nprint \"mesh_volume\", mi.volume\nlet val = mesh.validate(mesh: m.mesh[0])\nprint \"violations\", val.total\n";

/// Every cell is a hex: the shapes line is exactly `{Hex: <cells>}`.
fn assert_all_hex(o: &Outcome) {
    assert_eq!(print_of(&o.out, "shapes"), format!("{{Hex: {}}}", print_of(&o.out, "cells")), "{}", o.out);
}

struct Outcome {
    steps: Vec<CutStep>,
    out: String,
    before: f64,
    after: f64,
}

/// Build `model`, plan the host, apply the steps, mesh, and check that every volume then plans empty.
fn apply_plan(model: &str) -> Outcome {
    let mut s = server();
    run(&mut s, model);
    let before = total_volume(&mut s);
    let (host, steps) = host_plan(&s);
    let text = render_script(&steps, &host.to_string(), "cut");
    let out = run(&mut s, &format!("{text}{MESH}"));
    let after = total_volume(&mut s);
    for v in volumes(&s) {
        assert!(
            plan(&s, v).expect("replan").is_empty(),
            "{v} still needs cuts after the plan"
        );
    }
    Outcome {
        steps,
        out,
        before,
        after,
    }
}

fn pin(h: f64) -> String {
    format!(
        "let br = model.create_brick(x: 1.26, y: 1.26, z: 1.0)\nlet fu = model.create_cylinder(height: {h}, radius: 0.4)\nlet tl = model.create_cylinder(height: {h}, radius: 0.4)\nlet mo = model.subtract(targets: br.body, tools: tl.body)\nmodel.imprint(bodies: all bodies)\nmodel.merge()\n"
    )
}

#[test]
fn enclosed_pin_becomes_six_all_hex_volumes() {
    let o = apply_plan(&pin(0.8));
    assert_eq!(o.steps.len(), 3, "{:?}", o.steps);
    assert_eq!(
        print_of(&o.out, "nvol"),
        "6",
        "three square-annulus slabs, two caps, the fuel"
    );
    assert_eq!(print_of(&o.out, "shapes"), "{Hex: 80}");
    assert_eq!(print_of(&o.out, "cells"), "80");
    assert_eq!(print_of(&o.out, "violations"), "0");
    // Volume is preserved across the cuts, and the mesh fills the brick.
    assert!(
        (o.before - o.after).abs() <= 1e-12 * o.before,
        "{} vs {}",
        o.before,
        o.after
    );
    let mv: f64 = print_of(&o.out, "mesh_volume").parse().expect("float");
    assert!((mv - 1.26 * 1.26).abs() <= 1e-12 * mv);
    assert!((o.before - 1.26 * 1.26).abs() <= 1e-12, "{}", o.before);
}

#[test]
fn volumes_that_sweep_already_plan_empty() {
    // The through pin: the moderator has a through hole; the fuel is a convex cylinder.
    let mut s = server();
    run(&mut s, &pin(1.0));
    let vs = volumes(&s);
    assert_eq!(vs.len(), 2);
    for v in vs {
        assert!(plan(&s, v).expect("plan").is_empty());
    }
    let mut b = server();
    run(
        &mut b,
        "let br = model.create_brick(x: 1.0, y: 2.0, z: 3.0)\n",
    );
    assert!(plan(&b, volumes(&b)[0]).expect("brick").is_empty());
    // Not a volume.
    let e = plan(
        &b,
        EntityId::new(sm_core::EntityKind::TopoFace, 1).expect("id"),
    )
    .expect_err("a face");
    assert!(matches!(e, DecompError::NotAVolume(_)));
}

#[test]
fn two_enclosed_cavities_at_different_heights() {
    let model = "let br = model.create_brick(x: 3.0, y: 1.5, z: 1.0)\n\
        let f1 = model.create_cylinder(height: 0.6, radius: 0.4, center: [-0.8, 0.1, -0.1])\n\
        let f2 = model.create_cylinder(height: 0.5, radius: 0.3, center: [0.8, 0.0, 0.15])\n\
        let t1 = model.create_cylinder(height: 0.6, radius: 0.4, center: [-0.8, 0.1, -0.1])\n\
        let t2 = model.create_cylinder(height: 0.5, radius: 0.3, center: [0.8, 0.0, 0.15])\n\
        let m1 = model.subtract(targets: br.body, tools: t1.body)\n\
        let m2 = model.subtract(targets: m1.bodies, tools: t2.body)\n";
    let o = apply_plan(model);
    // Two cylinders and the four distinct end planes.
    assert_eq!(o.steps.len(), 6);
    assert_eq!(print_of(&o.out, "violations"), "0");
    assert_all_hex(&o);
    assert!((o.before - o.after).abs() <= 1e-12 * o.before);
    assert!((o.before - 3.0 * 1.5).abs() <= 1e-12 * o.before);
}

#[test]
fn off_axis_cavity() {
    let model = "let br = model.create_brick(x: 1.26, y: 1.26, z: 1.0)\n\
        let fu = model.create_cylinder(height: 0.5, radius: 0.25, center: [0.2, -0.15, 0.05])\n\
        let tl = model.create_cylinder(height: 0.5, radius: 0.25, center: [0.2, -0.15, 0.05])\n\
        let mo = model.subtract(targets: br.body, tools: tl.body)\n";
    let o = apply_plan(model);
    assert_eq!(o.steps.len(), 3);
    assert_eq!(print_of(&o.out, "violations"), "0");
    assert_all_hex(&o);
    assert!((o.before - o.after).abs() <= 1e-12 * o.before);
}

#[test]
fn blind_holes_from_the_bottom_and_the_top_face() {
    // A pin cavity open at one face: the end plane plus one cylinder on the solid slab beyond it
    // (a lone plane leaves a slab whose imprinted face has a hole, so it does not sweep).
    for (name, z) in [("bottom", -0.15), ("top", 0.15)] {
        let model = format!(
            "let br = model.create_brick(x: 1.26, y: 1.26, z: 1.0)\n\
             let fu = model.create_cylinder(height: 0.7, radius: 0.4, center: [0.0, 0.0, {z}])\n\
             let tl = model.create_cylinder(height: 0.7, radius: 0.4, center: [0.0, 0.0, {z}])\n\
             let mo = model.subtract(targets: br.body, tools: tl.body)\n"
        );
        let o = apply_plan(&model);
        assert_eq!(o.steps.len(), 2, "{name}: {:?}", o.steps);
        assert_eq!(print_of(&o.out, "nvol"), "4", "{name}: ring slab, solid slab split in ring and disc, fuel");
        assert_eq!(print_of(&o.out, "violations"), "0", "{name}");
        assert_all_hex(&o);
        assert!((o.before - o.after).abs() <= 1e-12 * o.before, "{name}");
    }
}

#[test]
fn stacked_fuel_zones_share_one_cavity() {
    // Two touching pellets in one bore: the moderator wall may be one face or two; either way the plan makes it sweepable.
    let model = "let br = model.create_brick(x: 1.26, y: 1.26, z: 1.0)\n\
        let f1 = model.create_cylinder(height: 0.4, radius: 0.4, center: [0.0, 0.0, -0.2])\n\
        let f2 = model.create_cylinder(height: 0.4, radius: 0.4, center: [0.0, 0.0, 0.2])\n\
        let t1 = model.create_cylinder(height: 0.4, radius: 0.4, center: [0.0, 0.0, -0.2])\n\
        let t2 = model.create_cylinder(height: 0.4, radius: 0.4, center: [0.0, 0.0, 0.2])\n\
        let m1 = model.subtract(targets: br.body, tools: t1.body)\n\
        let m2 = model.subtract(targets: m1.bodies, tools: t2.body)\n";
    let o = apply_plan(model);
    // The imprint splits the moderator wall at the pellet interface: a cylinder plus planes at -0.4, 0 and 0.4.
    assert_eq!(o.steps.len(), 4, "{:?}", o.steps);
    assert_eq!(
        print_of(&o.out, "nvol"),
        "8",
        "four ring slabs, two caps, two pellets"
    );
    assert_eq!(print_of(&o.out, "violations"), "0");
    assert_all_hex(&o);
    assert!((o.before - o.after).abs() <= 1e-12 * o.before);
}

#[test]
fn stepped_bore_is_refused_and_nothing_is_cut() {
    let mut s = server();
    run(
        &mut s,
        "let br = model.create_brick(x: 1.26, y: 1.26, z: 1.0)\n\
         let t1 = model.create_cylinder(height: 0.5, radius: 0.4, center: [0.0, 0.0, -0.05])\n\
         let t2 = model.create_cylinder(height: 0.2, radius: 0.25, center: [0.0, 0.0, 0.3])\n\
         let m1 = model.subtract(targets: br.body, tools: t1.body)\n\
         let m2 = model.subtract(targets: m1.bodies, tools: t2.body)\n",
    );
    let before = s.workspace().state_hash();
    let host = volumes(&s)[0];
    match plan(&s, host) {
        Err(DecompError::NoDecomposition { reason, .. }) => assert!(
            reason.contains("overlap or nest") || reason.contains("does not close"),
            "{reason}"
        ),
        other => panic!("expected a refusal, got {other:?}"),
    }
    assert_eq!(
        s.workspace().state_hash(),
        before,
        "planning leaves the state alone"
    );
}

#[test]
fn plans_are_bit_identical_across_runs_and_thread_counts() {
    fn digest() -> String {
        let mut s = server();
        run(&mut s, &pin(0.8));
        let (_, steps) = host_plan(&s);
        format!("{steps:?}")
    }
    let a = digest();
    assert_eq!(a, digest());
    let one = rayon::ThreadPoolBuilder::new()
        .num_threads(1)
        .build()
        .expect("pool")
        .install(digest);
    let many = rayon::ThreadPoolBuilder::new()
        .num_threads(4)
        .build()
        .expect("pool")
        .install(digest);
    assert_eq!(a, one);
    assert_eq!(a, many);
}
