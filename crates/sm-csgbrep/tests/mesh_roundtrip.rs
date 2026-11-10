//! deck -> B-rep -> mesh: a pin through `mesh.auto`, `mesh.run` and `mesh.validate` in the
//! standard distribution (the test-only dependency on `sm-server`; the crate itself has none).
mod common;

use base64::Engine as _;
use serde_json::{Value, json};
use sm_protocol::{Endpoint, Request};
use sm_server::{Server, standard_with};

fn server() -> Server {
    Server::new(standard_with(Some("tester".into()), Some("2026-10-24T00:00:00Z".into()))).expect("server")
}

fn b64(s: &str) -> String {
    base64::engine::general_purpose::STANDARD.encode(s.as_bytes())
}

fn script(s: &mut Server, text: &str) -> Value {
    s.call(Request::ScriptRun { text: text.into(), path: None, includes: Default::default() }).unwrap_or_else(|e| panic!("script failed: {e}\n{}", e.data["rendered"].as_str().unwrap_or("")))
}

/// Import `geometry`, mesh every volume at `size` with `mesh.auto default` and validate. The
/// conformal mesh of the regions fills the box exactly: `box_volume`.
fn round_trip(geometry: &str, materials: &str, size: f64, box_volume: f64, min_cells: f64) {
    let mut s = server();
    s.call(Request::Apply { op: "model.import_openmc".into(), params: json!({"geometry": {"inline": b64(geometry)}, "materials": {"inline": b64(materials)}}), version: None }).unwrap_or_else(|e| panic!("import: {e} {}", e.data));
    let out = script(
        &mut s,
        &format!(
            r#"
assert count(all volumes) == 4, "four volumes"
mesh.size(targets: all volumes, size: {size})
let au = mesh.auto(strategy: "default")
let m = mesh.run(targets: all volumes)
let mi = mesh.info(mesh: m.mesh[0])
print "cells", mi.cells
print "shapes", mi.cell_shapes
print "mesh_volume", mi.volume
let val = mesh.validate(mesh: m.mesh[0])
print "violations", val.total
assert val.valid and val.total == 0, "validate is empty (conformal interfaces)"
"#
        ),
    );
    let text = out["output"].as_str().unwrap_or_default().to_owned();
    println!("{text}");
    let get = |key: &str| -> f64 { text.lines().find_map(|l| l.strip_prefix(key).map(|r| r.trim().parse::<f64>().expect("number"))).unwrap_or_else(|| panic!("no `{key}` in {text}")) };
    assert!((get("mesh_volume") - box_volume).abs() < 1e-11 * box_volume.max(1.0), "{text}");
    assert_eq!(get("violations"), 0.0);
    assert!(get("cells") > min_cells);
}

#[test]
fn a_pin_meshes_and_validates() {
    // Fuel 0.3, gap to 0.4, clad to 0.5 in a 1.26 box of height 1.
    round_trip(&common::decks::pin_lattice(1, 1.26, (0.3, 0.4, 0.5), 1.0, false), &common::materials(4), 0.15, 1.26 * 1.26, 100.0);
}

/// The committed `pin` fixture (a 0.02-wide gap) meshes with `default` at size 0.04: 600k tets.
/// At size 0.3 `mesh.auto` fails (pave: no perfect matching on the gap annulus, trimesh: minimum
/// angle below the gate), and at 0.1 no scheme applies to the clad ring; a mesher finding, not an
/// import one.
#[test]
#[ignore = "about 80 s in debug (600k tets); run with --ignored"]
fn the_pin_fixture_meshes_and_validates_at_gap_resolution() {
    round_trip(&common::fixture("pin", "geometry.xml"), &common::fixture("pin", "materials.xml"), 0.04, 1.26 * 1.26 * 4.0, 1000.0);
}
