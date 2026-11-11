//! HEX-OGRID-S through the real server: scripts with an explicit `mesh.scheme`.

use serde_json::Value;
use sm_protocol::{Endpoint, Request};
use sm_server::{Server, standard_with};
use std::path::PathBuf;

struct Scratch(PathBuf);
impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// Run `text` in a fresh server; returns the script output (or the error text).
fn run(name: &str, text: &str) -> Result<String, String> {
    let dir = Scratch(PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(format!("ogrid-{name}-{}", std::process::id())));
    let mut s =
        Server::create(standard_with(Some("t".into()), Some("2026-01-01T00:00:00Z".into())), dir.0.join("proj"), "t")
            .map_err(|e| e.to_string())?;
    let v: Value = s
        .call(Request::ScriptRun { text: text.to_owned(), path: None, includes: Default::default() })
        .map_err(|x| format!("{x} {}", x.data["rendered"].as_str().unwrap_or("")))?;
    Ok(v["output"].as_str().unwrap_or("").to_owned())
}

/// Value of `key` among the printed `key value` lines.
fn val(out: &str, key: &str) -> f64 {
    let line = out.lines().find(|l| l.split(' ').next() == Some(key)).unwrap_or_else(|| panic!("no {key} in {out}"));
    line[key.len()..].trim().parse().unwrap_or_else(|_| panic!("{key}: {line}"))
}

fn text(out: &str, key: &str) -> String {
    let line = out.lines().find(|l| l.split(' ').next() == Some(key)).unwrap_or_else(|| panic!("no {key} in {out}"));
    line[key.len()..].trim().to_owned()
}

/// Inscribed-polygon area `(n/2) r^2 sin(2 pi / n)`.
fn ngon(n: f64, r: f64) -> f64 {
    0.5 * n * r * r * (std::f64::consts::TAU / n).sin()
}

/// The per-owner volumes of the `by_owner` line, in the printed (ascending id) order.
fn owner_volumes(out: &str) -> Vec<f64> {
    text(out, "by_owner")
        .trim_matches(|c| c == '{' || c == '}')
        .split(", ")
        .map(|kv| kv.rsplit(": ").next().and_then(|v| v.parse().ok()).unwrap_or_else(|| panic!("by_owner: {kv}")))
        .collect()
}

/// Every `circ` line (the solved count of each circle edge); they must all be equal.
fn circle_count(out: &str) -> f64 {
    let all: Vec<f64> = out
        .lines()
        .filter_map(|l| l.strip_prefix("circ ").map(|v| v.trim().parse().expect("count")))
        .collect();
    assert!(!all.is_empty() && all.iter().all(|&c| c == all[0]), "circle counts {all:?} in {out}");
    all[0]
}

/// Each fuel owner is the inscribed n-gon prism and each moderator owner the brick minus it,
/// to 1e-12 relative; `pins` of each. This fails for any mesh that merely fills the brick.
fn check_owners(out: &str, pins: usize) {
    let n = circle_count(out);
    assert_eq!(n, 16.0, "the circle count the plan solved");
    let fuel = ngon(n, 0.4);
    let moder = 1.26 * 1.26 - fuel;
    let vols = owner_volumes(out);
    assert_eq!(vols.len(), 2 * pins, "{vols:?}");
    let near = |a: f64, b: f64| (a - b).abs() <= 1e-12 * b;
    assert_eq!(vols.iter().filter(|&&v| near(v, fuel)).count(), pins, "fuel {fuel}: {vols:?}");
    assert_eq!(vols.iter().filter(|&&v| near(v, moder)).count(), pins, "moderator {moder}: {vols:?}");
}

/// `session.hash` of the script under a pool of `threads`.
fn hash_with(threads: usize, name: &str, text_: &str) -> String {
    let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool");
    let out = pool.install(|| run(&format!("{name}{threads}"), text_)).unwrap();
    text(&out, "hash")
}

/// A z cylinder swept from an `ogrid` bottom cap; `{SIZE}` and `{PARAMS}` are substituted.
fn cyl(size: &str, params: &str) -> String {
    r#"
let c = model.create_cylinder(height: 1.0, radius: 0.4)
mesh.size(targets: all volumes, size: {SIZE})
for f in topo.related(kind: "TopoFace", within: c.volume) {
  let ms = model.measure(entity: f)
  if ms.area > 0.5 and ms.area < 0.51 and ms.centroid[2] < 0.0 {
    mesh.scheme(targets: f, scheme: "ogrid"{PARAMS})
  }
}
mesh.scheme(targets: c.volume, scheme: "sweep")
let m = mesh.run(targets: all volumes)
let mi = mesh.info(mesh: m.mesh[0])
let val = mesh.validate(mesh: m.mesh[0])
let pl = mesh.plan(mesh: m.mesh[0])
print "n", pl.intervals[0].count
print "layers", pl.intervals[2].count
print "cells", mi.cells
print "shapes", mi.cell_shapes
print "volume", mi.volume
print "violations", val.total
print "hash", session.hash()
"#
    .replace("{SIZE}", size)
    .replace("{PARAMS}", params)
}

#[test]
fn acc1_cylinder_ogrid_sweep_volume_is_the_inscribed_prism() {
    let out = run("cyl", &cyl("0.15", "")).unwrap();
    let (n, l) = (val(&out, "n"), val(&out, "layers"));
    assert_eq!(n, 16.0);
    // a = b = n/4 central block plus k = 2 collar layers (the layers follow the size 0.15): a^2 + k n
    // quads, swept l layers.
    assert_eq!(val(&out, "cells"), (16.0 + 2.0 * 16.0) * l);
    assert_eq!(text(&out, "shapes"), format!("{{Hex: {}}}", 48.0 * l));
    let want = 1.0 * ngon(n, 0.4);
    assert!((val(&out, "volume") - want).abs() <= 1e-12 * want, "{} vs {want}", val(&out, "volume"));
    assert_eq!(val(&out, "violations"), 0.0);
}

#[test]
fn acc2_odd_request_on_the_circle_gives_an_even_count() {
    // Size 0.1675 asks for 15.0 intervals on the circle.
    let out = run("odd", &cyl("0.1675", "")).unwrap();
    let n = val(&out, "n");
    assert_eq!(n % 2.0, 0.0, "n = {n}");
    assert!(n >= 4.0);
    let want = ngon(n, 0.4);
    assert!((val(&out, "volume") - want).abs() <= 1e-12 * want);
    assert_eq!(val(&out, "violations"), 0.0);
}

#[test]
fn params_rings_and_ratio_shape_the_collar() {
    let out = run("rings", &cyl("0.15", ", params: {rings: 3, ratio: 2.0}")).unwrap();
    let (n, l) = (val(&out, "n"), val(&out, "layers"));
    assert_eq!(val(&out, "cells"), ((n / 4.0).powi(2) + 3.0 * n) * l);
    let want = ngon(n, 0.4);
    assert!((val(&out, "volume") - want).abs() <= 1e-12 * want);
    assert_eq!(val(&out, "violations"), 0.0);
}

#[test]
fn polar_mode_is_a_prism_over_the_central_polygon() {
    let out = run("polar", &cyl("0.15", ", params: {mode: \"polar\"}")).unwrap();
    let (n, l) = (val(&out, "n"), val(&out, "layers"));
    let want = ngon(n, 0.4);
    assert!((val(&out, "volume") - want).abs() <= 1e-12 * want, "{} vs {want}", val(&out, "volume"));
    assert_eq!(val(&out, "violations"), 0.0);
    assert!(
        text(&out, "shapes").contains("Poly") || text(&out, "shapes").contains("Prism"),
        "{}",
        text(&out, "shapes")
    );
    assert!(val(&out, "cells") >= l * n);
}

#[test]
fn bad_parameters_and_other_faces_are_refused_with_the_scheme_named() {
    for bad in
        [", params: {mode: \"spiral\"}", ", params: {rings: 0}", ", params: {ratio: 1e9}", ", params: {bogus: 1}"]
    {
        let e = run("bad", &cyl("0.15", bad)).unwrap_err();
        assert!(e.contains("ogrid"), "{bad}: {e}");
    }
    // A brick face is not a disk.
    let e = run(
        "brick",
        r#"
let b = model.create_brick(x: 1.0, y: 1.0, z: 1.0)
mesh.size(targets: all volumes, size: 0.25)
mesh.scheme(targets: b.faces[0], scheme: "ogrid")
let m = mesh.run(targets: b.faces[0])
"#,
    )
    .unwrap_err();
    assert!(e.contains("ogrid"), "{e}");
}

/// The W3c pin cell (moderator brick minus a cylinder, plus the fuel cylinder), bottom caps by
/// `ogrid`, both volumes swept; with `lattice` also the 3x3 lattice by `model.copy`.
fn pin(lattice: bool) -> String {
    let copies = if lattice {
        r#"
let fx = model.copy(entities: fu.volume, count: 2, xform: {kind: "move", delta: [1.26, 0.0, 0.0]})
let mx = model.copy(entities: mo.volumes, count: 2, xform: {kind: "move", delta: [1.26, 0.0, 0.0]})
let fy = model.copy(entities: fu.volume | fx.volumes, count: 2, xform: {kind: "move", delta: [0.0, 1.26, 0.0]})
let my = model.copy(entities: mo.volumes | mx.volumes, count: 2, xform: {kind: "move", delta: [0.0, 1.26, 0.0]})
model.imprint(bodies: all bodies)
model.merge()
"#
    } else {
        ""
    };
    format!(
        r#"
let br = model.create_brick(x: 1.26, y: 1.26, z: 1.0)
let fu = model.create_cylinder(height: 1.0, radius: 0.4)
let tl = model.create_cylinder(height: 1.0, radius: 0.4)
let mo = model.subtract(targets: br.body, tools: tl.body)
model.imprint(bodies: all bodies)
model.merge()
{copies}
mesh.size(targets: all volumes, size: 0.2)
let vs = topo.related(kind: "TopoVolume", within: all bodies)
for v in vs {{
  for f in topo.related(kind: "TopoFace", within: v) {{
    let ms = model.measure(entity: f)
    if ms.area > 0.5 and ms.area < 1.2 and ms.centroid[2] < -0.4 {{
      mesh.scheme(targets: f, scheme: "ogrid")
    }}
  }}
}}
mesh.scheme(targets: all volumes, scheme: "sweep")
let m = mesh.run(targets: all volumes)
let mi = mesh.info(mesh: m.mesh[0])
let val = mesh.validate(mesh: m.mesh[0])
let pl = mesh.plan(mesh: m.mesh[0])
for r in pl.intervals {{
  let ms = model.measure(entity: r.edge)
  if ms.length > 2.5 and ms.length < 2.52 {{
    print "circ", r.count
  }}
}}
print "cells", mi.cells
print "shapes", mi.cell_shapes
print "volume", mi.volume
print "by_owner", mi.volume_by_owner
print "violations", val.total
print "plan", pl
print "hash", session.hash()
"#
    )
}

#[test]
fn acc1_pin_square_annulus_and_disk_are_conformal_and_exact() {
    let out = run("pin", &pin(false)).unwrap();
    assert_eq!(val(&out, "violations"), 0.0, "{out}");
    // Moderator and fuel share the circle, so the total is the brick volume exactly ...
    assert!((val(&out, "volume") - 1.26 * 1.26).abs() <= 1e-12, "{}", val(&out, "volume"));
    // ... and each owner is its own exact n-gon prism (fuel 0.489834793427315 at n = 16).
    check_owners(&out, 1);
    // 5 layers x (16 + 2 x 16 fuel + 2 x 16 moderator): ogrid, not a fallback, made the annulus,
    // with two radial layers each (about one size, 0.2, thick).
    assert_eq!(val(&out, "cells"), 400.0, "{out}");
    assert!(
        text(&out, "shapes").starts_with("{Hex:") && !text(&out, "shapes").contains(','),
        "{}",
        text(&out, "shapes")
    );
    // The plan names ogrid for the fuel disk and for the square annulus.
    let plan = text(&out, "plan");
    assert!(plan.contains("reason: \"exact disk\""), "{plan}");
    assert!(plan.contains("circle in a square"), "{plan}");
}

#[test]
fn a_square_with_a_rectangular_hole_is_not_applicable() {
    let e = run(
        "rect",
        r#"
let b = model.create_brick(x: 2.0, y: 2.0, z: 1.0)
let t = model.create_brick(x: 0.5, y: 0.5, z: 1.0)
let mo = model.subtract(targets: b.body, tools: t.body)
mesh.size(targets: all volumes, size: 0.25)
for f in topo.related(kind: "TopoFace", within: mo.volumes) {
  let ms = model.measure(entity: f)
  if ms.area > 3.7 and ms.area < 3.8 and ms.centroid[2] < 0.0 {
    mesh.scheme(targets: f, scheme: "ogrid")
    let m = mesh.run(targets: f)
  }
}
"#,
    )
    .unwrap_err();
    assert!(e.contains("ogrid") && e.contains("exact circle"), "{e}");
}

#[test]
fn acc1_lattice_is_conformal_and_two_runs_agree() {
    let a = run("lat1", &pin(true)).unwrap();
    let b = run("lat2", &pin(true)).unwrap();
    assert_eq!(val(&a, "violations"), 0.0, "{a}");
    assert!((val(&a, "volume") - 9.0 * 1.26 * 1.26).abs() <= 1e-11, "{}", val(&a, "volume"));
    assert_eq!(text(&a, "hash"), text(&b, "hash"));
    check_owners(&a, 9);
    assert_eq!(val(&a, "cells"), 9.0 * 400.0, "{a}");
}

/// B5 (a 4 x 4 plate with a r = 1 through hole) under `strategy: "default"` at `size`: the scheme
/// of the annulus face, the cell count and the worst scaled Jacobian.
fn b5_default(name: &str, size: f64) -> (String, String, f64) {
    let script = format!(
        r#"
let a = model.create_brick(x: 4.0, y: 4.0, z: 1.0)
let t = model.create_cylinder(height: 3.0, radius: 1.0)
let r = model.subtract(targets: a.body, tools: t.body)
mesh.size(targets: r.volumes, size: {size})
mesh.auto(strategy: "default")
let m = mesh.run(targets: r.volumes)
let mi = mesh.info(mesh: m.mesh[0])
let pl = mesh.plan(mesh: m.mesh[0])
for e in pl.entities {{
  if e.kind == "surface" and e.scheme != "map" and e.scheme != "copy" {{
    print "face", e.scheme
  }}
}}
let q = mesh.quality(mesh: m.mesh[0], metrics: ["scaled_jacobian"], worst_n: 0)
for x in q.metrics {{
  print "sj", x.summary.min
}}
print "cells", mi.cells
"#
    );
    let out = run(name, &script).unwrap();
    (text(&out, "face"), text(&out, "cells"), val(&out, "sj"))
}

#[test]
fn the_score_follows_the_size_ogrid_where_it_is_good_pave_where_it_is_not() {
    // Fine enough: a balanced annulus with radial layers about one size thick wins ...
    let (face, cells, sj) = b5_default("b5-fine", 0.5);
    assert_eq!(face, "ogrid");
    // n = 20 circle nodes, 3 layers, 2 sweep layers.
    assert_eq!(cells, "120");
    assert!(sj > 0.6, "min scaled Jacobian {sj}");
    // ... too coarse for the circle count: the predicted edge ratio is above the limit, so pave
    // keeps the face and the mesh is not worse than before.
    let (face, _, _) = b5_default("b5-coarse", 1.0);
    assert_eq!(face, "pave");
}

/// A user count on the plate's top square edges only reaches the annulus sides through the map
/// side faces' equal-count chains: the size must not add hard rows that make it infeasible.
#[test]
fn a_user_count_elsewhere_in_the_intent_never_makes_default_infeasible() {
    let script = r#"
let a = model.create_brick(x: 4.0, y: 4.0, z: 1.0)
let t = model.create_cylinder(height: 3.0, radius: 1.0)
let r = model.subtract(targets: a.body, tools: t.body)
for e in all edges {
  let ms = model.measure(entity: e)
  if ms.length > 3.9 and ms.length < 4.1 and ms.bbox.lo[2] > 0.4 {
    mesh.intervals(targets: e, count: 8)
  }
}
mesh.size(targets: r.volumes, size: 0.5)
mesh.auto(strategy: "default")
let m = mesh.run(targets: r.volumes)
let mi = mesh.info(mesh: m.mesh[0])
print "cells", mi.cells
let q = mesh.quality(mesh: m.mesh[0], metrics: ["scaled_jacobian"], worst_n: 0)
for x in q.metrics {
  print "sj", x.summary.min
}
"#;
    let out = run("b5-top-count", script).unwrap_or_else(|e| panic!("default failed: {e}"));
    assert!(val(&out, "cells") > 0.0, "{out}");
    assert!(val(&out, "sj") > 0.5, "{out}");
}

#[test]
fn the_pin_at_the_pin_size_stays_with_the_unstructured_schemes() {
    // At size 0.3 a disk of r = 0.4 gets 8 circle nodes; the best five-block O-grid there has a
    // predicted edge ratio of 1.9, above the limit, and the moderator's is 2.4.
    let out = run(
        "pin-default",
        r#"
let br = model.create_brick(x: 1.26, y: 1.26, z: 1.0)
let fu = model.create_cylinder(height: 1.0, radius: 0.4)
let tl = model.create_cylinder(height: 1.0, radius: 0.4)
let mo = model.subtract(targets: br.body, tools: tl.body)
model.imprint(bodies: all bodies)
model.merge()
mesh.size(targets: all volumes, size: 0.3)
mesh.auto(strategy: "default")
let m = mesh.run(targets: all volumes)
let pl = mesh.plan(mesh: m.mesh[0])
for e in pl.entities {
  if e.kind == "surface" and e.scheme == "ogrid" {
    print "ogrid", e.target
  }
}
print "done", 1
"#,
    )
    .unwrap();
    assert!(!out.contains("ogrid TopoFace"), "{out}");
}

#[test]
fn pin_and_lattice_hashes_agree_for_1_and_n_threads() {
    for (name, lattice) in [("hp", false), ("hl", true)] {
        let script = pin(lattice);
        let one = hash_with(1, name, &script);
        assert_eq!(one, hash_with(4, name, &script), "{name}: 1 vs 4 threads");
    }
}

#[test]
fn tube_ring_between_two_circles() {
    let out = run(
        "tube",
        r#"
let big = model.create_cylinder(height: 1.0, radius: 0.5)
let small = model.create_cylinder(height: 1.0, radius: 0.3)
let t = model.subtract(targets: big.body, tools: small.body)
mesh.size(targets: all volumes, size: 0.12)
for f in topo.related(kind: "TopoFace", within: t.volumes) {
  let ms = model.measure(entity: f)
  if ms.area > 0.5 and ms.area < 0.51 and ms.centroid[2] < 0.0 {
    mesh.scheme(targets: f, scheme: "ogrid")
  }
}
mesh.scheme(targets: t.volumes, scheme: "sweep")
let m = mesh.run(targets: all volumes)
let mi = mesh.info(mesh: m.mesh[0])
let val = mesh.validate(mesh: m.mesh[0])
let pl = mesh.plan(mesh: m.mesh[0])
print "n", pl.intervals[0].count
print "shapes", mi.cell_shapes
print "volume", mi.volume
print "violations", val.total
print "plan", pl
"#,
    )
    .unwrap();
    let n = val(&out, "n");
    let want = ngon(n, 0.5) - ngon(n, 0.3);
    assert!((val(&out, "volume") - want).abs() <= 1e-12 * want, "{} vs {want}", val(&out, "volume"));
    assert_eq!(val(&out, "violations"), 0.0);
    assert!(text(&out, "plan").contains("reason: \"two concentric circles\""));
}
