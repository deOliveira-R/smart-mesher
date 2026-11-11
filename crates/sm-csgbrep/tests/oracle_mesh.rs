//! CSGBREP-ORACLE layers 2 and 3: deck -> B-rep -> mesh -> `.smrt` and polyhedral `.smsh`/VTU.
//!
//! * Planar deck (`disconnected`): the `.smrt` chord per material equals OpenMC's recorded chord to
//!   2e-9 and the block volumes are exact.
//! * Curved deck (`quadric_cyl`, a quadric that is a cylinder): the circle is meshed with explicit
//!   interval counts. The `.smrt` chords and block volumes stay inside OpenMC's circle and within
//!   `C h^2` of it, `h` the longest chord between the mesh's circle nodes (the tet mesher adds nodes
//!   on the cylinder, so the mesh is not the inscribed polygon prism).
//! * Polyhedral row: the all-tet mesh (`mesh.auto {strategy: "tet"}`) goes through `mesh.to_poly`;
//!   `.smsh` and VTU are written, the per-block volume of the polyhedral mesh equals the tet mesh's
//!   to 1e-12 and `mesh.validate` reports no violation. The hex mesh of POLY-MIXED is a later run.
mod common;

use std::collections::BTreeMap;

use base64::Engine as _;
use serde_json::{Value, json};
use sm_core::{EntityId, Point3, Vector3};
use sm_layer_api::{MeshLayer, ModelLayer};
use sm_mesh_api::{CellIx, MeshEntityKind, SetMembers, geometry::cell_volume};
use sm_meshio::SmshFile;
use sm_model_api::Domain;
use sm_protocol::{Endpoint, ExportFormat, ExportResult, ExportSource, Request};
use sm_rt_api::{Flow, Scratch, TraceOptions, Tracker};
use sm_server::{Server, standard_with};

fn b64(s: &str) -> String {
    base64::engine::general_purpose::STANDARD.encode(s.as_bytes())
}

fn server(geometry: &str, materials: &str) -> Server {
    let mut s = Server::new(standard_with(Some("tester".into()), Some("2026-10-24T00:00:00Z".into()))).expect("server");
    s.call(Request::Apply { op: "model.import_openmc".into(), params: json!({"geometry": {"inline": b64(geometry)}, "materials": {"inline": b64(materials)}}), version: None }).unwrap_or_else(|e| panic!("import: {e} {}", e.data));
    s
}

fn script(s: &mut Server, text: &str) -> String {
    let out = s.call(Request::ScriptRun { text: text.into(), path: None, includes: Default::default() }).unwrap_or_else(|e| panic!("script failed: {e}\n{}", e.data["rendered"].as_str().unwrap_or("")));
    out["output"].as_str().unwrap_or_default().to_owned()
}

fn export(s: &mut Server, format: ExportFormat, mesh: Option<EntityId>) -> Vec<u8> {
    let r = Request::Export { path: None, format: Some(format), mesh, provenance: false, inline: true, options: None, split_small_determinant: None, empty_patches: vec![], wedge_patches: vec![], split_warped_faces: false, source: ExportSource::Mesh };
    let res: ExportResult = serde_json::from_value(s.call(r).unwrap_or_else(|e| panic!("export {format:?}: {e} {}", e.data))).expect("ExportResult");
    base64::engine::general_purpose::STANDARD.decode(res.base64.as_deref().expect("inline data")).expect("b64")
}

fn mesh_ids(s: &Server) -> Vec<EntityId> {
    let mut v: Vec<EntityId> = s.workspace().state().layer::<MeshLayer>().expect("meshes").meshes().map(|(id, _)| id).collect();
    v.sort();
    v
}

/// Volume of every block of mesh `id` (sum of its volumes' classified cells).
fn block_volumes(s: &Server, id: EntityId) -> BTreeMap<String, f64> {
    let st = s.workspace().state();
    let model = st.layer::<ModelLayer>().expect("model").model();
    let layer = st.layer::<MeshLayer>().expect("meshes");
    let m = layer.get(id).expect("mesh").mesh();
    let mut out = BTreeMap::new();
    for c in model.colors().filter(|c| c.domain == Domain::Volume) {
        let mut sum = 0.0;
        for v in model.volumes_of_color(c.id) {
            if let Some(set) = m.classified_on(v) {
                sum += set.cells.iter().map(|&cell| cell_volume(m, cell)).sum::<f64>();
            }
        }
        out.insert(c.name.clone(), sum);
    }
    out
}

fn oracle(name: &str) -> Value {
    serde_json::from_str(&common::fixture(name, "oracle.json")).expect("oracle.json")
}

fn vec3(v: &Value) -> [f64; 3] {
    let a = v.as_array().expect("array");
    [a[0].as_f64().expect("x"), a[1].as_f64().expect("y"), a[2].as_f64().expect("z")]
}

/// Chord length per region name along a ray, to distance `max`, through any tracker.
fn chords_to(t: &dyn Tracker, names: &[String], o: [f64; 3], d: [f64; 3], max: f64) -> BTreeMap<String, f64> {
    let mut sc = Scratch::new();
    let mut st = t.init_state(Point3::new(o[0], o[1], o[2]), Vector3::new(d[0], d[1], d[2]), None, &mut sc).expect("init_state");
    let mut out: BTreeMap<String, f64> = BTreeMap::new();
    let mut sink = |s: &sm_rt_api::Segment| {
        *out.entry(names[s.region.get()].clone()).or_default() += s.length;
        Flow::Continue
    };
    t.trace(&mut st, &TraceOptions::default().with_max_dist(max), &mut sink, &mut sc).expect("trace");
    out
}

/// The `.smrt` of the current mesh, loaded back (the file a transport code opens).
fn smrt_tracker(s: &mut Server) -> (sm_rt::MeshTracker, Vec<String>) {
    let bytes = export(s, ExportFormat::Smrt, None);
    let t = sm_rt::load_image(&sm_rt::read_smrt(&bytes).expect("read_smrt")).expect("load_image");
    let names = t.regions().0.iter().map(|r| r.name.clone()).collect();
    (t, names)
}

/// OpenMC's material of each recorded segment, per ray: (origin, direction, total length, chord per material).
type RayChords = ([f64; 3], [f64; 3], f64, BTreeMap<String, f64>);

fn openmc_chords(o: &Value) -> Vec<RayChords> {
    let mat = o["cell_material"].as_object().expect("cell_material");
    o["chords"]
        .as_array()
        .expect("chords")
        .iter()
        .map(|c| {
            let mut per: BTreeMap<String, f64> = BTreeMap::new();
            for s in c["segments"].as_array().expect("segments") {
                *per.entry(mat[&s["cell"].as_u64().expect("cell").to_string()].as_str().expect("name").to_owned()).or_default() += s["t1"].as_f64().expect("t1") - s["t0"].as_f64().expect("t0");
            }
            let total = c["segments"].as_array().expect("segments").last().map_or(0.0, |s| s["t1"].as_f64().expect("t1"));
            (vec3(&c["o"]), vec3(&c["d"]), total, per)
        })
        .collect()
}

#[test]
fn a_planar_deck_has_openmcs_chords_through_the_smrt_to_2e_9() {
    let name = "disconnected";
    let mut s = server(&common::fixture(name, "geometry.xml"), &common::fixture(name, "materials.xml"));
    script(&mut s, "mesh.size(targets: all volumes, size: 0.5)\nmesh.auto(strategy: \"default\")\nlet m = mesh.run(targets: all volumes)\n");
    let ids = mesh_ids(&s);
    let vols = block_volumes(&s, ids[0]);
    // Box 3 x 2 x 1, slab |x| < 0.35 of fuel: the cell of two disconnected parts is one block.
    assert!((vols["fuel"] - 0.7 * 2.0).abs() < 1e-12, "{vols:?}");
    assert!((vols["water"] - (6.0 - 1.4)).abs() < 1e-12, "{vols:?}");
    let (t, names) = smrt_tracker(&mut s);
    let o = oracle(name);
    let rays = openmc_chords(&o);
    assert!(rays.len() >= 5);
    for (org, dir, total, want) in &rays {
        let got = chords_to(&t, &names, *org, *dir, *total);
        for k in want.keys().chain(got.keys()) {
            let (w, g) = (want.get(k).copied().unwrap_or(0.0), got.get(k).copied().unwrap_or(0.0));
            assert!((w - g).abs() < 2e-9, "ray {org:?} {dir:?}: material {k}: OpenMC {w}, `.smrt` {g}");
        }
    }
}

/// Clip the ray `o + t d`, `t` in `[0, max]`, by the convex prism (polygon `poly` counter-clockwise
/// in xy, z in `[z0, z1]`); the length inside.
fn prism_chord(poly: &[[f64; 2]], z: (f64, f64), o: [f64; 3], d: [f64; 3], max: f64) -> f64 {
    let (mut t0, mut t1) = (0.0f64, max);
    let mut clip = |num: f64, den: f64| {
        // Half space `num + t * den >= 0`.
        if den.abs() < 1e-300 {
            if num < 0.0 {
                t1 = -1.0;
            }
        } else if den > 0.0 {
            t0 = t0.max(-num / den);
        } else {
            t1 = t1.min(-num / den);
        }
    };
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
        let n = [b[1] - a[1], -(b[0] - a[0])]; // outward for a counter-clockwise polygon
        // Inside: n . (p - a) <= 0.
        clip(-(n[0] * (o[0] - a[0]) + n[1] * (o[1] - a[1])), -(n[0] * d[0] + n[1] * d[1]));
    }
    clip(o[2] - z.0, d[2]);
    clip(z.1 - o[2], -d[2]);
    (t1 - t0).max(0.0)
}

#[test]
fn a_curved_deck_stays_inside_the_circle_and_within_c_h2_of_openmc() {
    let name = "quadric_cyl";
    let (cx, cy, r, height) = (0.35f64, -0.2f64, 0.6f64, 0.2f64);
    let mut s = server(&common::fixture(name, "geometry.xml"), &common::fixture(name, "materials.xml"));
    // The circle is two arcs (1 rad and 2 pi - 1: the quadric's canonical seam and the import's seam
    // turn). Intervals are explicit: 3 on the short arc and 16 on the long one, 0.33 rad each.
    script(
        &mut s,
        r#"
mesh.size(targets: all volumes, size: 0.3)
for e in edges in all volumes {
  let me = model.measure(entity: e)
  let dx = me.bbox.hi[0] - me.bbox.lo[0]
  let dy = me.bbox.hi[1] - me.bbox.lo[1]
  let dz = me.bbox.hi[2] - me.bbox.lo[2]
  if me.length > ((dx * dx + dy * dy + dz * dz) ** 0.5) * 1.000001 {
    mesh.intervals(targets: e, count: 16)
  }
  if me.length < 1.0 and me.length >((dx * dx + dy * dy + dz * dz) ** 0.5) * 1.000001 {
    mesh.intervals(targets: e, count: 3)
  }
}
mesh.auto(strategy: "default")
let m = mesh.run(targets: all volumes)
"#,
    );
    let ids = mesh_ids(&s);
    // The circle's nodes: on the cylinder at the bottom cap, sorted by angle.
    let poly: Vec<[f64; 2]> = {
        let st = s.workspace().state();
        let m = st.layer::<MeshLayer>().expect("meshes").get(ids[0]).expect("mesh").mesh();
        let c = m.coords();
        let mut pts: Vec<(f64, [f64; 2])> = (0..c.x.len()).filter(|&i| c.z[i].abs() < 1e-12 && ((c.x[i] - cx).hypot(c.y[i] - cy) - r).abs() < 1e-9).map(|i| ((c.y[i] - cy).atan2(c.x[i] - cx), [c.x[i], c.y[i]])).collect();
        pts.sort_by(|a, b| a.0.total_cmp(&b.0));
        pts.into_iter().map(|p| p.1).collect()
    };
    assert_eq!(poly.len(), 19, "3 + 16 intervals on the circle");
    let area = 0.5 * (0..poly.len()).map(|i| poly[i][0] * poly[(i + 1) % poly.len()][1] - poly[(i + 1) % poly.len()][0] * poly[i][1]).sum::<f64>();
    let h = (0..poly.len()).map(|i| (poly[(i + 1) % poly.len()][0] - poly[i][0]).hypot(poly[(i + 1) % poly.len()][1] - poly[i][1])).fold(0.0, f64::max);
    assert!(h < 0.21 && h > 0.19, "equal intervals: longest chord {h}");

    // The tet mesher puts Steiner nodes on the cylinder between the circle's nodes, so the fuel is
    // not the polygon prism (it is neither inside nor outside it) but lies inside the circle's
    // cylinder. Block volumes: mesh <= circle, and the circle's excess is at most C h^2 with
    // C = pi / 6 (the sagitta area of an inscribed polygon, sum R^2 theta^3 / 12 <= (pi / 6) h^2,
    // chord h <= arc).
    let vols = block_volumes(&s, ids[0]);
    let circle = std::f64::consts::PI * r * r;
    let (lo, hi) = (area * height, circle * height);
    assert!(vols["fuel"] <= hi * (1.0 + 2e-9), "mesh {} <= circle {hi}", vols["fuel"]);
    assert!((vols["fuel"] + vols["water"] - 9.0 * height).abs() < 2e-9 * 9.0 * height, "{vols:?}");
    assert!((hi - vols["fuel"]) <= 0.55 * h * h * height, "circle excess {} > C h^2 {}", hi - vols["fuel"], 0.55 * h * h * height);
    assert!((hi - vols["fuel"]) > 1e-4 * hi, "mesh and circle must differ for the bound to mean something");
    eprintln!("quadric_cyl: polygon prism {lo:.6}, mesh {:.6}, circle {hi:.6}", vols["fuel"]);

    // Chords through the `.smrt`: not longer than OpenMC's circle chord, and OpenMC's within
    // C h^2 (C = 1 / R: a ray crossing at incidence cosine >= 1/2 moves each end by at most
    // sagitta / cos = h^2 / (4 R)).
    let (t, names) = smrt_tracker(&mut s);
    let o = oracle(name);
    let mut worst: f64 = 0.0;
    let mut any_differs = false;
    for (org, dir, total, openmc) in openmc_chords(&o) {
        let inscribed = prism_chord(&poly, (0.0, height), org, dir, total);
        assert!(inscribed <= openmc.get("fuel").copied().unwrap_or(0.0) + 2e-9, "the inscribed polygon is inside the circle");
        let got = chords_to(&t, &names, org, dir, total);
        let (gf, gw) = (got.get("fuel").copied().unwrap_or(0.0), got.get("water").copied().unwrap_or(0.0));
        let of = openmc.get("fuel").copied().unwrap_or(0.0);
        assert!(gf <= of + 2e-9, "ray {org:?} {dir:?}: `.smrt` {gf} <= OpenMC {of} (polygon {inscribed})");
        assert!((gf + gw - total).abs() < 2e-9, "ray {org:?} {dir:?}: chords {gf} + {gw} != {total}");
        worst = worst.max((of - gf).abs());
        any_differs |= (of - gf).abs() > 1e-6;
        assert!((of - gf).abs() <= h * h / r, "ray {org:?} {dir:?}: OpenMC fuel chord {of}, mesh {gf}, bound {}", h * h / r);
    }
    assert!(any_differs, "the circle and the mesh must differ somewhere, or the bound is vacuous");
    eprintln!("quadric_cyl: h {h:.4}, h^2/R {:.5}, worst chord difference to OpenMC {worst:.3e}", h * h / r);
}

/// Mesh `geometry` all-tet at `size`, dualize it and check the polyhedral export: `.smsh` and VTU
/// bytes, per-block volume of the polyhedral mesh equal to the tet mesh's to 1e-12, no violation.
fn poly_round_trip(geometry: &str, materials: &str, size: f64) -> (usize, usize) {
    let mut s = server(geometry, materials);
    let out = script(
        &mut s,
        &format!(
            r#"
mesh.size(targets: all volumes, size: {size})
mesh.auto(strategy: "tet")
let m = mesh.run(targets: all volumes)
let p = mesh.to_poly(mesh: m.mesh[0])
let v = mesh.validate(mesh: p.mesh[0])
print "violations", v.total
assert v.valid and v.total == 0, "the polyhedral mesh validates"
"#
        ),
    );
    assert!(out.contains("violations 0"), "{out}");
    let ids = mesh_ids(&s);
    assert_eq!(ids.len(), 2, "the tet mesh and its polyhedral dual");
    let (tet, poly) = (ids[0], ids[1]);
    let (vt, vp) = (block_volumes(&s, tet), block_volumes(&s, poly));
    assert_eq!(vt.keys().collect::<Vec<_>>(), vp.keys().collect::<Vec<_>>());
    for (k, a) in &vt {
        assert!((a - vp[k]).abs() <= 1e-12 * a.max(1.0), "block {k}: tets {a}, polyhedra {}", vp[k]);
    }
    let (ncells_tet, ncells_poly) = {
        let st = s.workspace().state();
        let l = st.layer::<MeshLayer>().expect("meshes");
        (l.get(tet).expect("tet").mesh().count(MeshEntityKind::Cell), l.get(poly).expect("poly").mesh().count(MeshEntityKind::Cell))
    };
    assert!(ncells_poly < ncells_tet, "the dual has fewer, larger cells");
    // `.smsh`: block cell sets decode and their cells' volumes (on the polyhedral mesh) sum to the block volume.
    let smsh = export(&mut s, ExportFormat::Smsh, Some(poly));
    let decoded = SmshFile::parse(&smsh).expect("parse").decode().expect("decode");
    assert_eq!(decoded.cell_ids.len(), ncells_poly);
    let st = s.workspace().state();
    let pm = st.layer::<MeshLayer>().expect("meshes").get(poly).expect("poly").mesh();
    let mut by_set: BTreeMap<String, f64> = BTreeMap::new();
    for set in &decoded.sets {
        if let SetMembers::Cells(c) = &set.members {
            by_set.insert(set.name.clone(), c.iter().map(|&cell| cell_volume(pm, CellIx(cell.0))).sum());
        }
    }
    for (k, a) in &vt {
        assert!((by_set[k] - a).abs() <= 1e-12 * a.max(1.0), "smsh set {k}: {} vs {a}", by_set[k]);
    }
    // VTU: an XML unstructured grid with one cell per polyhedron.
    let vtu = String::from_utf8_lossy(&export(&mut s, ExportFormat::Vtu, Some(poly))).into_owned();
    assert!(vtu.contains("<VTKFile") && vtu.contains(&format!("NumberOfCells=\"{ncells_poly}\"")), "VTU header: {}", &vtu[..vtu.len().min(300)]);
    (ncells_tet, ncells_poly)
}

#[test]
fn a_pin_and_a_lattice_go_through_to_poly_to_smsh_and_vtu() {
    // The committed `pin` fixture has a 0.02 gap and needs a 600k-tet mesh (the ignored test below);
    // the same deck with a wide gap meshes at 0.15. A 2 x 2 lattice of it covers the lattice path
    // (the `lattice2d` fixture itself does not mesh yet: its outer ring fails in `trimesh`).
    let mats = common::materials(4);
    let (a, b) = poly_round_trip(&common::decks::pin_lattice(1, 1.26, (0.3, 0.4, 0.5), 1.0, false), &mats, 0.15);
    eprintln!("pin: {a} tets -> {b} polyhedra");
    let (a, b) = poly_round_trip(&common::decks::pin_lattice(2, 1.26, (0.3, 0.4, 0.5), 1.0, false), &mats, 0.15);
    eprintln!("2x2 lattice: {a} tets -> {b} polyhedra");
}

#[test]
#[ignore = "about 130 s in debug (600k tets); run with --ignored"]
fn the_pin_fixture_goes_through_to_poly_at_gap_resolution() {
    let (a, b) = poly_round_trip(&common::fixture("pin", "geometry.xml"), &common::fixture("pin", "materials.xml"), 0.04);
    eprintln!("pin fixture: {a} tets -> {b} polyhedra");
}

/// The `.smrt`, `.smsh` and VTU bytes of a pin and of its polyhedral dual are the same on every run
/// and with one rayon thread as with many.
#[test]
fn the_exports_are_the_same_with_one_thread_and_with_many() {
    fn bytes() -> Vec<Vec<u8>> {
        let mut s = server(&common::decks::pin_lattice(1, 1.26, (0.3, 0.4, 0.5), 1.0, false), &common::materials(4));
        script(&mut s, "mesh.size(targets: all volumes, size: 0.15)\nmesh.auto(strategy: \"tet\")\nlet m = mesh.run(targets: all volumes)\nlet p = mesh.to_poly(mesh: m.mesh[0])\n");
        let ids = mesh_ids(&s);
        vec![export(&mut s, ExportFormat::Smrt, Some(ids[0])), export(&mut s, ExportFormat::Smsh, Some(ids[1])), export(&mut s, ExportFormat::Vtu, Some(ids[1]))]
    }
    let first = bytes();
    assert_eq!(first, bytes());
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool").install(bytes);
    let many = rayon::ThreadPoolBuilder::new().num_threads(4).build().expect("pool").install(bytes);
    assert_eq!(first, one);
    assert_eq!(first, many);
}
