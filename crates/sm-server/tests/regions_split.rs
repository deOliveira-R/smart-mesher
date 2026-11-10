//! W4-REGIONS-SPLIT (ruling D9): the `regions: {split: [...]}` export option of `smsh` and `smrt`
//! on a 3 x 3 pin lattice whose fuel is cut into 3 rings x 4 sectors (117 regions), the no-split
//! byte identity, option errors and determinism. Everything is exported inline (no files).
//!
//! The pin script rotates the fuel cylinder and its tool by one radian and starts the sectors
//! there: the cutting cylinders of `model.subdivide_pin` have their seam at one radian (its
//! `SEAM_TURN`), and a seam vertex inside a sector arc would force a non-uniform node spacing, so
//! the arcs would not be `2 pi / 16` and the closed form below would not hold.
mod common;

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::PI;

use base64::Engine as _;
use serde_json::{Value, json};
use sm_core::Point3;
use sm_layer_api::{MeshLayer, ModelLayer};
use sm_mesh_api::geometry::cell_volume;
use sm_meshio::{SmshFile, SmshMesh};
use sm_protocol::{Endpoint, ExportFormat, ExportResult, ExportSource, Request, RpcError};
use sm_rt_api::{Scratch, Tracker};
use sm_server::{Server, standard_with};
use sm_testkit::surf3::SplitMix64;

const TIME: &str = "2026-10-24T00:00:00Z";
const PITCH: f64 = 1.26;
const R: f64 = 0.4096;
const N: f64 = 16.0;
const S: f64 = 4.0;

/// The 3x3 subdivided lattice of the draft (acceptance 3), up to the mesh.
const LATTICE: &str = r#"
let br = model.create_brick(x: 1.26, y: 1.26, z: 1.0)
let fu = model.create_cylinder(height: 1.0, radius: 0.4096)
let tl = model.create_cylinder(height: 1.0, radius: 0.4096)
model.rotate(entities: fu.body | tl.body, angle_deg: 57.29577951308232, axis: [0.0, 0.0, 1.0])
let mo = model.subtract(targets: br.body, tools: tl.body)
model.imprint(bodies: all bodies)
model.merge()
model.lattice_define(name: "a", layout: {kind: "cartesian", n: [3, 3], pitch: [1.26, 1.26, 1.0]})
let fx = model.copy(entities: fu.volume, count: 2, xform: {kind: "move", delta: [1.26, 0.0, 0.0]}, lattice: "a")
let mx = model.copy(entities: mo.volumes, count: 2, xform: {kind: "move", delta: [1.26, 0.0, 0.0]}, lattice: "a")
let fy = model.copy(entities: fu.volume | fx.volumes, count: 2, xform: {kind: "move", delta: [0.0, 1.26, 0.0]}, lattice: "a")
let my = model.copy(entities: mo.volumes | mx.volumes, count: 2, xform: {kind: "move", delta: [0.0, 1.26, 0.0]}, lattice: "a")
model.imprint(bodies: all bodies)
model.merge()
let mods = mo.volumes | mx.volumes | my.volumes
let fuels = (all volumes) - mods
for v in fuels {
  let c = model.measure(entity: v).centroid
  model.subdivide_pin(volumes: v, axis: [0.0, 0.0, 1.0], center: [c[0], c[1], 0.0], rings: {count: 3, mode: "equal_area"}, sectors: {count: 4, angle0_deg: 57.29577951308232})
}
let fp = (all volumes) - mods
assert count(fp) == 108 and count(mods) == 9, "9 pins x 12 pieces and 9 moderator cells"
fem.block_add(1, volumes: fp, name: "fuel")
fem.block_add(2, volumes: mods, name: "water")
mesh.size(targets: all volumes, size: 0.3)
// n = 16 on every circle: each of its four arcs (the edges longer than their bounding-box diagonal)
for e in edges in fp {
  let me = model.measure(entity: e)
  let dx = me.bbox.hi[0] - me.bbox.lo[0]
  let dy = me.bbox.hi[1] - me.bbox.lo[1]
  let dz = me.bbox.hi[2] - me.bbox.lo[2]
  if me.length > ((dx * dx + dy * dy + dz * dz) ** 0.5) * 1.000001 {
    mesh.intervals(targets: e, count: 4)
  }
}
mesh.auto(strategy: "default")
let m = mesh.run(targets: all volumes)
"#;

/// Two bricks with blocks `fuel` and `water` and no tags at all.
const TWO_BRICKS: &str = r#"
let a = model.create_brick(x: 2.0, y: 2.0, z: 2.0, center: [1.0, 1.0, 1.0])
let b = model.create_brick(x: 2.0, y: 2.0, z: 2.0, center: [3.0, 1.0, 1.0])
model.imprint(bodies: a.body | b.body)
model.merge(entities: a.body | b.body)
fem.block_add(block: 1, volumes: a.volume, name: "fuel")
fem.block_add(block: 2, volumes: b.volume, name: "water")
mesh.size(targets: a.volume | b.volume, size: 1.0)
mesh.scheme(targets: a.volume | b.volume, scheme: "map")
let m = mesh.run(targets: a.volume | b.volume)
"#;

fn server(script: &str) -> Server {
    let mut s = Server::new(standard_with(Some("tester".into()), Some(TIME.into()))).expect("server");
    s.call(Request::ScriptRun { text: script.into(), path: None, includes: Default::default() }).unwrap_or_else(|e| panic!("fixture script: {e} {}", e.data));
    s
}

fn export(s: &mut Server, format: ExportFormat, options: Option<Value>) -> Result<Vec<u8>, RpcError> {
    export_hashed(s, format, options).map(|x| x.0)
}

/// The bytes and the SHA-256 the server reports.
fn export_hashed(s: &mut Server, format: ExportFormat, options: Option<Value>) -> Result<(Vec<u8>, String), RpcError> {
    let r = Request::Export {
        path: None,
        format: Some(format),
        mesh: None,
        provenance: false,
        inline: true,
        options,
        split_small_determinant: None,
        empty_patches: vec![],
        wedge_patches: vec![],
        split_warped_faces: false,
        source: ExportSource::Mesh,
    };
    let res: ExportResult = serde_json::from_value(s.call(r)?).expect("ExportResult");
    Ok((base64::engine::general_purpose::STANDARD.decode(res.base64.as_deref().expect("inline data")).expect("b64"), res.sha256.to_hex()))
}

/// The regions of a `.smrt` image, the implicit complement left out.
fn smrt_regions(bytes: &[u8]) -> Vec<sm_rt_api::RegionInfo> {
    let loaded = sm_rt::load_image(&sm_rt::read_smrt(bytes).expect("read_smrt")).expect("load_image");
    loaded.regions().0.iter().filter(|r| !r.implicit_complement).cloned().collect()
}

fn all_split() -> Option<Value> {
    Some(json!({"regions": {"split": ["lattice", "ring", "sector"]}}))
}

/// The expected region name and the volume's tag fields from the model's region tags.
fn expected_names(s: &Server) -> BTreeMap<String, sm_core::EntityId> {
    let model = s.workspace().state().layer::<ModelLayer>().expect("model");
    let mut out = BTreeMap::new();
    for (v, t) in model.model().region_tags() {
        let slot = t.slots["a"];
        let name = match (t.ring, t.sector) {
            (Some(r), Some(sec)) => format!("fuel@a[{},{}]/r{}/s{}", slot.i, slot.j, r.index, sec.index),
            (None, None) => format!("water@a[{},{}]", slot.i, slot.j),
            other => panic!("unexpected tag {other:?}"),
        };
        assert!(out.insert(name, *v).is_none());
    }
    out
}

fn smsh_of(bytes: &[u8]) -> SmshMesh {
    SmshFile::parse(bytes).expect("parse").decode().expect("decode")
}

fn region_sets(m: &SmshMesh) -> BTreeMap<String, Vec<u32>> {
    let mut out = BTreeMap::new();
    for s in &m.sets {
        if let Some(n) = s.name.strip_prefix("region/") {
            let sm_mesh_api::SetMembers::Cells(c) = &s.members else { panic!("{} is not a cell set", s.name) };
            out.insert(n.to_string(), c.iter().map(|c| c.0).collect());
        }
    }
    out
}

/// Inscribed-polygon area of the sector of radius `r` (`n / S` chords of the circle of `n`).
fn poly_sector(r: f64) -> f64 {
    (N / S) * 0.5 * r * r * (2.0 * PI / N).sin()
}

#[test]
fn the_3x3_lattice_splits_into_117_regions_in_smsh_and_smrt() {
    let mut s = server(LATTICE);
    let expected = expected_names(&s);
    let want: BTreeSet<String> = expected.keys().cloned().collect();
    assert_eq!(want.len(), 117, "9 x 12 + 9");
    for i in 0..3 {
        for j in 0..3 {
            assert!(want.contains(&format!("water@a[{i},{j}]")));
            for r in 0..3 {
                for sec in 0..4 {
                    assert!(want.contains(&format!("fuel@a[{i},{j}]/r{r}/s{sec}")), "{i} {j} {r} {sec}");
                }
            }
        }
    }

    let smsh = export(&mut s, ExportFormat::Smsh, all_split()).expect("smsh split");
    let mesh = smsh_of(&smsh);
    let sets = region_sets(&mesh);
    assert_eq!(sets.keys().cloned().collect::<BTreeSet<_>>(), want, "the sorted name list is the generated one");
    let blocks: Vec<&str> = mesh.sets.iter().filter(|x| !x.name.starts_with("region/")).map(|x| x.name.as_str()).collect();
    assert_eq!(blocks, ["fuel", "water"], "the block sets stay");

    // Every region set is exactly the cells classified on its volume, and its mesh volume is the
    // closed form of the inscribed polygon (rings r_k = R sqrt(k / 3), n = 16, S = 4).
    let st = s.workspace().state();
    let layer = st.layer::<MeshLayer>().expect("meshes");
    let (_, entry) = layer.meshes().next().expect("one mesh");
    let m = entry.mesh();
    let ring_r = |k: usize| R * (k as f64 / 3.0).sqrt();
    let mut covered = 0usize;
    for (name, vol) in &expected {
        let mut cells: Vec<u32> = m.classified_on(*vol).expect("classified").cells.iter().map(|c| c.0).collect();
        cells.sort_unstable();
        assert_eq!(sets[name], cells, "{name}");
        covered += cells.len();
        let got: f64 = cells.iter().map(|&c| cell_volume(m, sm_mesh_api::CellIx(c))).sum();
        let want = if let Some(rest) = name.strip_prefix("fuel@") {
            let r: usize = rest.split("/r").nth(1).and_then(|x| x[..1].parse().ok()).expect("ring");
            (poly_sector(ring_r(r + 1)) - if r == 0 { 0.0 } else { poly_sector(ring_r(r)) }) * 1.0
        } else {
            PITCH * PITCH - 4.0 * poly_sector(R)
        };
        assert!(((got - want) / want).abs() < 1e-12, "{name}: mesh volume {got} vs closed form {want}");
    }
    assert_eq!(covered, m.cell_count(), "the regions partition the cells");

    // `.smrt`: 117 tagged regions; the tags of `fuel@a[2,1]/r0/s3` are exactly the draft's.
    let smrt = export(&mut s, ExportFormat::Smrt, all_split()).expect("smrt split");
    let image = sm_rt::read_smrt(&smrt).expect("read_smrt");
    let regions = smrt_regions(&smrt);
    assert_eq!(regions.iter().map(|r| r.name.clone()).collect::<BTreeSet<_>>(), want);
    let r = regions.iter().find(|r| r.name == "fuel@a[2,1]/r0/s3").expect("region");
    let tags = [("lat.a.i", 2), ("lat.a.j", 1), ("ring", 0), ("ring_count", 3), ("sector", 3), ("sector_count", 4)].map(|(k, v)| (k.to_string(), v));
    assert_eq!(r.tags, tags);
    let w = regions.iter().find(|r| r.name == "water@a[0,2]").expect("water");
    assert_eq!(w.tags, [("lat.a.i".to_string(), 0), ("lat.a.j".to_string(), 2)]);

    // 600 seeded points: the `.smrt` locate gives the region the test computes from the slot, the
    // ring (radius) and the sector (angle), away from circles, planes and element borders.
    let loaded = sm_rt::load_image(&image).expect("load_image");
    let names: Vec<String> = loaded.regions().0.iter().map(|r| r.name.clone()).collect();
    let mut rng = SplitMix64(0x2e61_0517);
    let mut scratch = Scratch::new();
    let (mut checked, mut fuel_hits) = (0, 0);
    while checked < 600 {
        let u = |lo: f64, hi: f64, rng: &mut SplitMix64| lo + (hi - lo) * (rng.unit() + 1.0) / 2.0;
        let p = Point3::new(u(-0.63, 3.15, &mut rng), u(-0.63, 3.15, &mut rng), u(-0.45, 0.45, &mut rng));
        let (i, j) = ((p.x / PITCH).round(), (p.y / PITCH).round());
        let (dx, dy) = (p.x - i * PITCH, p.y - j * PITCH);
        let r = dx.hypot(dy);
        let near_border = (dx.abs() - PITCH / 2.0).abs() < 1e-9 || (dy.abs() - PITCH / 2.0).abs() < 1e-9;
        let circles = [ring_r(1), ring_r(2), R];
        let near_circle = circles.iter().any(|c| (r - c).abs() < c * (1.0 - (PI / N).cos()) + 1e-9);
        let theta = (dy.atan2(dx) - 1.0).rem_euclid(2.0 * PI);
        let near_plane = (0..4).any(|k| r * (theta - k as f64 * PI / 2.0).sin().abs() < 1e-9) || r * (theta - 2.0 * PI).sin().abs() < 1e-9;
        if near_border || near_circle || near_plane {
            continue;
        }
        let want = if r < R {
            fuel_hits += 1;
            let ring = circles.iter().position(|c| r < *c).expect("ring");
            let sector = (theta / (PI / 2.0)).floor() as usize;
            format!("fuel@a[{},{}]/r{ring}/s{sector}", i as i32, j as i32)
        } else {
            format!("water@a[{},{}]", i as i32, j as i32)
        };
        let got = loaded.locate(p, None, None, &mut scratch).expect("locate");
        assert_eq!(names[got.region.get()], want, "point {p:?}");
        checked += 1;
    }
    assert!(fuel_hits > 100, "the points reach the fuel ({fuel_hits})");

    // Fewer parts, fewer regions.
    let mut count = |parts: Value| {
        let b = export(&mut s, ExportFormat::Smrt, Some(json!({"regions": {"split": parts}}))).expect("smrt");
        smrt_regions(&b).into_iter().map(|r| r.name).collect::<Vec<_>>()
    };
    let lat = count(json!(["lattice"]));
    assert_eq!(lat.len(), 18);
    assert!(lat.contains(&"fuel@a[1,1]".to_string()) && lat.contains(&"water@a[2,2]".to_string()));
    assert_eq!(count(json!(["ring"])), ["fuel/r0", "fuel/r1", "fuel/r2", "water"]);
    let sector_only = count(json!(["sector"]));
    assert_eq!(sector_only.len(), 5, "{sector_only:?}");
    let smsh_lat = region_sets(&smsh_of(&export(&mut s, ExportFormat::Smsh, Some(json!({"regions": {"split": ["lattice"]}}))).expect("smsh")));
    assert_eq!(smsh_lat.len(), 18);

    // An empty split and the absent option are the plain export, byte for byte; so are exports
    // after the tags are cleared (the tags never leak into a no-split file).
    for format in [ExportFormat::Smsh, ExportFormat::Smrt] {
        let plain = export(&mut s, format, None).expect("plain");
        for o in [json!({}), json!({"regions": {}}), json!({"regions": {"split": []}}), json!({"regions": null})] {
            assert_eq!(export(&mut s, format, Some(o.clone())).expect("empty split"), plain, "{format:?} {o}");
        }
    }
    let (plain_smsh, plain_smrt) = (export(&mut s, ExportFormat::Smsh, None).expect("smsh"), export(&mut s, ExportFormat::Smrt, None).expect("smrt"));
    assert!(region_sets(&smsh_of(&plain_smsh)).is_empty());
    s.call(Request::Apply { op: "model.region_clear".into(), params: json!({}), version: None }).expect("region_clear");
    assert_eq!(export(&mut s, ExportFormat::Smsh, None).expect("smsh"), plain_smsh);
    assert_eq!(export(&mut s, ExportFormat::Smrt, None).expect("smrt"), plain_smrt);
    // With no tags a split is by block only.
    let untagged = region_sets(&smsh_of(&export(&mut s, ExportFormat::Smsh, all_split()).expect("smsh")));
    assert_eq!(untagged.keys().map(String::as_str).collect::<Vec<_>>(), ["fuel", "water"]);
}

#[test]
fn smsh_and_smrt_with_a_split_are_deterministic_across_runs_and_thread_counts() {
    // Two exports of one session, then a session built and exported on 1 thread.
    let go = || {
        let mut s = server(LATTICE);
        let first = (export(&mut s, ExportFormat::Smsh, all_split()).expect("smsh"), export(&mut s, ExportFormat::Smrt, all_split()).expect("smrt"));
        let again = (export(&mut s, ExportFormat::Smsh, all_split()).expect("smsh"), export(&mut s, ExportFormat::Smrt, all_split()).expect("smrt"));
        assert_eq!(first, again, "two exports");
        first
    };
    let a = go();
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool").install(go);
    assert_eq!(a, one, "1 thread vs default");
}

#[test]
fn split_errors_are_typed_and_the_options_are_checked() {
    let mut s = server(TWO_BRICKS);
    let ring = Some(json!({"regions": {"split": ["ring"]}}));
    // Untagged volumes: regions are the blocks.
    let regions = smrt_regions(&export(&mut s, ExportFormat::Smrt, ring.clone()).expect("smrt"));
    assert_eq!(regions.iter().map(|r| r.name.as_str()).collect::<Vec<_>>(), ["fuel", "water"]);
    assert!(regions.iter().all(|r| r.tags.is_empty()));

    let kind = |r: Result<Vec<u8>, RpcError>| r.expect_err("an error").kind;
    for format in [ExportFormat::Smsh, ExportFormat::Smrt] {
        // Bad options.
        assert_eq!(kind(export(&mut s, format, Some(json!({"regions": {"split": ["bogus"]}})))), "bad-request");
        assert_eq!(kind(export(&mut s, format, Some(json!({"regions": {"split": "ring"}})))), "bad-request");
        assert_eq!(kind(export(&mut s, format, Some(json!({"regions": {"by": []}})))), "bad-request");
        assert_eq!(kind(export(&mut s, format, Some(json!({"regions": 3})))), "bad-request");
        assert_eq!(kind(export(&mut s, format, Some(json!({"split": ["ring"]})))), "bad-request");
    }
    // The option belongs to smsh and smrt; the other formats still refuse it.
    assert_eq!(kind(export(&mut s, ExportFormat::Vtu, ring.clone())), "bad-request");
    assert_eq!(kind(export(&mut s, ExportFormat::Gmsh, ring.clone())), "bad-request");
    assert_eq!(kind(export(&mut s, ExportFormat::Dagmc, ring.clone())), "bad-request");

    // `source: model` (DAGMC, no mesh) takes no `regions` either, and so cannot be asked to split
    // an instanced model: the instance refusal is `unsupported-instances` in both paths.
    let r = Request::Export {
        path: None,
        format: Some(ExportFormat::Dagmc),
        mesh: None,
        provenance: false,
        inline: true,
        options: ring.clone(),
        split_small_determinant: None,
        empty_patches: vec![],
        wedge_patches: vec![],
        split_warped_faces: false,
        source: ExportSource::Model,
    };
    assert_eq!(s.call(r).expect_err("regions on a model export").kind, "bad-request");

    // A block name the grammar reserves, and blocks that do not cover the mesh.
    let mut s = server(&TWO_BRICKS.replace("name: \"fuel\"", "name: \"u@235\""));
    for format in [ExportFormat::Smsh, ExportFormat::Smrt] {
        assert_eq!(kind(export(&mut s, format, ring.clone())), "region-name-reserved");
    }
    assert!(export(&mut s, ExportFormat::Smsh, None).is_ok(), "no split, no restriction on block names");
    let mut s = server(&TWO_BRICKS.lines().filter(|l| !l.contains("block: 2")).collect::<Vec<_>>().join("\n"));
    for format in [ExportFormat::Smsh, ExportFormat::Smrt] {
        assert_eq!(kind(export(&mut s, format, ring.clone())), "regions-need-blocks");
    }
    let mut s = server(&TWO_BRICKS.lines().filter(|l| !l.contains("block_add")).collect::<Vec<_>>().join("\n"));
    assert_eq!(kind(export(&mut s, ExportFormat::Smsh, ring.clone())), "regions-need-blocks");
}

/// The W3c pin lattice (`tools/w3c_pin.smesh`): the no-split `.smsh` and `.smrt` bytes are those of
/// the build before this task. The hashes are of this platform (the mesher's float results are
/// pinned per os-arch by the milestones; INT pins the other platforms).
#[test]
fn the_w3c_pin_exports_are_byte_identical_to_the_build_before_the_split() {
    let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../tools/w3c_pin.smesh")).expect("script");
    let script: String = text.lines().filter(|l| !l.trim_start().starts_with("session.export")).collect::<Vec<_>>().join("\n");
    let mut s = server(&script);
    let ((smsh, hs), (smrt, hr)) = (export_hashed(&mut s, ExportFormat::Smsh, None).expect("smsh"), export_hashed(&mut s, ExportFormat::Smrt, None).expect("smrt"));
    eprintln!("w3c pin smsh sha256 {hs}\nw3c pin smrt sha256 {hr}");
    if cfg!(all(target_os = "macos", target_arch = "aarch64")) {
        assert_eq!(hs, "760e9838ee1f860644c6212a8663ff90a100fdc8f38ecb94590415edd7bb4e9a", "pin.smsh");
        assert_eq!(hr, "64cfff0283a7a3c1e11aa35fd034511e642be5dba89d3c10172c09df2de0bccb", "pin.smrt");
    }
    // An empty split is the same bytes.
    let none = Some(json!({"regions": {"split": []}}));
    assert_eq!(export(&mut s, ExportFormat::Smsh, none.clone()).expect("smsh"), smsh);
    assert_eq!(export(&mut s, ExportFormat::Smrt, none).expect("smrt"), smrt);
}
