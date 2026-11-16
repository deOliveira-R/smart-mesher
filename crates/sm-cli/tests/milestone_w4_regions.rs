//! W4 regions milestone (plan `docs/design/w4/plan.md` section 5 item 3; ruling D9; ruling D4;
//! `drafts/regions.md` W4-REGIONS-SPLIT acceptance 3).
//!
//! `tools/w4_regions.smesh` builds the 3x3 pin lattice (`model.copy ... lattice:`), cuts every
//! fuel into 3 equal-area rings x 4 sectors, meshes it with `hex_only` (n = 16 on every circle)
//! and runs VOLFIT on every cylinder face. This test exports that session with
//! `regions: {split: ["lattice", "ring", "sector"]}` to `.smsh` and `.smrt` and checks:
//!
//! | row | what |
//! |---|---|
//! | G1 | 117 regions; names equal the grammar's list; the `.smsh` `region/` sets partition the cells; every cell lies, by its centroid, in the analytic slot / ring / sector its set names |
//! | G2 | per-region volume after VOLFIT: fuel `pi r^2 h / 12`, water `(1.26^2 - pi r^2) h`, to 1e-12 relative |
//! | G3 | the `.smrt` tags read back through the C ABI (`smrt_region_n_tags`, `smrt_region_tag`) in a C driver, and 36 `smrt_locate` calls land in the region the test computes |
//! | G4 | the OpenMC 3x3 deck through `model.import_openmc` gives the same region names and slot tags as the model path under `split: ["lattice"]` |
//! | G5 | CLI (`--regions`, project run), in-process and 1-thread runs give identical bytes; two runs equal |
//!
//! The hashes of the exports are printed (`w4 regions ... sha256`) for INT to pin per os-arch; this
//! file asserts the macOS arm64 values.
#![allow(missing_docs)]

mod w3a_support;

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::PI;
use std::path::{Path, PathBuf};
use std::process::Command;

use serde_json::{Value, json};
use sm_layer_api::MeshLayer;
use sm_mesh_api::{CellIx, MeshRead, cell_volume};
use sm_meshio::{SmshFile, SmshMesh};
use sm_rt_api::Tracker;
use sm_protocol::{Endpoint, ExportFormat, ExportResult, ExportSource, Request};
use sm_server::{Server, standard_with};

use w3a_support::*;

const PITCH: f64 = 1.26;
const R: f64 = 0.4096;
/// The sectors start at one radian (see the script header).
const ANGLE0: f64 = 1.0;
const FRAME: &str = "lat20";

const MAC_SMSH: &str = "9353bb26406f74b184894f10aa57c2103a8bbf6ecdbae81111d2eb51cbbc2839";
const MAC_SMRT: &str = "efef863b96a3e9099fcd6beca3ca19b4a61a79e8c4ee0aa8c969c361cb4192ad";

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().expect("repo root")
}

fn script_text() -> String {
    read_text(&repo().join("tools/w4_regions.smesh"))
}

/// One scratch tree per test process and test thread, removed when the test ends.
struct Scratch(PathBuf);

impl Scratch {
    fn new() -> Self {
        let t = std::thread::current().name().unwrap_or("t").replace("::", "_");
        let d = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("milestone_w4_regions-{}", std::process::id())).join(t);
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).expect("scratch dir");
        Scratch(d)
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
        if let Some(p) = self.0.parent() {
            let _ = std::fs::remove_dir(p);
        }
    }
}

fn ts(p: &Path) -> String {
    p.to_string_lossy().into_owned()
}

fn server() -> Server {
    Server::new(standard_with(Some(AUTHOR.into()), Some(TIME.into()))).expect("server")
}

fn run_script(s: &mut Server, text: &str) -> String {
    let v = s
        .call(Request::ScriptRun { text: text.to_owned(), path: None, includes: Default::default() })
        .unwrap_or_else(|x| panic!("script.run: {x} {}", x.data["rendered"].as_str().unwrap_or("")));
    v["output"].as_str().expect("script output").to_owned()
}

fn prints(out: &str) -> BTreeMap<String, String> {
    printed(out)
}

fn all_split() -> Value {
    json!({"regions": {"split": ["lattice", "ring", "sector"]}})
}

fn export(s: &mut Server, format: ExportFormat, options: Option<Value>) -> (Vec<u8>, String) {
    let v = s
        .call(Request::Export {
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
        })
        .unwrap_or_else(|x| panic!("export {format:?}: {x} {}", x.data));
    let r: ExportResult = serde_json::from_value(v).expect("ExportResult");
    let b64 = r.base64.as_deref().expect("inline data");
    (unbase64(b64), r.sha256.to_hex())
}

/// Minimal standard-alphabet base64 decoder (the dev-dependencies carry no base64 crate).
fn unbase64(s: &str) -> Vec<u8> {
    let val = |c: u8| match c {
        b'A'..=b'Z' => c - b'A',
        b'a'..=b'z' => c - b'a' + 26,
        b'0'..=b'9' => c - b'0' + 52,
        b'+' => 62,
        b'/' => 63,
        _ => 64,
    };
    let mut out = Vec::with_capacity(s.len() / 4 * 3);
    let (mut acc, mut bits) = (0u32, 0);
    for c in s.bytes().filter(|&c| c != b'=' && c != b'\n') {
        let v = val(c);
        assert!(v < 64, "bad base64 byte {c}");
        acc = (acc << 6) | u32::from(v);
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            out.push((acc >> bits) as u8);
            acc &= (1 << bits) - 1;
        }
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

/// The 117 names the grammar gives for the 3x3 subdivided lattice.
fn expected_names() -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    for i in 0..3 {
        for j in 0..3 {
            out.insert(format!("water@{FRAME}[{i},{j}]"));
            for r in 0..3 {
                for s in 0..4 {
                    out.insert(format!("fuel@{FRAME}[{i},{j}]/r{r}/s{s}"));
                }
            }
        }
    }
    out
}

/// What a region name says: slot `(i, j)`, and for fuel the ring and the sector.
fn parse_name(name: &str) -> (usize, usize, Option<(usize, usize)>) {
    let (mat, rest) = name.split_once('@').expect("@");
    let rest = rest.strip_prefix(FRAME).expect("frame").strip_prefix('[').expect("[");
    let (ij, tail) = rest.split_once(']').expect("]");
    let (i, j) = ij.split_once(',').expect(",");
    let sub = if mat == "fuel" {
        let t = tail.strip_prefix("/r").expect("/r");
        let (r, s) = t.split_once("/s").expect("/s");
        Some((r.parse().expect("ring"), s.parse().expect("sector")))
    } else {
        assert!(tail.is_empty(), "{name}");
        None
    };
    (i.parse().expect("i"), j.parse().expect("j"), sub)
}

fn ring_r(k: usize) -> f64 {
    R * (k as f64 / 3.0).sqrt()
}

fn centroid_xy(m: &dyn MeshRead, c: usize) -> (f64, f64, f64) {
    let corners = m.cell_corners(CellIx::new(c)).expect("hex corners");
    let n = corners.len() as f64;
    let (mut x, mut y, mut z) = (0.0, 0.0, 0.0);
    for v in corners {
        let p = m.point(v.get());
        x += p.x;
        y += p.y;
        z += p.z;
    }
    (x / n, y / n, z / n)
}

/// The sector (0..4) of an angle, counted from `ANGLE0` counter-clockwise.
fn sector_of(dx: f64, dy: f64) -> usize {
    let theta = (dy.atan2(dx) - ANGLE0).rem_euclid(2.0 * PI);
    (theta / (PI / 2.0)).floor() as usize
}

// ------------------------------------------------------------------ the C driver

const C_DRIVER: &str = r#"
/* W4 regions milestone: the code's view of a split .smrt through libsmrt only. Prints the name
 * and the integer tags of every region, then the region name `smrt_locate` gives at each point
 * of argv (x y z triples). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "smrt.h"

#define MUST(c) do { if (!(c)) { char m_[256] = ""; smrt_last_error_message(m_, sizeof m_); \
    fprintf(stderr, "%s:%d: %s (%s)\n", __FILE__, __LINE__, #c, m_); return 1; } } while (0)

int main(int argc, char **argv) {
    MUST(argc >= 2 && (argc - 2) % 3 == 0);
    SmrtTracker *t = NULL;
    MUST(smrt_open(argv[1], &t) == 0 && t != NULL);
    SmrtInfo info;
    MUST(smrt_info(t, &info) == 0);
    printf("INFO regions %lld complement %lld\n", (long long)info.regions, (long long)info.complement_region);
    for (int64_t r = 0; r < info.regions; r++) {
        char name[256];
        int32_t len = smrt_region_name(t, r, name, sizeof name);
        MUST(len >= 0 && len < (int32_t)sizeof name);
        int64_t nt = 0;
        MUST(smrt_region_n_tags(t, r, &nt) == 0);
        printf("REGION %lld %s %lld", (long long)r, name, (long long)nt);
        for (int64_t k = 0; k < nt; k++) {
            char key[128];
            int64_t val = 0;
            int32_t kl = smrt_region_tag(t, r, k, key, sizeof key, &val);
            MUST(kl >= 0 && kl < (int32_t)sizeof key);
            printf(" %s=%lld", key, (long long)val);
        }
        printf("\n");
    }
    SmrtCtx *ctx = NULL;
    MUST(smrt_ctx_new(t, &ctx) == 0);
    for (int a = 2; a + 2 < argc; a += 3) {
        double p[3] = {atof(argv[a]), atof(argv[a + 1]), atof(argv[a + 2])};
        SmrtLocated loc;
        MUST(smrt_locate(ctx, p, NULL, -1, &loc) == 0);
        char name[256];
        int32_t len = smrt_region_name(t, loc.region, name, sizeof name);
        MUST(len >= 0 && len < (int32_t)sizeof name);
        printf("LOC %d %s\n", (a - 2) / 3, name);
    }
    smrt_ctx_free(ctx);
    smrt_tracker_free(t);
    return 0;
}
"#;

fn target_dir() -> PathBuf {
    Path::new(env!("CARGO_TARGET_TMPDIR")).parent().expect("target dir").to_path_buf()
}

/// `libsm_rt_capi` in the `w3b-capi` target directory the W3b milestone also uses, and the driver
/// above compiled against it.
fn build_c_driver(dir: &Path) -> R<PathBuf> {
    let capi_target = target_dir().join("w3b-capi");
    let profile = if cfg!(debug_assertions) { "debug" } else { "release" };
    let mut cargo = Command::new(std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into()));
    cargo.args(["build", "-p", "sm-rt-capi", "--target-dir"]).arg(&capi_target).current_dir(repo());
    if !cfg!(debug_assertions) {
        cargo.arg("--release");
    }
    let o = cargo.output().map_err(|x| format!("cargo build sm-rt-capi: {x}"))?;
    if !o.status.success() {
        return Err(format!("cargo build -p sm-rt-capi failed: {}", String::from_utf8_lossy(&o.stderr)));
    }
    let libdir = capi_target.join(profile);
    let lib = libdir.join(format!("{}sm_rt_capi{}", std::env::consts::DLL_PREFIX, std::env::consts::DLL_SUFFIX));
    if !lib.exists() {
        return Err(format!("{} not found", lib.display()));
    }
    let src = dir.join("regions_driver.c");
    std::fs::write(&src, C_DRIVER).map_err(e)?;
    let exe = dir.join("regions_driver");
    let cc = std::env::var("CC").unwrap_or_else(|_| "cc".into());
    let o = Command::new(&cc)
        .args(["-std=c11", "-Wall", "-Wextra", "-Werror", "-D_GNU_SOURCE", "-O1"])
        .arg("-I")
        .arg(repo().join("crates/sm-rt-capi/include"))
        .arg(&src)
        .arg("-o")
        .arg(&exe)
        .arg(&lib)
        .arg(format!("-Wl,-rpath,{}", libdir.display()))
        .output()
        .map_err(|x| format!("run {cc}: {x}"))?;
    if !o.status.success() {
        return Err(format!("compiling the regions driver failed: {}", String::from_utf8_lossy(&o.stderr)));
    }
    Ok(exe)
}

// ------------------------------------------------------------------ tests

/// The session of `tools/w4_regions.smesh` and its exports.
struct Built {
    s: Server,
    out: BTreeMap<String, String>,
    smsh: (Vec<u8>, String),
    smrt: (Vec<u8>, String),
}

fn build() -> Built {
    let mut s = server();
    let out = prints(&run_script(&mut s, &script_text()));
    let smsh = export(&mut s, ExportFormat::Smsh, Some(all_split()));
    let smrt = export(&mut s, ExportFormat::Smrt, Some(all_split()));
    Built { s, out, smsh, smrt }
}

#[test]
fn the_split_lattice_has_117_regions_with_exact_volumes_in_smsh() {
    let b = build();
    eprintln!("w4 regions h_mesh {}", b.out["h_mesh"]);
    eprintln!("w4 regions smsh sha256 {}", b.smsh.1);
    eprintln!("w4 regions smrt sha256 {}", b.smrt.1);
    assert_eq!(b.out["shapes"], "{Hex: 1998}", "all hex");
    assert_eq!(b.out["violations"], "0");
    assert_eq!(b.out["cyl_faces"], "108", "9 pins x 3 circles x 4 arcs");

    let want = expected_names();
    assert_eq!(want.len(), 117, "9 x 12 + 9");
    let mesh = smsh_of(&b.smsh.0);
    let sets = region_sets(&mesh);
    assert_eq!(sets.keys().cloned().collect::<BTreeSet<_>>(), want, "the sorted name list is the generated one");
    let blocks: Vec<&str> = mesh.sets.iter().filter(|x| !x.name.starts_with("region/")).map(|x| x.name.as_str()).collect();
    assert_eq!(blocks, ["fuel", "water"], "the block sets stay");

    let st = b.s.workspace().state();
    let layer = st.layer::<MeshLayer>().expect("meshes");
    let (_, entry) = layer.meshes().next().expect("one mesh");
    let m = entry.mesh();

    // G1: the regions partition the cells; each cell is, by its centroid, in the slot, ring and
    // sector its set names (the pin centre of slot (i, j) is (i p, j p)).
    let mut seen = vec![false; m.cell_count()];
    for (name, cells) in &sets {
        let (i, j, sub) = parse_name(name);
        let (cx, cy) = (i as f64 * PITCH, j as f64 * PITCH);
        for &c in cells {
            assert!(!std::mem::replace(&mut seen[c as usize], true), "{name}: cell {c} is in two regions");
            let (x, y, _) = centroid_xy(m, c as usize);
            let (dx, dy) = (x - cx, y - cy);
            let r = dx.hypot(dy);
            match sub {
                None => assert!(dx.abs() < PITCH / 2.0 && dy.abs() < PITCH / 2.0 && r > R * 0.99, "{name}: cell {c} centroid ({x}, {y})"),
                Some((ring, sector)) => {
                    // VOLFIT puts the circle nodes at their equal-area radius (<= 1.3 % outside the circle).
                    let (lo, hi) = (if ring == 0 { 0.0 } else { ring_r(ring) * 0.99 }, ring_r(ring + 1) * 1.02);
                    assert!(r > lo && r < hi, "{name}: cell {c} radius {r} outside ({lo}, {hi})");
                    assert_eq!(sector_of(dx, dy), sector, "{name}: cell {c} centroid ({x}, {y})");
                }
            }
        }
    }
    assert!(seen.iter().all(|&s| s), "the regions cover every cell");

    // G2: per-region volume after VOLFIT.
    let (fuel, water) = (PI * R * R / 12.0, PITCH * PITCH - PI * R * R);
    for (name, cells) in &sets {
        let got: f64 = cells.iter().map(|&c| cell_volume(m, CellIx::new(c as usize))).sum();
        let want = if name.starts_with("fuel@") { fuel } else { water };
        assert!(((got - want) / want).abs() < 1e-12, "{name}: volume {got} vs {want}");
    }
}

#[test]
fn the_smrt_tags_and_regions_read_back_through_the_c_abi() {
    let b = build();
    let dir = Scratch::new();
    let smrt = dir.0.join("lattice.smrt");
    std::fs::write(&smrt, &b.smrt.0).expect("write smrt");
    let exe = match build_c_driver(&dir.0) {
        Ok(x) => x,
        Err(x) => panic!("C driver: {x}"),
    };

    // 36 points: radius 0.1 (ring 0) in the middle of every sector of every pin, and the water
    // corners of every cell.
    let mut args = vec![ts(&smrt)];
    let mut want_loc = Vec::new();
    for i in 0..3 {
        for j in 0..3 {
            for s in 0..4 {
                let a = ANGLE0 + (s as f64 + 0.5) * PI / 2.0;
                args.extend([format!("{:.17e}", i as f64 * PITCH + 0.1 * a.cos()), format!("{:.17e}", j as f64 * PITCH + 0.1 * a.sin()), "0.0".into()]);
                want_loc.push(format!("fuel@{FRAME}[{i},{j}]/r0/s{s}"));
            }
        }
    }
    for i in 0..3 {
        for j in 0..3 {
            args.extend([format!("{:.17e}", i as f64 * PITCH + 0.55), format!("{:.17e}", j as f64 * PITCH + 0.55), "0.1".into()]);
            want_loc.push(format!("water@{FRAME}[{i},{j}]"));
        }
    }
    let o = Command::new(&exe).args(&args).output().expect("run driver");
    let text = String::from_utf8_lossy(&o.stdout).into_owned();
    assert!(o.status.success(), "driver failed: {text}\n{}", String::from_utf8_lossy(&o.stderr));

    let mut regions: BTreeMap<String, Vec<(String, i64)>> = BTreeMap::new();
    let mut locs: BTreeMap<usize, String> = BTreeMap::new();
    for l in text.lines() {
        let mut it = l.split_whitespace();
        match it.next() {
            Some("REGION") => {
                let (_, name, nt) = (it.next(), it.next().expect("name"), it.next().expect("ntags"));
                let tags: Vec<(String, i64)> = it
                    .map(|kv| {
                        let (k, v) = kv.split_once('=').expect("k=v");
                        (k.to_owned(), v.parse().expect("tag value"))
                    })
                    .collect();
                assert_eq!(nt.parse::<usize>().expect("n"), tags.len(), "{name}");
                regions.insert(name.to_owned(), tags);
            }
            Some("LOC") => {
                let k: usize = it.next().expect("k").parse().expect("k");
                locs.insert(k, it.next().expect("region").to_owned());
            }
            _ => {}
        }
    }
    assert!(text.contains("INFO regions"), "{text}");
    let want = expected_names();
    let named: BTreeSet<String> = regions.keys().filter(|n| want.contains(*n)).cloned().collect();
    assert_eq!(named, want, "the 117 regions reach the C code by name");
    assert_eq!(regions.len(), 117, "the regions cover the mesh: no implicit complement");
    for (name, tags) in &regions {
        if !want.contains(name) {
            assert!(tags.is_empty(), "complement `{name}` has no tags");
            continue;
        }
        let (i, j, sub) = parse_name(name);
        let mut t = vec![(format!("lat.{FRAME}.i"), i as i64), (format!("lat.{FRAME}.j"), j as i64)];
        if let Some((r, s)) = sub {
            t.extend([("ring".into(), r as i64), ("ring_count".into(), 3), ("sector".into(), s as i64), ("sector_count".into(), 4)]);
        }
        assert_eq!(tags, &t, "{name}");
    }
    for (k, w) in want_loc.iter().enumerate() {
        assert_eq!(locs.get(&k), Some(w), "point {k}");
    }
}

/// The OpenMC deck of the same lattice (pin of radius 0.4096 in a 1.26 water square, lattice id
/// 20, cells named `fuel` and `water`, z in [-0.5, 0.5]).
fn deck() -> String {
    let half = 1.5 * PITCH;
    let uni = ["10 10 10"; 3].join("\n");
    format!(
        "<?xml version='1.0' encoding='UTF-8'?>\n<geometry>\n\
  <cell id=\"1\" fill=\"20\" region=\"11 -12 13 -14 15 -16\" universe=\"0\"/>\n\
  <cell id=\"100\" name=\"fuel\" material=\"1\" region=\"-1\" universe=\"10\"/>\n\
  <cell id=\"101\" name=\"water\" material=\"2\" region=\"1\" universe=\"10\"/>\n\
  <lattice id=\"20\">\n    <pitch>{PITCH} {PITCH}</pitch>\n    <dimension>3 3</dimension>\n    <lower_left>{} {}</lower_left>\n    <universes>\n{uni}\n</universes>\n  </lattice>\n\
  <surface id=\"1\" type=\"z-cylinder\" coeffs=\"0.0 0.0 {R}\"/>\n\
  <surface id=\"11\" type=\"x-plane\" boundary=\"vacuum\" coeffs=\"{}\"/>\n  <surface id=\"12\" type=\"x-plane\" boundary=\"vacuum\" coeffs=\"{half}\"/>\n\
  <surface id=\"13\" type=\"y-plane\" boundary=\"vacuum\" coeffs=\"{}\"/>\n  <surface id=\"14\" type=\"y-plane\" boundary=\"vacuum\" coeffs=\"{half}\"/>\n\
  <surface id=\"15\" type=\"z-plane\" boundary=\"vacuum\" coeffs=\"-0.5\"/>\n  <surface id=\"16\" type=\"z-plane\" boundary=\"vacuum\" coeffs=\"0.5\"/>\n\
</geometry>\n",
        -half, -half, -half, -half
    )
}

const MATERIALS: &str = "<?xml version='1.0' encoding='UTF-8'?>\n<materials>\n\
  <material id=\"1\" name=\"fuel\"><density value=\"1.0\" units=\"macro\"/><macroscopic name=\"dummy\"/></material>\n\
  <material id=\"2\" name=\"water\"><density value=\"1.0\" units=\"macro\"/><macroscopic name=\"dummy\"/></material>\n\
</materials>\n";

/// Regions of a `.smrt` image, the implicit complement left out, with their tags.
fn smrt_regions(bytes: &[u8]) -> BTreeMap<String, Vec<(String, i64)>> {
    let loaded = sm_rt::load_image(&sm_rt::read_smrt(bytes).expect("read_smrt")).expect("load_image");
    loaded.regions().0.iter().filter(|r| !r.implicit_complement).map(|r| (r.name.clone(), r.tags.clone())).collect()
}

#[test]
fn the_openmc_deck_gives_the_same_lattice_region_names() {
    let dir = Scratch::new();
    let (g, m) = (dir.0.join("geometry.xml"), dir.0.join("materials.xml"));
    std::fs::write(&g, deck()).expect("geometry");
    std::fs::write(&m, MATERIALS).expect("materials");

    // The deck path: stage the bytes, import, mesh, export with the lattice split.
    let mut s = server();
    let mut stage = |p: &Path| -> Value {
        let text = format!("session.stage({})\n", serde_json::to_string(&ts(p)).expect("path"));
        let v = s.call(Request::ScriptRun { text, path: None, includes: Default::default() }).expect("stage");
        let ev = v["events"].as_array().expect("events").iter().find(|e| e["event"] == "session").expect("stage event");
        json!({"sha256": ev["result"]["sha256"]})
    };
    let (gs, ms) = (stage(&g), stage(&m));
    s.call(Request::Apply { op: "model.import_openmc".into(), params: json!({"geometry": gs, "materials": ms}), version: None }).unwrap_or_else(|x| panic!("import: {x} {}", x.data));
    run_script(&mut s, "mesh.size(targets: all volumes, size: 0.3)\nmesh.auto(strategy: \"default\")\nlet mm = mesh.run(targets: all volumes)\n");
    let lat = Some(json!({"regions": {"split": ["lattice"]}}));
    let from_deck = smrt_regions(&export(&mut s, ExportFormat::Smrt, lat.clone()).0);

    // The model path: the script's session (rings and sectors cut) under the same split.
    let mut b = build();
    let from_model = smrt_regions(&export(&mut b.s, ExportFormat::Smrt, lat).0);

    assert_eq!(from_deck.len(), 18, "9 fuel and 9 water regions: {:?}", from_deck.keys());
    assert_eq!(from_deck.keys().collect::<Vec<_>>(), from_model.keys().collect::<Vec<_>>(), "same region names");
    assert_eq!(from_deck, from_model, "same names and the same integer slot tags");
    let r = &from_deck[&format!("fuel@{FRAME}[2,1]")];
    assert_eq!(r, &[(format!("lat.{FRAME}.i"), 2), (format!("lat.{FRAME}.j"), 1)]);
}

#[test]
fn the_exports_are_identical_across_front_ends_runs_and_thread_counts() {
    let a = build();
    let again = build();
    assert_eq!((&a.smsh.0, &a.smrt.0), (&again.smsh.0, &again.smrt.0), "two runs");
    assert_eq!(a.out["h_mesh"], again.out["h_mesh"]);
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool").install(build);
    assert_eq!((&a.smsh.0, &a.smrt.0), (&one.smsh.0, &one.smrt.0), "1 thread vs default");
    assert_eq!(a.out["h_mesh"], one.out["h_mesh"]);

    // CLI: run the script (frame renamed as in `build`) in a project, export with `--regions`.
    let dir = Scratch::new();
    let script = dir.0.join("w4_regions.smesh");
    std::fs::write(&script, script_text()).expect("script");
    let proj = dir.0.join("proj");
    ok_text(&["new", &ts(&proj)], Some(&dir.0), Threads::Inherit).expect("new");
    let out = ok_text(&["-p", &ts(&proj), "run", &ts(&script), "--author", AUTHOR, "--fixed-time", TIME], Some(&dir.0), Threads::Inherit).expect("run");
    assert_eq!(prints(&out)["h_mesh"], a.out["h_mesh"], "CLI session hash");
    for (file, bytes) in [("lattice.smsh", &a.smsh.0), ("lattice.smrt", &a.smrt.0)] {
        let path = dir.0.join(file);
        ok_text(&["-p", &ts(&proj), "export", &ts(&path), "--regions", "lattice,ring,sector"], Some(&dir.0), Threads::Inherit).expect("export");
        assert_eq!(&read_bytes(&path).expect("file"), bytes, "CLI {file}");
    }

    // The pinned digests (macOS arm64; INT pins Linux).
    if cfg!(all(target_os = "macos", target_arch = "aarch64")) && !MAC_SMSH.is_empty() {
        assert_eq!(a.smsh.1, MAC_SMSH, "lattice.smsh");
        assert_eq!(a.smrt.1, MAC_SMRT, "lattice.smrt");
    }
}
