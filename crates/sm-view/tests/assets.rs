//! Acceptance 10 (assets and the dist hash) and the recorded base64 overhead.
#![allow(missing_docs)]

mod common;

use std::path::PathBuf;
use std::process::Command;

use common::*;
use serde_json::json;
use sm_op_api::Hash256;

fn web() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("web")
}

fn dist_listing(dir: &std::path::Path) -> String {
    let mut names: Vec<String> = std::fs::read_dir(dir)
        .expect("dist dir")
        .map(|e| e.expect("entry").file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    names
        .iter()
        .map(|n| format!("{}  {n}\n", Hash256::of(&std::fs::read(dir.join(n)).expect("read")).to_hex()))
        .collect()
}

#[test]
fn assets_lists_the_bundle_with_mime_types() {
    let a = sm_view::assets();
    let names: Vec<&str> = a.iter().map(|x| x.path).collect();
    assert_eq!(names, ["index.html", "viewer.js", "viewer.js.map"]);
    assert!(a[0].mime.starts_with("text/html"));
    assert!(a[1].mime.starts_with("text/javascript"));
    assert_eq!(a[2].mime, "application/json");
    for x in a {
        assert!(!x.bytes.is_empty(), "{}", x.path);
        assert_eq!(x.bytes, std::fs::read(web().join("dist").join(x.path)).expect("file").as_slice(), "{}", x.path);
    }
    assert!(std::str::from_utf8(a[0].bytes).expect("utf8").contains("viewer.js"));
}

#[test]
fn dist_matches_the_committed_hash_file() {
    let want = std::fs::read_to_string(web().join("dist.sha256")).expect("dist.sha256");
    assert!(
        want == dist_listing(&web().join("dist")),
        "web/dist is stale: run `npm run build` in crates/sm-view/web and commit dist/ and dist.sha256"
    );
}

#[test]
fn rebuild_with_esbuild_reproduces_dist() {
    let w = web();
    let esbuild = w.join("node_modules/.bin/esbuild");
    if !esbuild.exists() || Command::new("node").arg("--version").output().is_err() {
        eprintln!("skipped: node or web/node_modules is missing (run `npm ci` in crates/sm-view/web)");
        return;
    }
    let out = w.join(".distcheck");
    let _ = std::fs::remove_dir_all(&out);
    let st = Command::new("node").arg("build.mjs").arg(".distcheck").current_dir(&w).status().expect("node build");
    assert!(st.success(), "esbuild build failed");
    let (a, b) = (dist_listing(&w.join("dist")), dist_listing(&out));
    let _ = std::fs::remove_dir_all(&out);
    assert!(a == b, "web/dist differs from a fresh build: run `npm run build` and commit");
}

/// Records (never asserts a time) the wire size of a 10^5-cell skin: raw bytes versus base64.
/// Slow in debug builds: run with `cargo test -p sm-view -- --ignored --nocapture`.
#[test]
#[ignore = "builds a 1e5-cell mesh; run explicitly to record the base64 overhead"]
fn base64_overhead_is_recorded() {
    let (r, _) = Rig::with("block", [46, 46, 47], &[]);
    let full = r.query("view.mesh_skin", json!({})).expect("skin");
    let clipped = r.query("view.mesh_skin", json!({"clip": {"origin": [23.0, 0.0, 0.0], "normal": [1.0, 0.0, 0.0]}})).expect("clip");
    for (name, v) in [("full", &full), ("clipped", &clipped)] {
        let (mut raw, mut b64) = (0usize, 0usize);
        for k in ["positions", "indices", "tri_cell", "tri_owner", "outline_indices"] {
            let s = v[k].as_str().expect("string");
            b64 += s.len();
            raw += sm_view::b64::decode(s).expect("decode").len();
        }
        let json_len = serde_json::to_string(v).expect("json").len();
        eprintln!(
            "skin {name}: {} triangles, {} vertices, raw {raw} B, base64 {b64} B (x{:.4}), whole reply {json_len} B",
            v["tri_count"], v["vertex_count"], b64 as f64 / raw as f64
        );
        assert!(b64 >= raw && (b64 as f64) <= raw as f64 * 4.0 / 3.0 + 64.0, "base64 is 4/3 of raw plus padding");
    }
    assert_eq!(full["kept_cells"], 46 * 46 * 47);
}
