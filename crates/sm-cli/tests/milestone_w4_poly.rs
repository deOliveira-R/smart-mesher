//! W4 polyhedral milestone (plan `docs/design/w4/plan.md` section 5 item 1).
//!
//! `tools/w4_poly.smesh` takes the OpenMC `pin` deck (fixture bytes, pinned by sha256) through
//! `model.import_openmc` (blocks fuel, void, clad, water), `mesh.intervals` on the circles,
//! `mesh.auto {strategy: "tet"}`, `mesh.run`, `mesh.to_poly` (circumcentric when `mesh.poly_info`
//! says the tet mesh qualifies, barycentric otherwise) and exports `.smsh` (with `gc_*`) and
//! `.smrt`. Collect-all style like `milestone_w4_pin`. Run it as
//! `cargo test -p sm-cli --test milestone_w4_poly -- --nocapture` (and with `RAYON_NUM_THREADS=1`).
//!
//! | row | what |
//! |---|---|
//! | Q1 | the deck bytes are the pinned ones; the dual the script chose is the one `mesh.poly_info` allows (the strict circumcentric gate refuses this mesh and the refusal changes nothing); all-tet source |
//! | Q2 | block volumes: model volumes are the closed forms (1e-9); the blocks sum to the box (1e-9); each disc is bounded by its circle (inscribed mesh); tet, polyhedral, `.smsh` set and `.smrt` region volumes agree (1e-12) |
//! | Q3 | `.smrt` regions equal the block names (the `.smsh` cell sets) and the exports carry no warning |
//! | Q4 | the 600 oracle points through the `.smrt`: the region is OpenMC's or further out (then within the circle band), never further in |
//! | Q5 | `mesh.validate` 0 violations (tet and polyhedral); `.smsh` `gc_*` equals the recomputed geometry and `validate_geometry` is empty |
//! | Q6 | CLI (twice, 1 thread), MCP, in-process and Python agree on prints and file digests |
//! | Q7 | the project record replays (also under an unregistered strategy name) to the same state |
//! | Q8 | the hashes and the export digests are pinned (every platform) and printed |
//!
//! Not asserted, and why: the per-block volume is *not* the inscribed-polygon closed form (ruling
//! OR-j). The tet mesher puts Steiner nodes on the cylinders and its lateral triangles are not
//! the polygon's facets. The script turns on the curvature sizing (tol 0.0025) so the sides are
//! about one interval long: the fuel is 0.7% below the circle where the 36-gon prism is 0.5%
//! below it. The test asserts two-sided bounds per disc (mesh <= circle, mesh >= prism - 0.55 h^2 H)
//! and, for the oracle points the mesh puts further out than OpenMC, the band between the circle
//! and its 36-gon apothem. The closed form needs a swept mesh (`drafts/csgbrep.md` D7), and
//! `mesh.preserve_volume` refuses a tet mesh (not a sweep of the circle).
//!
//! The Python leg runs an inline driver through the installed wheel (`W3C_PYTHON` / `W3B_PYTHON`,
//! else `tools/w3b_python.sh`); a missing wheel is a skip, or a failure under `CI=true`.

mod w3a_support;

use std::collections::BTreeMap;
use std::f64::consts::PI;
use std::path::{Path, PathBuf};
use std::process::Command;

use serde_json::{Value, json};
use sm_core::Point3;
use sm_layer_api::MeshLayer;
use sm_mesh_api::{CellIx, GeomTol, MeshEntityKind, SetMembers, cell_volume, validate_geometry};
use sm_meshio::SmshFile;
use sm_protocol::{Endpoint, ExportFormat, ExportResult, Request};
use sm_rt_api::{Scratch, Tracker};
use sm_server::{Server, standard_with};

use w3a_support::*;

/// sha256 of the committed `pin` deck (`crates/sm-csgio/tests/fixtures/openmc/pin`).
const GEOMETRY_SHA: &str = "b4f59004dcb17ec94c2b17f75caa528dbe036e65828c5835cea28de52e246b46";
const MATERIALS_SHA: &str = "1998e8644ee0e165e98b569aec4b88251546df85deccca2a2fc9b6a7419b4015";
const KEYS: [&str; 6] = ["h_model", "h_sized", "h_auto", "h_tet", "h_mesh", "dual"];
const FILES: [&str; 2] = ["pin.smsh", "pin.smrt"];
/// Fuel, void gap, clad and water in radial order (the `ordinal` of a region name).
const NAMES: [&str; 4] = ["fuel", "void", "clad", "water"];
const RADII: [f64; 3] = [0.4, 0.42, 0.46];
const HEIGHT: f64 = 4.0;
const PITCH: f64 = 1.26;

/// Pins (Q8), one set for every platform since C12; the last is the oracle mismatch count.
const PINS: [&str; 7] = [
    "b0cddbd5ad1b0a00e5f255b86de5eadee394f36bb813b4b84be97250249629f8",
    "359a5bc6500704a93d6fed43a625976e9600483e35f377a03308b694b85d5f37",
    "e392a96ef45b53327fcf00677ea896bb6fe7bdcf9479b8a92ade8b04ebe3d2af",
    "7a49f60fb103c992f9f23b5ca9a6e88ea0b1975771b05d5a0850e09084f1ffe4",
    "2d8517692a2c46c7a0c3fed38420595371983f56ef91f850181b30925de89627",
    "barycentric",
    "3",
];

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().expect("repo root")
}

fn fixture(name: &str) -> PathBuf {
    repo().join("crates/sm-csgio/tests/fixtures/openmc/pin").join(name)
}

fn script_path() -> PathBuf {
    repo().join("tools/w4_poly.smesh")
}

fn base() -> PathBuf {
    // One tree per test process; this file has one test but keep the shape of the W4 pin test.
    Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("milestone_w4_poly-{}", std::process::id()))
}

fn scratch(name: &str) -> PathBuf {
    let d = base().join(name);
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).expect("scratch dir");
    d
}

fn ts(p: &Path) -> String {
    p.to_string_lossy().into_owned()
}

/// Removes this process's scratch tree when the test ends, pass or fail.
struct Cleanup;

impl Drop for Cleanup {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(base());
    }
}

/// The deck files copied into `dir` (the script stages them by relative path).
fn with_deck(dir: &Path) -> R<()> {
    for f in ["geometry.xml", "materials.xml"] {
        std::fs::copy(fixture(f), dir.join(f)).map_err(|x| format!("copy {f}: {x}"))?;
    }
    Ok(())
}

fn digest(p: &Path) -> R<String> {
    Ok(sha_hex(&read_bytes(p)?))
}

/// What one front end produced: the printed keys and the digest of each exported file. The files
/// are deleted after digesting unless `keep` (each is tens of MB).
struct Front {
    prints: BTreeMap<String, String>,
    files: BTreeMap<String, String>,
}

impl Front {
    fn new(output: &str, dir: &Path, keep: bool) -> R<Self> {
        let mut files = BTreeMap::new();
        for f in FILES {
            files.insert(f.to_owned(), digest(&dir.join(f))?);
            if !keep {
                let _ = std::fs::remove_file(dir.join(f));
            }
        }
        Ok(Self { prints: printed(output), files })
    }
}

fn cli_memory(name: &str, threads: Threads, keep: bool) -> R<Front> {
    let dir = scratch(name);
    with_deck(&dir)?;
    let args = ["--memory", "run", &ts(&script_path()), "--author", AUTHOR, "--fixed-time", TIME];
    let out = ok_text(&args, Some(&dir), threads)?;
    Front::new(&out, &dir, keep)
}

fn mcp_front(name: &str) -> R<Front> {
    let dir = scratch(name);
    with_deck(&dir)?;
    let mut lines = mcp_init();
    lines.push(tool_call(2, "run_script", json!({"text": read_text(&script_path())})));
    lines.push(tool_call(3, "session_hash", json!({})));
    let replies = mcp_session(&PIN, &lines, Threads::Inherit, Some(&dir))?;
    let run = tool_ok(&replies, 2)?;
    let hash = tool_ok(&replies, 3)?;
    let out = run["output"].as_str().ok_or_else(|| format!("run_script has no output: {run}"))?;
    let f = Front::new(out, &dir, false)?;
    if hash["state"].as_str() != f.prints.get("h_mesh").map(String::as_str) {
        return Err(format!("MCP session_hash {hash} != printed h_mesh"));
    }
    Ok(f)
}

/// An in-process server in `dir/proj` with the script run in it; the exports stay for `keep`.
fn in_process(name: &str) -> R<(Front, Server, PathBuf)> {
    let dir = scratch(name);
    let proj = dir.join("proj");
    let mut s = Server::create(standard_with(Some(AUTHOR.into()), Some(TIME.into())), &proj, "w4").map_err(e)?;
    with_deck(&proj)?;
    let v = s
        .call(Request::ScriptRun { text: read_text(&script_path()), path: None, includes: Default::default() })
        .map_err(|x| format!("script.run: {x} {}", x.data["rendered"].as_str().unwrap_or("")))?;
    let out = v["output"].as_str().ok_or_else(|| format!("script.run has no output: {v}"))?;
    let f = Front::new(out, &proj, true)?;
    let h = s.call(Request::Hash { at: None }).map_err(e)?;
    if h["state"].as_str() != f.prints.get("h_mesh").map(String::as_str) {
        return Err(format!("in-process session.hash {} != printed h_mesh", h["state"]));
    }
    Ok((f, s, proj))
}

/// A CLI project run (the project keeps the input store the replay needs).
fn cli_project(name: &str) -> R<(Front, PathBuf)> {
    let dir = scratch(name);
    let proj = dir.join("proj");
    ok_text(&["new", &ts(&proj)], Some(&dir), Threads::Inherit)?;
    with_deck(&proj)?;
    let out = ok_text(&["-p", &ts(&proj), "run", &ts(&script_path()), "--author", AUTHOR, "--fixed-time", TIME], Some(&dir), Threads::Inherit)?;
    Ok((Front::new(&out, &proj, false)?, proj))
}

// ------------------------------------------------------------------ Python leg

const PY_DRIVER: &str = r#"
import hashlib, os, sys
import smartmesh as sm

TIME, AUTHOR, script, out = sys.argv[1:5]
os.makedirs(out, exist_ok=True)
os.chdir(out)
s = sm.Session.memory(author=AUTHOR, fixed_time=TIME)
with open(script, encoding="utf-8") as fh:
    r = s.run_script(fh.read())
p = {}
for line in r.output.splitlines():
    k, _, rest = line.partition(" ")
    p[k] = rest.strip()
if s.hash().state != p["h_mesh"]:
    print("FAIL session hash differs from h_mesh")
    sys.exit(1)
for k in ("h_model", "h_sized", "h_auto", "h_tet", "h_mesh", "dual"):
    print("py_" + k + " " + p[k])
for f in ("pin.smsh", "pin.smrt"):
    with open(f, "rb") as fh:
        print("py_file " + f + " " + hashlib.sha256(fh.read()).hexdigest())
    os.remove(f)
print("OK")
"#;

fn python() -> R<Result<PathBuf, String>> {
    for var in ["W3C_PYTHON", "W3B_PYTHON"] {
        if let Some(p) = std::env::var_os(var) {
            return Ok(Ok(PathBuf::from(p)));
        }
    }
    let venv = Path::new(env!("CARGO_TARGET_TMPDIR")).join("w4-poly-venv");
    let o = Command::new("bash")
        .arg(repo().join("tools").join("w3b_python.sh"))
        .arg(&venv)
        .env("CARGO_TARGET_DIR", Path::new(env!("CARGO_TARGET_TMPDIR")).parent().expect("target dir"))
        .output()
        .map_err(|x| format!("run w3b_python.sh: {x}"))?;
    match o.status.code() {
        Some(0) => {
            let out = String::from_utf8_lossy(&o.stdout);
            Ok(Ok(PathBuf::from(out.lines().last().ok_or("w3b_python.sh printed nothing")?.trim())))
        }
        Some(3) => Ok(Err(String::from_utf8_lossy(&o.stderr).trim().to_owned())),
        _ => Err(format!("w3b_python.sh failed ({}): {}", o.status, String::from_utf8_lossy(&o.stderr))),
    }
}

fn python_lines() -> R<Result<Vec<String>, String>> {
    let py = match python()? {
        Ok(p) => p,
        Err(reason) => return Ok(Err(reason)),
    };
    let dir = scratch("python");
    let drv = dir.join("driver.py");
    std::fs::write(&drv, PY_DRIVER).map_err(e)?;
    let out = scratch("python-out");
    with_deck(&out)?;
    let o = Command::new(py).arg("-I").arg(&drv).args([TIME, AUTHOR]).arg(script_path()).arg(&out).output().map_err(|x| format!("run python driver: {x}"))?;
    let text = String::from_utf8_lossy(&o.stdout).into_owned();
    if o.status.code() != Some(0) || !text.lines().any(|l| l == "OK") {
        return Err(format!("python driver exited {:?}:\n{text}\n{}", o.status.code(), String::from_utf8_lossy(&o.stderr)));
    }
    Ok(Ok(text.lines().map(str::to_owned).collect()))
}

fn line_after(lines: &[String], prefix: &str) -> String {
    lines.iter().find_map(|l| l.strip_prefix(prefix).and_then(|r| r.strip_prefix(' '))).map(str::to_owned).unwrap_or_default()
}

// ------------------------------------------------------------------ helpers over results

fn query(s: &mut Server, name: &str, input: Value) -> R<Value> {
    s.call(Request::Query { name: name.into(), input, at: None }).map_err(|x| format!("{name}: {x} {}", x.data))
}

fn mesh_ids(s: &Server) -> Vec<sm_core::EntityId> {
    let mut v: Vec<_> = s.workspace().state().layer::<MeshLayer>().expect("meshes").meshes().map(|(id, _)| id).collect();
    v.sort();
    v
}

fn export_in_process(s: &mut Server, path: &Path, format: ExportFormat, mesh: Option<sm_core::EntityId>) -> R<ExportResult> {
    let v = s
        .call(Request::Export {
            path: Some(ts(path)),
            format: Some(format),
            mesh,
            provenance: false,
            inline: false,
            options: None,
            split_small_determinant: None,
            empty_patches: vec![],
            wedge_patches: vec![],
            split_warped_faces: false,
            source: Default::default(),
        })
        .map_err(|x| format!("export {}: {x} {}", path.display(), x.data))?;
    serde_json::from_value(v).map_err(e)
}

/// sha256 of the exported files (every platform).
const FILE_PINS: [(&str, &str); 2] = [("pin.smrt", "ce02bf55850fcd5c07b2f421a7cc05890f81386c55912479a71d0673428b0c4a"), ("pin.smsh", "1b99be6f3aae029698200664f963ef9ab55e0303338b0f9c200dca990ab753b4")];

/// The closed forms of the deck: model volume of fuel, void, clad and water.

fn model_volumes() -> [f64; 4] {
    let disc = |r: f64| PI * r * r * HEIGHT;
    [disc(RADII[0]), disc(RADII[1]) - disc(RADII[0]), disc(RADII[2]) - disc(RADII[1]), PITCH * PITCH * HEIGHT - disc(RADII[2])]
}

/// Inscribed prism volume of a circle meshed with `k1` equal intervals on the 1 rad arc and `k2`
/// on the other (the deck's seam): `r^2 / 2 (k1 sin(1 / k1) + k2 sin((2 pi - 1) / k2)) h`.
fn prism(r: f64, k1: f64, k2: f64) -> f64 {
    0.5 * r * r * (k1 * (1.0 / k1).sin() + k2 * ((2.0 * PI - 1.0) / k2).sin()) * HEIGHT
}

fn pr(p: &BTreeMap<String, String>, k: &str) -> String {
    p.get(k).cloned().unwrap_or_default()
}

fn region_rank(name: &str) -> Option<usize> {
    NAMES.iter().position(|n| *n == name)
}

#[test]
fn milestone_w4_poly() {
    let _cleanup = Cleanup;
    let mut rows = Rows::default();

    // ---------------- the front ends (concurrently: each run is tens of seconds in debug)
    let (cli, again, one, mcp, inproc, project) = std::thread::scope(|sc| {
        let a = sc.spawn(|| cli_memory("cli", Threads::Default, true));
        let b = sc.spawn(|| cli_memory("again", Threads::Default, false));
        let c = sc.spawn(|| cli_memory("one-thread", Threads::One, false));
        let d = sc.spawn(|| mcp_front("mcp"));
        let f = sc.spawn(|| cli_project("project"));
        let g = in_process("inproc");
        (a.join().expect("cli"), b.join().expect("again"), c.join().expect("1t"), d.join().expect("mcp"), g, f.join().expect("project"))
    });
    let py = python_lines();

    let cli = match cli {
        Ok(f) => f,
        Err(x) => {
            println!("FAIL CLI run: {x}");
            panic!("the CLI run failed: {x}");
        }
    };
    for (label, other) in [("CLI again", again.as_ref().map(|f| (&f.prints, &f.files))), ("CLI 1 thread", one.as_ref().map(|f| (&f.prints, &f.files))), ("MCP", mcp.as_ref().map(|f| (&f.prints, &f.files))), ("project run", project.as_ref().map(|(f, _)| (&f.prints, &f.files)))] {
        match other {
            Ok((p, f)) => {
                rows.eq(6, &format!("{label} prints"), p, &cli.prints);
                rows.eq(6, &format!("{label} files"), f, &cli.files);
            }
            Err(x) => rows.fail(6, format!("{label}: {x}")),
        }
    }
    let (mut s, proj) = match inproc {
        Ok((f, s, proj)) => {
            rows.eq(6, "in-process prints", &f.prints, &cli.prints);
            rows.eq(6, "in-process files", &f.files, &cli.files);
            (s, proj)
        }
        Err(x) => {
            rows.fail(6, format!("in-process: {x}"));
            finish(&rows);
            return;
        }
    };
    match &py {
        Ok(Ok(l)) => {
            for k in KEYS {
                rows.eq(6, &format!("python {k} vs CLI"), &line_after(l, &format!("py_{k}")), &cli.prints.get(k).cloned().unwrap_or_default());
            }
            for (f, d) in &cli.files {
                rows.eq(6, &format!("python {f} vs CLI"), &line_after(l, &format!("py_file {f}")), d);
            }
        }
        Ok(Err(reason)) => {
            let fatal = std::env::var("CI").ok().as_deref() == Some("true");
            println!("{} Q6 Python leg (wheel): {reason}", if fatal { "FAIL" } else { "SKIP" });
            if fatal {
                rows.fail(6, format!("Python wheel unavailable under CI=true: {reason}"));
            } else {
                rows.note(6, format!("Python skipped: {reason}"));
            }
        }
        Err(x) => rows.fail(6, format!("Python leg: {x}")),
    }

    // ---------------- Q1: deck, dual choice
    rows.eq(1, "geometry.xml sha256", &sha_hex(&read_bytes(&fixture("geometry.xml")).unwrap_or_default()), &GEOMETRY_SHA.to_owned());
    rows.eq(1, "materials.xml sha256", &sha_hex(&read_bytes(&fixture("materials.xml")).unwrap_or_default()), &MATERIALS_SHA.to_owned());
    rows.eq(1, "staged geometry sha256", &pr(&cli.prints, "sha_geometry"), &GEOMETRY_SHA.to_owned());
    rows.eq(1, "staged materials sha256", &pr(&cli.prints, "sha_materials"), &MATERIALS_SHA.to_owned());
    rows.eq(1, "deck volumes", &pr(&cli.prints, "deck_volumes"), &"4".to_owned());
    let ids = mesh_ids(&s);
    rows.eq(1, "meshes: the tet mesh and its dual", &ids.len(), &2);
    if ids.len() == 2 {
        let (tet, poly) = (ids[0], ids[1]);
        let n_tets = {
            let st = s.workspace().state();
            let l = st.layer::<MeshLayer>().expect("meshes");
            let m = l.get(tet).expect("tet").mesh();
            m.count(MeshEntityKind::Cell)
        };
        rows.eq(1, "tet_shapes (all Tet)", &pr(&cli.prints, "tet_shapes"), &format!("{{Tet: {n_tets}}}"));
        match query(&mut s, "mesh.poly_info", json!({"mesh": tet.to_string()})) {
            Ok(info) => {
                let el = &info["dual_eligibility"];
                let (nd, oc) = (el["non_delaunay_faces"].as_u64().unwrap_or(0), el["outside_circumcentres"].as_u64().unwrap_or(0));
                let want = if nd + oc == 0 { "circumcentric" } else { "barycentric" };
                rows.eq(1, "dual chosen = the one the eligibility report allows", &pr(&cli.prints, "dual"), &want.to_owned());
                rows.eq(1, "printed non_delaunay_faces", &cli.prints.get("non_delaunay_faces").and_then(|v| v.parse::<u64>().ok()), &Some(nd));
                rows.eq(1, "printed outside_circumcentres", &cli.prints.get("outside_circumcentres").and_then(|v| v.parse::<u64>().ok()), &Some(oc));
                println!("NOTE Q1 eligibility: {nd} non-Delaunay faces of {}, {oc} outside circumcentres of {} tets -> {want}", el["interior_faces"], el["tets"]);
                if want == "barycentric" {
                    // The strict gate refuses the mesh, and a refused op changes nothing.
                    let h0 = s.call(Request::Hash { at: None }).map(|v| v["state"].clone()).map_err(e);
                    let r = s.call(Request::Apply { op: "mesh.to_poly".into(), params: json!({"mesh": tet.to_string(), "dual": "circumcentric"}), version: None });
                    rows.check(1, r.is_err(), || "mesh.to_poly circumcentric must be refused on a mesh the report rejects".into());
                    let h1 = s.call(Request::Hash { at: None }).map(|v| v["state"].clone()).map_err(e);
                    rows.eq(1, "refused circumcentric leaves the state", &h1, &h0);
                }
            }
            Err(x) => rows.fail(1, x),
        }

        // ---------------- Q2 .. Q5: content of the meshes and exports
        let smsh_path = proj.join("pin.smsh");
        let smrt_path = proj.join("pin.smrt");
        content_rows(&mut rows, &mut s, tet, poly, &smsh_path, &smrt_path);
        // The exports are silent (no warning note), through the in-process server.
        let out = scratch("exports");
        for (f, fmt) in [("a.smsh", ExportFormat::Smsh), ("a.smrt", ExportFormat::Smrt)] {
            match export_in_process(&mut s, &out.join(f), fmt, Some(poly)) {
                Ok(res) => rows.check(3, !res.notes.iter().any(|n| n.is_warning()), || format!("{f} notes: {:?}", res.notes)),
                Err(x) => rows.fail(3, x),
            }
            let _ = std::fs::remove_file(out.join(f));
        }
    }

    // ---------------- Q7: replay of the project record
    match &project {
        Ok((_, pdir)) => replay_rows(&mut rows, pdir, &cli),
        Err(x) => rows.fail(7, format!("project run: {x}")),
    }

    // ---------------- Q8: pins
    for k in KEYS {
        println!("HASH w4_poly {k} {}", cli.prints.get(k).cloned().unwrap_or_default());
    }
    for (f, d) in &cli.files {
        println!("HASH w4_poly {f} {d}");
        let pin = FILE_PINS.iter().find(|(n, _)| n == f).map(|(_, h)| (*h).to_owned());
        rows.eq(8, &format!("pinned digest of {f}"), &Some(d.clone()), &pin);
    }
    for (i, k) in KEYS.iter().enumerate() {
        rows.eq(8, &format!("pinned {k}"), &cli.prints.get(*k).cloned().unwrap_or_default(), &PINS[i].to_owned());
    }
    finish(&rows);
}

/// Q2 to Q5 on the in-process server `s` and the exported files.
fn content_rows(rows: &mut Rows, s: &mut Server, tet: sm_core::EntityId, poly: sm_core::EntityId, smsh_path: &Path, smrt_path: &Path) {
    let want = model_volumes();
    let box_volume = PITCH * PITCH * HEIGHT;

    // `.smsh`: decode, sets by name, gc_*.
    let smsh = match read_bytes(smsh_path).and_then(|b| SmshFile::parse(&b).map_err(e).and_then(|f| f.decode().map_err(e))) {
        Ok(d) => d,
        Err(x) => {
            rows.fail(2, format!("pin.smsh: {x}"));
            return;
        }
    };
    let vcheck = query(s, "mesh.volume_check", json!({"mesh": tet.to_string()}));
    let validated = [("tet", query(s, "mesh.validate", json!({"mesh": tet.to_string()}))), ("polyhedral", query(s, "mesh.validate", json!({"mesh": poly.to_string()})))];
    let st = s.workspace().state();
    let layer = st.layer::<MeshLayer>().expect("meshes");
    let pm = layer.get(poly).expect("poly").mesh();

    let mut set_vol: BTreeMap<String, f64> = BTreeMap::new();
    for set in &smsh.sets {
        if let SetMembers::Cells(c) = &set.members {
            set_vol.insert(set.name.clone(), c.iter().map(|&cell| cell_volume(pm, CellIx(cell.0))).sum());
        }
    }
    let mut names: Vec<&str> = set_vol.keys().map(String::as_str).collect();
    names.sort_unstable();
    let mut expect_names = NAMES.to_vec();
    expect_names.sort_unstable();
    rows.eq(3, ".smsh cell sets are the block names", &names, &expect_names);

    // `.smrt`: regions are the block names; region volumes from the tracker's cells.
    let mut rt_vol: BTreeMap<String, f64> = BTreeMap::new();
    let tracker = match read_bytes(smrt_path).and_then(|b| sm_rt::read_smrt(&b).map_err(e)).and_then(|i| sm_rt::load_image(&i).map_err(e)) {
        Ok(t) => t,
        Err(x) => {
            rows.fail(3, format!("pin.smrt: {x}"));
            return;
        }
    };
    {
        let regions = tracker.regions();
        let mut names: Vec<&str> = regions.0.iter().map(|r| r.name.as_str()).collect();
        names.sort_unstable();
        rows.eq(3, ".smrt regions are the block names", &names, &expect_names);
        for c in 0..tracker.info().cells {
            let id = sm_rt_api::CellId::new(c);
            if let (Some(r), Some(v)) = (tracker.cell_region(id), tracker.cell_volume(id)) {
                *rt_vol.entry(regions.0[r.get()].name.clone()).or_default() += v;
            }
        }
    }

    // Tet block volumes: the owner whose model volume is the block's closed form.
    let tet_vol: BTreeMap<String, f64> = match vcheck {
        Ok(vc) => {
            let mut out = BTreeMap::new();
            for r in vc["regions"].as_array().cloned().unwrap_or_default() {
                let model = r["model_volume"].as_f64().unwrap_or(f64::NAN);
                let k = want.iter().position(|w| (w - model).abs() < 1e-9 * w);
                rows.check(2, k.is_some(), || format!("model volume {model} of {} is not a closed form of the deck", r["region"]));
                if let Some(k) = k {
                    out.insert(NAMES[k].to_owned(), r["mesh_volume"].as_f64().unwrap_or(f64::NAN));
                    println!("NOTE Q2 {}: model {model:.9} mesh {:.9} deficit {:.3e}", NAMES[k], r["mesh_volume"].as_f64().unwrap_or(f64::NAN), r["deficit"].as_f64().unwrap_or(f64::NAN));
                }
            }
            rows.eq(2, "four regions in the volume check", &out.len(), &4);
            out
        }
        Err(x) => {
            rows.fail(2, x);
            BTreeMap::new()
        }
    };
    let get = |m: &BTreeMap<String, f64>, k: &str| m.get(k).copied().unwrap_or(f64::NAN);
    for (k, name) in NAMES.iter().enumerate() {
        // The model volumes are the closed forms (B-rep exactness, 1e-9 after the cuts).
        rows.near(2, &format!("tet {name} vs polyhedral {name}"), get(&tet_vol, name), get(&set_vol, name), 1e-12 * want[k].max(1.0));
        rows.near(2, &format!(".smsh set {name} vs .smrt region {name}"), get(&set_vol, name), get(&rt_vol, name), 1e-12 * want[k].max(1.0));
    }
    // Sum of the blocks is the box (the outer boundary is planar, so this is exact).
    rows.near(2, "blocks sum to the box", NAMES.iter().map(|n| get(&set_vol, n)).sum::<f64>(), box_volume, 1e-9 * box_volume);
    // Every node of a disc boundary is on its cylinder, so the discs are inside the circles.
    let mut cum = 0.0;
    for k in 0..3 {
        cum += get(&set_vol, NAMES[k]);
        let circle = PI * RADII[k] * RADII[k] * HEIGHT;
        rows.check(2, cum <= circle * (1.0 + 1e-12), || format!("the mesh inside radius {} is {cum} > the circle {circle}", RADII[k]));
        // Two-sided (ruling OR-j): not below the 36-gon prism by more than 0.55 h^2 H, h = 2 pi r / 36
        // (the curvature sizing of the script keeps the lateral triangles to about one interval).
        let h = 2.0 * PI * RADII[k] / 36.0;
        let floor = prism(RADII[k], 6.0, 30.0) - 0.55 * h * h * HEIGHT;
        rows.check(2, cum >= floor, || format!("the mesh inside radius {} is {cum} < the 36-gon prism less 0.55 h^2 H = {floor} (circle {circle})", RADII[k]));
        println!("NOTE Q2 mesh inside r={}: {cum:.6} circle {circle:.6} (excess {:.3e})", RADII[k], circle - cum);
    }
    println!("NOTE Q2 36-gon prisms (not asserted): fuel {:.6} void+fuel {:.6} to the r=0.46 {:.6}", prism(RADII[0], 6.0, 30.0), prism(RADII[1], 6.0, 30.0), prism(RADII[2], 6.0, 30.0));

    // Q4: the 600 oracle points through the `.smrt`.
    oracle_rows(rows, &tracker);

    // Q5: validate and gc_*.
    for (label, res) in validated {
        match res {
            Ok(v) => {
                rows.eq(5, &format!("mesh.validate {label} total"), &v["total"].as_u64(), &Some(0));
                rows.eq(5, &format!("mesh.validate {label} valid"), &v["valid"].as_bool(), &Some(true));
            }
            Err(x) => rows.fail(5, x),
        }
    }
    let viol = validate_geometry(pm, &GeomTol::default());
    rows.check(5, viol.is_empty(), || format!("validate_geometry of the polyhedral mesh: {} violations, first {:?}", viol.len(), viol.first()));
    match &smsh.geometry_cache {
        Some(gc) => {
            rows.eq(5, "gc_cell_volume length", &gc.cell_volume.len(), &pm.count(MeshEntityKind::Cell));
            let mut worst = 0.0f64;
            for (i, v) in gc.cell_volume.iter().enumerate() {
                let c = cell_volume(pm, CellIx::new(i));
                worst = worst.max((v - c).abs() / c.abs().max(1e-300));
            }
            rows.check(5, worst <= 1e-12, || format!("gc_cell_volume differs from the recomputed volume by {worst:e}"));
            let total: f64 = gc.cell_volume.iter().sum();
            rows.near(5, "gc_cell_volume sums to the box", total, box_volume, 1e-9 * box_volume);
            for set in &smsh.sets {
                if let SetMembers::Cells(c) = &set.members {
                    let g: f64 = c.iter().map(|cell| gc.cell_volume[cell.0 as usize]).sum();
                    rows.near(5, &format!("gc volume of set {}", set.name), g, get(&set_vol, &set.name), 1e-12 * box_volume);
                }
            }
        }
        None => rows.fail(5, "pin.smsh has no geometry cache (gc_*), ruling D2"),
    }
}

/// Q4. The mesh circles are inscribed, so a point's mesh region is OpenMC's or further out
/// (fuel, void, clad, water), never further in; the mismatches are printed and pinned.
fn oracle_rows(rows: &mut Rows, t: &sm_rt::MeshTracker) {
    let o: Value = match serde_json::from_str(&read_text(&repo().join("crates/sm-csgio/tests/fixtures/openmc/pin/oracle.json"))) {
        Ok(v) => v,
        Err(x) => {
            rows.fail(4, format!("oracle.json: {x}"));
            return;
        }
    };
    let mat = o["cell_material"].as_object().cloned().unwrap_or_default();
    let (mut compared, mut equal, mut outside, mut further_out, mut further_in) = (0usize, 0usize, 0usize, 0usize, 0usize);
    let mut min_margin = f64::INFINITY;
    let mut sc = Scratch::new();
    for p in o["points"].as_array().cloned().unwrap_or_default() {
        let q = p["p"].as_array().map(|a| a.iter().map(|x| x.as_f64().unwrap_or(f64::NAN)).collect::<Vec<_>>()).unwrap_or_default();
        let Some(cell) = p["cell"].as_u64() else {
            outside += 1;
            continue;
        };
        let Some(want) = mat.get(&cell.to_string()).and_then(Value::as_str).and_then(region_rank) else {
            rows.fail(4, format!("OpenMC cell {cell} has no known material"));
            continue;
        };
        compared += 1;
        let got = t.locate(Point3::new(q[0], q[1], q[2]), None, None, &mut sc).ok().and_then(|l| t.regions().get(l.region).map(|r| r.name.clone())).and_then(|n| region_rank(&n));
        match got {
            Some(g) if g == want => equal += 1,
            Some(g) if g > want => {
                further_out += 1;
                // Band: inside the circle of OpenMC's region, outside its inscribed 36-gon apothem.
                let r = q[0].hypot(q[1]);
                let (hi, lo) = (RADII[want], RADII[want] * (PI / 36.0).cos());
                min_margin = min_margin.min((r - lo).min(hi - r));
                rows.check(4, r < hi && r > lo, || format!("point {q:?} (r = {r}) is in OpenMC region {} but the mesh says {}: outside the band ({lo}, {hi})", NAMES[want], NAMES[g]));
            }
            Some(_) => further_in += 1,
            None => rows.fail(4, format!("point {q:?} is outside the mesh but inside OpenMC cell {cell}")),
        }
    }
    rows.eq(4, "oracle points inside OpenMC cells + outside", &(compared + outside), &600);
    rows.eq(4, "points further in than OpenMC (inscribed mesh)", &further_in, &0);
    println!("NOTE Q4 600 points: {outside} outside the deck, {compared} compared, {equal} equal to OpenMC, {further_out} further out (inside the circle band), {further_in} further in");
    println!("NOTE Q4 band margin of the further-out points: {min_margin:.3e} (circle - r or r - apothem, smallest)");
    println!("MISMATCH w4_poly {further_out}");
    rows.eq(4, "pinned mismatch count", &further_out.to_string(), &PINS[6].to_owned());
}

/// Q7: record the project and replay the record in a fresh in-memory session that staged the deck
/// first (`session.stage` is not an op, so the record names the deck by sha256 only); then replay
/// with a strategy name no registry knows (the record holds the explicit assignments).
fn replay_rows(rows: &mut Rows, proj: &Path, cli: &Front) {
    let dir = scratch("replay");
    if let Err(x) = with_deck(&dir) {
        rows.fail(7, x);
        return;
    }
    let rec = dir.join("rec.smesh");
    if let Err(x) = ok_text(&["-p", &ts(proj), "record", "--forks", "refs", "--verify", "-o", &ts(&rec)], Some(proj), Threads::Inherit) {
        rows.fail(7, format!("smesh record --verify: {x}"));
        return;
    }
    let text = read_text(&rec);
    let auto: Vec<&str> = text.lines().filter(|l| l.starts_with("mesh.auto(")).collect();
    rows.eq(7, "one mesh.auto statement in the record", &auto.len(), &1);
    if let Some(l) = auto.first() {
        let n = l.matches("scheme:").count();
        rows.eq(7, "one explicit assignment per volume", &n, &4);
        rows.check(7, l.ends_with("strategy: \"tet\")"), || format!("mesh.auto record: {l}"));
    }
    rows.check(7, text.contains(GEOMETRY_SHA) && text.contains(MATERIALS_SHA), || "the record names the deck by sha256".into());
    let want = cli.prints.get("h_mesh").cloned().unwrap_or_default();
    let stage = "session.stage(\"geometry.xml\")\nsession.stage(\"materials.xml\")\n";
    let twin = text.replace("strategy: \"tet\")", "strategy: \"no_such_strategy\")");
    rows.check(7, twin != text, || "cannot build the unregistered-strategy twin".into());
    for (label, file, body) in [("replay", "replay.smesh", &text), ("replay under an unregistered strategy name", "ghost.smesh", &twin)] {
        let path = dir.join(file);
        if let Err(x) = std::fs::write(&path, format!("{stage}{body}")) {
            rows.fail(7, format!("{label}: {x}"));
            continue;
        }
        match ok_text(&["--memory", "run", &ts(&path), "--print-hash", "--author", AUTHOR, "--fixed-time", TIME], Some(&dir), Threads::Inherit) {
            Ok(out) => rows.eq(7, &format!("{label} reaches h_mesh"), &out.lines().last().unwrap_or("").to_owned(), &want),
            Err(x) => rows.fail(7, format!("{label}: {x}")),
        }
    }
}

/// Print every note and failure; panic once at the end if any row failed.
fn finish(rows: &Rows) {
    for (row, notes) in &rows.notes {
        for n in notes {
            println!("NOTE Q{row} {n}");
        }
    }
    for (row, fails) in &rows.fails {
        for f in fails {
            println!("FAIL Q{row} {f}");
        }
    }
    assert!(rows.fails.is_empty(), "failing rows: {:?}", rows.fails.keys().collect::<Vec<_>>());
}
