//! W4 pin milestone (plan `docs/design/w4/plan.md` section 4 W4-HEX-PIN-INT; `drafts/hexpin.md`
//! acceptance 1-5; ruling D4).
//!
//! `tools/w4_pin_ogrid.smesh` is the W3c through pin and its 3x3 lattice meshed with `hex_only`
//! (ogrid on discs and square annuli, sweep in z), n = 16 on every circle, then
//! `mesh.preserve_volume` (VOLFIT). `tools/w4_pin_enclosed_hex.smesh` is the enclosed pin cut by
//! the three steps `mesh.decompose_plan` returns, then `hex_only`, then VOLFIT. Collect-all style
//! like `milestone_w3c_pin`. Run it as
//! `cargo test -p sm-cli --test milestone_w4_pin -- --nocapture` (and with `RAYON_NUM_THREADS=1`).
//!
//! | row | what |
//! |---|---|
//! | H1 | through pin and lattice: all Hex, cell count equals `layers (n^2/16 + 2 n)`, 0 violations, fuel per pin `pi r^2 h` and moderator `(1.26^2 - pi r^2) h` to 1e-12 after VOLFIT; the same mesh before VOLFIT holds `h (n/2) r^2 sin(2 pi / n)` |
//! | H2 | enclosed pin: 6 volumes, all Hex, no `mesh.auto_tet` warning, 0 violations, the same fuel-volume checks |
//! | H3 | CLI (twice, 1 thread, project), MCP and in-process agree on prints and files; the `.smrt` and OpenFOAM exports have no warning |
//! | H4 | the hashes are printed for INT to pin in `tools/platform/expected.txt` (not asserted here) |
//! | H5 | lattice mesh is mirror symmetric in x and y through the lattice centre (1e-12) |
//! | H6 | `mesh.decompose_plan` gives equal replies and equal state hashes through in-process, CLI and MCP |
//!
//! The Python leg runs here too: an inline driver (written into the scratch dir at test time) runs
//! both scripts and the `mesh.decompose_plan` steps through `smartmesh.Session` from the installed
//! wheel (`W3C_PYTHON` / `W3B_PYTHON`, else `tools/w3b_python.sh`), and the printed hashes and
//! export digests are compared with the CLI front. A missing wheel is a skip, or a failure under
//! `CI=true` (as in `milestone_w3c_pin`).

mod w3a_support;

use std::collections::BTreeMap;
use std::f64::consts::PI;
use std::path::{Path, PathBuf};
use std::process::Command;

use serde_json::{Value, json};
use sm_core::EntityId;
use sm_layer_api::MeshLayer;
use sm_protocol::{Endpoint, ExportFormat, ExportResult, Request};
use sm_rt_api::{CellId, Tracker};
use sm_server::{Server, standard_with};

use w3a_support::*;

const KEYS: [&str; 4] = ["h_model", "h_auto", "h_linear", "h_mesh"];
const PIN_FILES: [&str; 2] = ["pin.smrt", "pin_foam"];
const ENCLOSED_FILES: [&str; 2] = ["enclosed.smrt", "enclosed_foam"];

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().expect("repo root")
}

fn script_path(name: &str) -> PathBuf {
    repo().join("tools").join(name)
}

fn script_text(name: &str) -> String {
    read_text(&script_path(name))
}

fn base() -> PathBuf {
    // One tree per test (the test thread's name), so a finishing test never removes another's files.
    let t = std::thread::current().name().unwrap_or("t").replace("::", "_");
    Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("milestone_w4_pin-{}", std::process::id())).join(t)
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
        let b = base();
        let _ = std::fs::remove_dir_all(&b);
        // The per-process parent, if this was the last test to finish.
        if let Some(p) = b.parent() {
            let _ = std::fs::remove_dir(p);
        }
    }
}

type Prints = BTreeMap<String, Vec<String>>;

fn prints_of(text: &str) -> Prints {
    let mut m: Prints = BTreeMap::new();
    for l in text.lines() {
        if let Some((k, v)) = l.split_once(' ')
            && k.chars().all(|c| c.is_ascii_lowercase() || c == '_')
        {
            m.entry(k.to_owned()).or_default().push(v.trim().to_owned());
        }
    }
    m
}

fn one(p: &Prints, k: &str) -> R<String> {
    p.get(k).and_then(|v| v.first()).cloned().ok_or_else(|| format!("no `{k}` line in the output"))
}

fn floats(p: &Prints, k: &str) -> R<f64> {
    let s = one(p, k)?;
    s.parse::<f64>().map_err(|x| format!("{k}: {s}: {x}"))
}

/// sha256 of a file, or of a directory (sha256 over its sorted `relpath sha256` lines).
fn digest(p: &Path) -> R<String> {
    if p.is_dir() {
        let lines: String = tree(p)?.iter().map(|(k, v)| format!("{k} {}\n", sha_hex(v))).collect();
        Ok(sha_hex(lines.as_bytes()))
    } else {
        Ok(sha_hex(&read_bytes(p)?))
    }
}

/// What one front end produced for one script.
struct Front {
    prints: Prints,
    files: BTreeMap<String, String>,
}

impl Front {
    fn new(output: &str, dir: &Path, files: &[&str]) -> R<Self> {
        let files = files.iter().map(|f| Ok(((*f).to_owned(), digest(&dir.join(f))?))).collect::<R<_>>()?;
        Ok(Self { prints: prints_of(output), files })
    }
}

// ------------------------------------------------------------------ front ends

fn cli_memory(script: &str, name: &str, threads: Threads, files: &[&str]) -> R<Front> {
    let dir = scratch(name);
    let args = ["--memory", "run", &ts(&script_path(script)), "--author", AUTHOR, "--fixed-time", TIME];
    let out = ok_text(&args, Some(&dir), threads)?;
    Front::new(&out, &dir, files)
}

/// The project run: files land in the project root; returns the project directory.
fn cli_project(script: &str, name: &str) -> R<PathBuf> {
    let dir = scratch(name);
    let proj = dir.join("proj");
    ok_text(&["new", &ts(&proj)], Some(&dir), Threads::Inherit)?;
    ok_text(&["-p", &ts(&proj), "run", &ts(&script_path(script)), "--author", AUTHOR, "--fixed-time", TIME], Some(&dir), Threads::Inherit)?;
    Ok(proj)
}

/// `smesh -p proj export ...`: exit code, stdout and stderr.
fn cli_export(proj: &Path, args: &[&str]) -> R<(i32, String, String)> {
    let p = ts(proj);
    let mut a = vec!["-p", p.as_str(), "export"];
    a.extend_from_slice(args);
    let o = spawn(&a, Some(proj), Threads::Inherit, None)?;
    Ok((o.status.code().unwrap_or(-1), String::from_utf8_lossy(&o.stdout).into_owned(), String::from_utf8_lossy(&o.stderr).into_owned()))
}

fn mcp_front(script: &str, name: &str, files: &[&str]) -> R<Front> {
    let dir = scratch(name);
    let mut lines = mcp_init();
    lines.push(tool_call(2, "run_script", json!({"text": script_text(script)})));
    lines.push(tool_call(3, "session_hash", json!({})));
    let replies = mcp_session(&PIN, &lines, Threads::Inherit, Some(&dir))?;
    let run = tool_ok(&replies, 2)?;
    let hash = tool_ok(&replies, 3)?;
    let out = run["output"].as_str().ok_or_else(|| format!("run_script has no output: {run}"))?;
    let front = Front::new(out, &dir, files)?;
    if hash["state"].as_str() != Some(one(&front.prints, "h_mesh")?.as_str()) {
        return Err(format!("MCP session_hash {hash} != printed h_mesh"));
    }
    Ok(front)
}

/// An in-process server in `dir/proj` with `text` run in it.
fn in_process_text(text: &str, dir: &Path) -> R<(Server, Value, PathBuf)> {
    let proj = dir.join("proj");
    let mut s = Server::create(standard_with(Some(AUTHOR.into()), Some(TIME.into())), &proj, "w4").map_err(e)?;
    let v = s
        .call(Request::ScriptRun { text: text.to_owned(), path: None, includes: Default::default() })
        .map_err(|x| format!("script.run: {x} {}", x.data["rendered"].as_str().unwrap_or("")))?;
    Ok((s, v, proj))
}

fn in_process(script: &str, name: &str, files: &[&str]) -> R<(Front, Server, PathBuf)> {
    let (mut s, v, proj) = in_process_text(&script_text(script), &scratch(name))?;
    let out = v["output"].as_str().ok_or_else(|| format!("script.run has no output: {v}"))?;
    let f = Front::new(out, &proj, files)?;
    let h = s.call(Request::Hash { at: None }).map_err(e)?;
    if h["state"].as_str() != Some(one(&f.prints, "h_mesh")?.as_str()) {
        return Err(format!("in-process session.hash {} != printed h_mesh", h["state"]));
    }
    Ok((f, s, proj))
}

fn export_in_process(s: &mut Server, path: &Path, format: Option<ExportFormat>) -> R<ExportResult> {
    let v = s
        .call(Request::Export {
            path: Some(ts(path)),
            format,
            mesh: None,
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

// ------------------------------------------------------------------ Python leg

/// Inline driver. `pin <tools> <out>`: both scripts through `Session.memory`, printing `<tag>_<key>`
/// and `<tag>_file <name> <digest>` lines. `plan <model> <imprint>`: H6 through `Session`.
const PY_DRIVER: &str = r#"
import hashlib, json, os, sys
import smartmesh as sm

TIME = sys.argv[1]
AUTHOR = sys.argv[2]
MODE = sys.argv[3]

def sha(path):
    if os.path.isdir(path):
        rows = []
        for d, _, names in os.walk(path):
            for n in names:
                p = os.path.join(d, n)
                with open(p, "rb") as fh:
                    rows.append(os.path.relpath(p, path).replace(os.sep, "/") + " " + hashlib.sha256(fh.read()).hexdigest() + "\n")
        return hashlib.sha256("".join(sorted(rows)).encode()).hexdigest()
    with open(path, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()

def printed(output):
    out = {}
    for line in output.splitlines():
        k, _, rest = line.partition(" ")
        out.setdefault(k, []).append(rest.strip())
    return out

def fail(msg):
    print("FAIL " + msg)
    sys.exit(1)

def new():
    return sm.Session.memory(author=AUTHOR, fixed_time=TIME)

if MODE == "pin":
    tools, out = sys.argv[4], sys.argv[5]
    os.makedirs(out, exist_ok=True)
    os.chdir(out)
    for tag, script, files in (("ogrid", "w4_pin_ogrid.smesh", ["pin.smrt", "pin_foam"]), ("enclosed", "w4_pin_enclosed_hex.smesh", ["enclosed.smrt", "enclosed_foam"])):
        s = new()
        with open(os.path.join(tools, script), encoding="utf-8") as fh:
            r = s.run_script(fh.read())
        p = printed(r.output)
        if s.hash().state != p["h_mesh"][0]:
            fail(script + ": session hash differs from h_mesh")
        for k in ("h_model", "h_auto", "h_linear", "h_mesh"):
            print(tag + "_" + k + " " + p[k][0])
        for f in files:
            print(tag + "_file " + f + " " + sha(f))
else:
    model, imprint = open(sys.argv[4], encoding="utf-8").read(), open(sys.argv[5], encoding="utf-8").read()
    s = new()
    s.run_script(model)
    h0 = s.hash().state
    print("h_model " + h0)
    plan = s.query("mesh.decompose_plan")
    if s.hash().state != h0:
        fail("the query moved the state hash")
    print("plan " + json.dumps(plan, sort_keys=True))
    host = [r for r in plan["volumes"] if r["status"] == "cuts"]
    if len(host) != 1:
        fail("%d rows with cuts" % len(host))
    outs = []
    for st in host[0]["steps"]:
        ids = []
        for t in st["targets"]:
            if "id" in t:
                ids.append(t["id"])
            else:
                ids.extend(outs[t["step"]][t["role"]])
        params = {k: v for k, v in st.items() if k not in ("targets", "op", "step")}
        params["targets"] = ids
        r = s.apply(st["op"], **params)
        outs.append(r.raw["delta"]["outputs"])
    s.run_script(imprint)
    print("h_final " + s.hash().state)
print("OK")
"#;

/// The venv python with the wheel installed, or the reason to skip (maturin unavailable).
fn python() -> R<Result<PathBuf, String>> {
    // Built once per test process (the two tests share the wheel).
    static PY: std::sync::OnceLock<R<Result<PathBuf, String>>> = std::sync::OnceLock::new();
    PY.get_or_init(python_uncached).clone()
}

fn python_uncached() -> R<Result<PathBuf, String>> {
    for var in ["W3C_PYTHON", "W3B_PYTHON"] {
        if let Some(p) = std::env::var_os(var) {
            return Ok(Ok(PathBuf::from(p)));
        }
    }
    let venv = Path::new(env!("CARGO_TARGET_TMPDIR")).join("w4-venv");
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

/// With `CI=true` a missing Python wheel (maturin) is a failure, not a skip.
fn python_skip_is_fatal(ci: Option<&str>) -> bool {
    ci == Some("true")
}

/// Runs the inline driver; returns its output lines (which end in `OK`).
fn python_driver(py: &Path, name: &str, mode_args: &[String]) -> R<Vec<String>> {
    let dir = scratch(name);
    let drv = dir.join("driver.py");
    std::fs::write(&drv, PY_DRIVER).map_err(e)?;
    let o = Command::new(py).arg("-I").arg(&drv).arg(TIME).arg(AUTHOR).args(mode_args).output().map_err(|x| format!("run python driver: {x}"))?;
    let out = String::from_utf8_lossy(&o.stdout).into_owned();
    if o.status.code() != Some(0) || !out.lines().any(|l| l == "OK") {
        return Err(format!("python driver exited {:?}:\n{out}\n{}", o.status.code(), String::from_utf8_lossy(&o.stderr)));
    }
    Ok(out.lines().map(str::to_owned).collect())
}

/// The Python lines, or `None` after recording a skip (a failure under `CI=true`, or a setup error) on `rows`.
fn python_lines(rows: &mut Rows, tags: &[u8], name: &str, mode_args: &[String]) -> Option<Vec<String>> {
    let res = match python() {
        Ok(Ok(py)) => python_driver(&py, name, mode_args),
        Ok(Err(reason)) => {
            let fatal = python_skip_is_fatal(std::env::var("CI").ok().as_deref());
            println!("{} H{tags:?} Python leg (wheel): {reason}", if fatal { "FAIL" } else { "SKIP" });
            for t in tags {
                if fatal {
                    rows.fail(*t, format!("Python wheel unavailable under CI=true: {reason}"));
                } else {
                    rows.note(*t, format!("Python skipped: {reason}"));
                }
            }
            return None;
        }
        Err(x) => Err(x),
    };
    match res {
        Ok(l) => Some(l),
        Err(x) => {
            for t in tags {
                rows.fail(*t, format!("Python leg: {x}"));
            }
            None
        }
    }
}

fn line_after(lines: &[String], prefix: &str) -> String {
    lines.iter().find_map(|l| l.strip_prefix(prefix).and_then(|r| r.strip_prefix(' '))).map(str::to_owned).unwrap_or_default()
}

/// Python's hashes and export digests for one script against the CLI front.
fn compare_python(rows: &mut Rows, tag: u8, script: &str, py_tag: &str, lines: &[String], cli: &Front) {
    for k in KEYS {
        rows.eq(tag, &format!("{script}: python {k} vs CLI"), &line_after(lines, &format!("{py_tag}_{k}")), &one(&cli.prints, k).unwrap_or_default());
    }
    for (f, d) in &cli.files {
        rows.eq(tag, &format!("{script}: python {f} vs CLI"), &line_after(lines, &format!("{py_tag}_file {f}")), d);
    }
}

// ------------------------------------------------------------------ helpers over results

/// Volume per region of a `.smrt`, summed from the tracker's cell volumes (regions in table order).
fn region_volumes(smrt: &Path) -> R<Vec<(String, f64)>> {
    let image = sm_rt::read_smrt(&read_bytes(smrt)?).map_err(e)?;
    let loaded = sm_rt::load_image(&image).map_err(e)?;
    let mut vols = vec![0.0; loaded.regions().0.len()];
    for c in 0..loaded.info().cells {
        let id = CellId::new(c);
        let r = loaded.cell_region(id).ok_or_else(|| format!("cell {c} has no region"))?;
        vols[r.get()] += loaded.cell_volume(id).ok_or_else(|| format!("cell {c} has no volume"))?;
    }
    Ok(loaded.regions().0.iter().zip(vols).map(|(r, v)| (r.name.clone(), v)).collect())
}

/// The side count `n` of the regular polygon inscribed in radius `r` whose prism of height `h`
/// has volume `v`: `v = (n/2) r^2 sin(2 pi / n) h`, matched to 1e-9 relative. `None` unless
/// exactly one `n` in 3..=256 fits.
fn polygon_sides(v: f64, r: f64, h: f64) -> Option<usize> {
    let fits: Vec<usize> = (3..=256usize)
        .filter(|&n| {
            let want = 0.5 * n as f64 * r * r * (2.0 * PI / n as f64).sin() * h;
            (v - want).abs() <= 1e-9 * want
        })
        .collect();
    (fits.len() == 1).then(|| fits[0])
}

/// Axial layers of a sweep over height `h` at mesh size `size`: `max(1, round(h / size))`.
fn layers(h: f64, size: f64) -> usize {
    ((h / size).round() as usize).max(1)
}

/// Quads of one ogrid disc (`(n/4)^2` core plus `k n` collar) and one ogrid annulus (`k n`),
/// per axial layer, with one ring (`k = 1`, the default at n = 16 and size 0.3).
fn ogrid_cells_per_layer(n: usize) -> (usize, usize) {
    ((n / 4) * (n / 4) + n, n)
}

fn info(s: &mut Server) -> Value {
    s.call(Request::Query { name: "mesh.info".into(), input: json!({"mesh": "Mesh#1"}), at: None }).expect("info")
}

fn by_owner(i: &Value) -> Vec<f64> {
    i["volume_by_owner"].as_object().expect("volume_by_owner").values().map(|v| v.as_f64().expect("f64")).collect()
}

/// Node coordinates of mesh `Mesh#1` of an in-process server.
fn nodes(s: &Server) -> R<Vec<[f64; 3]>> {
    let st = s.workspace().state();
    let layer = st.layer::<MeshLayer>().ok_or("no meshes layer")?;
    let id: EntityId = "Mesh#1".parse().map_err(|x| format!("Mesh#1: {x:?}"))?;
    let m = layer.get(id).ok_or("Mesh#1 not in the meshes layer")?.mesh();
    let c = m.coords();
    Ok((0..c.len()).map(|i| [c.x[i], c.y[i], c.z[i]]).collect())
}

/// Every point of `pts`, reflected by `map`, is a point of `pts` within `tol` (and the counts agree,
/// so the map is a bijection on the set).
fn invariant_under(pts: &[[f64; 3]], map: impl Fn([f64; 3]) -> [f64; 3], tol: f64) -> Result<(), String> {
    for p in pts {
        let q = map(*p);
        if !pts.iter().any(|r| (r[0] - q[0]).abs() <= tol && (r[1] - q[1]).abs() <= tol && (r[2] - q[2]).abs() <= tol) {
            return Err(format!("reflected node {q:?} of {p:?} is not a node"));
        }
    }
    Ok(())
}

fn auto_tet_warnings(run: &Value) -> usize {
    run["events"]
        .as_array()
        .map(|ev| ev.iter().filter(|e| e["op"] == "mesh.auto").flat_map(|e| e["delta"]["diagnostics"].as_array().cloned().unwrap_or_default()).filter(|d| d["code"] == "mesh.auto_tet").count())
        .unwrap_or(0)
}

// ------------------------------------------------------------------ H1 .. H5

/// One script through every front end; returns the first CLI front (the others are compared to it)
/// and the in-process server for the content checks.
fn all_fronts(rows: &mut Rows, tag: u8, script: &str, files: &[&str]) -> Option<(Front, Server, PathBuf)> {
    let name = script.trim_end_matches(".smesh");
    let cli = cli_memory(script, &format!("{name}-cli"), Threads::Default, files);
    let again = cli_memory(script, &format!("{name}-again"), Threads::Default, files);
    let one_t = cli_memory(script, &format!("{name}-1t"), Threads::One, files);
    let mcp = mcp_front(script, &format!("{name}-mcp"), files);
    let inproc = in_process(script, &format!("{name}-inproc"), files);
    let cli = match cli {
        Ok(c) => c,
        Err(x) => {
            rows.fail(tag, format!("{script}: CLI run failed: {x}"));
            return None;
        }
    };
    for (label, other) in [("CLI again", again.as_ref().map(|f| (&f.prints, &f.files))), ("CLI 1 thread", one_t.as_ref().map(|f| (&f.prints, &f.files))), ("MCP", mcp.as_ref().map(|f| (&f.prints, &f.files)))] {
        match other {
            Ok((p, f)) => {
                rows.eq(tag, &format!("{script}: {label} prints"), p, &cli.prints);
                rows.eq(tag, &format!("{script}: {label} files"), f, &cli.files);
            }
            Err(x) => rows.fail(tag, format!("{script}: {label}: {x}")),
        }
    }
    match inproc {
        Ok((f, s, proj)) => {
            rows.eq(tag, &format!("{script}: in-process prints"), &f.prints, &cli.prints);
            rows.eq(tag, &format!("{script}: in-process files"), &f.files, &cli.files);
            Some((cli, s, proj))
        }
        Err(x) => {
            rows.fail(tag, format!("{script}: in-process: {x}"));
            None
        }
    }
}

/// The checks both scripts share: the printed hashes exist and are distinct per stage (printed for INT).
fn hashes(rows: &mut Rows, tag: u8, script: &str, f: &Front) {
    for k in KEYS {
        match one(&f.prints, k) {
            Ok(h) => {
                rows.check(tag, h.len() == 64, || format!("{script}: {k} is not a sha256: {h}"));
                println!("HASH {script} {k} {h}");
            }
            Err(x) => rows.fail(tag, x),
        }
    }
}

/// CLI project exports of `.smrt` and OpenFOAM: exit 0 and nothing on stderr (no warning).
fn exports_are_silent(rows: &mut Rows, tag: u8, script: &str, name: &str) {
    match cli_project(script, name) {
        Ok(proj) => {
            let out = scratch(&format!("{name}-out"));
            for args in [vec![ts(&out.join("x.smrt"))], vec![ts(&out.join("x_foam")), "--format".to_owned(), "openfoam".to_owned()]] {
                let a: Vec<&str> = args.iter().map(String::as_str).collect();
                match cli_export(&proj, &a) {
                    Ok((0, stdout, stderr)) => {
                        rows.check(tag, !stderr.contains("warning"), || format!("{script}: export {a:?} printed on stderr: {stderr}"));
                        rows.check(tag, stdout.starts_with("wrote "), || format!("{script}: stdout: {stdout}"));
                    }
                    other => rows.fail(tag, format!("{script}: smesh export {a:?}: {other:?}")),
                }
            }
        }
        Err(x) => rows.fail(tag, format!("{script}: project run: {x}")),
    }
}

#[test]
fn milestone_w4_pin() {
    let _cleanup = Cleanup;
    let mut rows = Rows::default();
    let (r, h) = (0.4_f64, 1.0_f64);
    let disk = PI * r * r;
    let py_args = ["pin".to_owned(), ts(&repo().join("tools")), ts(&scratch("python-out"))];
    let py = python_lines(&mut rows, &[3], "python-pin", &py_args);

    // ---------------- H1: the through pin and its lattice
    if let Some((cli, mut s, proj)) = all_fronts(&mut rows, 1, "w4_pin_ogrid.smesh", &PIN_FILES) {
        hashes(&mut rows, 4, "w4_pin_ogrid", &cli);
        if let Some(l) = &py {
            compare_python(&mut rows, 3, "w4_pin_ogrid", "ogrid", l, &cli);
        }
        let n = floats(&cli.prints, "poly_volume").ok().and_then(|v| polygon_sides(v / 1.0, r, h)).unwrap_or(0);
        rows.eq(1, "circle intervals (n-gon of the linear fuel)", &n, &16);
        let (core, ring) = ogrid_cells_per_layer(n.max(4));
        let l = layers(h, 0.3);
        // 9 discs (core + collar) and 9 annuli, each `l` layers.
        let want = 9 * l * (core + ring);
        rows.eq(1, "cells = 9 layers (n^2/16 + 2 n)", &one(&cli.prints, "cells").unwrap_or_default(), &want.to_string());
        rows.eq(1, "shapes", &one(&cli.prints, "shapes").unwrap_or_default(), &format!("{{Hex: {want}}}"));
        rows.eq(1, "violations", &one(&cli.prints, "violations").unwrap_or_default(), &"0".to_owned());
        rows.eq(1, "head", &one(&cli.prints, "head").unwrap_or_default(), &"main".to_owned());
        // Linear mesh (second assertion, same mesh, before VOLFIT): the polygon value.
        let poly = 0.5 * n as f64 * r * r * (2.0 * PI / n.max(1) as f64).sin() * h;
        rows.near(1, "linear fuel pin = h (n/2) r^2 sin(2 pi / n)", floats(&cli.prints, "fuel_volume_linear").unwrap_or(f64::NAN), poly, 1e-12);
        // After VOLFIT: pi r^2 h, and the total is the lattice.
        rows.near(1, "fuel pin after VOLFIT = pi r^2 h", floats(&cli.prints, "fuel_volume").unwrap_or(f64::NAN), disk * h, 1e-12);
        rows.near(1, "lattice volume", floats(&cli.prints, "mesh_volume").unwrap_or(f64::NAN), 9.0 * 1.26 * 1.26 * h, 1e-12);
        // Per owner, from the query (not the script's own words).
        let i = info(&mut s);
        rows.eq(1, "mesh.info cell_shapes", &i["cell_shapes"]["Hex"].as_u64(), &Some(want as u64));
        let mut vols = by_owner(&i);
        vols.sort_by(f64::total_cmp);
        rows.eq(1, "volumes in volume_by_owner", &vols.len(), &18);
        let (fuels, mods) = vols.split_at(9);
        let want_mod = 1.26 * 1.26 * h - disk * h;
        for v in fuels {
            rows.near(1, "fuel volume", *v, disk * h, 1e-12);
        }
        for v in mods {
            rows.near(1, "moderator volume", *v, want_mod, 1e-12);
        }
        rows.near(1, "sum of owners = volume", vols.iter().sum::<f64>(), i["volume"].as_f64().unwrap_or(f64::NAN), 1e-12);
        // The .smrt regions are the blocks, with the same volumes.
        match region_volumes(&proj.join("pin.smrt")) {
            Ok(v) => {
                let find = |name: &str| v.iter().find(|(n, _)| n == name).map(|x| x.1).unwrap_or(f64::NAN);
                rows.near(1, ".smrt fuel", find("fuel"), 9.0 * disk * h, 1e-10);
                rows.near(1, ".smrt moderator", find("moderator"), 9.0 * want_mod, 1e-10);
            }
            Err(x) => rows.fail(1, format!("pin.smrt: {x}")),
        }
        // H5: mirror symmetry through the lattice centre (1.26, 1.26).
        match nodes(&s) {
            Ok(pts) => {
                rows.check(5, pts.len() > 100, || format!("{} nodes", pts.len()));
                for (axis, label) in [(0usize, "x"), (1, "y")] {
                    let res = invariant_under(
                        &pts,
                        |mut p| {
                            p[axis] = 2.52 - p[axis];
                            p
                        },
                        1e-12,
                    );
                    if let Err(x) = res {
                        rows.fail(5, format!("{label} reflection: {x}"));
                    }
                }
            }
            Err(x) => rows.fail(5, x),
        }
        exports_are_silent(&mut rows, 3, "w4_pin_ogrid.smesh", "ogrid-proj");
    }

    // ---------------- H2: the enclosed pin
    if let Some((cli, mut s, proj)) = all_fronts(&mut rows, 2, "w4_pin_enclosed_hex.smesh", &ENCLOSED_FILES) {
        hashes(&mut rows, 4, "w4_pin_enclosed_hex", &cli);
        if let Some(l) = &py {
            compare_python(&mut rows, 3, "w4_pin_enclosed_hex", "enclosed", l, &cli);
        }
        let hf = 0.8;
        rows.eq(2, "volumes after the cuts", &one(&cli.prints, "volumes").unwrap_or_default(), &"6".to_owned());
        let n = floats(&cli.prints, "poly_volume").ok().and_then(|v| polygon_sides(v, r, hf)).unwrap_or(0);
        rows.eq(2, "circle intervals", &n, &16);
        let (core, ring) = ogrid_cells_per_layer(n.max(4));
        // The fuel (0.8) and two caps (0.1 each) are discs; three square-annulus slabs of the same heights.
        let l = layers(0.8, 0.3) + 2 * layers(0.1, 0.3);
        let want = l * (core + ring);
        rows.eq(2, "cells = layers (n^2/16 + 2 n)", &one(&cli.prints, "cells").unwrap_or_default(), &want.to_string());
        rows.eq(2, "shapes (all Hex, no tet)", &one(&cli.prints, "shapes").unwrap_or_default(), &format!("{{Hex: {want}}}"));
        rows.eq(2, "violations", &one(&cli.prints, "violations").unwrap_or_default(), &"0".to_owned());
        let poly = 0.5 * n as f64 * r * r * (2.0 * PI / n.max(1) as f64).sin() * hf;
        rows.near(2, "linear fuel = h (n/2) r^2 sin(2 pi / n)", floats(&cli.prints, "fuel_volume_linear").unwrap_or(f64::NAN), poly, 1e-12);
        rows.near(2, "fuel after VOLFIT = pi r^2 h", floats(&cli.prints, "fuel_volume").unwrap_or(f64::NAN), disk * hf, 1e-12);
        rows.near(2, "mesh volume", floats(&cli.prints, "mesh_volume").unwrap_or(f64::NAN), 1.26 * 1.26 * h, 1e-12);
        let i = info(&mut s);
        let vols = by_owner(&i);
        rows.eq(2, "volumes in volume_by_owner", &vols.len(), &6);
        rows.near(2, "sum of owners = volume", vols.iter().sum::<f64>(), 1.26 * 1.26 * h, 1e-12);
        let fuel = vols.iter().copied().filter(|v| (v / (disk * hf) - 1.0).abs() < 1e-9).count();
        rows.eq(2, "volumes equal to pi r^2 h (the fuel)", &fuel, &1);
        // The moderator regions sum to the brick minus the fuel.
        match region_volumes(&proj.join("enclosed.smrt")) {
            Ok(v) => {
                let find = |name: &str| v.iter().find(|(n, _)| n == name).map(|x| x.1).unwrap_or(f64::NAN);
                rows.near(2, ".smrt fuel", find("fuel"), disk * hf, 1e-10);
                rows.near(2, ".smrt moderator", find("moderator"), 1.26 * 1.26 * h - disk * hf, 1e-10);
            }
            Err(x) => rows.fail(2, format!("enclosed.smrt: {x}")),
        }
        // No `mesh.auto_tet` warning anywhere in the run, and the exports are silent.
        let rerun = in_process_text(&script_text("w4_pin_enclosed_hex.smesh"), &scratch("enclosed-events"));
        match rerun {
            Ok((_, v, _)) => rows.eq(2, "mesh.auto_tet warnings", &auto_tet_warnings(&v), &0),
            Err(x) => rows.fail(2, format!("enclosed rerun: {x}")),
        }
        match export_in_process(&mut s, &proj.join("again.smrt"), None) {
            Ok(res) => rows.check(2, !res.notes.iter().any(|n| n.is_warning()), || format!(".smrt notes: {:?}", res.notes)),
            Err(x) => rows.fail(2, x),
        }
        match export_in_process(&mut s, &proj.join("again_foam"), Some(ExportFormat::Openfoam)) {
            Ok(res) => rows.check(2, !res.notes.iter().any(|n| n.is_warning()), || format!("OpenFOAM notes: {:?}", res.notes)),
            Err(x) => rows.fail(2, x),
        }
        exports_are_silent(&mut rows, 3, "w4_pin_enclosed_hex.smesh", "enclosed-proj");
    }

    finish(&rows);
}

// ------------------------------------------------------------------ H6: mesh.decompose_plan on the wire

const ENCLOSED_MODEL: &str = "let br = model.create_brick(x: 1.26, y: 1.26, z: 1.0)\nlet fu = model.create_cylinder(height: 0.8, radius: 0.4)\nlet tl = model.create_cylinder(height: 0.8, radius: 0.4)\nlet mo = model.subtract(targets: br.body, tools: tl.body)\nmodel.imprint(bodies: all bodies)\nmodel.merge()\n";
const IMPRINT_MERGE: &str = "model.imprint(bodies: all bodies)\nmodel.merge()\n";

/// `(op, params)` of a plan step with its targets resolved from the outputs of the earlier steps.
fn resolve(step: &Value, outs: &[Value]) -> R<(String, Value)> {
    let mut params = step.as_object().ok_or("step is not an object")?.clone();
    let targets = params.remove("targets").ok_or("step has no targets")?;
    params.remove("op");
    params.remove("step");
    let mut ids: Vec<Value> = Vec::new();
    for t in targets.as_array().ok_or("targets is not a list")? {
        if let Some(id) = t.get("id") {
            ids.push(id.clone());
        } else {
            let (st, role) = (t["step"].as_u64().ok_or("no step")? as usize, t["role"].as_str().ok_or("no role")?);
            ids.extend(outs.get(st).and_then(|o| o[role].as_array()).ok_or_else(|| format!("no output {role} of step {st}"))?.iter().cloned());
        }
    }
    params.insert("targets".into(), Value::Array(ids));
    Ok((step["op"].as_str().ok_or("step has no op")?.to_owned(), Value::Object(params)))
}

/// The host row of a `mesh.decompose_plan` reply (the one volume with cuts).
fn host_row(plan: &Value) -> R<Value> {
    let rows: Vec<&Value> = plan["volumes"].as_array().ok_or("no volumes")?.iter().filter(|r| r["status"] == "cuts").collect();
    if rows.len() != 1 {
        return Err(format!("{} rows with cuts: {plan}", rows.len()));
    }
    Ok(rows[0].clone())
}

#[test]
fn decompose_plan_gives_equal_replies_and_hashes_through_cli_and_mcp() {
    let _cleanup = Cleanup;
    let mut rows = Rows::default();

    // In-process: the reference. Query, then apply the steps as raw ops, resolving step references.
    let mut s = Server::new(standard_with(Some(AUTHOR.into()), Some(TIME.into()))).expect("server");
    s.call(Request::ScriptRun { text: ENCLOSED_MODEL.into(), path: None, includes: Default::default() }).expect("model");
    let h_model = s.call(Request::Hash { at: None }).expect("hash")["state"].as_str().unwrap_or_default().to_owned();
    let plan = s.call(Request::Query { name: "mesh.decompose_plan".into(), input: json!({}), at: None }).expect("plan");
    assert_eq!(s.call(Request::Hash { at: None }).expect("hash")["state"], json!(h_model), "a query leaves the state alone");
    let host = host_row(&plan).expect("host row");
    let steps: Vec<Value> = host["steps"].as_array().expect("steps").clone();
    assert_eq!(steps.len(), 3, "cylinder plus two planes: {host}");

    let mut outs: Vec<Value> = Vec::new();
    let mut raw: Vec<(String, Value)> = Vec::new();
    for st in &steps {
        let (op, params) = resolve(st, &outs).expect("resolve");
        let r = s.call(Request::Apply { op: op.clone(), params: params.clone(), version: None }).unwrap_or_else(|x| panic!("{op}: {x}"));
        outs.push(r["delta"]["outputs"].clone());
        raw.push((op, params));
    }
    s.call(Request::ScriptRun { text: IMPRINT_MERGE.into(), path: None, includes: Default::default() }).expect("imprint");
    let h_ref = s.call(Request::Hash { at: None }).expect("hash")["state"].as_str().unwrap_or_default().to_owned();
    // The script text the query returns reaches the same hash.
    let mut t = Server::new(standard_with(Some(AUTHOR.into()), Some(TIME.into()))).expect("server");
    t.call(Request::ScriptRun { text: format!("{ENCLOSED_MODEL}{}{IMPRINT_MERGE}", host["script"].as_str().unwrap_or_default()), path: None, includes: Default::default() }).expect("script");
    let h_script = t.call(Request::Hash { at: None }).expect("hash")["state"].as_str().unwrap_or_default().to_owned();
    rows.eq(6, "query script vs raw ops (in-process)", &h_script, &h_ref);

    // CLI: a project, the query over the wire, then each step through `smesh apply`.
    let cli = (|| -> R<(Value, String)> {
        let dir = scratch("plan-cli");
        let proj = dir.join("proj");
        let p = ts(&proj);
        ok_text(&["new", &p], Some(&dir), Threads::Inherit)?;
        let sp = dir.join("model.smesh");
        std::fs::write(&sp, ENCLOSED_MODEL).map_err(e)?;
        ok_text(&["-p", &p, "run", &ts(&sp), "--author", AUTHOR, "--fixed-time", TIME], Some(&dir), Threads::Inherit)?;
        let h0 = ok_text(&["-p", &p, "hash"], Some(&dir), Threads::Inherit)?.trim().to_owned();
        let q = ok_json(&["--json", "-p", &p, "query", "mesh.decompose_plan", "{}"], Some(&dir), Threads::Inherit)?;
        let h1 = ok_text(&["-p", &p, "hash"], Some(&dir), Threads::Inherit)?.trim().to_owned();
        if h0 != h1 {
            return Err(format!("CLI query moved the hash: {h0} -> {h1}"));
        }
        let mut couts: Vec<Value> = Vec::new();
        let hst = host_row(&q)?;
        for st in hst["steps"].as_array().ok_or("no steps")? {
            let (op, params) = resolve(st, &couts)?;
            let r = ok_json(&["--json", "-p", &p, "--author", AUTHOR, "--fixed-time", TIME, "apply", &op, &params.to_string()], Some(&dir), Threads::Inherit)?;
            couts.push(r["delta"]["outputs"].clone());
        }
        let sp2 = dir.join("im.smesh");
        std::fs::write(&sp2, IMPRINT_MERGE).map_err(e)?;
        ok_text(&["-p", &p, "run", &ts(&sp2), "--author", AUTHOR, "--fixed-time", TIME], Some(&dir), Threads::Inherit)?;
        Ok((q, ok_text(&["-p", &p, "hash"], Some(&dir), Threads::Inherit)?.trim().to_owned()))
    })();
    match cli {
        Ok((q, h)) => {
            rows.eq(6, "CLI plan reply equals in-process", &q, &plan);
            rows.eq(6, "CLI state hash equals in-process", &h, &h_ref);
        }
        Err(x) => rows.fail(6, format!("CLI leg: {x}")),
    }

    // MCP: one session; the query as a tool, the steps as tools with the in-process-resolved params
    // (ids are deterministic, and one stdin batch cannot chain outputs).
    let mcp = (|| -> R<(Value, String)> {
        let dir = scratch("plan-mcp");
        let mut lines = mcp_init();
        lines.push(tool_call(2, "run_script", json!({"text": ENCLOSED_MODEL})));
        lines.push(tool_call(3, "mesh__decompose_plan", json!({})));
        for (k, (op, params)) in raw.iter().enumerate() {
            lines.push(tool_call(10 + k as i64, &op.replace('.', "__"), params.clone()));
        }
        lines.push(tool_call(20, "run_script", json!({"text": IMPRINT_MERGE})));
        lines.push(tool_call(21, "session_hash", json!({})));
        let replies = mcp_session(&PIN, &lines, Threads::Inherit, Some(&dir))?;
        tool_ok(&replies, 2)?;
        let q = tool_ok(&replies, 3)?;
        for k in 0..raw.len() {
            tool_ok(&replies, 10 + k as i64)?;
        }
        tool_ok(&replies, 20)?;
        let h = tool_ok(&replies, 21)?;
        Ok((q, h["state"].as_str().ok_or("no state hash")?.to_owned()))
    })();
    match mcp {
        Ok((q, h)) => {
            rows.eq(6, "MCP plan reply equals in-process", &q, &plan);
            rows.eq(6, "MCP state hash equals in-process", &h, &h_ref);
        }
        Err(x) => rows.fail(6, format!("MCP leg: {x}")),
    }

    // Python: the query reply and the state hashes through `smartmesh.Session`.
    let dir = scratch("plan-py-files");
    let (mp, ip) = (dir.join("model.smesh"), dir.join("imprint.smesh"));
    let wrote = std::fs::write(&mp, ENCLOSED_MODEL).and_then(|()| std::fs::write(&ip, IMPRINT_MERGE));
    match wrote {
        Ok(()) => {
            let args = ["plan".to_owned(), ts(&mp), ts(&ip)];
            if let Some(l) = python_lines(&mut rows, &[6], "plan-py", &args) {
                rows.eq(6, "Python model hash equals in-process", &line_after(&l, "h_model"), &h_model);
                let q: R<Value> = serde_json::from_str(&line_after(&l, "plan")).map_err(e);
                match q {
                    Ok(q) => rows.eq(6, "Python plan reply equals in-process", &q, &plan),
                    Err(x) => rows.fail(6, format!("Python plan reply: {x}")),
                }
                rows.eq(6, "Python state hash equals in-process", &line_after(&l, "h_final"), &h_ref);
            }
        }
        Err(x) => rows.fail(6, format!("write python inputs: {x}")),
    }
    finish(&rows);
}

/// Print every note and failure; panic once at the end if any row failed.
fn finish(rows: &Rows) {
    for (row, notes) in &rows.notes {
        for n in notes {
            println!("NOTE H{row} {n}");
        }
    }
    for (row, fails) in &rows.fails {
        for f in fails {
            println!("FAIL H{row} {f}");
        }
    }
    assert!(rows.fails.is_empty(), "failing rows: {:?}", rows.fails.keys().collect::<Vec<_>>());
}
