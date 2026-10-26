//! Stretch S1: cross-checks against `ncdump` and `scipy.io.netcdf_file`. Each test prints a skip
//! message and passes when the tool is missing.

mod common;

use common::*;
use sm_exodus::netcdf::NcFormat;
use sm_exodus::{ExodusOptions, exodus_bytes, exodus_nc, plan};
use sm_mesh_api::*;
use sm_testkit::meshes;
use std::path::PathBuf;
use std::process::Command;

fn tmp(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(name)
}

fn available(tool: &str, arg: &str) -> bool {
    Command::new(tool).arg(arg).output().is_ok_and(|o| o.status.success() || !o.stdout.is_empty() || !o.stderr.is_empty())
}

fn normalise(s: &str) -> String {
    s.split_whitespace().collect::<Vec<_>>().join(" ")
}

fn hex_with_sets() -> sm_mesh_api::reference::ReferenceMesh {
    let mut fx = meshes::single_hex().unwrap();
    let mut a = SerialAlloc::new();
    add_set(&mut fx.mesh, &mut a, "ns", None, verts(&[1, 2, 5, 6]));
    let f = face_with_vertices(&fx.mesh, &[1, 2, 5, 6]);
    add_set(&mut fx.mesh, &mut a, "ss", None, sides(&[left(f)]));
    fx.mesh
}

#[test]
fn ncdump_header_equals_cdl_header() {
    if !available("ncdump", "-h") {
        eprintln!("ncdump not available: skipping");
        return;
    }
    let m = hex_with_sets();
    for fmt in [NcFormat::Classic, NcFormat::Offset64, NcFormat::Data64] {
        let opts = ExodusOptions::default().format(fmt);
        let path = tmp(&format!("hex_{fmt:?}.exo"));
        std::fs::write(&path, exodus_bytes(&m, &opts).unwrap()).unwrap();
        let out = Command::new("ncdump").arg("-h").arg(&path).output().unwrap();
        assert!(out.status.success(), "ncdump failed: {}", String::from_utf8_lossy(&out.stderr));
        let theirs = String::from_utf8_lossy(&out.stdout).into_owned();
        let p = plan(&m, &opts).unwrap();
        let cdl = exodus_nc(&m, &opts, &p).unwrap().to_cdl();
        let head = &cdl[..cdl.find("data:").unwrap()];
        let ours = format!("{head}}}");
        // Drop the "netcdf <name> {" line (the name is the file's base name for ncdump).
        let strip = |s: &str| normalise(s.split_once('{').map_or(s, |x| x.1));
        assert_eq!(strip(&ours), strip(&theirs), "{fmt:?}");
    }
}

#[test]
fn scipy_reads_classic_and_offset64() {
    let ok = Command::new("python3").args(["-I", "-c", "import scipy.io"]).output().is_ok_and(|o| o.status.success());
    if !ok {
        eprintln!("python3 with scipy not available: skipping");
        return;
    }
    let m = meshes::mixed_transition().unwrap().mesh;
    let script = r#"
import sys, json
from scipy.io import netcdf_file
f = netcdf_file(sys.argv[1], mmap=False)
out = {"x": f.variables["coordx"][:].tolist(), "y": f.variables["coordy"][:].tolist(),
       "z": f.variables["coordz"][:].tolist(),
       "conn": [f.variables["connect%d" % k][:].flatten().tolist() for k in range(1, f.dimensions["num_el_blk"] + 1)]}
print(json.dumps(out))
"#;
    for fmt in [NcFormat::Classic, NcFormat::Offset64] {
        let path = tmp(&format!("mixed_{fmt:?}.exo"));
        std::fs::write(&path, exodus_bytes(&m, &ExodusOptions::default().format(fmt)).unwrap()).unwrap();
        let out = Command::new("python3").args(["-I", "-c", script]).arg(&path).output().unwrap();
        assert!(out.status.success(), "{}", String::from_utf8_lossy(&out.stderr));
        let text = String::from_utf8_lossy(&out.stdout).trim().to_string();
        // Python's json.dumps and Rust's `{:?}` both print the shortest round-trip double.
        let d = sm_exodus::read_exodus(&std::fs::read(&path).unwrap()).unwrap();
        let list = |v: &[f64]| format!("[{}]", v.iter().map(|x| format!("{x:?}")).collect::<Vec<_>>().join(", "));
        let ints = |v: &[i64]| format!("[{}]", v.iter().map(|x| x.to_string()).collect::<Vec<_>>().join(", "));
        let conn = d.blocks.iter().map(|b| ints(&b.connectivity)).collect::<Vec<_>>().join(", ");
        let want = format!(
            "{{\"x\": {}, \"y\": {}, \"z\": {}, \"conn\": [{conn}]}}",
            list(&d.coords[0]),
            list(&d.coords[1]),
            list(&d.coords[2])
        );
        assert_eq!(text, want, "{fmt:?}");
    }
}
