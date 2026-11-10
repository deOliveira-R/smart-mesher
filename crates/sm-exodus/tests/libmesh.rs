//! W4-EXO-VERIFY: does a real reader (libMesh `meshtool-opt`, the reader MOOSE and Griffin use)
//! take our files? The tool comes from the user's moose env; every test prints a skip message and
//! passes when it is missing. Set `SM_LIBMESH_BIN` to the `meshtool-opt` path, or `CONDA_PREFIX`
//! to a moose env (`$CONDA_PREFIX/libmesh/bin/meshtool-opt`).
//!
//! Each test skips green when `meshtool-opt` is missing, so the libMesh result below is pinned
//! only on the user's Mac (moose env); machines without it neither confirm nor refute it.
//!
//! Recorded result (libMesh 1.9.0-pre, moose env, 2026-11-09): the control file (hex, wedge,
//! pyramid, tet blocks) is read; the `NFACED` file is refused with
//! `map_find() error: key "NFACED" not found` at `exodusII_io_helper.C:560`
//! (`ExodusII_IO_Helper::get_conversion`). So libMesh, hence MOOSE and Griffin through Exodus,
//! cannot read our polyhedra, and `ExodusOptions::polyhedra` stays experimental.

use sm_exodus::{ExodusOptions, exodus_bytes};
use sm_testkit::meshes;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};

fn meshtool() -> Option<PathBuf> {
    let p = std::env::var_os("SM_LIBMESH_BIN")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("CONDA_PREFIX").map(|c| PathBuf::from(c).join("libmesh/bin/meshtool-opt")))?;
    p.is_file().then_some(p)
}

/// Run `meshtool-opt -i input -o out`. The binary's rpath misses `libmesh/lib` (libnglib), so the
/// loader path is set from the tool's own env prefix.
fn run(tool: &Path, dir: &Path, input: &Path, out: &Path) -> Output {
    let prefix = tool.parent().and_then(Path::parent).expect("tool path has a libmesh prefix");
    let libs = format!("{}:{}", prefix.join("lib").display(), prefix.parent().unwrap_or(prefix).join("lib").display());
    Command::new(tool)
        .env("DYLD_LIBRARY_PATH", &libs)
        .env("LD_LIBRARY_PATH", &libs)
        .current_dir(dir)
        .arg("-i")
        .arg(input)
        .arg("-o")
        .arg(out)
        .output()
        .expect("spawn meshtool-opt")
}

fn tmp(name: &str) -> PathBuf {
    let d = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("libmesh").join(name);
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// Control: the same writer's non-polyhedral output is read, so a refusal below is about
/// `NFACED` and not about the netCDF container or the Exodus header.
#[test]
fn libmesh_reads_the_non_polyhedral_control() {
    let Some(tool) = meshtool() else {
        eprintln!("libMesh meshtool-opt not available: skipping");
        return;
    };
    let dir = tmp("control");
    let m = meshes::mixed_transition().unwrap().mesh;
    let input = dir.join("mixed.exo");
    std::fs::write(&input, exodus_bytes(&m, &ExodusOptions::default()).unwrap()).unwrap();
    let out = run(&tool, &dir, &input, &dir.join("mixed_out.exo"));
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(out.status.success(), "libMesh refused the control file: {err}");
    assert!(dir.join("mixed_out.exo").is_file(), "libMesh wrote no output");
}

/// The polyhedral dual patch (seven hexagonal prisms) as `NFACED`. libMesh 1.9.0-pre refuses it.
/// If a newer libMesh accepts it, this test fails on purpose: the acceptance check then becomes
/// equal faces per cell and cell volumes to 1e-12 relative (`3 sqrt(3) / 2` each).
#[test]
fn libmesh_refuses_nfaced() {
    let Some(tool) = meshtool() else {
        eprintln!("libMesh meshtool-opt not available: skipping");
        return;
    };
    let dir = tmp("nfaced");
    let m = meshes::dual_patch().unwrap().mesh;
    let input = dir.join("dual.exo");
    std::fs::write(&input, exodus_bytes(&m, &ExodusOptions::default()).unwrap()).unwrap();
    let out = run(&tool, &dir, &input, &dir.join("dual_out.exo"));
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(
        !out.status.success(),
        "libMesh now reads NFACED: add the face-count and volume comparison (1e-12 relative) and update the docs"
    );
    assert!(err.contains("\"NFACED\" not found"), "refused, but not for NFACED: {err}");
}
