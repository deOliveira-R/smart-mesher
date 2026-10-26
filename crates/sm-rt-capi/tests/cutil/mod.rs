//! Compile and run C / Fortran programs against the freshly built `libsm_rt_capi`.
#![allow(dead_code)]

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

pub fn crate_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// `target/<profile>/deps`: where cargo puts the cdylib / staticlib next to the test binaries.
pub fn deps_dir() -> PathBuf {
    std::env::current_exe().expect("exe").parent().expect("deps dir").to_path_buf()
}

pub fn scratch(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("smrt-capi-{}-{name}", std::process::id()));
    std::fs::create_dir_all(&d).expect("scratch dir");
    d
}

/// The C compiler, found by the `cc` crate (it needs TARGET/HOST outside a build script).
pub fn c_compiler() -> Command {
    let triple = format!("{}-unknown-linux-gnu", std::env::consts::ARCH);
    cc::Build::new()
        .target(&triple)
        .host(&triple)
        .opt_level(1)
        .cargo_metadata(false)
        .cargo_warnings(false)
        .cargo_debug(false)
        .get_compiler()
        .to_command()
}

pub fn run(cmd: &mut Command) -> Output {
    cmd.output().unwrap_or_else(|e| panic!("cannot run {cmd:?}: {e}"))
}

pub fn check_ok(what: &str, out: &Output) {
    assert!(
        out.status.success(),
        "{what} failed ({}):\n--- stdout\n{}\n--- stderr\n{}",
        out.status,
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
}

/// Build a C program linked to the cdylib and run it; returns its stdout.
pub fn build_and_run_c(src: &Path, name: &str) -> String {
    let dir = scratch(name);
    let exe = dir.join(name);
    let deps = deps_dir();
    let lib = deps.join("libsm_rt_capi.so");
    assert!(lib.exists(), "cdylib not found at {lib:?} (run with --all-features)");
    let inc = crate_dir().join("include");
    let tests_c = crate_dir().join("tests/c");
    let mut cc = c_compiler();
    cc.args(["-std=c11", "-Wall", "-Wextra", "-Werror", "-D_GNU_SOURCE"])
        .arg(format!("-I{}", inc.display()))
        .arg(format!("-I{}", tests_c.display()))
        .arg(src)
        .arg("-o")
        .arg(&exe)
        .arg(&lib)
        .arg(format!("-Wl,-rpath,{}", deps.display()))
        .arg("-lm");
    check_ok(&format!("compile {name}"), &run(&mut cc));
    let out = run(&mut Command::new(&exe));
    check_ok(&format!("run {name}"), &out);
    String::from_utf8_lossy(&out.stdout).into_owned()
}
