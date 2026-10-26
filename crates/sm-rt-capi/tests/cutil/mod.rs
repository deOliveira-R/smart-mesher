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

/// The host target triple (the tests always compile for the machine they run on).
pub fn host_triple() -> String {
    let arch = std::env::consts::ARCH;
    match std::env::consts::OS {
        "macos" => format!("{arch}-apple-darwin"),
        "linux" => format!("{arch}-unknown-linux-gnu"),
        os => format!("{arch}-unknown-{os}"),
    }
}

/// File name of the cdylib on this platform (`libsm_rt_capi.so` / `.dylib`).
pub fn cdylib_name() -> String {
    format!("{}sm_rt_capi{}", std::env::consts::DLL_PREFIX, std::env::consts::DLL_SUFFIX)
}

/// The C compiler, found by the `cc` crate (it needs TARGET/HOST outside a build script).
pub fn c_compiler() -> Command {
    let triple = host_triple();
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

/// The C++ compiler (the `cc` crate's C++ driver).
pub fn cxx_compiler() -> Command {
    let triple = host_triple();
    cc::Build::new().cpp(true).target(&triple).host(&triple).opt_level(1).cargo_metadata(false).cargo_warnings(false).cargo_debug(false).get_compiler().to_command()
}

/// Build a C program linked to the cdylib and run it; returns its stdout.
pub fn build_and_run_c(src: &Path, name: &str) -> String {
    let dir = scratch(name);
    let exe = dir.join(name);
    let deps = deps_dir();
    let lib = deps.join(cdylib_name());
    assert!(lib.exists(), "cdylib not found at {lib:?} (run with --all-features)");
    let inc = crate_dir().join("include");
    let tests_c = crate_dir().join("tests/c");
    let cpp = src.extension().is_some_and(|e| e == "cpp");
    let mut cc = if cpp { cxx_compiler() } else { c_compiler() };
    cc.args([if cpp { "-std=c++17" } else { "-std=c11" }, "-Wall", "-Wextra", "-Werror", "-D_GNU_SOURCE"])
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
