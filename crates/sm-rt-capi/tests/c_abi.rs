//! C programs (`tests/c/*.c`) compiled with the system C compiler (found through the `cc`
//! crate) and linked to the built cdylib: acceptance 1, 2, 3 and the C side of 4.
#![cfg(feature = "stubs")]

mod cutil;
use cutil::*;

#[test]
fn c_chords_lattice_mirror_and_white() {
    let out = build_and_run_c(&crate_dir().join("tests/c/chords.c"), "chords");
    assert!(out.contains("chords: ok"), "{out}");
}

#[test]
fn c_dag_shims_status_and_panic_containment() {
    let out = build_and_run_c(&crate_dir().join("tests/c/dag.c"), "dag");
    assert!(out.contains("dag: ok"), "{out}");
}

#[test]
fn cpp_raii_wrapper() {
    let out = build_and_run_c(&crate_dir().join("tests/c/raii.cpp"), "raii");
    assert!(out.contains("raii: ok"), "{out}");
}
