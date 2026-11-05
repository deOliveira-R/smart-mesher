//! C programs (`tests/c/*.c`) compiled with the system C compiler (found through the `cc`
//! crate) and linked to the built cdylib: acceptance 1, 2, 3 and the C side of 4.
#![cfg(feature = "stubs")]

mod cutil;
#[path = "common/flight_ref.rs"]
mod flight_ref;
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
fn c_flights_on_the_stub_majorant_world_equal_the_rust_path_and_contain_panics() {
    use sm_rt_capi::ffi::test_hooks::{STUB_GRID, tracker_for};
    let out = build_and_run_c(&crate_dir().join("tests/c/flight_stub.c"), "flight_stub");
    assert!(out.contains("flight_stub: ok"), "{out}");
    let lines: Vec<String> = out.lines().filter(|l| l.starts_with("CASE ")).map(str::to_owned).collect();
    let t = tracker_for(STUB_GRID, 5.0).expect("stub");
    let want = [
        flight_ref::run_case("stub_surface", &*t, 0, 10_000, 0x4444),
        flight_ref::run_case("stub_delta", &*t, 1, 10_000, 0x5555),
        flight_ref::run_case("stub_delta_block", &*t, 2, 10_000, 0x6666),
    ];
    assert_eq!(lines.len(), 3, "{out}");
    for (got, want) in lines.iter().zip(&want) {
        assert_eq!(got, want);
    }
    assert!(!want[1].contains("coll=0 ") && !want[1].contains("rej=0 "), "{}", want[1]);
}

#[test]
fn cpp_raii_wrapper() {
    let out = build_and_run_c(&crate_dir().join("tests/c/raii.cpp"), "raii");
    assert!(out.contains("raii: ok"), "{out}");
}
