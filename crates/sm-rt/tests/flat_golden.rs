//! Acceptance 7: hierarchy-less inputs keep their structure hashes and bundle hashes (the W3a
//! values, recorded on the base of this task before any change to the walker).
mod common;
use common::*;
use sm_geom_api::Ray;
use sm_mesh_api::SerialAlloc;
use sm_mesh_api::reference::hex_block;
use sm_rt_api::{BuildOptions, Tracker, TraceOptions};
use sm_testkit::meshes::{mixed_transition, nonconvex_poly};

fn rays(name: &str, n: usize, lo: [f64; 3], hi: [f64; 3]) -> Vec<Ray> {
    let mut r = rng(name);
    (0..n)
        .map(|_| {
            let (d, _, _) = random_frame(&mut r);
            let u = |k: usize, r: &mut SplitMix64| lo[k] + (hi[k] - lo[k]) * (r.unit() + 1.0) / 2.0;
            let o = [u(0, &mut r), u(1, &mut r), u(2, &mut r)];
            Ray { origin: p3(o), dir: v3(d) }
        })
        .collect()
}

fn record(t: &dyn Tracker, name: &str, lo: [f64; 3], hi: [f64; 3]) -> (u128, u128) {
    let b = t.trace_bundle(&rays(name, 20_000, lo, hi), &TraceOptions::default()).unwrap();
    (t.structure_hash().0, b.content_hash().0)
}

#[test]
fn flat_inputs_keep_their_w3a_hashes() {
    let opts = BuildOptions::default();
    let hb = hex_block([6; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let nc = nonconvex_poly().unwrap();
    let mt = mixed_transition().unwrap();
    let hb_t = build_mesh_tracker(&hb, &vacuum(), &opts);
    let nc_t = build_mesh_tracker(&nc.mesh, &vacuum(), &opts);
    let mt_t = build_mesh_tracker(&mt.mesh, &vacuum(), &opts);
    let facets = nested_facets();
    let f_t = build(&facets, &opts).unwrap();
    let got = [
        ("hex_block6", record(&hb_t, "g_hb", [-0.5; 3], [6.5; 3])),
        ("nonconvex_poly", record(&nc_t, "g_nc", [-0.2, -0.2, -0.2], [3.2, 2.2, 1.2])),
        ("mixed_transition", record(&mt_t, "g_mt", [-1.0, -1.5, -0.5], [3.0, 1.5, 2.5])),
        ("nested_facets", record(&*f_t, "g_nf", [-2.2; 3], [2.2; 3])),
    ];
    // Structure hashes were recorded on the base commit of this task (before the world walker,
    // the SoS swap and the surface tables) and must be equal on every platform. Bundle hashes
    // hash raw float bits and are recorded per (target_os, target_arch) (ruling C11). A moved
    // value on a platform that has one is a stop, never a re-bless; a new platform's values are
    // added as a new row, not by editing an existing one.
    let want: [(&str, u128); 4] = [
        ("hex_block6", 0x9d2acd0945032943d69768d2b42ebe92),
        ("nonconvex_poly", 0x4af44037ed018b8228e09125dc004e3f),
        ("mixed_transition", 0x5659f755d6b3ba0001f58b4a68cddb58),
        ("nested_facets", 0x156357c4b70cb571d51c1b58420b49aa),
    ];
    // Compute every case first so one run prints all tuples for a platform.
    let mut bad = Vec::new();
    for ((n, (s, b)), (wn, ws)) in got.into_iter().zip(want) {
        assert_eq!(n, wn);
        eprintln!("{n}: (structure hash, bundle hash) = ({s:#034x}, {b:#034x})");
        if s != ws {
            bad.push(format!("{n}: structure hash moved: got {s:#034x}, want {ws:#034x}"));
        }
        match bundle_hash(n) {
            Some(wb) if b != wb => bad.push(format!(
                "{n}: (structure hash, bundle hash) moved: got ({s:#034x}, {b:#034x}), want ({ws:#034x}, {wb:#034x})"
            )),
            Some(_) => {}
            None => eprintln!("{n}: no recorded bundle hash for this platform; structure only"),
        }
    }
    assert!(bad.is_empty(), "{}", bad.join("\n"));
}

/// Recorded bundle hash of a case on this (os, arch), if any.
#[allow(unused_variables)]
fn bundle_hash(name: &str) -> Option<u128> {
    #[cfg(all(target_os = "macos", target_arch = "aarch64"))]
    return match name {
        "hex_block6" => Some(0xcd433b5fc77169cad96f4c57eebb8e17),
        "nonconvex_poly" => Some(0x7f9e0cad2f60dd8a499b494fe2407b08),
        "mixed_transition" => Some(0xad87d0efcd2433934a62707dd4e1770c),
        "nested_facets" => Some(0xb87fb50821f41b2cb450a03bb5537d2d),
        _ => None,
    };
    // From the failing GitHub run 42412194333 (ubuntu-latest). Only hex_block6 is known: the
    // old test stopped at the first failing case. The rest are added once CI prints them.
    #[cfg(all(target_os = "linux", target_arch = "x86_64"))]
    return match name {
        "hex_block6" => Some(0x21a475890c57ab277f541a53d45710dd),
        _ => None,
    };
    #[allow(unreachable_code)]
    None
}
