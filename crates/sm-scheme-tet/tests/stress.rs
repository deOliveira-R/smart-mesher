//! Robustness sweep over the `surf3` fixtures at several resolutions: every run must give an
//! exact, boundary-conforming mesh of the right volume.

mod common;
use common::*;
use sm_scheme_tet::{TetOptions, tetrahedralize};
use sm_testkit::surf3;

fn check(name: String, f: &surf3::SurfaceFixture, v: usize, fails: &mut Vec<String>) {
    let inp = input_of(f, v);
    let t = std::time::Instant::now();
    let r = tetrahedralize(&inp, &TetOptions::default());
    eprintln!("{name}: {:.2}s", t.elapsed().as_secs_f64());
    match r {
        Ok(m) => {
            let want = f.volumes[v].enclosed_volume;
            let vol = total_volume(&m);
            if ((vol - want) / want).abs() > 1e-10 || !all_exact_positive(&m) || check_boundary(&m, &inp.tris).is_err() {
                fails.push(format!("{name}: BAD MESH vol {vol} want {want}"));
            }
        }
        Err(e) => fails.push(format!("{name}: {e}")),
    }
}

fn sweep(full: bool) -> Vec<String> {
    let mut fails = Vec::new();
    // Debug builds (not `full`) run a thinner sweep: the exact predicates are much slower.
    let light = !full && cfg!(debug_assertions);
    let cubes = if full { 1..=14 } else if light { 1..=3 } else { 1..=6 };
    for n in cubes {
        check(format!("cube({n})"), &surf3::tri_cube(n), 0, &mut fails);
    }
    let shells = if full { 1..=7 } else if light { 1..=3 } else { 1..=5 };
    for n in shells {
        check(format!("cavity({n})"), &surf3::cube_with_cavity(n), 0, &mut fails);
        let t = surf3::two_cubes_shared(n);
        check(format!("two0({n})"), &t, 0, &mut fails);
        check(format!("two1({n})"), &t, 1, &mut fails);
    }
    let levels = if full { 0..=4 } else if light { 0..=2 } else { 0..=3 };
    for l in levels {
        check(format!("ico({l})"), &surf3::icosphere(1.0, l), 0, &mut fails);
        check(format!("ico_r3({l})"), &surf3::icosphere(3.7, l), 0, &mut fails);
    }
    let tori: &[(u32, u32)] = if full { &[(8, 6), (12, 6), (16, 8), (24, 12), (32, 16), (48, 24), (30, 10)] } else if light { &[(8, 6)] } else { &[(8, 6), (16, 8), (24, 12)] };
    for &(nu, nv) in tori {
        check(format!("torus({nu},{nv})"), &surf3::tri_torus(2.0, 0.7, nu, nv), 0, &mut fails);
        check(format!("torus2({nu},{nv})"), &surf3::tri_torus(5.0, 0.5, nu, nv), 0, &mut fails);
    }
    let seeds = if full { 0..30 } else if light { 0..2 } else { 0..6 };
    for s in seeds {
        check(format!("noisy({s})"), &surf3::noisy_sphere(s), 0, &mut fails);
    }
    check("schonhardt".into(), &surf3::schonhardt(), 0, &mut fails);
    fails
}

#[test]
fn fixture_sweep_quick() {
    let fails = sweep(false);
    assert!(fails.is_empty(), "{fails:#?}");
}

#[test]
#[ignore = "long robustness sweep (about 25 s in release)"]
fn fixture_sweep_full() {
    let fails = sweep(true);
    assert!(fails.is_empty(), "{fails:#?}");
}
