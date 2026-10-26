mod common;
use common::*;
use sm_scheme_tet::{TetOptions, tetrahedralize};
use sm_testkit::surf3;

fn check(name: String, f: &surf3::SurfaceFixture, v: usize, fails: &mut Vec<String>) {
    let inp = input_of(f, v);
    match tetrahedralize(&inp, &TetOptions::default()) {
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

#[test]
fn stress() {
    let mut fails = Vec::new();
    for n in 1..=14 {
        check(format!("cube({n})"), &surf3::tri_cube(n), 0, &mut fails);
    }
    for n in 1..=7 {
        check(format!("cavity({n})"), &surf3::cube_with_cavity(n), 0, &mut fails);
        let t = surf3::two_cubes_shared(n);
        check(format!("two0({n})"), &t, 0, &mut fails);
        check(format!("two1({n})"), &t, 1, &mut fails);
    }
    for l in 0..=4 {
        check(format!("ico({l})"), &surf3::icosphere(1.0, l), 0, &mut fails);
        check(format!("ico_r3({l})"), &surf3::icosphere(3.7, l), 0, &mut fails);
    }
    for (nu, nv) in [(8, 6), (12, 6), (16, 8), (24, 12), (32, 16), (48, 24), (30, 10)] {
        check(format!("torus({nu},{nv})"), &surf3::tri_torus(2.0, 0.7, nu, nv), 0, &mut fails);
        check(format!("torus2({nu},{nv})"), &surf3::tri_torus(5.0, 0.5, nu, nv), 0, &mut fails);
    }
    for s in 0..30 {
        check(format!("noisy({s})"), &surf3::noisy_sphere(s), 0, &mut fails);
    }
    check("schonhardt".into(), &surf3::schonhardt(), 0, &mut fails);
    for f in &fails {
        eprintln!("FAIL {f}");
    }
    eprintln!("stress: {} failures", fails.len());
}
