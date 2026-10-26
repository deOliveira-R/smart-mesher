mod common;
use common::*;
use sm_scheme_tet::{TetOptions, tetrahedralize};
use sm_testkit::surf3;

fn run(name: &str, f: &surf3::SurfaceFixture, v: usize) {
    let inp = input_of(f, v);
    let opts = TetOptions { optimize: 0, ..TetOptions::default() };
    let t = std::time::Instant::now();
    match tetrahedralize(&inp, &opts) {
        Ok(m) => {
            let vol = total_volume(&m);
            let r = check_boundary(&m, &inp.tris);
            eprintln!("{name}: ok tets={} steiner={} vol={vol} ref={} bnd={:?} exact+={} {:?}", m.tets.len(), m.stats.n_steiner, f.volumes[v].enclosed_volume, r, all_exact_positive(&m), t.elapsed());
        }
        Err(e) => eprintln!("{name}: ERR {e}"),
    }
}

#[test]
fn all_fixtures() {
    run("cube4", &surf3::tri_cube(4), 0);
    run("cube10", &surf3::tri_cube(10), 0);
    run("ico3", &surf3::icosphere(1.0, 3), 0);
    run("ico4", &surf3::icosphere(1.0, 4), 0);
    run("torus", &surf3::tri_torus(2.0, 0.7, 48, 24), 0);
    let c = surf3::cube_with_cavity(4);
    run("cavity", &c, 0);
    let t = surf3::two_cubes_shared(4);
    run("two0", &t, 0);
    run("two1", &t, 1);
    run("schonhardt", &surf3::schonhardt(), 0);
    run("noisy", &surf3::noisy_sphere(1), 0);
}
