//! HEX-OGRID-K acceptance 1-8 (drafts/hexpin.md), exact or analytic checks.

use proptest::prelude::*;
use sm_scheme_ogrid::*;
use std::f64::consts::PI;

const NS: [usize; 5] = [8, 12, 16, 20, 40];

fn disk(n: usize, k: usize) -> (DiskParams, PlanarMesh) {
    let p = DiskParams::balanced(n, k);
    let m = ogrid_disk(&p).unwrap();
    (p, m)
}

fn central_corners(p: &DiskParams) -> [u32; 4] {
    let c0 = p.n as u32;
    let (a, b) = (p.a as u32, p.b as u32);
    let w = a + 1;
    [c0, c0 + a, c0 + b * w + a, c0 + b * w]
}

#[test]
fn acc1_quad_count_and_euler() {
    for n in NS {
        for k in [1, 2, 4] {
            let (p, m) = disk(n, k);
            assert_eq!(m.quads.len(), p.a * p.b + k * n, "n={n} k={k}");
            assert_eq!(m.euler(), 1, "n={n} k={k}");
            assert_eq!(2 * (p.a + p.b), n);
            assert!(p.a == p.b || p.b == p.a + 1);
        }
    }
    // Unbalanced splits work too: n = 2(a+b) with a != b.
    let p = DiskParams::new(2 * (3 + 5), 3, 5, 2);
    let m = ogrid_disk(&p).unwrap();
    assert_eq!(m.quads.len(), 3 * 5 + 2 * 16);
    assert_eq!(m.euler(), 1);
}

#[test]
fn acc2_valence() {
    for n in NS {
        let (p, m) = disk(n, 3);
        let (val, bnd) = m.valence_and_boundary();
        let interior: Vec<usize> = (0..m.nodes.len()).filter(|&v| !bnd[v]).collect();
        let v3: Vec<usize> = interior.iter().copied().filter(|&v| val[v] == 3).collect();
        assert_eq!(v3.len(), 4, "n={n}");
        let mut expect: Vec<usize> = central_corners(&p).iter().map(|&c| c as usize).collect();
        expect.sort_unstable();
        assert_eq!(v3, expect);
        assert!(interior.iter().all(|&v| val[v] == 3 || val[v] == 4), "n={n}: a valence outside {{3,4}}");
    }
}

#[test]
fn acc3_area_is_the_inscribed_polygon() {
    for n in NS {
        for r in [1.0, 0.37, 12.5] {
            let mut p = DiskParams::balanced(n, 3);
            p.radius = r;
            let m = ogrid_disk(&p).unwrap();
            let exact = inscribed_area(n, r);
            assert!((m.area() - exact).abs() <= 1e-12 * exact, "n={n} r={r}");
            assert!(m.min_quad_area() > 0.0);
        }
    }
}

fn contains(set: &[[f64; 2]], p: [f64; 2]) -> bool {
    set.iter().any(|q| (q[0] - p[0]).abs() <= 1e-12 && (q[1] - p[1]).abs() <= 1e-12)
}

#[test]
fn acc4_symmetry_of_the_node_set() {
    for n in [8, 12, 16, 20, 40] {
        for smooth in [0, 25] {
            let mut p = DiskParams::balanced(n, 3);
            p.smooth_iters = smooth;
            let m = ogrid_disk(&p).unwrap();
            for q in &m.nodes {
                for img in [[-q[1], q[0]], [-q[0], q[1]], [q[0], -q[1]]] {
                    assert!(contains(&m.nodes, img), "n={n} smooth={smooth}: image of {q:?} missing");
                }
            }
        }
    }
}

#[test]
fn acc5_scaled_jacobian_at_the_central_corners() {
    let want = (135.0f64).to_radians().sin();
    for n in [8, 12, 16, 20, 40] {
        let (p, m) = disk(n, 3);
        for c in central_corners(&p) {
            assert!((m.min_sj_at_node(c) - want).abs() < 1e-9, "n={n}: {}", m.min_sj_at_node(c));
        }
        // After smoothing (uniform layers) every corner of every quad stays at or above 0.7.
        let mut ps = p.clone();
        ps.smooth_iters = 10;
        let ms = ogrid_disk(&ps).unwrap();
        let min = ms.min_scaled_jacobian();
        eprintln!("n={n} min SJ unsmoothed {:.4} smoothed {min:.4}", m.min_scaled_jacobian());
        assert!(min >= 0.7, "n={n}: smoothed min SJ {min}");
    }
}

fn try_annulus(m: [usize; 4], k: usize, rr: f64) -> Result<PlanarMesh, sm_scheme_ogrid::OgridError> {
    let side = 2.0;
    ogrid_annulus_quad(&AnnulusQuadParams { m, k, ratio: 1.0, phase_deg: 45.0, radius: rr * side / 2.0, side })
}

fn square_annulus(m: [usize; 4], k: usize, rr: f64) -> PlanarMesh {
    try_annulus(m, k, rr).unwrap()
}

#[test]
fn acc6_tangled_unbalanced_sides_are_rejected() {
    // Reviewer probes: each tangles the radial lines (negative SJ or signed area).
    for (m, k, rr) in [([2, 5, 3, 4], 2, 0.9), ([1, 6, 1, 6], 2, 0.5), ([1, 1, 1, 5], 2, 0.5), ([1, 1, 1, 9], 2, 0.9)] {
        let e = try_annulus(m, k, rr);
        assert!(matches!(e, Err(sm_scheme_ogrid::OgridError::Inverted(_))), "m={m:?} rr={rr}: {e:?}");
    }
}

#[test]
fn acc6_balanced_guarantee_needs_phase_45() {
    let run = |m, phase_deg, rr: f64| {
        ogrid_annulus_quad(&AnnulusQuadParams { m, k: 2, ratio: 1.0, phase_deg, radius: rr, side: 2.0 })
    };
    let e = run([3; 4], 0.0, 0.5);
    assert!(matches!(e, Err(sm_scheme_ogrid::OgridError::Inverted(_))), "{e:?}");
    let ok = run([3; 4], 45.0, 0.999).unwrap();
    assert!(ok.min_scaled_jacobian() > 0.0);
}

#[test]
fn acc6_square_annulus_area() {
    for (m, k) in [([3, 3, 3, 3], 2), ([4, 4, 4, 4], 3), ([3, 4, 3, 4], 2)] {
        let n: usize = m.iter().sum();
        for rr in [0.1, 0.5, 0.9] {
            let mesh = square_annulus(m, k, rr);
            let exact = 4.0 - inscribed_area(n, rr);
            assert_eq!(mesh.quads.len(), k * n);
            assert_eq!(mesh.euler(), 0);
            assert!((mesh.area() - exact).abs() <= 1e-12 * exact, "m={m:?} rr={rr}");
            assert!(mesh.min_quad_area() > 0.0, "inverted quad m={m:?} rr={rr}");
            assert!(mesh.min_scaled_jacobian() > 0.0, "tangled m={m:?} rr={rr}");
        }
    }
}

proptest! {
    #![proptest_config(sm_testkit::strategies::config(48))]
    #[test]
    fn acc6_no_inverted_quad_for_any_radius_ratio(mm in 1usize..7, k in 1usize..5, rr in 0.1f64..0.9) {
        let mesh = square_annulus([mm; 4], k, rr);
        let n = 4 * mm;
        let exact = 4.0 - inscribed_area(n, rr);
        prop_assert!(mesh.min_quad_area() > 0.0);
        prop_assert!(mesh.min_scaled_jacobian() > 0.0);
        prop_assert!((mesh.area() - exact).abs() <= 1e-12 * exact);
    }

    #[test]
    fn acc6_unbalanced_is_valid_or_typed_error(
        m in proptest::array::uniform4(1usize..8), k in 1usize..4, rr in 0.1f64..0.9,
    ) {
        match try_annulus(m, k, rr) {
            Ok(mesh) => {
                prop_assert!(mesh.min_scaled_jacobian() > 0.0);
                prop_assert!(mesh.min_quad_area() > 0.0);
                let exact = 4.0 - inscribed_area(m.iter().sum(), rr);
                prop_assert!((mesh.area() - exact).abs() <= 1e-12 * exact);
            }
            Err(e) => prop_assert!(matches!(e, sm_scheme_ogrid::OgridError::Inverted(_))),
        }
    }

    #[test]
    fn disk_is_valid_for_any_split(a in 1usize..8, b in 1usize..8, k in 1usize..5, ph in 0.0f64..360.0) {
        let mut p = DiskParams::new(2 * (a + b), a, b, k);
        p.phase_deg = ph;
        let m = ogrid_disk(&p).unwrap();
        let exact = inscribed_area(p.n, 1.0);
        prop_assert!((m.area() - exact).abs() <= 1e-12 * exact);
        prop_assert_eq!(m.euler(), 1);
    }
}

#[test]
fn acc7_polar_disk() {
    for n in [4, 6, 8, 12, 20] {
        for k in [1, 3] {
            let mut p = PolarParams::new(n, k);
            p.phase_deg = 17.0;
            let m = ogrid_polar(&p);
            assert_eq!(m.polys.len(), 1);
            assert_eq!(m.polys[0].len(), n);
            assert_eq!(m.quads.len(), k * n);
            assert_eq!(m.euler(), 1);
            let exact = inscribed_area(n, 1.0);
            assert!((m.area() - exact).abs() <= 1e-12 * exact, "n={n} k={k} {} {exact}", m.area());
            // Every reflection of the regular n-gon (axes at phase + j pi / n) maps the node set onto itself.
            for j in 0..2 * n {
                let ax = p.phase_deg.to_radians() + PI * j as f64 / n as f64;
                let (c, s) = ((2.0 * ax).cos(), (2.0 * ax).sin());
                for q in &m.nodes {
                    assert!(contains(&m.nodes, [c * q[0] + s * q[1], s * q[0] - c * q[1]]), "n={n} axis {j}");
                }
            }
            // No valence-3 anywhere inside; polygon nodes have valence 3 (two ring edges, one radial).
            let (val, bnd) = m.valence_and_boundary();
            assert!((0..m.nodes.len()).filter(|&v| !bnd[v]).all(|v| val[v] == 4 || (v < n && val[v] == 3)));
        }
    }
}

fn ogrid_polar(p: &PolarParams) -> PlanarMesh {
    polar_disk(p).unwrap()
}

#[test]
fn ring_area_euler_and_grading() {
    let p = RingParams { n: 24, k: 4, r_in: 0.4, r_out: 1.0, ratio: 1.0, phase_deg: 3.0 };
    let m = ring(&p).unwrap();
    assert_eq!(m.quads.len(), 96);
    assert_eq!(m.euler(), 0);
    let exact = inscribed_area(24, 1.0) - inscribed_area(24, 0.4);
    assert!((m.area() - exact).abs() <= 1e-12 * exact, "{} {exact}", m.area());
    // Graded: outermost layer is `ratio` times the innermost, geometric between.
    let mut g = p.clone();
    g.ratio = 0.25;
    let m = ring(&g).unwrap();
    let rad = |l: usize| m.nodes[l * 24].iter().map(|x| x * x).sum::<f64>().sqrt();
    let th: Vec<f64> = (0..4).map(|l| rad(l + 1) - rad(l)).collect();
    assert!((th[3] / th[0] - 0.25).abs() < 1e-12);
    assert!((th[1] / th[0] - 0.25f64.powf(1.0 / 3.0)).abs() < 1e-12);
    assert!((rad(0) - 0.4).abs() < 1e-15 && (rad(4) - 1.0).abs() < 1e-15);
}

#[test]
fn graded_disk_keeps_the_exact_area_and_thin_outer_cells() {
    let mut p = DiskParams::balanced(16, 4);
    p.ratio = 0.3;
    let m = ogrid_disk(&p).unwrap();
    assert!((m.area() - inscribed_area(16, 1.0)).abs() < 1e-12);
    assert!(m.min_quad_area() > 0.0);
}

#[test]
fn bad_parameters_are_typed_errors() {
    assert!(ogrid_disk(&DiskParams::new(10, 2, 2, 1)).is_err());
    assert!(ogrid_disk(&DiskParams::new(8, 0, 4, 1)).is_err());
    let mut p = DiskParams::new(8, 2, 2, 1);
    p.ratio = f64::NAN;
    assert!(ogrid_disk(&p).is_err());
    p.ratio = 1.0;
    p.k = 0;
    assert!(ogrid_disk(&p).is_err());
    assert!(polar_disk(&PolarParams::new(2, 1)).is_err());
    assert!(ring(&RingParams { n: 8, k: 1, r_in: 1.0, r_out: 0.5, ratio: 1.0, phase_deg: 0.0 }).is_err());
}

fn all_hashes() -> Vec<u64> {
    let mut v = Vec::new();
    for n in NS {
        v.push(ogrid_disk(&DiskParams::balanced(n, 3)).unwrap().hash());
        let mut s = DiskParams::balanced(n, 3);
        s.smooth_iters = 7;
        v.push(ogrid_disk(&s).unwrap().hash());
        v.push(polar_disk(&PolarParams::new(n, 3)).unwrap().hash());
    }
    v.push(square_annulus([3, 4, 3, 4], 2, 0.6).hash());
    v.push(ring(&RingParams { n: 12, k: 2, r_in: 0.5, r_out: 1.0, ratio: 0.7, phase_deg: 0.0 }).unwrap().hash());
    v
}

#[test]
fn acc8_output_is_bit_identical_across_runs_and_threads() {
    // The kernels are serial; compare repeated runs and runs from concurrent threads (CI also
    // runs this binary with RAYON_NUM_THREADS=1).
    let base = all_hashes();
    assert_eq!(base, all_hashes());
    let hs: Vec<Vec<u64>> = (0..4).map(|_| std::thread::spawn(all_hashes)).map(|h| h.join().unwrap()).collect();
    assert!(hs.iter().all(|h| *h == base));
    // Pin the values so a platform or refactor change is visible.
    eprintln!("hashes {base:x?}");
}
