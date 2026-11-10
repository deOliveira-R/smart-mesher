//! OGRID-K review items: bounded ratio and phase, and the post-check on every kernel.

use sm_scheme_ogrid::*;

#[test]
fn extreme_ratio_and_phase_are_rejected_not_tangled() {
    for r in [1e7, 1e-7, 1e300, 1e-300, f64::INFINITY] {
        let mut d = DiskParams::balanced(16, 3);
        d.ratio = r;
        assert!(matches!(ogrid_disk(&d), Err(OgridError::Param(_))), "disk ratio {r}");
        let mut p = PolarParams::new(12, 3);
        p.ratio = r;
        assert!(matches!(polar_disk(&p), Err(OgridError::Param(_))), "polar ratio {r}");
        let rp = RingParams { n: 12, k: 3, r_in: 0.5, r_out: 1.0, ratio: r, phase_deg: 0.0 };
        assert!(matches!(ring(&rp), Err(OgridError::Param(_))), "ring ratio {r}");
        let a = AnnulusQuadParams { m: [3; 4], k: 3, ratio: r, phase_deg: 45.0, radius: 0.5, side: 2.0 };
        assert!(matches!(ogrid_annulus_quad(&a), Err(OgridError::Param(_))), "annulus ratio {r}");
    }
    for ph in [3601.0, -1e9, 1e300] {
        let mut d = DiskParams::balanced(16, 3);
        d.phase_deg = ph;
        assert!(matches!(ogrid_disk(&d), Err(OgridError::Param(_))), "phase {ph}");
        let mut p = PolarParams::new(12, 3);
        p.phase_deg = ph;
        assert!(matches!(polar_disk(&p), Err(OgridError::Param(_))), "phase {ph}");
        let rp = RingParams { n: 12, k: 3, r_in: 0.5, r_out: 1.0, ratio: 1.0, phase_deg: ph };
        assert!(matches!(ring(&rp), Err(OgridError::Param(_))), "ring phase {ph}");
    }
}

#[test]
fn edges_of_the_accepted_range_still_mesh_exactly() {
    for ratio in [1e-6, 1e6] {
        let mut d = DiskParams::balanced(16, 3);
        d.ratio = ratio;
        d.phase_deg = -3600.0 + 45.0;
        let m = ogrid_disk(&d).unwrap();
        assert!(m.min_scaled_jacobian() > 0.0);
        assert!((m.area() - inscribed_area(16, 1.0)).abs() < 1e-12, "disk ratio {ratio}");
        let a = AnnulusQuadParams { m: [4; 4], k: 4, ratio, phase_deg: 3600.0 + 45.0 - 360.0, radius: 0.9, side: 2.0 };
        let m = ogrid_annulus_quad(&a).unwrap();
        assert!(m.min_scaled_jacobian() > 0.0, "annulus ratio {ratio}");
        let mut p = PolarParams::new(12, 4);
        p.ratio = ratio;
        assert!(polar_disk(&p).unwrap().min_scaled_jacobian() > 0.0);
    }
}

#[test]
fn disk_never_returns_a_tangled_mesh() {
    // Sweep awkward parameters: every Ok mesh has a positive scaled Jacobian, the rest are Inverted.
    let (mut ok, mut inverted) = (0, 0);
    for (a, b) in [(1, 1), (1, 7), (2, 6), (4, 4)] {
        for core in [0.05, 0.5, 0.95, 0.999] {
            for smooth in [0, 20] {
                let mut d = DiskParams::new(2 * (a + b), a, b, 2);
                d.core = core;
                d.smooth_iters = smooth;
                match ogrid_disk(&d) {
                    Ok(m) => {
                        assert!(m.min_scaled_jacobian() > 0.0);
                        ok += 1;
                    }
                    Err(OgridError::Inverted(_)) => inverted += 1,
                    Err(e) => panic!("{e}"),
                }
            }
        }
    }
    assert!(ok > 0, "inverted {inverted}");
}
