//! Acceptance 8: quality floors (calibrated, then frozen) and the `sm_quality` cross-check.

mod common;
use common::*;
use sm_mesh_api::{CellIx, CellShape, MeshRead};
use sm_quality::{Metric, Options, compute_many};
use sm_scheme_tet::{TetMesh, TetOptions, tet_quality, tetrahedralize};
use sm_testkit::meshes::{CellSpec, build_mesh};
use sm_testkit::surf3::{self, SurfaceFixture};

fn fixtures() -> Vec<(&'static str, SurfaceFixture)> {
    vec![
        ("tri_cube(10)", surf3::tri_cube(10)),
        ("icosphere(1,4)", surf3::icosphere(1.0, 4)),
        ("tri_torus", surf3::tri_torus(2.0, 0.7, 48, 24)),
    ]
}

fn run(f: &SurfaceFixture, optimize: u8) -> TetMesh {
    tetrahedralize(&input_of(f, 0), &TetOptions { optimize, ..TetOptions::default() }).unwrap()
}

struct Summary {
    min_mr: f64,
    mean_mr: f64,
    min_dih: f64,
    max_dih: f64,
    frac_re: f64,
}

fn summarize(m: &TetMesh) -> Summary {
    let qs: Vec<_> = m.tets.iter().map(|t| tet_quality(t.map(|i| m.points[i as usize]))).collect();
    let n = qs.len() as f64;
    Summary {
        min_mr: qs.iter().map(|q| q.mean_ratio).fold(f64::INFINITY, f64::min),
        mean_mr: qs.iter().map(|q| q.mean_ratio).sum::<f64>() / n,
        min_dih: qs.iter().map(|q| q.min_dihedral).fold(f64::INFINITY, f64::min),
        max_dih: qs.iter().map(|q| q.max_dihedral).fold(0.0, f64::max),
        frac_re: qs.iter().filter(|q| q.radius_edge <= 2.0).count() as f64 / n,
    }
}

#[test]
fn optimize0_radius_edge_and_unrefinable_floors() {
    for (name, f) in fixtures() {
        let m = run(&f, 0);
        let s = summarize(&m);
        eprintln!("{name} optimize=0: frac(re<=2)={:.4} unrefinable={} of {}", s.frac_re, m.stats.unrefinable, m.tets.len());
        assert!(s.frac_re >= 0.98, "{name}: radius-edge fraction {}", s.frac_re);
        assert!(m.stats.unrefinable as f64 <= 0.005 * m.tets.len() as f64, "{name}: unrefinable {}", m.stats.unrefinable);
    }
}

#[test]
fn optimize2_floors_and_monotone() {
    for (name, f) in fixtures() {
        let m0 = run(&f, 0);
        let m1 = run(&f, 1);
        let m2 = run(&f, 2);
        let (s0, s1, s2) = (summarize(&m0), summarize(&m1), summarize(&m2));
        eprintln!(
            "{name}: opt2 min dihedral {:.1} max {:.1} min mean ratio {:.3} avg {:.3} (opt0 min {:.3} / opt1 min {:.3})",
            s2.min_dih, s2.max_dih, s2.min_mr, s2.mean_mr, s0.min_mr, s1.min_mr
        );
        // Frozen gates (task acceptance 8).
        assert!(s2.min_dih >= 10.0, "{name}: min dihedral {}", s2.min_dih);
        assert!(s2.max_dih <= 165.0, "{name}: max dihedral {}", s2.max_dih);
        assert!(s2.min_mr >= 0.10, "{name}: min mean ratio {}", s2.min_mr);
        assert!(s2.mean_mr >= 0.75, "{name}: mean mean ratio {}", s2.mean_mr);
        // Monotone: optimization never lowers the worst tet.
        assert!(s1.min_mr >= s0.min_mr && s2.min_mr >= s0.min_mr, "{name}: not monotone");
        // Regression guards, calibrated with margin on the measured values (DESIGN.md).
        assert!(s2.min_dih >= 15.0 && s2.min_mr >= 0.4 && s2.mean_mr >= 0.78, "{name}: regression guard");
        // Statistics agree with a direct evaluation.
        assert_eq!(m2.stats.min_mean_ratio.to_bits(), s2.min_mr.to_bits());
        assert!(((m2.stats.mean_mean_ratio - s2.mean_mr) / s2.mean_mr).abs() < 1e-12);
        assert_eq!(m2.stats.min_dihedral_deg.to_bits(), s2.min_dih.to_bits());
        assert_eq!(m2.stats.max_dihedral_deg.to_bits(), s2.max_dih.to_bits());
        assert!(all_exact_positive(&m2));
        assert!(m2.stats.smooth_passes > 0 && m2.stats.flips > 0);
    }
}

/// Cross-check against `sm_quality` on the same mesh.
///
/// Mapping: `sm_quality::Metric::Shape` of a tet is the mean ratio; `Metric::RadiusRatio` is
/// `R / (3 r_in)` with `r_in = 3 V / sum(face areas)`, so `R = 3 * RadiusRatio * r_in` gives the
/// circumradius and `R / l_min` the radius-edge ratio; `MinAngle`/`MaxAngle` are the dihedrals.
#[test]
fn cross_check_with_sm_quality() {
    let f = surf3::icosphere(1.0, 3);
    let m = run(&f, 2);
    let cells: Vec<CellSpec> = m.tets.iter().map(|t| CellSpec::Template(CellShape::Tet, t.iter().map(|&i| i as usize).collect())).collect();
    let mesh = build_mesh(&m.points, &cells, &mut sm_mesh_api::SerialAlloc::new()).unwrap();
    let table = compute_many(&mesh, &[Metric::Shape, Metric::RadiusRatio, Metric::MinAngle, Metric::MaxAngle], &Options::default()).unwrap();
    let shape = table.column(Metric::Shape).unwrap().to_vec();
    let rr = table.column(Metric::RadiusRatio).unwrap().to_vec();
    let min_a = table.column(Metric::MinAngle).unwrap().to_vec();
    let max_a = table.column(Metric::MaxAngle).unwrap().to_vec();
    assert_eq!(mesh.count(sm_mesh_api::MeshEntityKind::Cell), m.tets.len());
    let (mut worst_eta, mut worst_re, mut worst_dih): (f64, f64, f64) = (0.0, 0.0, 0.0);
    for (ci, t) in m.tets.iter().enumerate() {
        // Cells are created in tet order; find the cell's corner order to be safe.
        let corners = mesh.cell_corners(CellIx::new(ci)).expect("tet corners");
        assert_eq!(corners.len(), 4);
        let q = tet_quality(t.map(|i| m.points[i as usize]));
        worst_eta = worst_eta.max((q.mean_ratio - shape[ci]).abs());
        // Radius-edge from sm_quality's RadiusRatio.
        let p = t.map(|i| m.points[i as usize]);
        let d = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
        let cr = |a: [f64; 3], b: [f64; 3]| [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
        let nm = |a: [f64; 3]| (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
        let vol = tet_vol(&m, *t);
        let area = 0.5 * (nm(cr(d(p[1], p[0]), d(p[2], p[0]))) + nm(cr(d(p[1], p[0]), d(p[3], p[0]))) + nm(cr(d(p[2], p[0]), d(p[3], p[0]))) + nm(cr(d(p[2], p[1]), d(p[3], p[1]))));
        let r_in = 3.0 * vol / area;
        let big_r = 3.0 * rr[ci] * r_in;
        let mut lmin = f64::INFINITY;
        for i in 0..4 {
            for j in i + 1..4 {
                lmin = lmin.min(nm(d(p[i], p[j])));
            }
        }
        worst_re = worst_re.max((big_r / lmin - q.radius_edge).abs() / q.radius_edge);
        worst_dih = worst_dih.max((q.min_dihedral - min_a[ci]).abs()).max((q.max_dihedral - max_a[ci]).abs());
    }
    eprintln!("cross-check: mean ratio {worst_eta:.2e}, radius-edge (rel) {worst_re:.2e}, dihedral (deg) {worst_dih:.2e}");
    assert!(worst_eta < 1e-9, "mean ratio vs Shape: {worst_eta}");
    assert!(worst_re < 1e-9, "radius-edge vs RadiusRatio: {worst_re}");
    assert!(worst_dih < 1e-6, "dihedral vs MinAngle/MaxAngle: {worst_dih}");
}
