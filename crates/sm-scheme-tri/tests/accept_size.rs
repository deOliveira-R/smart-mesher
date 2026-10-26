//! Acceptance 7: size fields (graded and anisotropic).

mod common;

use common::*;
use sm_core::Point3;
use sm_mesh_api::{MeshRead, validate};
use sm_scheme_api::{SizeField, Sequential, SymMat3};
use sm_testkit::loops2d;
use sm_testkit::topo::planar_face;

/// `h(x) = 0.05 + 0.15 x`.
struct Graded;

impl SizeField for Graded {
    fn size(&self, p: Point3) -> f64 {
        0.05 + 0.15 * p.x
    }
}

/// Stretched 4:1 in x: target lengths `4 h` along x and `h` along y, z.
struct Stretched(f64);

impl SizeField for Stretched {
    fn size(&self, _p: Point3) -> f64 {
        self.0
    }
    fn metric(&self, _p: Point3) -> SymMat3 {
        let h = self.0;
        SymMat3([1.0 / (16.0 * h * h), 0.0, 0.0, 1.0 / (h * h), 0.0, 1.0 / (h * h)])
    }
}

#[test]
fn acc7_graded_size_field_counts_and_edge_lengths() {
    let fx = planar_face(&loops2d::square(), 0.0).unwrap();
    let reg = registry();
    // Counts come from the size field through the driver (geom::edge_size_goal).
    let out = mesh_faces_params(&reg, &fx.complex, &[fx.face], &[], serde_json::json!({}), &Sequential, Some(&Graded))
        .expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty());
    let n = m.cell_count() as f64;
    let n_ideal = 4.0 / 3f64.sqrt() * 100.0;
    println!("graded: N={n} ideal {n_ideal:.1} ratio {:.3}", n / n_ideal);
    assert!(n >= 0.8 * n_ideal && n <= 1.6 * n_ideal, "N = {n}");
    // Edge length ratios on interior edges.
    let use_ = edge_use(m);
    let mut ratios = Vec::new();
    for (&(a, b), &k) in &use_ {
        if k != 2 {
            continue;
        }
        let (pa, pb) = (m.point(a), m.point(b));
        let mid = Point3::new(0.5 * (pa.x + pb.x), 0.5 * (pa.y + pb.y), 0.0);
        ratios.push((pb - pa).norm() / Graded.size(mid));
    }
    let good = ratios.iter().filter(|&&r| (0.5..=1.5).contains(&r)).count() as f64 / ratios.len() as f64;
    let worst = ratios.iter().cloned().fold(0.0, f64::max);
    println!("graded: {:.1}% in [0.5, 1.5], worst {worst:.3}", 100.0 * good);
    assert!(good >= 0.9, "{good}");
    assert!(worst <= 2.2, "{worst}");
}

#[test]
fn acc7_anisotropic_metric_stretches_edges_four_to_one() {
    let fx = planar_face(&loops2d::square(), 0.0).unwrap();
    let e = &fx.edges[0];
    // h = 0.05 across, 4h = 0.2 along x: 5 intervals on the x sides, 20 on the y sides.
    let counts = vec![(e[0], 5u32), (e[1], 20), (e[2], 5), (e[3], 20)];
    let reg = registry();
    let f = Stretched(0.05);
    let out = mesh_faces_params(&reg, &fx.complex, &[fx.face], &counts, serde_json::json!({}), &Sequential, Some(&f))
        .expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty());
    let (mut sx, mut sy, mut n) = (0.0, 0.0, 0.0);
    for &(a, b) in edge_use(m).keys() {
        let (pa, pb) = (m.point(a), m.point(b));
        sx += (pb.x - pa.x).abs();
        sy += (pb.y - pa.y).abs();
        n += 1.0;
    }
    let ratio = (sx / n) / (sy / n);
    println!("aniso: N={} mean|dx|/mean|dy| = {ratio:.3}", m.cell_count());
    assert!((2.5..=5.0).contains(&ratio), "{ratio}");
}
