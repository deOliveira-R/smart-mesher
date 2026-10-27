//! Acceptance 7: size control.

mod common;
use common::*;
use sm_scheme_tet::{SizeSpec, TetOptions, tetrahedralize};
use sm_testkit::surf3;

fn longest_edge(m: &sm_scheme_tet::TetMesh, t: &[u32; 4]) -> f64 {
    let mut l: f64 = 0.0;
    for i in 0..4 {
        for j in i + 1..4 {
            let (a, b) = (m.points[t[i] as usize], m.points[t[j] as usize]);
            l = l.max(((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt());
        }
    }
    l
}

fn mean_edge(m: &sm_scheme_tet::TetMesh, t: &[u32; 4]) -> f64 {
    let mut s = 0.0;
    for i in 0..4 {
        for j in i + 1..4 {
            let (a, b) = (m.points[t[i] as usize], m.points[t[j] as usize]);
            s += ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
        }
    }
    s / 6.0
}

#[test]
fn constant_size_counts_and_edge_lengths() {
    let f = surf3::tri_cube(10);
    let inp = input_of(&f, 0);
    let opts = TetOptions { size: SizeSpec::Constant(0.1), ..TetOptions::default() };
    let m = tetrahedralize(&inp, &opts).unwrap();
    let n = m.tets.len();
    eprintln!("constant(0.1) on tri_cube(10): {n} tets");
    assert!((4000..=20000).contains(&n), "n = {n}");
    let ok = m.tets.iter().filter(|t| longest_edge(&m, t) <= 1.6 * 0.1).count();
    assert!(ok as f64 >= 0.99 * n as f64, "{ok} of {n} within 1.6h");
}

#[test]
fn graded_field_adjacent_ratio() {
    let f = surf3::tri_cube(4);
    let inp = input_of(&f, 0);
    let field = |p: [f64; 3]| 0.1 + 0.3 * p[0];
    let opts = TetOptions { size: SizeSpec::Field(&field), ..TetOptions::default() };
    let m = tetrahedralize(&inp, &opts).unwrap();
    // Face adjacency.
    let mut faces: std::collections::BTreeMap<[u32; 3], Vec<usize>> = std::collections::BTreeMap::new();
    for (ti, t) in m.tets.iter().enumerate() {
        for f in [[t[1], t[2], t[3]], [t[0], t[2], t[3]], [t[0], t[1], t[3]], [t[0], t[1], t[2]]] {
            let mut k = f;
            k.sort_unstable();
            faces.entry(k).or_default().push(ti);
        }
    }
    let size: Vec<f64> = m.tets.iter().map(|t| mean_edge(&m, t)).collect();
    let mut worst: f64 = 1.0;
    for v in faces.values() {
        if let [a, b] = v[..] {
            worst = worst.max((size[a] / size[b]).max(size[b] / size[a]));
        }
    }
    eprintln!("graded: {} tets, worst adjacent mean-edge ratio {worst:.3}", m.tets.len());
    assert!(worst <= 1.5, "worst adjacent ratio {worst}");
}
