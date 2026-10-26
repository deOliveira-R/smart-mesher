use super::*;
use sm_testkit::loops2d::{self, Region2, ngon_area};

fn id_metric(_: [f64; 2]) -> [f64; 3] {
    [1.0, 0.0, 1.0]
}

/// Subdivide every segment of the loop into `ceil(len / h)` equal pieces.
fn resample(l: &[(f64, f64)], h: f64) -> Vec<[f64; 2]> {
    let mut out = Vec::new();
    for i in 0..l.len() {
        let (a, b) = (l[i], l[(i + 1) % l.len()]);
        let len = (b.0 - a.0).hypot(b.1 - a.1);
        let k = ((len / h).ceil() as usize).max(1);
        for s in 0..k {
            let t = s as f64 / k as f64;
            out.push([a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1)]);
        }
    }
    out
}

#[test]
fn preflight_errors() {
    let o = PaveOptions::default();
    let sq = |n: usize| -> Vec<[f64; 2]> { resample(&loops2d::square().outer, 1.0 / n as f64) };
    assert_eq!(pave_region(&[vec![[0.0, 0.0], [1.0, 0.0]]], &id_metric, None, &o).err(), Some(PaveError::LoopTooSmall { ring: 0, nodes: 2 }));
    let mut odd = sq(2);
    odd.insert(1, [0.25, 0.0]);
    assert_eq!(pave_region(&[odd], &id_metric, None, &o).err(), Some(PaveError::OddBoundary { total: 9 }));
    let bow = vec![[0.0, 0.0], [2.0, 2.0], [2.0, 0.0], [0.0, 2.0]];
    assert_eq!(pave_region(&[bow], &id_metric, None, &o).err(), Some(PaveError::BoundaryIntersects));
}

#[test]
fn square_with_eight_boundary_nodes_is_four_quads() {
    let l = vec![resample(&loops2d::square().outer, 0.5)];
    assert_eq!(l[0].len(), 8);
    let m = pave_region(&l, &id_metric, None, &PaveOptions::default()).expect("pave");
    assert_eq!(m.points.len(), 9);
    assert_eq!(m.quads.len(), 4);
}
