//! Tet quality measures and statistics.
//!
//! * mean ratio `eta = 12 (3V)^(2/3) / sum(l^2)` (1 regular, 0 flat);
//! * radius-edge `R / l_min` (circumradius over shortest edge);
//! * dihedral angles in degrees.

use crate::geom::{V3, circumsphere, cross, dot, mean_ratio, norm2, sub};
use rayon::prelude::*;

/// Quality of one tet.
#[derive(Clone, Copy, Debug)]
pub struct TetQ {
    /// Mean ratio.
    pub mean_ratio: f64,
    /// Circumradius over shortest edge (infinite for flat tets).
    pub radius_edge: f64,
    /// Smallest dihedral angle (degrees).
    pub min_dihedral: f64,
    /// Largest dihedral angle (degrees).
    pub max_dihedral: f64,
}

/// Evaluate a tet (vertices in any order; positive or negative orientation).
pub fn tet_quality(p: [V3; 4]) -> TetQ {
    let eta = mean_ratio(p[0], p[1], p[2], p[3]).abs();
    let mut lmin2 = f64::INFINITY;
    for i in 0..4 {
        for j in i + 1..4 {
            lmin2 = lmin2.min(norm2(sub(p[i], p[j])));
        }
    }
    let radius_edge = match circumsphere(p[0], p[1], p[2], p[3]) {
        Some((_, r2)) if lmin2 > 0.0 => (r2 / lmin2).sqrt(),
        _ => f64::INFINITY,
    };
    // Outward unit normals of the faces opposite each vertex.
    let centroid = [
        0.25 * (p[0][0] + p[1][0] + p[2][0] + p[3][0]),
        0.25 * (p[0][1] + p[1][1] + p[2][1] + p[3][1]),
        0.25 * (p[0][2] + p[1][2] + p[2][2] + p[3][2]),
    ];
    let mut n = [[0.0; 3]; 4];
    for i in 0..4 {
        let (a, b, c) = (p[(i + 1) % 4], p[(i + 2) % 4], p[(i + 3) % 4]);
        let mut v = cross(sub(b, a), sub(c, a));
        let l = dot(v, v).sqrt();
        if l > 0.0 {
            v = [v[0] / l, v[1] / l, v[2] / l];
        }
        if dot(v, sub(centroid, a)) > 0.0 {
            v = [-v[0], -v[1], -v[2]];
        }
        n[i] = v;
    }
    let mut mn = f64::INFINITY;
    let mut mx = f64::NEG_INFINITY;
    for i in 0..4 {
        for j in i + 1..4 {
            let c = dot(n[i], n[j]).clamp(-1.0, 1.0);
            let ang = (std::f64::consts::PI - c.acos()).to_degrees();
            mn = mn.min(ang);
            mx = mx.max(ang);
        }
    }
    TetQ { mean_ratio: eta, radius_edge, min_dihedral: mn, max_dihedral: mx }
}

/// Summary statistics of a mesh, evaluated in parallel with order-independent reductions.
pub(crate) fn mesh_stats(points: &[V3], tets: &[[u32; 4]]) -> (f64, f64, f64, f64, f64) {
    if tets.is_empty() {
        return (0.0, 0.0, 0.0, 0.0, 0.0);
    }
    let qs: Vec<TetQ> = tets.par_iter().map(|t| tet_quality(t.map(|i| points[i as usize]))).collect();
    let min_d = qs.par_iter().map(|q| q.min_dihedral).reduce(|| f64::INFINITY, f64::min);
    let max_d = qs.par_iter().map(|q| q.max_dihedral).reduce(|| f64::NEG_INFINITY, f64::max);
    let max_re = qs.par_iter().map(|q| q.radius_edge).reduce(|| 0.0, f64::max);
    let min_mr = qs.par_iter().map(|q| q.mean_ratio).reduce(|| f64::INFINITY, f64::min);
    let mr: Vec<f64> = qs.iter().map(|q| q.mean_ratio).collect();
    let mean = sm_math::sum::par_sum(&mr) / qs.len() as f64;
    (min_d, max_d, max_re, min_mr, mean)
}
