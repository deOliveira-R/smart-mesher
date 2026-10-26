//! Smart-Laplacian smoothing of the interior points in the metric frame.

use super::metric::Met;
use sm_math::{Sign, orient2d};

/// Minimum corner sine of a quad, in the metric frame of its first vertex. Negative when a
/// corner turns the wrong way.
pub(crate) fn quad_quality(pts: &[[f64; 2]], met: &[Met], q: &[u32; 4]) -> f64 {
    let b = met[q[0] as usize];
    let c: [[f64; 2]; 4] = core::array::from_fn(|k| b.apply(pts[q[k] as usize]));
    let mut m = f64::INFINITY;
    for k in 0..4 {
        let (p0, p1, p2) = (c[(k + 3) % 4], c[k], c[(k + 1) % 4]);
        let e1 = [p1[0] - p0[0], p1[1] - p0[1]];
        let e2 = [p2[0] - p1[0], p2[1] - p1[1]];
        let l = e1[0].hypot(e1[1]) * e2[0].hypot(e2[1]);
        let s = if l > 0.0 { (e1[0] * e2[1] - e1[1] * e2[0]) / l } else { -1.0 };
        m = m.min(s);
    }
    m
}

/// All four corners strictly counter-clockwise (exact).
pub(crate) fn quad_valid(pts: &[[f64; 2]], q: &[u32; 4]) -> bool {
    (0..4).all(|k| orient2d(pts[q[(k + 3) % 4] as usize], pts[q[k] as usize], pts[q[(k + 1) % 4] as usize]) == Sign::Positive)
}

/// Gauss-Seidel smart Laplacian over the points `nb..`, `iters` sweeps in index order. A move
/// is kept only if every incident quad stays strictly convex, the worst incident quality does
/// not drop and the mean incident quality rises.
pub(crate) fn smooth(pts: &mut [[f64; 2]], met: &[Met], quads: &[[u32; 4]], nb: usize, iters: u32) {
    let n = pts.len();
    if n <= nb || quads.is_empty() {
        return;
    }
    // Incident quads and edge neighbours (CSR, sorted, unique).
    let mut qdeg = vec![0u32; n + 1];
    for q in quads {
        for v in q {
            qdeg[*v as usize + 1] += 1;
        }
    }
    for i in 0..n {
        qdeg[i + 1] += qdeg[i];
    }
    let mut fill = qdeg.clone();
    let mut inc = vec![0u32; qdeg[n] as usize];
    for (qi, q) in quads.iter().enumerate() {
        for v in q {
            inc[fill[*v as usize] as usize] = qi as u32;
            fill[*v as usize] += 1;
        }
    }
    let mut nbr: Vec<Vec<u32>> = vec![Vec::new(); n];
    for q in quads {
        for k in 0..4 {
            let (a, b) = (q[k], q[(k + 1) % 4]);
            nbr[a as usize].push(b);
            nbr[b as usize].push(a);
        }
    }
    for l in &mut nbr {
        l.sort_unstable();
        l.dedup();
    }
    let measure = |pts: &[[f64; 2]], v: usize| -> Option<(f64, f64)> {
        let (mut mn, mut sum, mut cnt) = (f64::INFINITY, 0.0, 0usize);
        for &qi in &inc[qdeg[v] as usize..qdeg[v + 1] as usize] {
            let q = &quads[qi as usize];
            if !quad_valid(pts, q) {
                return None;
            }
            let s = quad_quality(pts, met, q);
            mn = mn.min(s);
            sum += s;
            cnt += 1;
        }
        (cnt > 0).then(|| (mn, sum / cnt as f64))
    };
    for _ in 0..iters {
        let mut moved = 0.0f64;
        for v in nb..n {
            if nbr[v].is_empty() {
                continue;
            }
            let Some((old_min, old_avg)) = measure(pts, v) else { continue };
            let mut t = [0.0, 0.0];
            for &u in &nbr[v] {
                t[0] += pts[u as usize][0];
                t[1] += pts[u as usize][1];
            }
            let k = nbr[v].len() as f64;
            t = [t[0] / k, t[1] / k];
            let p0 = pts[v];
            for omega in [1.0, 0.5, 0.25] {
                let cand = [p0[0] + omega * (t[0] - p0[0]), p0[1] + omega * (t[1] - p0[1])];
                pts[v] = cand;
                if let Some((mn, avg)) = measure(pts, v)
                    && mn >= old_min - 1e-12
                    && avg > old_avg + 1e-12
                {
                    moved = moved.max((cand[0] - p0[0]).abs().max((cand[1] - p0[1]).abs()));
                    break;
                }
                pts[v] = p0;
            }
        }
        if moved < 1e-12 {
            break;
        }
    }
}
