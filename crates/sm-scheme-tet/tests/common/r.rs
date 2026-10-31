//! Test-side tiled relief and an independent implementation of the INT-7 spacing `r`.

use super::{input_of, point_tri_dist};
use sm_scheme_tet::{SurfaceInput, TetMesh};
use std::collections::BTreeMap;

/// `tri_cube(n)` whose top face carries hard relief in every `k x k` tile (tiles of the
/// `n / k` x `n / k` grid): interior vertices of each tile pushed down by up to `depth * k / n`
/// (SplitMix64 in point order). Tile borders stay flat, so the patches are independent.
pub fn tiled_cube(n: u32, k: u32, seed: u64, depth: f64) -> SurfaceInput {
    let f = sm_testkit::surf3::tri_cube(n);
    let mut inp = input_of(&f, 0);
    let mut rng = sm_testkit::surf3::SplitMix64(seed);
    let lim = f64::from(k) / f64::from(n);
    for p in &mut inp.points {
        if p[2] == 1.0 && p[0] > 0.0 && p[0] < 1.0 && p[1] > 0.0 && p[1] < 1.0 {
            let (ix, iy) = ((p[0] * f64::from(n)).round() as u32, (p[1] * f64::from(n)).round() as u32);
            if ix % k != 0 && iy % k != 0 {
                p[2] = 1.0 - depth * lim * rng.unit().abs();
            }
        }
    }
    inp
}

/// Independent implementation of the normalized spacing `r = d / h_loc` (ruling INT-7) for the
/// default options (`FromBoundary`, gradation 1.3): nearest input triangle by a bucket grid,
/// `h_loc = h_T + 0.3 d`, clipped at `1.5 / (1 + 0.3 * 1.5)`. Returns the sorted `r` of the
/// Steiner points.
pub fn spacing_r(m: &TetMesh, inp: &SurfaceInput) -> Vec<f64> {
    let e = |x: [f64; 3], y: [f64; 3]| ((x[0] - y[0]).powi(2) + (x[1] - y[1]).powi(2) + (x[2] - y[2]).powi(2)).sqrt();
    let tri: Vec<[[f64; 3]; 3]> = inp.tris.iter().map(|t| t.map(|i| inp.points[i as usize])).collect();
    let h: Vec<f64> = tri.iter().map(|t| (e(t[0], t[1]) + e(t[1], t[2]) + e(t[2], t[0])) / 3.0).collect();
    let hmax = h.iter().copied().fold(0.0, f64::max);
    let cell = 3.0 * hmax;
    let key = |p: [f64; 3]| [(p[0] / cell).floor() as i64, (p[1] / cell).floor() as i64, (p[2] / cell).floor() as i64];
    let mut grid: BTreeMap<[i64; 3], Vec<u32>> = BTreeMap::new();
    for (i, t) in tri.iter().enumerate() {
        let c = [(t[0][0] + t[1][0] + t[2][0]) / 3.0, (t[0][1] + t[1][1] + t[2][1]) / 3.0, (t[0][2] + t[1][2] + t[2][2]) / 3.0];
        grid.entry(key(c)).or_default().push(i as u32);
    }
    let cap = 1.5 / (1.0 + 0.3 * 1.5);
    let mut out = Vec::new();
    for p in &m.points[m.n_input..] {
        let k = key(*p);
        let mut best: Option<(f64, usize)> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    for &i in grid.get(&[k[0] + dx, k[1] + dy, k[2] + dz]).map_or(&[][..], |v| &v[..]) {
                        let t = tri[i as usize];
                        let d = point_tri_dist(*p, t[0], t[1], t[2]);
                        if best.is_none_or(|b| d < b.0) {
                            best = Some((d, i as usize));
                        }
                    }
                }
            }
        }
        out.push(match best {
            Some((d, i)) => (d / (h[i] + 0.3 * d)).min(cap),
            None => cap,
        });
    }
    out.sort_by(f64::total_cmp);
    out
}

/// `(count, mean, std dev, p01, min)` of sorted `r`.
pub fn r_summary(r: &[f64]) -> (usize, f64, f64, f64, f64) {
    if r.is_empty() {
        return (0, 0.0, 0.0, 0.0, 0.0);
    }
    let n = r.len() as f64;
    let mean = r.iter().sum::<f64>() / n;
    let sd = (r.iter().map(|x| (x - mean) * (x - mean)).sum::<f64>() / n).sqrt();
    (r.len(), mean, sd, r[((r.len() - 1) as f64 * 0.01).floor() as usize], r[0])
}
