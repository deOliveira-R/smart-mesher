//! Shared test support: an exact `i128` oracle independent of the crate's expansion code, and a
//! tiny deterministic RNG.
#![allow(dead_code)]

use sm_math::Sign;

/// SplitMix64: deterministic, seedable.
pub struct Rng(pub u64);

impl Rng {
    pub fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    /// Uniform integer in `[-m, m]`.
    pub fn int(&mut self, m: i64) -> i64 {
        (self.next() % (2 * m as u64 + 1)) as i64 - m
    }
    pub fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
}

pub fn sgn(x: i128) -> Sign {
    match x.signum() {
        -1 => Sign::Negative,
        0 => Sign::Zero,
        _ => Sign::Positive,
    }
}

/// Determinant by Laplace expansion along the first row (exact in i128).
pub fn det(m: &[Vec<i128>]) -> i128 {
    let n = m.len();
    if n == 1 {
        return m[0][0];
    }
    let mut acc = 0;
    for c in 0..n {
        if m[0][c] == 0 {
            continue;
        }
        let minor: Vec<Vec<i128>> = m[1..]
            .iter()
            .map(|r| {
                r.iter()
                    .enumerate()
                    .filter(|&(j, _)| j != c)
                    .map(|(_, &v)| v)
                    .collect()
            })
            .collect();
        let t = m[0][c] * det(&minor);
        acc += if c % 2 == 0 { t } else { -t };
    }
    acc
}

fn lift(p: &[i64]) -> i128 {
    p.iter().map(|&x| i128::from(x) * i128::from(x)).sum()
}

/// Raw-matrix definitions: rows `[coords.., |coords|^2 (circle/sphere only), 1]`.
pub fn orient2d_i(a: [i64; 2], b: [i64; 2], c: [i64; 2]) -> Sign {
    let row = |p: [i64; 2]| vec![i128::from(p[0]), i128::from(p[1]), 1];
    sgn(det(&[row(a), row(b), row(c)]))
}

pub fn orient3d_i(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> Sign {
    let row = |p: [i64; 3]| vec![i128::from(p[0]), i128::from(p[1]), i128::from(p[2]), 1];
    sgn(det(&[row(a), row(b), row(c), row(d)]))
}

pub fn incircle_i(a: [i64; 2], b: [i64; 2], c: [i64; 2], d: [i64; 2]) -> Sign {
    let row = |p: [i64; 2]| vec![i128::from(p[0]), i128::from(p[1]), lift(&p), 1];
    sgn(det(&[row(a), row(b), row(c), row(d)]))
}

pub fn insphere_i(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3], e: [i64; 3]) -> Sign {
    let row = |p: [i64; 3]| {
        vec![
            i128::from(p[0]),
            i128::from(p[1]),
            i128::from(p[2]),
            lift(&p),
            1,
        ]
    };
    sgn(det(&[row(a), row(b), row(c), row(d), row(e)]))
}

pub fn f2(p: [i64; 2], k: i32) -> [f64; 2] {
    let s = 2f64.powi(k);
    [p[0] as f64 * s, p[1] as f64 * s]
}

pub fn f3(p: [i64; 3], k: i32) -> sm_math::Point3 {
    let s = 2f64.powi(k);
    sm_math::Point3::new(p[0] as f64 * s, p[1] as f64 * s, p[2] as f64 * s)
}
