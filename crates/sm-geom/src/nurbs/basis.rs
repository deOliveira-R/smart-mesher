//! B-spline basis functions and their first two derivatives (Piegl and Tiller A2.1, A2.3),
//! allocation-free and sized to the degree so the scratch arrays stay small.

#![allow(clippy::needless_range_loop)] // index-heavy numeric kernels

/// Index `s` of the knot span with `k[s] <= t < k[s+1]` among `npoles` poles of degree `p`
/// (`p <= s < npoles`). `t` at or past the last knot gives the last non-empty span.
pub(crate) fn find_span(npoles: usize, p: usize, t: f64, k: &[f64]) -> usize {
    let n = npoles - 1;
    if t >= k[n + 1] {
        let mut s = n;
        while s > p && k[s] >= k[n + 1] {
            s -= 1;
        }
        return s;
    }
    if t <= k[p] {
        return p;
    }
    let (mut lo, mut hi) = (p, n + 1);
    let mut mid = (lo + hi) / 2;
    while t < k[mid] || t >= k[mid + 1] {
        if t < k[mid] {
            hi = mid;
        } else {
            lo = mid;
        }
        mid = (lo + hi) / 2;
    }
    mid
}

/// Basis function values `N_{span-p+j,p}(t)`, `j = 0..=p` (A2.2). `N >= p + 1`.
#[inline]
pub(crate) fn values_n<const N: usize>(span: usize, t: f64, p: usize, k: &[f64]) -> [f64; N] {
    let mut n = [0.0f64; N];
    let (mut left, mut right) = ([0.0f64; N], [0.0f64; N]);
    n[0] = 1.0;
    for j in 1..=p {
        left[j] = t - k[span + 1 - j];
        right[j] = k[span + j] - t;
        let mut saved = 0.0;
        for r in 0..j {
            let temp = n[r] / (right[r + 1] + left[j - r]);
            n[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        n[j] = saved;
    }
    n
}

/// `ders[d][j]` = d-th derivative (d = 0, 1, 2) of `N_{span-p+j,p}` at `t`.
/// `N` is the scratch width and must be at least `p + 1`.
pub(crate) fn ders_n<const N: usize>(span: usize, t: f64, p: usize, k: &[f64]) -> [[f64; N]; 3] {
    let mut ndu = [[0.0f64; N]; N];
    let (mut left, mut right) = ([0.0f64; N], [0.0f64; N]);
    ndu[0][0] = 1.0;
    for j in 1..=p {
        left[j] = t - k[span + 1 - j];
        right[j] = k[span + j] - t;
        let mut saved = 0.0;
        for r in 0..j {
            ndu[j][r] = right[r + 1] + left[j - r];
            let temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
    let mut ders = [[0.0f64; N]; 3];
    for j in 0..=p {
        ders[0][j] = ndu[j][p];
    }
    let nd = 2.min(p);
    let mut a = [[0.0f64; 3]; 2];
    for r in 0..=p {
        let (mut s1, mut s2) = (0usize, 1usize);
        a[0][0] = 1.0;
        for kk in 1..=nd {
            let mut d = 0.0;
            let rk = r as i64 - kk as i64;
            let pk = (p - kk) as i64;
            if rk >= 0 {
                a[s2][0] = a[s1][0] / ndu[(pk + 1) as usize][rk as usize];
                d = a[s2][0] * ndu[rk as usize][pk as usize];
            }
            let j1 = if rk >= -1 { 1 } else { -rk };
            let j2 = if r as i64 - 1 <= pk { kk as i64 - 1 } else { (p - r) as i64 };
            for j in j1..=j2 {
                let ju = j as usize;
                let idx = (rk + j) as usize;
                a[s2][ju] = (a[s1][ju] - a[s1][ju - 1]) / ndu[(pk + 1) as usize][idx];
                d += a[s2][ju] * ndu[idx][pk as usize];
            }
            if r as i64 <= pk {
                a[s2][kk] = -a[s1][kk - 1] / ndu[(pk + 1) as usize][r];
                d += a[s2][kk] * ndu[r][pk as usize];
            }
            ders[kk][r] = d;
            core::mem::swap(&mut s1, &mut s2);
        }
    }
    let mut f = p as f64;
    for kk in 1..=nd {
        for j in 0..=p {
            ders[kk][j] *= f;
        }
        f *= (p - kk) as f64;
    }
    ders
}

/// Basis derivatives in a fixed 16-wide array, picking the smallest scratch size that fits.
pub(crate) fn ders(span: usize, t: f64, p: usize, k: &[f64]) -> [[f64; 16]; 3] {
    let mut out = [[0.0f64; 16]; 3];
    macro_rules! go {
        ($n:literal) => {{
            let d = ders_n::<$n>(span, t, p, k);
            for (o, s) in out.iter_mut().zip(d.iter()) {
                o[..$n].copy_from_slice(s);
            }
        }};
    }
    if p < 4 {
        go!(4)
    } else if p < 8 {
        go!(8)
    } else {
        go!(16)
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn partition_of_unity_and_derivative_sums() {
        let k = [0.0, 0.0, 0.0, 0.0, 1.0, 2.5, 4.0, 4.0, 4.0, 4.0];
        for i in 0..=40 {
            let t = 4.0 * i as f64 / 40.0;
            let s = find_span(6, 3, t, &k);
            let d = ders(s, t, 3, &k);
            for (order, sum) in d.iter().map(|r| r[..4].iter().sum::<f64>()).enumerate() {
                let want = if order == 0 { 1.0 } else { 0.0 };
                assert!((sum - want).abs() < 1e-13, "order {order} at {t}: {sum}");
            }
        }
    }

    #[test]
    fn matches_finite_differences() {
        let k = [0.0, 0.0, 0.0, 1.0, 3.0, 3.0, 3.0];
        let t = 1.7;
        let s = find_span(4, 2, t, &k);
        let h = 1e-6;
        let (dp, dm, d0) = (ders(s, t + h, 2, &k), ders(s, t - h, 2, &k), ders(s, t, 2, &k));
        for j in 0..3 {
            assert!((d0[1][j] - (dp[0][j] - dm[0][j]) / (2.0 * h)).abs() < 1e-8);
            assert!((d0[2][j] - (dp[1][j] - dm[1][j]) / (2.0 * h)).abs() < 1e-7);
        }
    }
}
