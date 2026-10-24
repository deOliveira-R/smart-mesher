//! Spacing math in normalized arc-length coordinates.
//!
//! Every function returns the **interior** node positions as fractions of the total arc length,
//! strictly increasing inside `(0, 1)` (`n - 1` values for `n` intervals); the endpoints are the
//! curve's vertices. Conversion to curve-parameter fractions is the arc-length map's job.
//!
//! Geometric series are evaluated with `exp_m1` and scaled exponentials, so ratios near 1 and
//! very large `n * ln(ratio)` neither cancel nor overflow.

use crate::error::{Result, SpacingError};

/// Largest accepted `|ln ratio|` of a bias series.
const LAMBDA_CAP: f64 = 700.0;
/// Most intervals the fine/coarse count search considers.
const MAX_COUNT: u64 = 1 << 26;

fn positive(what: &'static str, value: f64) -> Result<f64> {
    if value.is_finite() && value > 0.0 { Ok(value) } else { Err(SpacingError::BadValue { what, value }) }
}

/// `ln |e^x - 1|` without cancellation or overflow.
fn ln_abs_expm1(x: f64) -> f64 {
    if x > 30.0 {
        x + (-(-x).exp()).ln_1p()
    } else if x > 0.0 {
        x.exp_m1().ln()
    } else {
        (-x.exp_m1()).ln()
    }
}

/// Fail unless the fractions strictly increase inside `(0, 1)`.
fn check_fractions(f: &[f64]) -> Result<()> {
    let mut prev = 0.0;
    for &x in f {
        if !(x.is_finite() && x > prev && x < 1.0) {
            return Err(SpacingError::Degenerate("spacing too extreme: interior nodes coincide in double precision".into()));
        }
        prev = x;
    }
    Ok(())
}

/// Find the root of an increasing `f` by bracketing then a safeguarded Newton iteration (finite
/// difference slope, bisection whenever the step leaves the bracket). `|x|` is limited to `cap`.
fn monotone_root(f: impl Fn(f64) -> f64, cap: f64) -> Result<f64> {
    let unreachable = || SpacingError::Unreachable("no spacing parameter satisfies the requested size on this curve".into());
    let w = cap.min(1.0);
    let (mut lo, mut hi) = (-w, w);
    let (mut flo, mut fhi) = (f(lo), f(hi));
    while flo > 0.0 {
        hi = lo;
        fhi = flo;
        lo *= 2.0;
        if lo.abs() > cap {
            return Err(unreachable());
        }
        flo = f(lo);
    }
    while fhi < 0.0 {
        lo = hi;
        flo = fhi;
        hi *= 2.0;
        if hi > cap {
            return Err(unreachable());
        }
        fhi = f(hi);
    }
    if !(flo.is_finite() && fhi.is_finite()) {
        return Err(unreachable());
    }
    let mut x = if fhi == 0.0 { hi } else { 0.5 * (lo + hi) };
    for _ in 0..200 {
        let fx = f(x);
        if fx == 0.0 {
            return Ok(x);
        }
        if fx < 0.0 {
            lo = x;
        } else {
            hi = x;
        }
        let h = 1e-7 * (1.0 + x.abs());
        let d = (f(x + h) - f(x - h)) / (2.0 * h);
        let mut nx = x - fx / d;
        if !(nx.is_finite() && nx > lo && nx < hi) {
            nx = 0.5 * (lo + hi);
        }
        let done = (nx - x).abs() <= 1e-15 * (1.0 + x.abs());
        x = nx;
        if done {
            break;
        }
    }
    Ok(x)
}

/// Interior fractions of the geometric series with `n` intervals and edge ratio `ratio`
/// (`e_{k+1} = ratio * e_k`): `s_k = (r^k - 1) / (r^n - 1)`, and `k / n` for `ratio == 1`.
pub fn bias_fractions(n: u32, ratio: f64) -> Result<Vec<f64>> {
    if n == 0 {
        return Err(SpacingError::ZeroCount);
    }
    let lam = positive("ratio", ratio)?.ln();
    let nf = f64::from(n);
    let out: Vec<f64> = (1..n).map(|k| bias_node(f64::from(k), nf, lam)).collect();
    check_fractions(&out)?;
    Ok(out)
}

fn bias_node(k: f64, n: f64, lam: f64) -> f64 {
    if lam == 0.0 {
        k / n
    } else if lam > 0.0 {
        ((k - n) * lam).exp() * (-(-k * lam).exp_m1()) / (-(-n * lam).exp_m1())
    } else {
        (k * lam).exp_m1() / (n * lam).exp_m1()
    }
}

/// Fraction of the length taken by the first edge of the geometric series.
pub fn bias_first_fraction(n: u32, ratio: f64) -> Result<f64> {
    if n == 0 {
        return Err(SpacingError::ZeroCount);
    }
    let lam = positive("ratio", ratio)?.ln();
    Ok(bias_node(1.0, f64::from(n), lam))
}

/// The edge ratio of the geometric series with `n` intervals whose first edge is the fraction
/// `alpha = first / length` of the length (monotone root; `n == 1` has no ratio and gives 1).
pub fn bias_ratio_from_first(n: u32, alpha: f64) -> Result<f64> {
    if n == 0 {
        return Err(SpacingError::ZeroCount);
    }
    if n == 1 {
        return Ok(1.0);
    }
    if !(alpha.is_finite() && alpha > 0.0) {
        return Err(SpacingError::BadValue { what: "first size", value: alpha });
    }
    if alpha >= 1.0 {
        return Err(SpacingError::Unreachable(format!("first size is {alpha} of the length; with {n} intervals it must be below the length")));
    }
    let nf = f64::from(n);
    let ln_first = |lam: f64| if lam == 0.0 { -nf.ln() } else { ln_abs_expm1(lam) - ln_abs_expm1(nf * lam) };
    let lam = monotone_root(|l| alpha.ln() - ln_first(l), LAMBDA_CAP)?;
    Ok(lam.exp())
}

/// The interval count of the series with first edge `first`, ratio `ratio` and total `length`:
/// the real-valued count rounded half-to-even, at least 1. Errors if the series cannot reach
/// `length` (ratio below 1 with `first / (1 - ratio) <= length`).
pub fn bias_count_from_ratio(length: f64, first: f64, ratio: f64) -> Result<u32> {
    let (length, first) = (positive("length", length)?, positive("first size", first)?);
    let lam = positive("ratio", ratio)?.ln();
    let n = if lam == 0.0 {
        length / first
    } else {
        let x = (length / first) * lam.exp_m1();
        if x <= -1.0 {
            return Err(SpacingError::Unreachable(format!("a series with first edge {first} and ratio {ratio} sums to less than the length {length}")));
        }
        x.ln_1p() / lam
    };
    if !(n.is_finite() && n < MAX_COUNT as f64) {
        return Err(SpacingError::Unreachable("interval count out of range".into()));
    }
    Ok(n.round_ties_even().max(1.0) as u32)
}

/// The interval count of a geometric series that starts with edge `fine`, ends with edge `coarse`
/// and best matches `length`: the integer minimizing `|sum - length|` (the sum grows with the
/// count, so a doubling search then bisection finds it).
pub fn bias_count_from_sizes(length: f64, fine: f64, coarse: f64) -> Result<u32> {
    let (length, fine, coarse) = (positive("length", length)?, positive("first size", fine)?, positive("last size", coarse)?);
    let ln_ratio = (coarse / fine).ln();
    let total = |n: u64| -> f64 {
        if n == 1 {
            return fine;
        }
        let lam = ln_ratio / (n - 1) as f64;
        if lam == 0.0 { n as f64 * fine } else { fine * (n as f64 * lam).exp_m1() / lam.exp_m1() }
    };
    let mut hi = 1u64;
    while total(hi) < length {
        hi *= 2;
        if hi > MAX_COUNT {
            return Err(SpacingError::Unreachable("interval count out of range".into()));
        }
    }
    let mut lo = hi / 2; // total(lo) < length (or lo == 0)
    while hi - lo > 1 {
        let mid = lo + (hi - lo) / 2;
        if total(mid) >= length { hi = mid } else { lo = mid }
    }
    let n = if lo >= 1 && (length - total(lo)) < (total(hi) - length) { lo } else { hi };
    Ok(n as u32)
}

/// Interior fractions of the dual geometric series: edges `a r^j` from both ends meeting in the
/// middle. Even `n = 2m`: `m` edges per half. Odd `n = 2m + 1`: the middle edge is `a r^m`.
/// `ratio > 1` is coarse in the middle. Exactly mirror symmetric.
pub fn dualbias_fractions(n: u32, ratio: f64) -> Result<Vec<f64>> {
    if n == 0 {
        return Err(SpacingError::ZeroCount);
    }
    let lam = positive("ratio", ratio)?.ln();
    let (m, odd) = ((n / 2) as usize, n % 2 == 1);
    // Scaled weights w_j = exp((j - shift) lam) <= ~1, so extreme ratios cannot overflow.
    let shift = if lam > 0.0 { m as f64 } else { 0.0 };
    let w = |j: usize| ((j as f64 - shift) * lam).exp();
    let mut cum = Vec::with_capacity(m + 1);
    cum.push(0.0);
    for j in 0..m {
        cum.push(cum[j] + w(j));
    }
    let total = 2.0 * cum[m] + if odd { w(m) } else { 0.0 };
    let out: Vec<f64> = (1..n as usize).map(|k| if k <= m { cum[k] / total } else { 1.0 - cum[n as usize - k] / total }).collect();
    check_fractions(&out)?;
    Ok(out)
}

/// `ln` of the unscaled dual series sum `T(lam)` in units of the first edge.
fn ln_dual_total(m: usize, odd: bool, lam: f64) -> f64 {
    let mf = m as f64;
    let e = if lam == 0.0 { mf } else { (mf * lam).exp_m1() / lam.exp_m1() };
    (2.0 * e + if odd { (mf * lam).exp() } else { 0.0 }).ln()
}

/// The ratio of the dual series with `n` intervals whose end edges are the fraction
/// `alpha = first / length` of the length. Needs `alpha < 1/2`; `n == 2` fixes `alpha = 1/2`.
pub fn dualbias_ratio_from_first(n: u32, alpha: f64) -> Result<f64> {
    if n == 0 {
        return Err(SpacingError::ZeroCount);
    }
    if !(alpha.is_finite() && alpha > 0.0) {
        return Err(SpacingError::BadValue { what: "first size", value: alpha });
    }
    if n == 1 {
        return Ok(1.0);
    }
    if n == 2 {
        if (alpha - 0.5).abs() <= 1e-9 {
            return Ok(1.0);
        }
        return Err(SpacingError::Unreachable("dualbias with 2 intervals forces first size = length / 2".into()));
    }
    if alpha >= 0.5 {
        return Err(SpacingError::Unreachable(format!("first size is {alpha} of the length; both end edges must fit, so it must be below half")));
    }
    let (m, odd) = ((n / 2) as usize, n % 2 == 1);
    let lam = monotone_root(|l| ln_dual_total(m, odd, l) + alpha.ln(), 600.0 / m as f64)?;
    Ok(lam.exp())
}

/// Interior fractions of a stretched spacing (see [`StretchSpec`]).
pub fn stretch_fractions(n: u32, spec: &StretchSpec) -> Result<Vec<f64>> {
    if n == 0 {
        return Err(SpacingError::ZeroCount);
    }
    let StretchSpec { length, first, last, max_ratio } = *spec;
    let (length, first) = (positive("length", length)?, positive("first size", first)?);
    if !(max_ratio.is_finite() && max_ratio >= 1.0) {
        return Err(SpacingError::BadValue { what: "max_ratio", value: max_ratio });
    }
    if n == 1 {
        return Ok(Vec::new());
    }
    let limit = max_ratio * (1.0 + 1e-9);
    let jump = |a: f64, b: f64| (a / b).max(b / a);
    let Some(last) = last else {
        let r = bias_ratio_from_first(n, first / length)?;
        let needed = jump(r, 1.0);
        if needed > limit {
            return Err(SpacingError::RatioLimit { needed, allowed: max_ratio });
        }
        return bias_fractions(n, r);
    };
    let last = positive("last size", last)?;
    let n_us = n as usize;
    let mut e = vec![0.0; n_us];
    e[0] = first;
    if n == 2 {
        e[1] = length - first;
        if e[1] <= 0.0 {
            return Err(SpacingError::Unreachable("first size must be below the length".into()));
        }
    } else {
        // Linear profile between the pinned end edges, interior scaled to close the length.
        let nm1 = (n_us - 1) as f64;
        let lin = |k: usize| first + (last - first) * k as f64 / nm1;
        let interior: f64 = (1..n_us - 1).map(lin).sum();
        let room = length - first - last;
        if room <= 0.0 {
            return Err(SpacingError::Unreachable(format!("first size {first} plus last size {last} leave no room on a curve of length {length}")));
        }
        let sigma = room / interior;
        for (k, ek) in e.iter_mut().enumerate().take(n_us - 1).skip(1) {
            *ek = sigma * lin(k);
        }
        e[n_us - 1] = last;
    }
    let needed = e.windows(2).map(|w| jump(w[0], w[1])).fold(1.0, f64::max);
    if needed > limit {
        return Err(SpacingError::RatioLimit { needed, allowed: max_ratio });
    }
    let mut acc = 0.0;
    let out: Vec<f64> = e[..n_us - 1]
        .iter()
        .map(|x| {
            acc += x;
            acc / length
        })
        .collect();
    check_fractions(&out)?;
    Ok(out)
}

/// Inputs of a stretched spacing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StretchSpec {
    /// Total arc length.
    pub length: f64,
    /// First edge (pinned).
    pub first: f64,
    /// Last edge (pinned for 3 or more intervals); `None` is a geometric series from `first`.
    pub last: Option<f64>,
    /// Largest allowed ratio of consecutive edge lengths (either direction), at least 1.
    pub max_ratio: f64,
}

/// Mirror a fraction list: the spacing as seen from the far end.
pub fn reversed(f: &[f64]) -> Vec<f64> {
    f.iter().rev().map(|x| 1.0 - x).collect()
}
