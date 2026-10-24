//! Deterministic summary statistics and histograms.
//!
//! All reductions run over fixed-size chunks ([`CHUNK`]), independent of the thread count: each
//! chunk is reduced sequentially (compensated sums), chunk results are collected in order and
//! combined sequentially. The result is bit-identical for any number of threads.

use crate::error::{QualityError, Result};
use rayon::prelude::*;

/// Fixed reduction chunk length; part of the determinism contract (changing it changes the bits).
pub const CHUNK: usize = 4096;

/// Neumaier compensated accumulator.
#[derive(Clone, Copy, Default)]
pub(crate) struct Acc {
    s: f64,
    c: f64,
}
impl Acc {
    #[inline]
    pub(crate) fn add(&mut self, x: f64) {
        let t = self.s + x;
        self.c += if self.s.abs() >= x.abs() { (self.s - t) + x } else { (x - t) + self.s };
        self.s = t;
    }
    #[inline]
    pub(crate) fn value(self) -> f64 {
        self.s + self.c
    }
    pub(crate) fn merge(&mut self, o: Acc) {
        self.add(o.s);
        self.add(o.c);
    }
}

/// Summary statistics of one metric over a set of cells. Values that are NaN or infinite are
/// counted in `undefined` and excluded from everything else.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct Summary {
    /// Number of finite values.
    pub count: usize,
    /// Number of NaN/infinite values (metric undefined, inactive cell, non-finite input).
    pub undefined: usize,
    /// Smallest finite value (`NaN` if `count == 0`).
    pub min: f64,
    /// Index of the first cell attaining `min` (under `f64::total_cmp`).
    pub argmin: Option<usize>,
    /// Largest finite value (`NaN` if `count == 0`).
    pub max: f64,
    /// Index of the first cell attaining `max`.
    pub argmax: Option<usize>,
    /// Arithmetic mean (`NaN` if `count == 0`).
    pub mean: f64,
    /// Population standard deviation `sqrt(sum (x - mean)^2 / count)` (`NaN` if `count == 0`).
    pub std_dev: f64,
}

/// A value and the index where it occurs.
type Hit = (f64, usize);

struct Part {
    count: usize,
    undefined: usize,
    min: Option<Hit>,
    max: Option<Hit>,
    sum: Acc,
}

fn part(base: usize, chunk: &[f64]) -> Part {
    let mut p = Part { count: 0, undefined: 0, min: None, max: None, sum: Acc::default() };
    for (i, &x) in chunk.iter().enumerate() {
        if !x.is_finite() {
            p.undefined += 1;
            continue;
        }
        p.count += 1;
        p.sum.add(x);
        if p.min.is_none_or(|(m, _)| x.total_cmp(&m).is_lt()) {
            p.min = Some((x, base + i));
        }
        if p.max.is_none_or(|(m, _)| x.total_cmp(&m).is_gt()) {
            p.max = Some((x, base + i));
        }
    }
    p
}

/// Deterministic sum of squared deviations from `mean`.
fn sum_sq_dev(values: &[f64], mean: f64) -> f64 {
    let parts: Vec<Acc> = values
        .par_chunks(CHUNK)
        .map(|c| {
            let mut a = Acc::default();
            for &x in c.iter().filter(|x| x.is_finite()) {
                a.add((x - mean) * (x - mean));
            }
            a
        })
        .collect();
    let mut t = Acc::default();
    for p in parts {
        t.merge(p);
    }
    t.value()
}

/// Summarise `values` (typically one metric column). Deterministic for any thread count.
pub fn summarize(values: &[f64]) -> Summary {
    let parts: Vec<Part> = values.par_chunks(CHUNK).enumerate().map(|(i, c)| part(i * CHUNK, c)).collect();
    let (mut count, mut undefined) = (0usize, 0usize);
    let (mut min, mut max): (Option<Hit>, Option<Hit>) = (None, None);
    let mut sum = Acc::default();
    for p in parts {
        count += p.count;
        undefined += p.undefined;
        sum.merge(p.sum);
        // Strict comparison: on ties the earlier chunk (lower index) wins.
        if let Some(m) = p.min
            && min.is_none_or(|(c, _)| m.0.total_cmp(&c).is_lt())
        {
            min = Some(m);
        }
        if let Some(m) = p.max
            && max.is_none_or(|(c, _)| m.0.total_cmp(&c).is_gt())
        {
            max = Some(m);
        }
    }
    if count == 0 {
        return Summary { count, undefined, min: f64::NAN, argmin: None, max: f64::NAN, argmax: None, mean: f64::NAN, std_dev: f64::NAN };
    }
    let mean = sum.value() / count as f64;
    let std_dev = (sum_sq_dev(values, mean) / count as f64).sqrt();
    Summary {
        count,
        undefined,
        min: min.map_or(f64::NAN, |m| m.0),
        argmin: min.map(|m| m.1),
        max: max.map_or(f64::NAN, |m| m.0),
        argmax: max.map(|m| m.1),
        mean,
        std_dev,
    }
}

/// Equal-width histogram on `[lo, hi]`.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct Histogram {
    /// Lower edge of the first bin.
    pub lo: f64,
    /// Upper edge of the last bin (a value equal to `hi` falls in the last bin).
    pub hi: f64,
    /// Counts per bin.
    pub counts: Vec<u64>,
    /// Finite values below `lo`.
    pub below: u64,
    /// Finite values above `hi`.
    pub above: u64,
    /// NaN/infinite values.
    pub undefined: u64,
}

impl Histogram {
    /// Left edge of bin `i` (`i == bins` gives `hi`).
    pub fn edge(&self, i: usize) -> f64 {
        self.lo + (self.hi - self.lo) * i as f64 / self.counts.len() as f64
    }
    /// Total number of finite values counted (in range or not).
    pub fn total(&self) -> u64 {
        self.counts.iter().sum::<u64>() + self.below + self.above
    }
}

/// Histogram of `values` over `[lo, hi]` with `bins` bins. Bin counts are integers, so the
/// merge over chunks is exact and thread-count independent.
pub fn histogram(values: &[f64], lo: f64, hi: f64, bins: usize) -> Result<Histogram> {
    if bins == 0 || !lo.is_finite() || !hi.is_finite() || hi <= lo {
        return Err(QualityError::BadArgument(format!("histogram needs bins > 0 and finite lo < hi, got bins={bins}, lo={lo}, hi={hi}")));
    }
    let w = hi - lo;
    let parts: Vec<(Vec<u64>, [u64; 3])> = values
        .par_chunks(CHUNK)
        .map(|c| {
            let mut counts = vec![0u64; bins];
            let mut other = [0u64; 3]; // below, above, undefined
            for &x in c {
                if !x.is_finite() {
                    other[2] += 1;
                } else if x < lo {
                    other[0] += 1;
                } else if x > hi {
                    other[1] += 1;
                } else {
                    counts[(((x - lo) / w * bins as f64) as usize).min(bins - 1)] += 1;
                }
            }
            (counts, other)
        })
        .collect();
    let mut h = Histogram { lo, hi, counts: vec![0; bins], below: 0, above: 0, undefined: 0 };
    for (counts, o) in parts {
        for (a, b) in h.counts.iter_mut().zip(counts) {
            *a += b;
        }
        h.below += o[0];
        h.above += o[1];
        h.undefined += o[2];
    }
    Ok(h)
}

/// Histogram over the finite range of `values` (a single-valued set gets a unit-wide range).
pub fn histogram_auto(values: &[f64], bins: usize) -> Result<Histogram> {
    let s = summarize(values);
    if s.count == 0 {
        return Err(QualityError::BadArgument("no finite values to histogram".into()));
    }
    let (lo, hi) = if s.max > s.min { (s.min, s.max) } else { (s.min - 0.5, s.min + 0.5) };
    histogram(values, lo, hi, bins)
}
