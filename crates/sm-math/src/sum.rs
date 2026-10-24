//! Compensated summation and deterministic parallel reductions.
//!
//! [`KahanSum`] and [`NeumaierSum`] carry a running error term so that the result is accurate
//! to about one rounding error regardless of length (error bound `2ε|Σx| + O(nε²)Σ|x|`).
//! Neumaier's variant also handles terms larger than the running sum, so prefer it; Kahan is
//! kept for completeness. Rust never reassociates float operations, so the compensation is not
//! optimised away.
//!
//! # Deterministic parallel sums
//! A parallel floating-point sum is only reproducible if the *reduction tree* is fixed. The
//! `par_*` functions therefore split the index range into chunks of a fixed length (a parameter
//! or [`DEFAULT_CHUNK`], never derived from the thread count), reduce every chunk sequentially,
//! collect the chunk partials in chunk order, and combine them sequentially. The result is
//! bit-identical for 1 thread and N threads (and for any scheduling), but it intentionally
//! depends on the chunk length: changing the chunk length changes the rounding.

use rayon::prelude::*;
use sm_core::Vector3;

/// Default chunk length for the `par_*` helpers (elements per sequential block).
pub const DEFAULT_CHUNK: usize = 4096;

/// Kahan compensated accumulator.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct KahanSum {
    sum: f64,
    c: f64,
}

impl KahanSum {
    /// An empty accumulator.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add one term.
    #[inline]
    pub fn add(&mut self, x: f64) {
        let y = x - self.c;
        let t = self.sum + y;
        self.c = (t - self.sum) - y;
        self.sum = t;
    }

    /// The accumulated value.
    #[inline]
    pub fn value(&self) -> f64 {
        self.sum
    }
}

impl Extend<f64> for KahanSum {
    fn extend<I: IntoIterator<Item = f64>>(&mut self, it: I) {
        for x in it {
            self.add(x);
        }
    }
}

/// Neumaier (improved Kahan–Babuška) compensated accumulator.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NeumaierSum {
    sum: f64,
    comp: f64,
}

impl NeumaierSum {
    /// An empty accumulator.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add one term.
    #[inline]
    pub fn add(&mut self, x: f64) {
        let t = self.sum + x;
        if self.sum.abs() >= x.abs() {
            self.comp += (self.sum - t) + x;
        } else {
            self.comp += (x - t) + self.sum;
        }
        self.sum = t;
    }

    /// Add another accumulator (its sum, then its compensation). Order matters for the exact
    /// bits, so merge partials in a fixed order.
    #[inline]
    pub fn merge(&mut self, other: &NeumaierSum) {
        self.add(other.sum);
        self.add(other.comp);
    }

    /// The accumulated value.
    #[inline]
    pub fn value(&self) -> f64 {
        self.sum + self.comp
    }
}

impl Extend<f64> for NeumaierSum {
    fn extend<I: IntoIterator<Item = f64>>(&mut self, it: I) {
        for x in it {
            self.add(x);
        }
    }
}

impl FromIterator<f64> for NeumaierSum {
    fn from_iter<I: IntoIterator<Item = f64>>(it: I) -> Self {
        let mut s = Self::new();
        s.extend(it);
        s
    }
}

/// Sequential Kahan sum of `xs`.
pub fn sum_kahan(xs: &[f64]) -> f64 {
    let mut s = KahanSum::new();
    s.extend(xs.iter().copied());
    s.value()
}

/// Sequential Neumaier sum of `xs`.
pub fn sum_neumaier(xs: &[f64]) -> f64 {
    xs.iter().copied().collect::<NeumaierSum>().value()
}

fn chunks(n: usize, chunk: usize) -> usize {
    n.div_ceil(chunk.max(1))
}

/// Generic fixed-chunk reduction: `map_chunk(lo..hi)` runs (possibly in parallel) on each block
/// `[i*chunk, min(n, (i+1)*chunk))`, and `fold` combines the results **sequentially in chunk
/// order** starting from `init`. Bit-identical for any thread count if `map_chunk` is.
pub fn par_reduce<T: Send, R>(
    n: usize,
    chunk: usize,
    map_chunk: impl Fn(core::ops::Range<usize>) -> T + Sync + Send,
    init: R,
    fold: impl FnMut(R, T) -> R,
) -> R {
    let chunk = chunk.max(1);
    let partials: Vec<T> = (0..chunks(n, chunk))
        .into_par_iter()
        .map(|i| map_chunk(i * chunk..((i + 1) * chunk).min(n)))
        .collect();
    partials.into_iter().fold(init, fold)
}

/// Deterministic parallel Neumaier sum of `f(0) + ... + f(n-1)` with blocks of `chunk` terms.
pub fn par_sum_by(n: usize, chunk: usize, f: impl Fn(usize) -> f64 + Sync + Send) -> f64 {
    let total = par_reduce(
        n,
        chunk,
        |r| r.map(&f).collect::<NeumaierSum>(),
        NeumaierSum::new(),
        |mut acc, p| {
            acc.merge(&p);
            acc
        },
    );
    total.value()
}

/// Deterministic parallel Neumaier sum of a slice with [`DEFAULT_CHUNK`]-sized blocks.
pub fn par_sum(xs: &[f64]) -> f64 {
    par_sum_chunked(xs, DEFAULT_CHUNK)
}

/// Deterministic parallel Neumaier sum of a slice with blocks of `chunk` elements.
pub fn par_sum_chunked(xs: &[f64], chunk: usize) -> f64 {
    par_sum_by(xs.len(), chunk, |i| xs[i])
}

/// Deterministic parallel dot product (Neumaier-compensated). Uses the shorter length.
pub fn par_dot(a: &[f64], b: &[f64]) -> f64 {
    par_sum_by(a.len().min(b.len()), DEFAULT_CHUNK, |i| a[i] * b[i])
}

/// Deterministic parallel component-wise sum of `f(0) + ... + f(n-1)` over vectors.
pub fn par_sum_vec3_by(
    n: usize,
    chunk: usize,
    f: impl Fn(usize) -> Vector3 + Sync + Send,
) -> Vector3 {
    let total = par_reduce(
        n,
        chunk,
        |r| {
            let mut s = [NeumaierSum::new(); 3];
            for i in r {
                let v = f(i);
                s[0].add(v.x);
                s[1].add(v.y);
                s[2].add(v.z);
            }
            s
        },
        [NeumaierSum::new(); 3],
        |mut acc, p| {
            for k in 0..3 {
                acc[k].merge(&p[k]);
            }
            acc
        },
    );
    Vector3::new(total[0].value(), total[1].value(), total[2].value())
}
