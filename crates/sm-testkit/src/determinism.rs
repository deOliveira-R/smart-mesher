//! Determinism harness (CLAUDE.md "Determinism (binding)").
//!
//! [`check`] runs a closure `f(threads)` repeatedly and demands bit-identical results:
//!
//! 1. `f(1)` is the baseline;
//! 2. `runs` times: `f(1)` again, and `f(threads)` (the closure decides how to use `threads`
//!    workers, for example through [`par_map`]);
//! 3. `f(1)` called concurrently from `threads` std threads (catches hidden global state, such as
//!    a counter or a shared cache that leaks into results).
//!
//! The crate has no `rayon` dependency, so "N threads" means `std::thread::scope`. Implementation
//! crates that use rayon call [`check`] with a closure that builds a scoped pool of `threads`
//! workers (`RAYON_NUM_THREADS=1` vs default is the same experiment).

use core::fmt::Debug;
use core::hash::Hasher;

/// A determinism violation.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum DeterminismError {
    /// Two runs that must agree differ.
    #[error("{what}: expected {expected}, got {actual}")]
    Mismatch {
        /// Which run differed.
        what: String,
        /// Debug form of the baseline.
        expected: String,
        /// Debug form of the differing result.
        actual: String,
    },
    /// A worker thread panicked.
    #[error("{0}: a thread panicked")]
    Panicked(String),
    /// `runs` or `threads` was zero.
    #[error("runs and threads must be at least 1")]
    BadArguments,
}

/// Run `f` as described in the module docs and return the baseline value.
///
/// `runs` repetitions of `f(1)` and `f(threads)`, plus one round of `threads` concurrent
/// `f(1)` calls. Returns the first [`DeterminismError`] found.
pub fn check<T: PartialEq + Debug + Send>(runs: usize, threads: usize, f: impl Fn(usize) -> T + Sync) -> Result<T, DeterminismError> {
    if runs == 0 || threads == 0 {
        return Err(DeterminismError::BadArguments);
    }
    let baseline = f(1);
    let compare = |what: String, v: T| -> Result<(), DeterminismError> {
        if v == baseline {
            Ok(())
        } else {
            Err(DeterminismError::Mismatch { what, expected: format!("{baseline:?}"), actual: format!("{v:?}") })
        }
    };
    for r in 0..runs {
        compare(format!("repeat {r} with 1 thread"), f(1))?;
        compare(format!("repeat {r} with {threads} threads"), f(threads))?;
    }
    let results: Vec<Option<T>> = std::thread::scope(|s| {
        let handles: Vec<_> = (0..threads).map(|_| s.spawn(|| f(1))).collect();
        handles.into_iter().map(|h| h.join().ok()).collect()
    });
    for (i, r) in results.into_iter().enumerate() {
        match r {
            Some(v) => compare(format!("concurrent call {i} of {threads}"), v)?,
            None => return Err(DeterminismError::Panicked(format!("concurrent call {i}"))),
        }
    }
    Ok(baseline)
}

/// Apply `f` to `0..n` on `threads` std threads and return the results **in index order**
/// (contiguous chunks, joined in order), so any reduction over the result is order-independent
/// of thread scheduling. `threads <= 1` runs sequentially. Panics in `f` propagate.
pub fn par_map<T: Send>(threads: usize, n: usize, f: impl Fn(usize) -> T + Sync) -> Vec<T> {
    if threads <= 1 || n <= 1 {
        return (0..n).map(f).collect();
    }
    let chunk = n.div_ceil(threads);
    let f = &f;
    std::thread::scope(|s| {
        let handles: Vec<_> = (0..n)
            .step_by(chunk)
            .map(|lo| {
                let hi = (lo + chunk).min(n);
                s.spawn(move || (lo..hi).map(f).collect::<Vec<T>>())
            })
            .collect();
        handles.into_iter().flat_map(|h| h.join().unwrap_or_else(|e| std::panic::resume_unwind(e))).collect()
    })
}

/// FNV-1a 64-bit hasher: stable across platforms and releases (unlike `DefaultHasher`).
#[derive(Clone, Copy, Debug)]
pub struct Fnv(u64);

impl Default for Fnv {
    fn default() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
}

impl Hasher for Fnv {
    fn finish(&self) -> u64 {
        self.0
    }
    fn write(&mut self, bytes: &[u8]) {
        for b in bytes {
            self.0 ^= u64::from(*b);
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
}

/// FNV-1a hash of the bit patterns of `xs` (`-0.0` and `0.0` differ; NaN payloads count).
pub fn hash_f64_bits(xs: &[f64]) -> u64 {
    let mut h = Fnv::default();
    for x in xs {
        h.write(&x.to_bits().to_le_bytes());
    }
    h.finish()
}
