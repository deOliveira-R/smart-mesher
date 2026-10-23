//! Executors for one generation wave. Results always come back in index order, so the
//! driver's fixed-order merge makes the output independent of the executor.

/// Runs `n` independent jobs and returns their results in index order.
///
/// Implementations may run jobs concurrently but must return `out[i] = f(i)`. The driver
/// never depends on completion order. (A rayon executor lives in the mesher crate; API crates
/// do not depend on rayon.)
pub trait Executor: Sync {
    /// Evaluate `f(0..n)`.
    fn map<R: Send>(&self, n: usize, f: &(dyn Fn(usize) -> R + Sync)) -> Vec<R>;
}

/// Runs jobs one after the other on the calling thread.
#[derive(Clone, Copy, Debug, Default)]
pub struct Sequential;

impl Executor for Sequential {
    fn map<R: Send>(&self, n: usize, f: &(dyn Fn(usize) -> R + Sync)) -> Vec<R> {
        (0..n).map(f).collect()
    }
}

/// Runs jobs on `threads` scoped `std` threads in contiguous chunks.
#[derive(Clone, Copy, Debug)]
pub struct ScopedThreads(pub usize);

impl Executor for ScopedThreads {
    fn map<R: Send>(&self, n: usize, f: &(dyn Fn(usize) -> R + Sync)) -> Vec<R> {
        let t = self.0.max(1).min(n.max(1));
        if t <= 1 {
            return (0..n).map(f).collect();
        }
        let chunk = n.div_ceil(t);
        let parts: Vec<Vec<R>> = std::thread::scope(|s| {
            let handles: Vec<_> = (0..t)
                .map(|c| {
                    let (lo, hi) = (c * chunk, ((c + 1) * chunk).min(n));
                    s.spawn(move || (lo..hi).map(f).collect::<Vec<R>>())
                })
                .collect();
            // A panicking job propagates its panic, like a sequential run would.
            handles.into_iter().map(|h| h.join().unwrap_or_else(|e| std::panic::resume_unwind(e))).collect()
        });
        parts.into_iter().flatten().collect()
    }
}
