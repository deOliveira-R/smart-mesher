//! Executors for one generation wave. The driver gives every job its own pre-allocated result
//! slot and reads the slots in index order, so the output is independent of the executor.

/// Runs `n` independent jobs.
///
/// Implementations may run `job(0..n)` concurrently and in any order, but every index must be
/// run exactly once before `run` returns. The driver never depends on completion order. The
/// trait is object-safe: a rayon-backed executor lives in the mesher crate (API crates do not
/// depend on rayon) and is passed as `&dyn Executor`.
pub trait Executor: Sync {
    /// Run `job(i)` for every `i` in `0..n`.
    fn run(&self, n: usize, job: &(dyn Fn(usize) + Sync));
}

/// Runs jobs one after the other on the calling thread.
#[derive(Clone, Copy, Debug, Default)]
pub struct Sequential;

impl Executor for Sequential {
    fn run(&self, n: usize, job: &(dyn Fn(usize) + Sync)) {
        for i in 0..n {
            job(i);
        }
    }
}

/// Runs jobs on up to `.0` scoped `std` threads in contiguous chunks.
#[derive(Clone, Copy, Debug)]
pub struct ScopedThreads(pub usize);

impl Executor for ScopedThreads {
    fn run(&self, n: usize, job: &(dyn Fn(usize) + Sync)) {
        let t = self.0.max(1).min(n.max(1));
        if t <= 1 {
            for i in 0..n {
                job(i);
            }
            return;
        }
        let chunk = n.div_ceil(t);
        std::thread::scope(|s| {
            let handles: Vec<_> = (0..t)
                .map(|c| {
                    let (lo, hi) = (c * chunk, ((c + 1) * chunk).min(n));
                    s.spawn(move || {
                        for i in lo..hi {
                            job(i);
                        }
                    })
                })
                .collect();
            // A panicking job propagates its panic, like a sequential run would.
            for h in handles {
                if let Err(e) = h.join() {
                    std::panic::resume_unwind(e);
                }
            }
        });
    }
}
