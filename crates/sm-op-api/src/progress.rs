//! Progress reporting and cancellation for long ops (meshing) and long replays.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

/// Sink for progress. Must be callable from worker threads.
pub trait Progress: Send + Sync {
    /// Report completion in `[0, 1]` with a short stage message.
    fn report(&self, fraction: f32, message: &str);
}

/// Ignores progress.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoProgress;

impl Progress for NoProgress {
    fn report(&self, _: f32, _: &str) {}
}

/// A shareable cancellation flag. Clone it, hand one clone to the op, call [`cancel`](Self::cancel)
/// from anywhere (including from inside a [`Progress`] callback).
#[derive(Clone, Debug, Default)]
pub struct CancelToken(Arc<AtomicBool>);

impl CancelToken {
    /// A token that is not cancelled.
    pub fn new() -> Self {
        Self::default()
    }
    /// Request cancellation.
    pub fn cancel(&self) {
        self.0.store(true, Ordering::Relaxed);
    }
    /// True once cancellation was requested.
    pub fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::Relaxed)
    }
}

/// Records every report. Useful in tests and for UIs that poll.
#[derive(Debug, Default)]
pub struct RecordingProgress {
    events: Mutex<Vec<(f32, String)>>,
}

impl RecordingProgress {
    /// An empty recorder.
    pub fn new() -> Self {
        Self::default()
    }
    /// Snapshot of recorded events.
    pub fn events(&self) -> Vec<(f32, String)> {
        self.events.lock().map(|e| e.clone()).unwrap_or_default()
    }
}

impl Progress for RecordingProgress {
    fn report(&self, fraction: f32, message: &str) {
        if let Ok(mut e) = self.events.lock() {
            e.push((fraction, message.to_owned()));
        }
    }
}

/// Progress sink plus cancellation token, passed together to every long operation.
#[derive(Clone, Copy)]
pub struct Control<'a> {
    /// Where progress goes.
    pub progress: &'a dyn Progress,
    /// Polled at safe points.
    pub cancel: &'a CancelToken,
}

impl<'a> Control<'a> {
    /// Bundle a sink and a token.
    pub fn new(progress: &'a dyn Progress, cancel: &'a CancelToken) -> Self {
        Self { progress, cancel }
    }
}

/// Owns a [`NoProgress`] and a fresh [`CancelToken`], for callers that need neither.
#[derive(Clone, Debug, Default)]
pub struct QuietControl {
    progress: NoProgress,
    cancel: CancelToken,
}

impl QuietControl {
    /// A quiet, never-cancelled control.
    pub fn new() -> Self {
        Self::default()
    }
    /// Borrow as a [`Control`].
    pub fn control(&self) -> Control<'_> {
        Control {
            progress: &self.progress,
            cancel: &self.cancel,
        }
    }
}

/// Maps `[0, 1]` reports into `[lo, hi]` of an outer sink.
pub(crate) struct Scaled<'a> {
    pub(crate) inner: &'a dyn Progress,
    pub(crate) lo: f32,
    pub(crate) hi: f32,
}

impl Progress for Scaled<'_> {
    fn report(&self, fraction: f32, message: &str) {
        self.inner.report(
            self.lo + (self.hi - self.lo) * fraction.clamp(0.0, 1.0),
            message,
        );
    }
}
