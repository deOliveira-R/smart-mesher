//! Progress reporting and cancellation for long ops (meshing).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

/// Sink for progress, and source of the cancellation flag. Must be callable from worker threads.
pub trait Progress: Send + Sync {
    /// Report completion in `[0, 1]` with a short stage message.
    fn report(&self, fraction: f32, message: &str);
    /// True once the caller wants the op to stop. Ops poll this at safe points.
    fn is_cancelled(&self) -> bool;
}

/// Ignores progress, never cancels.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoProgress;

impl Progress for NoProgress {
    fn report(&self, _: f32, _: &str) {}
    fn is_cancelled(&self) -> bool {
        false
    }
}

/// A shareable cancellation flag. Clone it, hand one clone to the op, call [`cancel`](Self::cancel)
/// from anywhere.
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
}

impl Progress for CancelToken {
    fn report(&self, _: f32, _: &str) {}
    fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::Relaxed)
    }
}

/// Records every report, optionally cancelling after a number of reports. Useful in tests and
/// for UIs that poll.
#[derive(Debug, Default)]
pub struct RecordingProgress {
    events: Mutex<Vec<(f32, String)>>,
    cancel_after: Option<usize>,
}

impl RecordingProgress {
    /// Record only.
    pub fn new() -> Self {
        Self::default()
    }
    /// Report cancellation once `n` events have been recorded.
    pub fn cancel_after(n: usize) -> Self {
        Self { events: Mutex::default(), cancel_after: Some(n) }
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
    fn is_cancelled(&self) -> bool {
        match self.cancel_after {
            Some(n) => self.events.lock().map(|e| e.len() >= n).unwrap_or(false),
            None => false,
        }
    }
}
