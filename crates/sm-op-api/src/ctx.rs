//! What an op sees while it runs: id allocation, upstream layers, progress and cancellation.

use std::any::Any;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use serde::Serialize;
use sm_core::EntityKind;
use smallvec::SmallVec;

use crate::error::OpError;
use crate::id::{Id, IdAllocator};

/// What an applied op changed, in deterministic (call) order.
///
/// `created` lists ids in allocation order, so a client (or the next op) can refer to them.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize)]
pub struct Delta {
    /// Name of the layer whose op produced this delta.
    pub layer: &'static str,
    /// Ids created by the op, in allocation order.
    pub created: SmallVec<[Id; 4]>,
    /// Ids whose content changed.
    pub modified: Vec<Id>,
    /// Ids removed (never reused).
    pub removed: Vec<Id>,
}

/// A progress report from a long op.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Progress<'a> {
    /// Units finished.
    pub done: u64,
    /// Total units (0 if unknown).
    pub total: u64,
    /// Short human-readable stage.
    pub message: &'a str,
}

/// Caller-side hook for progress and cancellation. Must never influence an op's output.
pub trait Control: Send + Sync {
    /// Receive a progress report.
    fn report(&self, progress: &Progress<'_>);
    /// Whether the caller wants the op abandoned.
    fn is_cancelled(&self) -> bool;
}

/// A [`Control`] that ignores progress and never cancels.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoControl;

impl Control for NoControl {
    fn report(&self, _: &Progress<'_>) {}
    fn is_cancelled(&self) -> bool {
        false
    }
}

/// A shareable cancellation flag usable as a [`Control`].
#[derive(Clone, Debug, Default)]
pub struct CancelToken(Arc<AtomicBool>);

impl CancelToken {
    /// New, un-cancelled token.
    pub fn new() -> Self {
        Self::default()
    }
    /// Request cancellation.
    pub fn cancel(&self) {
        self.0.store(true, Ordering::Relaxed);
    }
}

impl Control for CancelToken {
    fn report(&self, _: &Progress<'_>) {}
    fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::Relaxed)
    }
}

/// Read-only access to the layers upstream of the one being applied.
///
/// Layer dependencies follow the acyclic concept graph: partition reads model; intent reads model
/// and partition; meshes read model, partition and intent.
#[derive(Clone, Copy)]
pub struct Upstream<'a> {
    layers: &'a [&'a dyn Any],
}

impl<'a> Upstream<'a> {
    /// Wrap a slice of upstream stores.
    pub fn new(layers: &'a [&'a dyn Any]) -> Self {
        Self { layers }
    }

    /// The upstream store of type `T`, if present.
    pub fn get<T: Any>(&self) -> Option<&'a T> {
        self.layers.iter().find_map(|l| l.downcast_ref::<T>())
    }

    /// Like [`Upstream::get`] but an error naming `layer` when absent.
    pub fn require<T: Any>(&self, layer: &'static str) -> Result<&'a T, OpError> {
        self.get::<T>().ok_or(OpError::MissingUpstream(layer))
    }
}

/// Per-application context handed to [`crate::Layer::apply`].
pub struct ApplyCtx<'a> {
    ids: &'a mut IdAllocator,
    delta: &'a mut Delta,
    control: &'a dyn Control,
    /// Upstream (read-only) layer stores.
    pub upstream: Upstream<'a>,
}

impl<'a> ApplyCtx<'a> {
    /// Build a context. Used by the state machinery; ops receive it ready-made.
    pub fn new(
        ids: &'a mut IdAllocator,
        delta: &'a mut Delta,
        control: &'a dyn Control,
        upstream: Upstream<'a>,
    ) -> Self {
        Self {
            ids,
            delta,
            control,
            upstream,
        }
    }

    /// Allocate a fresh id of `kind` and record it in the delta's `created` list.
    pub fn alloc(&mut self, kind: EntityKind) -> Result<Id, OpError> {
        let id = self.ids.alloc(kind)?;
        self.delta.created.push(id);
        Ok(id)
    }

    /// Record that `id` changed.
    pub fn modified(&mut self, id: Id) {
        self.delta.modified.push(id);
    }

    /// Record that `id` was removed.
    pub fn removed(&mut self, id: Id) {
        self.delta.removed.push(id);
    }

    /// Report progress, then fail with [`OpError::Cancelled`] if the caller cancelled.
    /// Long ops call this at natural checkpoints.
    pub fn step(&mut self, done: u64, total: u64, message: &str) -> Result<(), OpError> {
        self.control.report(&Progress {
            done,
            total,
            message,
        });
        self.check_cancelled()
    }

    /// Fail with [`OpError::Cancelled`] if the caller cancelled.
    pub fn check_cancelled(&self) -> Result<(), OpError> {
        if self.control.is_cancelled() {
            Err(OpError::Cancelled)
        } else {
            Ok(())
        }
    }
}
