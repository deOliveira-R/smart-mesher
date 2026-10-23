//! The transactional apply engine.

use std::sync::Arc;

use crate::error::ApplyError;
use crate::op::{ApplyCtx, Delta, Op, OpRecord};
use crate::progress::Progress;
use crate::registry::Registry;
use crate::state::State;

/// Applies ops to states. Stateless apart from the shared registry.
#[derive(Clone)]
pub struct Engine {
    registry: Arc<Registry>,
}

impl Engine {
    /// Build over a finished registry.
    pub fn new(registry: Arc<Registry>) -> Self {
        Self { registry }
    }

    /// The registry.
    pub fn registry(&self) -> &Arc<Registry> {
        &self.registry
    }

    /// Apply a decoded op transactionally: it runs on a copy-on-write clone of `state`, and
    /// `state` is replaced only if the op returns `Ok`. On any error (including
    /// cancellation) `state` is left exactly as it was.
    pub fn apply(&self, state: &mut State, op: &dyn Op, progress: &dyn Progress) -> Result<Delta, ApplyError> {
        let mut scratch = state.clone();
        let mut ctx = ApplyCtx::new(&mut scratch, progress);
        op.apply(&mut ctx)
            .map_err(|source| ApplyError::Op { name: op.name().to_owned(), source })?;
        let delta = ctx.into_delta();
        *state = scratch;
        Ok(delta)
    }

    /// Decode a stored record and apply it. This is the path used for replay.
    pub fn apply_record(&self, state: &mut State, rec: &OpRecord, progress: &dyn Progress) -> Result<Delta, ApplyError> {
        let op = self.registry.decode(rec)?;
        self.apply(state, op.as_ref(), progress)
    }
}
