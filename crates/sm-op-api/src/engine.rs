//! The transactional apply engine.

use std::sync::Arc;

use crate::cache::DerivedCache;
use crate::error::ApplyError;
use crate::op::{ApplyCtx, Delta, Op, OpRecord};
use crate::progress::Control;
use crate::registry::Registry;
use crate::services::Services;
use crate::state::State;

/// Applies ops to states. Holds only shared, immutable-by-contract context: the registry, the
/// injected services and the session's derived cache.
#[derive(Clone)]
pub struct Engine {
    registry: Arc<Registry>,
    services: Arc<Services>,
    cache: Arc<DerivedCache>,
}

impl Engine {
    /// Build over a finished registry, the injected services and the derived cache.
    pub fn new(registry: Arc<Registry>, services: Services, cache: Arc<DerivedCache>) -> Self {
        Self { registry, services: Arc::new(services), cache }
    }

    /// The registry.
    pub fn registry(&self) -> &Arc<Registry> {
        &self.registry
    }

    /// The derived cache shared with every state.
    pub fn cache(&self) -> &Arc<DerivedCache> {
        &self.cache
    }

    /// Apply a decoded op transactionally: it runs on a copy-on-write clone of `state`, and
    /// `state` is replaced only if the op (and the `on_retired` reactions that follow it)
    /// succeed. On any error, including cancellation, `state` is left exactly as it was.
    pub fn apply(&self, state: &mut State, op: &dyn Op, ctl: &Control<'_>) -> Result<Delta, ApplyError> {
        let mut scratch = state.clone();
        let name = || op.name().to_owned();
        let delta = {
            let mut ctx = ApplyCtx::new(&mut scratch, *ctl, &self.services, &self.cache, &self.registry);
            op.apply(&mut ctx).map_err(|source| ApplyError::Op { name: name(), source })?;
            ctx.finish().map_err(|source| ApplyError::Op { name: name(), source })?
        };
        *state = scratch;
        Ok(delta)
    }

    /// Decode a stored record and apply it. This is the path used for replay.
    pub fn apply_record(&self, state: &mut State, rec: &OpRecord, ctl: &Control<'_>) -> Result<Delta, ApplyError> {
        let op = self.registry.decode(rec)?;
        self.apply(state, op.as_ref(), ctl)
    }
}
