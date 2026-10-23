//! The transactional apply engine.

use std::sync::Arc;

use crate::cache::DerivedCache;
use crate::error::{ApplyError, RegistryError};
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
    /// Layer dependency order, validated once at construction.
    order: Arc<[&'static str]>,
}

impl Engine {
    /// Build over a finished registry, the injected services and the derived cache.
    ///
    /// Validates the layer dependency graph (strict: unknown dependency or cycle is an error) and
    /// caches the `on_retired` order, so a misconfigured registry fails here, not mid-session.
    pub fn new(
        registry: Arc<Registry>,
        services: Services,
        cache: Arc<DerivedCache>,
    ) -> Result<Self, RegistryError> {
        let order: Arc<[&'static str]> = registry.layer_order()?.into();
        Ok(Self {
            registry,
            services: Arc::new(services),
            cache,
            order,
        })
    }

    /// True when `other` shares this engine's registry, services and cache.
    pub fn same_context(&self, other: &Engine) -> bool {
        Arc::ptr_eq(&self.registry, &other.registry)
            && Arc::ptr_eq(&self.services, &other.services)
            && Arc::ptr_eq(&self.cache, &other.cache)
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
    pub fn apply(
        &self,
        state: &mut State,
        op: &dyn Op,
        ctl: &Control<'_>,
    ) -> Result<Delta, ApplyError> {
        self.apply_canonical(state, op, ctl).map(|r| r.0)
    }

    /// Like [`apply`](Self::apply), also returning the canonical stored record when the op
    /// supplied one (`core.batch` canonicalizes its nested records after substitution).
    pub(crate) fn apply_canonical(
        &self,
        state: &mut State,
        op: &dyn Op,
        ctl: &Control<'_>,
    ) -> Result<(Delta, Option<OpRecord>), ApplyError> {
        let mut scratch = state.clone();
        let name = || op.name().to_owned();
        let (delta, canonical) = {
            let mut ctx = ApplyCtx::new(
                &mut scratch,
                *ctl,
                &self.services,
                &self.cache,
                &self.registry,
                &self.order,
            );
            op.apply(&mut ctx).map_err(|source| ApplyError::Op {
                name: name(),
                source,
            })?;
            let delta = ctx.finish().map_err(|source| ApplyError::Op {
                name: name(),
                source,
            })?;
            (delta, ctx.take_canonical())
        };
        *state = scratch;
        Ok((delta, canonical))
    }

    /// Run a registered query against `state` with this engine's services and derived cache.
    /// Read-only; nothing is recorded.
    pub fn run_query(
        &self,
        state: &State,
        name: &str,
        input: &crate::codec::Value,
        ctl: &Control<'_>,
    ) -> Result<crate::codec::Value, RegistryError> {
        let cx = crate::query::QueryCtx::new(&self.services, &self.cache, *ctl);
        self.registry.run_query(state, name, input, &cx)
    }

    /// Decode a stored record and apply it. This is the path used for replay.
    pub fn apply_record(
        &self,
        state: &mut State,
        rec: &OpRecord,
        ctl: &Control<'_>,
    ) -> Result<Delta, ApplyError> {
        let op = self.registry.decode(rec)?;
        self.apply(state, op.as_ref(), ctl)
    }

    /// [`apply_record`](Self::apply_record), also returning the op's canonical stored record.
    pub(crate) fn apply_record_canonical(
        &self,
        state: &mut State,
        rec: &OpRecord,
        ctl: &Control<'_>,
    ) -> Result<(Delta, Option<OpRecord>), ApplyError> {
        let op = self.registry.decode(rec)?;
        self.apply_canonical(state, op.as_ref(), ctl)
    }
}
