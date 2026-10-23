//! Ops: serializable, versioned, deterministic state transitions, and the context they run in.

use std::any::Any;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Debug;

use schemars::JsonSchema;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};

use crate::cache::DerivedCache;
use crate::codec::{CodecError, Value, canonicalize, to_params};
use crate::error::OpError;
use crate::layer::LayerType;
use crate::progress::{CancelToken, Control, Progress};
use crate::registry::Registry;
use crate::services::Services;
use crate::state::State;

/// A state change, as a trait object. Usually obtained for free by implementing [`TypedOp`].
///
/// `apply` must be a pure function of `(self, state, services)`: no clock, no thread-dependent
/// order, no unseeded randomness. Ids come from [`ApplyCtx::alloc`].
pub trait Op: Debug + Send + Sync + 'static {
    /// Registry name, namespaced by concept node (e.g. `"model.create_brick"`).
    fn name(&self) -> &'static str;
    /// Payload schema version, bumped when the parameters change shape.
    fn version(&self) -> u32;
    /// Canonical parameters (an object; see [`to_params`]).
    fn encode(&self) -> Result<Value, CodecError>;
    /// Apply to the state through the context. Returning `Err` discards every effect.
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError>;
    /// Upcast, so callers holding a `Box<dyn Op>` can recover the concrete type.
    fn as_any(&self) -> &dyn Any;
}

/// A typed op payload: plain data plus its `apply`. Registering one via
/// `Registry::register::<T>()` provides decoding, versioning and a JSON schema.
/// The payload must serialize to a struct or map.
pub trait TypedOp: Serialize + DeserializeOwned + JsonSchema + Debug + Send + Sync + 'static {
    /// Registry name.
    const NAME: &'static str;
    /// Payload version (default 1).
    const VERSION: u32 = 1;
    /// Apply this op.
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError>;
}

impl<T: TypedOp> Op for T {
    fn name(&self) -> &'static str {
        T::NAME
    }
    fn version(&self) -> u32 {
        T::VERSION
    }
    fn encode(&self) -> Result<Value, CodecError> {
        to_params(self)
    }
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        TypedOp::apply(self, ctx)
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
}

/// The stored form of an op: what a timeline node contains.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct OpRecord {
    /// Registry name.
    pub name: String,
    /// Payload version the params were written with.
    pub version: u32,
    /// Canonical params (an object).
    pub params: Value,
}

impl OpRecord {
    /// Encode an op.
    pub fn of(op: &dyn Op) -> Result<Self, CodecError> {
        Ok(Self { name: op.name().to_owned(), version: op.version(), params: op.encode()? })
    }

    /// Build a record from externally supplied params (e.g. a JSON client), canonicalizing them:
    /// null entries are dropped and `-0.0` folded; params must be an object.
    pub fn new(name: impl Into<String>, version: u32, mut params: Value) -> Result<Self, CodecError> {
        if !params.is_object() {
            return Err(CodecError::new("op parameters must be an object"));
        }
        canonicalize(&mut params)?;
        Ok(Self { name: name.into(), version, params })
    }
}

/// What an applied op changed. Deterministic order throughout.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, JsonSchema)]
pub struct Delta {
    /// Ids allocated, in allocation order.
    pub created: Vec<EntityId>,
    /// Ids retired, in call order.
    pub removed: Vec<EntityId>,
    /// Ids allocated through [`ApplyCtx::alloc_as`], by role.
    pub outputs: BTreeMap<String, Vec<EntityId>>,
    /// Names of layers mutably accessed (including by `on_retired`), sorted.
    pub touched_layers: Vec<&'static str>,
}

/// A scoped handle for progress and cancellation. `Copy` and `Sync`, so scoped worker threads
/// can each hold one and call [`step`](Self::step).
#[derive(Clone, Copy)]
pub struct ApplyCtxScope<'a> {
    progress: &'a dyn Progress,
    cancel: &'a CancelToken,
    lo: f32,
    hi: f32,
}

impl<'a> ApplyCtxScope<'a> {
    pub(crate) fn root(c: Control<'a>) -> Self {
        Self { progress: c.progress, cancel: c.cancel, lo: 0.0, hi: 1.0 }
    }

    /// Check cancellation, then report `done / total` mapped into this scope's range.
    /// Returns [`OpError::Cancelled`] if cancellation was requested; the engine then discards
    /// every partial effect.
    pub fn step(&self, done: u64, total: u64, msg: &str) -> Result<(), OpError> {
        if self.cancel.is_cancelled() {
            return Err(OpError::Cancelled);
        }
        let f = if total == 0 { 1.0 } else { (done as f64 / total as f64).clamp(0.0, 1.0) as f32 };
        self.progress.report(self.lo + (self.hi - self.lo) * f, msg);
        Ok(())
    }

    /// A nested scope covering `[lo, hi]` (fractions of this scope) for a sub-stage.
    pub fn sub(&self, lo: f32, hi: f32) -> ApplyCtxScope<'a> {
        let span = self.hi - self.lo;
        ApplyCtxScope {
            progress: self.progress,
            cancel: self.cancel,
            lo: self.lo + span * lo.clamp(0.0, 1.0),
            hi: self.lo + span * hi.clamp(0.0, 1.0),
        }
    }
}

/// The only handle through which an op reads and mutates state.
pub struct ApplyCtx<'a> {
    state: &'a mut State,
    scope: ApplyCtxScope<'a>,
    services: &'a Services,
    cache: &'a DerivedCache,
    registry: &'a Registry,
    created: Vec<EntityId>,
    removed: Vec<EntityId>,
    outputs: BTreeMap<String, Vec<EntityId>>,
    touched: BTreeSet<&'static str>,
}

impl<'a> ApplyCtx<'a> {
    pub(crate) fn new(
        state: &'a mut State,
        control: Control<'a>,
        services: &'a Services,
        cache: &'a DerivedCache,
        registry: &'a Registry,
    ) -> Self {
        Self {
            state,
            scope: ApplyCtxScope::root(control),
            services,
            cache,
            registry,
            created: Vec::new(),
            removed: Vec::new(),
            outputs: BTreeMap::new(),
            touched: BTreeSet::new(),
        }
    }

    pub(crate) fn finish(&mut self) -> Result<Delta, OpError> {
        if !self.removed.is_empty() {
            let order = self.registry.layer_order()?;
            let touched = self.state.notify_retired(&order, &self.removed);
            self.touched.extend(touched);
        }
        Ok(Delta {
            created: std::mem::take(&mut self.created),
            removed: std::mem::take(&mut self.removed),
            outputs: std::mem::take(&mut self.outputs),
            touched_layers: std::mem::take(&mut self.touched).into_iter().collect(),
        })
    }

    /// Read-only view of the (scratch) state.
    pub fn state(&self) -> &State {
        self.state
    }

    /// The registry this op runs under (for ops that decode nested records).
    pub fn registry(&self) -> &Registry {
        self.registry
    }

    /// Allocate the next id of `kind`. Deterministic: a function of the prior state only.
    pub fn alloc(&mut self, kind: EntityKind) -> Result<EntityId, OpError> {
        let id = self.state.alloc(kind)?;
        self.created.push(id);
        Ok(id)
    }

    /// Allocate and record the id under a named output `role` of the [`Delta`], so later ops in a
    /// `core.batch` (and clients) can refer to it without positional guesswork.
    pub fn alloc_as(&mut self, role: &str, kind: EntityKind) -> Result<EntityId, OpError> {
        let id = self.alloc(kind)?;
        self.outputs.entry(role.to_owned()).or_default().push(id);
        Ok(id)
    }

    /// Allocate one id per key as a contiguous range, assigned in sorted key order (ties by input
    /// position), so the ids do not depend on the order the caller produced the keys in
    /// (docs/research/mesh-ir-addendum-r2.md §7.2). `result[i]` belongs to `keys[i]`.
    pub fn alloc_sorted<K: Ord>(&mut self, kind: EntityKind, keys: &[K]) -> Result<Vec<EntityId>, OpError> {
        let mut order: Vec<usize> = (0..keys.len()).collect();
        order.sort_by(|&a, &b| keys[a].cmp(&keys[b]).then(a.cmp(&b)));
        let first = self.state.alloc_range(kind, keys.len() as u64)?;
        let mut out = vec![first; keys.len()];
        for (rank, &i) in order.iter().enumerate() {
            let id = EntityId::new(kind, first.serial() + rank as u64).ok_or(OpError::IdExhausted)?;
            out[i] = id;
            self.created.push(id);
        }
        Ok(out)
    }

    /// Retire a live entity. Its serial is never reused. Layers are told via
    /// [`LayerType::on_retired`] once the op body returns.
    pub fn retire(&mut self, id: EntityId) -> Result<(), OpError> {
        if self.state.retire(id) {
            self.removed.push(id);
            Ok(())
        } else {
            Err(OpError::UnknownEntity(id))
        }
    }

    /// Fail with [`OpError::UnknownEntity`] unless `id` is live. Use for every id in params.
    pub fn require(&self, id: EntityId) -> Result<(), OpError> {
        if self.state.contains(id) { Ok(()) } else { Err(OpError::UnknownEntity(id)) }
    }

    /// Like [`require`](Self::require) and returns the id, for use in expressions.
    pub fn resolve(&self, id: EntityId) -> Result<EntityId, OpError> {
        self.require(id)?;
        Ok(id)
    }

    /// Require `id` to be live and of kind `kind`.
    pub fn resolve_kind(&self, id: EntityId, kind: EntityKind) -> Result<EntityId, OpError> {
        self.require(id)?;
        if id.kind() == kind { Ok(id) } else { Err(OpError::WrongKind { id, expected: kind }) }
    }

    /// Read a layer.
    pub fn layer<L: LayerType>(&self) -> Result<&L, OpError> {
        self.state.layer::<L>().ok_or_else(|| OpError::MissingLayer(L::NAME.to_owned()))
    }

    /// Mutate a layer (copy-on-write; the committed state is untouched until the op succeeds).
    pub fn layer_mut<L: LayerType>(&mut self) -> Result<&mut L, OpError> {
        self.touched.insert(L::NAME);
        self.state.layer_mut::<L>().ok_or_else(|| OpError::MissingLayer(L::NAME.to_owned()))
    }

    /// The session's derived cache. A hit must never change the op's output.
    pub fn derived(&self) -> &DerivedCache {
        self.cache
    }

    /// Look up an injected service (geometry kernel, mesher, ...).
    pub fn service<S: Any + Send + Sync>(&self) -> Result<&S, OpError> {
        self.services.get::<S>().ok_or(OpError::MissingService(std::any::type_name::<S>()))
    }

    /// Check cancellation and report `done / total`. Callable through `&self` from scoped
    /// worker threads (`ApplyCtx` is `Sync`); for finer control take a [`scope`](Self::scope).
    pub fn step(&self, done: u64, total: u64, msg: &str) -> Result<(), OpError> {
        self.scope.step(done, total, msg)
    }

    /// A copyable progress scope covering the whole op.
    pub fn scope(&self) -> ApplyCtxScope<'a> {
        self.scope
    }

    /// A nested progress scope covering `[lo, hi]` of the op, for a sub-stage.
    pub fn sub(&self, lo: f32, hi: f32) -> ApplyCtxScope<'a> {
        self.scope.sub(lo, hi)
    }

    /// Decode and apply a nested record in this context (used by `core.batch`). Its named
    /// outputs are returned and also merged into this op's outputs as `"<prefix>.<role>"`.
    pub(crate) fn apply_nested(&mut self, rec: &OpRecord, prefix: usize) -> Result<BTreeMap<String, Vec<EntityId>>, OpError> {
        let op = self.registry.decode(rec)?;
        let saved = std::mem::take(&mut self.outputs);
        let r = op.apply(self);
        let sub = std::mem::replace(&mut self.outputs, saved);
        r?;
        for (role, ids) in &sub {
            self.outputs.entry(format!("{prefix}.{role}")).or_default().extend(ids.iter().copied());
        }
        Ok(sub)
    }
}
