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
use crate::layer::{LayerType, Successors};
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
pub trait TypedOp:
    Serialize + DeserializeOwned + JsonSchema + Debug + Send + Sync + 'static
{
    /// Registry name.
    const NAME: &'static str;
    /// Payload version (default 1).
    const VERSION: u32 = 1;
    /// Implementation version (default 1): bump it when `apply` changes its result for the
    /// same params (a bug fix, a better algorithm) without changing the payload. Not part of
    /// the node hash; recorded in every node's `Check::versions` as `op:<NAME>`, so replay of
    /// an older node reports `TimelineError::Upgraded` instead of a bare `Diverged`.
    const IMPL_VERSION: u32 = 1;
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
        Ok(Self {
            name: op.name().to_owned(),
            version: op.version(),
            params: op.encode()?,
        })
    }

    /// Build a record from externally supplied params (e.g. a JSON client), canonicalizing them:
    /// null entries are dropped and `-0.0` folded; params must be an object.
    pub fn new(
        name: impl Into<String>,
        version: u32,
        mut params: Value,
    ) -> Result<Self, CodecError> {
        if !params.is_object() {
            return Err(CodecError::new("op parameters must be an object"));
        }
        canonicalize(&mut params)?;
        Ok(Self {
            name: name.into(),
            version,
            params,
        })
    }
}

/// What an applied op changed. Deterministic order throughout. Not part of any state or node
/// hash; it is the op's reply to the caller (and the output schema of every op tool).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[non_exhaustive]
pub struct Delta {
    /// Ids allocated or adopted, in allocation order.
    #[serde(default)]
    pub created: Vec<EntityId>,
    /// Ids retired, in call order.
    #[serde(default)]
    pub removed: Vec<EntityId>,
    /// Ids allocated through [`ApplyCtx::alloc_as`] / [`ApplyCtx::adopt_as`], by role.
    #[serde(default)]
    pub outputs: BTreeMap<String, Vec<EntityId>>,
    /// Names of layers mutably accessed (including by `on_succession` / `on_retired`), sorted.
    #[serde(default)]
    pub touched_layers: Vec<String>,
    /// Successor map recorded with [`ApplyCtx::record_successors`]: old id to its successors
    /// (a copy is `A -> [A, A']`).
    #[serde(default)]
    pub successors: BTreeMap<EntityId, Vec<EntityId>>,
    /// Warnings and notes emitted with [`ApplyCtx::warn`] / [`ApplyCtx::info`], in call order.
    /// Verbosity, suppression and stop-on-warning are front-end policy.
    #[serde(default)]
    pub diagnostics: Vec<Diagnostic>,
}

/// Severity of a [`Diagnostic`]. Errors are not diagnostics: they fail the op.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum Severity {
    /// Informational note.
    Info,
    /// Something the user should look at; the op still succeeded.
    Warning,
}

/// A non-fatal message from an op. Never part of a state or node hash.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[non_exhaustive]
pub struct Diagnostic {
    /// Severity.
    pub severity: Severity,
    /// Stable machine-readable code, e.g. `"mesh.inverted_cells"`.
    pub code: String,
    /// Human-readable message.
    pub message: String,
    /// Entities the message is about.
    #[serde(default)]
    pub entities: Vec<EntityId>,
}

impl Diagnostic {
    /// Build a diagnostic.
    pub fn new(
        severity: Severity,
        code: impl Into<String>,
        message: impl Into<String>,
        entities: Vec<EntityId>,
    ) -> Self {
        Self {
            severity,
            code: code.into(),
            message: message.into(),
            entities,
        }
    }
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
        Self {
            progress: c.progress,
            cancel: c.cancel,
            lo: 0.0,
            hi: 1.0,
        }
    }

    /// Check cancellation, then report `done / total` mapped into this scope's range.
    /// Returns [`OpError::Cancelled`] if cancellation was requested; the engine then discards
    /// every partial effect.
    pub fn step(&self, done: u64, total: u64, msg: &str) -> Result<(), OpError> {
        if self.cancel.is_cancelled() {
            return Err(OpError::Cancelled);
        }
        let f = if total == 0 {
            1.0
        } else {
            (done as f64 / total as f64).clamp(0.0, 1.0) as f32
        };
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
    order: &'a [&'static str],
    notified: usize,
    created: Vec<EntityId>,
    removed: Vec<EntityId>,
    outputs: BTreeMap<String, Vec<EntityId>>,
    touched: BTreeSet<&'static str>,
    successors: Successors,
    pending_succ: Successors,
    diagnostics: Vec<Diagnostic>,
    canonical: Option<OpRecord>,
}

impl<'a> ApplyCtx<'a> {
    pub(crate) fn new(
        state: &'a mut State,
        control: Control<'a>,
        services: &'a Services,
        cache: &'a DerivedCache,
        registry: &'a Registry,
        order: &'a [&'static str],
    ) -> Self {
        Self {
            state,
            scope: ApplyCtxScope::root(control),
            services,
            cache,
            registry,
            order,
            notified: 0,
            created: Vec::new(),
            removed: Vec::new(),
            outputs: BTreeMap::new(),
            touched: BTreeSet::new(),
            successors: BTreeMap::new(),
            pending_succ: BTreeMap::new(),
            diagnostics: Vec::new(),
            canonical: None,
        }
    }

    /// Let the op supply the canonical stored form of itself (used by `core.batch`).
    pub(crate) fn set_canonical_record(&mut self, rec: OpRecord) {
        self.canonical = Some(rec);
    }

    pub(crate) fn take_canonical(&mut self) -> Option<OpRecord> {
        self.canonical.take()
    }

    pub(crate) fn finish(&mut self) -> Result<Delta, OpError> {
        self.flush_reactions();
        Ok(Delta {
            created: std::mem::take(&mut self.created),
            removed: std::mem::take(&mut self.removed),
            outputs: std::mem::take(&mut self.outputs),
            touched_layers: std::mem::take(&mut self.touched)
                .into_iter()
                .map(str::to_owned)
                .collect(),
            successors: std::mem::take(&mut self.successors),
            diagnostics: std::mem::take(&mut self.diagnostics),
        })
    }

    /// Run the cross-layer reactions for everything recorded since the last flush, in
    /// dependency order: first `on_succession` (so a layer can carry references over to split
    /// or copied entities), then `on_retired`.
    fn flush_reactions(&mut self) {
        if !self.pending_succ.is_empty() {
            let succ = std::mem::take(&mut self.pending_succ);
            let touched = self.state.notify_succession(self.order, &succ);
            self.touched.extend(touched);
        }
        if self.notified < self.removed.len() {
            let touched = self
                .state
                .notify_retired(self.order, &self.removed[self.notified..]);
            self.touched.extend(touched);
            self.notified = self.removed.len();
        }
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
    /// `core.batch` (and clients) can refer to it without positional guesswork. Meant for a few
    /// per-op outputs: it allocates a `String` per call, so do not use it in per-element loops;
    /// use [`alloc_sorted`](Self::alloc_sorted) for bulk ids.
    pub fn alloc_as(&mut self, role: &str, kind: EntityKind) -> Result<EntityId, OpError> {
        let id = self.alloc(kind)?;
        self.outputs.entry(role.to_owned()).or_default().push(id);
        Ok(id)
    }

    /// Allocate one id per key as a contiguous range, assigned in sorted key order (ties by input
    /// position), so the ids do not depend on the order the caller produced the keys in
    /// (docs/research/mesh-ir-addendum-r2.md §7.2). `result[i]` belongs to `keys[i]`.
    pub fn alloc_sorted<K: Ord>(
        &mut self,
        kind: EntityKind,
        keys: &[K],
    ) -> Result<Vec<EntityId>, OpError> {
        let mut order: Vec<usize> = (0..keys.len()).collect();
        order.sort_by(|&a, &b| keys[a].cmp(&keys[b]).then(a.cmp(&b)));
        let first = self.state.alloc_range(kind, keys.len() as u64)?;
        let mut out = vec![first; keys.len()];
        for (rank, &i) in order.iter().enumerate() {
            let id =
                EntityId::new(kind, first.serial() + rank as u64).ok_or(OpError::IdExhausted)?;
            out[i] = id;
            self.created.push(id);
        }
        Ok(out)
    }

    /// Adopt ids minted by a downstream store (topology complex, overlay, model, mesh) into the
    /// entity table. Each id must equal [`State::peek_next`] for its kind at the moment it is
    /// adopted, in order, otherwise [`OpError::IdMismatch`]. Adopted ids are listed in
    /// [`Delta::created`]. The store stays the single minting layer for its kinds; adoption
    /// keeps the shared table (and so [`State::contains`] and the state hash) in step with it.
    pub fn adopt(&mut self, ids: &[EntityId]) -> Result<(), OpError> {
        for &id in ids {
            let expected = self
                .state
                .peek_next(id.kind())
                .ok_or(OpError::IdExhausted)?;
            if id != expected {
                return Err(OpError::IdMismatch { expected, got: id });
            }
            self.state.alloc(id.kind())?;
            self.created.push(id);
        }
        Ok(())
    }

    /// Adopt `n` consecutive ids starting at `first` (bulk mesh ranges). `first` must equal
    /// [`State::peek_next`] for its kind. Every id is listed in [`Delta::created`].
    pub fn adopt_range(&mut self, first: EntityId, n: u64) -> Result<(), OpError> {
        let expected = self
            .state
            .peek_next(first.kind())
            .ok_or(OpError::IdExhausted)?;
        if first != expected {
            return Err(OpError::IdMismatch {
                expected,
                got: first,
            });
        }
        let start = self.state.alloc_range(first.kind(), n)?;
        self.created.reserve(usize::try_from(n).unwrap_or(0));
        for k in 0..n {
            let id = EntityId::new(first.kind(), start.serial() + k).ok_or(OpError::IdExhausted)?;
            self.created.push(id);
        }
        Ok(())
    }

    /// [`adopt`](Self::adopt) and record the ids under the named output `role` of the [`Delta`].
    pub fn adopt_as(&mut self, role: &str, ids: &[EntityId]) -> Result<(), OpError> {
        self.adopt(ids)?;
        self.outputs
            .entry(role.to_owned())
            .or_default()
            .extend_from_slice(ids);
        Ok(())
    }

    /// Record that `old` was succeeded by `new` (split children, merge survivor; a copy is
    /// `old = A`, `new = [A, A']`). Every id in `new` must be live. Downstream layers receive
    /// the map through [`LayerType::on_succession`] after the op body (inside `core.batch`:
    /// after each sub-op), before `on_retired`, so they can carry groups and names over to the
    /// successors. Repeated calls for the same `old` append. The whole map is returned as
    /// [`Delta::successors`].
    pub fn record_successors(&mut self, old: EntityId, new: &[EntityId]) -> Result<(), OpError> {
        for &n in new {
            self.require(n)?;
        }
        self.successors
            .entry(old)
            .or_default()
            .extend_from_slice(new);
        self.pending_succ
            .entry(old)
            .or_default()
            .extend_from_slice(new);
        Ok(())
    }

    /// Emit a warning (returned in [`Delta::diagnostics`]; not part of any hash).
    pub fn warn(&mut self, code: &'static str, msg: impl Into<String>, entities: &[EntityId]) {
        self.diagnostics.push(Diagnostic::new(
            Severity::Warning,
            code,
            msg,
            entities.to_vec(),
        ));
    }

    /// Emit an informational note (returned in [`Delta::diagnostics`]; not part of any hash).
    pub fn info(&mut self, code: &'static str, msg: impl Into<String>, entities: &[EntityId]) {
        self.diagnostics
            .push(Diagnostic::new(Severity::Info, code, msg, entities.to_vec()));
    }

    /// Retire a live entity. Its serial is never reused. Layers are told via
    /// [`LayerType::on_retired`] once the op body returns (inside `core.batch`: after each
    /// sub-op). Ops that drive a minting store mirror the ids it retired (e.g. sm-topo-api's
    /// `OpSummary::retired`) here.
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
        if self.state.contains(id) {
            Ok(())
        } else {
            Err(OpError::UnknownEntity(id))
        }
    }

    /// Like [`require`](Self::require) and returns the id, for use in expressions.
    pub fn resolve(&self, id: EntityId) -> Result<EntityId, OpError> {
        self.require(id)?;
        Ok(id)
    }

    /// Require `id` to be live and of kind `kind`.
    pub fn resolve_kind(&self, id: EntityId, kind: EntityKind) -> Result<EntityId, OpError> {
        self.require(id)?;
        if id.kind() == kind {
            Ok(id)
        } else {
            Err(OpError::WrongKind { id, expected: kind })
        }
    }

    /// Read a layer.
    pub fn layer<L: LayerType>(&self) -> Result<&L, OpError> {
        self.state
            .layer::<L>()
            .ok_or_else(|| OpError::MissingLayer(L::NAME.to_owned()))
    }

    /// Mutate a layer (copy-on-write; the committed state is untouched until the op succeeds).
    pub fn layer_mut<L: LayerType>(&mut self) -> Result<&mut L, OpError> {
        let l = self
            .state
            .layer_mut::<L>()
            .ok_or_else(|| OpError::MissingLayer(L::NAME.to_owned()))?;
        self.touched.insert(L::NAME);
        Ok(l)
    }

    /// The session's derived cache. A hit must never change the op's output.
    pub fn derived(&self) -> &DerivedCache {
        self.cache
    }

    /// Look up an injected service (geometry kernel, mesher, ...).
    pub fn service<S: Any + Send + Sync>(&self) -> Result<&S, OpError> {
        self.services
            .get::<S>()
            .ok_or(OpError::MissingService(std::any::type_name::<S>()))
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

    /// Decode and apply sub-op `index` of `count` in this context (used by `core.batch`), with
    /// progress confined to its slice of this op's range. Returns its named outputs (also merged
    /// into this op's outputs as `"<index>.<role>"`) and its canonical record (current version,
    /// re-encoded from the decoded op).
    pub(crate) fn apply_nested(
        &mut self,
        rec: &OpRecord,
        index: usize,
        count: usize,
    ) -> Result<(BTreeMap<String, Vec<EntityId>>, OpRecord), OpError> {
        let op = self.registry.decode(rec)?;
        let canonical = OpRecord::of(op.as_ref())?;
        let saved = std::mem::take(&mut self.outputs);
        let outer = self.scope;
        let n = count.max(1) as f32;
        self.scope = outer.sub(index as f32 / n, (index + 1) as f32 / n);
        let r = op.apply(self);
        self.scope = outer;
        let sub = std::mem::replace(&mut self.outputs, saved);
        r?;
        // Layers react after each nested op, so later sub-ops never see references to ids an
        // earlier sub-op retired.
        self.flush_reactions();
        for (role, ids) in &sub {
            self.outputs
                .entry(format!("{index}.{role}"))
                .or_default()
                .extend(ids.iter().copied());
        }
        Ok((sub, canonical))
    }
}
