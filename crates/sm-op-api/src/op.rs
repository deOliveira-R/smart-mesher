//! Ops, the apply context, and the [`Registry`] where concepts register their ops and layers.

use crate::error::{ApplyError, OpError, RegistryError};
use crate::hash::Hash;
use crate::layer::{DynLayer, Layer, LayerId};
use crate::state::{CounterChange, Delta, LayerPatch, State};
use crate::value::{Table, Value, ValueError, from_value, to_value};
use schemars::JsonSchema;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use std::collections::BTreeMap;
use std::panic::{AssertUnwindSafe, catch_unwind, resume_unwind};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

/// A state-changing operation: plain serializable data plus a deterministic `apply`.
///
/// `apply` must be a pure function of `(self, state)`: no clocks, no thread-dependent
/// order, no unseeded randomness. It changes state only through [`ApplyCtx`], which records
/// a reversible [`Delta`] and rolls everything back if `apply` returns `Err` or panics.
/// The doc comments on the type and its fields become the JSON-schema descriptions that
/// MCP tools, language help and Python stubs are generated from.
pub trait Op: Serialize + DeserializeOwned + JsonSchema + 'static {
    /// Stable, unique op name (e.g. `"model.create_brick"`). Part of the on-disk format.
    const KIND: &'static str;
    /// Schema version of the parameters. Bump on any incompatible change and register a
    /// migration with [`Registry::register_migration`].
    const VERSION: u32;
    /// Execute against the state held by `ctx`.
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError>;
}

/// The serialized form of an op as stored in a timeline node.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OpRecord {
    /// [`Op::KIND`].
    pub kind: String,
    /// [`Op::VERSION`] at the time of writing.
    pub version: u32,
    /// The op's fields.
    pub params: Value,
}

impl OpRecord {
    /// Record a typed op.
    pub fn new<O: Op>(op: &O) -> Result<Self, ValueError> {
        Ok(OpRecord {
            kind: O::KIND.to_owned(),
            version: O::VERSION,
            params: to_value(op)?,
        })
    }
}

/// Receiver of progress notifications from long ops (meshing).
pub trait ProgressSink: Send + Sync {
    /// `fraction` is in `0.0..=1.0`; `message` is a short human-readable phase name.
    fn report(&self, fraction: f64, message: &str);
}

/// Cooperative cancellation flag; clones share the flag.
#[derive(Clone, Debug, Default)]
pub struct CancelToken(Arc<AtomicBool>);

impl CancelToken {
    /// New, not cancelled.
    pub fn new() -> Self {
        Self::default()
    }
    /// Request cancellation.
    pub fn cancel(&self) {
        self.0.store(true, Ordering::SeqCst);
    }
    /// Whether cancellation was requested.
    pub fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::SeqCst)
    }
}

/// Per-call options for applying an op.
#[derive(Default)]
pub struct ApplyOptions<'a> {
    /// Progress receiver.
    pub progress: Option<&'a dyn ProgressSink>,
    /// Cancellation token.
    pub cancel: CancelToken,
}

/// What an op sees and may change. Every mutation goes through [`ApplyCtx::patch`] or
/// [`ApplyCtx::alloc`], which is how the delta gets recorded.
pub struct ApplyCtx<'a> {
    state: &'a mut State,
    journal: Vec<LayerPatch>,
    counters: BTreeMap<u8, CounterChange>,
    created: Vec<EntityId>,
    opts: &'a ApplyOptions<'a>,
}

impl<'a> ApplyCtx<'a> {
    fn new(state: &'a mut State, opts: &'a ApplyOptions<'a>) -> Self {
        ApplyCtx {
            state,
            journal: Vec::new(),
            counters: BTreeMap::new(),
            created: Vec::new(),
            opts,
        }
    }

    /// Read-only view of the current state (sees this op's earlier writes).
    pub fn state(&self) -> &State {
        self.state
    }

    /// Typed read access to a layer.
    pub fn layer<L: Layer>(&self, id: LayerId) -> Result<&L, OpError> {
        if !self.state.has_layer(id.as_str()) {
            return Err(OpError::NoSuchLayer(id.as_str().to_owned()));
        }
        self.state
            .layer::<L>(id)
            .ok_or_else(|| OpError::LayerType(id.as_str().to_owned()))
    }

    /// Allocate the next id of `kind`. Deterministic: depends only on the history so far
    /// and on the order of calls within this op, so ops must allocate in a data-defined
    /// canonical order (docs/research addendum D5.1), never in hash-map or thread order.
    pub fn alloc(&mut self, kind: EntityKind) -> Result<EntityId, OpError> {
        let (id, change) = self.state.alloc_id(kind)?;
        self.counters
            .entry(change.kind)
            .and_modify(|c| c.after = change.after)
            .or_insert(change);
        self.created.push(id);
        Ok(id)
    }

    /// Apply a patch to a layer, recording forward and inverse in the delta.
    pub fn patch<L: Layer>(&mut self, id: LayerId, patch: &L::Patch) -> Result<(), OpError> {
        self.layer::<L>(id)?;
        let forward = to_value(patch)?;
        let inverse = self.state.apply_patch(id.as_str(), &forward)?;
        self.journal.push(LayerPatch {
            layer: id.as_str().to_owned(),
            forward,
            inverse,
        });
        Ok(())
    }

    /// Progress and cancellation point for long ops: returns `Err(Cancelled)` if the token
    /// fired, otherwise forwards the progress report. Call it regularly (e.g. per block).
    pub fn checkpoint(&self, fraction: f64, message: &str) -> Result<(), OpError> {
        if self.opts.cancel.is_cancelled() {
            return Err(OpError::Cancelled);
        }
        if let Some(p) = self.opts.progress {
            p.report(fraction.clamp(0.0, 1.0), message);
        }
        Ok(())
    }

    fn rollback(self) -> Result<(), ApplyError> {
        let ApplyCtx {
            state,
            journal,
            counters,
            ..
        } = self;
        for p in journal.iter().rev() {
            state
                .apply_patch(&p.layer, &p.inverse)
                .map_err(|e| ApplyError::RollbackFailed(e.to_string()))?;
        }
        for c in counters.values() {
            state.set_counter(c.kind, c.before);
        }
        Ok(())
    }
}

/// Result of successfully applying one op.
#[derive(Clone, Debug)]
pub struct AppliedOp {
    /// Reversible effect, to be persisted next to the op.
    pub delta: Delta,
    /// Ids allocated by the op, in allocation order (these are the stable references later
    /// ops can use).
    pub created: Vec<EntityId>,
}

/// Parameter migration step: rewrites params of version `n` into version `n + 1`.
pub type Migration = fn(Value) -> Result<Value, ValueError>;

type RunFn = fn(&Value, &mut ApplyCtx<'_>) -> Result<(), ApplyError>;

struct OpEntry {
    version: u32,
    run: RunFn,
    schema: fn() -> Result<Value, ValueError>,
    migrations: BTreeMap<u32, Migration>,
}

fn run_op<O: Op>(params: &Value, ctx: &mut ApplyCtx<'_>) -> Result<(), ApplyError> {
    let op: O = from_value(params).map_err(|source| ApplyError::BadParams {
        kind: O::KIND.to_owned(),
        source,
    })?;
    op.apply(ctx).map_err(ApplyError::Op)
}

fn schema_of<O: Op>() -> Result<Value, ValueError> {
    to_value(&schemars::schema_for!(O))
}

/// A unit of functionality that contributes ops and layers (a concept node of the graph).
/// The api crate never names concrete concepts; each implementing crate provides one.
pub trait Concept {
    /// Name for diagnostics.
    fn name(&self) -> &'static str;
    /// Register this concept's layers and ops.
    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError>;
}

/// Registry of op kinds and layer factories. Cheap to share (`Arc`) between workspaces.
#[derive(Default)]
pub struct Registry {
    ops: BTreeMap<&'static str, OpEntry>,
    layers: BTreeMap<&'static str, fn() -> Box<dyn DynLayer>>,
}

impl Registry {
    /// Empty registry.
    pub fn new() -> Self {
        Self::default()
    }

    /// Install a concept.
    pub fn install(&mut self, c: &dyn Concept) -> Result<&mut Self, RegistryError> {
        c.register(self)?;
        Ok(self)
    }

    /// Register a layer store, created empty with `Default` in every new [`State`].
    pub fn register_layer<L: Layer + Default>(&mut self, id: LayerId) -> Result<(), RegistryError> {
        if self.layers.contains_key(id.as_str()) {
            return Err(RegistryError::DuplicateLayer(id.as_str().to_owned()));
        }
        self.layers.insert(id.as_str(), || Box::new(L::default()));
        Ok(())
    }

    /// Register an op type.
    pub fn register_op<O: Op>(&mut self) -> Result<(), RegistryError> {
        if self.ops.contains_key(O::KIND) {
            return Err(RegistryError::DuplicateOp(O::KIND.to_owned()));
        }
        self.ops.insert(
            O::KIND,
            OpEntry {
                version: O::VERSION,
                run: run_op::<O>,
                schema: schema_of::<O>,
                migrations: BTreeMap::new(),
            },
        );
        Ok(())
    }

    /// Register a migration of op `kind` params from `from_version` to `from_version + 1`.
    pub fn register_migration(
        &mut self,
        kind: &str,
        from_version: u32,
        f: Migration,
    ) -> Result<(), RegistryError> {
        let e = self
            .ops
            .get_mut(kind)
            .ok_or_else(|| RegistryError::BadMigration(kind.to_owned()))?;
        if from_version >= e.version {
            return Err(RegistryError::BadMigration(kind.to_owned()));
        }
        e.migrations.insert(from_version, f);
        Ok(())
    }

    /// A fresh empty state with one empty store per registered layer.
    pub fn new_state(&self) -> State {
        State::new(
            self.layers
                .iter()
                .map(|(n, f)| ((*n).to_owned(), f()))
                .collect(),
        )
    }

    /// Registered op kinds, sorted.
    pub fn op_kinds(&self) -> impl Iterator<Item = &'static str> + '_ {
        self.ops.keys().copied()
    }

    /// JSON schema of an op's parameters, as a [`Value`] (use [`Value::to_json`] for text).
    pub fn op_schema(&self, kind: &str) -> Option<Result<Value, ValueError>> {
        self.ops.get(kind).map(|e| (e.schema)())
    }

    /// Catalog of every op: `{ kind: { version, schema } }`. This is the single source for
    /// MCP tool definitions, command-language help and Python stub generation.
    pub fn catalog(&self) -> Result<Value, ValueError> {
        let mut all = Table::new();
        for (kind, e) in &self.ops {
            let mut t = Table::new();
            t.insert("version".into(), Value::Int(i64::from(e.version)));
            t.insert("schema".into(), (e.schema)()?);
            all.insert((*kind).to_owned(), Value::Table(t));
        }
        Ok(Value::Table(all))
    }

    /// Bring params of an old record up to the current version.
    fn migrate(&self, e: &OpEntry, rec: &OpRecord) -> Result<Value, ApplyError> {
        if rec.version > e.version {
            return Err(ApplyError::UnsupportedVersion {
                kind: rec.kind.clone(),
                found: rec.version,
                supported: e.version,
            });
        }
        let mut params = rec.params.clone();
        for v in rec.version..e.version {
            let step = e
                .migrations
                .get(&v)
                .ok_or_else(|| ApplyError::NoMigration {
                    kind: rec.kind.clone(),
                    from: v,
                })?;
            params = step(params).map_err(|source| ApplyError::BadParams {
                kind: rec.kind.clone(),
                source,
            })?;
        }
        Ok(params)
    }

    /// Apply a recorded op to `state`, transactionally.
    ///
    /// On success the state has advanced and the returned [`AppliedOp`] holds the reversible
    /// delta. On any error (including cancellation and panics in the op) the state is
    /// restored exactly (same [`State::hash`]) before returning; only
    /// [`ApplyError::RollbackFailed`] means the state is unusable.
    pub fn apply(
        &self,
        state: &mut State,
        rec: &OpRecord,
        opts: &ApplyOptions<'_>,
    ) -> Result<AppliedOp, ApplyError> {
        let e = self
            .ops
            .get(rec.kind.as_str())
            .ok_or_else(|| ApplyError::UnknownOp(rec.kind.clone()))?;
        let params = self.migrate(e, rec)?;
        let state_before = state.hash();
        let mut ctx = ApplyCtx::new(state, opts);
        let outcome = catch_unwind(AssertUnwindSafe(|| (e.run)(&params, &mut ctx)));
        match outcome {
            Ok(Ok(())) => {
                let ApplyCtx {
                    journal,
                    counters,
                    created,
                    ..
                } = ctx;
                let delta = Delta {
                    state_before,
                    state_after: state.hash(),
                    counters: counters.into_values().collect(),
                    patches: journal,
                };
                Ok(AppliedOp { delta, created })
            }
            Ok(Err(err)) => {
                ctx.rollback()?;
                Err(err)
            }
            Err(panic) => {
                // Restore state, then let the panic continue.
                let _ = ctx.rollback();
                resume_unwind(panic)
            }
        }
    }

    /// Hash identifying this registry's op set; handy for diagnostics and cache keys.
    pub fn fingerprint(&self) -> Hash {
        let mut h = crate::hash::StateHasher::new();
        for (k, e) in &self.ops {
            h.str(k);
            h.u64(u64::from(e.version));
        }
        for k in self.layers.keys() {
            h.str(k);
        }
        h.finish()
    }
}
