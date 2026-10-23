//! Ops: serializable, versioned, deterministic state transitions.

use std::any::Any;
use std::collections::BTreeSet;
use std::fmt::Debug;

use schemars::JsonSchema;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};

use crate::error::OpError;
use crate::hash::Hash256;
use crate::layer::{DerivedCache, LayerType};
use crate::progress::Progress;
use crate::state::State;
use crate::value::{CodecError, Value, to_value};

/// A state change, as a trait object. Usually obtained for free by implementing [`TypedOp`].
///
/// `apply` must be a pure function of `(self, state)`: no clock, no thread-dependent order,
/// no unseeded randomness. Ids come from [`ApplyCtx::alloc`].
pub trait Op: Debug + Send + Sync + 'static {
    /// Registry name, namespaced by concept node (e.g. `"model.create_brick"`).
    fn name(&self) -> &'static str;
    /// Payload schema version, bumped when the parameters change shape.
    fn version(&self) -> u32;
    /// Canonical parameters.
    fn encode(&self) -> Result<Value, CodecError>;
    /// Apply to the state through the context. Returning `Err` discards every effect.
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError>;
    /// Upcast, so callers holding a `Box<dyn Op>` can recover the concrete type.
    fn as_any(&self) -> &dyn Any;
}

/// A typed op payload: plain data plus its `apply`. Registering one via
/// `Registry::register::<T>()` provides decoding, versioning and a JSON schema.
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
        to_value(self)
    }
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        TypedOp::apply(self, ctx)
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
}

/// The stored form of an op: what a timeline node contains.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OpRecord {
    /// Registry name.
    pub name: String,
    /// Payload version the params were written with.
    pub version: u32,
    /// Canonical params.
    pub params: Value,
}

impl OpRecord {
    /// Encode an op.
    pub fn of(op: &dyn Op) -> Result<Self, CodecError> {
        Ok(Self { name: op.name().to_owned(), version: op.version(), params: op.encode()? })
    }
}

/// What an applied op changed. Deterministic order throughout.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Delta {
    /// Ids allocated, in allocation order.
    pub created: Vec<EntityId>,
    /// Ids retired, in call order.
    pub removed: Vec<EntityId>,
    /// Names of layers mutably accessed, sorted.
    pub touched_layers: Vec<&'static str>,
}

/// The only handle through which an op reads and mutates state.
pub struct ApplyCtx<'a> {
    state: &'a mut State,
    progress: &'a dyn Progress,
    created: Vec<EntityId>,
    removed: Vec<EntityId>,
    touched: BTreeSet<&'static str>,
}

impl<'a> ApplyCtx<'a> {
    pub(crate) fn new(state: &'a mut State, progress: &'a dyn Progress) -> Self {
        Self {
            state,
            progress,
            created: Vec::new(),
            removed: Vec::new(),
            touched: BTreeSet::new(),
        }
    }

    pub(crate) fn into_delta(self) -> Delta {
        Delta {
            created: self.created,
            removed: self.removed,
            touched_layers: self.touched.into_iter().collect(),
        }
    }

    /// Read-only view of the (scratch) state.
    pub fn state(&self) -> &State {
        self.state
    }

    /// Allocate the next id of `kind`. Deterministic: a function of the prior state only.
    pub fn alloc(&mut self, kind: EntityKind) -> Result<EntityId, OpError> {
        let id = self.state.alloc(kind)?;
        self.created.push(id);
        Ok(id)
    }

    /// Retire a live entity. Its serial is never reused.
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

    /// Read a layer.
    pub fn layer<L: LayerType>(&self) -> Result<&L, OpError> {
        self.state.layer::<L>().ok_or(OpError::MissingLayer(L::NAME))
    }

    /// Mutate a layer (copy-on-write; the committed state is untouched until the op succeeds).
    pub fn layer_mut<L: LayerType>(&mut self) -> Result<&mut L, OpError> {
        self.touched.insert(L::NAME);
        self.state.layer_mut::<L>().ok_or(OpError::MissingLayer(L::NAME))
    }

    /// Look up a derived-cache value (see [`State::cache_key`]).
    pub fn cache_get<T: Any + Send + Sync>(&self, kind: &str, key: Hash256) -> Option<std::sync::Arc<T>> {
        self.state.layer::<DerivedCache>()?.get(kind, key)
    }

    /// Store a derived-cache value. Silently ignored if the derived layer is not installed.
    pub fn cache_put<T: Any + Send + Sync>(&mut self, kind: &str, key: Hash256, value: std::sync::Arc<T>) {
        if let Some(c) = self.state.layer_mut::<DerivedCache>() {
            c.insert(kind, key, value);
        }
    }

    /// Report progress in `[0, 1]`.
    pub fn report(&self, fraction: f32, message: &str) {
        self.progress.report(fraction.clamp(0.0, 1.0), message);
    }

    /// Poll for cancellation: `Err(OpError::Cancelled)` if requested. Call at safe points in
    /// long loops; the engine then discards all partial effects.
    pub fn check_cancelled(&self) -> Result<(), OpError> {
        if self.progress.is_cancelled() { Err(OpError::Cancelled) } else { Ok(()) }
    }
}

/// An entity id as it appears in op params: a plain integer in TOML/JSON (the raw 64-bit
/// [`EntityId`]), so a later op can refer to an id an earlier op created.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct EntityRef(pub u64);

impl EntityRef {
    /// Decode to a checked [`EntityId`].
    pub fn id(self) -> Result<EntityId, OpError> {
        EntityId::from_raw(self.0).ok_or_else(|| OpError::Invalid(format!("bad entity id {}", self.0)))
    }

    /// Decode and require the entity to be live.
    pub fn resolve(self, ctx: &ApplyCtx<'_>) -> Result<EntityId, OpError> {
        let id = self.id()?;
        ctx.require(id)?;
        Ok(id)
    }
}

impl From<EntityId> for EntityRef {
    fn from(id: EntityId) -> Self {
        Self(id.to_raw())
    }
}
