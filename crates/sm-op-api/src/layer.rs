//! Layers, the closed per-layer op enums, and their composition.
//!
//! Each layer crate owns one *closed* op enum and one store. This crate fixes the set of layer
//! *slots* (model, partition, intent, meshes, from `docs/design/ir.md` section 5) and composes the
//! four op enums into one top-level enum, [`StackOp`], without knowing any concrete op.

use core::marker::PhantomData;
use std::any::Any;
use std::fmt::Debug;

use schemars::JsonSchema;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::ctx::ApplyCtx;
use crate::error::OpError;

/// A layer store plus its closed op enum. Implemented by the store type.
///
/// `apply` is pure: it reads `self`, the op, and (through `ctx`) upstream layers and the id
/// allocator, and returns the new store. It must not touch clocks, threads or unseeded RNG.
/// Stores should be cheap to `Clone` (wrap big tables in `Arc` and copy on write), because the
/// engine keeps the old store alive until the op succeeds.
pub trait Layer: Any + Clone + Default + Serialize + Send + Sync + 'static {
    /// Layer name, used in deltas, hashes and derived-cache keys.
    const NAME: &'static str;
    /// Version of this layer's op schema, recorded in every timeline node.
    const OP_VERSION: u32;
    /// The layer's closed op enum (serde-tagged, schema-generating).
    type Op: Clone
        + Debug
        + PartialEq
        + Serialize
        + DeserializeOwned
        + JsonSchema
        + Send
        + Sync
        + 'static;

    /// Apply `op`, returning the new store or an error (in which case nothing has changed).
    fn apply(&self, op: &Self::Op, ctx: &mut ApplyCtx<'_>) -> Result<Self, OpError>;
}

/// The top-level op: one variant per layer slot, each wrapping that layer's closed op enum.
///
/// Serialized as `{ layer = "model", args = { kind = "CreateBrick", ... } }`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "layer", content = "args", rename_all = "snake_case")]
pub enum StackOp<M, P, I, X> {
    /// An op of the model layer.
    Model(M),
    /// An op of the partition layer.
    Partition(P),
    /// An op of the intent layer.
    Intent(I),
    /// An op of the meshes layer.
    Meshes(X),
}

/// Binds four layers into a stack. Use [`Stack4`] rather than implementing this by hand.
pub trait Stack: 'static {
    /// Model layer (shape, topology, coloring, boundary kinds, symmetry, instancing).
    type Model: Layer;
    /// Partition layer (virtual decomposition).
    type Partition: Layer;
    /// Intent layer (sizes, schemes).
    type Intent: Layer;
    /// Meshes layer (mesh tables, hierarchy, interfaces).
    type Meshes: Layer;
}

/// The standard [`Stack`]: four layer types as generic parameters.
pub struct Stack4<M, P, I, X>(PhantomData<(M, P, I, X)>);

impl<M: Layer, P: Layer, I: Layer, X: Layer> Stack for Stack4<M, P, I, X> {
    type Model = M;
    type Partition = P;
    type Intent = I;
    type Meshes = X;
}

/// The composed op type of a stack.
pub type OpOf<S> = StackOp<
    <<S as Stack>::Model as Layer>::Op,
    <<S as Stack>::Partition as Layer>::Op,
    <<S as Stack>::Intent as Layer>::Op,
    <<S as Stack>::Meshes as Layer>::Op,
>;

/// The op schema version of `op`'s layer (stored in the timeline node).
pub fn op_version<S: Stack>(op: &OpOf<S>) -> u32 {
    match op {
        StackOp::Model(_) => <S::Model as Layer>::OP_VERSION,
        StackOp::Partition(_) => <S::Partition as Layer>::OP_VERSION,
        StackOp::Intent(_) => <S::Intent as Layer>::OP_VERSION,
        StackOp::Meshes(_) => <S::Meshes as Layer>::OP_VERSION,
    }
}

/// JSON schema of the whole composed op, for MCP tools, language help and Python stubs.
pub fn op_schema<S: Stack>() -> schemars::Schema {
    schemars::schema_for!(OpOf<S>)
}
