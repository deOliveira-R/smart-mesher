//! `sm-op-api`: the frozen contract for operations and the timeline.
//!
//! * [`Op`] / [`TypedOp`]: serializable, versioned, deterministic state transitions.
//! * [`Registry`] / [`Plugin`]: open registration of ops, queries, layers and migrations by name.
//! * [`State`]: independent [`LayerType`] stores plus the entity table, copy-on-write.
//! * [`Engine`]: transactional apply with progress, cancellation, services and a derived cache.
//! * [`Timeline`] / [`Session`]: content-addressed op DAG with branches, undo and replay.
//!
//! The crate performs no file I/O: [`Timeline::to_files`] / [`Timeline::from_files`] expose the
//! on-disk layout as a path to text map, and `sm-engine` writes it. See `DESIGN.md`.

pub mod batch;
pub mod cache;
pub mod codec;
pub mod engine;
pub mod error;
pub mod hash;
pub mod layer;
pub mod op;
pub mod progress;
pub mod query;
pub mod registry;
pub mod services;
pub mod session;
pub mod state;
mod text;
pub mod timeline;

pub use batch::Batch;
pub use cache::{CacheStats, DerivedCache};
pub use codec::{
    CodecError, Value, canonical_bytes, canonicalize, from_value, to_params, to_value,
};
pub use engine::Engine;
pub use error::{ApplyError, CacheError, OpError, RegistryError, TimelineError};
pub use hash::{Hash256, ParseHashError, content_hash};
pub use layer::{Digest, Layer, LayerRole, LayerType, MapLayer, Successors, names};
pub use op::{ApplyCtx, ApplyCtxScope, Delta, Diagnostic, Op, OpRecord, Severity, TypedOp};
pub use progress::{CancelToken, Control, NoProgress, Progress, QuietControl, RecordingProgress};
pub use query::{QueryCtx, QueryInfo, QueryType, TypedQuery};
pub use registry::{MAX_TOOL_NAME, Migration, OpInfo, OpType, Plugin, Registry, valid_op_name};
pub use services::Services;
pub use session::{PrepareBase, Prepared, ReflogEntry, ReplayPolicy, Rev, Session};
pub use state::{KindSnapshot, State, StateSnapshot};
pub use timeline::{Check, Head, Meta, NODE_DOMAIN, Node, NodeBody, Timeline, valid_ref_name};
