//! Contract for operations and the timeline (docs/ARCHITECTURE.md §3-4).
//!
//! Model: every state change is an [`Op`]. Ops are plain data registered **by name** in an open
//! [`Registry`] (plugin style: each concept node installs its own ops, layers and migrations,
//! and this crate knows none of them). A [`State`] is a set of independent layer stores plus a
//! live-entity table. The [`Engine`] applies an op transactionally. A [`Timeline`] is a DAG of
//! content-addressed [`Node`]s with git-like refs; [`Session`] ties these together with
//! snapshots, checkout and branching; [`fs`] persists the layout as one TOML file per node.
//!
//! See `DESIGN.md` for the rationale and how each requirement is met.

pub mod engine;
pub mod error;
pub mod fs;
pub mod hash;
pub mod layer;
pub mod op;
pub mod progress;
pub mod registry;
pub mod session;
pub mod state;
pub mod timeline;
pub mod toml;
pub mod value;

pub use engine::Engine;
pub use error::{ApplyError, OpError, RegistryError, TimelineError};
pub use hash::{Hash256, Sha256};
pub use layer::{DerivedCache, Digest, Layer, LayerRole, LayerType, names};
pub use op::{ApplyCtx, Delta, EntityRef, Op, OpRecord, TypedOp};
pub use progress::{CancelToken, NoProgress, Progress, RecordingProgress};
pub use registry::{Migration, OpInfo, OpType, Plugin, Registry};
pub use session::{Rev, Session};
pub use state::State;
pub use timeline::{Head, Meta, Node, Timeline};
pub use value::{CodecError, Value, from_value, to_value};
