//! Contract for operations and the timeline (docs/ARCHITECTURE.md §3-4).
//!
//! * [`Op`]: a serializable, versioned value with a deterministic `apply`.
//! * [`State`]: a composable set of independent layer stores ([`Layer`]) the api crate
//!   knows nothing about; concepts register layers and ops in a [`Registry`].
//! * Every applied op yields a reversible [`Delta`] (layer patches with their inverses plus
//!   id-counter movement). Ops remain the source of truth; deltas make moving along the DAG
//!   cheap.
//! * [`Node`]/[`Workspace`]: a content-addressed timeline DAG stored as one canonical TOML
//!   file per node, with branches and HEAD like git; see [`MemStore`] for the layout.
//! * [`Registry::catalog`]: JSON schemas of all ops for MCP tools, help and Python stubs.
//!
//! See `DESIGN.md` in the crate root for the model and trade-offs.

pub mod eid;
pub mod error;
pub mod hash;
pub mod layer;
pub mod op;
pub mod state;
pub mod timeline;
pub mod toml;
pub mod value;

pub use eid::Eid;
pub use error::{ApplyError, LayerError, OpError, RegistryError, TimelineError};
pub use hash::{Hash, StateHasher};
pub use layer::{DynLayer, Layer, LayerId, MapEntry, MapLayer, MapPatch};
pub use op::{
    AppliedOp, ApplyCtx, ApplyOptions, CancelToken, Concept, Migration, Op, OpRecord, ProgressSink,
    Registry,
};
pub use state::{CounterChange, Delta, DerivedCache, IdCounters, LayerPatch, State};
pub use timeline::{ApplyReport, Head, MemStore, Node, NodeMeta, Target, TimelineStore, Workspace};
pub use value::{Table, Value, ValueError, from_value, to_value};
