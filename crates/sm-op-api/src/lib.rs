//! Operation and timeline contract for smart-mesh (docs/ARCHITECTURE.md sections 3-4).
//!
//! Every state change is an *op*: serializable, versioned data applied by a pure, deterministic
//! function. Ops are recorded in a content-addressed timeline DAG with git-like refs. See
//! `DESIGN.md` in this crate for the model and the tradeoffs.
//!
//! Map of the crate:
//! - [`layer`]: the [`Layer`] trait (store + closed op enum), [`StackOp`] and [`Stack4`].
//! - [`state`]: [`State`], the composition of independent layer stores; transactional `apply`.
//! - [`ctx`]: what an op sees ([`ApplyCtx`]), [`Delta`], progress and cancellation.
//! - [`id`]: serializable [`Id`] and the deterministic [`IdAllocator`].
//! - [`hash`]: SHA-256 and the canonical encoding behind every content hash.
//! - [`timeline`]: nodes, refs, HEAD and the on-disk layout.
//! - [`session`]: the in-memory reference engine (snapshots, checkout, fork).
//! - [`derived`]: caches keyed by content hash.
//! - [`sample`]: a demonstration stack used by tests and as a template for layer crates.

pub mod ctx;
pub mod derived;
pub mod error;
pub mod hash;
pub mod id;
pub mod layer;
pub mod sample;
pub mod session;
pub mod state;
pub mod timeline;

pub use ctx::{ApplyCtx, CancelToken, Control, Delta, NoControl, Progress, Upstream};
pub use derived::DerivedCache;
pub use error::OpError;
pub use hash::{CanonError, Hash256, canonical_bytes, content_hash, sha256};
pub use id::{Id, IdAllocator};
pub use layer::{Layer, OpOf, Stack, Stack4, StackOp, op_schema, op_version};
pub use session::{Applied, Session, SessionConfig, SessionError};
pub use state::State;
pub use timeline::{Head, Meta, Node, NodeBody, Timeline, TimelineError};
