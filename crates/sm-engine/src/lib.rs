//! `sm-engine`: the session engine on top of `sm-op-api`.
//!
//! * [`Workspace`] is the facade: plugins (layers, ops, queries) register once, then `apply`,
//!   `query`, `checkout`, `fork`, `undo` and friends work on JSON values, so a server, an MCP
//!   adapter or Python bindings can drive it without knowing any op type.
//! * The project directory format, atomic writes and incremental autosave live in [`fs`],
//!   [`project`] and [`fsio`]; per-checkout state (HEAD, reflog, redo stack, snapshots) lives in
//!   the gitignored `.cache/`.
//! * [`policy`] holds the diagnostics policy (verbosity, suppression, stop-on-warning) and the
//!   clocks used for node timestamps.
//!
//! See `DESIGN.md` for the format, the crash-safety argument and the positioning statement.

#![forbid(unsafe_code)]

mod cache;
pub mod error;
pub mod fs;
pub mod fsio;
pub mod policy;
pub mod project;
pub mod workspace;

pub use error::{EngineError, Result};
pub use fs::{Layout, LoadReport, load_timeline, read_timeline_files, save_timeline};
pub use fsio::{Durability, FaultInjector};
pub use policy::{Clock, DiagnosticPolicy, FixedClock, SystemClock, Verbosity, rfc3339_utc};
pub use project::{FORMAT_VERSION, ProjectConfig, Tolerances};
pub use workspace::{
    ApplyOptions, ApplyReport, LogEntry, OpenReport, Status, Workspace, WorkspaceBuilder,
};
