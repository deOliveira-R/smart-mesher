//! Session host, protocol endpoint, transports and the standard distribution (W2 task T10).
//!
//! | Item | What |
//! |---|---|
//! | [`SessionHost`] | the command-language [`Host`](sm_lang::Host) over `&mut Workspace`: apply, query, timeline control, `session.*` commands, includes |
//! | [`Server`] | [`sm_protocol::Endpoint`]: a single-writer dispatcher for every [`Request`](sm_protocol::Request) variant |
//! | [`transport`] | in-process, JSON-lines (stdio) and WebSocket (feature `ws`) |
//! | [`standard`], [`try_standard`] | the standard plugin set and the `mesher` service |
//!
//! Errors always carry the contract `code()` string in `RpcError::kind` (see [`SessionError`]).
//! The crate implements no op and no layer and parses no command text itself; scripts go to
//! `sm-lang`, ops to the plugins.
#![warn(missing_docs)]

mod b64;
mod error;
mod host;
mod server;
mod standard;
pub mod transport;

pub use error::{ServerError, SessionError, StandardError};
pub use host::{ExportParams, RELATIONS_QUERY, SessionHost};
pub use server::Server;
pub use standard::{
    distribution, distribution_with_core, reference_standard, standard, standard_schemes, standard_with, try_standard,
};
/// Re-exported so consumers need not depend on `sm-engine` directly.
pub use sm_engine::{ApplyOptions, FixedClock, Workspace};
pub use transport::{InProcess, MAX_LINE_BYTES, serve_lines, serve_lines_with_limit, serve_stdio};
#[cfg(feature = "ws")]
pub use transport::{serve_ws, serve_ws_listener};
