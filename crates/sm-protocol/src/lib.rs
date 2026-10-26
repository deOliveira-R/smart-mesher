//! The smart-mesh wire protocol, shared by `sm-server`, `sm-mcp`, `sm-cli` and the Python
//! bindings: JSON-RPC 2.0 envelopes, the [`Request`] enum, result structs, [`RpcError`] and the
//! [`Endpoint`] trait a server implements.
//!
//! **Unstable:** [`PROTOCOL_VERSION`] is 0 and the wire format may change until W3. Ids, hashes,
//! deltas, revisions, op and query descriptions are the contract types of `sm-core` and
//! `sm-op-api`, not copies.
//!
//! Wire shape of a call: `{"jsonrpc":"2.0","id":1,"method":"apply","params":{...}}`; the reply is
//! a [`Response`] carrying either `result` or `error`.
//!
//! **Notifications are not supported.** A JSON-RPC request without an `id` is rejected by
//! [`Envelope::parse`] with [`INVALID_REQUEST`], and the transport (`sm-server`) must still send
//! that error reply, with `id: null`, instead of staying silent; [`Endpoint::handle_text`] does
//! exactly this. A transport that parses with `Envelope::parse` itself must do the same.
#![warn(missing_docs)]

#[doc(hidden)]
pub mod fixtures;
mod envelope;
mod error;
mod request;
mod result;

// Re-exported so front ends need not depend on three contract crates for the wire's id types.
pub use sm_core::EntityId;
pub use sm_op_api::{Hash256, Head, Rev};

pub use envelope::{Endpoint, Envelope, Response};
pub use error::{
    APPLICATION_ERROR, INTERNAL_ERROR, INVALID_PARAMS, INVALID_REQUEST, METHOD_NOT_FOUND,
    PARSE_ERROR, RpcError,
};
pub use request::{ExportFormat, METHODS, Request};
pub use result::{
    ApplyResult, CatalogResult, ExportResult, ExportedFile, HashResult, ScriptResult, StatusResult,
};

/// Protocol version carried in [`CatalogResult::protocol`]. 0 means unstable until W3: any
/// change may land without a bump.
pub const PROTOCOL_VERSION: u32 = 0;
