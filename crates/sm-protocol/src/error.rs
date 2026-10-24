use serde::{Deserialize, Serialize};
use serde_json::Value;

/// JSON-RPC: invalid JSON was received.
pub const PARSE_ERROR: i64 = -32700;
/// JSON-RPC: the JSON is not a valid request object (missing `jsonrpc`, `method`, `id`, ...).
pub const INVALID_REQUEST: i64 = -32600;
/// JSON-RPC: the `method` is not one of [`METHODS`](crate::METHODS).
pub const METHOD_NOT_FOUND: i64 = -32601;
/// JSON-RPC: `params` do not match the method's shape.
pub const INVALID_PARAMS: i64 = -32602;
/// JSON-RPC: internal error of the endpoint itself (a bug, not a user error).
pub const INTERNAL_ERROR: i64 = -32603;
/// Application errors (op failures, timeline errors, I/O, ...) use the JSON-RPC server-error
/// range `-32000..=-32099`; the code is always this one and [`RpcError::kind`] carries the
/// stable contract `code()` string (`op-invalid`, `unknown-entity`, ...) to branch on.
pub const APPLICATION_ERROR: i64 = -32000;

/// A JSON-RPC error object with the contract error code as `kind`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub struct RpcError {
    /// JSON-RPC code: one of the protocol constants ([`PARSE_ERROR`], ...,
    /// [`APPLICATION_ERROR`]).
    pub code: i64,
    /// Stable kebab-case string from the contract error's `code()`; for protocol-level errors
    /// one of `parse-error`, `invalid-request`, `method-not-found`, `invalid-params`, `internal`.
    pub kind: String,
    /// Human-readable message.
    pub message: String,
    /// Optional structured details; `null` when absent.
    #[serde(default, skip_serializing_if = "Value::is_null")]
    pub data: Value,
}

impl RpcError {
    /// Build an error from its parts, with no `data`.
    pub fn new(code: i64, kind: impl Into<String>, message: impl Into<String>) -> Self {
        Self {
            code,
            kind: kind.into(),
            message: message.into(),
            data: Value::Null,
        }
    }
    /// Application error ([`APPLICATION_ERROR`]) with a contract `code()` string as `kind`.
    pub fn application(kind: impl Into<String>, message: impl Into<String>) -> Self {
        Self::new(APPLICATION_ERROR, kind, message)
    }
    /// Invalid JSON text.
    pub fn parse(message: impl Into<String>) -> Self {
        Self::new(PARSE_ERROR, "parse-error", message)
    }
    /// Malformed request object.
    pub fn invalid_request(message: impl Into<String>) -> Self {
        Self::new(INVALID_REQUEST, "invalid-request", message)
    }
    /// Unknown method name.
    pub fn method_not_found(method: &str) -> Self {
        Self::new(
            METHOD_NOT_FOUND,
            "method-not-found",
            format!("unknown method `{method}`"),
        )
    }
    /// Params that do not fit the method.
    pub fn invalid_params(message: impl Into<String>) -> Self {
        Self::new(INVALID_PARAMS, "invalid-params", message)
    }
    /// Endpoint-internal failure.
    pub fn internal(message: impl Into<String>) -> Self {
        Self::new(INTERNAL_ERROR, "internal", message)
    }
    /// Attach structured details.
    #[must_use]
    pub fn with_data(mut self, data: Value) -> Self {
        self.data = data;
        self
    }
}

impl std::fmt::Display for RpcError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} ({}): {}", self.kind, self.code, self.message)
    }
}

impl std::error::Error for RpcError {}
