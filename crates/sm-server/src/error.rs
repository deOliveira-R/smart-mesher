//! Errors of the server crate.

use serde_json::Value;
use sm_engine::EngineError;
use sm_fem::FemError;
use sm_lang::HostError;
use sm_protocol::RpcError;

/// Errors of [`Server`](crate::Server) construction.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum ServerError {
    /// The workspace could not be built, created or opened.
    #[error("workspace: {0}")]
    Engine(#[from] EngineError),
}

/// Errors of [`try_standard`](crate::try_standard): the standard distribution is misassembled.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum StandardError {
    /// A scheme crate failed to register its schemes.
    #[error("registering schemes of `{crate_name}`: {message}")]
    Register {
        /// The scheme crate.
        crate_name: &'static str,
        /// The registry's message.
        message: String,
    },
    /// An auto list names a scheme that no scheme crate registered.
    #[error("auto list for {dim}: scheme `{scheme}` is not registered")]
    MissingScheme {
        /// Dimension name (`curve`, `surface`, `volume`).
        dim: &'static str,
        /// The missing scheme.
        scheme: &'static str,
    },
}

/// The code of [`SessionError::internal`].
const INTERNAL_KIND: &str = "internal";

/// A failure of one session operation, before it is spelled for a transport: the contract
/// `code()` string, a message and optional structured details. Converts to [`RpcError`]
/// (protocol) and [`HostError`] (command language) without losing the code.
#[derive(Clone, Debug, PartialEq)]
pub struct SessionError {
    /// Stable kebab-case code.
    pub code: String,
    /// Human message.
    pub message: String,
    /// Structured details (`null` when none).
    pub data: Value,
}

impl SessionError {
    /// A new error without details.
    pub fn new(code: impl Into<String>, message: impl Into<String>) -> Self {
        Self { code: code.into(), message: message.into(), data: Value::Null }
    }

    /// A malformed request (`bad-request`).
    pub fn bad_request(message: impl Into<String>) -> Self {
        Self::new("bad-request", message)
    }

    /// A failure of the server itself (a bug, not a user error): becomes the protocol's
    /// `internal` error.
    pub fn internal(message: impl Into<String>) -> Self {
        Self::new(INTERNAL_KIND, message)
    }

    /// Attach structured details.
    #[must_use]
    pub fn with_data(mut self, data: Value) -> Self {
        self.data = data;
        self
    }
}

impl std::fmt::Display for SessionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{}] {}", self.code, self.message)
    }
}

impl std::error::Error for SessionError {}

impl From<&EngineError> for SessionError {
    fn from(e: &EngineError) -> Self {
        let mut out = Self::new(e.code(), e.to_string());
        match e {
            EngineError::StoppedOnWarning { diagnostics } => {
                out.data = serde_json::json!({ "diagnostics": diagnostics });
            }
            EngineError::NotPersisted { node: Some(node), .. } => {
                out.data = serde_json::json!({ "node": node });
            }
            _ => {}
        }
        out
    }
}

impl From<EngineError> for SessionError {
    fn from(e: EngineError) -> Self {
        Self::from(&e)
    }
}

impl From<&FemError> for SessionError {
    fn from(e: &FemError) -> Self {
        Self::new(e.code(), e.to_string())
    }
}

impl From<FemError> for SessionError {
    fn from(e: FemError) -> Self {
        Self::from(&e)
    }
}

impl From<SessionError> for RpcError {
    fn from(e: SessionError) -> Self {
        if e.code == INTERNAL_KIND {
            return RpcError::internal(e.message);
        }
        RpcError::application(e.code, e.message).with_data(e.data)
    }
}

impl From<SessionError> for HostError {
    fn from(e: SessionError) -> Self {
        HostError { code: e.code, message: e.message }
    }
}
