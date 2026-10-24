use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use serde_json::Value;
use sm_op_api::{Delta, Hash256, Head, OpInfo, QueryInfo};

/// Result of `apply`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ApplyResult {
    /// The new timeline node.
    pub node: Hash256,
    /// State hash after the op.
    pub state: Hash256,
    /// What the op changed.
    pub delta: Delta,
}

/// Result of `session.hash`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct HashResult {
    /// Quantized hash of the whole state.
    pub state: Hash256,
    /// Per-layer digests by layer name.
    pub layers: BTreeMap<String, Hash256>,
}

/// Result of `session.export`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExportResult {
    /// File written, if any.
    pub path: Option<String>,
    /// Format produced.
    pub format: crate::ExportFormat,
    /// Size in bytes.
    pub bytes: u64,
    /// SHA-256 of the exported bytes.
    pub sha256: Hash256,
    /// The bytes, base64-encoded, when `inline` was requested.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub base64: Option<String>,
}

/// Result of `script.run`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ScriptResult {
    /// State hash after the script.
    pub state: Hash256,
    /// Text the script printed.
    pub output: String,
    /// One entry per executed command, in order (shape owned by the server).
    pub events: Vec<Value>,
    /// Warnings and notes (shape owned by the server).
    pub diagnostics: Value,
}

/// Snapshot of where the workspace stands. Mirrors `sm_engine::Status` field for field, built
/// from contract types so this crate need not depend on `sm-engine`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct StatusResult {
    /// HEAD (serializes like a `Rev`).
    pub head: Head,
    /// Current node (`null` = empty state).
    pub position: Option<Hash256>,
    /// Depth of the current node (0 when empty).
    pub depth: usize,
    /// Quantized hash of the current state.
    pub state: Hash256,
    /// Nodes in the timeline.
    pub nodes: usize,
    /// Branches and their tips.
    pub branches: BTreeMap<String, Hash256>,
    /// Nodes `redo` can step through.
    pub redo: usize,
    /// True when something is not on disk yet.
    pub dirty: bool,
    /// Project name, if a project is attached.
    pub project: Option<String>,
}

/// Result of `catalog`. Serialize-only: `OpInfo`, `QueryInfo` and `OutputRole` hold
/// `&'static` data and do not implement `Deserialize`, so remote clients (`sm-mcp`, `sm-cli`)
/// read the reply as a `serde_json::Value`; its exact shape is pinned by
/// `golden/catalog.json` (`fixtures::CATALOG`). A typed client form needs an owned
/// `Deserialize` form of those types in `sm-op-api` (orchestrator ACR).
#[derive(Clone, Debug, Serialize)]
pub struct CatalogResult {
    /// [`PROTOCOL_VERSION`](crate::PROTOCOL_VERSION) of the endpoint.
    pub protocol: u32,
    /// Registered ops.
    pub ops: Vec<OpInfo>,
    /// Registered queries.
    pub queries: Vec<QueryInfo>,
    /// Schema of the `core.batch` op.
    pub batch_schema: Value,
}
