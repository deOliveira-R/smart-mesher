use std::collections::BTreeMap;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sm_core::EntityId;
use sm_op_api::{Hash256, Rev};

/// Export file format of `session.export`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
#[non_exhaustive]
pub enum ExportFormat {
    /// The native mesh container.
    Smsh,
    /// VTK XML unstructured grid.
    Vtu,
}

/// Every method name of the protocol, sorted. [`Request`]'s serde tags are exactly these.
pub const METHODS: &[&str] = &[
    "apply",
    "catalog",
    "query",
    "script.run",
    "session.export",
    "session.hash",
    "session.open",
    "session.save",
    "timeline.branch",
    "timeline.checkout",
    "timeline.log",
    "timeline.redo",
    "timeline.status",
    "timeline.undo",
];

fn is_false(b: &bool) -> bool {
    !*b
}
fn is_null(v: &Value) -> bool {
    v.is_null()
}

/// One call. Serializes as `{"method": "...", "params": {...}}` (no `params` for unit
/// variants), the method/params part of a JSON-RPC request. Unset optional fields are omitted
/// on output and accepted missing or `null` on input. Unknown params fields are rejected
/// (`INVALID_PARAMS`) so a typo such as `creat` never silently changes meaning.
///
/// Plain serde parsing of `Request` is strict about the `params` key itself. The lenient shapes
/// real clients send are normalized by [`Envelope::from_value`](crate::Envelope::from_value),
/// the entry point for wire input.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "method", content = "params", deny_unknown_fields)]
#[non_exhaustive]
pub enum Request {
    /// List ops, queries and the batch schema: result is a [`CatalogResult`](crate::CatalogResult).
    #[serde(rename = "catalog")]
    Catalog,
    /// Apply one op: result is an [`ApplyResult`](crate::ApplyResult).
    #[serde(rename = "apply")]
    Apply {
        /// Registry name of the op.
        op: String,
        /// Op params (an object).
        params: Value,
        /// Payload version the params were written with; the current one when absent.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        version: Option<u32>,
    },
    /// Run a read-only query: the result is the query's output value.
    #[serde(rename = "query")]
    Query {
        /// Query name.
        name: String,
        /// Query input.
        #[serde(default, skip_serializing_if = "is_null")]
        input: Value,
        /// Revision to evaluate at; the current state when absent.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        at: Option<Rev>,
    },
    /// Run command-language text: result is a [`ScriptResult`](crate::ScriptResult).
    #[serde(rename = "script.run")]
    ScriptRun {
        /// Script source.
        text: String,
        /// Path of the script, for diagnostics and relative includes.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        path: Option<String>,
        /// In-memory include files by name.
        #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
        includes: BTreeMap<String, String>,
    },
    /// Where the workspace stands: result is a [`StatusResult`](crate::StatusResult).
    #[serde(rename = "timeline.status")]
    Status,
    /// Node log, newest first, as an array of nodes.
    #[serde(rename = "timeline.log")]
    Log {
        /// Start node; HEAD when absent.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        from: Option<Hash256>,
        /// Maximum number of entries.
        limit: usize,
    },
    /// Move HEAD to a revision: result is a [`StatusResult`](crate::StatusResult).
    #[serde(rename = "timeline.checkout")]
    Checkout {
        /// Target revision.
        rev: Rev,
    },
    /// Create a branch and attach to it: result is a [`StatusResult`](crate::StatusResult).
    #[serde(rename = "timeline.branch")]
    Branch {
        /// Branch name.
        name: String,
        /// Node to branch at; the current position when absent.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        at: Option<Hash256>,
    },
    /// Step back one node: result is a [`StatusResult`](crate::StatusResult).
    #[serde(rename = "timeline.undo")]
    Undo,
    /// Step forward one node: result is a [`StatusResult`](crate::StatusResult).
    #[serde(rename = "timeline.redo")]
    Redo,
    /// State hash: result is a [`HashResult`](crate::HashResult).
    #[serde(rename = "session.hash")]
    Hash {
        /// Revision to hash; the current state when absent.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        at: Option<Rev>,
    },
    /// Export a mesh: result is an [`ExportResult`](crate::ExportResult).
    #[serde(rename = "session.export")]
    Export {
        /// Output file path; none for inline-only output.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        path: Option<String>,
        /// Format; inferred from the path extension when absent.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        format: Option<ExportFormat>,
        /// Mesh to export. When absent, the only mesh in the state is exported; no mesh or several
        /// meshes is an error (`no-mesh` / `ambiguous-mesh`).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        mesh: Option<EntityId>,
        /// Include provenance data.
        #[serde(default, skip_serializing_if = "is_false")]
        provenance: bool,
        /// Return the bytes base64-encoded in the result.
        #[serde(default, skip_serializing_if = "is_false")]
        inline: bool,
    },
    /// Persist pending state to disk: result is a [`StatusResult`](crate::StatusResult).
    #[serde(rename = "session.save")]
    Save,
    /// Open (or create) a project directory: result is a [`StatusResult`](crate::StatusResult).
    #[serde(rename = "session.open")]
    Open {
        /// Project directory.
        path: String,
        /// Create it when missing.
        #[serde(default, skip_serializing_if = "is_false")]
        create: bool,
    },
}

impl Request {
    /// The wire method name (an element of [`METHODS`]).
    pub fn method(&self) -> &'static str {
        match self {
            Request::Catalog => "catalog",
            Request::Apply { .. } => "apply",
            Request::Query { .. } => "query",
            Request::ScriptRun { .. } => "script.run",
            Request::Status => "timeline.status",
            Request::Log { .. } => "timeline.log",
            Request::Checkout { .. } => "timeline.checkout",
            Request::Branch { .. } => "timeline.branch",
            Request::Undo => "timeline.undo",
            Request::Redo => "timeline.redo",
            Request::Hash { .. } => "session.hash",
            Request::Export { .. } => "session.export",
            Request::Save => "session.save",
            Request::Open { .. } => "session.open",
        }
    }

    /// True for methods without params (`catalog`, `timeline.status`, `timeline.undo`,
    /// `timeline.redo`, `session.save`). Their wire `params` may be absent, `null`, `{}` or `[]`.
    pub fn is_unit_method(method: &str) -> bool {
        matches!(
            method,
            "catalog" | "timeline.status" | "timeline.undo" | "timeline.redo" | "session.save"
        )
    }

    /// JSON Schema of the whole request, for client and tool generation.
    ///
    /// The schema is **stricter than the wire**: it describes the canonical form, so it requires
    /// a `params` object for `session.hash` and `session.export` even though every field of
    /// those is optional. [`Envelope::from_value`](crate::Envelope::from_value) accepts them
    /// with `params` absent, `null`, `{}` or `[]` (and unit methods likewise). Validate wire
    /// input with `Envelope::parse`, not with this schema.
    pub fn json_schema() -> Value {
        serde_json::to_value(schemars::schema_for!(Request)).unwrap_or(Value::Null)
    }
}
