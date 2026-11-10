//! The tool list: one tool per op and query of the catalog, `core__batch`, and the fixed tools.

use serde_json::{Map, Value, json};
use sm_op_api::{MAX_TOOL_NAME, OpInfoOwned, QueryInfoOwned};
use sm_protocol::CatalogResult;

use crate::version::McpVersion;

/// What a tool call turns into.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Target {
    /// `Request::Apply` of the named op.
    Op(String),
    /// `Request::Query` of the named query.
    Query(String),
    /// One of the fixed tools.
    Fixed(Fixed),
}

/// The fixed tools: protocol methods that are not ops, plus `help`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Fixed {
    RunScript,
    RecordScript,
    SessionHash,
    SessionExport,
    SessionStage,
    SessionStatus,
    TimelineBranch,
    TimelineCheckout,
    TimelineUndo,
    TimelineRedo,
    TimelineLog,
    Help,
}

const FIXED: [Fixed; 12] = [
    Fixed::RunScript,
    Fixed::RecordScript,
    Fixed::SessionHash,
    Fixed::SessionExport,
    Fixed::SessionStage,
    Fixed::SessionStatus,
    Fixed::TimelineBranch,
    Fixed::TimelineCheckout,
    Fixed::TimelineUndo,
    Fixed::TimelineRedo,
    Fixed::TimelineLog,
    Fixed::Help,
];

/// Default `limit` of `timeline_log` when the caller gives none.
pub(crate) const DEFAULT_LOG_LIMIT: u64 = 20;

const HASH_PATTERN: &str = "^[0-9a-f]{64}$";

impl Fixed {
    pub(crate) fn from_name(name: &str) -> Option<Fixed> {
        FIXED.into_iter().find(|f| f.name() == name)
    }

    pub(crate) fn name(self) -> &'static str {
        match self {
            Fixed::RunScript => "run_script",
            Fixed::RecordScript => "record_script",
            Fixed::SessionHash => "session_hash",
            Fixed::SessionExport => "session_export",
            Fixed::SessionStage => "session_stage",
            Fixed::SessionStatus => "session_status",
            Fixed::TimelineBranch => "timeline_branch",
            Fixed::TimelineCheckout => "timeline_checkout",
            Fixed::TimelineUndo => "timeline_undo",
            Fixed::TimelineRedo => "timeline_redo",
            Fixed::TimelineLog => "timeline_log",
            Fixed::Help => "help",
        }
    }

    /// The protocol method this tool calls; `None` for `help`, which is answered locally.
    pub(crate) fn method(self) -> Option<&'static str> {
        Some(match self {
            Fixed::RunScript => "script.run",
            Fixed::RecordScript => "script.record",
            Fixed::SessionHash => "session.hash",
            Fixed::SessionExport => "session.export",
            // No protocol method: answered by `script.run` of `session.stage(path)`.
            Fixed::SessionStage => "script.run",
            Fixed::SessionStatus => "timeline.status",
            Fixed::TimelineBranch => "timeline.branch",
            Fixed::TimelineCheckout => "timeline.checkout",
            Fixed::TimelineUndo => "timeline.undo",
            Fixed::TimelineRedo => "timeline.redo",
            Fixed::TimelineLog => "timeline.log",
            Fixed::Help => return None,
        })
    }

    fn title(self) -> &'static str {
        match self {
            Fixed::RunScript => "Run script",
            Fixed::RecordScript => "Record script",
            Fixed::SessionHash => "State hash",
            Fixed::SessionExport => "Export mesh",
            Fixed::SessionStage => "Stage input file",
            Fixed::SessionStatus => "Session status",
            Fixed::TimelineBranch => "Create branch",
            Fixed::TimelineCheckout => "Check out revision",
            Fixed::TimelineUndo => "Undo",
            Fixed::TimelineRedo => "Redo",
            Fixed::TimelineLog => "Timeline log",
            Fixed::Help => "Help",
        }
    }

    fn read_only(self) -> bool {
        matches!(
            self,
            Fixed::RecordScript
                | Fixed::SessionHash
                | Fixed::SessionStatus
                | Fixed::TimelineLog
                | Fixed::Help
        )
    }

    fn description(self) -> &'static str {
        match self {
            Fixed::RunScript => {
                "Run smart-mesh command-language text against the session. Commands apply ops in \
                 order and a failure stops the script. The result holds the final state hash, the \
                 printed output, one event per command and the diagnostics."
            }
            Fixed::RecordScript => {
                "Record the timeline as command-language text that rebuilds it on an empty project: \
                 one statement per node, branches as `timeline.branch`, state-hash checks at the \
                 tips. `forks: refs` records every named ref. `verify` replays the text in a scratch \
                 session and reports whether the hashes agree."
            }
            Fixed::SessionHash => {
                "Quantized hash of the whole state at `at` (the current state when absent), plus \
                 one digest per layer. Equal hashes mean equal states."
            }
            Fixed::SessionExport => {
                "Export a mesh to a file and/or inline as base64. A relative `path` resolves \
                 against the project root, else the server's working directory. `format` is \
                 inferred from the extension. Formats: smsh, vtu, exodus, openfoam, dagmc, gmsh, \
                 smrt. Without `mesh` the only mesh is exported; no mesh or several meshes is an \
                 error. With `source` model (dagmc only) no mesh is needed: the .h5m is \
                 faceted from the B-rep. Export notes come back in `notes`; those with severity `warning` (for \
                 example the `.smrt` fallback to volume-id region names) also lead the text \
                 content as `warning:` lines, before the JSON."
            }
            Fixed::SessionStage => {
                "Read a file (for example a STEP part) into the project's content-addressed input \
                 store. Returns `{sha256, bytes}`; pass the `sha256` as `source` of \
                 `model__import_step`. Staging is idempotent."
            }
            Fixed::SessionStatus => {
                "Where the session stands: HEAD, current node, depth, state hash, branches, redo \
                 depth and whether anything is unsaved."
            }
            Fixed::TimelineBranch => {
                "Create a branch (at `at`, else the current node) and attach HEAD to it."
            }
            Fixed::TimelineCheckout => {
                "Move HEAD to a revision: a branch name, a node hash, or `empty`."
            }
            Fixed::TimelineUndo => "Step back one node on the timeline.",
            Fixed::TimelineRedo => "Step forward one node after an undo.",
            Fixed::TimelineLog => "Timeline nodes, newest first, starting at `from` (else HEAD).",
            Fixed::Help => {
                "List the available tools, or describe one. `topic` is a tool name \
                 (`mesh__run`) or an op or query name (`mesh.run`)."
            }
        }
    }

    fn input_schema(self) -> Value {
        let hash = |d: &str| json!({"type": "string", "pattern": HASH_PATTERN, "description": d});
        let rev = |d: &str| {
            json!({"description": format!(
                "{d} A branch name, a 64-hex node hash, or \"empty\" (the state before any op). \
                 To name a branch literally called `empty`, or one whose name is 64 lowercase hex \
                 characters, use {{\"ref\": name}}; a node can also be given as {{\"node\": hash}}."
            ), "anyOf": [
                {"type": "string"},
                {"type": "object", "properties": {"ref": {"type": "string"}}, "required": ["ref"], "additionalProperties": false},
                {"type": "object", "properties": {"node": {"type": "string", "pattern": HASH_PATTERN}}, "required": ["node"], "additionalProperties": false},
            ]})
        };
        let (properties, required) = match self {
            Fixed::RunScript => (
                json!({
                    "text": {"type": "string", "description": "Script source."},
                    "path": {"type": "string", "description": "Script path, for diagnostics and relative includes."},
                    "includes": {
                        "type": "object",
                        "additionalProperties": {"type": "string"},
                        "description": "In-memory include files by name."
                    },
                }),
                json!(["text"]),
            ),
            Fixed::RecordScript => (
                json!({
                    "forks": {"type": "string", "enum": ["head", "refs"], "description": "Which heads to record (default head)."},
                    "check": {"type": "string", "enum": ["none", "tips", "every"], "description": "Hash checks to embed (default tips)."},
                    "verify": {"type": "boolean", "description": "Replay the text in a scratch session and report `verified`."},
                    "raw": {"type": "boolean", "description": "Write raw `apply` records instead of command sugar."},
                }),
                json!([]),
            ),
            Fixed::SessionHash => (json!({"at": rev("Revision to hash.")}), json!([])),
            Fixed::SessionExport => (
                json!({
                    "path": {"type": "string", "description": "Output file; omit for inline-only output."},
                    "format": {"type": "string", "enum": ["smsh", "vtu", "exodus", "openfoam", "dagmc", "gmsh", "smrt"], "description": "Defaults to the path extension. openfoam: path is the case directory."},
                    "mesh": {
                        "type": "string",
                        "pattern": "^[A-Za-z]+#[0-9]+$",
                        "x-entity-kind": ["Mesh"],
                        "description": "Mesh id; optional when the state holds exactly one mesh."
                    },
                    "provenance": {"type": "boolean", "description": "Include provenance data."},
                    "inline": {"type": "boolean", "description": "Return the bytes base64-encoded in the result."},
                    "options": {"type": "object", "description": "Per-format options, validated by the exporter: gmsh {binary: boolean, polyhedra: \"error\"|\"decompose\"|\"drop\"}, dagmc {graveyard, chord_tol (source model only)}; smrt takes none."},
                    "source": {"type": "string", "enum": ["mesh", "model"], "description": "What is exported (default mesh). model: DAGMC only; facets straight from the B-rep at options.chord_tol, no volume mesh (do not pass `mesh`)."},
                    "split_small_determinant": {"type": "boolean", "description": "OpenFOAM only: split cells that fail checkMesh's determinant check (default true)."},
                    "split_warped_faces": {"type": "boolean", "description": "OpenFOAM only: split warped faces into planar triangles (default false; makes checkMesh's concave-cell check fail on warped hex meshes)."},
                    "empty_patches": {"type": "array", "items": {"type": "string"}, "description": "OpenFOAM only: face set names written as `empty` patches (2D cases)."},
                    "wedge_patches": {"type": "array", "items": {"type": "string"}, "description": "OpenFOAM only: face set names written as `wedge` patches (axisymmetric cases)."},
                }),
                json!([]),
            ),
            Fixed::SessionStage => (
                json!({"path": {"type": "string", "minLength": 1, "description": "File to stage; relative paths resolve against the project root, else the working directory."}}),
                json!(["path"]),
            ),
            Fixed::TimelineBranch => (
                json!({
                    "name": {"type": "string", "description": "Branch name."},
                    "at": hash("Node to branch at; the current node when absent."),
                }),
                json!(["name"]),
            ),
            Fixed::TimelineCheckout => (json!({"rev": rev("Target revision.")}), json!(["rev"])),
            Fixed::TimelineLog => (
                json!({
                    "from": hash("Start node; HEAD when absent."),
                    "limit": {"type": "integer", "minimum": 0, "description": format!("Maximum entries; default {DEFAULT_LOG_LIMIT}.")},
                }),
                json!([]),
            ),
            Fixed::Help => (
                json!({"topic": {"type": "string", "description": "Tool, op or query name."}}),
                json!([]),
            ),
            Fixed::SessionStatus | Fixed::TimelineUndo | Fixed::TimelineRedo => {
                (json!({}), json!([]))
            }
        };
        json!({
            "type": "object",
            "properties": properties,
            "required": required,
            "additionalProperties": false,
        })
    }
}

/// One entry of `tools/list`.
#[derive(Clone, Debug)]
pub(crate) struct Tool {
    pub name: String,
    pub title: String,
    pub description: String,
    pub input_schema: Value,
    pub read_only: bool,
    pub target: Target,
}

impl Tool {
    /// The tool as `tools/list` shows it under `version`: newer fields are left out for older
    /// clients. Keys are sorted (see [`sorted`]), so the bytes do not depend on build features.
    pub(crate) fn to_json(&self, version: McpVersion) -> Value {
        let mut m = Map::new();
        m.insert("name".into(), json!(self.name));
        m.insert("description".into(), json!(self.description));
        m.insert("inputSchema".into(), self.input_schema.clone());
        if version.has_structured_output() {
            m.insert("title".into(), json!(self.title));
        }
        if version.has_annotations() && self.read_only {
            m.insert("annotations".into(), json!({"readOnlyHint": true}));
        }
        sorted(Value::Object(m))
    }
}

/// `v` with every object's keys in sorted order.
pub(crate) fn sorted(v: Value) -> Value {
    match v {
        Value::Object(m) => {
            let mut entries: Vec<(String, Value)> = m.into_iter().collect();
            entries.sort_by(|a, b| a.0.cmp(&b.0));
            Value::Object(entries.into_iter().map(|(k, v)| (k, sorted(v))).collect())
        }
        Value::Array(a) => Value::Array(a.into_iter().map(sorted).collect()),
        other => other,
    }
}

/// `^[a-zA-Z0-9_-]{1,64}$`: what MCP clients and LLM APIs accept as a tool name.
pub(crate) fn valid_tool_name(name: &str) -> bool {
    !name.is_empty()
        && name.len() <= MAX_TOOL_NAME
        && name
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-')
}

/// All tools of `catalog` plus the fixed ones, sorted by name. A catalog entry whose name is not
/// a valid tool name, or that repeats an earlier name (ops win over queries, fixed tools win
/// over both), is left out so the list is always valid.
pub(crate) fn build(catalog: &CatalogResult) -> Vec<Tool> {
    let mut tools: Vec<Tool> = FIXED.into_iter().map(fixed_tool).collect();
    let mut push = |t: Tool| {
        if valid_tool_name(&t.name) && !tools.iter().any(|x| x.name == t.name) {
            tools.push(t);
        }
    };
    let mut batch_seen = false;
    for op in &catalog.ops {
        batch_seen |= op.name == "core.batch";
        push(op_tool(op, &catalog.batch_schema));
    }
    if !batch_seen {
        push(batch_tool(&catalog.batch_schema));
    }
    for q in &catalog.queries {
        push(query_tool(q));
    }
    tools.sort_by(|a, b| a.name.cmp(&b.name));
    tools
}

fn fixed_tool(f: Fixed) -> Tool {
    Tool {
        name: f.name().into(),
        title: f.title().into(),
        description: f.description().into(),
        input_schema: f.input_schema(),
        read_only: f.read_only(),
        target: Target::Fixed(f),
    }
}

fn batch_tool(batch_schema: &Value) -> Tool {
    Tool {
        name: "core__batch".into(),
        title: "core.batch".into(),
        description: schema_description(batch_schema)
            .unwrap_or_else(|| "Apply several ops atomically as one node.".into()),
        input_schema: object_schema(batch_schema.clone()),
        read_only: false,
        target: Target::Op("core.batch".into()),
    }
}

fn op_tool(op: &OpInfoOwned, batch_schema: &Value) -> Tool {
    if op.name == "core.batch" {
        let mut t = batch_tool(batch_schema);
        t.name = op.tool_name.clone();
        return t;
    }
    let schema = serde_json::to_value(&op.schema).unwrap_or(Value::Null);
    let mut desc = schema_description(&schema)
        .unwrap_or_else(|| format!("Apply op `{}`.", op.name));
    desc.push_str(&format!(
        "\nOp `{}` (payload version {}). The result holds `node`, `state` and `delta`.",
        op.name, op.version
    ));
    if !op.outputs.is_empty() {
        desc.push_str("\n`delta.outputs` roles:");
        for r in &op.outputs {
            let kinds: Vec<String> = r
                .kinds
                .iter()
                .map(|k| value_str(serde_json::to_value(k).unwrap_or(Value::Null)))
                .collect();
            let kinds = if kinds.is_empty() {
                "any".to_owned()
            } else {
                kinds.join("|")
            };
            let card = value_str(serde_json::to_value(r.cardinality).unwrap_or(Value::Null));
            desc.push_str(&format!("\n- {} ({kinds}, {card}): {}", r.name, r.doc));
        }
    }
    Tool {
        name: op.tool_name.clone(),
        title: op.name.clone(),
        description: desc,
        input_schema: object_schema(schema),
        read_only: false,
        target: Target::Op(op.name.clone()),
    }
}

fn query_tool(q: &QueryInfoOwned) -> Tool {
    let schema = serde_json::to_value(&q.schema).unwrap_or(Value::Null);
    let mut desc = schema_description(&schema)
        .unwrap_or_else(|| format!("Run query `{}`.", q.name));
    desc.push_str(&format!(
        "\nRead-only query `{}` on the current state. A non-object result is returned as \
         `{{\"result\": value}}` in `structuredContent`.",
        q.name
    ));
    Tool {
        name: q.tool_name.clone(),
        title: q.name.clone(),
        description: desc,
        input_schema: object_schema(schema),
        read_only: true,
        target: Target::Query(q.name.clone()),
    }
}

fn value_str(v: Value) -> String {
    match v {
        Value::String(s) => s,
        other => other.to_string(),
    }
}

fn schema_description(schema: &Value) -> Option<String> {
    schema
        .get("description")
        .and_then(Value::as_str)
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(str::to_owned)
}

/// MCP requires `inputSchema` to have root `type: object`: a non-object schema becomes an empty
/// one, and `type` is set to `object`.
fn object_schema(schema: Value) -> Value {
    let Value::Object(mut m) = schema else {
        return json!({"type": "object"});
    };
    m.insert("type".into(), json!("object"));
    Value::Object(m)
}
