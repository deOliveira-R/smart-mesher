//! JSON-RPC dispatch: lifecycle, `tools/list`, `tools/call`, framing errors.

use std::borrow::Cow;

use serde_json::{Map, Value, json};
use sm_protocol::{
    APPLICATION_ERROR, CatalogResult, Endpoint, Envelope, INVALID_PARAMS, INVALID_REQUEST,
    METHOD_NOT_FOUND, PARSE_ERROR, Request, RpcError,
};

use crate::tools::{self, DEFAULT_LOG_LIMIT, Fixed, Target, Tool};
use crate::version::McpVersion;

const INSTRUCTIONS: &str = "smart-mesh meshing session. Tools named `<namespace>__<op>` apply \
ops (for example `model__create_brick`); query tools read the current state. Entity ids are \
`Kind#serial` strings (for example `TopoVolume#1`); take them from `delta.outputs` of earlier \
results. `run_script` runs command-language text, `session_hash` gives a reproducibility hash, \
`help` lists and describes tools.";

/// A JSON-RPC error this layer sends.
pub(crate) struct Rpc {
    code: i64,
    message: String,
    data: Option<Value>,
}

impl Rpc {
    fn new(code: i64, message: impl Into<String>) -> Self {
        Self {
            code,
            message: message.into(),
            data: None,
        }
    }
    fn invalid_params(message: impl Into<String>) -> Self {
        Self::new(INVALID_PARAMS, message)
    }
    fn invalid_request(message: impl Into<String>) -> Self {
        Self::new(INVALID_REQUEST, message)
    }
    fn internal(message: impl Into<String>) -> Self {
        Self::new(sm_protocol::INTERNAL_ERROR, message)
    }
    fn to_json(&self) -> Value {
        let mut e = Map::new();
        e.insert("code".into(), json!(self.code));
        e.insert("message".into(), json!(self.message));
        if let Some(d) = &self.data {
            e.insert("data".into(), d.clone());
        }
        Value::Object(e)
    }
}

/// An `RpcError` of the protocol layer that is a protocol error here too (not a tool failure).
fn lift(e: RpcError) -> Rpc {
    let data = json!({"kind": e.kind});
    let code = if e.code == INVALID_PARAMS {
        INVALID_PARAMS
    } else {
        sm_protocol::INTERNAL_ERROR
    };
    let message = if code == INVALID_PARAMS {
        e.message
    } else {
        format!("{} ({}): {}", e.kind, e.code, e.message)
    };
    Rpc {
        code,
        message,
        data: Some(data),
    }
}

fn reply_ok(id: &Value, result: Value) -> Value {
    json!({"jsonrpc": "2.0", "id": id, "result": result})
}

fn reply_err(id: &Value, e: &Rpc) -> Value {
    json!({"jsonrpc": "2.0", "id": id, "error": e.to_json()})
}

/// One MCP session over one endpoint.
pub(crate) struct Mcp<'e> {
    endpoint: &'e mut dyn Endpoint,
    /// Negotiated by `initialize`; the newest version until then.
    version: McpVersion,
    /// Memo of the tool list. The catalog is fixed when the endpoint is built, so this is a cache
    /// of an immutable endpoint property, not session state.
    tools: Option<Vec<Tool>>,
}

impl<'e> Mcp<'e> {
    pub(crate) fn new(endpoint: &'e mut dyn Endpoint) -> Self {
        Self {
            endpoint,
            version: McpVersion::LATEST,
            tools: None,
        }
    }

    /// Handle one input line; `None` when no reply is due (blank line, notification, a response
    /// from the client).
    pub(crate) fn handle_line(&mut self, line: &[u8]) -> Option<Value> {
        let Ok(text) = std::str::from_utf8(line) else {
            return Some(reply_err(
                &Value::Null,
                &Rpc::new(PARSE_ERROR, "input is not valid UTF-8"),
            ));
        };
        let text = text.trim();
        if text.is_empty() {
            return None;
        }
        let msg: Value = match serde_json::from_str(text) {
            Ok(v) => v,
            Err(e) => {
                return Some(reply_err(
                    &Value::Null,
                    &Rpc::new(PARSE_ERROR, format!("parse error: {e}")),
                ));
            }
        };
        self.handle_message(msg)
    }

    fn handle_message(&mut self, msg: Value) -> Option<Value> {
        let Value::Object(obj) = msg else {
            let what = if msg.is_array() {
                "JSON-RPC batching is not supported (removed in MCP 2025-06-18)"
            } else {
                "request must be a JSON object"
            };
            return Some(reply_err(&Value::Null, &Rpc::invalid_request(what)));
        };
        let id = obj.get("id");
        let Some(method) = obj.get("method") else {
            // A response to a request we never sent is ignored; anything else is malformed.
            if obj.contains_key("result") || obj.contains_key("error") {
                return None;
            }
            // JSON-RPC: `id: null` only when the id cannot be determined.
            let id = id.filter(|i| id_is_valid(i)).unwrap_or(&Value::Null);
            return Some(reply_err(id, &Rpc::invalid_request("missing `method`")));
        };
        let Some(id) = id else {
            return None; // notification: never answered
        };
        if !id_is_valid(id) {
            return Some(reply_err(
                &Value::Null,
                &Rpc::invalid_request("`id` must be a string or a number"),
            ));
        }
        if obj.get("jsonrpc").and_then(Value::as_str) != Some("2.0") {
            return Some(reply_err(id, &Rpc::invalid_request("`jsonrpc` must be \"2.0\"")));
        }
        let Some(method) = method.as_str() else {
            return Some(reply_err(id, &Rpc::invalid_request("`method` must be a string")));
        };
        let params = obj.get("params").unwrap_or(&Value::Null);
        let out = match method {
            "initialize" => self.initialize(params),
            "ping" => Ok(json!({})),
            "tools/list" => self.tools_list(params),
            "tools/call" => self.tools_call(params),
            other => Err(Rpc::new(
                METHOD_NOT_FOUND,
                format!("method not found: {other}"),
            )),
        };
        Some(match out {
            Ok(r) => reply_ok(id, r),
            Err(e) => reply_err(id, &e),
        })
    }

    fn initialize(&mut self, params: &Value) -> Result<Value, Rpc> {
        self.version = match params.get("protocolVersion") {
            None | Some(Value::Null) => McpVersion::LATEST,
            Some(Value::String(s)) => McpVersion::parse(s).unwrap_or(McpVersion::LATEST),
            Some(_) => return Err(Rpc::invalid_params("`protocolVersion` must be a string")),
        };
        let mut info = Map::new();
        info.insert("name".into(), json!("smesh"));
        info.insert("version".into(), json!(env!("CARGO_PKG_VERSION")));
        if self.version.has_structured_output() {
            info.insert("title".into(), json!("smart-mesh"));
        }
        Ok(json!({
            "protocolVersion": self.version.as_str(),
            "capabilities": {"tools": {"listChanged": false}},
            "serverInfo": info,
            "instructions": INSTRUCTIONS,
        }))
    }

    fn catalog(&mut self) -> Result<&[Tool], Rpc> {
        if self.tools.is_none() {
            let v = self.endpoint.call(Request::Catalog).map_err(lift)?;
            let catalog: CatalogResult = serde_json::from_value(v)
                .map_err(|e| Rpc::internal(format!("unreadable catalog: {e}")))?;
            self.tools = Some(tools::build(&catalog));
        }
        Ok(self.tools.as_deref().unwrap_or_default())
    }

    fn tools_list(&mut self, params: &Value) -> Result<Value, Rpc> {
        // The whole list is one page, so there is no valid cursor.
        match params.get("cursor") {
            None | Some(Value::Null) => {}
            Some(Value::String(s)) if s.is_empty() => {}
            Some(_) => return Err(Rpc::invalid_params("invalid cursor")),
        }
        let version = self.version;
        let list: Vec<Value> = self.catalog()?.iter().map(|t| t.to_json(version)).collect();
        Ok(json!({"tools": list}))
    }

    fn tools_call(&mut self, params: &Value) -> Result<Value, Rpc> {
        let Some(name) = params.get("name").and_then(Value::as_str) else {
            return Err(Rpc::invalid_params("`name` must be a string"));
        };
        let args = match params.get("arguments") {
            None | Some(Value::Null) => Map::new(),
            Some(Value::Object(m)) => m.clone(),
            Some(_) => return Err(Rpc::invalid_params("`arguments` must be an object")),
        };
        let target = match Fixed::from_name(name) {
            Some(f) => Target::Fixed(f),
            None => self
                .catalog()?
                .iter()
                .find(|t| t.name == name)
                .map(|t| t.target.clone())
                .ok_or_else(|| Rpc::invalid_params(format!("Unknown tool: {name}")))?,
        };
        match target {
            Target::Fixed(Fixed::Help) => self.help(args),
            Target::Fixed(f) => {
                let req = fixed_request(f, args)?;
                self.dispatch(req)
            }
            Target::Op(op) => self.dispatch(Request::Apply {
                op,
                params: Value::Object(args),
                version: None,
            }),
            Target::Query(name) => self.dispatch(Request::Query {
                name,
                input: Value::Object(args),
                at: None,
            }),
        }
    }

    /// Run `req`: success and application errors are tool results, everything else a protocol
    /// error.
    fn dispatch(&mut self, req: Request) -> Result<Value, Rpc> {
        match self.endpoint.call(req) {
            Ok(v) => Ok(self.tool_result(structured(v), false)),
            Err(e) if e.code == APPLICATION_ERROR => {
                let mut err = Map::new();
                err.insert("code".into(), json!(e.code));
                err.insert("kind".into(), json!(e.kind));
                err.insert("message".into(), json!(e.message));
                if !e.data.is_null() {
                    err.insert("data".into(), e.data);
                }
                Ok(self.tool_result(json!({"error": err}), true))
            }
            Err(e) => Err(lift(e)),
        }
    }

    /// A `tools/call` result: compact JSON text always, `structuredContent` from 2025-06-18.
    fn tool_result(&self, structured: Value, is_error: bool) -> Value {
        let shown: Cow<'_, Value> = display_form(&structured);
        let text = serde_json::to_string(shown.as_ref()).unwrap_or_default();
        let mut m = Map::new();
        m.insert("content".into(), json!([{"type": "text", "text": text}]));
        if self.version.has_structured_output() {
            m.insert("structuredContent".into(), structured);
        }
        m.insert("isError".into(), json!(is_error));
        Value::Object(m)
    }

    fn help(&mut self, args: Map<String, Value>) -> Result<Value, Rpc> {
        if let Some(k) = args.keys().find(|k| *k != "topic") {
            return Err(Rpc::invalid_params(format!("unknown argument `{k}`")));
        }
        let topic = match args.get("topic") {
            None | Some(Value::Null) => None,
            Some(Value::String(s)) => Some(s.clone()),
            Some(_) => return Err(Rpc::invalid_params("`topic` must be a string")),
        };
        let version = self.version;
        let tools = self.catalog()?;
        let Some(topic) = topic else {
            let list: Vec<Value> = tools
                .iter()
                .map(|t| {
                    let first = t.description.lines().next().unwrap_or_default();
                    json!({"name": t.name, "summary": first})
                })
                .collect();
            return Ok(self.tool_result(json!({"tools": list}), false));
        };
        let found = tools.iter().find(|t| {
            t.name == topic
                || matches!(&t.target, Target::Op(n) | Target::Query(n) if *n == topic)
        });
        match found {
            Some(t) => {
                let mut j = t.to_json(version);
                // `title` and `annotations` are protocol details; help shows the documentation.
                if let Value::Object(m) = &mut j {
                    m.remove("title");
                    m.remove("annotations");
                }
                let j = j;
                Ok(self.tool_result(j, false))
            }
            None => {
                let err = json!({"error": {"kind": "unknown-topic",
                    "message": format!("no tool, op or query named `{topic}`")}});
                Ok(self.tool_result(err, true))
            }
        }
    }
}

/// MCP: the id is a string or a number, never null.
fn id_is_valid(id: &Value) -> bool {
    matches!(id, Value::String(_) | Value::Number(_))
}

/// `structuredContent` must be an object: other results are wrapped as `{"result": value}`.
fn structured(v: Value) -> Value {
    if v.is_object() { v } else { json!({"result": v}) }
}

#[cfg(feature = "respell")]
fn display_form(v: &Value) -> Cow<'_, Value> {
    Cow::Owned(sm_lang::respell_json(v))
}

#[cfg(not(feature = "respell"))]
fn display_form(v: &Value) -> Cow<'_, Value> {
    Cow::Borrowed(v)
}

/// Turn a fixed tool call into its protocol request, validating the arguments with the
/// protocol's own strict parser (unknown fields are `invalid params`).
fn fixed_request(f: Fixed, mut args: Map<String, Value>) -> Result<Request, Rpc> {
    let Some(method) = f.method() else {
        return Err(Rpc::internal("tool has no protocol method"));
    };
    match f {
        Fixed::TimelineLog => {
            args.entry("limit").or_insert_with(|| json!(DEFAULT_LOG_LIMIT));
        }
        Fixed::TimelineCheckout => {
            if let Some(r) = args.get_mut("rev") {
                *r = normalize_rev(r.take());
            }
        }
        Fixed::SessionHash => {
            if let Some(r) = args.get_mut("at") {
                *r = normalize_rev(r.take());
            }
        }
        _ => {}
    }
    Envelope::from_value(json!({
        "jsonrpc": "2.0",
        "id": null,
        "method": method,
        "params": Value::Object(args),
    }))
    .map(|e| e.request)
    .map_err(|e| Rpc::invalid_params(e.message))
}

/// Revisions are written as plain strings in tool arguments: `"empty"`, a 64-hex node hash, or
/// a branch name. The object forms (`{"ref": ..}`, `{"node": ..}`) pass through unchanged and are
/// the only way to name a branch called `empty` or one whose name is 64 lowercase hex characters.
fn normalize_rev(v: Value) -> Value {
    match v {
        Value::String(s) if s == "empty" => json!("empty"),
        Value::String(s)
            if s.len() == 64 && s.bytes().all(|b| matches!(b, b'0'..=b'9' | b'a'..=b'f')) =>
        {
            json!({"node": s})
        }
        Value::String(s) => json!({"ref": s}),
        other => other,
    }
}
