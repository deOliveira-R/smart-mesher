use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::{METHODS, Request, RpcError};

/// A JSON-RPC 2.0 request: `{"jsonrpc":"2.0","id":..,"method":..,"params":..}`.
///
/// **Notification rule:** `id` is mandatory (string, number or `null`). A request without `id`
/// is a JSON-RPC notification, which this protocol does not support: parsing fails with
/// [`INVALID_REQUEST`](crate::INVALID_REQUEST) and the transport must still send that error as
/// a reply with `id: null` (see [`Endpoint::handle_text`]) rather than drop the message.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Envelope {
    /// Always `"2.0"`.
    pub jsonrpc: String,
    /// Caller's correlation id (string, number or null), echoed in the [`Response`].
    pub id: Value,
    /// Method and params.
    #[serde(flatten)]
    pub request: Request,
}

impl Envelope {
    /// Wrap a request with `jsonrpc: "2.0"`.
    pub fn new(id: Value, request: Request) -> Self {
        Self {
            jsonrpc: "2.0".into(),
            id,
            request,
        }
    }

    /// Parse one JSON text, mapping failures to the JSON-RPC errors: invalid JSON is
    /// [`PARSE_ERROR`](crate::PARSE_ERROR); a non-object, wrong `jsonrpc`, missing or
    /// ill-typed `id`, or missing `method` is [`INVALID_REQUEST`](crate::INVALID_REQUEST); an
    /// unknown method is [`METHOD_NOT_FOUND`](crate::METHOD_NOT_FOUND); params of the wrong
    /// shape (including unknown fields) are [`INVALID_PARAMS`](crate::INVALID_PARAMS).
    ///
    /// `id` must be a string, a number or `null`. **Notifications (requests without `id`) are
    /// not supported:** they are rejected as invalid requests and a transport must still send
    /// the error reply (with `id: null`) rather than stay silent.
    ///
    /// `params` is normalized before typing, as MCP and generic JSON-RPC clients always send it:
    /// absent, `null`, `{}` or `[]` mean "no params" for unit methods
    /// ([`Request::is_unit_method`]) and `{}` for every other method, so methods whose params are
    /// all optional (`session.hash`, `session.export`) work without a params object.
    pub fn parse(text: &str) -> Result<Self, RpcError> {
        let v: Value = serde_json::from_str(text).map_err(|e| RpcError::parse(e.to_string()))?;
        Self::from_value(v)
    }

    /// As [`Envelope::parse`], from an already parsed value.
    pub fn from_value(mut v: Value) -> Result<Self, RpcError> {
        let Value::Object(map) = &mut v else {
            return Err(RpcError::invalid_request("request must be a JSON object"));
        };
        if map.get("jsonrpc").and_then(Value::as_str) != Some("2.0") {
            return Err(RpcError::invalid_request("`jsonrpc` must be \"2.0\""));
        }
        match map.get("id") {
            None => {
                return Err(RpcError::invalid_request(
                    "missing `id` (notifications are not supported)",
                ));
            }
            Some(id) if !valid_id(id) => {
                return Err(RpcError::invalid_request(
                    "`id` must be a string, a number or null",
                ));
            }
            Some(_) => {}
        }
        let Some(name) = map.get("method").and_then(Value::as_str) else {
            return Err(RpcError::invalid_request("missing string `method`"));
        };
        let Some(&method) = METHODS.iter().find(|m| **m == name) else {
            return Err(RpcError::method_not_found(name));
        };
        let params = map.remove("params");
        let empty = match &params {
            None | Some(Value::Null) => true,
            Some(Value::Object(o)) => o.is_empty(),
            Some(Value::Array(a)) => a.is_empty(),
            Some(_) => false,
        };
        match params {
            // Non-empty params are kept as sent; typing decides (unit methods reject them).
            Some(p) if !empty => {
                map.insert("params".into(), p);
            }
            // Unit methods take no params; other methods see an empty object.
            _ if Request::is_unit_method(method) => {}
            _ => {
                map.insert("params".into(), Value::Object(Default::default()));
            }
        }
        serde_json::from_value(v).map_err(|e| RpcError::invalid_params(e.to_string()))
    }
}

/// JSON-RPC 2.0: an id is a string, a number or null.
pub(crate) fn valid_id(id: &Value) -> bool {
    matches!(id, Value::String(_) | Value::Number(_) | Value::Null)
}

/// A JSON-RPC 2.0 reply: exactly one of `result` and `error`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Response {
    /// Always `"2.0"`.
    pub jsonrpc: String,
    /// The request's id (`null` when it could not be read).
    pub id: Value,
    /// Success value.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub result: Option<Value>,
    /// Failure.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<RpcError>,
}

impl Response {
    /// A success reply.
    pub fn ok(id: Value, result: Value) -> Self {
        Self {
            jsonrpc: "2.0".into(),
            id,
            result: Some(result),
            error: None,
        }
    }
    /// A failure reply.
    pub fn err(id: Value, error: RpcError) -> Self {
        Self {
            jsonrpc: "2.0".into(),
            id,
            result: None,
            error: Some(error),
        }
    }
}

/// A server of the protocol. Single writer: calls take `&mut self` and run one at a time.
/// Front ends (`sm-mcp`, `sm-cli`) code against `dyn Endpoint`.
pub trait Endpoint {
    /// Execute one request. The `Ok` value is the method's result as JSON (see the variant docs
    /// of [`Request`]); errors carry the contract `code()` string in [`RpcError::kind`].
    fn call(&mut self, req: Request) -> Result<Value, RpcError>;

    /// Execute a whole envelope and build the reply.
    fn handle(&mut self, env: Envelope) -> Response {
        match self.call(env.request) {
            Ok(v) => Response::ok(env.id, v),
            Err(e) => Response::err(env.id, e),
        }
    }

    /// Parse one JSON-RPC text and execute it; protocol-level failures become error replies
    /// carrying the request's `id` when readable, else `null`.
    fn handle_text(&mut self, text: &str) -> Response {
        match Envelope::parse(text) {
            Ok(env) => self.handle(env),
            Err(e) => {
                let id = serde_json::from_str::<Value>(text)
                    .ok()
                    .and_then(|v| v.get("id").cloned())
                    .filter(valid_id)
                    .unwrap_or(Value::Null);
                Response::err(id, e)
            }
        }
    }
}
