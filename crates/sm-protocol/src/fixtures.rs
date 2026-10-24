//! Wire fixtures shared by `sm-protocol`, `sm-mcp`, `sm-cli` and `sm-server` tests, so every
//! front end is checked against the same bytes. Each constant is JSON text embedded from
//! `crates/sm-protocol/golden/`; parse it with `serde_json`.

/// Object `name -> request`: the wire form (`{"method", "params"?}`) of each sample request.
pub const REQUESTS: &str = include_str!("../golden/requests.json");

/// Object `name -> result`: the JSON of each result struct.
pub const RESULTS: &str = include_str!("../golden/results.json");

/// Exact `catalog` result of a registry holding `core.batch` and the query `demo.count`.
pub const CATALOG: &str = include_str!("../golden/catalog.json");

/// Array of `{"input": <full JSON-RPC envelope as a client may send it>, "request":
/// <canonical wire request it must parse to>}`: lenient-params cases.
pub const LENIENT: &str = include_str!("../golden/lenient.json");

/// Array of `{"input": <envelope or raw text>, "code": <JSON-RPC code>, "kind": <string>}`:
/// inputs that must be rejected.
pub const REJECTED: &str = include_str!("../golden/rejected.json");
