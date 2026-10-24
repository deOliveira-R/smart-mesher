//! Wire tests: golden JSON per method, serde round trips, schema, errors, determinism.
#![allow(missing_docs)]

use std::collections::BTreeMap;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sm_core::{EntityId, EntityKind};
use sm_op_api::{Delta, Hash256, Head, OpError, QueryCtx, Registry, Rev, State, TypedQuery};
use sm_protocol::*;

fn h(n: u8) -> Hash256 {
    Hash256([n; 32])
}
fn hs(n: u8) -> String {
    h(n).to_string()
}
fn eid(n: u64) -> EntityId {
    EntityId::new(EntityKind::TopoVolume, n).unwrap()
}

/// Request, expected `params`-bearing wire JSON of the request part.
fn cases() -> Vec<(Request, Value)> {
    vec![
        (Request::Catalog, json!({"method": "catalog"})),
        (
            Request::Apply {
                op: "model.create_brick".into(),
                params: json!({"size": [1, 2, 3]}),
                version: Some(1),
            },
            json!({"method": "apply", "params": {"op": "model.create_brick", "params": {"size": [1, 2, 3]}, "version": 1}}),
        ),
        (
            Request::Apply {
                op: "model.delete".into(),
                params: json!({}),
                version: None,
            },
            json!({"method": "apply", "params": {"op": "model.delete", "params": {}}}),
        ),
        (
            Request::Query {
                name: "model.volumes".into(),
                input: json!({"k": 1}),
                at: Some(Rev::Ref("main".into())),
            },
            json!({"method": "query", "params": {"name": "model.volumes", "input": {"k": 1}, "at": {"ref": "main"}}}),
        ),
        (
            Request::Query {
                name: "model.volumes".into(),
                input: Value::Null,
                at: None,
            },
            json!({"method": "query", "params": {"name": "model.volumes"}}),
        ),
        (
            Request::ScriptRun {
                text: "brick x 1".into(),
                path: Some("a.sm".into()),
                includes: BTreeMap::from([("b.sm".to_string(), "mesh all".to_string())]),
            },
            json!({"method": "script.run", "params": {"text": "brick x 1", "path": "a.sm", "includes": {"b.sm": "mesh all"}}}),
        ),
        (Request::Status, json!({"method": "timeline.status"})),
        (
            Request::Log {
                from: Some(h(1)),
                limit: 10,
            },
            json!({"method": "timeline.log", "params": {"from": hs(1), "limit": 10}}),
        ),
        (
            Request::Checkout {
                rev: Rev::Node(h(2)),
            },
            json!({"method": "timeline.checkout", "params": {"rev": {"node": hs(2)}}}),
        ),
        (
            Request::Branch {
                name: "alt".into(),
                at: Some(h(3)),
            },
            json!({"method": "timeline.branch", "params": {"name": "alt", "at": hs(3)}}),
        ),
        (Request::Undo, json!({"method": "timeline.undo"})),
        (Request::Redo, json!({"method": "timeline.redo"})),
        (
            Request::Hash {
                at: Some(Rev::Empty),
            },
            json!({"method": "session.hash", "params": {"at": "empty"}}),
        ),
        (
            Request::Export {
                path: Some("out.vtu".into()),
                format: Some(ExportFormat::Vtu),
                mesh: Some(eid(7)),
                provenance: true,
                inline: true,
            },
            json!({"method": "session.export", "params": {"path": "out.vtu", "format": "vtu", "mesh": "TopoVolume#7", "provenance": true, "inline": true}}),
        ),
        (
            Request::Export {
                path: None,
                format: Some(ExportFormat::Smsh),
                mesh: None,
                provenance: false,
                inline: false,
            },
            json!({"method": "session.export", "params": {"format": "smsh"}}),
        ),
        (Request::Save, json!({"method": "session.save"})),
        (
            Request::Open {
                path: "proj".into(),
                create: true,
            },
            json!({"method": "session.open", "params": {"path": "proj", "create": true}}),
        ),
    ]
}

#[test]
fn request_golden_and_round_trip() {
    for (req, golden) in cases() {
        let v = serde_json::to_value(&req).unwrap();
        assert_eq!(v, golden, "golden for {}", req.method());
        let back: Request = serde_json::from_value(golden).unwrap();
        assert_eq!(back, req);
    }
}

#[test]
fn every_method_is_covered_and_listed() {
    let mut seen: Vec<&str> = cases().iter().map(|(r, _)| r.method()).collect();
    seen.sort();
    seen.dedup();
    assert_eq!(seen, METHODS, "METHODS is sorted and matches the variants");
    for w in METHODS.windows(2) {
        assert!(w[0] < w[1]);
    }
}

#[test]
fn optional_fields_accept_missing_and_null() {
    let r: Request = serde_json::from_value(
        json!({"method": "session.export", "params": {"path": null, "format": null, "mesh": null}}),
    )
    .unwrap();
    assert_eq!(
        r,
        Request::Export {
            path: None,
            format: None,
            mesh: None,
            provenance: false,
            inline: false
        }
    );
    let r: Request =
        serde_json::from_value(json!({"method": "timeline.log", "params": {"limit": 3}})).unwrap();
    assert_eq!(
        r,
        Request::Log {
            from: None,
            limit: 3
        }
    );
}

#[test]
fn envelope_golden_and_round_trip() {
    let env = Envelope::new(
        json!(7),
        Request::Apply {
            op: "a.b".into(),
            params: json!({"x": 1}),
            version: None,
        },
    );
    let v = serde_json::to_value(&env).unwrap();
    assert_eq!(
        v,
        json!({"jsonrpc": "2.0", "id": 7, "method": "apply", "params": {"op": "a.b", "params": {"x": 1}}})
    );
    assert_eq!(Envelope::from_value(v).unwrap(), env);

    let unit = Envelope::new(json!("s"), Request::Undo);
    let v = serde_json::to_value(&unit).unwrap();
    assert_eq!(
        v,
        json!({"jsonrpc": "2.0", "id": "s", "method": "timeline.undo"})
    );
    assert_eq!(Envelope::from_value(v).unwrap(), unit);
    // Explicit null params on a unit method also parses.
    let e =
        Envelope::parse(r#"{"jsonrpc":"2.0","id":1,"method":"catalog","params":null}"#).unwrap();
    assert_eq!(e.request, Request::Catalog);
}

#[test]
fn every_request_round_trips_through_envelope_text() {
    for (i, (req, _)) in cases().into_iter().enumerate() {
        let env = Envelope::new(json!(i), req);
        let text = serde_json::to_string(&env).unwrap();
        assert_eq!(Envelope::parse(&text).unwrap(), env);
    }
}

#[test]
fn protocol_errors_map_to_json_rpc_codes() {
    let e = Envelope::parse("{not json").unwrap_err();
    assert_eq!((e.code, e.kind.as_str()), (PARSE_ERROR, "parse-error"));

    for bad in [
        "[1]",
        r#"{"id":1,"method":"catalog"}"#,
        r#"{"jsonrpc":"1.0","id":1,"method":"catalog"}"#,
        r#"{"jsonrpc":"2.0","method":"catalog"}"#,
        r#"{"jsonrpc":"2.0","id":1}"#,
        r#"{"jsonrpc":"2.0","id":1,"method":5}"#,
    ] {
        let e = Envelope::parse(bad).unwrap_err();
        assert_eq!(
            (e.code, e.kind.as_str()),
            (INVALID_REQUEST, "invalid-request"),
            "{bad}"
        );
    }

    let e = Envelope::parse(r#"{"jsonrpc":"2.0","id":1,"method":"nope.nothing"}"#).unwrap_err();
    assert_eq!(
        (e.code, e.kind.as_str()),
        (METHOD_NOT_FOUND, "method-not-found")
    );
    assert!(e.message.contains("nope.nothing"));

    for bad in [
        r#"{"jsonrpc":"2.0","id":1,"method":"timeline.log","params":{}}"#,
        r#"{"jsonrpc":"2.0","id":1,"method":"timeline.checkout","params":{"rev":42}}"#,
        r#"{"jsonrpc":"2.0","id":1,"method":"session.open"}"#,
    ] {
        let e = Envelope::parse(bad).unwrap_err();
        assert_eq!(
            (e.code, e.kind.as_str()),
            (INVALID_PARAMS, "invalid-params"),
            "{bad}"
        );
    }

    assert!((-32099..=-32000).contains(&APPLICATION_ERROR));
    assert_eq!(
        RpcError::application("op-invalid", "m").code,
        APPLICATION_ERROR
    );
    assert_eq!(RpcError::internal("x").code, INTERNAL_ERROR);
}

struct Fake;
impl Endpoint for Fake {
    fn call(&mut self, req: Request) -> Result<Value, RpcError> {
        match req {
            Request::Undo => Err(RpcError::application("nothing-to-undo", "at root")),
            r => Ok(json!({"echo": r.method()})),
        }
    }
}

#[test]
fn endpoint_handle_text_builds_responses() {
    let mut ep = Fake;
    let ok = ep.handle_text(r#"{"jsonrpc":"2.0","id":3,"method":"timeline.status"}"#);
    assert_eq!(
        serde_json::to_value(&ok).unwrap(),
        json!({"jsonrpc": "2.0", "id": 3, "result": {"echo": "timeline.status"}})
    );
    let err = ep.handle_text(r#"{"jsonrpc":"2.0","id":4,"method":"timeline.undo"}"#);
    assert_eq!(
        serde_json::to_value(&err).unwrap(),
        json!({"jsonrpc": "2.0", "id": 4, "error": {"code": -32000, "kind": "nothing-to-undo", "message": "at root"}})
    );
    // Unknown method keeps the caller's id.
    let nf = ep.handle_text(r#"{"jsonrpc":"2.0","id":"q","method":"zzz"}"#);
    assert_eq!(nf.id, json!("q"));
    assert_eq!(nf.error.unwrap().code, METHOD_NOT_FOUND);
    let pe = ep.handle_text("}");
    assert_eq!(pe.id, Value::Null);
    let back: Response = serde_json::from_value(serde_json::to_value(&err).unwrap()).unwrap();
    assert_eq!(back, err);
}

#[test]
fn rpc_error_data_round_trip() {
    let e = RpcError::application("op-invalid", "bad").with_data(json!({"at": "x"}));
    let v = serde_json::to_value(&e).unwrap();
    assert_eq!(v["data"], json!({"at": "x"}));
    assert_eq!(serde_json::from_value::<RpcError>(v).unwrap(), e);
    assert!(e.to_string().contains("op-invalid"));
}

fn round_trip<T: Serialize + for<'a> Deserialize<'a> + PartialEq + std::fmt::Debug>(
    t: &T,
    golden: Value,
) {
    let v = serde_json::to_value(t).unwrap();
    assert_eq!(v, golden);
    assert_eq!(&serde_json::from_value::<T>(golden).unwrap(), t);
}

#[test]
fn result_golden_and_round_trip() {
    let mut delta = Delta::default();
    delta.created.push(eid(1));
    delta.outputs.insert("volume".into(), vec![eid(1)]);
    delta.touched_layers.push("model".into());
    let a = ApplyResult {
        node: h(1),
        state: h(2),
        delta,
    };
    let dv = serde_json::to_value(&a.delta).unwrap();
    round_trip(&a, json!({"node": hs(1), "state": hs(2), "delta": dv}));
    assert_eq!(dv["created"], json!(["TopoVolume#1"]));

    round_trip(
        &HashResult {
            state: h(4),
            layers: BTreeMap::from([("model".into(), h(5)), ("a".into(), h(6))]),
        },
        json!({"state": hs(4), "layers": {"a": hs(6), "model": hs(5)}}),
    );

    round_trip(
        &ExportResult {
            path: Some("o.smsh".into()),
            format: ExportFormat::Smsh,
            bytes: 12,
            sha256: h(7),
            base64: None,
        },
        json!({"path": "o.smsh", "format": "smsh", "bytes": 12, "sha256": hs(7)}),
    );
    round_trip(
        &ExportResult {
            path: None,
            format: ExportFormat::Vtu,
            bytes: 3,
            sha256: h(8),
            base64: Some("AAEC".into()),
        },
        json!({"path": null, "format": "vtu", "bytes": 3, "sha256": hs(8), "base64": "AAEC"}),
    );

    round_trip(
        &ScriptResult {
            state: h(9),
            output: "ok\n".into(),
            events: vec![json!({"cmd": "brick"})],
            diagnostics: json!([]),
        },
        json!({"state": hs(9), "output": "ok\n", "events": [{"cmd": "brick"}], "diagnostics": []}),
    );

    round_trip(
        &StatusResult {
            head: Head::Ref("main".into()),
            position: Some(h(1)),
            depth: 2,
            state: h(2),
            nodes: 3,
            branches: BTreeMap::from([("main".into(), h(1))]),
            redo: 1,
            dirty: true,
            project: Some("demo".into()),
        },
        json!({"head": {"ref": "main"}, "position": hs(1), "depth": 2, "state": hs(2), "nodes": 3,
               "branches": {"main": hs(1)}, "redo": 1, "dirty": true, "project": "demo"}),
    );
    round_trip(
        &StatusResult {
            head: Head::Detached(None),
            position: None,
            depth: 0,
            state: h(0),
            nodes: 0,
            branches: BTreeMap::new(),
            redo: 0,
            dirty: false,
            project: None,
        },
        json!({"head": "empty", "position": null, "depth": 0, "state": hs(0), "nodes": 0,
               "branches": {}, "redo": 0, "dirty": false, "project": null}),
    );
}

#[derive(Deserialize, JsonSchema)]
struct CountQ {}
impl TypedQuery for CountQ {
    const NAME: &'static str = "demo.count";
    type Output = u32;
    fn run(&self, _: &State, _: &QueryCtx<'_>) -> Result<u32, OpError> {
        Ok(0)
    }
}

#[test]
fn catalog_result_serializes_registry_infos() {
    let mut reg = Registry::new();
    reg.register_query::<CountQ>().unwrap();
    let ops: Vec<_> = reg.catalog().into_iter().map(Result::unwrap).collect();
    let queries: Vec<_> = reg
        .query_catalog()
        .into_iter()
        .map(Result::unwrap)
        .collect();
    let batch_schema = serde_json::to_value(reg.batch_schema().unwrap()).unwrap();
    let c = CatalogResult {
        protocol: PROTOCOL_VERSION,
        ops,
        queries,
        batch_schema,
    };
    let v = serde_json::to_value(&c).unwrap();
    assert_eq!(v["protocol"], json!(0));
    assert_eq!(v["ops"][0]["name"], json!("core.batch"));
    assert_eq!(v["queries"][0]["name"], json!("demo.count"));
    assert_eq!(v["batch_schema"]["title"], json!("core.batch"));
    assert!(v["ops"][0]["schema"].is_object());
    assert_eq!(PROTOCOL_VERSION, 0);
}

#[test]
fn request_schema_lists_every_method() {
    let s = Request::json_schema();
    assert!(s.is_object());
    let text = serde_json::to_string(&s).unwrap();
    for m in METHODS {
        assert!(text.contains(&format!("\"{m}\"")), "schema mentions {m}");
    }
    assert!(text.contains("\"method\"") && text.contains("\"params\""));
}

#[test]
fn deterministic_encoding() {
    let enc = || {
        let v: Vec<Value> = cases()
            .iter()
            .map(|(r, _)| serde_json::to_value(r).unwrap())
            .collect();
        Hash256::of(serde_json::to_string(&v).unwrap().as_bytes())
    };
    assert_eq!(enc(), enc());
    let schema = || {
        Hash256::of(
            serde_json::to_string(&Request::json_schema())
                .unwrap()
                .as_bytes(),
        )
    };
    assert_eq!(schema(), schema());
}
