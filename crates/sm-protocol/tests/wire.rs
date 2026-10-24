//! Wire tests: golden JSON per method, serde round trips, schema, errors, determinism.
#![allow(missing_docs)]

use std::collections::BTreeMap;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sm_core::{EntityId, EntityKind};
use sm_op_api::{Delta, Hash256, Head, OpError, OpInfoOwned, QueryCtx, QueryInfoOwned, Registry, Rev, State, TypedQuery};
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
            Request::Hash { at: None },
            json!({"method": "session.hash", "params": {}}),
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

/// Fixture names, parallel to [`cases`].
const NAMES: &[&str] = &[
    "catalog",
    "apply.full",
    "apply.minimal",
    "query.full",
    "query.minimal",
    "script.run",
    "timeline.status",
    "timeline.log",
    "timeline.checkout",
    "timeline.branch",
    "timeline.undo",
    "timeline.redo",
    "session.hash.at_empty",
    "session.hash.default",
    "session.export.full",
    "session.export.minimal",
    "session.save",
    "session.open",
];

fn golden_path(name: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("golden")
        .join(name)
}

fn bless() -> bool {
    std::env::var_os("SM_PROTOCOL_BLESS").is_some()
}

/// Compare `actual` with the shipped fixture text (parsed), or rewrite the file when blessing.
fn check_fixture(file: &str, shipped: &str, actual: &Value) {
    if bless() {
        let text = serde_json::to_string_pretty(actual).unwrap() + "\n";
        std::fs::write(golden_path(file), text).unwrap();
        return;
    }
    let want: Value = serde_json::from_str(shipped).unwrap();
    assert_eq!(&want, actual, "fixture {file} is stale (SM_PROTOCOL_BLESS=1 regenerates)");
}

#[test]
fn shipped_request_fixtures_match() {
    let cs = cases();
    assert_eq!(NAMES.len(), cs.len());
    let map: serde_json::Map<String, Value> = NAMES
        .iter()
        .zip(cs)
        .map(|(n, (_, g))| ((*n).to_string(), g))
        .collect();
    check_fixture("requests.json", fixtures::REQUESTS, &Value::Object(map));
}

#[test]
fn lenient_params_parse_to_canonical_requests() {
    let table: Vec<Value> = serde_json::from_str(fixtures::LENIENT).unwrap();
    assert!(table.len() >= 20);
    for case in table {
        let env = Envelope::from_value(case["input"].clone())
            .unwrap_or_else(|e| panic!("{}: {e}", case["input"]));
        let want: Request = serde_json::from_value(case["request"].clone()).unwrap();
        assert_eq!(env.request, want, "{}", case["input"]);
    }
}

#[test]
fn lenient_covers_every_unit_method_and_all_optional_method() {
    let table: Vec<Value> = serde_json::from_str(fixtures::LENIENT).unwrap();
    for m in METHODS.iter().filter(|m| Request::is_unit_method(m)) {
        for shape in ["absent", "null", "{}", "[]"] {
            assert!(
                table.iter().any(|c| c["input"]["method"] == *m && c["shape"] == shape),
                "{m} {shape}"
            );
        }
    }
    for m in ["session.hash", "session.export"] {
        assert!(table.iter().any(|c| c["input"]["method"] == m && c["shape"] == "absent"));
    }
}

#[test]
fn rejected_inputs_map_to_codes() {
    let table: Vec<Value> = serde_json::from_str(fixtures::REJECTED).unwrap();
    for case in table {
        let text = case["input"].as_str().unwrap();
        let e = Envelope::parse(text).unwrap_err();
        assert_eq!(json!(e.code), case["code"], "{text}");
        assert_eq!(json!(e.kind), case["kind"], "{text}");
    }
}

#[test]
fn unit_methods_still_reject_real_params() {
    let e = Envelope::parse(r#"{"jsonrpc":"2.0","id":1,"method":"catalog","params":{"x":1}}"#)
        .unwrap_err();
    assert_eq!(e.code, INVALID_PARAMS);
    let e = Envelope::parse(r#"{"jsonrpc":"2.0","id":1,"method":"session.save","params":[1]}"#)
        .unwrap_err();
    assert_eq!(e.code, INVALID_PARAMS);
}

#[test]
fn non_notification_id_types() {
    for id in ["1", "\"a\"", "null", "1.5"] {
        let t = format!(r#"{{"jsonrpc":"2.0","id":{id},"method":"catalog"}}"#);
        assert!(Envelope::parse(&t).is_ok(), "{t}");
    }
    for id in [r#"{"a":1}"#, "[1]", "true"] {
        let t = format!(r#"{{"jsonrpc":"2.0","id":{id},"method":"catalog"}}"#);
        let e = Envelope::parse(&t).unwrap_err();
        assert_eq!(e.code, INVALID_REQUEST, "{t}");
        // The reply never echoes an ill-typed id.
        assert_eq!(Fake.handle_text(&t).id, Value::Null);
    }
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

fn rt<T: Serialize + for<'a> Deserialize<'a> + PartialEq + std::fmt::Debug>(
    m: &mut serde_json::Map<String, Value>,
    name: &str,
    t: &T,
    golden: Value,
) {
    m.insert(name.into(), golden.clone());
    let v = serde_json::to_value(t).unwrap();
    assert_eq!(v, golden);
    assert_eq!(&serde_json::from_value::<T>(golden).unwrap(), t);
}

#[test]
fn result_golden_and_round_trip() {
    let mut m = serde_json::Map::new();
    let mut delta = Delta::default();
    delta.created.push(eid(1));
    delta.outputs.insert("volume".into(), vec![eid(1)]);
    delta.touched_layers.push("model".into());
    let a = ApplyResult::new(h(1), h(2), delta);
    let dv = serde_json::to_value(&a.delta).unwrap();
    rt(&mut m, "apply", &a, json!({"node": hs(1), "state": hs(2), "delta": dv}));
    assert_eq!(dv["created"], json!(["TopoVolume#1"]));

    rt(&mut m, "hash", &HashResult::new(
            h(4),
            BTreeMap::from([("model".into(), h(5)), ("a".into(), h(6))]),
        ),
        json!({"state": hs(4), "layers": {"a": hs(6), "model": hs(5)}}),
    );

    rt(&mut m, "export.path", &ExportResult::new(Some("o.smsh".into()), ExportFormat::Smsh, 12, h(7)),
        json!({"path": "o.smsh", "format": "smsh", "bytes": 12, "sha256": hs(7)}),
    );
    rt(&mut m, "export.inline", &ExportResult::new(None, ExportFormat::Vtu, 3, h(8)).with_base64("AAEC"),
        json!({"path": null, "format": "vtu", "bytes": 3, "sha256": hs(8), "base64": "AAEC"}),
    );

    rt(&mut m, "script", &ScriptResult::new(h(9), "ok\n", vec![json!({"cmd": "brick"})], json!([])),
        json!({"state": hs(9), "output": "ok\n", "events": [{"cmd": "brick"}], "diagnostics": []}),
    );

    rt(&mut m, "status.attached", &StatusResult::new(Head::Ref("main".into()), h(2))
            .with_position(Some(h(1)), 2)
            .with_branches(3, BTreeMap::from([("main".into(), h(1))]))
            .with_redo(1)
            .with_dirty(true)
            .with_project(Some("demo".into())),
        json!({"head": {"ref": "main"}, "position": hs(1), "depth": 2, "state": hs(2), "nodes": 3,
               "branches": {"main": hs(1)}, "redo": 1, "dirty": true, "project": "demo"}),
    );
    rt(&mut m, "status.empty", &StatusResult::new(Head::Detached(None), h(0)),
        json!({"head": "empty", "position": null, "depth": 0, "state": hs(0), "nodes": 0,
               "branches": {}, "redo": 0, "dirty": false, "project": null}),
    );
    check_fixture("results.json", fixtures::RESULTS, &Value::Object(m));
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
    let ops: Vec<_> = reg
        .catalog()
        .into_iter()
        .map(|i| OpInfoOwned::from(&i.unwrap()))
        .collect();
    let queries: Vec<_> = reg
        .query_catalog()
        .into_iter()
        .map(|q| QueryInfoOwned::from(&q.unwrap()))
        .collect();
    let batch_schema = serde_json::to_value(reg.batch_schema().unwrap()).unwrap();
    let c = CatalogResult {
        protocol: PROTOCOL_VERSION,
        ops,
        queries,
        batch_schema,
    };
    let v = serde_json::to_value(&c).unwrap();
    check_fixture("catalog.json", fixtures::CATALOG, &v);
    // Typed round trip, and the same JSON as serializing the borrowed infos.
    let back: CatalogResult = serde_json::from_value(v.clone()).unwrap();
    assert_eq!(serde_json::to_value(&back).unwrap(), v);
    assert_eq!(v["ops"][0]["tool_name"], json!("core__batch"));
    assert_eq!(v["queries"][0]["tool_name"], json!("demo__count"));
    let direct: Vec<Value> = reg
        .catalog()
        .iter()
        .map(|i| serde_json::to_value(i.as_ref().unwrap()).unwrap())
        .collect();
    assert_eq!(v["ops"], Value::Array(direct));
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
