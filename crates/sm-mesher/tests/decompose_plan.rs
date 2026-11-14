//! `mesh.decompose_plan` against the real stack (W4-HEX-QUERY): same steps as the planner, no state change,
//! and the steps applied by script and by raw ops reach the same state hash; `hex_only` quotes the cuts.

use serde_json::{Value, json};
use sm_core::EntityId;
use sm_decomp::{CutKind, CutStep, Target, plan_cuts};
use sm_layer_api::ModelLayer;
use sm_protocol::{Endpoint, Request, RpcError};
use sm_server::{Server, standard_with};
use sm_topo_api::Dim;

fn server() -> Server {
    Server::new(standard_with(
        Some("tester".into()),
        Some("2026-11-09T00:00:00Z".into()),
    ))
    .expect("server")
}

fn script(s: &mut Server, text: &str) -> Result<Value, RpcError> {
    s.call(Request::ScriptRun {
        text: text.into(),
        path: None,
        includes: Default::default(),
    })
}

const PIN: &str = "let br = model.create_brick(x: 1.26, y: 1.26, z: 1.0)\nlet fu = model.create_cylinder(height: 0.8, radius: 0.4)\nlet tl = model.create_cylinder(height: 0.8, radius: 0.4)\nlet mo = model.subtract(targets: br.body, tools: tl.body)\nmodel.imprint(bodies: all bodies)\nmodel.merge()\n";

fn pin() -> Server {
    let mut s = server();
    script(&mut s, PIN).expect("pin");
    s
}

fn volumes(s: &Server) -> Vec<EntityId> {
    let st = s.workspace().state();
    let l = st.layer::<ModelLayer>().expect("model");
    l.topo().cells(Dim::Volume).to_vec()
}

fn query(s: &mut Server, input: Value) -> Value {
    s.call(Request::Query {
        name: "mesh.decompose_plan".into(),
        input,
        at: None,
    })
    .expect("query")
}

/// The host row (the one volume with cuts).
fn host(report: &Value) -> &Value {
    let rows: Vec<&Value> = report["volumes"]
        .as_array()
        .expect("rows")
        .iter()
        .filter(|r| r["status"] == "cuts")
        .collect();
    assert_eq!(rows.len(), 1, "{report}");
    rows[0]
}

#[test]
fn query_returns_the_planner_steps_and_leaves_the_state_alone() {
    let mut s = pin();
    let before = s.workspace().state_hash();
    let r = query(&mut s, json!({}));
    assert_eq!(s.workspace().state_hash(), before, "a query must not move the hash");
    // One row per volume, ascending.
    let ids: Vec<String> = r["volumes"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v["volume"].as_str().unwrap().to_owned())
        .collect();
    let mut want: Vec<EntityId> = volumes(&s);
    want.sort_unstable();
    assert_eq!(ids, want.iter().map(ToString::to_string).collect::<Vec<_>>());
    // The host row equals plan_cuts on the live topology.
    let h = host(&r);
    let host_id: EntityId = h["volume"].as_str().unwrap().parse().expect("id");
    let expect: Vec<CutStep> = {
        let st = s.workspace().state();
        let l = st.layer::<ModelLayer>().unwrap();
        plan_cuts(l.topo(), host_id).expect("plan")
    };
    assert_eq!(expect.len(), 3, "cylinder plus two planes");
    let got: Vec<CutStep> = serde_json::from_value(h["steps"].clone()).expect("steps");
    assert_eq!(got, expect);
    // Every other volume needs no cuts.
    for row in r["volumes"].as_array().unwrap() {
        if row["volume"] != h["volume"] {
            assert_eq!(row["status"], "no_cuts", "{row}");
            assert!(row["steps"].as_array().unwrap().is_empty());
        }
    }
    // `volumes` filters; an unknown id is an error, not a silent skip.
    let one = query(&mut s, json!({"volumes": [h["volume"]]}));
    assert_eq!(one["volumes"].as_array().unwrap().len(), 1);
    let bad = s.call(Request::Query {
        name: "mesh.decompose_plan".into(),
        input: json!({"volumes": ["TopoFace#1"]}),
        at: None,
    });
    assert!(bad.is_err(), "a face id is not a volume");
    // Identical on a second call.
    assert_eq!(query(&mut s, json!({})), r);
}

/// Apply the steps as raw ops (what MCP and Python do), resolving step references from each op's outputs.
fn apply_raw(s: &mut Server, host: EntityId, steps: &[CutStep]) {
    let mut outs: Vec<Value> = Vec::new();
    for st in steps {
        let targets: Vec<Value> = st
            .targets
            .iter()
            .flat_map(|t| match t {
                Target::Id { .. } => vec![json!(host.to_string())],
                Target::Step { step, role } => outs[*step][role.as_str()].as_array().expect("role").clone(),
            })
            .collect();
        let (op, mut params) = match &st.cut {
            CutKind::Cylinder { radius, axis, center } => (
                "model.webcut_cylinder",
                json!({"radius": radius, "axis": axis, "center": center}),
            ),
            CutKind::Plane { plane } => (
                "model.webcut_plane",
                json!({"plane": {"origin": plane.origin, "normal": plane.normal}}),
            ),
        };
        params["targets"] = json!(targets);
        let r = s
            .call(Request::Apply {
                op: op.into(),
                params,
                version: None,
            })
            .unwrap_or_else(|e| panic!("{op}: {e}"));
        outs.push(r["delta"]["outputs"].clone());
    }
}

#[test]
fn applying_the_steps_by_script_and_by_raw_ops_gives_equal_hashes() {
    let mut a = pin();
    let r = query(&mut a, json!({}));
    let h = host(&r).clone();
    let host_id: EntityId = h["volume"].as_str().unwrap().parse().unwrap();
    let steps: Vec<CutStep> = serde_json::from_value(h["steps"].clone()).unwrap();
    // The script text the query returns, then imprint and merge.
    let text = format!(
        "{}model.imprint(bodies: all bodies)\nmodel.merge()\n",
        h["script"].as_str().unwrap()
    );
    script(&mut a, &text).expect("script");
    let mut b = pin();
    apply_raw(&mut b, host_id, &steps);
    script(&mut b, "model.imprint(bodies: all bodies)\nmodel.merge()\n").expect("imprint");
    assert_eq!(a.workspace().state_hash(), b.workspace().state_hash());
    assert_eq!(volumes(&a).len(), 6, "three slabs, two caps, the fuel");
    // After the cuts nothing needs cutting.
    let r2 = query(&mut a, json!({}));
    assert!(r2["volumes"].as_array().unwrap().iter().all(|v| v["status"] == "no_cuts"));
}

#[test]
fn hex_only_failure_quotes_the_suggested_cuts() {
    let mut s = pin();
    let r = query(&mut s, json!({}));
    let h = host(&r).clone();
    script(&mut s, "mesh.size(targets: all volumes, size: 0.3)\n").expect("size");
    let before = s.workspace().state_hash();
    let err = script(&mut s, "let au = mesh.auto(strategy: \"hex_only\")\n")
        .expect_err("an enclosed pin has no hex-only mesh");
    let text = format!("{err} {}", err.data["rendered"].as_str().unwrap_or(""));
    let lines: Vec<&str> = h["script"].as_str().unwrap().lines().collect();
    // Error text prints ids without the `Topo` prefix.
    let want = format!("cannot be swept; suggested cuts: {}", lines.join("; ")).replace("TopoVolume#", "Volume#");
    assert!(text.contains(&want), "WANT {want}\nGOT {text}");
    assert!(text.contains("mesh.decompose_plan"), "{text}");
    assert_eq!(s.workspace().state_hash(), before, "the rejected op leaves the state alone");
}
