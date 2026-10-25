//! Determinism: the same request log gives byte-identical replies with a pinned clock and author,
//! under 1 and N threads and concurrent runs; and a cold replay from the root agrees.
mod common;

use common::*;
use serde_json::Value;
use sm_op_api::{Hash256, Rev};
use sm_protocol::{Endpoint, Request};
use sm_server::Server;
use sm_testkit::determinism::check;

/// A request log that exercises scripts, apply, forks, undo/redo, queries, export and hashing.
fn log() -> Vec<String> {
    let r = |id: u32, method: &str, params: Value| serde_json::json!({"jsonrpc": "2.0", "id": id, "method": method, "params": params}).to_string();
    vec![
        r(1, "apply", serde_json::json!({"op": "demo.add_item", "params": {"name": "a"}})),
        r(2, "script.run", serde_json::json!({"text": "let b = demo.add_item(\"b\")\ndemo.rename(item: b.item[0], name: \"c\")\nprint \"ok\"\n"})),
        r(3, "apply", serde_json::json!({"op": "demo.add_hex", "params": {"n": 3}})),
        r(4, "session.export", serde_json::json!({"format": "smsh", "inline": true})),
        r(5, "session.export", serde_json::json!({"format": "smsh", "inline": true, "provenance": true})),
        r(6, "timeline.undo", serde_json::json!({})),
        r(7, "timeline.redo", serde_json::json!({})),
        r(8, "timeline.log", serde_json::json!({"limit": 50})),
        r(9, "query", serde_json::json!({"name": "demo.count"})),
        r(10, "catalog", serde_json::json!({})),
        r(11, "session.hash", serde_json::json!({})),
        r(12, "timeline.status", serde_json::json!({})),
        r(13, "session.export", serde_json::json!({"format": "vtu", "inline": true})),
    ]
}

fn run(threads: usize) -> Vec<String> {
    let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool");
    pool.install(|| {
        let mut s = Server::new(demo_builder()).expect("server");
        log().iter().map(|l| serde_json::to_string(&s.handle_text(l)).expect("reply")).collect()
    })
}

#[test]
fn same_request_log_same_responses_for_1_and_n_threads() {
    let replies = check(2, 4, run).expect("deterministic");
    assert_eq!(replies.len(), log().len());
    assert!(replies.iter().all(|r| !r.contains("\"error\"")), "{replies:#?}");
}

#[test]
fn two_servers_agree_on_every_node_and_state() {
    let go = || {
        let mut s = Server::new(demo_builder()).expect("server");
        for l in log() {
            s.handle_text(&l);
        }
        let st = s.call(Request::Log { from: None, limit: 100 }).expect("log");
        (st.to_string(), s.workspace().state_hash())
    };
    assert_eq!(go(), go());
}

#[test]
fn cold_replay_from_the_root_does_not_diverge() {
    let mut s = Server::new(demo_builder()).expect("server");
    for l in log() {
        s.handle_text(&l);
    }
    let live = s.workspace().state_hash();
    let head: Hash256 = s.workspace().status().position.expect("position");
    // Replay from the root with no cached state: strict policy, so a divergence is an error.
    let cold = s.workspace().state_from_root(head).expect("no Diverged");
    assert_eq!(cold.hash(), live);
    assert_eq!(cold.layer_hashes(), s.workspace().state().layer_hashes());
    // And through a fresh workspace given the same recorded ops.
    let mut ws = s.into_workspace();
    assert_eq!(ws.state_at(&Rev::Node(head)).expect("state_at").hash(), live);
}
