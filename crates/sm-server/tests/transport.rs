//! Transports: JSON-lines over in-memory and OS pipes, the in-process client, and WebSocket.
mod common;

use std::io::{BufRead, BufReader, Cursor, Write};

use common::*;
use serde_json::{Value, json};
use sm_protocol::{Endpoint, Request};
use sm_server::{InProcess, Server, serve_lines};

fn server() -> Server {
    Server::new(demo_builder()).expect("server")
}

fn lines(out: &[u8]) -> Vec<Value> {
    String::from_utf8(out.to_vec()).expect("utf8").lines().map(|l| serde_json::from_str(l).expect("json line")).collect()
}

#[test]
fn stdio_round_trip_over_an_in_memory_pipe() {
    let input = [
        r#"{"jsonrpc":"2.0","id":1,"method":"apply","params":{"op":"demo.add_item","params":{"name":"a"}}}"#,
        "",
        r#"{"jsonrpc":"2.0","id":"two","method":"session.hash"}"#,
        r#"{"jsonrpc":"2.0","id":3,"method":"timeline.status"}"#,
        r#"{"jsonrpc":"2.0","id":4,"method":"catalog"}"#,
    ]
    .join("\n");
    let mut out = Vec::new();
    serve_lines(&mut server(), Cursor::new(input), &mut out).expect("serve");
    let r = lines(&out);
    assert_eq!(r.len(), 4, "one reply per non-blank request: {r:?}");
    assert_eq!(r[0]["id"], 1);
    assert_eq!(r[0]["result"]["delta"]["outputs"]["item"], json!(["TopoBody#1"]));
    assert_eq!(r[1]["id"], "two");
    assert_eq!(r[1]["result"]["state"], r[0]["result"]["state"]);
    assert_eq!(r[2]["result"]["nodes"], 1);
    assert_eq!(r[3]["result"]["protocol"], sm_protocol::PROTOCOL_VERSION);
    assert!(r.iter().all(|x| x["jsonrpc"] == "2.0"));
}

#[test]
fn a_request_without_id_gets_an_invalid_request_reply_with_null_id() {
    let input = "{\"jsonrpc\":\"2.0\",\"method\":\"timeline.status\"}\n{\"jsonrpc\":\"2.0\",\"id\":7,\"method\":\"timeline.status\"}\n";
    let mut out = Vec::new();
    serve_lines(&mut server(), Cursor::new(input), &mut out).expect("serve");
    let r = lines(&out);
    assert_eq!(r.len(), 2, "the notification is answered, not dropped");
    assert_eq!(r[0]["id"], Value::Null);
    assert_eq!(r[0]["error"]["code"], sm_protocol::INVALID_REQUEST);
    assert_eq!(r[0]["error"]["kind"], "invalid-request");
    assert_eq!(r[1]["id"], 7);
    assert!(r[1]["result"].is_object());
}

#[test]
fn malformed_lines_get_typed_error_replies_and_the_stream_continues() {
    let mut input: Vec<u8> = Vec::new();
    input.extend_from_slice(b"{not json\n");
    input.extend_from_slice(b"[1,2]\n");
    input.extend_from_slice(b"{\"jsonrpc\":\"1.0\",\"id\":5,\"method\":\"catalog\"}\n");
    input.extend_from_slice(b"{\"jsonrpc\":\"2.0\",\"id\":6,\"method\":\"apply\",\"params\":{\"op\":1}}\n");
    input.extend_from_slice(&[0xff, 0xfe, b'\n']);
    input.extend_from_slice(b"{\"jsonrpc\":\"2.0\",\"id\":8,\"method\":\"timeline.status\"}"); // no trailing newline
    let mut out = Vec::new();
    serve_lines(&mut server(), Cursor::new(input), &mut out).expect("serve");
    let r = lines(&out);
    let kinds: Vec<&str> = r.iter().map(|x| x["error"]["kind"].as_str().unwrap_or("ok")).collect();
    assert_eq!(kinds, ["parse-error", "invalid-request", "invalid-request", "invalid-params", "parse-error", "ok"]);
    assert_eq!(r[2]["id"], 5, "a readable id is echoed");
    assert_eq!(r[3]["id"], 6);
    assert_eq!(r[5]["id"], 8);
}

#[test]
fn lines_round_trip_over_a_real_os_pipe() {
    let (req_r, mut req_w) = std::io::pipe().expect("pipe");
    let (resp_r, resp_w) = std::io::pipe().expect("pipe");
    let t = std::thread::spawn(move || {
        let mut s = server();
        serve_lines(&mut s, BufReader::new(req_r), resp_w).expect("serve");
    });
    let mut replies = BufReader::new(resp_r);
    let mut ask = |text: &str| -> Value {
        writeln!(req_w, "{text}").expect("write");
        let mut l = String::new();
        replies.read_line(&mut l).expect("read");
        serde_json::from_str(&l).expect("json")
    };
    let a = ask(r#"{"jsonrpc":"2.0","id":1,"method":"apply","params":{"op":"demo.add_item","params":{"name":"a"}}}"#);
    let h = ask(r#"{"jsonrpc":"2.0","id":2,"method":"session.hash"}"#);
    assert_eq!(a["result"]["state"], h["result"]["state"]);
    drop(req_w);
    t.join().expect("server thread ends at EOF");
}

#[test]
fn in_process_client_matches_text_and_typed_paths() {
    let mut ip = InProcess::new(server());
    let v = ip.request(Request::Apply { op: "demo.add_item".into(), params: json!({"name": "a"}), version: None }).expect("apply");
    assert_eq!(v["delta"]["outputs"]["item"], json!(["TopoBody#1"]));
    let text = ip.send_text(r#"{"jsonrpc":"2.0","id":9,"method":"session.hash"}"#);
    let r: Value = serde_json::from_str(&text).expect("json");
    assert_eq!(r["result"]["state"], v["state"]);
    assert!(!text.contains('\n'), "one line");
    let e = ip.request(Request::Apply { op: "nope.nope".into(), params: json!({}), version: None }).expect_err("unknown op");
    assert_eq!(e.kind, "unknown-op");
    assert_eq!(ip.endpoint().workspace().timeline().len(), 1);
}

#[test]
fn oversized_lines_are_rejected_and_skipped() {
    // Exercised through the public limit with a small synthetic reader: a line over the limit is
    // answered once and the next line is served.
    let big = format!("{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"catalog\",\"pad\":\"{}\"}}\n", "x".repeat(300));
    let ok = "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"timeline.status\"}\n";
    let mut out = Vec::new();
    sm_server::transport::serve_lines_with_limit(&mut server(), Cursor::new(format!("{big}{ok}")), &mut out, 200).expect("serve");
    let r = lines(&out);
    assert_eq!(r.len(), 2);
    assert_eq!(r[0]["error"]["kind"], "invalid-request");
    assert_eq!(r[0]["id"], Value::Null);
    assert_eq!(r[1]["id"], 2);
}

#[test]
fn replies_are_deterministic_bytes() {
    let input = "{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"apply\",\"params\":{\"op\":\"demo.add_item\",\"params\":{\"name\":\"a\"}}}\n{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"timeline.log\",\"params\":{\"limit\":5}}\n{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"catalog\"}\n";
    let run = || {
        let mut out = Vec::new();
        serve_lines(&mut server(), Cursor::new(input), &mut out).expect("serve");
        out
    };
    assert_eq!(run(), run());
}

#[cfg(feature = "ws")]
#[test]
fn websocket_round_trip() {
    use std::net::TcpListener;
    use tungstenite::{Message, connect};

    let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
    let addr = listener.local_addr().expect("addr");
    let t = std::thread::spawn(move || {
        let mut s = server();
        sm_server::serve_ws_listener(&mut s, &listener, Some(1), |e| panic!("connection failed: {e}")).expect("serve");
    });
    let (mut ws, _) = connect(format!("ws://{addr}")).expect("connect");
    let mut ask = |text: &str| -> Value {
        ws.send(Message::text(text)).expect("send");
        match ws.read().expect("read") {
            Message::Text(t) => serde_json::from_str(t.as_str()).expect("json"),
            other => panic!("unexpected frame {other:?}"),
        }
    };
    let a = ask(r#"{"jsonrpc":"2.0","id":1,"method":"apply","params":{"op":"demo.add_item","params":{"name":"a"}}}"#);
    assert_eq!(a["result"]["delta"]["outputs"]["item"], json!(["TopoBody#1"]));
    let n = ask(r#"{"jsonrpc":"2.0","method":"timeline.status"}"#);
    assert_eq!(n["error"]["kind"], "invalid-request");
    assert_eq!(n["id"], Value::Null);
    ws.send(Message::Binary(vec![1, 2, 3].into())).expect("send binary");
    match ws.read().expect("read") {
        Message::Text(t) => assert!(t.as_str().contains("invalid-request")),
        other => panic!("unexpected frame {other:?}"),
    }
    ws.close(None).expect("close");
    let _ = ws.flush();
    drop(ws);
    t.join().expect("server thread");
}

#[test]
fn server_is_an_endpoint_object() {
    // Front ends code against `dyn Endpoint`.
    let mut s = server();
    let e: &mut dyn Endpoint = &mut s;
    assert!(e.call(Request::Status).is_ok());
}
