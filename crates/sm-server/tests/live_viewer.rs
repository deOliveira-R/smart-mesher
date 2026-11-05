//! W3b-SERVER-HOST: the viewer's transport. Static files and WebSocket clients on one port, any
//! number of connections sharing one session, and live updates by polling `session.hash`
//! (acceptance 1).
#![cfg(feature = "ws")]

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};

use serde_json::{Value, json};
use sm_server::{Server, serve_http_ws_listener, standard_with};
use tungstenite::{Message, WebSocket, connect, stream::MaybeTlsStream};

const TIME: &str = "2026-10-24T00:00:00Z";

type Client = WebSocket<MaybeTlsStream<TcpStream>>;

/// Start a server on a thread that serves `connections` connections, then returns.
fn start(connections: usize) -> (SocketAddr, std::thread::JoinHandle<()>) {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
    let addr = listener.local_addr().expect("addr");
    let h = std::thread::spawn(move || {
        let mut s = Server::new(standard_with(Some("tester".into()), Some(TIME.into()))).expect("server");
        serve_http_ws_listener(&mut s, &listener, Some(connections), |e| panic!("connection failed: {e}")).expect("serve");
    });
    (addr, h)
}

fn client(addr: SocketAddr) -> Client {
    connect(format!("ws://{addr}")).expect("connect").0
}

fn rpc(c: &mut Client, id: u64, method: &str, params: Value) -> Value {
    let req = json!({"jsonrpc": "2.0", "id": id, "method": method, "params": params});
    c.send(Message::text(req.to_string())).expect("send");
    loop {
        if let Message::Text(t) = c.read().expect("read") {
            let v: Value = serde_json::from_str(t.as_str()).expect("json");
            assert_eq!(v["id"], id, "{v}");
            assert!(v.get("error").is_none_or(Value::is_null), "{method}: {v}");
            return v["result"].clone();
        }
    }
}

fn http(addr: SocketAddr, request: &str) -> (String, Vec<u8>) {
    let mut s = TcpStream::connect(addr).expect("connect");
    s.write_all(request.as_bytes()).expect("write");
    let mut raw = Vec::new();
    s.read_to_end(&mut raw).expect("read");
    let split = raw.windows(4).position(|w| w == b"\r\n\r\n").expect("head end");
    (String::from_utf8_lossy(&raw[..split]).into_owned(), raw[split + 4..].to_vec())
}

#[test]
fn static_files_are_served_on_the_websocket_port() {
    let assets = sm_view::assets();
    // One connection per asset, plus a 404, a HEAD and a POST.
    let (addr, h) = start(assets.len() + 3);
    for a in assets {
        let path = if a.path == "index.html" { "/".to_owned() } else { format!("/{}", a.path) };
        let (head, body) = http(addr, &format!("GET {path} HTTP/1.1\r\nHost: x\r\n\r\n"));
        assert!(head.starts_with("HTTP/1.1 200 OK"), "{path}: {head}");
        assert!(head.contains(&format!("Content-Type: {}", a.mime)), "{head}");
        assert_eq!(body, a.bytes, "{path} body");
    }
    let (head, body) = http(addr, "GET /nope.js HTTP/1.1\r\nHost: x\r\n\r\n");
    assert!(head.starts_with("HTTP/1.1 404"), "{head}");
    assert!(!body.is_empty());
    let (head, body) = http(addr, "HEAD / HTTP/1.1\r\nHost: x\r\n\r\n");
    assert!(head.starts_with("HTTP/1.1 200") && body.is_empty(), "{head}");
    let (head, _) = http(addr, "POST / HTTP/1.1\r\nHost: x\r\nContent-Length: 0\r\n\r\n");
    assert!(head.starts_with("HTTP/1.1 405"), "{head}");
    h.join().expect("server thread");
}

#[test]
fn a_second_client_changes_the_state_and_the_first_sees_it_by_polling() {
    let (addr, h) = start(3);
    let mut a = client(addr);
    let scene0 = rpc(&mut a, 1, "query", json!({"name": "view.scene", "input": {}}));
    assert_eq!(scene0["volumes"], json!([]), "{scene0}");
    let hash0 = rpc(&mut a, 2, "session.hash", json!({}));

    // Client B, on a second connection, builds a brick through the shared session.
    let mut b = client(addr);
    let run = rpc(&mut b, 1, "script.run", json!({"text": "let x = model.create_brick(x: 2.0, y: 2.0, z: 2.0)\n"}));
    let hash_b = run["state"].clone();
    assert_ne!(hash_b, hash0["state"], "the script changed nothing");
    b.close(None).expect("close b");

    // A's next poll returns B's hash, and its re-query reflects the change.
    let hash1 = rpc(&mut a, 3, "session.hash", json!({}));
    assert_eq!(hash1["state"], hash_b);
    let scene1 = rpc(&mut a, 4, "query", json!({"name": "view.scene", "input": {}}));
    assert_eq!(scene1["volumes"].as_array().map(Vec::len), Some(1), "{scene1}");
    assert_ne!(scene0, scene1);

    // The page itself comes from the same port.
    let (head, _) = http(addr, "GET / HTTP/1.1\r\nHost: x\r\n\r\n");
    assert!(head.starts_with("HTTP/1.1 200"), "{head}");
    a.close(None).expect("close a");
    h.join().expect("server thread");
}

#[test]
fn many_clients_share_one_writer_and_agree() {
    let n = 6;
    let (addr, h) = start(n);
    let mut clients: Vec<Client> = (0..n).map(|_| client(addr)).collect();
    // Each client appends a different brick; every reply is a distinct state.
    let mut states = Vec::new();
    for (i, c) in clients.iter_mut().enumerate() {
        let size = 1.0 + i as f64;
        let r = rpc(c, 1, "script.run", json!({"text": format!("let b = model.create_brick(x: {size:?}, y: 1.0, z: 1.0)\n")}));
        states.push(r["state"].clone());
    }
    let mut distinct: Vec<String> = states.iter().map(Value::to_string).collect();
    distinct.sort();
    distinct.dedup();
    assert_eq!(distinct.len(), n, "every write produced a new state");
    let last = states.last().cloned().expect("states");
    for c in &mut clients {
        assert_eq!(rpc(c, 2, "session.hash", json!({}))["state"], last);
    }
    for c in &mut clients {
        c.close(None).expect("close");
    }
    h.join().expect("server thread");
}
