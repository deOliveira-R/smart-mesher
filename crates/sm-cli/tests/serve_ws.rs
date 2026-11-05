//! `smesh serve --ws ADDR`: the viewer page and WebSocket clients on one port, one shared session.
#![cfg(feature = "ws")]

use std::io::{Cursor, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::time::Duration;

fn free_addr() -> String {
    let l = TcpListener::bind("127.0.0.1:0").expect("bind");
    l.local_addr().expect("addr").to_string()
}

fn connect_retry(addr: &str) -> TcpStream {
    for _ in 0..200 {
        if let Ok(s) = TcpStream::connect(addr) {
            return s;
        }
        std::thread::sleep(Duration::from_millis(25));
    }
    panic!("server did not start on {addr}");
}

#[test]
fn serve_ws_serves_the_page_and_a_websocket_session() {
    let addr = free_addr();
    let a = addr.clone();
    // The server runs until the process ends.
    std::thread::spawn(move || {
        let (mut out, mut err) = (Vec::new(), Vec::new());
        let mut input = Cursor::new(Vec::new());
        let mut io = sm_cli::Io { stdin: &mut input, stdout: &mut out, stderr: &mut err };
        let mut connect = |s: &sm_cli::Settings| sm_cli::connect_standard(s);
        sm_cli::run(["smesh", "--memory", "serve", "--ws", a.as_str()], &mut connect, &mut io)
    });

    let mut page = connect_retry(&addr);
    page.write_all(b"GET / HTTP/1.1\r\nHost: x\r\n\r\n").expect("write");
    let mut raw = Vec::new();
    page.read_to_end(&mut raw).expect("read");
    let text = String::from_utf8_lossy(&raw);
    assert!(text.starts_with("HTTP/1.1 200 OK"), "{}", &text[..text.len().min(200)]);
    assert!(text.contains("<html") || text.contains("<!doctype") || text.contains("<!DOCTYPE"), "index page expected");

    let mut ws = tungstenite::client(format!("ws://{addr}"), connect_retry(&addr)).expect("upgrade").0;
    ws.send(tungstenite::Message::text(r#"{"jsonrpc":"2.0","id":1,"method":"session.hash","params":{}}"#)).expect("send");
    let reply = loop {
        if let tungstenite::Message::Text(t) = ws.read().expect("read") {
            break t;
        }
    };
    let v: serde_json::Value = serde_json::from_str(reply.as_str()).expect("json");
    assert!(v["result"]["state"].is_string(), "{v}");
}
