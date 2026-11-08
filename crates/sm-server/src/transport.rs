//! Transports: they move JSON-RPC text to an [`Endpoint`] and back, nothing else. Every method is
//! dispatched by the endpoint ([`Endpoint::handle_text`]), so a transport cannot change meaning.
//!
//! * [`InProcess`]: call an endpoint with text or typed requests, no I/O.
//! * [`serve_lines`] / [`serve_stdio`]: JSON-lines, one request per line, one reply per line.
//! * `serve_ws` (feature `ws`): one request per WebSocket text frame.
//! * `serve_http_ws` (feature `ws`): the same, with any number of simultaneous connections sharing one
//!   endpoint, plus plain `GET` of the web viewer's static files on the same port.
//!
//! A request without an `id` (a JSON-RPC notification, which this protocol does not support) gets
//! an `INVALID_REQUEST` reply with `id: null`; an unreadable line gets a `PARSE_ERROR` reply.
//! Nothing is ever dropped silently.

use std::io::{self, BufRead, Write};

use serde_json::Value;
use sm_protocol::{Endpoint, Envelope, Request, Response, RpcError};

/// Longest request line accepted by [`serve_lines`], in bytes. Longer lines are discarded and
/// answered with an `INVALID_REQUEST` error.
pub const MAX_LINE_BYTES: usize = 64 << 20;

/// An endpoint driven directly, without a pipe.
#[derive(Debug)]
pub struct InProcess<E> {
    endpoint: E,
    next_id: u64,
}

impl<E: Endpoint> InProcess<E> {
    /// Wrap `endpoint`.
    pub fn new(endpoint: E) -> Self {
        Self { endpoint, next_id: 1 }
    }

    /// Handle one JSON-RPC text and return the serialized reply (no trailing newline).
    pub fn send_text(&mut self, text: &str) -> String {
        reply_text(&self.endpoint_response(text))
    }

    fn endpoint_response(&mut self, text: &str) -> Response {
        self.endpoint.handle_text(text)
    }

    /// Send a typed request through the full envelope path (serialize, parse, dispatch), exactly
    /// as a remote client would, and return the typed outcome.
    pub fn request(&mut self, req: Request) -> Result<Value, RpcError> {
        let id = self.next_id;
        self.next_id += 1;
        let env = Envelope::new(Value::from(id), req);
        let text = serde_json::to_string(&env).map_err(|e| RpcError::internal(e.to_string()))?;
        let resp = self.endpoint.handle_text(&text);
        match (resp.result, resp.error) {
            (_, Some(e)) => Err(e),
            (Some(v), None) => Ok(v),
            (None, None) => Err(RpcError::internal("reply carries neither result nor error")),
        }
    }

    /// The wrapped endpoint.
    pub fn endpoint(&self) -> &E {
        &self.endpoint
    }

    /// The wrapped endpoint, mutably.
    pub fn endpoint_mut(&mut self) -> &mut E {
        &mut self.endpoint
    }

    /// Give the endpoint back.
    pub fn into_inner(self) -> E {
        self.endpoint
    }
}

fn reply_text(resp: &Response) -> String {
    serde_json::to_string(resp).unwrap_or_else(|e| {
        // A reply that cannot be serialized (non-finite float in a result, say) is a server bug.
        let err = Response::err(Value::Null, RpcError::internal(format!("cannot serialize reply: {e}")));
        serde_json::to_string(&err).unwrap_or_default()
    })
}

/// One line, or why there is none.
enum Line {
    Eof,
    Text(String),
    TooLong,
    NotUtf8,
}

fn read_line(input: &mut impl BufRead, buf: &mut Vec<u8>, max: usize) -> io::Result<Line> {
    buf.clear();
    let mut too_long = false;
    loop {
        let chunk = input.fill_buf()?;
        if chunk.is_empty() {
            return Ok(if buf.is_empty() && !too_long { Line::Eof } else { finish(buf, too_long) });
        }
        let (take, found) = match chunk.iter().position(|&b| b == b'\n') {
            Some(i) => (i + 1, true),
            None => (chunk.len(), false),
        };
        if !too_long {
            if buf.len() + take > max.saturating_add(1) {
                too_long = true;
                buf.clear();
            } else {
                buf.extend_from_slice(&chunk[..take]);
            }
        }
        input.consume(take);
        if found {
            return Ok(finish(buf, too_long));
        }
    }
}

fn finish(buf: &[u8], too_long: bool) -> Line {
    if too_long {
        return Line::TooLong;
    }
    match std::str::from_utf8(buf) {
        Ok(s) => Line::Text(s.trim_end_matches(['\n', '\r']).to_owned()),
        Err(_) => Line::NotUtf8,
    }
}

/// Serve JSON-lines: read requests from `input` until EOF, write one reply line per non-blank
/// request line to `output`, flushing after each. Blank lines are ignored.
pub fn serve_lines(endpoint: &mut (impl Endpoint + ?Sized), input: impl BufRead, output: impl Write) -> io::Result<()> {
    serve_lines_with_limit(endpoint, input, output, MAX_LINE_BYTES)
}

/// [`serve_lines`] with an explicit longest-line limit in bytes (default [`MAX_LINE_BYTES`]).
pub fn serve_lines_with_limit(
    endpoint: &mut (impl Endpoint + ?Sized),
    mut input: impl BufRead,
    mut output: impl Write,
    max_line: usize,
) -> io::Result<()> {
    let mut buf = Vec::new();
    loop {
        let resp = match read_line(&mut input, &mut buf, max_line)? {
            Line::Eof => return Ok(()),
            Line::Text(t) if t.trim().is_empty() => continue,
            Line::Text(t) => endpoint.handle_text(&t),
            Line::TooLong => Response::err(
                Value::Null,
                RpcError::invalid_request(format!("request line exceeds {max_line} bytes")),
            ),
            Line::NotUtf8 => Response::err(Value::Null, RpcError::parse("request line is not valid UTF-8")),
        };
        output.write_all(reply_text(&resp).as_bytes())?;
        output.write_all(b"\n")?;
        output.flush()?;
    }
}

/// [`serve_lines`] over the process's stdin and stdout.
pub fn serve_stdio(endpoint: &mut (impl Endpoint + ?Sized)) -> io::Result<()> {
    serve_lines(endpoint, io::stdin().lock(), io::stdout().lock())
}

#[cfg(feature = "ws")]
pub use ws::{MAX_LIVE_CONNECTIONS, serve_http_ws, serve_http_ws_listener, serve_ws, serve_ws_listener};

#[cfg(feature = "ws")]
mod ws {
    use std::io::{self, Read, Write};
    use std::net::{TcpListener, TcpStream, ToSocketAddrs};
    use std::sync::{PoisonError, mpsc};
    use std::time::{Duration, Instant};

    use serde_json::Value;
    use sm_protocol::{Endpoint, Response, RpcError};
    use std::net::{IpAddr, SocketAddr};
    use std::sync::atomic::{AtomicUsize, Ordering};
    use tungstenite::handshake::server::{ErrorResponse, Request, Response as HsResponse};
    use tungstenite::http::StatusCode;
    use tungstenite::{Message, accept_hdr};

    use super::reply_text;

    fn io_err(e: tungstenite::Error) -> io::Error {
        match e {
            tungstenite::Error::Io(e) => e,
            other => io::Error::other(other.to_string()),
        }
    }

    /// Whether a browser page with this `Origin` may talk to a server reached at `local`.
    ///
    /// No `Origin` (a non-browser client) is accepted. Otherwise the origin must be `http://host[:port]`
    /// naming the address the connection arrived on; `localhost`, `127.0.0.1` and `[::1]` are
    /// interchangeable only when that address is loopback, and the port must match either way.
    fn origin_allowed(origin: Option<&str>, local: SocketAddr) -> bool {
        let Some(origin) = origin else { return true };
        let Some(authority) = origin.strip_prefix("http://") else { return false };
        if authority.is_empty() || authority.contains(['/', '?', '#', '@']) {
            return false;
        }
        let (host, port) = match authority.rsplit_once(':') {
            Some((h, p)) if !p.contains(']') => match p.parse::<u16>() {
                Ok(p) => (h, p),
                Err(_) => return false,
            },
            _ => (authority, 80),
        };
        if port != local.port() {
            return false;
        }
        let host = host.to_ascii_lowercase();
        let ip = host.trim_start_matches('[').trim_end_matches(']').parse::<IpAddr>().ok();
        if local.ip().is_loopback() && (host == "localhost" || ip.is_some_and(|i| i.is_loopback())) {
            return true;
        }
        ip == Some(local.ip())
    }

    /// The handshake callback: refuse a foreign `Origin` with 403 before the upgrade.
    // The `Err` type is fixed by tungstenite's `Callback` trait.
    #[allow(clippy::result_large_err)]
    fn check_origin(local: SocketAddr) -> impl FnOnce(&Request, HsResponse) -> Result<HsResponse, ErrorResponse> {
        move |req, resp| {
            let origin = req.headers().get("origin").map(|v| v.to_str().unwrap_or("\u{0}invalid"));
            if origin_allowed(origin, local) {
                Ok(resp)
            } else {
                let mut r = ErrorResponse::new(Some("origin not allowed\n".to_owned()));
                *r.status_mut() = StatusCode::FORBIDDEN;
                Err(r)
            }
        }
    }

    fn serve_connection(endpoint: &mut (impl Endpoint + ?Sized), stream: TcpStream) -> io::Result<()> {
        let local = stream.local_addr()?;
        let mut socket = accept_hdr(stream, check_origin(local)).map_err(|e| io::Error::other(e.to_string()))?;
        loop {
            let msg = match socket.read() {
                Ok(m) => m,
                Err(tungstenite::Error::ConnectionClosed | tungstenite::Error::AlreadyClosed) => return Ok(()),
                Err(e) => return Err(io_err(e)),
            };
            let resp = match msg {
                Message::Text(t) => endpoint.handle_text(t.as_str()),
                Message::Binary(_) => Response::err(Value::Null, RpcError::invalid_request("binary frames are not supported; send JSON text")),
                Message::Close(_) => return Ok(()),
                // Ping/pong are answered by the library.
                _ => continue,
            };
            socket.send(Message::text(reply_text(&resp))).map_err(io_err)?;
        }
    }

    /// Serve connections from `listener` one after another (a single writer means no two
    /// connections interleave), until `max_connections` have been served (`None`: forever). A
    /// failing connection is reported through `on_error` and does not stop the loop.
    pub fn serve_ws_listener(
        endpoint: &mut (impl Endpoint + ?Sized),
        listener: &TcpListener,
        max_connections: Option<usize>,
        mut on_error: impl FnMut(io::Error),
    ) -> io::Result<()> {
        let mut served = 0usize;
        while max_connections.is_none_or(|m| served < m) {
            let (stream, _) = listener.accept()?;
            if let Err(e) = serve_connection(endpoint, stream) {
                on_error(e);
            }
            served += 1;
        }
        Ok(())
    }

    /// Bind `addr` and serve WebSocket clients forever, one text frame per JSON-RPC message.
    pub fn serve_ws(endpoint: &mut (impl Endpoint + ?Sized), addr: impl ToSocketAddrs) -> io::Result<()> {
        let listener = TcpListener::bind(addr)?;
        serve_ws_listener(endpoint, &listener, None, |e| eprintln!("sm-server: connection ended: {e}"))
    }

    /// Longest request head (request line and headers) read from a connection, in bytes.
    const MAX_HEAD: usize = 8192;
    /// How long a new connection may take to send its request head.
    const HEAD_TIMEOUT: Duration = Duration::from_secs(10);

    /// Most connections the shared listener keeps open at once; further ones get `503`.
    pub const MAX_LIVE_CONNECTIONS: usize = 64;

    /// One request waiting for the endpoint: the frame text and where its reply goes.
    type Job = (String, mpsc::Sender<Response>);

    /// What a new connection asked for.
    enum Kind {
        /// A WebSocket upgrade.
        Upgrade,
        /// A plain HTTP request: method and path.
        Http(String, String),
        /// Not HTTP, too large, or silent: drop.
        Bad,
    }

    /// Look at the request head without consuming it, so a WebSocket upgrade can still go to
    /// `tungstenite` untouched.
    fn classify(stream: &TcpStream) -> Kind {
        let deadline = Instant::now() + HEAD_TIMEOUT;
        let mut buf = vec![0u8; MAX_HEAD];
        let _ = stream.set_read_timeout(Some(HEAD_TIMEOUT));
        let kind = loop {
            let n = match stream.peek(&mut buf) {
                Ok(0) | Err(_) => break Kind::Bad,
                Ok(n) => n,
            };
            if let Some(end) = buf[..n].windows(4).position(|w| w == b"\r\n\r\n") {
                break parse_head(&buf[..end]);
            }
            if n == MAX_HEAD || Instant::now() >= deadline {
                break Kind::Bad;
            }
            // Only a prefix has arrived; peek returns it again until more comes.
            std::thread::sleep(Duration::from_millis(2));
        };
        let _ = stream.set_read_timeout(None);
        kind
    }

    fn parse_head(head: &[u8]) -> Kind {
        let Ok(text) = std::str::from_utf8(head) else { return Kind::Bad };
        let mut lines = text.split("\r\n");
        let mut first = lines.next().unwrap_or("").split(' ');
        let (Some(method), Some(target)) = (first.next(), first.next()) else { return Kind::Bad };
        let upgrade = lines.any(|l| {
            l.split_once(':')
                .is_some_and(|(k, v)| k.trim().eq_ignore_ascii_case("upgrade") && v.trim().eq_ignore_ascii_case("websocket"))
        });
        if upgrade {
            return Kind::Upgrade;
        }
        let path = target.split(['?', '#']).next().unwrap_or("/");
        Kind::Http(method.to_owned(), path.to_owned())
    }

    /// The asset served at `path` (`/` is the index page).
    fn asset_for(path: &str) -> Option<&'static sm_view::Asset> {
        let rel = path.trim_start_matches('/');
        let rel = if rel.is_empty() { "index.html" } else { rel };
        sm_view::assets().iter().find(|a| a.path == rel)
    }

    fn respond(stream: &mut TcpStream, status: &str, mime: &str, body: &[u8], with_body: bool) -> io::Result<()> {
        let head = format!(
            "HTTP/1.1 {status}\r\nContent-Type: {mime}\r\nContent-Length: {}\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n",
            body.len()
        );
        stream.write_all(head.as_bytes())?;
        if with_body {
            stream.write_all(body)?;
        }
        stream.flush()
    }

    fn serve_http(mut stream: TcpStream, method: &str, path: &str) -> io::Result<()> {
        // Consume the request head (it was only peeked) so closing does not reset the connection.
        let mut sink = vec![0u8; MAX_HEAD];
        let mut seen: Vec<u8> = Vec::new();
        while !seen.windows(4).any(|w| w == b"\r\n\r\n") {
            let n = stream.read(&mut sink)?;
            if n == 0 {
                break;
            }
            seen.extend_from_slice(&sink[..n]);
        }
        let get = method == "GET";
        if !get && method != "HEAD" {
            return respond(&mut stream, "405 Method Not Allowed", "text/plain; charset=utf-8", b"method not allowed\n", true);
        }
        match asset_for(path) {
            Some(a) => respond(&mut stream, "200 OK", a.mime, a.bytes, get),
            None => respond(&mut stream, "404 Not Found", "text/plain; charset=utf-8", b"not found\n", get),
        }
    }

    fn serve_shared_connection(jobs: &mpsc::Sender<Job>, stream: TcpStream) -> io::Result<()> {
        match classify(&stream) {
            Kind::Bad => Ok(()),
            Kind::Http(method, path) => serve_http(stream, &method, &path),
            Kind::Upgrade => {
                let local = stream.local_addr()?;
                let mut socket = accept_hdr(stream, check_origin(local)).map_err(|e| io::Error::other(e.to_string()))?;
                loop {
                    let msg = match socket.read() {
                        Ok(m) => m,
                        Err(tungstenite::Error::ConnectionClosed | tungstenite::Error::AlreadyClosed) => return Ok(()),
                        Err(e) => return Err(io_err(e)),
                    };
                    let resp = match msg {
                        Message::Text(t) => {
                            let (tx, rx) = mpsc::channel();
                            if jobs.send((t.as_str().to_owned(), tx)).is_err() {
                                return Ok(());
                            }
                            match rx.recv() {
                                Ok(r) => r,
                                Err(_) => return Ok(()),
                            }
                        }
                        Message::Binary(_) => {
                            Response::err(Value::Null, RpcError::invalid_request("binary frames are not supported; send JSON text"))
                        }
                        Message::Close(_) => return Ok(()),
                        _ => continue,
                    };
                    socket.send(Message::text(reply_text(&resp))).map_err(io_err)?;
                }
            }
        }
    }

    /// Serve any number of simultaneous connections from `listener` against one endpoint, until
    /// `max_connections` have been accepted and have ended (`None`: forever).
    ///
    /// A connection is either a WebSocket client (one JSON-RPC message per text frame) or a plain
    /// `GET`/`HEAD` of the viewer's static files ([`sm_view::assets`]; `/` is `index.html`).
    /// Connections run on their own threads but only move text: the calling thread owns the
    /// endpoint and handles requests one at a time in arrival order, so the session keeps its
    /// single writer and every client sees the same state. There is no server push; clients poll
    /// `session.hash` to notice changes. A failing connection is reported through `on_error`.
    ///
    /// Safety limits: a WebSocket upgrade is accepted only when its `Origin` header is absent or is
    /// `http://` plus the address the server is reached at (loopback names are equal only on a
    /// loopback address); any other origin gets `403` before the upgrade, so a web page cannot drive
    /// the session from the user's browser. At most [`MAX_LIVE_CONNECTIONS`] connections are open at
    /// once; the rest get `503` and are closed.
    pub fn serve_http_ws_listener(
        endpoint: &mut (impl Endpoint + ?Sized),
        listener: &TcpListener,
        max_connections: Option<usize>,
        on_error: impl Fn(io::Error) + Sync,
    ) -> io::Result<()> {
        serve_shared(endpoint, listener, max_connections, on_error, &RefusalStats::default())
    }

    /// Most `503` replies written at once; further excess connections are closed without a reply.
    const MAX_REFUSALS_IN_FLIGHT: usize = 8;

    /// Counters of the excess-connection path (kept for tests; they never affect behaviour).
    #[derive(Default)]
    struct RefusalStats {
        in_flight: AtomicUsize,
        high_water: AtomicUsize,
        refused: AtomicUsize,
        dropped: AtomicUsize,
    }

    impl RefusalStats {
        /// Claim a refusal slot; `false` (and one more drop counted) when all are taken.
        fn enter(&self) -> bool {
            let now = self.in_flight.fetch_add(1, Ordering::SeqCst) + 1;
            if now > MAX_REFUSALS_IN_FLIGHT {
                self.in_flight.fetch_sub(1, Ordering::SeqCst);
                self.dropped.fetch_add(1, Ordering::SeqCst);
                return false;
            }
            self.high_water.fetch_max(now, Ordering::SeqCst);
            self.refused.fetch_add(1, Ordering::SeqCst);
            true
        }

        fn leave(&self) {
            self.in_flight.fetch_sub(1, Ordering::SeqCst);
        }
    }

    /// Write the `503` and drain the request so closing does not reset the connection under it.
    fn refuse(stream: &mut TcpStream) {
        let _ = stream.set_write_timeout(Some(Duration::from_secs(1)));
        let _ = respond(stream, "503 Service Unavailable", "text/plain; charset=utf-8", b"too many connections\n", true);
        let _ = stream.set_read_timeout(Some(Duration::from_millis(200)));
        let _ = stream.read(&mut [0u8; MAX_HEAD]);
    }

    fn serve_shared(
        endpoint: &mut (impl Endpoint + ?Sized),
        listener: &TcpListener,
        max_connections: Option<usize>,
        on_error: impl Fn(io::Error) + Sync,
        stats: &RefusalStats,
    ) -> io::Result<()> {
        let (jobs_tx, jobs_rx) = mpsc::channel::<Job>();
        let accept_error = std::sync::Mutex::new(None::<io::Error>);
        std::thread::scope(|scope| {
            let (on_error, accept_error) = (&on_error, &accept_error);
            scope.spawn(move || {
                let mut served = 0usize;
                let live = AtomicUsize::new(0);
                let live = &live;
                std::thread::scope(|conns| {
                    while max_connections.is_none_or(|m| served < m) {
                        match listener.accept() {
                            Ok((mut stream, _)) => {
                                served += 1;
                                if live.fetch_add(1, Ordering::SeqCst) >= MAX_LIVE_CONNECTIONS {
                                    live.fetch_sub(1, Ordering::SeqCst);
                                    // The reply may wait on a slow peer, so it never runs on the accept thread.
                                    if stats.enter() {
                                        conns.spawn(move || {
                                            refuse(&mut stream);
                                            stats.leave();
                                        });
                                    }
                                    // Otherwise too many refusals are in flight: close without a reply.
                                    continue;
                                }
                                let tx = jobs_tx.clone();
                                conns.spawn(move || {
                                    if let Err(e) = serve_shared_connection(&tx, stream) {
                                        on_error(e);
                                    }
                                    live.fetch_sub(1, Ordering::SeqCst);
                                });
                            }
                            Err(e) => {
                                *accept_error.lock().unwrap_or_else(PoisonError::into_inner) = Some(e);
                                break;
                            }
                        }
                    }
                    drop(jobs_tx);
                });
            });
            for (text, reply) in jobs_rx {
                let _ = reply.send(endpoint.handle_text(&text));
            }
        });
        match accept_error.into_inner().unwrap_or_else(PoisonError::into_inner) {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    /// Bind `addr` and run [`serve_http_ws_listener`] forever.
    pub fn serve_http_ws(endpoint: &mut (impl Endpoint + ?Sized), addr: impl ToSocketAddrs) -> io::Result<()> {
        let listener = TcpListener::bind(addr)?;
        serve_http_ws_listener(endpoint, &listener, None, |e| eprintln!("sm-server: connection ended: {e}"))
    }

    #[cfg(test)]
    mod refusal_tests {
        use super::*;
        use std::sync::Arc;
        use std::sync::atomic::Ordering::SeqCst;

        /// Serve on a thread; it returns the stats once `connections` connections have ended.
        fn start(connections: usize) -> (SocketAddr, Arc<RefusalStats>, std::thread::JoinHandle<()>) {
            let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
            let addr = listener.local_addr().expect("addr");
            let stats = Arc::new(RefusalStats::default());
            let seen = Arc::clone(&stats);
            let h = std::thread::spawn(move || {
                let mut s = crate::Server::new(crate::standard_with(Some("t".into()), Some("2026-10-24T00:00:00Z".into()))).expect("server");
                serve_shared(&mut s, &listener, Some(connections), |_| {}, &seen).expect("serve");
            });
            (addr, stats, h)
        }

        /// `MAX_LIVE_CONNECTIONS` idle connections, each counted live once the server has them.
        fn hold(addr: SocketAddr) -> Vec<TcpStream> {
            (0..MAX_LIVE_CONNECTIONS).map(|_| TcpStream::connect(addr).expect("connect")).collect()
        }

        /// Acceptance 1: excess connections that never send or read do not delay a later client.
        #[test]
        fn silent_excess_connections_do_not_stall_the_accept_loop() {
            let silent = MAX_REFUSALS_IN_FLIGHT - 1;
            let (addr, stats, h) = start(MAX_LIVE_CONNECTIONS + silent + 1);
            let held = hold(addr);
            let idle: Vec<TcpStream> = (0..silent).map(|_| TcpStream::connect(addr).expect("connect")).collect();
            // One more excess connection that does send a request: it must get its 503 promptly
            // (before the fix each silent one cost the accept thread up to 200 ms first).
            let t = Instant::now();
            let mut c = TcpStream::connect(addr).expect("connect");
            c.write_all(b"GET / HTTP/1.1\r\nHost: x\r\nConnection: close\r\n\r\n").expect("write");
            let mut raw = Vec::new();
            c.read_to_end(&mut raw).expect("read");
            let waited = t.elapsed();
            let text = String::from_utf8_lossy(&raw);
            assert!(text.starts_with("HTTP/1.1 503 Service Unavailable\r\n"), "{text}");
            assert!(text.ends_with("\r\n\r\ntoo many connections\n"), "{text}");
            assert!(waited < Duration::from_secs(1), "served after {waited:?}");
            drop((held, idle, c));
            h.join().expect("server");
            assert_eq!(stats.refused.load(SeqCst), silent + 1);
            assert_eq!(stats.dropped.load(SeqCst), 0);
        }

        /// Acceptance 2: a flood never has more than the cap of refusals in flight.
        #[test]
        fn a_flood_of_excess_connections_is_capped() {
            let flood = 4 * MAX_REFUSALS_IN_FLIGHT;
            let (addr, stats, h) = start(MAX_LIVE_CONNECTIONS + flood);
            let held = hold(addr);
            let idle: Vec<TcpStream> = (0..flood).map(|_| TcpStream::connect(addr).expect("connect")).collect();
            // Hold everything until the server has dealt with every excess connection (silent
            // clients keep each refusal busy for its read wait, so the cap is reached).
            // Dropping earlier would free live slots and let later connections in.
            let t = Instant::now();
            while stats.refused.load(SeqCst) + stats.dropped.load(SeqCst) < flood {
                assert!(t.elapsed() < Duration::from_secs(20), "server stalled");
                std::thread::sleep(Duration::from_millis(5));
            }
            drop((held, idle));
            h.join().expect("server");
            let (refused, dropped) = (stats.refused.load(SeqCst), stats.dropped.load(SeqCst));
            assert_eq!(refused + dropped, flood);
            assert!(stats.high_water.load(SeqCst) <= MAX_REFUSALS_IN_FLIGHT, "{}", stats.high_water.load(SeqCst));
            assert!(dropped > 0, "the cap was exercised");
            assert_eq!(stats.in_flight.load(SeqCst), 0);
        }
    }
}
