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
pub use ws::{serve_http_ws, serve_http_ws_listener, serve_ws, serve_ws_listener};

#[cfg(feature = "ws")]
mod ws {
    use std::io::{self, Read, Write};
    use std::net::{TcpListener, TcpStream, ToSocketAddrs};
    use std::sync::{PoisonError, mpsc};
    use std::time::{Duration, Instant};

    use serde_json::Value;
    use sm_protocol::{Endpoint, Response, RpcError};
    use tungstenite::{Message, accept};

    use super::reply_text;

    fn io_err(e: tungstenite::Error) -> io::Error {
        match e {
            tungstenite::Error::Io(e) => e,
            other => io::Error::other(other.to_string()),
        }
    }

    fn serve_connection(endpoint: &mut (impl Endpoint + ?Sized), stream: TcpStream) -> io::Result<()> {
        let mut socket = accept(stream).map_err(|e| io::Error::other(e.to_string()))?;
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
                let mut socket = accept(stream).map_err(|e| io::Error::other(e.to_string()))?;
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
    pub fn serve_http_ws_listener(
        endpoint: &mut (impl Endpoint + ?Sized),
        listener: &TcpListener,
        max_connections: Option<usize>,
        on_error: impl Fn(io::Error) + Sync,
    ) -> io::Result<()> {
        let (jobs_tx, jobs_rx) = mpsc::channel::<Job>();
        let accept_error = std::sync::Mutex::new(None::<io::Error>);
        std::thread::scope(|scope| {
            let (on_error, accept_error) = (&on_error, &accept_error);
            scope.spawn(move || {
                let mut served = 0usize;
                std::thread::scope(|conns| {
                    while max_connections.is_none_or(|m| served < m) {
                        match listener.accept() {
                            Ok((stream, _)) => {
                                let tx = jobs_tx.clone();
                                conns.spawn(move || {
                                    if let Err(e) = serve_shared_connection(&tx, stream) {
                                        on_error(e);
                                    }
                                });
                                served += 1;
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
}
