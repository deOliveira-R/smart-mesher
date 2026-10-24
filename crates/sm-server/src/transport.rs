//! Transports: they move JSON-RPC text to an [`Endpoint`] and back, nothing else. Every method is
//! dispatched by the endpoint ([`Endpoint::handle_text`]), so a transport cannot change meaning.
//!
//! * [`InProcess`]: call an endpoint with text or typed requests, no I/O.
//! * [`serve_lines`] / [`serve_stdio`]: JSON-lines, one request per line, one reply per line.
//! * `serve_ws` (feature `ws`): one request per WebSocket text frame.
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
pub use ws::{serve_ws, serve_ws_listener};

#[cfg(feature = "ws")]
mod ws {
    use std::io;
    use std::net::{TcpListener, TcpStream, ToSocketAddrs};

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
}
