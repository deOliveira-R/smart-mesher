//! Canonical TOML text for [`Value`] trees.
//!
//! This is a deliberately small TOML subset (the `toml` crate is outside the dependency
//! ceiling): scalars and arrays are written inline, nested tables get `[a.b]` headers, keys
//! are sorted, floats use Rust's shortest round-trip form. Output is standard TOML, and the
//! parser reads exactly what the writer produces (plus comments and blank lines).

use std::collections::BTreeMap;
use std::fmt::Write as _;

use crate::value::{CodecError, Value};

/// Write a table as canonical TOML text.
pub fn to_toml(table: &BTreeMap<String, Value>) -> String {
    let mut out = String::new();
    emit_table(&mut out, &mut Vec::new(), table);
    out
}

fn bare_key(k: &str) -> bool {
    !k.is_empty() && k.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-')
}

fn emit_key(out: &mut String, k: &str) {
    if bare_key(k) {
        out.push_str(k);
    } else {
        emit_str(out, k);
    }
}

fn emit_str(out: &mut String, s: &str) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 || c as u32 == 0x7f => {
                let _ = write!(out, "\\u{:04X}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

fn emit_float(out: &mut String, f: f64) {
    if f.is_nan() {
        out.push_str("nan");
    } else if f.is_infinite() {
        out.push_str(if f > 0.0 { "inf" } else { "-inf" });
    } else {
        let _ = write!(out, "{f:?}");
    }
}

fn emit_inline(out: &mut String, v: &Value) {
    match v {
        Value::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
        Value::Int(i) => {
            let _ = write!(out, "{i}");
        }
        Value::Float(f) => emit_float(out, *f),
        Value::Str(s) => emit_str(out, s),
        Value::Array(a) => {
            out.push('[');
            for (i, e) in a.iter().enumerate() {
                if i > 0 {
                    out.push_str(", ");
                }
                emit_inline(out, e);
            }
            out.push(']');
        }
        Value::Table(t) => {
            if t.is_empty() {
                out.push_str("{}");
                return;
            }
            out.push_str("{ ");
            for (i, (k, e)) in t.iter().enumerate() {
                if i > 0 {
                    out.push_str(", ");
                }
                emit_key(out, k);
                out.push_str(" = ");
                emit_inline(out, e);
            }
            out.push_str(" }");
        }
    }
}

fn emit_table<'a>(out: &mut String, path: &mut Vec<&'a str>, t: &'a BTreeMap<String, Value>) {
    for (k, v) in t {
        if !matches!(v, Value::Table(_)) {
            emit_key(out, k);
            out.push_str(" = ");
            emit_inline(out, v);
            out.push('\n');
        }
    }
    for (k, v) in t {
        if let Value::Table(sub) = v {
            path.push(k);
            out.push('\n');
            out.push('[');
            for (i, p) in path.iter().enumerate() {
                if i > 0 {
                    out.push('.');
                }
                emit_key(out, p);
            }
            out.push_str("]\n");
            emit_table(out, path, sub);
            path.pop();
        }
    }
}

// -------------------------------------------------------------------- parser

struct Cursor<'a> {
    s: &'a [u8],
    src: &'a str,
    i: usize,
}

fn err<T>(msg: impl Into<String>) -> Result<T, CodecError> {
    Err(CodecError::new(msg))
}

impl<'a> Cursor<'a> {
    fn peek(&self) -> Option<u8> {
        self.s.get(self.i).copied()
    }
    fn skip_ws(&mut self) {
        while matches!(self.peek(), Some(b' ' | b'\t')) {
            self.i += 1;
        }
    }
    fn expect(&mut self, c: u8) -> Result<(), CodecError> {
        self.skip_ws();
        if self.peek() == Some(c) {
            self.i += 1;
            Ok(())
        } else {
            err(format!("expected '{}' at byte {}", c as char, self.i))
        }
    }

    fn key(&mut self) -> Result<String, CodecError> {
        self.skip_ws();
        if self.peek() == Some(b'"') {
            return self.string();
        }
        let start = self.i;
        while matches!(self.peek(), Some(b) if b.is_ascii_alphanumeric() || b == b'_' || b == b'-') {
            self.i += 1;
        }
        if start == self.i {
            return err(format!("expected key at byte {start}"));
        }
        Ok(self.src[start..self.i].to_owned())
    }

    fn string(&mut self) -> Result<String, CodecError> {
        self.expect(b'"')?;
        let mut out = String::new();
        loop {
            let rest = &self.src[self.i..];
            let Some(c) = rest.chars().next() else {
                return err("unterminated string");
            };
            self.i += c.len_utf8();
            match c {
                '"' => return Ok(out),
                '\\' => {
                    let Some(e) = self.src[self.i..].chars().next() else {
                        return err("bad escape");
                    };
                    self.i += e.len_utf8();
                    match e {
                        '"' => out.push('"'),
                        '\\' => out.push('\\'),
                        'n' => out.push('\n'),
                        'r' => out.push('\r'),
                        't' => out.push('\t'),
                        'u' => {
                            let hex = self.src.get(self.i..self.i + 4).unwrap_or("");
                            let cp = u32::from_str_radix(hex, 16)
                                .map_err(|_| CodecError::new("bad \\u escape"))?;
                            out.push(char::from_u32(cp).ok_or_else(|| CodecError::new("bad codepoint"))?);
                            self.i += 4;
                        }
                        _ => return err(format!("unknown escape \\{e}")),
                    }
                }
                c => out.push(c),
            }
        }
    }

    fn value(&mut self) -> Result<Value, CodecError> {
        self.skip_ws();
        match self.peek() {
            Some(b'"') => self.string().map(Value::Str),
            Some(b'[') => {
                self.i += 1;
                let mut a = Vec::new();
                loop {
                    self.skip_ws();
                    if self.peek() == Some(b']') {
                        self.i += 1;
                        return Ok(Value::Array(a));
                    }
                    if !a.is_empty() {
                        self.expect(b',')?;
                    }
                    a.push(self.value()?);
                }
            }
            Some(b'{') => {
                self.i += 1;
                let mut t = BTreeMap::new();
                loop {
                    self.skip_ws();
                    if self.peek() == Some(b'}') {
                        self.i += 1;
                        return Ok(Value::Table(t));
                    }
                    if !t.is_empty() {
                        self.expect(b',')?;
                    }
                    let k = self.key()?;
                    self.expect(b'=')?;
                    let v = self.value()?;
                    if t.insert(k.clone(), v).is_some() {
                        return err(format!("duplicate key {k}"));
                    }
                }
            }
            Some(_) => {
                let start = self.i;
                while matches!(self.peek(), Some(b) if !matches!(b, b',' | b']' | b'}' | b' ' | b'\t' | b'#' | b'\n' | b'\r'))
                {
                    self.i += 1;
                }
                scalar(&self.src[start..self.i])
            }
            None => err("unexpected end of input"),
        }
    }
}

fn scalar(tok: &str) -> Result<Value, CodecError> {
    match tok {
        "true" => return Ok(Value::Bool(true)),
        "false" => return Ok(Value::Bool(false)),
        "nan" | "+nan" | "-nan" => return Ok(Value::Float(f64::NAN)),
        "inf" | "+inf" => return Ok(Value::Float(f64::INFINITY)),
        "-inf" => return Ok(Value::Float(f64::NEG_INFINITY)),
        _ => {}
    }
    if let Ok(i) = tok.parse::<i64>() {
        return Ok(Value::Int(i));
    }
    if tok.bytes().any(|b| b.is_ascii_digit())
        && let Ok(f) = tok.parse::<f64>()
    {
        return Ok(Value::Float(f));
    }
    err(format!("bad scalar '{tok}'"))
}

/// Parse TOML text produced by [`to_toml`] (or hand-written in the same subset).
pub fn from_toml(text: &str) -> Result<BTreeMap<String, Value>, CodecError> {
    let mut root: BTreeMap<String, Value> = BTreeMap::new();
    let mut path: Vec<String> = Vec::new();
    for (n, line) in text.lines().enumerate() {
        let mut c = Cursor { s: line.as_bytes(), src: line, i: 0 };
        let ctx = |e: CodecError| CodecError::new(format!("line {}: {}", n + 1, e.0));
        c.skip_ws();
        match c.peek() {
            None | Some(b'#') => continue,
            Some(b'[') => {
                c.i += 1;
                path.clear();
                loop {
                    path.push(c.key().map_err(ctx)?);
                    c.skip_ws();
                    match c.peek() {
                        Some(b'.') => c.i += 1,
                        Some(b']') => break,
                        _ => return Err(ctx(CodecError::new("bad table header"))),
                    }
                }
                // Create the table now so empty tables survive.
                table_at(&mut root, &path).map_err(ctx)?;
            }
            Some(_) => {
                let k = c.key().map_err(ctx)?;
                c.expect(b'=').map_err(ctx)?;
                let v = c.value().map_err(ctx)?;
                c.skip_ws();
                if !matches!(c.peek(), None | Some(b'#')) {
                    return Err(ctx(CodecError::new("trailing characters")));
                }
                let t = table_at(&mut root, &path).map_err(ctx)?;
                if t.insert(k.clone(), v).is_some() {
                    return Err(ctx(CodecError::new(format!("duplicate key {k}"))));
                }
            }
        }
    }
    Ok(root)
}

fn table_at<'a>(
    root: &'a mut BTreeMap<String, Value>,
    path: &[String],
) -> Result<&'a mut BTreeMap<String, Value>, CodecError> {
    let mut cur = root;
    for p in path {
        let e = cur.entry(p.clone()).or_insert_with(|| Value::Table(BTreeMap::new()));
        match e {
            Value::Table(t) => cur = t,
            _ => return err(format!("key {p} is not a table")),
        }
    }
    Ok(cur)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_nested() {
        let mut inner = BTreeMap::new();
        inner.insert("x".to_owned(), Value::Float(-0.0));
        inner.insert("weird key".to_owned(), Value::Str("a\"b\\c\nd\u{1}é".into()));
        let mut root = BTreeMap::new();
        root.insert("n".to_owned(), Value::Int(-7));
        root.insert("f".to_owned(), Value::Float(1e-7));
        root.insert("big".to_owned(), Value::Float(1e300));
        root.insert("arr".to_owned(), Value::Array(vec![Value::Int(1), Value::Table(inner.clone()), Value::Array(vec![])]));
        root.insert("empty".to_owned(), Value::Table(BTreeMap::new()));
        root.insert("sub".to_owned(), Value::Table(BTreeMap::from([("deep".to_owned(), Value::Table(inner))])));
        let text = to_toml(&root);
        let back = from_toml(&text).expect("parse");
        assert_eq!(back, root, "{text}");
        assert_eq!(to_toml(&back), text);
    }

    #[test]
    fn rejects_garbage() {
        assert!(from_toml("a = ").is_err());
        assert!(from_toml("a = 1\na = 2").is_err());
        assert!(from_toml("a = 1 2").is_err());
    }
}
