//! Canonical TOML for [`Value`] tables.
//!
//! The emitter is the canonical form that node hashes are computed over: keys sorted,
//! scalars and arrays (always inline) before sub-tables, sub-tables as `[a.b]` headers,
//! tables inside arrays inline, floats in shortest round-trip form, one fixed string
//! escaping. The parser accepts that output plus ordinary hand-edited TOML of the same
//! subset (comments, whitespace, dotted keys, multi-line arrays; no datetimes, no `[[x]]`).

use crate::value::{Table, Value};
use std::collections::BTreeSet;

/// TOML parse or emit error.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("toml error at line {line}: {msg}")]
pub struct TomlError {
    /// 1-based line number (0 for emit errors).
    pub line: usize,
    /// What went wrong.
    pub msg: String,
}

/// Emit the canonical TOML text of a table.
pub fn to_string(t: &Table) -> String {
    let mut out = String::new();
    emit_table(&mut out, &mut Vec::new(), t);
    out
}

fn bare(k: &str) -> bool {
    !k.is_empty()
        && k.bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-')
}

fn push_key(out: &mut String, k: &str) {
    if bare(k) {
        out.push_str(k);
    } else {
        push_str(out, k);
    }
}

fn push_str(out: &mut String, s: &str) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 || c as u32 == 0x7f => {
                out.push_str(&format!("\\u{:04X}", c as u32));
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

fn inline(out: &mut String, v: &Value) {
    match v {
        Value::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
        Value::Int(i) => out.push_str(&i.to_string()),
        Value::Float(f) => {
            let s = format!("{f:?}");
            out.push_str(&s);
        }
        Value::Str(s) => push_str(out, s),
        Value::Array(a) => {
            out.push('[');
            for (i, x) in a.iter().enumerate() {
                if i > 0 {
                    out.push_str(", ");
                }
                inline(out, x);
            }
            out.push(']');
        }
        Value::Table(t) => {
            if t.is_empty() {
                out.push_str("{}");
                return;
            }
            out.push_str("{ ");
            for (i, (k, x)) in t.iter().enumerate() {
                if i > 0 {
                    out.push_str(", ");
                }
                push_key(out, k);
                out.push_str(" = ");
                inline(out, x);
            }
            out.push_str(" }");
        }
    }
}

fn emit_table(out: &mut String, path: &mut Vec<String>, t: &Table) {
    for (k, v) in t {
        if !matches!(v, Value::Table(_)) {
            push_key(out, k);
            out.push_str(" = ");
            inline(out, v);
            out.push('\n');
        }
    }
    for (k, v) in t {
        if let Value::Table(sub) = v {
            path.push(k.clone());
            if !out.is_empty() {
                out.push('\n');
            }
            out.push('[');
            for (i, p) in path.iter().enumerate() {
                if i > 0 {
                    out.push('.');
                }
                push_key(out, p);
            }
            out.push_str("]\n");
            emit_table(out, path, sub);
            path.pop();
        }
    }
}

/// Parse TOML text into a table.
pub fn from_str(s: &str) -> Result<Table, TomlError> {
    let mut p = Parser {
        c: s.chars().collect(),
        i: 0,
        line: 1,
    };
    p.document()
}

struct Parser {
    c: Vec<char>,
    i: usize,
    line: usize,
}

impl Parser {
    fn err<T>(&self, msg: impl Into<String>) -> Result<T, TomlError> {
        Err(TomlError {
            line: self.line,
            msg: msg.into(),
        })
    }

    fn peek(&self) -> Option<char> {
        self.c.get(self.i).copied()
    }

    fn bump(&mut self) -> Option<char> {
        let c = self.peek()?;
        self.i += 1;
        if c == '\n' {
            self.line += 1;
        }
        Some(c)
    }

    fn ws(&mut self) {
        while matches!(self.peek(), Some(' ' | '\t')) {
            self.i += 1;
        }
    }

    fn comment(&mut self) {
        if self.peek() == Some('#') {
            while !matches!(self.peek(), None | Some('\n')) {
                self.i += 1;
            }
        }
    }

    /// Skip whitespace, comments and newlines.
    fn skip_all(&mut self) {
        loop {
            self.ws();
            self.comment();
            match self.peek() {
                Some('\n' | '\r') => {
                    self.bump();
                }
                _ => break,
            }
        }
    }

    fn end_of_line(&mut self) -> Result<(), TomlError> {
        self.ws();
        self.comment();
        match self.peek() {
            None => Ok(()),
            Some('\n') => {
                self.bump();
                Ok(())
            }
            Some('\r') => {
                self.bump();
                Ok(())
            }
            Some(c) => self.err(format!("unexpected `{c}`")),
        }
    }

    fn document(&mut self) -> Result<Table, TomlError> {
        let mut root = Table::new();
        let mut cur: Vec<String> = Vec::new();
        let mut defined: BTreeSet<Vec<String>> = BTreeSet::new();
        loop {
            self.skip_all();
            match self.peek() {
                None => return Ok(root),
                Some('[') => {
                    self.bump();
                    if self.peek() == Some('[') {
                        return self.err("arrays of tables are not supported");
                    }
                    let path = self.key_path()?;
                    self.ws();
                    if self.bump() != Some(']') {
                        return self.err("expected `]`");
                    }
                    self.end_of_line()?;
                    if !defined.insert(path.clone()) {
                        return self.err("duplicate table header");
                    }
                    table_at(&mut root, &path).map_err(|m| TomlError {
                        line: self.line,
                        msg: m,
                    })?;
                    cur = path;
                }
                Some(_) => {
                    let mut path = self.key_path()?;
                    self.ws();
                    if self.bump() != Some('=') {
                        return self.err("expected `=`");
                    }
                    self.ws();
                    let v = self.value()?;
                    self.end_of_line()?;
                    let key = path.pop().expect("key_path is non-empty");
                    let mut full = cur.clone();
                    full.extend(path);
                    let t = table_at(&mut root, &full).map_err(|m| TomlError {
                        line: self.line,
                        msg: m,
                    })?;
                    if t.insert(key, v).is_some() {
                        return self.err("duplicate key");
                    }
                }
            }
        }
    }

    fn key_path(&mut self) -> Result<Vec<String>, TomlError> {
        let mut path = Vec::new();
        loop {
            self.ws();
            let k = match self.peek() {
                Some('"') => self.string()?,
                _ => {
                    let mut s = String::new();
                    while let Some(c) = self.peek() {
                        if c.is_ascii_alphanumeric() || c == '_' || c == '-' {
                            s.push(c);
                            self.i += 1;
                        } else {
                            break;
                        }
                    }
                    if s.is_empty() {
                        return self.err("expected key");
                    }
                    s
                }
            };
            path.push(k);
            self.ws();
            if self.peek() == Some('.') {
                self.i += 1;
            } else {
                return Ok(path);
            }
        }
    }

    fn string(&mut self) -> Result<String, TomlError> {
        self.bump(); // opening quote
        let mut s = String::new();
        loop {
            match self.bump() {
                None | Some('\n') => return self.err("unterminated string"),
                Some('"') => return Ok(s),
                Some('\\') => {
                    let c = match self.bump() {
                        Some('b') => '\u{8}',
                        Some('t') => '\t',
                        Some('n') => '\n',
                        Some('f') => '\u{c}',
                        Some('r') => '\r',
                        Some('"') => '"',
                        Some('\\') => '\\',
                        Some(u @ ('u' | 'U')) => {
                            let n = if u == 'u' { 4 } else { 8 };
                            let mut code = 0u32;
                            for _ in 0..n {
                                let d = self.bump().and_then(|d| d.to_digit(16));
                                match d {
                                    Some(d) => code = code.wrapping_mul(16).wrapping_add(d),
                                    None => return self.err("bad unicode escape"),
                                }
                            }
                            match char::from_u32(code) {
                                Some(c) => c,
                                None => return self.err("invalid unicode scalar"),
                            }
                        }
                        _ => return self.err("bad escape"),
                    };
                    s.push(c);
                }
                Some(c) => s.push(c),
            }
        }
    }

    fn value(&mut self) -> Result<Value, TomlError> {
        match self.peek() {
            Some('"') => Ok(Value::Str(self.string()?)),
            Some('[') => {
                self.bump();
                let mut a = Vec::new();
                loop {
                    self.skip_all();
                    if self.peek() == Some(']') {
                        self.bump();
                        return Ok(Value::Array(a));
                    }
                    a.push(self.value()?);
                    self.skip_all();
                    match self.peek() {
                        Some(',') => {
                            self.bump();
                        }
                        Some(']') => {}
                        _ => return self.err("expected `,` or `]`"),
                    }
                }
            }
            Some('{') => {
                self.bump();
                let mut t = Table::new();
                self.ws();
                if self.peek() == Some('}') {
                    self.bump();
                    return Ok(Value::Table(t));
                }
                loop {
                    self.ws();
                    let mut path = self.key_path()?;
                    self.ws();
                    if self.bump() != Some('=') {
                        return self.err("expected `=`");
                    }
                    self.ws();
                    let v = self.value()?;
                    let key = path.pop().expect("key_path is non-empty");
                    let dst = table_at(&mut t, &path).map_err(|m| TomlError {
                        line: self.line,
                        msg: m,
                    })?;
                    if dst.insert(key, v).is_some() {
                        return self.err("duplicate key");
                    }
                    self.ws();
                    match self.bump() {
                        Some(',') => {}
                        Some('}') => return Ok(Value::Table(t)),
                        _ => return self.err("expected `,` or `}`"),
                    }
                }
            }
            Some(_) => {
                let mut tok = String::new();
                while let Some(c) = self.peek() {
                    if matches!(c, ',' | ']' | '}' | ' ' | '\t' | '\r' | '\n' | '#') {
                        break;
                    }
                    tok.push(c);
                    self.i += 1;
                }
                match tok.as_str() {
                    "true" => Ok(Value::Bool(true)),
                    "false" => Ok(Value::Bool(false)),
                    "" => self.err("expected value"),
                    t if t.contains(['.', 'e', 'E']) => match t.parse::<f64>() {
                        Ok(f) if f.is_finite() => Ok(Value::Float(f)),
                        _ => self.err(format!("bad float `{t}`")),
                    },
                    t => match t.parse::<i64>() {
                        Ok(i) => Ok(Value::Int(i)),
                        Err(_) => self.err(format!("bad value `{t}`")),
                    },
                }
            }
            None => self.err("expected value"),
        }
    }
}

/// Walk (creating) nested tables along `path`.
fn table_at<'a>(root: &'a mut Table, path: &[String]) -> Result<&'a mut Table, String> {
    let mut t = root;
    for k in path {
        let e = t
            .entry(k.clone())
            .or_insert_with(|| Value::Table(Table::new()));
        match e {
            Value::Table(sub) => t = sub,
            _ => return Err(format!("`{k}` is not a table")),
        }
    }
    Ok(t)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Table {
        let mut inner = Table::new();
        inner.insert("k y".into(), Value::Str("a\"b\\c\n\t\u{1}é".into()));
        inner.insert("f".into(), Value::Float(1e21));
        inner.insert("g".into(), Value::Float(-0.0));
        inner.insert("e".into(), Value::Table(Table::new()));
        let mut t = Table::new();
        t.insert("n".into(), Value::Int(-5));
        t.insert("b".into(), Value::Bool(true));
        t.insert(
            "arr".into(),
            Value::Array(vec![
                Value::Int(1),
                Value::Float(2.0),
                Value::Table(inner.clone()),
                Value::Array(vec![]),
            ]),
        );
        t.insert("sub".into(), Value::Table(inner));
        t
    }

    #[test]
    fn roundtrip_and_canonical() {
        let t = sample();
        let s = to_string(&t);
        let back = from_str(&s).unwrap();
        assert_eq!(back, t);
        assert_eq!(to_string(&back), s);
    }

    #[test]
    fn hand_written() {
        let t = from_str("# c\na = 1 # x\n[b.c]\nd = [\n 1,\n 2,\n]\n").unwrap();
        assert_eq!(t["a"], Value::Int(1));
        assert_eq!(
            t["b"]
                .get("c")
                .unwrap()
                .get("d")
                .unwrap()
                .as_array()
                .unwrap()
                .len(),
            2
        );
        assert!(from_str("a = 1\na = 2").is_err());
        assert!(from_str("[[x]]").is_err());
    }
}
