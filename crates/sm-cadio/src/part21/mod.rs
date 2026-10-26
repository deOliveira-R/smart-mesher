//! ISO 10303-21 (Part 21) tokenizer and parser.
//!
//! [`parse`] turns the bytes of a clear-text exchange structure into a [`StepFile`]: the header
//! records and a `BTreeMap` of data entities by id. It is hand-written (no EXPRESS code
//! generation) so that every error carries a `line:col` and the nesting depth is capped.
//!
//! Accepted lexical forms: `/* */` comments anywhere between tokens, multi-line records, forward
//! references, complex entities `#5=(A() B(..));`, typed parameters `LENGTH_MEASURE(1.)`, strings
//! with `''` and the `\X2\..\X0\`, `\X4\..\X0\`, `\X\HH` and `\\` escapes (`\S\` and `\P?\` are
//! left raw), reals such as `1.`, `-.5` and `1.E-3` (parsed with `str::parse::<f64>`, so
//! correctly rounded), enumerations `.T.`, `$` and `*`. Only a single `DATA` section is supported.
//!
//! Limits: parameter lists nest at most [`MAX_DEPTH`] deep and the input is at most
//! [`MAX_INPUT_BYTES`] long.

use std::collections::BTreeMap;

use crate::error::StepError;

/// Maximum nesting of parameter lists and typed parameters.
pub const MAX_DEPTH: usize = 256;
/// Maximum input size (2 GiB).
pub const MAX_INPUT_BYTES: u64 = 2 << 30;

/// One parameter value of a record.
#[derive(Clone, Debug, PartialEq)]
pub enum Param {
    /// `$`: omitted.
    Null,
    /// `*`: derived.
    Derived,
    /// An integer.
    Int(i64),
    /// A real number.
    Real(f64),
    /// A string, escapes decoded.
    Str(String),
    /// An enumeration value without the dots, as written (`T`, `MILLI`).
    Enum(String),
    /// An instance reference `#n`.
    Ref(u64),
    /// A parenthesized list.
    List(Vec<Param>),
    /// A typed parameter such as `LENGTH_MEASURE(25.4)`; the name is upper-cased.
    Typed(String, Box<Param>),
}

/// One record `NAME(params)`.
#[derive(Clone, Debug, PartialEq)]
pub struct Rec {
    /// Upper-cased keyword.
    pub name: String,
    /// Parameters.
    pub params: Vec<Param>,
    /// Byte offset of the keyword in the input.
    pub offset: usize,
}

/// A data entity: a simple record or a complex entity (several partial records).
#[derive(Clone, Debug, PartialEq)]
pub enum Entity {
    /// `#n = NAME(...);`
    Simple(Rec),
    /// `#n = (A() B(...));`, partial records in file order.
    Complex(Vec<Rec>),
}

impl Entity {
    /// The first record (for display and sorting).
    pub fn first(&self) -> Option<&Rec> {
        match self {
            Entity::Simple(r) => Some(r),
            Entity::Complex(v) => v.first(),
        }
    }

    /// The record named `name` (upper-case), looking into complex entities.
    pub fn rec(&self, name: &str) -> Option<&Rec> {
        match self {
            Entity::Simple(r) => (r.name == name).then_some(r),
            Entity::Complex(v) => v.iter().find(|r| r.name == name),
        }
    }

    /// Whether the entity is (or, for a complex entity, contains) a record named `name`.
    pub fn is(&self, name: &str) -> bool {
        self.rec(name).is_some()
    }

    /// A display name: the keyword, or the keywords joined by `+` for a complex entity.
    pub fn display_name(&self) -> String {
        match self {
            Entity::Simple(r) => r.name.clone(),
            Entity::Complex(v) => v.iter().map(|r| r.name.as_str()).collect::<Vec<_>>().join("+"),
        }
    }
}

/// The header section records `FILE_DESCRIPTION`, `FILE_NAME` and `FILE_SCHEMA`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Header {
    /// `FILE_DESCRIPTION.description`.
    pub description: Vec<String>,
    /// `FILE_DESCRIPTION.implementation_level`.
    pub implementation_level: String,
    /// `FILE_NAME.name`.
    pub name: String,
    /// `FILE_NAME.time_stamp`.
    pub time_stamp: String,
    /// `FILE_NAME.author`.
    pub author: Vec<String>,
    /// `FILE_NAME.organization`.
    pub organization: Vec<String>,
    /// `FILE_NAME.preprocessor_version`.
    pub preprocessor_version: String,
    /// `FILE_NAME.originating_system`.
    pub originating_system: String,
    /// `FILE_NAME.authorization`.
    pub authorization: String,
    /// `FILE_SCHEMA.schema_identifiers`.
    pub schemas: Vec<String>,
}

/// A parsed exchange file.
#[derive(Clone, Debug, PartialEq)]
pub struct StepFile {
    /// The header records.
    pub header: Header,
    /// Data entities by id, ascending.
    pub entities: BTreeMap<u64, Entity>,
}

/// Parse a Part 21 file.
///
/// # Errors
/// [`StepError::Syntax`], [`StepError::Truncated`], [`StepError::TooDeep`] and
/// [`StepError::TooLarge`], each with `line:col` (or the size) of the offence.
pub fn parse(bytes: &[u8]) -> Result<StepFile, StepError> {
    if bytes.len() as u64 > MAX_INPUT_BYTES {
        return Err(StepError::TooLarge { bytes: bytes.len() as u64, max: MAX_INPUT_BYTES });
    }
    let mut p = Parser { b: bytes, pos: 0, scratch: Vec::new() };
    p.file()
}

/// 1-based `(line, column)` of byte offset `pos` (columns count bytes).
pub fn line_col(bytes: &[u8], pos: usize) -> (usize, usize) {
    let pos = pos.min(bytes.len());
    let mut line = 1;
    let mut last = 0;
    for (i, &c) in bytes[..pos].iter().enumerate() {
        if c == b'\n' {
            line += 1;
            last = i + 1;
        }
    }
    (line, pos - last + 1)
}

struct Parser<'a> {
    b: &'a [u8],
    pos: usize,
    /// Reused element stack so lists are allocated with exact capacity.
    scratch: Vec<Param>,
}

fn is_ident_start(c: u8) -> bool {
    c.is_ascii_alphabetic() || c == b'_'
}

fn is_ident(c: u8) -> bool {
    c.is_ascii_alphanumeric() || c == b'_'
}

impl<'a> Parser<'a> {
    fn syntax(&self, pos: usize, msg: impl Into<String>) -> StepError {
        let (line, col) = line_col(self.b, pos);
        StepError::Syntax { line, col, msg: msg.into() }
    }

    fn truncated(&self, msg: impl Into<String>) -> StepError {
        let (line, col) = line_col(self.b, self.b.len());
        StepError::Truncated { line, col, msg: msg.into() }
    }

    fn peek(&self) -> Option<u8> {
        self.b.get(self.pos).copied()
    }

    /// Skip whitespace and comments.
    fn ws(&mut self) -> Result<(), StepError> {
        loop {
            match self.peek() {
                Some(c) if c.is_ascii_whitespace() => self.pos += 1,
                Some(b'/') if self.b.get(self.pos + 1) == Some(&b'*') => {
                    let start = self.pos;
                    self.pos += 2;
                    loop {
                        match self.b.get(self.pos) {
                            None => {
                                let _ = start;
                                return Err(self.truncated("unterminated comment"));
                            }
                            Some(b'*') if self.b.get(self.pos + 1) == Some(&b'/') => {
                                self.pos += 2;
                                break;
                            }
                            Some(_) => self.pos += 1,
                        }
                    }
                }
                _ => return Ok(()),
            }
        }
    }

    fn expect(&mut self, c: u8, what: &str) -> Result<(), StepError> {
        self.ws()?;
        match self.peek() {
            Some(x) if x == c => {
                self.pos += 1;
                Ok(())
            }
            Some(x) => Err(self.syntax(self.pos, format!("expected {what}, found {}", show(x)))),
            None => Err(self.truncated(format!("expected {what}"))),
        }
    }

    /// A keyword (identifier), upper-cased. The current byte must start one.
    fn keyword(&mut self) -> Result<String, StepError> {
        self.ws()?;
        let start = self.pos;
        match self.peek() {
            Some(c) if is_ident_start(c) => {}
            Some(c) => return Err(self.syntax(start, format!("expected a keyword, found {}", show(c)))),
            None => return Err(self.truncated("expected a keyword")),
        }
        while self.peek().is_some_and(is_ident) {
            self.pos += 1;
        }
        let mut s = String::with_capacity(self.pos - start);
        s.extend(self.b[start..self.pos].iter().map(|c| c.to_ascii_uppercase() as char));
        Ok(s)
    }

    /// Match `word` case-insensitively at the cursor (after whitespace) as a whole keyword.
    fn eat_keyword(&mut self, word: &str) -> Result<bool, StepError> {
        self.ws()?;
        let n = word.len();
        let end = self.pos + n;
        if end <= self.b.len() && self.b[self.pos..end].eq_ignore_ascii_case(word.as_bytes()) && !self.b.get(end).copied().is_some_and(is_ident) {
            self.pos = end;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn file(&mut self) -> Result<StepFile, StepError> {
        self.ws()?;
        if self.b.is_empty() {
            return Err(self.truncated("empty input, expected ISO-10303-21;"));
        }
        let start = self.pos;
        if !self.eat_keyword("ISO-10303-21")? {
            return Err(self.syntax(start, "not a STEP file: expected `ISO-10303-21;`"));
        }
        self.expect(b';', "`;` after ISO-10303-21")?;
        if !self.eat_keyword("HEADER")? {
            let at = self.pos;
            return Err(self.syntax(at, "expected `HEADER;`"));
        }
        self.expect(b';', "`;` after HEADER")?;
        let mut header = Header::default();
        loop {
            self.ws()?;
            if self.eat_keyword("ENDSEC")? {
                self.expect(b';', "`;` after ENDSEC")?;
                break;
            }
            let rec = self.record()?;
            self.expect(b';', "`;` after a header record")?;
            fill_header(&mut header, &rec);
        }
        if !self.eat_keyword("DATA")? {
            let at = self.pos;
            if self.peek().is_none() {
                return Err(self.truncated("expected `DATA;`"));
            }
            return Err(self.syntax(at, "expected `DATA;`"));
        }
        self.ws()?;
        if self.peek() == Some(b'(') {
            return Err(self.syntax(self.pos, "DATA sections with parameters (edition 3) are not supported"));
        }
        self.expect(b';', "`;` after DATA")?;
        let mut entities = BTreeMap::new();
        loop {
            self.ws()?;
            match self.peek() {
                None => return Err(self.truncated("expected an entity or ENDSEC;")),
                Some(b'#') => {
                    let at = self.pos;
                    let (id, ent) = self.entity()?;
                    if entities.insert(id, ent).is_some() {
                        return Err(self.syntax(at, format!("duplicate entity #{id}")));
                    }
                }
                Some(_) => {
                    if self.eat_keyword("ENDSEC")? {
                        self.expect(b';', "`;` after ENDSEC")?;
                        break;
                    }
                    return Err(self.syntax(self.pos, "expected `#id = ...;` or ENDSEC;"));
                }
            }
        }
        self.ws()?;
        if self.peek().is_none() {
            return Err(self.truncated("expected END-ISO-10303-21;"));
        }
        if self.peek().is_some_and(|c| c == b'D' || c == b'd') && self.eat_keyword("DATA")? {
            return Err(self.syntax(self.pos - 4, "multiple DATA sections are not supported"));
        }
        if !self.eat_keyword("END-ISO-10303-21")? {
            return Err(self.syntax(self.pos, "expected `END-ISO-10303-21;`"));
        }
        self.expect(b';', "`;` after END-ISO-10303-21")?;
        Ok(StepFile { header, entities })
    }

    /// `#id = record;` or `#id = (record record ...);`
    fn entity(&mut self) -> Result<(u64, Entity), StepError> {
        let id = self.reference()?;
        self.expect(b'=', "`=` after the entity id")?;
        self.ws()?;
        let ent = if self.peek() == Some(b'(') {
            self.pos += 1;
            let mut recs = Vec::new();
            loop {
                self.ws()?;
                match self.peek() {
                    Some(b')') => {
                        self.pos += 1;
                        break;
                    }
                    Some(_) => recs.push(self.record()?),
                    None => return Err(self.truncated("unterminated complex entity")),
                }
            }
            if recs.is_empty() {
                return Err(self.syntax(self.pos - 1, "empty complex entity"));
            }
            Entity::Complex(recs)
        } else {
            Entity::Simple(self.record()?)
        };
        self.expect(b';', "`;` after the entity")?;
        Ok((id, ent))
    }

    fn reference(&mut self) -> Result<u64, StepError> {
        self.ws()?;
        let at = self.pos;
        match self.peek() {
            Some(b'#') => self.pos += 1,
            Some(c) => return Err(self.syntax(at, format!("expected `#`, found {}", show(c)))),
            None => return Err(self.truncated("expected an instance reference")),
        }
        let ds = self.pos;
        let mut v: u64 = 0;
        while let Some(c) = self.peek().filter(u8::is_ascii_digit) {
            v = v.checked_mul(10).and_then(|v| v.checked_add(u64::from(c - b'0'))).ok_or_else(|| self.syntax(at, "instance id out of range"))?;
            self.pos += 1;
        }
        if self.pos == ds {
            return Err(self.syntax(at, "expected digits after `#`"));
        }
        Ok(v)
    }

    /// `NAME ( params )`
    fn record(&mut self) -> Result<Rec, StepError> {
        self.ws()?;
        let offset = self.pos;
        let name = self.keyword()?;
        self.ws()?;
        if self.peek() != Some(b'(') {
            return match self.peek() {
                None => Err(self.truncated(format!("expected `(` after {name}"))),
                Some(c) => Err(self.syntax(self.pos, format!("expected `(` after {name}, found {}", show(c)))),
            };
        }
        let open = self.pos;
        self.pos += 1;
        let params = self.param_list(1, open)?;
        Ok(Rec { name, params, offset })
    }

    /// Parameters up to the closing `)`; the opening one is consumed. `depth` is the nesting
    /// level of this list.
    fn param_list(&mut self, depth: usize, open: usize) -> Result<Vec<Param>, StepError> {
        if depth > MAX_DEPTH {
            let (line, col) = line_col(self.b, open);
            return Err(StepError::TooDeep { line, col, max: MAX_DEPTH });
        }
        let base = self.scratch.len();
        self.ws()?;
        if self.peek() == Some(b')') {
            self.pos += 1;
            return Ok(Vec::new());
        }
        loop {
            let p = match self.param(depth) {
                Ok(p) => p,
                Err(e) => {
                    self.scratch.truncate(base);
                    return Err(e);
                }
            };
            self.scratch.push(p);
            self.ws()?;
            match self.peek() {
                Some(b',') => self.pos += 1,
                Some(b')') => {
                    self.pos += 1;
                    break;
                }
                Some(c) => {
                    self.scratch.truncate(base);
                    return Err(self.syntax(self.pos, format!("expected `,` or `)`, found {}", show(c))));
                }
                None => {
                    self.scratch.truncate(base);
                    return Err(self.truncated("unterminated parameter list"));
                }
            }
        }
        let mut v = Vec::with_capacity(self.scratch.len() - base);
        v.extend(self.scratch.drain(base..));
        Ok(v)
    }

    fn param(&mut self, depth: usize) -> Result<Param, StepError> {
        self.ws()?;
        let at = self.pos;
        let Some(c) = self.peek() else { return Err(self.truncated("expected a parameter")) };
        match c {
            b'$' => {
                self.pos += 1;
                Ok(Param::Null)
            }
            b'*' => {
                self.pos += 1;
                Ok(Param::Derived)
            }
            b'#' => self.reference().map(Param::Ref),
            b'\'' => self.string().map(Param::Str),
            b'.' => {
                // `.T.` enumeration, or a real such as `.5`.
                if self.b.get(self.pos + 1).is_some_and(u8::is_ascii_digit) {
                    self.number()
                } else {
                    self.pos += 1;
                    let s = self.pos;
                    while self.peek().is_some_and(is_ident) {
                        self.pos += 1;
                    }
                    if self.peek() != Some(b'.') {
                        return Err(self.syntax(at, "unterminated enumeration value"));
                    }
                    let name = String::from_utf8_lossy(&self.b[s..self.pos]).into_owned();
                    self.pos += 1;
                    Ok(Param::Enum(name))
                }
            }
            b'(' => {
                self.pos += 1;
                self.param_list(depth + 1, at).map(Param::List)
            }
            b'"' => {
                // Binary literal: kept as its hex digits.
                self.pos += 1;
                let s = self.pos;
                while self.peek().is_some_and(|c| c != b'"') {
                    self.pos += 1;
                }
                if self.peek().is_none() {
                    return Err(self.truncated("unterminated binary literal"));
                }
                let t = String::from_utf8_lossy(&self.b[s..self.pos]).into_owned();
                self.pos += 1;
                Ok(Param::Str(t))
            }
            b'+' | b'-' | b'0'..=b'9' => self.number(),
            c if is_ident_start(c) => {
                let name = self.keyword()?;
                self.ws()?;
                if self.peek() != Some(b'(') {
                    return Err(self.syntax(at, format!("bare keyword {name} as a parameter")));
                }
                let open = self.pos;
                self.pos += 1;
                let mut inner = self.param_list(depth + 1, open)?;
                let p = if inner.len() == 1 { inner.swap_remove(0) } else { Param::List(inner) };
                Ok(Param::Typed(name, Box::new(p)))
            }
            c => Err(self.syntax(at, format!("unexpected {} in a parameter list", show(c)))),
        }
    }

    fn number(&mut self) -> Result<Param, StepError> {
        let at = self.pos;
        let b = self.b;
        let mut i = self.pos;
        if matches!(b.get(i), Some(b'+' | b'-')) {
            i += 1;
        }
        let ds = i;
        while b.get(i).is_some_and(u8::is_ascii_digit) {
            i += 1;
        }
        let int_digits = i - ds;
        let mut real = false;
        let mut frac_digits = 0;
        if b.get(i) == Some(&b'.') {
            real = true;
            i += 1;
            let fs = i;
            while b.get(i).is_some_and(u8::is_ascii_digit) {
                i += 1;
            }
            frac_digits = i - fs;
        }
        if int_digits + frac_digits == 0 {
            return Err(self.syntax(at, "malformed number"));
        }
        if matches!(b.get(i), Some(b'e' | b'E')) {
            let mut j = i + 1;
            if matches!(b.get(j), Some(b'+' | b'-')) {
                j += 1;
            }
            let es = j;
            while b.get(j).is_some_and(u8::is_ascii_digit) {
                j += 1;
            }
            if j == es {
                return Err(self.syntax(at, "malformed exponent"));
            }
            real = true;
            i = j;
        }
        self.pos = i;
        let text = &b[at..i];
        // Number tokens are ASCII by construction.
        let s = std::str::from_utf8(text).map_err(|_| self.syntax(at, "malformed number"))?;
        if !real && let Ok(v) = s.parse::<i64>() {
            return Ok(Param::Int(v));
        }
        s.parse::<f64>().map(Param::Real).map_err(|_| self.syntax(at, "malformed number"))
    }

    fn string(&mut self) -> Result<String, StepError> {
        let at = self.pos;
        self.pos += 1;
        let s = self.pos;
        let mut plain = true;
        loop {
            match self.b.get(self.pos) {
                None => return Err(self.syntax(at, "unterminated string")),
                Some(b'\'') => {
                    if self.b.get(self.pos + 1) == Some(&b'\'') {
                        plain = false;
                        self.pos += 2;
                    } else {
                        break;
                    }
                }
                Some(b'\\') => {
                    plain = false;
                    // `\\` is an escaped backslash; any other directive cannot contain a quote.
                    self.pos += if self.b.get(self.pos + 1) == Some(&b'\\') { 2 } else { 1 };
                }
                Some(_) => self.pos += 1,
            }
        }
        let raw = &self.b[s..self.pos];
        self.pos += 1;
        if plain {
            return Ok(bytes_to_string(raw));
        }
        decode_string(raw).map_err(|m| self.syntax(at, m))
    }
}

fn show(c: u8) -> String {
    if c.is_ascii_graphic() { format!("`{}`", c as char) } else { format!("byte 0x{c:02x}") }
}

fn bytes_to_string(raw: &[u8]) -> String {
    match std::str::from_utf8(raw) {
        Ok(s) => s.to_owned(),
        // ISO 8859-1, the Part 21 default alphabet.
        Err(_) => raw.iter().map(|&c| c as char).collect(),
    }
}

/// Decode a string body with `''` and `\` directives.
fn decode_string(raw: &[u8]) -> Result<String, &'static str> {
    let mut out = String::with_capacity(raw.len());
    let mut i = 0;
    let n = raw.len();
    let hex = |s: &[u8]| -> Option<u32> {
        let mut v = 0u32;
        for &c in s {
            v = v.checked_mul(16)? + (c as char).to_digit(16)?;
        }
        Some(v)
    };
    let mut literal_start = 0;
    macro_rules! flush {
        ($upto:expr) => {
            if literal_start < $upto {
                out.push_str(&bytes_to_string(&raw[literal_start..$upto]));
            }
        };
    }
    while i < n {
        match raw[i] {
            b'\'' => {
                // A doubled quote (the scanner guarantees the pair).
                flush!(i);
                out.push('\'');
                i += 2;
                literal_start = i;
            }
            b'\\' => {
                if raw.get(i + 1) == Some(&b'\\') {
                    flush!(i);
                    out.push('\\');
                    i += 2;
                    literal_start = i;
                } else if raw.get(i + 1) == Some(&b'X') && raw.get(i + 2) == Some(&b'\\') {
                    // \X\HH: one ISO 8859-1 character.
                    let v = raw.get(i + 3..i + 5).and_then(hex).ok_or("malformed \\X\\ escape")?;
                    flush!(i);
                    out.push(char::from_u32(v).ok_or("invalid character in \\X\\ escape")?);
                    i += 5;
                    literal_start = i;
                } else if raw.get(i + 1) == Some(&b'X') && matches!(raw.get(i + 2), Some(b'2' | b'4')) && raw.get(i + 3) == Some(&b'\\') {
                    let width = if raw[i + 2] == b'2' { 4 } else { 8 };
                    flush!(i);
                    let mut j = i + 4;
                    let mut units: Vec<u32> = Vec::new();
                    loop {
                        if raw.get(j..j + 4) == Some(b"\\X0\\") {
                            j += 4;
                            break;
                        }
                        let v = raw.get(j..j + width).and_then(hex).ok_or("malformed \\X2\\ or \\X4\\ escape")?;
                        units.push(v);
                        j += width;
                    }
                    if width == 4 {
                        for ch in char::decode_utf16(units.iter().map(|&u| u as u16)) {
                            out.push(ch.map_err(|_| "invalid UTF-16 in \\X2\\ escape")?);
                        }
                    } else {
                        for u in units {
                            out.push(char::from_u32(u).ok_or("invalid code point in \\X4\\ escape")?);
                        }
                    }
                    i = j;
                    literal_start = i;
                } else {
                    // \S\ and \P?\ (and anything unknown) stay raw.
                    i += 1;
                }
            }
            _ => i += 1,
        }
    }
    flush!(n);
    Ok(out)
}

fn strings(p: &Param) -> Vec<String> {
    match p {
        Param::Str(s) => vec![s.clone()],
        Param::List(v) => v.iter().filter_map(|p| if let Param::Str(s) = p { Some(s.clone()) } else { None }).collect(),
        _ => Vec::new(),
    }
}

fn string(p: Option<&Param>) -> String {
    match p {
        Some(Param::Str(s)) => s.clone(),
        _ => String::new(),
    }
}

fn fill_header(h: &mut Header, r: &Rec) {
    let at = |i: usize| r.params.get(i);
    match r.name.as_str() {
        "FILE_DESCRIPTION" => {
            h.description = at(0).map(strings).unwrap_or_default();
            h.implementation_level = string(at(1));
        }
        "FILE_NAME" => {
            h.name = string(at(0));
            h.time_stamp = string(at(1));
            h.author = at(2).map(strings).unwrap_or_default();
            h.organization = at(3).map(strings).unwrap_or_default();
            h.preprocessor_version = string(at(4));
            h.originating_system = string(at(5));
            h.authorization = string(at(6));
        }
        "FILE_SCHEMA" => h.schemas = at(0).map(strings).unwrap_or_default(),
        _ => {}
    }
}

#[cfg(test)]
mod tests;
