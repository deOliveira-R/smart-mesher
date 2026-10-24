//! Lexer: bytes to tokens with exact spans. Never panics; lexical problems become diagnostics plus
//! [`Tok::Error`] tokens so the parser can keep going.

use crate::diag::Diagnostic;
use crate::span::Span;

/// Token kinds.
#[derive(Clone, Debug, PartialEq)]
pub enum Tok {
    /// Identifier.
    Ident,
    /// `Kind#` immediately followed by a digit or `[`; the span covers `Kind#`.
    EntityRef,
    /// Integer literal.
    Int(i64),
    /// Float literal.
    Float(f64),
    /// String literal, with escapes processed.
    Str(String),
    /// `let`
    Let,
    /// `set`
    Set,
    /// `alias`
    Alias,
    /// `batch`
    Batch,
    /// `if`
    If,
    /// `else`
    Else,
    /// `for`
    For,
    /// `in`
    In,
    /// `include`
    Include,
    /// `print`
    Print,
    /// `assert`
    Assert,
    /// `true`
    True,
    /// `false`
    False,
    /// `and`
    And,
    /// `or`
    Or,
    /// `not`
    Not,
    /// `all`
    All,
    /// `except`
    Except,
    /// `null` (reserved)
    Null,
    /// `(`
    LParen,
    /// `)`
    RParen,
    /// `[`
    LBracket,
    /// `]`
    RBracket,
    /// `{`
    LBrace,
    /// `}`
    RBrace,
    /// `,`
    Comma,
    /// `;`
    Semi,
    /// `:`
    Colon,
    /// `=`
    Assign,
    /// `.`
    Dot,
    /// `..`
    DotDot,
    /// `...`
    Ellipsis,
    /// `+`
    Plus,
    /// `-`
    Minus,
    /// `*`
    Star,
    /// `/`
    Slash,
    /// `%`
    Percent,
    /// `**`
    StarStar,
    /// `==`
    EqEq,
    /// `!=`
    NotEq,
    /// `<`
    Lt,
    /// `<=`
    Le,
    /// `>`
    Gt,
    /// `>=`
    Ge,
    /// `|`
    Pipe,
    /// `&`
    Amp,
    /// A line feed (statement separator).
    Newline,
    /// End of input.
    Eof,
    /// A lexical error (already reported).
    Error,
}

/// A token and where it is.
#[derive(Clone, Debug, PartialEq)]
pub struct Token {
    /// Kind (with payload for literals).
    pub tok: Tok,
    /// Byte span.
    pub span: Span,
}

/// Words reserved for user functions and loops in a later version. They are errors as names today so that adding the
/// syntax later cannot change the meaning of a valid script (`return (x)` would otherwise already parse as a call).
pub const FUTURE_WORDS: [&str; 7] = ["fn", "def", "return", "while", "break", "continue", "import"];

fn keyword(s: &str) -> Option<Tok> {
    Some(match s {
        "let" => Tok::Let,
        "set" => Tok::Set,
        "alias" => Tok::Alias,
        "batch" => Tok::Batch,
        "if" => Tok::If,
        "else" => Tok::Else,
        "for" => Tok::For,
        "in" => Tok::In,
        "include" => Tok::Include,
        "print" => Tok::Print,
        "assert" => Tok::Assert,
        "true" => Tok::True,
        "false" => Tok::False,
        "and" => Tok::And,
        "or" => Tok::Or,
        "not" => Tok::Not,
        "all" => Tok::All,
        "except" => Tok::Except,
        "null" => Tok::Null,
        _ => return None,
    })
}

/// Tokenize `text` (file index `file`). The result always ends with [`Tok::Eof`].
pub fn lex(file: u32, text: &str, diags: &mut Vec<Diagnostic>) -> Vec<Token> {
    Lexer { b: text.as_bytes(), text, i: 0, file, diags }.run()
}

struct Lexer<'a> {
    b: &'a [u8],
    text: &'a str,
    i: usize,
    file: u32,
    diags: &'a mut Vec<Diagnostic>,
}

impl Lexer<'_> {
    fn sp(&self, lo: usize, hi: usize) -> Span {
        Span::new(self.file, lo as u32, hi as u32)
    }

    fn at(&self, k: usize) -> u8 {
        self.b.get(self.i + k).copied().unwrap_or(0)
    }

    fn run(mut self) -> Vec<Token> {
        let mut out: Vec<Token> = Vec::with_capacity(self.b.len() / 3 + 4);
        while self.i < self.b.len() {
            let c = self.b[self.i];
            let lo = self.i;
            let tok = match c {
                b' ' | b'\t' | b'\r' => {
                    self.i += 1;
                    continue;
                }
                b'\n' => {
                    self.i += 1;
                    Tok::Newline
                }
                b'\\' if self.at(1) == b'\n' || (self.at(1) == b'\r' && self.at(2) == b'\n') => {
                    self.i += if self.at(1) == b'\r' { 3 } else { 2 };
                    continue;
                }
                b'/' if self.at(1) == b'/' => {
                    while self.i < self.b.len() && self.b[self.i] != b'\n' {
                        self.i += 1;
                    }
                    continue;
                }
                b'/' if self.at(1) == b'*' => {
                    self.block_comment();
                    continue;
                }
                b'0'..=b'9' => self.number(),
                b'"' => self.string(),
                b'A'..=b'Z' | b'a'..=b'z' | b'_' => self.ident(),
                b'(' => self.one(Tok::LParen),
                b')' => self.one(Tok::RParen),
                b'[' => self.one(Tok::LBracket),
                b']' => self.one(Tok::RBracket),
                b'{' => self.one(Tok::LBrace),
                b'}' => self.one(Tok::RBrace),
                b',' => self.one(Tok::Comma),
                b';' => self.one(Tok::Semi),
                b':' => self.one(Tok::Colon),
                b'+' => self.one(Tok::Plus),
                b'-' => self.one(Tok::Minus),
                b'%' => self.one(Tok::Percent),
                b'|' => self.one(Tok::Pipe),
                b'&' => self.one(Tok::Amp),
                b'/' => self.one(Tok::Slash),
                b'*' => {
                    if self.at(1) == b'*' {
                        self.i += 2;
                        Tok::StarStar
                    } else {
                        self.one(Tok::Star)
                    }
                }
                b'=' => {
                    if self.at(1) == b'=' {
                        self.i += 2;
                        Tok::EqEq
                    } else {
                        self.one(Tok::Assign)
                    }
                }
                b'<' => {
                    if self.at(1) == b'=' {
                        self.i += 2;
                        Tok::Le
                    } else {
                        self.one(Tok::Lt)
                    }
                }
                b'>' => {
                    if self.at(1) == b'=' {
                        self.i += 2;
                        Tok::Ge
                    } else {
                        self.one(Tok::Gt)
                    }
                }
                b'!' if self.at(1) == b'=' => {
                    self.i += 2;
                    Tok::NotEq
                }
                b'.' => {
                    if self.at(1) == b'.' && self.at(2) == b'.' {
                        self.i += 3;
                        Tok::Ellipsis
                    } else if self.at(1) == b'.' {
                        self.i += 2;
                        Tok::DotDot
                    } else {
                        self.one(Tok::Dot)
                    }
                }
                b'#' => {
                    self.i += 1;
                    let after_name = out.last().is_some_and(|t: &Token| t.tok == Tok::Ident && t.span.hi as usize == lo);
                    let help = if after_name {
                        "an id serial after `#` is a number, `[...]` or `(...)`; to build an id from a variable write `Volume#[i]` or `Volume#(i + 1)`"
                    } else {
                        "write entity ids as `Volume#3` (no spaces); comments start with `//`"
                    };
                    self.diags.push(
                        Diagnostic::error("invalid-character", self.sp(lo, self.i), "`#` is only valid right after a kind name and before a serial")
                            .label("unexpected `#`")
                            .help(help),
                    );
                    Tok::Error
                }
                _ => {
                    // Skip one full UTF-8 character.
                    let ch = self.text[self.i..].chars().next().unwrap_or('\u{fffd}');
                    self.i += ch.len_utf8();
                    self.diags.push(
                        Diagnostic::error("invalid-character", self.sp(lo, self.i), format!("unexpected character {ch:?}"))
                            .label("not part of the language"),
                    );
                    Tok::Error
                }
            };
            out.push(Token { tok, span: self.sp(lo, self.i) });
        }
        out.push(Token { tok: Tok::Eof, span: self.sp(self.i, self.i) });
        out
    }

    fn one(&mut self, t: Tok) -> Tok {
        self.i += 1;
        t
    }

    fn block_comment(&mut self) {
        let lo = self.i;
        let mut depth = 0usize;
        while self.i < self.b.len() {
            if self.b[self.i] == b'/' && self.at(1) == b'*' {
                depth += 1;
                self.i += 2;
            } else if self.b[self.i] == b'*' && self.at(1) == b'/' {
                depth -= 1;
                self.i += 2;
                if depth == 0 {
                    return;
                }
            } else {
                self.i += 1;
            }
        }
        self.diags.push(
            Diagnostic::error("unterminated-comment", self.sp(lo, lo + 2), "unterminated block comment")
                .label("comment starts here")
                .help("close it with `*/` (block comments nest)"),
        );
    }

    fn ident(&mut self) -> Tok {
        let lo = self.i;
        while matches!(self.at(0), b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'_') {
            self.i += 1;
        }
        if self.at(0) == b'#' && (self.at(1).is_ascii_digit() || self.at(1) == b'[' || self.at(1) == b'(') {
            self.i += 1;
            return Tok::EntityRef;
        }
        let word = &self.text[lo..self.i];
        if FUTURE_WORDS.contains(&word) {
            // After a single `.` any word is a valid path segment (`b.return`).
            let dotted = lo >= 1 && self.b[lo - 1] == b'.' && !(lo >= 2 && self.b[lo - 2] == b'.');
            if !dotted {
                self.diags.push(
                    Diagnostic::error("reserved-word", self.sp(lo, self.i), format!("`{word}` is reserved for a future version of the language"))
                        .label("cannot be used as a name")
                        .help("choose another name; an object key can be quoted (`{ \"return\": 1 }`)"),
                );
            }
        }
        let kw = keyword(word);
        // `f"x"`: an identifier glued to a string literal is a prefixed string, reserved for a future interpolation syntax.
        if kw.is_none() && self.at(0) == b'"' {
            self.diags.push(
                Diagnostic::error("reserved-prefix", self.sp(lo, self.i), format!("`{word}\"...\"`: prefixed string literals are reserved"))
                    .label("a string prefix is not part of the language")
                    .help(format!("strings are plain text and never interpolate; build the text with `\"...\" + str(...)`, e.g. `\"fuel_\" + str(i)` instead of `{word}\"fuel_{{i}}\"`")),
            );
            return Tok::Error;
        }
        kw.unwrap_or(Tok::Ident)
    }

    fn number(&mut self) -> Tok {
        let lo = self.i;
        while self.at(0).is_ascii_digit() || (self.at(0) == b'_' && self.at(1).is_ascii_digit()) {
            self.i += 1;
        }
        let int_end = self.i;
        // `1.e5`, `1.`: a `.` after the integer part needs a digit (`1..5` is a range and `1.max` a field access).
        if self.at(0) == b'.' && !self.at(1).is_ascii_digit() && self.at(1) != b'.' && (!(self.at(1).is_ascii_alphabetic() || self.at(1) == b'_') || self.exp_after_dot()) {
            self.i += 1;
            let exp_lo = self.i;
            if matches!(self.at(0), b'e' | b'E') {
                self.i += 1;
                if matches!(self.at(0), b'+' | b'-') {
                    self.i += 1;
                }
                while self.at(0).is_ascii_digit() {
                    self.i += 1;
                }
            }
            let int = &self.text[lo..int_end];
            let exp = &self.text[exp_lo..self.i];
            self.diags.push(
                Diagnostic::error("invalid-number", self.sp(lo, self.i), "a float needs a digit after the `.`")
                    .label("no digit after `.`")
                    .help(format!("write `{int}.0{exp}`")),
            );
            return Tok::Error;
        }
        let mut float = false;
        if self.at(0) == b'.' && self.at(1).is_ascii_digit() {
            float = true;
            self.i += 1;
            while self.at(0).is_ascii_digit() || (self.at(0) == b'_' && self.at(1).is_ascii_digit()) {
                self.i += 1;
            }
        }
        if matches!(self.at(0), b'e' | b'E') {
            if self.at(1).is_ascii_digit() || (matches!(self.at(1), b'+' | b'-') && self.at(2).is_ascii_digit()) {
                float = true;
                self.i += 2;
                while self.at(0).is_ascii_digit() {
                    self.i += 1;
                }
            } else if !(self.at(1).is_ascii_alphanumeric() || self.at(1) == b'_') || (matches!(self.at(1), b'+' | b'-') && !self.at(2).is_ascii_digit()) {
                // `1e`, `1e+`: an exponent marker with no digits.
                let sign = matches!(self.at(1), b'+' | b'-');
                self.i += if sign { 2 } else { 1 };
                self.diags.push(
                    Diagnostic::error("invalid-number", self.sp(lo, self.i), "an exponent needs digits")
                        .label("no digits after the exponent marker")
                        .help("write the exponent digits, e.g. `1e5`, `1.5e-3`"),
                );
                return Tok::Error;
            }
        }
        // `1_`, `1__0`, `1_.5`: a digit separator must sit between two digits.
        if self.at(0) == b'_' {
            while self.at(0).is_ascii_alphanumeric() || self.at(0) == b'_' {
                self.i += 1;
            }
            self.diags.push(
                Diagnostic::error("invalid-number", self.sp(lo, self.i), "`_` must separate digits, one at a time")
                    .label("misplaced digit separator")
                    .help("group digits like `1_000` (an underscore between two digits, never doubled or trailing)"),
            );
            return Tok::Error;
        }
        let clean: String = self.text[lo..self.i].chars().filter(|&c| c != '_').collect();
        let span = self.sp(lo, self.i);
        if float {
            match clean.parse::<f64>() {
                Ok(v) if v.is_finite() => Tok::Float(v),
                _ => {
                    self.diags.push(Diagnostic::error("bad-number", span, "number out of range").label("not a finite float"));
                    Tok::Error
                }
            }
        } else {
            match clean.parse::<i64>() {
                Ok(v) => Tok::Int(v),
                Err(_) => {
                    self.diags.push(
                        Diagnostic::error("bad-number", span, "integer literal out of range")
                            .label("does not fit in a 64-bit signed integer")
                            .help("write it as a float (e.g. `1e19`) if it is not an id or count"),
                    );
                    Tok::Error
                }
            }
        }
    }

    /// At a `.` after an integer: does an exponent (`e5`, `E+5`) follow, as in `1.e5`?
    fn exp_after_dot(&self) -> bool {
        matches!(self.at(1), b'e' | b'E') && (self.at(2).is_ascii_digit() || (matches!(self.at(2), b'+' | b'-') && self.at(3).is_ascii_digit()))
    }

    fn string(&mut self) -> Tok {
        let lo = self.i;
        self.i += 1;
        let mut s = String::new();
        // Index of a `{` that must not get the `literal-braces` warning (it follows `\$` or an already reported `${`).
        let mut quiet_brace = usize::MAX;
        loop {
            if self.i >= self.b.len() || self.b[self.i] == b'\n' {
                self.diags.push(
                    Diagnostic::error("unterminated-string", self.sp(lo, self.i), "unterminated string literal")
                        .label("string starts here")
                        .help("strings end with `\"` on the same line"),
                );
                return Tok::Error;
            }
            match self.b[self.i] {
                b'"' => {
                    self.i += 1;
                    return Tok::Str(s);
                }
                b'\\' => {
                    let esc = self.i;
                    self.i += 1;
                    match self.at(0) {
                        b'n' => s.push('\n'),
                        b't' => s.push('\t'),
                        b'r' => s.push('\r'),
                        b'"' => s.push('"'),
                        b'\\' => s.push('\\'),
                        b'$' => {
                            s.push('$');
                            quiet_brace = self.i + 1;
                        }
                        b'u' => {
                            let ok = self.unicode_escape(&mut s);
                            if !ok {
                                self.diags.push(
                                    Diagnostic::error("invalid-escape", self.sp(esc, self.i), "invalid unicode escape")
                                        .label("expected `\\u{HEX}` with a valid scalar value"),
                                );
                            }
                            continue;
                        }
                        _ => {
                            let ch = self.text.get(self.i..).and_then(|t| t.chars().next());
                            let w = ch.map_or(0, char::len_utf8);
                            self.diags.push(
                                Diagnostic::error("invalid-escape", self.sp(esc, self.i + w), "unknown escape sequence")
                                    .label("valid escapes: \\n \\t \\r \\\" \\\\ \\$ \\u{..}"),
                            );
                            self.i += w;
                            continue;
                        }
                    }
                    self.i += 1;
                }
                b'$' if self.at(1) == b'{' => {
                    self.diags.push(
                        Diagnostic::error("reserved-interpolation", self.sp(self.i, self.i + 2), "`${` in a string is reserved")
                            .label("reserved for string interpolation")
                            .help("strings are literal text; write `\\${` for a literal `${`, or build the text with `\"pin_\" + str(i)`"),
                    );
                    s.push('$');
                    quiet_brace = self.i + 1;
                    self.i += 1;
                }
                b'{' => {
                    // `"fuel_{i}"` is literal text in v1 (and stays so); warn, since it is almost certainly meant as interpolation.
                    let mut j = self.i + 1;
                    if matches!(self.b.get(j), Some(b'A'..=b'Z' | b'a'..=b'z' | b'_')) {
                        while matches!(self.b.get(j), Some(b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'_')) {
                            j += 1;
                        }
                        if self.b.get(j) == Some(&b'}') && self.i != quiet_brace {
                            self.diags.push(
                                Diagnostic::warning("literal-braces", self.sp(self.i, j + 1), "this string contains `{name}`, which is literal text, not interpolation")
                                    .label("printed as written")
                                    .help("strings never interpolate; build the text with `+ str(...)`, e.g. `\"fuel_\" + str(i)`"),
                            );
                        }
                    }
                    s.push('{');
                    self.i += 1;
                }
                _ => {
                    let ch = self.text[self.i..].chars().next().unwrap_or('\u{fffd}');
                    s.push(ch);
                    self.i += ch.len_utf8();
                }
            }
        }
    }

    /// At `u` after a backslash. Returns false on a malformed escape (position left after what was read).
    fn unicode_escape(&mut self, out: &mut String) -> bool {
        self.i += 1; // 'u'
        if self.at(0) != b'{' {
            return false;
        }
        self.i += 1;
        let start = self.i;
        while self.at(0).is_ascii_hexdigit() {
            self.i += 1;
        }
        let digits = &self.text[start..self.i];
        if self.at(0) != b'}' || digits.is_empty() || digits.len() > 6 {
            return false;
        }
        self.i += 1;
        match u32::from_str_radix(digits, 16).ok().and_then(char::from_u32) {
            Some(c) => {
                out.push(c);
                true
            }
            None => false,
        }
    }
}
