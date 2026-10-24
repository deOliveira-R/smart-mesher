//! Recursive-descent parser with statement- and argument-level error recovery.
//!
//! Every syntax error is reported with a span; the parser then resynchronizes and keeps going, so
//! one run finds many errors. The tree it returns always exists (broken nodes are `Error`).

use crate::ast::*;
use crate::diag::{Diagnostic, suggest};
use crate::kinds;
use crate::lexer::{Tok, Token, lex};
use crate::span::Span;

/// Limits that keep hostile input cheap.
#[derive(Clone, Copy, Debug)]
pub struct ParseLimits {
    /// Maximum expression/statement nesting.
    pub max_depth: usize,
    /// Maximum height of one expression tree (nesting plus operator chains such as `1 + 1 + ...`).
    pub max_height: u32,
    /// Sources longer than this many bytes are rejected without being scanned.
    pub max_source_bytes: usize,
    /// Stop after this many diagnostics (a final `too-many-errors` is added).
    pub max_errors: usize,
}

impl Default for ParseLimits {
    fn default() -> Self {
        Self { max_depth: 64, max_height: 256, max_source_bytes: 16 << 20, max_errors: 100 }
    }
}

/// Result of parsing one source.
#[derive(Clone, Debug)]
pub struct ParseOutput {
    /// The tree (complete even when there are errors).
    pub script: Script,
    /// Lexical and syntax diagnostics in source order.
    pub diagnostics: Vec<Diagnostic>,
}

impl ParseOutput {
    /// True if any diagnostic is an error.
    pub fn has_errors(&self) -> bool {
        self.diagnostics.iter().any(Diagnostic::is_error)
    }
}

/// Lex and parse `text` as file `file` with default limits.
pub fn parse(file: u32, text: &str) -> ParseOutput {
    parse_with(file, text, ParseLimits::default())
}

/// Lex and parse with explicit limits.
pub fn parse_with(file: u32, text: &str, limits: ParseLimits) -> ParseOutput {
    if text.len() > limits.max_source_bytes || text.len() >= u32::MAX as usize {
        let d = Diagnostic::error("source-too-large", Span::new(file, 0, 0), format!("source is {} bytes, the limit is {}", text.len(), limits.max_source_bytes))
            .label("not parsed");
        return ParseOutput { script: Script { file, stmts: Vec::new() }, diagnostics: vec![d] };
    }
    let mut diags = Vec::new();
    let toks = lex(file, text, &mut diags);
    let mut p = P { toks, pos: 0, text, file, diags, nl: 0, depth: 0, limits, last_hi: 0, aborted: false };
    if p.diags.len() >= limits.max_errors {
        p.aborted = true;
    }
    let stmts = p.script();
    ParseOutput { script: Script { file, stmts }, diagnostics: p.diags }
}

type R<T> = Result<T, ()>;

struct P<'a> {
    toks: Vec<Token>,
    pos: usize,
    text: &'a str,
    file: u32,
    diags: Vec<Diagnostic>,
    /// Newlines are skipped while this is positive (inside `()`/`[]`/object literals).
    nl: u32,
    depth: usize,
    limits: ParseLimits,
    last_hi: u32,
    aborted: bool,
}

fn is_stmt_kw(t: &Tok) -> bool {
    matches!(t, Tok::Let | Tok::Set | Tok::Alias | Tok::Batch | Tok::If | Tok::For | Tok::Include | Tok::Print | Tok::Assert)
}

fn tok_name(t: &Tok) -> String {
    let s = match t {
        Tok::Ident => return "identifier".into(),
        Tok::EntityRef => return "entity reference".into(),
        Tok::Int(_) | Tok::Float(_) => return "number".into(),
        Tok::Str(_) => return "string".into(),
        Tok::Let => "let",
        Tok::Set => "set",
        Tok::Alias => "alias",
        Tok::Batch => "batch",
        Tok::If => "if",
        Tok::Else => "else",
        Tok::For => "for",
        Tok::In => "in",
        Tok::Include => "include",
        Tok::Print => "print",
        Tok::Assert => "assert",
        Tok::True => "true",
        Tok::False => "false",
        Tok::And => "and",
        Tok::Or => "or",
        Tok::Not => "not",
        Tok::All => "all",
        Tok::Except => "except",
        Tok::LParen => "(",
        Tok::RParen => ")",
        Tok::LBracket => "[",
        Tok::RBracket => "]",
        Tok::LBrace => "{",
        Tok::RBrace => "}",
        Tok::Comma => ",",
        Tok::Semi => ";",
        Tok::Colon => ":",
        Tok::Assign => "=",
        Tok::Dot => ".",
        Tok::DotDot => "..",
        Tok::Ellipsis => "...",
        Tok::Plus => "+",
        Tok::Minus => "-",
        Tok::Star => "*",
        Tok::Slash => "/",
        Tok::Percent => "%",
        Tok::StarStar => "**",
        Tok::EqEq => "==",
        Tok::NotEq => "!=",
        Tok::Lt => "<",
        Tok::Le => "<=",
        Tok::Gt => ">",
        Tok::Ge => ">=",
        Tok::Pipe => "|",
        Tok::Amp => "&",
        Tok::Newline => return "end of line".into(),
        Tok::Eof => return "end of input".into(),
        Tok::Error => return "invalid token".into(),
    };
    format!("`{s}`")
}

impl P<'_> {
    // ---- token access -------------------------------------------------------------------

    fn idx(&self) -> usize {
        let mut i = self.pos;
        if self.nl > 0 {
            while self.toks[i].tok == Tok::Newline {
                i += 1;
            }
        }
        i
    }

    fn cur(&self) -> &Token {
        &self.toks[self.idx()]
    }

    fn tok(&self) -> &Tok {
        &self.cur().tok
    }

    /// The `k`-th token after the current one (newlines skipped only when insensitive).
    fn nth(&self, k: usize) -> &Tok {
        let mut i = self.idx();
        for _ in 0..k {
            if self.toks[i].tok == Tok::Eof {
                break;
            }
            i += 1;
            if self.nl > 0 {
                while self.toks[i].tok == Tok::Newline {
                    i += 1;
                }
            }
        }
        &self.toks[i].tok
    }

    fn bump(&mut self) -> Token {
        let i = self.idx();
        let t = self.toks[i].clone();
        if t.tok != Tok::Eof {
            self.pos = i + 1;
        } else {
            self.pos = i;
        }
        self.last_hi = t.span.hi;
        t
    }

    /// Continue onto the next line after an operator or comma, unless a new statement starts there
    /// (then the operator is left dangling and reported at the end of its own line).
    fn skip_nl(&mut self) {
        let mut j = self.pos;
        while self.toks[j].tok == Tok::Newline {
            j += 1;
        }
        if !is_stmt_kw(&self.toks[j].tok) {
            self.pos = j;
        }
    }

    fn skip_seps(&mut self) {
        while matches!(self.toks[self.pos].tok, Tok::Newline | Tok::Semi) {
            self.pos += 1;
        }
    }

    fn text_of(&self, s: Span) -> &str {
        self.text.get(s.lo as usize..s.hi as usize).unwrap_or("")
    }

    fn sp(&self, lo: u32) -> Span {
        Span::new(self.file, lo, self.last_hi.max(lo))
    }

    fn push(&mut self, d: Diagnostic) {
        if self.aborted {
            return;
        }
        let at = d.span;
        self.diags.push(d);
        if self.diags.len() >= self.limits.max_errors {
            self.aborted = true;
            self.diags.push(Diagnostic::error("too-many-errors", at, "too many errors, giving up on this source"));
        }
    }

    fn describe(&self, t: &Token) -> String {
        match t.tok {
            Tok::Ident => format!("identifier `{}`", self.text_of(t.span)),
            _ => tok_name(&t.tok),
        }
    }

    /// Zero-width span just after the previous token (where a missing token belongs).
    fn after_last(&self) -> Span {
        Span::new(self.file, self.last_hi, self.last_hi)
    }

    /// Span to blame for the current token; a line break or end of input is blamed on the end of the previous token.
    fn blame(&self, t: &Token) -> Span {
        if matches!(t.tok, Tok::Newline | Tok::Eof) { self.after_last() } else { t.span }
    }

    /// Report "expected X, found Y" at the current token (silent after a lexical error).
    fn expected<T>(&mut self, what: &str) -> R<T> {
        let t = self.cur().clone();
        if t.tok != Tok::Error {
            let found = self.describe(&t);
            let at = self.blame(&t);
            self.push(Diagnostic::error("expected-token", at, format!("expected {what}, found {found}")).label(format!("expected {what}")));
        }
        Err(())
    }

    fn expect(&mut self, want: Tok, what: &str) -> R<Token> {
        if *self.tok() == want { Ok(self.bump()) } else { self.expected(what) }
    }

    fn expect_ident(&mut self, what: &str) -> R<Ident> {
        if *self.tok() == Tok::Ident {
            let t = self.bump();
            Ok(Ident { name: self.text_of(t.span).to_owned(), span: t.span })
        } else {
            self.expected(what)
        }
    }

    fn enter(&mut self) -> R<()> {
        self.depth += 1;
        if self.depth > self.limits.max_depth {
            self.depth -= 1;
            let span = self.cur().span;
            let max = self.limits.max_depth;
            self.push(Diagnostic::error("too-deep", span, format!("nesting is deeper than the limit of {max}")).label("too deeply nested"));
            return Err(());
        }
        Ok(())
    }

    /// True if a newline separates the current token from the previous one and a statement keyword follows:
    /// the sign of an unclosed bracket.
    fn at_new_statement(&self) -> bool {
        self.idx() > self.pos && is_stmt_kw(&self.toks[self.idx()].tok)
    }

    // ---- statements ---------------------------------------------------------------------

    fn script(&mut self) -> Vec<Stmt> {
        let mut out = Vec::new();
        loop {
            self.skip_seps();
            if self.aborted || *self.tok() == Tok::Eof {
                break;
            }
            out.push(self.stmt_terminated(false));
        }
        out
    }

    fn stmt_terminated(&mut self, in_block: bool) -> Stmt {
        let saved = (self.nl, self.depth);
        let lo = self.cur().span.lo;
        match self.stmt() {
            Ok(s) => {
                match self.tok() {
                    Tok::Newline | Tok::Semi | Tok::Eof => {}
                    Tok::RBrace if in_block => {}
                    _ => {
                        let t = self.cur().clone();
                        if t.tok != Tok::Error {
                            let found = self.describe(&t);
                            let mut d = Diagnostic::error("expected-token", t.span, format!("expected end of statement, found {found}"))
                                .label("statements end at a line break or `;`");
                            if t.tok == Tok::Else {
                                d = d.help("put `else` on the same line as the closing `}` of the `if`");
                            }
                            self.push(d);
                        }
                        self.sync(in_block);
                    }
                }
                s
            }
            Err(()) => {
                self.nl = saved.0;
                self.depth = saved.1;
                self.sync(in_block);
                Stmt { span: self.sp(lo), kind: StmtKind::Error }
            }
        }
    }

    fn sync(&mut self, in_block: bool) {
        let mut depth = 0i32;
        loop {
            let t = &self.toks[self.pos].tok;
            match t {
                Tok::Eof => break,
                Tok::Newline | Tok::Semi if depth <= 0 => break,
                Tok::Newline => {
                    // A statement keyword at the start of the next line ends the broken statement.
                    let mut j = self.pos;
                    while self.toks[j].tok == Tok::Newline {
                        j += 1;
                    }
                    if is_stmt_kw(&self.toks[j].tok) {
                        break;
                    }
                }
                Tok::LParen | Tok::LBracket | Tok::LBrace => depth += 1,
                Tok::RParen | Tok::RBracket => depth = (depth - 1).max(0),
                Tok::RBrace => {
                    if depth == 0 {
                        if in_block {
                            break;
                        }
                    } else {
                        depth -= 1;
                    }
                }
                _ => {}
            }
            self.last_hi = self.toks[self.pos].span.hi;
            self.pos += 1;
        }
    }

    fn stmt(&mut self) -> R<Stmt> {
        self.enter()?;
        let r = self.stmt_inner();
        self.depth -= 1;
        r
    }

    fn stmt_inner(&mut self) -> R<Stmt> {
        let start = self.cur().clone();
        let lo = start.span.lo;
        let kind = match start.tok {
            Tok::Let | Tok::Set => {
                self.bump();
                let name = self.expect_ident("a variable name")?;
                self.expect(Tok::Assign, "`=`")?;
                let value = self.expr()?;
                if start.tok == Tok::Let { StmtKind::Let { name, value } } else { StmtKind::Set { name, value } }
            }
            Tok::Alias => {
                self.bump();
                let name = self.expect_ident("an alias name")?;
                self.expect(Tok::Assign, "`=`")?;
                let target = self.path("an op or query name")?;
                StmtKind::Alias { name, target }
            }
            Tok::Batch => {
                self.bump();
                StmtKind::Batch(self.block()?)
            }
            Tok::If => return self.if_stmt(),
            Tok::For => {
                self.bump();
                let var = self.expect_ident("a loop variable name")?;
                self.expect(Tok::In, "`in`")?;
                let iter = self.expr()?;
                let body = self.block()?;
                StmtKind::For { var, iter, body }
            }
            Tok::Include => {
                self.bump();
                let t = self.cur().clone();
                if let Tok::Str(s) = t.tok {
                    self.bump();
                    StmtKind::Include { path: s, path_span: t.span }
                } else {
                    return self.expected("a string path");
                }
            }
            Tok::Print => {
                self.bump();
                let mut es = vec![self.expr()?];
                while *self.tok() == Tok::Comma {
                    self.bump();
                    self.skip_nl();
                    es.push(self.expr()?);
                }
                StmtKind::Print(es)
            }
            Tok::Assert => {
                self.bump();
                let cond = self.expr()?;
                let msg = if *self.tok() == Tok::Comma {
                    self.bump();
                    self.skip_nl();
                    Some(self.expr()?)
                } else {
                    None
                };
                StmtKind::Assert { cond, msg }
            }
            Tok::Ident => {
                let is_help = self.text_of(start.span) == "help"
                    && matches!(self.nth(1), Tok::Newline | Tok::Semi | Tok::Eof | Tok::RBrace | Tok::Ident);
                if is_help {
                    self.bump();
                    if *self.tok() == Tok::Ident {
                        StmtKind::Help(Some(self.path("an op or query name")?))
                    } else {
                        StmtKind::Help(None)
                    }
                } else {
                    let path = self.path("an op or query name")?;
                    if *self.tok() != Tok::LParen {
                        let t = self.cur().clone();
                        let found = self.describe(&t);
                        if t.tok != Tok::Error {
                            let d = Diagnostic::error("expected-token", t.span, format!("expected `(` after `{}`, found {found}", path.dotted()))
                                .label("a statement is a call `name(args)` or a keyword statement")
                                .help("calls look like `model.create_brick(x: 1, y: 1, z: 1)`; variables are bound with `let`");
                            self.push(d);
                        }
                        return Err(());
                    }
                    StmtKind::Call(self.call(path)?)
                }
            }
            Tok::Else => {
                let span = start.span;
                self.push(
                    Diagnostic::error("expected-statement", span, "`else` without a matching `if`")
                        .label("no `if` to attach to")
                        .help("`else` must follow the closing `}` of an `if` on the same line"),
                );
                // Consume the dangling branch so its braces do not cascade into more errors.
                self.bump();
                if *self.tok() == Tok::If {
                    let _ = self.if_stmt();
                } else if *self.tok() == Tok::LBrace {
                    let _ = self.block();
                }
                return Err(());
            }
            Tok::Error => return Err(()),
            _ => {
                let found = self.describe(&start);
                self.push(
                    Diagnostic::error("expected-statement", start.span, format!("expected a statement, found {found}"))
                        .label("a statement cannot start here")
                        .help("statements: a call `name(args)`, `let`, `set`, `alias`, `batch`, `if`, `for`, `include`, `print`, `assert`, `help`"),
                );
                return Err(());
            }
        };
        Ok(Stmt { span: self.sp(lo), kind })
    }

    fn if_stmt(&mut self) -> R<Stmt> {
        let lo = self.cur().span.lo;
        self.bump();
        let cond = self.expr()?;
        let then = self.block()?;
        let els = if *self.tok() == Tok::Else {
            self.bump();
            if *self.tok() == Tok::If {
                self.enter()?;
                let r = self.if_stmt();
                self.depth -= 1;
                Some(Else::If(Box::new(r?)))
            } else {
                Some(Else::Block(self.block()?))
            }
        } else {
            None
        };
        Ok(Stmt { span: self.sp(lo), kind: StmtKind::If { cond, then, els } })
    }

    fn block(&mut self) -> R<Block> {
        let open = self.expect(Tok::LBrace, "`{`")?;
        let saved = self.nl;
        self.nl = 0;
        let mut stmts = Vec::new();
        loop {
            self.skip_seps();
            if self.aborted {
                break;
            }
            match self.tok() {
                Tok::RBrace => break,
                Tok::Eof => {
                    self.push(
                        Diagnostic::error("expected-token", self.after_last(), "expected `}` to close the block, found end of input")
                            .label("expected `}`")
                            .note(open.span, "block opened here"),
                    );
                    self.nl = saved;
                    return Ok(Block { span: self.sp(open.span.lo), stmts });
                }
                _ => stmts.push(self.stmt_terminated(true)),
            }
        }
        self.nl = saved;
        if *self.tok() == Tok::RBrace {
            self.bump();
        }
        Ok(Block { span: self.sp(open.span.lo), stmts })
    }

    fn path(&mut self, what: &str) -> R<Path> {
        let first = self.expect_ident(what)?;
        let lo = first.span.lo;
        let mut segs = vec![first];
        while *self.tok() == Tok::Dot && *self.nth(1) == Tok::Ident {
            self.bump();
            segs.push(self.expect_ident(what)?);
        }
        Ok(Path { segs, span: self.sp(lo) })
    }

    fn call(&mut self, path: Path) -> R<Call> {
        let open = self.expect(Tok::LParen, "`(`")?;
        self.nl += 1;
        let mut args = Vec::new();
        loop {
            if *self.tok() == Tok::RParen {
                break;
            }
            if *self.tok() == Tok::Eof || self.at_new_statement() {
                return self.unclosed(open.span, "`)`", "call");
            }
            let saved = (self.nl, self.depth);
            match self.arg() {
                Ok(a) => args.push(a),
                Err(()) => {
                    self.nl = saved.0;
                    self.depth = saved.1;
                    if self.aborted {
                        return Err(());
                    }
                    self.skip_to_arg_end();
                }
            }
            match self.tok() {
                Tok::Comma => {
                    self.bump();
                }
                Tok::RParen => break,
                Tok::Eof => return self.unclosed(open.span, "`)`", "call"),
                _ if self.at_new_statement() => return self.unclosed(open.span, "`)`", "call"),
                _ => {
                    let t = self.cur().clone();
                    if t.tok != Tok::Error {
                        let found = self.describe(&t);
                        self.push(
                            Diagnostic::error("expected-token", t.span, format!("expected `,` or `)`, found {found}"))
                                .label("missing `,` between arguments?")
                                .note(open.span, "arguments of this call"),
                        );
                    }
                    self.skip_to_arg_end();
                    if *self.tok() == Tok::Comma {
                        self.bump();
                    } else if *self.tok() != Tok::RParen {
                        return Err(());
                    }
                }
            }
        }
        let close = self.bump();
        self.nl -= 1;
        let lo = path.span.lo;
        Ok(Call { args_span: Span::new(self.file, open.span.lo, close.span.hi), path, args, span: self.sp(lo) })
    }

    fn unclosed<T>(&mut self, open: Span, want: &str, what: &str) -> R<T> {
        let t = self.cur().clone();
        let at = if t.tok == Tok::Eof || self.idx() > self.pos { self.after_last() } else { t.span };
        let d = Diagnostic::error("unclosed-bracket", at, format!("expected {want} to close the {what}"))
            .label(format!("expected {want}"))
            .note(open, format!("{what} opened here"));
        self.push(d);
        Err(())
    }

    fn skip_to_arg_end(&mut self) {
        let mut depth = 0i32;
        loop {
            let i = self.idx();
            let t = &self.toks[i].tok;
            if self.idx() > self.pos && is_stmt_kw(t) && depth == 0 {
                break;
            }
            match t {
                Tok::Eof => break,
                Tok::Comma | Tok::RParen if depth == 0 => break,
                Tok::LParen | Tok::LBracket | Tok::LBrace => depth += 1,
                Tok::RParen | Tok::RBracket | Tok::RBrace => depth = (depth - 1).max(0),
                _ => {}
            }
            self.bump();
        }
    }

    fn arg(&mut self) -> R<Arg> {
        match self.tok() {
            Tok::Ellipsis => {
                self.bump();
                Ok(Arg::Spread(self.expr()?))
            }
            Tok::Ident if *self.nth(1) == Tok::Colon => {
                let name = self.expect_ident("an argument name")?;
                self.bump(); // ':'
                Ok(Arg::Named { name, value: self.expr()? })
            }
            _ => Ok(Arg::Pos(self.expr()?)),
        }
    }

    // ---- expressions --------------------------------------------------------------------

    fn expr(&mut self) -> R<Expr> {
        self.enter()?;
        let r = self.or();
        self.depth -= 1;
        r
    }

    fn mk(&self, lo: u32, kind: ExprKind) -> Expr {
        Expr::new(self.sp(lo), kind)
    }

    /// Reject trees taller than the limit (long operator chains build deep left-leaning trees).
    fn check_height(&mut self, e: Expr) -> R<Expr> {
        if e.height > self.limits.max_height {
            let max = self.limits.max_height;
            self.push(
                Diagnostic::error("expression-too-long", e.span, format!("expression is too long (limit: {max} chained or nested terms)"))
                    .label("split it with `let`"),
            );
            return Err(());
        }
        Ok(e)
    }

    fn bin(&mut self, op: BinOp, l: Expr, r: Expr) -> R<Expr> {
        let span = l.span.to(r.span);
        self.check_height(Expr::new(span, ExprKind::Binary(op, Box::new(l), Box::new(r))))
    }

    fn or(&mut self) -> R<Expr> {
        let mut l = self.and()?;
        while *self.tok() == Tok::Or {
            self.bump();
            self.skip_nl();
            let r = self.and()?;
            l = self.bin(BinOp::Or, l, r)?;
        }
        Ok(l)
    }

    fn and(&mut self) -> R<Expr> {
        let mut l = self.not()?;
        while *self.tok() == Tok::And {
            self.bump();
            self.skip_nl();
            let r = self.not()?;
            l = self.bin(BinOp::And, l, r)?;
        }
        Ok(l)
    }

    fn not(&mut self) -> R<Expr> {
        if *self.tok() == Tok::Not {
            let lo = self.cur().span.lo;
            self.bump();
            self.enter()?;
            let e = self.not();
            self.depth -= 1;
            return Ok(self.mk(lo, ExprKind::Unary(UnOp::Not, Box::new(e?))));
        }
        self.cmp()
    }

    fn cmp_op(&self) -> Option<BinOp> {
        Some(match self.tok() {
            Tok::EqEq => BinOp::Eq,
            Tok::NotEq => BinOp::Ne,
            Tok::Lt => BinOp::Lt,
            Tok::Le => BinOp::Le,
            Tok::Gt => BinOp::Gt,
            Tok::Ge => BinOp::Ge,
            _ => return None,
        })
    }

    fn cmp(&mut self) -> R<Expr> {
        let l = self.union()?;
        if let Some(op) = self.cmp_op() {
            self.bump();
            self.skip_nl();
            let r = self.union()?;
            if self.cmp_op().is_some() {
                let span = self.cur().span;
                self.push(
                    Diagnostic::error("chained-comparison", span, "comparison operators cannot be chained")
                        .label("use `and` to combine comparisons")
                        .help("write `a < b and b < c`"),
                );
                return Err(());
            }
            return self.bin(op, l, r);
        }
        Ok(l)
    }

    fn union(&mut self) -> R<Expr> {
        let mut l = self.inter()?;
        while *self.tok() == Tok::Pipe {
            self.bump();
            self.skip_nl();
            let r = self.inter()?;
            l = self.bin(BinOp::Union, l, r)?;
        }
        Ok(l)
    }

    fn inter(&mut self) -> R<Expr> {
        let mut l = self.add()?;
        while *self.tok() == Tok::Amp {
            self.bump();
            self.skip_nl();
            let r = self.add()?;
            l = self.bin(BinOp::Inter, l, r)?;
        }
        Ok(l)
    }

    fn add(&mut self) -> R<Expr> {
        let mut l = self.mul()?;
        loop {
            let op = match self.tok() {
                Tok::Plus => BinOp::Add,
                Tok::Minus => BinOp::Sub,
                Tok::Except => BinOp::Except,
                _ => break,
            };
            self.bump();
            self.skip_nl();
            let r = self.mul()?;
            l = self.bin(op, l, r)?;
        }
        Ok(l)
    }

    fn mul(&mut self) -> R<Expr> {
        let mut l = self.unary()?;
        loop {
            let op = match self.tok() {
                Tok::Star => BinOp::Mul,
                Tok::Slash => BinOp::Div,
                Tok::Percent => BinOp::Rem,
                _ => break,
            };
            self.bump();
            self.skip_nl();
            let r = self.unary()?;
            l = self.bin(op, l, r)?;
        }
        Ok(l)
    }

    fn unary(&mut self) -> R<Expr> {
        if *self.tok() == Tok::Minus {
            let lo = self.cur().span.lo;
            self.bump();
            self.enter()?;
            let e = self.unary();
            self.depth -= 1;
            return Ok(self.mk(lo, ExprKind::Unary(UnOp::Neg, Box::new(e?))));
        }
        self.pow()
    }

    fn pow(&mut self) -> R<Expr> {
        let base = self.postfix()?;
        if *self.tok() == Tok::StarStar {
            self.bump();
            self.skip_nl();
            self.enter()?;
            let e = self.unary();
            self.depth -= 1;
            return self.bin(BinOp::Pow, base, e?);
        }
        Ok(base)
    }

    fn postfix(&mut self) -> R<Expr> {
        let lo = self.cur().span.lo;
        let mut e = self.primary()?;
        loop {
            match self.tok() {
                Tok::Dot => {
                    self.bump();
                    let f = self.expect_ident("a field name")?;
                    e = self.mk(lo, ExprKind::Field(Box::new(e), f));
                    e = self.check_height(e)?;
                }
                Tok::LBracket => {
                    let open = self.bump();
                    self.nl += 1;
                    let idx = if *self.tok() == Tok::Star && *self.nth(1) == Tok::RBracket {
                        self.bump();
                        Index::All
                    } else {
                        Index::At(Box::new(self.expr()?))
                    };
                    if *self.tok() != Tok::RBracket {
                        if *self.tok() == Tok::Eof || self.at_new_statement() {
                            return self.unclosed(open.span, "`]`", "index");
                        }
                        return self.expected("`]`");
                    }
                    self.bump();
                    self.nl -= 1;
                    e = self.mk(lo, ExprKind::Index(Box::new(e), idx));
                    e = self.check_height(e)?;
                }
                _ => break,
            }
        }
        Ok(e)
    }

    /// Is the current identifier the head of `a.b.c(`?
    fn is_call_head(&self) -> bool {
        let mut k = 1;
        while *self.nth(k) == Tok::Dot && *self.nth(k + 1) == Tok::Ident {
            k += 2;
        }
        *self.nth(k) == Tok::LParen
    }

    fn primary(&mut self) -> R<Expr> {
        let t = self.cur().clone();
        let lo = t.span.lo;
        match t.tok {
            Tok::Int(v) => {
                self.bump();
                Ok(self.mk(lo, ExprKind::Int(v)))
            }
            Tok::Float(v) => {
                self.bump();
                Ok(self.mk(lo, ExprKind::Num(v)))
            }
            Tok::Str(s) => {
                self.bump();
                Ok(self.mk(lo, ExprKind::Str(s)))
            }
            Tok::True | Tok::False => {
                self.bump();
                Ok(self.mk(lo, ExprKind::Bool(t.tok == Tok::True)))
            }
            Tok::LParen => {
                self.bump();
                self.nl += 1;
                let inner = self.expr()?;
                if *self.tok() != Tok::RParen {
                    if *self.tok() == Tok::Eof || self.at_new_statement() {
                        return self.unclosed(t.span, "`)`", "parenthesis");
                    }
                    return self.expected("`)`");
                }
                self.bump();
                self.nl -= 1;
                Ok(self.mk(lo, inner.kind))
            }
            Tok::LBracket => self.list(t.span),
            Tok::LBrace => self.object(t.span),
            Tok::EntityRef => self.entity_ref(),
            Tok::All => {
                self.bump();
                let k = self.cur().clone();
                if k.tok == Tok::Ident {
                    let name = self.text_of(k.span).to_owned();
                    self.bump();
                    match kinds::kind_from_plural(&name) {
                        Some(kind) => Ok(self.mk(lo, ExprKind::Entity(EntityExpr::All { kind, kind_span: k.span }))),
                        None => {
                            let mut d = Diagnostic::error("unknown-entity-kind", k.span, format!("`{name}` is not an entity kind keyword"))
                                .label("expected a plural kind like `faces` or `volumes`");
                            if let Some(s) = suggest(&name, kinds::plural_spellings()) {
                                d = d.help(format!("did you mean `{s}`?"));
                            }
                            self.push(d);
                            Err(())
                        }
                    }
                } else {
                    self.expected("a plural entity kind such as `faces` after `all`")
                }
            }
            Tok::Ident => {
                let name = self.text_of(t.span).to_owned();
                if *self.nth(1) == Tok::In
                    && let Some(kind) = kinds::kind_from_plural(&name)
                {
                    self.bump();
                    self.bump(); // `in`
                    self.skip_nl();
                    self.enter()?;
                    let within = self.postfix();
                    self.depth -= 1;
                    return Ok(self.mk(lo, ExprKind::Entity(EntityExpr::In { kind, kind_span: t.span, within: Box::new(within?) })));
                }
                if self.is_call_head() {
                    let path = self.path("a function name")?;
                    let call = self.call(path)?;
                    return Ok(Expr::new(call.span, ExprKind::Call(call)));
                }
                self.bump();
                Ok(self.mk(lo, ExprKind::Var(name)))
            }
            Tok::Error => Err(()),
            _ => {
                let found = self.describe(&t);
                let at = self.blame(&t);
                self.push(Diagnostic::error("expected-expression", at, format!("expected an expression, found {found}")).label("an expression is needed here"));
                Err(())
            }
        }
    }

    fn list(&mut self, open: Span) -> R<Expr> {
        self.bump();
        self.nl += 1;
        let mut items = Vec::new();
        loop {
            if *self.tok() == Tok::RBracket {
                break;
            }
            if *self.tok() == Tok::Eof || self.at_new_statement() {
                return self.unclosed(open, "`]`", "list");
            }
            items.push(self.expr()?);
            match self.tok() {
                Tok::Comma => {
                    self.bump();
                }
                Tok::RBracket => break,
                Tok::Eof => return self.unclosed(open, "`]`", "list"),
                _ if self.at_new_statement() => return self.unclosed(open, "`]`", "list"),
                _ => return self.expected("`,` or `]`"),
            }
        }
        self.bump();
        self.nl -= 1;
        Ok(self.mk(open.lo, ExprKind::List(items)))
    }

    fn object(&mut self, open: Span) -> R<Expr> {
        self.bump();
        self.nl += 1;
        let mut items = Vec::new();
        loop {
            if *self.tok() == Tok::RBrace {
                break;
            }
            if *self.tok() == Tok::Eof || self.at_new_statement() {
                return self.unclosed(open, "`}`", "object");
            }
            let k = self.cur().clone();
            let key = match &k.tok {
                Tok::Ident => Key { name: self.text_of(k.span).to_owned(), span: k.span },
                Tok::Str(s) => Key { name: s.clone(), span: k.span },
                _ => return self.expected("an object key (identifier or string)"),
            };
            self.bump();
            self.expect(Tok::Colon, "`:` after the object key")?;
            let v = self.expr()?;
            items.push((key, v));
            match self.tok() {
                Tok::Comma => {
                    self.bump();
                }
                Tok::RBrace => break,
                Tok::Eof => return self.unclosed(open, "`}`", "object"),
                _ if self.at_new_statement() => return self.unclosed(open, "`}`", "object"),
                _ => return self.expected("`,` or `}`"),
            }
        }
        self.bump();
        self.nl -= 1;
        Ok(self.mk(open.lo, ExprKind::Obj(items)))
    }

    // ---- entity references --------------------------------------------------------------

    fn entity_ref(&mut self) -> R<Expr> {
        let t = self.bump();
        let lo = t.span.lo;
        let kind_span = Span::new(self.file, t.span.lo, t.span.hi - 1);
        let name = self.text_of(kind_span).to_owned();
        let kind = kinds::kind_from_singular(&name);
        if kind.is_none() {
            let mut d = Diagnostic::error("unknown-entity-kind", kind_span, format!("unknown entity kind `{name}`")).label("not an entity kind");
            if let Some(s) = suggest(&name, kinds::singular_spellings()) {
                d = d.help(format!("did you mean `{s}`?"));
            } else {
                d = d.help("kinds: Vertex, Edge, Face, Volume, Body, Curve, Surface, Color, Group, ... (or a full name such as `MeshCell`)");
            }
            self.push(d);
        }
        // The lexer guarantees a digit or `[` follows.
        let mut items = Vec::new();
        let mut bad = kind.is_none();
        if *self.tok() == Tok::LBracket {
            let open = self.bump();
            self.nl += 1;
            loop {
                if *self.tok() == Tok::RBracket {
                    break;
                }
                if *self.tok() == Tok::Eof || self.at_new_statement() {
                    return self.unclosed(open.span, "`]`", "id list");
                }
                match self.id_item() {
                    Ok(Some(it)) => items.push(it),
                    Ok(None) => bad = true,
                    Err(()) => return Err(()),
                }
                match self.tok() {
                    Tok::Comma => {
                        self.bump();
                    }
                    Tok::RBracket => break,
                    _ => return self.expected("`,` or `]`"),
                }
            }
            let close = self.bump();
            self.nl -= 1;
            if items.is_empty() && !bad {
                self.push(
                    Diagnostic::error("bad-id-range", Span::new(self.file, open.span.lo, close.span.hi), "empty id list").label("list at least one id or range"),
                );
                bad = true;
            }
        } else {
            match self.id_item() {
                Ok(Some(it)) => items.push(it),
                Ok(None) => bad = true,
                Err(()) => return Err(()),
            }
        }
        match (kind, bad) {
            (Some(kind), false) => Ok(self.mk(lo, ExprKind::Entity(EntityExpr::Ids { kind, kind_span, items }))),
            _ => Err(()),
        }
    }

    /// One id or range. `Ok(None)`: a semantic problem was reported but tokens were consumed correctly.
    fn id_item(&mut self) -> R<Option<IdItem>> {
        let t = self.cur().clone();
        let Tok::Int(lo_v) = t.tok else {
            return self.expected("a serial number");
        };
        self.bump();
        let lo = t.span.lo;
        let mut item = Some(IdItem::One(lo_v as u64, t.span));
        if *self.tok() == Tok::DotDot {
            self.bump();
            let h = self.cur().clone();
            let Tok::Int(hi_v) = h.tok else {
                return self.expected("the end of the range");
            };
            self.bump();
            let mut step = 1u64;
            let mut step_ok = true;
            if *self.tok() == Tok::Ident && self.text_of(self.cur().span) == "by" {
                self.bump();
                let s = self.cur().clone();
                let Tok::Int(sv) = s.tok else {
                    return self.expected("a step number");
                };
                self.bump();
                if sv < 1 {
                    self.push(Diagnostic::error("bad-id-range", s.span, "range step must be at least 1").label("step is zero"));
                    step_ok = false;
                }
                step = sv.max(1) as u64;
            }
            let span = self.sp(lo);
            if hi_v < lo_v {
                self.push(
                    Diagnostic::error("bad-id-range", span, "range start is greater than its end")
                        .label(format!("{lo_v} > {hi_v}"))
                        .help("ranges are inclusive and ascending: `Volume#1..5`"),
                );
                item = None;
            } else if (hi_v as u64) > sm_core::EntityId::MAX_SERIAL {
                self.push(Diagnostic::error("bad-id-range", span, "serial too large").label("exceeds the 56-bit serial space"));
                item = None;
            } else if step_ok {
                item = Some(IdItem::Range { lo: lo_v as u64, hi: hi_v as u64, step, span });
            } else {
                item = None;
            }
        } else if (lo_v as u64) > sm_core::EntityId::MAX_SERIAL {
            self.push(Diagnostic::error("bad-id-range", t.span, "serial too large").label("exceeds the 56-bit serial space"));
            item = None;
        }
        Ok(item)
    }
}
