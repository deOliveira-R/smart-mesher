//! Completion candidates at a cursor position, from a tolerant token scan (works on incomplete input).

use std::collections::BTreeSet;

use sm_core::EntityKind;

use crate::eval::BUILTIN_NAMES;
use crate::host::Host;
use crate::interp::Interpreter;
use crate::kinds;
use crate::lexer::{Tok, Token, lex};
use crate::schema::ParamInfo;
use crate::value::{Outcome, Val};

/// What a candidate is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum CandidateKind {
    /// A parameter name of the enclosing call.
    Param,
    /// A constant value allowed by the schema (enum value, `true`/`false`).
    Value,
    /// An op.
    Op,
    /// A query.
    Query,
    /// A script alias.
    Alias,
    /// A variable.
    Variable,
    /// A field of a variable (`b.body`).
    Field,
    /// A live entity serial (after `Kind#`).
    Entity,
    /// An entity kind (`Volume#`, `faces`).
    Kind,
    /// A builtin function or constant.
    Function,
    /// A language keyword.
    Keyword,
}

/// One suggestion.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Candidate {
    /// Text shown.
    pub label: String,
    /// Text to insert in place of the typed prefix.
    pub insert: String,
    /// What it is.
    pub kind: CandidateKind,
    /// Type or description.
    pub detail: Option<String>,
}

/// Candidates plus the byte range of the already typed prefix they replace.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Completion {
    /// Start of the typed prefix.
    pub start: usize,
    /// End of the typed prefix (the cursor).
    pub end: usize,
    /// Candidates in a deterministic order (by kind, then label).
    pub items: Vec<Candidate>,
}

const STATEMENT_KEYWORDS: &[&str] = &["alias", "assert", "batch", "for", "help", "if", "include", "let", "print", "set"];
/// Cap on live-entity candidates.
const MAX_ENTITIES: usize = 50;

fn first_line(s: &str) -> String {
    s.lines().next().unwrap_or("").trim().to_owned()
}

fn cand(label: impl Into<String>, insert: impl Into<String>, kind: CandidateKind, detail: Option<String>) -> Candidate {
    Candidate { label: label.into(), insert: insert.into(), kind, detail }
}

fn is_word(t: &Tok) -> bool {
    matches!(
        t,
        Tok::Ident
            | Tok::Let
            | Tok::Set
            | Tok::Alias
            | Tok::Batch
            | Tok::If
            | Tok::Else
            | Tok::For
            | Tok::In
            | Tok::Include
            | Tok::Print
            | Tok::Assert
            | Tok::True
            | Tok::False
            | Tok::And
            | Tok::Or
            | Tok::Not
            | Tok::All
            | Tok::Except
    )
}

fn is_operator(t: &Tok) -> bool {
    matches!(
        t,
        Tok::Plus | Tok::Minus | Tok::Star | Tok::Slash | Tok::Percent | Tok::StarStar | Tok::EqEq | Tok::NotEq | Tok::Lt | Tok::Le | Tok::Gt | Tok::Ge | Tok::Pipe | Tok::Amp | Tok::And | Tok::Or | Tok::Not | Tok::Except
    )
}

fn field_names(v: &Val) -> Vec<String> {
    match v {
        Val::Obj(m) => m.keys().cloned().collect(),
        Val::Outcome(Outcome::Concrete { outputs, .. }) => outputs.keys().cloned().chain(["created".to_owned(), "removed".to_owned()]).collect(),
        Val::Outcome(Outcome::Applied { outputs }) => outputs.keys().cloned().collect(),
        _ => Vec::new(),
    }
}

impl<H: Host + ?Sized> Interpreter<'_, H> {
    /// Completion candidates for `text` with the cursor at byte offset `cursor` (clamped to the text and to a character
    /// boundary). Sees the interpreter's variables, aliases and live entities.
    pub fn complete(&mut self, text: &str, cursor: usize) -> Completion {
        let mut cursor = cursor.min(text.len());
        while !text.is_char_boundary(cursor) {
            cursor -= 1;
        }
        let src = &text[..cursor];
        let mut diags = Vec::new();
        let mut toks = lex(0, src, &mut diags);
        toks.pop(); // Eof
        let text_of = |t: &Token| -> &str { &src[t.span.lo as usize..t.span.hi as usize] };

        // The word being typed: a word token or a serial (after `Kind#`) that ends at the cursor.
        let mut word_idx = None;
        if let Some(last) = toks.last()
            && last.span.hi as usize == cursor
        {
            let after_ref = toks.len() >= 2 && toks[toks.len() - 2].tok == Tok::EntityRef && toks[toks.len() - 2].span.hi == last.span.lo;
            if is_word(&last.tok) || (matches!(last.tok, Tok::Int(_)) && after_ref) {
                word_idx = Some(toks.len() - 1);
            } else if matches!(last.tok, Tok::Int(_) | Tok::Float(_) | Tok::Str(_)) {
                return Completion { start: cursor, end: cursor, items: Vec::new() };
            }
        }
        let (mut start, mut prefix) = match word_idx {
            Some(i) => (toks[i].span.lo as usize, text_of(&toks[i]).to_owned()),
            None => (cursor, String::new()),
        };
        let mut ctx_end = word_idx.unwrap_or(toks.len());
        // A dotted name being typed (`model.cre`, `model.`): the word is the whole path, unless its head is a bound
        // variable, in which case it is a field access (`b.bo`).
        let adjacent = |a: &Token, b: &Token| a.span.hi == b.span.lo;
        let tail = match word_idx {
            Some(i) if is_word(&toks[i].tok) => Some(i),
            Some(_) => None,
            None => toks.len().checked_sub(1).filter(|&k| toks[k].tok == Tok::Dot && toks[k].span.hi as usize == cursor).map(|k| k + 1),
        };
        if let Some(t) = tail {
            let mut j = t;
            // `t` is the index one past a trailing dot, or the word itself.
            let mut k = if word_idx.is_some() { t } else { t - 1 };
            if word_idx.is_none() {
                // Trailing dot at k: the chain's last identifier is k - 1.
                if k >= 1 && is_word(&toks[k - 1].tok) && adjacent(&toks[k - 1], &toks[k]) {
                    k -= 1;
                    j = k;
                } else {
                    j = usize::MAX;
                }
            }
            if j != usize::MAX {
                while j >= 2 && toks[j - 1].tok == Tok::Dot && adjacent(&toks[j - 2], &toks[j - 1]) && toks[j - 1].span.hi == toks[j].span.lo && is_word(&toks[j - 2].tok) {
                    j -= 2;
                }
                let chained = j < k || word_idx.is_none();
                if chained {
                    let head = text_of(&toks[j]);
                    let bound = self.scopes.iter().any(|s| s.contains_key(head));
                    if !bound {
                        start = toks[j].span.lo as usize;
                        prefix = src[start..cursor].to_owned();
                        ctx_end = j;
                    }
                }
            }
        }
        let ctx: &[Token] = &toks[..ctx_end];
        let mut items = self.candidates(src, ctx, &diags);
        items.retain(|c| c.label.starts_with(&prefix) && !(c.label == prefix && c.kind == CandidateKind::Keyword && c.insert == c.label));
        items.sort_by_key(|c| c.kind); // stable: generation order is deterministic (required parameters first, then sorted names)
        let mut seen = BTreeSet::new();
        items.retain(|c| seen.insert((c.kind, c.label.clone())));
        Completion { start, end: cursor, items }
    }

    fn candidates(&mut self, src: &str, ctx: &[Token], _diags: &[crate::Diagnostic]) -> Vec<Candidate> {
        let text_of = |t: &Token| -> &str { &src[t.span.lo as usize..t.span.hi as usize] };
        let prev = ctx.last();

        // `b.` / `b.body.`: fields of a bound value.
        if let Some(p) = prev
            && p.tok == Tok::Dot
        {
            let mut chain = Vec::new();
            let mut i = ctx.len() - 1;
            while i >= 1 && ctx[i].tok == Tok::Dot && ctx[i - 1].tok == Tok::Ident {
                chain.push(text_of(&ctx[i - 1]).to_owned());
                if i >= 3 && ctx[i - 2].tok == Tok::Dot {
                    i -= 2;
                } else {
                    break;
                }
            }
            chain.reverse();
            return self.field_candidates(&chain);
        }

        // Entity serials: after `Kind#` (also `Kind#[1, ` and a lone `#` after a kind name).
        if let Some(kind) = self.serial_context(src, ctx) {
            return self.serial_candidates(kind);
        }

        // Bracket stack over the context tokens.
        let mut stack: Vec<usize> = Vec::new();
        for (i, t) in ctx.iter().enumerate() {
            match t.tok {
                Tok::LParen | Tok::LBracket | Tok::LBrace => stack.push(i),
                Tok::RParen | Tok::RBracket | Tok::RBrace => {
                    stack.pop();
                }
                Tok::Newline | Tok::Semi if stack.is_empty() => {}
                Tok::Newline => {
                    // A statement keyword on a new line inside brackets means the bracket was never closed.
                    if let Some(next) = ctx.get(i + 1)
                        && matches!(next.tok, Tok::Let | Tok::Set | Tok::Alias | Tok::Batch | Tok::If | Tok::For | Tok::Include | Tok::Print | Tok::Assert)
                    {
                        stack.clear();
                    }
                }
                _ => {}
            }
        }
        // Block braces are not argument lists.
        while let Some(&top) = stack.last() {
            if ctx[top].tok == Tok::LBrace && !self.is_object_brace(ctx, top) {
                stack.pop();
                // Statements inside a block start fresh at the last separator, handled below via `stmt_start`.
            } else {
                break;
            }
        }

        if let Some(&top) = stack.last()
            && ctx[top].tok == Tok::LParen
            && let Some(c) = self.call_candidates(src, ctx, top)
        {
            return c;
        }

        // Statement start or expression start.
        let at_stmt_start = match prev.map(|t| &t.tok) {
            None | Some(Tok::Newline | Tok::Semi) => stack.is_empty(),
            Some(Tok::LBrace) => stack.is_empty() || !self.is_object_brace(ctx, ctx.len() - 1),
            Some(Tok::RBrace) => true,
            Some(Tok::Else) => false,
            _ => false,
        };
        if at_stmt_start {
            return self.statement_candidates();
        }
        match prev.map(|t| &t.tok) {
            Some(Tok::Let | Tok::Set) => {
                if prev.map(|t| &t.tok) == Some(&Tok::Set) {
                    return self.variable_candidates();
                }
                return Vec::new();
            }
            Some(Tok::Alias) => return Vec::new(),
            Some(Tok::All) => return self.plural_candidates(false),
            Some(Tok::In) => {
                // `for x in ...` and `faces in ...` both take an expression.
                return self.expression_candidates(None);
            }
            Some(Tok::Include | Tok::Else) => return Vec::new(),
            _ => {}
        }
        if prev.is_some_and(|t| t.tok == Tok::Ident && text_of(t) == "help") && (ctx.len() == 1 || matches!(ctx[ctx.len() - 2].tok, Tok::Newline | Tok::Semi | Tok::LBrace | Tok::RBrace)) {
            return self.op_name_candidates();
        }
        self.expression_candidates(None)
    }

    fn is_object_brace(&self, ctx: &[Token], i: usize) -> bool {
        match i.checked_sub(1).map(|j| &ctx[j].tok) {
            None => false,
            Some(Tok::Colon | Tok::Comma | Tok::LParen | Tok::LBracket | Tok::Assign | Tok::Ellipsis | Tok::In) => true,
            Some(t) => is_operator(t),
        }
    }

    fn serial_context(&self, src: &str, ctx: &[Token]) -> Option<EntityKind> {
        let name_of = |t: &Token| &src[t.span.lo as usize..t.span.hi as usize];
        // `Kind#` ends the context.
        if let Some(last) = ctx.last()
            && last.tok == Tok::EntityRef
        {
            return kinds::kind_from_singular(name_of(last).trim_end_matches('#'));
        }
        // A lone `#` right after a kind name lexes as `Ident` + error token.
        if ctx.len() >= 2 {
            let (a, b) = (&ctx[ctx.len() - 2], &ctx[ctx.len() - 1]);
            if b.tok == Tok::Error && name_of(b) == "#" && a.tok == Tok::Ident && a.span.hi == b.span.lo {
                return kinds::kind_from_singular(name_of(a));
            }
        }
        // `Kind#[` / `Kind#[1, ` / `Kind#[1, 3..`
        let mut depth = 0i32;
        for i in (0..ctx.len()).rev() {
            match ctx[i].tok {
                Tok::RBracket => depth += 1,
                Tok::LBracket => {
                    if depth == 0 {
                        return (i >= 1 && ctx[i - 1].tok == Tok::EntityRef).then(|| kinds::kind_from_singular(name_of(&ctx[i - 1]).trim_end_matches('#'))).flatten();
                    }
                    depth -= 1;
                }
                Tok::Newline | Tok::Semi | Tok::LParen | Tok::LBrace => return None,
                _ => {}
            }
        }
        None
    }

    fn serial_candidates(&mut self, kind: EntityKind) -> Vec<Candidate> {
        self.host.state().entities_of(kind).take(MAX_ENTITIES).map(|id| cand(id.serial().to_string(), id.serial().to_string(), CandidateKind::Entity, Some(id.to_string()))).collect()
    }

    fn field_candidates(&self, chain: &[String]) -> Vec<Candidate> {
        let Some(head) = chain.first() else { return Vec::new() };
        let mut val: Option<&Val> = None;
        for scope in self.scopes.iter().rev() {
            if let Some(b) = scope.get(head) {
                val = Some(&b.val);
                break;
            }
        }
        let Some(mut v) = val else { return Vec::new() };
        for seg in &chain[1..] {
            match v {
                Val::Obj(m) => match m.get(seg) {
                    Some(x) => v = x,
                    None => return Vec::new(),
                },
                _ => return Vec::new(),
            }
        }
        let mut names = field_names(v);
        names.sort();
        names.into_iter().map(|f| cand(f.clone(), f, CandidateKind::Field, None)).collect()
    }

    fn statement_candidates(&mut self) -> Vec<Candidate> {
        let mut out: Vec<Candidate> = STATEMENT_KEYWORDS.iter().map(|k| cand(*k, *k, CandidateKind::Keyword, None)).collect();
        out.extend(self.op_name_candidates_calls());
        for (a, target) in self.aliases.clone() {
            out.push(cand(a.clone(), format!("{a}("), CandidateKind::Alias, Some(format!("alias of {target}"))));
        }
        out
    }

    fn op_name_candidates_calls(&mut self) -> Vec<Candidate> {
        self.callable_map()
            .values()
            .map(|c| {
                cand(
                    c.name.clone(),
                    format!("{}(", c.name),
                    if c.is_query { CandidateKind::Query } else { CandidateKind::Op },
                    c.description().map(|d| first_line(&d)),
                )
            })
            .collect()
    }

    fn op_name_candidates(&mut self) -> Vec<Candidate> {
        self.callable_map().values().map(|c| cand(c.name.clone(), c.name.clone(), if c.is_query { CandidateKind::Query } else { CandidateKind::Op }, c.description().map(|d| first_line(&d)))).collect()
    }

    fn variable_candidates(&self) -> Vec<Candidate> {
        let mut seen = BTreeSet::new();
        let mut out = Vec::new();
        for scope in self.scopes.iter().rev() {
            for (k, b) in scope {
                if seen.insert(k.clone()) {
                    out.push(cand(k.clone(), k.clone(), CandidateKind::Variable, Some(b.val.type_name().to_owned())));
                }
            }
        }
        out
    }

    fn plural_candidates(&self, with_in: bool) -> Vec<Candidate> {
        kinds::KIND_TABLE.iter().map(|r| cand(r.plural, if with_in { format!("{} in ", r.plural) } else { r.plural.to_owned() }, CandidateKind::Kind, Some(r.kind.name().to_owned()))).collect()
    }

    /// Candidates where an expression may start. `hint` is the parameter the expression is for, if known.
    fn expression_candidates(&mut self, hint: Option<&ParamInfo>) -> Vec<Candidate> {
        let mut out = Vec::new();
        if let Some(p) = hint {
            for c in &p.choices {
                out.push(cand(c.clone(), c.clone(), CandidateKind::Value, p.description.as_deref().map(first_line)));
            }
            if p.ty == "bool" {
                out.push(cand("true", "true", CandidateKind::Value, None));
                out.push(cand("false", "false", CandidateKind::Value, None));
            }
        }
        out.extend(self.variable_candidates());
        out.push(cand("true", "true", CandidateKind::Keyword, None));
        out.push(cand("false", "false", CandidateKind::Keyword, None));
        out.push(cand("not", "not ", CandidateKind::Keyword, None));
        out.push(cand("all", "all ", CandidateKind::Keyword, None));
        for b in BUILTIN_NAMES {
            let insert = if matches!(*b, "pi" | "tau") { (*b).to_owned() } else { format!("{b}(") };
            out.push(cand(*b, insert, CandidateKind::Function, None));
        }
        for s in kinds::KIND_TABLE.iter().map(|r| r.singular) {
            out.push(cand(format!("{s}#"), format!("{s}#"), CandidateKind::Kind, None));
        }
        out.extend(self.plural_candidates(true));
        out
    }

    /// Inside `name(`: parameter names or parameter values. `None` if the callee is not a registry call.
    fn call_candidates(&mut self, src: &str, ctx: &[Token], open: usize) -> Option<Vec<Candidate>> {
        let text_of = |t: &Token| -> &str { &src[t.span.lo as usize..t.span.hi as usize] };
        // Callee path right before `(`.
        let mut segs: Vec<&str> = Vec::new();
        let mut i = open;
        while i >= 1 && ctx[i - 1].tok == Tok::Ident {
            segs.push(text_of(&ctx[i - 1]));
            if i >= 3 && ctx[i - 2].tok == Tok::Dot {
                i -= 2;
            } else {
                break;
            }
        }
        if segs.is_empty() {
            return None;
        }
        segs.reverse();
        let written = segs.join(".");
        let name = if segs.len() == 1 { self.aliases.get(&written).cloned().unwrap_or(written) } else { written };
        let info = self.lookup_callable(&name)?;
        let params = info.params();

        // Split the arguments typed so far at depth-0 commas.
        let args = &ctx[open + 1..];
        let mut depth = 0i32;
        let mut current_start = 0;
        let mut given: BTreeSet<&str> = BTreeSet::new();
        let mut arg_start = 0;
        for (k, t) in args.iter().enumerate() {
            match t.tok {
                Tok::LParen | Tok::LBracket | Tok::LBrace => depth += 1,
                Tok::RParen | Tok::RBracket | Tok::RBrace => depth -= 1,
                Tok::Comma if depth == 0 => {
                    if args.get(arg_start).is_some_and(|a| a.tok == Tok::Ident) && args.get(arg_start + 1).is_some_and(|c| c.tok == Tok::Colon) {
                        given.insert(text_of(&args[arg_start]));
                    }
                    arg_start = k + 1;
                    current_start = k + 1;
                }
                _ => {}
            }
        }
        let current = &args[current_start.min(args.len())..];
        match current {
            [] => Some(
                params
                    .iter()
                    .filter(|p| !given.contains(p.name.as_str()))
                    .map(|p| cand(p.name.clone(), format!("{}: ", p.name), CandidateKind::Param, Some(match &p.description { Some(d) => format!("{}{}: {}", p.ty, if p.required { "" } else { "?" }, first_line(d)), None => format!("{}{}", p.ty, if p.required { "" } else { "?" }) })))
                    .collect(),
            ),
            [name_tok, colon, rest @ ..] if name_tok.tok == Tok::Ident && colon.tok == Tok::Colon => {
                let hint = params.iter().find(|p| p.name == text_of(name_tok));
                // Values are offered at the start of the value or after an operator.
                let at_value_start = rest.is_empty() || rest.last().is_some_and(|t| is_operator(&t.tok) || matches!(t.tok, Tok::LParen | Tok::LBracket | Tok::Comma | Tok::In));
                if at_value_start {
                    let mut c = self.expression_candidates(hint);
                    if let Some(h) = hint
                        && h.is_entity
                    {
                        c.retain(|x| x.kind != CandidateKind::Function || matches!(x.label.as_str(), "ents"));
                    }
                    Some(c)
                } else {
                    Some(Vec::new())
                }
            }
            _ => Some(self.expression_candidates(None)),
        }
    }
}
