//! The inverse of the parser for one stored op: a statement that lowers back to the same record.
//!
//! [`print_op`] writes `name(arg: value, ...)` with arguments in the schema's order (required first, then by name),
//! shortest round-trip floats, only the lexer's string escapes, and ids in id-typed positions as script ids
//! (`Face#3`, `GeomSurface#7`). A `core.batch` record prints as a `batch { ... }` block whose members that are
//! referenced are bound to `b<ordinal>m<member>`.
//!
//! Every printed statement is lexed and parsed again and its arguments are folded to JSON by a small constant folder
//! (no interpreter, no registry). The JSON must equal the record's params exactly (integer against float kind, f64 bits
//! except `-0.0`, string bytes, key set, array order); otherwise [`PrintError::RoundTrip`], never a silently wrong script.

use std::collections::{BTreeMap, BTreeSet};

use serde_json::{Map, Value as Json};
use sm_core::EntityId;
use sm_op_api::{Hash256, OpRecord, Registry};

use crate::ast::{Arg, BinOp, EntityExpr, Expr, ExprKind, IdItem, Index, StmtKind, UnOp};
use crate::lexer::{FUTURE_WORDS, keyword};
use crate::parser::{ParseLimits, parse_with};
use crate::schema::{SchemaDoc, View};
use crate::value::{fmt_id, fmt_num};

/// How a record is written.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub enum Style {
    /// Named arguments in schema order, ids as script ids. Falls back to [`Style::Raw`] per statement when the
    /// syntax self-check fails.
    #[default]
    Readable,
    /// `name(...{json})`: every parameter inside one spread object, ids stay strings.
    Raw,
}

/// Why a record has no source form.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum PrintError {
    /// A stored bare reference (`"$0.body"`) has no source form: the language lowers `b.body[0]` to `[0]`.
    /// `RecordOptions::normalize_refs` prints `[0]` instead.
    #[error("{}the bare batch reference at `{path}` has no source form (the language writes `[0]`); set normalize_refs to print `[0]`", at(*.node))]
    BareReference {
        /// The timeline node, when known.
        node: Option<Hash256>,
        /// Where in the params.
        path: String,
    },
    /// The printed text did not read back to the record's params (the first differing path is named).
    #[error("{}the printed statement does not read back to the stored value at `{path}`", at(*.node))]
    RoundTrip {
        /// The timeline node, when known.
        node: Option<Hash256>,
        /// Where in the params.
        path: String,
    },
    /// The value has no source form at all (null, an integer beyond i64, a role that is not an identifier, ...).
    #[error("{}`{path}` cannot be written as source: {reason}", at(*.node))]
    Unprintable {
        /// The timeline node, when known.
        node: Option<Hash256>,
        /// Where in the params.
        path: String,
        /// Why.
        reason: String,
    },
    /// The registry does not know the op (so its schema is unavailable).
    #[error("{}the registry has no op `{name}`", at(*.node))]
    UnknownOp {
        /// The timeline node, when known.
        node: Option<Hash256>,
        /// The op name.
        name: String,
    },
}

fn at(node: Option<Hash256>) -> String {
    node.map(|n| format!("node {n}: ")).unwrap_or_default()
}

impl PrintError {
    /// The same error naming `node` (kept when it already names one).
    #[must_use]
    pub fn at_node(mut self, n: Hash256) -> Self {
        match &mut self {
            Self::BareReference { node, .. } | Self::RoundTrip { node, .. } | Self::Unprintable { node, .. } | Self::UnknownOp { node, .. } => {
                node.get_or_insert(n);
            }
        }
        self
    }
}

/// What the printer noticed besides the text.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum PrintNote {
    /// Readable failed the self-check; the statement is `Raw`.
    RawFallback(String),
    /// This many bare references were written as `[0]`.
    Normalized(usize),
}

/// One printed record.
#[derive(Clone, Debug)]
pub(crate) struct Printed {
    pub text: String,
    pub notes: Vec<PrintNote>,
}

/// Print one stored record as a statement (a block for `core.batch`). The record is printed as given; the recorder
/// migrates older versions first.
///
/// Batch member variables are named `b0m<i>`. Notes (a `Raw` fallback) are dropped here; use `record_script` to see them.
pub fn print_op(reg: &Registry, rec: &OpRecord, style: Style) -> Result<String, PrintError> {
    Printer::new(reg).print(rec, style, false, 0).map(|p| p.text)
}

pub(crate) struct Printer<'r> {
    reg: &'r Registry,
    docs: BTreeMap<String, Option<(SchemaDoc, Vec<String>)>>,
}

/// Rendering state of one statement.
struct Rcx {
    readable: bool,
    /// `Some((ordinal, member index))` while printing a batch member.
    member: Option<(usize, usize)>,
    refs: BTreeSet<usize>,
}

const BATCH: &str = "core.batch";

impl<'r> Printer<'r> {
    pub(crate) fn new(reg: &'r Registry) -> Self {
        Self { reg, docs: BTreeMap::new() }
    }

    fn doc(&mut self, name: &str) -> Option<&(SchemaDoc, Vec<String>)> {
        let reg = self.reg;
        self.docs
            .entry(name.to_owned())
            .or_insert_with(|| {
                let schema = reg.schema(name).ok()?;
                let doc = SchemaDoc::new(schema.as_value().clone());
                let order = doc.view().params().into_iter().map(|p| p.name).collect();
                Some((doc, order))
            })
            .as_ref()
    }

    /// Print `rec`; `ordinal` numbers the batch blocks of the script so their variables are unique.
    pub(crate) fn print(&mut self, rec: &OpRecord, style: Style, normalize: bool, ordinal: usize) -> Result<Printed, PrintError> {
        if rec.name == BATCH {
            return self.print_batch(rec, style, normalize, ordinal);
        }
        let (text, _refs, notes) = self.print_call(rec, style, normalize, None)?;
        Ok(Printed { text, notes })
    }

    fn print_batch(&mut self, rec: &OpRecord, style: Style, normalize: bool, ordinal: usize) -> Result<Printed, PrintError> {
        let bad = |reason: &str| PrintError::Unprintable { node: None, path: "ops".into(), reason: reason.into() };
        let ops = rec.params.get("ops").cloned().ok_or_else(|| bad("a batch without `ops`"))?;
        let members: Vec<OpRecord> = serde_json::from_value(ops).map_err(|e| bad(&format!("`ops` is not a list of records: {e}")))?;
        if members.is_empty() {
            return Err(bad("an empty batch has no source form (it applies nothing)"));
        }
        let mut texts = Vec::with_capacity(members.len());
        let mut referenced = BTreeSet::new();
        let mut notes = Vec::new();
        for (i, m) in members.iter().enumerate() {
            if m.name == BATCH {
                return Err(bad("a batch inside a batch"));
            }
            match self.reg.op_version(&m.name) {
                Ok(v) if v == m.version => {}
                Ok(v) => return Err(bad(&format!("member {i} (`{}`) is version {}, the registry has {v}", m.name, m.version))),
                Err(_) => return Err(PrintError::UnknownOp { node: None, name: m.name.clone() }),
            }
            let (text, refs, n) = self.print_call(m, style, normalize, Some((ordinal, i))).map_err(|e| prefix_path(e, &format!("ops[{i}].")))?;
            referenced.extend(refs.into_iter().filter(|r| *r < i));
            texts.push(text);
            notes.extend(n);
        }
        let mut out = String::from("batch {\n");
        for (i, t) in texts.iter().enumerate() {
            if referenced.contains(&i) {
                out.push_str(&format!("  let {} = {t}\n", var_name(ordinal, i)));
            } else {
                out.push_str(&format!("  {t}\n"));
            }
        }
        out.push('}');
        // The whole block must still be one batch statement of the right size.
        let parsed = parse_with(0, &out, big_limits());
        let ok = !parsed.has_errors() && matches!(parsed.script.stmts.as_slice(), [s] if matches!(&s.kind, StmtKind::Batch(b) if b.stmts.len() == members.len()));
        if !ok {
            return Err(PrintError::RoundTrip { node: None, path: "ops".into() });
        }
        Ok(Printed { text: out, notes })
    }

    /// One call statement (a top-level op or a batch member), Readable with a Raw fallback.
    fn print_call(&mut self, rec: &OpRecord, style: Style, normalize: bool, member: Option<(usize, usize)>) -> Result<(String, BTreeSet<usize>, Vec<PrintNote>), PrintError> {
        if !rec.name.contains('.') {
            return Err(PrintError::Unprintable { node: None, path: String::new(), reason: format!("`{}` is not a dotted op name", rec.name) });
        }
        let Some((doc, order)) = self.doc(&rec.name).map(|(d, o)| (d.clone(), o.clone())) else {
            return Err(PrintError::UnknownOp { node: None, name: rec.name.clone() });
        };
        let view = doc.view();
        // Bare references have no source form: reject, or rewrite to `[0]` (the expected value is rewritten too).
        let mut params = rec.params.clone();
        let mut normalized = 0;
        if member.is_some() {
            normalize_bare(&mut params, Some(view), normalize, &mut normalized, &mut String::new())?;
        }
        let Json::Object(obj) = &params else {
            return Err(PrintError::Unprintable { node: None, path: String::new(), reason: "params are not an object".into() });
        };
        let mut notes = Vec::new();
        if normalized > 0 {
            notes.push(PrintNote::Normalized(normalized));
        }
        let expect = Expect { name: &rec.name, params: &params, member: member.map(|m| m.0) };
        let modes: &[bool] = if style == Style::Readable { &[true, false] } else { &[false] };
        let mut first: Option<PrintError> = None;
        for &readable in modes {
            let mut cx = Rcx { readable, member, refs: BTreeSet::new() };
            let attempt = render_call(&rec.name, obj, view, &order, &mut cx).and_then(|text| match verify_text(&text, &expect) {
                Ok(()) => Ok(text),
                Err(path) => Err(PrintError::RoundTrip { node: None, path }),
            });
            match attempt {
                Ok(text) => {
                    if let Some(PrintError::RoundTrip { path, .. }) = &first {
                        notes.push(PrintNote::RawFallback(format!("the readable form did not read back at `{path}`")));
                    }
                    return Ok((text, cx.refs, notes));
                }
                // Only a failed self-check is worth retrying as Raw; anything else fails the same way.
                Err(e @ PrintError::RoundTrip { .. }) if readable => first = Some(e),
                Err(e) => return Err(first.unwrap_or(e)),
            }
        }
        Err(first.unwrap_or(PrintError::RoundTrip { node: None, path: String::new() }))
    }
}

fn prefix_path(e: PrintError, prefix: &str) -> PrintError {
    match e {
        PrintError::BareReference { node, path } => PrintError::BareReference { node, path: format!("{prefix}{path}") },
        PrintError::RoundTrip { node, path } => PrintError::RoundTrip { node, path: format!("{prefix}{path}") },
        PrintError::Unprintable { node, path, reason } => PrintError::Unprintable { node, path: format!("{prefix}{path}"), reason },
        other => other,
    }
}

fn big_limits() -> ParseLimits {
    ParseLimits { max_source_bytes: usize::MAX / 2, max_errors: 8, ..ParseLimits::default() }
}

/// The variable a batch member is bound to when later members refer to it.
pub(crate) fn var_name(ordinal: usize, member: usize) -> String {
    format!("b{ordinal}m{member}")
}

// ---- references ------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Sel {
    Bare,
    At(usize),
    Star,
}

/// `$<i>.<role>`, `$<i>.<role>[<k>]`, `$<i>.<role>[*]` (the grammar of `sm_op_api::batch::REF_PATTERN`).
fn parse_ref(s: &str) -> Option<(usize, &str, Sel)> {
    let rest = s.strip_prefix('$')?;
    let (idx, tail) = rest.split_once('.')?;
    if idx.is_empty() || !idx.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    let idx: usize = idx.parse().ok()?;
    let (role, sel) = match tail.find('[') {
        Some(p) => {
            let k = tail[p + 1..].strip_suffix(']')?;
            let sel = if k == "*" {
                Sel::Star
            } else {
                if k.is_empty() || !k.bytes().all(|b| b.is_ascii_digit()) {
                    return None;
                }
                Sel::At(k.parse().ok()?)
            };
            (&tail[..p], sel)
        }
        None => (tail, Sel::Bare),
    };
    if role.is_empty() || role.contains(['[', ']']) {
        return None;
    }
    Some((idx, role, sel))
}

/// Inverse of the parse above, for the self-check.
fn is_var(s: &str) -> Option<(usize, usize)> {
    let rest = s.strip_prefix('b')?;
    let (o, m) = rest.split_once('m')?;
    if o.is_empty() || m.is_empty() || !o.bytes().all(|b| b.is_ascii_digit()) || !m.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    Some((o.parse().ok()?, m.parse().ok()?))
}

fn id_positions(view: Option<View<'_>>) -> (bool, Option<View<'_>>) {
    let Some(v) = view else { return (false, None) };
    let nn = v.non_null();
    (nn.is_entity_id(), nn.id_array_items())
}

fn normalize_bare(j: &mut Json, view: Option<View<'_>>, normalize: bool, count: &mut usize, path: &mut String) -> Result<(), PrintError> {
    let (is_id, id_items) = id_positions(view);
    match j {
        Json::String(s) if is_id => fix_bare(s, normalize, count, path)?,
        Json::Array(items) => {
            let nn = view.map(View::non_null);
            let item_view = nn.and_then(View::items);
            for (i, it) in items.iter_mut().enumerate() {
                let n = path.len();
                path.push_str(&format!("[{i}]"));
                match it {
                    Json::String(s) if id_items.is_some() => fix_bare(s, normalize, count, path)?,
                    other => normalize_bare(other, item_view, normalize, count, path)?,
                }
                path.truncate(n);
            }
        }
        Json::Object(m) => {
            let nn = view.map(View::non_null);
            for (k, v) in m.iter_mut() {
                let n = path.len();
                if !path.is_empty() {
                    path.push('.');
                }
                path.push_str(k);
                normalize_bare(v, nn.and_then(|n| n.property(k)), normalize, count, path)?;
                path.truncate(n);
            }
        }
        _ => {}
    }
    Ok(())
}

fn fix_bare(s: &mut String, normalize: bool, count: &mut usize, path: &str) -> Result<(), PrintError> {
    if let Some((_, _, Sel::Bare)) = parse_ref(s) {
        if !normalize {
            return Err(PrintError::BareReference { node: None, path: path.to_owned() });
        }
        s.push_str("[0]");
        *count += 1;
    }
    Ok(())
}

// ---- rendering -------------------------------------------------------------------------------

/// True for `[A-Za-z_][A-Za-z0-9_]*` that is neither a keyword nor a reserved word.
fn plain_ident(k: &str) -> bool {
    let mut c = k.chars();
    c.next().is_some_and(|f| f.is_ascii_alphabetic() || f == '_')
        && c.all(|x| x.is_ascii_alphanumeric() || x == '_')
        && keyword(k).is_none()
        && !FUTURE_WORDS.contains(&k)
}

fn key_text(k: &str) -> String {
    if plain_ident(k) { k.to_owned() } else { string_lit(k) }
}

/// A string literal using only the lexer's escapes; `\$` for a `$` that would open `${`.
pub(crate) fn string_lit(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    let mut it = s.chars().peekable();
    while let Some(c) = it.next() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\t' => out.push_str("\\t"),
            '\r' => out.push_str("\\r"),
            '$' if it.peek() == Some(&'{') => out.push_str("\\$"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{{{:x}}}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn render_call(name: &str, obj: &Map<String, Json>, view: View<'_>, order: &[String], cx: &mut Rcx) -> Result<String, PrintError> {
    let mut keys: Vec<&String> = order.iter().filter(|k| obj.contains_key(k.as_str())).collect();
    let mut rest: Vec<&String> = obj.keys().filter(|k| !order.contains(k)).collect();
    rest.sort();
    keys.extend(rest);
    let mut args = Vec::new();
    let mut spread = Vec::new();
    for k in keys {
        let mut path = k.clone();
        let v = render_value(&obj[k.as_str()], view.property(k), cx, &mut path)?;
        if cx.readable && plain_ident(k) {
            args.push(format!("{k}: {v}"));
        } else {
            spread.push(format!("{}: {v}", key_text(k)));
        }
    }
    if !spread.is_empty() {
        args.push(format!("...{{ {} }}", spread.join(", ")));
    }
    Ok(format!("{name}({})", args.join(", ")))
}

fn unprintable(path: &str, reason: impl Into<String>) -> PrintError {
    PrintError::Unprintable { node: None, path: path.to_owned(), reason: reason.into() }
}

fn render_value(j: &Json, view: Option<View<'_>>, cx: &mut Rcx, path: &mut String) -> Result<String, PrintError> {
    let (is_id, id_items) = id_positions(view);
    match j {
        Json::Null => Err(unprintable(path, "null has no source form")),
        Json::Bool(b) => Ok(b.to_string()),
        Json::Number(n) => {
            if let Some(i) = n.as_i64() {
                // `-9223372036854775808` does not lex (the magnitude overflows), so the minimum is a difference.
                Ok(if i == i64::MIN { "-9223372036854775807 - 1".to_owned() } else { i.to_string() })
            } else if n.is_u64() {
                Err(unprintable(path, "an integer beyond i64 has no source form"))
            } else {
                n.as_f64().filter(|f| f.is_finite()).map(fmt_num).ok_or_else(|| unprintable(path, "not a finite number"))
            }
        }
        Json::String(s) => render_string(s, is_id, cx, path, false),
        Json::Array(items) => {
            let nn = view.map(View::non_null);
            let item_view = nn.and_then(View::items);
            let mut parts = Vec::with_capacity(items.len());
            for (i, it) in items.iter().enumerate() {
                let n = path.len();
                path.push_str(&format!("[{i}]"));
                let v = match it {
                    Json::String(s) if id_items.is_some() => render_string(s, true, cx, path, true)?,
                    other => render_value(other, item_view, cx, path)?,
                };
                path.truncate(n);
                parts.push(v);
            }
            Ok(format!("[{}]", parts.join(", ")))
        }
        Json::Object(m) => {
            let nn = view.map(View::non_null);
            let mut keys: Vec<&String> = m.keys().collect();
            keys.sort();
            let mut parts = Vec::with_capacity(keys.len());
            for k in keys {
                let n = path.len();
                path.push('.');
                path.push_str(k);
                let v = render_value(&m[k.as_str()], nn.and_then(|n| n.property(k)), cx, path)?;
                path.truncate(n);
                parts.push(format!("{}: {v}", key_text(k)));
            }
            Ok(if parts.is_empty() { "{}".to_owned() } else { format!("{{ {} }}", parts.join(", ")) })
        }
    }
}

/// A string: in an id position a batch reference or a script id, otherwise a literal (with a batch `$$` escape undone).
fn render_string(s: &str, id_pos: bool, cx: &mut Rcx, path: &str, array_elem: bool) -> Result<String, PrintError> {
    if id_pos {
        if let Some((ordinal, member)) = cx.member
            && let Some((idx, role, sel)) = parse_ref(s)
        {
            if idx >= member {
                return Err(unprintable(path, format!("the reference `{s}` does not name an earlier member")));
            }
            if !role.chars().next().is_some_and(|c| c.is_ascii_alphabetic() || c == '_') || !role.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
                return Err(unprintable(path, format!("the role `{role}` is not an identifier")));
            }
            cx.refs.insert(idx);
            let var = var_name(ordinal, idx);
            return match sel {
                Sel::At(k) => Ok(format!("{var}.{role}[{k}]")),
                Sel::Star if array_elem => Ok(format!("{var}.{role}")),
                Sel::Star => Err(unprintable(path, "`[*]` outside an array has no source form")),
                Sel::Bare => Err(PrintError::BareReference { node: None, path: path.to_owned() }),
            };
        }
        if cx.readable
            && let Ok(id) = crate::kinds::parse_entity_text(s)
        {
            return Ok(fmt_id(id));
        }
    }
    if cx.member.is_some()
        && let Some(lit) = s.strip_prefix("$$")
    {
        return Ok(string_lit(&format!("${lit}")));
    }
    Ok(string_lit(s))
}

// ---- the self-check ----------------------------------------------------------------------------

struct Expect<'a> {
    name: &'a str,
    params: &'a Json,
    /// `Some(ordinal)` for a batch member.
    member: Option<usize>,
}

/// Read `text` back: one call statement of `expect.name` whose folded arguments equal `expect.params`.
fn verify_text(text: &str, expect: &Expect<'_>) -> Result<(), String> {
    let out = parse_with(0, text, big_limits());
    if out.has_errors() {
        return Err("<syntax>".into());
    }
    let [stmt] = out.script.stmts.as_slice() else { return Err("<statements>".into()) };
    let StmtKind::Call(call) = &stmt.kind else { return Err("<statement kind>".into()) };
    if call.path.dotted() != expect.name {
        return Err("<name>".into());
    }
    let mut got = Map::new();
    let put = |k: &str, v: Json, got: &mut Map<String, Json>| -> Result<(), String> {
        if got.insert(k.to_owned(), v).is_some() { Err(k.to_owned()) } else { Ok(()) }
    };
    for a in &call.args {
        match a {
            Arg::Named { name, value } => {
                if let Some(v) = fold(value, expect.member)? {
                    put(&name.name, v, &mut got)?;
                }
            }
            Arg::Spread(e) => match fold(e, expect.member)? {
                Some(Json::Object(m)) => {
                    for (k, v) in m {
                        put(&k, v, &mut got)?;
                    }
                }
                _ => return Err("<spread>".into()),
            },
            Arg::Pos(_) => return Err("<positional>".into()),
        }
    }
    match diff(&Json::Object(got), expect.params, String::new()) {
        None => Ok(()),
        Some(p) => Err(p),
    }
}

/// The first path at which two values differ, `None` when equal (integer and float stay distinct, `-0.0 == 0.0`).
fn diff(a: &Json, b: &Json, path: String) -> Option<String> {
    match (a, b) {
        (Json::Object(x), Json::Object(y)) => {
            let keys: BTreeSet<&String> = x.keys().chain(y.keys()).collect();
            for k in keys {
                let p = if path.is_empty() { k.clone() } else { format!("{path}.{k}") };
                match (x.get(k), y.get(k)) {
                    (Some(u), Some(v)) => {
                        if let Some(d) = diff(u, v, p) {
                            return Some(d);
                        }
                    }
                    _ => return Some(p),
                }
            }
            None
        }
        (Json::Array(x), Json::Array(y)) => {
            for (i, (u, v)) in x.iter().zip(y).enumerate() {
                if let Some(d) = diff(u, v, format!("{path}[{i}]")) {
                    return Some(d);
                }
            }
            if x.len() == y.len() { None } else { Some(format!("{path}[{}]", x.len().min(y.len()))) }
        }
        _ if a == b => None,
        _ => Some(if path.is_empty() { "<value>".into() } else { path }),
    }
}

/// Fold a constant expression to JSON the way argument lowering would (schema-less). `member` is the batch ordinal
/// when folding inside a batch: strings starting with `$` are escaped and `b<o>m<i>.role` is a reference.
/// `None` is `null` (the field is omitted).
fn fold(e: &Expr, member: Option<usize>) -> Result<Option<Json>, String> {
    let bad = || "<unsupported expression>".to_owned();
    Ok(Some(match &e.kind {
        ExprKind::Int(i) => Json::from(*i),
        ExprKind::Num(f) => Json::Number(serde_json::Number::from_f64(*f).ok_or_else(bad)?),
        ExprKind::Bool(b) => Json::Bool(*b),
        ExprKind::Str(s) => Json::String(if member.is_some() && s.starts_with('$') { format!("${s}") } else { s.clone() }),
        ExprKind::Null => return Ok(None),
        ExprKind::List(items) => {
            let mut out = Vec::with_capacity(items.len());
            for it in items {
                // An element that is a bare reference splices (`$i.role[*]`).
                let v = match &it.kind {
                    ExprKind::Field(..) => fold_ref(it, member, true)?,
                    _ => fold(it, member)?.ok_or_else(bad)?,
                };
                out.push(v);
            }
            Json::Array(out)
        }
        ExprKind::Obj(items) => {
            let mut m = Map::new();
            for (k, v) in items {
                if let Some(v) = fold(v, member)?
                    && m.insert(k.name.clone(), v).is_some()
                {
                    return Err(k.name.clone());
                }
            }
            Json::Object(m)
        }
        ExprKind::Unary(UnOp::Neg, a) => match fold(a, member)?.ok_or_else(bad)? {
            Json::Number(n) => match (n.as_i64(), n.as_f64()) {
                (Some(i), _) => Json::from(i.checked_neg().ok_or_else(bad)?),
                (None, Some(f)) if !n.is_u64() => Json::Number(serde_json::Number::from_f64(-f).ok_or_else(bad)?),
                _ => return Err(bad()),
            },
            _ => return Err(bad()),
        },
        ExprKind::Binary(BinOp::Sub, a, b) => match (fold(a, member)?.ok_or_else(bad)?, fold(b, member)?.ok_or_else(bad)?) {
            (Json::Number(x), Json::Number(y)) => match (x.as_i64(), y.as_i64()) {
                (Some(x), Some(y)) => Json::from(x.checked_sub(y).ok_or_else(bad)?),
                _ => return Err(bad()),
            },
            _ => return Err(bad()),
        },
        ExprKind::Entity(EntityExpr::Ids { kind, items, .. }) => match items.as_slice() {
            [IdItem::One(Expr { kind: ExprKind::Int(n), .. })] if *n >= 0 => {
                let id = EntityId::new(*kind, *n as u64).ok_or_else(bad)?;
                Json::String(id.to_string())
            }
            _ => return Err(bad()),
        },
        ExprKind::Field(..) | ExprKind::Index(..) => fold_ref(e, member, false)?,
        _ => return Err(bad()),
    }))
}

/// `b<o>m<i>.role` (a splice, only as an array element) or `b<o>m<i>.role[k]`.
fn fold_ref(e: &Expr, member: Option<usize>, elem: bool) -> Result<Json, String> {
    let bad = || "<reference>".to_owned();
    let (base, sel) = match &e.kind {
        ExprKind::Field(..) => (e, Sel::Star),
        ExprKind::Index(b, Index::At(k)) => match &k.kind {
            ExprKind::Int(k) if *k >= 0 => (&**b, Sel::At(*k as usize)),
            _ => return Err(bad()),
        },
        _ => return Err(bad()),
    };
    if sel == Sel::Star && !elem {
        return Err(bad());
    }
    let ExprKind::Field(v, role) = &base.kind else { return Err(bad()) };
    let ExprKind::Var(name) = &v.kind else { return Err(bad()) };
    let (o, i) = is_var(name).ok_or_else(bad)?;
    if member != Some(o) {
        return Err(bad());
    }
    Ok(Json::String(match sel {
        Sel::At(k) => format!("${i}.{}[{k}]", role.name),
        _ => format!("${i}.{}[*]", role.name),
    }))
}
