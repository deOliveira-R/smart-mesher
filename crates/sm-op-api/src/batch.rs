//! `core.batch`: several ops as one atomic node, with named-output references between them.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use std::collections::BTreeMap;

use crate::codec::{Value, canonicalize};
use crate::error::OpError;
use crate::op::{ApplyCtx, OpRecord, TypedOp};

/// A sequence of op records applied in order inside one transaction and recorded as one node.
///
/// A string parameter of the form `"$<i>.<role>"` (or `"$<i>.<role>[<k>]"` for the k-th id of a
/// role) in op `j > i` is replaced by the id that op `i` allocated with
/// `ApplyCtx::alloc_as(role, ..)`; the bare form means `[0]`. An array element
/// `"$<i>.<role>[*]"` is replaced in place by every id of that role, in allocation order (an
/// empty role splices nothing); `[*]` anywhere other than directly as an array element is
/// `OpError::Invalid`. The grammar is [`REF_PATTERN`]. To pass a literal string that starts
/// with `$`, double the first `$` (`"$$0.price"` becomes the literal `"$0.price"`). Layers react
/// to ids retired by op `i` (via `on_retired`) before op `i + 1` runs. There is no control flow.
/// The batch's own [`Delta`](crate::Delta) outputs carry every sub-op output as `"<i>.<role>"`.
/// A `core.batch` inside a `core.batch` is rejected (reference scoping would be ambiguous); use
/// [`Batch::concat`] to flatten batches instead.
///
/// The stored record is canonical: every nested record is decoded after substitution and
/// re-encoded (current version, unknown fields dropped), with the original reference strings
/// restored (a spliced `[*]` run folds back into its single reference string), so equal batches
/// are equal nodes whichever client built them.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct Batch {
    /// The ops, in application order. Params are canonicalized (nulls dropped) on encoding.
    #[serde(serialize_with = "ser_ops")]
    pub ops: Vec<OpRecord>,
}

fn ser_ops<S: serde::Serializer>(ops: &[OpRecord], ser: S) -> Result<S::Ok, S::Error> {
    let mut canon = Vec::with_capacity(ops.len());
    for r in ops {
        let mut r = r.clone();
        canonicalize(&mut r.params).map_err(serde::ser::Error::custom)?;
        canon.push(r);
    }
    canon.serialize(ser)
}

type Outputs = Vec<BTreeMap<String, Vec<EntityId>>>;

/// JSON-schema pattern of a batch reference string: `$<i>.<role>`, `$<i>.<role>[<k>]` or
/// `$<i>.<role>[*]`. `<i>` and `<k>` are ASCII digits; `<role>` is non-empty and has no `[` or
/// `]` (it may contain `.`, like a batch's own `"<i>.<role>"` outputs). This is exactly the
/// grammar `core.batch` parses; any other string (except a `$$` escape) is a literal. `[*]` is
/// only valid as an array element (see [`Batch`]).
pub const REF_PATTERN: &str = r"^\$[0-9]+\.[^\[\]]+(\[([0-9]+|\*)\])?$";

/// JSON-schema pattern of an escaped literal inside a batch (`$$...` stands for `$...`).
pub const ESCAPED_LITERAL_PATTERN: &str = r"^\$\$";

impl TypedOp for Batch {
    const NAME: &'static str = "core.batch";

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let total = self.ops.len();
        let mut done: Outputs = Vec::with_capacity(total);
        let mut canonical: Vec<OpRecord> = Vec::with_capacity(total);
        for (i, rec) in self.ops.iter().enumerate() {
            ctx.step(i as u64, total as u64, &rec.name)?;
            if rec.name == Self::NAME {
                return Err(OpError::Invalid(
                    "core.batch cannot be nested inside core.batch".into(),
                ));
            }
            let mut sub = rec.clone();
            let mut rewrites = Vec::new();
            substitute(&mut sub.params, &done, &mut Vec::new(), &mut rewrites)?;
            let (outputs, canon) = ctx.apply_nested(&sub, i, total)?;
            // Store the canonical form (typed re-encode) with the original references put back.
            // Fields the op does not know are gone from it, so their references are skipped. If a
            // migration reshaped the params, the canonical record keeps concrete ids instead
            // (replay reproduces it exactly); the raw client record is never stored.
            let mut canon = canon;
            restore_refs(&rewrites, &mut canon);
            canonical.push(canon);
            done.push(outputs);
        }
        ctx.step(total as u64, total as u64, "batch")?;
        ctx.set_canonical_record(OpRecord::of(&Batch { ops: canonical })?);
        Ok(())
    }
}

impl Batch {
    /// Flatten `records` into one batch whose application gives the same state as applying
    /// `records` one after another (a front end squashing an undo group into one node).
    ///
    /// * A `core.batch` record is inlined in order; every `$i.` reference in its members
    ///   (including `[k]` and `[*]`) is rebased by the position its batch starts at in the
    ///   result. A batch nested inside a batch is inlined recursively, and a reference to its
    ///   `"<j>.<role>"` output is rewritten to the inlined op `j`.
    /// * `$$` escapes inside batch members are kept byte-for-byte.
    /// * Any other record is copied unchanged, except that a string starting with `$` is
    ///   escaped to `$$...`: applied on its own it was a literal, and inside a batch it would
    ///   otherwise be read as a reference.
    ///
    /// Errors (`OpError::Invalid`): a batch record whose version is not the current one, batch
    /// params that do not decode, or a reference that does not name an earlier op's output.
    pub fn concat(records: &[OpRecord]) -> Result<Batch, OpError> {
        let mut ops = Vec::new();
        for r in records {
            if r.name == Self::NAME {
                let b = decode_batch(r)?;
                flatten_into(&b.ops, &mut ops)?;
            } else {
                let mut r = r.clone();
                escape_literals(&mut r.params);
                ops.push(r);
            }
        }
        Ok(Batch { ops })
    }
}

fn decode_batch(r: &OpRecord) -> Result<Batch, OpError> {
    if r.version != Batch::VERSION {
        return Err(OpError::Invalid(format!(
            "core.batch version {} is not the current version {}",
            r.version,
            Batch::VERSION
        )));
    }
    crate::codec::from_value::<Batch>(&r.params)
        .map_err(|e| OpError::Invalid(format!("core.batch params: {e}")))
}

/// Where a member of a batch being flattened ended up.
enum Slot {
    /// A plain op, now at this index of the flat list.
    Op(usize),
    /// A nested batch, whose members map as listed.
    Batch(Vec<Slot>),
}

/// Append the flattened `members` to `out`, rebasing their references; returns where each
/// member went.
fn flatten_into(members: &[OpRecord], out: &mut Vec<OpRecord>) -> Result<Vec<Slot>, OpError> {
    let mut map: Vec<Slot> = Vec::with_capacity(members.len());
    for m in members {
        if m.name == Batch::NAME {
            let inner = decode_batch(m)?;
            map.push(Slot::Batch(flatten_into(&inner.ops, out)?));
        } else {
            let mut r = m.clone();
            rebase(&mut r.params, &map)?;
            map.push(Slot::Op(out.len()));
            out.push(r);
        }
    }
    Ok(map)
}

/// Resolve op `idx` / `role` of a member list to (flat index, role).
fn resolve_slot<'r>(map: &[Slot], idx: usize, role: &'r str) -> Option<(usize, &'r str)> {
    match map.get(idx)? {
        Slot::Op(i) => Some((*i, role)),
        Slot::Batch(sub) => {
            // A batch's outputs are named "<j>.<role>".
            let (j, rest) = role.split_once('.')?;
            if rest.is_empty() {
                return None;
            }
            resolve_slot(sub, digits(j)?, rest)
        }
    }
}

fn rebase(v: &mut Value, map: &[Slot]) -> Result<(), OpError> {
    match v {
        Value::String(s) => {
            if s.starts_with("$$") {
                return Ok(());
            }
            if let Some(r) = reference(s) {
                let (idx, role) = resolve_slot(map, r.op, r.role).ok_or_else(|| {
                    OpError::Invalid(format!(
                        "batch reference '{s}' does not name an earlier op's output"
                    ))
                })?;
                let suffix = match r.sel {
                    Sel::Bare => String::new(),
                    Sel::At(k) => format!("[{k}]"),
                    Sel::All => "[*]".to_owned(),
                };
                *s = format!("${idx}.{role}{suffix}");
            }
            Ok(())
        }
        Value::Array(a) => a.iter_mut().try_for_each(|x| rebase(x, map)),
        Value::Object(m) => m.values_mut().try_for_each(|x| rebase(x, map)),
        _ => Ok(()),
    }
}

#[derive(Clone, Debug)]
enum Seg {
    Key(String),
    Idx(usize),
}

/// One place where substitution changed the params, in substitution order (pre-order, left to
/// right). Paths address the substituted value.
#[derive(Debug)]
enum Rewrite {
    /// A single reference replaced by one id string.
    One {
        path: Vec<Seg>,
        orig: String,
        subst: String,
    },
    /// A `[*]` reference spliced into the array at `path` as `ids`, starting at `start`.
    Run {
        path: Vec<Seg>,
        start: usize,
        orig: String,
        ids: Vec<String>,
    },
}

fn at<'v>(mut cur: &'v Value, path: &[Seg]) -> Option<&'v Value> {
    for seg in path {
        cur = match seg {
            Seg::Key(k) => cur.get(k.as_str())?,
            Seg::Idx(i) => cur.get(*i)?,
        };
    }
    Some(cur)
}

fn at_mut<'v>(mut cur: &'v mut Value, path: &[Seg]) -> Option<&'v mut Value> {
    for seg in path {
        cur = match seg {
            Seg::Key(k) => cur.get_mut(k.as_str())?,
            Seg::Idx(i) => cur.get_mut(*i)?,
        };
    }
    Some(cur)
}

fn run_matches(arr: &[Value], start: usize, ids: &[String]) -> bool {
    arr.get(start..start + ids.len()).is_some_and(|w| {
        w.iter()
            .zip(ids)
            .all(|(v, id)| v.as_str() == Some(id.as_str()))
    })
}

/// Put the original reference strings back into `canon` where substitution replaced them
/// (a `[*]` run folds back into its single reference string). A path absent from `canon` is
/// skipped: the op does not know that field, so the canonical re-encode dropped it and there is
/// nothing to restore. If a path is present but holds a different value (a migration reshaped
/// the params), nothing is restored and `false` is returned: the caller then stores `canon`
/// with concrete ids, which replays exactly (it never stores the raw client record).
fn restore_refs(rewrites: &[Rewrite], canon: &mut OpRecord) -> bool {
    let matches = rewrites.iter().all(|r| match r {
        Rewrite::One { path, subst, .. } => {
            at(&canon.params, path).is_none_or(|v| v.as_str() == Some(subst.as_str()))
        }
        Rewrite::Run {
            path, start, ids, ..
        } => match at(&canon.params, path) {
            None => true,
            Some(Value::Array(a)) => run_matches(a, *start, ids),
            Some(_) => false,
        },
    });
    // After substitution every string is a literal: escape the `$`-prefixed ones so replay
    // does not read them as references (this also covers the concrete-id fallback).
    escape_literals(&mut canon.params);
    if !matches {
        return false;
    }
    // Reverse substitution order: folding a run never shifts an index a path processed later
    // depends on (those paths all lie before the run).
    for r in rewrites.iter().rev() {
        match r {
            Rewrite::One { path, orig, .. } => {
                if let Some(v) = at_mut(&mut canon.params, path) {
                    *v = Value::String(orig.clone());
                }
            }
            Rewrite::Run {
                path,
                start,
                orig,
                ids,
            } => {
                if let Some(Value::Array(a)) = at_mut(&mut canon.params, path) {
                    a.splice(*start..*start + ids.len(), [Value::String(orig.clone())]);
                }
            }
        }
    }
    true
}

/// Prepend `$` to every string that starts with `$` (the inverse of the `$$` unescape).
fn escape_literals(v: &mut Value) {
    match v {
        Value::String(s) => {
            if s.starts_with('$') {
                s.insert(0, '$');
            }
        }
        Value::Array(a) => a.iter_mut().for_each(escape_literals),
        Value::Object(m) => m.values_mut().for_each(escape_literals),
        _ => {}
    }
}

/// Which ids of a role a reference names.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Sel {
    /// `$i.role`: the first id (alias of `[0]`).
    Bare,
    /// `$i.role[k]`.
    At(usize),
    /// `$i.role[*]`: every id, spliced into the enclosing array.
    All,
}

#[derive(Debug, PartialEq, Eq)]
struct Ref<'s> {
    op: usize,
    role: &'s str,
    sel: Sel,
}

fn digits(s: &str) -> Option<usize> {
    if s.is_empty() || !s.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    s.parse().ok()
}

/// True iff `s` is a batch reference: exactly the [`REF_PATTERN`] grammar that `core.batch`
/// parses (`$<i>.<role>`, `$<i>.<role>[<n>]` or `$<i>.<role>[*]`). A `$$` escape is not a
/// reference.
pub fn is_reference(s: &str) -> bool {
    reference(s).is_some()
}

/// Parse a reference string; the grammar is exactly [`REF_PATTERN`].
fn reference(s: &str) -> Option<Ref<'_>> {
    let rest = s.strip_prefix('$')?;
    let (idx, tail) = rest.split_once('.')?;
    let op = digits(idx)?;
    let (role, sel) = match tail.find('[') {
        Some(p) => {
            let k = tail[p + 1..].strip_suffix(']')?;
            let sel = if k == "*" {
                Sel::All
            } else {
                Sel::At(digits(k)?)
            };
            (&tail[..p], sel)
        }
        None => (tail, Sel::Bare),
    };
    if role.is_empty() || role.contains(['[', ']']) {
        return None;
    }
    Some(Ref { op, role, sel })
}

fn lookup<'d>(done: &'d Outputs, s: &str, r: &Ref<'_>) -> Result<&'d [EntityId], OpError> {
    let outs = done.get(r.op).ok_or_else(|| {
        OpError::Invalid(format!(
            "batch reference '{s}' points to an op that has not run"
        ))
    })?;
    Ok(outs.get(r.role).map(Vec::as_slice).unwrap_or(&[]))
}

fn substitute(
    v: &mut Value,
    done: &Outputs,
    path: &mut Vec<Seg>,
    out: &mut Vec<Rewrite>,
) -> Result<(), OpError> {
    match v {
        Value::String(s) => {
            if s.starts_with("$$") {
                s.remove(0);
            } else if let Some(r) = reference(s) {
                let k = match r.sel {
                    Sel::Bare => 0,
                    Sel::At(k) => k,
                    Sel::All => {
                        return Err(OpError::Invalid(format!(
                            "batch reference '{s}': [*] is only allowed as an array element"
                        )));
                    }
                };
                let id = lookup(done, s, &r)?.get(k).ok_or_else(|| {
                    OpError::Invalid(format!("batch reference '{s}': no such output"))
                })?;
                let subst = id.to_string();
                let orig = std::mem::replace(s, subst.clone());
                out.push(Rewrite::One {
                    path: path.clone(),
                    orig,
                    subst,
                });
            }
            Ok(())
        }
        Value::Array(a) => {
            let old = std::mem::take(a);
            a.reserve(old.len());
            for mut x in old {
                if let Value::String(s) = &x
                    && !s.starts_with("$$")
                    && let Some(r) = reference(s)
                    && r.sel == Sel::All
                {
                    let ids: Vec<String> = lookup(done, s, &r)?
                        .iter()
                        .map(|id| id.to_string())
                        .collect();
                    let start = a.len();
                    a.extend(ids.iter().cloned().map(Value::String));
                    out.push(Rewrite::Run {
                        path: path.clone(),
                        start,
                        orig: s.clone(),
                        ids,
                    });
                    continue;
                }
                path.push(Seg::Idx(a.len()));
                let r = substitute(&mut x, done, path, out);
                path.pop();
                r?;
                a.push(x);
            }
            Ok(())
        }
        Value::Object(m) => {
            for (k, x) in m.iter_mut() {
                path.push(Seg::Key(k.clone()));
                let r = substitute(x, done, path, out);
                path.pop();
                r?;
            }
            Ok(())
        }
        _ => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use sm_core::EntityKind;

    fn subst(v: &mut Value, done: &Outputs) -> Result<Vec<Rewrite>, OpError> {
        let mut rw = Vec::new();
        substitute(v, done, &mut Vec::new(), &mut rw)?;
        Ok(rw)
    }

    fn rec(p: Value) -> OpRecord {
        OpRecord {
            name: "x".into(),
            version: 1,
            params: p,
        }
    }

    #[test]
    fn parses_references() {
        let r = |op, role, sel| Some(Ref { op, role, sel });
        assert_eq!(reference("$0.volume"), r(0, "volume", Sel::Bare));
        assert_eq!(reference("$12.face[3]"), r(12, "face", Sel::At(3)));
        assert_eq!(reference("$2.piece[*]"), r(2, "piece", Sel::All));
        assert_eq!(reference("$2.0.piece[1]"), r(2, "0.piece", Sel::At(1)));
        for lit in [
            "$x.volume",
            "volume",
            "$+1.a",
            "$1.",
            "$1.a[",
            "$1.a]",
            "$1.a[x]",
            "$1.a[*]x",
            "$1.[0]",
            "$1.a[0][1]",
        ] {
            assert_eq!(reference(lit), None, "{lit}");
        }
    }

    #[test]
    fn is_reference_follows_the_grammar() {
        for ok in ["$0.price", "$12.a.b", "$0.x[3]", "$0.x[*]"] {
            assert!(is_reference(ok), "{ok}");
        }
        for bad in ["", "$", "$0", "$0.", "$.x", "$a.x", "$0.x[]", "$0.x[a]", "$0.x]", "$0.[1]", "$$0.x", "0.x", "$0.x[1]y"] {
            assert!(!is_reference(bad), "{bad}");
        }
    }

    /// `reference` and `REF_PATTERN` describe the same grammar (checked by a tiny matcher for
    /// the pattern's shape, so the two cannot drift silently).
    #[test]
    fn pattern_matches_parser() {
        assert_eq!(REF_PATTERN, r"^\$[0-9]+\.[^\[\]]+(\[([0-9]+|\*)\])?$");
        fn pattern(s: &str) -> bool {
            let Some(rest) = s.strip_prefix('$') else {
                return false;
            };
            let d = rest.bytes().take_while(u8::is_ascii_digit).count();
            if d == 0 || rest.as_bytes().get(d) != Some(&b'.') {
                return false;
            }
            let tail = &rest[d + 1..];
            let r = tail.bytes().take_while(|b| *b != b'[' && *b != b']').count();
            if r == 0 {
                return false;
            }
            let sel = &tail[r..];
            sel.is_empty()
                || sel == "[*]"
                || sel
                    .strip_prefix('[')
                    .and_then(|x| x.strip_suffix(']'))
                    .is_some_and(|k| !k.is_empty() && k.bytes().all(|b| b.is_ascii_digit()))
        }
        for s in [
            "$0.a", "$0.a[1]", "$0.a[*]", "$0.a.b", "$0.", "$a.b", "$0.a[", "$0.a]", "$0.a[]",
            "$0.a[*", "$00.a", "$0.a b", "$0.a[1]x", "$0..", "$0.a[-1]",
        ] {
            assert_eq!(reference(s).is_some(), pattern(s), "{s}");
        }
    }

    #[test]
    fn restore_skips_dropped_fields_and_falls_back_on_mismatch() {
        let id = EntityId::new(EntityKind::TopoBody, 1).unwrap();
        let done: Outputs = vec![BTreeMap::from([("body".to_owned(), vec![id])])];
        let mut v = json!({"id": "$0.body", "junk": "$0.body"});
        let rw = subst(&mut v, &done).unwrap();
        assert_eq!(v, json!({"id": "TopoBody#1", "junk": "TopoBody#1"}));
        // `junk` dropped by the canonical re-encode: skipped, `id` restored.
        let mut c = rec(json!({"id": "TopoBody#1"}));
        assert!(restore_refs(&rw, &mut c));
        assert_eq!(c.params, json!({"id": "$0.body"}));
        // Present but different (migration reshaped it): unchanged, concrete ids kept.
        let mut c = rec(json!({"id": "TopoBody#1", "junk": "Other#2"}));
        assert!(!restore_refs(&rw, &mut c));
        assert_eq!(c.params, json!({"id": "TopoBody#1", "junk": "Other#2"}));
    }

    #[test]
    fn substitutes_nested() {
        let id = EntityId::new(EntityKind::TopoVolume, 4).unwrap();
        let done: Outputs = vec![BTreeMap::from([("volume".to_owned(), vec![id])])];
        let mut v = json!({"a": ["$0.volume", "plain"], "b": {"c": "$0.volume[0]"}});
        subst(&mut v, &done).unwrap();
        assert_eq!(
            v,
            json!({"a": ["TopoVolume#4", "plain"], "b": {"c": "TopoVolume#4"}})
        );
        assert!(subst(&mut json!("$1.volume"), &done).is_err());
        assert!(subst(&mut json!("$0.nope"), &done).is_err());
        let mut lit = json!({"a": "$$0.volume", "b": "$$$x"});
        subst(&mut lit, &done).unwrap();
        assert_eq!(lit, json!({"a": "$0.volume", "b": "$$x"}));
    }

    #[test]
    fn splices_runs_and_folds_them_back() {
        let ids: Vec<EntityId> = (1..=3)
            .map(|s| EntityId::new(EntityKind::TopoVolume, s).unwrap())
            .collect();
        let done: Outputs = vec![BTreeMap::from([("piece".to_owned(), ids)])];
        let orig = json!({
            "a": ["x", "$0.piece[*]", "$0.piece[*]", ["$0.piece[*]", "$0.piece"]],
            "b": ["$0.nope[*]"],
            "c": ["$$0.piece[*]"],
        });
        let mut v = orig.clone();
        let rw = subst(&mut v, &done).unwrap();
        let t = |s: u64| format!("TopoVolume#{s}");
        assert_eq!(
            v,
            json!({
                "a": ["x", t(1), t(2), t(3), t(1), t(2), t(3), [t(1), t(2), t(3), t(1)]],
                "b": [],
                "c": ["$0.piece[*]"],
            })
        );
        let mut c = rec(v.clone());
        assert!(restore_refs(&rw, &mut c));
        assert_eq!(c.params, json!({
            "a": ["x", "$0.piece[*]", "$0.piece[*]", ["$0.piece[*]", "$0.piece"]],
            "b": ["$0.nope[*]"],
            "c": ["$$0.piece[*]"],
        }));
        // Reshaped canonical array: concrete-id fallback, literals still escaped.
        let mut short = v.clone();
        short["a"].as_array_mut().unwrap().remove(2);
        let mut c = rec(short);
        assert!(!restore_refs(&rw, &mut c));
        assert_eq!(c.params["c"], json!(["$$0.piece[*]"]));
        // `[*]` outside an array is invalid.
        assert!(subst(&mut json!({"a": "$0.piece[*]"}), &done).is_err());
        assert!(subst(&mut json!("$0.piece[*]"), &done).is_err());
    }

    #[test]
    fn concat_rebases_and_escapes() {
        let r = |name: &str, p: Value| OpRecord {
            name: name.into(),
            version: 1,
            params: p,
        };
        let b1 = OpRecord::of(&Batch {
            ops: vec![
                r("a.make", json!({})),
                r("a.use", json!({"x": "$0.p[*]", "l": "$$0.p", "y": ["$0.p[*]"]})),
            ],
        })
        .unwrap();
        let nested = OpRecord::of(&Batch {
            ops: vec![r("a.make", json!({})), b1.clone()],
        })
        .unwrap();
        let outer = OpRecord::of(&Batch {
            ops: vec![nested, r("a.use", json!({"z": "$0.1.0.p[2]", "w": "$0.0.p"}))],
        })
        .unwrap();
        let rs = vec![r("a.solo", json!({"s": "$0.lit"})), b1, outer];
        let b = Batch::concat(&rs).unwrap();
        let p: Vec<Value> = b.ops.iter().map(|o| o.params.clone()).collect();
        assert_eq!(p[0], json!({"s": "$$0.lit"}));
        assert_eq!(p[2], json!({"x": "$1.p[*]", "l": "$$0.p", "y": ["$1.p[*]"]}));
        // outer: [make(3), make(4), use(5)], then use(6).
        assert_eq!(p[5], json!({"x": "$4.p[*]", "l": "$$0.p", "y": ["$4.p[*]"]}));
        assert_eq!(p[6], json!({"z": "$4.p[2]", "w": "$3.p"}));
        assert_eq!(b.ops.len(), 7);
        // Forward references are rejected.
        let bad = OpRecord::of(&Batch {
            ops: vec![r("a.use", json!({"x": "$0.p"}))],
        })
        .unwrap();
        assert!(Batch::concat(&[bad]).is_err());
    }
}
