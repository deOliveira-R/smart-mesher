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
/// `ApplyCtx::alloc_as(role, ..)`. To pass a literal string that starts with `$`, double the
/// first `$` (`"$$0.price"` becomes the literal `"$0.price"`). Layers react to ids retired by op
/// `i` (via `on_retired`) before op `i + 1` runs. There is no control flow. The batch's own [`Delta`](crate::Delta)
/// outputs carry every sub-op output as `"<i>.<role>"`. A `core.batch` inside a `core.batch` is
/// rejected (reference scoping would be ambiguous).
///
/// The stored record is canonical: every nested record is decoded after substitution and
/// re-encoded (current version, unknown fields dropped), with the original reference strings
/// restored, so equal batches are equal nodes whichever client built them.
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
            substitute(&mut sub.params, &done)?;
            let (outputs, canon) = ctx.apply_nested(&sub, i, total)?;
            // Store the canonical form (typed re-encode) with the original references put back.
            // Fields the op does not know are gone from it, so their references are skipped. If a
            // migration reshaped the params, the canonical record keeps concrete ids instead
            // (replay reproduces it exactly); the raw client record is never stored.
            let mut canon = canon;
            restore_refs(&rec.params, &sub.params, &mut canon);
            canonical.push(canon);
            done.push(outputs);
        }
        ctx.step(total as u64, total as u64, "batch")?;
        ctx.set_canonical_record(OpRecord::of(&Batch { ops: canonical })?);
        Ok(())
    }
}

#[derive(Clone)]
enum Seg {
    Key(String),
    Idx(usize),
}

/// Collect the places where substitution changed a string: (path, original, substituted).
fn diffs(
    orig: &Value,
    subst: &Value,
    path: &mut Vec<Seg>,
    out: &mut Vec<(Vec<Seg>, String, String)>,
) {
    match (orig, subst) {
        (Value::String(o), Value::String(s)) if o != s => {
            out.push((path.clone(), o.clone(), s.clone()));
        }
        (Value::Array(o), Value::Array(s)) => {
            for (i, (a, b)) in o.iter().zip(s).enumerate() {
                path.push(Seg::Idx(i));
                diffs(a, b, path, out);
                path.pop();
            }
        }
        (Value::Object(o), Value::Object(s)) => {
            for (k, a) in o {
                if let Some(b) = s.get(k) {
                    path.push(Seg::Key(k.clone()));
                    diffs(a, b, path, out);
                    path.pop();
                }
            }
        }
        _ => {}
    }
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

/// Put the original reference strings (`$i.role`, `$$literal`) back into `canon` at the paths
/// where substitution replaced them. A path absent from `canon` is skipped: the op does not know
/// that field, so the canonical re-encode dropped it and there is nothing to restore. If a path
/// is present but holds a different value (a migration reshaped the params), nothing is changed
/// and `false` is returned: the caller then stores `canon` with concrete ids, which replays
/// exactly (it never stores the raw client record).
fn restore_refs(orig: &Value, subst: &Value, canon: &mut OpRecord) -> bool {
    let mut found = Vec::new();
    diffs(orig, subst, &mut Vec::new(), &mut found);
    let matches = found
        .iter()
        .all(|(path, _, s)| at(&canon.params, path).is_none_or(|v| v.as_str() == Some(s.as_str())));
    // After substitution every string is a literal: escape the `$`-prefixed ones so replay
    // does not read them as references (this also covers the concrete-id fallback).
    escape_literals(&mut canon.params);
    if !matches {
        return false;
    }
    for (path, o, _) in found {
        if let Some(v) = at_mut(&mut canon.params, &path) {
            *v = Value::String(o);
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

fn reference(s: &str) -> Option<(usize, &str, usize)> {
    let rest = s.strip_prefix('$')?;
    let (idx, tail) = rest.split_once('.')?;
    let idx: usize = idx.parse().ok()?;
    match tail.split_once('[') {
        Some((role, k)) => Some((idx, role, k.strip_suffix(']')?.parse().ok()?)),
        None => Some((idx, tail, 0)),
    }
}

fn substitute(v: &mut Value, done: &Outputs) -> Result<(), OpError> {
    match v {
        Value::String(s) => {
            if s.starts_with("$$") {
                s.remove(0);
            } else if let Some((i, role, k)) = reference(s) {
                let id = done
                    .get(i)
                    .ok_or_else(|| {
                        OpError::Invalid(format!(
                            "batch reference '{s}' points to an op that has not run"
                        ))
                    })?
                    .get(role)
                    .and_then(|ids| ids.get(k))
                    .ok_or_else(|| {
                        OpError::Invalid(format!("batch reference '{s}': no such output"))
                    })?;
                *s = id.to_string();
            }
            Ok(())
        }
        Value::Array(a) => a.iter_mut().try_for_each(|x| substitute(x, done)),
        Value::Object(m) => m.values_mut().try_for_each(|x| substitute(x, done)),
        _ => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use sm_core::EntityKind;

    #[test]
    fn parses_references() {
        assert_eq!(reference("$0.volume"), Some((0, "volume", 0)));
        assert_eq!(reference("$12.face[3]"), Some((12, "face", 3)));
        assert_eq!(reference("$x.volume"), None);
        assert_eq!(reference("volume"), None);
    }

    #[test]
    fn restore_skips_dropped_fields_and_falls_back_on_mismatch() {
        let rec = |p: Value| OpRecord {
            name: "x".into(),
            version: 1,
            params: p,
        };
        let orig = json!({"id": "$0.body", "junk": "$0.body"});
        let subst = json!({"id": "Body#1", "junk": "Body#1"});
        // `junk` dropped by the canonical re-encode: skipped, `id` restored.
        let mut c = rec(json!({"id": "Body#1"}));
        assert!(restore_refs(&orig, &subst, &mut c));
        assert_eq!(c.params, json!({"id": "$0.body"}));
        // Present but different (migration reshaped it): unchanged, concrete ids kept.
        let mut c = rec(json!({"id": "Body#1", "junk": "Other#2"}));
        assert!(!restore_refs(&orig, &subst, &mut c));
        assert_eq!(c.params, json!({"id": "Body#1", "junk": "Other#2"}));
    }

    #[test]
    fn substitutes_nested() {
        let id = EntityId::new(EntityKind::TopoVolume, 4).unwrap();
        let done: Outputs = vec![BTreeMap::from([("volume".to_owned(), vec![id])])];
        let mut v = json!({"a": ["$0.volume", "plain"], "b": {"c": "$0.volume[0]"}});
        substitute(&mut v, &done).unwrap();
        assert_eq!(
            v,
            json!({"a": ["TopoVolume#4", "plain"], "b": {"c": "TopoVolume#4"}})
        );
        assert!(substitute(&mut json!("$1.volume"), &done).is_err());
        assert!(substitute(&mut json!("$0.nope"), &done).is_err());
        let mut lit = json!({"a": "$$0.volume", "b": "$$$x"});
        substitute(&mut lit, &done).unwrap();
        assert_eq!(lit, json!({"a": "$0.volume", "b": "$$x"}));
    }
}
