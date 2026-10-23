//! `core.batch`: several ops as one atomic node, with named-output references between them.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use std::collections::BTreeMap;

use crate::codec::Value;
use crate::error::OpError;
use crate::op::{ApplyCtx, OpRecord, TypedOp};

/// A sequence of op records applied in order inside one transaction and recorded as one node.
///
/// A string parameter of the form `"$<i>.<role>"` (or `"$<i>.<role>[<k>]"` for the k-th id of a
/// role) in op `j > i` is replaced by the id that op `i` allocated with
/// `ApplyCtx::alloc_as(role, ..)`. To pass a literal string that starts with `$`, double the
/// first `$` (`"$$0.price"` becomes the literal `"$0.price"`). Layers react to ids retired by op
/// `i` (via `on_retired`) before op `i + 1` runs. There is no control flow. The batch's own [`Delta`](crate::Delta)
/// outputs carry every sub-op output as `"<i>.<role>"`.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct Batch {
    /// The ops, in application order.
    pub ops: Vec<OpRecord>,
}

type Outputs = Vec<BTreeMap<String, Vec<EntityId>>>;

impl TypedOp for Batch {
    const NAME: &'static str = "core.batch";

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let total = self.ops.len() as u64;
        let mut done: Outputs = Vec::with_capacity(self.ops.len());
        for (i, rec) in self.ops.iter().enumerate() {
            ctx.step(i as u64, total, &rec.name)?;
            let mut rec = rec.clone();
            substitute(&mut rec.params, &done)?;
            done.push(ctx.apply_nested(&rec, i)?);
        }
        ctx.step(total, total, "batch")
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
                    .ok_or_else(|| OpError::Invalid(format!("batch reference '{s}' points to an op that has not run")))?
                    .get(role)
                    .and_then(|ids| ids.get(k))
                    .ok_or_else(|| OpError::Invalid(format!("batch reference '{s}': no such output")))?;
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
    fn substitutes_nested() {
        let id = EntityId::new(EntityKind::TopoVolume, 4).unwrap();
        let done: Outputs = vec![BTreeMap::from([("volume".to_owned(), vec![id])])];
        let mut v = json!({"a": ["$0.volume", "plain"], "b": {"c": "$0.volume[0]"}});
        substitute(&mut v, &done).unwrap();
        assert_eq!(v, json!({"a": ["TopoVolume#4", "plain"], "b": {"c": "TopoVolume#4"}}));
        assert!(substitute(&mut json!("$1.volume"), &done).is_err());
        assert!(substitute(&mut json!("$0.nope"), &done).is_err());
        let mut lit = json!({"a": "$$0.volume", "b": "$$$x"});
        substitute(&mut lit, &done).unwrap();
        assert_eq!(lit, json!({"a": "$0.volume", "b": "$$x"}));
    }
}
