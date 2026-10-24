//! Runtime values of the language.

use std::collections::BTreeMap;
use std::fmt;
use std::sync::Arc;

use serde_json::Value as Json;
use sm_core::EntityId;

use crate::kinds;
use crate::entity::EntitySet;

/// Which outputs of a symbolic batch member a reference selects.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sel {
    /// `b.role`: every output of the role (a scalar id parameter takes the first).
    All,
    /// `b.role[k]`
    At(usize),
    /// `b.role[*]`
    Star,
}

/// The result of an op statement.
#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    /// Executed: the op's `Delta`.
    Concrete {
        /// Ids created, in the order the op reported them (an ordered list, not a sorted set).
        created: EntitySet,
        /// Ids removed, in the order the op reported them.
        removed: EntitySet,
        /// Named outputs by role, each in the op's order.
        outputs: BTreeMap<String, EntitySet>,
    },
    /// An op inside a batch that has been applied: only the named outputs are known (`created`/`removed` are not).
    Applied {
        /// Named outputs by role, each in the op's order.
        outputs: BTreeMap<String, EntitySet>,
    },
    /// Member `index` of the batch being lowered (batch number `batch`); becomes `Concrete` after the batch applies.
    Batch {
        /// Which batch block.
        batch: u64,
        /// Index among the batch's emitted ops.
        index: usize,
    },
    /// Plan mode: nothing was executed, so nothing is known (statement number `n`).
    Plan {
        /// Statement ordinal.
        n: usize,
    },
}

/// A language value.
#[derive(Clone, Debug, PartialEq)]
pub enum Val {
    /// Boolean.
    Bool(bool),
    /// 64-bit integer.
    Int(i64),
    /// Finite float.
    Num(f64),
    /// String.
    Str(String),
    /// List (cheap to clone).
    List(Arc<Vec<Val>>),
    /// Object with string keys, ordered by key (cheap to clone).
    Obj(Arc<BTreeMap<String, Val>>),
    /// Entity set or ordered id list (cheap to clone).
    Ents(EntitySet),
    /// Result of an op statement.
    Outcome(Outcome),
    /// A reference to output `role` of batch member `index` of batch `batch` (`"$index.role"` when lowered).
    Ref {
        /// Batch number.
        batch: u64,
        /// Member index.
        index: usize,
        /// Role name.
        role: String,
        /// Selection.
        sel: Sel,
    },
    /// A plan-mode stand-in for an output that is unknown without executing (`"$plan.n.role"`).
    Placeholder {
        /// Statement ordinal.
        n: usize,
        /// Role or field name.
        role: String,
    },
}

impl Val {
    /// A list value.
    pub fn list(items: Vec<Val>) -> Val {
        Val::List(Arc::new(items))
    }

    /// An object value.
    pub fn obj(items: BTreeMap<String, Val>) -> Val {
        Val::Obj(Arc::new(items))
    }

    /// Short type name for messages.
    pub fn type_name(&self) -> &'static str {
        match self {
            Val::Bool(_) => "bool",
            Val::Int(_) => "int",
            Val::Num(_) => "number",
            Val::Str(_) => "string",
            Val::List(_) => "list",
            Val::Obj(_) => "object",
            Val::Ents(_) => "entity set",
            Val::Outcome(_) => "op result",
            Val::Ref { .. } => "batch reference",
            Val::Placeholder { .. } => "unknown plan value",
        }
    }

    /// The type name with its indefinite article ("an int", "a string").
    pub fn a_type(&self) -> String {
        let n = self.type_name();
        let art = if n.starts_with(['a', 'e', 'i', 'o', 'u']) { "an" } else { "a" };
        format!("{art} {n}")
    }

    /// Plain JSON, without schema guidance (entity sets become arrays of id strings). `None` for values that have no
    /// JSON form (op results, references, placeholders) or non-finite floats.
    pub fn to_json_plain(&self) -> Option<Json> {
        Some(match self {
            Val::Bool(b) => Json::Bool(*b),
            Val::Int(i) => Json::from(*i),
            Val::Num(f) => Json::Number(serde_json::Number::from_f64(*f)?),
            Val::Str(s) => Json::String(s.clone()),
            Val::List(v) => Json::Array(v.iter().map(Val::to_json_plain).collect::<Option<_>>()?),
            Val::Obj(m) => Json::Object(m.iter().map(|(k, v)| Some((k.clone(), v.to_json_plain()?))).collect::<Option<_>>()?),
            Val::Ents(s) => Json::Array(s.iter().map(|i| Json::String(i.to_string())).collect()),
            _ => return None,
        })
    }

    /// Convert a JSON value (a query result) into a language value. `Err` for `null`.
    pub fn from_json(v: &Json) -> Result<Val, String> {
        Self::from_json_limited(v, usize::MAX)
    }

    /// Like [`from_json`](Self::from_json), rejecting any string longer than `max_len` bytes or array longer than `max_len` elements.
    pub fn from_json_limited(v: &Json, max_len: usize) -> Result<Val, String> {
        Ok(match v {
            Json::Null => return Err("null has no language value".into()),
            Json::Bool(b) => Val::Bool(*b),
            Json::Number(n) => match n.as_i64() {
                Some(i) => Val::Int(i),
                None => Val::Num(n.as_f64().ok_or("number out of range")?),
            },
            Json::String(s) if s.len() > max_len => return Err(format!("a string of {} bytes exceeds the limit of {max_len}", s.len())),
            Json::String(s) => Val::Str(s.clone()),
            Json::Array(a) if a.len() > max_len => return Err(format!("a list of {} elements exceeds the limit of {max_len}", a.len())),
            Json::Array(a) => Val::list(a.iter().map(|x| Val::from_json_limited(x, max_len)).collect::<Result<_, _>>()?),
            Json::Object(m) => Val::obj(m.iter().map(|(k, x)| Ok((k.clone(), Val::from_json_limited(x, max_len)?))).collect::<Result<_, String>>()?),
        })
    }
}

/// Format a float so it reads back as a float: shortest round-trip digits, a trailing `.0` for whole numbers,
/// and an exponent for very large or small magnitudes (never hundreds of digits).
pub fn fmt_num(n: f64) -> String {
    let a = n.abs();
    if a != 0.0 && !(1e-5..1e16).contains(&a) {
        return format!("{n:e}");
    }
    let s = format!("{n}");
    if s.contains('.') { s } else { format!("{s}.0") }
}

/// An entity id spelled the way scripts write it (`Face#3`, the preferred alias of its kind).
pub fn fmt_id(id: EntityId) -> String {
    format!("{}#{}", kinds::singular_of(id.kind()), id.serial())
}

impl fmt::Display for Val {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Val::Bool(b) => write!(f, "{b}"),
            Val::Int(i) => write!(f, "{i}"),
            Val::Num(n) => f.write_str(&fmt_num(*n)),
            Val::Str(s) => f.write_str(s),
            Val::List(v) => {
                f.write_str("[")?;
                for (i, x) in v.iter().enumerate() {
                    if i > 0 {
                        f.write_str(", ")?;
                    }
                    match x {
                        Val::Str(s) => write!(f, "{s:?}")?,
                        other => write!(f, "{other}")?,
                    }
                }
                f.write_str("]")
            }
            Val::Obj(m) => {
                f.write_str("{")?;
                for (i, (k, v)) in m.iter().enumerate() {
                    if i > 0 {
                        f.write_str(", ")?;
                    }
                    match v {
                        Val::Str(s) => write!(f, "{k}: {s:?}")?,
                        other => write!(f, "{k}: {other}")?,
                    }
                }
                f.write_str("}")
            }
            Val::Ents(s) => {
                f.write_str("{")?;
                for (i, id) in s.iter().enumerate() {
                    if i > 0 {
                        f.write_str(", ")?;
                    }
                    f.write_str(&fmt_id(*id))?;
                }
                f.write_str("}")
            }
            Val::Outcome(_) => f.write_str("<op result>"),
            Val::Ref { index, role, sel, .. } => match sel {
                Sel::All => write!(f, "${index}.{role}"),
                Sel::At(k) => write!(f, "${index}.{role}[{k}]"),
                Sel::Star => write!(f, "${index}.{role}[*]"),
            },
            Val::Placeholder { n, role } => write!(f, "$plan.{n}.{role}"),
        }
    }
}
