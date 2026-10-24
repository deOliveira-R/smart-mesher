//! Pure expression semantics: arithmetic, comparison, set algebra and builtins.
//!
//! Errors are `(code, message)` pairs; the interpreter attaches the span. Every float result is
//! checked finite (`non-finite`), so no `NaN`/`inf` can reach an op record.

use std::cmp::Ordering;

use crate::ast::{BinOp, UnOp};
use crate::entity::EntitySet;
use crate::value::Val;

/// An evaluation failure before a span is attached: `(code, message)`.
pub type EvalErr = (&'static str, String);
/// Result of a pure evaluation step.
pub type EvalRes<T> = Result<T, EvalErr>;

/// Builtin function and constant names (not reserved, but cannot be op names, which are dotted).
pub const BUILTIN_NAMES: &[&str] = &[
    "abs", "min", "max", "floor", "ceil", "round", "sqrt", "sin", "cos", "tan", "asin", "acos", "atan", "atan2", "exp", "ln", "log10", "pow", "int",
    "float", "len", "count", "range", "str", "ents", "contains", "pi", "tau",
];

fn finite(v: f64) -> EvalRes<Val> {
    if v.is_finite() { Ok(Val::Num(v)) } else { Err(("non-finite", "the result is not a finite number".into())) }
}

fn type_err<T>(msg: String) -> EvalRes<T> {
    Err(("type-mismatch", msg))
}

fn as_f64(v: &Val) -> Option<f64> {
    match v {
        Val::Int(i) => Some(*i as f64),
        Val::Num(n) => Some(*n),
        _ => None,
    }
}

fn opaque(v: &Val) -> bool {
    matches!(v, Val::Outcome(_) | Val::Ref { .. } | Val::Placeholder { .. })
}

/// Why a value cannot take part in an operation: symbolic values are `not-concrete` / `plan-unresolved`.
pub fn opaque_err(v: &Val) -> EvalErr {
    match v {
        Val::Placeholder { .. } => ("plan-unresolved", "this value is unknown when planning without executing".into()),
        Val::Ref { .. } | Val::Outcome(crate::value::Outcome::Batch { .. } | crate::value::Outcome::Applied { .. }) => {
            ("not-concrete", "a batch output is only known after the batch is applied; it can only be passed to an op argument".into())
        }
        Val::Outcome(crate::value::Outcome::Plan { .. }) => ("plan-unresolved", "this op result is unknown when planning without executing".into()),
        other => ("type-mismatch", format!("unexpected {}", other.type_name())),
    }
}

/// Unary operators.
pub fn unop(op: UnOp, v: &Val) -> EvalRes<Val> {
    match (op, v) {
        (UnOp::Neg, Val::Int(i)) => i.checked_neg().map(Val::Int).ok_or(("overflow", "integer overflow".into())),
        (UnOp::Neg, Val::Num(n)) => Ok(Val::Num(-n)),
        (UnOp::Not, Val::Bool(b)) => Ok(Val::Bool(!b)),
        (_, v) if opaque(v) => Err(opaque_err(v)),
        (UnOp::Neg, v) => type_err(format!("cannot negate {}", v.a_type())),
        (UnOp::Not, v) => type_err(format!("`not` needs a bool, found {}", v.a_type())),
    }
}

/// Binary operators except the short-circuit `and`/`or` (the interpreter evaluates those lazily). No size limit.
pub fn binop(op: BinOp, l: &Val, r: &Val) -> EvalRes<Val> {
    binop_lim(op, l, r, usize::MAX)
}

fn too_long(what: &str, len: usize, max: usize) -> EvalErr {
    ("value-too-long", format!("{what} of {len} would exceed the limit of {max}"))
}

/// Exact order of an integer and a float (no rounding of the integer); `None` for NaN.
pub fn cmp_int_num(i: i64, f: f64) -> Option<Ordering> {
    if f.is_nan() {
        return None;
    }
    const TWO63: f64 = (1u64 << 63) as f64;
    if f >= TWO63 {
        return Some(Ordering::Less);
    }
    if f < -TWO63 {
        return Some(Ordering::Greater);
    }
    let t = f.trunc();
    Some(match i.cmp(&(t as i64)) {
        Ordering::Equal => 0.0f64.partial_cmp(&(f - t)).unwrap_or(Ordering::Equal),
        o => o,
    })
}

/// Like [`binop`], refusing results where a string or list would be longer than `max_len`.
pub fn binop_lim(op: BinOp, l: &Val, r: &Val, max_len: usize) -> EvalRes<Val> {
    for v in [l, r] {
        if opaque(v) {
            return Err(opaque_err(v));
        }
    }
    use BinOp::*;
    match op {
        Add => match (l, r) {
            (Val::Int(a), Val::Int(b)) => a.checked_add(*b).map(Val::Int).ok_or(("overflow", "integer overflow in `+`".into())),
            (Val::Str(a), Val::Str(b)) => {
                if a.len().saturating_add(b.len()) > max_len {
                    return Err(too_long("a string", a.len().saturating_add(b.len()), max_len));
                }
                Ok(Val::Str(format!("{a}{b}")))
            }
            (Val::List(a), Val::List(b)) => {
                if a.len().saturating_add(b.len()) > max_len {
                    return Err(too_long("a list", a.len().saturating_add(b.len()), max_len));
                }
                Ok(Val::list(a.iter().chain(b.iter()).cloned().collect()))
            }
            (Val::Ents(_), Val::Ents(_)) => type_err("`+` is not defined for entity sets; use `|` for union (or `-` for difference)".into()),
            _ => num2(l, r, "+", |a, b| a + b),
        },
        Sub => match (l, r) {
            (Val::Int(a), Val::Int(b)) => a.checked_sub(*b).map(Val::Int).ok_or(("overflow", "integer overflow in `-`".into())),
            (Val::Ents(a), Val::Ents(b)) => Ok(Val::Ents(a.difference(b))),
            _ => num2(l, r, "-", |a, b| a - b),
        },
        Except => match (l, r) {
            (Val::Ents(a), Val::Ents(b)) => Ok(Val::Ents(a.difference(b))),
            _ => type_err(format!("`except` needs two entity sets, found {} and {}", l.a_type(), r.a_type())),
        },
        Mul => match (l, r) {
            (Val::Int(a), Val::Int(b)) => a.checked_mul(*b).map(Val::Int).ok_or(("overflow", "integer overflow in `*`".into())),
            _ => num2(l, r, "*", |a, b| a * b),
        },
        Div => match (as_f64(l), as_f64(r)) {
            (Some(a), Some(b)) => {
                if b == 0.0 {
                    Err(("division-by-zero", "division by zero".into()))
                } else {
                    finite(a / b)
                }
            }
            _ => type_err(format!("cannot divide {} by {}", l.a_type(), r.a_type())),
        },
        Rem => match (l, r) {
            (Val::Int(_), Val::Int(0)) => Err(("division-by-zero", "remainder by zero".into())),
            (Val::Int(a), Val::Int(b)) => a.checked_rem(*b).map(Val::Int).ok_or(("overflow", "integer overflow in `%`".into())),
            _ => match (as_f64(l), as_f64(r)) {
                (Some(a), Some(b)) => {
                    if b == 0.0 {
                        Err(("division-by-zero", "remainder by zero".into()))
                    } else {
                        finite(a % b)
                    }
                }
                _ => type_err(format!("cannot take {} modulo {}", l.a_type(), r.a_type())),
            },
        },
        Pow => match (l, r) {
            (Val::Int(a), Val::Int(b)) if *b >= 0 => {
                let e = u32::try_from(*b).map_err(|_| ("overflow", "exponent too large".to_owned()))?;
                a.checked_pow(e).map(Val::Int).ok_or(("overflow", "integer overflow in `**`".into()))
            }
            _ => match (as_f64(l), as_f64(r)) {
                (Some(a), Some(b)) => finite(a.powf(b)),
                _ => type_err(format!("cannot raise {} to {}", l.a_type(), r.a_type())),
            },
        },
        Eq => Ok(Val::Bool(equal(l, r)?)),
        Ne => Ok(Val::Bool(!equal(l, r)?)),
        Lt | Le | Gt | Ge => {
            let ord = match (l, r) {
                (Val::Str(a), Val::Str(b)) => a.cmp(b),
                (Val::Int(a), Val::Int(b)) => a.cmp(b),
                (Val::Int(a), Val::Num(b)) => cmp_int_num(*a, *b).ok_or(("non-finite", "cannot compare NaN".to_owned()))?,
                (Val::Num(a), Val::Int(b)) => cmp_int_num(*b, *a).ok_or(("non-finite", "cannot compare NaN".to_owned()))?.reverse(),
                _ => match (l, r) {
                    (Val::Num(a), Val::Num(b)) => a.partial_cmp(b).ok_or(("non-finite", "cannot compare NaN".to_owned()))?,
                    _ => return type_err(format!("cannot order {} and {}", l.a_type(), r.a_type())),
                },
            };
            Ok(Val::Bool(match op {
                Lt => ord.is_lt(),
                Le => ord.is_le(),
                Gt => ord.is_gt(),
                _ => ord.is_ge(),
            }))
        }
        Union | Inter => match (l, r) {
            (Val::Ents(a), Val::Ents(b)) => Ok(Val::Ents(if op == Union { a.union(b) } else { a.intersection(b) })),
            _ => type_err(format!("`{}` needs two entity sets, found {} and {}", if op == Union { "|" } else { "&" }, l.a_type(), r.a_type())),
        },
        And | Or => type_err("`and`/`or` are evaluated lazily".into()),
    }
}

fn num2(l: &Val, r: &Val, sym: &str, f: fn(f64, f64) -> f64) -> EvalRes<Val> {
    match (as_f64(l), as_f64(r)) {
        (Some(a), Some(b)) => finite(f(a, b)),
        _ => type_err(format!("`{sym}` is not defined for {} and {}", l.a_type(), r.a_type())),
    }
}

/// Structural equality; ints and floats compare numerically; different types are unequal.
pub fn equal(l: &Val, r: &Val) -> EvalRes<bool> {
    Ok(match (l, r) {
        (Val::Int(a), Val::Int(b)) => a == b,
        (Val::Int(a), Val::Num(b)) => cmp_int_num(*a, *b) == Some(Ordering::Equal),
        (Val::Num(a), Val::Int(b)) => cmp_int_num(*b, *a) == Some(Ordering::Equal),
        (Val::Num(a), Val::Num(b)) => a == b,
        (Val::Bool(a), Val::Bool(b)) => a == b,
        (Val::Str(a), Val::Str(b)) => a == b,
        (Val::Ents(a), Val::Ents(b)) => a == b,
        (Val::List(a), Val::List(b)) => {
            if a.len() != b.len() {
                return Ok(false);
            }
            for (x, y) in a.iter().zip(b.iter()) {
                if !equal(x, y)? {
                    return Ok(false);
                }
            }
            true
        }
        (Val::Obj(a), Val::Obj(b)) => {
            if a.len() != b.len() || a.keys().ne(b.keys()) {
                return Ok(false);
            }
            for (x, y) in a.values().zip(b.values()) {
                if !equal(x, y)? {
                    return Ok(false);
                }
            }
            true
        }
        (a, _) if opaque(a) => return Err(opaque_err(a)),
        (_, b) if opaque(b) => return Err(opaque_err(b)),
        _ => false,
    })
}

fn want_num(name: &str, v: &Val) -> EvalRes<f64> {
    match v {
        Val::Int(i) => Ok(*i as f64),
        Val::Num(n) => Ok(*n),
        o if opaque(o) => Err(opaque_err(o)),
        o => type_err(format!("`{name}` needs a number, found {}", o.a_type())),
    }
}

fn arity(name: &str, args: &[Val], n: usize) -> EvalRes<()> {
    if args.len() == n { Ok(()) } else { Err(("bad-arguments", format!("`{name}` takes {n} argument{}, found {}", if n == 1 { "" } else { "s" }, args.len()))) }
}

fn f2i(v: f64) -> EvalRes<Val> {
    if v.is_finite() && (-9.223_372_036_854_775e18..=9.223_372_036_854_775e18).contains(&v) {
        Ok(Val::Int(v as i64))
    } else {
        Err(("overflow", "the value does not fit in an integer".into()))
    }
}

/// Value of a builtin constant name.
pub fn constant(name: &str) -> Option<Val> {
    match name {
        "pi" => Some(Val::Num(std::f64::consts::PI)),
        "tau" => Some(Val::Num(std::f64::consts::TAU)),
        _ => None,
    }
}

/// Call a builtin function. `max_range` bounds `range`.
pub fn call_builtin(name: &str, args: &[Val], max_range: u64) -> EvalRes<Val> {
    call_builtin_lim(name, args, max_range, usize::MAX)
}

/// Like [`call_builtin`], also bounding the length of strings and lists a builtin may build (`max_len`).
pub fn call_builtin_lim(name: &str, args: &[Val], max_range: u64, max_len: usize) -> EvalRes<Val> {
    for a in args {
        if opaque(a) && !matches!(name, "str") {
            return Err(opaque_err(a));
        }
    }
    let math1 = |f: fn(f64) -> f64| -> EvalRes<Val> {
        arity(name, args, 1)?;
        finite(f(want_num(name, &args[0])?))
    };
    match name {
        "abs" => {
            arity(name, args, 1)?;
            match &args[0] {
                Val::Int(i) => i.checked_abs().map(Val::Int).ok_or(("overflow", "integer overflow in `abs`".into())),
                v => finite(want_num(name, v)?.abs()),
            }
        }
        "min" | "max" => {
            let items: Vec<Val> = match args {
                [Val::List(l)] => l.to_vec(),
                _ => args.to_vec(),
            };
            if items.is_empty() {
                return Err(("bad-arguments", format!("`{name}` needs at least one value")));
            }
            if items.iter().all(|v| matches!(v, Val::Int(_))) {
                let ints = items.iter().filter_map(|v| if let Val::Int(i) = v { Some(*i) } else { None });
                let best = if name == "min" { ints.min() } else { ints.max() };
                return best.map(Val::Int).ok_or(("bad-arguments", format!("`{name}` needs at least one value")));
            }
            let mut best = want_num(name, &items[0])?;
            for v in &items[1..] {
                let x = want_num(name, v)?;
                if (name == "min" && x < best) || (name != "min" && x > best) {
                    best = x;
                }
            }
            Ok(Val::Num(best))
        }
        "floor" => {
            arity(name, args, 1)?;
            match &args[0] {
                Val::Int(i) => Ok(Val::Int(*i)),
                v => f2i(want_num(name, v)?.floor()),
            }
        }
        "ceil" => {
            arity(name, args, 1)?;
            match &args[0] {
                Val::Int(i) => Ok(Val::Int(*i)),
                v => f2i(want_num(name, v)?.ceil()),
            }
        }
        "round" => {
            arity(name, args, 1)?;
            match &args[0] {
                Val::Int(i) => Ok(Val::Int(*i)),
                v => f2i(want_num(name, v)?.round()),
            }
        }
        "int" => {
            arity(name, args, 1)?;
            match &args[0] {
                Val::Int(i) => Ok(Val::Int(*i)),
                Val::Num(n) => f2i(n.trunc()),
                Val::Bool(b) => Ok(Val::Int(i64::from(*b))),
                Val::Str(s) => s.trim().parse::<i64>().map(Val::Int).map_err(|_| ("bad-arguments", format!("cannot convert {s:?} to an integer"))),
                v => type_err(format!("`int` cannot convert {}", v.a_type())),
            }
        }
        "float" => {
            arity(name, args, 1)?;
            match &args[0] {
                Val::Str(s) => s.trim().parse::<f64>().map_err(|_| ("bad-arguments", format!("cannot convert {s:?} to a number"))).and_then(finite),
                v => finite(want_num(name, v)?),
            }
        }
        "sqrt" => math1(f64::sqrt),
        "sin" => math1(f64::sin),
        "cos" => math1(f64::cos),
        "tan" => math1(f64::tan),
        "asin" => math1(f64::asin),
        "acos" => math1(f64::acos),
        "atan" => math1(f64::atan),
        "exp" => math1(f64::exp),
        "ln" => math1(f64::ln),
        "log10" => math1(f64::log10),
        "atan2" => {
            arity(name, args, 2)?;
            finite(want_num(name, &args[0])?.atan2(want_num(name, &args[1])?))
        }
        "pow" => {
            arity(name, args, 2)?;
            binop(BinOp::Pow, &args[0], &args[1])
        }
        "len" => {
            arity(name, args, 1)?;
            Ok(Val::Int(match &args[0] {
                Val::List(l) => l.len(),
                Val::Str(s) => s.chars().count(),
                Val::Obj(o) => o.len(),
                Val::Ents(e) => e.len(),
                v => return type_err(format!("`len` needs a list, string, object or entity set, found {}", v.a_type())),
            } as i64))
        }
        "count" => {
            arity(name, args, 1)?;
            match &args[0] {
                Val::Ents(e) => Ok(Val::Int(e.len() as i64)),
                v => type_err(format!("`count` needs an entity set, found {}", v.a_type())),
            }
        }
        "range" => {
            if !(2..=3).contains(&args.len()) {
                return Err(("bad-arguments", "`range` takes (start, end) or (start, end, step)".into()));
            }
            let int = |v: &Val| match v {
                Val::Int(i) => Ok(*i),
                v => type_err(format!("`range` needs integers, found {}", v.a_type())),
            };
            let (a, b) = (int(&args[0])?, int(&args[1])?);
            let step = if args.len() == 3 { int(&args[2])? } else { 1 };
            if step == 0 {
                return Err(("bad-arguments", "`range` step must not be zero".into()));
            }
            let len = if (step > 0 && a < b) || (step < 0 && a > b) {
                let diff = (i128::from(b) - i128::from(a)).unsigned_abs();
                diff.div_ceil(i128::from(step).unsigned_abs())
            } else {
                0
            };
            if len > u128::from(max_range) {
                return Err(("range-too-long", format!("range has {len} elements, the limit is {max_range}")));
            }
            let mut v = Vec::with_capacity(len as usize);
            let mut x = a;
            for _ in 0..len {
                v.push(Val::Int(x));
                x = x.wrapping_add(step);
            }
            Ok(Val::list(v))
        }
        "str" => {
            arity(name, args, 1)?;
            match &args[0] {
                v if opaque(v) => Err(opaque_err(v)),
                v => {
                    let s = v.to_string();
                    if s.len() > max_len {
                        return Err(too_long("a string", s.len(), max_len));
                    }
                    Ok(Val::Str(s))
                }
            }
        }
        "contains" => {
            arity(name, args, 2)?;
            match (&args[0], &args[1]) {
                (Val::Ents(s), Val::Ents(x)) => Ok(Val::Bool(!x.is_empty() && x.iter().all(|i| s.contains(*i)))),
                (Val::Ents(s), Val::Str(x)) => Ok(Val::Bool(crate::kinds::parse_entity_text(x).is_ok_and(|i| s.contains(i)))),
                (Val::List(l), x) => {
                    for y in l.iter() {
                        if equal(y, x)? {
                            return Ok(Val::Bool(true));
                        }
                    }
                    Ok(Val::Bool(false))
                }
                (Val::Str(h), Val::Str(n)) => Ok(Val::Bool(h.contains(n.as_str()))),
                (Val::Obj(m), Val::Str(k)) => Ok(Val::Bool(m.contains_key(k))),
                (a, b) => type_err(format!("`contains` takes (entity set, entity set), (list, value), (string, string) or (object, key); found {} and {}", a.a_type(), b.a_type())),
            }
        }
        "ents" => {
            arity(name, args, 1)?;
            let one = |v: &Val, out: &mut Vec<sm_core::EntityId>| -> EvalRes<()> {
                match v {
                    Val::Str(s) => out.push(crate::kinds::parse_entity_text(s).map_err(|m| ("bad-arguments", m))?),
                    Val::Ents(e) => out.extend_from_slice(e.as_slice()),
                    v => return type_err(format!("`ents` needs id strings or entity sets, found {}", v.a_type())),
                }
                Ok(())
            };
            let mut out = Vec::new();
            match &args[0] {
                Val::List(l) => {
                    for v in l.iter() {
                        one(v, &mut out)?;
                    }
                }
                v => one(v, &mut out)?,
            }
            Ok(Val::Ents(EntitySet::from_vec(out)))
        }
        _ => Err(("unknown-function", format!("unknown function `{name}`"))),
    }
}
