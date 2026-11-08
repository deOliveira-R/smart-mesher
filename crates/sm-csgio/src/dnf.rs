//! Region expressions to disjunctive normal form (a union of intersections of half-spaces).

use crate::ir::{Expr, Sign};

/// Cap on the terms of one cell's DNF.
pub const MAX_TERMS: usize = 4096;

/// One half-space of a term: surface key and sign (`false` negative, `true` positive).
pub type Lit = (u32, bool);
/// An intersection: literals sorted by `(surface, sign)`, no repeats, no surface in both senses.
pub type Term = Vec<Lit>;

/// The expansion stopped at this many terms (over [`MAX_TERMS`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TooMany(pub usize);

fn positive(s: Sign) -> bool {
    s == Sign::Positive
}

/// Merge two sorted terms; `None` when they contain a surface in both senses.
fn merge(a: &Term, b: &Term) -> Option<Term> {
    let mut out: Term = Vec::with_capacity(a.len() + b.len());
    let (mut i, mut j) = (0, 0);
    while i < a.len() || j < b.len() {
        let next = if j >= b.len() || (i < a.len() && a[i] <= b[j]) {
            let x = a[i];
            i += 1;
            if j < b.len() && b[j] == x {
                j += 1;
            }
            x
        } else {
            j += 1;
            b[j - 1]
        };
        if let Some(last) = out.last()
            && last.0 == next.0
            && last.1 != next.1
        {
            return None;
        }
        out.push(next);
    }
    Some(out)
}

fn normalize(mut v: Vec<Term>) -> Vec<Term> {
    v.sort();
    v.dedup();
    v
}

fn go(e: &Expr, neg: bool) -> Result<Vec<Term>, TooMany> {
    let and = |parts: &[Expr]| -> Result<Vec<Term>, TooMany> {
        let mut acc: Vec<Term> = vec![Vec::new()];
        for p in parts {
            let rhs = go(p, neg)?;
            let mut next = Vec::with_capacity(acc.len() * rhs.len());
            for a in &acc {
                for b in &rhs {
                    if let Some(m) = merge(a, b) {
                        next.push(m);
                    }
                    if next.len() > 4 * MAX_TERMS {
                        return Err(TooMany(next.len()));
                    }
                }
            }
            acc = normalize(next);
            if acc.len() > MAX_TERMS {
                return Err(TooMany(acc.len()));
            }
        }
        Ok(acc)
    };
    let or = |parts: &[Expr]| -> Result<Vec<Term>, TooMany> {
        let mut acc: Vec<Term> = Vec::new();
        for p in parts {
            acc.extend(go(p, neg)?);
            if acc.len() > 4 * MAX_TERMS {
                return Err(TooMany(acc.len()));
            }
        }
        let acc = normalize(acc);
        if acc.len() > MAX_TERMS { Err(TooMany(acc.len())) } else { Ok(acc) }
    };
    match e {
        Expr::Half(s, sign) => Ok(vec![vec![(*s, positive(*sign) != neg)]]),
        Expr::Not(x) => go(x, !neg),
        // De Morgan: the complement of an intersection is the union of the complements.
        Expr::And(parts) => if neg { or(parts) } else { and(parts) },
        Expr::Or(parts) => if neg { and(parts) } else { or(parts) },
    }
}

/// The DNF of `e`: terms sorted, no duplicates, contradictory terms dropped. An empty result is
/// the empty region, a single empty term all of space.
///
/// # Errors
/// [`TooMany`] when an intermediate or final form exceeds [`MAX_TERMS`] terms.
pub fn to_dnf(e: &Expr) -> Result<Vec<Term>, TooMany> {
    go(e, false)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn de_morgan_and_distribution() {
        // ~(1 2) = -1 | -2 ; (1 | 2) 3 = 1 3 | 2 3
        let e = Expr::Not(Box::new(Expr::signed(&[1, 2])));
        assert_eq!(to_dnf(&e).unwrap(), vec![vec![(1, false)], vec![(2, false)]]);
        let e = Expr::And(vec![Expr::Or(vec![Expr::pos(1), Expr::pos(2)]), Expr::pos(3)]);
        assert_eq!(to_dnf(&e).unwrap(), vec![vec![(1, true), (3, true)], vec![(2, true), (3, true)]]);
    }

    #[test]
    fn contradictions_and_repeats_vanish() {
        assert!(to_dnf(&Expr::And(vec![Expr::pos(1), Expr::neg(1)])).unwrap().is_empty());
        assert_eq!(to_dnf(&Expr::And(vec![Expr::pos(1), Expr::pos(1)])).unwrap(), vec![vec![(1, true)]]);
        assert_eq!(to_dnf(&Expr::Or(vec![Expr::pos(1), Expr::pos(1)])).unwrap().len(), 1);
        assert_eq!(to_dnf(&Expr::And(vec![])).unwrap(), vec![Vec::<Lit>::new()]);
        assert!(to_dnf(&Expr::Or(vec![])).unwrap().is_empty());
    }

    #[test]
    fn precedence_is_the_callers_tree() {
        // -1 | 2 -3 is -1 or (2 and -3)
        let e = Expr::Or(vec![Expr::neg(1), Expr::signed(&[2, -3])]);
        assert_eq!(to_dnf(&e).unwrap(), vec![vec![(1, false)], vec![(2, true), (3, false)]]);
    }

    #[test]
    fn blow_up_is_capped() {
        let parts: Vec<Expr> = (0..13).map(|i| Expr::Or(vec![Expr::neg(2 * i), Expr::neg(2 * i + 1)])).collect();
        assert!(to_dnf(&Expr::And(parts)).is_err());
    }
}
