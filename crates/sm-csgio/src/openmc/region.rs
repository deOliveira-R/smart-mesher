//! OpenMC region expressions: signed surface ids, `~` complement, juxtaposition (intersection),
//! `|` union and parentheses. Precedence: `~` over intersection over `|`.

use super::{Ctx, invalid};
use crate::ImportError;
use crate::ir::Expr;

/// Maximum nesting of parentheses (a hostile `((((...` must not overflow the stack).
const MAX_DEPTH: usize = 128;

#[derive(Clone, Copy, PartialEq, Debug)]
enum Tok {
    Open,
    Close,
    Or,
    Not,
    Surf(i64),
}

fn lex(cx: &Ctx, s: &str) -> Result<Vec<Tok>, ImportError> {
    let b = s.as_bytes();
    let (mut i, mut out) = (0, Vec::new());
    while i < b.len() {
        match b[i] {
            c if c.is_ascii_whitespace() => i += 1,
            b'(' => {
                out.push(Tok::Open);
                i += 1;
            }
            b')' => {
                out.push(Tok::Close);
                i += 1;
            }
            b'|' => {
                out.push(Tok::Or);
                i += 1;
            }
            b'~' => {
                out.push(Tok::Not);
                i += 1;
            }
            c if c == b'+' || c == b'-' || c.is_ascii_digit() => {
                let start = i;
                i += 1;
                while i < b.len() && b[i].is_ascii_digit() {
                    i += 1;
                }
                let t = &s[start..i];
                let v: i64 = t.trim_start_matches('+').parse().map_err(|_| invalid(cx, format!("region: bad surface reference {t:?}")))?;
                if v == 0 || v.unsigned_abs() > u64::from(u32::MAX) {
                    return Err(invalid(cx, format!("region: surface id {t:?} is out of range")));
                }
                out.push(Tok::Surf(v));
            }
            c => return Err(invalid(cx, format!("region: unexpected character {:?}", c as char))),
        }
    }
    Ok(out)
}

struct P<'a> {
    t: &'a [Tok],
    i: usize,
    cx: &'a Ctx,
}

impl P<'_> {
    fn peek(&self) -> Option<Tok> {
        self.t.get(self.i).copied()
    }
    fn union(&mut self, depth: usize) -> Result<Expr, ImportError> {
        let mut terms = vec![self.inter(depth)?];
        while self.peek() == Some(Tok::Or) {
            self.i += 1;
            terms.push(self.inter(depth)?);
        }
        Ok(if terms.len() == 1 { terms.remove(0) } else { Expr::Or(terms) })
    }
    fn inter(&mut self, depth: usize) -> Result<Expr, ImportError> {
        let mut terms = vec![self.unary(depth)?];
        while matches!(self.peek(), Some(Tok::Surf(_) | Tok::Open | Tok::Not)) {
            terms.push(self.unary(depth)?);
        }
        Ok(if terms.len() == 1 { terms.remove(0) } else { Expr::And(terms) })
    }
    fn unary(&mut self, depth: usize) -> Result<Expr, ImportError> {
        if depth > MAX_DEPTH {
            return Err(invalid(self.cx, format!("region: nesting deeper than {MAX_DEPTH}")));
        }
        match self.peek() {
            Some(Tok::Surf(v)) => {
                self.i += 1;
                Ok(Expr::signed(&[v]))
            }
            Some(Tok::Not) => {
                self.i += 1;
                Ok(Expr::Not(Box::new(self.unary(depth + 1)?)))
            }
            Some(Tok::Open) => {
                self.i += 1;
                let e = self.union(depth + 1)?;
                if self.peek() != Some(Tok::Close) {
                    return Err(invalid(self.cx, "region: missing ')'".into()));
                }
                self.i += 1;
                Ok(e)
            }
            Some(Tok::Close) => Err(invalid(self.cx, "region: unexpected ')'".into())),
            Some(Tok::Or) => Err(invalid(self.cx, "region: '|' needs an operand on its left".into())),
            None => Err(invalid(self.cx, "region: the expression ends where an operand is expected".into())),
        }
    }
}

/// Parse a region string. An empty string is all of space.
pub(super) fn parse(cx: &Ctx, s: &str) -> Result<Expr, ImportError> {
    let toks = lex(cx, s)?;
    if toks.is_empty() {
        return Ok(Expr::And(vec![]));
    }
    let mut p = P { t: &toks, i: 0, cx };
    let e = p.union(0)?;
    if p.i != toks.len() {
        return Err(invalid(cx, "region: unbalanced ')'".into()));
    }
    Ok(e)
}
