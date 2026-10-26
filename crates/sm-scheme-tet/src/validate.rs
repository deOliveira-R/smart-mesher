//! Input validation.

use crate::error::TetError;
use crate::geom::{V3, coord_ok};
use sm_math::PredicateError;

fn norm0(x: f64) -> f64 {
    if x == 0.0 { 0.0 } else { x }
}

/// Finite, in exact range, and no duplicate coordinates.
pub(crate) fn check_points(points: &[V3]) -> Result<(), TetError> {
    for p in points {
        if !p.iter().all(|&x| x.is_finite() && coord_ok(x)) {
            return Err(TetError::OutOfRange(PredicateError::OutOfRange));
        }
    }
    let idx = lex_order(points);
    for w in idx.windows(2) {
        let (p, q) = (points[w[0] as usize], points[w[1] as usize]);
        if (0..3).all(|k| norm0(p[k]) == norm0(q[k])) {
            return Err(TetError::DuplicatePoint { a: w[0], b: w[1] });
        }
    }
    Ok(())
}

/// Point indices in lexicographic coordinate order (ties by index).
pub(crate) fn lex_order(points: &[V3]) -> Vec<u32> {
    let mut idx: Vec<u32> = (0..points.len() as u32).collect();
    idx.sort_unstable_by(|&a, &b| {
        let (p, q) = (points[a as usize], points[b as usize]);
        norm0(p[0])
            .total_cmp(&norm0(q[0]))
            .then(norm0(p[1]).total_cmp(&norm0(q[1])))
            .then(norm0(p[2]).total_cmp(&norm0(q[2])))
            .then(a.cmp(&b))
    });
    idx
}
