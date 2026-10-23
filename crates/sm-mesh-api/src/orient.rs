//! The declared orientation rule and loop helpers.
//!
//! **Rule** (addendum §7.3): a face's vertex loop defines its normal by the right-hand rule.
//! `left` is the cell with the lower id; the normal points from `left` into `right`. A boundary
//! face has `right = None` and its normal points out of `left`. A cell therefore sees a face as
//! [`Sign::Plus`] when it is `left` (normal outward) and [`Sign::Minus`] when it is `right`.
//! Loops are stored starting at their smallest vertex index (canonical rotation).

use crate::index::{Sign, VertexIx};

/// Rotate a loop so it starts at its smallest vertex, keeping direction.
pub fn canonical_loop(l: &[VertexIx]) -> Vec<VertexIx> {
    let Some(start) = l.iter().enumerate().min_by_key(|(_, v)| **v).map(|(i, _)| i) else {
        return Vec::new();
    };
    l[start..].iter().chain(l[..start].iter()).copied().collect()
}

/// Reverse the direction of a loop and re-canonicalize (start at the smallest vertex).
pub fn reversed_loop(l: &[VertexIx]) -> Vec<VertexIx> {
    let r: Vec<VertexIx> = l.iter().rev().copied().collect();
    canonical_loop(&r)
}

/// Sorted vertex set of a loop: the identity key of a face for deduplication.
pub fn face_key(l: &[VertexIx]) -> Vec<u32> {
    let mut k: Vec<u32> = l.iter().map(|v| v.0).collect();
    k.sort_unstable();
    k
}

/// True if the loop has at least 3 vertices and no repeats.
pub fn loop_is_simple(l: &[VertexIx]) -> bool {
    if l.len() < 3 {
        return false;
    }
    let k = face_key(l);
    k.windows(2).all(|w| w[0] != w[1])
}

/// The sign a cell with id-rank `cell` has on a face whose `left` is `left`.
pub fn sign_of(cell: u32, left: u32) -> Sign {
    if cell == left { Sign::Plus } else { Sign::Minus }
}
