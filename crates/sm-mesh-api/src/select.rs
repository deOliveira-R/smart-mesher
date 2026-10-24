//! Selector algebra over `&dyn MeshRead`: free functions, outside the traits.
//!
//! Selectors pick entities of one kind by model classification, set membership or tag value and
//! combine with `And` / `Or` / `Not`. Results are ascending dense indices.

use crate::index::{FaceIx, MeshEntityKind};
use crate::read::MeshRead;
use crate::sets::TagValues;
use sm_core::EntityId;

/// A predicate over entities of one kind.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum Selector {
    /// Every entity.
    All,
    /// Entities classified on this model/partition entity.
    OnModel(EntityId),
    /// Members of the named set (a set of another kind selects nothing).
    InSet(String),
    /// Entities whose integer tag `(kind, name)` equals the value.
    TagEq(String, i64),
    /// Entities whose real tag `(kind, name)` lies in `[lo, hi]`.
    TagRange(String, f64, f64),
    /// Boundary faces (faces only).
    Boundary,
    /// Complement.
    Not(Box<Selector>),
    /// Intersection.
    And(Vec<Selector>),
    /// Union.
    Or(Vec<Selector>),
}

/// True if entity `ix` of kind `k` matches `sel`.
pub fn matches(m: &dyn MeshRead, k: MeshEntityKind, ix: u32, sel: &Selector) -> bool {
    match sel {
        Selector::All => true,
        Selector::OnModel(e) => m.classified_on(*e).is_some_and(|s| match k {
            MeshEntityKind::Vertex => s.vertices.binary_search(&crate::VertexIx(ix)).is_ok(),
            MeshEntityKind::Edge => s.edges.binary_search(&crate::EdgeIx(ix)).is_ok(),
            MeshEntityKind::Face => s.faces.binary_search(&FaceIx(ix)).is_ok(),
            MeshEntityKind::Cell => s.cells.binary_search(&crate::CellIx(ix)).is_ok(),
        }),
        Selector::InSet(name) => m
            .set_by_name(name)
            .is_some_and(|s| s.members.kind() == k && s.members.contains(ix)),
        Selector::TagEq(name, v) => m
            .tag(k, name)
            .and_then(|t| t.values.as_i64().map(|a| a[ix as usize] == *v))
            .unwrap_or(false),
        Selector::TagRange(name, lo, hi) => m
            .tag(k, name)
            .and_then(|t| match &t.values {
                TagValues::F64(a) => Some((*lo..=*hi).contains(&a[ix as usize])),
                TagValues::I64(_) => None,
            })
            .unwrap_or(false),
        Selector::Boundary => k == MeshEntityKind::Face && m.is_boundary_face(FaceIx(ix)),
        Selector::Not(s) => !matches(m, k, ix, s),
        Selector::And(v) => v.iter().all(|s| matches(m, k, ix, s)),
        Selector::Or(v) => v.iter().any(|s| matches(m, k, ix, s)),
    }
}

/// Ascending indices of all entities of kind `k` matching `sel`.
pub fn select(m: &dyn MeshRead, k: MeshEntityKind, sel: &Selector) -> Vec<u32> {
    (0..m.count(k) as u32)
        .filter(|&i| matches(m, k, i, sel))
        .collect()
}

/// Union of two ascending index lists.
pub fn union(a: &[u32], b: &[u32]) -> Vec<u32> {
    let mut out = Vec::with_capacity(a.len() + b.len());
    let (mut i, mut j) = (0, 0);
    while i < a.len() || j < b.len() {
        match (a.get(i), b.get(j)) {
            (Some(x), Some(y)) if x == y => {
                out.push(*x);
                i += 1;
                j += 1;
            }
            (Some(x), Some(y)) if x < y => {
                out.push(*x);
                i += 1;
            }
            (Some(_), Some(y)) => {
                out.push(*y);
                j += 1;
            }
            (Some(x), None) => {
                out.push(*x);
                i += 1;
            }
            (None, Some(y)) => {
                out.push(*y);
                j += 1;
            }
            (None, None) => break,
        }
    }
    out
}

/// Intersection of two ascending index lists.
pub fn intersection(a: &[u32], b: &[u32]) -> Vec<u32> {
    a.iter()
        .copied()
        .filter(|x| b.binary_search(x).is_ok())
        .collect()
}

/// Elements of `a` not in `b` (both ascending).
pub fn difference(a: &[u32], b: &[u32]) -> Vec<u32> {
    a.iter()
        .copied()
        .filter(|x| b.binary_search(x).is_err())
        .collect()
}
