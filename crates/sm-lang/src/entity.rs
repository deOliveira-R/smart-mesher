//! Entity sets: sorted, duplicate-free id vectors with data-parallel set algebra, and the resolver
//! hook for relation filters (`faces in Volume#2`).

use rayon::prelude::*;
use sm_core::{EntityId, EntityKind};
use sm_op_api::State;

/// Inputs at least this long use the parallel algorithms.
pub const PAR_THRESHOLD: usize = 1 << 16;
/// Work chunk of the parallel algorithms (fixed, so results never depend on the thread count).
pub const PAR_CHUNK: usize = 1 << 14;

/// A set of entity ids kept sorted by `EntityId` order (kind, then serial) without duplicates.
/// Every operation preserves that invariant, so output never depends on how a set was built.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct EntitySet(Vec<EntityId>);

impl EntitySet {
    /// The empty set.
    pub fn new() -> Self {
        Self(Vec::new())
    }

    /// Adopt a vector that is already sorted and duplicate-free (debug-asserted).
    pub fn from_sorted_unique(v: Vec<EntityId>) -> Self {
        debug_assert!(v.windows(2).all(|w| w[0] < w[1]));
        Self(v)
    }

    /// Sort and de-duplicate any vector.
    pub fn from_vec(mut v: Vec<EntityId>) -> Self {
        if v.len() >= PAR_THRESHOLD {
            v.par_sort_unstable();
        } else {
            v.sort_unstable();
        }
        v.dedup();
        Self(v)
    }

    /// A one-element set.
    pub fn single(id: EntityId) -> Self {
        Self(vec![id])
    }

    /// Number of ids.
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// True when empty.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// The ids in order.
    pub fn as_slice(&self) -> &[EntityId] {
        &self.0
    }

    /// Iterate in order.
    pub fn iter(&self) -> std::slice::Iter<'_, EntityId> {
        self.0.iter()
    }

    /// Membership by binary search.
    pub fn contains(&self, id: EntityId) -> bool {
        self.0.binary_search(&id).is_ok()
    }

    /// Consume into the sorted vector.
    pub fn into_vec(self) -> Vec<EntityId> {
        self.0
    }

    /// Union.
    pub fn union(&self, other: &Self) -> Self {
        self.union_chunked(other, self.chunk_for(other))
    }

    /// Intersection.
    pub fn intersection(&self, other: &Self) -> Self {
        self.intersection_chunked(other, self.chunk_for(other))
    }

    /// Difference (`self` without `other`).
    pub fn difference(&self, other: &Self) -> Self {
        self.difference_chunked(other, self.chunk_for(other))
    }

    fn chunk_for(&self, other: &Self) -> usize {
        if self.0.len() + other.0.len() >= PAR_THRESHOLD { PAR_CHUNK } else { usize::MAX }
    }

    /// Union with an explicit chunk size (`usize::MAX` forces the sequential path). Chunks split `self` and cut `other`
    /// at the same *values*, so the result is the same for every chunk size and thread count.
    pub fn union_chunked(&self, other: &Self, chunk: usize) -> Self {
        Self(chunked(&self.0, &other.0, chunk, merge_union))
    }

    /// Intersection with an explicit chunk size (see [`union_chunked`](Self::union_chunked)).
    pub fn intersection_chunked(&self, other: &Self, chunk: usize) -> Self {
        Self(chunked(&self.0, &other.0, chunk, merge_inter))
    }

    /// Difference with an explicit chunk size (see [`union_chunked`](Self::union_chunked)).
    pub fn difference_chunked(&self, other: &Self, chunk: usize) -> Self {
        Self(chunked(&self.0, &other.0, chunk, merge_diff))
    }
}

type MergeFn = fn(&[EntityId], &[EntityId], &mut Vec<EntityId>);

fn merge_union(a: &[EntityId], b: &[EntityId], out: &mut Vec<EntityId>) {
    let (mut i, mut j) = (0, 0);
    while i < a.len() && j < b.len() {
        match a[i].cmp(&b[j]) {
            std::cmp::Ordering::Less => {
                out.push(a[i]);
                i += 1;
            }
            std::cmp::Ordering::Greater => {
                out.push(b[j]);
                j += 1;
            }
            std::cmp::Ordering::Equal => {
                out.push(a[i]);
                i += 1;
                j += 1;
            }
        }
    }
    out.extend_from_slice(&a[i..]);
    out.extend_from_slice(&b[j..]);
}

fn merge_inter(a: &[EntityId], b: &[EntityId], out: &mut Vec<EntityId>) {
    let (mut i, mut j) = (0, 0);
    while i < a.len() && j < b.len() {
        match a[i].cmp(&b[j]) {
            std::cmp::Ordering::Less => i += 1,
            std::cmp::Ordering::Greater => j += 1,
            std::cmp::Ordering::Equal => {
                out.push(a[i]);
                i += 1;
                j += 1;
            }
        }
    }
}

fn merge_diff(a: &[EntityId], b: &[EntityId], out: &mut Vec<EntityId>) {
    let (mut i, mut j) = (0, 0);
    while i < a.len() {
        if j >= b.len() {
            out.extend_from_slice(&a[i..]);
            return;
        }
        match a[i].cmp(&b[j]) {
            std::cmp::Ordering::Less => {
                out.push(a[i]);
                i += 1;
            }
            std::cmp::Ordering::Greater => j += 1,
            std::cmp::Ordering::Equal => {
                i += 1;
                j += 1;
            }
        }
    }
}

/// Run `f` over aligned chunks of `a` and the matching value-range of `b`, concatenating in chunk order.
fn chunked(a: &[EntityId], b: &[EntityId], chunk: usize, f: MergeFn) -> Vec<EntityId> {
    if chunk == usize::MAX || a.len() <= chunk {
        let mut out = Vec::with_capacity(a.len().max(b.len()));
        f(a, b, &mut out);
        return out;
    }
    let chunk = chunk.max(1);
    let n = a.len().div_ceil(chunk);
    // b-boundaries: chunk i takes b elements in [a[i*chunk], a[(i+1)*chunk]); the first chunk starts at 0, the last ends at b.len().
    let cut = |i: usize| -> usize {
        if i == 0 {
            0
        } else if i >= n {
            b.len()
        } else {
            let pivot = a[i * chunk];
            b.partition_point(|x| *x < pivot)
        }
    };
    let parts: Vec<Vec<EntityId>> = (0..n)
        .into_par_iter()
        .map(|i| {
            let a_lo = i * chunk;
            let a_hi = ((i + 1) * chunk).min(a.len());
            let mut out = Vec::new();
            f(&a[a_lo..a_hi], &b[cut(i)..cut(i + 1)], &mut out);
            out
        })
        .collect();
    let total = parts.iter().map(Vec::len).sum();
    let mut out = Vec::with_capacity(total);
    for p in parts {
        out.extend(p);
    }
    out
}

/// Live members of `kind` with serial in `lo..=hi` and `(serial - lo) % step == 0`.
/// The range is clipped to the serials ever allocated, so `Volume#1..9999999999` costs O(allocated), not O(10^10).
pub fn live_range(state: &State, kind: EntityKind, lo: u64, hi: u64, step: u64) -> EntitySet {
    let next = state.peek_next(kind).map_or(1, EntityId::serial);
    let lo_c = lo.max(1);
    let hi_c = hi.min(next.saturating_sub(1));
    if lo_c > hi_c {
        return EntitySet::new();
    }
    let step = step.max(1);
    let live = |s: u64| -> Option<EntityId> {
        if !(s - lo).is_multiple_of(step) {
            return None;
        }
        EntityId::new(kind, s).filter(|id| state.contains(*id))
    };
    let span = hi_c - lo_c + 1;
    let v: Vec<EntityId> = if span as usize >= PAR_THRESHOLD {
        (lo_c..=hi_c).into_par_iter().filter_map(live).collect()
    } else {
        (lo_c..=hi_c).filter_map(live).collect()
    };
    EntitySet(v)
}

/// Every live entity of `kind`.
pub fn all_of(state: &State, kind: EntityKind) -> EntitySet {
    live_range(state, kind, 1, EntityId::MAX_SERIAL, 1)
}

/// Adjacency provider for `kind in X` filters. Implemented by the topology/mesh layers' host adapter.
///
/// `related` returns the entities of `target` that are *contained in or incident to* any entity of
/// `within` (in either hierarchy direction, as the owning layer defines it). The language sorts,
/// de-duplicates and filters the result to live entities, so implementations need not.
pub trait Relations: Send + Sync {
    /// Entities of kind `target` related to `within`.
    fn related(&self, state: &State, target: EntityKind, within: &[EntityId]) -> Result<Vec<EntityId>, String>;
}
