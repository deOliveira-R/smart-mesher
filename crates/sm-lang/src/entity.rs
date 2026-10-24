//! Entity sets: sorted, duplicate-free id vectors with data-parallel set algebra, and the resolver
//! hook for relation filters (`faces in Volume#2`).

use std::borrow::Cow;
use std::collections::BTreeSet;
use std::sync::Arc;

use rayon::prelude::*;
use sm_core::{EntityId, EntityKind};
use sm_op_api::State;

/// Inputs at least this long use the parallel algorithms.
pub const PAR_THRESHOLD: usize = 1 << 16;
/// Work chunk of the parallel algorithms (fixed, so results never depend on the thread count).
pub const PAR_CHUNK: usize = 1 << 14;

/// A set of entity ids behind an `Arc` (cloning is cheap).
///
/// Two flavours share the type: a **sorted set** (ids ordered by `EntityId`, no duplicates; what ranges, `all`, `in`
/// and every set operator produce, so output never depends on how a set was built) and an **ordered list** (an op's
/// output role, `created` or `removed`, in the order the op reported them, duplicates removed keeping the first).
/// Indexing, iteration and conversion to parameters follow the stored order; set algebra always returns sorted sets.
/// Equality is set equality.
#[derive(Clone, Debug, Default)]
pub struct EntitySet {
    ids: Arc<Vec<EntityId>>,
    sorted: bool,
}

impl PartialEq for EntitySet {
    fn eq(&self, other: &Self) -> bool {
        if self.sorted && other.sorted {
            return self.ids == other.ids;
        }
        self.sorted_slice().as_ref() == other.sorted_slice().as_ref()
    }
}

impl Eq for EntitySet {}

impl EntitySet {
    /// The empty set.
    pub fn new() -> Self {
        Self { ids: Arc::new(Vec::new()), sorted: true }
    }

    /// Adopt a vector that is already sorted and duplicate-free (debug-asserted).
    pub fn from_sorted_unique(v: Vec<EntityId>) -> Self {
        debug_assert!(v.windows(2).all(|w| w[0] < w[1]));
        Self { ids: Arc::new(v), sorted: true }
    }

    /// Sort and de-duplicate any vector.
    pub fn from_vec(mut v: Vec<EntityId>) -> Self {
        if v.len() >= PAR_THRESHOLD {
            v.par_sort_unstable();
        } else {
            v.sort_unstable();
        }
        v.dedup();
        Self { ids: Arc::new(v), sorted: true }
    }

    /// An ordered list: keeps the given order (a repeated id keeps its first position). A strictly increasing input is a sorted set.
    pub fn from_ordered(v: Vec<EntityId>) -> Self {
        if v.windows(2).all(|w| w[0] < w[1]) {
            return Self { ids: Arc::new(v), sorted: true };
        }
        let mut seen = BTreeSet::new();
        let out: Vec<EntityId> = v.into_iter().filter(|i| seen.insert(*i)).collect();
        let sorted = out.windows(2).all(|w| w[0] < w[1]);
        Self { ids: Arc::new(out), sorted }
    }

    /// A one-element set.
    pub fn single(id: EntityId) -> Self {
        Self { ids: Arc::new(vec![id]), sorted: true }
    }

    /// Number of ids.
    pub fn len(&self) -> usize {
        self.ids.len()
    }

    /// True when empty.
    pub fn is_empty(&self) -> bool {
        self.ids.is_empty()
    }

    /// The ids in stored order (sorted for a set, op order for an ordered list).
    pub fn as_slice(&self) -> &[EntityId] {
        &self.ids
    }

    /// True when the stored order is the sorted order.
    pub fn is_sorted(&self) -> bool {
        self.sorted
    }

    /// The ids sorted, borrowed when they already are.
    pub fn sorted_slice(&self) -> Cow<'_, [EntityId]> {
        if self.sorted {
            Cow::Borrowed(&self.ids[..])
        } else {
            let mut v = self.ids.to_vec();
            v.sort_unstable();
            Cow::Owned(v)
        }
    }

    /// This collection as a sorted set (cheap when it already is one).
    #[must_use]
    pub fn to_sorted(&self) -> Self {
        if self.sorted { self.clone() } else { Self { ids: Arc::new(self.sorted_slice().into_owned()), sorted: true } }
    }

    /// Iterate in stored order.
    pub fn iter(&self) -> std::slice::Iter<'_, EntityId> {
        self.ids.iter()
    }

    /// Membership (binary search for a sorted set).
    pub fn contains(&self, id: EntityId) -> bool {
        if self.sorted { self.ids.binary_search(&id).is_ok() } else { self.ids.contains(&id) }
    }

    /// Consume into the vector in stored order.
    pub fn into_vec(self) -> Vec<EntityId> {
        Arc::try_unwrap(self.ids).unwrap_or_else(|a| (*a).clone())
    }

    /// Union (a sorted set).
    pub fn union(&self, other: &Self) -> Self {
        self.union_chunked(other, self.chunk_for(other))
    }

    /// Intersection (a sorted set).
    pub fn intersection(&self, other: &Self) -> Self {
        self.intersection_chunked(other, self.chunk_for(other))
    }

    /// Difference (`self` without `other`, a sorted set).
    pub fn difference(&self, other: &Self) -> Self {
        self.difference_chunked(other, self.chunk_for(other))
    }

    fn chunk_for(&self, other: &Self) -> usize {
        if self.len() + other.len() >= PAR_THRESHOLD { PAR_CHUNK } else { usize::MAX }
    }

    /// Union with an explicit chunk size (`usize::MAX` forces the sequential path). Chunks split `self` and cut `other`
    /// at the same *values*, so the result is the same for every chunk size and thread count.
    pub fn union_chunked(&self, other: &Self, chunk: usize) -> Self {
        Self::from_sorted_unique(chunked(&self.sorted_slice(), &other.sorted_slice(), chunk, merge_union))
    }

    /// Intersection with an explicit chunk size (see [`union_chunked`](Self::union_chunked)).
    pub fn intersection_chunked(&self, other: &Self, chunk: usize) -> Self {
        Self::from_sorted_unique(chunked(&self.sorted_slice(), &other.sorted_slice(), chunk, merge_inter))
    }

    /// Difference with an explicit chunk size (see [`union_chunked`](Self::union_chunked)).
    pub fn difference_chunked(&self, other: &Self, chunk: usize) -> Self {
        Self::from_sorted_unique(chunked(&self.sorted_slice(), &other.sorted_slice(), chunk, merge_diff))
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
    EntitySet::from_sorted_unique(v)
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
