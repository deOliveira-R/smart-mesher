//! Small shared helpers: sentinels, hashing, chunking and deterministic error selection.

use sm_core::{EntityId, EntityKind};
use sm_mesh_api::MeshError;
use std::hash::{BuildHasherDefault, Hasher};
use std::ops::Range;
use std::sync::Mutex;

/// Items per parallel chunk for per-entity stages.
pub(crate) const CHUNK: usize = 4096;

/// "No classification yet" marker of builder columns. A `MeshCell` id can never be a valid
/// classification target (classifications name model or decomposition entities).
pub(crate) fn nil() -> EntityId {
    EntityId::new(EntityKind::MeshCell, 0).unwrap_or_else(|| unreachable!("serial 0 is in range"))
}

/// Range of item `i` in a CSR offset array.
#[inline]
pub(crate) fn range(off: &[u64], i: usize) -> Range<usize> {
    off[i] as usize..off[i + 1] as usize
}

/// splitmix64 finalizer: a strong 64-bit mixer.
#[inline]
pub(crate) fn mix64(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9e37_79b9_7f4a_7c15);
    x = (x ^ (x >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    x ^ (x >> 31)
}

/// Order-independent hash of a vertex set (commutative sum of mixed members).
#[inline]
pub(crate) fn set_hash(vs: impl Iterator<Item = u32>) -> u64 {
    let mut h = 0u64;
    let mut n = 0u64;
    for v in vs {
        h = h.wrapping_add(mix64(u64::from(v)));
        n += 1;
    }
    mix64(h ^ n.wrapping_mul(0xff51_afd7_ed55_8ccd))
}

/// Fx-style hasher for integer keys. Only used for lookups: iteration order of the maps that use
/// it never reaches an output.
#[derive(Default, Clone, Copy)]
pub(crate) struct FxHasher(u64);

impl Hasher for FxHasher {
    fn finish(&self) -> u64 {
        mix64(self.0)
    }
    fn write(&mut self, bytes: &[u8]) {
        for b in bytes {
            self.0 = (self.0.rotate_left(5) ^ u64::from(*b)).wrapping_mul(0x517c_c1b7_2722_0a95);
        }
    }
    fn write_u64(&mut self, i: u64) {
        self.0 = (self.0.rotate_left(5) ^ i).wrapping_mul(0x517c_c1b7_2722_0a95);
    }
    fn write_i64(&mut self, i: i64) {
        self.write_u64(i as u64);
    }
    fn write_u32(&mut self, i: u32) {
        self.write_u64(u64::from(i));
    }
}

/// `BuildHasher` for [`FxHasher`].
pub(crate) type FxBuild = BuildHasherDefault<FxHasher>;

/// Boundaries of chunks of at most `chunk` items covering `0..n` (always ends with `n`).
pub(crate) fn chunk_bounds(n: usize, chunk: usize) -> Vec<usize> {
    let mut b: Vec<usize> = (0..n).step_by(chunk.max(1)).collect();
    b.push(n);
    b
}

/// Split `buf` into the disjoint CSR slices of the item ranges `bounds[k]..bounds[k+1]`
/// (`off[bounds[0]]` must be 0). Safe parallel writing without `unsafe`.
pub(crate) fn split_by_offsets<'a, T>(
    buf: &'a mut [T],
    off: &[u64],
    bounds: &[usize],
) -> Vec<&'a mut [T]> {
    let mut rest = buf;
    let mut out = Vec::with_capacity(bounds.len().saturating_sub(1));
    for w in bounds.windows(2) {
        let len = (off[w[1]] - off[w[0]]) as usize;
        let (a, b) = rest.split_at_mut(len);
        out.push(a);
        rest = b;
    }
    out
}

/// Errors recorded from parallel workers; [`ErrSink::take_first`] picks the one with the lowest
/// item index, so the reported error does not depend on scheduling.
#[derive(Default)]
pub(crate) struct ErrSink(Mutex<Vec<(usize, MeshError)>>);

impl ErrSink {
    pub(crate) fn push(&self, at: usize, e: MeshError) {
        if let Ok(mut g) = self.0.lock() {
            g.push((at, e));
        }
    }
    pub(crate) fn take_first(self) -> Option<MeshError> {
        let v = self.0.into_inner().unwrap_or_default();
        v.into_iter().min_by_key(|(i, _)| *i).map(|(_, e)| e)
    }
}

/// Exclusive prefix sum of `lens` into a `u64` offset array of length `lens.len() + 1`.
pub(crate) fn prefix(lens: &[u32]) -> Vec<u64> {
    let mut off = Vec::with_capacity(lens.len() + 1);
    let mut acc = 0u64;
    off.push(0);
    for l in lens {
        acc += u64::from(*l);
        off.push(acc);
    }
    off
}
