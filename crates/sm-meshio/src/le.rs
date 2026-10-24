//! Little-endian packing helpers. All output goes through these so the files are identical on
//! every host, and large columns are filled in parallel over fixed-size chunks (the result does
//! not depend on the thread count).

use rayon::prelude::*;
use std::borrow::Cow;

/// A fixed-size little-endian scalar.
pub(crate) trait Le: Copy + Send + Sync {
    const SIZE: usize;
    fn put(self, out: &mut [u8]);
    fn get(b: &[u8]) -> Self;
}

macro_rules! le_impl {
    ($($t:ty),*) => {$(
        impl Le for $t {
            const SIZE: usize = core::mem::size_of::<$t>();
            #[inline]
            fn put(self, out: &mut [u8]) { out.copy_from_slice(&self.to_le_bytes()); }
            #[inline]
            fn get(b: &[u8]) -> Self {
                let mut a = [0u8; core::mem::size_of::<$t>()];
                a.copy_from_slice(b);
                <$t>::from_le_bytes(a)
            }
        }
    )*};
}
le_impl!(u8, i8, u16, u32, i32, u64, i64, f64);

/// Elements per parallel task.
const CHUNK: usize = 1 << 14;

/// `n` elements `f(0..n)` as little-endian bytes.
pub(crate) fn pack<T: Le>(n: usize, f: impl Fn(usize) -> T + Sync) -> Vec<u8> {
    let mut out = vec![0u8; n * T::SIZE];
    out.par_chunks_mut(CHUNK * T::SIZE).enumerate().for_each(|(ci, chunk)| {
        let base = ci * CHUNK;
        for (k, dst) in chunk.chunks_exact_mut(T::SIZE).enumerate() {
            f(base + k).put(dst);
        }
    });
    out
}

/// A slice of scalars as little-endian bytes (copy).
pub(crate) fn pack_slice<T: Le>(s: &[T]) -> Vec<u8> {
    pack(s.len(), |i| s[i])
}

/// A slice of `f64` as little-endian bytes, borrowed on little-endian hosts.
pub(crate) fn f64_bytes(s: &[f64]) -> Cow<'_, [u8]> {
    if cfg!(target_endian = "little") { Cow::Borrowed(bytemuck::cast_slice(s)) } else { Cow::Owned(pack_slice(s)) }
}

/// Cells per block of a CSR fill.
const BLOCK: usize = 4096;

/// Offsets and flat data of `n` variable-length rows (`get(i)` is row `i`), values mapped by
/// `map`. Offsets are exclusive prefix sums computed in row order; blocks of rows fill disjoint
/// pre-sized windows of the output, so there is no per-row allocation.
pub(crate) fn csr<'a, S: Copy + Send + Sync + 'a, T: Le>(
    n: usize,
    get: impl Fn(usize) -> &'a [S] + Sync,
    map: impl Fn(S) -> T + Sync,
) -> (Vec<u64>, Vec<u8>) {
    let lens: Vec<u64> = (0..n).into_par_iter().map(|i| get(i).len() as u64).collect();
    let mut off = Vec::with_capacity(n + 1);
    off.push(0u64);
    let mut acc = 0u64;
    for l in lens {
        acc += l;
        off.push(acc);
    }
    let mut data = vec![0u8; acc as usize * T::SIZE];
    let mut parts: Vec<(usize, &mut [u8])> = Vec::with_capacity(n / BLOCK + 1);
    let mut rest = data.as_mut_slice();
    let mut c = 0;
    while c < n {
        let e = (c + BLOCK).min(n);
        let nb = (off[e] - off[c]) as usize * T::SIZE;
        let (head, tail) = rest.split_at_mut(nb);
        parts.push((c, head));
        rest = tail;
        c = e;
    }
    parts.into_par_iter().for_each(|(c0, buf)| {
        let mut w = 0;
        for i in c0..(c0 + BLOCK).min(n) {
            for &s in get(i) {
                map(s).put(&mut buf[w..w + T::SIZE]);
                w += T::SIZE;
            }
        }
    });
    (off, data)
}

/// Decode a little-endian array: borrowed when the host is little-endian and the bytes are
/// suitably aligned, a copy otherwise.
pub(crate) fn decode<T: Le + bytemuck::Pod>(b: &[u8]) -> Cow<'_, [T]> {
    if cfg!(target_endian = "little")
        && let Ok(s) = bytemuck::try_cast_slice::<u8, T>(b)
    {
        return Cow::Borrowed(s);
    }
    Cow::Owned(b.chunks_exact(T::SIZE).map(T::get).collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack_and_csr() {
        let b = pack(5, |i| i as u32 * 3);
        assert_eq!(decode::<u32>(&b).as_ref(), &[0, 3, 6, 9, 12]);
        let rows: Vec<Vec<u32>> = (0..10_000usize).map(|i| (0..(i % 5) as u32).map(|k| k + i as u32).collect()).collect();
        let (off, data) = csr(rows.len(), |i| rows[i].as_slice(), |v| v);
        let flat: Vec<u32> = rows.concat();
        assert_eq!(decode::<u32>(&data).as_ref(), flat.as_slice());
        assert_eq!(off.len(), rows.len() + 1);
        assert_eq!(*off.last().unwrap() as usize, flat.len());
    }

    #[test]
    fn unaligned_decode_copies() {
        let mut v = [0u8; 17];
        v[1..9].copy_from_slice(&1.5f64.to_le_bytes());
        assert_eq!(decode::<f64>(&v[1..9]).as_ref(), &[1.5]);
    }
}
