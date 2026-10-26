//! Wire layout of a track bundle returned by `smrt_trace_bundle`, and its Rust-side codec.
//!
//! The buffer is a slice of `u64` words (8-byte aligned, so a C caller can hold it in a
//! `uint64_t` array). All integer columns are packed two `u32` per word (element `2k` in the low
//! half), so the layout does not depend on the host byte order. Header (12 words):
//!
//! | word | content |
//! |---|---|
//! | 0 | magic, the bytes `SMRTBNDL` as a little-endian `u64` |
//! | 1 | layout version (1) |
//! | 2, 3 | `n_rays`, `n_segs` |
//! | 4..=9 | word index of the sections `offsets`, `cell`, `region`, `length`, `end`, `end_surface` |
//! | 10 | total words |
//! | 11 | reserved (0) |
//!
//! Sections: `offsets` (`n_rays+1` u32), `cell`, `region` (`n_segs` u32 each), `length`
//! (`n_segs` words, the `f64` bit patterns), `end` (`n_rays` u32, the `EndCode` discriminant),
//! `end_surface` (`n_rays` u32). Each section starts on a word boundary. The content is the
//! `TrackBundle` of the contract, so `decode(..).content_hash()` equals the hash of the Rust
//! bundle.

use sm_rt_api::{EndCode, TrackBundle};

/// Magic word.
pub const MAGIC: u64 = u64::from_le_bytes(*b"SMRTBNDL");
/// Layout version.
pub const VERSION: u64 = 1;
/// Header size in words.
pub const HEADER_WORDS: usize = 12;

fn packed(n: usize) -> usize {
    n.div_ceil(2)
}

/// Section word offsets and the total size for `n_rays` rays and `n_segs` segments.
fn layout(n_rays: usize, n_segs: usize) -> Option<[usize; 7]> {
    let sizes = [packed(n_rays.checked_add(1)?), packed(n_segs), packed(n_segs), n_segs, packed(n_rays), packed(n_rays)];
    let mut at = [0usize; 7];
    let mut cur = HEADER_WORDS;
    for (i, s) in sizes.iter().enumerate() {
        at[i] = cur;
        cur = cur.checked_add(*s)?;
    }
    at[6] = cur;
    Some(at)
}

/// Words needed to encode `b`.
pub fn encoded_words(b: &TrackBundle) -> usize {
    layout(b.ray_count(), b.cell.len()).map_or(usize::MAX, |l| l[6])
}

fn put32(words: &mut [u64], at: usize, i: usize, v: u32) {
    let w = &mut words[at + i / 2];
    let shift = 32 * (i % 2);
    *w = (*w & !(0xFFFF_FFFFu64 << shift)) | (u64::from(v) << shift);
}

fn get32(words: &[u64], at: usize, i: usize) -> u32 {
    (words[at + i / 2] >> (32 * (i % 2))) as u32
}

/// Encode `b` into `out` (which must hold at least [`encoded_words`] words; it is zeroed over
/// the encoded range first). Returns the words written, or `None` if `out` is too small.
pub fn encode_into(b: &TrackBundle, out: &mut [u64]) -> Option<usize> {
    let (n_rays, n_segs) = (b.ray_count(), b.cell.len());
    let l = layout(n_rays, n_segs)?;
    let out = out.get_mut(..l[6])?;
    out.fill(0);
    let head = [MAGIC, VERSION, n_rays as u64, n_segs as u64, l[0] as u64, l[1] as u64, l[2] as u64, l[3] as u64, l[4] as u64, l[5] as u64, l[6] as u64, 0];
    out[..HEADER_WORDS].copy_from_slice(&head);
    for (i, &o) in b.offsets.iter().enumerate() {
        put32(out, l[0], i, o);
    }
    for i in 0..n_segs {
        put32(out, l[1], i, b.cell[i]);
        put32(out, l[2], i, b.region[i]);
        out[l[3] + i] = b.length[i].to_bits();
    }
    for i in 0..n_rays {
        put32(out, l[4], i, b.end[i] as u32);
        put32(out, l[5], i, b.end_surface[i]);
    }
    Some(l[6])
}

/// Encode into a fresh vector.
pub fn encode(b: &TrackBundle) -> Vec<u64> {
    let mut v = vec![0u64; encoded_words(b)];
    let _ = encode_into(b, &mut v);
    v
}

/// Decode a buffer produced by [`encode_into`]. Every header field is validated; a malformed
/// buffer is an `Err`, never a panic.
pub fn decode(words: &[u64]) -> Result<TrackBundle, String> {
    if words.len() < HEADER_WORDS || words[0] != MAGIC || words[1] != VERSION {
        return Err("not a smrt bundle (bad magic or version)".into());
    }
    let n_rays = usize::try_from(words[2]).map_err(|_| "ray count overflow")?;
    let n_segs = usize::try_from(words[3]).map_err(|_| "segment count overflow")?;
    let l = layout(n_rays, n_segs).ok_or("size overflow")?;
    let stored: Vec<u64> = words[4..=10].to_vec();
    if stored != l.iter().map(|&x| x as u64).collect::<Vec<_>>() || words.len() < l[6] {
        return Err("bundle header does not match its sizes".into());
    }
    let mut b = TrackBundle::new();
    b.offsets = (0..=n_rays).map(|i| get32(words, l[0], i)).collect();
    b.cell = (0..n_segs).map(|i| get32(words, l[1], i)).collect();
    b.region = (0..n_segs).map(|i| get32(words, l[2], i)).collect();
    b.length = (0..n_segs).map(|i| f64::from_bits(words[l[3] + i])).collect();
    b.end = (0..n_rays)
        .map(|i| EndCode::from_u8(get32(words, l[4], i) as u8).ok_or("unknown end code"))
        .collect::<Result<_, _>>()?;
    b.end_surface = (0..n_rays).map(|i| get32(words, l[5], i)).collect();
    Ok(b)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_rt_api::{CellId, RegionId, Segment, TraceEnd};

    fn sample() -> TrackBundle {
        let mut b = TrackBundle::new();
        let s = |c: u32, t0: f64, l: f64| Segment::new(CellId(c), RegionId(c % 2), t0, l);
        b.push_ray(&[s(3, 0.0, 1.5), s(4, 1.5, 0.25)], &TraceEnd::LeftModel).unwrap();
        b.push_ray(&[], &TraceEnd::Limit).unwrap();
        b.push_ray(&[s(9, 0.0, -0.0)], &TraceEnd::Limit).unwrap();
        b
    }

    #[test]
    fn round_trip_preserves_hash_and_bits() {
        let b = sample();
        let w = encode(&b);
        assert_eq!(w.len(), encoded_words(&b));
        let d = decode(&w).unwrap();
        assert_eq!(d, b);
        assert_eq!(d.content_hash(), b.content_hash());
        assert!(d.length[2].is_sign_negative());
    }

    #[test]
    fn short_buffer_and_garbage_are_rejected() {
        let b = sample();
        let mut small = vec![0u64; encoded_words(&b) - 1];
        assert!(encode_into(&b, &mut small).is_none());
        assert!(decode(&[]).is_err());
        let mut w = encode(&b);
        w[3] += 1;
        assert!(decode(&w).is_err());
        let mut w = encode(&b);
        w[10] = u64::MAX;
        assert!(decode(&w).is_err());
        let w = encode(&b);
        assert!(decode(&w[..w.len() - 1]).is_err());
    }

    #[test]
    fn empty_bundle() {
        let b = TrackBundle::new();
        let w = encode(&b);
        assert_eq!(decode(&w).unwrap(), b);
    }
}
