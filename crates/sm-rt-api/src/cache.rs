//! Content hashing and the serialized form of a tracker: the **`.smrt` sidecar format**.
//!
//! A tracker is a *rebuildable cache*: it is never authoritative. The authoritative data are the
//! mesh (or B-rep), the boundary conditions, the hierarchy and the build options. A cache image
//! records the hashes of exactly those plus the builder version; a reader that finds a different
//! key discards the image and rebuilds. Caching is optional: a tracker rebuilt from inputs is
//! bit-identical to a loaded one (contract), so a cache only saves time (BVH, convexity flags
//! and face planes at 10^7 cells).
//!
//! # File layout (`FORMAT_VERSION` 2, all integers little-endian)
//! ```text
//! 0   magic "SMRTCACH"                8 bytes
//! 8   format version                  u32
//! 12  tracker kind                    u32   (TrackerKind code)
//! 16  builder version                 u32
//! 20  section count n                 u32
//! 24  topology_hash                   u128
//! 40  hierarchy_hash                  u128
//! 56  boundary_hash                   u128
//! 72  options_hash                    u128
//! 88  zero padding up to 128
//! 128 n section entries, 32 bytes each: code u32, 0 u32, offset u64, length u64, hash64 u64
//!     zero padding up to the next multiple of 64
//!     section payloads, each starting at a multiple of 64, zero-padded to a multiple of 64
//! ```
//! Every section offset is 64-byte aligned, so a memory-mapped file can be cast to `&[f64]`,
//! `&[u32]` or SIMD-aligned arrays without copying. Each payload carries a 64-bit hash that the
//! decoder verifies. Encoding is canonical (sections sorted by tag code), so equal images give
//! equal bytes.
//!
//! # Where it lives
//! In W3a the image is a **`.smrt` sidecar** next to the mesh (`<name>.smrt`). Embedding the same
//! bytes as an `rt/*` chunk of the `.smsh` container is W3b work; sm-meshio would store the
//! opaque [`CacheImage::encode`] bytes and never interpret them.

use crate::error::{Result, RtError};
use core::fmt;

/// 128-bit content hash. Non-cryptographic; stable across platforms and releases of the
/// format (bumping the algorithm bumps [`FORMAT_VERSION`]).
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct ContentHash(pub u128);

impl fmt::Debug for ContentHash {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:032x}", self.0)
    }
}
impl fmt::Display for ContentHash {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:032x}", self.0)
    }
}

/// Streaming hasher behind [`ContentHash`] (two independent 64-bit FNV-style lanes with a final
/// avalanche). Floats hash by bit pattern with `-0.0` folded to `+0.0` and all NaNs folded to
/// one pattern, so equal geometry hashes equal.
#[derive(Clone, Debug)]
pub struct ContentHasher {
    a: u64,
    b: u64,
}

impl Default for ContentHasher {
    fn default() -> Self {
        Self::new()
    }
}

fn avalanche(mut x: u64) -> u64 {
    x ^= x >> 30;
    x = x.wrapping_mul(0xbf58_476d_1ce4_e5b9);
    x ^= x >> 27;
    x = x.wrapping_mul(0x94d0_49bb_1331_11eb);
    x ^ (x >> 31)
}

impl ContentHasher {
    /// A fresh hasher.
    pub const fn new() -> Self {
        Self { a: 0xcbf2_9ce4_8422_2325, b: 0x9e37_79b9_7f4a_7c15 }
    }
    /// Absorb bytes.
    pub fn write_bytes(&mut self, bytes: &[u8]) {
        for &x in bytes {
            self.a = (self.a ^ u64::from(x)).wrapping_mul(0x0000_0100_0000_01b3);
            self.b = (self.b.rotate_left(5) ^ u64::from(x)).wrapping_mul(0xff51_afd7_ed55_8ccd);
        }
    }
    /// Absorb a `u64`.
    pub fn write_u64(&mut self, v: u64) {
        self.write_bytes(&v.to_le_bytes());
    }
    /// Absorb an `f64` by canonical bit pattern.
    pub fn write_f64(&mut self, v: f64) {
        let bits = if v.is_nan() {
            0x7ff8_0000_0000_0000
        } else if v == 0.0 {
            0
        } else {
            v.to_bits()
        };
        self.write_u64(bits);
    }
    /// Absorb another hash.
    pub fn write_hash(&mut self, h: ContentHash) {
        self.write_bytes(&h.0.to_le_bytes());
    }
    /// Finish.
    pub fn finish(&self) -> ContentHash {
        let hi = avalanche(self.a ^ self.b.rotate_left(32));
        let lo = avalanche(self.b ^ hi);
        ContentHash((u128::from(hi) << 64) | u128::from(lo))
    }
}

/// 64-bit hash of a byte slice (section integrity check).
pub fn hash64(bytes: &[u8]) -> u64 {
    let mut h = ContentHasher::new();
    h.write_bytes(bytes);
    (h.finish().0 >> 64) as u64
}

/// Cache image format version. Bump on any layout change; older images are then stale.
pub const FORMAT_VERSION: u32 = 2;
/// File magic.
pub const MAGIC: [u8; 8] = *b"SMRTCACH";

/// Which tracker built the image. Frozen codes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
#[non_exhaustive]
pub enum TrackerKind {
    /// Polyhedral mesh walker (+ boundary BVH for point location and the complement).
    MeshWalk = 1,
    /// Facet-only BVH tracker.
    Facets = 2,
    /// Hierarchical world of the above.
    World = 3,
    /// Analytic/reference (not normally cached).
    Analytic = 4,
}

impl TrackerKind {
    /// Decode.
    pub fn from_u32(v: u32) -> Option<Self> {
        Some(match v {
            1 => Self::MeshWalk,
            2 => Self::Facets,
            3 => Self::World,
            4 => Self::Analytic,
            _ => return None,
        })
    }
}

/// Section tags. Frozen codes; unknown tags below `0x1000` make a reader reject the image,
/// tags `0x1000..=0x1FFFF` are implementor-private (`Private(c)` is code `0x1000 + c`, with `c`
/// a `u16` so the sum cannot overflow); codes above that range are rejected as corrupt.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[non_exhaustive]
pub enum SectionTag {
    /// Face planes (unit normal + offset, f64 x4 per face).
    FacePlanes,
    /// Per-cell convexity flags and bounding boxes.
    CellInfo,
    /// BVH nodes over boundary faces / facets (point location, complement).
    Bvh,
    /// BVH over cells (point location without a hint).
    CellBvh,
    /// Region and surface tables.
    Tables,
    /// Boundary conditions and periodic face matching.
    Boundaries,
    /// Hierarchy: components, placements, lattices.
    Hierarchy,
    /// Implementor-private section; wire code `0x1000 + c`.
    Private(u16),
}

impl SectionTag {
    /// Wire code.
    pub fn code(self) -> u32 {
        match self {
            Self::FacePlanes => 1,
            Self::CellInfo => 2,
            Self::Bvh => 3,
            Self::CellBvh => 4,
            Self::Tables => 5,
            Self::Boundaries => 6,
            Self::Hierarchy => 7,
            Self::Private(c) => 0x1000 + c as u32,
        }
    }
    /// Decode a wire code (`None` for unknown public codes).
    pub fn from_code(c: u32) -> Option<Self> {
        Some(match c {
            1 => Self::FacePlanes,
            2 => Self::CellInfo,
            3 => Self::Bvh,
            4 => Self::CellBvh,
            5 => Self::Tables,
            6 => Self::Boundaries,
            7 => Self::Hierarchy,
            c if (0x1000..=0x1000 + u16::MAX as u32).contains(&c) => Self::Private((c - 0x1000) as u16),
            _ => return None,
        })
    }
}

/// What a cache image must match to be reused. Decomposed so an edit invalidates only what it
/// should: a boundary-condition edit changes `boundary_hash` and nothing about the geometry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CacheKey {
    /// Tracker kind.
    pub kind: TrackerKind,
    /// Builder (algorithm) version; bump when the built structure changes.
    pub builder_version: u32,
    /// Hash of the geometry: for a mesh, `sm_mesh_api::geometry_hash` + `connectivity_hash` +
    /// classification; for facets, [`crate::FacetInput::content_hash`]; for a B-rep, the facets
    /// plus the model revision hash.
    pub topology_hash: ContentHash,
    /// Hash of the instancing: [`crate::WorldSpec::content_hash`] (zero for a flat input).
    pub hierarchy_hash: ContentHash,
    /// Hash of the boundary conditions: [`crate::BoundarySpec::content_hash`].
    pub boundary_hash: ContentHash,
    /// Hash of the build options: [`crate::BuildOptions::content_hash`].
    pub options_hash: ContentHash,
}

/// A serialized tracker: header + sections.
#[derive(Clone, Debug, PartialEq)]
pub struct CacheImage {
    /// The key.
    pub key: CacheKey,
    /// Sections (encoded in ascending tag-code order).
    pub sections: Vec<(SectionTag, Vec<u8>)>,
}

/// Alignment of every section payload.
pub const SECTION_ALIGN: usize = 64;
const HEADER_LEN: usize = 128;
const ENTRY_LEN: usize = 32;

fn align_up(n: usize) -> usize {
    n.div_ceil(SECTION_ALIGN) * SECTION_ALIGN
}

fn rd_u32(b: &[u8], at: usize) -> Result<u32> {
    b.get(at..at + 4)
        .and_then(|s| s.try_into().ok())
        .map(u32::from_le_bytes)
        .ok_or_else(|| RtError::CacheCorrupt("truncated header".into()))
}
fn rd_u64(b: &[u8], at: usize) -> Result<u64> {
    b.get(at..at + 8)
        .and_then(|s| s.try_into().ok())
        .map(u64::from_le_bytes)
        .ok_or_else(|| RtError::CacheCorrupt("truncated header".into()))
}
fn rd_u128(b: &[u8], at: usize) -> Result<u128> {
    b.get(at..at + 16)
        .and_then(|s| s.try_into().ok())
        .map(u128::from_le_bytes)
        .ok_or_else(|| RtError::CacheCorrupt("truncated header".into()))
}

impl CacheImage {
    /// Encode to bytes (layout in the module docs). Sections are written sorted by tag code.
    pub fn encode(&self) -> Vec<u8> {
        let mut secs: Vec<&(SectionTag, Vec<u8>)> = self.sections.iter().collect();
        secs.sort_by_key(|s| s.0.code());
        let mut out = Vec::new();
        out.extend_from_slice(&MAGIC);
        out.extend_from_slice(&FORMAT_VERSION.to_le_bytes());
        out.extend_from_slice(&(self.key.kind as u32).to_le_bytes());
        out.extend_from_slice(&self.key.builder_version.to_le_bytes());
        out.extend_from_slice(&(secs.len() as u32).to_le_bytes());
        for h in [self.key.topology_hash, self.key.hierarchy_hash, self.key.boundary_hash, self.key.options_hash] {
            out.extend_from_slice(&h.0.to_le_bytes());
        }
        out.resize(HEADER_LEN, 0);
        let mut offset = align_up(HEADER_LEN + ENTRY_LEN * secs.len());
        for (tag, data) in &secs {
            out.extend_from_slice(&tag.code().to_le_bytes());
            out.extend_from_slice(&0u32.to_le_bytes());
            out.extend_from_slice(&(offset as u64).to_le_bytes());
            out.extend_from_slice(&(data.len() as u64).to_le_bytes());
            out.extend_from_slice(&hash64(data).to_le_bytes());
            offset += align_up(data.len());
        }
        out.resize(align_up(out.len()), 0);
        for (_, data) in &secs {
            out.extend_from_slice(data);
            out.resize(align_up(out.len()), 0);
        }
        out
    }

    /// Decode and verify magic, version, alignment and every section hash.
    pub fn decode(b: &[u8]) -> Result<Self> {
        if b.get(..8) != Some(&MAGIC[..]) {
            return Err(RtError::CacheCorrupt("bad magic".into()));
        }
        if rd_u32(b, 8)? != FORMAT_VERSION {
            return Err(RtError::CacheStale("format version"));
        }
        let kind = TrackerKind::from_u32(rd_u32(b, 12)?).ok_or_else(|| RtError::CacheCorrupt("unknown tracker kind".into()))?;
        let builder_version = rd_u32(b, 16)?;
        let n = rd_u32(b, 20)? as usize;
        let topology_hash = ContentHash(rd_u128(b, 24)?);
        let hierarchy_hash = ContentHash(rd_u128(b, 40)?);
        let boundary_hash = ContentHash(rd_u128(b, 56)?);
        let options_hash = ContentHash(rd_u128(b, 72)?);
        if n > (b.len() / ENTRY_LEN) {
            return Err(RtError::CacheCorrupt("section count exceeds file".into()));
        }
        let mut sections = Vec::with_capacity(n);
        for i in 0..n {
            let at = HEADER_LEN + i * ENTRY_LEN;
            let code = rd_u32(b, at)?;
            let off = rd_u64(b, at + 8)? as usize;
            let len = rd_u64(b, at + 16)? as usize;
            let h = rd_u64(b, at + 24)?;
            if !off.is_multiple_of(SECTION_ALIGN) {
                return Err(RtError::CacheCorrupt(format!("section {code} is not {SECTION_ALIGN}-byte aligned")));
            }
            let data = off
                .checked_add(len)
                .and_then(|end| b.get(off..end))
                .ok_or_else(|| RtError::CacheCorrupt("section out of range".into()))?;
            if hash64(data) != h {
                return Err(RtError::CacheCorrupt(format!("section {code} hash mismatch")));
            }
            let Some(tag) = SectionTag::from_code(code) else {
                return Err(RtError::CacheCorrupt(format!("unknown section {code}")));
            };
            sections.push((tag, data.to_vec()));
        }
        Ok(Self { key: CacheKey { kind, builder_version, topology_hash, hierarchy_hash, boundary_hash, options_hash }, sections })
    }

    /// Check the image against the key a fresh build would have. `Err(CacheStale)` means
    /// "rebuild from inputs"; the message names the first component that differs.
    pub fn check_key(&self, want: &CacheKey) -> Result<()> {
        let k = &self.key;
        if k.kind != want.kind {
            return Err(RtError::CacheStale("tracker kind"));
        }
        if k.builder_version != want.builder_version {
            return Err(RtError::CacheStale("builder version"));
        }
        if k.topology_hash != want.topology_hash {
            return Err(RtError::CacheStale("geometry changed"));
        }
        if k.hierarchy_hash != want.hierarchy_hash {
            return Err(RtError::CacheStale("hierarchy changed"));
        }
        if k.boundary_hash != want.boundary_hash {
            return Err(RtError::CacheStale("boundary conditions changed"));
        }
        if k.options_hash != want.options_hash {
            return Err(RtError::CacheStale("options changed"));
        }
        Ok(())
    }

    /// Payload of a section.
    pub fn section(&self, tag: SectionTag) -> Option<&[u8]> {
        self.sections.iter().find(|s| s.0 == tag).map(|s| s.1.as_slice())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key(h: u128) -> CacheKey {
        CacheKey {
            kind: TrackerKind::MeshWalk,
            builder_version: 3,
            topology_hash: ContentHash(h),
            hierarchy_hash: ContentHash(5),
            boundary_hash: ContentHash(7),
            options_hash: ContentHash(9),
        }
    }

    #[test]
    fn roundtrip_and_64_byte_alignment() {
        let img = CacheImage {
            key: key(77),
            sections: vec![(SectionTag::Bvh, vec![1, 2, 3]), (SectionTag::FacePlanes, vec![9; 100]), (SectionTag::Private(5), vec![])],
        };
        let bytes = img.encode();
        assert_eq!(&bytes[..8], b"SMRTCACH");
        assert_eq!(bytes.len() % 64, 0);
        for i in 0..3 {
            let off = u64::from_le_bytes(bytes[HEADER_LEN + i * ENTRY_LEN + 8..][..8].try_into().unwrap());
            assert_eq!(off % 64, 0);
        }
        let back = CacheImage::decode(&bytes).unwrap();
        assert_eq!(back.section(SectionTag::Bvh), Some(&[1u8, 2, 3][..]));
        assert_eq!(back.section(SectionTag::FacePlanes).map(<[u8]>::len), Some(100));
        assert_eq!(back.key, img.key);
        assert_eq!(back.encode(), bytes, "encode is canonical");
    }

    #[test]
    fn private_tags_cannot_overflow_and_roundtrip() {
        assert_eq!(SectionTag::Private(0).code(), 0x1000);
        assert_eq!(SectionTag::Private(u16::MAX).code(), 0x1000 + 0xFFFF);
        for c in [0u16, 1, 5, u16::MAX] {
            let t = SectionTag::Private(c);
            assert_eq!(SectionTag::from_code(t.code()), Some(t));
        }
        assert_eq!(SectionTag::from_code(0x1000 + 0x1_0000), None);
        assert_eq!(SectionTag::from_code(u32::MAX), None);
        assert_eq!(SectionTag::from_code(0x0FFF), None);
    }

    #[test]
    fn stale_names_the_component_and_corrupt_is_detected() {
        let img = CacheImage { key: key(1), sections: vec![(SectionTag::Bvh, vec![5; 16])] };
        assert!(img.check_key(&key(1)).is_ok());
        assert!(matches!(img.check_key(&key(2)), Err(RtError::CacheStale("geometry changed"))));
        let mut k = key(1);
        k.boundary_hash = ContentHash(8);
        assert!(matches!(img.check_key(&k), Err(RtError::CacheStale("boundary conditions changed"))));
        let mut k = key(1);
        k.hierarchy_hash = ContentHash(0);
        assert!(matches!(img.check_key(&k), Err(RtError::CacheStale("hierarchy changed"))));
        let mut k = key(1);
        k.options_hash = ContentHash(0);
        assert!(matches!(img.check_key(&k), Err(RtError::CacheStale("options changed"))));
        let mut bytes = img.encode();
        let n = bytes.len();
        bytes[n - 60] ^= 0xff; // inside the 16-byte payload at the start of the last block
        assert!(matches!(CacheImage::decode(&bytes), Err(RtError::CacheCorrupt(_))));
        assert!(matches!(CacheImage::decode(&bytes[..10]), Err(RtError::CacheCorrupt(_))));
        let mut stale = img.encode();
        stale[8] = 99;
        assert!(matches!(CacheImage::decode(&stale), Err(RtError::CacheStale(_))));
    }

    #[test]
    fn hash_folds_signed_zero_and_nan() {
        let mut a = ContentHasher::new();
        a.write_f64(0.0);
        a.write_f64(f64::NAN);
        let mut b = ContentHasher::new();
        b.write_f64(-0.0);
        b.write_f64(-f64::NAN);
        assert_eq!(a.finish(), b.finish());
        let mut c = ContentHasher::new();
        c.write_f64(1.0);
        assert_ne!(a.finish(), c.finish());
    }
}
