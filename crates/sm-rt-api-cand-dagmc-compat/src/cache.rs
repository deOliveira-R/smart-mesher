//! Content hashing and the serialized form of the acceleration structure.
//!
//! The cache is a rebuildable by-product keyed by a content hash, never a source of truth. It
//! is stored as a section of a `.smsh` file (or beside it as `<name>.smsh-rt`), in the same
//! container style as the mesh format: a header, then 64-byte-aligned little-endian sections.
//!
//! Layout (all integers little-endian):
//! ```text
//! 0   magic "SMRTCACH"          8
//! 8   format_version u32        4
//! 12  precision u32             4   0 = F64, 1 = F32Traversal
//! 16  input_hash u128           16
//! 32  params_hash u128          16
//! 48  backend_name_len u32      4   then name bytes, then pad to 8
//!     section_count u32         4
//!     per section: tag[8], offset u64, len u64, checksum u64   (32 bytes)
//! ... payload sections, each starting at a multiple of 64
//! ```

use crate::build::{BuildInput, BuildParams, Precision};
use crate::error::{Result, RtError};

/// 128-bit content hash (two independent 64-bit FNV-1a lanes, avalanche-mixed). Not
/// cryptographic; it keys a cache and detects accidental change.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ContentHash(pub u128);

/// Incremental hasher for [`ContentHash`]. Feed it values in a canonical order (sorted by
/// entity id), never by hash-map order.
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

impl ContentHasher {
    /// New hasher.
    pub const fn new() -> Self {
        Self { a: 0xcbf2_9ce4_8422_2325, b: 0x8422_2325_cbf2_9ce4 }
    }
    /// Absorb bytes.
    pub fn write(&mut self, bytes: &[u8]) {
        for &x in bytes {
            self.a = (self.a ^ u64::from(x)).wrapping_mul(0x0000_0100_0000_01b3);
            self.b = (self.b ^ u64::from(x).wrapping_add(0x9e)).wrapping_mul(0x0000_0100_0000_01b3 ^ 0x2f);
        }
    }
    /// Absorb a `u64`.
    pub fn write_u64(&mut self, v: u64) {
        self.write(&v.to_le_bytes());
    }
    /// Absorb an `f64` by bit pattern (`-0.0` and `0.0` differ; NaN payloads differ).
    pub fn write_f64(&mut self, v: f64) {
        self.write_u64(v.to_bits());
    }
    /// Finish.
    pub fn finish(&self) -> ContentHash {
        let mix = |mut z: u64| {
            z ^= z >> 33;
            z = z.wrapping_mul(0xff51_afd7_ed55_8ccd);
            z ^= z >> 33;
            z = z.wrapping_mul(0xc4ce_b9fe_1a85_ec53);
            z ^ (z >> 33)
        };
        ContentHash((u128::from(mix(self.a ^ self.b.rotate_left(17))) << 64) | u128::from(mix(self.b ^ self.a.rotate_left(31))))
    }
}

/// Hash a build input. Hash order is canonical: volumes and surfaces by entity id.
pub(crate) fn hash_input(input: &BuildInput<'_>) -> ContentHash {
    let mut h = ContentHasher::new();
    match input {
        BuildInput::Analytic(spec) => {
            h.write(b"analytic");
            spec.hash_into(&mut h);
        }
        BuildInput::Brep(b) => {
            h.write(b"brep");
            let mut vols: Vec<_> = b.volumes.iter().collect();
            vols.sort_by_key(|v| v.id);
            for v in vols {
                h.write_u64(v.id.to_raw());
                h.write_u64(u64::from(v.tag));
            }
            let mut surfs: Vec<_> = b.surfaces.iter().collect();
            surfs.sort_by_key(|s| s.id);
            for s in surfs {
                h.write_u64(s.id.to_raw());
                let soup = match &s.geometry {
                    crate::build::SurfaceGeometry::Facets(f) => f,
                    crate::build::SurfaceGeometry::Exact { carrier, trim } => {
                        h.write_u64(carrier.to_raw());
                        trim
                    }
                };
                for p in &soup.xyz {
                    p.iter().for_each(|&c| h.write_f64(c));
                }
                for t in &soup.tris {
                    t.iter().for_each(|&i| h.write_u64(u64::from(i)));
                }
                let reg = |r: &sm_topo_api::Region| match r {
                    sm_topo_api::Region::Volume(v) => v.to_raw(),
                    sm_topo_api::Region::Exterior => 0,
                };
                h.write_u64(reg(&s.forward));
                h.write_u64(reg(&s.reverse));
                h.write(format!("{:?}", s.boundary).as_bytes());
            }
        }
        BuildInput::Mesh(m) => {
            h.write(b"mesh");
            let c = sm_mesh_api::connectivity_hash(m.mesh);
            h.write_u64(c);
            h.write_u64(sm_mesh_api::coordinate_hash(m.mesh));
            for (id, b) in &m.boundaries {
                h.write_u64(id.to_raw());
                h.write(format!("{b:?}").as_bytes());
            }
        }
        BuildInput::Instanced(i) => {
            h.write(b"instanced");
            for p in &i.prototypes {
                h.write_u64(hash_input(p).0 as u64);
            }
            for s in &i.instances {
                h.write_u64(u64::from(s.prototype));
                h.write(format!("{:?}", s.transform).as_bytes());
            }
            for l in &i.lattices {
                h.write(format!("{l:?}").as_bytes());
            }
        }
    }
    h.finish()
}

/// Hash the parameters that change the built structure.
pub fn hash_params(p: &BuildParams) -> ContentHash {
    let mut h = ContentHasher::new();
    h.write_u64(p.precision as u64);
    h.write_f64(p.surface_eps);
    h.write_f64(p.seal_tol);
    h.write_u64(u64::from(p.leaf_size));
    // `threads` and `require_sealed` do not change the structure.
    h.finish()
}

/// One payload section.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SectionEntry {
    /// Section tag (backend-defined, e.g. `b"BVHNODES"`).
    pub tag: [u8; 8],
    /// Byte offset from the start of the blob (multiple of 64).
    pub offset: u64,
    /// Length in bytes.
    pub len: u64,
    /// Checksum of the section bytes (low 64 bits of [`ContentHasher`]).
    pub checksum: u64,
}

/// Header of a cache blob.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CacheHeader {
    /// Layout version of this header.
    pub format_version: u32,
    /// Backend name.
    pub backend: String,
    /// Precision the structure was built for.
    pub precision: Precision,
    /// [`BuildInput::content_hash`] of the input.
    pub input_hash: ContentHash,
    /// [`hash_params`] of the parameters.
    pub params_hash: ContentHash,
    /// Payload sections.
    pub sections: Vec<SectionEntry>,
}

/// Current header layout version.
pub const CACHE_FORMAT_VERSION: u32 = 1;
const MAGIC: &[u8; 8] = b"SMRTCACH";

impl CacheHeader {
    /// True if the blob was built from this input, with these parameters, by this backend.
    pub fn matches(&self, backend: &str, input: ContentHash, params: &BuildParams) -> bool {
        self.format_version == CACHE_FORMAT_VERSION
            && self.backend == backend
            && self.input_hash == input
            && self.params_hash == hash_params(params)
    }
}

/// A header plus its payload: what [`crate::World::export_cache`] returns and
/// [`crate::Backend::restore`] consumes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CacheBlob {
    /// Header.
    pub header: CacheHeader,
    /// Payload bytes; section offsets are relative to the start of the encoded blob, so the
    /// payload here already includes alignment padding.
    pub payload: Vec<u8>,
}

fn align(n: usize, a: usize) -> usize {
    n.div_ceil(a) * a
}

fn checksum(bytes: &[u8]) -> u64 {
    let mut h = ContentHasher::new();
    h.write(bytes);
    h.finish().0 as u64
}

impl CacheBlob {
    /// Assemble a blob from named sections.
    pub fn from_sections(
        backend: &str,
        precision: Precision,
        input_hash: ContentHash,
        params: &BuildParams,
        sections: &[([u8; 8], Vec<u8>)],
    ) -> Self {
        let header_len = Self::header_len(backend, sections.len());
        let mut entries = Vec::with_capacity(sections.len());
        let mut payload = Vec::new();
        let mut at = align(header_len, 64);
        for (tag, bytes) in sections {
            payload.resize(at - align(header_len, 64), 0);
            payload.extend_from_slice(bytes);
            entries.push(SectionEntry { tag: *tag, offset: at as u64, len: bytes.len() as u64, checksum: checksum(bytes) });
            at = align(at + bytes.len(), 64);
        }
        payload.resize(at - align(header_len, 64), 0);
        let header = CacheHeader {
            format_version: CACHE_FORMAT_VERSION,
            backend: backend.to_string(),
            precision,
            input_hash,
            params_hash: hash_params(params),
            sections: entries,
        };
        Self { header, payload }
    }

    fn header_len(backend: &str, nsec: usize) -> usize {
        48 + 4 + align(backend.len(), 8) + 4 + 32 * nsec
    }

    /// Bytes of a section, verified against its checksum.
    pub fn section(&self, tag: &[u8; 8]) -> Result<&[u8]> {
        let e = self
            .header
            .sections
            .iter()
            .find(|e| &e.tag == tag)
            .ok_or_else(|| RtError::Cache(format!("missing section {}", String::from_utf8_lossy(tag))))?;
        let base = align(Self::header_len(&self.header.backend, self.header.sections.len()), 64) as u64;
        let (lo, hi) = (e.offset - base, e.offset - base + e.len);
        let bytes = self
            .payload
            .get(lo as usize..hi as usize)
            .ok_or_else(|| RtError::Cache("section out of range".into()))?;
        if checksum(bytes) != e.checksum {
            return Err(RtError::Cache("section checksum mismatch".into()));
        }
        Ok(bytes)
    }

    /// Encode to the on-disk layout.
    pub fn to_bytes(&self) -> Vec<u8> {
        let h = &self.header;
        let mut out = Vec::new();
        out.extend_from_slice(MAGIC);
        out.extend_from_slice(&h.format_version.to_le_bytes());
        out.extend_from_slice(&(h.precision as u32).to_le_bytes());
        out.extend_from_slice(&h.input_hash.0.to_le_bytes());
        out.extend_from_slice(&h.params_hash.0.to_le_bytes());
        out.extend_from_slice(&(h.backend.len() as u32).to_le_bytes());
        out.extend_from_slice(h.backend.as_bytes());
        out.resize(48 + 4 + align(h.backend.len(), 8), 0);
        out.extend_from_slice(&(h.sections.len() as u32).to_le_bytes());
        for e in &h.sections {
            out.extend_from_slice(&e.tag);
            out.extend_from_slice(&e.offset.to_le_bytes());
            out.extend_from_slice(&e.len.to_le_bytes());
            out.extend_from_slice(&e.checksum.to_le_bytes());
        }
        out.resize(align(out.len(), 64), 0);
        out.extend_from_slice(&self.payload);
        out
    }

    /// Decode; fails with `RtError::Cache` on any inconsistency.
    pub fn from_bytes(b: &[u8]) -> Result<Self> {
        let bad = |m: &str| RtError::Cache(m.to_string());
        let u32_at = |o: usize| -> Result<u32> {
            Ok(u32::from_le_bytes(b.get(o..o + 4).ok_or_else(|| bad("truncated"))?.try_into().map_err(|_| bad("truncated"))?))
        };
        let u64_at = |o: usize| -> Result<u64> {
            Ok(u64::from_le_bytes(b.get(o..o + 8).ok_or_else(|| bad("truncated"))?.try_into().map_err(|_| bad("truncated"))?))
        };
        let u128_at = |o: usize| -> Result<u128> {
            Ok(u128::from_le_bytes(b.get(o..o + 16).ok_or_else(|| bad("truncated"))?.try_into().map_err(|_| bad("truncated"))?))
        };
        if b.get(0..8) != Some(MAGIC) {
            return Err(bad("bad magic"));
        }
        let format_version = u32_at(8)?;
        if format_version != CACHE_FORMAT_VERSION {
            return Err(bad("unsupported version"));
        }
        let precision = match u32_at(12)? {
            0 => Precision::F64,
            1 => Precision::F32Traversal,
            _ => return Err(bad("bad precision")),
        };
        let (input_hash, params_hash) = (ContentHash(u128_at(16)?), ContentHash(u128_at(32)?));
        let nlen = u32_at(48)? as usize;
        let name = b.get(52..52 + nlen).ok_or_else(|| bad("truncated"))?;
        let backend = String::from_utf8(name.to_vec()).map_err(|_| bad("backend name not utf-8"))?;
        let mut o = 52 + align(nlen, 8);
        let nsec = u32_at(o)? as usize;
        o += 4;
        let mut sections = Vec::new();
        for _ in 0..nsec.min(4096) {
            let tag: [u8; 8] = b.get(o..o + 8).ok_or_else(|| bad("truncated"))?.try_into().map_err(|_| bad("truncated"))?;
            sections.push(SectionEntry { tag, offset: u64_at(o + 8)?, len: u64_at(o + 16)?, checksum: u64_at(o + 24)? });
            o += 32;
        }
        if sections.len() != nsec {
            return Err(bad("too many sections"));
        }
        let start = align(o, 64);
        let payload = b.get(start..).ok_or_else(|| bad("truncated"))?.to_vec();
        Ok(Self { header: CacheHeader { format_version, backend, precision, input_hash, params_hash, sections }, payload })
    }
}
