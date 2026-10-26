//! Serialized form: a rebuildable cache keyed by content hash.
//!
//! The tracking structure is **derived data**. It is never authoritative and is never needed to
//! open a project: a missing or mismatched cache is rebuilt. The blob has the same layout idea
//! as `.smsh` (a header that names the arrays, then 64-byte aligned little-endian arrays), so
//! `sm-meshio` can embed it as the `rt/` group of a `.smsh` file (array names are prefixed with
//! `rt/` there) or store it next to the project in `.cache/rt/<digest>.smrt`.
//!
//! Layout of [`RtCacheBlob::to_bytes`] (all integers little endian):
//! `"SMRT"`, `u32 version`, five `u64` key words, `u32 n_arrays`, then per array
//! `u16 name_len, name, u8 dtype, u64 count, u64 absolute offset`; padding to 64; array bytes,
//! each starting at a 64-aligned offset.

use crate::boundary::{BoundarySpec, FaceAction, ResponseKind};
use crate::error::{Result, RtError};
use crate::tracker::RtOptions;

/// Format version written into every blob.
pub const CACHE_VERSION: u32 = 1;

/// FNV-1a 64-bit, the same family the topology state hash uses. Stable across platforms because
/// callers feed it integers and float *bit patterns* of values already quantized by the model.
#[derive(Clone, Copy, Debug)]
pub struct Fnv64(u64);

impl Default for Fnv64 {
    fn default() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
}

impl Fnv64 {
    /// Feed one word (little-endian bytes).
    pub fn word(&mut self, w: u64) {
        for b in w.to_le_bytes() {
            self.0 = (self.0 ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3);
        }
    }

    /// The hash.
    pub fn finish(&self) -> u64 {
        self.0
    }
}

/// What the cache is a function of. Equal keys mean the cached structure is valid.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct RtCacheKey {
    /// `TopologyView::canonical_hash()` for B-rep input, the facet soup hash, or the mesh
    /// connectivity hash.
    pub topology_hash: u64,
    /// Hash of the hierarchy (`Instancing` canonical words), 0 if there is none.
    pub hierarchy_hash: u64,
    /// [`boundary_hash`] of the `BoundarySpec`.
    pub boundary_hash: u64,
    /// Hash of [`RtOptions::hash_words`].
    pub options_hash: u64,
    /// Version of the implementor's layout; bump it whenever the arrays change meaning.
    pub builder_version: u32,
}

impl RtCacheKey {
    /// Assemble a key; `options` and `boundaries` are hashed here.
    pub fn new(topology_hash: u64, hierarchy_hash: u64, boundaries: &BoundarySpec, options: &RtOptions, builder_version: u32) -> Self {
        let mut h = Fnv64::default();
        options.hash_words().iter().for_each(|w| h.word(*w));
        Self { topology_hash, hierarchy_hash, boundary_hash: boundary_hash(boundaries), options_hash: h.finish(), builder_version }
    }

    /// One digest of the whole key (the cache file name).
    pub fn digest(&self) -> u64 {
        let mut h = Fnv64::default();
        for w in [self.topology_hash, self.hierarchy_hash, self.boundary_hash, self.options_hash, u64::from(self.builder_version)] {
            h.word(w);
        }
        h.finish()
    }

    fn words(&self) -> [u64; 5] {
        [self.topology_hash, self.hierarchy_hash, self.boundary_hash, self.options_hash, u64::from(self.builder_version)]
    }
}

/// Hash of a boundary spec, in ascending face order.
pub fn boundary_hash(spec: &BoundarySpec) -> u64 {
    let mut h = Fnv64::default();
    let act = |h: &mut Fnv64, a: FaceAction| match a {
        FaceAction::Transmit => h.word(0),
        FaceAction::Reflect => h.word(1),
        FaceAction::Wrap { transform, mate } => {
            h.word(2);
            h.word(mate.to_raw());
            for r in transform.m {
                r.iter().for_each(|v| h.word(v.to_bits()));
            }
            [transform.t.x, transform.t.y, transform.t.z].iter().for_each(|v| h.word(v.to_bits()));
        }
        FaceAction::Respond(k) => {
            h.word(3);
            match k {
                ResponseKind::Vacuum => h.word(0),
                ResponseKind::White => h.word(1),
                ResponseKind::Albedo(a) => {
                    h.word(2);
                    h.word(a.to_bits());
                }
                ResponseKind::Reflective => h.word(3),
            }
        }
    };
    act(&mut h, spec.exterior);
    for (f, a) in spec.iter() {
        h.word(f.to_raw());
        act(&mut h, a);
    }
    h.finish()
}

/// Element type of a cache array.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Dtype {
    /// Bytes.
    U8 = 0,
    /// `u32`.
    U32 = 1,
    /// `u64`.
    U64 = 2,
    /// `f32`.
    F32 = 3,
    /// `f64`.
    F64 = 4,
}

impl Dtype {
    /// Bytes per element.
    pub const fn size(self) -> usize {
        match self {
            Dtype::U8 => 1,
            Dtype::U32 | Dtype::F32 => 4,
            Dtype::U64 | Dtype::F64 => 8,
        }
    }

    fn from_code(c: u8) -> Option<Self> {
        Some(match c {
            0 => Dtype::U8,
            1 => Dtype::U32,
            2 => Dtype::U64,
            3 => Dtype::F32,
            4 => Dtype::F64,
            _ => return None,
        })
    }
}

/// One named array of the blob (CSR-style structure-of-arrays: BVH nodes, leaf face lists,
/// per-prototype face tables, lattice slot tables, copy table).
#[derive(Clone, Debug, PartialEq)]
pub struct RtArray {
    /// Name, e.g. `bvh/lo`, `proto/face_ptr`, `lattice/slot_fill`.
    pub name: String,
    /// Element type.
    pub dtype: Dtype,
    /// Number of elements.
    pub count: u64,
    /// Little-endian element bytes, `count * dtype.size()` long.
    pub bytes: Vec<u8>,
}

/// The serializable tracking structure.
#[derive(Clone, Debug, PartialEq)]
pub struct RtCacheBlob {
    /// What it was built from.
    pub key: RtCacheKey,
    /// Arrays in a fixed order chosen by the implementor.
    pub arrays: Vec<RtArray>,
}

const ALIGN: usize = 64;

fn pad(v: &mut Vec<u8>) {
    while !v.len().is_multiple_of(ALIGN) {
        v.push(0);
    }
}

impl RtCacheBlob {
    /// Encode. Fails if an array's byte length disagrees with `count * dtype.size()`.
    pub fn to_bytes(&self) -> Result<Vec<u8>> {
        let mut head_len = 4 + 4 + 40 + 4;
        for a in &self.arrays {
            if a.bytes.len() as u64 != a.count * a.dtype.size() as u64 {
                return Err(RtError::Cache(format!("array {} has wrong byte length", a.name)));
            }
            head_len += 2 + a.name.len() + 1 + 8 + 8;
        }
        let mut offs = Vec::with_capacity(self.arrays.len());
        let mut at = head_len.div_ceil(ALIGN) * ALIGN;
        for a in &self.arrays {
            offs.push(at as u64);
            at = (at + a.bytes.len()).div_ceil(ALIGN) * ALIGN;
        }
        let mut out = Vec::with_capacity(at);
        out.extend_from_slice(b"SMRT");
        out.extend_from_slice(&CACHE_VERSION.to_le_bytes());
        for w in self.key.words() {
            out.extend_from_slice(&w.to_le_bytes());
        }
        out.extend_from_slice(&(self.arrays.len() as u32).to_le_bytes());
        for (a, off) in self.arrays.iter().zip(&offs) {
            let n = u16::try_from(a.name.len()).map_err(|_| RtError::Cache("array name too long".into()))?;
            out.extend_from_slice(&n.to_le_bytes());
            out.extend_from_slice(a.name.as_bytes());
            out.push(a.dtype as u8);
            out.extend_from_slice(&a.count.to_le_bytes());
            out.extend_from_slice(&off.to_le_bytes());
        }
        for a in &self.arrays {
            pad(&mut out);
            out.extend_from_slice(&a.bytes);
        }
        Ok(out)
    }

    /// Decode, checking magic, version and every offset. `expect` rejects a blob built for other
    /// content (`RtError::Cache`), which the caller answers by rebuilding.
    pub fn from_bytes(b: &[u8], expect: Option<&RtCacheKey>) -> Result<Self> {
        let bad = |m: &str| RtError::Cache(m.to_string());
        let mut p = 0usize;
        let mut take = |n: usize| -> Result<&[u8]> {
            let s = b.get(p..p + n).ok_or_else(|| bad("truncated"))?;
            p += n;
            Ok(s)
        };
        if take(4)? != b"SMRT" {
            return Err(bad("bad magic"));
        }
        let ver = u32::from_le_bytes(take(4)?.try_into().map_err(|_| bad("truncated"))?);
        if ver != CACHE_VERSION {
            return Err(bad("unsupported version"));
        }
        let mut w = [0u64; 5];
        for x in &mut w {
            *x = u64::from_le_bytes(take(8)?.try_into().map_err(|_| bad("truncated"))?);
        }
        let key = RtCacheKey { topology_hash: w[0], hierarchy_hash: w[1], boundary_hash: w[2], options_hash: w[3], builder_version: w[4] as u32 };
        if expect.is_some_and(|e| *e != key) {
            return Err(bad("content key mismatch"));
        }
        let n = u32::from_le_bytes(take(4)?.try_into().map_err(|_| bad("truncated"))?) as usize;
        let mut table = Vec::new();
        for _ in 0..n {
            let nl = u16::from_le_bytes(take(2)?.try_into().map_err(|_| bad("truncated"))?) as usize;
            let name = String::from_utf8(take(nl)?.to_vec()).map_err(|_| bad("name is not utf-8"))?;
            let dt = Dtype::from_code(take(1)?[0]).ok_or_else(|| bad("unknown dtype"))?;
            let count = u64::from_le_bytes(take(8)?.try_into().map_err(|_| bad("truncated"))?);
            let off = u64::from_le_bytes(take(8)?.try_into().map_err(|_| bad("truncated"))?);
            table.push((name, dt, count, off));
        }
        let mut arrays = Vec::with_capacity(n);
        for (name, dtype, count, off) in table {
            let len = count.checked_mul(dtype.size() as u64).ok_or_else(|| bad("size overflow"))?;
            let (off, len) = (usize::try_from(off).map_err(|_| bad("offset"))?, usize::try_from(len).map_err(|_| bad("size"))?);
            if !off.is_multiple_of(ALIGN) {
                return Err(bad("unaligned array"));
            }
            let bytes = b.get(off..off.checked_add(len).ok_or_else(|| bad("offset"))?).ok_or_else(|| bad("array out of range"))?.to_vec();
            arrays.push(RtArray { name, dtype, count, bytes });
        }
        Ok(Self { key, arrays })
    }

    /// The array called `name`.
    pub fn array(&self, name: &str) -> Option<&RtArray> {
        self.arrays.iter().find(|a| a.name == name)
    }
}
