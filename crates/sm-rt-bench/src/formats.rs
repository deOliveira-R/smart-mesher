//! Case file formats: `smfacets v1`, `smrays v1` (as in `sm-rt-capi/bench/README.md`),
//! `smtets v1` and the `smcase.json` manifest. All binary data is little-endian.
//!
//! | Format | Layout |
//! |---|---|
//! | `smfacets v1` | `u32 nv, u32 nt`; `nv` x 3 `f64`; `nt` x 3 `u32`; `nt` x 2 `i32` (front, back region; -1 is the complement) |
//! | `smrays v1` | `u32 n`; `n` x (origin 3 `f64`, unit direction 3 `f64`); `n` x `i32` start region |
//! | `smtets v1` | `u32 nn, u32 nt`; `nn` x 3 `f64`; `nt` x 4 `u32`; `nt` x `i32` region |

use crate::error::{BenchError, Result};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Triangle soup with the region on each side. The normal (right-hand rule) points out of
/// `front` into `back`; `-1` is the implicit complement.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Facets {
    /// Vertices.
    pub verts: Vec<[f64; 3]>,
    /// Triangles (vertex indices).
    pub tris: Vec<[u32; 3]>,
    /// `[front, back]` region per triangle.
    pub sides: Vec<[i32; 2]>,
}

impl Facets {
    /// Append a triangle soup with one `(front, back)` pair; `flip` reverses every triangle.
    pub fn add(&mut self, verts: &[[f64; 3]], tris: &[[u32; 3]], front: i32, back: i32, flip: bool) -> Result<()> {
        let base = u32::try_from(self.verts.len()).map_err(|_| BenchError::Geometry("too many vertices".into()))?;
        self.verts.extend_from_slice(verts);
        for t in tris {
            let t = if flip { [base + t[0], base + t[2], base + t[1]] } else { [base + t[0], base + t[1], base + t[2]] };
            self.tris.push(t);
            self.sides.push([front, back]);
        }
        Ok(())
    }

    /// Vertex positions of triangle `i`.
    pub fn corners(&self, i: usize) -> [[f64; 3]; 3] {
        let t = self.tris[i];
        [self.verts[t[0] as usize], self.verts[t[1] as usize], self.verts[t[2] as usize]]
    }

    /// Serialise as `smfacets v1`.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut b = Vec::with_capacity(8 + self.verts.len() * 24 + self.tris.len() * 20);
        b.extend_from_slice(&(self.verts.len() as u32).to_le_bytes());
        b.extend_from_slice(&(self.tris.len() as u32).to_le_bytes());
        for v in &self.verts {
            for c in v {
                b.extend_from_slice(&c.to_le_bytes());
            }
        }
        for t in &self.tris {
            for c in t {
                b.extend_from_slice(&c.to_le_bytes());
            }
        }
        for s in &self.sides {
            for c in s {
                b.extend_from_slice(&c.to_le_bytes());
            }
        }
        b
    }

    /// Parse `smfacets v1`; every length and index is checked.
    pub fn from_bytes(b: &[u8]) -> Result<Self> {
        let mut r = Reader::new(b, "smfacets v1");
        let nv = r.u32()? as usize;
        let nt = r.u32()? as usize;
        r.need(nv.saturating_mul(24).saturating_add(nt.saturating_mul(20)))?;
        let mut f = Facets::default();
        for _ in 0..nv {
            f.verts.push([r.f64()?, r.f64()?, r.f64()?]);
        }
        for _ in 0..nt {
            let t = [r.u32()?, r.u32()?, r.u32()?];
            if t.iter().any(|&i| i as usize >= nv) {
                return Err(r.bad("triangle index out of range"));
            }
            f.tris.push(t);
        }
        for _ in 0..nt {
            f.sides.push([r.i32()?, r.i32()?]);
        }
        r.done()?;
        Ok(f)
    }

    /// Bounding box `(lo, hi)`; `None` when empty.
    pub fn bbox(&self) -> Option<([f64; 3], [f64; 3])> {
        bbox_of(self.verts.iter())
    }
}

/// Bounding box of points.
pub fn bbox_of<'a>(it: impl Iterator<Item = &'a [f64; 3]>) -> Option<([f64; 3], [f64; 3])> {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    let mut any = false;
    for p in it {
        any = true;
        for a in 0..3 {
            lo[a] = lo[a].min(p[a]);
            hi[a] = hi[a].max(p[a]);
        }
    }
    any.then_some((lo, hi))
}

/// Seeded rays with their start regions.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Rays {
    /// Origins.
    pub origin: Vec<[f64; 3]>,
    /// Unit directions.
    pub dir: Vec<[f64; 3]>,
    /// Region of each origin.
    pub region: Vec<i32>,
}

impl Rays {
    /// Number of rays.
    pub fn len(&self) -> usize {
        self.origin.len()
    }

    /// True without rays.
    pub fn is_empty(&self) -> bool {
        self.origin.is_empty()
    }

    /// Append one ray.
    pub fn push(&mut self, o: [f64; 3], d: [f64; 3], region: i32) {
        self.origin.push(o);
        self.dir.push(d);
        self.region.push(region);
    }

    /// The first `n` rays (all when fewer).
    pub fn truncated(&self, n: usize) -> Rays {
        let n = n.min(self.len());
        Rays { origin: self.origin[..n].to_vec(), dir: self.dir[..n].to_vec(), region: self.region[..n].to_vec() }
    }

    /// Serialise as `smrays v1`.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut b = Vec::with_capacity(4 + self.len() * 52);
        b.extend_from_slice(&(self.len() as u32).to_le_bytes());
        for i in 0..self.len() {
            for c in self.origin[i].iter().chain(self.dir[i].iter()) {
                b.extend_from_slice(&c.to_le_bytes());
            }
        }
        for g in &self.region {
            b.extend_from_slice(&g.to_le_bytes());
        }
        b
    }

    /// Parse `smrays v1`; the direction must be a unit vector within 1e-9.
    pub fn from_bytes(b: &[u8]) -> Result<Self> {
        let mut r = Reader::new(b, "smrays v1");
        let n = r.u32()? as usize;
        r.need(n.saturating_mul(52))?;
        let mut out = Rays::default();
        for _ in 0..n {
            let o = [r.f64()?, r.f64()?, r.f64()?];
            let d = [r.f64()?, r.f64()?, r.f64()?];
            let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            if (len - 1.0).abs().partial_cmp(&1e-9) != Some(core::cmp::Ordering::Less) || o.iter().any(|c| !c.is_finite()) {
                return Err(r.bad("non-finite origin or non-unit direction"));
            }
            out.origin.push(o);
            out.dir.push(d);
        }
        for _ in 0..n {
            out.region.push(r.i32()?);
        }
        r.done()?;
        Ok(out)
    }
}

/// Tetrahedral volume mesh (`smtets v1`), the input of cell-walking modes and XDG.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Tets {
    /// Nodes.
    pub nodes: Vec<[f64; 3]>,
    /// Tetrahedra (node indices, positive orientation).
    pub tets: Vec<[u32; 4]>,
    /// Region per tetrahedron.
    pub region: Vec<i32>,
}

impl Tets {
    /// Serialise as `smtets v1`.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut b = Vec::with_capacity(8 + self.nodes.len() * 24 + self.tets.len() * 20);
        b.extend_from_slice(&(self.nodes.len() as u32).to_le_bytes());
        b.extend_from_slice(&(self.tets.len() as u32).to_le_bytes());
        for n in &self.nodes {
            for c in n {
                b.extend_from_slice(&c.to_le_bytes());
            }
        }
        for t in &self.tets {
            for c in t {
                b.extend_from_slice(&c.to_le_bytes());
            }
        }
        for g in &self.region {
            b.extend_from_slice(&g.to_le_bytes());
        }
        b
    }

    /// Parse `smtets v1`; indices are range checked.
    pub fn from_bytes(b: &[u8]) -> Result<Self> {
        let mut r = Reader::new(b, "smtets v1");
        let nn = r.u32()? as usize;
        let nt = r.u32()? as usize;
        r.need(nn.saturating_mul(24).saturating_add(nt.saturating_mul(20)))?;
        let mut m = Tets::default();
        for _ in 0..nn {
            m.nodes.push([r.f64()?, r.f64()?, r.f64()?]);
        }
        for _ in 0..nt {
            let t = [r.u32()?, r.u32()?, r.u32()?, r.u32()?];
            if t.iter().any(|&i| i as usize >= nn) {
                return Err(r.bad("tet index out of range"));
            }
            m.tets.push(t);
        }
        for _ in 0..nt {
            m.region.push(r.i32()?);
        }
        r.done()?;
        Ok(m)
    }
}

struct Reader<'a> {
    b: &'a [u8],
    at: usize,
    what: &'static str,
}

impl<'a> Reader<'a> {
    fn new(b: &'a [u8], what: &'static str) -> Self {
        Self { b, at: 0, what }
    }
    fn bad(&self, why: &str) -> BenchError {
        BenchError::Format { what: self.what, why: why.to_string() }
    }
    fn need(&self, n: usize) -> Result<()> {
        if self.b.len().saturating_sub(self.at) < n { Err(self.bad("truncated")) } else { Ok(()) }
    }
    fn take<const N: usize>(&mut self) -> Result<[u8; N]> {
        self.need(N)?;
        let mut a = [0u8; N];
        a.copy_from_slice(&self.b[self.at..self.at + N]);
        self.at += N;
        Ok(a)
    }
    fn u32(&mut self) -> Result<u32> {
        Ok(u32::from_le_bytes(self.take()?))
    }
    fn i32(&mut self) -> Result<i32> {
        Ok(i32::from_le_bytes(self.take()?))
    }
    fn f64(&mut self) -> Result<f64> {
        Ok(f64::from_le_bytes(self.take()?))
    }
    fn done(&self) -> Result<()> {
        if self.at == self.b.len() { Ok(()) } else { Err(self.bad("trailing bytes")) }
    }
}

/// One region of a case with the benchmark's own transport workload constants. These are never
/// input to sm-rt (cross sections stay outside it, ruling Q2).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RegionSpec {
    /// Region id used in all case files.
    pub id: i32,
    /// Name.
    pub name: String,
    /// Total cross section of the W-flight workload, 1/cm.
    pub sigma_t: f64,
    /// Absorption cross section of the W-flight workload, 1/cm.
    pub sigma_a: f64,
}

/// `smcase.json`: everything a competitor needs to know besides the binary files.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Manifest {
    /// Always `smcase v1`.
    pub schema: String,
    /// Case name.
    pub case: String,
    /// Scale name (`full`, `reduced`, `tiny`, `model`).
    pub scale: String,
    /// Regions with workload constants.
    pub regions: Vec<RegionSpec>,
    /// Facet twin tolerance (maximum deviation from the exact surface), cm.
    pub sag: f64,
    /// Agreement tolerance for chords, cm: `max(1e-9 diag, sag bound)`.
    pub chord_tol: f64,
    /// Bounding box low corner.
    pub bbox_lo: [f64; 3],
    /// Bounding box high corner.
    pub bbox_hi: [f64; 3],
    /// Bounding box diagonal, cm.
    pub diag: f64,
    /// Number of vertices of the facet twin.
    pub vertices: usize,
    /// Number of triangles of the facet twin.
    pub triangles: usize,
    /// Number of tetrahedra (0 when the case has none).
    pub tets: usize,
    /// Number of rays.
    pub rays: usize,
    /// Prototype count of the hierarchical model.
    pub prototypes: usize,
    /// Placed copies of prototypes.
    pub copies: usize,
    /// `os-arch` the tessellated files were produced on (tessellation uses platform `sin`/`cos`).
    pub platform: String,
    /// Free-form notes (deviations, limitations).
    pub notes: Vec<String>,
    /// SHA-256 of every other file of the case, by file name.
    pub files: BTreeMap<String, String>,
}

/// The platform tag `os-arch` of this build.
pub fn platform_tag() -> String {
    format!("{}-{}", std::env::consts::OS, std::env::consts::ARCH)
}
