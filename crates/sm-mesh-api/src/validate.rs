//! Implementor-independent validation and canonical content hashing.
//!
//! [`validate`] checks the declared contract (orientation rule, closure, classification rule,
//! schema consistency) on any [`MeshRead`]. [`canonical_hash`] digests the mesh in canonical
//! order for determinism tests and cache keys; it is stable across runs and thread counts.

use sm_core::Vector3;

use crate::classify::check_classification;
use crate::index::{CellIx, Dim, FaceIx, Sign};
use crate::query::MeshRead;
use crate::tag::TagData;

/// One validation finding.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Violation {
    /// Stable machine-readable code, e.g. `orientation.left_not_lower`.
    pub code: &'static str,
    /// Human-readable detail.
    pub detail: String,
}

/// Validation knobs.
#[derive(Clone, Copy, Debug)]
pub struct ValidateOptions {
    /// Relative tolerance for geometric closure checks.
    pub tol: f64,
    /// Report entities with no classification.
    pub require_classification: bool,
    /// Check geometry (volume sign, closure, normal direction). Needs real coordinates.
    pub check_geometry: bool,
}

impl Default for ValidateOptions {
    fn default() -> Self {
        Self { tol: 1e-9, require_classification: false, check_geometry: true }
    }
}

fn v(code: &'static str, detail: String) -> Violation {
    Violation { code, detail }
}

/// Check a mesh against the contract. An empty result means valid.
pub fn validate(m: &dyn MeshRead, opts: &ValidateOptions) -> Vec<Violation> {
    let mut out = Vec::new();
    let (nv, nf, nc) = (m.len(Dim::Vertex), m.len(Dim::Face), m.len(Dim::Cell));

    for f in 0..nf as u32 {
        let fi = FaceIx(f);
        let l = m.face_vertices(fi);
        if l.len() < 3 || l.iter().any(|x| x.index() >= nv) {
            out.push(v("face.loop", format!("face {f} has a bad vertex loop")));
        }
        if l.first() != l.iter().min() {
            out.push(v("face.loop_start", format!("face {f} loop does not start at its smallest vertex")));
        }
        let left = m.face_left(fi);
        if left.index() >= nc {
            out.push(v("face.left_range", format!("face {f} left out of range")));
            continue;
        }
        if let Some(r) = m.face_right(fi)
            && (r.index() >= nc || r <= left)
        {
            out.push(v("orientation.left_not_lower", format!("face {f}: left {left:?} must be lower than right {r:?}")));
        }
    }

    // Cell->face and face->cell must agree: Plus iff the cell is `left`.
    let mut seen = vec![0u8; nf];
    for c in 0..nc as u32 {
        let ci = CellIx(c);
        for sf in m.cell_faces(ci) {
            let f = sf.face();
            if f.index() >= nf {
                out.push(v("cell.face_range", format!("cell {c} references face {}", f.0)));
                continue;
            }
            seen[f.index()] = seen[f.index()].saturating_add(1);
            let ok = match sf.sign() {
                Sign::Plus => m.face_left(f) == ci,
                Sign::Minus => m.face_right(f) == Some(ci),
            };
            if !ok {
                out.push(v("orientation.sign", format!("cell {c} sign on face {} disagrees with left/right", f.0)));
            }
        }
    }
    for (f, &count) in seen.iter().enumerate() {
        let want = if m.face_right(FaceIx(f as u32)).is_some() { 2 } else { 1 };
        if count != want {
            out.push(v("face.incidence", format!("face {f} listed by {count} cells, expected {want}")));
        }
    }

    if opts.check_geometry {
        for c in 0..nc as u32 {
            let ci = CellIx(c);
            let mut sum = Vector3::default();
            let mut scale = 0.0;
            for sf in m.cell_faces(ci) {
                let a = m.outward_area_vector(*sf);
                scale += a.norm();
                sum = sum + a;
            }
            if sum.norm() > opts.tol * scale.max(1e-300) {
                out.push(v("cell.closure", format!("cell {c} is not closed (sum of outward areas {:.3e})", sum.norm())));
            }
            let (vol, cen) = m.cell_volume_centroid(ci);
            if vol <= 0.0 {
                out.push(v("cell.volume", format!("cell {c} has non-positive volume {vol:.3e}")));
            }
            // The normal of every interior face must point from left to right.
            for sf in m.cell_faces(ci) {
                let f = sf.face();
                if m.face_left(f) == ci
                    && let Some(r) = m.face_right(f)
                {
                    let (_, rc) = m.cell_volume_centroid(r);
                    let d = Vector3::new(rc.x - cen.x, rc.y - cen.y, rc.z - cen.z);
                    if m.face_area_vector(f).dot(d) <= 0.0 {
                        out.push(v("orientation.normal", format!("face {} normal does not point left to right", f.0)));
                    }
                }
            }
        }
    }

    // Classification dimension rule and completeness.
    for dim in Dim::ALL {
        for ix in 0..m.len(dim) as u32 {
            match m.classification(dim, ix) {
                Some(c) => {
                    if check_classification(dim, c.entity).is_err() {
                        out.push(v("class.dim", format!("{dim:?} {ix} classified on {:?}", c.entity)));
                    }
                }
                None if opts.require_classification => {
                    out.push(v("class.missing", format!("{dim:?} {ix} is unclassified")));
                }
                None => {}
            }
        }
    }

    // Schema vs instance.
    for def in &m.schema().tags {
        match m.tag(def.dim, &def.name) {
            Some(col) if col.len() == m.len(def.dim) && col.def == *def => {}
            _ => out.push(v("schema.tag", format!("tag `{}` missing or wrong length", def.name))),
        }
    }
    for sd in &m.schema().sets {
        match m.set(&sd.name) {
            Some(s) if s.dim == sd.dim && s.members().iter().all(|&i| (i as usize) < m.len(sd.dim)) => {}
            _ => out.push(v("schema.set", format!("set `{}` missing or out of range", sd.name))),
        }
    }
    out
}

/// FNV-1a 64-bit hasher (no dependencies, fixed algorithm so hashes are stable across releases).
struct Fnv(u64);
impl Fnv {
    fn bytes(&mut self, b: &[u8]) {
        for &x in b {
            self.0 ^= u64::from(x);
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    fn u64(&mut self, x: u64) {
        self.bytes(&x.to_le_bytes());
    }
}

/// Digest of the whole mesh in canonical order: ids, coordinates (bit patterns), topology,
/// shapes, classification, tags and sets. Equal meshes hash equal; any change flips it.
pub fn canonical_hash(m: &dyn MeshRead) -> u64 {
    let mut h = Fnv(0xcbf2_9ce4_8422_2325);
    h.u64(m.mesh_id().to_raw());
    for dim in Dim::ALL {
        h.u64(m.len(dim) as u64);
        for ix in 0..m.len(dim) as u32 {
            h.u64(m.id_of(dim, ix).map_or(0, |i| i.to_raw()));
            h.u64(m.classification(dim, ix).map_or(0, |c| c.entity.to_raw()));
        }
    }
    let c = m.coords();
    for i in 0..m.len(Dim::Vertex) {
        h.u64(c.x[i].to_bits());
        h.u64(c.y[i].to_bits());
        h.u64(c.z[i].to_bits());
    }
    for f in 0..m.len(Dim::Face) as u32 {
        let fi = FaceIx(f);
        for x in m.face_vertices(fi) {
            h.u64(u64::from(x.0));
        }
        h.u64(u64::from(m.face_left(fi).0));
        h.u64(m.face_right(fi).map_or(u64::MAX, |r| u64::from(r.0)));
    }
    for cix in 0..m.len(Dim::Cell) as u32 {
        let ci = CellIx(cix);
        h.u64(m.cell_shape(ci) as u64);
        let spec = m.cell_spec(ci);
        h.u64(u64::from(spec.degree));
        h.u64(spec.basis as u64);
        for sf in m.cell_faces(ci) {
            h.u64(u64::from(sf.face().0) * 2 + u64::from(sf.sign() == Sign::Minus));
        }
        for n in m.cell_nodes(ci) {
            h.u64(u64::from(n.0));
        }
    }
    for def in &m.schema().tags {
        h.bytes(def.name.as_bytes());
        if let Some(col) = m.tag(def.dim, &def.name) {
            match &col.data {
                TagData::I64(d) => d.iter().for_each(|x| h.u64(*x as u64)),
                TagData::F64(d) => d.iter().for_each(|x| h.u64(x.to_bits())),
                TagData::Id(d) => d.iter().for_each(|x| h.u64(*x)),
            }
        }
    }
    for sd in &m.schema().sets {
        h.bytes(sd.name.as_bytes());
        if let Some(s) = m.set(&sd.name) {
            s.members().iter().for_each(|x| h.u64(u64::from(*x)));
        }
    }
    h.0
}
