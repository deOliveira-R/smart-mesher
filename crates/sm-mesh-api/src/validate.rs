//! Implementation-independent validation of the contract's invariants, plus the canonical
//! content hash.

use crate::metrics::MeshMetrics;
use crate::point::{Orient, Point, Stratum};
use crate::shape::CellShape;
use crate::traits::{MeshView, model_dim};
use std::collections::BTreeMap;

/// One violated invariant.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum Violation {
    /// Face loop shorter than 3 or repeating a vertex.
    DegenerateFace(Point),
    /// `left` is not strictly lower than `right` (by `EntityId`), or `left` is missing.
    SideOrder(Point),
    /// A cell's cone sign disagrees with the face's sides.
    SignMismatch {
        /// The cell.
        cell: Point,
        /// The face.
        face: Point,
    },
    /// Directed face-loop edges of a cell do not pair up (cell not closed).
    CellNotClosed(Point),
    /// Non-positive cell volume under the geometry rule.
    NonPositiveVolume(Point),
    /// Shape tag disagrees with topology.
    ShapeTag(Point),
    /// Classifying entity is not a model entity, or its dimension is too low for the stratum.
    BadClassification(Point),
    /// A lower point is classified on a higher-dimensional entity than a point it bounds.
    ClassificationOrder {
        /// The bounded (lower) point.
        lower: Point,
        /// The bounding (higher) point.
        upper: Point,
    },
    /// A label entry refers to a point that does not exist.
    DanglingLabel(String, Point),
}

/// Check every invariant of the contract; an empty vector means the mesh is valid.
pub fn validate<M: MeshView + ?Sized>(m: &M) -> Vec<Violation> {
    let mut out = Vec::new();
    for f in Point::range(Stratum::Face, m.count(Stratum::Face)) {
        let loop_ = m.cone(f).indices;
        let mut s = loop_.to_vec();
        s.sort_unstable();
        s.dedup();
        if loop_.len() < 3 || s.len() != loop_.len() {
            out.push(Violation::DegenerateFace(f));
        }
        let sides = m.face_sides(f);
        let ok = match sides.right {
            Some(r) => m.entity_id(sides.left) < m.entity_id(r),
            None => true,
        };
        if !ok || !m.contains(sides.left) {
            out.push(Violation::SideOrder(f));
        }
    }
    for c in Point::range(Stratum::Cell, m.count(Stratum::Cell)) {
        let cone = m.cone(c);
        let mut directed: BTreeMap<(u32, u32), i32> = BTreeMap::new();
        let mut sizes = Vec::new();
        for (f, o) in cone.iter() {
            let sides = m.face_sides(f);
            let expect = if sides.left == c {
                Some(Orient::Fwd)
            } else if sides.right == Some(c) {
                Some(Orient::Rev)
            } else {
                None
            };
            if expect != Some(o) {
                out.push(Violation::SignMismatch { cell: c, face: f });
            }
            let l = m.outward_loop(f, o);
            sizes.push(l.len());
            for i in 0..l.len() {
                *directed.entry((l[i], l[(i + 1) % l.len()])).or_insert(0) += 1;
            }
        }
        let closed = directed.iter().all(|(&(a, b), &n)| n == 1 && directed.get(&(b, a)) == Some(&1));
        if !closed {
            out.push(Violation::CellNotClosed(c));
        }
        let tag = m.cell_shape(c);
        if tag != CellShape::Polyhedron && CellShape::detect(m.closure_vertices(c).len(), &sizes) != tag {
            out.push(Violation::ShapeTag(c));
        }
        if closed && m.cell_volume(c) <= 0.0 {
            out.push(Violation::NonPositiveVolume(c));
        }
    }
    // Classification: valid dims, and dim(lower) <= dim(upper) across incidences.
    let dim_of = |p: Point| m.classification(p).and_then(|c| model_dim(c.entity));
    for s in Stratum::ALL {
        for p in Point::range(s, m.count(s)) {
            if let Some(c) = m.classification(p) {
                match model_dim(c.entity) {
                    Some(d) if d >= s.dim() => {}
                    _ => out.push(Violation::BadClassification(p)),
                }
            }
            if matches!(s, Stratum::Face | Stratum::Cell) {
                let Some(du) = dim_of(p) else { continue };
                let lower: Vec<Point> = m.cone(p).iter().map(|x| x.0).collect();
                for q in lower {
                    if dim_of(q).is_some_and(|dl| dl > du) {
                        out.push(Violation::ClassificationOrder { lower: q, upper: p });
                    }
                }
            }
        }
    }
    for name in m.label_names() {
        for (p, _) in m.label_entries(&name) {
            if !m.contains(p) {
                out.push(Violation::DanglingLabel(name.clone(), p));
            }
        }
    }
    out
}

struct Fnv(u64);
impl Fnv {
    fn put(&mut self, x: u64) {
        for b in x.to_le_bytes() {
            self.0 = (self.0 ^ u64::from(b)).wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
}

/// Canonical content hash (FNV-1a over counts, ids, loops, signs, shapes, coordinate bits,
/// classification and labels, all in canonical order). Equal meshes hash equal regardless of
/// how they were built, on any thread count.
pub fn content_hash<M: MeshView + ?Sized>(m: &M) -> u64 {
    let mut h = Fnv(0xcbf2_9ce4_8422_2325);
    h.put(m.mesh_id().to_raw());
    for s in Stratum::ALL {
        h.put(u64::from(m.count(s)));
        for p in Point::range(s, m.count(s)) {
            h.put(m.entity_id(p).to_raw());
            if matches!(s, Stratum::Face | Stratum::Cell) {
                let c = m.cone(p);
                h.put(c.len() as u64);
                for (q, o) in c.iter() {
                    h.put(u64::from(q.index));
                    h.put(o.sign().to_bits());
                }
            }
            if let Some(c) = m.classification(p) {
                h.put(c.entity.to_raw());
                match c.param {
                    crate::traits::Param::None => h.put(0),
                    crate::traits::Param::Curve(t) => h.put(t.to_bits()),
                    crate::traits::Param::Surface([u, v]) => {
                        h.put(u.to_bits());
                        h.put(v.to_bits());
                    }
                }
            }
        }
    }
    for c in Point::range(Stratum::Cell, m.count(Stratum::Cell)) {
        h.put(m.cell_shape(c) as u64);
        h.put(u64::from(m.cell_geometry(c).degree));
    }
    for a in m.coords_soa() {
        for x in a {
            h.put(x.to_bits());
        }
    }
    for name in m.label_names() {
        for b in name.bytes() {
            h.put(u64::from(b));
        }
        for (p, v) in m.label_entries(&name) {
            h.put(u64::from(p.index));
            h.put(p.stratum as u64);
            h.put(i64::from(v) as u64);
        }
    }
    h.0
}
