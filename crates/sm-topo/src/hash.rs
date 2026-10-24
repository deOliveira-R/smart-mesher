//! The canonical word stream, emitted without per-word dynamic dispatch.
//!
//! [`TopologyView::canonical_visit`] has a provided implementation that goes through the trait
//! for every cell, row and pcurve. This is the same traversal specialised to the store's
//! columns: it emits **exactly the same words in the same order** (a test compares the streams
//! of both implementations on every state it builds), but reads the columns by slot, finds the
//! shell rings of long volume rows by binary search, and is generic over the sink, so the FNV
//! fold of `canonical_hash` is inlined.

use crate::store::TopoStore;
use sm_core::EntityKind;
use sm_geom_api::{ModelTolerance, ShapeValue};
use sm_topo_api::{Carrier, Dim, HashWord, Incidence, LineageKind, TopologyView};

/// FNV-1a over little-endian `u64` words.
const FNV_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

#[inline(always)]
fn fnv(h: &mut u64, v: u64) {
    for b in v.to_le_bytes() {
        *h ^= u64::from(b);
        *h = h.wrapping_mul(FNV_PRIME);
    }
}

#[inline(always)]
fn int<F: FnMut(HashWord)>(f: &mut F, v: u64) {
    f(HashWord::Int(v));
}

#[inline(always)]
fn len<F: FnMut(HashWord)>(f: &mut F, tol: &ModelTolerance, v: f64) {
    f(HashWord::Float { raw: v, q: tol.quantize(v) });
}

/// The words of one shape definition: integers as is, floats quantized relative to the
/// definition's largest scale-free magnitude.
fn shape_words<F: FnMut(HashWord)>(tol: &ModelTolerance, visit: impl Fn(&mut dyn FnMut(ShapeValue)), f: &mut F) {
    let mut scale = 0.0f64;
    visit(&mut |v| {
        if let ShapeValue::Unitless(x) = v
            && x.is_finite()
        {
            scale = scale.max(x.abs());
        }
    });
    visit(&mut |v| match v {
        ShapeValue::Int(n) => f(HashWord::Int(n)),
        ShapeValue::Length(x) | ShapeValue::Angle(x) | ShapeValue::Unitless(x) | ShapeValue::Param(x) => f(HashWord::Float { raw: x, q: tol.quantize_shape_in(v, scale) }),
        other => f(HashWord::Int(tol.quantize_shape_in(other, scale) as u64)),
    });
}

fn carrier_words<F: FnMut(HashWord)>(tol: &ModelTolerance, c: &Carrier, f: &mut F) {
    match c {
        Carrier::None => int(f, 0),
        Carrier::Point(p) => {
            int(f, 1);
            for x in [p.x, p.y, p.z] {
                len(f, tol, x);
            }
        }
        Carrier::Curve(c) => {
            int(f, 2);
            int(f, c.curve.to_raw());
            for x in [c.range.lo, c.range.hi] {
                f(HashWord::Float { raw: x, q: tol.quantize_param(x) });
            }
            int(f, c.sense as u64);
        }
        Carrier::Surface(s) => {
            int(f, 3);
            int(f, s.surface.to_raw());
            int(f, s.sense as u64);
        }
        Carrier::Csg(cell) => {
            int(f, 4);
            int(f, cell.halfspaces.len() as u64);
            for hs in &cell.halfspaces {
                int(f, hs.surface.to_raw());
                int(f, hs.sign as u64);
            }
        }
        Carrier::Composite(m) => {
            int(f, 5);
            int(f, m.len() as u64);
            m.iter().for_each(|x| int(f, x.to_raw()));
        }
        // A later carrier kind: its discriminant stands in until it gets words of its own.
        _ => int(f, u64::MAX),
    }
}

impl TopoStore {
    /// Emit the canonical words (see [`TopologyView::canonical_visit`]) into `f`.
    pub(crate) fn visit_words<F: FnMut(HashWord)>(&self, f: &mut F) {
        let tol = self.tol;
        for v in [tol.linear, tol.angular, tol.parametric, tol.linear_max] {
            int(f, v.to_bits());
        }
        int(f, self.revision.0);
        let shapes = &self.shapes;
        let (nc, ns) = (sm_geom_api::GeomStore::curve_count(shapes), sm_geom_api::GeomStore::surface_count(shapes));
        int(f, nc as u64);
        int(f, ns as u64);
        for n in 0..nc {
            match sm_core::EntityId::new(EntityKind::Curve, n as u64).and_then(|id| sm_geom_api::GeomStore::curve_def(shapes, id)) {
                None => int(f, u64::MAX),
                Some(d) => shape_words(&tol, |g| d.for_each_value(g), f),
            }
        }
        for n in 0..ns {
            match sm_core::EntityId::new(EntityKind::Surface, n as u64).and_then(|id| sm_geom_api::GeomStore::surface_def(shapes, id)) {
                None => int(f, u64::MAX),
                Some(d) => shape_words(&tol, |g| d.for_each_value(g), f),
            }
        }
        for dim in Dim::ALL {
            let t = &self.t[dim.index()];
            let cells = self.cells(dim);
            int(f, cells.len() as u64);
            for &id in cells {
                let slot = (id.serial() - 1) as usize;
                int(f, id.to_raw());
                int(f, t.born[slot].0);
                int(f, t.flags[slot].bits());
                let ov = t.tol[slot];
                if ov.is_nan() {
                    int(f, i64::MIN as u64);
                } else {
                    len(f, &tol, ov);
                }
                carrier_words(&tol, &t.carrier[slot], f);
                let row = t.down.row(slot);
                int(f, row.len() as u64);
                for e in row {
                    int(f, e.cell.to_raw());
                    int(f, e.side as u64);
                    int(f, u64::from(e.ring));
                }
            }
        }
        // Pcurves, per coedge of every live face in loop order.
        let ft = &self.t[Dim::Face.index()];
        for &face in self.cells(Dim::Face) {
            let slot = (face.serial() - 1) as usize;
            let list = &self.pcurves[slot];
            for e in ft.down.row(slot) {
                if e.ring == Incidence::EMBEDDED {
                    continue;
                }
                let found = if list.is_empty() { None } else { list.binary_search_by(|(le, ls, _)| (*le, *ls).cmp(&(e.cell, e.side))).ok().map(|i| &list[i].2) };
                match found {
                    None => int(f, 0),
                    Some(pc) => {
                        int(f, pc.tag());
                        shape_words(&tol, |g| pc.for_each_word(g), f);
                    }
                }
            }
        }
        // Shell kinds: one word per shell of each volume, in ring order.
        let vt = &self.t[Dim::Volume.index()];
        for &v in self.cells(Dim::Volume) {
            let slot = (v.serial() - 1) as usize;
            let row = vt.down.row(slot);
            let mut pos = 0;
            while pos < row.len() {
                let ring = row[pos].ring;
                if ring != Incidence::EMBEDDED {
                    int(f, self.shells[slot].get(usize::from(ring)).copied().flatten().map_or(u64::MAX, |k| k as u64));
                }
                pos += row[pos..].partition_point(|e| e.ring == ring);
            }
        }
        int(f, self.pairs.len() as u64);
        for p in &self.pairs {
            int(f, p.a.to_raw());
            int(f, p.b.to_raw());
            int(f, p.status as u64);
            int(f, p.op.0);
        }
        let log = &self.lineage;
        int(f, log.len() as u64);
        for r in log.records() {
            int(f, r.op.0);
            int(f, r.entity.to_raw());
            match &r.kind {
                LineageKind::Created => int(f, 1),
                LineageKind::Split { children } => {
                    int(f, 2);
                    int(f, children.len() as u64);
                    children.iter().for_each(|c| int(f, c.to_raw()));
                }
                LineageKind::Merged { into, gap } => {
                    int(f, 3);
                    int(f, into.to_raw());
                    len(f, &tol, *gap);
                }
                LineageKind::Deleted => int(f, 4),
                LineageKind::Modified(m) => {
                    int(f, 5);
                    int(f, *m as u64);
                }
                LineageKind::Generated { from_surface } => {
                    int(f, 6);
                    int(f, from_surface.to_raw());
                }
                // A later lineage kind: hashed as an unknown record until it gets words of its own.
                _ => int(f, u64::MAX),
            }
        }
    }

    /// The FNV-1a fold of [`Self::visit_words`] (integers as is, floats by their quantized code).
    pub(crate) fn hash_words(&self) -> u64 {
        let mut h = FNV_OFFSET;
        self.visit_words(&mut |w| match w {
            HashWord::Int(v) => fnv(&mut h, v),
            HashWord::Float { q, .. } => fnv(&mut h, q as u64),
        });
        h
    }
}
