//! [`TopologyView`]: the read-only query contract for meshers, semantics and tracking.

use crate::types::{Carrier, CellFlags, Dim, FacePair, Incidence, LineageRecord, OpId};
use sm_core::EntityId;
use sm_geom_api::Tolerance;

/// Read-only access to a cellular complex. Object-safe: consumers take `&dyn TopologyView`.
///
/// The model: cells of five ranks ([`Dim`]), one oriented incidence relation stored as boundary
/// rows (`boundary`) with its transpose (`coboundary`), carriers, tolerances, coincidence
/// records and lineage. Everything else (loops, shells, coedges, face uses, face sides) is a
/// *derived view*, see [`crate::TopologyViewExt`].
///
/// All slices are in canonical order, so iteration is deterministic. Unknown or dead ids give
/// empty slices / `None` / defaults rather than panics.
pub trait TopologyView: Send + Sync {
    /// The global tolerance policy of the model.
    fn model_tolerance(&self) -> sm_geom_api::Tolerance;

    /// Dimension of `id`, if the id is known to this view. (Virtual overlay cells share the
    /// `PartitionCell` kind, so the dimension is not recoverable from the kind alone.)
    fn dim_of(&self, id: EntityId) -> Option<Dim>;

    /// True if `id` exists and has not been deleted, split or merged away.
    fn is_alive(&self, id: EntityId) -> bool;

    /// All live cells of a rank, ascending by id.
    fn cells(&self, dim: Dim) -> &[EntityId];

    /// Boundary row of `id`: its lower-dimensional cells, ordered by `ring` then position
    /// (face loops are cyclic coedge sequences, each rotated to start at its smallest
    /// `(edge id, side)`; volume/body rows are sorted by id).
    fn boundary(&self, id: EntityId) -> &[Incidence];

    /// Coboundary row of `id`: its live parents ascending by parent id. `Incidence::cell` is the
    /// parent; `side` and `ring` are copied from the parent's row entry, so a face's coboundary
    /// entry `{volume, Left}` reads "that volume is on my left".
    fn coboundary(&self, id: EntityId) -> &[Incidence];

    /// Geometry carrier of `id` ([`Carrier::None`] if none).
    fn carrier(&self, id: EntityId) -> Carrier;

    /// Flag bits of `id`.
    fn flags(&self, id: EntityId) -> CellFlags;

    /// Sparse per-entity tolerance override (linear, model length units), if any.
    fn tolerance_override(&self, id: EntityId) -> Option<f64>;

    /// Effective linear tolerance of `id`: the override if present, else the global linear one.
    fn tolerance(&self, id: EntityId) -> f64 {
        self.tolerance_override(id).unwrap_or_else(|| self.model_tolerance().linear)
    }

    /// All recorded face-pair relations, sorted by `(a, b)`.
    fn coincidences(&self) -> &[FacePair];

    /// The full append-only lineage log in operation order.
    fn lineage(&self) -> &[LineageRecord];

    /// The operation that created `id`.
    fn born(&self, id: EntityId) -> Option<OpId>;

    /// The most recent operation applied (the `OpId` last passed to `begin_op`).
    fn revision(&self) -> OpId;

    /// Deterministic 64-bit content hash (FNV-1a) of everything above, in canonical order.
    /// Equal complexes hash equal on every platform; used for golden tests and cache keys.
    fn canonical_hash(&self) -> u64 {
        let mut h = Fnv::new();
        let tol: Tolerance = self.model_tolerance();
        h.f64(tol.linear);
        h.f64(tol.angular);
        h.f64(tol.parametric);
        h.u64(self.revision().0);
        for dim in Dim::ALL {
            let cells = self.cells(dim);
            h.u64(cells.len() as u64);
            for &id in cells {
                h.u64(id.to_raw());
                h.u64(self.born(id).map_or(u64::MAX, |o| o.0));
                h.u64(u64::from(self.flags(id).0));
                h.f64(self.tolerance_override(id).unwrap_or(f64::NAN));
                hash_carrier(&mut h, self.carrier(id));
                let row = self.boundary(id);
                h.u64(row.len() as u64);
                for e in row {
                    h.u64(e.cell.to_raw());
                    h.u64(e.side as u64);
                    h.u64(u64::from(e.ring));
                }
            }
        }
        h.u64(self.coincidences().len() as u64);
        for p in self.coincidences() {
            h.u64(p.a.to_raw());
            h.u64(p.b.to_raw());
            h.u64(p.status as u64);
            h.f64(p.overlap_area.unwrap_or(f64::NAN));
        }
        h.u64(self.lineage().len() as u64);
        for r in self.lineage() {
            h.u64(r.op.0);
            h.u64(r.kind as u64);
            for id in r.from.iter().chain(r.to.iter()) {
                h.u64(id.to_raw());
            }
            h.u64(r.from.len() as u64);
            h.u64(r.to.len() as u64);
        }
        h.finish()
    }
}

fn hash_carrier(h: &mut Fnv, c: Carrier) {
    match c {
        Carrier::None => h.u64(0),
        Carrier::Point(p) => {
            h.u64(1);
            h.f64(p.x);
            h.f64(p.y);
            h.f64(p.z);
        }
        Carrier::Curve(c) => {
            h.u64(2);
            h.u64(c.curve.to_raw());
            h.f64(c.range.lo);
            h.f64(c.range.hi);
            h.u64(c.sense as u64);
        }
        Carrier::Surface(s) => {
            h.u64(3);
            h.u64(s.surface.to_raw());
            h.u64(s.sense as u64);
        }
    }
}

/// FNV-1a over little-endian `u64` words; floats hash by bit pattern (NaN canonicalized).
pub(crate) struct Fnv(u64);

impl Fnv {
    pub(crate) fn new() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
    pub(crate) fn u64(&mut self, v: u64) {
        for b in v.to_le_bytes() {
            self.0 ^= u64::from(b);
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    pub(crate) fn f64(&mut self, v: f64) {
        self.u64(if v.is_nan() { 0x7ff8_0000_0000_0000 } else if v == 0.0 { 0 } else { v.to_bits() });
    }
    pub(crate) fn finish(&self) -> u64 {
        self.0
    }
}
