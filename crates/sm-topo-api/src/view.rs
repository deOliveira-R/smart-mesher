//! [`TopologyView`]: the read-only query contract for meshers, semantics and tracking.

use crate::derived::TopologyViewExt;
use crate::error::QueryError;
use crate::geometry::{self, FaceMoments, Moments, PointClass};
use crate::lineage::LineageLog;
use crate::types::{Carrier, CellFlags, CoedgeKey, Dim, EntityState, FacePair, Incidence, LineageKind, OpId, Sense, ShellKind};
use sm_core::EntityId;
use sm_geom_api::{GeomStore, ModelTolerance, PCurve, Uv};

/// The carrier returned for unknown cells.
pub static NO_CARRIER: Carrier = Carrier::None;

/// One use of an edge by a face loop, as listed in the radial ring of the edge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct EdgeUse {
    /// The face using the edge.
    pub face: EntityId,
    /// Coedge sense in that face (`Forward` = the loop runs along the edge, face on its left).
    pub sense: Sense,
}

/// Radial ring angle step: angles are rounded to `2^-20` rad before sorting, so coincident or
/// nearly coincident faces tie and break by `(face, sense)`.
pub const RADIAL_ANGLE_STEP: f64 = 1.0 / 1_048_576.0;

/// Read-only access to a cellular complex. Object-safe: consumers take `&dyn TopologyView`.
///
/// The model: cells of five ranks ([`Dim`]), one oriented incidence relation stored as boundary
/// rows (`boundary`) with its transpose (`coboundary`), carriers, tolerances, coincidence
/// records and lineage. Everything else (loops, shells, coedges, face uses, face sides) is a
/// *derived view*, see [`crate::TopologyViewExt`].
///
/// # Contracts
/// - All slices are in canonical order, so iteration is deterministic: `cells` and
///   `coboundary` ascending by id, `boundary` in the documented ring order.
/// - Unknown or dead ids give empty slices, `None` or defaults, never panics.
/// - Derived indexes (`radial_uses`, `face_ordinal`) describe the state at the last committed
///   operation; between `begin_op` and `end_op` of a [`crate::TopologyEdit`] they may lag.
pub trait TopologyView: Send + Sync {
    /// The global tolerance policy of the model.
    fn tolerance(&self) -> &ModelTolerance;

    /// The geometry store that resolves every carrier handle of this view.
    fn shapes(&self) -> &dyn GeomStore;

    /// Dimension of `id`, if the id is known to this view. Virtual overlay cells use the per-rank
    /// `Partition*` kinds.
    fn dim_of(&self, id: EntityId) -> Option<Dim>;

    /// Lifecycle state of `id`; retired ids keep resolving.
    fn state(&self, id: EntityId) -> Option<EntityState>;

    /// All live cells of a rank, ascending by id.
    fn cells(&self, dim: Dim) -> &[EntityId];

    /// Boundary row of `id`: its lower-dimensional cells, ordered by `ring` then position
    /// (face loops are cyclic coedge sequences, each rotated to start at its smallest
    /// `(edge id, side)`; volume/body rows are sorted by `(ring, id)`). Embedded members come
    /// last with [`Incidence::EMBEDDED`].
    fn boundary(&self, id: EntityId) -> &[Incidence];

    /// Coboundary row of `id`: its live parents ascending by parent id. `Incidence::cell` is the
    /// parent; `side` and `ring` are copied from the parent's row entry, so a face's coboundary
    /// entry `{volume, Left}` reads "that volume is on my left".
    fn coboundary(&self, id: EntityId) -> &[Incidence];

    /// Geometry carrier of `id` ([`Carrier::None`] if none).
    fn carrier(&self, id: EntityId) -> &Carrier;

    /// Flag bits of `id`.
    fn flags(&self, id: EntityId) -> CellFlags;

    /// The pcurve of a coedge, if recorded. It is parametrized by the same parameter as the
    /// edge's curve (valid over the edge's carrier range).
    fn pcurve(&self, k: CoedgeKey) -> Option<&PCurve>;

    /// Kind of shell `ring` of `volume`.
    fn shell_kind(&self, volume: EntityId, ring: u16) -> Option<ShellKind>;

    /// The face uses around an edge in canonical angular order: the ring starts at the use of the
    /// lowest `(face, sense)` and runs counter-clockwise about the edge direction, with angles
    /// rounded to [`RADIAL_ANGLE_STEP`] and ties broken by `(face, sense)`. Without carriers the
    /// order is `(face, sense)`. Degenerate (pole) edges have an empty ring. Derived from
    /// geometry, rebuilt per changed edge at `end_op`.
    fn radial_uses(&self, edge: EntityId) -> &[EdgeUse];

    /// Sparse per-entity tolerance override (linear, model length units), if any.
    fn tolerance_override(&self, id: EntityId) -> Option<f64>;

    /// All recorded face-pair relations, sorted by `(a, b)`.
    fn coincidences(&self) -> &[FacePair];

    /// The pair records involving face `f` (live or retired), sorted by `(a, b)`: imprint and
    /// merge status of each coincident neighbour.
    fn coincident_faces(&self, f: EntityId) -> &[FacePair];

    /// The append-only lineage log, indexed both ways.
    fn lineage(&self) -> &LineageLog;

    /// The operation that created `id`.
    fn born(&self, id: EntityId) -> Option<OpId>;

    /// The most recent operation applied (the `OpId` last passed to `begin_op`).
    fn revision(&self) -> OpId;

    /// Faces of a volume (all shells, baffles once), ascending, deduplicated. Unknown id: empty.
    fn volume_faces(&self, v: EntityId) -> &[EntityId];

    /// Distinct edges of the loops of a face, ascending. Unknown id: empty.
    fn face_edges(&self, f: EntityId) -> &[EntityId];

    /// Raw moments of a volume (about the origin). See [`crate::geometry`] for coverage.
    fn volume_moments(&self, v: EntityId) -> Result<Moments, QueryError> {
        geometry::volume_moments(self, v)
    }

    /// Integrated moments of a face (about the origin), oriented left to right.
    fn face_moments(&self, f: EntityId) -> Result<FaceMoments, QueryError> {
        geometry::face_moments(self, f)
    }

    /// Classify a point of the face's parameter space against the trimmed face.
    fn classify_point_on_face(&self, f: EntityId, uv: Uv) -> Result<PointClass, QueryError> {
        geometry::classify_point_on_face(self, f, uv)
    }

    /// True if `id` exists and is alive.
    fn is_alive(&self, id: EntityId) -> bool {
        self.state(id) == Some(EntityState::Live)
    }

    /// Effective linear tolerance of `id`: the override if present, else the global linear one.
    fn tolerance_of(&self, id: EntityId) -> f64 {
        self.tolerance_override(id).unwrap_or(self.tolerance().linear)
    }

    /// Dense ordinal of a live face within `cells(Dim::Face)` (`0..n`), by binary search. Valid
    /// within one revision: ordinals shift when faces are added or retired.
    fn face_ordinal(&self, f: EntityId) -> Option<usize> {
        self.cells(Dim::Face).binary_search(&f).ok()
    }

    /// Deterministic 64-bit content hash (FNV-1a) of the whole complex in canonical order.
    ///
    /// Floats never enter the hash as raw bits: lengths are quantized with
    /// `ModelTolerance::quantum() = linear * 2^-10` and parameters with
    /// `ModelTolerance::param_quantum()` (see [`ModelTolerance`] for the exact rule), so equal
    /// complexes hash equal on every platform. Covered: tolerance policy, revision, every live
    /// cell (id, birth op, flags, quantized tolerance override, carrier, boundary row), the
    /// pcurves by content, shell kinds, coincidence records and the full lineage log.
    fn canonical_hash(&self) -> u64 {
        let tol = *self.tolerance();
        let mut h = Fnv::new();
        for v in [tol.linear, tol.angular, tol.parametric, tol.linear_max] {
            h.u64(v.to_bits());
        }
        h.u64(self.revision().0);
        h.u64(self.shapes().curve_count() as u64);
        h.u64(self.shapes().surface_count() as u64);
        for dim in Dim::ALL {
            let cells = self.cells(dim);
            h.u64(cells.len() as u64);
            for &id in cells {
                h.u64(id.to_raw());
                h.u64(self.born(id).map_or(u64::MAX, |o| o.0));
                h.u64(u64::from(self.flags(id).0));
                h.i64(self.tolerance_override(id).map_or(i64::MIN, |t| tol.quantize(t)));
                hash_carrier(&mut h, &tol, self.carrier(id));
                let row = self.boundary(id);
                h.u64(row.len() as u64);
                for e in row {
                    h.u64(e.cell.to_raw());
                    h.u64(e.side as u64);
                    h.u64(u64::from(e.ring));
                }
            }
        }
        for &f in self.cells(Dim::Face) {
            for ce in self.coedges(f) {
                match self.pcurve(ce.key) {
                    None => h.u64(0),
                    Some(pc) => {
                        h.u64(pc.tag());
                        pc.for_each_value(|x| h.i64(tol.quantize_param(x)));
                    }
                }
            }
        }
        for &v in self.cells(Dim::Volume) {
            let mut last = u16::MAX;
            for e in self.boundary(v) {
                if e.ring != last && e.ring != Incidence::EMBEDDED {
                    last = e.ring;
                    h.u64(self.shell_kind(v, e.ring).map_or(u64::MAX, |k| k as u64));
                }
            }
        }
        h.u64(self.coincidences().len() as u64);
        for p in self.coincidences() {
            h.u64(p.a.to_raw());
            h.u64(p.b.to_raw());
            h.u64(p.status as u64);
            h.u64(p.op.0);
        }
        let log = self.lineage();
        h.u64(log.len() as u64);
        for r in log.records() {
            h.u64(r.op.0);
            h.u64(r.entity.to_raw());
            match &r.kind {
                LineageKind::Created => h.u64(1),
                LineageKind::Split { children } => {
                    h.u64(2);
                    h.u64(children.len() as u64);
                    children.iter().for_each(|c| h.u64(c.to_raw()));
                }
                LineageKind::Merged { into, gap } => {
                    h.u64(3);
                    h.u64(into.to_raw());
                    h.i64(tol.quantize(*gap));
                }
                LineageKind::Deleted => h.u64(4),
                LineageKind::Modified(m) => {
                    h.u64(5);
                    h.u64(*m as u64);
                }
                LineageKind::Generated { from_surface } => {
                    h.u64(6);
                    h.u64(from_surface.to_raw());
                }
            }
        }
        h.finish()
    }
}

fn hash_carrier(h: &mut Fnv, tol: &ModelTolerance, c: &Carrier) {
    match c {
        Carrier::None => h.u64(0),
        Carrier::Point(p) => {
            h.u64(1);
            h.i64(tol.quantize(p.x));
            h.i64(tol.quantize(p.y));
            h.i64(tol.quantize(p.z));
        }
        Carrier::Curve(c) => {
            h.u64(2);
            h.u64(c.curve.to_raw());
            h.i64(tol.quantize_param(c.range.lo));
            h.i64(tol.quantize_param(c.range.hi));
            h.u64(c.sense as u64);
        }
        Carrier::Surface(s) => {
            h.u64(3);
            h.u64(s.surface.to_raw());
            h.u64(s.sense as u64);
        }
        Carrier::Csg(cell) => {
            h.u64(4);
            h.u64(cell.halfspaces.len() as u64);
            for hs in &cell.halfspaces {
                h.u64(hs.surface.to_raw());
                h.u64(hs.sign as u64);
            }
        }
        Carrier::Composite(m) => {
            h.u64(5);
            h.u64(m.len() as u64);
            m.iter().for_each(|x| h.u64(x.to_raw()));
        }
    }
}

/// FNV-1a over little-endian `u64` words.
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
    pub(crate) fn i64(&mut self, v: i64) {
        self.u64(v as u64);
    }
    pub(crate) fn finish(&self) -> u64 {
        self.0
    }
}
