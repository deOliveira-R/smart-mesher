//! [`TopologyView`]: the read-only query contract for meshers, semantics and tracking.

use crate::derived::TopologyViewExt;
use crate::error::QueryError;
use crate::geometry::{self, FaceMoments, Moments, PointClass};
use crate::lineage::LineageLog;
use crate::types::{Carrier, CellFlags, CoedgeKey, Dim, EntityState, FacePair, Incidence, LineageKind, OpId, Sense, ShellKind};
use sm_core::{EntityId, EntityKind};
use sm_geom_api::{GeomStore, ModelTolerance, PCurve, ShapeValue, Uv};

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

    /// The live cells covering `id`, ascending and deduplicated: `vec![id]` if it is alive,
    /// else [`TopologyViewExt::live_descendants`] (what it became through splits and merges).
    /// [`crate::Overlay`] overrides it so a hidden host cell maps to every live overlay cell
    /// standing for it (partition children, composites), recursively. Group and color
    /// expansion in `sm-model-api` maps members through this.
    fn live_cover(&self, id: EntityId) -> Vec<EntityId> {
        if self.is_alive(id) { vec![id] } else { self.live_descendants(id) }
    }

    /// Visit, in canonical order, exactly the words [`canonical_hash`](Self::canonical_hash)
    /// covers: integers as [`HashWord::Int`] and every geometric float as [`HashWord::Float`]
    /// with its raw value and its quantized code (the [`ModelTolerance`] rule for its class).
    /// The tolerance policy fields are user constants and are emitted by bit pattern as `Int`.
    /// A state layer folds this into an `sm-op-api` `Digest` (`Float` via `f64_with(raw, q)`), so
    /// its identity hash is quantized and its exact (cache-key) hash is bit-for-bit.
    fn canonical_visit(&self, f: &mut dyn FnMut(HashWord)) {
        let tol = *self.tolerance();
        let int = |f: &mut dyn FnMut(HashWord), v: u64| f(HashWord::Int(v));
        let len = |f: &mut dyn FnMut(HashWord), v: f64| f(HashWord::Float { raw: v, q: tol.quantize(v) });
        // Frozen: a ModelTolerance field added later is emitted only when it differs from its
        // default, so existing hashes stay stable (sm-geom-api DESIGN.md, W0g freeze rules).
        for v in [tol.linear, tol.angular, tol.parametric, tol.linear_max] {
            int(f, v.to_bits());
        }
        int(f, self.revision().0);
        let shapes = self.shapes();
        int(f, shapes.curve_count() as u64);
        int(f, shapes.surface_count() as u64);
        // Geometry definitions, per handle in serial order.
        for n in 0..shapes.curve_count() {
            match EntityId::new(EntityKind::Curve, n as u64).and_then(|id| shapes.curve_def(id)) {
                None => int(f, u64::MAX),
                Some(d) => shape_words(&tol, |g| d.for_each_value(g), f),
            }
        }
        for n in 0..shapes.surface_count() {
            match EntityId::new(EntityKind::Surface, n as u64).and_then(|id| shapes.surface_def(id)) {
                None => int(f, u64::MAX),
                Some(d) => shape_words(&tol, |g| d.for_each_value(g), f),
            }
        }
        for dim in Dim::ALL {
            let cells = self.cells(dim);
            int(f, cells.len() as u64);
            for &id in cells {
                int(f, id.to_raw());
                int(f, self.born(id).map_or(u64::MAX, |o| o.0));
                int(f, self.flags(id).bits());
                match self.tolerance_override(id) {
                    None => int(f, i64::MIN as u64),
                    Some(t) => len(f, t),
                }
                carrier_words(&tol, self.carrier(id), f);
                let row = self.boundary(id);
                int(f, row.len() as u64);
                for e in row {
                    int(f, e.cell.to_raw());
                    int(f, e.side as u64);
                    int(f, u64::from(e.ring));
                }
            }
        }
        for &face in self.cells(Dim::Face) {
            for ce in self.coedges(face) {
                match self.pcurve(ce.key) {
                    None => int(f, 0),
                    Some(pc) => {
                        int(f, pc.tag());
                        shape_words(&tol, |g| pc.for_each_word(g), f);
                    }
                }
            }
        }
        for &v in self.cells(Dim::Volume) {
            let mut last = u16::MAX;
            for e in self.boundary(v) {
                if e.ring != last && e.ring != Incidence::EMBEDDED {
                    last = e.ring;
                    int(f, self.shell_kind(v, e.ring).map_or(u64::MAX, |k| k as u64));
                }
            }
        }
        int(f, self.coincidences().len() as u64);
        for p in self.coincidences() {
            int(f, p.a.to_raw());
            int(f, p.b.to_raw());
            int(f, p.status as u64);
            int(f, p.op.0);
        }
        let log = self.lineage();
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
                    len(f, *gap);
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
            }
        }
    }

    /// Deterministic 64-bit content hash (FNV-1a) of the whole complex in canonical order: the
    /// fold of [`canonical_visit`](Self::canonical_visit), integers as is and floats by their
    /// quantized code.
    ///
    /// Floats never enter the hash as raw bits: lengths are quantized with
    /// `ModelTolerance::quantum() = linear * 2^-10` and parameters with
    /// `ModelTolerance::param_quantum()` (see [`ModelTolerance`] for the exact rule), so equal
    /// complexes hash equal on every platform. Covered: tolerance policy, revision, every curve and
    /// surface definition by value (lengths, angles and unitless values each quantized per
    /// [`ModelTolerance`]; a unitless value below one relative step of the largest unitless value
    /// of its definition hashes as zero, see `CurveDef::quantized_values`), every live
    /// cell (id, birth op, flags, quantized tolerance override, carrier, boundary row), the
    /// pcurves by content, shell kinds, coincidence records and the full lineage log.
    fn canonical_hash(&self) -> u64 {
        let mut h = Fnv::new();
        self.canonical_visit(&mut |w| match w {
            HashWord::Int(v) => h.u64(v),
            HashWord::Float { q, .. } => h.i64(q),
        });
        h.finish()
    }
}

/// One word of [`TopologyView::canonical_visit`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HashWord {
    /// An integer (id, count, tag, flag bits, a tolerance policy field's bit pattern).
    Int(u64),
    /// A geometric float: `raw` is the stored value, `q` its quantized code per the
    /// [`ModelTolerance`] rule for its class (length, parameter, angle, scale-free).
    Float {
        /// The value as stored.
        raw: f64,
        /// Its quantized code.
        q: i64,
    },
}

/// The words of one shape definition, matching `CurveDef::quantized_values`: integers as is,
/// floats with `quantize_shape_in` relative to the definition's largest scale-free magnitude.
fn shape_words(tol: &ModelTolerance, visit: impl Fn(&mut dyn FnMut(ShapeValue)), f: &mut dyn FnMut(HashWord)) {
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
        // A later value class: hashed by its code until it gets its own arm.
        other => f(HashWord::Int(tol.quantize_shape_in(other, scale) as u64)),
    });
}

fn carrier_words(tol: &ModelTolerance, c: &Carrier, f: &mut dyn FnMut(HashWord)) {
    let i = HashWord::Int;
    match c {
        Carrier::None => f(i(0)),
        Carrier::Point(p) => {
            f(i(1));
            for x in [p.x, p.y, p.z] {
                f(HashWord::Float { raw: x, q: tol.quantize(x) });
            }
        }
        Carrier::Curve(c) => {
            f(i(2));
            f(i(c.curve.to_raw()));
            for x in [c.range.lo, c.range.hi] {
                f(HashWord::Float { raw: x, q: tol.quantize_param(x) });
            }
            f(i(c.sense as u64));
        }
        Carrier::Surface(s) => {
            f(i(3));
            f(i(s.surface.to_raw()));
            f(i(s.sense as u64));
        }
        Carrier::Csg(cell) => {
            f(i(4));
            f(i(cell.halfspaces.len() as u64));
            for hs in &cell.halfspaces {
                f(i(hs.surface.to_raw()));
                f(i(hs.sign as u64));
            }
        }
        Carrier::Composite(m) => {
            f(i(5));
            f(i(m.len() as u64));
            m.iter().for_each(|x| f(i(x.to_raw())));
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fixtures;
    use crate::CellComplex;
    use sm_core::Point3;

    fn legacy_carrier(h: &mut Fnv, tol: &ModelTolerance, c: &Carrier) {
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

    fn legacy_hash(t: &dyn TopologyView) -> u64 {
        let this = t;
        let tol = *this.tolerance();
        let mut h = Fnv::new();
        for v in [tol.linear, tol.angular, tol.parametric, tol.linear_max] {
            h.u64(v.to_bits());
        }
        h.u64(this.revision().0);
        h.u64(this.shapes().curve_count() as u64);
        h.u64(this.shapes().surface_count() as u64);
        // Geometry definitions, per handle in serial order.
        let shapes = this.shapes();
        for n in 0..shapes.curve_count() {
            let def = EntityId::new(EntityKind::Curve, n as u64).and_then(|id| shapes.curve_def(id));
            match def {
                None => h.u64(u64::MAX),
                Some(d) => d.quantized_values(&tol, |c| h.i64(c)),
            }
        }
        for n in 0..shapes.surface_count() {
            let def = EntityId::new(EntityKind::Surface, n as u64).and_then(|id| shapes.surface_def(id));
            match def {
                None => h.u64(u64::MAX),
                Some(d) => d.quantized_values(&tol, |c| h.i64(c)),
            }
        }
        for dim in Dim::ALL {
            let cells = this.cells(dim);
            h.u64(cells.len() as u64);
            for &id in cells {
                h.u64(id.to_raw());
                h.u64(this.born(id).map_or(u64::MAX, |o| o.0));
                h.u64(this.flags(id).bits());
                h.i64(this.tolerance_override(id).map_or(i64::MIN, |t| tol.quantize(t)));
                legacy_carrier(&mut h, &tol, this.carrier(id));
                let row = this.boundary(id);
                h.u64(row.len() as u64);
                for e in row {
                    h.u64(e.cell.to_raw());
                    h.u64(e.side as u64);
                    h.u64(u64::from(e.ring));
                }
            }
        }
        for &f in this.cells(Dim::Face) {
            for ce in this.coedges(f) {
                match this.pcurve(ce.key) {
                    None => h.u64(0),
                    Some(pc) => {
                        h.u64(pc.tag());
                        pc.quantized_values(&tol, |c| h.i64(c));
                    }
                }
            }
        }
        for &v in this.cells(Dim::Volume) {
            let mut last = u16::MAX;
            for e in this.boundary(v) {
                if e.ring != last && e.ring != Incidence::EMBEDDED {
                    last = e.ring;
                    h.u64(this.shell_kind(v, e.ring).map_or(u64::MAX, |k| k as u64));
                }
            }
        }
        h.u64(this.coincidences().len() as u64);
        for p in this.coincidences() {
            h.u64(p.a.to_raw());
            h.u64(p.b.to_raw());
            h.u64(p.status as u64);
            h.u64(p.op.0);
        }
        let log = this.lineage();
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

    fn sample() -> CellComplex {
        use crate::fixtures::with_op;
        use crate::types::OpId;
        let mut t = CellComplex::new();
        let (a, b) = with_op(&mut t, OpId(1), fixtures::two_bricks).unwrap();
        with_op(&mut t, OpId(2), |t| fixtures::merge_touching_bricks_gap(t, &a, &b, Some(1e-4))).unwrap();
        let c = with_op(&mut t, OpId(3), |t| fixtures::brick(t, Point3::new(5.0, 0.0, 0.0), [1.0, 2.0, 3.0])).unwrap();
        with_op(&mut t, OpId(4), |t| fixtures::webcut_brick(t, &c, 0.37)).unwrap();
        with_op(&mut t, OpId(5), |t| fixtures::cylinder(t, 0.7, 2.0)).unwrap();
        with_op(&mut t, OpId(6), |t| fixtures::sphere(t, 1.3)).unwrap();
        t
    }

    #[test]
    fn canonical_hash_is_unchanged_by_the_visit_refactor() {
        let t = sample();
        assert_eq!(t.canonical_hash(), legacy_hash(&t));
        let e = CellComplex::new();
        assert_eq!(e.canonical_hash(), legacy_hash(&e));
    }

    #[test]
    fn canonical_hash_agrees_with_legacy_for_polyline_and_nurbs_pcurves() {
        use crate::TopologyEdit;
        use sm_geom_api::NurbsCurve2;
        let mut t = sample();
        let keys: Vec<CoedgeKey> = t.cells(Dim::Face).to_vec().into_iter().flat_map(|f| t.coedges(f).map(|c| c.key).collect::<Vec<_>>()).collect();
        assert!(keys.len() >= 2);
        let poly = PCurve::Polyline2(vec![Uv::new(0.0, 0.0), Uv::new(2.0, 0.5), Uv::new(2.0, 4.0)]);
        let ctrl = vec![Uv::new(0.0, 0.0), Uv::new(1.0, 2.0), Uv::new(3.0, 2.5), Uv::new(4.0, 0.0)];
        let nurbs = NurbsCurve2::new(2, vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0], ctrl, Some(vec![1.0, 2.0, 0.5, 1.0])).unwrap();
        crate::fixtures::with_op(&mut t, OpId(7), |t| {
            t.set_pcurve(keys[0], Some(poly))?;
            t.set_pcurve(keys[1], Some(PCurve::Nurbs2(nurbs)))?;
            Ok(())
        })
        .unwrap();
        assert_eq!(t.canonical_hash(), legacy_hash(&t));
    }

    #[test]
    fn canonical_visit_floats_carry_raw_values_and_codes() {
        let t = sample();
        let tol = *t.tolerance();
        let (mut ints, mut floats) = (0usize, 0usize);
        t.canonical_visit(&mut |w| match w {
            HashWord::Int(_) => ints += 1,
            HashWord::Float { raw, q } => {
                floats += 1;
                assert!(raw.is_finite());
                // Every code is within one step of some quantum of the raw value.
                let ok = [tol.quantum(), tol.param_quantum(), tol.angular_quantum()]
                    .iter()
                    .any(|s| ModelTolerance::quantize_with(raw, *s) == q)
                    || q == tol.quantize_scale_free(raw)
                    || q == 0;
                assert!(ok, "{raw} -> {q}");
            }
        });
        assert!(ints > 100 && floats > 100, "{ints} {floats}");
    }
}
