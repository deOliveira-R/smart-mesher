//! The decomposition layer: virtual composites and partitions layered over a host complex
//! without changing a single host entity (Cubit's virtual topology; docs/design/ir.md §5).
//!
//! An [`Overlay`] **is** a [`TopologyView`]: meshers, semantics and tracking consume it exactly
//! like the host. Virtual cells use the per-rank kinds `PartitionVertex/Edge/Face`, so
//! `dim_of` needs no table; host cells that a virtual cell absorbs are *hidden* (not live in the
//! overlay, `state` = `Merged(host revision)`) but stay untouched in the host.
//!
//! - **Composite faces** get a derived boundary (the interior edges and the vertices only they
//!   used are hidden), the sides of their members, and `Carrier::Composite(members)`.
//!   [`Overlay::composite_faces`] rejects members that are not connected through shared edges,
//!   that sit on different sides or in different volumes (or bodies), or that share an edge with
//!   a face outside the composite.
//! - **Partitions** take real cut geometry: [`Overlay::partition_edge`] adds a virtual vertex on
//!   an edge curve, [`Overlay::partition_face`] adds a virtual chord edge (with its own curve,
//!   stored in the overlay's geometry layer) between two vertices of a face's outer loop.
//! - Ids are **never reused**: each rank keeps a monotonic counter, and [`Overlay::remove`] undoes
//!   one virtual cell (rebuilding the rest) without recycling its id.
//! - The overlay keeps the host as an `Arc<dyn TopologyView>` snapshot with its
//!   `host_revision`; [`Overlay::is_stale_against`] / [`Overlay::rebase`] detect and repair a
//!   host that moved on. [`Overlay::data`] is the serializable record (replayed by
//!   [`Overlay::from_data`]).
//!
//! Limits (documented, not hidden): virtual cells have no lineage of their own (the host's log is
//! shown) and no pcurves; the radial ring of an edge whose faces changed keeps the host's angular
//! order (absorbed faces replaced by their virtual face) and appends new faces in `(face, sense)`
//! order; `face_moments` of a partition child is `Unsupported`.

use crate::complex::canonical_rotate;
use crate::error::{QueryError, TopoError};
use crate::geometry::{FaceMoments, Moments, PointClass};
use crate::lineage::LineageLog;
use crate::types::{Carrier, CellFlags, CoedgeKey, CurveCarrier, Dim, EntityState, FacePair, Incidence, OpId, Sense, ShellKind, Side};
use crate::view::{EdgeUse, NO_CARRIER, TopologyView};
use sm_core::{EntityId, EntityKind, Point3};
use sm_geom_api::{CurveDef, GeomError, GeomStore, GeomStoreMut, ImplicitSurface, Interval, MemoryStore, ModelTolerance, PCurve, ParametricCurve, ParametricSurface, SurfaceDef, Uv};
use smallvec::SmallVec;
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

/// The queries that distinguish a virtual layer from a plain view.
pub trait VirtualOverlay: TopologyView {
    /// The host revision this layer was built on.
    fn host_revision(&self) -> OpId;
    /// The host cells a virtual cell stands for (composite members; the parent for a partition
    /// child); empty for non-virtual ids.
    fn resolve(&self, virt: EntityId) -> &[EntityId];
    /// The virtual cell that absorbed a hidden host cell (the composite, or the first child of a
    /// partitioned face / split edge).
    fn virtual_of(&self, host: EntityId) -> Option<EntityId>;
}

/// Real geometry of a virtual face cut: a chord between two vertices of the face's outer loop.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VirtualCut {
    /// Start vertex (host or virtual) on the face's outer loop.
    pub from: EntityId,
    /// End vertex on the face's outer loop.
    pub to: EntityId,
    /// The chord curve, added to the overlay's geometry layer.
    pub curve: CurveDef,
    /// Curve parameter range from `from` to `to`.
    pub range: Interval,
}

/// One recorded virtual operation, with the ids it allocated, so replay reproduces them.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum VirtualOp {
    /// A composite face.
    Composite {
        /// The merged host faces, ascending.
        faces: Vec<EntityId>,
        /// The virtual face.
        id: EntityId,
    },
    /// A virtual vertex on an edge curve.
    PartitionEdge {
        /// The split edge.
        edge: EntityId,
        /// Curve parameter of the new vertex.
        param: f64,
        /// The new vertex.
        vertex: EntityId,
        /// The two halves, in edge order.
        halves: [EntityId; 2],
    },
    /// A virtual chord splitting a face.
    PartitionFace {
        /// The split face.
        face: EntityId,
        /// The cut geometry.
        cut: VirtualCut,
        /// The new chord edge.
        chord: EntityId,
        /// The two child faces.
        halves: [EntityId; 2],
    },
}

impl VirtualOp {
    fn created(&self) -> SmallVec<[EntityId; 4]> {
        match self {
            VirtualOp::Composite { id, .. } => SmallVec::from_slice(&[*id]),
            VirtualOp::PartitionEdge { vertex, halves, .. } => SmallVec::from_slice(&[*vertex, halves[0], halves[1]]),
            VirtualOp::PartitionFace { chord, halves, .. } => SmallVec::from_slice(&[*chord, halves[0], halves[1]]),
        }
    }

    fn inputs(&self) -> SmallVec<[EntityId; 4]> {
        match self {
            VirtualOp::Composite { faces, .. } => faces.iter().copied().collect(),
            VirtualOp::PartitionEdge { edge, .. } => SmallVec::from_slice(&[*edge]),
            VirtualOp::PartitionFace { face, cut, .. } => SmallVec::from_slice(&[*face, cut.from, cut.to]),
        }
    }
}

/// The persisted form of an overlay: the host revision, the operation history and the id
/// counters. Replayed against the host by [`Overlay::from_data`].
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OverlayData {
    /// Revision of the host the overlay was built on.
    pub host_revision: OpId,
    /// The operations, in order.
    pub ops: Vec<VirtualOp>,
    /// Next unused serial per virtual rank (vertex, edge, face).
    pub next_serial: [u64; 3],
}

/// Host store plus the overlay's own curves, behind one [`GeomStore`].
#[derive(Clone)]
struct Layered {
    base: Arc<dyn TopologyView>,
    extra: MemoryStore,
    curve_off: usize,
    surface_off: usize,
}

impl Layered {
    fn new(base: Arc<dyn TopologyView>) -> Self {
        let (curve_off, surface_off) = (base.shapes().curve_count(), base.shapes().surface_count());
        Self { base, extra: MemoryStore::new(), curve_off, surface_off }
    }

    fn local(id: EntityId, kind: EntityKind, off: usize) -> Option<Result<EntityId, ()>> {
        if id.kind() != kind {
            return Some(Err(()));
        }
        let s = usize::try_from(id.serial()).ok()?;
        if s < off { None } else { Some(EntityId::new(kind, (s - off) as u64).ok_or(())) }
    }

    fn add_curve(&mut self, def: CurveDef) -> Result<EntityId, GeomError> {
        let local = self.extra.add_curve(def)?;
        EntityId::new(EntityKind::Curve, local.serial() + self.curve_off as u64).ok_or(GeomError::Invalid("id space exhausted"))
    }
}

impl GeomStore for Layered {
    fn curve(&self, id: EntityId) -> Option<&dyn ParametricCurve> {
        match Self::local(id, EntityKind::Curve, self.curve_off) {
            None => self.base.shapes().curve(id),
            Some(Ok(l)) => self.extra.curve(l),
            Some(Err(())) => None,
        }
    }
    fn surface(&self, id: EntityId) -> Option<&dyn ParametricSurface> {
        match Self::local(id, EntityKind::Surface, self.surface_off) {
            None => self.base.shapes().surface(id),
            Some(Ok(l)) => self.extra.surface(l),
            Some(Err(())) => None,
        }
    }
    fn implicit(&self, id: EntityId) -> Option<&dyn ImplicitSurface> {
        match Self::local(id, EntityKind::Surface, self.surface_off) {
            None => self.base.shapes().implicit(id),
            Some(Ok(l)) => self.extra.implicit(l),
            Some(Err(())) => None,
        }
    }
    fn curve_def(&self, id: EntityId) -> Option<&CurveDef> {
        match Self::local(id, EntityKind::Curve, self.curve_off) {
            None => self.base.shapes().curve_def(id),
            Some(Ok(l)) => self.extra.curve_def(l),
            Some(Err(())) => None,
        }
    }
    fn surface_def(&self, id: EntityId) -> Option<&SurfaceDef> {
        match Self::local(id, EntityKind::Surface, self.surface_off) {
            None => self.base.shapes().surface_def(id),
            Some(Ok(l)) => self.extra.surface_def(l),
            Some(Err(())) => None,
        }
    }
    fn curve_count(&self) -> usize {
        self.curve_off + self.extra.curve_count()
    }
    fn surface_count(&self) -> usize {
        self.surface_off + self.extra.surface_count()
    }
}

#[derive(Clone, Debug)]
struct OCell {
    dim: Dim,
    is_virtual: bool,
    down: Vec<Incidence>,
    up: Vec<Incidence>,
    carrier: Carrier,
    members: Vec<EntityId>,
    aux: SmallVec<[EntityId; 8]>,
    radial: SmallVec<[EdgeUse; 4]>,
}

/// Everything derived from the host plus the operations; rebuilt by replay.
#[derive(Clone)]
struct Layer {
    cells: BTreeMap<EntityId, OCell>,
    hidden: BTreeSet<EntityId>,
    covering: BTreeMap<EntityId, EntityId>,
    live: [Vec<EntityId>; 5],
    shapes: Layered,
}

/// A virtual topology layer over an immutable host snapshot. See the module docs.
#[derive(Clone)]
pub struct Overlay {
    host: Arc<dyn TopologyView>,
    host_revision: OpId,
    host_hash: u64,
    ops: Vec<VirtualOp>,
    next_serial: [u64; 3],
    layer: Layer,
}

fn vrank(dim: Dim) -> Option<usize> {
    match dim {
        Dim::Vertex => Some(0),
        Dim::Edge => Some(1),
        Dim::Face => Some(2),
        _ => None,
    }
}

fn normalize_face_row(row: &mut Vec<Incidence>) {
    let mut out = Vec::with_capacity(row.len());
    let mut embedded: Vec<Incidence> = Vec::new();
    for g in row.chunk_by(|a, b| a.ring == b.ring) {
        if g[0].ring == Incidence::EMBEDDED {
            embedded.extend_from_slice(g);
        } else {
            let mut ring = g.to_vec();
            canonical_rotate(&mut ring);
            out.extend(ring);
        }
    }
    embedded.sort();
    embedded.dedup();
    out.extend(embedded);
    *row = out;
}

impl Overlay {
    /// An empty overlay over `host` (a view identical to the host).
    pub fn new(host: Arc<dyn TopologyView>) -> Self {
        let host_revision = host.revision();
        let host_hash = host.canonical_hash();
        let layer = Layer::fresh(&host);
        let mut o = Self { host, host_revision, host_hash, ops: Vec::new(), next_serial: [0; 3], layer };
        o.rebuild_live();
        o
    }

    /// Rebuild an overlay from its persisted record by replaying the operations on `host`.
    pub fn from_data(host: Arc<dyn TopologyView>, data: OverlayData) -> Result<Self, TopoError> {
        if host.revision() != data.host_revision {
            return Err(TopoError::StaleOverlay { built: data.host_revision.0, host: host.revision().0 });
        }
        let mut o = Self::new(host);
        o.next_serial = data.next_serial;
        for op in data.ops {
            o.apply(&op)?;
            o.ops.push(op);
        }
        Ok(o)
    }

    /// The persisted record.
    pub fn data(&self) -> OverlayData {
        OverlayData { host_revision: self.host_revision, ops: self.ops.clone(), next_serial: self.next_serial }
    }

    /// The untouched host view the overlay was built on.
    pub fn host(&self) -> &dyn TopologyView {
        self.host.as_ref()
    }

    /// The host's content hash at construction (unchanged by any overlay operation).
    pub fn host_hash(&self) -> u64 {
        self.host_hash
    }

    /// The recorded operations.
    pub fn ops(&self) -> &[VirtualOp] {
        &self.ops
    }

    /// True if `live` (the host as it is now) has moved on from the revision this overlay was
    /// built on; a stale overlay must be [`rebase`](Self::rebase)d before it is trusted.
    pub fn is_stale_against(&self, live: &dyn TopologyView) -> bool {
        live.revision() != self.host_revision
    }

    /// Re-apply the recorded operations on a newer host snapshot. Fails (leaving the overlay
    /// unchanged) if an operation no longer applies, e.g. a member face was deleted.
    pub fn rebase(&mut self, host: Arc<dyn TopologyView>) -> Result<(), TopoError> {
        let mut fresh = Self::new(host);
        fresh.next_serial = self.next_serial;
        for op in self.ops.clone() {
            fresh.apply(&op)?;
            fresh.ops.push(op);
        }
        *self = fresh;
        Ok(())
    }

    // ---------------------------------------------------------------- row access

    fn down_of(&self, id: EntityId) -> &[Incidence] {
        if self.layer.hidden.contains(&id) {
            return &[];
        }
        match self.layer.cells.get(&id) {
            Some(c) => &c.down,
            None => self.host.boundary(id),
        }
    }

    fn up_of(&self, id: EntityId) -> &[Incidence] {
        if self.layer.hidden.contains(&id) {
            return &[];
        }
        match self.layer.cells.get(&id) {
            Some(c) => &c.up,
            None => self.host.coboundary(id),
        }
    }

    fn is_live(&self, id: EntityId) -> bool {
        if self.layer.hidden.contains(&id) {
            return false;
        }
        match self.layer.cells.get(&id) {
            Some(_) => true,
            None => self.host.is_alive(id),
        }
    }

    fn edge_ends_of(&self, e: EntityId) -> Option<(EntityId, EntityId)> {
        match self.down_of(e) {
            [only] if only.side == Side::Both => Some((only.cell, only.cell)),
            [a, b] if a.side == Side::Left && b.side == Side::Right => Some((a.cell, b.cell)),
            _ => None,
        }
    }

    fn carrier_of(&self, id: EntityId) -> &Carrier {
        match self.layer.cells.get(&id) {
            Some(c) => &c.carrier,
            None => self.host.carrier(id),
        }
    }

    /// Copy-on-write access to a cell.
    fn cell_mut(&mut self, id: EntityId) -> Option<&mut OCell> {
        if !self.layer.cells.contains_key(&id) {
            let dim = self.host.dim_of(id)?;
            let c = OCell { dim, is_virtual: false, down: self.host.boundary(id).to_vec(), up: self.host.coboundary(id).to_vec(), carrier: self.host.carrier(id).clone(), members: Vec::new(), aux: SmallVec::new(), radial: SmallVec::new() };
            self.layer.cells.insert(id, c);
        }
        self.layer.cells.get_mut(&id)
    }

    fn up_remove(&mut self, child: EntityId, inc: Incidence) {
        if let Some(c) = self.cell_mut(child) {
            if let Ok(i) = c.up.binary_search(&inc) {
                c.up.remove(i);
            }
        }
    }

    fn up_insert(&mut self, child: EntityId, inc: Incidence) {
        if let Some(c) = self.cell_mut(child) {
            let i = match c.up.binary_search(&inc) {
                Ok(i) | Err(i) => i,
            };
            c.up.insert(i, inc);
        }
    }

    /// Replace the boundary row of `id`, maintaining the coboundary rows of old and new children.
    fn set_down(&mut self, id: EntityId, new: Vec<Incidence>) {
        let old: Vec<Incidence> = self.down_of(id).to_vec();
        for e in &old {
            self.up_remove(e.cell, Incidence { cell: id, side: e.side, ring: e.ring });
        }
        for e in &new {
            self.up_insert(e.cell, Incidence { cell: id, side: e.side, ring: e.ring });
        }
        let mut aux: SmallVec<[EntityId; 8]> = new.iter().filter(|e| e.ring != Incidence::EMBEDDED).map(|e| e.cell).collect();
        aux.sort();
        aux.dedup();
        if let Some(c) = self.cell_mut(id) {
            c.down = new;
            c.aux = aux;
        }
    }

    /// Hide a host or virtual cell: detach it from its children and parents.
    fn hide(&mut self, id: EntityId, covered_by: EntityId) {
        let down: Vec<Incidence> = self.down_of(id).to_vec();
        for e in &down {
            self.up_remove(e.cell, Incidence { cell: id, side: e.side, ring: e.ring });
        }
        let ups: Vec<Incidence> = self.up_of(id).to_vec();
        for p in ups {
            let row: Vec<Incidence> = self.down_of(p.cell).iter().copied().filter(|e| e.cell != id).collect();
            self.set_down(p.cell, row);
        }
        if self.layer.cells.get(&id).is_some_and(|c| c.is_virtual) {
            self.layer.cells.remove(&id);
        }
        self.layer.hidden.insert(id);
        self.layer.covering.insert(id, covered_by);
    }

    /// Rebuild the radial rings of edges whose faces changed: the host's angular order with each
    /// absorbed face replaced by the virtual face that now stands for it; faces that did not
    /// exist in the host (new chords) follow in `(face, sense)` order.
    fn rebuild_radial(&mut self) {
        let edges: Vec<EntityId> = self.layer.cells.iter().filter(|(id, c)| c.dim == Dim::Edge && !self.layer.hidden.contains(id)).map(|(id, _)| *id).collect();
        for e in edges {
            let current: Vec<EdgeUse> = self
                .up_of(e)
                .iter()
                .filter(|u| self.host.dim_of(u.cell) == Some(Dim::Face) || self.layer.cells.get(&u.cell).is_some_and(|c| c.dim == Dim::Face))
                .map(|u| EdgeUse { face: u.cell, sense: u.side.sense() })
                .collect();
            let mut out: SmallVec<[EdgeUse; 4]> = SmallVec::new();
            for hu in self.host.radial_uses(e) {
                let mapped = current.iter().find(|c| c.sense == hu.sense && (c.face == hu.face || self.layer.cells.get(&c.face).is_some_and(|vc| vc.is_virtual && vc.members.contains(&hu.face))));
                if let Some(m) = mapped {
                    if !out.contains(m) {
                        out.push(*m);
                    }
                }
            }
            for c in current {
                if !out.contains(&c) {
                    out.push(c);
                }
            }
            if let Some(c) = self.layer.cells.get_mut(&e) {
                c.radial = out;
            }
        }
    }

    fn rebuild_live(&mut self) {
        self.rebuild_radial();
        for dim in Dim::ALL {
            let mut l: Vec<EntityId> = self.host.cells(dim).iter().copied().filter(|c| !self.layer.hidden.contains(c)).collect();
            l.extend(self.layer.cells.iter().filter(|(_, c)| c.is_virtual && c.dim == dim).map(|(id, _)| *id));
            l.sort();
            self.layer.live[dim.index()] = l;
        }
    }

    fn add_virtual(&mut self, id: EntityId, dim: Dim, down: Vec<Incidence>, carrier: Carrier, members: Vec<EntityId>) {
        self.layer.cells.insert(id, OCell { dim, is_virtual: true, down: Vec::new(), up: Vec::new(), carrier, members, aux: SmallVec::new(), radial: SmallVec::new() });
        self.set_down(id, down);
    }

    // ---------------------------------------------------------------- operations

    fn atomic(&mut self, op: VirtualOp, bump: [u64; 3]) -> Result<VirtualOp, TopoError> {
        let snap_layer = self.layer.clone();
        let snap_serial = self.next_serial;
        for (n, b) in self.next_serial.iter_mut().zip(bump) {
            *n += b;
        }
        let r = self.apply(&op);
        match r {
            Ok(()) => Ok(op),
            Err(e) => {
                self.layer = snap_layer;
                self.next_serial = snap_serial;
                Err(e)
            }
        }
    }

    /// Merge `faces` (host faces, at least two, connected through shared edges) into one virtual
    /// face. The interior edges (and vertices used only by them) disappear from the overlay; the
    /// composite's loops are the remaining coedges chained head to tail.
    ///
    /// Rejected: fewer than two faces, faces that are not live host faces, members that are not
    /// connected, members whose parents (volumes, shells, sides) differ, members that share an
    /// edge with a face outside the composite, a non-manifold interior, and boundaries that pinch
    /// at a vertex or do not form simple loops.
    pub fn composite_faces(&mut self, faces: &[EntityId]) -> Result<EntityId, TopoError> {
        let mut members: Vec<EntityId> = faces.to_vec();
        members.sort();
        members.dedup();
        let id = self.peek(Dim::Face, 0)?;
        let done = self.atomic(VirtualOp::Composite { faces: members, id }, [0, 0, 1])?;
        self.ops.push(done);
        self.rebuild_live();
        Ok(id)
    }

    /// Add a virtual vertex on the curve of `edge` at `param`, replacing the edge by two virtual
    /// halves in every face that uses it. Returns the vertex and the halves in edge order.
    pub fn partition_edge(&mut self, edge: EntityId, param: f64) -> Result<(EntityId, [EntityId; 2]), TopoError> {
        let op = VirtualOp::PartitionEdge { edge, param, vertex: self.peek(Dim::Vertex, 0)?, halves: [self.peek(Dim::Edge, 0)?, self.peek(Dim::Edge, 1)?] };
        let done = self.atomic(op, [1, 2, 0])?;
        let VirtualOp::PartitionEdge { vertex, halves, .. } = &done else { return Err(TopoError::Unsupported("internal")) };
        let out = (*vertex, *halves);
        self.ops.push(done);
        self.rebuild_live();
        Ok(out)
    }

    /// Split `face` by a virtual chord with real geometry (see [`VirtualCut`]). Returns the two
    /// child faces `[first, second]` (the first runs from the earlier loop position of the two
    /// chord ends). The face must have a single loop.
    pub fn partition_face(&mut self, face: EntityId, cut: VirtualCut) -> Result<[EntityId; 2], TopoError> {
        let op = VirtualOp::PartitionFace { face, cut, chord: self.peek(Dim::Edge, 0)?, halves: [self.peek(Dim::Face, 0)?, self.peek(Dim::Face, 1)?] };
        let done = self.atomic(op, [0, 1, 2])?;
        let VirtualOp::PartitionFace { halves, .. } = &done else { return Err(TopoError::Unsupported("internal")) };
        let out = *halves;
        self.ops.push(done);
        self.rebuild_live();
        Ok(out)
    }

    /// Undo the operation that created virtual cell `virt` (any of the cells it created names
    /// it), restoring the cells it hid. Fails with `StillReferenced` if a later operation uses
    /// one of its cells. The id is not reused.
    pub fn remove(&mut self, virt: EntityId) -> Result<(), TopoError> {
        let Some(idx) = self.ops.iter().position(|o| o.created().contains(&virt)) else { return Err(TopoError::NoSuchCell(virt)) };
        let created = self.ops[idx].created();
        if self.ops[idx + 1..].iter().any(|later| later.inputs().iter().any(|i| created.contains(i))) {
            return Err(TopoError::StillReferenced(virt));
        }
        let mut ops = self.ops.clone();
        ops.remove(idx);
        let mut fresh = Self::new(self.host.clone());
        fresh.next_serial = self.next_serial;
        for op in &ops {
            fresh.apply(op)?;
        }
        fresh.ops = ops;
        fresh.rebuild_live();
        *self = fresh;
        Ok(())
    }

    /// Remove every virtual cell, restoring the host view.
    pub fn clear(&mut self) {
        let serial = self.next_serial;
        *self = Self::new(self.host.clone());
        self.next_serial = serial;
    }

    fn peek(&self, dim: Dim, offset: u64) -> Result<EntityId, TopoError> {
        let r = vrank(dim).ok_or(TopoError::Unsupported("virtual cells of this rank"))?;
        let kind = dim.virtual_kind().ok_or(TopoError::Unsupported("virtual cells of this rank"))?;
        EntityId::new(kind, self.next_serial[r] + offset).ok_or(TopoError::IdExhausted)
    }

    /// Apply one operation to the layer (ids come from the operation).
    fn apply(&mut self, op: &VirtualOp) -> Result<(), TopoError> {
        match op {
            VirtualOp::Composite { faces, id } => self.apply_composite(faces, *id),
            VirtualOp::PartitionEdge { edge, param, vertex, halves } => self.apply_partition_edge(*edge, *param, *vertex, *halves),
            VirtualOp::PartitionFace { face, cut, chord, halves } => self.apply_partition_face(*face, cut, *chord, *halves),
        }?;
        self.rebuild_live();
        Ok(())
    }

    fn apply_composite(&mut self, members: &[EntityId], vf: EntityId) -> Result<(), TopoError> {
        if members.len() < 2 {
            return Err(TopoError::InvalidParameter("a composite needs at least two faces"));
        }
        for &f in members {
            if self.host.dim_of(f) != Some(Dim::Face) {
                return Err(TopoError::WrongDim { id: f, expected: Dim::Face });
            }
            if !self.is_live(f) || self.layer.cells.get(&f).is_some_and(|c| c.is_virtual) {
                return Err(TopoError::Dead(f));
            }
        }
        // Parents must agree: same volumes/bodies, sides and shells for every member.
        let parents_of = |o: &Self, f: EntityId| -> Vec<Incidence> { o.up_of(f).to_vec() };
        let first = parents_of(self, members[0]);
        if members.iter().any(|&f| parents_of(self, f) != first) {
            return Err(TopoError::Unsupported("members sit in different volumes, shells or on different sides"));
        }
        // Net coedge uses per edge; an edge shared by two members cancels.
        let mut uses: BTreeMap<EntityId, Vec<(EntityId, Side)>> = BTreeMap::new();
        for &f in members {
            for e in self.down_of(f) {
                if e.ring != Incidence::EMBEDDED {
                    uses.entry(e.cell).or_default().push((f, e.side));
                }
            }
        }
        let mset: BTreeSet<EntityId> = members.iter().copied().collect();
        let mut interior: Vec<EntityId> = Vec::new();
        let mut boundary: Vec<(EntityId, Side)> = Vec::new();
        for (edge, us) in &uses {
            let distinct: BTreeSet<EntityId> = us.iter().map(|u| u.0).collect();
            if distinct.len() >= 2 {
                if self.up_of(*edge).iter().any(|u| self.host.dim_of(u.cell) == Some(Dim::Face) && !mset.contains(&u.cell)) {
                    return Err(TopoError::Unsupported("a member shares an edge with a face outside the composite"));
                }
                let l = us.iter().filter(|u| u.1 == Side::Left).count();
                if l * 2 != us.len() {
                    return Err(TopoError::Unsupported("non-manifold composite interior"));
                }
                interior.push(*edge);
            } else {
                boundary.extend(us.iter().map(|u| (*edge, u.1)));
            }
        }
        // Connectivity through the interior edges.
        let mut comp: BTreeSet<EntityId> = BTreeSet::from([members[0]]);
        let mut grew = true;
        while grew {
            grew = false;
            for e in &interior {
                let fs: BTreeSet<EntityId> = uses[e].iter().map(|u| u.0).collect();
                if fs.iter().any(|f| comp.contains(f)) && fs.iter().any(|f| !comp.contains(f)) {
                    comp.extend(fs);
                    grew = true;
                }
            }
        }
        if comp.len() != mset.len() {
            return Err(TopoError::InvalidParameter("members are not connected through shared edges"));
        }
        // Chain the remaining coedges into loops.
        let ends = |o: &Self, e: EntityId, s: Side| -> Option<(EntityId, EntityId)> {
            let (a, b) = o.edge_ends_of(e)?;
            Some(if s == Side::Right { (b, a) } else { (a, b) })
        };
        let mut by_tail: BTreeMap<EntityId, usize> = BTreeMap::new();
        for (i, (e, s)) in boundary.iter().enumerate() {
            let (tail, _) = ends(self, *e, *s).ok_or(TopoError::WrongDim { id: *e, expected: Dim::Edge })?;
            if by_tail.insert(tail, i).is_some() {
                return Err(TopoError::Unsupported("composite boundary pinches at a vertex"));
            }
        }
        let mut used = vec![false; boundary.len()];
        let mut order: Vec<usize> = (0..boundary.len()).collect();
        order.sort_by_key(|&i| boundary[i]);
        let mut down: Vec<Incidence> = Vec::new();
        let mut ring = 0u16;
        for &start in &order {
            if used[start] {
                continue;
            }
            let mut cur = start;
            let mut lp = Vec::new();
            loop {
                used[cur] = true;
                let (e, s) = boundary[cur];
                lp.push(Incidence::new(e, s, ring));
                let (_, head) = ends(self, e, s).ok_or(TopoError::WrongDim { id: e, expected: Dim::Edge })?;
                let Some(&next) = by_tail.get(&head) else { return Err(TopoError::OpenLoop { ring: usize::from(ring), at: lp.len() }) };
                if next == start {
                    break;
                }
                if used[next] {
                    return Err(TopoError::Unsupported("composite boundary is not a set of simple loops"));
                }
                cur = next;
            }
            canonical_rotate(&mut lp);
            down.extend(lp);
            ring += 1;
        }
        // Commit: hide the members and the interior, then create the virtual face.
        let parents = first;
        for &f in members {
            self.hide(f, vf);
        }
        let mut verts: BTreeSet<EntityId> = BTreeSet::new();
        for &e in &interior {
            if let Some((a, b)) = self.edge_ends_of(e) {
                verts.extend([a, b]);
            }
            self.hide(e, vf);
        }
        for v in verts {
            if self.up_of(v).is_empty() && self.is_live(v) {
                self.hide(v, vf);
            }
        }
        let carrier = Carrier::Composite(members.iter().copied().collect());
        self.add_virtual(vf, Dim::Face, down, carrier, members.to_vec());
        for p in parents {
            let mut row = self.down_of(p.cell).to_vec();
            row.push(Incidence::new(vf, p.side, p.ring));
            row.sort_by_key(|e| (e.ring, e.cell, e.side));
            self.set_down(p.cell, row);
        }
        Ok(())
    }

    fn apply_partition_edge(&mut self, edge: EntityId, param: f64, vertex: EntityId, halves: [EntityId; 2]) -> Result<(), TopoError> {
        if self.host.dim_of(edge) != Some(Dim::Edge) && !self.layer.cells.get(&edge).is_some_and(|c| c.dim == Dim::Edge) {
            return Err(TopoError::WrongDim { id: edge, expected: Dim::Edge });
        }
        if !self.is_live(edge) {
            return Err(TopoError::Dead(edge));
        }
        let (start, end) = self.edge_ends_of(edge).ok_or(TopoError::WrongDim { id: edge, expected: Dim::Edge })?;
        let Carrier::Curve(cc) = self.carrier_of(edge).clone() else { return Err(TopoError::Unsupported("partition_edge needs a curve carrier")) };
        if !(cc.range.lo < param && param < cc.range.hi) {
            return Err(TopoError::InvalidParameter("split parameter outside edge range"));
        }
        let curve = self.layer.shapes.curve(cc.curve).ok_or(TopoError::Geom(GeomError::Invalid("curve not in store")))?;
        let p: Point3 = curve.eval(param);
        let (lo, hi) = (Interval::new(cc.range.lo, param), Interval::new(param, cc.range.hi));
        let (r1, r2) = if cc.sense == Sense::Forward { (lo, hi) } else { (hi, lo) };
        let [e1, e2] = halves;
        self.add_virtual(vertex, Dim::Vertex, Vec::new(), Carrier::Point(p), vec![edge]);
        self.add_virtual(e1, Dim::Edge, Vec::new(), Carrier::Curve(CurveCarrier { range: r1, ..cc }), vec![edge]);
        self.add_virtual(e2, Dim::Edge, Vec::new(), Carrier::Curve(CurveCarrier { range: r2, ..cc }), vec![edge]);
        self.set_down(e1, vec![Incidence::new(start, Side::Left, 0), Incidence::new(vertex, Side::Right, 0)]);
        self.set_down(e2, vec![Incidence::new(vertex, Side::Left, 0), Incidence::new(end, Side::Right, 0)]);
        let parents: Vec<Incidence> = self.up_of(edge).to_vec();
        for par in parents {
            let is_face = self.host.dim_of(par.cell) == Some(Dim::Face) || self.layer.cells.get(&par.cell).is_some_and(|c| c.dim == Dim::Face);
            let mut row: Vec<Incidence> = Vec::new();
            for e in self.down_of(par.cell) {
                if e.cell == edge {
                    match e.side {
                        Side::Right => {
                            row.push(Incidence { cell: e2, ..*e });
                            row.push(Incidence { cell: e1, ..*e });
                        }
                        _ => {
                            row.push(Incidence { cell: e1, ..*e });
                            row.push(Incidence { cell: e2, ..*e });
                        }
                    }
                } else {
                    row.push(*e);
                }
            }
            if is_face {
                normalize_face_row(&mut row);
            }
            self.set_down(par.cell, row);
        }
        self.hide(edge, e1);
        Ok(())
    }

    fn apply_partition_face(&mut self, face: EntityId, cut: &VirtualCut, chord: EntityId, halves: [EntityId; 2]) -> Result<(), TopoError> {
        let is_face = self.host.dim_of(face) == Some(Dim::Face) || self.layer.cells.get(&face).is_some_and(|c| c.dim == Dim::Face);
        if !is_face {
            return Err(TopoError::WrongDim { id: face, expected: Dim::Face });
        }
        if !self.is_live(face) {
            return Err(TopoError::Dead(face));
        }
        if cut.from == cut.to {
            return Err(TopoError::InvalidParameter("a chord joins two distinct vertices"));
        }
        let row = self.down_of(face).to_vec();
        if row.iter().any(|e| e.ring != 0) {
            return Err(TopoError::Unsupported("partition_face of a face with inner loops or embedded members"));
        }
        let tails: Vec<Option<EntityId>> = row.iter().map(|e| self.edge_ends_of(e.cell).map(|(s, t)| if e.side == Side::Right { t } else { s })).collect();
        let pos = |v: EntityId| -> Vec<usize> { tails.iter().enumerate().filter(|(_, t)| **t == Some(v)).map(|(i, _)| i).collect() };
        let (pi, pj) = (pos(cut.from), pos(cut.to));
        if pi.is_empty() || pj.is_empty() {
            return Err(TopoError::ChordNotOnLoop);
        }
        if pi.len() != 1 || pj.len() != 1 {
            return Err(TopoError::Unsupported("chord end vertex appears more than once in the loop"));
        }
        let (a, b) = (pi[0].min(pj[0]), pi[0].max(pj[0]));
        let side1 = if tails[b] == Some(cut.from) && tails[a] == Some(cut.to) { Side::Left } else { Side::Right };
        let mut r1: Vec<Incidence> = row[a..b].to_vec();
        r1.push(Incidence::new(chord, side1, 0));
        let mut r2: Vec<Incidence> = row[b..].iter().chain(row[..a].iter()).copied().collect();
        r2.push(Incidence::new(chord, side1.flip(), 0));
        for r in [&mut r1, &mut r2] {
            normalize_face_row(r);
        }
        let curve = self.layer.shapes.add_curve(cut.curve.clone())?;
        self.add_virtual(chord, Dim::Edge, Vec::new(), Carrier::Curve(CurveCarrier { curve, range: cut.range, sense: Sense::Forward }), vec![face]);
        self.set_down(chord, vec![Incidence::new(cut.from, Side::Left, 0), Incidence::new(cut.to, Side::Right, 0)]);
        let carrier = self.carrier_of(face).clone();
        let [f1, f2] = halves;
        self.add_virtual(f1, Dim::Face, r1, carrier.clone(), vec![face]);
        self.add_virtual(f2, Dim::Face, r2, carrier, vec![face]);
        let parents: Vec<Incidence> = self.up_of(face).to_vec();
        self.hide(face, f1);
        for p in parents {
            let mut prow = self.down_of(p.cell).to_vec();
            prow.push(Incidence::new(f1, p.side, p.ring));
            prow.push(Incidence::new(f2, p.side, p.ring));
            prow.sort_by_key(|e| (e.ring, e.cell, e.side));
            self.set_down(p.cell, prow);
        }
        Ok(())
    }
}

impl Layer {
    fn fresh(host: &Arc<dyn TopologyView>) -> Self {
        Self { cells: BTreeMap::new(), hidden: BTreeSet::new(), covering: BTreeMap::new(), live: Default::default(), shapes: Layered::new(host.clone()) }
    }
}

impl TopologyView for Overlay {
    fn tolerance(&self) -> &ModelTolerance {
        self.host.tolerance()
    }

    fn shapes(&self) -> &dyn GeomStore {
        &self.layer.shapes
    }

    fn dim_of(&self, id: EntityId) -> Option<Dim> {
        match Dim::of_kind(id.kind()) {
            Some(d) if id.kind() != d.kind() => self.layer.cells.get(&id).map(|c| c.dim),
            _ => self.host.dim_of(id),
        }
    }

    fn state(&self, id: EntityId) -> Option<EntityState> {
        if self.layer.cells.get(&id).is_some_and(|c| c.is_virtual) {
            return Some(EntityState::Live);
        }
        if self.layer.hidden.contains(&id) {
            return Some(EntityState::Merged(self.host_revision));
        }
        self.host.state(id)
    }

    fn cells(&self, dim: Dim) -> &[EntityId] {
        &self.layer.live[dim.index()]
    }

    fn boundary(&self, id: EntityId) -> &[Incidence] {
        self.down_of(id)
    }

    fn coboundary(&self, id: EntityId) -> &[Incidence] {
        self.up_of(id)
    }

    fn carrier(&self, id: EntityId) -> &Carrier {
        if self.layer.hidden.contains(&id) {
            return &NO_CARRIER;
        }
        self.carrier_of(id)
    }

    fn flags(&self, id: EntityId) -> CellFlags {
        if self.layer.cells.get(&id).is_some_and(|c| c.is_virtual) { CellFlags::NONE } else { self.host.flags(id) }
    }

    fn pcurve(&self, k: CoedgeKey) -> Option<&PCurve> {
        let virt = |id: EntityId| self.layer.cells.get(&id).is_some_and(|c| c.is_virtual);
        if virt(k.face) || virt(k.edge) { None } else { self.host.pcurve(k) }
    }

    fn shell_kind(&self, volume: EntityId, ring: u16) -> Option<ShellKind> {
        self.host.shell_kind(volume, ring)
    }

    fn radial_uses(&self, edge: EntityId) -> &[EdgeUse] {
        if self.layer.hidden.contains(&edge) {
            return &[];
        }
        match self.layer.cells.get(&edge) {
            Some(c) => &c.radial,
            None => self.host.radial_uses(edge),
        }
    }

    fn tolerance_override(&self, id: EntityId) -> Option<f64> {
        self.host.tolerance_override(id)
    }

    fn coincidences(&self) -> &[FacePair] {
        self.host.coincidences()
    }

    fn coincident_faces(&self, f: EntityId) -> &[FacePair] {
        self.host.coincident_faces(f)
    }

    fn lineage(&self) -> &LineageLog {
        self.host.lineage()
    }

    fn born(&self, id: EntityId) -> Option<OpId> {
        self.host.born(id)
    }

    fn revision(&self) -> OpId {
        self.host.revision()
    }

    fn volume_faces(&self, v: EntityId) -> &[EntityId] {
        match self.layer.cells.get(&v) {
            Some(c) => &c.aux,
            None => self.host.volume_faces(v),
        }
    }

    fn face_edges(&self, f: EntityId) -> &[EntityId] {
        if self.layer.hidden.contains(&f) {
            return &[];
        }
        match self.layer.cells.get(&f) {
            Some(c) => &c.aux,
            None => self.host.face_edges(f),
        }
    }

    fn volume_moments(&self, v: EntityId) -> Result<Moments, QueryError> {
        self.host.volume_moments(v)
    }

    fn face_moments(&self, f: EntityId) -> Result<FaceMoments, QueryError> {
        if self.layer.hidden.contains(&f) {
            return Err(QueryError::NoSuchCell(f));
        }
        match self.layer.cells.get(&f) {
            Some(c) if c.is_virtual => match &c.carrier {
                Carrier::Composite(members) => {
                    let mut acc = FaceMoments::default();
                    for m in members {
                        let fm = self.host.face_moments(*m)?;
                        acc.area += fm.area;
                        acc.first = acc.first + fm.first;
                        acc.vector_area = acc.vector_area + fm.vector_area;
                        for i in 0..3 {
                            for j in 0..3 {
                                acc.second[i][j] += fm.second[i][j];
                            }
                        }
                    }
                    Ok(acc)
                }
                _ => Err(QueryError::Unsupported("moments of a partitioned virtual face")),
            },
            _ => self.host.face_moments(f),
        }
    }

    fn classify_point_on_face(&self, f: EntityId, uv: Uv) -> Result<PointClass, QueryError> {
        match self.layer.cells.get(&f) {
            Some(c) if c.is_virtual => Err(QueryError::Unsupported("point classification on a virtual face")),
            _ => self.host.classify_point_on_face(f, uv),
        }
    }
}

impl VirtualOverlay for Overlay {
    fn host_revision(&self) -> OpId {
        self.host_revision
    }

    fn resolve(&self, virt: EntityId) -> &[EntityId] {
        self.layer.cells.get(&virt).filter(|c| c.is_virtual).map_or(&[], |c| c.members.as_slice())
    }

    fn virtual_of(&self, host: EntityId) -> Option<EntityId> {
        self.layer.covering.get(&host).copied()
    }
}
