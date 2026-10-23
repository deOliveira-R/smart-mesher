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
//! - **Volume splits** ([`Overlay::partition_volume`]) add a virtual cut face (with its own
//!   carrier surface) spanning a closed loop of live edges and replace the volume by two virtual
//!   volumes. The two volume ids are **supplied by the caller** (the model's decomposition mints
//!   `PartitionCell` piece ids; the overlay mints vertex, edge and face ids), so a piece and its
//!   overlay volume are the same id. [`Overlay::partition_volume_by_plane`] does the whole plane
//!   cut: virtual vertices where edges cross the plane, virtual chords across the faces it
//!   cuts, then the volume split.
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
//! order; `face_moments` of a partition child and `volume_moments` of a virtual volume are
//! `Unsupported`. Virtual volumes have one `Outer` shell and no baffles; the orientation of the
//! cut face (normal from the first child into the second) is found from the side of the first
//! child's vertices, exact for planar cuts and a convexity assumption for curved ones. Faces
//! keep their host orientation (the lowest-id-left rule is not re-applied across virtual
//! volumes).

use crate::complex::canonical_rotate;
use crate::error::{QueryError, TopoError};
use crate::geometry::{FaceMoments, Moments, PointClass};
use crate::lineage::LineageLog;
use crate::types::{Carrier, CellFlags, CoedgeKey, CurveCarrier, Dim, EntityState, FacePair, Incidence, OpId, Sense, ShellKind, Side, SurfaceCarrier};
use crate::derived::TopologyViewExt;
use crate::view::{EdgeUse, NO_CARRIER, TopologyView};
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_geom_api::{CurveDef, Frame, GeomError, GeomAppend, GeomStore, ImplicitSurface, Interval, Line, MemoryStore, ModelTolerance, PCurve, ParametricCurve, ParametricSurface, Plane, SurfaceDef, Uv};
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

/// A virtual volume split: the cut face spans the closed loop `edges` and lies on `surface`.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VolumeCut {
    /// Live edges (host or virtual) of the volume's faces forming one closed loop, in loop order
    /// (consecutive edges share a vertex; either direction).
    pub edges: Vec<EntityId>,
    /// Carrier surface of the cut face (added to the overlay's geometry layer). Must be
    /// parametric (not implicit-only).
    pub surface: SurfaceDef,
    /// A face of the volume that goes to the first child (the cut face's left side).
    pub seed: EntityId,
}

/// What [`Overlay::partition_volume_by_plane`] created.
#[derive(Clone, Debug, PartialEq)]
pub struct PlaneCut {
    /// The virtual cut face.
    pub face: EntityId,
    /// The two virtual volumes: `[negative side, positive side]` of the plane (the ids passed in).
    pub children: [EntityId; 2],
    /// Edges that crossed the plane: `(edge, new virtual vertex, halves in edge order)`.
    pub split_edges: Vec<(EntityId, EntityId, [EntityId; 2])>,
    /// Faces the plane cut: `(face, virtual chord, [negative half, positive half])`.
    pub split_faces: Vec<(EntityId, EntityId, [EntityId; 2])>,
    /// The cut face's loop edges, ascending.
    pub loop_edges: Vec<EntityId>,
    /// The cut face's loop vertices, ascending.
    pub loop_vertices: Vec<EntityId>,
}

/// One recorded virtual operation, with the ids it allocated, so replay reproduces them.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
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
    /// A virtual volume split.
    PartitionVolume {
        /// The split volume (host or virtual).
        volume: EntityId,
        /// The cut.
        cut: VolumeCut,
        /// The new cut face.
        face: EntityId,
        /// The two virtual volumes (caller-supplied `PartitionCell` ids), the first on the cut
        /// face's left.
        children: [EntityId; 2],
    },
}

impl VirtualOp {
    fn created(&self) -> SmallVec<[EntityId; 4]> {
        match self {
            VirtualOp::Composite { id, .. } => SmallVec::from_slice(&[*id]),
            VirtualOp::PartitionEdge { vertex, halves, .. } => SmallVec::from_slice(&[*vertex, halves[0], halves[1]]),
            VirtualOp::PartitionFace { chord, halves, .. } => SmallVec::from_slice(&[*chord, halves[0], halves[1]]),
            VirtualOp::PartitionVolume { face, children, .. } => SmallVec::from_slice(&[*face, children[0], children[1]]),
        }
    }

    fn inputs(&self) -> SmallVec<[EntityId; 4]> {
        match self {
            VirtualOp::Composite { faces, .. } => faces.iter().copied().collect(),
            VirtualOp::PartitionEdge { edge, .. } => SmallVec::from_slice(&[*edge]),
            VirtualOp::PartitionFace { face, cut, .. } => SmallVec::from_slice(&[*face, cut.from, cut.to]),
            VirtualOp::PartitionVolume { volume, cut, .. } => std::iter::once(*volume).chain(cut.edges.iter().copied()).chain(std::iter::once(cut.seed)).collect(),
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
    /// Next unused serial per virtual rank (vertex, edge, face); starts at 1.
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

    fn add_surface(&mut self, def: SurfaceDef) -> Result<EntityId, GeomError> {
        let local = self.extra.add_surface(def)?;
        EntityId::new(EntityKind::Surface, local.serial() + self.surface_off as u64).ok_or(GeomError::Invalid("id space exhausted"))
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
        let mut o = Self { host, host_revision, host_hash, ops: Vec::new(), next_serial: [1; 3], layer };
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

    /// A shared handle to the host snapshot (usable while the overlay is borrowed mutably).
    pub fn host_arc(&self) -> Arc<dyn TopologyView> {
        self.host.clone()
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
            // The aux row (face -> edges, volume -> faces) is derived from `down`; copy it too.
            let aux: SmallVec<[EntityId; 8]> = match dim {
                Dim::Face => SmallVec::from_slice(self.host.face_edges(id)),
                Dim::Volume => SmallVec::from_slice(self.host.volume_faces(id)),
                _ => SmallVec::new(),
            };
            let c = OCell { dim, is_virtual: false, down: self.host.boundary(id).to_vec(), up: self.host.coboundary(id).to_vec(), carrier: self.host.carrier(id).clone(), members: Vec::new(), aux, radial: SmallVec::new() };
            self.layer.cells.insert(id, c);
        }
        self.layer.cells.get_mut(&id)
    }

    fn up_remove(&mut self, child: EntityId, inc: Incidence) {
        if let Some(c) = self.cell_mut(child)
            && let Ok(i) = c.up.binary_search(&inc)
        {
            c.up.remove(i);
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
                if let Some(m) = mapped
                    && !out.contains(m)
                {
                    out.push(*m);
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

    /// Split `volume` (host or virtual, one outer shell, no baffles) into two virtual volumes
    /// `children` (fresh `PartitionCell` ids, ascending; the caller mints them) by a virtual cut
    /// face spanning `cut.edges`. The volume's faces are partitioned by flood fill across the
    /// edges that are not on the loop; the part containing `cut.seed` goes to `children[0]`,
    /// which sits on the cut face's left (its normal points into `children[1]`). Exactly two
    /// parts are required. Returns the cut face. The loop edges must already exist (use
    /// [`Overlay::partition_edge`] / [`Overlay::partition_face`], or
    /// [`Overlay::partition_volume_by_plane`] for the whole plane cut).
    pub fn partition_volume(&mut self, volume: EntityId, cut: VolumeCut, children: [EntityId; 2]) -> Result<EntityId, TopoError> {
        let face = self.peek(Dim::Face, 0)?;
        let done = self.atomic(VirtualOp::PartitionVolume { volume, cut, face, children }, [0, 0, 1])?;
        self.ops.push(done);
        self.rebuild_live();
        Ok(face)
    }

    /// Cut `volume` by the plane through `origin` with normal `normal` into `children =
    /// [negative side, positive side]` (fresh `PartitionCell` ids, ascending): a virtual vertex
    /// where an edge's end vertices lie strictly on opposite sides (bisection on its curve), a
    /// straight virtual chord across every face with vertices strictly on both sides (the face
    /// must be planar, with exactly two outer-loop vertices on the plane), then
    /// [`Overlay::partition_volume`] with the plane as carrier. Atomic: on error the overlay is
    /// unchanged. Vertices within the model's linear tolerance of the plane count as on it.
    ///
    /// Limits: an edge is assumed to cross the plane at most once (only its end vertices are
    /// tested); the section must be one simple loop; a face lying in the plane is refused.
    pub fn partition_volume_by_plane(&mut self, volume: EntityId, origin: Point3, normal: Vector3, children: [EntityId; 2]) -> Result<PlaneCut, TopoError> {
        let snapshot = self.clone();
        let r = self.plane_cut(volume, origin, normal, children);
        if r.is_err() {
            *self = snapshot;
        }
        r
    }

    fn point_of(&self, v: EntityId) -> Result<Point3, TopoError> {
        match self.carrier_of(v) {
            Carrier::Point(p) => Ok(*p),
            _ => Err(TopoError::Unsupported("vertex without a point carrier")),
        }
    }

    fn plane_cut(&mut self, volume: EntityId, origin: Point3, normal: Vector3, children: [EntityId; 2]) -> Result<PlaneCut, TopoError> {
        let len = normal.norm();
        if !(len.is_finite() && len > 0.0) {
            return Err(TopoError::InvalidParameter("zero plane normal"));
        }
        let n = normal * (1.0 / len);
        let tol = self.host.tolerance().linear;
        let dist = |p: Point3| n.dot(p - origin);
        if self.dim_of(volume) != Some(Dim::Volume) || !self.is_live(volume) {
            return Err(TopoError::WrongDim { id: volume, expected: Dim::Volume });
        }
        // 1. Edges whose ends lie strictly on opposite sides.
        let mut edges: Vec<EntityId> = self.volume_faces(volume).iter().flat_map(|f| self.face_edges(*f).to_vec()).collect();
        edges.sort();
        edges.dedup();
        let mut split_edges = Vec::new();
        for e in edges {
            let Some((a, b)) = self.edge_ends_of(e) else { continue };
            let (da, db) = (dist(self.point_of(a)?), dist(self.point_of(b)?));
            if !((da < -tol && db > tol) || (da > tol && db < -tol)) {
                continue;
            }
            let Carrier::Curve(cc) = self.carrier_of(e).clone() else { return Err(TopoError::Unsupported("plane cut of an edge without a curve")) };
            let curve = self.layer.shapes.curve(cc.curve).ok_or(TopoError::Geom(GeomError::Invalid("curve not in store")))?;
            let (mut lo, mut hi) = (cc.range.lo, cc.range.hi);
            let mut flo = dist(curve.eval(lo));
            for _ in 0..200 {
                let mid = 0.5 * (lo + hi);
                let fm = dist(curve.eval(mid));
                if (fm < 0.0) == (flo < 0.0) {
                    lo = mid;
                    flo = fm;
                } else {
                    hi = mid;
                }
            }
            let t = 0.5 * (lo + hi);
            let (v, halves) = self.partition_edge(e, t)?;
            split_edges.push((e, v, halves));
        }
        // 2. Faces with vertices strictly on both sides get a chord.
        let mut split_faces = Vec::new();
        let faces: Vec<EntityId> = self.volume_faces(volume).to_vec();
        for f in faces {
            let verts: Vec<EntityId> = self.loop_vertices(f, 0).collect();
            let ds: Vec<f64> = verts.iter().map(|v| self.point_of(*v).map(dist)).collect::<Result<_, _>>()?;
            let (neg, pos) = (ds.iter().any(|d| *d < -tol), ds.iter().any(|d| *d > tol));
            if !neg && !pos {
                return Err(TopoError::Unsupported("the cut plane contains a face of the volume"));
            }
            if !(neg && pos) {
                continue;
            }
            let on: Vec<EntityId> = verts.iter().zip(&ds).filter(|(_, d)| d.abs() <= tol).map(|(v, _)| *v).collect();
            if on.len() != 2 {
                return Err(TopoError::Unsupported("the plane must cross a cut face through exactly two of its vertices"));
            }
            let planar = match self.carrier_of(f) {
                Carrier::Surface(sc) => matches!(self.layer.shapes.surface_def(sc.surface), Some(SurfaceDef::Plane(_))),
                _ => false,
            };
            if !planar {
                return Err(TopoError::Unsupported("plane cut of a face that is not planar"));
            }
            let (pa, pb) = (self.point_of(on[0])?, self.point_of(on[1])?);
            let d = pb - pa;
            let chord_len = d.norm();
            let line = Line::new(pa, d)?;
            let halves = self.partition_face(f, VirtualCut { from: on[0], to: on[1], curve: CurveDef::Line(line), range: Interval::new(0.0, chord_len) })?;
            let chord = self.face_edges(halves[0]).iter().copied().find(|e| self.face_edges(halves[1]).contains(e)).ok_or(TopoError::Unsupported("internal: chord not shared"))?;
            // Order the halves [negative, positive].
            let side_of = |o: &Self, h: EntityId| -> Result<f64, TopoError> {
                let mut s = 0.0f64;
                for v in o.loop_vertices(h, 0) {
                    let d = dist(o.point_of(v)?);
                    if d.abs() > s.abs() {
                        s = d;
                    }
                }
                Ok(s)
            };
            let ordered = if side_of(self, halves[0])? < 0.0 { halves } else { [halves[1], halves[0]] };
            split_faces.push((f, chord, ordered));
        }
        // 3. The loop: edges on the plane separating a negative face from a positive one.
        let faces: Vec<EntityId> = self.volume_faces(volume).to_vec();
        let mut face_side: BTreeMap<EntityId, bool> = BTreeMap::new();
        for f in &faces {
            let mut neg = false;
            for v in self.loop_vertices(*f, 0) {
                neg |= dist(self.point_of(v)?) < -tol;
            }
            face_side.insert(*f, neg);
        }
        let mut cand: BTreeMap<EntityId, (EntityId, EntityId)> = BTreeMap::new();
        for f in &faces {
            for e in self.face_edges(*f) {
                if cand.contains_key(e) {
                    continue;
                }
                let Some((a, b)) = self.edge_ends_of(*e) else { continue };
                if dist(self.point_of(a)?).abs() > tol || dist(self.point_of(b)?).abs() > tol {
                    continue;
                }
                let users: Vec<bool> = faces.iter().filter(|g| self.face_edges(**g).contains(e)).map(|g| face_side[g]).collect();
                if users.contains(&true) && users.contains(&false) {
                    cand.insert(*e, (a, b));
                }
            }
        }
        let mut by_vertex: BTreeMap<EntityId, Vec<EntityId>> = BTreeMap::new();
        for (e, (a, b)) in &cand {
            by_vertex.entry(*a).or_default().push(*e);
            by_vertex.entry(*b).or_default().push(*e);
        }
        if cand.is_empty() || by_vertex.values().any(|v| v.len() != 2) {
            return Err(TopoError::Unsupported("the plane section is not one simple loop"));
        }
        let first = *cand.keys().next().ok_or(TopoError::Unsupported("empty plane section"))?;
        let mut ordered = vec![first];
        let mut at = cand[&first].1;
        while ordered.len() < cand.len() {
            let next = by_vertex[&at].iter().copied().find(|e| !ordered.contains(e)).ok_or(TopoError::Unsupported("the plane section is not one simple loop"))?;
            let (a, b) = cand[&next];
            at = if a == at { b } else { a };
            ordered.push(next);
        }
        if at != cand[&first].0 {
            return Err(TopoError::Unsupported("the plane section is not one simple loop"));
        }
        let seed = *faces.iter().find(|f| face_side[*f]).ok_or(TopoError::Unsupported("nothing on the negative side of the plane"))?;
        let frame = Frame::from_z(origin, n)?;
        let face = self.partition_volume(volume, VolumeCut { edges: ordered.clone(), surface: SurfaceDef::Plane(Plane { frame }), seed }, children)?;
        let mut loop_edges = ordered;
        loop_edges.sort();
        let mut loop_vertices: Vec<EntityId> = by_vertex.keys().copied().collect();
        loop_vertices.sort();
        Ok(PlaneCut { face, children, split_edges, split_faces, loop_edges, loop_vertices })
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
            VirtualOp::PartitionVolume { volume, cut, face, children } => self.apply_partition_volume(*volume, cut, *face, *children),
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

impl Overlay {
    fn apply_partition_volume(&mut self, volume: EntityId, cut: &VolumeCut, face: EntityId, children: [EntityId; 2]) -> Result<(), TopoError> {
        if self.dim_of(volume) != Some(Dim::Volume) {
            return Err(TopoError::WrongDim { id: volume, expected: Dim::Volume });
        }
        if !self.is_live(volume) {
            return Err(TopoError::Dead(volume));
        }
        for c in children {
            if c.kind() != EntityKind::PartitionCell || self.layer.cells.contains_key(&c) || self.layer.hidden.contains(&c) || self.host.dim_of(c).is_some() {
                return Err(TopoError::InvalidParameter("volume children must be fresh PartitionCell ids"));
            }
        }
        if children[0] >= children[1] {
            return Err(TopoError::InvalidParameter("volume children must ascend"));
        }
        let rows: Vec<Incidence> = self.down_of(volume).to_vec();
        if rows.iter().any(|r| r.ring != 0 || r.side == Side::Both) {
            return Err(TopoError::Unsupported("partition_volume of a volume with several shells or baffles"));
        }
        // Chain the loop head to tail.
        let n = cut.edges.len();
        if n == 0 {
            return Err(TopoError::InvalidParameter("a volume cut needs a loop of edges"));
        }
        let ends: Vec<(EntityId, EntityId)> = cut.edges.iter().map(|e| self.edge_ends_of(*e).ok_or(TopoError::WrongDim { id: *e, expected: Dim::Edge })).collect::<Result<_, _>>()?;
        let s0 = if n == 1 || ends[0].1 == ends[1].0 || ends[0].1 == ends[1].1 {
            Side::Left
        } else if ends[0].0 == ends[1].0 || ends[0].0 == ends[1].1 {
            Side::Right
        } else {
            return Err(TopoError::OpenLoop { ring: 0, at: 1 });
        };
        let mut sides = vec![s0];
        let (tail0, mut head) = if s0 == Side::Left { ends[0] } else { (ends[0].1, ends[0].0) };
        for (i, (a, b)) in ends.iter().enumerate().skip(1) {
            let s = if *a == head {
                head = *b;
                Side::Left
            } else if *b == head {
                head = *a;
                Side::Right
            } else {
                return Err(TopoError::OpenLoop { ring: 0, at: i });
            };
            sides.push(s);
        }
        if head != tail0 {
            return Err(TopoError::OpenLoop { ring: 0, at: n });
        }
        let loop_set: BTreeSet<EntityId> = cut.edges.iter().copied().collect();
        if loop_set.len() != n {
            return Err(TopoError::InvalidParameter("a loop edge is listed twice"));
        }
        let vfaces: Vec<EntityId> = rows.iter().map(|r| r.cell).collect();
        for e in &cut.edges {
            if !vfaces.iter().any(|f| self.face_edges(*f).contains(e)) {
                return Err(TopoError::InvalidParameter("a loop edge is not an edge of the volume"));
            }
        }
        if !vfaces.contains(&cut.seed) {
            return Err(TopoError::InvalidParameter("the seed is not a face of the volume"));
        }
        // Flood fill across the edges that are not on the loop.
        let fill = |o: &Self, start: EntityId, within: &[EntityId]| -> BTreeSet<EntityId> {
            let mut part = BTreeSet::from([start]);
            let mut stack = vec![start];
            while let Some(f) = stack.pop() {
                for e in o.face_edges(f).iter().filter(|e| !loop_set.contains(e)) {
                    for g in within {
                        if !part.contains(g) && o.face_edges(*g).contains(e) {
                            part.insert(*g);
                            stack.push(*g);
                        }
                    }
                }
            }
            part
        };
        let part_a = fill(self, cut.seed, &vfaces);
        let rest: Vec<EntityId> = vfaces.iter().copied().filter(|f| !part_a.contains(f)).collect();
        let Some(&b0) = rest.first() else { return Err(TopoError::Unsupported("the cut loop does not separate the volume")) };
        if fill(self, b0, &rest).len() != rest.len() {
            return Err(TopoError::Unsupported("the cut loop separates the volume into more than two parts"));
        }
        // Orientation: the normal points out of the first child.
        let tails: Vec<EntityId> = ends.iter().zip(&sides).map(|((a, b), s)| if *s == Side::Left { *a } else { *b }).collect();
        let pts: Vec<Point3> = tails.iter().map(|v| self.point_of(*v)).collect::<Result<_, _>>()?;
        let mut nrm = Vector3::default();
        let mut c = Vector3::default();
        for i in 0..pts.len() {
            let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
            nrm = nrm + Vector3::new((a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y));
            c = c + Vector3::new(a.x, a.y, a.z) * (1.0 / pts.len() as f64);
        }
        let centre = Point3::new(c.x, c.y, c.z);
        let loop_verts: BTreeSet<EntityId> = ends.iter().flat_map(|(a, b)| [*a, *b]).collect();
        let mut a_verts: BTreeSet<EntityId> = BTreeSet::new();
        for f in &part_a {
            a_verts.extend(self.loop_vertices(*f, 0).filter(|v| !loop_verts.contains(v)));
        }
        if a_verts.is_empty() || nrm.norm() == 0.0 {
            return Err(TopoError::Unsupported("cannot orient the cut face"));
        }
        let mut pa = Vector3::default();
        for v in &a_verts {
            let p = self.point_of(*v)?;
            pa = pa + Vector3::new(p.x, p.y, p.z) * (1.0 / a_verts.len() as f64);
        }
        let mut lp: Vec<Incidence> = cut.edges.iter().zip(&sides).map(|(e, s)| Incidence::new(*e, *s, 0)).collect();
        if nrm.dot(Point3::new(pa.x, pa.y, pa.z) - centre) > 0.0 {
            lp.reverse();
            for i in &mut lp {
                i.side = i.side.flip();
            }
            nrm = -nrm;
        }
        canonical_rotate(&mut lp);
        let surface = self.layer.shapes.add_surface(cut.surface.clone())?;
        let s = self.layer.shapes.surface(surface).ok_or(TopoError::Unsupported("the cut surface must be parametric"))?;
        let uv = s.project(centre, None)?.uv;
        let sense = if s.normal(uv)?.dot(nrm) < 0.0 { Sense::Reversed } else { Sense::Forward };
        // Commit.
        let parents: Vec<Incidence> = self.up_of(volume).to_vec();
        self.hide(volume, children[0]);
        self.add_virtual(face, Dim::Face, lp, Carrier::Surface(SurfaceCarrier { surface, sense }), vec![volume]);
        for (child, part_side) in [(children[0], Side::Left), (children[1], Side::Right)] {
            let mut row: Vec<Incidence> = rows.iter().copied().filter(|r| part_a.contains(&r.cell) == (part_side == Side::Left)).collect();
            row.push(Incidence::new(face, part_side, 0));
            row.sort_by_key(|e| (e.ring, e.cell, e.side));
            self.add_virtual(child, Dim::Volume, row, Carrier::None, vec![volume]);
        }
        for p in parents {
            let mut prow = self.down_of(p.cell).to_vec();
            prow.push(Incidence::new(children[0], p.side, p.ring));
            prow.push(Incidence::new(children[1], p.side, p.ring));
            prow.sort_by_key(|e| (e.ring, e.cell, e.side));
            self.set_down(p.cell, prow);
        }
        Ok(())
    }

    fn is_virtual_volume(&self, id: EntityId) -> bool {
        self.layer.cells.get(&id).is_some_and(|c| c.is_virtual && c.dim == Dim::Volume)
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
        if self.layer.hidden.contains(&volume) {
            return None;
        }
        if self.is_virtual_volume(volume) {
            return (ring == 0).then_some(ShellKind::Outer);
        }
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
        if self.layer.hidden.contains(&v) {
            return &[];
        }
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
        if self.layer.hidden.contains(&v) {
            return Err(QueryError::NoSuchCell(v));
        }
        if self.is_virtual_volume(v) {
            return Err(QueryError::Unsupported("moments of a virtual volume"));
        }
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
