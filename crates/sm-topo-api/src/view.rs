//! The read-only query contract, [`TopologyView`].
//!
//! Meshers, semantics (coloring, boundary kinds) and tracking see the complex only through this
//! trait. It is object-safe, returns borrowed slices and records (no per-call allocation on the hot
//! accessors), and never exposes mutation.

use crate::ids::*;
use crate::lineage::LineageLog;
use crate::model::*;
use sm_core::EntityKind;
use sm_geom_api::{ModelTolerance, Sense, ShapeStore};

/// Read-only view of a cell complex.
///
/// Conventions every implementor must honour:
/// - Entity ids are stable and never reused. Serials are dense `0..serial_count(kind)` for host
///   entities, so `vertex(VertexId::new(s))` works for every `s` in range, including retired ones
///   (their record stays, with `state != Active`).
/// - `bodies()`, `volumes()`, ... list *active* entities in ascending id order.
/// - `radial_ring`, `edge_coedges` and `vertex_uses` are canonical: their order depends only on
///   the model, not on the order of edits.
pub trait TopologyView: Send + Sync {
    /// The model's single global tolerance.
    fn tolerance(&self) -> &ModelTolerance;
    /// Geometry resolver for the shape handles stored in edges and faces.
    fn shapes(&self) -> &dyn ShapeStore;
    /// Number of serials ever allocated for a topology kind (active + retired).
    fn serial_count(&self, kind: EntityKind) -> u64;

    /// Active bodies, ascending.
    fn bodies(&self) -> &[BodyId];
    /// Active volumes, ascending.
    fn volumes(&self) -> &[VolumeId];
    /// Active faces, ascending.
    fn faces(&self) -> &[FaceId];
    /// Active edges, ascending.
    fn edges(&self) -> &[EdgeId];
    /// Active vertices, ascending.
    fn vertices(&self) -> &[VertexId];

    /// Body record (also for retired bodies).
    fn body(&self, id: BodyId) -> Option<&BodyRec>;
    /// Volume record.
    fn volume(&self, id: VolumeId) -> Option<&VolumeRec>;
    /// Face record.
    fn face(&self, id: FaceId) -> Option<&FaceRec>;
    /// Edge record.
    fn edge(&self, id: EdgeId) -> Option<&EdgeRec>;
    /// Vertex record.
    fn vertex(&self, id: VertexId) -> Option<&VertexRec>;

    /// The radial ring of an edge: every edge use around it, counter-clockwise about the edge's
    /// start-to-end direction. The two uses of one coedge (its face's two sides) are adjacent;
    /// between consecutive uses of different faces lies exactly one volume wedge (or exterior),
    /// owned by both neighbouring uses. Wire uses sit inside the wedge of their volume.
    /// The ring starts at the use of the smallest `(face, loop, index)` coedge.
    fn radial_ring(&self, e: EdgeId) -> &[EdgeUse];
    /// Coedges that reference an edge, ascending (each is two edge uses in the ring).
    fn edge_coedges(&self, e: EdgeId) -> &[CoedgeRef];
    /// Every occurrence of a vertex, ascending.
    fn vertex_uses(&self, v: VertexId) -> &[VertexUse];
    /// Imprint/merge status records, in the order they were first set.
    fn coincidences(&self) -> &[CoincidenceRecord];
    /// The lineage log.
    fn lineage(&self) -> &LineageLog;

    /// Shell record.
    fn shell(&self, r: ShellRef) -> Option<&ShellRec> {
        self.volume(r.volume)?.shells.get(usize::from(r.index))
    }

    /// Face use record.
    fn face_use(&self, fu: FaceUse) -> Option<&FaceUseRec> {
        Some(&self.face(fu.face)?.uses[fu.side.index()])
    }

    /// Coedge record.
    fn coedge(&self, c: CoedgeRef) -> Option<&CoedgeRec> {
        self.face(c.face)?.loops.get(usize::from(c.loop_index))?.coedges.get(c.index as usize)
    }

    /// What lies on a side of a face: a volume or the exterior.
    fn side_of(&self, fu: FaceUse) -> Option<Side> {
        Some(match self.face_use(fu)?.owner {
            UseOwner::Shell(s) => Side::Volume(s.volume),
            UseOwner::Exterior => Side::Exterior,
        })
    }

    /// The `(Negative, Positive)` sides of a face.
    fn face_sides(&self, f: FaceId) -> Option<[Side; 2]> {
        Some([self.side_of(FaceUse { face: f, side: FaceSide::Negative })?, self.side_of(FaceUse { face: f, side: FaceSide::Positive })?])
    }

    /// Role of a face: boundary, interface, baffle or sheet.
    fn face_class(&self, f: FaceId) -> Option<FaceClass> {
        let [n, p] = self.face_sides(f)?;
        Some(match (n, p) {
            (Side::Volume(a), Side::Volume(b)) if a == b => FaceClass::Baffle(a),
            (Side::Volume(a), Side::Volume(b)) => FaceClass::Interface(a, b),
            (Side::Volume(a), Side::Exterior) | (Side::Exterior, Side::Volume(a)) => FaceClass::Boundary(a),
            (Side::Exterior, Side::Exterior) => FaceClass::Sheet,
        })
    }

    /// Faces touched by a volume's shells, ascending, deduplicated.
    fn faces_of_volume(&self, v: VolumeId) -> Vec<FaceId> {
        let mut out: Vec<FaceId> = self.volume(v).into_iter().flat_map(|r| r.shells.iter()).flat_map(|s| s.face_uses.iter()).map(|fu| fu.face).collect();
        out.sort();
        out.dedup();
        out
    }

    /// Volumes that share at least one face with `v`, ascending.
    fn neighbors_of_volume(&self, v: VolumeId) -> Vec<VolumeId> {
        let mut out = Vec::new();
        for f in self.faces_of_volume(v) {
            if let Some(FaceClass::Interface(a, b)) = self.face_class(f) {
                out.push(if a == v { b } else { a });
            }
        }
        out.sort();
        out.dedup();
        out
    }

    /// The volume (or exterior) that owns an edge use.
    fn edge_use_side(&self, eu: EdgeUse) -> Option<Side> {
        match eu {
            EdgeUse::Coedge { coedge, side } => self.side_of(FaceUse { face: coedge.face, side }),
            EdgeUse::Wire { volume, .. } => Some(volume.map_or(Side::Exterior, Side::Volume)),
        }
    }

    /// Status record for a face pair (order-insensitive).
    fn coincidence_of(&self, a: FaceId, b: FaceId) -> Option<&CoincidenceRecord> {
        self.coincidences().iter().find(|r| (r.a == a && r.b == b) || (r.a == b && r.b == a))
    }
}

/// Start and end vertex of a coedge in its stored traversal direction.
pub fn coedge_ends(view: &dyn TopologyView, c: &CoedgeRec) -> Option<(VertexId, VertexId)> {
    match c.kind {
        CoedgeKind::Pole { vertex } => Some((vertex, vertex)),
        CoedgeKind::Edge { edge, sense } => {
            let e = view.edge(edge)?;
            Some(match sense {
                Sense::Forward => (e.start, e.end),
                Sense::Reversed => (e.end, e.start),
            })
        }
    }
}

/// The edge uses of a loop use, in traversal order. The `Positive` side traverses the stored
/// loop backwards.
pub fn loop_use_edge_uses(face: FaceId, rec: &FaceRec, side: FaceSide, loop_index: u16) -> Vec<EdgeUse> {
    let Some(l) = rec.loops.get(usize::from(loop_index)) else { return Vec::new() };
    let mk = |i: usize| EdgeUse::Coedge { coedge: CoedgeRef { face, loop_index, index: i as u32 }, side };
    match side {
        FaceSide::Negative => (0..l.coedges.len()).map(mk).collect(),
        FaceSide::Positive => (0..l.coedges.len()).rev().map(mk).collect(),
    }
}

/// Tail vertex of an edge use (where traversal of that use starts).
pub fn edge_use_tail(view: &dyn TopologyView, eu: EdgeUse) -> Option<VertexId> {
    match eu {
        EdgeUse::Coedge { coedge, side } => {
            let (s, e) = coedge_ends(view, view.coedge(coedge)?)?;
            Some(if side == FaceSide::Negative { s } else { e })
        }
        EdgeUse::Wire { edge, .. } => Some(view.edge(edge)?.start),
    }
}

/// `V - E + F` over active entities. For a single closed genus-0 shell this is 2; pole coedges are
/// not edges and do not count.
pub fn euler_characteristic(view: &dyn TopologyView) -> i64 {
    view.vertices().len() as i64 - view.edges().len() as i64 + view.faces().len() as i64
}

/// Platform-independent hash of the whole complex: ids, states, topology, quantized geometry
/// (vertex coordinates on `tolerance().hash_quantum()`), coincidence records and lineage. Equal
/// builds hash equal, regardless of thread count or allocation order.
pub fn state_hash(view: &dyn TopologyView) -> u64 {
    let mut h = StableHasher::default();
    let tol = view.tolerance();
    h.write_u64(tol.resolution.to_bits());
    h.write_u64(tol.angular.to_bits());
    let q = tol.hash_quantum();
    let state = |h: &mut StableHasher, s: EntityState| match s {
        EntityState::Active => h.write_u64(0),
        EntityState::Split(o) => {
            h.write_u64(1);
            h.write_u64(o.0);
        }
        EntityState::Merged(o) => {
            h.write_u64(2);
            h.write_u64(o.0);
        }
        EntityState::Deleted(o) => {
            h.write_u64(3);
            h.write_u64(o.0);
        }
    };
    let n = view.serial_count(EntityKind::TopoVertex);
    h.write_u64(n);
    for s in 0..n {
        if let Some(v) = VertexId::new(s).and_then(|id| view.vertex(id)) {
            state(&mut h, v.state);
            h.write_quantized(v.point.x, q);
            h.write_quantized(v.point.y, q);
            h.write_quantized(v.point.z, q);
        }
    }
    let n = view.serial_count(EntityKind::TopoEdge);
    h.write_u64(n);
    for s in 0..n {
        if let Some(e) = EdgeId::new(s).and_then(|id| view.edge(id)) {
            state(&mut h, e.state);
            h.write_u64(e.curve.to_raw());
            h.write_quantized(e.t0, q);
            h.write_quantized(e.t1, q);
            h.write_u64(e.start.serial());
            h.write_u64(e.end.serial());
        }
    }
    let n = view.serial_count(EntityKind::TopoFace);
    h.write_u64(n);
    for s in 0..n {
        if let Some(f) = FaceId::new(s).and_then(|id| view.face(id)) {
            state(&mut h, f.state);
            h.write_u64(f.surface.to_raw());
            h.write_u64(u64::from(f.surface_sense == Sense::Reversed));
            h.write_u64(f.loops.len() as u64);
            for l in &f.loops {
                h.write_u64(u64::from(l.kind == LoopKind::Inner));
                h.write_u64(l.coedges.len() as u64);
                for c in &l.coedges {
                    match c.kind {
                        CoedgeKind::Edge { edge, sense } => {
                            h.write_u64(1);
                            h.write_u64(edge.serial());
                            h.write_u64(u64::from(sense == Sense::Reversed));
                        }
                        CoedgeKind::Pole { vertex } => {
                            h.write_u64(2);
                            h.write_u64(vertex.serial());
                        }
                    }
                    h.write_u64(u64::from(c.pcurve.is_some()));
                }
            }
            for u in &f.uses {
                match u.owner {
                    UseOwner::Exterior => h.write_u64(0),
                    UseOwner::Shell(r) => {
                        h.write_u64(1);
                        h.write_u64(r.volume.serial());
                        h.write_u64(u64::from(r.index));
                    }
                }
            }
        }
    }
    let n = view.serial_count(EntityKind::TopoVolume);
    h.write_u64(n);
    for s in 0..n {
        if let Some(v) = VolumeId::new(s).and_then(|id| view.volume(id)) {
            state(&mut h, v.state);
            h.write_u64(v.body.serial());
            for sh in &v.shells {
                h.write_u64(sh.kind as u64);
                for fu in &sh.face_uses {
                    h.write_u64(fu.face.serial());
                    h.write_u64(fu.side.index() as u64);
                }
                h.write_u64(u64::MAX);
                sh.wires.iter().for_each(|w| h.write_u64(w.serial()));
                h.write_u64(u64::MAX);
                sh.points.iter().for_each(|p| h.write_u64(p.serial()));
                h.write_u64(u64::MAX - 1);
            }
        }
    }
    let n = view.serial_count(EntityKind::TopoBody);
    h.write_u64(n);
    for s in 0..n {
        if let Some(b) = BodyId::new(s).and_then(|id| view.body(id)) {
            state(&mut h, b.state);
            b.volumes.iter().for_each(|x| h.write_u64(x.serial()));
            h.write_u64(u64::MAX);
            b.sheets.iter().for_each(|x| h.write_u64(x.serial()));
            h.write_u64(u64::MAX);
            b.wires.iter().for_each(|x| h.write_u64(x.serial()));
            h.write_u64(u64::MAX);
            b.points.iter().for_each(|x| h.write_u64(x.serial()));
        }
    }
    for c in view.coincidences() {
        h.write_u64(c.a.serial());
        h.write_u64(c.b.serial());
        h.write_u64(c.status as u64);
        h.write_u64(c.op.0);
        h.write_quantized(c.overlap_area.unwrap_or(-1.0), q);
    }
    for r in view.lineage().records() {
        h.write_u64(r.op.0);
        h.write_u64(u64::from(r.seq));
        use crate::lineage::LineageEvent as E;
        match &r.event {
            E::Created { entity } => {
                h.write_u64(1);
                h.write_u64(entity.to_raw());
            }
            E::Split { parent, children } => {
                h.write_u64(2);
                h.write_u64(parent.to_raw());
                children.iter().for_each(|c| h.write_u64(c.to_raw()));
            }
            E::Merged { survivor, absorbed, gap } => {
                h.write_u64(3);
                h.write_u64(survivor.to_raw());
                h.write_u64(absorbed.to_raw());
                h.write_quantized(*gap, q);
            }
            E::Deleted { entity } => {
                h.write_u64(4);
                h.write_u64(entity.to_raw());
            }
        }
    }
    h.finish()
}
