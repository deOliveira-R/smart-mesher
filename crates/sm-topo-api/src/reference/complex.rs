//! [`RefComplex`]: the in-memory reference implementation of [`TopologyView`] and [`TopologyEdit`].
//!
//! Storage is a dense `Vec` per kind indexed by serial (ids are allocated sequentially and never
//! reused, so the serial *is* the index). Derived indices (coedge lists, vertex uses, radial rings)
//! are rebuilt from scratch by `end_op` in canonical order, which makes them independent of the
//! order of edits. This favours obviousness over speed; a production store would update them
//! incrementally and keep the same observable behaviour.

use super::ring::build_ring;
use crate::edit::*;
use crate::error::TopoError;
use crate::ids::*;
use crate::lineage::{LineageEvent, LineageLog};
use crate::model::*;
use crate::view::{TopologyView, coedge_ends};
use sm_core::{EntityId, EntityKind, Point3};
use sm_geom_api::{ModelTolerance, Sense, ShapeLibrary, ShapeStore};

#[derive(Clone, Debug)]
struct OpState {
    id: OpId,
    created: Vec<EntityId>,
    retired: Vec<EntityId>,
}

/// Reference in-memory cell complex. `Clone` is cheap enough for snapshots in tests.
#[derive(Clone, Debug)]
pub struct RefComplex {
    tol: ModelTolerance,
    shapes: ShapeLibrary,
    vertices: Vec<VertexRec>,
    edges: Vec<EdgeRec>,
    faces: Vec<FaceRec>,
    volumes: Vec<VolumeRec>,
    bodies: Vec<BodyRec>,
    act_vertices: Vec<VertexId>,
    act_edges: Vec<EdgeId>,
    act_faces: Vec<FaceId>,
    act_volumes: Vec<VolumeId>,
    act_bodies: Vec<BodyId>,
    rings: Vec<Vec<EdgeUse>>,
    edge_coedges: Vec<Vec<CoedgeRef>>,
    vertex_uses: Vec<Vec<VertexUse>>,
    coincidences: Vec<CoincidenceRecord>,
    lineage: LineageLog,
    op: Option<OpState>,
    last_op: Option<OpId>,
}

fn remove_sorted<T: Ord + Copy>(v: &mut Vec<T>, x: T) {
    if let Ok(i) = v.binary_search(&x) {
        v.remove(i);
    }
}

fn dedup_push<T: PartialEq>(v: &mut Vec<T>, x: T) {
    if !v.contains(&x) {
        v.push(x);
    }
}

impl RefComplex {
    /// Empty complex with the given tolerance and shape library.
    pub fn new(tol: ModelTolerance, shapes: ShapeLibrary) -> Self {
        Self {
            tol,
            shapes,
            vertices: Vec::new(),
            edges: Vec::new(),
            faces: Vec::new(),
            volumes: Vec::new(),
            bodies: Vec::new(),
            act_vertices: Vec::new(),
            act_edges: Vec::new(),
            act_faces: Vec::new(),
            act_volumes: Vec::new(),
            act_bodies: Vec::new(),
            rings: Vec::new(),
            edge_coedges: Vec::new(),
            vertex_uses: Vec::new(),
            coincidences: Vec::new(),
            lineage: LineageLog::new(),
            op: None,
            last_op: None,
        }
    }

    /// The concrete shape library.
    pub fn library(&self) -> &ShapeLibrary {
        &self.shapes
    }

    /// Mutable access to the shape library (shapes are appended by ops before topology refers to them).
    pub fn library_mut(&mut self) -> &mut ShapeLibrary {
        &mut self.shapes
    }

    pub(crate) fn edges_raw(&self) -> &[EdgeRec] {
        &self.edges
    }

    pub(crate) fn faces_raw(&self) -> &[FaceRec] {
        &self.faces
    }

    fn op_id(&self) -> Result<OpId, TopoError> {
        self.op.as_ref().map(|o| o.id).ok_or(TopoError::NoOpenOp)
    }

    fn created(&mut self, e: impl Into<EntityId>) -> Result<(), TopoError> {
        let e = e.into();
        let id = self.op_id()?;
        self.lineage.push(id, LineageEvent::Created { entity: e });
        if let Some(o) = self.op.as_mut() {
            o.created.push(e);
        }
        Ok(())
    }

    fn retired(&mut self, e: impl Into<EntityId>) {
        if let Some(o) = self.op.as_mut() {
            o.retired.push(e.into());
        }
    }

    fn active_vertex(&self, id: VertexId) -> Result<&VertexRec, TopoError> {
        let r = self.vertices.get(id.serial() as usize).ok_or(TopoError::NotFound(id.entity()))?;
        if r.state.is_active() { Ok(r) } else { Err(TopoError::NotActive(id.entity())) }
    }

    fn active_edge(&self, id: EdgeId) -> Result<&EdgeRec, TopoError> {
        let r = self.edges.get(id.serial() as usize).ok_or(TopoError::NotFound(id.entity()))?;
        if r.state.is_active() { Ok(r) } else { Err(TopoError::NotActive(id.entity())) }
    }

    fn active_face(&self, id: FaceId) -> Result<&FaceRec, TopoError> {
        let r = self.faces.get(id.serial() as usize).ok_or(TopoError::NotFound(id.entity()))?;
        if r.state.is_active() { Ok(r) } else { Err(TopoError::NotActive(id.entity())) }
    }

    fn active_volume(&self, id: VolumeId) -> Result<&VolumeRec, TopoError> {
        let r = self.volumes.get(id.serial() as usize).ok_or(TopoError::NotFound(id.entity()))?;
        if r.state.is_active() { Ok(r) } else { Err(TopoError::NotActive(id.entity())) }
    }

    fn active_body(&self, id: BodyId) -> Result<&BodyRec, TopoError> {
        let r = self.bodies.get(id.serial() as usize).ok_or(TopoError::NotFound(id.entity()))?;
        if r.state.is_active() { Ok(r) } else { Err(TopoError::NotActive(id.entity())) }
    }

    fn shell_mut(&mut self, r: ShellRef) -> Result<&mut ShellRec, TopoError> {
        self.active_volume(r.volume)?;
        self.volumes[r.volume.serial() as usize].shells.get_mut(usize::from(r.index)).ok_or(TopoError::NotFound(r.volume.entity()))
    }

    fn check_loops(&self, loops: &[LoopRec]) -> Result<(), TopoError> {
        for (i, l) in loops.iter().enumerate() {
            if l.coedges.is_empty() {
                return Err(TopoError::InvalidLoop(format!("loop {i} is empty")));
            }
            if (i == 0) != (l.kind == LoopKind::Outer) {
                return Err(TopoError::InvalidLoop("exactly the first loop must be Outer".into()));
            }
            let mut ends = Vec::new();
            for c in &l.coedges {
                match c.kind {
                    CoedgeKind::Edge { edge, .. } => {
                        self.active_edge(edge)?;
                    }
                    CoedgeKind::Pole { vertex } => {
                        self.active_vertex(vertex)?;
                    }
                }
                ends.push(coedge_ends(self, c).ok_or_else(|| TopoError::InvalidLoop("dangling coedge".into()))?);
            }
            if !(0..ends.len()).all(|k| ends[k].1 == ends[(k + 1) % ends.len()].0) {
                return Err(TopoError::InvalidLoop(format!("loop {i} is not a closed chain")));
            }
        }
        Ok(())
    }

    fn upsert_coincidence(&mut self, rec: CoincidenceRecord) {
        let same = |r: &CoincidenceRecord| (r.a == rec.a && r.b == rec.b) || (r.a == rec.b && r.b == rec.a);
        if let Some(slot) = self.coincidences.iter_mut().find(|r| same(r)) {
            *slot = rec;
        } else {
            self.coincidences.push(rec);
        }
    }

    /// Replace every occurrence of a face use in shells by `with` (in place, preserving order).
    fn replace_face_use(&mut self, old: FaceUse, with: &[FaceUse]) {
        for v in &mut self.volumes {
            for s in &mut v.shells {
                if let Some(i) = s.face_uses.iter().position(|x| *x == old) {
                    s.face_uses.splice(i..=i, with.iter().copied());
                }
            }
        }
    }

    fn rebuild_derived(&mut self) {
        let ne = self.edges.len();
        self.edge_coedges = vec![Vec::new(); ne];
        self.vertex_uses = vec![Vec::new(); self.vertices.len()];
        for &fid in &self.act_faces {
            let f = &self.faces[fid.serial() as usize];
            for (li, l) in f.loops.iter().enumerate() {
                for (ci, c) in l.coedges.iter().enumerate() {
                    let cr = CoedgeRef { face: fid, loop_index: li as u16, index: ci as u32 };
                    if let CoedgeKind::Edge { edge, .. } = c.kind {
                        self.edge_coedges[edge.serial() as usize].push(cr);
                    }
                    if let Some((s, e)) = coedge_ends(self, c) {
                        for side in FaceSide::BOTH {
                            let tail = if side == FaceSide::Negative { s } else { e };
                            self.vertex_uses[tail.serial() as usize].push(VertexUse { vertex: tail, site: VertexSite::Corner { coedge: cr, side } });
                        }
                    }
                }
            }
        }
        let mut extra: Vec<VertexUse> = Vec::new();
        let wire_ends =|edge: EdgeId, vols: &mut Vec<VertexUse>| {
            let e = &self.edges[edge.serial() as usize];
            vols.push(VertexUse { vertex: e.start, site: VertexSite::WireEnd { edge, at_end: false } });
            vols.push(VertexUse { vertex: e.end, site: VertexSite::WireEnd { edge, at_end: true } });
        };
        for &vid in &self.act_volumes {
            for s in &self.volumes[vid.serial() as usize].shells {
                s.wires.iter().for_each(|&w| wire_ends(w, &mut extra));
                extra.extend(s.points.iter().map(|&p| VertexUse { vertex: p, site: VertexSite::Lone { volume: Some(vid) } }));
            }
        }
        for &bid in &self.act_bodies {
            let b = &self.bodies[bid.serial() as usize];
            b.wires.iter().for_each(|&w| wire_ends(w, &mut extra));
            extra.extend(b.points.iter().map(|&p| VertexUse { vertex: p, site: VertexSite::Lone { volume: None } }));
        }
        for u in extra {
            self.vertex_uses[u.vertex.serial() as usize].push(u);
        }
        for v in &mut self.vertex_uses {
            v.sort();
        }
        let mut rings = vec![Vec::new(); ne];
        for &eid in &self.act_edges {
            rings[eid.serial() as usize] = build_ring(self, eid);
        }
        self.rings = rings;
    }
}

impl TopologyView for RefComplex {
    fn tolerance(&self) -> &ModelTolerance {
        &self.tol
    }
    fn shapes(&self) -> &dyn ShapeStore {
        &self.shapes
    }
    fn serial_count(&self, kind: EntityKind) -> u64 {
        (match kind {
            EntityKind::TopoVertex => self.vertices.len(),
            EntityKind::TopoEdge => self.edges.len(),
            EntityKind::TopoFace => self.faces.len(),
            EntityKind::TopoVolume => self.volumes.len(),
            EntityKind::TopoBody => self.bodies.len(),
            _ => 0,
        }) as u64
    }
    fn bodies(&self) -> &[BodyId] {
        &self.act_bodies
    }
    fn volumes(&self) -> &[VolumeId] {
        &self.act_volumes
    }
    fn faces(&self) -> &[FaceId] {
        &self.act_faces
    }
    fn edges(&self) -> &[EdgeId] {
        &self.act_edges
    }
    fn vertices(&self) -> &[VertexId] {
        &self.act_vertices
    }
    fn body(&self, id: BodyId) -> Option<&BodyRec> {
        self.bodies.get(id.serial() as usize)
    }
    fn volume(&self, id: VolumeId) -> Option<&VolumeRec> {
        self.volumes.get(id.serial() as usize)
    }
    fn face(&self, id: FaceId) -> Option<&FaceRec> {
        self.faces.get(id.serial() as usize)
    }
    fn edge(&self, id: EdgeId) -> Option<&EdgeRec> {
        self.edges.get(id.serial() as usize)
    }
    fn vertex(&self, id: VertexId) -> Option<&VertexRec> {
        self.vertices.get(id.serial() as usize)
    }
    fn radial_ring(&self, e: EdgeId) -> &[EdgeUse] {
        self.rings.get(e.serial() as usize).map_or(&[], Vec::as_slice)
    }
    fn edge_coedges(&self, e: EdgeId) -> &[CoedgeRef] {
        self.edge_coedges.get(e.serial() as usize).map_or(&[], Vec::as_slice)
    }
    fn vertex_uses(&self, v: VertexId) -> &[VertexUse] {
        self.vertex_uses.get(v.serial() as usize).map_or(&[], Vec::as_slice)
    }
    fn coincidences(&self) -> &[CoincidenceRecord] {
        &self.coincidences
    }
    fn lineage(&self) -> &LineageLog {
        &self.lineage
    }
}

fn param_in_range(t: f64, t0: f64, t1: f64, period: Option<f64>, eps: f64) -> bool {
    match period {
        Some(p) => {
            let d = (t - t0).rem_euclid(p);
            d <= (t1 - t0) + eps || p - d <= eps
        }
        None => t >= t0 - eps && t <= t1 + eps,
    }
}

impl TopologyEdit for RefComplex {
    fn begin_op(&mut self, op: OpId) -> Result<(), TopoError> {
        if self.op.is_some() || self.last_op.is_some_and(|l| op <= l) {
            return Err(TopoError::BadOp(op));
        }
        self.op = Some(OpState { id: op, created: Vec::new(), retired: Vec::new() });
        Ok(())
    }

    fn end_op(&mut self) -> Result<OpSummary, TopoError> {
        let o = self.op.take().ok_or(TopoError::NoOpenOp)?;
        self.last_op = Some(o.id);
        self.rebuild_derived();
        Ok(OpSummary { op: o.id, created: o.created, retired: o.retired })
    }

    fn make_body(&mut self) -> Result<BodyId, TopoError> {
        self.op_id()?;
        let id = BodyId::new(self.bodies.len() as u64).ok_or(TopoError::IdSpaceExhausted(EntityKind::TopoBody))?;
        self.bodies.push(BodyRec { volumes: vec![], sheets: vec![], wires: vec![], points: vec![], state: EntityState::Active });
        self.act_bodies.push(id);
        self.created(id)?;
        Ok(id)
    }

    fn make_vertex(&mut self, point: Point3) -> Result<VertexId, TopoError> {
        self.op_id()?;
        let id = VertexId::new(self.vertices.len() as u64).ok_or(TopoError::IdSpaceExhausted(EntityKind::TopoVertex))?;
        self.vertices.push(VertexRec { point, state: EntityState::Active });
        self.act_vertices.push(id);
        self.created(id)?;
        Ok(id)
    }

    fn make_edge(&mut self, spec: EdgeSpec) -> Result<EdgeId, TopoError> {
        self.op_id()?;
        let (sp, ep) = (self.active_vertex(spec.start)?.point, self.active_vertex(spec.end)?.point);
        let curve = self.shapes.curve(spec.curve).ok_or(TopoError::UnknownShape(spec.curve))?;
        if spec.t1 <= spec.t0 {
            return Err(TopoError::InvalidLoop("edge parameter range is empty".into()));
        }
        let (a, b) = (curve.eval(spec.t0), curve.eval(spec.t1));
        if (a - sp).norm() > self.tol.resolution || (b - ep).norm() > self.tol.resolution {
            return Err(TopoError::NotCoincident("edge end points differ from its vertices".into()));
        }
        let id = EdgeId::new(self.edges.len() as u64).ok_or(TopoError::IdSpaceExhausted(EntityKind::TopoEdge))?;
        self.edges.push(EdgeRec { curve: spec.curve, t0: spec.t0, t1: spec.t1, start: spec.start, end: spec.end, state: EntityState::Active });
        self.act_edges.push(id);
        self.created(id)?;
        Ok(id)
    }

    fn make_face(&mut self, spec: FaceSpec) -> Result<FaceId, TopoError> {
        self.op_id()?;
        if self.shapes.surface(spec.surface).is_none() && self.shapes.implicit(spec.surface).is_none() {
            return Err(TopoError::UnknownShape(spec.surface));
        }
        self.check_loops(&spec.loops)?;
        let id = FaceId::new(self.faces.len() as u64).ok_or(TopoError::IdSpaceExhausted(EntityKind::TopoFace))?;
        let uses = FaceSide::BOTH.map(|s| FaceUseRec { sense: s.sense(), owner: UseOwner::Exterior });
        self.faces.push(FaceRec { surface: spec.surface, surface_sense: spec.surface_sense, loops: spec.loops, uses, state: EntityState::Active });
        self.act_faces.push(id);
        self.created(id)?;
        Ok(id)
    }

    fn make_volume(&mut self, body: BodyId) -> Result<VolumeId, TopoError> {
        self.op_id()?;
        self.active_body(body)?;
        let id = VolumeId::new(self.volumes.len() as u64).ok_or(TopoError::IdSpaceExhausted(EntityKind::TopoVolume))?;
        self.volumes.push(VolumeRec { body, shells: vec![], state: EntityState::Active });
        self.act_volumes.push(id);
        self.bodies[body.serial() as usize].volumes.push(id);
        self.created(id)?;
        Ok(id)
    }

    fn make_shell(&mut self, volume: VolumeId, kind: ShellKind) -> Result<ShellRef, TopoError> {
        self.op_id()?;
        let v = self.active_volume(volume)?;
        if kind == ShellKind::Outer && v.shells.iter().any(|s| s.kind == ShellKind::Outer) {
            return Err(TopoError::Unsupported("a volume has at most one outer shell"));
        }
        let index = u16::try_from(v.shells.len()).map_err(|_| TopoError::Unsupported("too many shells"))?;
        self.volumes[volume.serial() as usize].shells.push(ShellRec { kind, face_uses: vec![], wires: vec![], points: vec![] });
        Ok(ShellRef { volume, index })
    }

    fn bind_face_use(&mut self, fu: FaceUse, shell: ShellRef) -> Result<(), TopoError> {
        self.op_id()?;
        if self.active_face(fu.face)?.uses[fu.side.index()].owner != UseOwner::Exterior {
            return Err(TopoError::SideOccupied(fu));
        }
        self.shell_mut(shell)?.face_uses.push(fu);
        self.faces[fu.face.serial() as usize].uses[fu.side.index()].owner = UseOwner::Shell(shell);
        for b in &mut self.bodies {
            b.sheets.retain(|&f| f != fu.face);
        }
        Ok(())
    }

    fn release_face_use(&mut self, fu: FaceUse) -> Result<(), TopoError> {
        self.op_id()?;
        let UseOwner::Shell(sr) = self.active_face(fu.face)?.uses[fu.side.index()].owner else { return Err(TopoError::NotBound(fu)) };
        self.shell_mut(sr)?.face_uses.retain(|x| *x != fu);
        self.faces[fu.face.serial() as usize].uses[fu.side.index()].owner = UseOwner::Exterior;
        Ok(())
    }

    fn add_wire_edge(&mut self, shell: ShellRef, edge: EdgeId) -> Result<(), TopoError> {
        self.op_id()?;
        self.active_edge(edge)?;
        dedup_push(&mut self.shell_mut(shell)?.wires, edge);
        Ok(())
    }

    fn add_lone_vertex(&mut self, shell: ShellRef, vertex: VertexId) -> Result<(), TopoError> {
        self.op_id()?;
        self.active_vertex(vertex)?;
        dedup_push(&mut self.shell_mut(shell)?.points, vertex);
        Ok(())
    }

    fn add_sheet(&mut self, body: BodyId, face: FaceId) -> Result<(), TopoError> {
        self.op_id()?;
        self.active_body(body)?;
        let f = self.active_face(face)?;
        if f.uses.iter().any(|u| u.owner != UseOwner::Exterior) {
            return Err(TopoError::SideOccupied(FaceUse { face, side: FaceSide::Negative }));
        }
        dedup_push(&mut self.bodies[body.serial() as usize].sheets, face);
        Ok(())
    }

    fn split_edge(&mut self, edge: EdgeId, at: SplitAt) -> Result<EdgeSplit, TopoError> {
        let op = self.op_id()?;
        let e = self.active_edge(edge)?.clone();
        let t = match at {
            SplitAt::NewVertex(t) | SplitAt::Vertex { t, .. } => t,
        };
        let span = e.t1 - e.t0;
        if t <= e.t0 + 1e-12 * span.abs() || t >= e.t1 - 1e-12 * span.abs() {
            return Err(TopoError::InvalidLoop("split parameter is not strictly inside the edge".into()));
        }
        let p = self.shapes.curve(e.curve).ok_or(TopoError::UnknownShape(e.curve))?.eval(t);
        let vertex = match at {
            SplitAt::NewVertex(_) => self.make_vertex(p)?,
            SplitAt::Vertex { vertex, .. } => {
                if (self.active_vertex(vertex)?.point - p).norm() > self.tol.resolution {
                    return Err(TopoError::NotCoincident("vertex is not on the curve at the split parameter".into()));
                }
                vertex
            }
        };
        let first = EdgeId::new(self.edges.len() as u64).ok_or(TopoError::IdSpaceExhausted(EntityKind::TopoEdge))?;
        let second = EdgeId::new(self.edges.len() as u64 + 1).ok_or(TopoError::IdSpaceExhausted(EntityKind::TopoEdge))?;
        self.edges.push(EdgeRec { curve: e.curve, t0: e.t0, t1: t, start: e.start, end: vertex, state: EntityState::Active });
        self.edges.push(EdgeRec { curve: e.curve, t0: t, t1: e.t1, start: vertex, end: e.end, state: EntityState::Active });
        self.act_edges.push(first);
        self.act_edges.push(second);
        self.created(first)?;
        self.created(second)?;
        self.edges[edge.serial() as usize].state = EntityState::Split(op);
        remove_sorted(&mut self.act_edges, edge);
        self.lineage.push(op, LineageEvent::Split { parent: edge.entity(), children: vec![first.entity(), second.entity()] });
        self.retired(edge);

        let frac = (t - e.t0) / span;
        for &fid in &self.act_faces.clone() {
            for l in &mut self.faces[fid.serial() as usize].loops {
                let mut out = Vec::with_capacity(l.coedges.len() + 1);
                for c in l.coedges.drain(..) {
                    match c.kind {
                        CoedgeKind::Edge { edge: ce, sense } if ce == edge => {
                            let cut = |pc: &Option<sm_geom_api::PCurve>, lo: f64, hi: f64| match pc {
                                Some(sm_geom_api::PCurve::Segment { start, end }) => {
                                    let at = |s: f64| [start[0] + s * (end[0] - start[0]), start[1] + s * (end[1] - start[1])];
                                    Some(sm_geom_api::PCurve::Segment { start: at(lo), end: at(hi) })
                                }
                                _ => None,
                            };
                            match sense {
                                Sense::Forward => {
                                    out.push(CoedgeRec { kind: CoedgeKind::Edge { edge: first, sense }, pcurve: cut(&c.pcurve, 0.0, frac) });
                                    out.push(CoedgeRec { kind: CoedgeKind::Edge { edge: second, sense }, pcurve: cut(&c.pcurve, frac, 1.0) });
                                }
                                Sense::Reversed => {
                                    out.push(CoedgeRec { kind: CoedgeKind::Edge { edge: second, sense }, pcurve: cut(&c.pcurve, 0.0, 1.0 - frac) });
                                    out.push(CoedgeRec { kind: CoedgeKind::Edge { edge: first, sense }, pcurve: cut(&c.pcurve, 1.0 - frac, 1.0) });
                                }
                            }
                        }
                        _ => out.push(c),
                    }
                }
                l.coedges = out;
            }
        }
        for v in &mut self.volumes {
            for s in &mut v.shells {
                if let Some(i) = s.wires.iter().position(|&w| w == edge) {
                    s.wires.splice(i..=i, [first, second]);
                }
            }
        }
        for b in &mut self.bodies {
            if let Some(i) = b.wires.iter().position(|&w| w == edge) {
                b.wires.splice(i..=i, [first, second]);
            }
        }
        Ok(EdgeSplit { vertex, first, second })
    }

    fn split_face(&mut self, face: FaceId, cut: FaceCut) -> Result<FaceSplit, TopoError> {
        let op = self.op_id()?;
        let f = self.active_face(face)?.clone();
        let outer = f.loops.first().ok_or_else(|| TopoError::InvalidLoop("face has no loops".into()))?;
        let n = outer.coedges.len() as u32;
        if cut.from >= cut.to || cut.to >= n {
            return Err(TopoError::InvalidLoop("cut corners must satisfy from < to < loop length".into()));
        }
        let tail = |i: u32| coedge_ends(self, &outer.coedges[i as usize]).map(|p| p.0).ok_or_else(|| TopoError::InvalidLoop("dangling coedge".into()));
        let (vi, vj) = (tail(cut.from)?, tail(cut.to)?);
        if vi == vj {
            return Err(TopoError::Unsupported("cut between identical corner vertices"));
        }
        let ce = self.active_edge(cut.edge)?.clone();
        let fwd_is_ij = if (ce.start, ce.end) == (vi, vj) {
            true
        } else if (ce.start, ce.end) == (vj, vi) {
            false
        } else {
            return Err(TopoError::InvalidLoop("cut edge does not join the two corners".into()));
        };
        let n_inner = f.loops.len() as u16;
        if cut.inner_loops_to_first.iter().any(|&i| i == 0 || i >= n_inner) {
            return Err(TopoError::InvalidLoop("inner loop index out of range".into()));
        }
        let cedge = |sense: Sense| CoedgeRec { kind: CoedgeKind::Edge { edge: cut.edge, sense }, pcurve: None };
        let mut a: Vec<CoedgeRec> = outer.coedges[cut.from as usize..cut.to as usize].to_vec();
        a.push(cedge(if fwd_is_ij { Sense::Reversed } else { Sense::Forward }));
        let mut b: Vec<CoedgeRec> = outer.coedges[cut.to as usize..].to_vec();
        b.extend_from_slice(&outer.coedges[..cut.from as usize]);
        b.push(cedge(if fwd_is_ij { Sense::Forward } else { Sense::Reversed }));
        let mut la = vec![LoopRec { kind: LoopKind::Outer, coedges: a }];
        let mut lb = vec![LoopRec { kind: LoopKind::Outer, coedges: b }];
        for (i, l) in f.loops.iter().enumerate().skip(1) {
            if cut.inner_loops_to_first.contains(&(i as u16)) { la.push(l.clone()) } else { lb.push(l.clone()) }
        }
        self.check_loops(&la)?;
        self.check_loops(&lb)?;

        let mk = |this: &mut Self, loops: Vec<LoopRec>| -> Result<FaceId, TopoError> {
            let id = FaceId::new(this.faces.len() as u64).ok_or(TopoError::IdSpaceExhausted(EntityKind::TopoFace))?;
            let uses = FaceSide::BOTH.map(|s| FaceUseRec { sense: s.sense(), owner: UseOwner::Exterior });
            this.faces.push(FaceRec { surface: f.surface, surface_sense: f.surface_sense, loops, uses, state: EntityState::Active });
            this.act_faces.push(id);
            this.created(id)?;
            Ok(id)
        };
        let first = mk(self, la)?;
        let second = mk(self, lb)?;
        for side in FaceSide::BOTH {
            let owner = f.uses[side.index()].owner;
            self.faces[first.serial() as usize].uses[side.index()].owner = owner;
            self.faces[second.serial() as usize].uses[side.index()].owner = owner;
            if matches!(owner, UseOwner::Shell(_)) {
                self.replace_face_use(FaceUse { face, side }, &[FaceUse { face: first, side }, FaceUse { face: second, side }]);
            }
        }
        for b in &mut self.bodies {
            if let Some(i) = b.sheets.iter().position(|&s| s == face) {
                b.sheets.splice(i..=i, [first, second]);
            }
        }
        self.faces[face.serial() as usize].state = EntityState::Split(op);
        remove_sorted(&mut self.act_faces, face);
        self.lineage.push(op, LineageEvent::Split { parent: face.entity(), children: vec![first.entity(), second.entity()] });
        self.retired(face);
        Ok(FaceSplit { first, second })
    }

    fn merge_vertices(&mut self, keep: VertexId, drop: VertexId) -> Result<(), TopoError> {
        let op = self.op_id()?;
        if keep == drop {
            return Err(TopoError::Unsupported("cannot merge a vertex with itself"));
        }
        let gap = (self.active_vertex(keep)?.point - self.active_vertex(drop)?.point).norm();
        if gap > self.tol.resolution {
            return Err(TopoError::NotCoincident(format!("vertices are {gap:e} apart, resolution is {:e}", self.tol.resolution)));
        }
        for &eid in &self.act_edges.clone() {
            let e = &mut self.edges[eid.serial() as usize];
            if e.start == drop {
                e.start = keep;
            }
            if e.end == drop {
                e.end = keep;
            }
        }
        for &fid in &self.act_faces.clone() {
            for l in &mut self.faces[fid.serial() as usize].loops {
                for c in &mut l.coedges {
                    if c.kind == (CoedgeKind::Pole { vertex: drop }) {
                        c.kind = CoedgeKind::Pole { vertex: keep };
                    }
                }
            }
        }
        for v in &mut self.volumes {
            for s in &mut v.shells {
                if s.points.contains(&drop) {
                    s.points.retain(|&p| p != drop);
                    dedup_push(&mut s.points, keep);
                }
            }
        }
        for b in &mut self.bodies {
            if b.points.contains(&drop) {
                b.points.retain(|&p| p != drop);
                dedup_push(&mut b.points, keep);
            }
        }
        self.vertices[drop.serial() as usize].state = EntityState::Merged(op);
        remove_sorted(&mut self.act_vertices, drop);
        self.lineage.push(op, LineageEvent::Merged { survivor: keep.entity(), absorbed: drop.entity(), gap });
        self.retired(drop);
        Ok(())
    }

    fn merge_edges(&mut self, keep: EdgeId, drop: EdgeId) -> Result<(), TopoError> {
        let op = self.op_id()?;
        if keep == drop {
            return Err(TopoError::Unsupported("cannot merge an edge with itself"));
        }
        let (k, d) = (self.active_edge(keep)?.clone(), self.active_edge(drop)?.clone());
        let (ck, cd) = (self.shapes.curve(k.curve).ok_or(TopoError::UnknownShape(k.curve))?, self.shapes.curve(d.curve).ok_or(TopoError::UnknownShape(d.curve))?);
        let mut gap = 0.0f64;
        let mut ts = Vec::new();
        for s in [0.2, 0.4, 0.6, 0.8] {
            let p = cd.eval(d.t0 + s * (d.t1 - d.t0));
            let pr = ck.project(p, None, &self.tol)?;
            gap = gap.max(pr.distance);
            if pr.distance > self.tol.resolution || !param_in_range(pr.t, k.t0, k.t1, ck.period(), 1e-9 * (k.t1 - k.t0).abs()) {
                return Err(TopoError::NotCoincident("edge curves do not coincide".into()));
            }
            ts.push(pr.t);
        }
        let mut dt = ts[1] - ts[0];
        if let Some(p) = ck.period() {
            dt -= p * (dt / p).round();
        }
        let rel = if dt >= 0.0 { Sense::Forward } else { Sense::Reversed };
        let ok = match rel {
            Sense::Forward => (k.start, k.end) == (d.start, d.end),
            Sense::Reversed => (k.start, k.end) == (d.end, d.start),
        };
        if !ok {
            return Err(TopoError::NotCoincident("edge end vertices are not identical (merge vertices first)".into()));
        }
        for &fid in &self.act_faces.clone() {
            for l in &mut self.faces[fid.serial() as usize].loops {
                for c in &mut l.coedges {
                    if let CoedgeKind::Edge { edge, sense } = c.kind
                        && edge == drop
                    {
                        c.kind = CoedgeKind::Edge { edge: keep, sense: sense.compose(rel) };
                    }
                }
            }
        }
        for v in &mut self.volumes {
            for s in &mut v.shells {
                if s.wires.contains(&drop) {
                    s.wires.retain(|&w| w != drop);
                    dedup_push(&mut s.wires, keep);
                }
            }
        }
        for b in &mut self.bodies {
            if b.wires.contains(&drop) {
                b.wires.retain(|&w| w != drop);
                dedup_push(&mut b.wires, keep);
            }
        }
        self.edges[drop.serial() as usize].state = EntityState::Merged(op);
        remove_sorted(&mut self.act_edges, drop);
        self.lineage.push(op, LineageEvent::Merged { survivor: keep.entity(), absorbed: drop.entity(), gap });
        self.retired(drop);
        Ok(())
    }

    fn merge_faces(&mut self, keep: FaceId, drop: FaceId, relative: Sense) -> Result<(), TopoError> {
        let op = self.op_id()?;
        if keep == drop {
            return Err(TopoError::Unsupported("cannot merge a face with itself"));
        }
        let (k, d) = (self.active_face(keep)?.clone(), self.active_face(drop)?.clone());
        let edges_of = |f: &FaceRec| {
            let mut v: Vec<EdgeId> = f.loops.iter().flat_map(|l| l.coedges.iter()).filter_map(|c| c.kind.edge()).collect();
            v.sort();
            v
        };
        if k.loops.len() != d.loops.len() || edges_of(&k) != edges_of(&d) {
            return Err(TopoError::NotCoincident("faces do not reference the same edges (imprint and merge edges first)".into()));
        }
        let mut moves = Vec::new();
        for side in FaceSide::BOTH {
            if let UseOwner::Shell(sh) = d.uses[side.index()].owner {
                let target = if relative == Sense::Forward { side } else { side.opposite() };
                if k.uses[target.index()].owner != UseOwner::Exterior {
                    return Err(TopoError::SideOccupied(FaceUse { face: keep, side: target }));
                }
                moves.push((side, target, sh));
            }
        }
        for (side, target, sh) in moves {
            self.faces[keep.serial() as usize].uses[target.index()].owner = UseOwner::Shell(sh);
            self.faces[drop.serial() as usize].uses[side.index()].owner = UseOwner::Exterior;
            self.replace_face_use(FaceUse { face: drop, side }, &[FaceUse { face: keep, side: target }]);
        }
        for b in &mut self.bodies {
            b.sheets.retain(|&f| f != drop);
        }
        self.faces[drop.serial() as usize].state = EntityState::Merged(op);
        remove_sorted(&mut self.act_faces, drop);
        self.lineage.push(op, LineageEvent::Merged { survivor: keep.entity(), absorbed: drop.entity(), gap: 0.0 });
        self.retired(drop);
        self.upsert_coincidence(CoincidenceRecord { a: keep, b: drop, status: Coincidence::Merged, overlap_area: None, op });
        Ok(())
    }

    fn kill_face(&mut self, face: FaceId) -> Result<(), TopoError> {
        let op = self.op_id()?;
        let f = self.active_face(face)?;
        if f.uses.iter().any(|u| u.owner != UseOwner::Exterior) {
            return Err(TopoError::InUse(face.entity()));
        }
        for b in &mut self.bodies {
            b.sheets.retain(|&x| x != face);
        }
        self.faces[face.serial() as usize].state = EntityState::Deleted(op);
        remove_sorted(&mut self.act_faces, face);
        self.lineage.push(op, LineageEvent::Deleted { entity: face.entity() });
        self.retired(face);
        Ok(())
    }

    fn kill_edge(&mut self, edge: EdgeId) -> Result<(), TopoError> {
        let op = self.op_id()?;
        self.active_edge(edge)?;
        let in_face = self.act_faces.iter().any(|&f| self.faces[f.serial() as usize].loops.iter().any(|l| l.coedges.iter().any(|c| c.kind.edge() == Some(edge))));
        let in_wire = self.volumes.iter().any(|v| v.shells.iter().any(|s| s.wires.contains(&edge))) || self.bodies.iter().any(|b| b.wires.contains(&edge));
        if in_face || in_wire {
            return Err(TopoError::InUse(edge.entity()));
        }
        self.edges[edge.serial() as usize].state = EntityState::Deleted(op);
        remove_sorted(&mut self.act_edges, edge);
        self.lineage.push(op, LineageEvent::Deleted { entity: edge.entity() });
        self.retired(edge);
        Ok(())
    }

    fn kill_vertex(&mut self, vertex: VertexId) -> Result<(), TopoError> {
        let op = self.op_id()?;
        self.active_vertex(vertex)?;
        let by_edge = self.act_edges.iter().any(|&e| self.edges[e.serial() as usize].start == vertex || self.edges[e.serial() as usize].end == vertex);
        let by_pole = self.act_faces.iter().any(|&f| self.faces[f.serial() as usize].loops.iter().any(|l| l.coedges.iter().any(|c| c.kind == (CoedgeKind::Pole { vertex }))));
        let by_point = self.volumes.iter().any(|v| v.shells.iter().any(|s| s.points.contains(&vertex))) || self.bodies.iter().any(|b| b.points.contains(&vertex));
        if by_edge || by_pole || by_point {
            return Err(TopoError::InUse(vertex.entity()));
        }
        self.vertices[vertex.serial() as usize].state = EntityState::Deleted(op);
        remove_sorted(&mut self.act_vertices, vertex);
        self.lineage.push(op, LineageEvent::Deleted { entity: vertex.entity() });
        self.retired(vertex);
        Ok(())
    }

    fn kill_volume(&mut self, volume: VolumeId) -> Result<(), TopoError> {
        let op = self.op_id()?;
        let v = self.active_volume(volume)?;
        if v.shells.iter().any(|s| !s.face_uses.is_empty() || !s.wires.is_empty() || !s.points.is_empty()) {
            return Err(TopoError::InUse(volume.entity()));
        }
        let body = v.body;
        self.bodies[body.serial() as usize].volumes.retain(|&x| x != volume);
        self.volumes[volume.serial() as usize].state = EntityState::Deleted(op);
        remove_sorted(&mut self.act_volumes, volume);
        self.lineage.push(op, LineageEvent::Deleted { entity: volume.entity() });
        self.retired(volume);
        Ok(())
    }

    fn kill_body(&mut self, body: BodyId) -> Result<(), TopoError> {
        let op = self.op_id()?;
        let b = self.active_body(body)?;
        if !b.volumes.is_empty() || !b.sheets.is_empty() || !b.wires.is_empty() || !b.points.is_empty() {
            return Err(TopoError::InUse(body.entity()));
        }
        self.bodies[body.serial() as usize].state = EntityState::Deleted(op);
        remove_sorted(&mut self.act_bodies, body);
        self.lineage.push(op, LineageEvent::Deleted { entity: body.entity() });
        self.retired(body);
        Ok(())
    }

    fn set_coincidence(&mut self, a: FaceId, b: FaceId, status: Coincidence, overlap_area: Option<f64>) -> Result<(), TopoError> {
        let op = self.op_id()?;
        if status == Coincidence::Merged {
            return Err(TopoError::Unsupported("Merged is set by merge_faces"));
        }
        if a == b {
            return Err(TopoError::Unsupported("a face cannot coincide with itself"));
        }
        for f in [a, b] {
            if self.faces.get(f.serial() as usize).is_none() {
                return Err(TopoError::NotFound(f.entity()));
            }
        }
        let (a, b) = if a < b { (a, b) } else { (b, a) };
        self.upsert_coincidence(CoincidenceRecord { a, b, status, overlap_area, op });
        Ok(())
    }
}
