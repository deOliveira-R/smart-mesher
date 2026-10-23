//! Reference in-memory implementor of [`TopologyView`] + [`TopologyEdit`].
//!
//! Arena per entity kind, indexed by serial; retired entities leave a `None` tombstone so
//! serials are never reused. Reverse indices are rebuilt after every structural edit
//! (O(N) per edit: simple and obviously deterministic; a production store would update them
//! incrementally).

use crate::data::*;
use crate::edit::*;
use crate::error::TopoError;
use crate::ids::*;
use crate::lineage::{LineageKind, LineageLog, LineageRecord};
use crate::location::{InstanceData, ShapeRef};
use crate::view::TopologyView;
use smallvec::smallvec;
use sm_core::{EntityId, EntityKind, Point3};
use sm_geom_api::TolerancePolicy;

/// Reverse indices (always consistent with the arenas outside an edit method).
#[derive(Clone, Debug, Default)]
struct Adjacency {
    edge_coedges: Vec<Vec<CoedgeRef>>,
    vertex_edges: Vec<Vec<EdgeId>>,
    face_uses: Vec<Vec<FaceUseRef>>,
}

/// Reference cellular non-manifold complex.
#[derive(Clone, Debug)]
pub struct Complex {
    policy: TolerancePolicy,
    alloc: IdAllocator,
    vertices: Vec<Option<VertexData>>,
    edges: Vec<Option<EdgeData>>,
    faces: Vec<Option<FaceData>>,
    volumes: Vec<Option<VolumeData>>,
    bodies: Vec<Option<BodyData>>,
    instances: Vec<Option<InstanceData>>,
    coincidences: Vec<CoincidenceRecord>,
    lineage: LineageLog,
    op: Option<OpId>,
    adj: Adjacency,
}

impl Default for Complex {
    fn default() -> Self {
        Self::new(TolerancePolicy::default())
    }
}

fn ids_of<T>(arena: &[Option<T>], f: impl Fn(&T) -> EntityId) -> Vec<EntityId> {
    arena.iter().flatten().map(f).collect()
}

fn alive<T>(arena: &[Option<T>], id: EntityId) -> bool {
    arena.get(id.serial() as usize).is_some_and(Option::is_some)
}

fn reverse_loop(cs: &[Coedge]) -> Vec<(EdgeId, Sense)> {
    cs.iter().rev().map(|c| (c.edge, c.sense.flip())).collect()
}

/// True if `b` equals `a` up to rotation.
fn cyclic_eq(a: &[(EdgeId, Sense)], b: &[(EdgeId, Sense)]) -> bool {
    a.len() == b.len() && (0..a.len().max(1)).any(|r| (0..a.len()).all(|i| a[i] == b[(i + r) % b.len()]))
}

fn loops_match(s: &[LoopData], d: &[LoopData], reversed: bool) -> bool {
    if s.len() != d.len() {
        return false;
    }
    let mut used = vec![false; d.len()];
    s.iter().all(|ls| {
        let a: Vec<(EdgeId, Sense)> = ls.coedges.iter().map(|c| (c.edge, c.sense)).collect();
        let hit = d.iter().enumerate().position(|(i, ld)| {
            !used[i] && ld.kind == ls.kind && {
                let b: Vec<(EdgeId, Sense)> = if reversed { reverse_loop(&ld.coedges) } else { ld.coedges.iter().map(|c| (c.edge, c.sense)).collect() };
                cyclic_eq(&a, &b)
            }
        });
        hit.is_some_and(|i| {
            used[i] = true;
            true
        })
    })
}

impl Complex {
    /// Empty complex with the given tolerance policy.
    pub fn new(policy: TolerancePolicy) -> Self {
        Self {
            policy,
            alloc: IdAllocator::new(),
            vertices: Vec::new(),
            edges: Vec::new(),
            faces: Vec::new(),
            volumes: Vec::new(),
            bodies: Vec::new(),
            instances: Vec::new(),
            coincidences: Vec::new(),
            lineage: LineageLog::new(),
            op: None,
            adj: Adjacency::default(),
        }
    }

    fn need_op(&self) -> Result<OpId, TopoError> {
        self.op.ok_or(TopoError::NoActiveOp)
    }

    fn log(&mut self, kind: LineageKind) {
        if let Some(op) = self.op {
            self.lineage.push(LineageRecord { op, kind });
        }
    }

    fn next_id(&mut self, kind: EntityKind) -> Result<EntityId, TopoError> {
        self.alloc.alloc(kind).ok_or(TopoError::IdSpace)
    }

    fn rebuild(&mut self) {
        let mut a = Adjacency {
            edge_coedges: vec![Vec::new(); self.edges.len()],
            vertex_edges: vec![Vec::new(); self.vertices.len()],
            face_uses: vec![Vec::new(); self.faces.len()],
        };
        let mut add = |owner: CoedgeOwner, ring: usize, cs: &[Coedge]| {
            for (i, c) in cs.iter().enumerate() {
                a.edge_coedges[c.edge.serial() as usize].push(CoedgeRef { owner, ring: ring as u32, index: i as u32 });
            }
        };
        for f in self.faces.iter().flatten() {
            for (li, l) in f.loops.iter().enumerate() {
                add(CoedgeOwner::Face(f.id), li, &l.coedges);
            }
        }
        for v in self.volumes.iter().flatten() {
            for (wi, w) in v.wires.iter().enumerate() {
                add(CoedgeOwner::Volume(v.id), wi, &w.coedges);
            }
        }
        for b in self.bodies.iter().flatten() {
            for (wi, w) in b.free_wires.iter().enumerate() {
                add(CoedgeOwner::Body(b.id), wi, &w.coedges);
            }
        }
        for e in self.edges.iter().flatten() {
            a.vertex_edges[e.start.serial() as usize].push(e.id);
            if e.end != e.start {
                a.vertex_edges[e.end.serial() as usize].push(e.id);
            }
        }
        for v in self.volumes.iter().flatten() {
            for (si, s) in v.shells.iter().enumerate() {
                for (ui, u) in s.uses.iter().enumerate() {
                    a.face_uses[u.face.serial() as usize].push(FaceUseRef { volume: v.id, shell: si as u32, index: ui as u32 });
                }
            }
        }
        self.adj = a;
    }

    fn edge_ok(&self, e: EdgeId) -> Result<&EdgeData, TopoError> {
        self.edges.get(e.serial() as usize).and_then(Option::as_ref).ok_or(TopoError::NotFound(e.id()))
    }
    fn vertex_ok(&self, v: VertexId) -> Result<&VertexData, TopoError> {
        self.vertices.get(v.serial() as usize).and_then(Option::as_ref).ok_or(TopoError::NotFound(v.id()))
    }
    fn face_ok(&self, f: FaceId) -> Result<&FaceData, TopoError> {
        self.faces.get(f.serial() as usize).and_then(Option::as_ref).ok_or(TopoError::NotFound(f.id()))
    }

    fn check_tol(&self, tol: f64) -> Result<f64, TopoError> {
        if !tol.is_finite() || tol > self.policy.linear_max {
            return Err(TopoError::Tolerance(tol));
        }
        Ok(tol.max(self.policy.linear_min))
    }

    /// Chain check shared by loops (closed) and wires (open).
    fn check_chain(&self, cs: &[Coedge], closed: bool) -> Result<(), TopoError> {
        let mut ends = Vec::with_capacity(cs.len());
        for c in cs {
            let e = self.edge_ok(c.edge)?;
            ends.push(match c.sense {
                Sense::Forward => (e.start, e.end),
                Sense::Reversed => (e.end, e.start),
            });
        }
        let n = cs.len();
        let upto = if closed { n } else { n.saturating_sub(1) };
        for i in 0..upto {
            if ends[i].1 != ends[(i + 1) % n].0 {
                return Err(TopoError::LoopNotClosed { at: i + 1 });
            }
        }
        Ok(())
    }

    fn rewrite_rings(&mut self, f: &mut dyn FnMut(&Coedge, &mut Vec<Coedge>)) {
        fn map(cs: &mut Vec<Coedge>, f: &mut dyn FnMut(&Coedge, &mut Vec<Coedge>)) {
            let mut out = Vec::with_capacity(cs.len());
            for c in cs.iter() {
                f(c, &mut out);
            }
            *cs = out;
        }
        for face in self.faces.iter_mut().flatten() {
            for l in &mut face.loops {
                map(&mut l.coedges, f);
            }
        }
        for v in self.volumes.iter_mut().flatten() {
            for w in &mut v.wires {
                map(&mut w.coedges, f);
            }
        }
        for b in self.bodies.iter_mut().flatten() {
            for w in &mut b.free_wires {
                map(&mut w.coedges, f);
            }
        }
    }

    fn replace_embedded(&mut self, old: EdgeId, new: &[EdgeId]) {
        for f in self.faces.iter_mut().flatten() {
            if f.embedded_edges.contains(&old) {
                f.embedded_edges = f.embedded_edges.iter().flat_map(|e| if *e == old { new.to_vec() } else { vec![*e] }).collect();
            }
        }
    }

    fn face_use_senses(&self, f: FaceId) -> Vec<Sense> {
        self.adj.face_uses[f.serial() as usize].iter().filter_map(|r| self.face_use(*r)).map(|u| u.sense).collect()
    }

    fn uses_valid(senses: &[Sense]) -> bool {
        senses.len() <= 1 || (senses.len() == 2 && senses[0] != senses[1])
    }
}

impl TopologyView for Complex {
    fn policy(&self) -> &TolerancePolicy {
        &self.policy
    }
    fn is_alive(&self, id: EntityId) -> bool {
        match id.kind() {
            EntityKind::TopoVertex => alive(&self.vertices, id),
            EntityKind::TopoEdge => alive(&self.edges, id),
            EntityKind::TopoFace => alive(&self.faces, id),
            EntityKind::TopoVolume => alive(&self.volumes, id),
            EntityKind::TopoBody => alive(&self.bodies, id),
            EntityKind::Instance => alive(&self.instances, id),
            _ => false,
        }
    }
    fn entity_ids(&self, kind: EntityKind) -> Vec<EntityId> {
        match kind {
            EntityKind::TopoVertex => ids_of(&self.vertices, |x| x.id.id()),
            EntityKind::TopoEdge => ids_of(&self.edges, |x| x.id.id()),
            EntityKind::TopoFace => ids_of(&self.faces, |x| x.id.id()),
            EntityKind::TopoVolume => ids_of(&self.volumes, |x| x.id.id()),
            EntityKind::TopoBody => ids_of(&self.bodies, |x| x.id.id()),
            EntityKind::Instance => ids_of(&self.instances, |x| x.id.id()),
            _ => Vec::new(),
        }
    }
    fn vertex(&self, id: VertexId) -> Option<&VertexData> {
        self.vertices.get(id.serial() as usize)?.as_ref()
    }
    fn edge(&self, id: EdgeId) -> Option<&EdgeData> {
        self.edges.get(id.serial() as usize)?.as_ref()
    }
    fn face(&self, id: FaceId) -> Option<&FaceData> {
        self.faces.get(id.serial() as usize)?.as_ref()
    }
    fn volume(&self, id: VolumeId) -> Option<&VolumeData> {
        self.volumes.get(id.serial() as usize)?.as_ref()
    }
    fn body(&self, id: BodyId) -> Option<&BodyData> {
        self.bodies.get(id.serial() as usize)?.as_ref()
    }
    fn instance(&self, id: InstanceId) -> Option<&InstanceData> {
        self.instances.get(id.serial() as usize)?.as_ref()
    }
    fn edge_coedges(&self, e: EdgeId) -> Vec<CoedgeRef> {
        self.adj.edge_coedges.get(e.serial() as usize).cloned().unwrap_or_default()
    }
    fn vertex_edges(&self, v: VertexId) -> Vec<EdgeId> {
        self.adj.vertex_edges.get(v.serial() as usize).cloned().unwrap_or_default()
    }
    fn face_uses(&self, f: FaceId) -> Vec<FaceUseRef> {
        self.adj.face_uses.get(f.serial() as usize).cloned().unwrap_or_default()
    }
    fn coincidences(&self) -> &[CoincidenceRecord] {
        &self.coincidences
    }
    fn lineage(&self) -> &LineageLog {
        &self.lineage
    }
}

impl TopologyEdit for Complex {
    fn begin_op(&mut self, op: OpId) -> TopoResult<()> {
        if let Some(open) = self.op {
            return Err(TopoError::OpAlreadyActive(open));
        }
        self.op = Some(op);
        Ok(())
    }

    fn end_op(&mut self) -> TopoResult<()> {
        self.need_op()?;
        self.rebuild();
        self.op = None;
        Ok(())
    }

    fn make_vertex(&mut self, point: Point3, tol: f64) -> TopoResult<VertexId> {
        self.need_op()?;
        let tol = self.check_tol(tol)?;
        if ![point.x, point.y, point.z].iter().all(|c| c.is_finite()) {
            return Err(TopoError::Invalid("non-finite point"));
        }
        let id = VertexId(self.next_id(EntityKind::TopoVertex)?);
        self.vertices.push(Some(VertexData { id, point, tol }));
        self.adj.vertex_edges.push(Vec::new());
        self.log(LineageKind::Created { entity: id.id() });
        Ok(id)
    }

    fn make_edge(&mut self, spec: EdgeSpec) -> TopoResult<EdgeId> {
        self.need_op()?;
        self.vertex_ok(spec.start)?;
        self.vertex_ok(spec.end)?;
        let tol = self.check_tol(spec.tol)?;
        let nonempty = spec.range.lo < spec.range.hi;
        if !nonempty && !spec.degenerate {
            return Err(TopoError::Invalid("empty edge range"));
        }
        if spec.degenerate && spec.start != spec.end {
            return Err(TopoError::Invalid("degenerate edge must be closed"));
        }
        let id = EdgeId(self.next_id(EntityKind::TopoEdge)?);
        self.edges.push(Some(EdgeData { id, curve: spec.curve, range: spec.range, start: spec.start, end: spec.end, tol, degenerate: spec.degenerate }));
        self.log(LineageKind::Created { entity: id.id() });
        self.rebuild();
        Ok(id)
    }

    fn make_face(&mut self, spec: FaceSpec) -> TopoResult<FaceId> {
        self.need_op()?;
        let tol = self.check_tol(spec.tol)?;
        if spec.loops.first().map(|l| l.kind) != Some(LoopKind::Outer) || spec.loops.iter().skip(1).any(|l| l.kind != LoopKind::Inner) {
            return Err(TopoError::Invalid("first loop must be Outer, the rest Inner"));
        }
        for l in &spec.loops {
            if l.coedges.is_empty() {
                return Err(TopoError::Invalid("empty loop"));
            }
            self.check_chain(&l.coedges, true)?;
        }
        for e in &spec.embedded_edges {
            self.edge_ok(*e)?;
        }
        let id = FaceId(self.next_id(EntityKind::TopoFace)?);
        self.faces.push(Some(FaceData { id, surface: spec.surface, surface_sense: spec.surface_sense, tol, loops: spec.loops, embedded_edges: spec.embedded_edges }));
        self.log(LineageKind::Created { entity: id.id() });
        self.rebuild();
        Ok(id)
    }

    fn make_body(&mut self) -> TopoResult<BodyId> {
        self.need_op()?;
        let id = BodyId(self.next_id(EntityKind::TopoBody)?);
        self.bodies.push(Some(BodyData { id, volumes: Vec::new(), free_wires: Vec::new() }));
        self.log(LineageKind::Created { entity: id.id() });
        Ok(id)
    }

    fn make_volume(&mut self, body: BodyId, shells: Vec<ShellData>) -> TopoResult<VolumeId> {
        self.need_op()?;
        if !alive(&self.bodies, body.id()) {
            return Err(TopoError::NotFound(body.id()));
        }
        if shells.first().map(|s| s.kind) != Some(ShellKind::Outer) || shells.iter().skip(1).any(|s| s.kind != ShellKind::Inner) {
            return Err(TopoError::Invalid("first shell must be Outer, the rest Inner"));
        }
        // Prospective use senses per face, including existing uses.
        let mut added: Vec<(FaceId, Vec<Sense>)> = Vec::new();
        for u in shells.iter().flat_map(|s| s.uses.iter()) {
            self.face_ok(u.face)?;
            match added.iter_mut().find(|(f, _)| *f == u.face) {
                Some((_, v)) => v.push(u.sense),
                None => added.push((u.face, vec![u.sense])),
            }
        }
        for (f, mut senses) in added {
            senses.extend(self.face_use_senses(f));
            if !Self::uses_valid(&senses) {
                return Err(TopoError::FaceUses(f.id()));
            }
        }
        let id = VolumeId(self.next_id(EntityKind::TopoVolume)?);
        self.volumes.push(Some(VolumeData { id, body, shells, wires: Vec::new() }));
        if let Some(Some(b)) = self.bodies.get_mut(body.serial() as usize) {
            b.volumes.push(id);
        }
        self.log(LineageKind::Created { entity: id.id() });
        self.rebuild();
        Ok(id)
    }

    fn add_wire(&mut self, owner: WireOwner, coedges: Vec<Coedge>) -> TopoResult<()> {
        self.need_op()?;
        if coedges.is_empty() {
            return Err(TopoError::Invalid("empty wire"));
        }
        self.check_chain(&coedges, false)?;
        match owner {
            WireOwner::Volume(v) => match self.volumes.get_mut(v.serial() as usize) {
                Some(Some(vol)) => vol.wires.push(WireData { coedges }),
                _ => return Err(TopoError::NotFound(v.id())),
            },
            WireOwner::Body(b) => match self.bodies.get_mut(b.serial() as usize) {
                Some(Some(body)) => body.free_wires.push(WireData { coedges }),
                _ => return Err(TopoError::NotFound(b.id())),
            },
        }
        self.rebuild();
        Ok(())
    }

    fn add_baffle(&mut self, volume: VolumeId, face: FaceId) -> TopoResult<()> {
        self.need_op()?;
        self.face_ok(face)?;
        if !self.adj.face_uses[face.serial() as usize].is_empty() {
            return Err(TopoError::FaceUses(face.id()));
        }
        match self.volumes.get_mut(volume.serial() as usize) {
            Some(Some(v)) => {
                v.shells[0].uses.push(FaceUse { face, sense: Sense::Forward });
                v.shells[0].uses.push(FaceUse { face, sense: Sense::Reversed });
            }
            _ => return Err(TopoError::NotFound(volume.id())),
        }
        self.rebuild();
        Ok(())
    }

    fn make_instance(&mut self, target: ShapeRef, parent: Option<InstanceId>) -> TopoResult<InstanceId> {
        self.need_op()?;
        if !matches!(target.id.kind(), EntityKind::TopoBody | EntityKind::TopoVolume) || !self.is_alive(target.id) {
            return Err(TopoError::NotFound(target.id));
        }
        if let Some(p) = parent
            && !alive(&self.instances, p.id())
        {
            return Err(TopoError::NotFound(p.id()));
        }
        let id = InstanceId(self.next_id(EntityKind::Instance)?);
        self.instances.push(Some(InstanceData { id, target, parent }));
        self.log(LineageKind::Created { entity: id.id() });
        Ok(id)
    }

    fn split_edge(&mut self, edge: EdgeId, at: VertexId, t: f64) -> TopoResult<[EdgeId; 2]> {
        self.need_op()?;
        let e = self.edge_ok(edge)?.clone();
        self.vertex_ok(at)?;
        let inside = t > e.range.lo && t < e.range.hi;
        if e.degenerate || at == e.start || at == e.end || !inside {
            return Err(TopoError::Invalid("split point must be a new vertex strictly inside the edge"));
        }
        let e1 = EdgeId(self.next_id(EntityKind::TopoEdge)?);
        let e2 = EdgeId(self.next_id(EntityKind::TopoEdge)?);
        let mk = |id, lo, hi, start, end| EdgeData { id, curve: e.curve, range: sm_geom_api::Interval::new(lo, hi), start, end, tol: e.tol, degenerate: false };
        self.edges.push(Some(mk(e1, e.range.lo, t, e.start, at)));
        self.edges.push(Some(mk(e2, t, e.range.hi, at, e.end)));
        self.rewrite_rings(&mut |c, out| {
            if c.edge != edge {
                out.push(c.clone());
                return;
            }
            let (a, b) = (Coedge { edge: e1, ..c.clone() }, Coedge { edge: e2, ..c.clone() });
            match c.sense {
                Sense::Forward => out.extend([a, b]),
                Sense::Reversed => out.extend([b, a]),
            }
        });
        self.replace_embedded(edge, &[e1, e2]);
        self.edges[edge.serial() as usize] = None;
        self.log(LineageKind::Split { from: edge.id(), into: smallvec![e1.id(), e2.id()] });
        self.rebuild();
        Ok([e1, e2])
    }

    fn split_face(&mut self, spec: SplitFaceSpec) -> TopoResult<[FaceId; 2]> {
        self.need_op()?;
        let f = self.face_ok(spec.face)?.clone();
        let chord = self.edge_ok(spec.chord)?.clone();
        if f.loops.len() != 1 || !f.embedded_edges.is_empty() {
            return Err(TopoError::Unsupported("split_face needs a single loop and no embedded edges"));
        }
        let cs = &f.loops[0].coedges;
        let n = cs.len();
        if spec.a >= n || spec.b >= n || spec.a == spec.b {
            return Err(TopoError::Invalid("split positions"));
        }
        let start_of = |i: usize| self.coedge_vertices(&cs[i]).map(|p| p.0);
        if start_of(spec.a) != Some(chord.start) || start_of(spec.b) != Some(chord.end) {
            return Err(TopoError::NotCoincident(spec.chord.id(), spec.face.id()));
        }
        let arc = |from: usize, to: usize| -> Vec<Coedge> {
            let mut v = Vec::new();
            let mut i = from;
            while i != to {
                v.push(cs[i].clone());
                i = (i + 1) % n;
            }
            v
        };
        let mut c0 = arc(spec.a, spec.b);
        c0.push(Coedge { edge: spec.chord, sense: Sense::Reversed, pcurve: spec.pcurves[0] });
        let mut c1 = arc(spec.b, spec.a);
        c1.push(Coedge { edge: spec.chord, sense: Sense::Forward, pcurve: spec.pcurves[1] });
        let f0 = FaceId(self.next_id(EntityKind::TopoFace)?);
        let f1 = FaceId(self.next_id(EntityKind::TopoFace)?);
        for (id, coedges) in [(f0, c0), (f1, c1)] {
            self.faces.push(Some(FaceData {
                id,
                surface: f.surface,
                surface_sense: f.surface_sense,
                tol: f.tol,
                loops: vec![LoopData { kind: LoopKind::Outer, coedges }],
                embedded_edges: Vec::new(),
            }));
        }
        for v in self.volumes.iter_mut().flatten() {
            for s in &mut v.shells {
                if s.uses.iter().any(|u| u.face == spec.face) {
                    s.uses = s
                        .uses
                        .iter()
                        .flat_map(|u| if u.face == spec.face { vec![FaceUse { face: f0, sense: u.sense }, FaceUse { face: f1, sense: u.sense }] } else { vec![*u] })
                        .collect();
                }
            }
        }
        self.faces[spec.face.serial() as usize] = None;
        self.log(LineageKind::Split { from: spec.face.id(), into: smallvec![f0.id(), f1.id()] });
        self.rebuild();
        Ok([f0, f1])
    }

    fn merge_vertices(&mut self, a: VertexId, b: VertexId) -> TopoResult<VertexId> {
        self.need_op()?;
        let (s, d) = if a <= b { (a, b) } else { (b, a) };
        if s == d {
            return Err(TopoError::Invalid("merging a vertex with itself"));
        }
        let (vs, vd) = (self.vertex_ok(s)?.clone(), self.vertex_ok(d)?.clone());
        let tol = self.policy.grown(vs.tol, (vs.point - vd.point).norm(), vd.tol);
        if !self.policy.within_cap(tol) {
            return Err(TopoError::Tolerance(tol));
        }
        for e in self.edges.iter_mut().flatten() {
            if e.start == d {
                e.start = s;
            }
            if e.end == d {
                e.end = s;
            }
        }
        if let Some(Some(v)) = self.vertices.get_mut(s.serial() as usize) {
            v.tol = tol;
        }
        self.vertices[d.serial() as usize] = None;
        self.log(LineageKind::Merged { from: smallvec![s.id(), d.id()], into: s.id() });
        self.rebuild();
        Ok(s)
    }

    fn merge_edges(&mut self, a: EdgeId, b: EdgeId) -> TopoResult<EdgeId> {
        self.need_op()?;
        let (s, d) = if a <= b { (a, b) } else { (b, a) };
        if s == d {
            return Err(TopoError::Invalid("merging an edge with itself"));
        }
        let (es, ed) = (self.edge_ok(s)?.clone(), self.edge_ok(d)?.clone());
        let opposed = if (es.start, es.end) == (ed.start, ed.end) {
            false
        } else if (es.start, es.end) == (ed.end, ed.start) {
            true
        } else {
            return Err(TopoError::NotCoincident(s.id(), d.id()));
        };
        self.rewrite_rings(&mut |c, out| {
            if c.edge == d {
                out.push(Coedge { edge: s, sense: if opposed { c.sense.flip() } else { c.sense }, pcurve: None });
            } else {
                out.push(c.clone());
            }
        });
        self.replace_embedded(d, &[s]);
        if let Some(Some(e)) = self.edges.get_mut(s.serial() as usize) {
            e.tol = e.tol.max(ed.tol);
        }
        self.edges[d.serial() as usize] = None;
        self.log(LineageKind::Merged { from: smallvec![s.id(), d.id()], into: s.id() });
        self.rebuild();
        Ok(s)
    }

    fn merge_faces(&mut self, a: FaceId, b: FaceId) -> TopoResult<FaceId> {
        self.need_op()?;
        let (s, d) = if a <= b { (a, b) } else { (b, a) };
        if s == d {
            return Err(TopoError::Invalid("merging a face with itself"));
        }
        let (fs, fd) = (self.face_ok(s)?.clone(), self.face_ok(d)?.clone());
        let opposed = if loops_match(&fs.loops, &fd.loops, false) {
            false
        } else if loops_match(&fs.loops, &fd.loops, true) {
            true
        } else {
            return Err(TopoError::NotCoincident(s.id(), d.id()));
        };
        let mut senses = self.face_use_senses(s);
        senses.extend(self.face_use_senses(d).into_iter().map(|x| if opposed { x.flip() } else { x }));
        if !Self::uses_valid(&senses) {
            return Err(TopoError::FaceUses(s.id()));
        }
        for v in self.volumes.iter_mut().flatten() {
            for sh in &mut v.shells {
                for u in &mut sh.uses {
                    if u.face == d {
                        *u = FaceUse { face: s, sense: if opposed { u.sense.flip() } else { u.sense } };
                    }
                }
            }
        }
        self.faces[d.serial() as usize] = None;
        let area = self.coincidence(s, d).and_then(|r| r.overlap_area);
        let rec = CoincidenceRecord { a: s, b: d, status: ImprintStatus::Merged, overlap_area: area };
        match self.coincidences.iter_mut().find(|r| r.a == s && r.b == d) {
            Some(r) => *r = rec,
            None => {
                self.coincidences.push(rec);
                self.coincidences.sort_by_key(|r| (r.a, r.b));
            }
        }
        self.log(LineageKind::Merged { from: smallvec![s.id(), d.id()], into: s.id() });
        self.rebuild();
        Ok(s)
    }

    fn set_coincidence(&mut self, a: FaceId, b: FaceId, status: ImprintStatus, overlap_area: Option<f64>) -> TopoResult<()> {
        self.need_op()?;
        if status == ImprintStatus::Merged || a == b {
            return Err(TopoError::Invalid("`Merged` is set only by merge_faces; pair must be two faces"));
        }
        self.face_ok(a)?;
        self.face_ok(b)?;
        let (a, b) = if a <= b { (a, b) } else { (b, a) };
        let rec = CoincidenceRecord { a, b, status, overlap_area };
        match self.coincidences.iter_mut().find(|r| r.a == a && r.b == b) {
            Some(r) => *r = rec,
            None => {
                self.coincidences.push(rec);
                self.coincidences.sort_by_key(|r| (r.a, r.b));
            }
        }
        Ok(())
    }

    fn raise_tolerance(&mut self, id: EntityId, tol: f64) -> TopoResult<()> {
        self.need_op()?;
        let cap = self.policy.linear_max;
        let slot: Option<&mut f64> = match id.kind() {
            EntityKind::TopoVertex => self.vertices.get_mut(id.serial() as usize).and_then(Option::as_mut).map(|x| &mut x.tol),
            EntityKind::TopoEdge => self.edges.get_mut(id.serial() as usize).and_then(Option::as_mut).map(|x| &mut x.tol),
            EntityKind::TopoFace => self.faces.get_mut(id.serial() as usize).and_then(Option::as_mut).map(|x| &mut x.tol),
            _ => return Err(TopoError::Invalid("entity has no tolerance")),
        };
        let slot = slot.ok_or(TopoError::NotFound(id))?;
        if !tol.is_finite() || tol < *slot || tol > cap {
            return Err(TopoError::Tolerance(tol));
        }
        *slot = tol;
        self.log(LineageKind::Modified { entity: id });
        Ok(())
    }

    fn record_generated(&mut self, from: EntityId, into: &[EntityId]) -> TopoResult<()> {
        self.need_op()?;
        if !self.is_alive(from) {
            return Err(TopoError::NotFound(from));
        }
        if let Some(x) = into.iter().find(|x| !self.is_alive(**x)) {
            return Err(TopoError::NotFound(*x));
        }
        self.log(LineageKind::Generated { from, into: into.iter().copied().collect() });
        Ok(())
    }

    fn delete_face(&mut self, f: FaceId) -> TopoResult<()> {
        self.need_op()?;
        self.face_ok(f)?;
        for v in self.volumes.iter_mut().flatten() {
            for s in &mut v.shells {
                s.uses.retain(|u| u.face != f);
            }
        }
        self.faces[f.serial() as usize] = None;
        self.log(LineageKind::Deleted { entity: f.id() });
        self.rebuild();
        Ok(())
    }

    fn delete_edge(&mut self, e: EdgeId) -> TopoResult<()> {
        self.need_op()?;
        self.edge_ok(e)?;
        if !self.adj.edge_coedges[e.serial() as usize].is_empty() || self.faces.iter().flatten().any(|f| f.embedded_edges.contains(&e)) {
            return Err(TopoError::InUse(e.id()));
        }
        self.edges[e.serial() as usize] = None;
        self.log(LineageKind::Deleted { entity: e.id() });
        self.rebuild();
        Ok(())
    }

    fn delete_vertex(&mut self, v: VertexId) -> TopoResult<()> {
        self.need_op()?;
        self.vertex_ok(v)?;
        if !self.adj.vertex_edges[v.serial() as usize].is_empty() {
            return Err(TopoError::InUse(v.id()));
        }
        self.vertices[v.serial() as usize] = None;
        self.log(LineageKind::Deleted { entity: v.id() });
        Ok(())
    }

    fn delete_volume(&mut self, v: VolumeId) -> TopoResult<()> {
        self.need_op()?;
        let body = self.volume(v).ok_or(TopoError::NotFound(v.id()))?.body;
        if let Some(Some(b)) = self.bodies.get_mut(body.serial() as usize) {
            b.volumes.retain(|x| *x != v);
        }
        self.volumes[v.serial() as usize] = None;
        self.log(LineageKind::Deleted { entity: v.id() });
        self.rebuild();
        Ok(())
    }
}
