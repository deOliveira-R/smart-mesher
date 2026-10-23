//! [`Ledger`]: the committed meshes of all targets, keyed by [`NodeKey`] and [`Target`].
//!
//! The ledger is the only mutable state of a meshing run. Patches are checked as a whole before
//! anything is applied ([`Ledger::commit`]), so a job commits all of its targets or none. The
//! ledger is written to `sm-mesh-api` once, at the end ([`Ledger::write_mesh`]): a failed run never
//! touches the mesh database or its id allocator.

use crate::error::{DriverError, PatchError};
use crate::job::{JobId, JobKind, MeshJob};
use crate::patch::MeshPatch;
use crate::target::{NodeKey, Target};
use sm_core::{EntityId, Point3};
use sm_mesh_api::{CellShape, Classification, FaceIx, MeshBuild, MeshEntityRef, Param, Side, VertexIx};
use sm_model_api::{Keep, Model, PieceFace, PieceRef};
use sm_topo_api::{TopologyView, TopologyViewExt};
use smallvec::SmallVec;
use std::collections::{BTreeMap, BTreeSet};

/// A committed node.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NodeRec {
    /// Position.
    pub point: Point3,
    /// Parameter on the owner.
    pub param: Param,
    /// The job that created it (`None` for seeded model vertices).
    pub job: Option<JobId>,
}

/// Committed face elements of one target (CSR over node keys).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FaceMesh {
    /// Offsets (`len = elements + 1`).
    pub off: Vec<u32>,
    /// Node keys.
    pub nodes: Vec<NodeKey>,
}

impl FaceMesh {
    /// Number of elements.
    pub fn len(&self) -> usize {
        self.off.len().saturating_sub(1)
    }
    /// True if there are no elements.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
    /// Element `k`.
    pub fn element(&self, k: usize) -> &[NodeKey] {
        &self.nodes[self.off[k] as usize..self.off[k + 1] as usize]
    }
}

/// Committed cells of one target.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CellMesh {
    /// Shapes.
    pub shapes: Vec<CellShape>,
    /// Offsets (`len = cells + 1`).
    pub off: Vec<u32>,
    /// Corner node keys (template order).
    pub nodes: Vec<NodeKey>,
}

impl CellMesh {
    /// Number of cells.
    pub fn len(&self) -> usize {
        self.shapes.len()
    }
    /// True if there are no cells.
    pub fn is_empty(&self) -> bool {
        self.shapes.is_empty()
    }
    /// Corners of cell `k`.
    pub fn corners(&self, k: usize) -> &[NodeKey] {
        &self.nodes[self.off[k] as usize..self.off[k + 1] as usize]
    }
}

/// Vertex index of every written node.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct WriteMap {
    /// Node -> builder vertex.
    pub vertices: BTreeMap<NodeKey, VertexIx>,
}

/// Committed meshes.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Ledger {
    nodes: BTreeMap<NodeKey, NodeRec>,
    curves: BTreeMap<Target, Vec<NodeKey>>,
    faces: BTreeMap<Target, FaceMesh>,
    cells: BTreeMap<Target, CellMesh>,
    writer: BTreeMap<Target, JobId>,
}

fn malformed(s: impl Into<String>) -> PatchError {
    PatchError::Malformed(s.into())
}

impl Ledger {
    /// Empty ledger.
    pub fn new() -> Self {
        Self::default()
    }

    /// Seed the node of a model vertex (done by the driver for every vertex in the closure).
    pub fn seed_vertex(&mut self, v: EntityId, p: Point3) {
        self.nodes.insert(NodeKey::vertex(v), NodeRec { point: p, param: Param::None, job: None });
    }

    /// A committed node.
    pub fn node(&self, k: &NodeKey) -> Option<&NodeRec> {
        self.nodes.get(k)
    }
    /// Position of a committed node.
    pub fn point(&self, k: &NodeKey) -> Option<Point3> {
        self.nodes.get(k).map(|n| n.point)
    }
    /// Number of committed nodes.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }
    /// Committed edge mesh (start vertex -> end vertex).
    pub fn curve(&self, t: &Target) -> Option<&[NodeKey]> {
        self.curves.get(t).map(|v| v.as_slice())
    }
    /// Committed face mesh.
    pub fn face(&self, t: &Target) -> Option<&FaceMesh> {
        self.faces.get(t)
    }
    /// Committed cells.
    pub fn cells(&self, t: &Target) -> Option<&CellMesh> {
        self.cells.get(t)
    }
    /// The job that wrote a target.
    pub fn writer(&self, t: &Target) -> Option<JobId> {
        self.writer.get(t).copied()
    }

    /// Check a patch against the commit rules, then apply it; on `Err` nothing changed.
    ///
    /// Rules: every part and every new node's owner is in `job.footprint.writes`; every write
    /// target gets a part (vertex targets excepted); no target is written twice (except by
    /// `Transform` jobs, which replace); node keys are unique, new nodes are not yet committed
    /// and have finite points, referenced nodes are committed; CSR and indices are well formed;
    /// cells have their shape's corner count; an edge mesh runs from its start vertex node to its
    /// end vertex node.
    pub fn commit(&mut self, job: &MeshJob, p: &MeshPatch, topo: &dyn TopologyView) -> Result<(), PatchError> {
        let replace = matches!(job.kind, JobKind::Transform(_));
        let writes = &job.footprint.writes;
        let mut seen: BTreeSet<NodeKey> = BTreeSet::new();
        for n in &p.nodes {
            if !seen.insert(n.key) {
                return Err(PatchError::DuplicateNode(n.key));
            }
            match n.point {
                Some(pt) => {
                    if !writes.contains(&n.key.owner) {
                        return Err(PatchError::OutsideFootprint(n.key.owner));
                    }
                    if !replace && self.nodes.contains_key(&n.key) {
                        return Err(PatchError::DuplicateNode(n.key));
                    }
                    if !(pt.x.is_finite() && pt.y.is_finite() && pt.z.is_finite()) {
                        return Err(malformed(format!("non-finite point for {:?}", n.key)));
                    }
                }
                None => {
                    if !self.nodes.contains_key(&n.key) {
                        return Err(PatchError::UnknownNode(n.key));
                    }
                }
            }
        }
        let nn = p.nodes.len() as u32;
        let idx_ok = |s: &[u32]| s.iter().all(|&i| i < nn);
        if !idx_ok(&p.curve_nodes) || !idx_ok(&p.face_nodes) || !idx_ok(&p.cell_nodes) {
            return Err(malformed("node index out of range"));
        }
        let csr_ok = |off: &[u32], len: usize| off.first() == Some(&0) && off.windows(2).all(|w| w[0] <= w[1]) && off.last().copied() == Some(len as u32);
        if !csr_ok(&p.face_off, p.face_nodes.len()) || !csr_ok(&p.cell_off, p.cell_nodes.len()) || p.cell_shape.len() != p.cell_count() {
            return Err(malformed("bad CSR offsets"));
        }
        for k in 0..p.face_count() {
            if p.face_off[k + 1] - p.face_off[k] < 3 {
                return Err(malformed("face element with fewer than 3 nodes"));
            }
        }
        for (k, s) in p.cell_shape.iter().enumerate() {
            let n = (p.cell_off[k + 1] - p.cell_off[k]) as usize;
            if s.vertex_count() != Some(n) {
                return Err(malformed(format!("cell {k}: {s:?} with {n} corners")));
            }
        }
        let mut parts: BTreeSet<Target> = BTreeSet::new();
        let check_parts = |parts: &mut BTreeSet<Target>, list: &[crate::patch::Part], len: usize| -> Result<(), PatchError> {
            for part in list {
                if !writes.contains(&part.target) {
                    return Err(PatchError::OutsideFootprint(part.target));
                }
                if part.range.start > part.range.end || part.range.end as usize > len {
                    return Err(malformed(format!("bad range for {:?}", part.target)));
                }
                if !parts.insert(part.target) {
                    return Err(malformed(format!("{:?} has two parts", part.target)));
                }
                if !replace && self.writer.contains_key(&part.target) {
                    return Err(PatchError::AlreadyWritten(part.target));
                }
            }
            Ok(())
        };
        check_parts(&mut parts, &p.curves, p.curve_nodes.len())?;
        check_parts(&mut parts, &p.faces, p.face_count())?;
        check_parts(&mut parts, &p.cells, p.cell_count())?;
        for w in writes {
            if w.dim() != Some(0) && !parts.contains(w) {
                return Err(PatchError::Missing(*w));
            }
        }
        for part in &p.curves {
            let list = &p.curve_nodes[part.range.start as usize..part.range.end as usize];
            if let Some(e) = part.target.topo()
                && let Some((s, t)) = topo.edge_ends(e)
            {
                let first = list.first().map(|&i| p.nodes[i as usize].key);
                let last = list.last().map(|&i| p.nodes[i as usize].key);
                if list.len() < 2 || first != Some(NodeKey::vertex(s)) || last != Some(NodeKey::vertex(t)) {
                    return Err(PatchError::EdgeEnds(part.target));
                }
            }
        }
        // Apply.
        for n in &p.nodes {
            if let Some(pt) = n.point {
                self.nodes.insert(n.key, NodeRec { point: pt, param: n.param, job: Some(job.id) });
            }
        }
        let key = |i: &u32| p.nodes[*i as usize].key;
        for part in &p.curves {
            let list = p.curve_nodes[part.range.start as usize..part.range.end as usize].iter().map(key).collect();
            self.curves.insert(part.target, list);
            self.writer.insert(part.target, job.id);
        }
        for part in &p.faces {
            let mut fm = FaceMesh { off: vec![0], nodes: Vec::new() };
            for k in part.range.start as usize..part.range.end as usize {
                fm.nodes.extend(p.face_nodes[p.face_off[k] as usize..p.face_off[k + 1] as usize].iter().map(key));
                fm.off.push(fm.nodes.len() as u32);
            }
            self.faces.insert(part.target, fm);
            self.writer.insert(part.target, job.id);
        }
        for part in &p.cells {
            let mut cm = CellMesh { shapes: Vec::new(), off: vec![0], nodes: Vec::new() };
            for k in part.range.start as usize..part.range.end as usize {
                cm.shapes.push(p.cell_shape[k]);
                cm.nodes.extend(p.cell_nodes[p.cell_off[k] as usize..p.cell_off[k + 1] as usize].iter().map(key));
                cm.off.push(cm.nodes.len() as u32);
            }
            self.cells.insert(part.target, cm);
            self.writer.insert(part.target, job.id);
        }
        Ok(())
    }

    /// FNV-1a content hash over keys, raw point bits, parameters and element lists, in key order.
    /// Bit-exact (single platform), for determinism tests.
    pub fn hash(&self) -> u64 {
        let mut h = Fnv::new();
        for (k, n) in &self.nodes {
            h.key(k);
            h.f(n.point.x);
            h.f(n.point.y);
            h.f(n.point.z);
            match n.param {
                Param::None => h.u(0),
                Param::Curve(u) => {
                    h.u(1);
                    h.f(u);
                }
                Param::Surface([u, v]) => {
                    h.u(2);
                    h.f(u);
                    h.f(v);
                }
                Param::Volume([u, v, w]) => {
                    h.u(3);
                    h.f(u);
                    h.f(v);
                    h.f(w);
                }
            }
        }
        for (t, l) in &self.curves {
            h.target(t);
            h.u(l.len() as u64);
            l.iter().for_each(|k| h.key(k));
        }
        for (t, f) in &self.faces {
            h.target(t);
            f.off.iter().for_each(|&o| h.u(u64::from(o)));
            f.nodes.iter().for_each(|k| h.key(k));
        }
        for (t, c) in &self.cells {
            h.target(t);
            c.off.iter().for_each(|&o| h.u(u64::from(o)));
            c.nodes.iter().for_each(|k| h.key(k));
        }
        h.0
    }

    /// Write every committed cell into a mesh builder, with classification.
    ///
    /// Vertices are added in [`NodeKey`] order (only nodes used by cells), classified on the
    /// node's owner. Cells are added target by target in ascending order and classified on the
    /// target (`TopoVolume` or `PartitionCell`). Faces whose vertex set matches a committed face
    /// element are classified on that face target; a face between two cells of different pieces
    /// of one volume (a virtual cut) is classified on the volume, since `finish` only fills
    /// interior faces whose cells agree. Edge and face meshes without cells are not written
    /// (`MeshBuild` holds 3D cells only). The caller runs `finish`.
    pub fn write_mesh<B: MeshBuild>(&self, b: &mut B, model: &Model) -> Result<WriteMap, DriverError> {
        let mut used: BTreeSet<NodeKey> = BTreeSet::new();
        for c in self.cells.values() {
            used.extend(c.nodes.iter().copied());
        }
        let mut map = WriteMap::default();
        for k in &used {
            let rec = self.nodes.get(k).ok_or_else(|| DriverError::Unwritable(k.owner, "node not committed".into()))?;
            let class = k.classification(rec.param).ok_or_else(|| DriverError::Unwritable(k.owner, "owner has no mesh classification".into()))?;
            let v = b.add_vertex(rec.point)?;
            b.set_class(MeshEntityRef::Vertex(v), class)?;
            map.vertices.insert(*k, v);
        }
        let mut verts: SmallVec<[VertexIx; 8]> = SmallVec::new();
        let mut cell_entity: Vec<EntityId> = Vec::new();
        for (t, cm) in &self.cells {
            let entity = t.class_entity().ok_or_else(|| DriverError::Unwritable(*t, "volume target has no classification".into()))?;
            for k in 0..cm.len() {
                verts.clear();
                verts.extend(cm.corners(k).iter().map(|key| map.vertices[key]));
                let added = b.add_cell_from_vertices(cm.shapes[k], &verts)?;
                b.set_class(MeshEntityRef::Cell(added.cell), Classification::on(entity))?;
                cell_entity.push(entity);
            }
        }
        // Face classification by vertex set.
        let mut by_set: BTreeMap<SmallVec<[VertexIx; 4]>, Target> = BTreeMap::new();
        for (t, fm) in &self.faces {
            for k in 0..fm.len() {
                let mut s: SmallVec<[VertexIx; 4]> = SmallVec::new();
                for key in fm.element(k) {
                    match map.vertices.get(key) {
                        Some(&v) => s.push(v),
                        None => break,
                    }
                }
                if s.len() == fm.element(k).len() {
                    s.sort_unstable();
                    by_set.insert(s, *t);
                }
            }
        }
        let volume_of = |e: EntityId| match e.kind() {
            sm_core::EntityKind::PartitionCell => model.piece_volume(PieceRef::Cell(e)),
            _ => e,
        };
        for f in 0..b.face_count() {
            let fi = FaceIx::new(f);
            let mut s: SmallVec<[VertexIx; 4]> = b.face_loop(fi).iter().copied().collect();
            s.sort_unstable();
            let class = match by_set.get(&s) {
                Some(t) => match t.class_entity() {
                    Some(e) => Some(e),
                    None => b.face_cell(fi, Side::Left).map(|c| volume_of(cell_entity[c.get()])),
                },
                None => match (b.face_cell(fi, Side::Left), b.face_cell(fi, Side::Right)) {
                    (Some(l), Some(r)) if cell_entity[l.get()] != cell_entity[r.get()] => {
                        let (vl, vr) = (volume_of(cell_entity[l.get()]), volume_of(cell_entity[r.get()]));
                        (vl == vr).then_some(vl)
                    }
                    _ => None,
                },
            };
            if let Some(e) = class {
                b.set_class(MeshEntityRef::Face(fi), Classification::on(e))?;
            }
        }
        Ok(map)
    }
}

struct Fnv(u64);

impl Fnv {
    fn new() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
    fn u(&mut self, w: u64) {
        for b in w.to_le_bytes() {
            self.0 ^= u64::from(b);
            self.0 = self.0.wrapping_mul(0x0100_0000_01b3);
        }
    }
    fn f(&mut self, x: f64) {
        self.u(x.to_bits());
    }
    fn target(&mut self, t: &Target) {
        let side = |k: &Keep| u64::from(*k == Keep::Positive);
        match t {
            Target::Topo(id) => {
                self.u(1);
                self.u(id.to_raw());
            }
            Target::Piece(PieceRef::Whole(v)) => {
                self.u(2);
                self.u(v.to_raw());
            }
            Target::Piece(PieceRef::Cell(c)) => {
                self.u(3);
                self.u(c.to_raw());
            }
            Target::PieceFace(PieceFace::Whole(f)) => {
                self.u(4);
                self.u(f.to_raw());
            }
            Target::PieceFace(PieceFace::Part { face, piece }) => {
                self.u(5);
                self.u(face.to_raw());
                self.u(piece.to_raw());
            }
            Target::PieceFace(PieceFace::Cut { cut, side: s }) => {
                self.u(6);
                self.u(u64::from(cut.0));
                self.u(side(s));
            }
            Target::PieceFace(PieceFace::CutPart { cut, side: s, piece }) => {
                self.u(7);
                self.u(u64::from(cut.0));
                self.u(side(s));
                self.u(piece.to_raw());
            }
        }
    }
    fn key(&mut self, k: &NodeKey) {
        self.target(&k.owner);
        self.u(u64::from(k.local));
    }
}
