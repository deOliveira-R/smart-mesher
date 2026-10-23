//! Reference builder: collects vertices, faces and oriented cells, then `finish` normalises.

use super::ReferenceMesh;
use crate::build::{MeshBuild, SerialAlloc};
use crate::classify::{ClassSet, Classification};
use crate::error::{MeshError, Result};
use crate::highorder::{Basis, HighOrderCell, NodeRef};
use crate::index::{CellFace, CellIx, FaceIx, MeshEntity, MeshEntityKind, Side, VertexIx};
use crate::sets::{MeshSet, SetMembers, TagColumn};
use crate::shape::{CellShape, LoopRelation, loop_relation};
use sm_core::{EntityId, EntityKind, Point3};
use std::collections::BTreeMap;

struct BCell {
    faces: Vec<CellFace>,
    ordered: Option<Vec<VertexIx>>,
}

/// Builder producing a [`ReferenceMesh`].
///
/// `finish` guarantees: every face has one or two cells; the lower cell is `left` and sees the
/// loop normal as outward (loops are reversed where needed); faces are sorted by
/// `(left, right, loop)` with loops starting at their smallest vertex; ids are assigned from the
/// allocator in dense order; the result passes [`crate::validate::validate`].
pub struct ReferenceBuilder {
    mesh_id: EntityId,
    pts: Vec<Point3>,
    extra: Vec<Point3>,
    vclass: Vec<Option<Classification>>,
    faces: Vec<Vec<VertexIx>>,
    fclass: Vec<Option<Classification>>,
    face_key: BTreeMap<Vec<VertexIx>, Vec<FaceIx>>,
    face_deg: Vec<usize>,
    cells: Vec<BCell>,
    cclass: Vec<Option<Classification>>,
    ho: BTreeMap<u32, (u8, Basis, Vec<NodeRef>)>,
    sets: Vec<(String, SetMembers)>,
    tags: Vec<(String, MeshEntityKind, Vec<i64>)>,
}

impl ReferenceBuilder {
    /// Start a mesh with the given id (`EntityKind::Mesh`).
    pub fn new(mesh_id: EntityId) -> Result<Self> {
        if mesh_id.kind() != EntityKind::Mesh {
            return Err(MeshError::State("mesh id must have kind Mesh".into()));
        }
        Ok(Self {
            mesh_id,
            pts: vec![],
            extra: vec![],
            vclass: vec![],
            faces: vec![],
            fclass: vec![],
            face_key: BTreeMap::new(),
            face_deg: vec![],
            cells: vec![],
            cclass: vec![],
            ho: BTreeMap::new(),
            sets: vec![],
            tags: vec![],
        })
    }
}

impl MeshBuild for ReferenceBuilder {
    type Output = ReferenceMesh;

    fn add_vertex(&mut self, p: Point3) -> Result<VertexIx> {
        if self.pts.len() >= u32::MAX as usize - 1 {
            return Err(MeshError::SerialOverflow);
        }
        self.pts.push(p);
        self.vclass.push(None);
        Ok(VertexIx::new(self.pts.len() - 1))
    }
    fn add_extra_node(&mut self, p: Point3) -> Result<NodeRef> {
        self.extra.push(p);
        Ok(NodeRef::Extra(self.extra.len() as u32 - 1))
    }
    fn vertex_count(&self) -> usize {
        self.pts.len()
    }
    fn face_count(&self) -> usize {
        self.faces.len()
    }
    fn cell_count(&self) -> usize {
        self.cells.len()
    }
    fn vertex_point(&self, v: VertexIx) -> Point3 {
        self.pts[v.get()]
    }
    fn find_vertex(&self, p: Point3, tol: f64) -> Option<VertexIx> {
        // O(n); a production backend would use a deterministic grid.
        self.pts
            .iter()
            .position(|q| (q.x - p.x).abs() <= tol && (q.y - p.y).abs() <= tol && (q.z - p.z).abs() <= tol)
            .map(VertexIx::new)
    }

    fn add_face(&mut self, l: &[VertexIx]) -> Result<FaceIx> {
        let mut key = l.to_vec();
        key.sort_unstable();
        key.dedup();
        if l.len() < 3 || key.len() != l.len() {
            return Err(MeshError::InvalidLoop("needs at least 3 distinct vertices".into()));
        }
        if let Some(v) = l.iter().find(|v| v.get() >= self.pts.len()) {
            return Err(MeshError::UnknownVertex(*v));
        }
        let f = FaceIx::new(self.faces.len());
        self.faces.push(l.to_vec());
        self.fclass.push(None);
        self.face_deg.push(0);
        self.face_key.entry(key).or_default().push(f);
        Ok(f)
    }
    fn find_face(&self, l: &[VertexIx]) -> Option<(FaceIx, LoopRelation)> {
        let mut key = l.to_vec();
        key.sort_unstable();
        self.face_key.get(&key)?.iter().find_map(|&f| loop_relation(l, &self.faces[f.get()]).map(|r| (f, r)))
    }
    fn face_loop(&self, f: FaceIx) -> &[VertexIx] {
        &self.faces[f.get()]
    }
    fn face_degree(&self, f: FaceIx) -> usize {
        self.face_deg[f.get()]
    }

    fn add_cell(&mut self, faces: &[CellFace], ordered: Option<&[VertexIx]>) -> Result<CellIx> {
        let c = CellIx::new(self.cells.len());
        let mut seen: Vec<FaceIx> = faces.iter().map(|cf| cf.face).collect();
        seen.sort_unstable();
        if seen.windows(2).any(|w| w[0] == w[1]) {
            return Err(MeshError::RepeatedCellFace(c));
        }
        if let Some(cf) = faces.iter().find(|cf| cf.face.get() >= self.faces.len()) {
            return Err(MeshError::OutOfRange(format!("face {:?}", cf.face)));
        }
        if faces.len() < 4 {
            return Err(MeshError::State("a cell needs at least 4 faces".into()));
        }
        if let Some(ov) = ordered
            && (CellShape::from_ordered_len(ov.len()).is_none() || ov.iter().any(|v| v.get() >= self.pts.len())) {
                return Err(MeshError::State("ordered vertices do not describe a fast-path shape".into()));
            }
        for cf in faces {
            self.face_deg[cf.face.get()] += 1;
        }
        self.cells.push(BCell { faces: faces.to_vec(), ordered: ordered.map(<[VertexIx]>::to_vec) });
        self.cclass.push(None);
        Ok(c)
    }

    fn set_class(&mut self, e: MeshEntity, c: Classification) -> Result<()> {
        if c.dim().is_none() {
            return Err(MeshError::State("classification must name a model or partition entity".into()));
        }
        let oob = || MeshError::OutOfRange(format!("{e:?}"));
        let slot = match e {
            MeshEntity::Vertex(v) => self.vclass.get_mut(v.get()),
            MeshEntity::Face(f) => self.fclass.get_mut(f.get()),
            MeshEntity::Cell(c) => self.cclass.get_mut(c.get()),
            MeshEntity::Edge(_) => return Err(MeshError::State("edges are classified on the finished mesh".into())),
        };
        *slot.ok_or_else(oob)? = Some(c);
        Ok(())
    }
    fn set_high_order(&mut self, c: CellIx, degree: u8, basis: Basis, nodes: &[NodeRef]) -> Result<()> {
        if c.get() >= self.cells.len() || degree < 2 {
            return Err(MeshError::OutOfRange("high-order cell".into()));
        }
        self.ho.insert(c.0, (degree, basis, nodes.to_vec()));
        Ok(())
    }
    fn add_set(&mut self, name: &str, members: SetMembers) -> Result<()> {
        if self.sets.iter().any(|(n, _)| n == name) {
            return Err(MeshError::DuplicateName(name.into()));
        }
        self.sets.push((name.into(), members));
        Ok(())
    }
    fn add_tag(&mut self, name: &str, target: MeshEntityKind, values: Vec<i64>) -> Result<()> {
        if target == MeshEntityKind::Edge {
            return Err(MeshError::State("edge tags belong on the finished mesh".into()));
        }
        if self.tags.iter().any(|(n, ..)| n == name) {
            return Err(MeshError::DuplicateName(name.into()));
        }
        self.tags.push((name.into(), target, values));
        Ok(())
    }

    fn finish(self, alloc: &mut SerialAlloc) -> Result<ReferenceMesh> {
        let (nv, nf, nc) = (self.pts.len(), self.faces.len(), self.cells.len());
        let vclass: Vec<Classification> = self
            .vclass
            .iter()
            .enumerate()
            .map(|(i, c)| c.ok_or(MeshError::Unclassified(MeshEntity::Vertex(VertexIx::new(i)))))
            .collect::<Result<_>>()?;
        let cclass: Vec<Classification> = self
            .cclass
            .iter()
            .enumerate()
            .map(|(i, c)| c.ok_or(MeshError::Unclassified(MeshEntity::Cell(CellIx::new(i)))))
            .collect::<Result<_>>()?;
        let fclass_old: Vec<Classification> = self
            .fclass
            .iter()
            .enumerate()
            .map(|(i, c)| c.ok_or(MeshError::Unclassified(MeshEntity::Face(FaceIx::new(i)))))
            .collect::<Result<_>>()?;

        // Incidences, ascending by cell.
        let mut inc: Vec<Vec<(CellIx, Side)>> = vec![vec![]; nf];
        for (ci, c) in self.cells.iter().enumerate() {
            for cf in &c.faces {
                inc[cf.face.get()].push((CellIx::new(ci), cf.side));
            }
        }
        // Apply the orientation rule per face.
        struct Fin {
            old: usize,
            lp: Vec<VertexIx>,
            left: CellIx,
            right: u32,
        }
        let mut fin: Vec<Fin> = Vec::with_capacity(nf);
        for (f, i) in inc.iter().enumerate() {
            let fi = FaceIx::new(f);
            if i.is_empty() {
                return Err(MeshError::DanglingFace(fi));
            }
            if i.len() > 2 {
                return Err(MeshError::NonManifoldFace(fi));
            }
            if i.len() == 2 && i[0].1 == i[1].1 {
                return Err(MeshError::InconsistentOrientation(fi));
            }
            let mut lp = self.faces[f].clone();
            if i[0].1 == Side::Right {
                lp.reverse();
            }
            let start = lp.iter().enumerate().min_by_key(|(_, v)| **v).map_or(0, |(p, _)| p);
            lp.rotate_left(start);
            fin.push(Fin { old: f, lp, left: i[0].0, right: i.get(1).map_or(CellIx::NONE, |x| x.0.0) });
        }
        fin.sort_by(|a, b| (a.left, a.right, &a.lp).cmp(&(b.left, b.right, &b.lp)));
        if let Some(w) = fin.windows(2).position(|w| w[0].left == w[1].left && w[0].right == w[1].right && w[0].lp == w[1].lp) {
            return Err(MeshError::DuplicateFace(FaceIx::new(w)));
        }
        let mut new_of_old = vec![0u32; nf];
        for (n, f) in fin.iter().enumerate() {
            new_of_old[f.old] = n as u32;
        }

        // Face CSR.
        let mut face_off = vec![0u32];
        let mut face_verts = Vec::new();
        let mut face_left = Vec::with_capacity(nf);
        let mut face_right = Vec::with_capacity(nf);
        let mut fclass = Vec::with_capacity(nf);
        for f in &fin {
            face_verts.extend_from_slice(&f.lp);
            face_off.push(face_verts.len() as u32);
            face_left.push(f.left);
            face_right.push(f.right);
            fclass.push(fclass_old[f.old]);
        }

        // Cell CSR, shapes, ordered vertices.
        let (mut cell_off, mut cell_faces, mut ordered_off, mut ordered) = (vec![0u32], Vec::new(), vec![0u32], Vec::new());
        let mut cell_shape = Vec::with_capacity(nc);
        for (ci, c) in self.cells.iter().enumerate() {
            let ci = CellIx::new(ci);
            let mut cfs: Vec<CellFace> = c
                .faces
                .iter()
                .map(|cf| {
                    let nf_ix = new_of_old[cf.face.get()];
                    let side = if face_left[nf_ix as usize] == ci { Side::Left } else { Side::Right };
                    CellFace { face: FaceIx(nf_ix), side }
                })
                .collect();
            cfs.sort_by_key(|cf| cf.face);
            let mut vs: Vec<VertexIx> =
                cfs.iter().flat_map(|cf| face_verts[face_off[cf.face.get()] as usize..face_off[cf.face.get() + 1] as usize].iter().copied()).collect();
            vs.sort_unstable();
            vs.dedup();
            let shape = CellShape::detect(
                cfs.iter().map(|cf| (face_off[cf.face.get() + 1] - face_off[cf.face.get()]) as usize),
                vs.len(),
            );
            if let Some(ov) = &c.ordered {
                if CellShape::from_ordered_len(ov.len()) != Some(shape) {
                    return Err(MeshError::ShapeMismatch(ci, format!("ordered vertex list of {} does not match topology {shape:?}", ov.len())));
                }
                ordered.extend_from_slice(ov);
            }
            ordered_off.push(ordered.len() as u32);
            cell_faces.extend_from_slice(&cfs);
            cell_off.push(cell_faces.len() as u32);
            cell_shape.push(shape);
        }

        // Ids.
        let ids = |alloc: &mut SerialAlloc, kind: EntityKind, n: usize| -> Result<Vec<EntityId>> {
            let first = alloc.take(kind, n)?;
            (0..n as u64).map(|i| EntityId::new(kind, first + i).ok_or(MeshError::SerialOverflow)).collect()
        };
        let vertex_ids = ids(alloc, EntityKind::MeshVertex, nv)?;
        let face_ids = ids(alloc, EntityKind::MeshFace, nf)?;
        let cell_ids = ids(alloc, EntityKind::MeshCell, nc)?;

        // Coordinates, high order.
        let all: Vec<Point3> = self.pts.iter().chain(self.extra.iter()).copied().collect();
        let mut ho = BTreeMap::new();
        for (c, (degree, basis, nodes)) in &self.ho {
            let nodes: Vec<u32> = nodes
                .iter()
                .map(|n| match *n {
                    NodeRef::Vertex(v) => v.0,
                    NodeRef::Extra(k) => (nv as u32) + k,
                })
                .collect();
            if nodes.iter().any(|&n| n as usize >= all.len()) {
                return Err(MeshError::OutOfRange(format!("high-order node of cell {c}")));
            }
            ho.insert(*c, HighOrderCell { degree: *degree, basis: *basis, nodes });
        }

        // Classification index.
        let mut class_index: BTreeMap<EntityId, ClassSet> = BTreeMap::new();
        for (i, c) in vclass.iter().enumerate() {
            class_index.entry(c.entity).or_default().vertices.push(VertexIx::new(i));
        }
        for (i, c) in fclass.iter().enumerate() {
            class_index.entry(c.entity).or_default().faces.push(FaceIx::new(i));
        }
        for (i, c) in cclass.iter().enumerate() {
            class_index.entry(c.entity).or_default().cells.push(CellIx::new(i));
        }

        // Sets and tags, remapped to final face numbering.
        let set_ids = ids(alloc, EntityKind::Group, self.sets.len())?;
        let mut sets = Vec::new();
        for ((name, members), id) in self.sets.into_iter().zip(set_ids) {
            let members = match members {
                SetMembers::Faces(v) => {
                    if let Some(f) = v.iter().find(|f| f.get() >= nf) {
                        return Err(MeshError::OutOfRange(format!("set '{name}' face {f:?}")));
                    }
                    let mut m: Vec<FaceIx> = v.iter().map(|f| FaceIx(new_of_old[f.get()])).collect();
                    m.sort_unstable();
                    m.dedup();
                    SetMembers::Faces(m)
                }
                SetMembers::Vertices(mut v) => {
                    v.sort_unstable();
                    v.dedup();
                    if v.last().is_some_and(|x| x.get() >= nv) {
                        return Err(MeshError::OutOfRange(format!("set '{name}' vertex")));
                    }
                    SetMembers::Vertices(v)
                }
                SetMembers::Cells(mut v) => {
                    v.sort_unstable();
                    v.dedup();
                    if v.last().is_some_and(|x| x.get() >= nc) {
                        return Err(MeshError::OutOfRange(format!("set '{name}' cell")));
                    }
                    SetMembers::Cells(v)
                }
            };
            sets.push(MeshSet { id, name, members });
        }
        let mut tags = Vec::new();
        for (name, target, values) in self.tags {
            let n = match target {
                MeshEntityKind::Vertex => nv,
                MeshEntityKind::Face => nf,
                MeshEntityKind::Cell => nc,
                MeshEntityKind::Edge => 0,
            };
            if values.len() != n {
                return Err(MeshError::OutOfRange(format!("tag '{name}' has {} values, expected {n}", values.len())));
            }
            let values = if target == MeshEntityKind::Face { fin.iter().map(|f| values[f.old]).collect() } else { values };
            tags.push(TagColumn { name, target, values });
        }

        let mesh = ReferenceMesh {
            id: self.mesh_id,
            vertex_ids,
            face_ids,
            cell_ids,
            x: all.iter().map(|p| p.x).collect(),
            y: all.iter().map(|p| p.y).collect(),
            z: all.iter().map(|p| p.z).collect(),
            n_vertices: nv,
            face_off,
            face_verts,
            face_left,
            face_right,
            cell_off,
            cell_faces,
            cell_shape,
            ordered_off,
            ordered,
            vclass,
            fclass,
            cclass,
            class_index,
            ho,
            sets,
            tags,
            edges: None,
        };
        let report = crate::validate::validate(&mesh);
        if let Some(i) = report.issues.first() {
            return Err(MeshError::Invalid(format!("{:?}: {}", i.kind, i.message)));
        }
        Ok(mesh)
    }
}
