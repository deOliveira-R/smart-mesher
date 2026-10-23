//! In-memory reference implementation of the mesh contract.
//!
//! [`MeshBuilder`] implements [`MeshEdit`]; [`MeshBuilder::finish`] yields an immutable
//! [`Mesh`] implementing every read trait. Storage is CSR with `u64` offsets and `u32`
//! indices; coordinates are SoA. Everything is in canonical order: faces are numbered by
//! first use in ascending cell order, face loops start at their smallest vertex index,
//! edges are sorted by `(low, high)` vertex pair, supports are ascending.

use crate::error::MeshError;
use crate::point::{NONE, Orient, Point, Stratum};
use crate::shape::{Basis, CellGeometry, CellShape, NodeRef};
use crate::traits::{
    CellAdded, CellSpec, Classification, Cone, FaceLoop, FaceSides, MeshClassification, MeshEdit, MeshGeometry,
    MeshLabels, MeshTopology, Orients, model_dim,
};
use sm_core::{EntityId, Point3};
use smallvec::SmallVec;
use std::collections::BTreeMap;

mod grid;
pub use grid::{hex_block, hex_block_builder};

const SIDE_ORIENT: [Orient; 2] = [Orient::Fwd, Orient::Rev];

/// Immutable in-memory mesh.
#[derive(Clone, Debug)]
pub struct Mesh {
    id: EntityId,
    base: u64,
    coords: [Vec<f64>; 3],
    extra: [Vec<f64>; 3],
    class: [Vec<Option<Classification>>; 4],
    f_off: Vec<u64>,
    f_verts: Vec<u32>,
    f_sides: Vec<[u32; 2]>,
    c_off: Vec<u64>,
    c_faces: Vec<u32>,
    c_orient: Vec<Orient>,
    c_shape: Vec<CellShape>,
    c_geom: Vec<CellGeometry>,
    cn_off: Vec<u64>,
    cn: Vec<NodeRef>,
    // Derived adjacency, built by `finish`.
    e_verts: Vec<u32>,
    fe_idx: Vec<u32>,
    fe_orient: Vec<Orient>,
    es_off: Vec<u64>,
    es_idx: Vec<u32>,
    es_orient: Vec<Orient>,
    vs_off: Vec<u64>,
    vs_idx: Vec<u32>,
    class_inv: BTreeMap<EntityId, Vec<Point>>,
    labels: BTreeMap<String, BTreeMap<Point, i32>>,
}

fn range(off: &[u64], i: u32) -> std::ops::Range<usize> {
    off[i as usize] as usize..off[i as usize + 1] as usize
}

impl MeshTopology for Mesh {
    fn mesh_id(&self) -> EntityId {
        self.id
    }
    fn count(&self, s: Stratum) -> u32 {
        match s {
            Stratum::Vertex => self.coords[0].len() as u32,
            Stratum::Edge => (self.e_verts.len() / 2) as u32,
            Stratum::Face => self.f_sides.len() as u32,
            Stratum::Cell => self.c_shape.len() as u32,
        }
    }
    fn entity_id(&self, p: Point) -> EntityId {
        // Serial overflow is impossible for u32 indices below a 56-bit base; fall back to the mesh id.
        EntityId::new(p.stratum.entity_kind(), self.base + u64::from(p.index)).unwrap_or(self.id)
    }
    fn point_of(&self, id: EntityId) -> Option<Point> {
        let s = Stratum::from_entity_kind(id.kind())?;
        let i = id.serial().checked_sub(self.base)?;
        let p = Point::new(s, u32::try_from(i).ok()?);
        self.contains(p).then_some(p)
    }
    fn cone(&self, p: Point) -> Cone<'_> {
        let i = p.index;
        match p.stratum {
            Stratum::Vertex => Cone::EMPTY,
            Stratum::Edge => Cone {
                stratum: Stratum::Vertex,
                indices: &self.e_verts[2 * i as usize..2 * i as usize + 2],
                orients: Orients::Uniform(Orient::Fwd),
            },
            Stratum::Face => Cone {
                stratum: Stratum::Vertex,
                indices: &self.f_verts[range(&self.f_off, i)],
                orients: Orients::Uniform(Orient::Fwd),
            },
            Stratum::Cell => {
                let r = range(&self.c_off, i);
                Cone { stratum: Stratum::Face, indices: &self.c_faces[r.clone()], orients: Orients::Slice(&self.c_orient[r]) }
            }
        }
    }
    fn support(&self, p: Point) -> Cone<'_> {
        let i = p.index;
        match p.stratum {
            Stratum::Vertex => Cone {
                stratum: Stratum::Face,
                indices: &self.vs_idx[range(&self.vs_off, i)],
                orients: Orients::Uniform(Orient::Fwd),
            },
            Stratum::Edge => {
                let r = range(&self.es_off, i);
                Cone { stratum: Stratum::Face, indices: &self.es_idx[r.clone()], orients: Orients::Slice(&self.es_orient[r]) }
            }
            Stratum::Face => {
                let s = &self.f_sides[i as usize];
                let n = if s[1] == NONE { 1 } else { 2 };
                Cone { stratum: Stratum::Cell, indices: &s[..n], orients: Orients::Slice(&SIDE_ORIENT[..n]) }
            }
            Stratum::Cell => Cone::EMPTY,
        }
    }
    fn face_sides(&self, face: Point) -> FaceSides {
        let s = self.f_sides[face.index as usize];
        FaceSides { left: Point::cell(s[0]), right: (s[1] != NONE).then(|| Point::cell(s[1])) }
    }
    fn cell_shape(&self, cell: Point) -> CellShape {
        self.c_shape[cell.index as usize]
    }
    fn face_edges(&self, face: Point) -> Cone<'_> {
        let r = range(&self.f_off, face.index);
        Cone { stratum: Stratum::Edge, indices: &self.fe_idx[r.clone()], orients: Orients::Slice(&self.fe_orient[r]) }
    }
}

impl MeshGeometry for Mesh {
    fn coords_soa(&self) -> [&[f64]; 3] {
        [&self.coords[0], &self.coords[1], &self.coords[2]]
    }
    fn node_coord(&self, n: NodeRef) -> Point3 {
        match (n.as_vertex(), n.as_extra()) {
            (Some(v), _) => self.vertex_coord(v),
            (_, Some(i)) => Point3::new(self.extra[0][i as usize], self.extra[1][i as usize], self.extra[2][i as usize]),
            _ => Point3::default(),
        }
    }
    fn cell_geometry(&self, cell: Point) -> CellGeometry {
        self.c_geom[cell.index as usize]
    }
    fn cell_nodes(&self, cell: Point) -> &[NodeRef] {
        &self.cn[range(&self.cn_off, cell.index)]
    }
}

impl MeshClassification for Mesh {
    fn classification(&self, p: Point) -> Option<Classification> {
        self.class[p.stratum as usize].get(p.index as usize).copied().flatten()
    }
    fn points_on(&self, model: EntityId) -> Vec<Point> {
        self.class_inv.get(&model).cloned().unwrap_or_default()
    }
    fn classified_entities(&self) -> Vec<EntityId> {
        self.class_inv.keys().copied().collect()
    }
}

impl MeshLabels for Mesh {
    fn label_names(&self) -> Vec<String> {
        self.labels.keys().cloned().collect()
    }
    fn label_value(&self, name: &str, p: Point) -> Option<i32> {
        self.labels.get(name)?.get(&p).copied()
    }
    fn label_points(&self, name: &str, value: i32) -> Vec<Point> {
        self.labels.get(name).map(|m| m.iter().filter(|&(_, &v)| v == value).map(|(&p, _)| p).collect()).unwrap_or_default()
    }
    fn label_entries(&self, name: &str) -> Vec<(Point, i32)> {
        self.labels.get(name).map(|m| m.iter().map(|(&p, &v)| (p, v)).collect()).unwrap_or_default()
    }
}

/// Cyclic equality of two vertex loops (same direction, any start).
fn cyc_eq(a: &[u32], b: &[u32]) -> bool {
    a.len() == b.len() && b.iter().position(|&x| x == a[0]).is_some_and(|s| (0..a.len()).all(|i| a[i] == b[(s + i) % b.len()]))
}

/// Builder for [`Mesh`]; implements [`MeshEdit`].
#[derive(Debug)]
pub struct MeshBuilder {
    m: Mesh,
    face_key: BTreeMap<Vec<u32>, u32>,
    edge_class: BTreeMap<(u32, u32), Classification>,
}

impl MeshBuilder {
    /// Start a mesh with id `mesh` whose entity serials are `serial_base + dense index`.
    /// Use disjoint bases for meshes sharing a model so ids stay globally unique.
    pub fn new(mesh: EntityId, serial_base: u64) -> Self {
        let m = Mesh {
            id: mesh,
            base: serial_base,
            coords: Default::default(),
            extra: Default::default(),
            class: Default::default(),
            f_off: vec![0],
            f_verts: vec![],
            f_sides: vec![],
            c_off: vec![0],
            c_faces: vec![],
            c_orient: vec![],
            c_shape: vec![],
            c_geom: vec![],
            cn_off: vec![0],
            cn: vec![],
            e_verts: vec![],
            fe_idx: vec![],
            fe_orient: vec![],
            es_off: vec![0],
            es_idx: vec![],
            es_orient: vec![],
            vs_off: vec![0],
            vs_idx: vec![],
            class_inv: BTreeMap::new(),
            labels: BTreeMap::new(),
        };
        Self { m, face_key: BTreeMap::new(), edge_class: BTreeMap::new() }
    }

    fn check(&self, p: Point) -> Result<(), MeshError> {
        let n = match p.stratum {
            Stratum::Vertex => self.m.coords[0].len(),
            Stratum::Edge => 0,
            Stratum::Face => self.m.f_sides.len(),
            Stratum::Cell => self.m.c_shape.len(),
        };
        if (p.index as usize) < n { Ok(()) } else { Err(MeshError::InvalidPoint(p)) }
    }

    fn vertex_index(&self, p: Point) -> Result<u32, MeshError> {
        if p.stratum != Stratum::Vertex {
            return Err(MeshError::InvalidPoint(p));
        }
        self.check(p)?;
        Ok(p.index)
    }
}

enum Plan {
    New(Vec<u32>),
    Existing(u32),
}

impl MeshEdit for MeshBuilder {
    type Output = Mesh;

    fn add_vertex(&mut self, pos: Point3, class: Option<Classification>) -> Point {
        let m = &mut self.m;
        m.coords[0].push(pos.x);
        m.coords[1].push(pos.y);
        m.coords[2].push(pos.z);
        m.class[0].push(class);
        Point::vertex(m.coords[0].len() as u32 - 1)
    }

    fn add_cell(&mut self, spec: &CellSpec) -> Result<CellAdded, MeshError> {
        // Phase 1: validate and plan, without mutating.
        let mut plans = Vec::with_capacity(spec.faces.len());
        let mut sizes = Vec::new();
        let mut all_v: Vec<u32> = Vec::new();
        let mut keys: Vec<Vec<u32>> = Vec::new();
        for lp in &spec.faces {
            let l: Vec<u32> = lp.iter().map(|&p| self.vertex_index(p)).collect::<Result<_, _>>()?;
            let mut key = l.clone();
            key.sort_unstable();
            key.dedup();
            if l.len() < 3 || key.len() != l.len() {
                return Err(MeshError::DegenerateLoop);
            }
            if keys.contains(&key) {
                return Err(MeshError::DuplicateFace);
            }
            sizes.push(l.len());
            all_v.extend_from_slice(&key);
            match self.face_key.get(&key) {
                Some(&f) => {
                    let stored = &self.m.f_verts[range(&self.m.f_off, f)];
                    let rev: Vec<u32> = l.iter().rev().copied().collect();
                    if cyc_eq(&l, stored) {
                        return Err(MeshError::InconsistentOrientation);
                    }
                    if !cyc_eq(&rev, stored) {
                        return Err(MeshError::FaceCycleMismatch);
                    }
                    if self.m.f_sides[f as usize][1] != NONE {
                        return Err(MeshError::NonManifoldFace);
                    }
                    plans.push(Plan::Existing(f));
                }
                None => {
                    let s = l.iter().enumerate().min_by_key(|&(_, &v)| v).map_or(0, |(i, _)| i);
                    plans.push(Plan::New((0..l.len()).map(|i| l[(s + i) % l.len()]).collect()));
                }
            }
            keys.push(key);
        }
        all_v.sort_unstable();
        all_v.dedup();
        let detected = CellShape::detect(all_v.len(), &sizes);
        if spec.shape != CellShape::Polyhedron && spec.shape != detected {
            return Err(MeshError::ShapeMismatch { tag: format!("{:?}", spec.shape), detected: format!("{detected:?}") });
        }
        if let Some(c) = spec.class
            && model_dim(c.entity).is_none()
        {
            return Err(MeshError::NotAModelEntity(c.entity));
        }
        // Phase 2: commit.
        let cell = self.m.c_shape.len() as u32;
        let mut faces: SmallVec<[Point; 6]> = SmallVec::new();
        for (plan, key) in plans.into_iter().zip(keys) {
            match plan {
                Plan::Existing(f) => {
                    self.m.f_sides[f as usize][1] = cell;
                    self.m.c_faces.push(f);
                    self.m.c_orient.push(Orient::Rev);
                    faces.push(Point::face(f));
                }
                Plan::New(l) => {
                    let f = self.m.f_sides.len() as u32;
                    self.m.f_verts.extend_from_slice(&l);
                    self.m.f_off.push(self.m.f_verts.len() as u64);
                    self.m.f_sides.push([cell, NONE]);
                    self.m.class[2].push(None);
                    self.face_key.insert(key, f);
                    self.m.c_faces.push(f);
                    self.m.c_orient.push(Orient::Fwd);
                    faces.push(Point::face(f));
                }
            }
        }
        self.m.c_off.push(self.m.c_faces.len() as u64);
        self.m.c_shape.push(spec.shape);
        self.m.c_geom.push(CellGeometry::LINEAR);
        self.m.cn_off.push(self.m.cn.len() as u64);
        self.m.class[3].push(spec.class);
        Ok(CellAdded { cell: Point::cell(cell), faces })
    }

    fn add_node(&mut self, pos: Point3) -> NodeRef {
        let e = &mut self.m.extra;
        e[0].push(pos.x);
        e[1].push(pos.y);
        e[2].push(pos.z);
        NodeRef::extra(e[0].len() as u32 - 1)
    }

    fn set_cell_order(&mut self, cell: Point, degree: u8, basis: Basis, nodes: &[NodeRef]) -> Result<(), MeshError> {
        if cell.stratum != Stratum::Cell || cell.index as usize >= self.m.c_shape.len() {
            return Err(MeshError::InvalidPoint(cell));
        }
        let (nv, ne) = (self.m.coords[0].len() as u32, self.m.extra[0].len() as u32);
        if nodes.iter().any(|n| n.as_vertex().is_some_and(|v| v >= nv) || n.as_extra().is_some_and(|e| e >= ne)) {
            return Err(MeshError::InvalidPoint(cell));
        }
        // Rebuild this cell's node slice in the CSR (cheap: high-order setup is rare).
        let i = cell.index as usize;
        let (a, b) = (self.m.cn_off[i] as usize, self.m.cn_off[i + 1] as usize);
        self.m.cn.splice(a..b, nodes.iter().copied());
        let delta = nodes.len() as i64 - (b - a) as i64;
        for o in &mut self.m.cn_off[i + 1..] {
            *o = (*o as i64 + delta) as u64;
        }
        self.m.c_geom[i] = CellGeometry { degree, basis };
        Ok(())
    }

    fn classify(&mut self, p: Point, class: Classification) -> Result<(), MeshError> {
        if p.stratum == Stratum::Edge {
            return Err(MeshError::InvalidPoint(p));
        }
        self.check(p)?;
        if model_dim(class.entity).is_none() {
            return Err(MeshError::NotAModelEntity(class.entity));
        }
        self.m.class[p.stratum as usize][p.index as usize] = Some(class);
        Ok(())
    }

    fn classify_edge(&mut self, a: Point, b: Point, class: Classification) -> Result<(), MeshError> {
        let (a, b) = (self.vertex_index(a)?, self.vertex_index(b)?);
        if model_dim(class.entity).is_none() {
            return Err(MeshError::NotAModelEntity(class.entity));
        }
        self.edge_class.insert((a.min(b), a.max(b)), class);
        Ok(())
    }

    fn set_label(&mut self, name: &str, p: Point, value: i32) -> Result<(), MeshError> {
        if p.stratum == Stratum::Edge {
            return Err(MeshError::InvalidPoint(p));
        }
        self.check(p)?;
        self.m.labels.entry(name.to_string()).or_default().insert(p, value);
        Ok(())
    }

    fn finish(self) -> Result<Mesh, MeshError> {
        let Self { mut m, edge_class, .. } = self;
        let nv = m.coords[0].len();
        let nf = m.f_sides.len();
        // Edges: sorted distinct (low, high) pairs.
        let mut pairs: Vec<(u32, u32)> = Vec::with_capacity(m.f_verts.len());
        for f in 0..nf as u32 {
            let l = &m.f_verts[range(&m.f_off, f)];
            for i in 0..l.len() {
                let (a, b) = (l[i], l[(i + 1) % l.len()]);
                pairs.push((a.min(b), a.max(b)));
            }
        }
        pairs.sort_unstable();
        pairs.dedup();
        m.e_verts = pairs.iter().flat_map(|&(a, b)| [a, b]).collect();
        m.class[1] = pairs.iter().map(|p| edge_class.get(p).copied()).collect();
        if let Some(&(a, b)) = edge_class.keys().find(|k| pairs.binary_search(k).is_err()) {
            return Err(MeshError::NoSuchEdge(a, b));
        }
        // face -> edge (with orientation), edge -> face, vertex -> face.
        let mut es: Vec<Vec<(u32, Orient)>> = vec![Vec::new(); pairs.len()];
        let mut vs: Vec<Vec<u32>> = vec![Vec::new(); nv];
        for f in 0..nf as u32 {
            let l = m.f_verts[range(&m.f_off, f)].to_vec();
            for i in 0..l.len() {
                let (a, b) = (l[i], l[(i + 1) % l.len()]);
                let e = pairs.binary_search(&(a.min(b), a.max(b))).map_err(|_| MeshError::NoSuchEdge(a, b))? as u32;
                let o = if a < b { Orient::Fwd } else { Orient::Rev };
                m.fe_idx.push(e);
                m.fe_orient.push(o);
                es[e as usize].push((f, o));
                vs[a as usize].push(f);
            }
        }
        for row in &es {
            for &(f, o) in row {
                m.es_idx.push(f);
                m.es_orient.push(o);
            }
            m.es_off.push(m.es_idx.len() as u64);
        }
        for row in &vs {
            m.vs_idx.extend_from_slice(row);
            m.vs_off.push(m.vs_idx.len() as u64);
        }
        for s in Stratum::ALL {
            let n = m.count(s);
            for i in 0..n {
                if let Some(c) = m.class[s as usize][i as usize] {
                    m.class_inv.entry(c.entity).or_default().push(Point::new(s, i));
                }
            }
        }
        Ok(m)
    }
}

/// Convenience: a face loop from raw vertex indices.
pub fn loop_of(vs: &[u32]) -> FaceLoop {
    vs.iter().map(|&v| Point::vertex(v)).collect()
}
