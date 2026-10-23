//! In-memory reference implementor of the mesh contract.
//!
//! [`Mesh`] is a set of SoA tables (one per dimension) plus CSR relations; [`RefBuilder`]
//! produces it. It favours clarity over speed (face merging uses `BTreeMap`s for determinism)
//! but the produced layout is the one a fast backend is expected to mirror.

use std::collections::{BTreeMap, BTreeSet};

use indexmap::IndexMap;
use sm_core::{EntityId, EntityKind, Point3, Transform};

use crate::build::{MeshBuilder, MeshEdit};
use crate::classify::{ClassTable, Classification, check_classification};
use crate::error::{MeshError, MeshResult};
use crate::index::{CellIx, Dim, EdgeIx, FaceIx, Sign, SignedFace, VertexIx};
use crate::orient::{canonical_loop, face_key, loop_is_simple, reversed_loop};
use crate::query::{CoordsRef, FaceCurve, MeshClassification, MeshGeometry, MeshTags, MeshTopology};
use crate::schema::{MeshSchema, TagDef};
use crate::shape::{Basis, CellShape, CellSpec};
use crate::table::Csr;
use crate::tag::{EntitySet, TagColumn, TagRow};

const NONE: u32 = u32::MAX;

const fn kind_of(dim: Dim) -> EntityKind {
    match dim {
        Dim::Vertex => EntityKind::MeshVertex,
        Dim::Edge => EntityKind::MeshEdge,
        Dim::Face => EntityKind::MeshFace,
        Dim::Cell => EntityKind::MeshCell,
    }
}

const fn dim_of(kind: EntityKind) -> Option<Dim> {
    match kind {
        EntityKind::MeshVertex => Some(Dim::Vertex),
        EntityKind::MeshEdge => Some(Dim::Edge),
        EntityKind::MeshFace => Some(Dim::Face),
        EntityKind::MeshCell => Some(Dim::Cell),
        _ => None,
    }
}

/// A curved face record (sparse side table).
#[derive(Clone, Debug, PartialEq)]
struct CurveRec {
    degree: u8,
    basis: Basis,
    nodes: Vec<VertexIx>,
}

/// The reference mesh: typed columnar tables, face-first.
#[derive(Clone, Debug)]
pub struct Mesh {
    mesh_id: EntityId,
    schema: MeshSchema,
    ids: [Vec<EntityId>; 4],
    x: Vec<f64>,
    y: Vec<f64>,
    z: Vec<f64>,
    edge_verts: Vec<[VertexIx; 2]>,
    face_verts: Csr<VertexIx>,
    face_left: Vec<u32>,
    face_right: Vec<u32>,
    cell_shape: Vec<CellShape>,
    cell_degree: Vec<u8>,
    cell_basis: Vec<Basis>,
    cell_faces: Csr<SignedFace>,
    cell_corners: Csr<VertexIx>,
    cell_nodes: Csr<VertexIx>,
    curves: BTreeMap<u32, CurveRec>,
    class: [ClassTable; 4],
    tags: Vec<TagColumn>,
    sets: IndexMap<String, EntitySet>,
}

fn d(dim: Dim) -> usize {
    dim as usize
}

impl Mesh {
    /// Start a builder for a mesh with this id (kind `Mesh`).
    pub fn builder(mesh_id: EntityId) -> MeshResult<RefBuilder> {
        RefBuilder::new(mesh_id)
    }

    /// Record a curved face (degree > 1) on a polyhedron: the sparse curvature side table.
    pub fn set_face_curve(&mut self, f: FaceIx, degree: u8, basis: Basis, nodes: Vec<VertexIx>) -> MeshResult<()> {
        if f.index() >= self.face_left.len() {
            return Err(MeshError::OutOfRange { dim: Dim::Face, index: f.0, len: self.face_left.len() });
        }
        if let Some(bad) = nodes.iter().find(|v| v.index() >= self.x.len()) {
            return Err(MeshError::OutOfRange { dim: Dim::Vertex, index: bad.0, len: self.x.len() });
        }
        self.curves.insert(f.0, CurveRec { degree, basis, nodes });
        Ok(())
    }

    fn check_ix(&self, dim: Dim, ix: u32) -> MeshResult<()> {
        let len = self.ids[d(dim)].len();
        if (ix as usize) < len { Ok(()) } else { Err(MeshError::OutOfRange { dim, index: ix, len }) }
    }
}

impl MeshTopology for Mesh {
    fn mesh_id(&self) -> EntityId {
        self.mesh_id
    }
    fn len(&self, dim: Dim) -> usize {
        self.ids[d(dim)].len()
    }
    fn id_of(&self, dim: Dim, ix: u32) -> Option<EntityId> {
        self.ids[d(dim)].get(ix as usize).copied()
    }
    fn index_of(&self, id: EntityId) -> Option<u32> {
        let dim = dim_of(id.kind())?;
        self.ids[d(dim)].binary_search(&id).ok().map(|i| i as u32)
    }
    fn cell_shape(&self, c: CellIx) -> CellShape {
        self.cell_shape[c.index()]
    }
    fn cell_faces(&self, c: CellIx) -> &[SignedFace] {
        self.cell_faces.row(c.index())
    }
    fn cell_corners(&self, c: CellIx) -> Option<&[VertexIx]> {
        (self.cell_shape[c.index()] != CellShape::Polyhedron).then(|| self.cell_corners.row(c.index()))
    }
    fn face_vertices(&self, f: FaceIx) -> &[VertexIx] {
        self.face_verts.row(f.index())
    }
    fn face_left(&self, f: FaceIx) -> CellIx {
        CellIx(self.face_left[f.index()])
    }
    fn face_right(&self, f: FaceIx) -> Option<CellIx> {
        let r = self.face_right[f.index()];
        (r != NONE).then_some(CellIx(r))
    }
    fn edge_vertices(&self, e: EdgeIx) -> [VertexIx; 2] {
        self.edge_verts[e.index()]
    }
}

impl MeshGeometry for Mesh {
    fn coords(&self) -> CoordsRef<'_> {
        CoordsRef { x: &self.x, y: &self.y, z: &self.z }
    }
    fn cell_spec(&self, c: CellIx) -> CellSpec {
        let i = c.index();
        CellSpec { shape: self.cell_shape[i], degree: self.cell_degree[i], basis: self.cell_basis[i] }
    }
    fn cell_nodes(&self, c: CellIx) -> &[VertexIx] {
        let n = self.cell_nodes.row(c.index());
        if n.is_empty() { self.cell_corners.row(c.index()) } else { n }
    }
    fn face_curve(&self, f: FaceIx) -> Option<FaceCurve<'_>> {
        self.curves.get(&f.0).map(|r| FaceCurve { degree: r.degree, basis: r.basis, nodes: &r.nodes })
    }
    fn geometry_rule(&self) -> &str {
        &self.schema.geometry_rule
    }
}

impl MeshClassification for Mesh {
    fn classification(&self, dim: Dim, ix: u32) -> Option<Classification> {
        self.class[d(dim)].get(ix as usize)
    }
    fn entities_on(&self, dim: Dim, model: EntityId) -> &[u32] {
        self.class[d(dim)].on(model)
    }
    fn classified_models(&self, dim: Dim) -> Vec<EntityId> {
        self.class[d(dim)].models()
    }
}

impl MeshTags for Mesh {
    fn schema(&self) -> &MeshSchema {
        &self.schema
    }
    fn tag(&self, dim: Dim, name: &str) -> Option<&TagColumn> {
        self.tags.iter().find(|t| t.def.dim == dim && t.def.name == name)
    }
    fn set(&self, name: &str) -> Option<&EntitySet> {
        self.sets.get(name)
    }
}

impl MeshEdit for Mesh {
    fn set_point(&mut self, v: VertexIx, p: Point3) -> MeshResult<()> {
        self.check_ix(Dim::Vertex, v.0)?;
        self.x[v.index()] = p.x;
        self.y[v.index()] = p.y;
        self.z[v.index()] = p.z;
        Ok(())
    }

    fn set_classification(&mut self, dim: Dim, ix: u32, c: Option<Classification>) -> MeshResult<()> {
        self.check_ix(dim, ix)?;
        if let Some(c) = c {
            check_classification(dim, c.entity)?;
        }
        self.class[d(dim)].set(ix as usize, c);
        Ok(())
    }

    fn define_tag(&mut self, def: TagDef) -> MeshResult<()> {
        if self.schema.tag(def.dim, &def.name).is_some() {
            return Err(MeshError::DuplicateName(def.name));
        }
        if def.width == 0 {
            return Err(MeshError::TagMismatch(def.name));
        }
        let col = TagColumn::new(def.clone(), self.ids[d(def.dim)].len());
        self.schema.tags.push(def);
        self.tags.push(col);
        Ok(())
    }

    fn set_tag(&mut self, dim: Dim, name: &str, ix: u32, value: TagRow<'_>) -> MeshResult<()> {
        let col = self
            .tags
            .iter_mut()
            .find(|t| t.def.dim == dim && t.def.name == name)
            .ok_or_else(|| MeshError::UnknownName(name.to_string()))?;
        col.set_row(ix as usize, value)
    }

    fn define_set(&mut self, name: &str, dim: Dim, members: Vec<u32>) -> MeshResult<()> {
        let len = self.ids[d(dim)].len();
        if let Some(&bad) = members.iter().find(|&&m| m as usize >= len) {
            return Err(MeshError::OutOfRange { dim, index: bad, len });
        }
        if self.sets.contains_key(name) {
            // replacing members is allowed only for the same dimension
            if self.sets[name].dim != dim {
                return Err(MeshError::TagMismatch(name.to_string()));
            }
        } else {
            self.schema.sets.push(crate::schema::SetDef { name: name.to_string(), dim });
        }
        self.sets.insert(name.to_string(), EntitySet::new(dim, members));
        Ok(())
    }

    fn apply_transform(&mut self, t: &Transform) -> MeshResult<()> {
        let improper = t.is_improper();
        if improper && (self.cell_degree.iter().any(|&g| g > 1) || !self.curves.is_empty()) {
            return Err(MeshError::Unsupported("improper transform of high-order or curved entities"));
        }
        for i in 0..self.x.len() {
            let p = t.apply_point(Point3::new(self.x[i], self.y[i], self.z[i]));
            (self.x[i], self.y[i], self.z[i]) = (p.x, p.y, p.z);
        }
        if !improper {
            return Ok(());
        }
        // Mirror: reverse every face loop; left/right and cell signs are unchanged because the
        // reflected normal of the old loop pointed right to left.
        self.face_verts = Csr::from_rows(self.face_verts.rows().map(reversed_loop));
        let keymap: BTreeMap<Vec<u32>, u32> =
            (0..self.face_verts.len()).map(|f| (face_key(self.face_verts.row(f)), f as u32)).collect();
        for c in 0..self.cell_shape.len() {
            let shape = self.cell_shape[c];
            let Some(sigma) = shape.mirror_permutation() else { continue };
            let old: Vec<VertexIx> = self.cell_corners.row(c).to_vec();
            for (i, &s) in sigma.iter().enumerate() {
                self.cell_corners.row_mut(c)[i] = old[s as usize];
            }
            // Re-derive the local face order from the permuted corners.
            let corners: Vec<VertexIx> = self.cell_corners.row(c).to_vec();
            for (k, lf) in shape.local_faces().iter().enumerate() {
                let key = face_key(&lf.iter().map(|&i| corners[i as usize]).collect::<Vec<_>>());
                let f = *keymap.get(&key).ok_or_else(|| MeshError::Invalid("template face missing".into()))?;
                let sign = if self.face_left[f as usize] == c as u32 { Sign::Plus } else { Sign::Minus };
                self.cell_faces.row_mut(c)[k] = SignedFace::new(FaceIx(f), sign);
            }
        }
        Ok(())
    }
}

/// Builder for [`Mesh`]; see [`MeshBuilder`].
#[derive(Clone, Debug)]
pub struct RefBuilder {
    mesh_id: EntityId,
    with_edges: bool,
    x: Vec<f64>,
    y: Vec<f64>,
    z: Vec<f64>,
    vclass: Vec<Option<Classification>>,
    shape: Vec<CellShape>,
    degree: Vec<u8>,
    basis: Vec<Basis>,
    corners: Csr<VertexIx>,
    nodes: Csr<VertexIx>,
    loops: Csr<VertexIx>,
    cell_loop_start: Vec<u32>,
    cclass: Vec<Option<EntityId>>,
    eclass: BTreeMap<[u32; 2], Classification>,
    fclass: BTreeMap<Vec<u32>, Classification>,
    tag_cols: Vec<TagColumn>,
}

impl RefBuilder {
    /// New builder. `mesh_id` must have kind `Mesh`. Edges are derived by default.
    pub fn new(mesh_id: EntityId) -> MeshResult<Self> {
        if mesh_id.kind() != EntityKind::Mesh {
            return Err(MeshError::WrongKind(mesh_id));
        }
        Ok(Self {
            mesh_id,
            with_edges: true,
            x: Vec::new(),
            y: Vec::new(),
            z: Vec::new(),
            vclass: Vec::new(),
            shape: Vec::new(),
            degree: Vec::new(),
            basis: Vec::new(),
            corners: Csr::new(),
            nodes: Csr::new(),
            loops: Csr::new(),
            cell_loop_start: vec![0],
            cclass: Vec::new(),
            eclass: BTreeMap::new(),
            fclass: BTreeMap::new(),
            tag_cols: Vec::new(),
        })
    }

    /// Choose whether `finish` derives the edge table (default true).
    pub fn with_edges(mut self, yes: bool) -> Self {
        self.with_edges = yes;
        self
    }

    fn check_v(&self, v: VertexIx) -> MeshResult<()> {
        if v.index() < self.x.len() {
            Ok(())
        } else {
            Err(MeshError::OutOfRange { dim: Dim::Vertex, index: v.0, len: self.x.len() })
        }
    }

    fn push_cell(
        &mut self,
        spec: CellSpec,
        corners: &[VertexIx],
        nodes: &[VertexIx],
        loops: Vec<Vec<VertexIx>>,
    ) -> MeshResult<CellIx> {
        for l in &loops {
            if !loop_is_simple(l) {
                return Err(MeshError::DegenerateLoop("fewer than 3 vertices or repeated vertex"));
            }
            for &v in l {
                self.check_v(v)?;
            }
        }
        if loops.len() < 4 {
            return Err(MeshError::DegenerateLoop("a cell needs at least 4 faces"));
        }
        let ix = CellIx(self.shape.len() as u32);
        for l in loops {
            self.loops.push_row(canonical_loop(&l));
        }
        self.cell_loop_start.push(self.loops.len() as u32);
        self.shape.push(spec.shape);
        self.degree.push(spec.degree);
        self.basis.push(spec.basis);
        self.corners.push_row(corners.iter().copied());
        self.nodes.push_row(nodes.iter().copied());
        self.cclass.push(None);
        Ok(ix)
    }

    fn template_loops(shape: CellShape, corners: &[VertexIx]) -> Vec<Vec<VertexIx>> {
        shape.local_faces().iter().map(|lf| lf.iter().map(|&i| corners[i as usize]).collect()).collect()
    }

    fn tag_col_mut(&mut self, dim: Dim, name: &str) -> MeshResult<&mut TagColumn> {
        let n = if dim == Dim::Vertex { self.x.len() } else { self.shape.len() };
        let col = self
            .tag_cols
            .iter_mut()
            .find(|t| t.def.dim == dim && t.def.name == name)
            .ok_or_else(|| MeshError::UnknownName(name.to_string()))?;
        col.resize(n);
        Ok(col)
    }
}

impl MeshBuilder for RefBuilder {
    type Mesh = Mesh;

    fn add_vertex(&mut self, p: Point3) -> VertexIx {
        self.x.push(p.x);
        self.y.push(p.y);
        self.z.push(p.z);
        self.vclass.push(None);
        VertexIx(self.x.len() as u32 - 1)
    }

    fn add_cell(&mut self, shape: CellShape, corners: &[VertexIx]) -> MeshResult<CellIx> {
        let expected = shape.num_corners().ok_or(MeshError::Unsupported("use add_polyhedron for polyhedra"))?;
        if corners.len() != expected {
            return Err(MeshError::BadArity { shape: shape.name(), expected, got: corners.len() });
        }
        for &v in corners {
            self.check_v(v)?;
        }
        let loops = Self::template_loops(shape, corners);
        self.push_cell(CellSpec::linear(shape), corners, &[], loops)
    }

    fn add_high_order_cell(&mut self, spec: CellSpec, nodes: &[VertexIx]) -> MeshResult<CellIx> {
        let ncorner = spec.shape.num_corners().ok_or(MeshError::Unsupported("high-order polyhedra"))?;
        let expected = spec.shape.num_nodes(spec.degree).ok_or(MeshError::Unsupported("degree 0"))?;
        if nodes.len() != expected {
            return Err(MeshError::BadArity { shape: spec.shape.name(), expected, got: nodes.len() });
        }
        for &v in nodes {
            self.check_v(v)?;
        }
        let corners = &nodes[..ncorner];
        let loops = Self::template_loops(spec.shape, corners);
        let stored: &[VertexIx] = if spec.degree > 1 { nodes } else { &[] };
        self.push_cell(spec, corners, stored, loops)
    }

    fn add_polyhedron(&mut self, faces: &[&[VertexIx]]) -> MeshResult<CellIx> {
        let loops = faces.iter().map(|l| l.to_vec()).collect();
        self.push_cell(CellSpec::linear(CellShape::Polyhedron), &[], &[], loops)
    }

    fn classify_vertex(&mut self, v: VertexIx, c: Classification) -> MeshResult<()> {
        self.check_v(v)?;
        check_classification(Dim::Vertex, c.entity)?;
        self.vclass[v.index()] = Some(c);
        Ok(())
    }

    fn classify_cell(&mut self, cell: CellIx, entity: EntityId) -> MeshResult<()> {
        check_classification(Dim::Cell, entity)?;
        let len = self.shape.len();
        *self.cclass.get_mut(cell.index()).ok_or(MeshError::OutOfRange { dim: Dim::Cell, index: cell.0, len })? =
            Some(entity);
        Ok(())
    }

    fn classify_edge(&mut self, a: VertexIx, b: VertexIx, c: Classification) -> MeshResult<()> {
        self.check_v(a)?;
        self.check_v(b)?;
        check_classification(Dim::Edge, c.entity)?;
        self.eclass.insert([a.0.min(b.0), a.0.max(b.0)], c);
        Ok(())
    }

    fn classify_face(&mut self, vertices: &[VertexIx], c: Classification) -> MeshResult<()> {
        for &v in vertices {
            self.check_v(v)?;
        }
        check_classification(Dim::Face, c.entity)?;
        self.fclass.insert(face_key(vertices), c);
        Ok(())
    }

    fn define_tag(&mut self, def: TagDef) -> MeshResult<()> {
        if !matches!(def.dim, Dim::Vertex | Dim::Cell) {
            return Err(MeshError::Unsupported("builder tags are vertex or cell only"));
        }
        if def.width == 0 || self.tag_cols.iter().any(|t| t.def.dim == def.dim && t.def.name == def.name) {
            return Err(MeshError::DuplicateName(def.name));
        }
        self.tag_cols.push(TagColumn::new(def, 0));
        Ok(())
    }

    fn set_tag(&mut self, dim: Dim, name: &str, ix: u32, value: TagRow<'_>) -> MeshResult<()> {
        self.tag_col_mut(dim, name)?.set_row(ix as usize, value)
    }

    fn finish(self) -> MeshResult<Mesh> {
        let (nv, nc) = (self.x.len(), self.shape.len());
        let mut face_verts: Csr<VertexIx> = Csr::new();
        let mut face_left: Vec<u32> = Vec::new();
        let mut face_right: Vec<u32> = Vec::new();
        let mut keymap: BTreeMap<Vec<u32>, u32> = BTreeMap::new();
        let mut cell_faces: Csr<SignedFace> = Csr::new();

        // Orientation rule: cells ascend, so the first cell to see a face is `left` and its
        // outward loop becomes the stored loop; the later cell is `right` and must see the
        // reverse loop.
        for c in 0..nc {
            let mut row: Vec<SignedFace> = Vec::new();
            for li in self.cell_loop_start[c]..self.cell_loop_start[c + 1] {
                let l = self.loops.row(li as usize);
                let key = face_key(l);
                match keymap.get(&key) {
                    None => {
                        let f = face_left.len() as u32;
                        if f >= (1 << 30) {
                            return Err(MeshError::Invalid("too many faces".into()));
                        }
                        face_verts.push_row(l.iter().copied());
                        face_left.push(c as u32);
                        face_right.push(NONE);
                        keymap.insert(key, f);
                        row.push(SignedFace::new(FaceIx(f), Sign::Plus));
                    }
                    Some(&f) => {
                        let fi = f as usize;
                        if face_right[fi] != NONE {
                            return Err(MeshError::NonManifoldFace(c as u32));
                        }
                        if face_verts.row(fi) != reversed_loop(l).as_slice() {
                            return Err(MeshError::InconsistentOrientation(face_left[fi], c as u32));
                        }
                        face_right[fi] = c as u32;
                        row.push(SignedFace::new(FaceIx(f), Sign::Minus));
                    }
                }
            }
            cell_faces.push_row(row);
        }

        let edge_verts: Vec<[VertexIx; 2]> = if self.with_edges {
            let mut set: BTreeSet<[u32; 2]> = BTreeSet::new();
            for l in face_verts.rows() {
                for i in 0..l.len() {
                    let (a, b) = (l[i].0, l[(i + 1) % l.len()].0);
                    set.insert([a.min(b), a.max(b)]);
                }
            }
            set.into_iter().map(|[a, b]| [VertexIx(a), VertexIx(b)]).collect()
        } else {
            Vec::new()
        };

        let counts = [nv, edge_verts.len(), face_left.len(), nc];
        let mut ids: [Vec<EntityId>; 4] = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];
        for dim in Dim::ALL {
            ids[d(dim)] = (0..counts[d(dim)] as u64)
                .map(|s| EntityId::new(kind_of(dim), s).ok_or_else(|| MeshError::Invalid("serial overflow".into())))
                .collect::<MeshResult<_>>()?;
        }

        let mut class = [ClassTable::new(nv), ClassTable::new(edge_verts.len()), ClassTable::new(face_left.len()), ClassTable::new(nc)];
        for (i, c) in self.vclass.iter().enumerate() {
            class[0].set(i, *c);
        }
        for (k, c) in &self.eclass {
            let pos = edge_verts
                .binary_search(&[VertexIx(k[0]), VertexIx(k[1])])
                .map_err(|_| MeshError::Invalid("classified edge does not exist".into()))?;
            class[1].set(pos, Some(*c));
        }
        for (k, c) in &self.fclass {
            let f = keymap.get(k).ok_or_else(|| MeshError::Invalid("classified face does not exist".into()))?;
            class[2].set(*f as usize, Some(*c));
        }
        for (i, c) in self.cclass.iter().enumerate() {
            class[3].set(i, c.map(Classification::on));
        }

        let mut schema = MeshSchema::default();
        let mut tags = self.tag_cols;
        for t in &mut tags {
            t.resize(if t.def.dim == Dim::Vertex { nv } else { nc });
            schema.tags.push(t.def.clone());
        }

        Ok(Mesh {
            mesh_id: self.mesh_id,
            schema,
            ids,
            x: self.x,
            y: self.y,
            z: self.z,
            edge_verts,
            face_verts,
            face_left,
            face_right,
            cell_shape: self.shape,
            cell_degree: self.degree,
            cell_basis: self.basis,
            cell_faces,
            cell_corners: self.corners,
            cell_nodes: self.nodes,
            curves: BTreeMap::new(),
            class,
            tags,
            sets: IndexMap::new(),
        })
    }
}
