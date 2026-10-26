//! Shared helpers for the integration tests.
#![allow(dead_code)]

use sm_mesh_api::reference::ReferenceMesh;
use sm_mesh_api::{CellIx, FaceIx, MeshEdit, MeshRead, SerialAlloc, SetMembers, Side, SignedFace, VertexIx};

/// Attach a labelled set to a mesh (fresh serial space per call is fine: ids only need to be unique
/// within the mesh, and the sets here are the only mesh sets).
pub fn add_set(m: &mut ReferenceMesh, alloc: &mut SerialAlloc, name: &str, label: Option<i64>, members: SetMembers) {
    m.add_set(name, members, alloc).expect("add_set");
    if label.is_some() {
        m.set_label(name, label).expect("set_label");
    }
}

pub fn cells(ix: &[u32]) -> SetMembers {
    SetMembers::Cells(ix.iter().map(|c| CellIx(*c)).collect())
}

pub fn verts(ix: &[u32]) -> SetMembers {
    SetMembers::Vertices(ix.iter().map(|v| VertexIx(*v)).collect())
}

/// The face whose vertex set equals `want` (0-based vertex indices).
pub fn face_with_vertices(m: &dyn MeshRead, want: &[u32]) -> FaceIx {
    let mut w = want.to_vec();
    w.sort_unstable();
    (0..m.face_count())
        .map(FaceIx::new)
        .find(|f| {
            let mut v: Vec<u32> = m.face_vertices(*f).iter().map(|x| x.0).collect();
            v.sort_unstable();
            v == w
        })
        .expect("face")
}

pub fn left(f: FaceIx) -> SignedFace {
    SignedFace::new(f, Side::Left)
}

pub fn right(f: FaceIx) -> SignedFace {
    SignedFace::new(f, Side::Right)
}

pub fn sides(v: &[SignedFace]) -> SetMembers {
    let mut v = v.to_vec();
    v.sort_by_key(|s| s.to_raw());
    SetMembers::Sides(v)
}

/// FNV-1a 64 of `bytes` (testkit hasher): a stable content hash for regression constants.
pub fn fnv1a(bytes: &[u8]) -> u64 {
    use std::hash::Hasher;
    let mut h = sm_testkit::determinism::Fnv::default();
    h.write(bytes);
    h.finish()
}

/// Exodus II manual node lists (1-based, outward loops) of side `s` (1-based) of an element type,
/// typed from the manual's figures, NOT derived from the crate's tables.
pub fn manual_side_nodes(elem_type: &str, side: usize) -> Vec<usize> {
    let t: &[&[usize]] = match elem_type {
        "HEX8" => &[&[1, 2, 6, 5], &[2, 3, 7, 6], &[3, 4, 8, 7], &[1, 5, 8, 4], &[1, 4, 3, 2], &[5, 6, 7, 8]],
        "TET4" => &[&[1, 2, 4], &[2, 3, 4], &[1, 4, 3], &[1, 3, 2]],
        "WEDGE6" => &[&[1, 2, 5, 4], &[2, 3, 6, 5], &[1, 4, 6, 3], &[1, 3, 2], &[4, 5, 6]],
        "PYRAMID5" => &[&[1, 2, 5], &[2, 3, 5], &[3, 4, 5], &[1, 5, 4], &[1, 4, 3, 2]],
        "QUAD4" => &[&[1, 2], &[2, 3], &[3, 4], &[4, 1]],
        "TRI3" => &[&[1, 2], &[2, 3], &[3, 1]],
        other => panic!("no manual table for {other}"),
    };
    t[side - 1].to_vec()
}

/// Newell area vector of a loop of points.
pub fn newell(p: &[[f64; 3]]) -> [f64; 3] {
    let mut n = [0.0; 3];
    for i in 0..p.len() {
        let (a, b) = (p[i], p[(i + 1) % p.len()]);
        n[0] += (a[1] - b[1]) * (a[2] + b[2]);
        n[1] += (a[2] - b[2]) * (a[0] + b[0]);
        n[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    n
}

pub fn mean(p: &[[f64; 3]]) -> [f64; 3] {
    let n = p.len() as f64;
    let mut c = [0.0; 3];
    for q in p {
        for k in 0..3 {
            c[k] += q[k] / n;
        }
    }
    c
}

pub fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// A set with every `(face, side)` of every cell in `cells` (the mesh's own sides, sorted).
pub fn all_sides_of(m: &dyn MeshRead, cells: impl Iterator<Item = usize>) -> Vec<SignedFace> {
    let mut v: Vec<SignedFace> = cells.flat_map(|c| m.cell_faces(CellIx::new(c)).to_vec()).collect();
    v.sort_by_key(|s| s.to_raw());
    v
}

/// Node coordinates of the file (0-based node `i`).
pub fn node_xyz(d: &sm_exodus::read::ExodusData, i: usize) -> [f64; 3] {
    [d.coords[0][i], d.coords.get(1).map_or(0.0, |c| c[i]), d.coords.get(2).map_or(0.0, |c| c[i])]
}

use sm_mesh_api::{
    CellShape, Classification, ClassSet, Coords, EdgeTable, FaceCurve, FaceKind, HighOrderCell, MeshDim, MeshEntityKind, MeshSet, TagColumn,
    TopologyColumns, Basis,
};

/// A forwarding `MeshRead` that can mark cells inactive and report a high-order record.
pub struct Over<'a> {
    pub inner: &'a ReferenceMesh,
    pub inactive: Vec<bool>,
    pub high_order: Option<HighOrderCell>,
}

impl<'a> Over<'a> {
    pub fn new(inner: &'a ReferenceMesh) -> Self {
        Self { inner, inactive: vec![false; inner.cell_count()], high_order: None }
    }
    pub fn with_high_order(mut self) -> Self {
        self.high_order = Some(HighOrderCell::new(2, Basis::LagrangeEquispaced, Vec::new()));
        self
    }
}

impl MeshRead for Over<'_> {
    fn mesh_id(&self) -> sm_core::EntityId {
        self.inner.mesh_id()
    }
    fn dim(&self) -> MeshDim {
        self.inner.dim()
    }
    fn count(&self, k: MeshEntityKind) -> usize {
        self.inner.count(k)
    }
    fn id(&self, k: MeshEntityKind, ix: u32) -> sm_core::EntityId {
        self.inner.id(k, ix)
    }
    fn index_of(&self, id: sm_core::EntityId) -> Option<u32> {
        self.inner.index_of(id)
    }
    fn coords(&self) -> Coords<'_> {
        self.inner.coords()
    }
    fn columns(&self) -> TopologyColumns<'_> {
        self.inner.columns()
    }
    fn face_vertices(&self, f: FaceIx) -> &[VertexIx] {
        self.inner.face_vertices(f)
    }
    fn face_left(&self, f: FaceIx) -> CellIx {
        self.inner.face_left(f)
    }
    fn face_right(&self, f: FaceIx) -> Option<CellIx> {
        self.inner.face_right(f)
    }
    fn face_kind(&self, f: FaceIx) -> FaceKind {
        self.inner.face_kind(f)
    }
    fn cell_faces(&self, c: CellIx) -> &[SignedFace] {
        self.inner.cell_faces(c)
    }
    fn cell_shape(&self, c: CellIx) -> CellShape {
        self.inner.cell_shape(c)
    }
    fn cell_corners(&self, c: CellIx) -> Option<&[VertexIx]> {
        self.inner.cell_corners(c)
    }
    fn high_order_cell(&self, c: CellIx) -> Option<&HighOrderCell> {
        if c.get() == 0 { self.high_order.as_ref() } else { None }
    }
    fn face_curve(&self, f: FaceIx) -> Option<FaceCurve<'_>> {
        self.inner.face_curve(f)
    }
    fn cell_parent(&self, c: CellIx) -> Option<CellIx> {
        self.inner.cell_parent(c)
    }
    fn cell_active(&self, c: CellIx) -> bool {
        !self.inactive[c.get()]
    }
    fn classification(&self, k: MeshEntityKind, ix: u32) -> Classification {
        self.inner.classification(k, ix)
    }
    fn classified_on(&self, model: sm_core::EntityId) -> Option<&ClassSet> {
        self.inner.classified_on(model)
    }
    fn classified_entities(&self) -> &[sm_core::EntityId] {
        self.inner.classified_entities()
    }
    fn edges(&self) -> Option<&EdgeTable> {
        self.inner.edges()
    }
    fn sets(&self) -> &[MeshSet] {
        self.inner.sets()
    }
    fn tags(&self) -> &[TagColumn] {
        self.inner.tags()
    }
}
