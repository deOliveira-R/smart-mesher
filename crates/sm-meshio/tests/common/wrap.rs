//! A `MeshRead` wrapper that lets tests present a mesh with permuted faces, other face kinds,
//! replaced sets, inactive cells or swapped left/right cells, without touching the real mesh.
//! Cell->face tables are passed through unchanged (the exporters under test do not read them).
#![allow(dead_code)]

use sm_core::EntityId;
use sm_mesh_api::reference::ReferenceMesh;
use sm_mesh_api::*;

pub struct Wrap<'a> {
    pub inner: &'a ReferenceMesh,
    /// `new face i = old face perm[i]`.
    perm: Vec<usize>,
    kinds: Vec<FaceKind>,
    sets: Option<Vec<MeshSet>>,
    active: Option<Vec<bool>>,
    dim: MeshDim,
    xyz: Option<[Vec<f64>; 3]>,
    swap: Vec<usize>,
    // derived
    offsets: Vec<u64>,
    verts: Vec<VertexIx>,
    left: Vec<CellIx>,
    right: Vec<u32>,
}

impl<'a> Wrap<'a> {
    pub fn new(inner: &'a ReferenceMesh) -> Self {
        let nf = inner.face_count();
        let kinds = (0..nf).map(|f| inner.face_kind(FaceIx::new(f))).collect();
        let mut w = Self {
            inner,
            perm: (0..nf).collect(),
            kinds,
            sets: None,
            active: None,
            dim: inner.dim(),
            xyz: None,
            swap: vec![],
            offsets: vec![],
            verts: vec![],
            left: vec![],
            right: vec![],
        };
        w.rebuild();
        w
    }

    fn rebuild(&mut self) {
        self.offsets = vec![0];
        self.verts.clear();
        self.left.clear();
        self.right.clear();
        for (i, &f) in self.perm.iter().enumerate() {
            let fi = FaceIx::new(f);
            self.verts.extend_from_slice(self.inner.face_vertices(fi));
            self.offsets.push(self.verts.len() as u64);
            let (l, r) = (self.inner.face_left(fi), self.inner.face_right(fi).map_or(NO_CELL, |c| c.0));
            if self.swap.contains(&i) {
                self.left.push(CellIx(r));
                self.right.push(l.0);
            } else {
                self.left.push(l);
                self.right.push(r);
            }
        }
    }

    /// New face order: `new face i = old face perm[i]` (kinds follow the faces).
    pub fn permute(mut self, perm: Vec<usize>) -> Self {
        assert_eq!(perm.len(), self.perm.len());
        self.kinds = perm.iter().map(|&f| self.inner.face_kind(FaceIx::new(f))).collect();
        self.perm = perm;
        self.rebuild();
        self
    }
    /// Override the kind of new face `i`.
    pub fn kind(mut self, i: usize, k: FaceKind) -> Self {
        self.kinds[i] = k;
        self
    }
    pub fn sets(mut self, s: Vec<MeshSet>) -> Self {
        self.sets = Some(s);
        self
    }
    pub fn inactive(mut self, cells: &[usize]) -> Self {
        let mut a = vec![true; self.inner.cell_count()];
        for c in cells {
            a[*c] = false;
        }
        self.active = Some(a);
        self
    }
    pub fn dim2(mut self) -> Self {
        self.dim = MeshDim::Two;
        self
    }
    /// Replace the x coordinate of point `i`.
    pub fn with_x(mut self, i: usize, v: f64) -> Self {
        let co = self.inner.coords();
        let mut xyz = self.xyz.take().unwrap_or_else(|| [co.x.to_vec(), co.y.to_vec(), co.z.to_vec()]);
        xyz[0][i] = v;
        self.xyz = Some(xyz);
        self
    }
    /// Swap left and right of new face `i` (breaks the `left < right` rule).
    pub fn swap_cells(mut self, i: usize) -> Self {
        self.swap.push(i);
        self.rebuild();
        self
    }
    /// Old index of new face `i`.
    pub fn old(&self, i: usize) -> usize {
        self.perm[i]
    }
}

impl MeshRead for Wrap<'_> {
    fn mesh_id(&self) -> EntityId {
        self.inner.mesh_id()
    }
    fn dim(&self) -> MeshDim {
        self.dim
    }
    fn count(&self, k: MeshEntityKind) -> usize {
        self.inner.count(k)
    }
    fn id(&self, k: MeshEntityKind, ix: u32) -> EntityId {
        self.inner.id(k, ix)
    }
    fn index_of(&self, id: EntityId) -> Option<u32> {
        self.inner.index_of(id)
    }
    fn coords(&self) -> Coords<'_> {
        match &self.xyz {
            Some([x, y, z]) => Coords::new(x, y, z),
            None => self.inner.coords(),
        }
    }
    fn columns(&self) -> TopologyColumns<'_> {
        let c = self.inner.columns();
        TopologyColumns::new(&self.offsets, &self.verts, &self.left, &self.right, c.cell_face_offsets, c.cell_faces)
    }
    fn face_vertices(&self, f: FaceIx) -> &[VertexIx] {
        &self.verts[self.offsets[f.get()] as usize..self.offsets[f.get() + 1] as usize]
    }
    fn face_left(&self, f: FaceIx) -> CellIx {
        self.left[f.get()]
    }
    fn face_right(&self, f: FaceIx) -> Option<CellIx> {
        let r = self.right[f.get()];
        (r != NO_CELL).then_some(CellIx(r))
    }
    fn face_kind(&self, f: FaceIx) -> FaceKind {
        self.kinds[f.get()]
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
        self.inner.high_order_cell(c)
    }
    fn face_curve(&self, f: FaceIx) -> Option<FaceCurve<'_>> {
        self.inner.face_curve(FaceIx::new(self.perm[f.get()]))
    }
    fn cell_parent(&self, c: CellIx) -> Option<CellIx> {
        self.inner.cell_parent(c)
    }
    fn cell_active(&self, c: CellIx) -> bool {
        self.active.as_ref().map_or_else(|| self.inner.cell_active(c), |a| a[c.get()])
    }
    fn classification(&self, k: MeshEntityKind, ix: u32) -> Classification {
        match k {
            MeshEntityKind::Face => self.inner.classification(k, self.perm[ix as usize] as u32),
            _ => self.inner.classification(k, ix),
        }
    }
    fn classified_on(&self, model: EntityId) -> Option<&ClassSet> {
        self.inner.classified_on(model)
    }
    fn classified_entities(&self) -> &[EntityId] {
        self.inner.classified_entities()
    }
    fn edges(&self) -> Option<&EdgeTable> {
        self.inner.edges()
    }
    fn sets(&self) -> &[MeshSet] {
        self.sets.as_deref().unwrap_or_else(|| self.inner.sets())
    }
    fn tags(&self) -> &[TagColumn] {
        self.inner.tags()
    }
}
