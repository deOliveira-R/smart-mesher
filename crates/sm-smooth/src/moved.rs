//! A read-only mesh view with replaced vertex coordinates, so `sm_quality` can score the result.

use sm_core::EntityId;
use sm_mesh_api::{
    CellIx, CellShape, ClassSet, Classification, Coords, EdgeTable, FaceCurve, FaceIx, FaceKind, GeometryCache,
    HighOrderCell, MeshDim, MeshEntityKind, MeshRead, MeshSet, SignedFace, TagColumn, TopologyColumns, VertexIx,
};

/// `inner` with the coordinates `x`, `y`, `z` (one per vertex; the smoother rejects high-order
/// meshes, so there are no extra nodes).
pub(crate) struct Moved<'a> {
    pub inner: &'a dyn MeshRead,
    pub x: Vec<f64>,
    pub y: Vec<f64>,
    pub z: Vec<f64>,
}

impl<'a> Moved<'a> {
    pub fn new(inner: &'a dyn MeshRead, pts: &[[f64; 3]]) -> Self {
        Self {
            inner,
            x: pts.iter().map(|p| p[0]).collect(),
            y: pts.iter().map(|p| p[1]).collect(),
            z: pts.iter().map(|p| p[2]).collect(),
        }
    }
}

impl MeshRead for Moved<'_> {
    fn mesh_id(&self) -> EntityId {
        self.inner.mesh_id()
    }
    fn dim(&self) -> MeshDim {
        self.inner.dim()
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
        Coords::new(&self.x, &self.y, &self.z)
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
        self.inner.high_order_cell(c)
    }
    fn face_curve(&self, f: FaceIx) -> Option<FaceCurve<'_>> {
        self.inner.face_curve(f)
    }
    fn cell_parent(&self, c: CellIx) -> Option<CellIx> {
        self.inner.cell_parent(c)
    }
    fn cell_active(&self, c: CellIx) -> bool {
        self.inner.cell_active(c)
    }
    fn classification(&self, k: MeshEntityKind, ix: u32) -> Classification {
        self.inner.classification(k, ix)
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
        self.inner.sets()
    }
    fn tags(&self) -> &[TagColumn] {
        self.inner.tags()
    }
    fn geometry_rule(&self) -> &str {
        self.inner.geometry_rule()
    }
    fn geometry_cache(&self) -> Option<&GeometryCache> {
        None
    }
}
