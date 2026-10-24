#![allow(dead_code, missing_docs)]
use sm_mesh_api::CellShape;
use sm_quality::V;
pub const S3: f64 = 1.732_050_807_568_877_2;

pub fn tri() -> Vec<V> {
    vec![[0., 0., 0.], [1., 0., 0.], [0.5, S3 / 2., 0.]]
}
pub fn quad() -> Vec<V> {
    vec![[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.]]
}
pub fn tet() -> Vec<V> {
    vec![[0., 0., 0.], [1., 0., 0.], [0.5, S3 / 2., 0.], [0.5, S3 / 6., (2.0f64 / 3.0).sqrt()]]
}
pub fn hex() -> Vec<V> {
    vec![
        [0., 0., 0.],
        [1., 0., 0.],
        [1., 1., 0.],
        [0., 1., 0.],
        [0., 0., 1.],
        [1., 0., 1.],
        [1., 1., 1.],
        [0., 1., 1.],
    ]
}
pub fn wedge() -> Vec<V> {
    let h = S3 / 2.;
    vec![[0., 0., 0.], [1., 0., 0.], [0.5, h, 0.], [0., 0., 1.], [1., 0., 1.], [0.5, h, 1.]]
}
pub fn pyramid() -> Vec<V> {
    vec![[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0.5, 0.5, 0.5f64.sqrt()]]
}
pub fn polygon(n: usize) -> Vec<V> {
    (0..n).map(|k| { let t = std::f64::consts::TAU * k as f64 / n as f64; [t.cos(), t.sin(), 0.0] }).collect()
}

pub fn ideal_elements() -> Vec<(CellShape, Vec<V>)> {
    vec![
        (CellShape::Tri, tri()),
        (CellShape::Quad, quad()),
        (CellShape::Polygon, polygon(6)),
        (CellShape::Tet, tet()),
        (CellShape::Hex, hex()),
        (CellShape::Wedge, wedge()),
        (CellShape::Pyramid, pyramid()),
    ]
}


use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::{
    CellIx, Classification, ClassSet, Coords, EdgeTable, FaceCurve, FaceIx, FaceKind, HighOrderCell, MeshBuild, MeshDim,
    MeshEntityKind, MeshEntityRef, MeshRead, MeshSet, SerialAlloc, TagColumn, TopologyColumns, VertexIx,
};

/// A mesh wrapper that reports chosen cells as inactive (refined parents).
pub struct WithInactive<'a>(pub &'a ReferenceMesh, pub Vec<u32>);

impl MeshRead for WithInactive<'_> {
    fn mesh_id(&self) -> EntityId { self.0.mesh_id() }
    fn dim(&self) -> MeshDim { self.0.dim() }
    fn count(&self, k: MeshEntityKind) -> usize { self.0.count(k) }
    fn id(&self, k: MeshEntityKind, ix: u32) -> EntityId { self.0.id(k, ix) }
    fn index_of(&self, id: EntityId) -> Option<u32> { self.0.index_of(id) }
    fn coords(&self) -> Coords<'_> { self.0.coords() }
    fn columns(&self) -> TopologyColumns<'_> { self.0.columns() }
    fn face_vertices(&self, f: FaceIx) -> &[VertexIx] { self.0.face_vertices(f) }
    fn face_left(&self, f: FaceIx) -> CellIx { self.0.face_left(f) }
    fn face_right(&self, f: FaceIx) -> Option<CellIx> { self.0.face_right(f) }
    fn face_kind(&self, f: FaceIx) -> FaceKind { self.0.face_kind(f) }
    fn cell_faces(&self, c: CellIx) -> &[sm_mesh_api::SignedFace] { self.0.cell_faces(c) }
    fn cell_shape(&self, c: CellIx) -> CellShape { self.0.cell_shape(c) }
    fn cell_corners(&self, c: CellIx) -> Option<&[VertexIx]> { self.0.cell_corners(c) }
    fn high_order_cell(&self, c: CellIx) -> Option<&HighOrderCell> { self.0.high_order_cell(c) }
    fn face_curve(&self, f: FaceIx) -> Option<FaceCurve<'_>> { self.0.face_curve(f) }
    fn cell_parent(&self, c: CellIx) -> Option<CellIx> { self.0.cell_parent(c) }
    fn cell_active(&self, c: CellIx) -> bool { !self.1.contains(&c.0) }
    fn classification(&self, k: MeshEntityKind, ix: u32) -> Classification { self.0.classification(k, ix) }
    fn classified_on(&self, model: EntityId) -> Option<&ClassSet> { self.0.classified_on(model) }
    fn classified_entities(&self) -> &[EntityId] { self.0.classified_entities() }
    fn edges(&self) -> Option<&EdgeTable> { self.0.edges() }
    fn sets(&self) -> &[MeshSet] { self.0.sets() }
    fn tags(&self) -> &[TagColumn] { self.0.tags() }
}

fn on(kind: EntityKind, n: u64) -> Classification {
    Classification::on(EntityId::new(kind, n).expect("serial"))
}

/// A planar surface mesh (z = 0) of the given polygons (point indices, counter-clockwise), all
/// on `TopoFace#1`, vertices on their own `TopoVertex`, boundary facets on `TopoEdge`s.
pub fn surface_mesh(points: &[[f64; 2]], polys: &[Vec<usize>]) -> ReferenceMesh {
    let mut b = ReferenceBuilder::surface();
    let vs: Vec<VertexIx> = points
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let v = b.add_vertex(Point3::new(p[0], p[1], 0.0)).unwrap();
            b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoVertex, 1 + i as u64)).unwrap();
            v
        })
        .collect();
    for poly in polys {
        let l: Vec<VertexIx> = poly.iter().map(|&i| vs[i]).collect();
        let added = b.add_polygon(&l).unwrap();
        b.set_class(MeshEntityRef::Cell(added.cell), on(EntityKind::TopoFace, 1)).unwrap();
    }
    let mut k = 1;
    for f in 0..b.face_count() {
        let f = FaceIx::new(f);
        if b.face_degree(f) == 1 {
            b.set_class(MeshEntityRef::Face(f), on(EntityKind::TopoEdge, k)).unwrap();
            k += 1;
        }
    }
    b.finish(&mut SerialAlloc::new()).unwrap().mesh
}

use sm_mesh_api::{MeshEdit, PointIx};

/// A regular `n^3` hex block whose interior vertices are displaced by a seeded LCG, so every
/// metric column is non-trivial (deterministic: the seed is fixed).
pub fn perturbed_block(n: u32, amp: f64) -> ReferenceMesh {
    let mut m = sm_mesh_api::reference::hex_block([n, n, n], [1.0, 1.0, 1.0], &mut SerialAlloc::new()).unwrap();
    let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
    let mut next = || {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((s >> 11) as f64 / (1u64 << 53) as f64) - 0.5
    };
    for i in 0..m.vertex_count() {
        let p = m.point(i);
        let on_boundary = [p.x, p.y, p.z].iter().any(|c| *c < 1e-9 || (*c - n as f64).abs() < 1e-9);
        if !on_boundary {
            m.set_point(PointIx::new(i), Point3::new(p.x + amp * next(), p.y + amp * next(), p.z + amp * next())).unwrap();
        }
    }
    m
}
