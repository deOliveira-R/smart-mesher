//! Deliberately broken inputs for negative tests of the checkers.
//!
//! `MeshBuild::finish` canonicalises orientation and rejects invalid incidence, so a finished
//! mesh cannot be invalid. [`Tampered`] therefore copies a good mesh's columns into owned
//! vectors that tests may edit freely, and implements [`MeshRead`] over them.

use crate::topo::{BuildError, ModelFixture, finish_boundaries};
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_mesh_api::reference::ReferenceMesh;
use sm_mesh_api::{
    CellIx, CellShape, ClassSet, Classification, Coords, EdgeTable, FaceCurve, FaceIx, FaceKind, HighOrderCell, MeshEntityKind, MeshRead, MeshSet,
    SignedFace, TagColumn, TopologyColumns, VertexIx,
};
use sm_model_api::{BoundaryKind, Deck, Domain, Model, Plane};
use sm_topo_api::fixtures::{self, FixtureError, with_op};
use sm_topo_api::{CellComplex, OpId, TopologyEdit};
use std::collections::BTreeMap;

use crate::topo::TopoFixture;

/// An editable copy of a [`ReferenceMesh`]: topology and coordinate columns are owned and public;
/// everything else (ids, shapes, sets, edges, ...) is read from the original.
///
/// The stale reverse classification index is *not* rebuilt, as in a real corrupted file.
/// `face_kind` is derived from the owned columns (a face with a right cell is `Interior`; a
/// one-cell face keeps the original's one-cell kind, `Boundary` if the original was interior),
/// so `is_boundary_face` and `face_right().is_none()` agree after `set_face_right` or a swap.
/// The dimension is the original's.
#[derive(Clone, Debug)]
pub struct Tampered<'a> {
    inner: &'a ReferenceMesh,
    /// x coordinates.
    pub x: Vec<f64>,
    /// y coordinates.
    pub y: Vec<f64>,
    /// z coordinates.
    pub z: Vec<f64>,
    /// CSR offsets of the face loops.
    pub face_off: Vec<u64>,
    /// Concatenated face loops.
    pub face_verts: Vec<VertexIx>,
    /// Left cell of every face.
    pub face_left: Vec<CellIx>,
    /// Right cell of every face (`CellIx::NONE` for none).
    pub face_right: Vec<u32>,
    /// CSR offsets of the cell face lists.
    pub cell_off: Vec<u64>,
    /// Concatenated signed faces of every cell.
    pub cell_faces: Vec<SignedFace>,
    /// Replacement classifications by `(kind, index)`.
    pub class_override: BTreeMap<(MeshEntityKind, u32), Classification>,
}

impl<'a> Tampered<'a> {
    /// Copy `m`'s columns unchanged.
    pub fn of(m: &'a ReferenceMesh) -> Self {
        let c = m.coords();
        let t = m.columns();
        Self {
            inner: m,
            x: c.x.to_vec(),
            y: c.y.to_vec(),
            z: c.z.to_vec(),
            face_off: t.face_vertex_offsets.to_vec(),
            face_verts: t.face_vertices.to_vec(),
            face_left: t.face_left.to_vec(),
            face_right: t.face_right.to_vec(),
            cell_off: t.cell_face_offsets.to_vec(),
            cell_faces: t.cell_faces.to_vec(),
            class_override: BTreeMap::new(),
        }
    }

    /// Swap `left` and `right` of a two-cell face (breaks `left < right` and the normal rule).
    pub fn swap_left_right(mut self, f: usize) -> Self {
        let r = self.face_right[f];
        if r != CellIx::NONE {
            self.face_right[f] = self.face_left[f].0;
            self.face_left[f] = CellIx(r);
        }
        self
    }

    /// Reverse the loop of face `f` (flips its normal and breaks closedness of its cells).
    pub fn reverse_loop(mut self, f: usize) -> Self {
        let (a, b) = (self.face_off[f] as usize, self.face_off[f + 1] as usize);
        self.face_verts[a..b].reverse();
        self
    }

    /// Move vertex `v` to `p` (can invert cells).
    pub fn move_vertex(mut self, v: usize, p: Point3) -> Self {
        (self.x[v], self.y[v], self.z[v]) = (p.x, p.y, p.z);
        self
    }

    /// Remove the `k`-th face from the face list of cell `c` (the cell is no longer closed).
    pub fn remove_cell_face(mut self, c: usize, k: usize) -> Self {
        let at = self.cell_off[c] as usize + k;
        self.cell_faces.remove(at);
        for o in &mut self.cell_off[c + 1..] {
            *o -= 1;
        }
        self
    }

    /// Give a face a right cell, so it is no longer a boundary face (opens a hole in the
    /// boundary surface).
    pub fn set_face_right(mut self, f: usize, r: usize) -> Self {
        self.face_right[f] = r as u32;
        self
    }

    /// Classify an entity on a non-model entity (an invalid classification).
    pub fn declassify(mut self, k: MeshEntityKind, ix: u32) -> Self {
        let bogus = EntityId::new(EntityKind::MeshVertex, 0).map(Classification::on);
        if let Some(c) = bogus {
            self.class_override.insert((k, ix), c);
        }
        self
    }
}

impl MeshRead for Tampered<'_> {
    fn mesh_id(&self) -> EntityId {
        self.inner.mesh_id()
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
        TopologyColumns::new(&self.face_off, &self.face_verts, &self.face_left, &self.face_right, &self.cell_off, &self.cell_faces)
    }
    fn face_vertices(&self, f: FaceIx) -> &[VertexIx] {
        &self.face_verts[self.face_off[f.get()] as usize..self.face_off[f.get() + 1] as usize]
    }
    fn face_left(&self, f: FaceIx) -> CellIx {
        self.face_left[f.get()]
    }
    fn face_right(&self, f: FaceIx) -> Option<CellIx> {
        let r = self.face_right[f.get()];
        (r != CellIx::NONE).then_some(CellIx(r))
    }
    fn face_kind(&self, f: FaceIx) -> FaceKind {
        match (self.face_right(f), self.inner.face_kind(f)) {
            (Some(_), _) => FaceKind::Interior,
            (None, FaceKind::Interior) => FaceKind::Boundary,
            (None, k) => k,
        }
    }
    fn dim(&self) -> sm_mesh_api::MeshDim {
        self.inner.dim()
    }
    fn cell_faces(&self, c: CellIx) -> &[SignedFace] {
        &self.cell_faces[self.cell_off[c.get()] as usize..self.cell_off[c.get() + 1] as usize]
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
        self.class_override.get(&(k, ix)).copied().unwrap_or_else(|| self.inner.classification(k, ix))
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
}

/// A unit brick with a non-manifold fin (three faces on one edge): valid, but fails
/// [`crate::check::manifold_edges`]. Counts `[10, 15, 7, 1, 1]`: the fin adds 2
/// vertices, 3 edges and 1 face to the brick.
pub fn brick_with_fin() -> Result<TopoFixture, FixtureError> {
    let mut t = CellComplex::new();
    with_op(&mut t, OpId(1), fixtures::brick_with_fin)?;
    Ok(TopoFixture { name: "brick_with_fin", complex: t, counts: [10, 15, 7, 1, 1], euler: 2, volumes: vec![1.0], area: 6.0 + 2.0_f64.sqrt() })
}

/// A unit brick with an internal baffle sheet: valid, but its baffle edges have a single face
/// use, so it fails [`crate::check::manifold_edges`].
pub fn brick_with_baffle() -> Result<TopoFixture, FixtureError> {
    let mut t = CellComplex::new();
    with_op(&mut t, OpId(1), fixtures::brick_with_baffle)?;
    Ok(TopoFixture { name: "brick_with_baffle", complex: t, counts: [12, 16, 7, 1, 1], euler: 3, volumes: vec![1.0], area: 6.25 })
}

/// Two bricks that do not touch: two components, so `V - E + F = 4`, not 2.
pub fn two_disjoint_bricks() -> Result<TopoFixture, FixtureError> {
    let mut t = CellComplex::new();
    with_op(&mut t, OpId(1), |t| {
        fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3])?;
        fixtures::brick(t, Point3::new(5.0, 0.0, 0.0), [1.0; 3])?;
        Ok(())
    })?;
    Ok(TopoFixture { name: "two_disjoint_bricks", complex: t, counts: [16, 24, 12, 2, 2], euler: 4, volumes: vec![1.0, 1.0], area: 12.0 })
}

/// A unit brick whose vertex `0` was moved off its three edges: the combinatorics stay valid, the
/// carriers contradict the topology, so `sm_topo_api::validate_geometry` reports it.
pub fn brick_with_moved_vertex() -> Result<TopoFixture, FixtureError> {
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3]))?;
    with_op(&mut t, OpId(2), |t| Ok(t.set_vertex_point(b.vertices[0], Point3::new(-0.3, -0.2, -0.1))?))?;
    Ok(TopoFixture { name: "brick_with_moved_vertex", complex: t, counts: [8, 12, 6, 1, 1], euler: 2, volumes: vec![1.0], area: 6.0 })
}

/// A model that is invalid: a mirror deck at `z = 1/2` has the `-x` face of the unit brick as its
/// self-paired face, but that face does not lie on the mirror plane.
pub fn model_with_bad_mirror() -> Result<ModelFixture, BuildError> {
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3]))?;
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume)?;
    m.color_volumes(mat, &[b.volume])?;
    let deck = m.add_boundary("mirror", BoundaryKind::Deck(Deck::Mirror { plane: Plane::new(Vector3::new(0.0, 0.0, 1.0), 0.5)? }))?;
    m.assign_self_paired(deck, &[b.faces[0]])?;
    finish_boundaries(&mut m, &t, 7)?;
    Ok(ModelFixture { name: "model_with_bad_mirror", complex: t, model: m, flat_paths: 0, flat_entries: 0, deck_faces: 1, volume: 1.0 })
}
