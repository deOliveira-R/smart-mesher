//! Shared helpers: resolvers, hashing, invariants.
#![allow(dead_code)]

use sm_core::{EntityId, EntityKind};
use sm_mesh::MeshBuilder;
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::{
    CellIx, Classification, FaceIx, MeshEntityKind, MeshRead, SerialAlloc, cell_volume,
    face_area_vector,
};
use sm_poly::{DualReport, EdgeResolver, PolyError, PolyOptions, dualize};

/// Mints a `TopoEdge` id from the face pair (for fixtures whose edges are single intervals).
pub struct PairResolver;

impl EdgeResolver for PairResolver {
    fn resolve(&self, a: EntityId, b: EntityId, _ends: [Classification; 2]) -> Option<EntityId> {
        EntityId::new(EntityKind::TopoEdge, a.serial() * 1000 + b.serial())
    }
}

pub fn dual_ref(
    src: &dyn MeshRead,
    opts: &PolyOptions,
    r: &dyn EdgeResolver,
) -> Result<(ReferenceMesh, DualReport), PolyError> {
    let mut alloc = SerialAlloc::new();
    let (f, rep) = dualize(src, opts, r, ReferenceBuilder::new(), &mut alloc)?;
    Ok((f.mesh, rep))
}

pub fn dual_prod(
    src: &dyn MeshRead,
    opts: &PolyOptions,
    r: &dyn EdgeResolver,
) -> Result<(sm_mesh::Mesh, DualReport), PolyError> {
    let mut alloc = SerialAlloc::new();
    let (f, rep) = dualize(src, opts, r, MeshBuilder::new(), &mut alloc)?;
    Ok((f.mesh, rep))
}

pub fn volumes(m: &dyn MeshRead) -> Vec<f64> {
    (0..m.count(MeshEntityKind::Cell)).map(|c| cell_volume(m, CellIx::new(c))).collect()
}

pub fn boundary_area_on(m: &dyn MeshRead, model: EntityId) -> f64 {
    m.classified_on(model)
        .map(|s| s.faces.iter().filter(|f| m.is_boundary_face(**f)).map(|f| face_area_vector(m, *f).norm()).sum())
        .unwrap_or(0.0)
}

pub fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-300)
}

pub fn id(kind: EntityKind, serial: u64) -> EntityId {
    EntityId::new(kind, serial).expect("id")
}

pub fn n_faces(m: &dyn MeshRead) -> usize {
    m.count(MeshEntityKind::Face)
}

pub fn face_sizes(m: &dyn MeshRead) -> Vec<usize> {
    (0..n_faces(m)).map(|f| m.face_vertices(FaceIx::new(f)).len()).collect()
}
