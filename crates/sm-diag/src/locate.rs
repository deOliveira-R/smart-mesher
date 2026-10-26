//! Where a finding is: the entity, its own classification (owner), the lower-dimensional model
//! entities around it (near) and a point.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_mesh_api::{
    CellIx, FaceIx, MeshEntity, MeshEntityKind, MeshRead, VertexIx, cell_volume_centroid, face_centroid,
};

/// Cap on the number of `near` model entities.
pub const NEAR_CAP: usize = 8;

/// A mesh entity on the wire: its stable id and dense index.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct EntityRef {
    /// The stable id (`MeshCell#12`); `null` when the index is out of range.
    pub id: Option<EntityId>,
    /// The dense index.
    pub index: u32,
}

/// The `(kind, index)` of a mesh entity.
pub(crate) fn parts(e: MeshEntity) -> (MeshEntityKind, u32) {
    match e {
        MeshEntity::Vertex(v) => (MeshEntityKind::Vertex, v.0),
        MeshEntity::Edge(x) => (MeshEntityKind::Edge, x.0),
        MeshEntity::Face(f) => (MeshEntityKind::Face, f.0),
        MeshEntity::Cell(c) => (MeshEntityKind::Cell, c.0),
        _ => (MeshEntityKind::Cell, u32::MAX),
    }
}

/// Everything a located finding carries besides its text.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Located {
    pub entity: EntityRef,
    pub owner: Option<EntityId>,
    pub near: Vec<EntityId>,
    pub location: Option<[f64; 3]>,
}

fn in_range(m: &dyn MeshRead, k: MeshEntityKind, ix: u32) -> bool {
    (ix as usize) < m.count(k)
}

/// Entity reference of `(k, ix)`.
pub(crate) fn entity_ref(m: &dyn MeshRead, k: MeshEntityKind, ix: u32) -> EntityRef {
    EntityRef { id: in_range(m, k, ix).then(|| m.id(k, ix)), index: ix }
}

fn point(m: &dyn MeshRead, v: VertexIx) -> Option<[f64; 3]> {
    let co = m.coords();
    let i = v.get();
    (i < co.len()).then(|| [co.x[i], co.y[i], co.z[i]])
}

/// Vertices of an entity (all in range), for `near` and the location of a cell.
fn vertices_of(m: &dyn MeshRead, k: MeshEntityKind, ix: u32) -> Vec<VertexIx> {
    let nv = m.count(MeshEntityKind::Vertex);
    let mut out: Vec<VertexIx> = match k {
        MeshEntityKind::Vertex => vec![VertexIx(ix)],
        MeshEntityKind::Edge => m.edges().and_then(|t| t.vertices.get(ix as usize)).map(|p| p.to_vec()).unwrap_or_default(),
        MeshEntityKind::Face => m.face_vertices(FaceIx(ix)).to_vec(),
        MeshEntityKind::Cell => {
            let c = CellIx(ix);
            match m.cell_corners(c) {
                Some(corners) => corners.to_vec(),
                None => m
                    .cell_faces(c)
                    .iter()
                    .filter(|sf| sf.face().get() < m.count(MeshEntityKind::Face))
                    .flat_map(|sf| m.face_vertices(sf.face()).iter().copied())
                    .collect(),
            }
        }
        _ => Vec::new(),
    };
    out.retain(|v| v.get() < nv);
    out.sort_unstable();
    out.dedup();
    out
}

/// Representative point of an entity: the vertex, the edge midpoint, the face centroid or the
/// cell centroid. A cell centroid that is not finite or leaves the bounding box of the cell's
/// vertices (an inverted or collapsed cell) is replaced by the vertex mean. `None` if the index
/// is out of range or the entity references missing vertices.
pub(crate) fn location(m: &dyn MeshRead, k: MeshEntityKind, ix: u32) -> Option<[f64; 3]> {
    if !in_range(m, k, ix) {
        return None;
    }
    let vs = vertices_of(m, k, ix);
    let pts: Vec<[f64; 3]> = vs.iter().filter_map(|&v| point(m, v)).collect();
    if pts.is_empty() {
        return None;
    }
    let mean = {
        let n = pts.len() as f64;
        let s = pts.iter().fold([0.0; 3], |a, p| [a[0] + p[0], a[1] + p[1], a[2] + p[2]]);
        [s[0] / n, s[1] / n, s[2] / n]
    };
    match k {
        MeshEntityKind::Vertex => Some(pts[0]),
        MeshEntityKind::Edge => (pts.len() == 2).then(|| [(pts[0][0] + pts[1][0]) / 2.0, (pts[0][1] + pts[1][1]) / 2.0, (pts[0][2] + pts[1][2]) / 2.0]),
        MeshEntityKind::Face => {
            let c = face_centroid(m, FaceIx(ix));
            Some([c.x, c.y, c.z])
        }
        MeshEntityKind::Cell => {
            let c = cell_volume_centroid(m, CellIx(ix)).1;
            let c = [c.x, c.y, c.z];
            let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
            for p in &pts {
                for a in 0..3 {
                    lo[a] = lo[a].min(p[a]);
                    hi[a] = hi[a].max(p[a]);
                }
            }
            let inside = (0..3).all(|a| c[a].is_finite() && c[a] >= lo[a] && c[a] <= hi[a]);
            Some(if inside { c } else { mean })
        }
        _ => None,
    }
}

/// The classification entity of `(k, ix)`, or `None` if the index is out of range.
pub(crate) fn owner(m: &dyn MeshRead, k: MeshEntityKind, ix: u32) -> Option<EntityId> {
    if !in_range(m, k, ix) {
        return None;
    }
    // An edge table is optional: `classification` on a mesh without one has no edge to read.
    if k == MeshEntityKind::Edge && m.edges().is_none() {
        return None;
    }
    Some(m.classification(k, ix).entity)
}

/// Model entities of dimension below the owner's, from the vertex classifications of the
/// entity, sorted, deduplicated, at most [`NEAR_CAP`].
pub(crate) fn near(m: &dyn MeshRead, k: MeshEntityKind, ix: u32, owner: Option<EntityId>) -> Vec<EntityId> {
    let Some(owner_dim) = owner.and_then(|o| sm_mesh_api::entity_dim(o.kind())) else { return Vec::new() };
    let mut out: Vec<EntityId> = vertices_of(m, k, ix)
        .into_iter()
        .map(|v| m.classification(MeshEntityKind::Vertex, v.0).entity)
        .filter(|e| sm_mesh_api::entity_dim(e.kind()).is_some_and(|d| d < owner_dim))
        .collect();
    out.sort_unstable();
    out.dedup();
    out.truncate(NEAR_CAP);
    out
}

/// Full location data of `(k, ix)`.
pub(crate) fn locate(m: &dyn MeshRead, k: MeshEntityKind, ix: u32) -> Located {
    let owner = owner(m, k, ix);
    Located { entity: entity_ref(m, k, ix), owner, near: near(m, k, ix, owner), location: location(m, k, ix) }
}
