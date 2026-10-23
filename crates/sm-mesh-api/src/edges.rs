//! Optional derived edge table. Edges are never primary data in the face-first model.

use crate::build::SerialAlloc;
use crate::classify::Classification;
use crate::error::Result;
use crate::index::{EdgeIx, FaceIx, VertexIx};
use crate::read::MeshRead;
use sm_core::{EntityId, EntityKind};
use std::collections::BTreeMap;

/// One face-edge incidence: the edge and whether the face loop traverses it low->high vertex.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaceEdge {
    /// The edge.
    pub edge: EdgeIx,
    /// True if the loop runs from the lower to the higher vertex index along this edge.
    pub forward: bool,
}

/// Derived edges: unique vertex pairs sorted lexicographically, plus face->edge CSR.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct EdgeTable {
    /// Stable ids (`EntityKind::MeshEdge`), ascending.
    pub ids: Vec<EntityId>,
    /// Endpoints with `a < b`, sorted.
    pub vertices: Vec<[VertexIx; 2]>,
    /// CSR offsets into `face_edges` (length `face_count + 1`).
    pub face_off: Vec<u32>,
    /// Edges of each face, in loop order (edge `i` joins loop vertices `i` and `i+1`).
    pub face_edges: Vec<FaceEdge>,
    /// Classification per edge; `None` until a mesher/modifier sets it.
    pub class: Vec<Option<Classification>>,
}

impl EdgeTable {
    /// Derive the table from any mesh; ids come from `alloc`.
    pub fn derive<M: MeshRead + ?Sized>(m: &M, alloc: &mut SerialAlloc) -> Result<Self> {
        let mut map: BTreeMap<[VertexIx; 2], u32> = BTreeMap::new();
        for f in 0..m.face_count() {
            let l = m.face_vertices(FaceIx::new(f));
            for i in 0..l.len() {
                let (a, b) = (l[i], l[(i + 1) % l.len()]);
                map.entry(if a < b { [a, b] } else { [b, a] }).or_insert(0);
            }
        }
        let mut t = EdgeTable::default();
        let first = alloc.take(EntityKind::MeshEdge, map.len())?;
        for (i, (k, v)) in map.iter_mut().enumerate() {
            *v = i as u32;
            t.vertices.push(*k);
            t.ids.push(EntityId::new(EntityKind::MeshEdge, first + i as u64).ok_or(crate::MeshError::SerialOverflow)?);
        }
        t.class = vec![None; t.vertices.len()];
        t.face_off.push(0);
        for f in 0..m.face_count() {
            let l = m.face_vertices(FaceIx::new(f));
            for i in 0..l.len() {
                let (a, b) = (l[i], l[(i + 1) % l.len()]);
                let key = if a < b { [a, b] } else { [b, a] };
                t.face_edges.push(FaceEdge { edge: EdgeIx(map[&key]), forward: a < b });
            }
            t.face_off.push(t.face_edges.len() as u32);
        }
        Ok(t)
    }

    /// Number of edges.
    pub fn len(&self) -> usize {
        self.vertices.len()
    }

    /// True if there are no edges.
    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty()
    }

    /// Edges of face `f`.
    pub fn of_face(&self, f: FaceIx) -> &[FaceEdge] {
        &self.face_edges[self.face_off[f.get()] as usize..self.face_off[f.get() + 1] as usize]
    }
}
