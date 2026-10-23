//! Optional derived edge table. Edges are never primary data in the face-first model, but when
//! present they are fully classified and part of both content hashes.

use crate::build::SerialAlloc;
use crate::classify::Classification;
use crate::error::{MeshError, Result};
use crate::index::{CellIx, EdgeIx, FaceIx, FaceKind, MeshEntity, VertexIx};
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
    /// Classification per edge (always present once the table exists).
    pub class: Vec<Classification>,
}

/// Sort a vertex pair into table order.
pub fn edge_key(a: VertexIx, b: VertexIx) -> [VertexIx; 2] {
    if a < b { [a, b] } else { [b, a] }
}

impl EdgeTable {
    /// Derive the table from any mesh; ids come from `alloc`.
    ///
    /// `explicit` gives classifications by (sorted) vertex pair. An edge without one is
    /// classified on the volume or partition cell shared by all cells around it, but only if
    /// none of its faces is a boundary, non-conformal or partition-cut face; otherwise
    /// the derivation fails with [`MeshError::Unclassified`]. Pairs in `explicit` that are not
    /// edges of the mesh are an error.
    pub fn derive(
        m: &dyn MeshRead,
        alloc: &mut SerialAlloc,
        explicit: &BTreeMap<[VertexIx; 2], Classification>,
    ) -> Result<Self> {
        let nf = m.count(crate::index::MeshEntityKind::Face);
        let mut map: BTreeMap<[VertexIx; 2], u32> = BTreeMap::new();
        for f in 0..nf {
            let l = m.face_vertices(FaceIx::new(f));
            for i in 0..l.len() {
                map.entry(edge_key(l[i], l[(i + 1) % l.len()])).or_insert(0);
            }
        }
        if let Some(k) = explicit.keys().find(|k| !map.contains_key(*k)) {
            return Err(MeshError::State(format!(
                "edge class given for {k:?}, which is not an edge of the mesh"
            )));
        }
        let first = alloc.take(EntityKind::MeshEdge, map.len())?;
        let mut t = EdgeTable::default();
        for (i, (k, v)) in map.iter_mut().enumerate() {
            *v = i as u32;
            t.vertices.push(*k);
            t.ids.push(
                EntityId::new(EntityKind::MeshEdge, first + i as u64)
                    .ok_or(MeshError::SerialOverflow)?,
            );
        }
        t.face_off.push(0);
        // Cells around each edge, found through the faces (sorted pairs, deduplicated).
        let mut around: Vec<Vec<CellIx>> = vec![Vec::new(); t.vertices.len()];
        let mut all_interior = vec![true; t.vertices.len()];
        for f in 0..nf {
            let fi = FaceIx::new(f);
            let l = m.face_vertices(fi);
            let (left, right) = (m.face_left(fi), m.face_right(fi));
            let interior = m.face_kind(fi) == FaceKind::Interior;
            for i in 0..l.len() {
                let (a, b) = (l[i], l[(i + 1) % l.len()]);
                let e = map[&edge_key(a, b)] as usize;
                t.face_edges.push(crate::edges::FaceEdge {
                    edge: EdgeIx(e as u32),
                    forward: a < b,
                });
                all_interior[e] &= interior;
                around[e].push(left);
                around[e].extend(right);
            }
            t.face_off.push(t.face_edges.len() as u32);
        }
        for (e, cells) in around.iter_mut().enumerate() {
            cells.sort_unstable();
            cells.dedup();
            let c = if let Some(c) = explicit.get(&t.vertices[e]) {
                *c
            } else {
                let mut it = cells
                    .iter()
                    .map(|&c| m.classification(crate::index::MeshEntityKind::Cell, c.0));
                match it.next() {
                    Some(c0) if all_interior[e] && it.all(|c| c.entity == c0.entity) => {
                        Classification::on(c0.entity)
                    }
                    _ => return Err(MeshError::Unclassified(MeshEntity::Edge(EdgeIx(e as u32)))),
                }
            };
            t.class.push(c);
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

    /// Index of the edge joining two vertices (`O(log n)`).
    pub fn find(&self, a: VertexIx, b: VertexIx) -> Option<EdgeIx> {
        let k = edge_key(a, b);
        self.vertices.binary_search(&k).ok().map(EdgeIx::new)
    }
}
