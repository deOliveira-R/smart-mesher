//! Optional derived edge table. Edges are never primary data in the face-first model, but when
//! present they are fully classified and part of both content hashes.

use crate::build::SerialAlloc;
use crate::classify::Classification;
use crate::error::{MeshError, Result};
use crate::index::{CellIx, EdgeIx, FaceIx, FaceKind, VertexIx};
use crate::read::MeshRead;
use sm_core::{EntityId, EntityKind};
use std::collections::BTreeMap;

/// One face-edge incidence: the edge and whether the face loop traverses it low->high vertex.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct FaceEdge {
    /// The edge.
    pub edge: EdgeIx,
    /// True if the loop runs from the lower to the higher vertex index along this edge.
    pub forward: bool,
}

/// Derived edges: unique vertex pairs sorted lexicographically, plus face->edge CSR.
#[derive(Clone, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct EdgeTable {
    /// Stable ids (`EntityKind::MeshEdge`), ascending.
    pub ids: Vec<EntityId>,
    /// Endpoints with `a < b`, sorted.
    pub vertices: Vec<[VertexIx; 2]>,
    /// CSR offsets into `face_edges` (length `face_count + 1`).
    pub face_off: Vec<u64>,
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
    /// `explicit` gives classifications by (sorted) vertex pair. An edge without one takes the
    /// classification of its incident faces: among their model entities it takes those of the
    /// lowest dimension, and uses that entity if there is exactly one. So a surface edge inside
    /// a model face lies on that face even though it also touches interior faces (volume), and
    /// an edge strictly inside a volume lies on the volume. An edge on a model edge is
    /// ambiguous (several model faces) and needs an explicit class. Failing that, if no incident face is a boundary, non-conformal or
    /// partition-cut face, it takes the volume or partition cell shared by all cells around it. An edge
    /// that is still unclassified fails with [`MeshError::UnclassifiedEdge`] naming its vertices.
    /// Pairs in `explicit` that are not edges of the mesh are an error.
    ///
    /// `alloc` is unchanged on `Err`: serials are drawn only after every check has passed.
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
        if map.len() > u32::MAX as usize {
            return Err(MeshError::TooLarge);
        }
        let mut t = EdgeTable::default();
        for (i, (k, v)) in map.iter_mut().enumerate() {
            *v = i as u32;
            t.vertices.push(*k);
        }
        t.face_off.push(0);
        // Cells and face classes around each edge, found through the faces.
        let ne = t.vertices.len();
        let mut around: Vec<Vec<CellIx>> = vec![Vec::new(); ne];
        let mut all_interior = vec![true; ne];
        // Lowest-dimension model entity among the incident faces, and that entity if unique.
        let mut face_low: Vec<Option<(u8, Option<EntityId>)>> = vec![None; ne];
        for f in 0..nf {
            let fi = FaceIx::new(f);
            let l = m.face_vertices(fi);
            let (left, right) = (m.face_left(fi), m.face_right(fi));
            let interior = m.face_kind(fi) == FaceKind::Interior;
            let fcl = m.classification(crate::index::MeshEntityKind::Face, f as u32);
            let (fc, fd) = (fcl.entity, fcl.dim().unwrap_or(u8::MAX));
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
                face_low[e] = match face_low[e] {
                    None => Some((fd, Some(fc))),
                    Some((d, _)) if fd < d => Some((fd, Some(fc))),
                    Some((d, Some(x))) if fd == d && x != fc => Some((d, None)),
                    keep => keep,
                };
            }
            t.face_off.push(t.face_edges.len() as u64);
        }
        for (e, cells) in around.iter_mut().enumerate() {
            cells.sort_unstable();
            cells.dedup();
            let c = if let Some(c) = explicit.get(&t.vertices[e]) {
                *c
            } else if let Some((_, Some(ent))) = face_low[e] {
                Classification::on(ent)
            } else {
                let mut it = cells
                    .iter()
                    .map(|&c| m.classification(crate::index::MeshEntityKind::Cell, c.0));
                match it.next() {
                    Some(c0) if all_interior[e] && it.all(|c| c.entity == c0.entity) => {
                        Classification::on(c0.entity)
                    }
                    _ => {
                        let [a, b] = t.vertices[e];
                        return Err(MeshError::UnclassifiedEdge(a, b));
                    }
                }
            };
            t.class.push(c);
        }
        // Everything that can fail has passed: draw the ids.
        // `take` checks the range before it writes, so it is atomic.
        let first = alloc.take(EntityKind::MeshEdge, ne)?;
        for i in 0..ne as u64 {
            t.ids.push(
                EntityId::new(EntityKind::MeshEdge, first + i).ok_or(MeshError::SerialOverflow)?,
            );
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
