//! Adjacency tables (CSR) built on demand, in parallel, with deterministic row order.
//!
//! Every table has ascending, duplicate-free rows, so its content depends only on the mesh, never
//! on thread scheduling. Tables are cached in the mesh ([`Mesh::vertex_cells`] and friends) and
//! are not part of its identity: they are derived data.

use crate::store::Mesh;
use crate::util::{CHUNK, chunk_bounds, prefix, range};
use rayon::prelude::*;
use sm_mesh_api::{CellIx, FaceIx, MeshRead};
use smallvec::SmallVec;
use std::sync::OnceLock;

/// A compressed-sparse-row table of `u32` indices: `row(i)` is the (ascending) list of entity
/// indices related to entity `i`.
#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub struct Csr {
    off: Vec<u64>,
    idx: Vec<u32>,
}

impl Csr {
    /// Number of rows.
    pub fn len(&self) -> usize {
        self.off.len().saturating_sub(1)
    }
    /// True if there are no rows.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
    /// Row `i`. Panics if `i >= len()`.
    pub fn row(&self, i: usize) -> &[u32] {
        &self.idx[range(&self.off, i)]
    }
    /// The offsets column (`len() + 1` entries).
    pub fn offsets(&self) -> &[u64] {
        &self.off
    }
    /// The concatenated rows.
    pub fn indices(&self) -> &[u32] {
        &self.idx
    }

    /// Build from `(row, value)` pairs packed as `row << 32 | value`; sorts and removes
    /// duplicates, so rows come out ascending.
    pub(crate) fn from_pairs(n_rows: usize, mut pairs: Vec<u64>) -> Self {
        pairs.par_sort_unstable();
        pairs.dedup();
        let off: Vec<u64> = (0..=n_rows)
            .into_par_iter()
            .map(|r| pairs.partition_point(|p| (p >> 32) < r as u64) as u64)
            .collect();
        let idx = pairs.par_iter().map(|p| *p as u32).collect();
        Csr { off, idx }
    }

    /// Build row by row: `f(i, &mut row)` appends the values of row `i` (they are sorted and
    /// deduplicated afterwards).
    pub(crate) fn from_rows(n_rows: usize, f: impl Fn(usize, &mut Vec<u32>) + Sync) -> Self {
        let bounds = chunk_bounds(n_rows, CHUNK);
        let parts: Vec<(Vec<u32>, Vec<u32>)> = bounds
            .par_windows(2)
            .map(|w| {
                let mut lens = Vec::with_capacity(w[1] - w[0]);
                let mut data = Vec::new();
                let mut row = Vec::new();
                for i in w[0]..w[1] {
                    row.clear();
                    f(i, &mut row);
                    row.sort_unstable();
                    row.dedup();
                    lens.push(row.len() as u32);
                    data.extend_from_slice(&row);
                }
                (lens, data)
            })
            .collect();
        let lens: Vec<u32> = parts.iter().flat_map(|p| p.0.iter().copied()).collect();
        let mut idx = Vec::with_capacity(parts.iter().map(|p| p.1.len()).sum());
        for p in &parts {
            idx.extend_from_slice(&p.1);
        }
        Csr { off: prefix(&lens), idx }
    }
}

/// Lazily built adjacency tables; always compares equal (derived data).
#[derive(Debug, Default)]
pub(crate) struct Caches {
    pub(crate) vertex_cells: OnceLock<Csr>,
    pub(crate) vertex_faces: OnceLock<Csr>,
    pub(crate) cell_neighbors: OnceLock<Csr>,
    pub(crate) edge_faces: OnceLock<Csr>,
    pub(crate) vertex_edges: OnceLock<Csr>,
}

impl Clone for Caches {
    fn clone(&self) -> Self {
        // Derived data: a clone rebuilds on demand.
        Caches::default()
    }
}

impl PartialEq for Caches {
    fn eq(&self, _: &Self) -> bool {
        true
    }
}

impl Mesh {
    /// Cells around each vertex (row `v` = ascending cell indices). Built on first use.
    pub fn vertex_cells(&self) -> &Csr {
        self.adj.vertex_cells.get_or_init(|| {
            let nc = self.n_cells();
            let bounds = chunk_bounds(nc, CHUNK);
            let parts: Vec<Vec<u64>> = bounds
                .par_windows(2)
                .map(|w| {
                    let mut out = Vec::new();
                    let mut vs: SmallVec<[u32; 32]> = SmallVec::new();
                    for c in w[0]..w[1] {
                        vs.clear();
                        for sf in self.cell_faces(CellIx::new(c)) {
                            vs.extend(self.face_vertices(sf.face()).iter().map(|v| v.0));
                        }
                        vs.sort_unstable();
                        vs.dedup();
                        out.extend(vs.iter().map(|v| (u64::from(*v) << 32) | c as u64));
                    }
                    out
                })
                .collect();
            let mut pairs = Vec::with_capacity(parts.iter().map(Vec::len).sum());
            for p in parts {
                pairs.extend(p);
            }
            Csr::from_pairs(self.n_vertices(), pairs)
        })
    }

    /// Faces around each vertex (row `v` = ascending face indices). Built on first use.
    pub fn vertex_faces(&self) -> &Csr {
        self.adj.vertex_faces.get_or_init(|| {
            let nf = self.n_faces();
            let pairs: Vec<u64> = (0..nf)
                .into_par_iter()
                .flat_map_iter(|f| {
                    self.face_vertices(FaceIx::new(f))
                        .iter()
                        .map(move |v| (u64::from(v.0) << 32) | f as u64)
                })
                .collect();
            Csr::from_pairs(self.n_vertices(), pairs)
        })
    }

    /// Cells sharing a face with each cell (row `c` = ascending, distinct cell indices; the cells
    /// across `Interior` faces). Built on first use.
    pub fn cell_neighbors(&self) -> &Csr {
        self.adj.cell_neighbors.get_or_init(|| {
            Csr::from_rows(self.n_cells(), |c, row| {
                for sf in self.cell_faces(CellIx::new(c)) {
                    if let Some(n) = self.across(*sf) {
                        row.push(n.0);
                    }
                }
            })
        })
    }

    /// Faces around each edge (row `e` = ascending face indices); `None` without an edge table.
    pub fn edge_faces(&self) -> Option<&Csr> {
        let t = self.edges.as_ref()?;
        Some(self.adj.edge_faces.get_or_init(|| {
            let pairs: Vec<u64> = (0..self.n_faces())
                .into_par_iter()
                .flat_map_iter(|f| {
                    t.of_face(FaceIx::new(f))
                        .iter()
                        .map(move |fe| (u64::from(fe.edge.0) << 32) | f as u64)
                })
                .collect();
            Csr::from_pairs(t.len(), pairs)
        }))
    }

    /// Edges around each vertex (row `v` = ascending edge indices); `None` without an edge table.
    pub fn vertex_edges(&self) -> Option<&Csr> {
        let t = self.edges.as_ref()?;
        Some(self.adj.vertex_edges.get_or_init(|| {
            let pairs: Vec<u64> = (0..t.len())
                .into_par_iter()
                .flat_map_iter(|e| {
                    let [a, b] = t.vertices[e];
                    [(u64::from(a.0) << 32) | e as u64, (u64::from(b.0) << 32) | e as u64]
                })
                .collect();
            Csr::from_pairs(self.n_vertices(), pairs)
        }))
    }

    /// Drop the cached tables that depend on the edge table.
    pub(crate) fn reset_edge_caches(&mut self) {
        self.adj.edge_faces = Default::default();
        self.adj.vertex_edges = Default::default();
    }
}
