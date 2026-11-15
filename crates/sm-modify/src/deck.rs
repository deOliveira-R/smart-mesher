//! Deck pairs seen from a mesh: vertex roots for the diagonal rule of `mesh.htet`, and the
//! re-verification of the pairs on a converted mesh.
//!
//! A translation or rotation deck pairs model faces A and B with `B = T(A)`. A mesh vertex of an A
//! face and the mesh vertex of a B face at `T(p)` (to a relative `1e-9`) are partners; partners
//! share one root (the lowest index of their class), so the rank `(root, index)` orders the two
//! faces of a pair alike. Mirror decks are self-paired (no partner) and add nothing.

use std::collections::{BTreeMap, BTreeSet};

use sm_core::{EntityId, Point3};
use sm_layer_api::ModelLayer;
use sm_mesh_api::{FaceIx, MeshRead};
use sm_model_api::{BoundaryKind, DeckSide, FaceRole};

/// The deck pairs of a mesh.
pub(crate) struct Decks {
    /// Root of every vertex (itself when it has no partner).
    pub root: Vec<u32>,
    /// One entry per deck pair record: the A-side model faces, the B-side model faces and the
    /// vertex map `A vertex -> B vertex`.
    pub pairs: Vec<DeckPairs>,
}

pub(crate) struct DeckPairs {
    pub name: String,
    pub a: Vec<EntityId>,
    pub b: Vec<EntityId>,
    pub a_to_b: BTreeMap<u32, u32>,
}

fn faces_of(model: &ModelLayer, m: &dyn MeshRead, face: EntityId) -> Vec<FaceIx> {
    let topo = model.topo();
    let own = m.classified_on(face);
    let cover = if own.is_none() && !topo.is_alive(face) { topo.live_cover(face) } else { Vec::new() };
    let mut out: Vec<FaceIx> = own.into_iter().chain(cover.iter().filter_map(|x| m.classified_on(*x))).flat_map(|s| s.faces.iter().copied()).collect();
    out.sort_unstable();
    out.dedup();
    out
}

fn verts_of(m: &dyn MeshRead, faces: &[FaceIx]) -> Vec<u32> {
    let mut v: Vec<u32> = faces.iter().flat_map(|&f| m.face_vertices(f).iter().map(|x| x.0)).collect();
    v.sort_unstable();
    v.dedup();
    v
}

/// Find the vertex of `cands` nearest to `p` within `tol`, by a grid of cell `tol`.
struct Grid {
    h: f64,
    cells: BTreeMap<[i64; 3], Vec<u32>>,
}

impl Grid {
    fn new(m: &dyn MeshRead, verts: &[u32], h: f64) -> Self {
        let mut cells: BTreeMap<[i64; 3], Vec<u32>> = BTreeMap::new();
        for &v in verts {
            cells.entry(Self::key(h, m.point(v as usize))).or_default().push(v);
        }
        Self { h, cells }
    }
    fn key(h: f64, p: Point3) -> [i64; 3] {
        [(p.x / h).floor() as i64, (p.y / h).floor() as i64, (p.z / h).floor() as i64]
    }
    fn find(&self, m: &dyn MeshRead, p: Point3) -> Option<u32> {
        let k = Self::key(self.h, p);
        let mut best: Option<(f64, u32)> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    for &v in self.cells.get(&[k[0] + dx, k[1] + dy, k[2] + dz]).into_iter().flatten() {
                        let q = m.point(v as usize);
                        let d = (q - p).norm();
                        if d <= self.h && best.is_none_or(|(bd, bv)| d < bd || (d == bd && v < bv)) {
                            best = Some((d, v));
                        }
                    }
                }
            }
        }
        best.map(|(_, v)| v)
    }
}

/// No decks: every vertex is its own root.
pub(crate) fn none(m: &dyn MeshRead) -> Decks {
    Decks { root: (0..m.vertex_count() as u32).collect(), pairs: Vec::new() }
}

/// Deck roots and pairs of `m` under the decks of `model`.
pub(crate) fn decks(model: &ModelLayer, m: &dyn MeshRead) -> Decks {
    let nv = m.vertex_count();
    let mut parent: Vec<u32> = (0..nv as u32).collect();
    fn find(p: &mut [u32], mut x: u32) -> u32 {
        while p[x as usize] != x {
            p[x as usize] = p[p[x as usize] as usize];
            x = p[x as usize];
        }
        x
    }
    // Scale of the mesh for the matching tolerance.
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for v in 0..nv {
        let p = m.point(v);
        for (d, x) in [p.x, p.y, p.z].into_iter().enumerate() {
            lo[d] = lo[d].min(x);
            hi[d] = hi[d].max(x);
        }
    }
    let diag = (0..3).map(|d| (hi[d] - lo[d]).powi(2)).sum::<f64>().sqrt().max(1e-300);
    let tol = 1e-9 * diag;
    let mut pairs = Vec::new();
    for row in model.model().boundaries() {
        let BoundaryKind::Deck(deck) = row.kind else { continue };
        let (mut a, mut b): (Vec<EntityId>, Vec<EntityId>) = (Vec::new(), Vec::new());
        for (face, fb) in model.model().boundary_faces(row.id) {
            match fb.role {
                FaceRole::Deck { side: DeckSide::A, .. } => a.push(face),
                FaceRole::Deck { side: DeckSide::B, .. } => b.push(face),
                _ => {}
            }
        }
        if a.is_empty() || b.is_empty() {
            continue;
        }
        let a_faces: Vec<FaceIx> = a.iter().flat_map(|&f| faces_of(model, m, f)).collect();
        let b_faces: Vec<FaceIx> = b.iter().flat_map(|&f| faces_of(model, m, f)).collect();
        let (av, bv) = (verts_of(m, &a_faces), verts_of(m, &b_faces));
        let grid = Grid::new(m, &bv, tol.max(1e-12));
        let t = deck.transform();
        let mut a_to_b = BTreeMap::new();
        for &v in &av {
            if let Some(w) = grid.find(m, t.apply_point(m.point(v as usize))) {
                a_to_b.insert(v, w);
                let (ra, rb) = (find(&mut parent, v), find(&mut parent, w));
                if ra != rb {
                    let (lo, hi) = (ra.min(rb), ra.max(rb));
                    parent[hi as usize] = lo;
                }
            }
        }
        pairs.push(DeckPairs { name: row.name.clone(), a, b, a_to_b });
    }
    let root = (0..nv as u32).map(|v| find(&mut parent, v)).collect();
    Decks { root, pairs }
}

/// Count the faces of `m` (a converted mesh with the same vertex indices) on the A side of each
/// deck whose image under the vertex map is not a face of the B side, and the reverse. Returns
/// `(deck name, unmatched)` for every deck with a mismatch.
pub(crate) fn verify(model: &ModelLayer, m: &dyn MeshRead, d: &Decks) -> Vec<(String, usize)> {
    let mut out = Vec::new();
    for p in &d.pairs {
        let key = |f: FaceIx| -> Vec<u32> {
            let mut l: Vec<u32> = m.face_vertices(f).iter().map(|v| v.0).collect();
            l.sort_unstable();
            l
        };
        let a: BTreeSet<Vec<u32>> = p
            .a
            .iter()
            .flat_map(|&f| faces_of(model, m, f))
            .map(|f| {
                let mut l: Vec<u32> = key(f).iter().map(|v| p.a_to_b.get(v).copied().unwrap_or(u32::MAX)).collect();
                l.sort_unstable();
                l
            })
            .collect();
        let b: BTreeSet<Vec<u32>> = p.b.iter().flat_map(|&f| faces_of(model, m, f)).map(key).collect();
        let n = a.symmetric_difference(&b).count();
        if n > 0 {
            out.push((p.name.clone(), n));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grid_finds_the_nearest_vertex_within_tolerance_only() {
        let fx = sm_testkit::meshes::single_hex().expect("hex");
        let g = Grid::new(&fx.mesh, &(0..8).collect::<Vec<u32>>(), 1e-9);
        assert_eq!(g.find(&fx.mesh, Point3::new(1.0, 1.0, 1.0)), Some(6));
        assert_eq!(g.find(&fx.mesh, Point3::new(1.0, 1.0, 1.0 + 1e-6)), None);
    }
}
