//! Local refinement with polyhedral hanging faces (OpenFOAM `hexRef8` style, conformal by
//! construction).
//!
//! Selected hexes are split `levels` times (8^levels children each); a 2:1 balance across faces
//! raises their neighbours. A coarser neighbour is not transitioned: it gains the split sub-faces
//! of the shared face and the hanging edge nodes on the polygons of its other faces, and becomes a
//! `Polyhedron`. Every node is named by the source vertices it spans and their integer
//! multilinear weights (denominator `2^(3 MAXL)`), so two cells that meet at a node, an edge or a
//! face name it identically and the builder merges the shared faces without any matching code.
//!
//! The pass is a serial function of the source mesh: ids come from a fixed traversal (cells in
//! order, leaves in lattice order), so the result is bit-identical for any thread count.

use std::collections::{BTreeMap, HashMap};

use sm_core::Point3;
use sm_mesh_api::{
    CellIx, CellShape, Classification, EdgeTable, FaceIx, FaceKind, Finished, MeshBuild, MeshDim, MeshEntityRef, MeshError, MeshRead, NO_CELL,
    Param, SerialAlloc, Side, VertexIx,
};
use sm_quality::{Context, Metric, eval};
use sm_topo_api::TopologyView;
use smallvec::SmallVec;

use crate::carry::{Lineage, carry_sets_and_tags};
use crate::project::Projector;
use crate::refine::fallback_edge_classes;

/// Most levels a selected hex is split in one call (`8^3 = 512` children).
pub const MAX_LOCAL_LEVELS: u32 = 3;
const M: u32 = MAX_LOCAL_LEVELS;
/// Weight of a source vertex that is a node: `2^(3M)`.
const W: i64 = 1 << (3 * M);
const NONE: u32 = u32::MAX;
const SJ_FLOOR: f64 = 1e-12;
/// Lattice coordinates of the hex corners in the template order.
const HEX_XYZ: [[i64; 3]; 8] = [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]];

type Key = SmallVec<[(u32, i64); 8]>;
type V = [f64; 3];

/// Why a local refinement was refused.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum LocalError {
    /// A cell to be refined is not a hex.
    #[error("cell {cell} is a {shape:?} but would be refined (selected or raised by the 2:1 balance): local refinement splits hexes only")]
    Unrefinable {
        /// The cell index.
        cell: u32,
        /// Its shape.
        shape: CellShape,
    },
    /// With `balance: false`, a face joins cells whose levels differ by more than 1.
    #[error("face {face} joins cells of level {a} and {b}: a jump above 1 needs `balance: true`")]
    Unbalanced {
        /// The face.
        face: u32,
        /// Level of its left cell.
        a: u8,
        /// Level of its right cell.
        b: u8,
    },
    /// Surface meshes.
    #[error("local refinement needs a volume mesh")]
    NotVolume,
    /// High-order cells, extra nodes or face curves.
    #[error("the mesh carries high-order geometry; local refinement is linear only")]
    HighOrder,
    /// Inactive cell.
    #[error("cell {0} is inactive (a refined parent); refine the leaf mesh")]
    Inactive(u32),
    /// A non-conformal face.
    #[error("face {0} is non-conformal; local refinement needs a conformal mesh")]
    NonConformal(u32),
    /// The mesh has no cell, or nothing is selected.
    #[error("{0}")]
    Empty(&'static str),
    /// The level is out of range.
    #[error("levels must be 1..={MAX_LOCAL_LEVELS}, got {0}")]
    Levels(u32),
    /// The edge table could not be derived.
    #[error("cannot classify the new edge nodes: {0}")]
    EdgeClass(String),
    /// More than `u32::MAX` nodes.
    #[error("the refined mesh would exceed 2^32 entities")]
    TooLarge,
    /// An inconsistent mesh.
    #[error("inconsistent mesh: {0}")]
    Inconsistent(String),
    /// A mesh-API refusal while building.
    #[error(transparent)]
    Mesh(#[from] MeshError),
}

/// Counters of one local pass.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct LocalStats {
    /// Cells of the source.
    pub parents: usize,
    /// Cells of the result.
    pub children: usize,
    /// Vertices of the result.
    pub vertices: usize,
    /// Source cells refined (selected or raised by the balance).
    pub refined: usize,
    /// Source cells raised by the balance only.
    pub balanced: usize,
    /// Hex cells of the result.
    pub hexes: usize,
    /// Polyhedra of the result (unrefined cells with hanging faces or nodes, and refined cells next to finer ones).
    pub polyhedra: usize,
    /// New nodes moved onto their model edge or face.
    pub projected: usize,
    /// New boundary nodes left straight because the projection failed.
    pub projection_failed: usize,
    /// New boundary nodes left straight because the projection inverted a cell.
    pub projection_rejected: usize,
}

/// The result of one local pass.
#[non_exhaustive]
pub struct LocalRefined<M> {
    /// The finished mesh.
    pub finished: Finished<M>,
    /// The source parent of every new cell.
    pub coarse_of: Vec<CellIx>,
    /// For every new face the source face that carries it, or [`NO_CELL`].
    pub face_map: Vec<u32>,
    /// Counters.
    pub stats: LocalStats,
    /// Losses, as `(code, message)`.
    pub notes: Vec<(&'static str, String)>,
}

fn vkey(v: u32) -> Key {
    SmallVec::from_slice(&[(v, W)])
}

/// `a + (b - a) j / n` on weights (exact: lattice steps divide `n`).
fn lerp(a: &Key, b: &Key, j: i64, n: i64) -> Key {
    let mut out = Key::new();
    let (mut i, mut k) = (0, 0);
    while i < a.len() || k < b.len() {
        let (v, wa, wb) = match (a.get(i), b.get(k)) {
            (Some(&(va, wa)), Some(&(vb, wb))) if va == vb => {
                i += 1;
                k += 1;
                (va, wa, wb)
            }
            (Some(&(va, wa)), Some(&(vb, _))) if va < vb => {
                i += 1;
                (va, wa, 0)
            }
            (Some(&(va, wa)), None) => {
                i += 1;
                (va, wa, 0)
            }
            (_, Some(&(vb, wb))) => {
                k += 1;
                (vb, 0, wb)
            }
            (None, None) => break,
        };
        let w = wa * (n - j) + wb * j;
        debug_assert_eq!(w % n, 0, "lattice step does not divide the segment");
        if w / n != 0 {
            out.push((v, w / n));
        }
    }
    out
}

/// The multilinear point `(x, y, z) / 2^M` of a hex.
fn hex_pt(c: &[u32; 8], x: i64, y: i64, z: i64) -> Key {
    let u = 1i64 << M;
    let mut out: Key = (0..8)
        .filter_map(|i| {
            let p = HEX_XYZ[i];
            let w = (if p[0] == 1 { x } else { u - x }) * (if p[1] == 1 { y } else { u - y }) * (if p[2] == 1 { z } else { u - z });
            (w > 0).then_some((c[i], w))
        })
        .collect();
    out.sort_unstable_by_key(|e| e.0);
    out
}

/// A cell of the result, before it is handed to the builder.
struct Desc {
    parent: u32,
    shape: CellShape,
    /// Template corners (`Hex` leaves and untouched template cells), else empty.
    corners: SmallVec<[u32; 8]>,
    /// Outward face loops (template face order when `corners` is non-empty).
    loops: Vec<SmallVec<[u32; 6]>>,
    /// Source face carrying each loop, or `NONE`.
    pf: Vec<u32>,
}

struct Pass<'a> {
    src: &'a dyn MeshRead,
    nv: usize,
    pairs: Vec<[u32; 2]>,
    eclass: Vec<Classification>,
    elev: Vec<u8>,
    flev: Vec<u8>,
    face_by_verts: BTreeMap<Vec<u32>, u32>,
    pos: Vec<V>,
    class: Vec<Classification>,
    keys: Vec<Key>,
    /// Lookup only (ids come from creation order, never from iteration), so it cannot affect output order.
    map: HashMap<Key, u32>,
}

impl Pass<'_> {
    fn pt(&self, v: u32) -> V {
        let c = self.src.coords();
        [c.x[v as usize], c.y[v as usize], c.z[v as usize]]
    }

    /// The node of a weighted key, created on first use. `cc` classifies a node inside a cell.
    fn node(&mut self, key: &Key, cc: Classification) -> u32 {
        if key.len() == 1 {
            return key[0].0;
        }
        if let Some(&n) = self.map.get(key) {
            return n;
        }
        let id = (self.nv + self.keys.len()) as u32;
        let mut p = [0.0f64; 3];
        for &(v, w) in key {
            let q = self.pt(v);
            for d in 0..3 {
                p[d] += w as f64 * q[d];
            }
        }
        let scale = W as f64;
        let verts: SmallVec<[u32; 8]> = key.iter().map(|e| e.0).collect();
        let mut class = match verts.len() {
            2 => self.pairs.binary_search(&[verts[0], verts[1]]).ok().map(|e| self.eclass[e]),
            4 => self.face_by_verts.get(verts.as_slice()).map(|&f| self.src.face_class(FaceIx::new(f as usize))),
            _ => None,
        }
        .unwrap_or(cc);
        class.param = Param::None;
        self.pos.push([p[0] / scale, p[1] / scale, p[2] / scale]);
        self.class.push(class);
        self.keys.push(key.clone());
        self.map.insert(key.clone(), id);
        id
    }

    /// Level of the grid the segment `p q` lies on: its source edge's or face's, else 0.
    fn host(&self, p: &Key, q: &Key) -> u8 {
        let mut u: SmallVec<[u32; 8]> = p.iter().chain(q.iter()).map(|e| e.0).collect();
        u.sort_unstable();
        u.dedup();
        match u.len() {
            2 => self.pairs.binary_search(&[u[0], u[1]]).map_or(0, |e| self.elev[e]),
            4 => self.face_by_verts.get(u.as_slice()).map_or(0, |&f| self.flev[f as usize]),
            _ => 0,
        }
    }

    /// Node ids around `ring` (points on a lattice of level `lam`), with the hanging nodes of every
    /// edge whose host grid is finer.
    fn sub_ring(&mut self, ring: &[Key], lam: u32, cc: Classification) -> SmallVec<[u32; 6]> {
        let mut out: SmallVec<[u32; 6]> = SmallVec::new();
        for i in 0..ring.len() {
            let (p, q) = (&ring[i], &ring[(i + 1) % ring.len()]);
            out.push(self.node(p, cc));
            let h = u32::from(self.host(p, q)).max(lam);
            let n = 1i64 << (h - lam);
            for j in 1..n {
                let k = lerp(p, q, j, n);
                out.push(self.node(&k, cc));
            }
        }
        out
    }

    /// The `n x n` sub-quads of a quad ring, each in the parent's orientation.
    fn grid(ring: &[Key; 4], n: i64) -> Vec<[Key; 4]> {
        let g = |a: i64, b: i64| lerp(&lerp(&ring[0], &ring[1], a, n), &lerp(&ring[3], &ring[2], a, n), b, n);
        let mut out = Vec::with_capacity((n * n) as usize);
        for b in 0..n {
            for a in 0..n {
                out.push([g(a, b), g(a + 1, b), g(a + 1, b + 1), g(a, b + 1)]);
            }
        }
        out
    }

    fn lf2f(&self, corners: &[u32], shape: CellShape) -> Vec<u32> {
        shape
            .local_faces()
            .iter()
            .map(|lf| {
                let mut k: Vec<u32> = lf.iter().map(|&i| corners[i as usize]).collect();
                k.sort_unstable();
                self.face_by_verts.get(&k).copied().unwrap_or(NONE)
            })
            .collect()
    }

    /// An unrefined cell: itself when nothing hangs on it, else a polyhedron.
    fn level0(&mut self, c: usize) -> Result<Desc, LocalError> {
        let src = self.src;
        let ci = CellIx::new(c);
        let cc = src.cell_class(ci);
        let shape = src.cell_shape(ci);
        let mut loops: Vec<SmallVec<[u32; 6]>> = Vec::new();
        let mut pf: Vec<u32> = Vec::new();
        let mut changed = false;
        for sf in src.cell_faces(ci) {
            let f = sf.face().get();
            let mut ring: Vec<u32> = src.face_vertices(sf.face()).iter().map(|v| v.0).collect();
            if sf.side() == Side::Right {
                ring.reverse();
            }
            let pts: Vec<Key> = ring.iter().map(|&v| vkey(v)).collect();
            let fl = u32::from(self.flev[f]);
            if ring.len() == 4 && fl > 0 {
                changed = true;
                let q: [Key; 4] = [pts[0].clone(), pts[1].clone(), pts[2].clone(), pts[3].clone()];
                for sq in Self::grid(&q, 1i64 << fl) {
                    loops.push(self.sub_ring(&sq, fl, cc));
                    pf.push(f as u32);
                }
            } else {
                let r = self.sub_ring(&pts, 0, cc);
                changed |= r.len() != ring.len();
                loops.push(r);
                pf.push(f as u32);
            }
        }
        if !changed && let Some(cs) = src.cell_corners(ci) && shape != CellShape::Polyhedron {
            let corners: SmallVec<[u32; 8]> = cs.iter().map(|v| v.0).collect();
            let lf = self.lf2f(&corners, shape);
            let loops = shape.local_faces().iter().map(|l| l.iter().map(|&i| corners[i as usize]).collect()).collect();
            return Ok(Desc { parent: c as u32, shape, corners, loops, pf: lf });
        }
        Ok(Desc { parent: c as u32, shape: CellShape::Polyhedron, corners: SmallVec::new(), loops, pf })
    }

    /// The `8^lv` leaves of a refined hex.
    fn leaves(&mut self, c: usize, lv: u32, out: &mut Vec<Desc>) -> Result<(), LocalError> {
        let src = self.src;
        let ci = CellIx::new(c);
        let cc = src.cell_class(ci);
        let cs = src.cell_corners(ci).ok_or_else(|| LocalError::Inconsistent(format!("hex {c} has no corners")))?;
        let corners: [u32; 8] = std::array::from_fn(|i| cs[i].0);
        let lf2f = self.lf2f(&corners, CellShape::Hex);
        let (n, unit) = (1i64 << lv, 1i64 << (M - lv));
        let faces = CellShape::Hex.local_faces();
        for k in 0..n {
            for j in 0..n {
                for i in 0..n {
                    let lc: Vec<Key> = HEX_XYZ.iter().map(|d| hex_pt(&corners, (i + d[0]) * unit, (j + d[1]) * unit, (k + d[2]) * unit)).collect();
                    let mut loops: Vec<SmallVec<[u32; 6]>> = Vec::with_capacity(6);
                    let mut pf: Vec<u32> = Vec::with_capacity(6);
                    let mut plain = true;
                    for (jf, lf) in faces.iter().enumerate() {
                        let ring: [Key; 4] = std::array::from_fn(|t| lc[lf[t] as usize].clone());
                        let outer = [k == 0, k == n - 1, j == 0, i == n - 1, j == n - 1, i == 0][jf];
                        if outer {
                            let f = lf2f[jf];
                            if f == NONE {
                                return Err(LocalError::Inconsistent(format!("hex {c}: a face is missing from its face list")));
                            }
                            let fl = u32::from(self.flev[f as usize]).max(lv);
                            if fl > lv {
                                plain = false;
                                for sq in Self::grid(&ring, 1i64 << (fl - lv)) {
                                    loops.push(self.sub_ring(&sq, fl, cc));
                                    pf.push(f);
                                }
                                continue;
                            }
                            let r = self.sub_ring(&ring, lv, cc);
                            plain &= r.len() == 4;
                            loops.push(r);
                            pf.push(f);
                        } else {
                            let r = self.sub_ring(&ring, lv, cc);
                            plain &= r.len() == 4;
                            loops.push(r);
                            pf.push(NONE);
                        }
                    }
                    let d = if plain {
                        let ids: SmallVec<[u32; 8]> = lc.iter().map(|k| self.node(k, cc)).collect();
                        Desc { parent: c as u32, shape: CellShape::Hex, corners: ids, loops, pf }
                    } else {
                        Desc { parent: c as u32, shape: CellShape::Polyhedron, corners: SmallVec::new(), loops, pf }
                    };
                    out.push(d);
                }
            }
        }
        Ok(())
    }
}

/// Levels of every cell: `levels` where selected, raised by the 2:1 balance across faces
/// (`balance`), else refused when a face has a jump above 1.
fn levels_of(src: &dyn MeshRead, selected: &[bool], levels: u32, balance: bool) -> Result<Vec<u8>, LocalError> {
    let nc = src.cell_count();
    let mut l: Vec<u8> = selected.iter().map(|&s| if s { levels as u8 } else { 0 }).collect();
    let faces: Vec<(usize, usize)> = (0..src.face_count())
        .filter_map(|f| {
            let fi = FaceIx::new(f);
            src.face_right(fi).map(|r| (src.face_left(fi).get(), r.get()))
        })
        .collect();
    if balance {
        loop {
            let mut changed = false;
            for &(a, b) in &faces {
                if l[a] > l[b] + 1 {
                    l[b] = l[a] - 1;
                    changed = true;
                } else if l[b] > l[a] + 1 {
                    l[a] = l[b] - 1;
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }
    } else {
        for (f, (a, b)) in (0..src.face_count()).zip(faces_by_index(src)) {
            if l[a].abs_diff(l[b]) > 1 {
                return Err(LocalError::Unbalanced { face: f as u32, a: l[a], b: l[b] });
            }
        }
    }
    for c in 0..nc {
        let shape = src.cell_shape(CellIx::new(c));
        if l[c] > 0 && (shape != CellShape::Hex || src.cell_corners(CellIx::new(c)).is_none()) {
            return Err(LocalError::Unrefinable { cell: c as u32, shape });
        }
    }
    Ok(l)
}

/// `(left, right)` per face, with `(0, 0)` for boundary faces (never a jump).
fn faces_by_index(src: &dyn MeshRead) -> Vec<(usize, usize)> {
    (0..src.face_count())
        .map(|f| {
            let fi = FaceIx::new(f);
            let a = src.face_left(fi).get();
            (a, src.face_right(fi).map_or(a, |r| r.get()))
        })
        .collect()
}

fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn norm(a: V) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

fn mean(p: &[V]) -> V {
    let k = 1.0 / p.len() as f64;
    let mut s = [0.0; 3];
    for q in p {
        s = [s[0] + q[0], s[1] + q[1], s[2] + q[2]];
    }
    [s[0] * k, s[1] * k, s[2] * k]
}

/// What [`refine_local`] refines and how.
#[derive(Clone, Copy, Debug)]
pub struct LocalOptions<'a> {
    /// One flag per source cell: refine it (hexes only).
    pub selected: &'a [bool],
    /// Times a selected cell is split, `1..=MAX_LOCAL_LEVELS`.
    pub levels: u32,
    /// Raise neighbours so no face has a level jump above 1; else such a jump is refused.
    pub balance: bool,
    /// Project new boundary nodes onto the model (needs the topology view).
    pub project: bool,
}

/// Refine the cells flagged in `selected` (hexes) `levels` times, balance across faces when
/// `balance`, and return the new mesh with the parent map and face map of a `LevelMap`. `topo`
/// resolves edge classes and, with `project`, moves new boundary nodes onto the model unless a
/// hex would invert. `b` must be an empty builder.
///
/// # Errors
/// [`LocalError`] for an unrefinable selection, an unbalanced jump with `balance: false`, or a
/// mesh the pass does not handle; `alloc` is unchanged then.
pub fn refine_local<B: MeshBuild>(
    src: &dyn MeshRead,
    topo: Option<&dyn TopologyView>,
    opts: &LocalOptions<'_>,
    mut b: B,
    alloc: &mut SerialAlloc,
) -> Result<LocalRefined<B::Output>, LocalError> {
    let LocalOptions { selected, levels, balance, project } = *opts;
    if !(1..=MAX_LOCAL_LEVELS).contains(&levels) {
        return Err(LocalError::Levels(levels));
    }
    if src.dim() != MeshDim::Three {
        return Err(LocalError::NotVolume);
    }
    let (nc, nv, nf) = (src.cell_count(), src.vertex_count(), src.face_count());
    if nc == 0 {
        return Err(LocalError::Empty("the mesh has no cells"));
    }
    if !selected.iter().any(|&s| s) {
        return Err(LocalError::Empty("no cell is selected"));
    }
    if src.coords().len() != nv {
        return Err(LocalError::HighOrder);
    }
    for c in 0..nc {
        let ci = CellIx::new(c);
        if !src.cell_active(ci) {
            return Err(LocalError::Inactive(c as u32));
        }
        if src.high_order_cell(ci).is_some() {
            return Err(LocalError::HighOrder);
        }
    }
    for f in 0..nf {
        let fi = FaceIx::new(f);
        if src.face_curve(fi).is_some() {
            return Err(LocalError::HighOrder);
        }
        if src.face_kind(fi) == FaceKind::NonConformal {
            return Err(LocalError::NonConformal(f as u32));
        }
    }
    let lv = levels_of(src, selected, levels, balance)?;
    let (pairs, eclass): (Vec<[u32; 2]>, Vec<Classification>) = if let Some(t) = src.edges() {
        (t.vertices.iter().map(|[a, bb]| [a.0, bb.0]).collect(), t.class.clone())
    } else {
        let t = EdgeTable::derive(src, &mut SerialAlloc::new(), &fallback_edge_classes(src, topo)).map_err(|e| LocalError::EdgeClass(e.to_string()))?;
        (t.vertices.iter().map(|[a, bb]| [a.0, bb.0]).collect(), t.class)
    };
    // Grid levels: a face takes the finer of its two cells, an edge the finest around it.
    let mut elev = vec![0u8; pairs.len()];
    for c in 0..nc {
        for sf in src.cell_faces(CellIx::new(c)) {
            let l = src.face_vertices(sf.face());
            for i in 0..l.len() {
                let (x, y) = (l[i].0, l[(i + 1) % l.len()].0);
                if let Ok(e) = pairs.binary_search(&[x.min(y), x.max(y)]) {
                    elev[e] = elev[e].max(lv[c]);
                }
            }
        }
    }
    let flev: Vec<u8> = (0..nf)
        .map(|f| {
            let fi = FaceIx::new(f);
            let a = lv[src.face_left(fi).get()];
            src.face_right(fi).map_or(a, |r| a.max(lv[r.get()]))
        })
        .collect();
    let mut face_by_verts = BTreeMap::new();
    for f in 0..nf {
        let mut k: Vec<u32> = src.face_vertices(FaceIx::new(f)).iter().map(|v| v.0).collect();
        k.sort_unstable();
        face_by_verts.insert(k, f as u32);
    }
    let mut p = Pass {
        src,
        nv,
        pairs,
        eclass,
        elev,
        flev,
        face_by_verts,
        pos: Vec::new(),
        class: Vec::new(),
        keys: Vec::new(),
        map: HashMap::new(),
    };
    let mut descs: Vec<Desc> = Vec::with_capacity(nc);
    let mut child_start: Vec<u32> = Vec::with_capacity(nc + 1);
    for c in 0..nc {
        child_start.push(descs.len() as u32);
        if lv[c] == 0 {
            let d = p.level0(c)?;
            descs.push(d);
        } else {
            p.leaves(c, u32::from(lv[c]), &mut descs)?;
        }
    }
    child_start.push(descs.len() as u32);
    let nn = nv + p.keys.len();
    if nn >= NONE as usize || descs.len() >= NONE as usize {
        return Err(LocalError::TooLarge);
    }
    let mut stats = LocalStats { parents: nc, children: descs.len(), vertices: nn, ..LocalStats::default() };
    stats.refined = lv.iter().filter(|&&l| l > 0).count();
    stats.balanced = lv.iter().zip(selected).filter(|&(&l, &s)| l > 0 && !s).count();
    stats.hexes = descs.iter().filter(|d| d.shape == CellShape::Hex).count();
    stats.polyhedra = descs.iter().filter(|d| d.shape == CellShape::Polyhedron).count();

    // ---- projection of new boundary nodes (hex leaves guard against inversion) --------------------
    let mut pos: Vec<V> = (0..nv as u32).map(|v| p.pt(v)).collect();
    pos.extend(p.pos.iter().copied());
    let mut class: Vec<Classification> = (0..nv).map(|v| src.vertex_class(VertexIx::new(v))).collect();
    class.extend(p.class.iter().copied());
    if let (Some(topo), true) = (topo, project) {
        let pj = Projector::new(topo);
        let mut inc: Vec<Vec<u32>> = vec![Vec::new(); nn - nv];
        for (k, d) in descs.iter().enumerate() {
            let mut seen: SmallVec<[u32; 32]> = d.corners.iter().copied().chain(d.loops.iter().flatten().copied()).filter(|&n| n as usize >= nv).collect();
            seen.sort_unstable();
            seen.dedup();
            for n in seen {
                inc[n as usize - nv].push(k as u32);
            }
        }
        // Hexes: scaled Jacobian. Polyhedra: the smallest sine of the fan tets (cell centre, edge,
        // face centre) of every face loop; positive for a star-shaped cell.
        let measure = |k: usize, pos: &[V]| -> f64 {
            let d = &descs[k];
            if d.shape == CellShape::Hex {
                let pts: SmallVec<[V; 8]> = d.corners.iter().map(|&v| pos[v as usize]).collect();
                return eval(Metric::ScaledJacobian, CellShape::Hex, &pts, &Context { normal: None, ref_size: f64::NAN }).unwrap_or(f64::NAN);
            }
            let mut all: SmallVec<[u32; 32]> = d.loops.iter().flatten().copied().collect();
            all.sort_unstable();
            all.dedup();
            let c = mean(&all.iter().map(|&n| pos[n as usize]).collect::<SmallVec<[V; 32]>>());
            let mut m = f64::INFINITY;
            for l in &d.loops {
                let f = mean(&l.iter().map(|&n| pos[n as usize]).collect::<SmallVec<[V; 8]>>());
                for i in 0..l.len() {
                    let (a, bb) = (sub(pos[l[i] as usize], c), sub(pos[l[(i + 1) % l.len()] as usize], c));
                    let ff = sub(f, c);
                    let det = a[0] * (bb[1] * ff[2] - bb[2] * ff[1]) - a[1] * (bb[0] * ff[2] - bb[2] * ff[0]) + a[2] * (bb[0] * ff[1] - bb[1] * ff[0]);
                    let s = norm(a) * norm(bb) * norm(ff);
                    let v = if s > 0.0 { det / s } else { f64::NAN };
                    if v.is_nan() {
                        return f64::NAN;
                    }
                    m = m.min(v);
                }
            }
            m
        };
        let min_sj = |n: usize, pos: &[V]| -> f64 {
            let mut m = f64::INFINITY;
            for &k in &inc[n - nv] {
                let v = measure(k as usize, pos);
                if v.is_nan() {
                    return f64::NAN;
                }
                m = m.min(v);
            }
            m
        };
        for n in nv..nn {
            if !pj.applies(class[n]) {
                continue;
            }
            let Some(q) = pj.project(class[n], pos[n]) else {
                stats.projection_failed += 1;
                continue;
            };
            let old = pos[n];
            let scale = 1.0 + old[0].abs().max(old[1].abs()).max(old[2].abs());
            let d = sub(old, q.point);
            let new = if d[0] * d[0] + d[1] * d[1] + d[2] * d[2] <= (1e-13 * scale).powi(2) { old } else { q.point };
            let before = min_sj(n, &pos);
            pos[n] = new;
            let after = min_sj(n, &pos);
            let bad = after.is_nan() || after <= SJ_FLOOR;
            if bad && (before > SJ_FLOOR || after < before) {
                pos[n] = old;
                stats.projection_rejected += 1;
                continue;
            }
            class[n].param = q.param;
            if new != old {
                stats.projected += 1;
            }
        }
    }

    // ---- build ----------------------------------------------------------------------------------------
    b.reserve(nn, descs.len());
    for n in 0..nn {
        let v = b.add_vertex(Point3::new(pos[n][0], pos[n][1], pos[n][2]))?;
        b.set_class(MeshEntityRef::Vertex(v), class[n])?;
    }
    let mut on_parent: Vec<(u32, FaceIx)> = Vec::new();
    for d in &descs {
        let parent = CellIx::new(d.parent as usize);
        let added = if d.corners.is_empty() {
            let loops: Vec<Vec<VertexIx>> = d.loops.iter().map(|l| l.iter().map(|&v| VertexIx(v)).collect()).collect();
            let refs: Vec<&[VertexIx]> = loops.iter().map(Vec::as_slice).collect();
            b.add_polyhedron(&refs)?
        } else {
            let verts: SmallVec<[VertexIx; 8]> = d.corners.iter().map(|&v| VertexIx(v)).collect();
            b.add_cell_from_vertices(d.shape, &verts)?
        };
        b.set_class(MeshEntityRef::Cell(added.cell), src.cell_class(parent))?;
        for (i, sf) in added.faces.iter().enumerate() {
            if added.reused[i] {
                continue;
            }
            let pf = d.pf[i];
            if pf == NONE {
                b.set_class(MeshEntityRef::Face(sf.face()), src.cell_class(parent))?;
            } else {
                let pfi = FaceIx::new(pf as usize);
                b.set_class(MeshEntityRef::Face(sf.face()), src.face_class(pfi))?;
                if src.face_kind(pfi) == FaceKind::PartitionCut {
                    b.set_face_kind(sf.face(), FaceKind::PartitionCut)?;
                }
                on_parent.push((pf, sf.face()));
            }
        }
    }
    if src.edges().is_some() {
        // Chains of nodes along refined source edges keep the edge's class.
        for (e, [a, bb]) in p.pairs.iter().enumerate() {
            let n = 1i64 << p.elev[e];
            if n == 1 {
                b.set_class(MeshEntityRef::Edge(VertexIx(*a), VertexIx(*bb)), p.eclass[e])?;
                continue;
            }
            let (ka, kb) = (vkey(*a), vkey(*bb));
            let ids: Vec<u32> = (0..=n)
                .map(|j| {
                    let k = lerp(&ka, &kb, j, n);
                    if k.len() == 1 { k[0].0 } else { p.map.get(&k).copied().unwrap_or(NONE) }
                })
                .collect();
            for w in ids.windows(2) {
                if w[0] != NONE && w[1] != NONE {
                    b.set_class(MeshEntityRef::Edge(VertexIx(w[0]), VertexIx(w[1])), p.eclass[e])?;
                }
            }
        }
    }
    on_parent.sort_by_key(|(pf, fh)| (*pf, fh.get()));
    let c_parent: Vec<u32> = descs.iter().map(|d| d.parent).collect();
    let span: Vec<SmallVec<[u32; 8]>> = p.keys.iter().map(|k| k.iter().map(|e| e.0).collect()).collect();
    // A node lies on the mesh boundary when it sits on an edge or a quad of a boundary face.
    let mut bedges: std::collections::BTreeSet<[u32; 2]> = std::collections::BTreeSet::new();
    for f in 0..nf {
        let fi = FaceIx::new(f);
        if src.face_right(fi).is_none() {
            let l = src.face_vertices(fi);
            for i in 0..l.len() {
                let (x, y) = (l[i].0, l[(i + 1) % l.len()].0);
                bedges.insert([x.min(y), x.max(y)]);
            }
        }
    }
    let joins: Vec<bool> = span
        .iter()
        .map(|sp| match sp.len() {
            2 => bedges.contains(&[sp[0], sp[1]]),
            4 => p.face_by_verts.get(sp.as_slice()).is_some_and(|&f| src.face_right(FaceIx::new(f as usize)).is_none()),
            _ => false,
        })
        .collect();
    let notes = carry_sets_and_tags(
        src,
        &mut b,
        &Lineage {
            nv,
            nn,
            pairs: &[],
            edge_ent: &[],
            qfaces: &[],
            cen_cells: &[],
            base_e: nv,
            base_q: nn,
            base_c: nn,
            child_start: &child_start,
            c_parent: &c_parent,
            on_parent: &on_parent,
            span: Some(&span),
            joins: &joins,
        },
    )?;
    let fin = b.finish(alloc)?;
    let mut face_map = vec![NO_CELL; fin.mesh.face_count()];
    for (pf, fh) in &on_parent {
        face_map[fin.face_remap[fh.get()].get()] = *pf;
    }
    let coarse_of = c_parent.iter().map(|&q| CellIx::new(q as usize)).collect();
    Ok(LocalRefined { finished: fin, coarse_of, face_map, stats, notes })
}
