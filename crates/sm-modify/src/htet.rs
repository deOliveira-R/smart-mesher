//! Hex, wedge and pyramid to tetrahedra (`mesh.htet`) and quads to triangles (`mesh.qtri`), with
//! conformity by vertex ordering (ruling D2: no propagation, no transition pyramids).
//!
//! Every quad face is cut along the diagonal through its lowest-ranked corner, where the rank of
//! a vertex is `(root, index)`: the root is the lowest index of the vertex's deck class, so the
//! two faces of a deck pair get matching diagonals whatever the index permutation. A cell is cut
//! as a cone from its lowest-ranked corner over the faces that do not contain it (those contain
//! the corner in degenerate form only), each quad face split by the rule above. The cone is the
//! Dompierre decomposition: a hex gives 6 tets, or 5 when the three far faces are cut away from
//! the corner opposite the lowest one; a wedge 3; a pyramid 2. No ordering needs a Steiner point
//! (the exhaustive 8! test of `tests/htet.rs` pins that).
//!
//! A selected cell next to an unselected one cuts the shared quad into two triangles; the
//! unselected neighbour keeps its other faces and becomes a polyhedron with one more face per cut
//! quad (`closure: polyhedral`) or the pass is refused (`closure: whole`).

use std::collections::BTreeMap;

use sm_core::Point3;
use sm_mesh_api::{
    CellIx, CellShape, FaceIx, FaceKind, Finished, MeshBuild, MeshDim, MeshEntityRef, MeshError, MeshRead,
    NO_CELL, SerialAlloc, Side, VertexIx,
};
use smallvec::SmallVec;

use crate::carry::{Lineage, carry_sets_and_tags};

type V = [f64; 3];

/// How a selection that is not closed under face adjacency is treated.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum Closure {
    /// An unselected neighbour of a cut quad gains a face and becomes a polyhedron.
    #[default]
    Polyhedral,
    /// Refuse unless the selection is closed: no unselected cell may share a quad with a selected one.
    Whole,
}

/// Why a mesh cannot be converted.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum HtetError {
    /// `htet` needs a 3D mesh, `qtri` a 2D one.
    #[error("{0}")]
    WrongDimension(&'static str),
    /// High-order cells, extra nodes or face curves.
    #[error("the mesh carries high-order geometry; the conversion is linear only")]
    HighOrder,
    /// A refined parent retained inactive.
    #[error("cell {0} is inactive (a refined parent); convert the leaf mesh")]
    Inactive(u32),
    /// A non-conformal face.
    #[error("face {0} is non-conformal; the conversion needs a conformal mesh")]
    NonConformal(u32),
    /// The mesh has no cell.
    #[error("the mesh has no cells")]
    Empty,
    /// `closure: whole` with a selection that is not closed.
    #[error("closure `whole`: cell {neighbour} shares a quad with selected cell {selected} but is not selected; select it too or use closure `polyhedral`")]
    OpenClosure {
        /// The unselected neighbour that would need a polyhedral face.
        neighbour: u32,
        /// The selected cell it touches.
        selected: u32,
    },
    /// A fixed diagonal choice gives a tet of zero or negative volume.
    #[error("htet.inverted_tet: cell {cell} is distorted (a folded or concave face): its cut gives a tetrahedron of signed volume {volume:.3e}")]
    InvertedTet {
        /// The offending cell.
        cell: u32,
        /// The signed volume of the tet.
        volume: f64,
    },
    /// A deck-root array of the wrong length.
    #[error("root array has {got} entries for {want} vertices")]
    BadRoot {
        /// Entries given.
        got: usize,
        /// Vertices of the mesh.
        want: usize,
    },
    /// A selection mask of the wrong length.
    #[error("selection has {got} entries for {want} cells")]
    BadSelection {
        /// Entries given.
        got: usize,
        /// Cells of the mesh.
        want: usize,
    },
    /// The mesh is inconsistent.
    #[error("inconsistent mesh: {0}")]
    Inconsistent(String),
    /// A mesh-API refusal while building.
    #[error(transparent)]
    Mesh(#[from] MeshError),
}

/// Counters of one conversion.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct HtetStats {
    /// Cells of the source.
    pub parents: usize,
    /// Cells of the result.
    pub children: usize,
    /// Source cells that were cut.
    pub cut_cells: usize,
    /// Hexes cut into 5 tets.
    pub hex5: usize,
    /// Hexes cut into 6 tets.
    pub hex6: usize,
    /// Quads (faces in 3D, cells in 2D) cut into two triangles.
    pub cut_quads: usize,
    /// Unselected cells that became polyhedra.
    pub polyhedra: usize,
}

/// The result of a conversion.
#[non_exhaustive]
pub struct Converted<M> {
    /// The finished mesh.
    pub finished: Finished<M>,
    /// The source parent of every new cell.
    pub coarse_of: Vec<CellIx>,
    /// For every new face the source face that carries it, or [`NO_CELL`].
    pub face_map: Vec<u32>,
    /// Counters.
    pub stats: HtetStats,
    /// Losses and choices, as `(code, message)`.
    pub notes: Vec<(&'static str, String)>,
}

/// Options of [`htet`].
#[derive(Clone, Copy, Debug, Default)]
pub struct HtetOptions<'a> {
    /// Cells to cut (`None`: all convertible cells). Entries for tets are ignored.
    pub select: Option<&'a [bool]>,
    /// Deck root of every vertex (`None`: the vertex itself), so deck partners rank alike.
    pub root: Option<&'a [u32]>,
    /// What to do with an unselected neighbour of a cut quad.
    pub closure: Closure,
}

fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn det(a: V, b: V, c: V) -> f64 {
    a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])
}

fn convertible(s: CellShape) -> bool {
    matches!(s, CellShape::Hex | CellShape::Wedge | CellShape::Pyramid)
}

/// Position in `loop_` of the lowest-ranked vertex.
fn lowest(loop_: &[u32], rank: &impl Fn(u32) -> (u32, u32)) -> usize {
    (0..loop_.len()).min_by_key(|&i| rank(loop_[i])).unwrap_or(0)
}

/// The two triangles of a quad loop cut through its lowest-ranked corner (same orientation as the loop).
fn cut_quad(l: &[u32], rank: &impl Fn(u32) -> (u32, u32)) -> [[u32; 3]; 2] {
    let k = lowest(l, rank);
    [[l[k], l[(k + 1) % 4], l[(k + 2) % 4]], [l[k], l[(k + 2) % 4], l[(k + 3) % 4]]]
}

fn key3(t: [u32; 3]) -> SmallVec<[u32; 4]> {
    let mut k: SmallVec<[u32; 4]> = SmallVec::from_slice(&t);
    k.sort_unstable();
    k
}

/// The tets of one convertible cell with corners `c` (template order): `(tets, five)`. Tets are
/// `[apex, a, b, c]` with `(a, b, c)` outward, so a valid cell gives positive determinants.
fn cell_tets(shape: CellShape, c: &[u32], rank: &impl Fn(u32) -> (u32, u32)) -> (SmallVec<[[u32; 4]; 6]>, bool) {
    let faces = shape.local_faces();
    let loops: SmallVec<[SmallVec<[u32; 4]>; 6]> = faces.iter().map(|f| f.iter().map(|&i| c[i as usize]).collect()).collect();
    let v0 = (0..c.len()).min_by_key(|&i| rank(c[i])).map_or(c[0], |i| c[i]);
    let far: SmallVec<[&SmallVec<[u32; 4]>; 3]> = loops.iter().filter(|l| !l.contains(&v0)).collect();
    let mut tets: SmallVec<[[u32; 4]; 6]> = SmallVec::new();
    if shape == CellShape::Hex {
        // The corner common to all three far faces is opposite to v0.
        let v7 = far[0].iter().copied().find(|v| far.iter().all(|l| l.contains(v)));
        if let Some(v7) = v7 {
            let avoid = |l: &SmallVec<[u32; 4]>| {
                let k = lowest(l, rank);
                l[k] != v7 && l[(k + 2) % 4] != v7
            };
            if far.iter().all(|l| avoid(l)) {
                // Cap triangles (P, v7, N) around v7; the three neighbours of v7 are P1, N1 and the
                // third one of face 2.
                let pn = |l: &SmallVec<[u32; 4]>| {
                    let p = l.iter().position(|&v| v == v7).unwrap_or(0);
                    (l[(p + 3) % 4], l[(p + 1) % 4])
                };
                let (p1, n1) = pn(far[0]);
                let (p2, n2) = pn(far[1]);
                let third = if p2 != p1 && p2 != n1 { p2 } else { n2 };
                for l in &far {
                    let t = cut_quad(l, rank);
                    // The triangle not containing v7 (its cone tet with v0).
                    let tri = if t[0].contains(&v7) { t[1] } else { t[0] };
                    tets.push([v0, tri[0], tri[1], tri[2]]);
                }
                tets.push([third, p1, v7, n1]);
                tets.push([v0, n1, p1, third]);
                return (tets, true);
            }
        }
    }
    for l in &far {
        if l.len() == 3 {
            tets.push([v0, l[0], l[1], l[2]]);
        } else {
            for t in cut_quad(l, rank) {
                tets.push([v0, t[0], t[1], t[2]]);
            }
        }
    }
    (tets, false)
}

/// The tetrahedra of a hexahedron with corners `0..8` (template order) whose corners rank in the order
/// of `rank` (lower first): `(tets, five)`, each tet `[apex, a, b, c]` of corner indices, positive for a
/// right-handed convex cell, `five` true for the 5-tet decomposition. Exposed for the exhaustive
/// 8! test; [`htet`] applies the same rule per cell.
pub fn hex_tets(rank: &[u32; 8]) -> (Vec<[u32; 4]>, bool) {
    let c: [u32; 8] = core::array::from_fn(|i| i as u32);
    let (t, five) = cell_tets(CellShape::Hex, &c, &|v| (rank[v as usize], v));
    (t.to_vec(), five)
}

/// Convert the selected hexes, wedges and pyramids of `src` to tetrahedra.
///
/// Vertices keep their indices; classification, sets, blocks and tags carry over (see
/// `mesh.htet`). `b` must be an empty 3D builder.
///
/// # Errors
/// [`HtetError`]; `alloc` is unchanged then.
pub fn htet<B: MeshBuild>(
    src: &dyn MeshRead,
    opts: &HtetOptions<'_>,
    mut b: B,
    alloc: &mut SerialAlloc,
) -> Result<Converted<B::Output>, HtetError> {
    if src.dim() != MeshDim::Three {
        return Err(HtetError::WrongDimension("mesh.htet needs a 3D mesh (use mesh.qtri for surface meshes)"));
    }
    let (nc, nv, nf) = (src.cell_count(), src.vertex_count(), src.face_count());
    if nc == 0 {
        return Err(HtetError::Empty);
    }
    if src.coords().len() != nv {
        return Err(HtetError::HighOrder);
    }
    if let Some(s) = opts.select
        && s.len() != nc
    {
        return Err(HtetError::BadSelection { got: s.len(), want: nc });
    }
    for c in 0..nc {
        let ci = CellIx::new(c);
        if !src.cell_active(ci) {
            return Err(HtetError::Inactive(c as u32));
        }
        if src.high_order_cell(ci).is_some() {
            return Err(HtetError::HighOrder);
        }
    }
    for f in 0..nf {
        let fi = FaceIx::new(f);
        if src.face_curve(fi).is_some() {
            return Err(HtetError::HighOrder);
        }
        if src.face_kind(fi) == FaceKind::NonConformal {
            return Err(HtetError::NonConformal(f as u32));
        }
    }
    if let Some(r) = opts.root
        && r.len() != nv
    {
        return Err(HtetError::BadRoot { got: r.len(), want: nv });
    }
    let root = |v: u32| opts.root.map_or(v, |r| r[v as usize]);
    let rank = |v: u32| (root(v), v);
    let co = src.coords();
    let pt = |v: u32| -> V { [co.x[v as usize], co.y[v as usize], co.z[v as usize]] };

    // ---- selection and cut faces -----------------------------------------------------------------
    let selected: Vec<bool> = (0..nc)
        .map(|c| convertible(src.cell_shape(CellIx::new(c))) && opts.select.is_none_or(|s| s[c]))
        .collect();
    let mut split = vec![false; nf];
    for c in (0..nc).filter(|&c| selected[c]) {
        for sf in src.cell_faces(CellIx::new(c)) {
            if src.face_vertices(sf.face()).len() == 4 {
                split[sf.face().get()] = true;
            }
        }
    }
    if opts.closure == Closure::Whole {
        for f in (0..nf).filter(|&f| split[f]) {
            let fi = FaceIx::new(f);
            let cells = [Some(src.face_left(fi)), src.face_right(fi)];
            let sel = cells.iter().flatten().find(|c| selected[c.get()]).map(|c| c.get() as u32);
            let open = cells.iter().flatten().find(|c| !selected[c.get()]);
            if let (Some(selected), Some(n)) = (sel, open) {
                return Err(HtetError::OpenClosure { neighbour: n.get() as u32, selected });
            }
        }
    }

    // ---- build ----------------------------------------------------------------------------------------
    b.reserve(nv, nc * 5);
    for v in 0..nv {
        let p = pt(v as u32);
        let nvx = b.add_vertex(Point3::new(p[0], p[1], p[2]))?;
        b.set_class(MeshEntityRef::Vertex(nvx), src.vertex_class(VertexIx::new(v)))?;
    }
    // Parent face of every new triangle half and of every kept face, by sorted vertex set.
    let mut parent_face: BTreeMap<SmallVec<[u32; 4]>, u32> = BTreeMap::new();
    for f in 0..nf {
        let l: SmallVec<[u32; 4]> = src.face_vertices(FaceIx::new(f)).iter().map(|v| v.0).collect();
        if split[f] {
            for t in cut_quad(&l, &rank) {
                parent_face.insert(key3(t), f as u32);
            }
        } else {
            let mut k = l;
            k.sort_unstable();
            parent_face.insert(k, f as u32);
        }
    }
    let mut stats = HtetStats { parents: nc, cut_quads: split.iter().filter(|&&s| s).count(), ..HtetStats::default() };
    let mut child_start: Vec<u32> = Vec::with_capacity(nc + 1);
    let mut c_parent: Vec<u32> = Vec::with_capacity(nc * 5);
    let mut on_parent: Vec<(u32, FaceIx)> = Vec::new();
    let mut ncells = 0u32;
    let place = |b: &mut B,
                     added: sm_mesh_api::CellAdded,
                     parent: usize,
                     on_parent: &mut Vec<(u32, FaceIx)>,
                     parent_face: &BTreeMap<SmallVec<[u32; 4]>, u32>|
     -> Result<(), HtetError> {
        b.set_class(MeshEntityRef::Cell(added.cell), src.cell_class(CellIx::new(parent)))?;
        for (i, sf) in added.faces.iter().enumerate() {
            if added.reused[i] {
                continue;
            }
            let mut k: SmallVec<[u32; 4]> = b.face_loop(sf.face()).iter().map(|v| v.0).collect();
            k.sort_unstable();
            if let Some(&pf) = parent_face.get(&k) {
                let pfi = FaceIx::new(pf as usize);
                b.set_class(MeshEntityRef::Face(sf.face()), src.face_class(pfi))?;
                if src.face_kind(pfi) == FaceKind::PartitionCut {
                    b.set_face_kind(sf.face(), FaceKind::PartitionCut)?;
                }
                on_parent.push((pf, sf.face()));
            } else {
                b.set_class(MeshEntityRef::Face(sf.face()), src.cell_class(CellIx::new(parent)))?;
            }
        }
        Ok(())
    };
    for c in 0..nc {
        child_start.push(ncells);
        let ci = CellIx::new(c);
        let shape = src.cell_shape(ci);
        if selected[c] {
            let corners: SmallVec<[u32; 8]> = src
                .cell_corners(ci)
                .ok_or_else(|| HtetError::Inconsistent(format!("cell {c} has no corner list")))?
                .iter()
                .map(|v| v.0)
                .collect();
            let (tets, five) = cell_tets(shape, &corners, &rank);
            stats.cut_cells += 1;
            if shape == CellShape::Hex {
                if five {
                    stats.hex5 += 1;
                } else {
                    stats.hex6 += 1;
                }
            }
            for t in tets {
                let p: [V; 4] = [pt(t[0]), pt(t[1]), pt(t[2]), pt(t[3])];
                let d = det(sub(p[1], p[0]), sub(p[2], p[0]), sub(p[3], p[0]));
                if d <= 0.0 || !d.is_finite() {
                    return Err(HtetError::InvertedTet { cell: c as u32, volume: d / 6.0 });
                }
                let verts = [VertexIx(t[0]), VertexIx(t[1]), VertexIx(t[2]), VertexIx(t[3])];
                let added = b.add_cell_from_vertices(CellShape::Tet, &verts)?;
                place(&mut b, added, c, &mut on_parent, &parent_face)?;
                c_parent.push(c as u32);
                ncells += 1;
            }
            continue;
        }
        // Unselected: kept as it is unless one of its quads was cut.
        let faces = src.cell_faces(ci);
        let any_cut = faces.iter().any(|sf| split[sf.face().get()]);
        let added = match src.cell_corners(ci) {
            Some(cs) if !any_cut && shape != CellShape::Polyhedron => b.add_cell_from_vertices(shape, cs)?,
            _ => {
                if any_cut {
                    stats.polyhedra += 1;
                }
                let mut loops: Vec<Vec<VertexIx>> = Vec::new();
                for sf in faces {
                    let mut l: Vec<VertexIx> = src.face_vertices(sf.face()).to_vec();
                    if sf.side() == Side::Right {
                        l.reverse();
                    }
                    if split[sf.face().get()] {
                        let raw: Vec<u32> = l.iter().map(|v| v.0).collect();
                        for t in cut_quad(&raw, &rank) {
                            loops.push(t.iter().map(|&v| VertexIx(v)).collect());
                        }
                    } else {
                        loops.push(l);
                    }
                }
                let refs: Vec<&[VertexIx]> = loops.iter().map(Vec::as_slice).collect();
                b.add_polyhedron(&refs)?
            }
        };
        place(&mut b, added, c, &mut on_parent, &parent_face)?;
        c_parent.push(c as u32);
        ncells += 1;
    }
    child_start.push(ncells);
    stats.children = ncells as usize;
    if let Some(t) = src.edges() {
        for (i, [a, bb]) in t.vertices.iter().enumerate() {
            b.set_class(MeshEntityRef::Edge(*a, *bb), t.class[i])?;
        }
    }
    on_parent.sort_by_key(|(pf, fh)| (*pf, fh.get()));
    let notes = carry_sets_and_tags(
        src,
        &mut b,
        &Lineage {
            nv,
            nn: nv,
            pairs: &[],
            edge_ent: &[],
            qfaces: &[],
            cen_cells: &[],
            base_e: nv,
            base_q: nv,
            base_c: nv,
            child_start: &child_start,
            c_parent: &c_parent,
            on_parent: &on_parent,
        },
    )?;
    let fin = b.finish(alloc)?;
    let mut face_map = vec![NO_CELL; fin.mesh.face_count()];
    for (pf, fh) in &on_parent {
        face_map[fin.face_remap[fh.get()].get()] = *pf;
    }
    let coarse_of = c_parent.iter().map(|&p| CellIx::new(p as usize)).collect();
    Ok(Converted { finished: fin, coarse_of, face_map, stats, notes })
}

/// How `mesh.qtri` splits a quad.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum QtriMode {
    /// Two triangles along the diagonal through the lowest-ranked corner.
    Two,
    /// Four triangles around a new centroid vertex.
    Four,
}

/// Convert the selected quads of a 2D mesh to triangles (`select`: `None` for all quads).
///
/// # Errors
/// [`HtetError`]; `alloc` is unchanged then.
pub fn qtri<B: MeshBuild>(
    src: &dyn MeshRead,
    mode: QtriMode,
    select: Option<&[bool]>,
    root: Option<&[u32]>,
    mut b: B,
    alloc: &mut SerialAlloc,
) -> Result<Converted<B::Output>, HtetError> {
    if src.dim() != MeshDim::Two {
        return Err(HtetError::WrongDimension("mesh.qtri needs a 2D (surface) mesh (use mesh.htet for volumes)"));
    }
    let (nc, nv, nf) = (src.cell_count(), src.vertex_count(), src.face_count());
    if nc == 0 {
        return Err(HtetError::Empty);
    }
    if src.coords().len() != nv {
        return Err(HtetError::HighOrder);
    }
    if let Some(s) = select
        && s.len() != nc
    {
        return Err(HtetError::BadSelection { got: s.len(), want: nc });
    }
    for c in 0..nc {
        if !src.cell_active(CellIx::new(c)) {
            return Err(HtetError::Inactive(c as u32));
        }
        if src.high_order_cell(CellIx::new(c)).is_some() {
            return Err(HtetError::HighOrder);
        }
    }
    for f in 0..nf {
        if src.face_curve(FaceIx::new(f)).is_some() {
            return Err(HtetError::HighOrder);
        }
    }
    if let Some(r) = root
        && r.len() != nv
    {
        return Err(HtetError::BadRoot { got: r.len(), want: nv });
    }
    let rank = |v: u32| (root.map_or(v, |r| r[v as usize]), v);
    let co = src.coords();
    let pt = |v: u32| -> V { [co.x[v as usize], co.y[v as usize], co.z[v as usize]] };
    let selected: Vec<bool> = (0..nc).map(|c| src.cell_shape(CellIx::new(c)) == CellShape::Quad && select.is_none_or(|s| s[c])).collect();
    let centres: Vec<u32> = (0..nc as u32).filter(|&c| selected[c as usize] && mode == QtriMode::Four).collect();
    let mut stats = HtetStats { parents: nc, ..HtetStats::default() };
    b.reserve(nv + centres.len(), nc * 2);
    for v in 0..nv {
        let p = pt(v as u32);
        let nvx = b.add_vertex(Point3::new(p[0], p[1], p[2]))?;
        b.set_class(MeshEntityRef::Vertex(nvx), src.vertex_class(VertexIx::new(v)))?;
    }
    for &c in &centres {
        let cs = src.cell_corners(CellIx::new(c as usize)).ok_or_else(|| HtetError::Inconsistent(format!("cell {c} has no corners")))?;
        let k = 1.0 / cs.len() as f64;
        let m = cs.iter().fold([0.0; 3], |s, v| {
            let p = pt(v.0);
            [s[0] + p[0], s[1] + p[1], s[2] + p[2]]
        });
        let nvx = b.add_vertex(Point3::new(m[0] * k, m[1] * k, m[2] * k))?;
        b.set_class(MeshEntityRef::Vertex(nvx), src.cell_class(CellIx::new(c as usize)))?;
    }
    // Parent edge of every source edge, by sorted vertex pair.
    let mut parent_edge: BTreeMap<[u32; 2], u32> = BTreeMap::new();
    for f in 0..nf {
        let l = src.face_vertices(FaceIx::new(f));
        parent_edge.insert([l[0].0.min(l[1].0), l[0].0.max(l[1].0)], f as u32);
    }
    let mut child_start = Vec::with_capacity(nc + 1);
    let mut c_parent: Vec<u32> = Vec::new();
    let mut on_parent: Vec<(u32, FaceIx)> = Vec::new();
    let mut ncells = 0u32;
    let mut next_centre = nv as u32;
    for c in 0..nc {
        child_start.push(ncells);
        let ci = CellIx::new(c);
        let shape = src.cell_shape(ci);
        let cs: SmallVec<[u32; 8]> = src
            .cell_corners(ci)
            .ok_or_else(|| HtetError::Inconsistent(format!("cell {c} has no corner list")))?
            .iter()
            .map(|v| v.0)
            .collect();
        let mut add = |b: &mut B, shape: CellShape, v: &[u32]| -> Result<(), HtetError> {
            let verts: SmallVec<[VertexIx; 8]> = v.iter().map(|&x| VertexIx(x)).collect();
            let added = if shape == CellShape::Polygon { b.add_polygon(&verts)? } else { b.add_cell_from_vertices(shape, &verts)? };
            b.set_class(MeshEntityRef::Cell(added.cell), src.cell_class(ci))?;
            for (i, sf) in added.faces.iter().enumerate() {
                if added.reused[i] {
                    continue;
                }
                let l = b.face_loop(sf.face());
                let k = [l[0].0.min(l[1].0), l[0].0.max(l[1].0)];
                if let Some(&pf) = parent_edge.get(&k) {
                    let pfi = FaceIx::new(pf as usize);
                    b.set_class(MeshEntityRef::Face(sf.face()), src.face_class(pfi))?;
                    if src.face_kind(pfi) == FaceKind::PartitionCut {
                        b.set_face_kind(sf.face(), FaceKind::PartitionCut)?;
                    }
                    on_parent.push((pf, sf.face()));
                } else {
                    b.set_class(MeshEntityRef::Face(sf.face()), src.cell_class(ci))?;
                }
            }
            c_parent.push(c as u32);
            ncells += 1;
            Ok(())
        };
        if !selected[c] {
            add(&mut b, shape, &cs)?;
            continue;
        }
        stats.cut_cells += 1;
        stats.cut_quads += 1;
        match mode {
            QtriMode::Two => {
                for t in cut_quad(&cs, &rank) {
                    add(&mut b, CellShape::Tri, &t)?;
                }
            }
            QtriMode::Four => {
                let centre = next_centre;
                next_centre += 1;
                for i in 0..4 {
                    add(&mut b, CellShape::Tri, &[cs[i], cs[(i + 1) % 4], centre])?;
                }
            }
        }
    }
    child_start.push(ncells);
    stats.children = ncells as usize;
    on_parent.sort_by_key(|(pf, fh)| (*pf, fh.get()));
    let cen_cells: Vec<u32> = centres.clone();
    let notes = carry_sets_and_tags(
        src,
        &mut b,
        &Lineage {
            nv,
            nn: nv + centres.len(),
            pairs: &[],
            edge_ent: &[],
            qfaces: &[],
            cen_cells: &cen_cells,
            base_e: nv,
            base_q: nv,
            base_c: nv,
            child_start: &child_start,
            c_parent: &c_parent,
            on_parent: &on_parent,
        },
    )?;
    let fin = b.finish(alloc)?;
    let mut face_map = vec![NO_CELL; fin.mesh.face_count()];
    for (pf, fh) in &on_parent {
        face_map[fin.face_remap[fh.get()].get()] = *pf;
    }
    let coarse_of = c_parent.iter().map(|&p| CellIx::new(p as usize)).collect();
    Ok(Converted { finished: fin, coarse_of, face_map, stats, notes })
}
