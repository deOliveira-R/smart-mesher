//! The uniform refinement pass: one mesh in, one finer independent mesh out, with the
//! child-to-parent cell map and the face map of the CMFD hierarchy.
//!
//! Rules: tet 1 to 8 (corner tets plus the octahedron cut along its shortest diagonal), hex 1 to
//! 8, wedge 1 to 8, pyramid 1 to 6 pyramids plus 4 tets, tri 1 to 4, quad 1 to 4. New nodes sit
//! on edge midpoints, quad face centres and hex centres; nodes classified on a model edge or
//! face are projected onto it, unless that would invert an incident cell.
//!
//! The pass is a pure function of the source mesh and the topology view: every choice depends on
//! vertex, edge, face and cell indices only, so the result is bit-identical for any thread count.

use std::collections::BTreeMap;

use rayon::prelude::*;
use sm_core::Point3;
use sm_mesh_api::{
    CellIx, CellShape, Classification, EdgeTable, FaceIx, FaceKind, Finished, MeshBuild, MeshDim, MeshEntityKind,
    MeshEntityRef, MeshError, MeshRead, NO_CELL, Param, SerialAlloc, SetMembers, Side, SignedFace, TagValues, VertexIx,
};
use sm_quality::{Context, Metric, eval};
use sm_topo_api::TopologyView;
use smallvec::SmallVec;

use crate::project::Projector;
use crate::template::{supported, template, tet_diagonals};

const NONE: u32 = u32::MAX;
/// A scaled Jacobian at or below this counts as an inverted or collapsed cell.
const SJ_FLOOR: f64 = 1e-12;

/// Why a mesh cannot be refined.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum RefineError {
    /// A cell shape without a uniform rule.
    #[error(
        "cell {cell} is a {shape:?}: uniform refinement handles tet, hex, wedge, pyramid, tri and quad cells; \
         refine the tet source and dualize again for polyhedra"
    )]
    UnsupportedCell {
        /// The cell index.
        cell: u32,
        /// Its shape.
        shape: CellShape,
    },
    /// High-order cells, extra nodes or face curves.
    #[error("the mesh carries high-order geometry (cell, extra node or face curve); uniform refinement is linear only")]
    HighOrder,
    /// A refined parent retained inactive.
    #[error("cell {0} is inactive (a refined parent); refine the leaf mesh")]
    Inactive(u32),
    /// A non-conformal face (hanging nodes).
    #[error("face {0} is non-conformal; uniform refinement needs a conformal mesh")]
    NonConformal(u32),
    /// The mesh has no cell.
    #[error("the mesh has no cells")]
    Empty,
    /// The edge table could not be derived, so new edge nodes cannot be classified.
    #[error("cannot classify the new edge nodes: {0}")]
    EdgeClass(String),
    /// The result has more than `u32::MAX` nodes or cells.
    #[error("the refined mesh would exceed 2^32 entities")]
    TooLarge,
    /// A mesh-API refusal while building.
    #[error(transparent)]
    Mesh(#[from] MeshError),
}

/// Counters of one pass.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct RefineStats {
    /// Cells of the source.
    pub parents: usize,
    /// Cells of the result.
    pub children: usize,
    /// Vertices of the result.
    pub vertices: usize,
    /// New nodes moved onto their model edge or face.
    pub projected: usize,
    /// New boundary nodes left at the straight position because the projection failed.
    pub projection_failed: usize,
    /// New boundary nodes left at the straight position because the projection inverted a cell.
    pub projection_rejected: usize,
}

/// The result of one pass.
#[non_exhaustive]
pub struct Refined<M> {
    /// The finished fine mesh.
    pub finished: Finished<M>,
    /// The coarse (source) parent of every fine cell.
    pub coarse_of: Vec<CellIx>,
    /// For every fine face the source face that carries it, or [`NO_CELL`].
    pub face_map: Vec<u32>,
    /// Counters.
    pub stats: RefineStats,
    /// Losses and choices, as `(code, message)`.
    pub notes: Vec<(&'static str, String)>,
}

type V = [f64; 3];

fn mean(p: &[V]) -> V {
    let k = 1.0 / p.len() as f64;
    let mut s = [0.0; 3];
    for q in p {
        s = [s[0] + q[0], s[1] + q[1], s[2] + q[2]];
    }
    [s[0] * k, s[1] * k, s[2] * k]
}

fn sorted(mut v: SmallVec<[u32; 8]>) -> SmallVec<[u32; 8]> {
    v.sort_unstable();
    v
}

fn dist2(a: V, b: V) -> f64 {
    (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)
}

/// Refine `src` once. `topo` is the model topology for the projection of new boundary nodes
/// (`None`: straight midpoints). `b` must be an empty builder of the source's dimension.
///
/// # Errors
/// [`RefineError`] for a mesh without a uniform rule (polyhedra, high order, inactive or
/// non-conformal cells) or when the builder refuses the result; `alloc` is unchanged then.
pub fn refine<B: MeshBuild>(
    src: &dyn MeshRead,
    topo: Option<&dyn TopologyView>,
    mut b: B,
    alloc: &mut SerialAlloc,
) -> Result<Refined<B::Output>, RefineError> {
    let dim2 = src.dim() == MeshDim::Two;
    let (nc, nv, nf) = (src.cell_count(), src.vertex_count(), src.face_count());
    if nc == 0 {
        return Err(RefineError::Empty);
    }
    if src.coords().len() != nv {
        return Err(RefineError::HighOrder);
    }
    for c in 0..nc {
        let ci = CellIx::new(c);
        let shape = src.cell_shape(ci);
        if !supported(shape) || src.cell_corners(ci).is_none() {
            return Err(RefineError::UnsupportedCell { cell: c as u32, shape });
        }
        if !src.cell_active(ci) {
            return Err(RefineError::Inactive(c as u32));
        }
        if src.high_order_cell(ci).is_some() {
            return Err(RefineError::HighOrder);
        }
    }
    for f in 0..nf {
        let fi = FaceIx::new(f);
        if src.face_curve(fi).is_some() {
            return Err(RefineError::HighOrder);
        }
        if src.face_kind(fi) == FaceKind::NonConformal {
            return Err(RefineError::NonConformal(f as u32));
        }
    }
    let co = src.coords();
    let pt = |v: u32| -> V { [co.x[v as usize], co.y[v as usize], co.z[v as usize]] };

    // ---- edges: sorted vertex pairs and their classes -------------------------------------------
    let (pairs, eclass): (Vec<[u32; 2]>, Vec<Classification>) = if dim2 {
        let mut v: Vec<([u32; 2], u32)> = (0..nf)
            .map(|f| {
                let l = src.face_vertices(FaceIx::new(f));
                let (a, b) = (l[0].0, l[1].0);
                ([a.min(b), a.max(b)], f as u32)
            })
            .collect();
        v.sort_unstable();
        let class = v.iter().map(|(_, f)| src.face_class(FaceIx::new(*f as usize))).collect();
        (v.into_iter().map(|(k, _)| k).collect(), class)
    } else if let Some(t) = src.edges() {
        (t.vertices.iter().map(|[a, b]| [a.0, b.0]).collect(), t.class.clone())
    } else {
        let t = EdgeTable::derive(src, &mut SerialAlloc::new(), &fallback_edge_classes(src))
            .map_err(|e| RefineError::EdgeClass(e.to_string()))?;
        (t.vertices.iter().map(|[a, b]| [a.0, b.0]).collect(), t.class)
    };
    let edge_id = |a: u32, b: u32| -> u32 {
        let k = [a.min(b), a.max(b)];
        pairs.binary_search(&k).expect("every cell edge is a face edge") as u32
    };
    let ne = pairs.len();

    // ---- quad faces and cell centres ---------------------------------------------------------------
    let mut qfaces: Vec<u32> = Vec::new();
    let mut qrank = vec![NONE; nf];
    if !dim2 {
        for f in 0..nf {
            if src.face_vertices(FaceIx::new(f)).len() == 4 {
                qrank[f] = qfaces.len() as u32;
                qfaces.push(f as u32);
            }
        }
    }
    let mut cen_cells: Vec<u32> = Vec::new();
    let mut crank = vec![NONE; nc];
    for c in 0..nc {
        let s = src.cell_shape(CellIx::new(c));
        if s == CellShape::Hex || (dim2 && s == CellShape::Quad) {
            crank[c] = cen_cells.len() as u32;
            cen_cells.push(c as u32);
        }
    }
    let (base_e, base_q) = (nv, nv + ne);
    let base_c = base_q + qfaces.len();
    let nn = base_c + cen_cells.len();
    if nn >= NONE as usize || nc >= (NONE / 10) as usize {
        return Err(RefineError::TooLarge);
    }

    // ---- node positions and classes -------------------------------------------------------------
    let mut pos: Vec<V> = Vec::with_capacity(nn);
    pos.extend((0..nv as u32).map(pt));
    pos.par_extend(pairs.par_iter().map(|[a, b]| mean(&[pt(*a), pt(*b)])));
    pos.par_extend(qfaces.par_iter().map(|&f| {
        let l = src.face_vertices(FaceIx::new(f as usize));
        let p: SmallVec<[V; 4]> = l.iter().map(|v| pt(v.0)).collect();
        mean(&p)
    }));
    pos.par_extend(cen_cells.par_iter().map(|&c| {
        let p: SmallVec<[V; 8]> = src.cell_corners(CellIx::new(c as usize)).unwrap_or(&[]).iter().map(|v| pt(v.0)).collect();
        mean(&p)
    }));
    let mut class: Vec<Classification> = Vec::with_capacity(nn);
    class.extend((0..nv).map(|v| src.vertex_class(VertexIx::new(v))));
    class.extend(eclass.iter().copied());
    class.extend(qfaces.iter().map(|&f| src.face_class(FaceIx::new(f as usize))));
    class.extend(cen_cells.iter().map(|&c| src.cell_class(CellIx::new(c as usize))));
    // Edge and face nodes start without a parameter.
    for c in class.iter_mut().skip(nv) {
        c.param = Param::None;
    }

    // ---- children --------------------------------------------------------------------------------
    let diag = tet_diagonals();
    let mut c_shape: Vec<CellShape> = Vec::with_capacity(nc * 8);
    let mut c_off: Vec<u32> = vec![0];
    let mut c_nodes: Vec<u32> = Vec::with_capacity(nc * 40);
    let mut c_parent: Vec<u32> = Vec::with_capacity(nc * 8);
    let mut c_pf: Vec<u32> = Vec::with_capacity(nc * 40);
    let mut child_start: Vec<u32> = Vec::with_capacity(nc + 1);
    let mut gid: SmallVec<[u32; 27]> = SmallVec::new();
    for c in 0..nc {
        child_start.push(c_shape.len() as u32);
        let ci = CellIx::new(c);
        let shape = src.cell_shape(ci);
        let t = template(shape);
        let corners: SmallVec<[u32; 8]> = src.cell_corners(ci).unwrap_or(&[]).iter().map(|v| v.0).collect();
        // Source face of every local face of the parent, by vertex set.
        let cf = src.cell_faces(ci);
        let lf2f: SmallVec<[u32; 6]> = shape
            .local_faces()
            .iter()
            .map(|lf| {
                let want = sorted(lf.iter().map(|&i| corners[i as usize]).collect());
                cf.iter()
                    .map(|sf| sf.face())
                    .find(|&f| {
                        let l = src.face_vertices(f);
                        l.len() == want.len() && sorted(l.iter().map(|v| v.0).collect()) == want
                    })
                    .map_or(NONE, |f| f.get() as u32)
            })
            .collect();
        gid.clear();
        for set in &t.nodes {
            let id = match set.len() {
                1 => corners[set[0] as usize],
                2 => base_e as u32 + edge_id(corners[set[0] as usize], corners[set[1] as usize]),
                n if n == 8 || (dim2 && n == 4) => (base_c + crank[c] as usize) as u32,
                _ => {
                    let j = shape
                        .local_faces()
                        .iter()
                        .position(|lf| lf.len() == set.len() && set.iter().all(|s| lf.contains(s)))
                        .ok_or(RefineError::UnsupportedCell { cell: c as u32, shape })?;
                    (base_q + qrank[lf2f[j] as usize] as usize) as u32
                }
            };
            gid.push(id);
        }
        let variant = if shape == CellShape::Tet {
            // Octahedron diagonal: the shortest one (ties to the first), independent of vertex order.
            let len = |d: usize| dist2(pos[gid[diag[d].0 as usize] as usize], pos[gid[diag[d].1 as usize] as usize]);
            let l = [len(0), len(1), len(2)];
            let m = l[0].min(l[1]).min(l[2]);
            l.iter().position(|&x| x <= m * (1.0 + 1e-12)).unwrap_or(0)
        } else {
            0
        };
        for ch in &t.variants[variant] {
            let mut nodes: SmallVec<[u8; 8]> = ch.nodes.iter().copied().collect();
            if ch.shape == CellShape::Tet {
                let p: SmallVec<[V; 4]> = nodes.iter().map(|&n| pos[gid[n as usize] as usize]).collect();
                let (a, bb, cc) = (sub(p[1], p[0]), sub(p[2], p[0]), sub(p[3], p[0]));
                let det = a[0] * (bb[1] * cc[2] - bb[2] * cc[1]) - a[1] * (bb[0] * cc[2] - bb[2] * cc[0])
                    + a[2] * (bb[0] * cc[1] - bb[1] * cc[0]);
                if det < 0.0 {
                    nodes.swap(2, 3);
                }
            }
            c_shape.push(ch.shape);
            c_parent.push(c as u32);
            c_nodes.extend(nodes.iter().map(|&n| gid[n as usize]));
            c_off.push(c_nodes.len() as u32);
            for lf in ch.shape.local_faces() {
                let m = lf.iter().fold(0xFFu8, |a, &i| a & t.nmask[nodes[i as usize] as usize]);
                c_pf.push(if m == 0 { NONE } else { lf2f[m.trailing_zeros() as usize] });
            }
        }
    }
    child_start.push(c_shape.len() as u32);
    let ncl = c_shape.len();
    let child_nodes = |k: usize| &c_nodes[c_off[k] as usize..c_off[k + 1] as usize];

    // ---- projection of new boundary nodes -----------------------------------------------------------
    let mut stats = RefineStats { parents: nc, children: ncl, vertices: nn, ..RefineStats::default() };
    if let Some(topo) = topo {
        let pj = Projector::new(topo);
        let pnormal: Vec<Option<V>> = (0..nc)
            .map(|c| src.cell_normal(CellIx::new(c)).map(|n| [n.x, n.y, n.z]))
            .collect();
        // Incident children of every new node.
        let mut off = vec![0u32; nn - nv + 1];
        for k in 0..ncl {
            for &n in child_nodes(k) {
                if n as usize >= nv {
                    off[n as usize - nv + 1] += 1;
                }
            }
        }
        for i in 0..nn - nv {
            off[i + 1] += off[i];
        }
        let mut cur = off.clone();
        let mut inc = vec![0u32; off[nn - nv] as usize];
        for k in 0..ncl {
            for &n in child_nodes(k) {
                if n as usize >= nv {
                    let s = &mut cur[n as usize - nv];
                    inc[*s as usize] = k as u32;
                    *s += 1;
                }
            }
        }
        let min_sj = |n: usize, pos: &[V]| -> f64 {
            let mut m = f64::INFINITY;
            for &k in &inc[off[n - nv] as usize..off[n - nv + 1] as usize] {
                let k = k as usize;
                let p: SmallVec<[V; 8]> = child_nodes(k).iter().map(|&v| pos[v as usize]).collect();
                let ctx = Context { normal: pnormal[c_parent[k] as usize], ref_size: f64::NAN };
                let v = eval(Metric::ScaledJacobian, c_shape[k], &p, &ctx).unwrap_or(f64::NAN);
                m = m.min(v);
                if v.is_nan() {
                    return f64::NAN;
                }
            }
            m
        };
        for n in nv..nn {
            if !pj.applies(class[n]) {
                continue;
            }
            let Some(p) = pj.project(class[n], pos[n]) else {
                stats.projection_failed += 1;
                continue;
            };
            let old = pos[n];
            let scale = 1.0 + old[0].abs().max(old[1].abs()).max(old[2].abs());
            // A projection that moves the node by rounding only keeps the straight bits.
            let new = if dist2(old, p.point) <= (1e-13 * scale).powi(2) { old } else { p.point };
            let before = min_sj(n, &pos);
            pos[n] = new;
            let after = min_sj(n, &pos);
            let bad = after.is_nan() || after <= SJ_FLOOR;
            if bad && (before > SJ_FLOOR || after < before) {
                pos[n] = old;
                stats.projection_rejected += 1;
                continue;
            }
            class[n].param = p.param;
            if new != old {
                stats.projected += 1;
            }
        }
    }

    // ---- build -----------------------------------------------------------------------------------
    b.reserve(nn, ncl);
    for n in 0..nn {
        let v = b.add_vertex(Point3::new(pos[n][0], pos[n][1], pos[n][2]))?;
        b.set_class(MeshEntityRef::Vertex(v), class[n])?;
    }
    let mut on_parent: Vec<(u32, FaceIx)> = Vec::new();
    let mut fcursor = 0usize;
    let mut verts: SmallVec<[VertexIx; 8]> = SmallVec::new();
    for k in 0..ncl {
        verts.clear();
        verts.extend(child_nodes(k).iter().map(|&n| VertexIx(n)));
        let parent = CellIx::new(c_parent[k] as usize);
        let added = b.add_cell_from_vertices(c_shape[k], &verts)?;
        b.set_class(MeshEntityRef::Cell(added.cell), src.cell_class(parent))?;
        for (i, sf) in added.faces.iter().enumerate() {
            let pf = c_pf[fcursor + i];
            if pf != NONE {
                let pfi = FaceIx::new(pf as usize);
                if !added.reused[i] {
                    b.set_class(MeshEntityRef::Face(sf.face()), src.face_class(pfi))?;
                    if src.face_kind(pfi) == FaceKind::PartitionCut {
                        b.set_face_kind(sf.face(), FaceKind::PartitionCut)?;
                    }
                    on_parent.push((pf, sf.face()));
                }
            } else if !added.reused[i] {
                b.set_class(MeshEntityRef::Face(sf.face()), src.cell_class(parent))?;
            }
        }
        fcursor += added.faces.len();
    }
    if !dim2 && src.edges().is_some() {
        for (e, [a, bb]) in pairs.iter().enumerate() {
            let mid = VertexIx((base_e + e) as u32);
            b.set_class(MeshEntityRef::Edge(VertexIx(*a), mid), eclass[e])?;
            b.set_class(MeshEntityRef::Edge(mid, VertexIx(*bb)), eclass[e])?;
        }
    }

    // ---- sets and tags -----------------------------------------------------------------------------
    let mut notes: Vec<(&'static str, String)> = Vec::new();
    on_parent.sort_by_key(|(pf, fh)| (*pf, fh.get()));
    let children_of = |c: u32| child_start[c as usize]..child_start[c as usize + 1];
    for set in src.sets() {
        let members = match &set.members {
            SetMembers::Cells(v) => SetMembers::Cells(
                v.iter().flat_map(|c| children_of(c.0).map(|k| CellIx::new(k as usize))).collect(),
            ),
            SetMembers::Faces(v) => {
                let mut out: Vec<FaceIx> = Vec::new();
                for f in v {
                    let lo = on_parent.partition_point(|(pf, _)| *pf < f.0);
                    out.extend(on_parent[lo..].iter().take_while(|(pf, _)| *pf == f.0).map(|(_, fh)| *fh));
                }
                out.sort_unstable();
                out.dedup();
                SetMembers::Faces(out)
            }
            SetMembers::Sides(v) => {
                let mut out: Vec<SignedFace> = Vec::new();
                for sf in v {
                    let f = sf.face();
                    let parent = match sf.side() {
                        Side::Left => Some(src.face_left(f)),
                        Side::Right => src.face_right(f),
                    };
                    let Some(parent) = parent else { continue };
                    let lo = on_parent.partition_point(|(pf, _)| *pf < f.0);
                    for (_, fh) in on_parent[lo..].iter().take_while(|(pf, _)| *pf == f.0) {
                        for side in [Side::Left, Side::Right] {
                            if b.face_cell(*fh, side).is_some_and(|cc| c_parent[cc.get()] == parent.get() as u32) {
                                out.push(SignedFace::new(*fh, side));
                            }
                        }
                    }
                }
                out.sort_unstable_by_key(|s| s.to_raw());
                out.dedup();
                SetMembers::Sides(out)
            }
            SetMembers::Vertices(v) => {
                let mut inset = vec![false; nv];
                for x in v {
                    inset[x.get()] = true;
                }
                let mut out: Vec<VertexIx> = v.iter().map(|x| VertexIx(x.0)).collect();
                out.extend(pairs.iter().enumerate().filter(|(_, [a, bb])| inset[*a as usize] && inset[*bb as usize]).map(|(e, _)| VertexIx((base_e + e) as u32)));
                out.extend(qfaces.iter().enumerate().filter(|(_, f)| src.face_vertices(FaceIx::new(**f as usize)).iter().all(|x| inset[x.get()])).map(|(r, _)| VertexIx((base_q + r) as u32)));
                out.extend(cen_cells.iter().enumerate().filter(|(_, c)| src.cell_corners(CellIx::new(**c as usize)).unwrap_or(&[]).iter().all(|x| inset[x.get()])).map(|(r, _)| VertexIx((base_c + r) as u32)));
                SetMembers::Vertices(out)
            }
            _ => {
                notes.push(("refine.set_dropped", format!("set `{}` (shell sides or points) is not carried over", set.name)));
                continue;
            }
        };
        b.add_set(&set.name, members)?;
        if let Some(l) = set.label {
            b.set_label(&set.name, Some(l))?;
        }
    }
    for t in src.tags() {
        match (t.target, &t.values) {
            (MeshEntityKind::Cell, v) => {
                let vals = match v {
                    TagValues::I64(x) => TagValues::I64(c_parent.iter().map(|&p| x[p as usize]).collect()),
                    TagValues::F64(x) => TagValues::F64(c_parent.iter().map(|&p| x[p as usize]).collect()),
                    _ => {
                        notes.push(("refine.tag_dropped", format!("tag `{}` has an unsupported value type", t.name)));
                        continue;
                    }
                };
                b.add_tag(&t.name, MeshEntityKind::Cell, vals)?;
            }
            (MeshEntityKind::Vertex, v) => {
                // Spanned vertices of every new node, in node order.
                let span = |n: usize| -> SmallVec<[u32; 8]> {
                    if n < nv {
                        SmallVec::from_slice(&[n as u32])
                    } else if n < base_q {
                        SmallVec::from_slice(&pairs[n - base_e])
                    } else if n < base_c {
                        src.face_vertices(FaceIx::new(qfaces[n - base_q] as usize)).iter().map(|x| x.0).collect()
                    } else {
                        src.cell_corners(CellIx::new(cen_cells[n - base_c] as usize)).unwrap_or(&[]).iter().map(|x| x.0).collect()
                    }
                };
                let vals = match v {
                    TagValues::I64(x) => TagValues::I64((0..nn).map(|n| x[span(n).iter().copied().min().unwrap_or(0) as usize]).collect()),
                    TagValues::F64(x) => TagValues::F64(
                        (0..nn)
                            .map(|n| {
                                let s = span(n);
                                s.iter().map(|&i| x[i as usize]).sum::<f64>() / s.len() as f64
                            })
                            .collect(),
                    ),
                    _ => {
                        notes.push(("refine.tag_dropped", format!("tag `{}` has an unsupported value type", t.name)));
                        continue;
                    }
                };
                b.add_tag(&t.name, MeshEntityKind::Vertex, vals)?;
            }
            _ => notes.push(("refine.tag_dropped", format!("tag `{}` on {:?} is not carried over", t.name, t.target))),
        }
    }

    let fin = b.finish(alloc)?;
    let mut face_map = vec![NO_CELL; fin.mesh.face_count()];
    for (pf, fh) in &on_parent {
        face_map[fin.face_remap[fh.get()].get()] = *pf;
    }
    let coarse_of = c_parent.iter().map(|&p| CellIx::new(p as usize)).collect();
    Ok(Refined { finished: fin, coarse_of, face_map, stats, notes })
}

/// Classes for the edges of a mesh without an edge table that the face rule of
/// [`EdgeTable::derive`] cannot decide: an edge touching two or more distinct boundary faces lies
/// on a model edge, taken from its end vertices (one on a `TopoEdge`, the other on the same
/// edge or on a `TopoVertex`).
fn fallback_edge_classes(src: &dyn MeshRead) -> BTreeMap<[VertexIx; 2], Classification> {
    let mut seen: BTreeMap<[VertexIx; 2], (sm_core::EntityId, bool)> = BTreeMap::new();
    for f in 0..src.face_count() {
        let fi = FaceIx::new(f);
        if src.face_kind(fi) == FaceKind::Interior {
            continue;
        }
        let ent = src.face_class(fi).entity;
        let l = src.face_vertices(fi);
        for i in 0..l.len() {
            let (a, bb) = (l[i], l[(i + 1) % l.len()]);
            let k = if a < bb { [a, bb] } else { [bb, a] };
            let e = seen.entry(k).or_insert((ent, false));
            e.1 |= e.0 != ent;
        }
    }
    let on_edge = |e: sm_core::EntityId| e.kind() == sm_core::EntityKind::TopoEdge;
    let on_vertex = |e: sm_core::EntityId| e.kind() == sm_core::EntityKind::TopoVertex;
    let mut out = BTreeMap::new();
    for (k, (_, multi)) in seen {
        if !multi {
            continue;
        }
        let (ea, eb) = (src.vertex_class(k[0]).entity, src.vertex_class(k[1]).entity);
        if on_edge(ea) && (ea == eb || on_vertex(eb)) {
            out.insert(k, Classification::on(ea));
        } else if on_edge(eb) && on_vertex(ea) {
            out.insert(k, Classification::on(eb));
        }
    }
    out
}

fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
