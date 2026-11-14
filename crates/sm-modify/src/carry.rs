//! Carry-over of sets and tag columns through a refinement, from the child-to-parent cell map
//! and the child-face-to-parent-face pairs. Written against [`Lineage`] only, so the other
//! `sm-modify` passes (conversion, local refinement) reuse it with their own maps.

use sm_core::EntityId;
use std::collections::{BTreeMap, BTreeSet};
use sm_mesh_api::{
    CellIx, FaceIx, MeshBuild, MeshDim, MeshEntityKind, MeshError, MeshRead, SetMembers, Side, SignedFace, TagValues, VertexIx,
};
use smallvec::SmallVec;

/// How the entities of a new mesh descend from those of its source.
#[derive(Clone, Copy)]
pub(crate) struct Lineage<'a> {
    /// Vertices of the source.
    pub nv: usize,
    /// Vertices of the new mesh (original vertices first, then edge, quad-face and cell nodes).
    pub nn: usize,
    /// Source edges as sorted vertex pairs; node `base_e + i` is the midpoint of edge `i`.
    pub pairs: &'a [[u32; 2]],
    /// Whether source edge `i` lies on a model edge.
    pub edge_on_curve: &'a [bool],
    /// Source faces with a centre node (`base_q + rank`), ascending.
    pub qfaces: &'a [u32],
    /// Source cells with a centre node (`base_c + rank`), ascending.
    pub cen_cells: &'a [u32],
    /// First node of the edge nodes.
    pub base_e: usize,
    /// First node of the quad-face centre nodes.
    pub base_q: usize,
    /// First node of the cell centre nodes.
    pub base_c: usize,
    /// First child cell of every source cell, plus the total (cells are added in parent order).
    pub child_start: &'a [u32],
    /// The source parent of every new cell.
    pub c_parent: &'a [u32],
    /// `(source face, new provisional face)` of every new face lying on a source face, ascending.
    pub on_parent: &'a [(u32, FaceIx)],
}

/// Per-mesh wall data for vertex-set coverage, built once on the first non-empty vertex set.
struct WallCover {
    /// Dimension of the model entities that carry walls (face in 3D, edge in 2D).
    wall_dim: u8,
    /// `(mesh face, model entity)` of every wall face.
    wall_faces: Vec<(u32, EntityId)>,
    /// Model entities of the wall dimension with a mesh vertex in their interior.
    has_interior: BTreeSet<EntityId>,
    /// CSR of the vertices of every face.
    off: Vec<u32>,
    vs: Vec<u32>,
}

impl WallCover {
    fn new(src: &dyn MeshRead, nv: usize) -> Self {
        let wall_dim = if matches!(src.dim(), MeshDim::Two) { 1 } else { 2 };
        let mut wall_faces = Vec::new();
        let (mut off, mut vs) = (vec![0u32], Vec::new());
        for f in 0..src.face_count() {
            let fi = FaceIx::new(f);
            let c = src.face_class(fi);
            if c.dim() == Some(wall_dim) {
                wall_faces.push((f as u32, c.entity));
            }
            vs.extend(src.face_vertices(fi).iter().map(|x| x.0));
            off.push(vs.len() as u32);
        }
        let mut has_interior = BTreeSet::new();
        for v in 0..nv {
            let c = src.vertex_class(VertexIx::new(v));
            if c.dim() == Some(wall_dim) {
                has_interior.insert(c.entity);
            }
        }
        WallCover { wall_dim, wall_faces, has_interior, off, vs }
    }

    fn verts(&self, f: usize) -> &[u32] {
        &self.vs[self.off[f] as usize..self.off[f + 1] as usize]
    }
}

/// Add the sets and cell / vertex tags of `src` to the builder `b` of the new mesh.
///
/// Cell sets become the children; face sets the new faces on the member faces; side sets the new
/// sides whose cell descends from the member side; vertex sets the original vertices plus the new
/// nodes lying on a covered entity: a model face (model edge in 2D) all of whose mesh vertices are
/// members covers its mesh faces (so does a single wall face with member vertices, one of them on
/// that model face, which carries box-picked patches; on a model face without any interior mesh
/// vertex, such as a wall one interval thick, every wall face with member vertices covers, so a
/// corner triangle there is indistinguishable from a patch and also covers: reported as
/// `refine.set_ambiguous`), and the midpoints of their edges and their centres join; the
/// midpoint of a model edge with both ends members joins too (on a one-interval model edge two
/// corner nodes thus gain its midpoint, equally ambiguous). Interior faces and cell centres
/// never join. Without classification, boundary faces whose vertices are all members cover
/// (this cannot tell walls apart). Cell tags are inherited; vertex tags take
/// the mean (real) or the lowest spanned vertex's value (integer). Returns `(code, message)` notes
/// for what is dropped (face and edge tags, shell-side and point sets).
pub(crate) fn carry_sets_and_tags<B: MeshBuild>(
    src: &dyn MeshRead,
    b: &mut B,
    l: &Lineage<'_>,
) -> Result<Vec<(&'static str, String)>, MeshError> {
    let mut notes: Vec<(&'static str, String)> = Vec::new();
    let Lineage { nv, nn, pairs, edge_on_curve, qfaces, cen_cells, base_e, base_q, base_c, child_start, c_parent, on_parent } = *l;
    let mut wall: Option<WallCover> = None;
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
                if v.is_empty() {
                    b.add_set(&set.name, SetMembers::Vertices(Vec::new()))?;
                    if let Some(l) = set.label {
                        b.set_label(&set.name, Some(l))?;
                    }
                    continue;
                }
                let wc = wall.get_or_insert_with(|| WallCover::new(src, nv));
                let mut inset = vec![false; nv];
                for x in v {
                    inset[x.get()] = true;
                }
                let mut out: Vec<VertexIx> = v.iter().map(|x| VertexIx(x.0)).collect();
                // A face "covers" when it lies on a model face (a model edge in 2D) all of whose
                // mesh faces have only member vertices, so the whole entity is in the set. A mesh
                // without classification falls back to boundary faces whose vertices are all
                // members; it cannot tell walls apart. Interior faces never cover.
                let all_in = |f: usize| wc.verts(f).iter().all(|&x| inset[x as usize]);
                let mut ent_cov: BTreeMap<EntityId, bool> = BTreeMap::new();
                for &(f, ent) in &wc.wall_faces {
                    *ent_cov.entry(ent).or_insert(true) &= all_in(f as usize);
                }
                // Covered when the whole model face is, or locally when all its vertices are
                // members and one of them lies on this model face (a patch); corner faces with
                // every vertex on model edges or vertices do not cover, unless the model face has
                // no interior vertex at all (a wall one interval thick), where every wall face is
                // such a face and all of them cover (the ambiguous rule, reported once per face).
                let mut ambiguous: BTreeSet<EntityId> = BTreeSet::new();
                let mut face_cov = vec![false; src.face_count()];
                for (f, cov) in face_cov.iter_mut().enumerate() {
                    let fi = FaceIx::new(f);
                    let c = src.face_class(fi);
                    *cov = if c.dim() == Some(wc.wall_dim) {
                        if ent_cov.get(&c.entity).copied().unwrap_or(false) {
                            true
                        } else if all_in(f) {
                            if wc.verts(f).iter().any(|&x| src.vertex_class(VertexIx(x)).entity == c.entity) {
                                true
                            } else if !wc.has_interior.contains(&c.entity) {
                                ambiguous.insert(c.entity);
                                true
                            } else {
                                false
                            }
                        } else {
                            false
                        }
                    } else {
                        c.dim().is_none() && src.face_right(fi).is_none() && all_in(f)
                    };
                }
                for ent in ambiguous {
                    notes.push((
                        "refine.set_ambiguous",
                        format!(
                            "set `{}`: model face {ent} has no interior mesh vertex (one interval thick), so its member faces covered \
                             on the ambiguous rule: face-centre and edge-midpoint nodes joined, which a corner patch would not have given",
                            set.name
                        ),
                    ));
                }
                // An edge joins if it is an edge of a covering face (in 2D a face is the edge
                // itself), or lies on a model edge and both its ends are members.
                let mut edge_in = vec![false; pairs.len()];
                for (f, _) in face_cov.iter().enumerate().filter(|(_, c)| **c) {
                    let l = src.face_vertices(FaceIx::new(f));
                    let n = if l.len() == 2 { 1 } else { l.len() };
                    for i in 0..n {
                        let (a, c) = (l[i].0, l[(i + 1) % l.len()].0);
                        if let Ok(e) = pairs.binary_search(&[a.min(c), a.max(c)]) {
                            edge_in[e] = true;
                        }
                    }
                }
                out.extend(
                    pairs
                        .iter()
                        .enumerate()
                        .filter(|(e, [a, bb])| edge_in[*e] || (edge_on_curve[*e] && inset[*a as usize] && inset[*bb as usize]))
                        .map(|(e, _)| VertexIx((base_e + e) as u32)),
                );
                out.extend(qfaces.iter().enumerate().filter(|(_, f)| face_cov[**f as usize]).map(|(r, _)| VertexIx((base_q + r) as u32)));
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
    Ok(notes)
}
