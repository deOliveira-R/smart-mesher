//! Carry-over of sets and tag columns through a refinement, from the child-to-parent cell map
//! and the child-face-to-parent-face pairs. Written against [`Lineage`] only, so the other
//! `sm-modify` passes (conversion, local refinement) reuse it with their own maps.

use sm_mesh_api::{
    CellIx, FaceIx, MeshBuild, MeshEntityKind, MeshError, MeshRead, SetMembers, Side, SignedFace, TagValues, VertexIx,
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

/// Add the sets and cell / vertex tags of `src` to the builder `b` of the new mesh.
///
/// Cell sets become the children; face sets the new faces on the member faces; side sets the new
/// sides whose cell descends from the member side; vertex sets the original vertices plus every
/// new node all of whose spanned vertices are members. Cell tags are inherited; vertex tags take
/// the mean (real) or the lowest spanned vertex's value (integer). Returns `(code, message)` notes
/// for what is dropped (face and edge tags, shell-side and point sets).
pub(crate) fn carry_sets_and_tags<B: MeshBuild>(
    src: &dyn MeshRead,
    b: &mut B,
    l: &Lineage<'_>,
) -> Result<Vec<(&'static str, String)>, MeshError> {
    let mut notes: Vec<(&'static str, String)> = Vec::new();
    let Lineage { nv, nn, pairs, qfaces, cen_cells, base_e, base_q, base_c, child_start, c_parent, on_parent } = *l;
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
    Ok(notes)
}
