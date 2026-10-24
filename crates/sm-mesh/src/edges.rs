//! Parallel edge-table derivation, identical in result (including errors) to
//! `sm_mesh_api::EdgeTable::derive`.
//!
//! Every face-edge incidence becomes a record `(vertex pair, face, loop position)`; a parallel
//! sort groups the records by edge, so edges come out in lexicographic vertex-pair order.
//! Classification of each edge looks only at its own group, in parallel.

use crate::store::Mesh;
use crate::util::{CHUNK, ErrSink, nil};
use rayon::prelude::*;
use sm_core::{EntityId, EntityKind};
use sm_mesh_api::{
    CellIx, Classification, EdgeIx, EdgeTable, FaceEdge, FaceKind, MeshDim, MeshError, Result,
    SerialAlloc, VertexIx, entity_dim,
};
use smallvec::SmallVec;
use std::collections::BTreeMap;

/// One face-edge incidence. `f_fwd` is `face << 1 | forward` (the loop runs low -> high vertex).
#[derive(Clone, Copy)]
struct Inc {
    key: u64,
    f_fwd: u32,
    i: u32,
}

/// Derive the edge table of `m`; `alloc` is unchanged on `Err`.
pub(crate) fn derive(
    m: &Mesh,
    alloc: &mut SerialAlloc,
    explicit: &BTreeMap<[VertexIx; 2], Classification>,
) -> Result<EdgeTable> {
    if m.dim == MeshDim::Two {
        return Err(MeshError::Unsupported(
            "a dimension-2 mesh has no edge table: its facets are the edges".into(),
        ));
    }
    let nf = m.n_faces();
    let total = m.face_verts.len();
    if total >= (1 << 31) {
        return Err(MeshError::TooLarge);
    }
    let mut incs: Vec<Inc> = (0..nf)
        .into_par_iter()
        .flat_map_iter(|f| {
            let l = &m.face_verts[crate::util::range(&m.face_off, f)];
            (0..l.len()).map(move |i| {
                let (a, b) = (l[i].0, l[(i + 1) % l.len()].0);
                Inc {
                    key: (u64::from(a.min(b)) << 32) | u64::from(a.max(b)),
                    f_fwd: ((f as u32) << 1) | u32::from(a < b),
                    i: i as u32,
                }
            })
        })
        .collect();
    incs.par_sort_unstable_by(|a, b| (a.key, a.f_fwd, a.i).cmp(&(b.key, b.f_fwd, b.i)));
    let starts: Vec<usize> = (0..incs.len())
        .into_par_iter()
        .filter(|&i| i == 0 || incs[i].key != incs[i - 1].key)
        .collect();
    let ne = starts.len();
    let ekeys: Vec<u64> = starts.par_iter().map(|&s| incs[s].key).collect();
    let pair = |k: u64| [VertexIx((k >> 32) as u32), VertexIx(k as u32)];
    for k in explicit.keys() {
        let key = (u64::from(k[0].0) << 32) | u64::from(k[1].0);
        if ekeys.binary_search(&key).is_err() {
            return Err(MeshError::State(format!(
                "edge class given for {k:?}, which is not an edge of the mesh"
            )));
        }
    }
    if ne > u32::MAX as usize {
        return Err(MeshError::TooLarge);
    }

    let end = |e: usize| if e + 1 < ne { starts[e + 1] } else { incs.len() };
    let mut class = vec![Classification::on(nil()); ne];
    let errs = ErrSink::default();
    class.par_chunks_mut(CHUNK).enumerate().for_each(|(ci, out)| {
        for (k, slot) in out.iter_mut().enumerate() {
            let e = ci * CHUNK + k;
            let group = &incs[starts[e]..end(e)];
            let mut all_interior = true;
            // Lowest-dimension model entity among the incident faces, and that entity if unique.
            let mut low: Option<(u8, Option<EntityId>)> = None;
            let mut around: SmallVec<[u32; 16]> = SmallVec::new();
            for r in group {
                let f = (r.f_fwd >> 1) as usize;
                all_interior &= m.face_kind[f] == FaceKind::Interior;
                around.push(m.face_left[f].0);
                if m.face_right[f] != CellIx::NONE {
                    around.push(m.face_right[f]);
                }
                let fc = m.fclass.ent[f];
                let fd = entity_dim(fc.kind()).unwrap_or(u8::MAX);
                low = match low {
                    None => Some((fd, Some(fc))),
                    Some((d, _)) if fd < d => Some((fd, Some(fc))),
                    Some((d, Some(x))) if fd == d && x != fc => Some((d, None)),
                    keep => keep,
                };
            }
            around.sort_unstable();
            around.dedup();
            let ex = if explicit.is_empty() { None } else { explicit.get(&pair(ekeys[e])) };
            *slot = if let Some(c) = ex {
                *c
            } else if let Some((_, Some(ent))) = low {
                Classification::on(ent)
            } else {
                let mut it = around.iter().map(|&c| m.cclass.ent[c as usize]);
                match it.next() {
                    Some(c0) if all_interior && it.all(|c| c == c0) => Classification::on(c0),
                    _ => {
                        let [a, b] = pair(ekeys[e]);
                        errs.push(e, MeshError::UnclassifiedEdge(a, b));
                        return;
                    }
                }
            };
        }
    });
    if let Some(e) = errs.take_first() {
        return Err(e);
    }

    // Everything that can fail has passed: draw the ids (`take` checks the range first).
    let first = alloc.take(EntityKind::MeshEdge, ne)?;
    let ids = (0..ne as u64)
        .map(|i| EntityId::new(EntityKind::MeshEdge, first + i).ok_or(MeshError::SerialOverflow))
        .collect::<Result<Vec<_>>>()?;
    let vertices: Vec<[VertexIx; 2]> = ekeys.par_iter().map(|&k| pair(k)).collect();
    let mut face_edges = vec![FaceEdge { edge: EdgeIx(0), forward: false }; total];
    for e in 0..ne {
        for r in &incs[starts[e]..end(e)] {
            let f = (r.f_fwd >> 1) as usize;
            face_edges[m.face_off[f] as usize + r.i as usize] =
                FaceEdge { edge: EdgeIx(e as u32), forward: r.f_fwd & 1 == 1 };
        }
    }
    Ok(EdgeTable { ids, vertices, face_off: m.face_off.clone(), face_edges, class })
}
