//! Index-range precheck: the metric drivers index flat arrays with mesh indices and would panic on
//! a structurally corrupt mesh. This O(n) pass turns that into a typed error.

use rayon::prelude::*;
use sm_mesh_api::{CellIx, FaceIx, MeshRead};

use crate::error::{QualityError, Result};
use crate::stats::CHUNK;

fn corrupt(what: String) -> QualityError {
    QualityError::CorruptMesh(format!("{what}; the mesh is structurally invalid: run mesh.validate to locate the violations"))
}

/// Check that every index stored in the topology is in range: face cells below the cell count,
/// face vertices and cell corners below the vertex count, cell faces below the face count.
///
/// Reports the lowest offending face, else the lowest offending cell, so the answer does not
/// depend on scheduling. Called by [`crate::fv_geometry`] and [`crate::validity`]; `sm-diag`
/// calls it before any other metric.
pub fn check_indices(mesh: &dyn MeshRead) -> Result<()> {
    let (nv, nf, nc) = (mesh.vertex_count(), mesh.face_count(), mesh.cell_count());
    let faces: Vec<Option<String>> = (0..nf.div_ceil(CHUNK))
        .into_par_iter()
        .map(|ci| {
            (ci * CHUNK..((ci + 1) * CHUNK).min(nf)).find_map(|f| {
                let fi = FaceIx::new(f);
                if mesh.face_left(fi).get() >= nc {
                    return Some(format!("face {f} has left cell {} but the mesh has {nc} cells", mesh.face_left(fi).get()));
                }
                if let Some(r) = mesh.face_right(fi).filter(|r| r.get() >= nc) {
                    return Some(format!("face {f} has right cell {} but the mesh has {nc} cells", r.get()));
                }
                mesh.face_vertices(fi).iter().find(|v| v.get() >= nv).map(|v| format!("face {f} has vertex {} but the mesh has {nv} vertices", v.get()))
            })
        })
        .collect();
    if let Some(msg) = faces.into_iter().flatten().next() {
        return Err(corrupt(msg));
    }
    let cells: Vec<Option<String>> = (0..nc.div_ceil(CHUNK))
        .into_par_iter()
        .map(|ci| {
            (ci * CHUNK..((ci + 1) * CHUNK).min(nc)).find_map(|c| {
                let ce = CellIx::new(c);
                if let Some(sf) = mesh.cell_faces(ce).iter().find(|sf| sf.face().get() >= nf) {
                    return Some(format!("cell {c} has face {} but the mesh has {nf} faces", sf.face().get()));
                }
                mesh.cell_corners(ce)?.iter().find(|v| v.get() >= nv).map(|v| format!("cell {c} has corner {} but the mesh has {nv} vertices", v.get()))
            })
        })
        .collect();
    match cells.into_iter().flatten().next() {
        Some(msg) => Err(corrupt(msg)),
        None => Ok(()),
    }
}
