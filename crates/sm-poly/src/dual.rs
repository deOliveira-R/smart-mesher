//! The barycentric dual: the entry point [`dualize`].

use crate::agglom;
use crate::concave::{self, Split};
use crate::error::PolyError;
use crate::options::{DualReport, PolyOptions};
use crate::polydata;
use crate::prep::Prep;
use crate::resolver::EdgeResolver;
use crate::stats;
use sm_mesh_api::{Finished, MeshBuild, MeshDim, MeshError, MeshRead, SerialAlloc};

/// Rounds of the concave-cell repair. Each round splits every cell that still fails; split
/// wedges are valid by construction, so only neighbours whose edge faces were refined can fail
/// in the next round (in practice one or two rounds).
const MAX_SPLIT_ROUNDS: usize = 8;

/// Dualize a conformal all-tetrahedron mesh into polyhedral cells.
///
/// One dual cell per (vertex, region) with the dual points and faces described in the crate
/// docs and `DESIGN.md`. A cell that is not star-shaped about its volume centroid (as at a
/// reflex interface or boundary edge) is split into wedges of tet corners
/// ([`DualReport::split_cells`]). `b` must be a fresh dimension-3 builder; `alloc` is unchanged
/// on `Err`. Faces are planar in [`FaceMode::Planar`](crate::FaceMode::Planar) (hub triangles
/// and per-triangle quads replace non-planar polygons); `Keep` emits one polygon per edge and
/// per fan. With [`PolyOptions::agglomerate`] small cells are then merged with a neighbour.
///
/// `edges` maps a boundary edge between two model faces to its `TopoEdge`; a source edge table,
/// when present, is consulted first.
///
/// # Errors
/// [`PolyError::NotTet`], [`PolyError::Dim2`], [`PolyError::UnsupportedFaceKind`],
/// [`PolyError::OpenRing`], [`PolyError::AmbiguousEdge`], [`PolyError::UnsupportedFeature`],
/// [`PolyError::BadClassification`] and [`PolyError::Mesh`] (builder failure).
pub fn dualize<B: MeshBuild>(
    src: &dyn MeshRead,
    opts: &PolyOptions,
    edges: &dyn EdgeResolver,
    mut b: B,
    alloc: &mut SerialAlloc,
) -> Result<(Finished<B::Output>, DualReport), PolyError> {
    if b.dim() != MeshDim::Three || b.vertex_count() != 0 || b.face_count() != 0 || b.cell_count() != 0 {
        return Err(PolyError::Mesh(MeshError::State("dualize needs a fresh dimension-3 builder".into())));
    }
    let mut prep = Prep::build(src, opts.faces, edges)?;
    let want_edges = opts.edges.unwrap_or_else(|| src.edges().is_some());
    let mut split = Split::default();
    let mut round = 0;
    let (mut data, counts) = loop {
        let (data, counts, c_key) = polydata::build(&prep, &split);
        round += 1;
        if round > MAX_SPLIT_ROUNDS {
            break (data, counts);
        }
        let mut new: Vec<u32> = concave::bad_cells(&data)
            .into_iter()
            .map(|c| c_key[c as usize])
            .filter(|&c| !split.is_split(c))
            .collect();
        new.dedup();
        if new.is_empty() {
            break (data, counts);
        }
        let mut vsplit = prep.vsplit.clone();
        let mut planned = false;
        for c in new {
            if let Some(plan) = concave::plan(&prep, c as usize) {
                vsplit[prep.ckeys[c as usize] as u32 as usize] = true;
                split.cells.insert(c, plan);
                planned = true;
            }
        }
        if !planned {
            // nothing changes: rebuilding would give the same data
            break (data, counts);
        }
        prep.set_split_vertices(vsplit)?;
    };
    let mut rep = DualReport::default();
    polydata::fill_report(&mut rep, &counts, &prep);
    if let Some(a) = &opts.agglomerate {
        rep.merges = agglom::agglomerate(&mut data, a);
    }
    polydata::commit(&data, &mut b, want_edges)?;
    let fin = b.finish(alloc)?;
    rep.cells = data.c_alive.iter().filter(|a| **a).count();
    rep.points = sm_mesh_api::MeshRead::vertex_count(&fin.mesh);
    rep.max_nonplanarity = stats::max_nonplanarity(&fin.mesh);
    let (lo, _hi, sum) = stats::volume_range(&fin.mesh);
    rep.min_cell_volume = lo;
    rep.total_volume = sum;
    Ok((fin, rep))
}
