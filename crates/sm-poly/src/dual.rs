//! The barycentric dual: the entry point [`dualize`].

use crate::agglom;
use crate::concave::{self, Split};
use crate::error::PolyError;
use crate::options::{DualReport, PolyOptions};
use crate::polydata;
use crate::prep::Prep;
use crate::resolver::EdgeResolver;
use crate::stats;
use sm_mesh_api::{Finished, MeshBuild, MeshDim, MeshEntityKind, MeshError, MeshRead, SerialAlloc, TagValues};

/// Rounds of the concave-cell repair. Each round splits every cell that still fails; split
/// wedges are valid by construction, but refining a cell bends the faces of its neighbours, so
/// a failure can cascade along a model edge (10 rounds on a 10x10x6 tet-meshed brick). The cap
/// only stops a pathological loop; it is far above what real meshes need.
const MAX_SPLIT_ROUNDS: usize = 256;

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
    let (mut data, counts, c_key) = loop {
        let (data, counts, c_key) = polydata::build(&prep, &split);
        round += 1;
        if round > MAX_SPLIT_ROUNDS {
            break (data, counts, c_key);
        }
        let mut new: Vec<u32> = concave::bad_cells(&data)
            .into_iter()
            .map(|c| c_key[c as usize])
            .filter(|&c| !split.is_split(c))
            .collect();
        new.dedup();
        if new.is_empty() {
            break (data, counts, c_key);
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
            break (data, counts, c_key);
        }
        prep.set_split_vertices(vsplit)?;
    };
    let mut rep = DualReport::default();
    polydata::fill_report(&mut rep, &counts, &prep);
    if let Some(a) = &opts.agglomerate {
        rep.merges = agglom::agglomerate(&mut data, a);
    }
    // Per output cell, the wedge count of the dual cell it came from (0 for an unsplit cell), so
    // `mesh.poly_info` can report `wedges` and `split_cells` from the mesh alone.
    let tags_all: Vec<i64> = (0..data.n_cells())
        .map(|c| if split.is_split(c_key[c]) { i64::from(split.n_wedges(c_key[c])) } else { 0 })
        .collect();
    let wedge_tags: Vec<i64> = (0..data.n_cells()).filter(|&c| data.c_alive[c]).map(|c| tags_all[c]).collect();
    let wedged: Vec<bool> = tags_all.iter().map(|&t| t > 0).collect();
    concave::break_primitive_cells(&mut data, &wedged, opts.all_polyhedra);
    polydata::commit(&data, &mut b, want_edges)?;
    if rep.split_cells > 0 {
        b.add_tag(stats::WEDGE_TAG, MeshEntityKind::Cell, TagValues::I64(wedge_tags))?;
    }
    let fin = b.finish(alloc)?;
    rep.cells = data.c_alive.iter().filter(|a| **a).count();
    rep.points = sm_mesh_api::MeshRead::vertex_count(&fin.mesh);
    rep.max_nonplanarity = stats::max_nonplanarity(&fin.mesh);
    let (lo, _hi, sum) = stats::volume_range(&fin.mesh);
    rep.min_cell_volume = lo;
    rep.total_volume = sum;
    Ok((fin, rep))
}
