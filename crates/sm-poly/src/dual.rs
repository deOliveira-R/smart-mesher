//! The barycentric dual: the entry point [`dualize`].

use crate::agglom;
use crate::concave::{self, Split};
use crate::error::PolyError;
use crate::options::{DualPoints, DualReport, PolyOptions};
use crate::polydata;
use crate::prep::Prep;
use crate::resolver::EdgeResolver;
use crate::measure;
use crate::mixed::{self, Mixed};
use crate::stats;
use sm_mesh_api::{
    Finished, GeomTol, MeshBuild, MeshDim, MeshEntityKind, MeshError, MeshRead, SerialAlloc, TagValues, VertexIx, validate_geometry,
};

/// Rounds of the concave-cell repair. Each round splits every cell that still fails; split
/// wedges are valid by construction, but refining a cell bends the faces of its neighbours, so
/// a failure can cascade along a model edge (10 rounds on a 10x10x6 tet-meshed brick). The cap
/// only stops a pathological loop; it is far above what real meshes need.
const MAX_SPLIT_ROUNDS: usize = 256;

#[cfg(feature = "test-hooks")]
thread_local! {
    static UNGATED: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    static GATE_OFF: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// True when the test hook disabled the strict gate and the repair refusal only (the final
/// validity check still runs).
fn gate_off() -> bool {
    #[cfg(feature = "test-hooks")]
    {
        GATE_OFF.with(|k| k.get())
    }
    #[cfg(not(feature = "test-hooks"))]
    {
        false
    }
}

/// Test hook: like [`with_ungated_circumcentric`] but the final validity check of the output stays
/// on, so its refusal ([`PolyError::InvalidOutput`]) can be exercised. Not part of the supported API.
#[doc(hidden)]
#[cfg(feature = "test-hooks")]
pub fn with_gate_off_circumcentric<R>(f: impl FnOnce() -> R) -> R {
    let prev = GATE_OFF.with(|k| k.replace(true));
    let r = f();
    GATE_OFF.with(|k| k.set(prev));
    r
}

/// True when the test hook disabled the circumcentric gate and the validity refusal.
fn ungated() -> bool {
    #[cfg(feature = "test-hooks")]
    {
        UNGATED.with(|k| k.get())
    }
    #[cfg(not(feature = "test-hooks"))]
    {
        false
    }
}

/// Test hook: run `f` with the circumcentric strict gate and the `UnrepairedCell` refusal
/// disabled on this thread, so the shape of the interior cells of a patch whose boundary is not
/// orthogonal (a BCC block, a random Delaunay set) can be checked. Boundary cells of such a
/// mesh may be invalid. Not part of the supported API.
#[doc(hidden)]
#[cfg(feature = "test-hooks")]
pub fn with_ungated_circumcentric<R>(f: impl FnOnce() -> R) -> R {
    let prev = UNGATED.with(|k| k.replace(true));
    let r = f();
    UNGATED.with(|k| k.set(prev));
    r
}

/// Dualize a conformal tetrahedron mesh into polyhedral cells.
///
/// A mixed mesh (tets plus other cells) dualizes its tet region only: the other cells are kept as
/// polyhedra, and each face between a tet and one of them is cut into the dual sub-faces of its
/// three vertices (`DESIGN.md`, "Mixed meshes"). The circumcentric dual and agglomeration are
/// refused there ([`PolyError::Mixed`]). A mesh with no tet is [`PolyError::NotTet`].
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
/// [`PolyError::BadClassification`], [`PolyError::UnrepairedCell`], [`PolyError::InvalidAgglomeration`],
/// [`PolyError::InvalidOutput`] (circumcentric), [`PolyError::Mixed`] and [`PolyError::Mesh`] (builder failure).
///
/// With [`PolyOptions::agglomerate`] and the circumcentric dual, the generator tags are omitted when
/// cells were merged (a merged cell is not a bisector cell of one generator).
pub fn dualize<B: MeshBuild>(
    src: &dyn MeshRead,
    opts: &PolyOptions,
    edges: &dyn EdgeResolver,
    b: B,
    alloc: &mut SerialAlloc,
) -> Result<(Finished<B::Output>, DualReport), PolyError> {
    // No wedge repair in the circumcentric dual: its faces run through circumcentres, and a
    // wedge rebuilds a cell from centroids (see `DESIGN.md`). A failing cell is refused.
    let rounds = if opts.dual == DualPoints::Circumcentric { 0 } else { MAX_SPLIT_ROUNDS };
    dualize_with_rounds(src, opts, edges, b, alloc, rounds)
}

/// [`dualize`] with an explicit cap on the concave-cell repair rounds (`0` disables the repair,
/// so any cell that is not star-shaped yields [`PolyError::UnrepairedCell`]). For tests.
#[doc(hidden)]
pub fn dualize_with_rounds<B: MeshBuild>(
    src: &dyn MeshRead,
    opts: &PolyOptions,
    edges: &dyn EdgeResolver,
    b: B,
    alloc: &mut SerialAlloc,
    max_rounds: usize,
) -> Result<(Finished<B::Output>, DualReport), PolyError> {
    if b.dim() != MeshDim::Three || b.vertex_count() != 0 || b.face_count() != 0 || b.cell_count() != 0 {
        return Err(PolyError::Mesh(MeshError::State("dualize needs a fresh dimension-3 builder".into())));
    }
    if let Some(mx) = mixed::split(src)? {
        if opts.dual == DualPoints::Circumcentric {
            return Err(PolyError::Mixed("the circumcentric dual needs an all-tet mesh"));
        }
        if opts.agglomerate.is_some() {
            return Err(PolyError::Mixed("agglomeration is not supported on a mixed mesh"));
        }
        return run(&mx.sub, src, opts, edges, b, alloc, max_rounds, Some(&mx));
    }
    run(src, src, opts, edges, b, alloc, max_rounds, None)
}

/// The dualization proper. `src` is the all-tet mesh to dualize (the tet part of `full` on a
/// mixed mesh, which `mixed` then stitches back).
#[allow(clippy::too_many_arguments)]
fn run<B: MeshBuild>(
    src: &dyn MeshRead,
    full: &dyn MeshRead,
    opts: &PolyOptions,
    edges: &dyn EdgeResolver,
    mut b: B,
    alloc: &mut SerialAlloc,
    max_rounds: usize,
    mixed: Option<&Mixed<'_>>,
) -> Result<(Finished<B::Output>, DualReport), PolyError> {
    // The strict gate runs first (exact predicates on the source alone); a mesh that is not
    // all-tet skips it and `Prep::build` refuses it.
    if opts.dual == DualPoints::Circumcentric
        && !ungated()
        && !gate_off()
        && let Some(el) = measure::dual_eligibility(src)
        && el.non_delaunay_faces + el.outside_circumcentres > 0
    {
        return Err(PolyError::NotOrthogonalDual {
            non_delaunay_faces: el.non_delaunay_faces,
            outside_circumcentres: el.outside_circumcentres,
            first_at: measure::first_violation(src).unwrap_or([0.0; 3]),
        });
    }
    let mut prep = Prep::build(src, opts.faces, edges, opts.dual, mixed.map_or(&[], |m| &m.inner))?;
    if let Some(m) = mixed {
        // transition vertices emit one kite per triangle so the non-tet cells can share them
        prep.set_split_vertices(m.vsplit.clone())?;
    }
    let want_edges = opts.edges.unwrap_or_else(|| full.edges().is_some());
    let mut split = Split::default();
    let mut round = 0;
    let (mut data, counts, c_key, unrepaired) = loop {
        let (data, counts, c_key) = polydata::build(&prep, &split);
        round += 1;
        let bad = concave::bad_cells(&data);
        if round > max_rounds {
            break (data, counts, c_key, bad);
        }
        let mut new: Vec<u32> = bad.iter().map(|&c| c_key[c as usize]).filter(|&c| !split.is_split(c)).collect();
        new.dedup();
        if new.is_empty() {
            break (data, counts, c_key, bad);
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
            break (data, counts, c_key, bad);
        }
        prep.set_split_vertices(vsplit)?;
    };
    if let Some(&first) = unrepaired.first()
        && !ungated()
        && !gate_off()
    {
        let key = prep.ckeys[c_key[first as usize] as usize];
        let at = concave::cell_location(&data, first as usize);
        return Err(PolyError::UnrepairedCell {
            vertex: VertexIx(key as u32),
            region: prep.regions[(key >> 32) as usize],
            at,
            count: unrepaired.len(),
        });
    }
    let mut rep = DualReport::default();
    polydata::fill_report(&mut rep, &counts, &prep);
    if let Some(a) = &opts.agglomerate {
        rep.merges = agglom::agglomerate(&mut data, a);
        if rep.merges > 0 {
            let bad = concave::bad_alive_cells(&data);
            if let Some(&first) = bad.first() {
                return Err(PolyError::InvalidAgglomeration {
                    region: data.regions[data.c_region[first as usize] as usize],
                    at: concave::cell_location(&data, first as usize),
                    count: bad.len(),
                });
            }
        }
    }
    // Per output cell, the wedge count of the dual cell it came from (0 for an unsplit cell), so
    // `mesh.poly_info` can report `wedges` and `split_cells` from the mesh alone.
    if let Some(m) = mixed {
        rep.kept_cells = m.stitch(&mut data, &prep, edges)?;
    }
    let tags_all: Vec<i64> = (0..data.n_cells())
        .map(|c| if c < c_key.len() && split.is_split(c_key[c]) { i64::from(split.n_wedges(c_key[c])) } else { 0 })
        .collect();
    let wedge_tags: Vec<i64> = (0..data.n_cells()).filter(|&c| data.c_alive[c]).map(|c| tags_all[c]).collect();
    let wedged: Vec<bool> = tags_all.iter().map(|&t| t > 0).collect();
    concave::break_primitive_cells(&mut data, &wedged, opts.all_polyhedra);
    polydata::commit(&data, &mut b, want_edges)?;
    if rep.split_cells > 0 {
        b.add_tag(stats::WEDGE_TAG, MeshEntityKind::Cell, TagValues::I64(wedge_tags))?;
    }
    // Generators are not meaningful for merged cells (a merged cell is no longer a bisector cell).
    if opts.dual == DualPoints::Circumcentric && rep.merges == 0 {
        // The generator of each output cell: the primal vertex its dual cell belongs to.
        let gen_of = |c: usize| {
            let v = prep.ckeys[c_key[c] as usize] as u32 as usize;
            let co = src.coords();
            [co.x[v], co.y[v], co.z[v]]
        };
        let alive: Vec<usize> = (0..data.n_cells()).filter(|&c| data.c_alive[c]).collect();
        for (k, name) in stats::GENERATOR_TAGS.iter().enumerate() {
            let col: Vec<f64> = alive.iter().map(|&c| gen_of(c)[k]).collect();
            b.add_tag(name, MeshEntityKind::Cell, TagValues::F64(col))?;
        }
    }
    let fin = if opts.dual == DualPoints::Circumcentric && !ungated() {
        // The strict gate cannot see every folded or zero-area face: refuse an invalid mesh.
        let mut next = alloc.clone();
        let fin = b.finish(&mut next)?;
        let bad = validate_geometry(&fin.mesh, &GeomTol::default());
        if let Some(first) = bad.first() {
            return Err(PolyError::InvalidOutput { count: bad.len(), first: format!("{first:?}") });
        }
        *alloc = next;
        fin
    } else {
        b.finish(alloc)?
    };
    rep.cells = data.c_alive.iter().filter(|a| **a).count();
    rep.points = sm_mesh_api::MeshRead::vertex_count(&fin.mesh);
    rep.max_nonplanarity = stats::max_nonplanarity(&fin.mesh);
    let (lo, _hi, sum) = stats::volume_range(&fin.mesh);
    rep.min_cell_volume = lo;
    rep.total_volume = sum;
    Ok((fin, rep))
}
