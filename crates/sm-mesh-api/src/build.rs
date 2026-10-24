//! Builder and edit traits for meshers and modifiers (separate from the read-only queries).
//!
//! * [`MeshBuild`] creates a new mesh. Handles it returns (`VertexIx`, `FaceIx`, `CellIx`) are
//!   *provisional*: `finish` normalises orientation, canonicalises the face order and assigns
//!   stable ids. Cell and vertex handles keep their position; face handles are renumbered and
//!   [`Finished::face_remap`] says how.
//! * [`MeshEdit`] changes a finished mesh without changing topology (smoothing, snapping,
//!   reclassification, tagging). Topology-changing operations rebuild through [`MeshBuild`].

use crate::classify::Classification;
use crate::error::{MeshError, Result};
use crate::highorder::{Basis, HighOrderCell, NodeRef};
use crate::index::{
    CellIx, FaceIx, FaceKind, MeshDim, MeshEntity, MeshEntityKind, MeshEntityRef, PointIx, Side,
    SignedFace, VertexIx,
};
use crate::read::{GeometryCache, MeshRead};
use crate::sets::{SetMembers, TagValues};
use crate::shape::{CellShape, LoopRelation};
use sm_core::{EntityId, EntityKind, Point3};
use smallvec::SmallVec;
use std::collections::BTreeMap;

/// Deterministic per-kind serial allocator. Serials are never reused; one allocator is shared by
/// every mesh of a model so ids stay unique across hierarchy levels and instances.
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
// Deserialization goes through `from_state` so a bad snapshot is rejected at load time.
#[cfg_attr(feature = "serde", serde(try_from = "SerialAllocRaw"))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct SerialAlloc {
    next: BTreeMap<EntityKind, u64>,
}

/// Unchecked wire form of [`SerialAlloc`].
#[cfg(feature = "serde")]
#[derive(serde::Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
struct SerialAllocRaw {
    next: BTreeMap<EntityKind, u64>,
}

#[cfg(feature = "serde")]
impl TryFrom<SerialAllocRaw> for SerialAlloc {
    type Error = MeshError;
    fn try_from(r: SerialAllocRaw) -> Result<Self> {
        Self::from_state(r.next)
    }
}

impl SerialAlloc {
    /// A fresh allocator (all kinds start at serial 1, like the op layer's entity table, so
    /// the minted ids can be adopted in order: W0f id authority).
    pub fn new() -> Self {
        Self::default()
    }
    /// The next serial that `take` would hand out for `kind` (1 if none was taken yet).
    pub fn next_serial(&self, kind: EntityKind) -> u64 {
        self.next.get(&kind).copied().unwrap_or(1)
    }
    /// The counters in ascending kind order, for snapshots.
    pub fn state(&self) -> impl Iterator<Item = (EntityKind, u64)> + '_ {
        self.next.iter().map(|(k, n)| (*k, *n))
    }
    /// Restore an allocator from `(kind, next serial)` pairs (as produced by
    /// [`SerialAlloc::state`]). A repeated kind, a serial of 0 (serials start at 1) or a serial
    /// above [`EntityId::MAX_SERIAL`] + 1 is an error.
    pub fn from_state(state: impl IntoIterator<Item = (EntityKind, u64)>) -> Result<Self> {
        let mut next = BTreeMap::new();
        for (k, n) in state {
            if n > EntityId::MAX_SERIAL.saturating_add(1) {
                return Err(MeshError::SerialOverflow);
            }
            if n == 0 {
                return Err(MeshError::State(format!("kind {k:?}: serials start at 1")));
            }
            if next.insert(k, n).is_some() {
                return Err(MeshError::State(format!("kind {k:?} given twice")));
            }
        }
        Ok(Self { next })
    }
    /// Reserve `n` consecutive serials of `kind`; returns the first.
    pub fn take(&mut self, kind: EntityKind, n: usize) -> Result<u64> {
        let first = self.next_serial(kind);
        let end = first
            .checked_add(n as u64)
            .ok_or(MeshError::SerialOverflow)?;
        if end - 1 > EntityId::MAX_SERIAL {
            return Err(MeshError::SerialOverflow);
        }
        self.next.insert(kind, end);
        Ok(first)
    }
    /// Reserve one id.
    pub fn one(&mut self, kind: EntityKind) -> Result<EntityId> {
        let s = self.take(kind, 1)?;
        EntityId::new(kind, s).ok_or(MeshError::SerialOverflow)
    }
}

/// What `add_cell` / `add_polyhedron` / `add_cell_from_vertices` created or reused, so meshers
/// do not need to call `find_face` again.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct CellAdded {
    /// The new cell.
    pub cell: CellIx,
    /// The cell's faces in input order, with sides relative to the faces' *current* loops.
    pub faces: SmallVec<[SignedFace; 6]>,
    /// For each entry of `faces`: true if the face already existed (shared with an earlier cell).
    pub reused: SmallVec<[bool; 6]>,
}

impl CellAdded {
    /// A result; implementers of [`MeshBuild::add_cell`] build it with this.
    pub fn new(cell: CellIx, faces: SmallVec<[SignedFace; 6]>, reused: SmallVec<[bool; 6]>) -> Self {
        Self { cell, faces, reused }
    }
}

/// Result of [`MeshBuild::finish`].
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct Finished<M> {
    /// The finished, canonical mesh.
    pub mesh: M,
    /// Provisional face handle -> final face index (`face_remap[provisional.get()]`).
    pub face_remap: Vec<FaceIx>,
}

impl<M> Finished<M> {
    /// A result; implementers of [`MeshBuild::finish`] build it with this.
    pub fn new(mesh: M, face_remap: Vec<FaceIx>) -> Self {
        Self { mesh, face_remap }
    }
}

/// Construct a new face-first mesh.
///
/// Meshers normally use [`MeshBuild::add_cell_from_vertices`] (template shapes) or
/// [`MeshBuild::add_polyhedron`] (general). Importers that already hold face-first data
/// (OpenFOAM, CGNS NFACE) use `add_face` + `add_cell`.
///
/// **Classification is mandatory** for vertices, boundary-like faces and cells; `finish` fills in
/// an unset interior (or non-conformal / partition-cut) face from its cell when the cells around
/// it share a classification, and an unset edge likewise, and errors otherwise.
///
/// **Dimension.** A builder makes meshes of one [`MeshDim`] ([`MeshBuild::dim`]). In dimension 2
/// faces are two-vertex facets and cells are polygons ([`MeshBuild::add_polygon`],
/// `add_cell_from_vertices` with `Tri` / `Quad`); cells are classified on a `TopoFace` /
/// `PartitionFace` and boundary facets on a `TopoEdge` / `PartitionEdge`.
pub trait MeshBuild {
    /// The finished mesh type.
    type Output: MeshRead;

    /// Dimension of the mesh being built (default [`MeshDim::Three`]).
    fn dim(&self) -> MeshDim {
        MeshDim::Three
    }

    /// Add a vertex.
    fn add_vertex(&mut self, p: Point3) -> Result<VertexIx>;
    /// Add a non-vertex node (high-order geometry); returns a reference usable in `set_high_order`.
    fn add_extra_node(&mut self, p: Point3) -> Result<NodeRef>;
    /// Number of vertices so far.
    fn vertex_count(&self) -> usize;
    /// Number of (provisional) faces so far.
    fn face_count(&self) -> usize;
    /// Number of cells so far.
    fn cell_count(&self) -> usize;
    /// Coordinates of vertex `v`. Panics if `v` is out of range.
    fn vertex_point(&self, v: VertexIx) -> Point3;
    /// Lowest-index vertex within `tol` of `p` (Chebyshev distance), for welding. `tol <= 0`
    /// means exact coordinate equality. Expected `O(1)` per query.
    fn find_vertex(&self, p: Point3, tol: f64) -> Option<VertexIx>;

    /// Add a face with the given loop (no deduplication). Needs >= 3 distinct, existing vertices
    /// (exactly 2 in a dimension-2 mesh).
    fn add_face(&mut self, vertex_loop: &[VertexIx]) -> Result<FaceIx>;
    /// Find an existing face with the same vertex cycle, and whether it runs the same way.
    fn find_face(&self, vertex_loop: &[VertexIx]) -> Option<(FaceIx, LoopRelation)>;
    /// First existing face (in creation order) with the same vertex cycle for which `accept`
    /// returns true. Coincident faces (baffle pairs) share a cycle, so [`Self::find_face`] alone
    /// can miss a usable one. The default only considers what `find_face` returns; builders that
    /// keep coincident faces should override it.
    fn find_face_where(
        &self,
        vertex_loop: &[VertexIx],
        accept: &mut dyn FnMut(FaceIx, LoopRelation) -> bool,
    ) -> Option<(FaceIx, LoopRelation)> {
        self.find_face(vertex_loop).filter(|&(f, r)| accept(f, r))
    }
    /// Current loop of a face. Panics if `f` is out of range.
    fn face_loop(&self, f: FaceIx) -> &[VertexIx];
    /// Number of cells attached to a face so far. Panics if `f` is out of range.
    fn face_degree(&self, f: FaceIx) -> usize;
    /// Drop all faces with index `>= len`. They must have no cells (used to roll back
    /// `add_polyhedron`); fails otherwise.
    fn truncate_faces(&mut self, len: usize) -> Result<()>;

    /// Add a cell from signed faces (`side` relative to each face's current loop; see
    /// [`SignedFace`]). `corners` is the template vertex order for template-shaped cells.
    ///
    /// Two-phase: every check (face range, repeated faces, a side already taken, corners that
    /// do not fit) runs before anything is stored, so an error leaves the builder unchanged.
    /// "Fit" means the right number of distinct, existing vertices. Whether the corners
    /// realise the template's faces is checked by `finish` ([`MeshError::ShapeMismatch`]).
    fn add_cell(&mut self, faces: &[SignedFace], corners: Option<&[VertexIx]>) -> Result<CellIx>;
    /// Signed faces of a (provisional) cell. Panics if `c` is out of range.
    fn cell_faces(&self, c: CellIx) -> &[SignedFace];
    /// The cell attached to `f` on the given side so far, if any.
    fn face_cell(&self, f: FaceIx, side: Side) -> Option<CellIx>;
    /// First cell that was added using vertex `v`, if any.
    fn vertex_cell(&self, v: VertexIx) -> Option<CellIx>;
    /// Classification set so far for a vertex.
    fn vertex_class(&self, v: VertexIx) -> Option<Classification>;
    /// Classification set so far for a (provisional) face.
    fn face_class(&self, f: FaceIx) -> Option<Classification>;
    /// Classification set so far for a cell.
    fn cell_class(&self, c: CellIx) -> Option<Classification>;

    /// Classify a vertex, edge (by vertex pair, resolved at `finish`), face or cell. The
    /// parameter must fit the entity kind ([`crate::Param::fits`]); a mismatch is rejected.
    /// Setting any edge classification makes `finish` create the edge table.
    fn set_class(&mut self, e: MeshEntityRef, c: Classification) -> Result<()>;
    /// Mark a one-cell face as [`FaceKind::NonConformal`] or [`FaceKind::PartitionCut`]
    /// (the default kinds are derived: two cells = `Interior`, one cell = `Boundary`).
    /// Passing `Boundary` clears an earlier mark; `Interior` is rejected.
    fn set_face_kind(&mut self, f: FaceIx, kind: FaceKind) -> Result<()>;
    /// Record that `c` is a child of `parent` (refinement lineage). The parent must have a
    /// lower index than the child.
    fn set_cell_parent(&mut self, c: CellIx, parent: CellIx) -> Result<()>;
    /// Mark a cell inactive (a parent retained after refinement) or active (default).
    fn set_cell_active(&mut self, c: CellIx, active: bool) -> Result<()>;
    /// Give a template-shaped cell high-order geometry. `nodes` are in `smartmesh.nodes.v1`
    /// order, corners first; the count must equal [`crate::node_count`]. Polyhedra are rejected
    /// (they curve only through face curves).
    fn set_high_order(
        &mut self,
        c: CellIx,
        degree: u8,
        basis: Basis,
        nodes: &[NodeRef],
    ) -> Result<()>;
    /// Curve a face: `nodes` are the loop vertices (in the face's *current* loop order) followed
    /// by the edge nodes and centre ([`crate::face_node_count`]). `finish` permutes them when
    /// it re-orients the loop.
    fn set_face_curve(
        &mut self,
        f: FaceIx,
        degree: u8,
        basis: Basis,
        nodes: &[NodeRef],
    ) -> Result<()>;
    /// Add a named set (face members use provisional face handles; a side set's sides are
    /// relative to the provisional face's cells and are re-sided by `finish`).
    fn add_set(&mut self, name: &str, members: SetMembers) -> Result<()>;
    /// Set or clear the integer label of the set `name` (unique per members kind, checked at
    /// `finish`). Provided: implementations without labels return `Unsupported`.
    fn set_label(&mut self, name: &str, label: Option<i64>) -> Result<()> {
        let _ = (name, label);
        Err(MeshError::Unsupported("set labels".into()))
    }
    /// Add a named tag column over all entities of `target` (length checked at `finish`).
    /// Face columns are indexed by provisional face handle.
    fn add_tag(&mut self, name: &str, target: MeshEntityKind, values: TagValues) -> Result<()>;

    /// Orient, canonicalise, validate incidence, assign ids and return the finished mesh with the
    /// face remap. The mesh id and all other ids come from `alloc`.
    ///
    /// **`alloc` is unchanged on `Err`**: ids are what the model history records, so a failed
    /// attempt must not shift the ids of a later successful one.
    fn finish(self, alloc: &mut SerialAlloc) -> Result<Finished<Self::Output>>
    where
        Self: Sized;

    /// Add a general polyhedron from outward-oriented face loops, reusing faces that already
    /// exist. On error the builder is left unchanged. Template-shaped results get their
    /// corner order derived at `finish`.
    fn add_polyhedron(&mut self, outward: &[&[VertexIx]]) -> Result<CellAdded> {
        self.add_polyhedron_with_corners(outward, None)
    }

    /// Like [`MeshBuild::add_polyhedron`], recording `corners` (template order) for the cell.
    /// Used to preserve corner order when copying template cells.
    fn add_polyhedron_with_corners(
        &mut self,
        outward: &[&[VertexIx]],
        corners: Option<&[VertexIx]>,
    ) -> Result<CellAdded> {
        add_poly(self, outward, corners)
    }

    /// Add a polygon cell of a dimension-2 mesh from its vertex cycle (boundary order), reusing
    /// facets that already exist. The cycle is recorded as the cell's corners. On error the
    /// builder is left unchanged.
    fn add_polygon(&mut self, cycle: &[VertexIx]) -> Result<CellAdded> {
        if self.dim() != MeshDim::Two {
            return Err(MeshError::Unsupported(
                "add_polygon builds dimension-2 meshes".into(),
            ));
        }
        let n = cycle.len();
        if n < 3 {
            return Err(MeshError::InvalidLoop(
                "a polygon needs at least 3 vertices".into(),
            ));
        }
        let edges: SmallVec<[[VertexIx; 2]; 8]> =
            (0..n).map(|i| [cycle[i], cycle[(i + 1) % n]]).collect();
        let refs: SmallVec<[&[VertexIx]; 8]> = edges.iter().map(|e| e.as_slice()).collect();
        self.add_polyhedron_with_corners(&refs, Some(cycle))
    }

    /// Add a template-shaped cell from vertices in the template order of `shape`.
    fn add_cell_from_vertices(
        &mut self,
        shape: CellShape,
        verts: &[VertexIx],
    ) -> Result<CellAdded> {
        if shape.vertex_count() != Some(verts.len()) {
            return Err(MeshError::State(format!(
                "{shape:?} needs {:?} vertices, got {}",
                shape.vertex_count(),
                verts.len()
            )));
        }
        if shape.dim() != self.dim() {
            return Err(MeshError::State(format!(
                "{shape:?} cells do not belong in a {:?} mesh",
                self.dim()
            )));
        }
        let loops: SmallVec<[SmallVec<[VertexIx; 4]>; 6]> = shape
            .local_faces()
            .iter()
            .map(|lf| lf.iter().map(|&i| verts[i as usize]).collect())
            .collect();
        let refs: SmallVec<[&[VertexIx]; 6]> = loops.iter().map(|l| l.as_slice()).collect();
        self.add_polyhedron_with_corners(&refs, Some(verts))
    }

    /// Add many template cells: cell `c` has shape `shapes[c]` and vertices
    /// `verts[offsets[c]..offsets[c + 1]]` in the template order of its shape. `offsets` is a CSR
    /// array over `verts` (`offsets.len() == shapes.len() + 1`, starts at 0, non-decreasing, ends
    /// at `verts.len()`), otherwise `MeshError::State`. Returns the range of new cell indices.
    ///
    /// The default is [`add_template_cells_sequential`]: a loop of
    /// [`add_cell_from_vertices`](Self::add_cell_from_vertices). Errors stop at the first bad
    /// cell; the cells before it stay added.
    ///
    /// **Contract for overrides.** A faster (e.g. parallel) override must produce the same cells,
    /// the same face numbering and so the same `connectivity_hash` as the default on success, and
    /// the same error for the first offending cell. Which cells survive an error is up to the
    /// implementation unless it matches the default. `sm_testkit::check::template_cells_match_default`
    /// checks it.
    fn add_template_cells(
        &mut self,
        shapes: &[CellShape],
        offsets: &[u64],
        verts: &[VertexIx],
    ) -> Result<core::ops::Range<usize>> {
        add_template_cells_sequential(self, shapes, offsets, verts)
    }

    /// Hint that about `vertices` more vertices and `cells` more cells will be added. Never
    /// changes results; the default does nothing.
    fn reserve(&mut self, vertices: usize, cells: usize) {
        let _ = (vertices, cells);
    }
}

fn add_poly<B: MeshBuild + ?Sized>(
    b: &mut B,
    outward: &[&[VertexIx]],
    corners: Option<&[VertexIx]>,
) -> Result<CellAdded> {
    let mark = b.face_count();
    let mut faces: SmallVec<[SignedFace; 6]> = SmallVec::new();
    let mut reused: SmallVec<[bool; 6]> = SmallVec::new();
    for l in outward {
        match b.find_face(l) {
            Some((f, rel)) => {
                faces.push(SignedFace::new(
                    f,
                    if rel == LoopRelation::Same {
                        Side::Left
                    } else {
                        Side::Right
                    },
                ));
                reused.push(true);
            }
            None => match b.add_face(l) {
                Ok(f) => {
                    faces.push(SignedFace::new(f, Side::Left));
                    reused.push(false);
                }
                Err(e) => {
                    let _ = b.truncate_faces(mark);
                    return Err(e);
                }
            },
        }
    }
    match b.add_cell(&faces, corners) {
        Ok(cell) => Ok(CellAdded {
            cell,
            faces,
            reused,
        }),
        Err(e) => {
            let _ = b.truncate_faces(mark);
            Err(e)
        }
    }
}

/// The default of [`MeshBuild::add_template_cells`]: validate the CSR `offsets`, then call
/// [`MeshBuild::add_cell_from_vertices`] for each cell in order. Public so an override can fall
/// back to it (for example to reproduce the exact error and partial state after a fast path
/// failed).
pub fn add_template_cells_sequential<B: MeshBuild + ?Sized>(
    b: &mut B,
    shapes: &[CellShape],
    offsets: &[u64],
    verts: &[VertexIx],
) -> Result<core::ops::Range<usize>> {
    let n = shapes.len();
    if offsets.len() != n + 1
        || offsets[0] != 0
        || offsets[n] != verts.len() as u64
        || offsets.windows(2).any(|w| w[0] > w[1])
    {
        return Err(MeshError::State(
            "offsets must be a CSR array over `verts`".into(),
        ));
    }
    let start = b.cell_count();
    for (c, shape) in shapes.iter().enumerate() {
        b.add_cell_from_vertices(*shape, &verts[offsets[c] as usize..offsets[c + 1] as usize])?;
    }
    Ok(start..start + n)
}

/// Topology-preserving edits of a finished mesh.
///
/// Edits check the rules local to one entity (the parameter fits, a `Boundary` face stays on a
/// `TopoFace`, a cell stays on a volume, an edge tag needs an edge table). They do **not** check
/// rules that relate entities, such as the vertex <= edge <= face dimension order, so a caller
/// that edits classifications should run [`crate::validate()`] afterwards.
///
/// Every edit that changes stored geometry
/// (`set_point`, `add_extra_node`) or the connectivity-keyed cache inputs drops the geometry
/// cache; recompute it with [`crate::compute_geometry_cache`] and `set_geometry_cache`.
pub trait MeshEdit: MeshRead {
    /// Move a point (vertex or extra node). Drops the geometry cache.
    fn set_point(&mut self, i: PointIx, p: Point3) -> Result<()>;
    /// Append an extra (non-vertex) node; returns its point index.
    fn add_extra_node(&mut self, p: Point3) -> Result<PointIx>;
    /// Reclassify a vertex, edge, face or cell. The parameter must fit the entity kind.
    fn set_class(&mut self, e: MeshEntity, c: Classification) -> Result<()>;
    /// Reclassify many entities at once; all-or-nothing (every item is checked before any is
    /// applied). Implementations should rebuild their reverse index once, so reclassifying a
    /// patch of `k` entities does not cost `k` full passes. The default loops over
    /// [`MeshEdit::set_class`] and is not atomic.
    fn set_classes(&mut self, items: &[(MeshEntity, Classification)]) -> Result<()> {
        for (e, c) in items {
            self.set_class(*e, *c)?;
        }
        Ok(())
    }
    /// Change the kind of a one-cell face to `Boundary`, `NonConformal` or `PartitionCut`.
    ///
    /// Face kinds are part of the connectivity hash, so this drops the geometry cache and makes
    /// every `LevelMap` built over the mesh stale.
    fn set_face_kind(&mut self, f: FaceIx, kind: FaceKind) -> Result<()>;
    /// Set or clear a cell's high-order record (count-checked; polyhedra rejected).
    /// On a dimension-3 mesh a new record needs the edge table first ([`MeshEdit::ensure_edges`]).
    fn set_high_order(&mut self, c: CellIx, h: Option<HighOrderCell>) -> Result<()>;
    /// Set or clear a face's curvature record `(degree, basis, nodes)` (count-checked; the
    /// first nodes must be the face's loop vertices).
    /// On a dimension-3 mesh a new record needs the edge table first ([`MeshEdit::ensure_edges`]).
    fn set_face_curve(&mut self, f: FaceIx, c: Option<(u8, Basis, Vec<PointIx>)>) -> Result<()>;
    /// Add a named set; its id comes from `alloc`, which is unchanged on `Err`.
    fn add_set(
        &mut self,
        name: &str,
        members: SetMembers,
        alloc: &mut SerialAlloc,
    ) -> Result<EntityId>;
    /// Set or clear the integer label of the set `name` (must stay unique per members kind).
    /// Provided: implementations without labels return `Unsupported`.
    fn set_label(&mut self, name: &str, label: Option<i64>) -> Result<()> {
        let _ = (name, label);
        Err(MeshError::Unsupported("set labels".into()))
    }
    /// Add or replace a tag column.
    fn set_tag(&mut self, name: &str, target: MeshEntityKind, values: TagValues) -> Result<()>;
    /// Derive and attach the edge table (no-op if present). `explicit` gives classifications by
    /// vertex pair; other edges are classified like `finish` does (see [`crate::EdgeTable::derive`]).
    /// `alloc` is unchanged on `Err`.
    fn ensure_edges(
        &mut self,
        alloc: &mut SerialAlloc,
        explicit: &[(VertexIx, VertexIx, Classification)],
    ) -> Result<()>;
    /// Detach the edge table together with all edge-target tags. Refused
    /// ([`crate::MeshError::State`]) on a dimension-3 mesh with high-order cells or face curves,
    /// whose edge nodes take the class of their edge.
    fn drop_edges(&mut self) -> Result<()>;
    /// Attach or drop a geometry cache. A cache whose rule, sizes or connectivity hash do not
    /// match the mesh (including the coordinate hash) is rejected.
    fn set_geometry_cache(&mut self, g: Option<GeometryCache>) -> Result<()>;
}
