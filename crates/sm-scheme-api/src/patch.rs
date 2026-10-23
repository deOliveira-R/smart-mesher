//! What a scheme returns from `apply`: a local patch that references already committed nodes
//! ([`PatchRef::Old`]) and its own new nodes ([`PatchRef::New`]). Patches are built in parallel
//! and committed in a fixed order; the commit numbers new nodes, so global numbering never
//! depends on scheduling. Connectivity is CSR (offsets + flat indices).

use crate::key::ShapeKey;
use serde::{Deserialize, Serialize};
use sm_core::Point3;
use sm_mesh_api::{CellShape, Param};

/// A committed mesh node (dense, assigned at commit in deterministic order).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct NodeId(pub u32);

impl NodeId {
    /// Dense index.
    pub const fn get(self) -> usize {
        self.0 as usize
    }
}

/// A node reference inside a patch.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum PatchRef {
    /// A node committed before this patch (boundary nodes).
    Old(NodeId),
    /// Index into the patch's `nodes`.
    New(u32),
}

/// A node created by a patch.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct NewNode {
    /// Position.
    pub point: Point3,
    /// The entity it lies on (a target of the plan, or one of its claims).
    pub on: ShapeKey,
    /// Parameters on that entity (`Param::None` if unknown).
    pub param: Param,
}

/// Output of a curve scheme: the interior node positions as arc-length fractions.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CurvePatch {
    /// `intervals - 1` strictly increasing values in `(0, 1)`, measured from the curve's start.
    pub fractions: Vec<f64>,
}

/// Polygons in CSR form.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FaceList<R> {
    /// `off[i]..off[i+1]` indexes `nodes` for face `i`; `off[0] = 0`.
    pub off: Vec<u32>,
    /// Loop nodes, face after face.
    pub nodes: Vec<R>,
}

impl<R: Copy> Default for FaceList<R> {
    fn default() -> Self {
        FaceList { off: vec![0], nodes: Vec::new() }
    }
}

impl<R: Copy> FaceList<R> {
    /// Empty list.
    pub fn new() -> Self {
        Self::default()
    }
    /// Append a face loop.
    pub fn push(&mut self, face: &[R]) {
        self.nodes.extend_from_slice(face);
        self.off.push(self.nodes.len() as u32);
    }
    /// Number of faces.
    pub fn len(&self) -> usize {
        self.off.len().saturating_sub(1)
    }
    /// True if empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
    /// Loop of face `i`.
    pub fn face(&self, i: usize) -> &[R] {
        &self.nodes[self.off[i] as usize..self.off[i + 1] as usize]
    }
    /// All loops in order.
    pub fn iter(&self) -> impl Iterator<Item = &[R]> + '_ {
        (0..self.len()).map(move |i| self.face(i))
    }
}

/// Logical `(i, j)` grid of a structured surface patch: `(ni + 1) x (nj + 1)` nodes, row-major
/// (`nodes[j * (ni + 1) + i]`). `j = 0` runs along logical side 0 in loop direction.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Grid<R> {
    /// Segments along `i`.
    pub ni: u32,
    /// Segments along `j`.
    pub nj: u32,
    /// The nodes.
    pub nodes: Vec<R>,
}

impl<R: Copy> Grid<R> {
    /// Node `(i, j)`.
    pub fn at(&self, i: u32, j: u32) -> R {
        self.nodes[(j * (self.ni + 1) + i) as usize]
    }
}

/// Output of a surface scheme. Faces are oriented like the face (right-hand rule on the loop
/// gives the face normal, i.e. the topology face's left-to-right normal).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SurfacePatch {
    /// New interior nodes.
    pub nodes: Vec<NewNode>,
    /// The 2D elements.
    pub faces: FaceList<PatchRef>,
    /// Logical grid, for structured schemes (`Caps::STRUCTURED`).
    pub grid: Option<Grid<PatchRef>>,
}

/// 3D cells in CSR form with template node order (`sm_mesh_api::CellShape`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CellList {
    /// Shape per cell (template shapes only; general polyhedra are an open item).
    pub shape: Vec<CellShape>,
    /// CSR offsets into `nodes`.
    pub off: Vec<u32>,
    /// Nodes in template order.
    pub nodes: Vec<PatchRef>,
    /// Index into the plan's `targets` (which volume/piece the cell belongs to).
    pub target: Vec<u32>,
}

impl Default for CellList {
    fn default() -> Self {
        CellList { shape: Vec::new(), off: vec![0], nodes: Vec::new(), target: Vec::new() }
    }
}

impl CellList {
    /// Append a template cell for target index `target`.
    pub fn push(&mut self, shape: CellShape, nodes: &[PatchRef], target: u32) {
        self.shape.push(shape);
        self.nodes.extend_from_slice(nodes);
        self.off.push(self.nodes.len() as u32);
        self.target.push(target);
    }
    /// Number of cells.
    pub fn len(&self) -> usize {
        self.shape.len()
    }
    /// True if empty.
    pub fn is_empty(&self) -> bool {
        self.shape.is_empty()
    }
    /// Nodes of cell `i`.
    pub fn cell(&self, i: usize) -> &[PatchRef] {
        &self.nodes[self.off[i] as usize..self.off[i + 1] as usize]
    }
}

/// The mesh of a claimed face produced by a volume scheme (sweep target).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ClaimedSurface {
    /// The face (must be in `Plan::claims`).
    pub face: ShapeKey,
    /// Its elements, oriented like the face.
    pub faces: FaceList<PatchRef>,
    /// Its grid, if structured.
    pub grid: Option<Grid<PatchRef>>,
}

/// Output of a volume scheme or an atomic group.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct VolumePatch {
    /// New nodes (interior, and on claimed faces).
    pub nodes: Vec<NewNode>,
    /// The cells.
    pub cells: CellList,
    /// Meshes of claimed faces.
    pub surfaces: Vec<ClaimedSurface>,
}

/// Output of a [`crate::MeshTransform`]: replaces the cells of all its targets and the meshes of
/// the listed surfaces in one all-or-nothing commit (THex on merged volumes, QTri, HTet).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct TransformPatch {
    /// New cells and nodes for the targets (replace the old cells of every target).
    pub volume: VolumePatch,
    /// Surface meshes replaced (shared faces of merged volumes must change consistently).
    pub replace_surfaces: Vec<ClaimedSurface>,
}
