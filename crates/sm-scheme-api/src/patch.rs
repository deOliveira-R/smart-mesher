//! [`MeshPatch`]: the delta one job produces, in columnar form.
//!
//! A patch has a local node table (`nodes`); elements index into it with `u32`, so element data is
//! flat CSR with no per-element allocation. A node is either **new** (`point: Some`, owned by a
//! target the job writes) or a **reference** to a committed node (`point: None`).

use crate::target::{NodeKey, Target};
use sm_core::Point3;
use sm_mesh_api::{CellShape, Param};
use std::collections::BTreeMap;
use std::ops::Range;

/// One entry of the local node table.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PatchNode {
    /// Stable name.
    pub key: NodeKey,
    /// `Some` for a node created by this patch, `None` for a committed node it references.
    pub point: Option<Point3>,
    /// Parameter on the owner (`Curve(t)` on an edge, `Surface([u, v])` on a face).
    pub param: Param,
}

/// The mesh of one target inside a patch: a range of elements of its dimension.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Part {
    /// The target.
    pub target: Target,
    /// Range into the element list of the part's dimension.
    pub range: Range<u32>,
}

/// A job's output.
///
/// - Curves: one part per edge; `curve_nodes[range]` lists the nodes start vertex -> end vertex
///   (inclusive), so the part's range is a node range, not an element range.
/// - Faces: element `k` has nodes `face_nodes[face_off[k]..face_off[k + 1]]`, oriented by the
///   face's right-hand rule (normal from its left region to its right region).
/// - Cells: element `k` has shape `cell_shape[k]` and corners `cell_nodes[cell_off[k]..]` in the
///   `sm_mesh_api::CellShape` template order.
#[derive(Clone, Debug, PartialEq)]
pub struct MeshPatch {
    /// Local node table (keys unique).
    pub nodes: Vec<PatchNode>,
    /// Edge parts (ranges into `curve_nodes`).
    pub curves: Vec<Part>,
    /// Edge node lists.
    pub curve_nodes: Vec<u32>,
    /// Face parts (ranges of face elements).
    pub faces: Vec<Part>,
    /// Face element offsets (`len = elements + 1`).
    pub face_off: Vec<u32>,
    /// Face element nodes.
    pub face_nodes: Vec<u32>,
    /// Cell parts (ranges of cells).
    pub cells: Vec<Part>,
    /// Cell shapes.
    pub cell_shape: Vec<CellShape>,
    /// Cell offsets (`len = cells + 1`).
    pub cell_off: Vec<u32>,
    /// Cell corner nodes.
    pub cell_nodes: Vec<u32>,
}

impl Default for MeshPatch {
    fn default() -> Self {
        Self { nodes: Vec::new(), curves: Vec::new(), curve_nodes: Vec::new(), faces: Vec::new(), face_off: vec![0], face_nodes: Vec::new(), cells: Vec::new(), cell_shape: Vec::new(), cell_off: vec![0], cell_nodes: Vec::new() }
    }
}

impl MeshPatch {
    /// Every target with a part, ascending.
    pub fn written(&self) -> Vec<Target> {
        let mut v: Vec<Target> = self.curves.iter().chain(&self.faces).chain(&self.cells).map(|p| p.target).collect();
        v.sort_unstable();
        v
    }
    /// Number of face elements.
    pub fn face_count(&self) -> usize {
        self.face_off.len().saturating_sub(1)
    }
    /// Number of cells.
    pub fn cell_count(&self) -> usize {
        self.cell_off.len().saturating_sub(1)
    }
}

/// Convenience builder for schemes: deduplicates node keys and keeps the CSR consistent.
#[derive(Default)]
pub struct PatchBuilder {
    patch: MeshPatch,
    index: BTreeMap<NodeKey, u32>,
    open: Option<(u8, Target, u32)>,
}

impl PatchBuilder {
    /// Empty builder.
    pub fn new() -> Self {
        Self::default()
    }

    /// A new node (owned by a written target). Re-adding a key returns the existing index.
    pub fn new_node(&mut self, key: NodeKey, point: Point3, param: Param) -> u32 {
        self.add(PatchNode { key, point: Some(point), param })
    }

    /// A reference to a committed node.
    pub fn reference(&mut self, key: NodeKey) -> u32 {
        self.add(PatchNode { key, point: None, param: Param::None })
    }

    fn add(&mut self, n: PatchNode) -> u32 {
        if let Some(&i) = self.index.get(&n.key) {
            return i;
        }
        let i = self.patch.nodes.len() as u32;
        self.index.insert(n.key, i);
        self.patch.nodes.push(n);
        i
    }

    /// The local index of a key already in the table.
    pub fn local(&self, key: &NodeKey) -> Option<u32> {
        self.index.get(key).copied()
    }

    fn close(&mut self) {
        if let Some((dim, target, start)) = self.open.take() {
            let (fc, cc, nc) = (self.patch.face_count() as u32, self.patch.cell_count() as u32, self.patch.curve_nodes.len() as u32);
            let (list, end) = match dim {
                1 => (&mut self.patch.curves, nc),
                2 => (&mut self.patch.faces, fc),
                _ => (&mut self.patch.cells, cc),
            };
            list.push(Part { target, range: start..end });
        }
    }

    /// Write an edge mesh: node indices from start vertex to end vertex.
    pub fn curve(&mut self, target: Target, nodes: &[u32]) {
        self.close();
        let start = self.patch.curve_nodes.len() as u32;
        self.patch.curve_nodes.extend_from_slice(nodes);
        self.open = Some((1, target, start));
        self.close();
    }

    /// Start the elements of a face target (closes the previous part).
    pub fn begin_face(&mut self, target: Target) {
        self.close();
        self.open = Some((2, target, self.patch.face_count() as u32));
    }

    /// Append a face element to the open face part.
    pub fn face(&mut self, nodes: &[u32]) {
        self.patch.face_nodes.extend_from_slice(nodes);
        self.patch.face_off.push(self.patch.face_nodes.len() as u32);
    }

    /// Start the cells of a volume target (closes the previous part).
    pub fn begin_cells(&mut self, target: Target) {
        self.close();
        self.open = Some((3, target, self.patch.cell_count() as u32));
    }

    /// Append a cell (corners in template order) to the open cell part.
    pub fn cell(&mut self, shape: CellShape, corners: &[u32]) {
        self.patch.cell_shape.push(shape);
        self.patch.cell_nodes.extend_from_slice(corners);
        self.patch.cell_off.push(self.patch.cell_nodes.len() as u32);
    }

    /// Finish.
    pub fn finish(mut self) -> MeshPatch {
        self.close();
        self.patch
    }
}
