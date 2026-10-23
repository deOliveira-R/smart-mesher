//! Patches: the pure output of one scheme invocation, before commit.

use crate::target::{NodeKey, Target};
use sm_core::Point3;
use sm_mesh_api::{CellShape, Param};
use std::collections::BTreeMap;

/// Shape of one patch element.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ElemShape {
    /// Two nodes (curve patches).
    Segment,
    /// Triangle.
    Tri,
    /// Quadrilateral.
    Quad,
    /// Polygon with five or more nodes.
    Polygon,
    /// Volume cell with a template shape; nodes in `sm-mesh-api` template order.
    Cell(CellShape),
}

/// Elements in CSR form (one flat node array plus offsets; no per-element allocation).
#[derive(Clone, Debug, PartialEq, Default)]
pub struct Elements {
    shapes: Vec<ElemShape>,
    offsets: Vec<u32>,
    nodes: Vec<NodeKey>,
}

impl Elements {
    /// Empty element block.
    pub fn new() -> Self {
        Self { shapes: Vec::new(), offsets: vec![0], nodes: Vec::new() }
    }

    /// Empty element block with room for `n` elements of `k` nodes.
    pub fn with_capacity(n: usize, k: usize) -> Self {
        let mut offsets = Vec::with_capacity(n + 1);
        offsets.push(0);
        Self { shapes: Vec::with_capacity(n), offsets, nodes: Vec::with_capacity(n * k) }
    }

    /// Append an element. Surface elements are oriented with the face normal (right-hand
    /// rule); volume cells use the template order of their shape.
    pub fn push(&mut self, shape: ElemShape, nodes: &[NodeKey]) {
        self.shapes.push(shape);
        self.nodes.extend_from_slice(nodes);
        self.offsets.push(self.nodes.len() as u32);
    }

    /// Number of elements.
    pub fn len(&self) -> usize {
        self.shapes.len()
    }

    /// True if there are no elements.
    pub fn is_empty(&self) -> bool {
        self.shapes.is_empty()
    }

    /// Element `i`: shape and nodes. Panics if `i >= len()` (caller contract).
    pub fn get(&self, i: usize) -> (ElemShape, &[NodeKey]) {
        let (a, b) = (self.offsets[i] as usize, self.offsets[i + 1] as usize);
        (self.shapes[i], &self.nodes[a..b])
    }

    /// All elements in order.
    pub fn iter(&self) -> impl Iterator<Item = (ElemShape, &[NodeKey])> + '_ {
        (0..self.len()).map(move |i| self.get(i))
    }

    /// All node references (flat).
    pub fn node_refs(&self) -> &[NodeKey] {
        &self.nodes
    }
}

/// A new node created by a patch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PatchNode {
    /// Canonical key (its owner must be the target entity or, for groups, a member).
    pub key: NodeKey,
    /// Position.
    pub point: Point3,
    /// Parameter on the owner (must fit the owner's kind, see [`Param::fits`]).
    pub param: Param,
}

/// Structured view of a mapped surface patch: `ni x nj` node keys, row-major (`j * ni + i`).
/// Row `j = 0` runs along logical side 0 of the face layout, column `i = 0` along side 3.
/// Lets a volume scheme (sweep, map) walk lines of a neighbouring mapped face without
/// knowing its private numbering.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NodeGrid {
    /// Nodes along `i`.
    pub ni: u32,
    /// Nodes along `j`.
    pub nj: u32,
    /// The keys.
    pub nodes: Vec<NodeKey>,
}

impl NodeGrid {
    /// Node `(i, j)`. Panics if outside the grid (caller contract).
    pub fn at(&self, i: u32, j: u32) -> NodeKey {
        self.nodes[(j * self.ni + i) as usize]
    }

    /// The grid line that starts at boundary node `key` and runs across the grid
    /// (perpendicular to the boundary side `key` lies on), or `None` if `key` is not on the
    /// grid boundary or is a grid corner.
    pub fn line_from(&self, key: NodeKey) -> Option<Vec<NodeKey>> {
        let pos = self.nodes.iter().position(|k| *k == key)? as u32;
        let (i, j) = (pos % self.ni, pos / self.ni);
        let (li, lj) = (self.ni - 1, self.nj - 1);
        let corner = (i == 0 || i == li) && (j == 0 || j == lj);
        if corner {
            return None;
        }
        if j == 0 {
            Some((0..self.nj).map(|q| self.at(i, q)).collect())
        } else if j == lj {
            Some((0..self.nj).rev().map(|q| self.at(i, q)).collect())
        } else if i == 0 {
            Some((0..self.ni).map(|q| self.at(q, j)).collect())
        } else if i == li {
            Some((0..self.ni).rev().map(|q| self.at(q, j)).collect())
        } else {
            None
        }
    }
}

/// What one scheme invocation produced. Pure data: nothing is written to the mesh until commit.
#[derive(Clone, Debug, PartialEq)]
pub struct Patch {
    /// The target.
    pub target: Target,
    /// Nodes created by this patch (owned by the target entity).
    pub nodes: Vec<PatchNode>,
    /// Elements, referencing these nodes and the nodes of lower-dimensional patches.
    pub elements: Elements,
    /// Structured view, if the scheme is structured (map, submap blocks).
    pub grid: Option<NodeGrid>,
    /// Per-element owning piece, for group patches only (empty: every element belongs to the
    /// target entity).
    pub owners: Vec<sm_core::EntityId>,
}

impl Patch {
    /// An empty patch for `target`.
    pub fn new(target: Target) -> Self {
        Self { target, nodes: Vec::new(), elements: Elements::new(), grid: None, owners: Vec::new() }
    }
}

/// All patches generated so far plus the node table, in canonical (key) order. Filled wave
/// by wave in a fixed order, so its content does not depend on scheduling.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PatchSet {
    pub(crate) patches: BTreeMap<Target, Patch>,
    pub(crate) nodes: BTreeMap<NodeKey, (Point3, Param)>,
}

impl PatchSet {
    /// The patch of `t`.
    pub fn get(&self, t: Target) -> Option<&Patch> {
        self.patches.get(&t)
    }

    /// Position of a node.
    pub fn point(&self, k: NodeKey) -> Option<Point3> {
        self.nodes.get(&k).map(|n| n.0)
    }

    /// Position and parameter of a node.
    pub fn node(&self, k: NodeKey) -> Option<(Point3, Param)> {
        self.nodes.get(&k).copied()
    }

    /// Every patch in target order.
    pub fn patches(&self) -> impl Iterator<Item = &Patch> {
        self.patches.values()
    }

    /// Every node in key order.
    pub fn nodes(&self) -> impl Iterator<Item = (NodeKey, Point3, Param)> + '_ {
        self.nodes.iter().map(|(k, v)| (*k, v.0, v.1))
    }

    /// Number of nodes.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }
}
