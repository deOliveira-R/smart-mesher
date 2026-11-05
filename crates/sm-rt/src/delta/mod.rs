//! The majorant partition and delta flights over a meshed leaf (`MajorantTracker`).
//!
//! * [`Partition`] is a kd tree over the model box (padded by a few tolerances) whose leaves
//!   tile it; every leaf lists the cells that may meet it (CSR) and the regions they carry (a
//!   deduplicated material set). The split rule never looks at a cross section: split while the
//!   candidates hold more than one region and more than `leaf_cells` cells. Inside a node that
//!   stopped splitting, a *location-only* subtree (not nodes, no `NodeId`) keeps splitting by
//!   cell count, so a hinted locate tests few cells even in a large one-region leaf.
//! * `impl MajorantTracker for MeshTracker` (in `flight`) produces surface and delta segments,
//!   locates collisions from the owning node, and gives the safety distance.
//! * `jump` overrides `fire` with a per-region face BVH when `PartitionOptions::region_jump`.
//! * `io` writes and reads the partition as `.smrt` sections `Private(5..8)`.
//!
//! Cross sections never enter: the caller passes one majorant per node per call.

pub(crate) mod flight;
mod io;
mod jump;
mod kd;

pub(crate) use io::{read_sections, write_sections};
pub(crate) use jump::{JumpIndex, fire as jump_fire};

use crate::core::NONE;
use sm_core::Aabb;
use sm_rt_api::{CellId, ContentHash, ContentHasher, HierarchyView, MaterialSetId, MaterialSets, NodeId, NodeInfo, NodeKind, RegionId};

/// "No node / no child".
pub(crate) const NO: u32 = u32::MAX;
/// kd leaf flag: the leaf box lies inside the one candidate cell.
pub(crate) const F_PRE: u8 = 1;
/// kd axis value of a leaf.
pub(crate) const LEAF: u8 = 3;

/// One kd node in preorder (the lo child is `index + 1`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct KdNode {
    /// Split value (internal nodes).
    pub split: f64,
    /// `0..3` split axis, [`LEAF`] for a leaf.
    pub axis: u8,
    /// Leaf flags ([`F_PRE`]).
    pub flags: u8,
    /// The `NodeId` when this kd node is a partition node, else [`NO`] (location-only).
    pub node: u32,
    /// Internal: index of the hi child. Leaf: first candidate slot.
    pub a: u32,
    /// Leaf: candidate count.
    pub b: u32,
}

/// One partition node (an entry of the prototype tree).
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct NodeRec {
    pub kd: u32,
    pub parent: u32,
    pub set: u32,
    pub cells: u32,
    pub bounds: [f64; 6],
    pub leaf: bool,
    /// Some candidate cell of the node has a face with no cell behind it (model boundary or
    /// complement), so a delta segment must watch for it.
    pub boundary: bool,
}

/// Build parameters kept with the partition (they are hashed with it).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Params {
    pub leaf_cells: u32,
    pub max_depth: u32,
    pub region_jump: bool,
}

/// The disjoint majorant partition of one meshed leaf.
#[derive(Debug)]
pub(crate) struct Partition {
    pub params: Params,
    pub root_box: [f64; 6],
    pub kd: Vec<KdNode>,
    pub cands: Vec<u32>,
    pub nodes: Vec<NodeRec>,
    pub sets: MaterialSets,
    pub region_set: Vec<u32>,
    pub(crate) cell_node: Vec<u32>,
    pub jump: Option<JumpIndex>,
}

/// Structure facts of a partition: no cross section and no geometry beyond counts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PartitionStats {
    pub nodes: usize,
    pub leaves: usize,
    /// Edges on the longest root-to-leaf path of the partition nodes.
    pub depth: u32,
    pub preassigned: usize,
    pub sets: usize,
    pub cells_min: u32,
    pub cells_max: u32,
    pub cells_mean: f64,
    pub bytes: usize,
}

impl Partition {
    /// Counts for `rt.partition`.
    pub fn stats(&self) -> PartitionStats {
        let mut depth = vec![0u32; self.nodes.len()];
        let (mut max_depth, mut leaves, mut pre) = (0u32, 0usize, 0usize);
        let (mut cmin, mut cmax, mut csum) = (u32::MAX, 0u32, 0u64);
        for (i, n) in self.nodes.iter().enumerate() {
            // Nodes are in preorder: a parent has a lower index.
            depth[i] = if n.parent == NO { 0 } else { depth[n.parent as usize] + 1 };
            max_depth = max_depth.max(depth[i]);
            if n.leaf {
                leaves += 1;
                pre += usize::from(self.kd[n.kd as usize].flags & F_PRE != 0);
                cmin = cmin.min(n.cells);
                cmax = cmax.max(n.cells);
                csum += u64::from(n.cells);
            }
        }
        PartitionStats {
            nodes: self.nodes.len(),
            leaves,
            depth: max_depth,
            preassigned: pre,
            sets: self.sets.len(),
            cells_min: if leaves == 0 { 0 } else { cmin },
            cells_max: cmax,
            cells_mean: if leaves == 0 { 0.0 } else { csum as f64 / leaves as f64 },
            bytes: self.bytes(),
        }
    }

    /// Heap bytes.
    pub fn bytes(&self) -> usize {
        self.kd.len() * core::mem::size_of::<KdNode>()
            + self.cands.len() * 4
            + self.nodes.len() * core::mem::size_of::<NodeRec>()
            + self.cell_node.len() * 4
            + self.jump.as_ref().map_or(0, JumpIndex::bytes)
    }

    /// Hash of every stored array (the partition's own identity; independent of thread count).
    pub fn content_hash(&self) -> ContentHash {
        let mut h = ContentHasher::new();
        h.write_u64(u64::from(self.params.leaf_cells));
        h.write_u64(u64::from(self.params.max_depth));
        h.write_u64(u64::from(self.params.region_jump));
        for x in self.root_box {
            h.write_f64(x);
        }
        h.write_u64(self.kd.len() as u64);
        for n in &self.kd {
            h.write_f64(n.split);
            for v in [u32::from(n.axis), u32::from(n.flags), n.node, n.a, n.b] {
                h.write_u64(u64::from(v));
            }
        }
        h.write_u64(self.cands.len() as u64);
        for &c in &self.cands {
            h.write_u64(u64::from(c));
        }
        h.write_u64(self.nodes.len() as u64);
        for n in &self.nodes {
            h.write_u64(u64::from(n.set));
            h.write_u64(u64::from(n.cells));
        }
        h.write_u64(self.sets.len() as u64);
        for s in 0..self.sets.len() {
            let r = self.sets.get(MaterialSetId::new(s)).unwrap_or(&[]);
            h.write_u64(r.len() as u64);
            for x in r {
                h.write_u64(u64::from(x.0));
            }
        }
        h.finish()
    }

    /// Descend from kd index `k` (box `bx`) to the kd leaf holding `p`; a point on a split goes
    /// to the child the direction enters (`d == 0`: the lower child). Returns the kd leaf, the
    /// deepest partition node on the way and that node's box. `visits` counts the nodes touched.
    #[inline]
    pub fn descend(&self, mut k: usize, mut bx: [f64; 6], p: [f64; 3], d: [f64; 3], visits: &mut u64) -> (usize, u32, [f64; 6]) {
        let mut node = NO;
        let mut node_box = bx;
        loop {
            let n = &self.kd[k];
            *visits += 1;
            if n.node != NO {
                node = n.node;
                node_box = bx;
            }
            if n.axis == LEAF {
                return (k, node, node_box);
            }
            let ax = n.axis as usize;
            if p[ax] > n.split || (p[ax] == n.split && d[ax] > 0.0) {
                bx[ax] = n.split;
                k = n.a as usize;
            } else {
                bx[3 + ax] = n.split;
                k += 1;
            }
        }
    }

    /// Candidate cells of a kd leaf (ascending).
    #[inline]
    pub fn leaf_cands(&self, k: usize) -> &[u32] {
        let n = &self.kd[k];
        &self.cands[n.a as usize..(n.a + n.b) as usize]
    }
}

impl HierarchyView for Partition {
    fn node_count(&self) -> usize {
        self.nodes.len()
    }
    fn root(&self) -> NodeId {
        NodeId::new(0)
    }
    fn node(&self, n: NodeId) -> Option<NodeInfo> {
        let r = self.nodes.get(n.get())?;
        let b = r.bounds;
        let kind = if !r.leaf {
            NodeKind::Internal
        } else if r.cells == 1 {
            NodeKind::Homogeneous
        } else {
            NodeKind::Mesh
        };
        let parent = if r.parent == NO { NodeId::NONE } else { NodeId(r.parent) };
        let bounds = Aabb { lo: sm_core::Point3::new(b[0], b[1], b[2]), hi: sm_core::Point3::new(b[3], b[4], b[5]) };
        Some(NodeInfo::new(n, parent, bounds, kind, MaterialSetId(r.set), r.cells))
    }
    fn node_of_cell(&self, c: CellId) -> Option<NodeId> {
        self.cell_node.get(c.get()).copied().filter(|&n| n != NONE).map(NodeId)
    }
    fn set_count(&self) -> usize {
        self.sets.len()
    }
    fn set_regions(&self, s: MaterialSetId) -> Option<&[RegionId]> {
        self.sets.get(s)
    }
    fn region_set(&self, r: RegionId) -> Option<MaterialSetId> {
        self.region_set.get(r.get()).copied().filter(|&s| s != NONE).map(MaterialSetId)
    }
}

/// The hierarchy of a tracker without a partition: no nodes.
pub(crate) struct NoHierarchy;

impl HierarchyView for NoHierarchy {
    fn node_count(&self) -> usize {
        0
    }
    fn root(&self) -> NodeId {
        NodeId::NONE
    }
    fn node(&self, _: NodeId) -> Option<NodeInfo> {
        None
    }
    fn node_of_cell(&self, _: CellId) -> Option<NodeId> {
        None
    }
    fn set_count(&self) -> usize {
        0
    }
    fn set_regions(&self, _: MaterialSetId) -> Option<&[RegionId]> {
        None
    }
    fn region_set(&self, _: RegionId) -> Option<MaterialSetId> {
        None
    }
}
