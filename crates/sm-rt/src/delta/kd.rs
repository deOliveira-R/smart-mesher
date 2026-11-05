//! Build of the kd partition and validation of a decoded one.

use super::{F_PRE, JumpIndex, KdNode, LEAF, NO, NodeRec, Params, Partition};
use crate::bvh::{Box6, EMPTY_BOX, union};
use crate::core::{C_CONVEX, Core, NONE};
use sm_rt_api::{BuildIssue, MaterialSetId, MaterialSets, RegionId, Result, RtError};
use std::cell::RefCell;

/// Candidate and containment margins, in units of the linear tolerance.
const PAD_TOL: f64 = 2.0;
/// The root box is the cell boxes padded by this many tolerances.
const ROOT_PAD_TOL: f64 = 4.0;
/// Subtrees larger than this many candidates are built in parallel.
const PAR_CANDS: usize = 2048;
/// Hard cap on the tree depth (a box cannot be split below the tolerance anyway).
const DEPTH_CAP: u32 = 48;

fn bad(msg: &str) -> RtError {
    RtError::CacheCorrupt(msg.to_owned())
}

fn issue(code: &'static str, msg: String) -> RtError {
    RtError::Build(vec![BuildIssue::new(code, msg)])
}

thread_local! {
    /// Region stamps for counting distinct regions without allocating per call.
    static STAMPS: RefCell<Stamps> = const { RefCell::new(Stamps { lo: Vec::new(), hi: Vec::new(), gen_lo: 0, gen_hi: 0 }) };
}

struct Stamps {
    lo: Vec<u32>,
    hi: Vec<u32>,
    gen_lo: u32,
    gen_hi: u32,
}

impl Stamps {
    fn ensure(&mut self, n: usize) {
        if self.lo.len() < n {
            self.lo.resize(n, 0);
            self.hi.resize(n, 0);
        }
    }
    fn next_lo(&mut self) -> u32 {
        if self.gen_lo == u32::MAX {
            self.lo.iter_mut().for_each(|x| *x = 0);
            self.gen_lo = 0;
        }
        self.gen_lo += 1;
        self.gen_lo
    }
    fn next_hi(&mut self) -> u32 {
        if self.gen_hi == u32::MAX {
            self.hi.iter_mut().for_each(|x| *x = 0);
            self.gen_hi = 0;
        }
        self.gen_hi += 1;
        self.gen_hi
    }
}

struct Ctx<'a> {
    core: &'a Core,
    nreg: usize,
    complement: Option<u32>,
    tol: f64,
    leaf_cells: usize,
    max_depth: u32,
}

/// Intermediate tree.
struct Tmp {
    is_node: bool,
    ncells: u32,
    regions: Vec<u32>,
    pre: Option<u32>,
    cands: Vec<u32>,
    split: Option<(u8, f64, Box<Tmp>, Box<Tmp>)>,
}

impl Ctx<'_> {
    fn pad(&self) -> f64 {
        PAD_TOL * self.tol
    }

    fn meets(&self, c: u32, bx: &Box6) -> bool {
        let core = self.core;
        let cu = c as usize;
        let b = &core.c_bbox[cu];
        let pad = self.pad();
        for k in 0..3 {
            if b[k] - pad > bx[3 + k] || b[3 + k] + pad < bx[k] {
                return false;
            }
        }
        if core.c_flags[cu] & C_CONVEX != 0 {
            for &f in core.cell_faces(cu) {
                let pl = core.f_plane[f as usize];
                let s = core.sign(cu, f as usize);
                // Largest slack over the corners: minimise (s n) . x.
                let mut min_dot = 0.0;
                for k in 0..3 {
                    let sn = s * pl[k];
                    min_dot += sn * if sn > 0.0 { bx[k] } else { bx[3 + k] };
                }
                if s * pl[3] - min_dot < -pad {
                    return false;
                }
            }
        }
        true
    }

    /// First candidate (lowest id) whose convex body contains the box with a margin.
    fn preassigned(&self, bx: &Box6, cands: &[u32]) -> Option<u32> {
        let core = self.core;
        let margin = self.pad();
        'cell: for &c in cands {
            let cu = c as usize;
            if core.c_flags[cu] & C_CONVEX == 0 {
                continue;
            }
            let b = &core.c_bbox[cu];
            if (0..3).any(|k| b[k] > bx[k] || b[3 + k] < bx[3 + k]) {
                continue;
            }
            for &f in core.cell_faces(cu) {
                let pl = core.f_plane[f as usize];
                let s = core.sign(cu, f as usize);
                // Smallest slack over the corners: maximise (s n) . x.
                let mut max_dot = 0.0;
                for k in 0..3 {
                    let sn = s * pl[k];
                    max_dot += sn * if sn > 0.0 { bx[3 + k] } else { bx[k] };
                }
                if s * pl[3] - max_dot < margin {
                    continue 'cell;
                }
            }
            return Some(c);
        }
        None
    }

    fn distinct_regions(&self, cands: &[u32]) -> usize {
        STAMPS.with(|s| {
            let s = &mut *s.borrow_mut();
            s.ensure(self.nreg);
            let g = s.next_lo();
            let mut n = 0;
            for &c in cands {
                let r = self.core.c_region[c as usize] as usize;
                if s.lo[r] != g {
                    s.lo[r] = g;
                    n += 1;
                }
            }
            n
        })
    }

    fn regions_of(&self, cands: &[u32]) -> Vec<u32> {
        let mut v: Vec<u32> = cands.iter().map(|&c| self.core.c_region[c as usize]).collect();
        v.sort_unstable();
        v.dedup();
        v
    }

    /// The split plane: 8 equally spaced planes per axis, cost = sum over the children of
    /// volume fraction x (candidates + region spread); ties go to the lowest axis then position.
    fn choose(&self, bx: &Box6, cands: &[u32]) -> Option<(u8, f64)> {
        let core = self.core;
        let pad = self.pad();
        let mut best: Option<(f64, u8, f64)> = None;
        STAMPS.with(|s| {
            let s = &mut *s.borrow_mut();
            s.ensure(self.nreg);
            for ax in 0..3usize {
                let (lo, hi) = (bx[ax], bx[3 + ax]);
                let w = hi - lo;
                if w.is_nan() || w <= 16.0 * self.tol {
                    continue;
                }
                for k in 0..8 {
                    let at = lo + w * (k + 1) as f64 / 9.0;
                    let (gl, gh) = (s.next_lo(), s.next_hi());
                    let (mut nl, mut nh, mut sl, mut sh) = (0usize, 0usize, 0usize, 0usize);
                    for &c in cands {
                        let b = &core.c_bbox[c as usize];
                        let r = core.c_region[c as usize] as usize;
                        if b[ax] - pad <= at {
                            nl += 1;
                            if s.lo[r] != gl {
                                s.lo[r] = gl;
                                sl += 1;
                            }
                        }
                        if b[3 + ax] + pad >= at {
                            nh += 1;
                            if s.hi[r] != gh {
                                s.hi[r] = gh;
                                sh += 1;
                            }
                        }
                    }
                    let vf = (at - lo) / w;
                    let cost = vf * (nl + sl) as f64 + (1.0 - vf) * (nh + sh) as f64;
                    if best.is_none_or(|b| cost < b.0) {
                        best = Some((cost, ax as u8, at));
                    }
                }
            }
        });
        best.map(|b| (b.1, b.2))
    }

    fn children(&self, bx: &Box6, cands: &[u32], ax: usize, at: f64) -> (Box6, Vec<u32>, Box6, Vec<u32>) {
        let mut lbx = *bx;
        lbx[3 + ax] = at;
        let mut hbx = *bx;
        hbx[ax] = at;
        let lc: Vec<u32> = cands.iter().copied().filter(|&c| self.meets(c, &lbx)).collect();
        let hc: Vec<u32> = cands.iter().copied().filter(|&c| self.meets(c, &hbx)).collect();
        (lbx, lc, hbx, hc)
    }

    fn leaf(&self, cands: Vec<u32>, pre: Option<u32>) -> Tmp {
        match pre {
            Some(c) => Tmp { is_node: false, ncells: 1, regions: Vec::new(), pre, cands: vec![c], split: None },
            None => Tmp { is_node: false, ncells: cands.len() as u32, regions: Vec::new(), pre: None, cands, split: None },
        }
    }

    /// Location-only growth below (or at) a node leaf: split by candidate count alone.
    fn grow_loc(&self, bx: &Box6, cands: Vec<u32>, depth: u32, pre: Option<u32>) -> Tmp {
        if pre.is_some() || cands.len() <= self.leaf_cells || depth >= self.max_depth {
            return self.leaf(cands, pre);
        }
        let Some((ax, at)) = self.choose(bx, &cands) else { return self.leaf(cands, None) };
        let (lbx, lc, hbx, hc) = self.children(bx, &cands, ax as usize, at);
        if lc.len() == cands.len() && hc.len() == cands.len() {
            return self.leaf(cands, None);
        }
        let (lpre, hpre) = (self.preassigned(&lbx, &lc), self.preassigned(&hbx, &hc));
        let n = cands.len();
        let (lo, hi) = if n > PAR_CANDS {
            rayon::join(|| self.grow_loc(&lbx, lc, depth + 1, lpre), || self.grow_loc(&hbx, hc, depth + 1, hpre))
        } else {
            (self.grow_loc(&lbx, lc, depth + 1, lpre), self.grow_loc(&hbx, hc, depth + 1, hpre))
        };
        Tmp { is_node: false, ncells: n as u32, regions: Vec::new(), pre: None, cands: Vec::new(), split: Some((ax, at, Box::new(lo), Box::new(hi))) }
    }

    /// Partition-node growth: split while the candidates hold more than one region and more than
    /// `leaf_cells` cells; the node that stops continues with [`Ctx::grow_loc`].
    fn grow(&self, bx: &Box6, cands: Vec<u32>, depth: u32) -> Tmp {
        let pre = self.preassigned(bx, &cands);
        let stop = pre.is_some() || cands.len() <= self.leaf_cells || depth >= self.max_depth || self.distinct_regions(&cands) <= 1;
        if !stop && let Some((ax, at)) = self.choose(bx, &cands) {
            let (lbx, lc, hbx, hc) = self.children(bx, &cands, ax as usize, at);
            if !(lc.len() == cands.len() && hc.len() == cands.len()) {
                let n = cands.len();
                let (lo, hi) = if n > PAR_CANDS {
                    rayon::join(|| self.grow(&lbx, lc, depth + 1), || self.grow(&hbx, hc, depth + 1))
                } else {
                    (self.grow(&lbx, lc, depth + 1), self.grow(&hbx, hc, depth + 1))
                };
                let mut regions = lo.regions.clone();
                regions.extend_from_slice(&hi.regions);
                regions.sort_unstable();
                regions.dedup();
                return Tmp { is_node: true, ncells: n as u32, regions, pre: None, cands: Vec::new(), split: Some((ax, at, Box::new(lo), Box::new(hi))) };
            }
        }
        // A node leaf: its set is the regions of its candidates, plus the complement unless the
        // box lies inside one cell.
        let mut regions = match pre {
            Some(c) => vec![self.core.c_region[c as usize]],
            None => self.regions_of(&cands),
        };
        if pre.is_none()
            && let Some(cr) = self.complement
        {
            regions.push(cr);
            regions.sort_unstable();
            regions.dedup();
        }
        let n = if pre.is_some() { 1 } else { cands.len() as u32 };
        let mut t = self.grow_loc(bx, cands, depth, pre);
        t.is_node = true;
        t.ncells = n;
        t.regions = regions;
        t
    }
}

fn flatten(t: &Tmp, kd: &mut Vec<KdNode>, cands: &mut Vec<u32>, node_regions: &mut Vec<Vec<u32>>, node_cells: &mut Vec<u32>) {
    let idx = kd.len();
    let node = if t.is_node {
        node_regions.push(t.regions.clone());
        node_cells.push(t.ncells);
        (node_regions.len() - 1) as u32
    } else {
        NO
    };
    kd.push(KdNode { split: 0.0, axis: LEAF, flags: 0, node, a: 0, b: 0 });
    match &t.split {
        None => {
            kd[idx].a = cands.len() as u32;
            kd[idx].b = t.cands.len() as u32;
            kd[idx].flags = if t.pre.is_some() { F_PRE } else { 0 };
            cands.extend_from_slice(&t.cands);
        }
        Some((ax, at, lo, hi)) => {
            flatten(lo, kd, cands, node_regions, node_cells);
            let hi_idx = kd.len() as u32;
            flatten(hi, kd, cands, node_regions, node_cells);
            kd[idx].axis = *ax;
            kd[idx].split = *at;
            kd[idx].a = hi_idx;
        }
    }
}

/// The decoded or freshly built arrays of a partition, before validation and derivation.
pub(crate) struct Raw {
    pub params: Params,
    pub root_box: [f64; 6],
    pub kd: Vec<KdNode>,
    pub cands: Vec<u32>,
    pub node_set: Vec<u32>,
    pub node_cells: Vec<u32>,
    /// Regions of each set, in set-id order.
    pub set_lists: Vec<Vec<u32>>,
}

impl Partition {
    /// Build the partition of `core`. `nreg` is the region count, `complement` the complement
    /// region (when there is one), `tol` the linear tolerance.
    pub(crate) fn build(core: &Core, nreg: usize, complement: Option<u32>, tol: f64, params: Params) -> Result<Self> {
        let nc = core.n_cells();
        if nc == 0 {
            return Err(issue("partition.empty", "the mesh has no cells to partition".into()));
        }
        let mut bb = EMPTY_BOX;
        for b in &core.c_bbox {
            bb = union(&bb, b);
        }
        let pad = ROOT_PAD_TOL * tol;
        let root_box = [bb[0] - pad, bb[1] - pad, bb[2] - pad, bb[3] + pad, bb[4] + pad, bb[5] + pad];
        if !root_box.iter().all(|x| x.is_finite()) {
            return Err(issue("partition.bounds", "the model box is not finite".into()));
        }
        let ctx = Ctx { core, nreg, complement, tol, leaf_cells: params.leaf_cells.max(1) as usize, max_depth: params.max_depth.min(DEPTH_CAP) };
        let all: Vec<u32> = (0..nc as u32).collect();
        let tree = ctx.grow(&root_box, all, 0);
        let (mut kd, mut cands, mut node_regions, mut node_cells) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        flatten(&tree, &mut kd, &mut cands, &mut node_regions, &mut node_cells);
        // Sets: nodes in ascending order, then the singleton of every region.
        let mut sets = MaterialSets::new();
        let node_set: Vec<u32> = node_regions.iter().map(|r| sets.intern(r.iter().map(|&x| RegionId(x))).0).collect();
        for r in 0..nreg {
            sets.intern([RegionId::new(r)]);
        }
        let set_lists = (0..sets.len()).map(|s| sets.get(MaterialSetId::new(s)).unwrap_or(&[]).iter().map(|r| r.0).collect()).collect();
        Self::from_raw(Raw { params, root_box, kd, cands, node_set, node_cells, set_lists }, core, nreg)
    }

    /// Validate the arrays (hostile input safe) and derive node boxes, parents, flags, the
    /// cell-to-node table and the region-jump index. Validation covers the structure (counts,
    /// links, ranges) and the meaning that does not need the mesh geometry: node sets cover the
    /// regions of their candidate cells and their children's sets, and leaf cell counts match
    /// their candidate lists. It does not re-derive that a candidate list is geometrically
    /// complete (a cell dropped from a leaf it meets is not detected), so a hostile image can
    /// make `locate` answer wrongly but never read out of range.
    pub(crate) fn from_raw(raw: Raw, core: &Core, nreg: usize) -> Result<Self> {
        let Raw { params, root_box, kd, cands, node_set, node_cells, set_lists } = raw;
        let nc = core.n_cells();
        let nk = kd.len();
        if nk == 0 || nk > u32::MAX as usize / 2 {
            return Err(bad("partition has no nodes"));
        }
        if !(root_box.iter().all(|x| x.is_finite()) && (0..3).all(|k| root_box[k] < root_box[3 + k])) {
            return Err(bad("partition box"));
        }
        // Sets.
        let mut sets = MaterialSets::new();
        for (i, l) in set_lists.iter().enumerate() {
            if l.iter().any(|&r| r as usize >= nreg) || l.windows(2).any(|w| w[0] >= w[1]) {
                return Err(bad("material set"));
            }
            if sets.intern(l.iter().map(|&r| RegionId(r))).get() != i {
                return Err(bad("material set order"));
            }
        }
        let mut region_set = vec![NONE; nreg];
        for (r, slot) in region_set.iter_mut().enumerate() {
            *slot = sets.intern([RegionId::new(r)]).0;
        }
        if sets.len() != set_lists.len() {
            return Err(bad("singleton sets are missing"));
        }
        if node_set.len() != node_cells.len() || node_set.iter().any(|&s| s as usize >= sets.len()) {
            return Err(bad("node tables"));
        }
        // kd structure: children after parents, each node referenced once.
        let mut refs = vec![0u8; nk];
        for (i, n) in kd.iter().enumerate() {
            if n.axis == LEAF {
                let end = u64::from(n.a) + u64::from(n.b);
                if end > cands.len() as u64 || n.flags & !F_PRE != 0 || (n.flags & F_PRE != 0 && n.b != 1) {
                    return Err(bad("kd leaf"));
                }
                let l = &cands[n.a as usize..end as usize];
                if l.iter().any(|&c| c as usize >= nc) || l.windows(2).any(|w| w[0] >= w[1]) {
                    return Err(bad("candidate list"));
                }
            } else {
                let hi = n.a as usize;
                if n.axis > 2 || !n.split.is_finite() || i + 1 >= nk || hi <= i + 1 || hi >= nk {
                    return Err(bad("kd split"));
                }
                refs[i + 1] = refs[i + 1].saturating_add(1);
                refs[hi] = refs[hi].saturating_add(1);
            }
        }
        if refs[0] != 0 || refs[1..].iter().any(|&r| r != 1) {
            return Err(bad("kd links"));
        }
        // External faces per cell.
        let mut ext = vec![false; nc];
        for f in 0..core.n_faces() {
            let (l, r) = (core.f_left[f], core.f_right[f]);
            if l == NONE && r != NONE {
                ext[r as usize] = true;
            } else if r == NONE && l != NONE {
                ext[l as usize] = true;
            }
        }
        // Traverse: node boxes, parents, leaf flags, boundary flags, cell -> node.
        let nn = node_set.len();
        let mut nodes: Vec<NodeRec> = Vec::with_capacity(nn);
        let mut cell_node = vec![NONE; nc];
        // (owner node-leaf, cell) over every kd leaf below it, for the meaning checks below.
        let mut owned: Vec<(u32, u32)> = Vec::new();
        // (kd index, box, parent node, owner node-leaf)
        let mut stack: Vec<(usize, [f64; 6], u32, u32)> = vec![(0, root_box, NO, NO)];
        while let Some((k, bx, parent, owner)) = stack.pop() {
            let n = kd[k];
            let (mut my_parent, mut my_owner) = (parent, owner);
            if n.node != NO {
                if n.node as usize != nodes.len() || nodes.len() >= nn {
                    return Err(bad("node ids"));
                }
                let leaf = n.axis == LEAF || kd[k + 1].node == NO;
                let id = n.node;
                nodes.push(NodeRec { kd: k as u32, parent, set: node_set[id as usize], cells: node_cells[id as usize], bounds: bx, leaf, boundary: false });
                my_parent = id;
                my_owner = if leaf { id } else { NO };
            } else if parent == NO || owner == NO {
                return Err(bad("location node outside a leaf"));
            }
            if n.axis == LEAF {
                if my_owner == NO {
                    return Err(bad("kd leaf outside a node leaf"));
                }
                for &c in &cands[n.a as usize..(n.a + n.b) as usize] {
                    owned.push((my_owner, c));
                    if ext[c as usize] {
                        nodes[my_owner as usize].boundary = true;
                    }
                    if cell_node[c as usize] == NONE {
                        cell_node[c as usize] = my_owner;
                    }
                }
            } else {
                let ax = n.axis as usize;
                let (ln, hn) = (kd[k + 1].node != NO, kd[n.a as usize].node != NO);
                if ln != hn || (n.node == NO && ln) {
                    return Err(bad("node children"));
                }
                if !(bx[ax] < n.split && n.split < bx[3 + ax]) {
                    return Err(bad("kd split outside its box"));
                }
                let (mut lbx, mut hbx) = (bx, bx);
                lbx[3 + ax] = n.split;
                hbx[ax] = n.split;
                // Preorder: the lo child must be processed first.
                stack.push((n.a as usize, hbx, my_parent, my_owner));
                stack.push((k + 1, lbx, my_parent, my_owner));
            }
        }
        if nodes.len() != nn {
            return Err(bad("node count"));
        }
        // Meaning, not only structure. Every node set holds the regions of the candidate cells
        // of every kd leaf below a leaf node, a node that is itself a kd leaf has exactly that
        // many cells (one when preassigned), and a parent's set covers its children's.
        owned.sort_unstable();
        owned.dedup();
        let mut distinct = vec![0u32; nn];
        for &(owner, c) in &owned {
            distinct[owner as usize] += 1;
            let regs = sets.get(MaterialSetId(nodes[owner as usize].set)).unwrap_or(&[]);
            if regs.binary_search(&RegionId(core.c_region[c as usize])).is_err() {
                return Err(bad("a node set misses the region of one of its candidate cells"));
            }
        }
        for (i, n) in nodes.iter().enumerate() {
            if n.leaf {
                let k = kd[n.kd as usize];
                let ok = if k.axis == LEAF {
                    n.cells == if k.flags & F_PRE != 0 { 1 } else { k.b }
                } else {
                    distinct[i] <= n.cells && n.cells as usize <= nc
                };
                if !ok {
                    return Err(bad("node cell count does not match its candidates"));
                }
            }
            if n.parent != NO {
                let ps = sets.get(MaterialSetId(nodes[n.parent as usize].set)).unwrap_or(&[]);
                let cs = sets.get(MaterialSetId(n.set)).unwrap_or(&[]);
                if cs.iter().any(|r| ps.binary_search(r).is_err()) {
                    return Err(bad("a node set is not covered by its parent's"));
                }
            }
        }
        let jump = if params.region_jump { Some(JumpIndex::build(core, nreg, 4)) } else { None };
        Ok(Self { params, root_box, kd, cands, nodes, sets, region_set, cell_node, jump })
    }
}

#[cfg(test)]
mod tests {
    //! `from_raw` checks the meaning of a decoded partition, not only its structure.
    use super::*;
    use crate::{MeshTracker, MeshTrackerBuilder};
    use sm_rt_api::{BoundarySpec, BuildInput, BuildOptions, ComplementPolicy, MeshInput, PartitionOptions, RegionRule};
    use sm_testkit::meshes::hex_block_two_volumes;

    fn tracker() -> MeshTracker {
        let fx = hex_block_two_volumes([8, 4, 4], [1.0; 3], 4).unwrap();
        let (rules, bnd, comp) = (RegionRule::ByVolumeClassification, BoundarySpec::default(), ComplementPolicy::Outside);
        let mut p = PartitionOptions::default();
        p.enabled = true;
        p.leaf_cells = 4;
        let input = BuildInput::Mesh(MeshInput::new(&fx.mesh, &rules, &bnd, &comp));
        MeshTrackerBuilder.build_concrete(&input, &BuildOptions::default().with_partition(p)).unwrap().0
    }

    fn raw_of(p: &Partition) -> Raw {
        Raw {
            params: p.params,
            root_box: p.root_box,
            kd: p.kd.clone(),
            cands: p.cands.clone(),
            node_set: p.nodes.iter().map(|n| n.set).collect(),
            node_cells: p.nodes.iter().map(|n| n.cells).collect(),
            set_lists: (0..p.sets.len()).map(|s| p.sets.get(MaterialSetId::new(s)).unwrap().iter().map(|r| r.0).collect()).collect(),
        }
    }

    fn corrupt(r: Result<Partition>) -> bool {
        matches!(r, Err(RtError::CacheCorrupt(_)))
    }

    #[test]
    fn a_meaningless_but_well_formed_partition_is_cache_corrupt() {
        let t = tracker();
        let p = t.part.as_ref().unwrap();
        let nreg = t.regions.0.len();
        assert_eq!(nreg, 2);
        let singleton = |r: u32| p.region_set[r as usize];
        let sets_of = |n: &NodeRec| p.sets.get(MaterialSetId(n.set)).unwrap().to_vec();
        // The unmodified arrays round trip.
        let ok = Partition::from_raw(raw_of(p), &t.core, nreg).unwrap();
        assert_eq!(ok.content_hash(), p.content_hash());
        // A leaf that holds both regions, with a set that lists only one.
        let both = p.nodes.iter().position(|n| n.leaf && sets_of(n).len() == 2).expect("an interface leaf");
        let mut raw = raw_of(p);
        raw.node_set[both] = singleton(0);
        assert!(corrupt(Partition::from_raw(raw, &t.core, nreg)), "set misses a candidate region");
        // A single-region leaf given the other region's set.
        let one = p.nodes.iter().position(|n| n.leaf && sets_of(n) == [RegionId(0)]).expect("a pure leaf");
        let mut raw = raw_of(p);
        raw.node_set[one] = singleton(1);
        assert!(corrupt(Partition::from_raw(raw, &t.core, nreg)));
        // A node leaf that is itself a kd leaf with the wrong cell count.
        let kd_leaf = p.nodes.iter().position(|n| n.leaf && p.kd[n.kd as usize].axis == LEAF && p.kd[n.kd as usize].flags & F_PRE == 0).expect("a plain kd leaf");
        let mut raw = raw_of(p);
        raw.node_cells[kd_leaf] += 1;
        assert!(corrupt(Partition::from_raw(raw, &t.core, nreg)), "cell count");
        // A preassigned leaf must count one cell.
        if let Some(pre) = p.nodes.iter().position(|n| n.leaf && p.kd[n.kd as usize].flags & F_PRE != 0) {
            let mut raw = raw_of(p);
            raw.node_cells[pre] = 2;
            assert!(corrupt(Partition::from_raw(raw, &t.core, nreg)));
        }
        // An internal node whose set does not cover a child's.
        let inner = p.nodes.iter().position(|n| !n.leaf && sets_of(n).len() == 2).expect("an internal node over both regions");
        let child = p.nodes.iter().position(|n| n.parent == inner as u32 && sets_of(n).len() == 2);
        if child.is_some() {
            let mut raw = raw_of(p);
            raw.node_set[inner] = singleton(0);
            assert!(corrupt(Partition::from_raw(raw, &t.core, nreg)), "parent set must cover the child's");
        }
    }
}
