//! Disjoint kd tiling over the cell boxes of a CSG leaf (W4-RT-CELLINDEX).
//!
//! The leaves of the tree tile a root box. Each tile lists the cells whose padded box meets it
//! (ascending) and the surfaces of those cells (ascending). A point is then classified against
//! the tile's cells only, and a complement ray visits tiles front to back and tests only their
//! surfaces. The index is derived data: it is not hashed and not cached, and a leaf without it
//! (fewer cells than the threshold) takes the all-cells, all-surfaces paths, which stay as the oracle.
//!
//! Cell boxes come from `term_box` (necessary conditions only), padded by `4 tol` so that a point
//! inside the on-face band of a surface still lands in the tile of every cell that could claim it.

use sm_rt_api::{ContentHash, ContentHasher};

/// Cells at or above which the index is built by default.
pub(crate) const DEFAULT_MIN_CELLS: usize = 64;
/// A tile with at most this many cells is not split.
const LEAF_CELLS: usize = 8;
/// Depth cap of the tree.
const MAX_DEPTH: u32 = 40;
/// Capacity of the traversal stack (more than the depth cap).
const STACK: usize = 64;

#[derive(Clone, Copy, Debug)]
struct Node {
    /// 0..3 split axis, `LEAF` for a tile.
    axis: u32,
    split: f64,
    /// Right child (node index) for a split, tile index for a leaf. The left child is `self + 1`.
    aux: u32,
}
const LEAF: u32 = 3;

/// The kd tiling.
pub(crate) struct CellIndex {
    /// `[lo x, lo y, lo z, hi x, hi y, hi z]`.
    root: [f64; 6],
    nodes: Vec<Node>,
    cell_off: Vec<u32>,
    cells: Vec<u32>,
    surf_off: Vec<u32>,
    surfs: Vec<u32>,
}

/// Build state.
struct Build<'a> {
    ix: CellIndex,
    cbox: Vec<[f64; 6]>,
    cell_surfs: &'a dyn Fn(usize) -> &'a [u32],
}

impl CellIndex {
    /// Build over the raw cell boxes (infinite sides allowed; an unbounded cell is a candidate
    /// everywhere). `bounds` seeds the root box.
    pub(crate) fn build<'a>(raw: &[[f64; 6]], bounds: [f64; 6], tol: f64, cell_surfs: &'a dyn Fn(usize) -> &'a [u32]) -> Self {
        let pad = |v: f64| 4.0 * tol + 1e-14 * v.abs();
        let mut root = [0.0f64; 6];
        for k in 0..3 {
            root[k] = bounds[k] - pad(bounds[k]);
            root[3 + k] = bounds[3 + k] + pad(bounds[3 + k]);
        }
        let mut cbox: Vec<[f64; 6]> = Vec::with_capacity(raw.len());
        for b in raw {
            let mut c = *b;
            for k in 0..3 {
                if c[k].is_finite() {
                    c[k] -= pad(c[k]);
                    root[k] = root[k].min(c[k]);
                }
                if c[3 + k].is_finite() {
                    c[3 + k] += pad(c[3 + k]);
                    root[3 + k] = root[3 + k].max(c[3 + k]);
                }
            }
            cbox.push(c);
        }
        let ix = Self { root, nodes: Vec::new(), cell_off: vec![0], cells: Vec::new(), surf_off: vec![0], surfs: Vec::new() };
        let mut b = Build { ix, cbox, cell_surfs };
        let all: Vec<u32> = (0..raw.len() as u32).collect();
        b.split(root, all, 0);
        b.ix
    }

    /// Number of tiles.
    pub(crate) fn n_tiles(&self) -> usize {
        self.cell_off.len() - 1
    }

    /// Hash of the node order and the tile lists (determinism checks).
    pub(crate) fn digest(&self) -> ContentHash {
        let mut h = ContentHasher::new();
        for v in self.root {
            h.write_f64(v);
        }
        h.write_u64(self.nodes.len() as u64);
        for n in &self.nodes {
            h.write_u64(u64::from(n.axis));
            h.write_f64(n.split);
            h.write_u64(u64::from(n.aux));
        }
        for (off, list) in [(&self.cell_off, &self.cells), (&self.surf_off, &self.surfs)] {
            h.write_u64(list.len() as u64);
            for &o in off {
                h.write_u64(u64::from(o));
            }
            for &c in list {
                h.write_u64(u64::from(c));
            }
        }
        h.finish()
    }

    /// Is `p` inside the root box?
    pub(crate) fn contains_point(&self, p: [f64; 3]) -> bool {
        (0..3).all(|k| p[k] >= self.root[k] && p[k] <= self.root[3 + k])
    }

    fn tile_of(&self, p: [f64; 3]) -> usize {
        let mut i = 0usize;
        loop {
            let n = self.nodes[i];
            if n.axis == LEAF {
                return n.aux as usize;
            }
            i = if p[n.axis as usize] < n.split { i + 1 } else { n.aux as usize };
        }
    }

    /// Candidate cells at `p` (ascending), `None` when `p` is outside the root box.
    pub(crate) fn cells_at(&self, p: [f64; 3]) -> Option<&[u32]> {
        if !self.contains_point(p) {
            return None;
        }
        let t = self.tile_of(p);
        Some(&self.cells[self.cell_off[t] as usize..self.cell_off[t + 1] as usize])
    }

    /// Surfaces of the tile's cells (ascending).
    pub(crate) fn tile_surfs(&self, t: usize) -> &[u32] {
        &self.surfs[self.surf_off[t] as usize..self.surf_off[t + 1] as usize]
    }

    /// Visit the tiles met by the ray `o + t d`, `t` in `[0, t_end]`, front to back. `f` gets the
    /// tile and its exit parameter and returns `true` to stop. Tiles share their boundary
    /// parameter, so consecutive calls see non-decreasing exits.
    pub(crate) fn walk(&self, o: [f64; 3], d: [f64; 3], t_end: f64, mut f: impl FnMut(usize, f64) -> bool) {
        // Clip to the root box.
        let (mut t0, mut t1) = (0.0f64, t_end);
        for k in 0..3 {
            if d[k] == 0.0 {
                if o[k] < self.root[k] || o[k] > self.root[3 + k] {
                    return;
                }
            } else {
                let (a, b) = ((self.root[k] - o[k]) / d[k], (self.root[3 + k] - o[k]) / d[k]);
                t0 = t0.max(a.min(b));
                t1 = t1.min(a.max(b));
            }
        }
        if t0 > t1 {
            return;
        }
        let mut stack = [(0u32, 0.0f64, 0.0f64); STACK];
        let mut sp = 0usize;
        let (mut node, mut tmin, mut tmax) = (0usize, t0, t1);
        loop {
            let n = self.nodes[node];
            if n.axis == LEAF {
                if f(n.aux as usize, tmax) || sp == 0 {
                    return;
                }
                sp -= 1;
                let (nn, a, b) = stack[sp];
                (node, tmin, tmax) = (nn as usize, a, b);
                continue;
            }
            let k = n.axis as usize;
            let (left, right) = (node + 1, n.aux as usize);
            let (near, far, toward) = if o[k] < n.split {
                (left, right, d[k] > 0.0)
            } else if o[k] > n.split {
                (right, left, d[k] < 0.0)
            } else if d[k] > 0.0 {
                (right, right, false)
            } else {
                (left, left, false)
            };
            if !toward {
                node = near;
                continue;
            }
            let ts = (n.split - o[k]) / d[k];
            if ts >= tmax {
                node = near;
            } else if ts <= tmin {
                node = far;
            } else {
                // Depth is capped below STACK, so the push cannot overflow.
                stack[sp] = (far as u32, ts, tmax);
                sp += 1;
                node = near;
                tmax = ts;
            }
        }
    }
}

impl Build<'_> {
    fn leaf(&mut self, cells: &[u32]) {
        let t = self.ix.n_tiles() as u32;
        self.ix.nodes.push(Node { axis: LEAF, split: 0.0, aux: t });
        self.ix.cells.extend_from_slice(cells);
        self.ix.cell_off.push(self.ix.cells.len() as u32);
        let mut s: Vec<u32> = cells.iter().flat_map(|&c| (self.cell_surfs)(c as usize).iter().copied()).collect();
        s.sort_unstable();
        s.dedup();
        self.ix.surfs.extend_from_slice(&s);
        self.ix.surf_off.push(self.ix.surfs.len() as u32);
    }

    fn split(&mut self, bx: [f64; 6], cells: Vec<u32>, depth: u32) {
        if cells.len() <= LEAF_CELLS || depth >= MAX_DEPTH {
            return self.leaf(&cells);
        }
        // Best axis: smallest larger child, then smallest total, then lowest axis.
        let mut best: Option<(usize, usize, usize, f64)> = None;
        for k in 0..3 {
            let mut mids: Vec<f64> = cells
                .iter()
                .filter_map(|&c| {
                    let b = &self.cbox[c as usize];
                    (b[k].is_finite() && b[3 + k].is_finite()).then(|| 0.5 * (b[k].max(bx[k]) + b[3 + k].min(bx[3 + k])))
                })
                .collect();
            if mids.is_empty() {
                continue;
            }
            mids.sort_by(f64::total_cmp);
            let m = mids[mids.len() / 2];
            if !(m > bx[k] && m < bx[3 + k]) {
                continue;
            }
            let nl = cells.iter().filter(|&&c| self.cbox[c as usize][k] <= m).count();
            let nr = cells.iter().filter(|&&c| self.cbox[c as usize][3 + k] >= m).count();
            let key = (nl.max(nr), nl + nr);
            if best.is_none_or(|b| key < (b.1, b.2)) {
                best = Some((k, key.0, key.1, m));
            }
        }
        let Some((k, worst, _, m)) = best else { return self.leaf(&cells) };
        if worst >= cells.len() {
            return self.leaf(&cells);
        }
        let left: Vec<u32> = cells.iter().copied().filter(|&c| self.cbox[c as usize][k] <= m).collect();
        let right: Vec<u32> = cells.iter().copied().filter(|&c| self.cbox[c as usize][3 + k] >= m).collect();
        let at = self.ix.nodes.len();
        self.ix.nodes.push(Node { axis: k as u32, split: m, aux: 0 });
        let (mut lb, mut rb) = (bx, bx);
        lb[3 + k] = m;
        rb[k] = m;
        self.split(lb, left, depth + 1);
        self.ix.nodes[at].aux = self.ix.nodes.len() as u32;
        self.split(rb, right, depth + 1);
    }
}
