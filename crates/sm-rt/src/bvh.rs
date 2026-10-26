//! Deterministic bounding-volume hierarchy over index primitives.
//!
//! Nodes are stored in depth-first preorder (left child = `node + 1`), median split on the
//! longest centroid axis, ties broken by primitive id, so the build is a pure function of the
//! boxes and the leaf size. Traversals use a caller-provided stack (`Scratch::ids`), so a warmed
//! scratch makes them allocation free.

/// Axis-aligned box as `[lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]`.
pub(crate) type Box6 = [f64; 6];

pub(crate) const EMPTY_BOX: Box6 = [f64::INFINITY, f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];

#[inline]
pub(crate) fn union(a: &Box6, b: &Box6) -> Box6 {
    [a[0].min(b[0]), a[1].min(b[1]), a[2].min(b[2]), a[3].max(b[3]), a[4].max(b[4]), a[5].max(b[5])]
}

/// Squared distance from `p` to the box (0 inside).
#[inline]
pub(crate) fn box_dist2(b: &Box6, p: [f64; 3]) -> f64 {
    let mut s = 0.0;
    for k in 0..3 {
        let d = (b[k] - p[k]).max(p[k] - b[3 + k]).max(0.0);
        s += d * d;
    }
    s
}

/// Slab test of the ray `o + t d` against `b` padded by `pad`; returns the entry parameter
/// clipped to `[t0, t1]` or `None` when the interval is empty.
#[inline]
pub(crate) fn ray_box(o: [f64; 3], d: [f64; 3], inv: [f64; 3], b: &Box6, pad: f64, t0: f64, t1: f64) -> Option<f64> {
    let (mut lo, mut hi) = (t0, t1);
    for k in 0..3 {
        let (bl, bh) = (b[k] - pad, b[3 + k] + pad);
        if d[k] == 0.0 {
            if o[k] < bl || o[k] > bh {
                return None;
            }
        } else {
            let (mut a, mut c) = ((bl - o[k]) * inv[k], (bh - o[k]) * inv[k]);
            if a > c {
                core::mem::swap(&mut a, &mut c);
            }
            lo = lo.max(a);
            hi = hi.min(c);
            if lo > hi {
                return None;
            }
        }
    }
    Some(lo)
}

/// Inverse direction for [`ray_box`] (components with `d == 0` are unused there).
#[inline]
pub(crate) fn inv_dir(d: [f64; 3]) -> [f64; 3] {
    [1.0 / d[0], 1.0 / d[1], 1.0 / d[2]]
}

/// Flattened BVH.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct Bvh {
    /// Node boxes.
    pub bb: Vec<Box6>,
    /// Internal node: right child index; leaf: first primitive slot.
    pub link: Vec<u32>,
    /// Leaf: primitive count (> 0); internal node: 0.
    pub count: Vec<u32>,
    /// Primitive ids in leaf order.
    pub prims: Vec<u32>,
}

impl Bvh {
    /// Build over `ids` (a subset of `0..boxes.len()`).
    pub fn build(boxes: &[Box6], ids: Vec<u32>, leaf_size: usize) -> Self {
        let mut bvh = Self::default();
        if ids.is_empty() {
            return bvh;
        }
        let cent: Vec<[f64; 3]> = boxes.iter().map(|b| [0.5 * (b[0] + b[3]), 0.5 * (b[1] + b[4]), 0.5 * (b[2] + b[5])]).collect();
        let mut ids = ids;
        bvh.prims.reserve(ids.len());
        bvh.rec(boxes, &cent, &mut ids, leaf_size.max(1));
        bvh
    }

    fn rec(&mut self, boxes: &[Box6], cent: &[[f64; 3]], ids: &mut [u32], leaf: usize) -> u32 {
        let me = self.bb.len() as u32;
        let mut bb = EMPTY_BOX;
        let mut cb = EMPTY_BOX;
        for &i in ids.iter() {
            bb = union(&bb, &boxes[i as usize]);
            let c = cent[i as usize];
            cb = union(&cb, &[c[0], c[1], c[2], c[0], c[1], c[2]]);
        }
        self.bb.push(bb);
        self.link.push(0);
        self.count.push(0);
        if ids.len() <= leaf {
            self.link[me as usize] = self.prims.len() as u32;
            self.count[me as usize] = ids.len() as u32;
            ids.sort_unstable();
            self.prims.extend_from_slice(ids);
            return me;
        }
        let ext = [cb[3] - cb[0], cb[4] - cb[1], cb[5] - cb[2]];
        let axis = if ext[0] >= ext[1] && ext[0] >= ext[2] { 0 } else if ext[1] >= ext[2] { 1 } else { 2 };
        let mid = ids.len() / 2;
        ids.select_nth_unstable_by(mid, |&a, &b| cent[a as usize][axis].total_cmp(&cent[b as usize][axis]).then(a.cmp(&b)));
        let (l, r) = ids.split_at_mut(mid);
        self.rec(boxes, cent, l, leaf);
        let right = self.rec(boxes, cent, r, leaf);
        self.link[me as usize] = right;
        me
    }

    /// Heap bytes.
    pub fn bytes(&self) -> usize {
        self.bb.len() * 48 + (self.link.len() + self.count.len() + self.prims.len()) * 4
    }

    /// Visit every primitive whose (padded) box contains `p`, in traversal order.
    pub fn visit_point(&self, p: [f64; 3], pad: f64, stack: &mut Vec<u32>, mut f: impl FnMut(u32)) {
        if self.bb.is_empty() {
            return;
        }
        stack.clear();
        stack.push(0);
        while let Some(n) = stack.pop() {
            let n = n as usize;
            let b = &self.bb[n];
            if (0..3).any(|k| p[k] < b[k] - pad || p[k] > b[3 + k] + pad) {
                continue;
            }
            if self.count[n] > 0 {
                let first = self.link[n] as usize;
                for &pr in &self.prims[first..first + self.count[n] as usize] {
                    f(pr);
                }
            } else {
                stack.push(self.link[n]);
                stack.push(n as u32 + 1);
            }
        }
    }

    /// Visit primitives along the ray `o + t d` for `t in [t0, tmax]`, nearest box first.
    /// `f` returns the (possibly reduced) `tmax`; nodes whose entry exceeds it are pruned.
    pub fn visit_ray(&self, o: [f64; 3], d: [f64; 3], pad: f64, t0: f64, tmax: f64, stack: &mut Vec<u32>, mut f: impl FnMut(u32) -> f64) {
        if self.bb.is_empty() {
            return;
        }
        let inv = inv_dir(d);
        let mut tmax = tmax;
        stack.clear();
        if ray_box(o, d, inv, &self.bb[0], pad, t0, tmax).is_none() {
            return;
        }
        stack.push(0);
        while let Some(n) = stack.pop() {
            let n = n as usize;
            if ray_box(o, d, inv, &self.bb[n], pad, t0, tmax).is_none() {
                continue;
            }
            if self.count[n] > 0 {
                let first = self.link[n] as usize;
                for &pr in &self.prims[first..first + self.count[n] as usize] {
                    tmax = f(pr).min(tmax);
                }
            } else {
                let (l, r) = (n as u32 + 1, self.link[n]);
                let tl = ray_box(o, d, inv, &self.bb[l as usize], pad, t0, tmax);
                let tr = ray_box(o, d, inv, &self.bb[r as usize], pad, t0, tmax);
                match (tl, tr) {
                    (Some(a), Some(b)) => {
                        if a <= b {
                            stack.push(r);
                            stack.push(l);
                        } else {
                            stack.push(l);
                            stack.push(r);
                        }
                    }
                    (Some(_), None) => stack.push(l),
                    (None, Some(_)) => stack.push(r),
                    (None, None) => {}
                }
            }
        }
    }

    /// Branch-and-bound over primitives by box distance. `f` returns the (possibly reduced)
    /// squared radius `best2`; boxes at distance `>= best2` are pruned.
    pub fn visit_closest(&self, p: [f64; 3], best2: f64, stack: &mut Vec<u32>, mut f: impl FnMut(u32) -> f64) {
        if self.bb.is_empty() {
            return;
        }
        let mut best2 = best2;
        stack.clear();
        stack.push(0);
        while let Some(n) = stack.pop() {
            let n = n as usize;
            if box_dist2(&self.bb[n], p) > best2 {
                continue;
            }
            if self.count[n] > 0 {
                let first = self.link[n] as usize;
                for &pr in &self.prims[first..first + self.count[n] as usize] {
                    best2 = f(pr).min(best2);
                }
            } else {
                let (l, r) = (n as u32 + 1, self.link[n]);
                let (dl, dr) = (box_dist2(&self.bb[l as usize], p), box_dist2(&self.bb[r as usize], p));
                if dl <= dr {
                    stack.push(r);
                    stack.push(l);
                } else {
                    stack.push(l);
                    stack.push(r);
                }
            }
        }
    }

    /// Structural validation of a decoded BVH (indices in range, children after parents).
    pub fn validate(&self, n_prims: usize) -> bool {
        let n = self.bb.len();
        if self.link.len() != n || self.count.len() != n {
            return false;
        }
        for i in 0..n {
            if self.count[i] > 0 {
                let (a, c) = (self.link[i] as usize, self.count[i] as usize);
                if a.checked_add(c).is_none_or(|e| e > self.prims.len()) {
                    return false;
                }
            } else {
                let r = self.link[i] as usize;
                if i + 1 >= n || r <= i + 1 || r >= n {
                    return false;
                }
            }
        }
        self.prims.iter().all(|&p| (p as usize) < n_prims)
    }
}
