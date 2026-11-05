//! Valence cleanup of an all-quad mesh: valence-2 interior node removal and diagonal collapse.
//!
//! Both operations keep the boundary nodes (indices below `nb`) untouched, keep every quad
//! strictly convex (exact predicates) and never lower the worst corner sine of the quads they
//! change by more than 10 percent. Each pass visits nodes and quads in index order and skips an
//! operation whose neighbourhood an earlier one in the same pass already changed, so the result
//! is deterministic.

use super::smooth::quad_valid;

/// A changed quad may have a worst corner sine this fraction of the old one at least.
const KEEP: f64 = 0.9;

/// Minimum corner sine of quad `q` (negative if a corner turns the wrong way).
fn sine(pts: &[[f64; 2]], q: &[u32; 4]) -> f64 {
    let mut m = f64::INFINITY;
    for k in 0..4 {
        let (p0, p1, p2) = (pts[q[(k + 3) % 4] as usize], pts[q[k] as usize], pts[q[(k + 1) % 4] as usize]);
        let e1 = [p1[0] - p0[0], p1[1] - p0[1]];
        let e2 = [p2[0] - p1[0], p2[1] - p1[1]];
        let l = e1[0].hypot(e1[1]) * e2[0].hypot(e2[1]);
        m = m.min(if l > 0.0 { (e1[0] * e2[1] - e1[1] * e2[0]) / l } else { -1.0 });
    }
    m
}

/// Quads incident to each node (CSR).
struct Inc {
    start: Vec<u32>,
    quads: Vec<u32>,
}

impl Inc {
    fn build(n: usize, quads: &[[u32; 4]], dead: &[bool]) -> Inc {
        let mut start = vec![0u32; n + 1];
        for (qi, q) in quads.iter().enumerate() {
            if !dead[qi] {
                for v in q {
                    start[*v as usize + 1] += 1;
                }
            }
        }
        for i in 0..n {
            start[i + 1] += start[i];
        }
        let mut fill = start.clone();
        let mut list = vec![0u32; start[n] as usize];
        for (qi, q) in quads.iter().enumerate() {
            if !dead[qi] {
                for v in q {
                    list[fill[*v as usize] as usize] = qi as u32;
                    fill[*v as usize] += 1;
                }
            }
        }
        Inc { start, quads: list }
    }

    fn of(&self, v: u32) -> &[u32] {
        &self.quads[self.start[v as usize] as usize..self.start[v as usize + 1] as usize]
    }
}

/// Sorted distinct edge neighbours of `v` into `out`.
fn neighbours(inc: &Inc, quads: &[[u32; 4]], v: u32, out: &mut Vec<u32>) {
    out.clear();
    for &qi in inc.of(v) {
        let q = quads[qi as usize];
        let k = q.iter().position(|x| *x == v).unwrap_or(0);
        out.push(q[(k + 1) % 4]);
        out.push(q[(k + 3) % 4]);
    }
    out.sort_unstable();
    out.dedup();
}

/// Removes valence-2 interior nodes and collapses quad diagonals between interior nodes whose
/// valences sum to a better one (two valence-3 nodes make one regular node). Runs at most `passes` passes
/// or until nothing changes. Returns whether the mesh changed; interior nodes are renumbered
/// in their old order.
pub(crate) fn cleanup(pts: &mut Vec<[f64; 2]>, quads: &mut Vec<[u32; 4]>, nb: usize, passes: u32) -> bool {
    let n = pts.len();
    let mut dead = vec![false; quads.len()];
    let mut gone = vec![false; n];
    let mut changed = false;
    let (mut na, mut nb2) = (Vec::new(), Vec::new());
    for _ in 0..passes {
        let inc = Inc::build(n, quads, &dead);
        let mut touched = vec![false; quads.len()];
        let mut any = false;
        let free = |touched: &[bool], nodes: &[u32]| nodes.iter().all(|v| inc.of(*v).iter().all(|q| !touched[*q as usize]));
        let mark = |touched: &mut [bool], nodes: &[u32]| {
            for v in nodes {
                for q in inc.of(*v) {
                    touched[*q as usize] = true;
                }
            }
        };
        // Valence-2 interior nodes: merge the two quads around them.
        for v in nb as u32..n as u32 {
            if gone[v as usize] || inc.of(v).len() != 2 {
                continue;
            }
            let (ia, ib) = (inc.of(v)[0] as usize, inc.of(v)[1] as usize);
            let rot = |q: [u32; 4]| -> [u32; 4] {
                let k = q.iter().position(|x| *x == v).unwrap_or(0);
                core::array::from_fn(|i| q[(k + i) % 4])
            };
            let (qa, qb) = (rot(quads[ia]), rot(quads[ib]));
            if !(qa[1] == qb[3] && qa[3] == qb[1]) {
                continue;
            }
            let merged = [qa[1], qa[2], qa[3], qb[2]];
            if merged[1] == merged[3] || !quad_valid(pts, &merged) {
                continue;
            }
            if !free(&touched, &merged) || !free(&touched, &[v]) {
                continue;
            }
            mark(&mut touched, &merged);
            mark(&mut touched, &[v]);
            quads[ia] = merged;
            dead[ib] = true;
            gone[v as usize] = true;
            any = true;
        }
        // Diagonal collapse. Valences are those of the pass start; touched neighbourhoods are
        // skipped, so they are still right where used.
        for qi in 0..quads.len() {
            if dead[qi] || touched[qi] {
                continue;
            }
            let q = quads[qi];
            for (u, v, x, y) in [(q[0], q[2], q[1], q[3]), (q[1], q[3], q[2], q[0])] {
                if (u as usize) < nb || (v as usize) < nb || gone[u as usize] || gone[v as usize] {
                    continue;
                }
                neighbours(&inc, quads, u, &mut na);
                neighbours(&inc, quads, v, &mut nb2);
                let (vu, vv) = (na.len() as i32, nb2.len() as i32);
                let nv = vu + vv - 2;
                if nv < 3 || (4 - vu).abs() + (4 - vv).abs() <= (4 - nv).abs() {
                    continue;
                }
                // Common neighbours must be exactly the two other corners of the quad.
                let common = na.iter().filter(|a| nb2.binary_search(a).is_ok()).count();
                if common != 2 || !free(&touched, &[u, v, x, y]) {
                    continue;
                }
                let changed_quads: Vec<usize> = inc.of(u).iter().chain(inc.of(v)).map(|k| *k as usize).filter(|k| *k != qi).collect();
                if changed_quads.iter().any(|k| quads[*k].contains(&u) && quads[*k].contains(&v)) {
                    continue;
                }
                let old = changed_quads.iter().map(|k| sine(pts, &quads[*k])).fold(f64::INFINITY, f64::min);
                let (pu, pv) = (pts[u as usize], pts[v as usize]);
                let mut done = false;
                for cand in [[0.5 * (pu[0] + pv[0]), 0.5 * (pu[1] + pv[1])], pu, pv] {
                    pts[u as usize] = cand;
                    let moved: Vec<[u32; 4]> = changed_quads.iter().map(|k| quads[*k].map(|w| if w == v { u } else { w })).collect();
                    let new = moved.iter().map(|m| sine(pts, m)).fold(f64::INFINITY, f64::min);
                    if moved.iter().all(|m| quad_valid(pts, m)) && new >= KEEP * old.min(1.0) {
                        for (k, m) in changed_quads.iter().zip(moved) {
                            quads[*k] = m;
                        }
                        done = true;
                        break;
                    }
                }
                if done {
                    mark(&mut touched, &[u, v, x, y]);
                    dead[qi] = true;
                    gone[v as usize] = true;
                    any = true;
                    break;
                }
                pts[u as usize] = pu;
            }
        }
        if !any {
            break;
        }
        changed = true;
    }
    if changed {
        let mut remap = vec![u32::MAX; n];
        let mut next = 0u32;
        let mut kept = Vec::with_capacity(n);
        for i in 0..n {
            if !gone[i] {
                remap[i] = next;
                next += 1;
                kept.push(pts[i]);
            }
        }
        *pts = kept;
        let new: Vec<[u32; 4]> = quads.iter().zip(&dead).filter(|(_, d)| !**d).map(|(q, _)| q.map(|v| remap[v as usize])).collect();
        *quads = new;
    }
    changed
}
