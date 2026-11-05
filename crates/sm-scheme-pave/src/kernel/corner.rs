//! The corner rule: all-quad meshes of three-corner regions (triangles, curved or straight).
//!
//! A region whose loop has three corners and side counts `a, b, c` (sum even) splits into three
//! structured blocks around one valence-3 centre when every arm `r = (b + c - a) / 2` (and its
//! two permutations) is at least one: each corner owns a block, the acute corners one quad at
//! least. When an arm is zero or negative (a long side as long as the other two together) the
//! region is first cut from a corner to a node of the opposite side into two three-corner
//! regions, each of which then splits into three blocks. Interior positions come from transfinite
//! (Coons) interpolation over the node-index fractions; every candidate is checked with exact
//! predicates and the best by worst corner sine wins, ties to the first in a fixed order.

use super::smooth::quad_valid;

/// Smallest accepted corner sine of any quad.
const MIN_SINE: f64 = 1e-3;
/// At most this many cut positions are tried per corner (stride over longer sides).
const MAX_CUT_POSITIONS: usize = 128;

/// Node-index lists of the three sides; `s[k]` runs from corner `k` to corner `k + 1`.
type Sides = [Vec<u32>; 3];

/// Points and quads that fill a region: indices from `base` (the input point count) on name
/// the new points in order.
struct Piece {
    pts: Vec<[f64; 2]>,
    quads: Vec<[u32; 4]>,
    quality: f64,
}

/// Interior points and quads of a filled region.
pub(crate) type Filled = (Vec<[f64; 2]>, Vec<[u32; 4]>);

/// Minimum corner sine of a quad (negative if a corner turns the wrong way).
fn min_sine(p: [[f64; 2]; 4]) -> f64 {
    let mut m = f64::INFINITY;
    for k in 0..4 {
        let (p0, p1, p2) = (p[(k + 3) % 4], p[k], p[(k + 1) % 4]);
        let e1 = [p1[0] - p0[0], p1[1] - p0[1]];
        let e2 = [p2[0] - p1[0], p2[1] - p1[1]];
        let l = e1[0].hypot(e1[1]) * e2[0].hypot(e2[1]);
        m = m.min(if l > 0.0 { (e1[0] * e2[1] - e1[1] * e2[0]) / l } else { -1.0 });
    }
    m
}

/// Position of node `i`: from the base points or the piece under construction.
fn at(base: &[[f64; 2]], new: &[[f64; 2]], i: u32) -> [f64; 2] {
    let i = i as usize;
    if i < base.len() { base[i] } else { new[i - base.len()] }
}

/// The arms `[r_a, r_b, r_c]` of the three-block split, `None` for an odd sum or an arm below one.
fn arms(a: usize, b: usize, c: usize) -> Option<[usize; 3]> {
    if (a + b + c) % 2 == 1 {
        return None;
    }
    let r = |x: usize, y: usize, z: usize| (y + z).checked_sub(x).filter(|d| *d >= 2).map(|d| d / 2);
    Some([r(a, b, c)?, r(b, c, a)?, r(c, a, b)?])
}

/// One structured block: `bottom` and `top` have `p + 1` nodes (left to right), `left` and
/// `right` `q + 1` (bottom to top). Pushes the interior points to `new` and the quads.
fn block(base: &[[f64; 2]], new: &mut Vec<[f64; 2]>, quads: &mut Vec<[u32; 4]>, bottom: &[u32], right: &[u32], top: &[u32], left: &[u32]) {
    let (p, q) = (bottom.len() - 1, right.len() - 1);
    debug_assert!(top.len() == p + 1 && left.len() == q + 1);
    let mut idx = vec![0u32; (p + 1) * (q + 1)];
    let g = |i: usize, j: usize| i * (q + 1) + j;
    for i in 0..=p {
        idx[g(i, 0)] = bottom[i];
        idx[g(i, q)] = top[i];
    }
    for j in 0..=q {
        idx[g(0, j)] = left[j];
        idx[g(p, j)] = right[j];
    }
    let (b0, b1, t0, t1) = (at(base, new, bottom[0]), at(base, new, bottom[p]), at(base, new, top[0]), at(base, new, top[p]));
    for i in 1..p {
        for j in 1..q {
            let (s, t) = (i as f64 / p as f64, j as f64 / q as f64);
            let (bi, ti, lj, rj) = (at(base, new, bottom[i]), at(base, new, top[i]), at(base, new, left[j]), at(base, new, right[j]));
            let x: [f64; 2] = core::array::from_fn(|k| {
                (1.0 - t) * bi[k] + t * ti[k] + (1.0 - s) * lj[k] + s * rj[k]
                    - ((1.0 - s) * (1.0 - t) * b0[k] + s * (1.0 - t) * b1[k] + (1.0 - s) * t * t0[k] + s * t * t1[k])
            });
            idx[g(i, j)] = (base.len() + new.len()) as u32;
            new.push(x);
        }
    }
    for i in 0..p {
        for j in 0..q {
            quads.push([idx[g(i, j)], idx[g(i + 1, j)], idx[g(i + 1, j + 1)], idx[g(i, j + 1)]]);
        }
    }
}

/// A straight arm from `from` to the node `to` with `r` intervals: the node list (ends
/// included); the `r - 1` inner points are pushed to `new`.
fn arm(base: &[[f64; 2]], new: &mut Vec<[f64; 2]>, from: u32, to: u32, r: usize) -> Vec<u32> {
    let (a, b) = (at(base, new, from), at(base, new, to));
    let mut v = vec![from];
    for k in 1..r {
        let t = k as f64 / r as f64;
        v.push((base.len() + new.len()) as u32);
        new.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
    }
    v.push(to);
    v
}

/// Three-block fill of a three-sided region, the best over a few centre positions.
fn three_block(base: &[[f64; 2]], s: &Sides) -> Option<Piece> {
    let (a, b, c) = (s[0].len() - 1, s[1].len() - 1, s[2].len() - 1);
    let [ra, rb, rc] = arms(a, b, c)?;
    let (ma, mb, mc) = (s[0][rc], s[1][ra], s[2][rb]);
    let (pa, pb, pc) = (base[ma as usize], base[mb as usize], base[mc as usize]);
    let (c0, c1, c2) = (base[s[0][0] as usize], base[s[1][0] as usize], base[s[2][0] as usize]);
    let comb = |x: [f64; 2], y: [f64; 2], z: [f64; 2]| -> [f64; 2] { [x[0] + y[0] - z[0], x[1] + y[1] - z[1]] };
    let (q1, q2, q0) = (comb(pa, pb, c1), comb(pb, pc, c2), comb(pc, pa, c0));
    let cen = [(pa[0] + pb[0] + pc[0]) / 3.0, (pa[1] + pb[1] + pc[1]) / 3.0];
    let par = [(q0[0] + q1[0] + q2[0]) / 3.0, (q0[1] + q1[1] + q2[1]) / 3.0];
    let mut best: Option<Piece> = None;
    for o in [cen, par, q0, q1, q2] {
        let mut new = vec![o];
        let oi = base.len() as u32;
        let arm_a = arm(base, &mut new, ma, oi, ra);
        let arm_b = arm(base, &mut new, mb, oi, rb);
        let arm_c = arm(base, &mut new, mc, oi, rc);
        let rev = |v: &[u32]| -> Vec<u32> { v.iter().rev().copied().collect() };
        let mut quads = Vec::new();
        block(base, &mut new, &mut quads, &s[0][rc..=a], &s[1][..=ra], &rev(&arm_b), &arm_a);
        block(base, &mut new, &mut quads, &s[1][ra..=b], &s[2][..=rb], &rev(&arm_c), &arm_b);
        block(base, &mut new, &mut quads, &s[2][rb..=c], &s[0][..=rc], &rev(&arm_a), &arm_c);
        let pos = |i: u32| at(base, &new, i);
        let mut quality = f64::INFINITY;
        let mut ok = true;
        for q in &quads {
            quality = quality.min(min_sine(core::array::from_fn(|k| pos(q[k]))));
        }
        if quality < MIN_SINE {
            ok = false;
        }
        if ok {
            let all: Vec<[f64; 2]> = base.iter().chain(new.iter()).copied().collect();
            ok = quads.iter().all(|q| quad_valid(&all, q));
        }
        if ok && best.as_ref().is_none_or(|p| quality > p.quality) {
            best = Some(Piece { pts: new, quads, quality });
        }
    }
    best
}

/// Is there a three-block fill for these counts?
fn feasible(a: usize, b: usize, c: usize) -> bool {
    arms(a, b, c).is_some()
}

/// Fills a three-sided region: three blocks, or one cut from a corner and two three-block
/// fills.
fn fill_three(base: &[[f64; 2]], s: &Sides, mean_len: f64) -> Option<Piece> {
    if let Some(p) = three_block(base, s) {
        return Some(p);
    }
    let mut best: Option<Piece> = None;
    for r in 0..3 {
        let (sa, sb, sc) = (&s[r], &s[(r + 1) % 3], &s[(r + 2) % 3]);
        let (alen, blen, clen) = (sa.len() - 1, sb.len() - 1, sc.len() - 1);
        let corner = sc[0];
        let stride = alen.div_ceil(MAX_CUT_POSITIONS).max(1);
        let mut m = 1;
        while m < alen {
            let to = sa[m];
            let (pc, pm) = (base[corner as usize], base[to as usize]);
            let d0 = ((pc[0] - pm[0]).hypot(pc[1] - pm[1]) / mean_len).round().max(1.0) as usize;
            for d in d0.saturating_sub(2).max(1)..=d0 + 2 {
                if !(feasible(m, d, clen) && feasible(alen - m, blen, d)) {
                    continue;
                }
                let mut ext: Vec<[f64; 2]> = base.to_vec();
                let mut cut = vec![corner];
                for k in 1..d {
                    let t = k as f64 / d as f64;
                    cut.push(ext.len() as u32);
                    ext.push([pc[0] + t * (pm[0] - pc[0]), pc[1] + t * (pm[1] - pc[1])]);
                }
                cut.push(to);
                let cut_new: Vec<[f64; 2]> = ext[base.len()..].to_vec();
                let rev: Vec<u32> = cut.iter().rev().copied().collect();
                let s1: Sides = [sa[..=m].to_vec(), rev, sc.clone()];
                let Some(p1) = three_block(&ext, &s1) else { continue };
                ext.extend_from_slice(&p1.pts);
                let s2: Sides = [sa[m..].to_vec(), sb.clone(), cut];
                let Some(p2) = three_block(&ext, &s2) else { continue };
                let quality = p1.quality.min(p2.quality);
                if best.as_ref().is_none_or(|b| quality > b.quality) {
                    let mut pts = cut_new;
                    pts.extend(p1.pts);
                    pts.extend(p2.pts);
                    let mut quads = p1.quads;
                    quads.extend(p2.quads);
                    best = Some(Piece { pts, quads, quality });
                }
            }
            m += stride;
        }
    }
    best
}

/// The three corners of a loop: the three nodes with the largest left turn (ties to the lower
/// index), in loop order.
fn corners(w: &[[f64; 2]]) -> Option<[usize; 3]> {
    let n = w.len();
    let mut turn: Vec<(f64, usize)> = (0..n)
        .map(|i| {
            let (p, q, r) = (w[(i + n - 1) % n], w[i], w[(i + 1) % n]);
            let (e1, e2) = ([q[0] - p[0], q[1] - p[1]], [r[0] - q[0], r[1] - q[1]]);
            ((e1[0] * e2[1] - e1[1] * e2[0]).atan2(e1[0] * e2[0] + e1[1] * e2[1]), i)
        })
        .collect();
    turn.sort_by(|x, y| y.0.total_cmp(&x.0).then(x.1.cmp(&y.1)));
    let top = &turn[..3.min(n)];
    if top.len() < 3 || top[2].0 <= 0.0 {
        return None;
    }
    let mut c = [top[0].1, top[1].1, top[2].1];
    c.sort_unstable();
    Some(c)
}

/// All-quad fill of a single counter-clockwise loop `w` read as a three-corner region.
///
/// Returns the interior points (to follow the `w.len()` boundary points) and the quads, or
/// `None` when the loop has no three clear corners, the counts do not allow a block split even
/// with one cut, or no candidate has strictly convex quads of corner sine at least `1e-3`.
pub(crate) fn corner_fill(w: &[[f64; 2]]) -> Option<Filled> {
    let n = w.len();
    let [c0, c1, c2] = corners(w)?;
    let run = |from: usize, to: usize| -> Vec<u32> {
        let len = (to + n - from) % n;
        (0..=len).map(|k| ((from + k) % n) as u32).collect()
    };
    let sides: Sides = [run(c0, c1), run(c1, c2), run(c2, c0)];
    let perimeter: f64 = (0..n).map(|i| (w[(i + 1) % n][0] - w[i][0]).hypot(w[(i + 1) % n][1] - w[i][1])).sum();
    let piece = fill_three(w, &sides, perimeter / n as f64)?;
    Some((piece.pts, piece.quads))
}
