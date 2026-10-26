//! Exact closed-contact predicates used by cavity filling.
//!
//! Unlike the symbolic-perturbation tests, these treat touching and coplanar configurations
//! exactly: two simplices of a valid triangulation may meet only in a common sub-simplex.

use crate::geom::{V3, ori, pt};
use sm_math::Sign;

#[inline]
fn o2(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> i32 {
    sm_math::orient2d(a, b, c).as_i32()
}

/// Axis pair (dropped coordinate) with a non-degenerate projection of triangle `a,b,c`.
fn proj_axes(a: V3, b: V3, c: V3) -> Option<[usize; 2]> {
    [[0, 1], [1, 2], [2, 0]].into_iter().find(|ax| o2(pr(a, *ax), pr(b, *ax), pr(c, *ax)) != 0)
}

#[inline]
fn pr(p: V3, ax: [usize; 2]) -> [f64; 2] {
    [p[ax[0]], p[ax[1]]]
}

fn pt_in_tri_2d(p: [f64; 2], a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> bool {
    let (s1, s2, s3) = (o2(a, b, p), o2(b, c, p), o2(c, a, p));
    !((s1 < 0 || s2 < 0 || s3 < 0) && (s1 > 0 || s2 > 0 || s3 > 0))
}

fn on_seg_2d(a: [f64; 2], b: [f64; 2], p: [f64; 2]) -> bool {
    o2(a, b, p) == 0 && p[0] >= a[0].min(b[0]) && p[0] <= a[0].max(b[0]) && p[1] >= a[1].min(b[1]) && p[1] <= a[1].max(b[1])
}

fn seg_seg_2d(p1: [f64; 2], p2: [f64; 2], q1: [f64; 2], q2: [f64; 2]) -> bool {
    let (d1, d2, d3, d4) = (o2(q1, q2, p1), o2(q1, q2, p2), o2(p1, p2, q1), o2(p1, p2, q2));
    if d1 * d2 < 0 && d3 * d4 < 0 {
        return true;
    }
    on_seg_2d(q1, q2, p1) || on_seg_2d(q1, q2, p2) || on_seg_2d(p1, p2, q1) || on_seg_2d(p1, p2, q2)
}

/// Does the closed segment `p-q` touch the closed triangle `a,b,c`?
pub(crate) fn seg_tri_contact(p: V3, q: V3, a: V3, b: V3, c: V3) -> bool {
    let sp = ori(a, b, c, p);
    let sq = ori(a, b, c, q);
    if (sp > 0 && sq > 0) || (sp < 0 && sq < 0) {
        return false;
    }
    if sp == 0 && sq == 0 {
        let Some(ax) = proj_axes(a, b, c) else { return false };
        let (pa, pb, pc, pp, pq) = (pr(a, ax), pr(b, ax), pr(c, ax), pr(p, ax), pr(q, ax));
        if pt_in_tri_2d(pp, pa, pb, pc) || pt_in_tri_2d(pq, pa, pb, pc) {
            return true;
        }
        return seg_seg_2d(pp, pq, pa, pb) || seg_seg_2d(pp, pq, pb, pc) || seg_seg_2d(pp, pq, pc, pa);
    }
    let (o1, o2_, o3) = (ori(p, q, a, b), ori(p, q, b, c), ori(p, q, c, a));
    !((o1 < 0 || o2_ < 0 || o3 < 0) && (o1 > 0 || o2_ > 0 || o3 > 0))
}

/// Is `x` in the closed positive tet?
pub(crate) fn pt_in_closed_tet(t: [V3; 4], x: V3) -> bool {
    use crate::tds::FACE;
    (0..4).all(|k| {
        let f = FACE[k];
        ori(t[f[0]], t[f[1]], t[f[2]], x) <= 0
    })
}

/// Do triangles `g` and `h` meet in more than their common sub-simplex? `r` is a reference point
/// off the plane of `h` (used to compare sides in the coplanar case). Vertices are given as
/// `(index, coordinates)`; shared vertices are recognised by index.
pub(crate) fn tris_conflict(g: [(u32, V3); 3], h: [(u32, V3); 3], r: V3) -> bool {
    let shared: Vec<usize> = (0..3).filter(|&i| h.iter().any(|x| x.0 == g[i].0)).collect();
    let gp = [g[0].1, g[1].1, g[2].1];
    let hp = [h[0].1, h[1].1, h[2].1];
    match shared.len() {
        3 => false,
        2 => {
            // Share edge e; conflict iff coplanar and the third vertices are on the same side.
            let o = (0..3).find(|i| !shared.contains(i)).unwrap_or(0);
            let (e0, e1) = (g[shared[0]].1, g[shared[1]].1);
            let go = g[o].1;
            let ho = h.iter().find(|x| !g.iter().any(|y| y.0 == x.0)).map(|x| x.1).unwrap_or(hp[0]);
            if ori(e0, e1, go, ho) != 0 {
                return false;
            }
            let (s1, s2) = (ori(e0, e1, go, r), ori(e0, e1, ho, r));
            s1 != 0 && s1 == s2
        }
        1 => {
            let s = shared[0];
            let sv = g[s].0;
            let (p, q) = (g[(s + 1) % 3].1, g[(s + 2) % 3].1);
            let hs = h.iter().position(|x| x.0 == sv).unwrap_or(0);
            let (x, y) = (h[(hs + 1) % 3].1, h[(hs + 2) % 3].1);
            // Opposite edges against the other triangle.
            if seg_tri_contact(p, q, hp[0], hp[1], hp[2]) || seg_tri_contact(x, y, gp[0], gp[1], gp[2]) {
                return true;
            }
            // Coplanar wedges overlapping at the shared vertex.
            let sv3 = g[s].1;
            if ori(sv3, p, q, x) == 0 && ori(sv3, p, q, y) == 0 {
                let Some(ax) = proj_axes(sv3, p, q) else { return false };
                let (s2, p2, q2, x2, y2) = (pr(sv3, ax), pr(p, ax), pr(q, ax), pr(x, ax), pr(y, ax));
                let in_wedge = |a: [f64; 2], b: [f64; 2], t: [f64; 2]| -> bool {
                    // t in the closed wedge (s2; a, b), excluding the apex direction itself
                    let w = o2(s2, a, b);
                    if w == 0 {
                        return false;
                    }
                    let (s_a, s_b) = (o2(s2, a, t), o2(s2, t, b));
                    if w > 0 { s_a >= 0 && s_b >= 0 } else { s_a <= 0 && s_b <= 0 }
                };
                return in_wedge(p2, q2, x2) || in_wedge(p2, q2, y2) || in_wedge(x2, y2, p2) || in_wedge(x2, y2, q2);
            }
            false
        }
        _ => {
            for i in 0..3 {
                let (p, q) = (gp[i], gp[(i + 1) % 3]);
                if seg_tri_contact(p, q, hp[0], hp[1], hp[2]) {
                    return true;
                }
                let (p, q) = (hp[i], hp[(i + 1) % 3]);
                if seg_tri_contact(p, q, gp[0], gp[1], gp[2]) {
                    return true;
                }
            }
            false
        }
    }
}

/// Does the closed tet `t` touch the open segment `u-w` anywhere other than at the endpoints
/// (when they are vertices of `t`)? A tet that contains both endpoints contains the edge and does
/// not "touch".
pub(crate) fn seg_touches_tet(ids: [u32; 4], pts4: [V3; 4], uid: u32, wid: u32, up: V3, wp: V3) -> bool {
    use crate::tds::FACE;
    let hu = ids.iter().position(|&x| x == uid);
    let hw = ids.iter().position(|&x| x == wid);
    let cone = |apex: usize, target: V3| -> bool {
        (0..4).filter(|&k| k != apex).all(|k| {
            let f = FACE[k];
            ori(pts4[f[0]], pts4[f[1]], pts4[f[2]], target) <= 0
        })
    };
    match (hu, hw) {
        (Some(_), Some(_)) => false,
        (Some(a), None) => cone(a, wp),
        (None, Some(b)) => cone(b, up),
        (None, None) => (0..4).any(|k| {
            let f = FACE[k];
            seg_tri_contact(up, wp, pts4[f[0]], pts4[f[1]], pts4[f[2]])
        }),
    }
}

#[allow(dead_code)]
fn _unused() {
    let _ = pt([0.0; 3]);
    let _ = Sign::Zero;
}
