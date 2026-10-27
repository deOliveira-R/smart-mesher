//! Exit-face search inside one cell, ray queries over the face BVH, and exact distances.

use crate::core::{C_BVH, Core, NONE};
use sm_rt_api::Scratch;
use crate::vec3::{add, dot, linf, madd, scale, sub};

/// Why no exit face could be returned.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ExitFail {
    /// The position is outside the cell by more than the tolerance.
    Outside,
    /// No face is exited.
    NoExit,
    /// The first crossing is an entering one and no exiting crossing is within the retry window.
    Sense,
}

/// The exit of a cell along a ray.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Exit {
    /// Distance along the (unit) direction, clamped to `>= 0`.
    pub t: f64,
    /// The face exited (lowest id among exact ties).
    pub face: u32,
}

/// Direction cosines up to this size count as grazing a face (the entry face is then never an exit).
const GRAZE: f64 = GRAZE_COS;
/// Direction cosine below which a direction counts as tangent to a face.
pub(crate) const GRAZE_COS: f64 = 1e-9;

/// The nearest few crossings of a ray (smallest `t` first while collecting).
#[derive(Clone, Copy, Default)]
pub(crate) struct Ties {
    t: [f64; 4],
    pub face: [u32; 4],
    pub along: [bool; 4],
    pub n: usize,
}

impl Ties {
    fn push(&mut self, t: f64, f: u32, along: bool) {
        if self.n == 4 {
            let (wi, wt) = (0..4).fold((0, f64::NEG_INFINITY), |a, i| if self.t[i] > a.1 { (i, self.t[i]) } else { a });
            if t >= wt {
                return;
            }
            self.n -= 1;
            self.swap_remove(wi);
        }
        self.t[self.n] = t;
        self.face[self.n] = f;
        self.along[self.n] = along;
        self.n += 1;
    }
    fn swap_remove(&mut self, i: usize) {
        let l = self.n;
        self.t[i] = self.t[l];
        self.face[i] = self.face[l];
        self.along[i] = self.along[l];
    }
    /// Drop crossings beyond `tmax` and sort the rest by face id.
    fn keep_within(&mut self, tmax: f64) {
        let mut k = 0;
        for i in 0..self.n {
            if self.t[i] <= tmax {
                (self.t[k], self.face[k], self.along[k]) = (self.t[i], self.face[i], self.along[i]);
                k += 1;
            }
        }
        self.n = k;
        for i in 1..k {
            let mut j = i;
            while j > 0 && (self.face[j - 1], self.along[j - 1]) > (self.face[j], self.along[j]) {
                self.face.swap(j - 1, j);
                self.along.swap(j - 1, j);
                self.t.swap(j - 1, j);
                j -= 1;
            }
        }
    }
}

#[inline]
fn tie_width(o: [f64; 3], t: f64) -> f64 {
    64.0 * f64::EPSILON * (1.0 + t.abs().min(1e300) + linf(o))
}

impl Core {
    /// Cyrus-Beck exit of a convex all-planar cell: the minimum parameter over the outward
    /// planes, the entry face excluded by identity, lowest face id among ties.
    pub fn exit_convex(&self, c: usize, o: [f64; 3], d: [f64; 3], entry: u32, tol: f64) -> Result<Exit, ExitFail> {
        let faces = self.cell_faces(c);
        let mut tmin = f64::INFINITY;
        for &f in faces {
            let pl = self.f_plane[f as usize];
            let s = self.sign(c, f as usize);
            let md = s * dot([pl[0], pl[1], pl[2]], d);
            let slack = s * (pl[3] - dot([pl[0], pl[1], pl[2]], o));
            // Beyond the tolerance outside any face plane: the state is not in this cell.
            if slack < -tol {
                return Err(ExitFail::Outside);
            }
            // A face the ray runs along (or the entry face when grazing) is never an exit.
            if md <= 0.0 || (md <= GRAZE && (f == entry || slack <= tol)) {
                continue;
            }
            let t = (slack / md).max(0.0);
            if t < tmin {
                tmin = t;
            }
        }
        if !tmin.is_finite() {
            return Err(ExitFail::NoExit);
        }
        let tie = tie_width(o, tmin);
        let mut best = NONE;
        for &f in faces {
            if f > best {
                continue;
            }
            let pl = self.f_plane[f as usize];
            let s = self.sign(c, f as usize);
            let md = s * dot([pl[0], pl[1], pl[2]], d);
            if md <= 0.0 {
                continue;
            }
            let slack = s * (pl[3] - dot([pl[0], pl[1], pl[2]], o));
            if md <= GRAZE && (f == entry || slack <= tol) {
                continue;
            }
            if (slack / md).max(0.0) <= tmin + tie {
                best = f;
            }
        }
        Ok(Exit { t: tmin, face: best })
    }

    /// Crossing-based exit of any cell (non-convex, non-planar faces, facet cells). `hits`
    /// and `stack` are the caller's scratch buffers.
    pub fn exit_generic(&self, c: usize, o: [f64; 3], d: [f64; 3], entry: u32, tol: f64, sc: &mut Scratch) -> Result<Exit, ExitFail> {
        let (hits, stack) = (&mut sc.hits, &mut sc.ids);
        hits.clear();
        let cu = c as u32;
        let push = |f: u32, hits: &mut Vec<(f64, u32)>| -> f64 {
            let fi = f as usize;
            if self.f_left[fi] != cu && self.f_right[fi] != cu {
                return f64::INFINITY;
            }
            let s_pos = self.f_left[fi] == cu;
            let graze = f == entry && {
                let pl = self.f_plane[fi];
                dot([pl[0], pl[1], pl[2]], d).abs() <= GRAZE
            };
            let mut tmax = f64::INFINITY;
            self.face_crossings(fi, o, d, |t, along| {
                let outward = along == s_pos;
                if f == entry && (!outward || graze) {
                    return;
                }
                hits.push((t, f << 1 | u32::from(outward)));
                if outward && t >= -tol {
                    tmax = tmax.min(t.max(0.0) + 4.0 * tol + tie_width(o, t));
                }
            });
            tmax
        };
        if self.c_flags[c] & C_BVH != 0 {
            let mut tmax = f64::INFINITY;
            self.face_bvh.visit_ray((o, d), tol, (-tol, tmax), stack, |f| {
                let r = push(f, hits);
                tmax = tmax.min(r);
                tmax
            });
        } else {
            for &f in self.cell_faces(c) {
                push(f, hits);
            }
        }
        decide_exit(hits, o, tol)
    }

    /// First crossing of the line `o + t d` (`t >= 0`) over all BVH faces as `(t, face, along)`, plus the (up to four) nearest crossings `(face, along)` that tie
    /// with the first within the ray tie width, in ascending face id; it includes the first
    /// itself. Coincident facets of touching shells show up here.
    pub fn first_hit_ties(&self, o: [f64; 3], d: [f64; 3], tol: f64, stack: &mut Vec<u32>) -> (Option<(f64, u32, bool)>, Ties) {
        let mut best: Option<(f64, u32, bool)> = None;
        let mut near = Ties::default();
        let mut tmax = f64::INFINITY;
        self.face_bvh.visit_ray((o, d), tol, (0.0, tmax), stack, |f| {
            self.face_crossings(f as usize, o, d, |t, along| {
                if t < 0.0 {
                    return;
                }
                near.push(t, f, along);
                let better = match best {
                    None => true,
                    Some((bt, bf, _)) => t < bt || (t == bt && f < bf),
                };
                if better {
                    best = Some((t, f, along));
                }
            });
            if let Some((bt, _, _)) = best {
                tmax = bt + tie_width(o, bt);
            }
            tmax
        });
        if let Some((bt, _, _)) = best {
            near.keep_within(bt + tie_width(o, bt));
        }
        (best, near)
    }

    /// Nearest entering crossing of an exterior-facing boundary face (`f_right == NONE`, line
    /// against the outward normal), `t >= -tol`.
    pub fn enter_from_outside(&self, o: [f64; 3], d: [f64; 3], tol: f64, stack: &mut Vec<u32>) -> Option<(f64, u32)> {
        let mut best: Option<(f64, u32)> = None;
        let mut tmax = f64::INFINITY;
        self.face_bvh.visit_ray((o, d), tol, (-tol, tmax), stack, |f| {
            if self.f_right[f as usize] != NONE {
                return tmax;
            }
            self.face_crossings(f as usize, o, d, |t, along| {
                if along || t < -tol {
                    return;
                }
                let better = match best {
                    None => true,
                    Some((bt, bf)) => t < bt || (t == bt && f < bf),
                };
                if better {
                    best = Some((t, f));
                }
            });
            if let Some((bt, _)) = best {
                tmax = bt + tie_width(o, bt);
            }
            tmax
        });
        best
    }

    /// Exact squared distance from `p` to face `f` (its fan) and the closest point.
    pub fn face_dist2(&self, f: usize, p: [f64; 3]) -> (f64, [f64; 3]) {
        let mut best = (f64::INFINITY, [0.0; 3]);
        for i in 0..self.n_tris(f) {
            let (_, [a, b, c]) = self.tri(f, i);
            let q = closest_on_triangle(p, a, b, c);
            let d = sub(p, q);
            let d2 = dot(d, d);
            if d2 < best.0 {
                best = (d2, q);
            }
        }
        best
    }
}

/// Apply the exit rule to the collected crossings (`face << 1 | outward`).
fn decide_exit(hits: &[(f64, u32)], o: [f64; 3], tol: f64) -> Result<Exit, ExitFail> {
    // Entering crossings at or behind the start (within 4 tol, twice the 2-tol locate probe)
    // are the entry itself, not evidence of a sense error; outward crossings slightly behind the start are exits at once.
    let usable = |h: &&(f64, u32)| h.0 >= -tol && (h.1 & 1 == 1 || h.0 > 4.0 * tol);
    let first = hits
        .iter()
        .filter(usable)
        .min_by(|a, b| a.0.total_cmp(&b.0).then((a.1 >> 1).cmp(&(b.1 >> 1))))
        .copied();
    let Some((t0, w0)) = first else { return Err(ExitFail::NoExit) };
    if w0 & 1 == 1 {
        let tie = tie_width(o, t0);
        let mut best = (f64::INFINITY, NONE);
        for &(t, w) in hits {
            if w & 1 == 1 && t >= -tol && t <= t0 + tie {
                let f = w >> 1;
                if f < best.1 {
                    best.1 = f;
                }
                best.0 = best.0.min(t);
            }
        }
        return Ok(Exit { t: best.0.max(0.0), face: best.1 });
    }
    // The first crossing is an entering one: retry against exiting crossings within 4 tol.
    let mut best: Option<(f64, u32)> = None;
    for &(t, w) in hits {
        if w & 1 == 1 && t >= -tol && t <= t0 + 4.0 * tol {
            let f = w >> 1;
            if best.is_none_or(|(bt, bf)| t < bt || (t == bt && f < bf)) {
                best = Some((t, f));
            }
        }
    }
    match best {
        Some((t, f)) => Ok(Exit { t: t.max(0.0), face: f }),
        None => Err(ExitFail::Sense),
    }
}

/// Closest point on triangle `abc` to `p` (Ericson, Real-Time Collision Detection 5.1.5).
pub(crate) fn closest_on_triangle(p: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> [f64; 3] {
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ap = sub(p, a);
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = sub(p, b);
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return madd(a, ab, d1 / (d1 - d3));
    }
    let cp = sub(p, c);
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return madd(a, ac, d2 / (d2 - d6));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return madd(b, sub(c, b), w);
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    add(a, add(scale(ab, v), scale(ac, w)))
}
