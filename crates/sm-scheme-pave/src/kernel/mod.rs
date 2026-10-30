//! The pure paving kernel: an all-quad mesh of a planar region with fixed boundary nodes.
//!
//! Pipeline (all sequential and deterministic): metric-correct the parameter space with a
//! global affine map; constrained Delaunay triangulation of the boundary nodes (exact
//! predicates, nodes never split); frontal rows of points built as squares on the front;
//! Delaunay refinement of what the rows leave; perfect matching of the triangle dual into
//! convex quads (greedy by quality plus blossom augmentation, edge splitting when no
//! perfect matching exists); smart-Laplacian smoothing.

mod cdt;
mod front;
mod matching;
mod metric;
mod smooth;
mod strip;

use cdt::{Cdt, CdtError};
use core::ops::Range;
use front::{Mesher, Tuning};
use metric::{Frame, Met, SizeMap, Space};
use sm_math::{Sign, orient2d};

/// Options of [`pave_region`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PaveOptions {
    /// Maximum relative size change per unit length of the default (boundary-derived) size
    /// field.
    pub gradation: f64,
    /// Smoothing sweeps.
    pub smooth_iters: u32,
    /// Maximum rounds of edge splitting when the triangle dual has no perfect matching.
    pub max_rounds: u32,
}

impl Default for PaveOptions {
    fn default() -> Self {
        PaveOptions { gradation: 0.3, smooth_iters: 30, max_rounds: 6 }
    }
}

/// An all-quad mesh of a region.
#[derive(Clone, Debug, PartialEq)]
pub struct QuadMesh2 {
    /// Points: `points[..nb]` are the input boundary points in loop-concatenated order,
    /// bit-exact; the rest are interior points in creation order.
    pub points: Vec<[f64; 2]>,
    /// Quads (indices into `points`). A quad that owns the input segment `a -> b` lists `a`
    /// then `b` consecutively; all quads wind like the input loops (interior on the left of
    /// every input segment, whatever the handedness of the input).
    pub quads: Vec<[u32; 4]>,
    /// Rounds of edge splitting the recombination needed (0 when the triangle dual had a
    /// perfect matching at once).
    pub rounds: u32,
    /// Factor the winning strategy applied to the wanted size. 1.0 means the mesh follows
    /// the requested (or boundary-derived) size; above 1.0 the size was coarsened to find a
    /// perfect matching, below 1.0 refined. The structured strip fill reports the measured
    /// ratio of its mean edge length to the wanted size.
    pub size_scale: f64,
}

/// Why [`pave_region`] failed.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum PaveError {
    /// An all-quad mesh needs an even number of boundary nodes in total (over all loops).
    #[error("odd number of boundary nodes ({total}); a quad mesh needs an even total")]
    OddBoundary {
        /// The total boundary node count.
        total: usize,
    },
    /// A loop with fewer than three nodes encloses nothing.
    #[error("loop {ring} has only {nodes} nodes")]
    LoopTooSmall {
        /// Loop index (0 = outer).
        ring: usize,
        /// Its node count.
        nodes: usize,
    },
    /// Two boundary segments cross or touch, nodes coincide, or the outer loop has no area.
    #[error("the boundary loops intersect themselves or each other")]
    BoundaryIntersects,
    /// The triangle dual has no perfect matching even after edge splitting.
    #[error("no perfect matching: {unmatched} triangles left unmatched")]
    NoPerfectMatching {
        /// Triangles still unmatched.
        unmatched: usize,
    },
    /// The metric is not finite and positive definite somewhere it was evaluated, or the
    /// size function returned a non-positive value.
    #[error("the metric or size function is not finite and positive")]
    BadMetric,
    /// A hole winds the same way as the outer loop (the interior must be on the left of
    /// every segment of every loop).
    #[error("loop {ring} winds the wrong way")]
    BadOrientation {
        /// Loop index.
        ring: usize,
    },
    /// A boundary coordinate is not finite.
    #[error("non-finite boundary coordinate")]
    NonFinite,
    /// The mesh is valid in the corrected frame but a quad degenerates in the original
    /// parameter space (the metric map is numerically singular).
    #[error("a quad degenerates in the parameter space")]
    Degenerate,
    /// The wanted size asks for more vertices than the kernel allows (`limit`); `estimate`
    /// is the count it asked for (or reached). Typical cause: a size field of 1e-9.
    #[error("the wanted size needs about {estimate} vertices; the limit is {limit}")]
    TooLarge {
        /// Estimated (or reached) vertex count.
        estimate: usize,
        /// The absolute bound.
        limit: usize,
    },
    /// An internal invariant broke.
    #[error("internal error: {0}")]
    Internal(&'static str),
}

impl From<CdtError> for PaveError {
    fn from(e: CdtError) -> Self {
        match e {
            CdtError::Duplicate | CdtError::Crossing => PaveError::BoundaryIntersects,
            CdtError::Orientation(ring) => PaveError::BadOrientation { ring },
            CdtError::Internal(s) => PaveError::Internal(s),
        }
    }
}

/// Do the closed segments `ab` and `cd` share a point?
fn segments_touch(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> bool {
    let on = |p: [f64; 2], q: [f64; 2], r: [f64; 2]| {
        r[0] >= p[0].min(q[0]) && r[0] <= p[0].max(q[0]) && r[1] >= p[1].min(q[1]) && r[1] <= p[1].max(q[1])
    };
    let (o1, o2) = (orient2d(a, b, c), orient2d(a, b, d));
    let (o3, o4) = (orient2d(c, d, a), orient2d(c, d, b));
    if o1 != o2 && o3 != o4 && o1 != Sign::Zero && o2 != Sign::Zero && o3 != Sign::Zero && o4 != Sign::Zero {
        return true;
    }
    (o1 == Sign::Zero && on(a, b, c)) || (o2 == Sign::Zero && on(a, b, d)) || (o3 == Sign::Zero && on(c, d, a)) || (o4 == Sign::Zero && on(c, d, b))
}

/// Exact check that no two segments of the loops cross or touch, apart from the shared end
/// of consecutive segments of one loop (which must not fold back on each other either).
fn check_boundary(pts: &[[f64; 2]], ranges: &[Range<usize>]) -> Result<(), PaveError> {
    struct Seg {
        ring: usize,
        k: usize,
        n: usize,
        a: [f64; 2],
        b: [f64; 2],
    }
    let mut segs: Vec<Seg> = Vec::with_capacity(pts.len());
    for (ring, r) in ranges.iter().enumerate() {
        let n = r.len();
        for k in 0..n {
            segs.push(Seg { ring, k, n, a: pts[r.start + k], b: pts[r.start + (k + 1) % n] });
        }
    }
    let lo = |s: &Seg| s.a[0].min(s.b[0]);
    let hi = |s: &Seg| s.a[0].max(s.b[0]);
    let mut order: Vec<usize> = (0..segs.len()).collect();
    order.sort_unstable_by(|x, y| lo(&segs[*x]).total_cmp(&lo(&segs[*y])).then(x.cmp(y)));
    for (oi, &i) in order.iter().enumerate() {
        let s = &segs[i];
        for &j in &order[oi + 1..] {
            let t = &segs[j];
            if lo(t) > hi(s) {
                break;
            }
            if s.a[1].min(s.b[1]) > t.a[1].max(t.b[1]) || t.a[1].min(t.b[1]) > s.a[1].max(s.b[1]) {
                continue;
            }
            let adjacent = s.ring == t.ring && (s.k + 1) % s.n == t.k || s.ring == t.ring && (t.k + 1) % t.n == s.k;
            if adjacent {
                // Shared end: only a fold-back along the same line is an overlap.
                let (first, second) = if (s.k + 1) % s.n == t.k { (s, t) } else { (t, s) };
                if orient2d(first.a, first.b, second.b) == Sign::Zero {
                    let d1 = [first.b[0] - first.a[0], first.b[1] - first.a[1]];
                    let d2 = [second.b[0] - second.a[0], second.b[1] - second.a[1]];
                    if d1[0] * d2[0] + d1[1] * d2[1] <= 0.0 {
                        return Err(PaveError::BoundaryIntersects);
                    }
                }
                continue;
            }
            if segments_touch(s.a, s.b, t.a, t.b) {
                return Err(PaveError::BoundaryIntersects);
            }
        }
    }
    Ok(())
}

/// Work units (triangle visits of the recombination, plus 8 per vertex) a failing region may
/// spend on later attempts, as a multiple of the work of its first attempt, plus a floor. A
/// count, never wall-clock, so the cut-off is deterministic: it is about 0.4 microseconds per
/// unit in release on the development machine, so a small face fails in a fraction of a
/// second and a large one after a few attempts.
const WORK_FACTOR: usize = 6;
/// The floor of the work budget (see [`WORK_FACTOR`]).
const MIN_WORK: usize = 1_000_000;
/// Smallest per-attempt vertex cap.
const MIN_ATTEMPT_CAP: usize = 4_000;
/// One later attempt may grow to this many times the first attempt's vertices before it is
/// abandoned (finer strategies would otherwise blow up on thin regions).
const ATTEMPT_GROWTH: usize = 4;
/// Hard cap on the number of attempts.
const MAX_ATTEMPTS: usize = 40;
/// A strip grid at least this good (corner sine) is taken before any triangle route: it is
/// (nearly) a perfect structured mesh.
const STRIP_PERFECT: f64 = 0.95;
/// A strip grid at least this good (corner sine) is taken right after the default placement
/// fails; a worse one only after the whole ladder failed.
const STRIP_GOOD: f64 = 0.3;
/// ... and only if its quads are at most this many times the wanted size.
const STRIP_MAX_SCALE: f64 = 1.2;

/// The production ladder: first the strategies that recovered the most regions in the
/// surveys of `tests.rs` (`survey_cover`), then every other candidate.
fn ladder() -> Vec<Strategy> {
    let c = candidates();
    let first = [0usize, 73, 33, 93, 10, 70, 71, 35, 40, 12, 3, 6, 21, 72];
    let mut v: Vec<Strategy> = first.iter().filter_map(|i| c.get(*i).copied()).collect();
    // The remaining candidates follow in their original order (they matter for metrics the
    // survey corpus does not cover); the work budget bounds the total.
    v.extend(c.iter().enumerate().filter(|(i, _)| !first.contains(i)).map(|(_, s)| *s));
    v
}

/// Points, quads, rounds and size scale of one successful strategy.
type Solution = (Vec<[f64; 2]>, Vec<[u32; 4]>, u32, f64);

/// One entry of the strategy ladder.
#[derive(Clone, Copy)]
struct Strategy {
    rows: bool,
    fill: bool,
    tuning: Tuning,
}

/// Candidate strategies the production ladder is chosen from: the default placement first, then variations of the size scale, the row
/// distance and the thinning, then no rows at all. The first strategy that recombines
/// completely wins.
fn candidates() -> Vec<Strategy> {
    let t = |hscale, row_factor, drop_run| Tuning { hscale, row_factor, drop_run };
    let mut v = vec![Strategy { rows: true, fill: true, tuning: t(1.0, 1.0, 2) }];
    for drop_run in [2, 1, 3] {
        for hscale in [1.0, 1.25, 0.8, 1.5, 0.65, 2.0] {
            for row_factor in [1.0, 0.9, 1.1, 0.8, 1.2] {
                if (hscale, row_factor, drop_run) != (1.0, 1.0, 2) {
                    v.push(Strategy { rows: true, fill: true, tuning: t(hscale, row_factor, drop_run) });
                }
            }
        }
    }
    v.push(Strategy { rows: false, fill: false, tuning: t(1.0, 1.0, 2) });
    for hscale in [1.0, 1.6, 0.8] {
        v.push(Strategy { rows: false, fill: true, tuning: t(hscale, 1.0, 2) });
    }
    v
}

/// Meshes the region bounded by `loops` with quads.
///
/// `loops[0]` is the outer loop and the rest are holes; either handedness of the outer loop
/// is accepted (holes must wind opposite to it, so that the interior is on the same side of
/// every segment). Boundary nodes are kept exactly and never split; the total number of
/// boundary nodes must be even.
///
/// `metric(uv) = [E, F, G]` is the first fundamental form `[du.du, du.dv, dv.dv]` of the
/// surface being meshed, so lengths and angles are those of the surface. `size(uv)` is the
/// wanted metric edge length; by default the boundary segment lengths are interpolated over
/// the boundary-node triangulation and limited by `opts.gradation`.
///
/// Fails with a typed [`PaveError`]; see its variants. Deterministic and sequential.
pub fn pave_region(
    loops: &[Vec<[f64; 2]>],
    metric: &dyn Fn([f64; 2]) -> [f64; 3],
    size: Option<&dyn Fn([f64; 2]) -> f64>,
    opts: &PaveOptions,
) -> Result<QuadMesh2, PaveError> {
    let ladder = ladder();
    match pave_with(loops, metric, size, opts, &ladder) {
        // A size finer than the boundary spacing can leave every placement without a perfect
        // matching: fall back to the size the boundary allows and report how much coarser
        // than asked the result is (`size_scale`).
        Err(PaveError::NoPerfectMatching { .. } | PaveError::Internal(_)) if size.is_some() => {
            let mut m = pave_with(loops, metric, None, opts, &ladder)?;
            if let Some(f) = size {
                m.size_scale = measured_scale(&m, metric, f);
            }
            Ok(m)
        }
        r => r,
    }
}

/// Geometric mean over the quads of (mean metric edge length) / (wanted size at the centre).
fn measured_scale(m: &QuadMesh2, metric: &dyn Fn([f64; 2]) -> [f64; 3], size: &dyn Fn([f64; 2]) -> f64) -> f64 {
    let mut log_sum = 0.0;
    for q in &m.quads {
        let p: [[f64; 2]; 4] = core::array::from_fn(|k| m.points[q[k] as usize]);
        let c = [0.25 * (p[0][0] + p[1][0] + p[2][0] + p[3][0]), 0.25 * (p[0][1] + p[1][1] + p[2][1] + p[3][1])];
        let g = metric(c);
        let edge = (0..4)
            .map(|k| {
                let d = [p[(k + 1) % 4][0] - p[k][0], p[(k + 1) % 4][1] - p[k][1]];
                (g[0] * d[0] * d[0] + 2.0 * g[1] * d[0] * d[1] + g[2] * d[1] * d[1]).max(0.0).sqrt()
            })
            .sum::<f64>()
            / 4.0;
        log_sum += (edge / size(c)).ln();
    }
    let s = (log_sum / m.quads.len().max(1) as f64).exp();
    if s.is_finite() { s } else { 1.0 }
}

/// [`pave_region`] over an explicit strategy ladder.
fn pave_with(
    loops: &[Vec<[f64; 2]>],
    metric: &dyn Fn([f64; 2]) -> [f64; 3],
    size: Option<&dyn Fn([f64; 2]) -> f64>,
    opts: &PaveOptions,
    ladder: &[Strategy],
) -> Result<QuadMesh2, PaveError> {
    if loops.is_empty() {
        return Err(PaveError::LoopTooSmall { ring: 0, nodes: 0 });
    }
    for (ring, l) in loops.iter().enumerate() {
        if l.len() < 3 {
            return Err(PaveError::LoopTooSmall { ring, nodes: l.len() });
        }
    }
    let nb: usize = loops.iter().map(Vec::len).sum();
    if loops.iter().flatten().any(|p| !(p[0].is_finite() && p[1].is_finite())) {
        return Err(PaveError::NonFinite);
    }
    if nb % 2 == 1 {
        return Err(PaveError::OddBoundary { total: nb });
    }
    if !(opts.gradation.is_finite() && opts.gradation > 0.0) {
        return Err(PaveError::BadMetric);
    }
    let uv: Vec<[f64; 2]> = loops.iter().flatten().copied().collect();
    let mut ranges: Vec<Range<usize>> = Vec::with_capacity(loops.len());
    let mut at = 0;
    for l in loops {
        ranges.push(at..at + l.len());
        at += l.len();
    }
    check_boundary(&uv, &ranges)?;
    let area = cdt::signed_area(&loops[0]);
    if !(area != 0.0) {
        return Err(PaveError::BoundaryIntersects);
    }
    let reflect = area < 0.0;

    // Mean first fundamental form over the boundary nodes defines the affine frame.
    let mut g_mean = [0.0; 3];
    for p in &uv {
        let g = metric(*p);
        if Met::from_sym(g).is_none() {
            return Err(PaveError::BadMetric);
        }
        for k in 0..3 {
            g_mean[k] += g[k] / nb as f64;
        }
    }
    let frame = Frame::new(g_mean, reflect).ok_or(PaveError::BadMetric)?;
    let space = Space { frame, metric, size };
    let w: Vec<[f64; 2]> = uv.iter().map(|p| frame.to_w(*p)).collect();
    let cdt = Cdt::build(&w, &ranges)?;

    // Boundary-derived size field; an explicit size is limited by it (gradation).
    let size_map = {
        let mut seg_len = vec![0.0; nb];
        let mut prev = vec![0usize; nb];
        for r in &ranges {
            let n = r.len();
            for k in 0..n {
                let (i, j) = (r.start + k, r.start + (k + 1) % n);
                prev[j] = i;
                let (a, b) = (w[i], w[j]);
                let mid = [0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])];
                seg_len[i] = space.met(mid)?.len([b[0] - a[0], b[1] - a[1]]);
            }
        }
        Some(SizeMap::new(&cdt, nb, &seg_len, &prev, opts.gradation))
    };
    // A structured grid for a single loop whose nodes split into four sides with equal
    // opposite counts (thin strips); the triangle route cannot recombine those.
    let mut strip: Option<strip::Strip> = if ranges.len() == 1 { strip::strip_fill(&w) } else { None };
    let finish_strip = |s: strip::Strip| -> Result<Solution, PaveError> {
        let mut pts = w.clone();
        pts.extend_from_slice(&s.interior);
        let met = pts.iter().map(|p| space.met(*p)).collect::<Result<Vec<_>, _>>()?;
        smooth::smooth(&mut pts, &met, &s.quads, nb, opts.smooth_iters);
        // The grid ignores the size field: report how much coarser (or finer) its quads are
        // than the wanted size, as the geometric mean of edge length over wanted size.
        let mut probe = Mesher::new(&space, cdt.clone(), nb, size_map.clone(), Tuning { hscale: 1.0, row_factor: 1.0, drop_run: 2 })?;
        let mut log_sum = 0.0;
        for q in &s.quads {
            let p: [[f64; 2]; 4] = core::array::from_fn(|k| pts[q[k] as usize]);
            let c = [0.25 * (p[0][0] + p[1][0] + p[2][0] + p[3][0]), 0.25 * (p[0][1] + p[1][1] + p[2][1] + p[3][1])];
            let m = space.met(c)?;
            let edge = (0..4).map(|k| m.len([p[(k + 1) % 4][0] - p[k][0], p[(k + 1) % 4][1] - p[k][1]])).sum::<f64>() / 4.0;
            log_sum += (edge / probe.wanted(c)?).ln();
        }
        let scale = (log_sum / s.quads.len().max(1) as f64).exp();
        Ok((pts, s.quads, 0, if scale.is_finite() { scale } else { 1.0 }))
    };

    // The ladder of strategies: the first that recombines completely wins. Coarse regions
    // (a few boundary nodes against the size) are where the default placement can leave a
    // triangle dual without a perfect matching. A failing region is cut off by counts
    // (vertices of the failed attempts), never by the clock.
    let mut last_err = PaveError::Internal("no strategy ran");
    let mut result: Option<Solution> = None;
    let mut held: Option<Solution> = None;
    let (mut spent, mut first, mut first_work) = (0usize, 0usize, 0usize);
    // A perfectly regular grid cannot be beaten by an unstructured mesh: take it at once.
    if let Some(s) = strip.take_if(|s| s.quality >= STRIP_PERFECT) {
        let sol = finish_strip(s)?;
        if sol.3 <= STRIP_MAX_SCALE {
            result = Some(sol);
        } else {
            held = Some(sol);
        }
    }
    for (si, strat) in ladder.iter().enumerate().take(MAX_ATTEMPTS) {
        if result.is_some() {
            break;
        }
        if si == 1
            && let Some(s) = strip.take_if(|s| s.quality >= STRIP_GOOD)
        {
            // Taken at once unless it is much coarser than the wanted size (then the
            // ladder may still find a mesh that follows the size).
            let sol = finish_strip(s)?;
            if sol.3 <= STRIP_MAX_SCALE {
                result = Some(sol);
                break;
            }
            held = Some(sol);
        }
        let mut mesher = Mesher::new(&space, cdt.clone(), nb, size_map.clone(), strat.tuning)?;
        if si >= 1 {
            mesher.limit_vertices((ATTEMPT_GROWTH * first).max(MIN_ATTEMPT_CAP));
        }
        let run = (|| {
            if strat.rows {
                mesher.rows(&ranges)?;
            }
            if strat.fill {
                mesher.fill()?;
            }
            matching::recombine(&mut mesher, opts.max_rounds)
        })();
        match run {
            Ok((quads, rounds)) => {
                let Mesher { cdt, met, .. } = mesher;
                let mut pts = cdt.pts;
                smooth::smooth(&mut pts, &met, &quads, nb, opts.smooth_iters);
                result = Some((pts, quads, rounds, strat.tuning.hscale));
                break;
            }
            Err(e @ (PaveError::NoPerfectMatching { .. } | PaveError::Internal(_))) => {
                last_err = e;
                let n = mesher.cdt.pts.len();
                let w = mesher.work as usize + 8 * n;
                spent += w;
                if si == 0 {
                    first = n.max(nb);
                    first_work = w;
                }
                if si >= 1 && spent > WORK_FACTOR * first_work + MIN_WORK {
                    break;
                }
            }
            Err(e) => return Err(e),
        }
    }
    if result.is_none() {
        result = match held {
            Some(sol) => Some(sol),
            None => strip.take().map(finish_strip).transpose()?,
        };
    }
    let Some((pts, quads, rounds, size_scale)) = result else {
        return Err(last_err);
    };

    // Back to the parameter space; boundary nodes are the input, bit for bit.
    let mut points: Vec<[f64; 2]> = uv;
    points.extend(pts[nb..].iter().map(|p| frame.to_uv(*p)));
    let want = if reflect { Sign::Negative } else { Sign::Positive };
    for q in &quads {
        for k in 0..4 {
            if orient2d(points[q[(k + 3) % 4] as usize], points[q[k] as usize], points[q[(k + 1) % 4] as usize]) != want {
                return Err(PaveError::Degenerate);
            }
        }
    }
    Ok(QuadMesh2 { points, quads, rounds, size_scale })
}

#[cfg(test)]
mod tests;
