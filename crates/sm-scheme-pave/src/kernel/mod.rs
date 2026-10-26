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

/// Triangle-attempts a failing region may spend on the strategy ladder (at least two
/// strategies always run).
const ATTEMPT_BUDGET: usize = 150_000;

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

/// Points, quads and rounds of one successful strategy.
type Solution = (Vec<[f64; 2]>, Vec<[u32; 4]>, u32);

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
    pave_with(loops, metric, size, opts, &ladder())
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

    // Default size field from the boundary segments.
    let size_map = if size.is_none() {
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
    } else {
        None
    };
    let area_w = cdt.triangles().map(|t| {
        let v = cdt.tv[t as usize];
        cdt::signed_area(&[cdt.pts[v[0] as usize], cdt.pts[v[1] as usize], cdt.pts[v[2] as usize]])
    }).sum::<f64>();

    // The ladder of strategies: the first that recombines completely wins. Coarse regions
    // (a few boundary nodes against the size) are where the default placement can leave a
    // triangle dual without a perfect matching.
    let mut last_err = PaveError::Internal("no strategy ran");
    let mut result: Option<Solution> = None;
    for (si, strat) in ladder.iter().enumerate() {
        let mut mesher = Mesher::new(&space, cdt.clone(), nb, size_map.clone(), area_w, strat.tuning)?;
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
                result = Some((pts, quads, rounds));
                break;
            }
            Err(e @ (PaveError::NoPerfectMatching { .. } | PaveError::Internal(_))) => {
                last_err = e;
                // Work budget in triangles: small regions try the whole ladder, huge ones a
                // few strategies (a failed attempt costs about as much as a successful one).
                if si >= 1 && (si + 1) * mesher.cdt.len().max(1) >= ATTEMPT_BUDGET {
                    break;
                }
            }
            Err(e) => return Err(e),
        }
    }
    let Some((pts, quads, rounds)) = result else {
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
    Ok(QuadMesh2 { points, quads, rounds })
}

#[cfg(test)]
mod tests;
