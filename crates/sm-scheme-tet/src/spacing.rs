//! Ruling INT-7: normalized spacing of interior Steiner points and the quality floor.
//!
//! The spacing of a point `p` is `r = d(p) / h_loc(p)`, `d` the distance to the nearest input
//! triangle (a boundary vertex lies on a triangle, so the triangles decide) and `h_loc` the
//! local size: the nearest triangle's mean edge length graded into the interior by the
//! gradation, lowered by the user size, as in refinement. Only triangles within [`REACH`] of
//! their own size are searched, so `r` is clipped at [`Meter::cap`] (what a point beyond that
//! reach has at least).
//!
//! The quality floor flags tets below [`MIN_DIHEDRAL_DEG`] or [`MIN_MEAN_RATIO`] as located
//! diagnostics ([`crate::QualityDiagnostic`]).

use crate::geom::V3;
use crate::guard::Guard;
use crate::quality::tet_quality;
use crate::{QualityDiagnostic, QualityMetric, SizeSpec, SpacingStats, SuggestedFix, TetOptions};
use rayon::prelude::*;

/// Target of `r`: the height of a regular tet over its base face in units of the edge length.
pub const TARGET_R: f64 = 0.816_496_580_927_726;
/// Reach (in triangle sizes) of the nearest-triangle search of the meter.
const REACH: f64 = 1.5;
/// Quality floor: smallest dihedral angle (degrees) of a tet.
pub const MIN_DIHEDRAL_DEG: f64 = 5.0;
/// Quality floor: smallest mean ratio of a tet.
pub const MIN_MEAN_RATIO: f64 = 0.05;
/// Diagnostics kept in `TetStats::quality_diagnostics` (the worst; the count is exact).
pub const MAX_DIAGNOSTICS: usize = 256;
/// Nearby input triangles listed per diagnostic.
const NEAR_TRIS: usize = 6;

/// Measures `r` for points of one build.
pub(crate) struct Meter<'a, 'o> {
    guard: Guard<'a>,
    opts: &'a TetOptions<'o>,
}

impl<'a, 'o> Meter<'a, 'o> {
    pub fn new(pts: &'a [V3], tris: &'a [[u32; 3]], opts: &'a TetOptions<'o>) -> Self {
        Self { guard: Guard::with_grow(pts, tris, REACH), opts }
    }

    /// The largest value `r` takes: a point past the reach of the search has at least this.
    pub fn cap(&self) -> f64 {
        REACH / (1.0 + (self.opts.gradation - 1.0) * REACH)
    }

    /// Local size at distance `d` from a triangle of size `h`.
    fn h_loc(&self, p: V3, h: f64, d: f64) -> f64 {
        let g1 = self.opts.gradation - 1.0;
        let hi = h + g1 * d;
        let lo = (h - g1 * d).max(0.0);
        let user = match self.opts.size {
            SizeSpec::FromBoundary => f64::INFINITY,
            SizeSpec::Constant(c) => c,
            SizeSpec::Field(f) => {
                let u = f(p);
                if u.is_finite() && u > 0.0 { u } else { f64::INFINITY }
            }
        };
        hi.min(user).max(lo.min(hi))
    }

    /// Normalized spacing of `p` (at most [`Meter::cap`]).
    pub fn r(&self, p: V3) -> f64 {
        match self.guard.nearest_listed(p) {
            Some((d, ti)) => (d / self.h_loc(p, self.guard.tri_h(ti), d)).min(self.cap()),
            None => self.cap(),
        }
    }

    /// Local size at `p` (the nearest triangle's size when none is in reach).
    fn size_at(&self, p: V3) -> f64 {
        match self.guard.nearest_listed(p) {
            Some((d, ti)) => self.h_loc(p, self.guard.tri_h(ti), d),
            None => self.h_loc(p, 1.0, 0.0),
        }
    }

    /// Where `p` should be to have `r = TARGET_R`: `(position, deviation now)` along the line from
    /// the nearest input triangle's closest point through `p`; `None` if no triangle is in reach,
    /// `p` is on the surface or `p` is already at least [`TARGET_R`] away (only points too close
    /// to the surface are pushed).
    pub fn respace_target(&self, p: V3) -> Option<(V3, f64)> {
        let (d, ti) = self.guard.nearest_listed(p)?;
        if d <= 0.0 {
            return None;
        }
        let foot = self.guard.foot(p, ti);
        let dir = crate::geom::scale(crate::geom::sub(p, foot), 1.0 / d);
        let h = self.h_loc(p, self.guard.tri_h(ti), d);
        let dev = TARGET_R - (d / h).min(self.cap());
        if dev <= 0.0 {
            return None;
        }
        let want = TARGET_R * h;
        Some((crate::geom::add(foot, crate::geom::scale(dir, want)), dev))
    }

    /// Deviation of `r(p)` from the target (the quantity relocation minimizes).
    pub fn deviation(&self, p: V3) -> f64 {
        (self.r(p) - TARGET_R).abs()
    }

    /// Distribution of `r` over `pts`.
    pub fn stats(&self, pts: &[V3]) -> SpacingStats {
        let mut r: Vec<f64> = pts.par_iter().map(|&p| self.r(p)).collect();
        let count = r.len();
        let mut s = SpacingStats { count, target: TARGET_R, cap: self.cap(), ..SpacingStats::default() };
        if count == 0 {
            return s;
        }
        r.sort_unstable_by(f64::total_cmp);
        let n = count as f64;
        let mean = r.iter().sum::<f64>() / n;
        s.mean = mean;
        s.std_dev = (r.iter().map(|x| (x - mean) * (x - mean)).sum::<f64>() / n).sqrt();
        s.rms_dev = (r.iter().map(|x| (x - TARGET_R) * (x - TARGET_R)).sum::<f64>() / n).sqrt();
        s.p01 = r[((count - 1) as f64 * 0.01).floor() as usize];
        s.min = r[0];
        s
    }

    /// Tets below the quality floor as `(count, worst diagnostics)`; triangle ids in the
    /// diagnostics are indices into the triangle list this meter was built over.
    pub fn quality_floor(&self, points: &[V3], n_input: usize, tets: &[[u32; 4]]) -> (usize, Vec<QualityDiagnostic>) {
        let per_tet: Vec<[Option<f64>; 2]> = tets
            .par_iter()
            .map(|t| {
                let q = tet_quality(t.map(|i| points[i as usize]));
                [(q.min_dihedral < MIN_DIHEDRAL_DEG).then_some(q.min_dihedral), (q.mean_ratio < MIN_MEAN_RATIO).then_some(q.mean_ratio)]
            })
            .collect();
        let mut found: Vec<(f64, u32, QualityMetric)> = Vec::new();
        for (i, v) in per_tet.iter().enumerate() {
            if let Some(x) = v[0] {
                found.push((x, i as u32, QualityMetric::MinDihedral));
            }
            if let Some(x) = v[1] {
                found.push((x, i as u32, QualityMetric::MeanRatio));
            }
        }
        let total = found.len();
        // Worst first by value / threshold; ties by tet index (canonically sorted), then metric.
        let sev = |x: &(f64, u32, QualityMetric)| match x.2 {
            QualityMetric::MinDihedral => x.0 / MIN_DIHEDRAL_DEG,
            QualityMetric::MeanRatio => x.0 / MIN_MEAN_RATIO,
        };
        found.sort_by(|a, b| sev(a).total_cmp(&sev(b)).then(a.1.cmp(&b.1)).then((a.2 as u8).cmp(&(b.2 as u8))));
        found.truncate(MAX_DIAGNOSTICS);
        let diags = found
            .into_iter()
            .map(|(value, ti, metric)| {
                let t = tets[ti as usize];
                let mut c = [0.0; 3];
                for &x in &t {
                    for (k, ck) in c.iter_mut().enumerate() {
                        *ck += 0.25 * points[x as usize][k];
                    }
                }
                let mut triangles: Vec<u32> = self.guard.near(c, NEAR_TRIS).into_iter().map(|x| x.1).collect();
                triangles.sort_unstable();
                let h_new = 0.5 * self.size_at(c);
                let input_only = t.iter().all(|&x| (x as usize) < n_input);
                let fix = if input_only { SuggestedFix::RefineInputSurface { h: h_new } } else { SuggestedFix::RefineSizeNear { h: h_new } };
                let threshold = match metric {
                    QualityMetric::MinDihedral => MIN_DIHEDRAL_DEG,
                    QualityMetric::MeanRatio => MIN_MEAN_RATIO,
                };
                QualityDiagnostic { position: c, triangles, metric, value, threshold, fix }
            })
            .collect();
        (total, diags)
    }
}
