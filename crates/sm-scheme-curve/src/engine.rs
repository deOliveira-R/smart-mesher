//! From a validated spacing request to curve-parameter fractions.

use crate::arc::ArcMap;
use crate::error::{Result, SpacingError};
use crate::params::{BiasMode, DualMode};
use crate::spacing::{self, StretchSpec};
use rayon::prelude::*;
use sm_geom_api::ParametricCurve;
use sm_scheme_api::geom::edge_param;
use sm_topo_api::CurveCarrier;

/// Sample count per interval of the curvature density (at least [`MIN_CURVATURE_SAMPLES`]).
const SAMPLES_PER_INTERVAL: usize = 64;
const MIN_CURVATURE_SAMPLES: usize = 512;
const PAR_SAMPLES: usize = 1 << 14;
/// Largest `|factor| * length * max curvature` the density accepts.
const MAX_DENSITY_EXPONENT: f64 = 1e6;

/// A validated spacing request.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Spec {
    /// Equal arc length.
    Equal,
    /// Geometric series.
    Bias {
        /// Which parameters were given.
        mode: BiasMode,
        /// Start the series at the edge's far vertex.
        reverse: bool,
    },
    /// Two geometric series meeting in the middle.
    Dualbias(DualMode),
    /// Pinned end edges with smooth interior.
    Stretch {
        /// First edge.
        first: f64,
        /// Last edge.
        last: Option<f64>,
        /// Largest ratio of consecutive edges.
        max_ratio: f64,
    },
    /// Curvature-adapted density.
    Curvature {
        /// The factor `c`.
        factor: f64,
    },
}

/// Curve-parameter fractions `(0, 1)` (from the edge's start vertex, in the sense of
/// `GenCtx::curve_patch`) of the `n - 1` interior nodes of `spec` on the edge with this curve
/// and carrier.
pub fn node_params(curve: &dyn ParametricCurve, cc: CurveCarrier, n: u32, spec: &Spec) -> Result<Vec<f64>> {
    if n == 0 {
        return Err(SpacingError::ZeroCount);
    }
    if n == 1 {
        return Ok(Vec::new());
    }
    let map = ArcMap::new(curve, cc)?;
    let len = map.length();
    let arc = match *spec {
        Spec::Equal => (1..n).map(|k| f64::from(k) / f64::from(n)).collect(),
        Spec::Bias { mode, reverse } => {
            let f = match mode {
                BiasMode::Ratio(r) | BiasMode::RatioFirst { ratio: r, .. } => spacing::bias_fractions(n, r)?,
                BiasMode::First(a) | BiasMode::FirstLast { first: a, .. } => {
                    let r = spacing::bias_ratio_from_first(n, a / len)?;
                    spacing::bias_fractions(n, r)?
                }
            };
            if reverse { spacing::reversed(&f) } else { f }
        }
        Spec::Dualbias(DualMode::Ratio(r)) => spacing::dualbias_fractions(n, r)?,
        Spec::Dualbias(DualMode::First(a)) => {
            let r = spacing::dualbias_ratio_from_first(n, a / len)?;
            spacing::dualbias_fractions(n, r)?
        }
        Spec::Stretch { first, last, max_ratio } => spacing::stretch_fractions(n, &StretchSpec { length: len, first, last, max_ratio })?,
        Spec::Curvature { factor } => return curvature_params(&map, n, factor),
    };
    Ok(map.taus(&arc))
}

/// Curvature-adapted nodes: equal integrals of `rho = (1 + |c| L k)^sign(c)` between nodes
/// (`k` the curvature, `L` the length), by trapezoid cumulative integration over the edge
/// fraction and linear monotone inversion.
fn curvature_params(map: &ArcMap<'_>, n: u32, factor: f64) -> Result<Vec<f64>> {
    if !factor.is_finite() {
        return Err(SpacingError::BadValue { what: "factor", value: factor });
    }
    let (curve, cc) = map.curve();
    let len = map.length();
    let m = (SAMPLES_PER_INTERVAL * n as usize).max(MIN_CURVATURE_SAMPLES);
    let sample = |i: usize| -> (f64, f64) {
        let j = curve.jet(edge_param(&cc, i as f64 / m as f64));
        let speed = j.d1.norm();
        (map.speed(i as f64 / m as f64), j.d1.cross(j.d2).norm() / (speed * speed * speed))
    };
    let samples: Vec<(f64, f64)> = if m >= PAR_SAMPLES { (0..=m).into_par_iter().map(sample).collect() } else { (0..=m).map(sample).collect() };
    let kmax = samples.iter().fold(0.0f64, |a, s| a.max(s.1));
    if !kmax.is_finite() || factor.abs() * len * kmax > MAX_DENSITY_EXPONENT {
        return Err(SpacingError::Unreachable(format!("|factor| * length * max curvature = {} exceeds {MAX_DENSITY_EXPONENT}", factor.abs() * len * kmax)));
    }
    let c = factor.abs() * len;
    let w: Vec<f64> = samples
        .iter()
        .map(|&(g, k)| {
            let base = 1.0 + c * k;
            g * if factor >= 0.0 { base } else { 1.0 / base }
        })
        .collect();
    let mut cum = Vec::with_capacity(m + 1);
    cum.push(0.0);
    let mut acc = 0.0;
    for i in 0..m {
        acc += 0.5 * (w[i] + w[i + 1]) / m as f64;
        cum.push(acc);
    }
    if !(acc.is_finite() && acc > 0.0) {
        return Err(SpacingError::Geometry("curve has zero or non-finite speed".into()));
    }
    let out: Vec<f64> = (1..n)
        .map(|k| {
            let target = acc * f64::from(k) / f64::from(n);
            let j = cum.partition_point(|c| *c <= target).saturating_sub(1).min(m - 1);
            let t = ((target - cum[j]) / (cum[j + 1] - cum[j])).clamp(0.0, 1.0);
            (j as f64 + t) / m as f64
        })
        .collect();
    let mut prev = 0.0;
    for &x in &out {
        if !(x > prev && x < 1.0) {
            return Err(SpacingError::Degenerate("curvature spacing: interior nodes coincide in double precision".into()));
        }
        prev = x;
    }
    Ok(out)
}
