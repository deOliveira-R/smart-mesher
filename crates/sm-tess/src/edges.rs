//! Edge sampling: one polyline per topological edge, shared by every face that uses it.
//!
//! Polylines run in edge-forward order (start vertex to end vertex). End points are the
//! vertex positions (`Carrier::Point`); interior points come from the curve. A face never
//! re-evaluates its boundary, so adjacent faces see bit-identical points.

use sm_core::EntityId;
use sm_geom_api::{CurveDef, ParametricCurve};
use sm_topo_api::{Carrier, Sense, TopologyView};

use crate::options::{TessError, TessOptions};
use crate::sample::{self, P3};

/// Samples of one edge in forward order.
pub(crate) struct EdgeSamples {
    /// Curve parameters, one per point including both ends; empty when the edge has no curve.
    pub params: Vec<f64>,
    /// Interior points (`params.len() - 2` of them, or none for a curve-less edge).
    pub interior: Vec<P3>,
}

/// `ring_radius`: radius to sample a circle with if larger than its own (see `tess::edge_rule`).
pub(crate) fn sample_edge(view: &dyn TopologyView, edge: EntityId, closed: bool, o: &TessOptions, ring_radius: Option<f64>) -> Result<EdgeSamples, TessError> {
    let Carrier::Curve(cc) = view.carrier(edge) else {
        return Ok(EdgeSamples { params: Vec::new(), interior: Vec::new() });
    };
    let shapes = view.shapes();
    let curve: &dyn ParametricCurve = shapes.curve(cc.curve).ok_or_else(|| TessError::edge(edge, format!("curve {:?} is not in the geometry store", cc.curve)))?;
    let (lo, hi) = (cc.range.lo, cc.range.hi);
    if !(lo.is_finite() && hi.is_finite() && lo < hi) {
        return Err(TessError::edge(edge, format!("unbounded or empty curve range [{lo}, {hi}]")));
    }
    let eval = |t: f64| -> P3 {
        let p = curve.eval(t);
        [p.x, p.y, p.z]
    };
    let map_err = |e: String| TessError::edge(edge, e);
    let mut params = match shapes.curve_def(cc.curve) {
        Some(CurveDef::Line(_)) => {
            let (a, b) = (eval(lo), eval(hi));
            let len = ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
            sample::uniform(lo, hi, sample::line_segments(len, o).map_err(map_err)?)
        }
        Some(CurveDef::Circle(c)) => {
            let span = hi - lo;
            let full = (span - core::f64::consts::TAU).abs() < 1e-9 || closed;
            sample::uniform(lo, hi, sample::circle_segments(span, c.radius.max(ring_radius.unwrap_or(0.0)), full, o).map_err(map_err)?)
        }
        _ => sample::adaptive_params(&eval, lo, hi, o).map_err(map_err)?,
    };
    if cc.sense == Sense::Reversed {
        params.reverse();
    }
    let n = params.len();
    let interior = params[1..n - 1].iter().map(|&t| eval(t)).collect();
    Ok(EdgeSamples { params, interior })
}
