//! Parameter ranges of edges on their carrier curves.

use sm_core::Point3;
use sm_geom_api::{CurveDef, Interval};
use sm_topo_api::Sense;

use crate::error::StepError;

/// The outcome of fitting an edge to its curve.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct EdgeFit {
    /// Ascending parameter range.
    pub range: Interval,
    /// `Forward` if the edge starts at `range.lo`.
    pub sense: Sense,
    /// Largest distance of an end vertex from the curve.
    pub gap: f64,
    /// True when `same_sense` contradicted the vertices on a non-periodic curve and the sense
    /// was taken from the geometry.
    pub repaired: bool,
}

fn mismatch(msg: impl Into<String>) -> StepError {
    // The caller replaces the placeholder id with the id of the edge.
    StepError::GeometryMismatch { id: 0, msg: msg.into() }
}

/// Parameter range and sense of an edge on `def`.
///
/// `p_start` and `p_end` are the edge's vertices, `same_sense` is `EDGE_CURVE.same_sense`
/// (the edge runs along increasing curve parameter), `closed` says the two vertices are the
/// same. Parameters come from `ParametricCurve::project`. The returned range is ascending and
/// `Sense::Forward` means the edge starts at its lower end.
///
/// - A closed circle/ellipse (or periodic curve) edge gets `[t0, t0 + period]`.
/// - A closed B-spline edge needs the vertex at the curve start, otherwise
///   [`StepError::Unsupported`].
/// - A vertex farther than `tol` from the curve gives [`StepError::GeometryMismatch`] (with
///   `id: 0`; callers that know the edge id replace it).
pub fn edge_range(def: &CurveDef, p_start: Point3, p_end: Point3, same_sense: bool, closed: bool, tol: f64) -> Result<(Interval, Sense), StepError> {
    fit_edge(def, p_start, p_end, same_sense, closed, tol).map(|f| (f.range, f.sense))
}

pub(crate) fn fit_edge(def: &CurveDef, p_start: Point3, p_end: Point3, same_sense: bool, closed: bool, tol: f64) -> Result<EdgeFit, StepError> {
    let curve = def.as_curve();
    let dom = curve.domain();
    let ps = curve.project(p_start, None).map_err(|e| mismatch(format!("cannot project the start vertex: {e}")))?;
    let pe = if closed { ps } else { curve.project(p_end, None).map_err(|e| mismatch(format!("cannot project the end vertex: {e}")))? };
    let gap = ps.distance.max(pe.distance);
    if gap.is_nan() || gap > tol {
        return Err(mismatch(format!("vertex lies {gap:e} from the edge curve (limit {tol:e})")));
    }
    let side = if same_sense { Sense::Forward } else { Sense::Reversed };
    if let Some(period) = dom.period {
        let norm = |t: f64| -> f64 {
            let mut t = t;
            if t >= dom.range.lo + period * (1.0 - 1e-12) {
                t -= period;
            }
            if (t - dom.range.lo).abs() < period * 1e-12 {
                t = dom.range.lo;
            }
            t
        };
        let t0 = norm(ps.t);
        if closed {
            return Ok(EdgeFit { range: Interval::new(t0, t0 + period), sense: side, gap, repaired: false });
        }
        let te = norm(pe.t);
        if (te - t0).abs() < period * 1e-12 {
            return Err(mismatch("start and end vertex lie at the same curve parameter of a periodic curve"));
        }
        return Ok(if same_sense {
            let te = if te < t0 { te + period } else { te };
            EdgeFit { range: Interval::new(t0, te), sense: Sense::Forward, gap, repaired: false }
        } else {
            let te = if te > t0 { te - period } else { te };
            EdgeFit { range: Interval::new(te, t0), sense: Sense::Reversed, gap, repaired: false }
        });
    }
    if closed {
        return match def {
            CurveDef::Nurbs(_) => {
                let (lo, hi) = (dom.range.lo, dom.range.hi);
                let at_start = (curve.eval(lo) - p_start).norm() <= tol && (curve.eval(hi) - p_start).norm() <= tol;
                if at_start {
                    Ok(EdgeFit { range: dom.range, sense: side, gap, repaired: false })
                } else {
                    Err(StepError::Unsupported { id: 0, entity: "closed B-spline edge whose vertex is not at the curve start".into() })
                }
            }
            _ => Err(mismatch("a closed edge needs a closed curve")),
        };
    }
    let ts = dom.range.clamp(ps.t);
    let te = dom.range.clamp(pe.t);
    if ts == te {
        return Err(mismatch("start and end vertex lie at the same curve parameter"));
    }
    let ascending = ts < te;
    let repaired = ascending != same_sense;
    let (range, sense) = if ascending { (Interval::new(ts, te), Sense::Forward) } else { (Interval::new(te, ts), Sense::Reversed) };
    Ok(EdgeFit { range, sense, gap, repaired })
}

/// Give a placeholder error the id of the edge being processed.
pub(crate) fn with_id(e: StepError, id: u64) -> StepError {
    match e {
        StepError::GeometryMismatch { msg, .. } => StepError::GeometryMismatch { id, msg },
        StepError::Unsupported { entity, .. } => StepError::Unsupported { id, entity },
        other => other,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::{FRAC_PI_2, PI, TAU};
    use sm_core::Vector3;
    use sm_geom_api::{Circle, Frame, Line, NurbsCurve};

    fn circle() -> CurveDef {
        CurveDef::Circle(Circle::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 2.0).expect("circle"))
    }

    fn p(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(x, y, z)
    }

    #[test]
    fn closed_circle_gets_a_full_turn_from_its_vertex() {
        let (r, s) = edge_range(&circle(), p(2.0, 0.0, 0.0), p(2.0, 0.0, 0.0), true, true, 1e-6).expect("range");
        assert_eq!((r.lo, r.hi, s), (0.0, TAU, Sense::Forward));
        // A vertex at angle -2.4e-16 (OCC) is snapped to t0 = 0.
        let (r, s) = edge_range(&circle(), p(2.0, -4.9e-16, 0.0), p(2.0, -4.9e-16, 0.0), false, true, 1e-6).expect("range");
        assert_eq!((r.lo, r.hi, s), (0.0, TAU, Sense::Reversed));
        let (r, _) = edge_range(&circle(), p(0.0, 2.0, 0.0), p(0.0, 2.0, 0.0), true, true, 1e-6).expect("range");
        assert!((r.lo - FRAC_PI_2).abs() < 1e-15 && (r.hi - r.lo - TAU).abs() < 1e-15);
    }

    #[test]
    fn circular_arcs_follow_the_sense() {
        let (a, b) = (p(2.0, 0.0, 0.0), p(0.0, 2.0, 0.0));
        let (r, s) = edge_range(&circle(), a, b, true, false, 1e-6).expect("fwd");
        assert!((r.lo - 0.0).abs() < 1e-15 && (r.hi - FRAC_PI_2).abs() < 1e-15 && s == Sense::Forward);
        // The same two vertices with the edge running against the curve: the long way round.
        let (r, s) = edge_range(&circle(), a, b, false, false, 1e-6).expect("rev");
        assert!((r.hi - 0.0).abs() < 1e-15 && (r.lo + 1.5 * PI).abs() < 1e-12 && s == Sense::Reversed);
        // Wrapping past 2 pi: from 270 degrees to 90 degrees counter-clockwise.
        let (r, s) = edge_range(&circle(), p(0.0, -2.0, 0.0), b, true, false, 1e-6).expect("wrap");
        assert!((r.hi - r.lo - PI).abs() < 1e-12 && s == Sense::Forward);
    }

    #[test]
    fn line_ranges_and_repair() {
        let l = CurveDef::Line(Line::new(p(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).expect("line"));
        let f = fit_edge(&l, p(1.0, 0.0, 2.0), p(1.0, 0.0, 5.0), true, false, 1e-6).expect("fit");
        assert_eq!((f.range.lo, f.range.hi, f.sense, f.repaired), (2.0, 5.0, Sense::Forward, false));
        let f = fit_edge(&l, p(1.0, 0.0, 5.0), p(1.0, 0.0, 2.0), false, false, 1e-6).expect("fit");
        assert_eq!((f.range.lo, f.range.hi, f.sense, f.repaired), (2.0, 5.0, Sense::Reversed, false));
        // same_sense contradicts the vertices: repaired from the geometry.
        let f = fit_edge(&l, p(1.0, 0.0, 5.0), p(1.0, 0.0, 2.0), true, false, 1e-6).expect("fit");
        assert_eq!((f.sense, f.repaired), (Sense::Reversed, true));
    }

    #[test]
    fn vertex_off_the_curve_is_a_geometry_mismatch() {
        let l = CurveDef::Line(Line::new(p(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).expect("line"));
        let e = edge_range(&l, p(0.0, 0.0, 0.0), p(1.0, 1e-3, 0.0), true, false, 1e-6).expect_err("off");
        assert!(matches!(e, StepError::GeometryMismatch { .. }), "{e:?}");
        // Within the limit but above the weld tolerance: accepted, the gap is reported.
        let f = fit_edge(&l, p(0.0, 0.0, 0.0), p(1.0, 5e-7, 0.0), true, false, 1e-6).expect("within");
        assert!((f.gap - 5e-7).abs() < 1e-15);
    }

    #[test]
    fn closed_bspline_needs_the_vertex_at_the_curve_start() {
        // A closed quadratic loop through the unit square corners (polynomial, clamped).
        let c = NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 4.0], vec![p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(1.0, 1.0, 0.0), p(0.0, 1.0, 0.0), p(0.0, 0.0, 0.0)], None).expect("nurbs");
        let d = CurveDef::Nurbs(c);
        let (r, s) = edge_range(&d, p(0.0, 0.0, 0.0), p(0.0, 0.0, 0.0), true, true, 1e-6).expect("closed");
        assert_eq!((r.lo, r.hi, s), (0.0, 4.0, Sense::Forward));
        let e = edge_range(&d, p(1.0, 0.0, 0.0), p(1.0, 0.0, 0.0), true, true, 1e-6).expect_err("not at start");
        assert!(matches!(e, StepError::Unsupported { .. }), "{e:?}");
    }
}
