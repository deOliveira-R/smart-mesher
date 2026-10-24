//! The five curve schemes as one parametrized [`Scheme`].

use crate::engine::{Spec, node_params};
use crate::params::{BiasMode, BiasParams, CurvatureParams, DualMode, DualbiasParams, StretchParams, parse};
use crate::spacing;
use sm_core::EntityId;
use sm_scheme_api::{
    Applicability, Caps, FailureKind, GenCtx, GenerateFailure, IntervalCtx, IntervalSink, Params, Patch, PlanCtx, Result as SchemeResult, Scheme,
    SchemeDim, SchemeError, SchemeName, Target, Firmness,
};
use sm_scheme_api::geom;
use sm_topo_api::{TopologyView, TopologyViewExt};

/// Which curve scheme a [`CurveScheme`] is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum CurveKind {
    /// Equal arc length (no parameters).
    Equal,
    /// Geometric series, see [`BiasParams`].
    Bias,
    /// Two geometric series meeting in the middle, see [`DualbiasParams`].
    Dualbias,
    /// Pinned end edges and smooth interior, see [`StretchParams`].
    Stretch,
    /// Curvature-adapted density, see [`CurvatureParams`].
    Curvature,
}

impl CurveKind {
    /// The registered scheme name.
    pub const fn name(self) -> &'static str {
        match self {
            CurveKind::Equal => "equal",
            CurveKind::Bias => "bias",
            CurveKind::Dualbias => "dualbias",
            CurveKind::Stretch => "stretch",
            CurveKind::Curvature => "curvature",
        }
    }
}

/// A curve scheme (`dim = Curve`). Arc-length spacing is converted to parameter fractions here;
/// node keys and positions come from `GenCtx::curve_patch`.
#[derive(Clone, Copy, Debug)]
pub struct CurveScheme {
    kind: CurveKind,
}

/// A request resolved against one edge.
struct Resolved {
    spec: Spec,
    /// Interval count this request implies when it derives one from sizes.
    count: Option<u32>,
}

impl CurveScheme {
    /// The scheme of `kind`.
    pub const fn new(kind: CurveKind) -> Self {
        Self { kind }
    }

    /// Parse and validate `params` for `edge`; `want_count` also derives the implied count.
    fn resolve(&self, topo: &dyn TopologyView, edge: EntityId, params: &Params, want_count: bool) -> Result<Resolved, String> {
        let (a, b) = topo.edge_ends(edge).ok_or_else(|| format!("edge {edge} has no ends"))?;
        let closed = a == b;
        geom::edge_curve(topo, edge).map_err(|e| e.to_string())?;
        let length = || geom::edge_length(topo, edge).map_err(|e| e.to_string());
        let below = |what: &str, size: f64, limit: f64| if size < limit { Ok(()) } else { Err(format!("{what} {size} must be below {limit}")) };
        let count = |r: crate::error::Result<u32>| r.map_err(|e| e.to_string());
        match self.kind {
            CurveKind::Equal => {
                if !params.is_empty() {
                    return Err("equal takes no parameters".into());
                }
                Ok(Resolved { spec: Spec::Equal, count: None })
            }
            CurveKind::Bias => {
                let p: BiasParams = parse(params)?;
                let mode = p.mode()?;
                let reverse = match p.start {
                    None => false,
                    Some(v) if closed => return Err(format!("start {v} is ambiguous on a closed edge")),
                    Some(v) if v == a => false,
                    Some(v) if v == b => true,
                    Some(v) => return Err(format!("start {v} is not an end of edge {edge}")),
                };
                let mut n = None;
                if let BiasMode::First(f) | BiasMode::FirstLast { first: f, .. } | BiasMode::RatioFirst { first: f, .. } = mode {
                    let len = length()?;
                    below("first_size", f, len)?;
                    if let BiasMode::FirstLast { last, .. } = mode {
                        below("last_size", last, len)?;
                        if want_count {
                            n = Some(count(spacing::bias_count_from_sizes(len, f, last))?);
                        }
                    }
                    if let BiasMode::RatioFirst { ratio, .. } = mode
                        && want_count
                    {
                        n = Some(count(spacing::bias_count_from_ratio(len, f, ratio))?);
                    }
                }
                Ok(Resolved { spec: Spec::Bias { mode, reverse }, count: n })
            }
            CurveKind::Dualbias => {
                let p: DualbiasParams = parse(params)?;
                let mode = p.mode()?;
                if let DualMode::First(f) = mode {
                    below("first_size (end edges of a mirrored series, half the length at most)", f, length()? / 2.0 * (1.0 + 1e-12))?;
                }
                Ok(Resolved { spec: Spec::Dualbias(mode), count: None })
            }
            CurveKind::Stretch => {
                if closed {
                    return Err("stretch does not support closed (periodic) curves".into());
                }
                let p: StretchParams = parse(params)?;
                let (first, last, max_ratio) = p.resolved()?;
                let len = length()?;
                below("first_size", first, len)?;
                let mut n = None;
                if let Some(l) = last {
                    below("last_size", l, len)?;
                    if want_count {
                        n = Some(((2.0 * len / (first + l)).round_ties_even().max(1.0)) as u32);
                    }
                }
                Ok(Resolved { spec: Spec::Stretch { first, last, max_ratio }, count: n })
            }
            CurveKind::Curvature => {
                if closed {
                    return Err("curvature does not support closed (periodic) curves".into());
                }
                let p: CurvatureParams = parse(params)?;
                if !p.factor.is_finite() {
                    return Err(format!("factor must be finite, got {}", p.factor));
                }
                Ok(Resolved { spec: if p.factor == 0.0 { Spec::Equal } else { Spec::Curvature { factor: p.factor } }, count: None })
            }
        }
    }
}

impl Scheme for CurveScheme {
    fn name(&self) -> &str {
        self.kind.name()
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Curve
    }
    fn caps(&self, _params: &Params) -> Caps {
        Caps::NONE
    }
    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, params: &Params) -> Applicability {
        let Target::Curve(e) = target else {
            return Applicability::no("not a curve");
        };
        match self.resolve(cx.input.topo, e, params, false) {
            Ok(_) if self.kind == CurveKind::Equal => Applicability::yes(50, "uniform in arc length"),
            Ok(_) => Applicability::yes(40, format!("{} spacing", self.kind.name())),
            Err(r) => Applicability::no(r),
        }
    }
    fn interval_constraints(&self, cx: &IntervalCtx<'_>, target: Target, params: &Params, out: &mut IntervalSink<'_>) -> SchemeResult<()> {
        let Target::Curve(e) = target else {
            return Ok(());
        };
        let bad = |reason: String| SchemeError::BadParams { scheme: SchemeName::new(self.kind.name()), reason };
        let r = self.resolve(cx.input.topo, e, params, true).map_err(bad)?;
        if let Some(n) = r.count {
            out.fix(e, n, Firmness::Soft);
        }
        Ok(())
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, params: &Params) -> Result<Patch, GenerateFailure> {
        let name = self.kind.name();
        let fail = |kind: FailureKind, r: String| GenerateFailure::new(target, name, kind, r);
        let Target::Curve(e) = target else {
            return Err(fail(FailureKind::Internal, "not a curve".into()));
        };
        let res = self.resolve(cx.input.topo, e, params, false).map_err(|r| fail(FailureKind::NotApplicable, r))?;
        let n = cx.intervals(e).ok_or_else(|| fail(FailureKind::Internal, format!("edge {e} has no interval count")))?;
        let (curve, cc) = geom::edge_curve(cx.input.topo, e).map_err(|err| fail(FailureKind::Geometry, err.to_string()))?;
        let fractions = node_params(curve, cc, n, &res.spec).map_err(|err| fail(err.failure_kind(), err.to_string()))?;
        cx.curve_patch(e, &fractions)
    }
}
