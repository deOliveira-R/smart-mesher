//! Curves in a surface's parameter space (pcurves), stored per coedge by topology and hashed by
//! their contents.

use crate::nurbs::NurbsCurve2;
use crate::store::{PERIODIC_DEGREE_FLAG, unitless_scale};
use crate::{Interval, ModelTolerance, ShapeValue, Uv};

/// A straight segment in parameter space: `(u, v) = origin + t * dir`, `t` in `range`.
/// A periodic seam is expressed by two such pcurves, at `u = 0` and `u = period`, for the two
/// uses of the seam edge.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Line2 {
    /// Point at `t = 0`.
    pub origin: Uv,
    /// Direction (not necessarily unit; same parameter as the edge curve).
    pub dir: Uv,
    /// Parameter range covered.
    pub range: Interval,
}

/// A coedge's curve in its face's `(u, v)` space. All variants are closed, serializable data
/// with `eval`, `d1` and `range`.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub enum PCurve {
    /// A straight segment.
    Line2(Line2),
    /// A polyline through the given points; the parameter is the vertex index (`0..n-1`).
    Polyline2(Vec<Uv>),
    /// A 2D NURBS curve.
    Nurbs2(NurbsCurve2),
}

impl PCurve {
    /// The parameter range covered.
    pub fn range(&self) -> Interval {
        match self {
            Self::Line2(l) => l.range,
            Self::Polyline2(p) => Interval::new(0.0, p.len().saturating_sub(1) as f64),
            Self::Nurbs2(n) => n.range(),
        }
    }

    /// `(u, v)` at parameter `t` (clamped to the range for the bounded variants).
    pub fn eval(&self, t: f64) -> Uv {
        self.eval_d1(t).0
    }

    /// First derivative `(du/dt, dv/dt)` at `t`. For a polyline it is the slope of the segment
    /// containing `t`.
    pub fn d1(&self, t: f64) -> Uv {
        self.eval_d1(t).1
    }

    fn eval_d1(&self, t: f64) -> (Uv, Uv) {
        match self {
            Self::Line2(l) => (Uv::new(l.origin.u + t * l.dir.u, l.origin.v + t * l.dir.v), l.dir),
            Self::Polyline2(p) => match p.len() {
                0 => (Uv::default(), Uv::default()),
                1 => (p[0], Uv::default()),
                n => {
                    let t = t.clamp(0.0, (n - 1) as f64);
                    let i = (t.floor() as usize).min(n - 2);
                    let f = t - i as f64;
                    let (a, b) = (p[i], p[i + 1]);
                    (Uv::new(a.u + f * (b.u - a.u), a.v + f * (b.v - a.v)), Uv::new(b.u - a.u, b.v - a.v))
                }
            },
            Self::Nurbs2(n) => n.eval_d1(t),
        }
    }

    /// Visit the hash words of the curve (W0g r2, frozen), after the caller's [`Self::tag`]:
    /// a prefix-free sequence where every variable-length run is preceded by its count.
    ///
    /// - `Line2`: `Param` x6 (`origin.u, origin.v, dir.u, dir.v, range.lo, range.hi`).
    /// - `Polyline2`: `Int(n)`, then `Param(u), Param(v)` per point.
    /// - `Nurbs2`: `Int(degree | periodic << 32)`, `Int(knot count)`, `Int(control count)`,
    ///   `Int(0)` without weights or `Int(1), Int(weight count)` with them; then the knots and
    ///   the control `(u, v)` as `Param`, then the weights as `Unitless`.
    ///
    /// `sm-topo-api` hashes pcurves with this (`Param` at `parametric * 2^-10`, weights
    /// scale-free relative to the largest weight, see [`Self::quantized_values`]).
    pub fn for_each_word(&self, mut f: impl FnMut(ShapeValue)) {
        let par = ShapeValue::Param;
        match self {
            Self::Line2(l) => {
                for v in [l.origin.u, l.origin.v, l.dir.u, l.dir.v, l.range.lo, l.range.hi] {
                    f(par(v));
                }
            }
            Self::Polyline2(p) => {
                f(ShapeValue::Int(p.len() as u64));
                for q in p {
                    f(par(q.u));
                    f(par(q.v));
                }
            }
            Self::Nurbs2(n) => {
                f(ShapeValue::Int(n.degree as u64 | if n.periodic { PERIODIC_DEGREE_FLAG } else { 0 }));
                f(ShapeValue::Int(n.knots.len() as u64));
                f(ShapeValue::Int(n.control.len() as u64));
                match &n.weights {
                    None => f(ShapeValue::Int(0)),
                    Some(w) => {
                        f(ShapeValue::Int(1));
                        f(ShapeValue::Int(w.len() as u64));
                    }
                }
                n.knots.iter().for_each(|k| f(par(*k)));
                for q in &n.control {
                    f(par(q.u));
                    f(par(q.v));
                }
                if let Some(w) = &n.weights {
                    w.iter().for_each(|x| f(ShapeValue::Unitless(*x)));
                }
            }
        }
    }

    /// The hash codes of [`Self::for_each_word`]: `Int` as is, `Param` through
    /// [`ModelTolerance::quantize_param`], weights through
    /// [`ModelTolerance::quantize_scale_free_in`] with the largest weight as the scale (the rule
    /// `CurveDef::quantized_values` applies).
    pub fn quantized_values(&self, tol: &ModelTolerance, mut f: impl FnMut(i64)) {
        let scale = unitless_scale(|g| self.for_each_word(g));
        self.for_each_word(|v| f(tol.quantize_shape_in(v, scale)));
    }

    /// Visit every number of the curve in a fixed order, untagged and without counts. Not
    /// prefix-free: hashes use [`Self::for_each_word`] instead.
    pub fn for_each_value(&self, mut f: impl FnMut(f64)) {
        match self {
            Self::Line2(l) => {
                for v in [l.origin.u, l.origin.v, l.dir.u, l.dir.v, l.range.lo, l.range.hi] {
                    f(v);
                }
            }
            Self::Polyline2(p) => {
                for q in p {
                    f(q.u);
                    f(q.v);
                }
            }
            Self::Nurbs2(n) => {
                f(if n.periodic { -(n.degree as f64) } else { n.degree as f64 }); // negative degree: periodic
                n.knots.iter().for_each(|k| f(*k));
                for q in &n.control {
                    f(q.u);
                    f(q.v);
                }
                if let Some(w) = &n.weights {
                    w.iter().for_each(|x| f(*x));
                }
            }
        }
    }

    /// A numeric tag distinguishing the variants (stable across versions).
    pub fn tag(&self) -> u64 {
        match self {
            Self::Line2(_) => 1,
            Self::Polyline2(_) => 2,
            Self::Nurbs2(_) => 3,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn line_polyline_nurbs() {
        let l = PCurve::Line2(Line2 { origin: Uv::new(1.0, 2.0), dir: Uv::new(0.0, 1.0), range: Interval::new(0.0, 3.0) });
        assert_eq!(l.eval(2.0), Uv::new(1.0, 4.0));
        assert_eq!(l.d1(2.0), Uv::new(0.0, 1.0));
        assert_eq!(l.range(), Interval::new(0.0, 3.0));
        let p = PCurve::Polyline2(vec![Uv::new(0.0, 0.0), Uv::new(2.0, 0.0), Uv::new(2.0, 4.0)]);
        assert_eq!(p.range(), Interval::new(0.0, 2.0));
        assert_eq!(p.eval(0.5), Uv::new(1.0, 0.0));
        assert_eq!(p.eval(1.5), Uv::new(2.0, 2.0));
        assert_eq!(p.d1(1.5), Uv::new(0.0, 4.0));
        assert_eq!(p.eval(9.0), Uv::new(2.0, 4.0));
        let n = PCurve::Nurbs2(NurbsCurve2 { degree: 1, knots: vec![0.0, 0.0, 1.0, 1.0], control: vec![Uv::new(0.0, 0.0), Uv::new(2.0, 4.0)], weights: None, periodic: false });
        assert_eq!(n.range(), Interval::new(0.0, 1.0));
        let (a, d) = (n.eval(0.5), n.d1(0.5));
        assert!((a.u - 1.0).abs() < 1e-14 && (a.v - 2.0).abs() < 1e-14 && (d.u - 2.0).abs() < 1e-14 && (d.v - 4.0).abs() < 1e-14);
        let mut vals = Vec::new();
        l.for_each_value(|v| vals.push(v));
        assert_eq!(vals.len(), 6);
    }
}
