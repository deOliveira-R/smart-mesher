//! NURBS as plain data. Evaluation lives in `sm-geom`, behind the same traits.

use crate::GeomError;
use crate::vecs::ParamRange;
use sm_core::Point3;

/// B-spline / NURBS curve data.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NurbsCurve {
    /// Polynomial degree (>= 1).
    pub degree: usize,
    /// Non-decreasing knot vector of length `control.len() + degree + 1`.
    pub knots: Vec<f64>,
    /// Control points.
    pub control: Vec<Point3>,
    /// Positive weights (same length as `control`); `None` means a non-rational B-spline.
    pub weights: Option<Vec<f64>>,
}

/// Tensor-product NURBS surface data; control points are stored `u`-fastest.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NurbsSurface {
    /// Degree in `u`.
    pub degree_u: usize,
    /// Degree in `v`.
    pub degree_v: usize,
    /// Knots in `u`.
    pub knots_u: Vec<f64>,
    /// Knots in `v`.
    pub knots_v: Vec<f64>,
    /// Number of control points in `u`.
    pub count_u: usize,
    /// Number of control points in `v`.
    pub count_v: usize,
    /// Control net, `count_u * count_v` points, index `i + count_u * j`.
    pub control: Vec<Point3>,
    /// Positive weights, same layout as `control`.
    pub weights: Option<Vec<f64>>,
}

fn check_knots(k: &[f64], degree: usize, n: usize) -> Result<(), GeomError> {
    let bad = |m: &str| Err(GeomError::InvalidNurbs(m.to_string()));
    if degree < 1 {
        return bad("degree must be >= 1");
    }
    if n < degree + 1 {
        return bad("need at least degree+1 control points");
    }
    if k.len() != n + degree + 1 {
        return bad("knot vector length must be n + degree + 1");
    }
    if k.windows(2).any(|w| w[0] > w[1] || w[0].is_nan() || w[1].is_nan()) || k.iter().any(|x| !x.is_finite()) {
        return bad("knots must be finite and non-decreasing");
    }
    if k[degree] >= k[n] {
        return bad("empty parameter range");
    }
    Ok(())
}

fn check_weights(w: &Option<Vec<f64>>, n: usize) -> Result<(), GeomError> {
    match w {
        Some(w) if w.len() != n || w.iter().any(|&x| x.is_nan() || x <= 0.0 || !x.is_finite()) => {
            Err(GeomError::InvalidNurbs("weights must be positive and match the control count".into()))
        }
        _ => Ok(()),
    }
}

impl NurbsCurve {
    /// Check structural validity.
    pub fn validate(&self) -> Result<(), GeomError> {
        check_knots(&self.knots, self.degree, self.control.len())?;
        check_weights(&self.weights, self.control.len())
    }
    /// Valid parameter range `[knots[degree], knots[n]]`. Call [`Self::validate`] first.
    pub fn domain(&self) -> ParamRange {
        ParamRange::new(self.knots[self.degree], self.knots[self.control.len()])
    }
}

impl NurbsSurface {
    /// Check structural validity.
    pub fn validate(&self) -> Result<(), GeomError> {
        check_knots(&self.knots_u, self.degree_u, self.count_u)?;
        check_knots(&self.knots_v, self.degree_v, self.count_v)?;
        if self.control.len() != self.count_u * self.count_v {
            return Err(GeomError::InvalidNurbs("control net size mismatch".into()));
        }
        check_weights(&self.weights, self.control.len())
    }
    /// Valid ranges `(u, v)`. Call [`Self::validate`] first.
    pub fn domain(&self) -> (ParamRange, ParamRange) {
        (ParamRange::new(self.knots_u[self.degree_u], self.knots_u[self.count_u]), ParamRange::new(self.knots_v[self.degree_v], self.knots_v[self.count_v]))
    }
}
