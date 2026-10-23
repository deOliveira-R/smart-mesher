//! NURBS curves and surfaces as validated *data*. Evaluation is implemented in `sm-geom`
//! (which turns this data into a [`crate::ParametricCurve`] / [`crate::ParametricSurface`]).

use crate::{GeomError, Interval};
use sm_core::Point3;

/// Check a clamped-or-open knot vector: non-decreasing, length `n + degree + 1`, and every
/// knot multiplicity at most `degree + 1`.
fn check_knots(knots: &[f64], degree: usize, n_ctrl: usize) -> Result<(), GeomError> {
    if degree == 0 || n_ctrl < degree + 1 {
        return Err(GeomError::Invalid("need degree >= 1 and at least degree+1 control points"));
    }
    if knots.len() != n_ctrl + degree + 1 {
        return Err(GeomError::Invalid("knot count must be control points + degree + 1"));
    }
    if knots.windows(2).any(|w| w[0].partial_cmp(&w[1]).is_none_or(|o| o.is_gt())) || knots.iter().any(|k| !k.is_finite()) {
        return Err(GeomError::Invalid("knots must be finite and non-decreasing"));
    }
    let mut run = 1;
    for w in knots.windows(2) {
        run = if w[0] == w[1] { run + 1 } else { 1 };
        if run > degree + 1 {
            return Err(GeomError::Invalid("knot multiplicity exceeds degree + 1"));
        }
    }
    if knots[degree] >= knots[n_ctrl] {
        return Err(GeomError::Invalid("empty parameter range"));
    }
    Ok(())
}

fn check_weights(weights: &Option<Vec<f64>>, n: usize) -> Result<(), GeomError> {
    if let Some(w) = weights {
        if w.len() != n {
            return Err(GeomError::Invalid("weight count must equal control point count"));
        }
        if w.iter().any(|v| !(v.is_finite() && *v > 0.0)) {
            return Err(GeomError::Invalid("weights must be positive"));
        }
    }
    Ok(())
}

/// A NURBS curve: degree, knot vector, control points, optional weights (absent = polynomial B-spline).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NurbsCurve {
    /// Polynomial degree `p >= 1`.
    pub degree: usize,
    /// Knots, length `control.len() + degree + 1`, non-decreasing.
    pub knots: Vec<f64>,
    /// Control points.
    pub control: Vec<Point3>,
    /// Positive weights, one per control point; `None` for non-rational.
    pub weights: Option<Vec<f64>>,
}

impl NurbsCurve {
    /// Validate the data (counts, knot order and multiplicity, positive weights).
    pub fn validate(&self) -> Result<(), GeomError> {
        check_knots(&self.knots, self.degree, self.control.len())?;
        check_weights(&self.weights, self.control.len())
    }
    /// The valid parameter range `[knots[p], knots[n]]`.
    pub fn range(&self) -> Interval {
        Interval::new(self.knots[self.degree], self.knots[self.control.len()])
    }
}

/// A tensor-product NURBS surface; control points are stored `u`-major: index `iu * nv + iv`.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NurbsSurface {
    /// Degree in `u`.
    pub degree_u: usize,
    /// Degree in `v`.
    pub degree_v: usize,
    /// Knots in `u`, length `nu + degree_u + 1`.
    pub knots_u: Vec<f64>,
    /// Knots in `v`, length `nv + degree_v + 1`.
    pub knots_v: Vec<f64>,
    /// Number of control points in `u`.
    pub nu: usize,
    /// Number of control points in `v`.
    pub nv: usize,
    /// `nu * nv` control points, `u`-major.
    pub control: Vec<Point3>,
    /// Positive weights (same layout) or `None`.
    pub weights: Option<Vec<f64>>,
}

impl NurbsSurface {
    /// Validate the data.
    pub fn validate(&self) -> Result<(), GeomError> {
        check_knots(&self.knots_u, self.degree_u, self.nu)?;
        check_knots(&self.knots_v, self.degree_v, self.nv)?;
        if self.control.len() != self.nu * self.nv {
            return Err(GeomError::Invalid("control net must be nu * nv"));
        }
        check_weights(&self.weights, self.control.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line() -> NurbsCurve {
        NurbsCurve { degree: 1, knots: vec![0.0, 0.0, 1.0, 1.0], control: vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)], weights: None }
    }

    #[test]
    fn validation() {
        let mut c = line();
        assert!(c.validate().is_ok());
        assert_eq!(c.range(), Interval::new(0.0, 1.0));
        c.knots = vec![0.0, 1.0, 0.5, 1.0];
        assert!(c.validate().is_err());
        let mut c = line();
        c.weights = Some(vec![1.0, -1.0]);
        assert!(c.validate().is_err());
        let mut c = line();
        c.knots = vec![0.0, 0.0, 0.0, 1.0];
        assert!(c.validate().is_err(), "multiplicity 3 > degree + 1");
    }

    #[test]
    fn surface_validation() {
        let s = NurbsSurface {
            degree_u: 1,
            degree_v: 1,
            knots_u: vec![0.0, 0.0, 1.0, 1.0],
            knots_v: vec![0.0, 0.0, 1.0, 1.0],
            nu: 2,
            nv: 2,
            control: vec![Point3::default(); 4],
            weights: None,
        };
        assert!(s.validate().is_ok());
        let bad = NurbsSurface { control: vec![Point3::default(); 3], ..s };
        assert!(bad.validate().is_err());
    }
}
