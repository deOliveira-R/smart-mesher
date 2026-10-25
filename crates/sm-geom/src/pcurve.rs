//! A pcurve lifted to space: [`CurveOnSurface`] evaluates `S(pcurve(t))` with the chain rule to
//! second order, so a coedge's carrier can be used wherever a [`ParametricCurve`] is expected.

use crate::nurbs::project::newton_curve;
use sm_core::{Point3, Vector3};
use sm_geom_api::{CurveJet, CurveProjection, Domain1, GeomError, Interval, PCurve, ParametricCurve, ParametricSurface, Uv};

/// A curve on a surface: `t -> S(u(t), v(t))`.
#[derive(Clone, Debug)]
pub struct CurveOnSurface<'a> {
    surface: &'a dyn ParametricSurface,
    pc: PCurve,
    range: Interval,
    periodic: bool,
}

impl<'a> CurveOnSurface<'a> {
    /// Combine a surface and a pcurve. A periodic 2D NURBS pcurve gives a periodic curve.
    pub fn new(surface: &'a dyn ParametricSurface, pcurve: &PCurve) -> Result<Self, GeomError> {
        let range = pcurve.range();
        let periodic = match pcurve {
            PCurve::Line2(_) => false,
            PCurve::Polyline2(p) => {
                if p.len() < 2 {
                    return Err(GeomError::Invalid("polyline pcurve needs at least two points"));
                }
                false
            }
            PCurve::Nurbs2(n) => {
                n.validate()?;
                n.periodic
            }
            _ => return Err(GeomError::Unsupported("pcurve kind")),
        };
        Ok(Self { surface, pc: pcurve.clone(), range, periodic })
    }

    /// `(uv, uv', uv'')` of the pcurve at `t`.
    fn uv_jet(&self, t: f64) -> (Uv, Uv, Uv) {
        self.pc.jet(t)
    }
}

impl ParametricCurve for CurveOnSurface<'_> {
    /// The pcurve's breakpoints (surface seams and creases are not reported).
    fn breakpoints(&self) -> Vec<f64> {
        self.pc.breakpoints()
    }

    fn domain(&self) -> Domain1 {
        if self.periodic { Domain1::periodic(self.range.lo, self.range.len()) } else { Domain1::open(self.range.lo, self.range.hi) }
    }

    fn jet(&self, t: f64) -> CurveJet {
        let (uv, d, dd) = self.uv_jet(t);
        let s = self.surface.jet(uv);
        let d1: Vector3 = s.du * d.u + s.dv * d.v;
        let d2: Vector3 = s.duu * (d.u * d.u) + s.duv * (2.0 * d.u * d.v) + s.dvv * (d.v * d.v) + s.du * dd.u + s.dv * dd.v;
        CurveJet { p: s.p, d1, d2 }
    }

    fn project(&self, p: Point3, hint: Option<f64>) -> Result<CurveProjection, GeomError> {
        if !self.range.is_bounded() {
            return Err(GeomError::Invalid("projection needs a bounded pcurve"));
        }
        let n = 128usize;
        let mut best = (self.range.lo, f64::INFINITY);
        for i in 0..=n {
            let t = self.range.lo + self.range.len() * i as f64 / n as f64;
            let d = (self.eval(t) - p).norm();
            if d < best.1 {
                best = (t, d);
            }
        }
        if let Some(h) = hint {
            let d = (self.eval(h) - p).norm();
            if d < best.1 {
                best = (h, d);
            }
        }
        let t = newton_curve(self, p, best.0);
        let point = self.eval(t);
        Ok(CurveProjection::new(t, point, (point - p).norm()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_geom_api::{Cylinder, Frame, Line, Line2};

    #[test]
    fn helix_on_a_cylinder_matches_analytic_derivatives() {
        let cyl = Cylinder::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 2.0).unwrap();
        // (u, v) = (t, 0.5 t): a helix of radius 2 and pitch 0.5 per radian.
        let pc = PCurve::Line2(Line2 { origin: Uv::new(0.0, 0.0), dir: Uv::new(1.0, 0.5), range: Interval::new(0.0, 6.0) });
        let c = CurveOnSurface::new(&cyl, &pc).unwrap();
        for i in 0..=30 {
            let t = 0.2 * i as f64;
            let j = c.jet(t);
            assert!((j.p - Point3::new(2.0 * t.cos(), 2.0 * t.sin(), 0.5 * t)).norm() < 1e-13);
            assert!((j.d1 - Vector3::new(-2.0 * t.sin(), 2.0 * t.cos(), 0.5)).norm() < 1e-13);
            assert!((j.d2 - Vector3::new(-2.0 * t.cos(), -2.0 * t.sin(), 0.0)).norm() < 1e-13);
        }
        // Speed is sqrt(4 + 0.25): arc length of the full range.
        let len = c.length(Interval::new(0.0, 6.0), 1e-10).unwrap();
        assert!((len - 6.0 * (4.25f64).sqrt()).abs() < 1e-9);
        let q = Point3::new(3.0, 0.0, 0.1);
        let pr = c.project(q, None).unwrap();
        assert!(pr.distance < 1.2);
        let _ = Line::new(Point3::default(), Vector3::new(1.0, 0.0, 0.0));
    }
}
