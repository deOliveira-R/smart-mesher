//! Curves and surfaces from STEP geometry entities, scaled into the target unit.

use sm_core::{Point3, Vector3};
use sm_geom_api::{Circle, Cone, CurveDef, Cylinder, Ellipse, Frame, GeomError, Line, NurbsCurve, NurbsSurface, Plane, Sphere, SurfaceDef, Torus};

use crate::args::{Args, Ents};
use crate::error::StepError;
use crate::part21::{Entity, Param};

/// Geometry decoder for one length/angle context.
#[derive(Clone, Copy)]
pub(crate) struct Geo<'a> {
    pub ents: Ents<'a>,
    /// Multiplier from file length units to target units.
    pub scale: f64,
    /// Radians per file plane-angle unit.
    pub angle: f64,
}

/// Entity names of curves that are supported (the underlying geometry of an edge).
pub(crate) const CURVE_WRAPPERS: [&str; 3] = ["SURFACE_CURVE", "SEAM_CURVE", "INTERSECTION_CURVE"];

fn geom_err(a: &Args<'_>, index: usize, e: GeomError) -> StepError {
    a.bad(index, e.to_string())
}

impl<'a> Geo<'a> {
    fn point(&self, id: u64, by: u64) -> Result<Point3, StepError> {
        let a = self.ents.rec(id, by, "CARTESIAN_POINT")?;
        let c = a.reals(1)?;
        if c.len() != 3 || c.iter().any(|v| !v.is_finite()) {
            return Err(a.bad(1, "expected three finite coordinates"));
        }
        Ok(Point3::new(c[0] * self.scale, c[1] * self.scale, c[2] * self.scale))
    }

    fn direction(&self, id: u64, by: u64) -> Result<Vector3, StepError> {
        let a = self.ents.rec(id, by, "DIRECTION")?;
        let c = a.reals(1)?;
        if c.len() != 3 || c.iter().any(|v| !v.is_finite()) {
            return Err(a.bad(1, "expected three finite direction ratios"));
        }
        if c.iter().all(|v| *v == 0.0) {
            return Err(a.bad(1, "zero direction"));
        }
        Ok(Vector3::new(c[0], c[1], c[2]))
    }

    /// `AXIS2_PLACEMENT_3D` as a frame (origin scaled).
    pub fn frame(&self, id: u64, by: u64) -> Result<Frame, StepError> {
        let a = self.ents.rec(id, by, "AXIS2_PLACEMENT_3D")?;
        let origin = self.point(a.reference(1)?, id)?;
        let axis = a.opt_ref(2)?;
        let refd = a.opt_ref(3)?;
        let r = match (axis, refd) {
            (None, None) => Ok(Frame::at(origin)),
            (Some(z), None) => Frame::from_z(origin, self.direction(z, id)?),
            (None, Some(x)) => Frame::from_zx(origin, Vector3::new(0.0, 0.0, 1.0), self.direction(x, id)?),
            (Some(z), Some(x)) => Frame::from_zx(origin, self.direction(z, id)?, self.direction(x, id)?),
        };
        r.map_err(|e| geom_err(&a, 3, e))
    }

    fn radius(&self, a: &Args<'_>, i: usize) -> Result<f64, StepError> {
        let r = a.finite(i)? * self.scale;
        if r > 0.0 && r.is_finite() { Ok(r) } else { Err(a.bad(i, "radius must be positive")) }
    }

    /// Follow `SURFACE_CURVE`-style wrappers to the id of the 3D curve entity.
    pub fn curve_ref(&self, id: u64, by: u64) -> Result<u64, StepError> {
        let mut cur = id;
        let mut by = by;
        for _ in 0..16 {
            let e = self.ents.get(cur, by)?;
            match CURVE_WRAPPERS.iter().find_map(|n| e.rec(n)) {
                Some(rec) => {
                    let a = Args { id: cur, rec };
                    by = cur;
                    cur = a.reference(1)?;
                    if cur == id {
                        return Err(StepError::Cycle { id });
                    }
                }
                None => return Ok(cur),
            }
        }
        Err(StepError::Cycle { id })
    }

    /// Decode the curve entity `id` (a plain curve, not a wrapper).
    pub fn curve(&self, id: u64, by: u64) -> Result<CurveDef, StepError> {
        let e = self.ents.get(id, by)?;
        let name = e.first().map(|r| r.name.as_str()).unwrap_or("");
        match name {
            "LINE" => {
                let a = self.ents.rec(id, by, "LINE")?;
                let origin = self.point(a.reference(1)?, id)?;
                let v = self.ents.rec(a.reference(2)?, id, "VECTOR")?;
                if v.finite(2)? <= 0.0 {
                    return Err(v.bad(2, "vector magnitude must be positive"));
                }
                let d = self.direction(v.reference(1)?, v.id)?;
                Line::new(origin, d).map(CurveDef::Line).map_err(|e| geom_err(&a, 2, e))
            }
            "CIRCLE" => {
                let a = self.ents.rec(id, by, "CIRCLE")?;
                let frame = self.frame(a.reference(1)?, id)?;
                let r = self.radius(&a, 2)?;
                Circle::new(frame, r).map(CurveDef::Circle).map_err(|e| geom_err(&a, 2, e))
            }
            "ELLIPSE" => {
                let a = self.ents.rec(id, by, "ELLIPSE")?;
                let frame = self.frame(a.reference(1)?, id)?;
                let (ra, rb) = (self.radius(&a, 2)?, self.radius(&a, 3)?);
                Ellipse::new(frame, ra, rb).map(CurveDef::Ellipse).map_err(|e| geom_err(&a, 2, e))
            }
            _ if e.is("B_SPLINE_CURVE_WITH_KNOTS") => self.bspline_curve(id, e),
            _ => Err(StepError::Unsupported { id, entity: e.display_name() }),
        }
    }

    fn bspline_curve(&self, id: u64, e: &Entity) -> Result<CurveDef, StepError> {
        let simple = matches!(e, Entity::Simple(_));
        let kn = e.rec("B_SPLINE_CURVE_WITH_KNOTS").ok_or(StepError::Unsupported { id, entity: e.display_name() })?;
        let kna = Args { id, rec: kn };
        let (head, deg_i, ctl_i, mult_i, knot_i) = if simple {
            (kna, 1, 2, 6, 7)
        } else {
            (Args { id, rec: e.rec("B_SPLINE_CURVE").ok_or(StepError::Unsupported { id, entity: e.display_name() })? }, 0, 1, 0, 1)
        };
        let degree = head.int(deg_i)?;
        if !(1..=15).contains(&degree) {
            return Err(head.bad(deg_i, "degree must be in 1..=15"));
        }
        let control: Vec<Point3> = head.refs(ctl_i)?.into_iter().map(|p| self.point(p, id)).collect::<Result<_, _>>()?;
        let knots = expand_knots(&kna, mult_i, knot_i)?;
        let weights = match e.rec("RATIONAL_B_SPLINE_CURVE") {
            Some(r) => {
                let w = Args { id, rec: r }.reals(0)?;
                if w.len() != control.len() || w.iter().any(|x| !(x.is_finite() && *x > 0.0)) {
                    return Err(Args { id, rec: r }.bad(0, "weights must be positive, one per control point"));
                }
                Some(w)
            }
            None => None,
        };
        NurbsCurve::new(degree as usize, knots, control, weights).map(CurveDef::Nurbs).map_err(|g| geom_err(&head, ctl_i, g))
    }

    /// Decode the surface entity `id`.
    pub fn surface(&self, id: u64, by: u64) -> Result<SurfaceDef, StepError> {
        let e = self.ents.get(id, by)?;
        let name = e.first().map(|r| r.name.as_str()).unwrap_or("");
        match name {
            "PLANE" => {
                let a = self.ents.rec(id, by, "PLANE")?;
                Ok(SurfaceDef::Plane(Plane { frame: self.frame(a.reference(1)?, id)? }))
            }
            "CYLINDRICAL_SURFACE" => {
                let a = self.ents.rec(id, by, "CYLINDRICAL_SURFACE")?;
                let frame = self.frame(a.reference(1)?, id)?;
                let r = self.radius(&a, 2)?;
                Cylinder::new(frame, r).map(SurfaceDef::Cylinder).map_err(|e| geom_err(&a, 2, e))
            }
            "CONICAL_SURFACE" => {
                let a = self.ents.rec(id, by, "CONICAL_SURFACE")?;
                let frame = self.frame(a.reference(1)?, id)?;
                let r = a.finite(2)? * self.scale;
                if r < 0.0 {
                    return Err(a.bad(2, "radius must not be negative"));
                }
                let half = a.finite(3)? * self.angle;
                if !(half > 0.0 && half < core::f64::consts::FRAC_PI_2) {
                    return Err(a.bad(3, "semi-angle must be in (0, pi/2) radians"));
                }
                Cone::new(frame, r, half).map(SurfaceDef::Cone).map_err(|e| geom_err(&a, 3, e))
            }
            "SPHERICAL_SURFACE" => {
                let a = self.ents.rec(id, by, "SPHERICAL_SURFACE")?;
                let frame = self.frame(a.reference(1)?, id)?;
                let r = self.radius(&a, 2)?;
                Sphere::new(frame, r).map(SurfaceDef::Sphere).map_err(|e| geom_err(&a, 2, e))
            }
            "TOROIDAL_SURFACE" => {
                let a = self.ents.rec(id, by, "TOROIDAL_SURFACE")?;
                let frame = self.frame(a.reference(1)?, id)?;
                let (major, minor) = (self.radius(&a, 2)?, self.radius(&a, 3)?);
                if major <= minor {
                    return Err(a.bad(2, "major radius must exceed the minor radius"));
                }
                Torus::new(frame, major, minor).map(SurfaceDef::Torus).map_err(|e| geom_err(&a, 2, e))
            }
            _ if e.is("B_SPLINE_SURFACE_WITH_KNOTS") => self.bspline_surface(id, e),
            _ => Err(StepError::Unsupported { id, entity: e.display_name() }),
        }
    }

    fn bspline_surface(&self, id: u64, e: &Entity) -> Result<SurfaceDef, StepError> {
        let simple = matches!(e, Entity::Simple(_));
        let kn = e.rec("B_SPLINE_SURFACE_WITH_KNOTS").ok_or(StepError::Unsupported { id, entity: e.display_name() })?;
        let kna = Args { id, rec: kn };
        // (u_degree, v_degree, control net, u_mult, v_mult, u_knots, v_knots)
        let (head, idx) = if simple {
            (kna, [1, 2, 3, 8, 9, 10, 11])
        } else {
            (Args { id, rec: e.rec("B_SPLINE_SURFACE").ok_or(StepError::Unsupported { id, entity: e.display_name() })? }, [0, 1, 2, 0, 1, 2, 3])
        };
        let (du, dv) = (head.int(idx[0])?, head.int(idx[1])?);
        if !(1..=15).contains(&du) || !(1..=15).contains(&dv) {
            return Err(head.bad(idx[0], "degrees must be in 1..=15"));
        }
        let rows = head.list(idx[2])?;
        let nu = rows.len();
        let mut control: Vec<Point3> = Vec::new();
        let mut nv = 0;
        for (iu, row) in rows.iter().enumerate() {
            let Param::List(cols) = row else { return Err(head.bad(idx[2], "expected a list of rows of control points")) };
            if iu == 0 {
                nv = cols.len();
            } else if cols.len() != nv {
                return Err(head.bad(idx[2], "control net rows differ in length"));
            }
            for c in cols {
                let Param::Ref(p) = c else { return Err(head.bad(idx[2], "expected control point references")) };
                control.push(self.point(*p, id)?);
            }
        }
        let knots_u = expand_knots_at(&kna, idx[3], idx[5])?;
        let knots_v = expand_knots_at(&kna, idx[4], idx[6])?;
        let weights = match e.rec("RATIONAL_B_SPLINE_SURFACE") {
            Some(r) => {
                let a = Args { id, rec: r };
                let mut w = Vec::with_capacity(nu * nv);
                for row in a.list(0)? {
                    let Param::List(cols) = row else { return Err(a.bad(0, "expected rows of weights")) };
                    for c in cols {
                        match c {
                            Param::Real(v) => w.push(*v),
                            Param::Int(v) => w.push(*v as f64),
                            _ => return Err(a.bad(0, "expected real weights")),
                        }
                    }
                }
                if w.len() != control.len() || w.iter().any(|x| !(x.is_finite() && *x > 0.0)) {
                    return Err(a.bad(0, "weights must be positive and match the control net"));
                }
                Some(w)
            }
            None => None,
        };
        NurbsSurface::new(du as usize, dv as usize, knots_u, knots_v, nu, nv, control, weights).map(SurfaceDef::Nurbs).map_err(|g| geom_err(&head, idx[2], g))
    }
}

fn expand_knots(a: &Args<'_>, mult_i: usize, knot_i: usize) -> Result<Vec<f64>, StepError> {
    expand_knots_at(a, mult_i, knot_i)
}

/// Expand (multiplicity, distinct knot) lists into a full knot vector.
fn expand_knots_at(a: &Args<'_>, mult_i: usize, knot_i: usize) -> Result<Vec<f64>, StepError> {
    let mults = a.ints(mult_i)?;
    let knots = a.reals(knot_i)?;
    if mults.len() != knots.len() {
        return Err(a.bad(knot_i, "knot and multiplicity lists differ in length"));
    }
    let mut out = Vec::new();
    for (m, k) in mults.iter().zip(&knots) {
        if !(1..=16).contains(m) || !k.is_finite() {
            return Err(a.bad(mult_i, "multiplicities must be in 1..=16 and knots finite"));
        }
        out.extend(std::iter::repeat_n(*k, *m as usize));
    }
    Ok(out)
}
