//! Surface-surface intersection: result types, options and the dispatcher.

use super::conic::{Section, plane_quadric, section_to_result};
use super::march;
use core::f64::consts::TAU;
use sm_core::{Point3, Vector3};
use sm_geom_api::{Circle, CurveDef, Frame, GeomError, ImplicitDef, Interval, ModelTolerance, Plane, Sphere, SurfaceDef, Uv};

/// How a marched branch ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum MarchEnd {
    /// The branch returned to its start (closed loop).
    Closed,
    /// The branch left the window or a parameter boundary.
    Boundary,
    /// The surfaces became tangent or a normal degenerated (a singular point of the curve).
    Singular,
    /// The point or step budget was exhausted.
    Limit,
}

/// A curve traced by marching: a polyline of points on both surfaces.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct MarchedCurve {
    /// Points in order; every point satisfies both surfaces to the requested tolerance.
    pub points: Vec<Point3>,
    /// Parameters on the first surface (empty if it has no parametrization).
    pub uv_a: Vec<Uv>,
    /// Parameters on the second surface (empty if it has no parametrization).
    pub uv_b: Vec<Uv>,
    /// True if the branch closes on itself (the last point repeats the first).
    pub closed: bool,
    /// Why the first end of the branch stopped.
    pub start: MarchEnd,
    /// Why the last end of the branch stopped.
    pub end: MarchEnd,
}

impl MarchedCurve {
    /// The polyline in the parameter space of the first surface as a pcurve (the parameter is
    /// the vertex index). Periodic parameters are continuous along the branch, not wrapped.
    /// `None` if the first surface has no parametrization.
    pub fn pcurve_a(&self) -> Option<sm_geom_api::PCurve> {
        (self.uv_a.len() == self.points.len() && self.uv_a.len() >= 2).then(|| sm_geom_api::PCurve::Polyline2(self.uv_a.clone()))
    }

    /// The polyline in the parameter space of the second surface (see [`Self::pcurve_a`]).
    pub fn pcurve_b(&self) -> Option<sm_geom_api::PCurve> {
        (self.uv_b.len() == self.points.len() && self.uv_b.len() >= 2).then(|| sm_geom_api::PCurve::Polyline2(self.uv_b.clone()))
    }

    /// Interpolating cubic NURBS through the traced points ([`crate::fit::interpolate_points`]).
    /// Consecutive duplicate points (a closed loop's repeated end) are dropped first, then the
    /// closing point is restored so the fitted curve is a closed, clamped curve.
    pub fn to_nurbs(&self) -> Result<sm_geom_api::NurbsCurve, GeomError> {
        let mut pts: Vec<Point3> = Vec::with_capacity(self.points.len());
        for p in &self.points {
            if pts.last().is_none_or(|l| (*p - *l).norm() > 0.0) {
                pts.push(*p);
            }
        }
        crate::fit::interpolate_points(&pts, 3)
    }
}

/// Geometry of one intersection curve.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum CurveGeometry {
    /// An exact curve (line, circle, ellipse or rational NURBS) over `range` of its parameter.
    /// Lines may have an infinite range; circles and ellipses have `[0, 2 pi]`.
    Analytic {
        /// The curve.
        curve: CurveDef,
        /// Parameter range of the intersection on the curve.
        range: Interval,
    },
    /// A traced polyline (general case).
    Marched(MarchedCurve),
}

/// One curve of an intersection.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct IntersectionCurve {
    /// The geometry.
    pub geometry: CurveGeometry,
    /// True if the curve is closed.
    pub closed: bool,
}

impl IntersectionCurve {
    pub(crate) fn analytic(curve: CurveDef, range: Interval, closed: bool) -> Self {
        Self { geometry: CurveGeometry::Analytic { curve, range }, closed }
    }
}

/// Result of a surface-surface intersection.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum SsiResult {
    /// No common point.
    Empty,
    /// A single tangential contact point.
    Point(Point3),
    /// The surfaces coincide (same plane, concentric equal spheres).
    Coincident,
    /// The intersection curves, in a deterministic order.
    Curves(Vec<IntersectionCurve>),
}

/// Options for [`intersect_surfaces`].
///
/// **Limits of the general (marched) path**, which closed forms avoid: seeds come from iso-curves
/// of a parameter grid (`seed_lines` per direction), so a component smaller than the grid
/// spacing that touches no iso-curve is missed; tangential contacts are reported as points or
/// as branches that end at a `Singular` point, never continued through; branches that cross at
/// a singular point are traced as separate pieces; and implicit-only against implicit-only
/// pairs are `Unsupported`. Within those limits every returned point satisfies both surfaces to
/// `tol.linear` and the polyline deviates from the true curve by at most `chord_tol`; the tracer
/// rejects steps that jump between nearby branches (corrector displacement, corrected midpoint
/// and a step proportional to the sine of the normal angle).
///
/// **Near-tangent branches.** Where two branches of the intersection approach each other the
/// tracer tightens its chord tolerance to a twelfth of their separation (read off the signed
/// distance across the curve, implicit second surface only) and does not step over the closest
/// approach. With the default options this separates the two loops of perpendicular cylinders
/// (radius difference `dr`, branch gap `2 sqrt(2 dr)`) from `dr = 1e-2` down to `1e-9`, a gap of
/// 9e-5 (0.09 `chord_tol`); `dr = 1e-10` (gap 2.8e-5, 28 `tol.linear`) merges the branches and
/// is the floor. For a NURBS second surface there is no separation estimate: a branch gap of
/// less than about `3 chord_tol` can still be bridged and the trace may run to `max_points`
/// (`MarchEnd::Limit`); tighten `chord_tol` there. A component smaller than the seed grid is
/// found only when it is an isolated contact or a loop against an implicit second surface that
/// the contact search can locate (local minima of the signed distance on a 40 x 40 grid of the
/// marching surface).
///
/// **Tangency.** Every closed form decides a tangent contact by a distance: a gap (or, for
/// internal sphere contact, a nesting gap) of at most `tol.linear` is a contact (`Point`, or the
/// tangent line), a larger gap is `Empty`. A marcher-found isolated contact is reported as
/// `Point` only when it is the whole result and the marching surface meets an implicit one.
/// Pairs whose normals are within `100 tol.angular` (1e-7 rad by default) end a branch as
/// `Singular`. The conic type thresholds (1e-10 relative, parabola against ellipse) are shape
/// classifications, not tolerances.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct SsiOptions {
    /// Model tolerance (`linear` is the position accuracy of marched points).
    pub tol: ModelTolerance,
    /// World box `[xmin, ymin, zmin, xmax, ymax, zmax]` that clips the result. Required for
    /// open conics (parabola, hyperbola) and for marching surfaces with unbounded parameters.
    pub window: Option<[f64; 6]>,
    /// Maximum polyline deviation from the curve (default `1000 * tol.linear`).
    pub chord_tol: Option<f64>,
    /// Maximum step (default: window diagonal / 20, or 1/20 of the surface extent).
    pub max_step: Option<f64>,
    /// Iso-curves per parameter direction used to seed marching.
    pub seed_lines: usize,
    /// Cap on points per branch.
    pub max_points: usize,
}

impl SsiOptions {
    /// Defaults for a tolerance policy.
    pub fn new(tol: ModelTolerance) -> Self {
        Self { tol, window: None, chord_tol: None, max_step: None, seed_lines: 24, max_points: 20_000 }
    }
    /// Copy with the clipping window set.
    pub fn with_window(mut self, window: [f64; 6]) -> Self {
        self.window = Some(window);
        self
    }
    /// Copy with the chord tolerance set.
    pub fn with_chord_tol(mut self, chord_tol: f64) -> Self {
        self.chord_tol = Some(chord_tol);
        self
    }
    /// Copy with the maximum step set.
    pub fn with_max_step(mut self, max_step: f64) -> Self {
        self.max_step = Some(max_step);
        self
    }
    /// Copy with the number of seeding iso-curves set.
    pub fn with_seed_lines(mut self, seed_lines: usize) -> Self {
        self.seed_lines = seed_lines;
        self
    }
}

impl Default for SsiOptions {
    fn default() -> Self {
        Self::new(ModelTolerance::DEFAULT)
    }
}

fn v3(p: Point3) -> Vector3 {
    Vector3::new(p.x, p.y, p.z)
}

fn plane_plane(p: &Plane, q: &Plane, o: &SsiOptions) -> Result<SsiResult, GeomError> {
    let (n1, n2) = (p.frame.z, q.frame.z);
    let d = n1.cross(n2);
    if d.norm() <= o.tol.angular {
        let dist = n1.dot(q.frame.origin - p.frame.origin);
        return Ok(if dist.abs() <= o.tol.linear { SsiResult::Coincident } else { SsiResult::Empty });
    }
    let (c1, c2) = (n1.dot(v3(p.frame.origin)), n2.dot(v3(q.frame.origin)));
    let x0 = (n2.cross(d) * c1 + d.cross(n1) * c2) * (1.0 / d.dot(d));
    let sec = Section::Lines(vec![(Point3::new(x0.x, x0.y, x0.z), d)]);
    section_to_result(&sec, &o.tol, o.window.as_ref(), &|_| true, None)
}

fn sphere_sphere(a: &Sphere, b: &Sphere, o: &SsiOptions) -> Result<SsiResult, GeomError> {
    let c = b.frame.origin - a.frame.origin;
    let d = c.norm();
    if d <= o.tol.linear {
        return Ok(if (a.radius - b.radius).abs() <= o.tol.linear { SsiResult::Coincident } else { SsiResult::Empty });
    }
    let u = c * (1.0 / d);
    let (ra, rb) = (a.radius, b.radius);
    // Tangent contacts by the gap between the surfaces, compared with the linear tolerance.
    let ext = d - (ra + rb);
    let int = (ra - rb).abs() - d;
    if ext > 0.0 || int > 0.0 {
        let gap = ext.max(int);
        if gap > o.tol.linear {
            return Ok(SsiResult::Empty);
        }
        // Midpoint of the two nearest surface points on the line of centres.
        let p = if ext > 0.0 {
            a.frame.origin + u * (ra + 0.5 * gap)
        } else if ra >= rb {
            a.frame.origin + u * (ra - 0.5 * gap)
        } else {
            a.frame.origin + u * -(ra + 0.5 * gap)
        };
        return Ok(SsiResult::Point(p));
    }
    let h = (d * d + ra * ra - rb * rb) / (2.0 * d);
    let rho2 = (ra * ra - h * h).max(0.0);
    let center = a.frame.origin + u * h;
    let rho = rho2.sqrt();
    if rho <= o.tol.linear {
        return Ok(SsiResult::Point(center));
    }
    let circle = Circle::new(Frame::from_z(center, u)?, rho)?;
    Ok(SsiResult::Curves(vec![IntersectionCurve::analytic(CurveDef::Circle(circle), Interval::new(0.0, TAU), true)]))
}

/// Plane against a torus: closed forms for the cuts perpendicular to the axis and through it.
fn plane_torus(p: &Plane, t: &sm_geom_api::Torus, o: &SsiOptions) -> Result<Option<SsiResult>, GeomError> {
    let n = p.frame.z;
    let z = t.frame.z;
    let s = n.cross(z).norm();
    let mut circles: Vec<Circle> = Vec::new();
    if s <= o.tol.angular {
        // Perpendicular to the axis: concentric circles at height h.
        let h = (p.frame.origin - t.frame.origin).dot(z);
        if h.abs() > t.minor + o.tol.linear {
            return Ok(Some(SsiResult::Empty));
        }
        let inner = (t.minor * t.minor - h * h).max(0.0).sqrt();
        let fr = Frame { origin: t.frame.origin + z * h, ..t.frame };
        if inner <= o.tol.linear {
            circles.push(Circle::new(fr, t.major)?);
        } else {
            circles.push(Circle::new(fr, t.major - inner)?);
            circles.push(Circle::new(fr, t.major + inner)?);
        }
    } else if n.dot(z).abs() <= o.tol.angular {
        let d = n.dot(t.frame.origin - p.frame.origin);
        if d.abs() > o.tol.linear {
            return Ok(None);
        }
        // Plane through the axis: two meridian circles of radius `minor` at +-major.
        let u = n.cross(z);
        let u = u * (1.0 / u.norm());
        for sgn in [1.0, -1.0] {
            let center = t.frame.origin + u * (sgn * t.major);
            let w = u.cross(z);
            circles.push(Circle::new(Frame { origin: center, x: u, y: z, z: w }, t.minor)?);
        }
    } else {
        return Ok(None);
    }
    Ok(Some(SsiResult::Curves(circles.into_iter().map(|c| IntersectionCurve::analytic(CurveDef::Circle(c), Interval::new(0.0, TAU), true)).collect())))
}

/// Nappe predicate of a cone (all points for other surfaces) and its apex and axis.
type Nappe = (Box<dyn Fn(Point3) -> bool>, Option<(Point3, Vector3)>);

fn nappe_keep(s: &SurfaceDef, tol: f64) -> Nappe {
    match s {
        SurfaceDef::Cone(c) => {
            let (apex, axis) = (c.apex(), c.frame.z);
            (Box::new(move |p| (p - apex).dot(axis) >= -tol), Some((apex, axis)))
        }
        SurfaceDef::EllipticCone(c) => {
            let (apex, axis) = (c.apex(), c.frame.z);
            (Box::new(move |p| (p - apex).dot(axis) >= -tol), Some((apex, axis)))
        }
        _ => (Box::new(|_| true), None),
    }
}

/// Intersect two surfaces.
///
/// Closed forms (exact [`CurveGeometry::Analytic`] results): plane-plane, plane against every
/// quadric (sphere, cylinder, cone, elliptic cylinder and cone, general quadrics) including the
/// parabola and hyperbola cases (rational NURBS inside `options.window`), sphere-sphere, and
/// the torus cut by planes perpendicular to or through its axis. Everything else (plane-torus
/// in general, quadric-quadric, anything with a NURBS surface) is traced by marching, see
/// [`SsiOptions`] for the documented limits. A cone is the parametric nappe only.
pub fn intersect_surfaces(a: &SurfaceDef, b: &SurfaceDef, options: &SsiOptions) -> Result<SsiResult, GeomError> {
    a.validate()?;
    b.validate()?;
    let tol = &options.tol;
    match (a, b) {
        (SurfaceDef::Plane(p), SurfaceDef::Plane(q)) => return plane_plane(p, q, options),
        (SurfaceDef::Sphere(s1), SurfaceDef::Sphere(s2)) => return sphere_sphere(s1, s2, options),
        (SurfaceDef::Plane(p), other) | (other, SurfaceDef::Plane(p)) => {
            match other {
                SurfaceDef::Torus(t) => {
                    if let Some(r) = plane_torus(p, t, options)? {
                        return Ok(r);
                    }
                }
                SurfaceDef::Nurbs(_) => {}
                _ => {
                    if let Some(ImplicitDef::Quadric(q)) = other.implicit_def() {
                        let sec = plane_quadric(p, &q, tol);
                        let (keep, ray) = nappe_keep(other, tol.linear);
                        return section_to_result(&sec, tol, options.window.as_ref(), &*keep, ray);
                    }
                }
            }
        }
        _ => {}
    }
    march::intersect_general(a, b, options)
}
