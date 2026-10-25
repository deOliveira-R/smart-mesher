//! Geometric queries on a view: face and volume moments, and point-in-face classification.
//!
//! These are *defaults* of [`crate::TopologyView`], computed from the carriers and the geometry
//! store. They are exact for planar faces and accurate (composite Gauss quadrature, error far
//! below the model tolerance) for any face on a parametric surface whose loops are made of
//! curve-carrying edges and degenerate (pole/apex) coedges: planes, cylinders, tori, spheres,
//! cones and bounded NURBS. A degenerate coedge contributes `dv = 0` to the boundary integral;
//! its pcurve (needed on periodic surfaces) carries the parameter jump. Faces without a surface
//! carrier, and virtual faces without a numeric carrier, return [`QueryError::Unsupported`].
//!
//! Method: the surface integral over a trimmed face is turned into a boundary integral in the
//! face's parameter space by Green's theorem, `∬ h du dv = ∮ H dv` with `H(u,v) = ∫ h du`, and
//! both integrals use Gauss-Legendre quadrature (the inner one in panels of at most `pi/8` along
//! periodic parameters). Seams are handled by unwrapping the parameters along each loop.

use crate::derived::TopologyViewExt;
use crate::error::QueryError;
use crate::types::{CellFlags, Dim, Carrier, Side};
use crate::view::TopologyView;
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{Domain2, ParametricSurface, SurfaceJet, Uv};

/// Raw moments of a volume about the ORIGIN (additive across volumes, so per-colour sums are
/// plain sums).
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Moments {
    /// `∫ dV`.
    pub volume: f64,
    /// `∫ x dV`.
    pub first: Vector3,
    /// `∫ x xᵀ dV`.
    pub second: [[f64; 3]; 3],
}

impl Moments {
    /// The centroid, if the volume is non-zero.
    pub fn centroid(&self) -> Option<Point3> {
        (self.volume.abs() > 0.0).then(|| Point3::new(self.first.x / self.volume, self.first.y / self.volume, self.first.z / self.volume))
    }

    /// Second moment about the centroid: `∫ xxᵀ − V c cᵀ` (parallel-axis theorem).
    pub fn central_second(&self) -> [[f64; 3]; 3] {
        let Some(c) = self.centroid() else { return self.second };
        let c = [c.x, c.y, c.z];
        let m = [self.first.x, self.first.y, self.first.z];
        let mut out = self.second;
        for (i, row) in out.iter_mut().enumerate() {
            for (j, e) in row.iter_mut().enumerate() {
                *e -= m[i] * c[j];
            }
        }
        out
    }

    /// Sum of two moment sets.
    pub fn add(&self, o: &Moments) -> Moments {
        let mut second = self.second;
        for (i, row) in second.iter_mut().enumerate() {
            for (j, e) in row.iter_mut().enumerate() {
                *e += o.second[i][j];
            }
        }
        Moments { volume: self.volume + o.volume, first: self.first + o.first, second }
    }
}

/// Integrated face moments about the ORIGIN, oriented by the face normal (left to right).
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FaceMoments {
    /// `∫ dA`.
    pub area: f64,
    /// `∫ x dA`.
    pub first: Vector3,
    /// `∫ x xᵀ dA`.
    pub second: [[f64; 3]; 3],
    /// `∫ n dA`, `n` the face normal (left to right); outward on a boundary face.
    pub vector_area: Vector3,
}

impl FaceMoments {
    /// The area centroid, if the area is non-zero.
    pub fn centroid(&self) -> Option<Point3> {
        (self.area > 0.0).then(|| Point3::new(self.first.x / self.area, self.first.y / self.area, self.first.z / self.area))
    }

    /// If the face is planar within `tol` (rms distance of the face from its mean plane), the
    /// unit normal (left to right) and the plane offset `d` (`n·x = d`).
    ///
    /// With `n = vector_area/|vector_area|` and `d = n·first/area`, the mean square deviation is
    /// `(nᵀ second n − d² area)/area`. This form cancels far from the origin (relative error
    /// about `1e-16 (|x|/h)²` for a face of size `h` at distance `|x|`); use
    /// [`TopologyView::face_plane`] or [`Self::plane_with`] for a stable test.
    pub fn plane(&self, tol: f64) -> Option<(Vector3, f64)> {
        let va = self.vector_area.norm();
        if !(self.area > 0.0 && va > 0.0) {
            return None;
        }
        let n = self.vector_area * (1.0 / va);
        let d = n.dot(self.first) / self.area;
        let sn = [n.x, n.y, n.z];
        let mut nsn = 0.0;
        for (i, row) in self.second.iter().enumerate() {
            for (j, e) in row.iter().enumerate() {
                nsn += sn[i] * e * sn[j];
            }
        }
        let ms = ((nsn - d * d * self.area) / self.area).max(0.0);
        (ms.sqrt() <= tol).then_some((n, d))
    }

    /// Like [`Self::plane`], but the mean square deviation is taken from the centred second
    /// moment `c` (same face, same orientation): `nᵀ C n / area`, which does not cancel far from
    /// the origin. `(n, d)` are computed exactly as in `plane`, so only the pass/fail gate differs.
    pub fn plane_with(&self, c: &FaceCentral, tol: f64) -> Option<(Vector3, f64)> {
        let va = self.vector_area.norm();
        if !(self.area > 0.0 && va > 0.0) {
            return None;
        }
        let n = self.vector_area * (1.0 / va);
        let d = n.dot(self.first) / self.area;
        let sn = [n.x, n.y, n.z];
        let mut nsn = 0.0;
        for (i, row) in c.central.iter().enumerate() {
            for (j, e) in row.iter().enumerate() {
                nsn += sn[i] * e * sn[j];
            }
        }
        let ms = (nsn / self.area).max(0.0);
        (ms.sqrt() <= tol).then_some((n, d))
    }

    /// The unit normal if the face is planar within `tol`; see [`Self::plane`].
    pub fn unit_normal(&self, tol: f64) -> Option<Vector3> {
        self.plane(tol).map(|p| p.0)
    }
}

/// Area, centroid and second moment about the centroid of a face: `∫ (x−c)(x−c)ᵀ dA`. It is
/// accumulated in a frame local to the face, so unlike [`FaceMoments::second`] it does not lose
/// digits far from the origin. Same orientation convention as [`FaceMoments`].
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FaceCentral {
    /// `∫ dA`.
    pub area: f64,
    /// The area centroid `c` (the local origin when `area <= 0`).
    pub centroid: Point3,
    /// `∫ (x−c)(x−c)ᵀ dA` (zero when `area <= 0`).
    pub central: [[f64; 3]; 3],
}

impl FaceCentral {
    /// Union of two disjoint patches (parallel-axis theorem), e.g. the members of a composite face.
    pub fn combine(&self, o: &Self) -> Self {
        let area = self.area + o.area;
        if area.is_nan() || area <= 0.0 {
            return if self.area >= o.area { *self } else { *o };
        }
        let (wa, wb) = (self.area / area, o.area / area);
        let c = Point3::new(self.centroid.x * wa + o.centroid.x * wb, self.centroid.y * wa + o.centroid.y * wb, self.centroid.z * wa + o.centroid.z * wb);
        let mut central = [[0.0; 3]; 3];
        for part in [self, o] {
            let d = [part.centroid.x - c.x, part.centroid.y - c.y, part.centroid.z - c.z];
            for i in 0..3 {
                for j in 0..3 {
                    central[i][j] += part.central[i][j] + part.area * d[i] * d[j];
                }
            }
        }
        Self { area, centroid: c, central }
    }
}

/// Where a parameter-space point lies relative to a face.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PointClass {
    /// Strictly inside the trimmed face.
    In,
    /// Within the model tolerance of its boundary.
    On,
    /// Outside.
    Out,
}

const GX: [f64; 5] = [0.0, 0.538_469_310_105_683_1, -0.538_469_310_105_683_1, 0.906_179_845_938_664, -0.906_179_845_938_664];
const GW: [f64; 5] = [0.568_888_888_888_888_9, 0.478_628_670_499_366_5, 0.478_628_670_499_366_5, 0.236_926_885_056_189_1, 0.236_926_885_056_189_1];
/// Gauss nodes sorted ascending (for ordered traversal).
fn gauss_sorted() -> [(f64, f64); 5] {
    let mut v: [(f64, f64); 5] = core::array::from_fn(|i| (GX[i], GW[i]));
    v.sort_by(|a, b| a.0.total_cmp(&b.0));
    v
}

const EDGE_PANELS: usize = 16;
const POLY_SAMPLES: usize = 24;

/// One boundary sample in parameter space.
#[derive(Clone, Copy, Debug)]
struct Sample {
    uv: Uv,
    /// Quadrature weight times `dv/dt` times the traversal direction (for `∮ H dv`).
    dv: f64,
}

struct FaceSamples<'a> {
    surf: &'a dyn ParametricSurface,
    dom: Domain2,
    loops: Vec<Vec<Sample>>,
}

fn solve_duv(j: &SurfaceJet, c: Vector3) -> Option<(f64, f64)> {
    let (a, b, d) = (j.du.dot(j.du), j.du.dot(j.dv), j.dv.dot(j.dv));
    let det = a * d - b * b;
    if det.abs() <= 1e-24 * (a * d).abs().max(1e-300) {
        return None;
    }
    let (r0, r1) = (j.du.dot(c), j.dv.dot(c));
    Some(((d * r0 - b * r1) / det, (a * r1 - b * r0) / det))
}

fn unwrap_uv(dom: &Domain2, uv: Uv, prev: Option<Uv>) -> Uv {
    match prev {
        Some(p) => Uv::new(dom.u.nearest_to(uv.u, p.u), dom.v.nearest_to(uv.v, p.v)),
        None => uv,
    }
}

/// Sample every loop of a face in traversal order. `gauss`: composite Gauss nodes with weights
/// (for integration); otherwise uniform nodes including both end points (for polygons).
fn sample_face<'a, V: TopologyView + ?Sized>(view: &'a V, f: EntityId, gauss: bool) -> Result<FaceSamples<'a>, QueryError> {
    if view.dim_of(f) != Some(Dim::Face) || !view.is_alive(f) {
        return Err(QueryError::NoSuchCell(f));
    }
    let Carrier::Surface(sc) = view.carrier(f) else { return Err(QueryError::Unsupported("face without a surface carrier")) };
    let shapes = view.shapes();
    let surf = shapes.surface(sc.surface).ok_or(QueryError::Incomplete("surface not in store"))?;
    let dom = surf.domain();
    let nodes = gauss_sorted();
    let mut loops = Vec::new();
    for lp in view.loops(f) {
        let mut samples: Vec<Sample> = Vec::new();
        let mut prev: Option<Uv> = None;
        // With pole coedges the loop's parameter path is anchored by the pcurves: the seam
        // positions (e.g. `u = 0` vs `2 pi`) cannot be recovered by projection alone.
        let anchored = lp.keys().any(|k| view.flags(k.edge).contains(CellFlags::DEGENERATE));
        for key in lp.keys() {
            let edge_flags: CellFlags = view.flags(key.edge);
            if edge_flags.contains(CellFlags::DEGENERATE) {
                // Pole/apex coedge: it contributes `dv = 0` to the boundary integral, but its
                // pcurve carries the parameter jump (e.g. `u` across a full turn), so move the
                // unwrapping reference to its far end.
                match view.pcurve(key) {
                    Some(pc) => {
                        let rg = pc.range();
                        let end = if key.side == Side::Left { pc.eval(rg.hi) } else { pc.eval(rg.lo) };
                        prev = Some(end);
                    }
                    None if dom.u.is_periodic() || dom.v.is_periodic() => return Err(QueryError::Incomplete("pole coedge without a pcurve")),
                    None => {}
                }
                continue;
            }
            let Carrier::Curve(cc) = view.carrier(key.edge) else { return Err(QueryError::Unsupported("edge without a curve carrier")) };
            let curve = shapes.curve(cc.curve).ok_or(QueryError::Incomplete("curve not in store"))?;
            let along = (cc.sense == crate::types::Sense::Forward) == (key.side == Side::Left);
            let (lo, hi) = (cc.range.lo, cc.range.hi);
            if anchored && let Some(pc) = view.pcurve(key) {
                let rg = pc.range();
                prev = Some(pc.eval(if along { rg.lo } else { rg.hi }));
            }
            let mut push = |t: f64, w: f64, prev: &mut Option<Uv>| -> Result<(), QueryError> {
                let cj = curve.jet(t);
                let proj = surf.project(cj.p, *prev)?;
                let uv = unwrap_uv(&dom, proj.uv, *prev);
                let j = surf.jet(uv);
                let dvdt = match solve_duv(&j, cj.d1) {
                    Some((_, dvdt)) => dvdt,
                    // A polygon end point may sit on a pole; its weight is zero anyway.
                    None if !gauss && !dom.singularities.is_empty() => 0.0,
                    None => return Err(QueryError::Unsupported("degenerate parametrization on boundary")),
                };
                let dir = if along { 1.0 } else { -1.0 };
                samples.push(Sample { uv, dv: dir * w * dvdt });
                *prev = Some(uv);
                Ok(())
            };
            if gauss {
                let h = (hi - lo) / EDGE_PANELS as f64;
                let order: Vec<usize> = if along { (0..EDGE_PANELS).collect() } else { (0..EDGE_PANELS).rev().collect() };
                for p in order {
                    let seq: Vec<(f64, f64)> = if along { nodes.to_vec() } else { nodes.iter().rev().copied().collect() };
                    for (x, w) in seq {
                        let t = lo + (p as f64 + 0.5 * (x + 1.0)) * h;
                        push(t, w * 0.5 * h, &mut prev)?;
                    }
                }
            } else {
                let idx: Vec<usize> = if along { (0..=POLY_SAMPLES).collect() } else { (0..=POLY_SAMPLES).rev().collect() };
                for k in idx {
                    let t = lo + (hi - lo) * (k as f64 / POLY_SAMPLES as f64);
                    push(t, 0.0, &mut prev)?;
                }
            }
        }
        if samples.is_empty() {
            return Err(QueryError::Incomplete("loop without curve-carrying edges"));
        }
        let (a, b) = (samples[0].uv, samples[samples.len() - 1].uv);
        let scale = 1.0 + a.u.abs() + a.v.abs();
        // Uniform sampling includes the end points, so the loop closes exactly there; Gauss nodes
        // stop short of the ends by at most a panel fraction.
        if gauss {
            let _ = scale;
        } else if (a.u - b.u).abs() + (a.v - b.v).abs() > 1e-6 * scale {
            return Err(QueryError::Unsupported("loop does not close in parameter space"));
        }
        loops.push(samples);
    }
    if loops.is_empty() {
        return Err(QueryError::Incomplete("face without loops"));
    }
    Ok(FaceSamples { surf, dom, loops })
}

/// `∮ H dv` over all loops for a vector-valued integrand `g(jet)` (the unsigned parametric
/// integrand: multiply area-type results by the carrier sense).
fn green<const K: usize>(fs: &FaceSamples<'_>, g: &dyn Fn(&SurfaceJet) -> [f64; K]) -> [f64; K] {
    let u_ref = fs.loops[0][0].uv.u;
    let nodes = gauss_sorted();
    let step = core::f64::consts::FRAC_PI_8;
    let mut total = [0.0; K];
    for lp in &fs.loops {
        for s in lp {
            if s.dv == 0.0 {
                continue;
            }
            let span = s.uv.u - u_ref;
            let panels = if fs.dom.u.is_periodic() { (span.abs() / step).ceil().max(1.0) as usize } else { 1 };
            let h = span / panels as f64;
            let mut hval = [0.0; K];
            for p in 0..panels {
                for (x, w) in nodes {
                    let u = u_ref + (p as f64 + 0.5 * (x + 1.0)) * h;
                    let gv = g(&fs.surf.jet(Uv::new(u, s.uv.v)));
                    for k in 0..K {
                        hval[k] += w * 0.5 * h * gv[k];
                    }
                }
            }
            for k in 0..K {
                total[k] += hval[k] * s.dv;
            }
        }
    }
    total
}

fn mat(v: [f64; 6]) -> [[f64; 3]; 3] {
    [[v[0], v[3], v[5]], [v[3], v[1], v[4]], [v[5], v[4], v[2]]]
}

/// Moments of one face. See the module docs for coverage.
pub fn face_moments<V: TopologyView + ?Sized>(view: &V, f: EntityId) -> Result<FaceMoments, QueryError> {
    let fs = sample_face(view, f, true)?;
    let sigma = match view.carrier(f) {
        Carrier::Surface(sc) => sc.sense.sign(),
        _ => 1.0,
    };
    let r = green::<13>(&fs, &|j| {
        let n = j.du.cross(j.dv);
        let (x, y, z) = (j.p.x, j.p.y, j.p.z);
        let da = n.norm();
        [da, x * da, y * da, z * da, x * x * da, y * y * da, z * z * da, x * y * da, y * z * da, z * x * da, n.x, n.y, n.z]
    });
    Ok(FaceMoments {
        area: sigma * r[0],
        first: Vector3::new(sigma * r[1], sigma * r[2], sigma * r[3]),
        second: mat([sigma * r[4], sigma * r[5], sigma * r[6], sigma * r[7], sigma * r[8], sigma * r[9]]),
        vector_area: Vector3::new(r[10], r[11], r[12]),
    })
}

/// Centred moments of one face (see [`FaceCentral`]), computed in a frame local to the face (its
/// first boundary sample). Same coverage as [`face_moments`].
pub fn face_central<V: TopologyView + ?Sized>(view: &V, f: EntityId) -> Result<FaceCentral, QueryError> {
    let fs = sample_face(view, f, true)?;
    let sigma = match view.carrier(f) {
        Carrier::Surface(sc) => sc.sense.sign(),
        _ => 1.0,
    };
    let first = fs.loops.iter().find_map(|l| l.first()).ok_or(QueryError::Incomplete("face without boundary samples"))?;
    let o = fs.surf.eval(first.uv);
    let r = green::<10>(&fs, &|j| {
        let da = j.du.cross(j.dv).norm();
        let (a, b, c) = (j.p.x - o.x, j.p.y - o.y, j.p.z - o.z);
        [da, a * da, b * da, c * da, a * a * da, b * b * da, c * c * da, a * b * da, b * c * da, c * a * da]
    });
    let area = sigma * r[0];
    if area.is_nan() || area <= 0.0 {
        return Ok(FaceCentral { area, centroid: o, central: [[0.0; 3]; 3] });
    }
    let fm = [sigma * r[1], sigma * r[2], sigma * r[3]];
    let mut central = mat([sigma * r[4], sigma * r[5], sigma * r[6], sigma * r[7], sigma * r[8], sigma * r[9]]);
    for i in 0..3 {
        for j in 0..3 {
            central[i][j] -= fm[i] * fm[j] / area;
        }
    }
    Ok(FaceCentral { area, centroid: Point3::new(o.x + fm[0] / area, o.y + fm[1] / area, o.z + fm[2] / area), central })
}

/// Planarity test with the stable centred form: `Ok(Some((n, d)))` if the face is planar within
/// `tol` (see [`FaceMoments::plane_with`]), `Ok(None)` if not.
pub fn face_plane<V: TopologyView + ?Sized>(view: &V, f: EntityId, tol: f64) -> Result<Option<(Vector3, f64)>, QueryError> {
    let fm = view.face_moments(f)?;
    if fm.area.is_nan() || fm.area <= 0.0 {
        return Ok(None);
    }
    let c = view.face_central(f)?;
    Ok(fm.plane_with(&c, tol))
}

/// Raw moments of a volume by the divergence theorem over its face uses. Baffles (`Both`) cancel
/// and are skipped.
pub fn volume_moments<V: TopologyView + ?Sized>(view: &V, v: EntityId) -> Result<Moments, QueryError> {
    if view.dim_of(v) != Some(Dim::Volume) || !view.is_alive(v) {
        return Err(QueryError::NoSuchCell(v));
    }
    let mut acc = [0.0f64; 10];
    for inc in view.boundary(v) {
        if inc.ring == crate::types::Incidence::EMBEDDED || inc.side == Side::Both {
            continue;
        }
        let fs = sample_face(view, inc.cell, true)?;
        let r = green::<10>(&fs, &|j| {
            let n = j.du.cross(j.dv);
            let (x, y, z) = (j.p.x, j.p.y, j.p.z);
            [
                (x * n.x + y * n.y + z * n.z) / 3.0,
                0.5 * x * x * n.x,
                0.5 * y * y * n.y,
                0.5 * z * z * n.z,
                x * x * x * n.x / 3.0,
                y * y * y * n.y / 3.0,
                z * z * z * n.z / 3.0,
                0.25 * (x * x * y * n.x + y * y * x * n.y),
                0.25 * (y * y * z * n.y + z * z * y * n.z),
                0.25 * (z * z * x * n.z + x * x * z * n.x),
            ]
        });
        // Outward normal: the face normal if the volume is on the left, its negative on the right.
        let out = if inc.side == Side::Left { 1.0 } else { -1.0 };
        for k in 0..10 {
            acc[k] += out * r[k];
        }
    }
    Ok(Moments { volume: acc[0], first: Vector3::new(acc[1], acc[2], acc[3]), second: mat([acc[4], acc[5], acc[6], acc[7], acc[8], acc[9]]) })
}


/// Classify a parameter-space point against a face (winding number of the sampled boundary;
/// `On` within the model's linear tolerance of the boundary curves).
pub fn classify_point_on_face<V: TopologyView + ?Sized>(view: &V, f: EntityId, uv: Uv) -> Result<PointClass, QueryError> {
    let fs = sample_face(view, f, false)?;
    let tol = view.tolerance().linear.max(1e-12);
    let p3 = fs.surf.eval(uv);
    // On the boundary: exact distance to the edge curves (clamped to the edge ranges).
    for lp in view.loops(f) {
        for key in lp.keys() {
            if view.flags(key.edge).contains(CellFlags::DEGENERATE) {
                continue;
            }
            let Carrier::Curve(cc) = view.carrier(key.edge) else { continue };
            let Some(curve) = view.shapes().curve(cc.curve) else { continue };
            let Ok(pr) = curve.project(p3, None) else { continue };
            let dom = curve.domain();
            let mut best = f64::INFINITY;
            let mut cands = vec![cc.range.lo, cc.range.hi];
            for k in [-1.0, 0.0, 1.0] {
                let t = pr.t + k * dom.period.unwrap_or(0.0);
                if cc.range.contains(t, 0.0) {
                    cands.push(t);
                }
            }
            for t in cands {
                best = best.min((curve.eval(t) - p3).norm());
            }
            if best <= tol {
                return Ok(PointClass::On);
            }
        }
    }
    // Try the periodic images of the query point.
    let shifts = |dom: &sm_geom_api::Domain1| -> Vec<f64> {
        match dom.period {
            Some(p) => vec![-p, 0.0, p],
            None => vec![0.0],
        }
    };
    for du in shifts(&fs.dom.u) {
        for dv in shifts(&fs.dom.v) {
            let q = Uv::new(uv.u + du, uv.v + dv);
            let mut winding = 0i64;
            for lp in &fs.loops {
                for k in 0..lp.len() {
                    let (a, b) = (lp[k].uv, lp[(k + 1) % lp.len()].uv);
                    if a.v <= q.v {
                        if b.v > q.v && (b.u - a.u) * (q.v - a.v) - (q.u - a.u) * (b.v - a.v) > 0.0 {
                            winding += 1;
                        }
                    } else if b.v <= q.v && (b.u - a.u) * (q.v - a.v) - (q.u - a.u) * (b.v - a.v) < 0.0 {
                        winding -= 1;
                    }
                }
            }
            if winding != 0 {
                return Ok(PointClass::In);
            }
        }
    }
    Ok(PointClass::Out)
}
