//! [`PreparedSurface`]: a NURBS surface compiled for fast evaluation, projection and editing.

use super::basis::{ders, ders_n, find_span, values_n};
use super::curve::{bbox_of, hp_from, hp_point};
use super::project;
use super::spline1::{Hp, Layout, multiplicity};
use sm_core::{Point3, Vector3};
use sm_geom_api::{Domain1, Domain2, GeomError, Interval, NurbsSurface, ParamAxis, ParametricSurface, Singularity, SurfaceJet, SurfaceProjection, Uv};
use smallvec::SmallVec;

/// Inline pole storage for one Bezier patch (up to bicubic stays on the stack).
pub(crate) type PatchPoles = SmallVec<[Hp; 16]>;

/// A Bezier patch of the decomposition: parameter box, `(pu+1)*(pv+1)` homogeneous poles
/// (`u`-major) and the box of its Cartesian control net.
#[derive(Clone, Debug)]
pub(crate) struct Patch {
    pub u: Interval,
    pub v: Interval,
    pub poles: PatchPoles,
    pub bbox: [f64; 6],
}

/// A NURBS surface prepared for the hot path.
#[derive(Clone, Debug)]
pub struct PreparedSurface {
    lu: Layout,
    lv: Layout,
    nu: usize,
    nv: usize,
    /// `nu * nv` homogeneous poles, `u`-major.
    poles: Vec<Hp>,
    rational: bool,
    du: Interval,
    dv: Interval,
    domain: Domain2,
    pub(crate) patches: Vec<Patch>,
    bbox: [f64; 6],
}

fn fold(r: Interval, periodic: bool, t: f64) -> f64 {
    if periodic {
        let w = r.lo + (t - r.lo).rem_euclid(r.len());
        if w >= r.hi { r.lo } else { w }
    } else {
        r.clamp(t)
    }
}

fn near(a: Point3, b: Point3) -> bool {
    (a - b).norm() <= 1e-14 * (1.0 + a.x.abs().max(a.y.abs()).max(a.z.abs()))
}

impl PreparedSurface {
    /// Prepare a validated surface.
    pub fn new(s: &NurbsSurface) -> Result<Self, GeomError> {
        s.validate()?;
        let rational = s.weights.is_some();
        let poles: Vec<Hp> = s.control.iter().enumerate().map(|(i, p)| hp_from(*p, s.weights.as_ref().map_or(1.0, |w| w[i]))).collect();
        Self::from_parts(
            Layout { degree: s.degree_u, knots: s.knots_u.clone(), periodic: s.periodic[0] },
            Layout { degree: s.degree_v, knots: s.knots_v.clone(), periodic: s.periodic[1] },
            s.nu,
            s.nv,
            poles,
            rational,
        )
    }

    pub(crate) fn from_parts(lu: Layout, lv: Layout, nu: usize, nv: usize, poles: Vec<Hp>, rational: bool) -> Result<Self, GeomError> {
        let ((ulo, uhi), (vlo, vhi)) = (lu.range(), lv.range());
        let (du, dv) = (Interval::new(ulo, uhi), Interval::new(vlo, vhi));
        let dom1 = |r: Interval, per: bool| if per { Domain1::periodic(r.lo, r.len()) } else { Domain1::open(r.lo, r.hi) };
        let mut domain = Domain2::regular(dom1(du, lu.periodic), dom1(dv, lv.periodic));
        let pt = |iu: usize, iv: usize| hp_point(poles[iu * nv + iv]);
        let mut sing: SmallVec<[Singularity; 2]> = SmallVec::new();
        for (iu, at) in [(0, du.lo), (nu - 1, du.hi)] {
            let first = pt(iu, 0);
            if (0..nv).all(|iv| near(pt(iu, iv), first)) {
                sing.push(Singularity::new(ParamAxis::U, at, first));
            }
        }
        for (iv, at) in [(0, dv.lo), (nv - 1, dv.hi)] {
            let first = pt(0, iv);
            if (0..nu).all(|iu| near(pt(iu, iv), first)) {
                sing.push(Singularity::new(ParamAxis::V, at, first));
            }
        }
        domain.singularities = sing;
        let (patches, bbox) = Self::decompose(&lu, &lv, nu, nv, &poles)?;
        Ok(Self { lu, lv, nu, nv, poles, rational, du, dv, domain, patches, bbox })
    }

    fn decompose(lu: &Layout, lv: &Layout, nu: usize, nv: usize, poles: &[Hp]) -> Result<(Vec<Patch>, [f64; 6]), GeomError> {
        let (bu, net1) = apply_u(lu, poles, nu, nv, |l, c| l.bezierized(c))?;
        let nu1 = bu.npoles();
        let (bv, net2) = apply_v(lv, &net1, nu1, nv, |l, c| l.bezierized(c))?;
        let nv2 = bv.npoles();
        let (pu, pv) = (lu.degree, lv.degree);
        let (spu, spv) = (bu.spans(), bv.spans());
        let mut patches = Vec::with_capacity(spu.len() * spv.len());
        let mut total = [f64::INFINITY, f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
        for &(ua, ub, i0) in &spu {
            for &(va, vb, j0) in &spv {
                let mut pp = PatchPoles::new();
                for a in 0..=pu {
                    for b in 0..=pv {
                        pp.push(net2[(i0 + a) * nv2 + j0 + b]);
                    }
                }
                let bbox = bbox_of(pp.iter().map(|h| hp_point(*h)));
                for k in 0..3 {
                    total[k] = total[k].min(bbox[k]);
                    total[k + 3] = total[k + 3].max(bbox[k + 3]);
                }
                patches.push(Patch { u: Interval::new(ua, ub), v: Interval::new(va, vb), poles: pp, bbox });
            }
        }
        Ok((patches, total))
    }

    /// The contract data (for storing an edited surface).
    pub fn to_def(&self) -> Result<NurbsSurface, GeomError> {
        let control: Vec<Point3> = self.poles.iter().map(|h| hp_point(*h)).collect();
        let weights = self.rational.then(|| self.poles.iter().map(|h| h[3]).collect());
        NurbsSurface::new_periodic(self.lu.degree, self.lv.degree, self.lu.knots.clone(), self.lv.knots.clone(), self.nu, self.nv, control, weights, [self.lu.periodic, self.lv.periodic])
    }

    /// Degrees `(u, v)`.
    pub fn degrees(&self) -> (usize, usize) {
        (self.lu.degree, self.lv.degree)
    }

    /// Control net size `(nu, nv)`.
    pub fn net_size(&self) -> (usize, usize) {
        (self.nu, self.nv)
    }

    /// Axis-aligned box `[xmin, ymin, zmin, xmax, ymax, zmax]` of the control net.
    pub fn control_bbox(&self) -> [f64; 6] {
        self.bbox
    }

    /// Parameter ranges `(u, v)`.
    pub fn ranges(&self) -> (Interval, Interval) {
        (self.du, self.dv)
    }

    pub(crate) fn fold(&self, uv: Uv) -> Uv {
        Uv::new(fold(self.du, self.lu.periodic, uv.u), fold(self.dv, self.lv.periodic, uv.v))
    }

    #[inline]
    fn jet_at(&self, uv: Uv) -> SurfaceJet {
        let (pu, pv) = (self.lu.degree, self.lv.degree);
        let class = |p: usize| if p < 4 { 0 } else if p < 8 { 1 } else { 2 };
        let a = match (class(pu), class(pv)) {
            (0, 0) => self.homogeneous_n::<4, 4>(uv),
            (0, 1) => self.homogeneous_n::<4, 8>(uv),
            (0, _) => self.homogeneous_n::<4, 16>(uv),
            (1, 0) => self.homogeneous_n::<8, 4>(uv),
            (1, 1) => self.homogeneous_n::<8, 8>(uv),
            (1, _) => self.homogeneous_n::<8, 16>(uv),
            (_, 0) => self.homogeneous_n::<16, 4>(uv),
            (_, 1) => self.homogeneous_n::<16, 8>(uv),
            (_, _) => self.homogeneous_n::<16, 16>(uv),
        };
        let v3 = |k: usize, l: usize| Vector3::new(a[k][l][0], a[k][l][1], a[k][l][2]);
        let w = |k: usize, l: usize| a[k][l][3];
        let iw = 1.0 / w(0, 0);
        let s = v3(0, 0) * iw;
        let s_u = (v3(1, 0) - s * w(1, 0)) * iw;
        let s_v = (v3(0, 1) - s * w(0, 1)) * iw;
        let s_uu = (v3(2, 0) - s_u * (2.0 * w(1, 0)) - s * w(2, 0)) * iw;
        let s_vv = (v3(0, 2) - s_v * (2.0 * w(0, 1)) - s * w(0, 2)) * iw;
        let s_uv = (v3(1, 1) - s_u * w(0, 1) - s_v * w(1, 0) - s * w(1, 1)) * iw;
        SurfaceJet { p: Point3::new(s.x, s.y, s.z), du: s_u, dv: s_v, duu: s_uu, duv: s_uv, dvv: s_vv }
    }

    #[inline]
    fn homogeneous_n<const NU: usize, const NV: usize>(&self, uv: Uv) -> [[[f64; 4]; 3]; 3] {
        let (pu, pv) = (self.lu.degree, self.lv.degree);
        let su = find_span(self.nu, pu, uv.u, &self.lu.knots);
        let sv = find_span(self.nv, pv, uv.v, &self.lv.knots);
        let nd_u = ders_n::<NU>(su, uv.u, pu, &self.lu.knots);
        let nd_v = ders_n::<NV>(sv, uv.v, pv, &self.lv.knots);
        let mut a = [[[0.0f64; 4]; 3]; 3];
        for i in 0..=pu {
            let row = (su - pu + i) * self.nv + sv - pv;
            for j in 0..=pv {
                let h = self.poles[row + j];
                for k in 0..3 {
                    let nk = nd_u[k][i];
                    for l in 0..3 {
                        let c = nk * nd_v[l][j];
                        let al = &mut a[k][l];
                        al[0] += c * h[0];
                        al[1] += c * h[1];
                        al[2] += c * h[2];
                        al[3] += c * h[3];
                    }
                }
            }
        }
        a
    }

    #[inline]
    fn point_n<const NU: usize, const NV: usize>(&self, uv: Uv) -> Point3 {
        let (pu, pv) = (self.lu.degree, self.lv.degree);
        let su = find_span(self.nu, pu, uv.u, &self.lu.knots);
        let sv = find_span(self.nv, pv, uv.v, &self.lv.knots);
        let nu_ = values_n::<NU>(su, uv.u, pu, &self.lu.knots);
        let nv_ = values_n::<NV>(sv, uv.v, pv, &self.lv.knots);
        let mut a = [0.0f64; 4];
        for i in 0..=pu {
            let row = (su - pu + i) * self.nv + sv - pv;
            for j in 0..=pv {
                let h = self.poles[row + j];
                let c = nu_[i] * nv_[j];
                a[0] += c * h[0];
                a[1] += c * h[1];
                a[2] += c * h[2];
                a[3] += c * h[3];
            }
        }
        let iw = 1.0 / a[3];
        Point3::new(a[0] * iw, a[1] * iw, a[2] * iw)
    }

    fn rebuild(&self, lu: Layout, lv: Layout, nu: usize, nv: usize, poles: Vec<Hp>) -> Result<Self, GeomError> {
        Self::from_parts(lu, lv, nu, nv, poles, self.rational)
    }

    /// Insert the `u` knot `t` `times` times (shape preserved). `t` must lie in the open parameter
    /// interval: the ends (the seam of a periodic direction) are refused with
    /// [`GeomError::OutOfDomain`] because a knot there is already present.
    pub fn insert_knot_u(&self, t: f64, times: usize) -> Result<PreparedSurface, GeomError> {
        if !(t > self.du.lo && t < self.du.hi) {
            return Err(GeomError::OutOfDomain(t));
        }
        if multiplicity(&self.lu.knots, t) + times > self.lu.degree {
            return Err(GeomError::Invalid("knot multiplicity would exceed the degree"));
        }
        let mut lu = self.lu.clone();
        let mut poles = self.poles.clone();
        let mut nu = self.nu;
        for _ in 0..times {
            let (l, q) = apply_u(&lu, &poles, nu, self.nv, |l, c| l.insert_once(c, t))?;
            lu = l;
            poles = q;
            nu = lu.npoles();
        }
        self.rebuild(lu, self.lv.clone(), nu, self.nv, poles)
    }

    /// Insert the `v` knot `t` `times` times (shape preserved); same domain rule as
    /// [`Self::insert_knot_u`].
    pub fn insert_knot_v(&self, t: f64, times: usize) -> Result<PreparedSurface, GeomError> {
        if !(t > self.dv.lo && t < self.dv.hi) {
            return Err(GeomError::OutOfDomain(t));
        }
        if multiplicity(&self.lv.knots, t) + times > self.lv.degree {
            return Err(GeomError::Invalid("knot multiplicity would exceed the degree"));
        }
        let mut lv = self.lv.clone();
        let mut poles = self.poles.clone();
        let mut nv = self.nv;
        for _ in 0..times {
            let (l, q) = apply_v(&lv, &poles, self.nu, nv, |l, c| l.insert_once(c, t))?;
            lv = l;
            poles = q;
            nv = lv.npoles();
        }
        self.rebuild(self.lu.clone(), lv, self.nu, nv, poles)
    }

    /// The same surface with both directions clamped and non-periodic (periodic directions are
    /// cut at their seam).
    pub fn clamped(&self) -> Result<PreparedSurface, GeomError> {
        let (lu, p1) = apply_u(&self.lu, &self.poles, self.nu, self.nv, |l, c| l.clamped(c))?;
        let nu = lu.npoles();
        let (lv, p2) = apply_v(&self.lv, &p1, nu, self.nv, |l, c| l.clamped(c))?;
        let nv = lv.npoles();
        self.rebuild(lu, lv, nu, nv, p2)
    }

    /// The iso-parametric curve with `u = value` (`fixed_u`) or `v = value`, as an exact NURBS
    /// curve in the other parameter (same knots, degree and periodicity as that direction).
    pub fn iso_curve(&self, fixed_u: bool, value: f64) -> Result<sm_geom_api::NurbsCurve, GeomError> {
        let value = if fixed_u { fold(self.du, self.lu.periodic, value) } else { fold(self.dv, self.lv.periodic, value) };
        let (lay_fixed, lay_free, n_fixed, n_free) = if fixed_u { (&self.lu, &self.lv, self.nu, self.nv) } else { (&self.lv, &self.lu, self.nv, self.nu) };
        let p = lay_fixed.degree;
        let span = find_span(n_fixed, p, value, &lay_fixed.knots);
        let basis = ders(span, value, p, &lay_fixed.knots);
        let mut q = vec![[0.0f64; 4]; n_free];
        for (j, qj) in q.iter_mut().enumerate() {
            for i in 0..=p {
                let idx = if fixed_u { (span - p + i) * self.nv + j } else { j * self.nv + (span - p + i) };
                let h = self.poles[idx];
                for c in 0..4 {
                    qj[c] += basis[0][i] * h[c];
                }
            }
        }
        let control: Vec<Point3> = q.iter().map(|h| hp_point(*h)).collect();
        let weights = self.rational.then(|| q.iter().map(|h| h[3]).collect());
        if lay_free.periodic {
            sm_geom_api::NurbsCurve::new_periodic(lay_free.degree, lay_free.knots.clone(), control, weights)
        } else {
            sm_geom_api::NurbsCurve::new(lay_free.degree, lay_free.knots.clone(), control, weights)
        }
    }

    /// Number of Bezier patches of the decomposition.
    pub fn patch_count(&self) -> usize {
        self.patches.len()
    }

    /// Closest points of many query points, in parallel; result `i` belongs to `points[i]`.
    pub fn project_many(&self, points: &[Point3]) -> Vec<SurfaceProjection> {
        use rayon::prelude::*;
        points.par_iter().map(|p| project::project_surface(self, *p, None)).collect()
    }

    /// Jets at many parameters, in parallel.
    pub fn jet_many(&self, uvs: &[Uv], out: &mut [SurfaceJet]) {
        use rayon::prelude::*;
        assert_eq!(uvs.len(), out.len(), "jet_many: length mismatch");
        out.par_chunks_mut(128).zip(uvs.par_chunks(128)).for_each(|(o, q)| {
            for (o, q) in o.iter_mut().zip(q) {
                *o = self.jet(*q);
            }
        });
    }
}

/// Apply a 1D operation along `u` to every column and reassemble the net.
pub(crate) fn apply_u(l: &Layout, poles: &[Hp], nu: usize, nv: usize, op: impl Fn(&Layout, &[Hp]) -> Result<(Layout, Vec<Hp>), GeomError>) -> Result<(Layout, Vec<Hp>), GeomError> {
    let mut cols: Vec<Vec<Hp>> = Vec::with_capacity(nv);
    let mut new_l = None;
    let mut col = vec![[0.0; 4]; nu];
    for iv in 0..nv {
        for iu in 0..nu {
            col[iu] = poles[iu * nv + iv];
        }
        let (nl, nc) = op(l, &col)?;
        new_l.get_or_insert(nl);
        cols.push(nc);
    }
    let nl = new_l.ok_or(GeomError::Invalid("empty net"))?;
    let nu2 = cols[0].len();
    let mut out = vec![[0.0; 4]; nu2 * nv];
    for (iv, c) in cols.iter().enumerate() {
        for (iu, h) in c.iter().enumerate() {
            out[iu * nv + iv] = *h;
        }
    }
    Ok((nl, out))
}

/// Apply a 1D operation along `v` to every row and reassemble the net.
pub(crate) fn apply_v(l: &Layout, poles: &[Hp], nu: usize, nv: usize, op: impl Fn(&Layout, &[Hp]) -> Result<(Layout, Vec<Hp>), GeomError>) -> Result<(Layout, Vec<Hp>), GeomError> {
    let mut rows: Vec<Vec<Hp>> = Vec::with_capacity(nu);
    let mut new_l = None;
    for iu in 0..nu {
        let (nl, nr) = op(l, &poles[iu * nv..(iu + 1) * nv])?;
        new_l.get_or_insert(nl);
        rows.push(nr);
    }
    let nl = new_l.ok_or(GeomError::Invalid("empty net"))?;
    Ok((nl, rows.concat()))
}

impl ParametricSurface for PreparedSurface {
    fn domain(&self) -> Domain2 {
        self.domain.clone()
    }

    fn jet(&self, uv: Uv) -> SurfaceJet {
        self.jet_at(self.fold(uv))
    }

    /// Position only: skips the derivative bases.
    fn eval(&self, uv: Uv) -> Point3 {
        let uv = self.fold(uv);
        let (pu, pv) = (self.lu.degree, self.lv.degree);
        let class = |p: usize| if p < 4 { 0 } else if p < 8 { 1 } else { 2 };
        match (class(pu), class(pv)) {
            (0, 0) => self.point_n::<4, 4>(uv),
            (0, 1) => self.point_n::<4, 8>(uv),
            (0, _) => self.point_n::<4, 16>(uv),
            (1, 0) => self.point_n::<8, 4>(uv),
            (1, 1) => self.point_n::<8, 8>(uv),
            (1, _) => self.point_n::<8, 16>(uv),
            (_, 0) => self.point_n::<16, 4>(uv),
            (_, 1) => self.point_n::<16, 8>(uv),
            (_, _) => self.point_n::<16, 16>(uv),
        }
    }

    /// Parallel batch evaluation; element-independent writes, so the output is bit-identical to
    /// the default loop for any thread count.
    fn eval_many(&self, uvs: &[Uv], out: &mut [Point3]) {
        use rayon::prelude::*;
        assert_eq!(uvs.len(), out.len(), "eval_many: length mismatch");
        out.par_chunks_mut(128).zip(uvs.par_chunks(128)).for_each(|(o, q)| {
            for (o, q) in o.iter_mut().zip(q) {
                *o = self.eval(*q);
            }
        });
    }

    fn project(&self, p: Point3, hint: Option<Uv>) -> Result<SurfaceProjection, GeomError> {
        Ok(project::project_surface(self, p, hint))
    }

    /// Intersect the ray with the surface: pairwise Bezier box subdivision of the ray segment
    /// and the patches, then damped Newton. Hits ascend in `t`.
    fn ray_hits(&self, ray: &sm_geom_api::Ray, range: Interval, out: &mut Vec<sm_geom_api::RayHit>) -> Result<(), GeomError> {
        crate::intersect::ray_nurbs_surface(self, ray, range, out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wavy() -> NurbsSurface {
        let mut control = Vec::new();
        for i in 0..5 {
            for j in 0..4 {
                control.push(Point3::new(i as f64, j as f64 * 1.3, ((i * 3 + j * 5) % 7) as f64 * 0.3));
            }
        }
        let weights: Vec<f64> = (0..20).map(|i| 1.0 + 0.1 * (i % 5) as f64).collect();
        NurbsSurface::new(3, 2, vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0], vec![0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0], 5, 4, control, Some(weights)).unwrap()
    }

    #[test]
    fn matches_reference_and_edits_preserve_shape() {
        let s = wavy();
        let ps = PreparedSurface::new(&s).unwrap();
        let ref_s = &s;
        let ins = ps.insert_knot_u(0.3, 2).unwrap().insert_knot_v(1.4, 1).unwrap();
        for i in 0..=12 {
            for j in 0..=12 {
                let uv = Uv::new(i as f64 / 12.0, 2.0 * j as f64 / 12.0);
                let (a, b) = (ref_s.jet(uv), ps.jet(uv));
                assert!((a.p - b.p).norm() < 1e-13 && (a.du - b.du).norm() < 1e-12 && (a.dv - b.dv).norm() < 1e-12);
                assert!((a.duu - b.duu).norm() < 1e-10 && (a.duv - b.duv).norm() < 1e-10 && (a.dvv - b.dvv).norm() < 1e-10);
                assert!((ins.eval(uv) - a.p).norm() < 1e-12, "{uv:?}");
            }
        }
        assert!(ins.to_def().unwrap().validate().is_ok());
        assert_eq!(ps.patch_count(), 2 * 2);
    }

    /// A surface periodic in `u` (cubic uniform, 6 unique rows) and clamped quadratic in `v`.
    fn periodic_surface() -> NurbsSurface {
        let (m, p) = (6usize, 3usize);
        let ring = |i: usize| {
            let a = core::f64::consts::TAU * (i % m) as f64 / m as f64;
            (1.0 + 0.2 * (3.0 * a).sin(), a)
        };
        let mut control = Vec::new();
        for i in 0..m + p {
            let (r, a) = ring(i);
            for j in 0..4 {
                control.push(Point3::new(r * a.cos(), r * a.sin(), j as f64 * 0.7 + 0.1 * (i % 3) as f64));
            }
        }
        let weights: Vec<f64> = (0..(m + p) * 4).map(|k| 1.0 + 0.1 * (((k / 4) % m + k % 4) % 3) as f64).collect();
        let ku: Vec<f64> = (0..m + 2 * p + 1).map(|i| i as f64 - p as f64).collect();
        NurbsSurface::new_periodic(p, 2, ku, vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0], m + p, 4, control, Some(weights), [true, false]).unwrap()
    }

    #[test]
    fn periodic_direction_matches_reference_and_survives_editing() {
        let s = periodic_surface();
        let ps = PreparedSurface::new(&s).unwrap();
        assert!(ps.domain().u.is_periodic() && !ps.domain().v.is_periodic());
        let edited = ps.insert_knot_u(1.4, 1).unwrap().insert_knot_u(4.1, 2).unwrap().insert_knot_v(0.3, 1).unwrap();
        let clamped = ps.clamped().unwrap();
        assert!(edited.to_def().unwrap().validate().is_ok());
        assert!(!clamped.domain().u.is_periodic());
        for i in 0..=30 {
            for j in 0..=8 {
                // Parameters outside one period wrap.
                let uv = Uv::new(-2.5 + 11.0 * i as f64 / 30.0, j as f64 / 8.0);
                let (a, b) = (s.jet(uv), ps.jet(uv));
                assert!((a.p - b.p).norm() < 1e-12 && (a.du - b.du).norm() < 1e-10 && (a.duv - b.duv).norm() < 1e-9, "{uv:?}");
                assert!((edited.eval(uv) - a.p).norm() < 1e-11, "edited {uv:?}");
                let inside = Uv::new(ps.ranges().0.lo + (uv.u - ps.ranges().0.lo).rem_euclid(ps.ranges().0.len()), uv.v);
                assert!((clamped.eval(inside) - a.p).norm() < 1e-11, "clamped {inside:?}");
            }
        }
        // Projection across the seam agrees with a dense scan.
        let q = Point3::new(0.9, -0.2, 0.8);
        let pr = ps.project(q, None).unwrap();
        let mut best = f64::INFINITY;
        for i in 0..400 {
            for j in 0..=40 {
                best = best.min((ps.eval(Uv::new(6.0 * i as f64 / 400.0, j as f64 / 40.0)) - q).norm());
            }
        }
        assert!(pr.distance <= best + 1e-9, "{} vs {}", pr.distance, best);
    }
}
