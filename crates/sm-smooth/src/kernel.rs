//! Per-node kernels: cell evaluation, the guard, projection and the update rules.

use smallvec::SmallVec;
use sm_core::Point3;
use sm_geom_api::Uv;
use sm_mesh_api::FaceNormals;
use sm_quality::{Context, Metric, eval};
use sm_topo_api::PointClass;

use crate::normals::CarrierNormals;
use crate::prep::{Geo, Mode, Prep, V};
use crate::types::{Method, Objective};

/// `beta` values at or above this are treated as infinitely bad.
const BETA_MAX: f64 = 1e15;

/// Outcome of one node update.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Prop {
    /// No change (candidate equals the position, or no descent step exists).
    Stay,
    /// Accepted: new position and parameter hint.
    Move { p: V, h: [f64; 2] },
    /// The guard (or the smart-Laplacian shape test) refused it.
    RejGuard,
    /// The projection failed or left the curve range or the trimmed face.
    RejGeom,
}

type Pts = SmallVec<[V; 8]>;

/// Evaluator over a [`Prep`]: all methods are pure functions of the positions they are given.
pub(crate) struct Kernel<'p, 'a> {
    pub prep: &'p Prep<'a>,
    pub normals: CarrierNormals<'a>,
    pub floor: f64,
}

fn mean(p: &[V]) -> V {
    let k = 1.0 / p.len().max(1) as f64;
    p.iter().fold([0.0; 3], |s, q| [s[0] + q[0] * k, s[1] + q[1] * k, s[2] + q[2] * k])
}

fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn norm(a: V) -> f64 {
    dot(a, a).sqrt()
}

impl Kernel<'_, '_> {
    /// Corner points of cell `c`, with vertex `v` replaced by `x` when given.
    fn gather(&self, c: usize, pos: &[V], sub: Option<(u32, V)>, out: &mut Pts) {
        out.clear();
        for &w in self.prep.cell_corners(c) {
            out.push(match sub {
                Some((v, x)) if v == w => x,
                _ => pos[w as usize],
            });
        }
    }

    fn ctx(&self, c: usize, pts: &[V]) -> Context {
        let mut cx = Context::default();
        if self.prep.dim2 {
            let m = mean(pts);
            cx.normal = self
                .normals
                .normal(self.prep.cell_ent[c], Point3::new(m[0], m[1], m[2]))
                .map(|n| [n.x, n.y, n.z])
                .filter(|n| n.iter().all(|x| x.is_finite()) && n.iter().any(|x| *x != 0.0));
        }
        cx
    }

    fn metric(&self, m: Metric, c: usize, pts: &[V]) -> f64 {
        let cx = self.ctx(c, pts);
        eval(m, self.prep.shape[c], pts, &cx).unwrap_or(f64::NAN)
    }

    /// `(ScaledJacobian, Volume)` of cell `c` at `pts`.
    fn sj_vol(&self, c: usize, pts: &[V]) -> (f64, f64) {
        let cx = self.ctx(c, pts);
        let s = self.prep.shape[c];
        (
            eval(Metric::ScaledJacobian, s, pts, &cx).unwrap_or(f64::NAN),
            eval(Metric::Volume, s, pts, &cx).unwrap_or(f64::NAN),
        )
    }

    /// The guard predicate for one cell.
    fn cell_ok(&self, old: (f64, f64), new: (f64, f64)) -> bool {
        let lim = if old.0.is_nan() { self.floor } else { old.0.min(self.floor) };
        new.0.is_finite() && new.0 >= lim && (old.1 <= 0.0 || old.1.is_nan() || new.1 > 0.0)
    }

    /// Guard: every incident cell keeps `SJ' >= min(SJ, floor)` and a positive volume stays positive.
    fn guard(&self, v: usize, x: V, pos: &[V], old: &[(f64, f64)]) -> bool {
        let mut buf = Pts::new();
        for (k, &c) in self.prep.cells_of(v).iter().enumerate() {
            self.gather(c as usize, pos, Some((v as u32, x)), &mut buf);
            if !self.cell_ok(old[k], self.sj_vol(c as usize, &buf)) {
                return false;
            }
        }
        true
    }

    fn old_metrics(&self, v: usize, pos: &[V]) -> SmallVec<[(f64, f64); 16]> {
        let mut buf = Pts::new();
        self.prep
            .cells_of(v)
            .iter()
            .map(|&c| {
                self.gather(c as usize, pos, None, &mut buf);
                self.sj_vol(c as usize, &buf)
            })
            .collect()
    }

    /// Guard check of a whole cell against the pre-sweep positions (Jacobi repair).
    pub fn cell_violates(&self, c: usize, before: &[V], after: &[V]) -> bool {
        let mut a = Pts::new();
        let mut b = Pts::new();
        self.gather(c, before, None, &mut a);
        self.gather(c, after, None, &mut b);
        !self.cell_ok(self.sj_vol(c, &a), self.sj_vol(c, &b))
    }

    /// Minimum `Shape` over the cells of `v` with `v` at `x`.
    fn min_shape(&self, v: usize, x: V, pos: &[V]) -> f64 {
        let mut buf = Pts::new();
        let mut m = f64::INFINITY;
        for &c in self.prep.cells_of(v) {
            self.gather(c as usize, pos, Some((v as u32, x)), &mut buf);
            let s = self.metric(Metric::Shape, c as usize, &buf);
            if s.is_nan() {
                return f64::NAN;
            }
            m = m.min(s);
        }
        m
    }

    /// `sum_c beta_c^p` over the cells of `v` with `v` at `x`; infinite if a cell is degenerate
    /// or its metric is undefined.
    fn objective(&self, v: usize, x: V, pos: &[V], obj: Objective, p: f64) -> f64 {
        let mut buf = Pts::new();
        let mut f = 0.0;
        for &c in self.prep.cells_of(v) {
            self.gather(c as usize, pos, Some((v as u32, x)), &mut buf);
            let beta = match obj {
                Objective::MeanRatio => {
                    let s = self.metric(Metric::Shape, c as usize, &buf);
                    if s > 1e-14 { 1.0 / s } else { f64::INFINITY }
                }
                Objective::Condition => self.metric(Metric::Condition, c as usize, &buf),
            };
            if !(0.0..BETA_MAX).contains(&beta) {
                return f64::INFINITY;
            }
            f += beta.powf(p);
        }
        f
    }

    /// The discrete Winslow update of node `v` (`None` if the metric terms vanish).
    fn winslow_target(&self, v: usize, pos: &[V]) -> Option<V> {
        let [e, n, w, s, ne, nw, sw, se] = self.prep.wnb[v].map(|i| pos[i as usize]);
        let xi = [(e[0] - w[0]) * 0.5, (e[1] - w[1]) * 0.5, (e[2] - w[2]) * 0.5];
        let eta = [(n[0] - s[0]) * 0.5, (n[1] - s[1]) * 0.5, (n[2] - s[2]) * 0.5];
        let (alpha, gamma, beta) = (dot(eta, eta), dot(xi, xi), dot(xi, eta));
        let den = 2.0 * (alpha + gamma);
        if den.is_nan() || den <= 0.0 {
            return None;
        }
        let mut x = [0.0; 3];
        for (k, xk) in x.iter_mut().enumerate() {
            let cross = ne[k] - nw[k] - se[k] + sw[k];
            *xk = (alpha * (e[k] + w[k]) + gamma * (n[k] + s[k]) - 0.5 * beta * cross) / den;
        }
        Some(x)
    }

    /// Project `raw` onto the geometry of node `v`. `Err` = geometry rejection.
    fn project(&self, v: usize, raw: V, hint: [f64; 2]) -> Result<(V, [f64; 2]), ()> {
        match self.prep.mode[v] {
            Mode::Free => Ok((raw, [f64::NAN; 2])),
            Mode::Pinned => Err(()),
            Mode::Curve => {
                let Geo::Curve { curve, range, domain } = &self.prep.geos[self.prep.geo_ix[v] as usize] else {
                    return Err(());
                };
                let h = hint[0].is_finite().then_some(hint[0]);
                let r = curve.project(Point3::new(raw[0], raw[1], raw[2]), h).map_err(|_| ())?;
                let mut t = r.t;
                if let Some(per) = domain.period {
                    t += ((range.lo - t) / per).ceil() * per;
                }
                if !(t >= range.lo && t <= range.hi) {
                    return Err(());
                }
                Ok(([r.point.x, r.point.y, r.point.z], [t, 0.0]))
            }
            Mode::Surf => {
                let Geo::Surf { surf, face } = &self.prep.geos[self.prep.geo_ix[v] as usize] else {
                    return Err(());
                };
                let h = (hint[0].is_finite() && hint[1].is_finite()).then(|| Uv::new(hint[0], hint[1]));
                let r = surf.project(Point3::new(raw[0], raw[1], raw[2]), h).map_err(|_| ())?;
                match self.prep.topo.classify_point_on_face(*face, r.uv) {
                    Ok(PointClass::Out) | Err(_) => Err(()),
                    Ok(_) => Ok(([r.point.x, r.point.y, r.point.z], [r.uv.u, r.uv.v])),
                }
            }
        }
    }

    /// Unit tangent-space projector data for node `v` at `x`: `None` for a free node, else
    /// `Tangent(t)` or `Normal(n)`.
    fn frame(&self, v: usize, x: V, hint: [f64; 2]) -> Option<Option<(bool, V)>> {
        match self.prep.mode[v] {
            Mode::Free => Some(None),
            Mode::Pinned => None,
            Mode::Curve => {
                let Geo::Curve { curve, .. } = &self.prep.geos[self.prep.geo_ix[v] as usize] else { return None };
                let t = if hint[0].is_finite() {
                    hint[0]
                } else {
                    curve.project(Point3::new(x[0], x[1], x[2]), None).ok()?.t
                };
                let d = curve.tangent(t).ok()?;
                let d = [d.x, d.y, d.z];
                let l = norm(d);
                (l > 0.0 && l.is_finite()).then(|| Some((true, [d[0] / l, d[1] / l, d[2] / l])))
            }
            Mode::Surf => {
                let Geo::Surf { surf, .. } = &self.prep.geos[self.prep.geo_ix[v] as usize] else { return None };
                let uv = if hint[0].is_finite() && hint[1].is_finite() {
                    Uv::new(hint[0], hint[1])
                } else {
                    surf.project(Point3::new(x[0], x[1], x[2]), None).ok()?.uv
                };
                let n = surf.normal(uv).ok()?;
                let n = [n.x, n.y, n.z];
                let l = norm(n);
                (l > 0.0 && l.is_finite()).then(|| Some((false, [n[0] / l, n[1] / l, n[2] / l])))
            }
        }
    }

    /// Mean of the in-entity neighbours.
    fn neighbour_mean(&self, v: usize, pos: &[V]) -> V {
        let nb = self.prep.nbrs(v);
        let mut s = [0.0; 3];
        for &w in nb {
            let q = pos[w as usize];
            s = [s[0] + q[0], s[1] + q[1], s[2] + q[2]];
        }
        let k = 1.0 / nb.len() as f64;
        [s[0] * k, s[1] * k, s[2] * k]
    }

    /// Area/volume-weighted centroid of the stencil of `v` (`None` if the weights vanish).
    fn centroid_area(&self, v: usize, pos: &[V]) -> Option<V> {
        let (mut wsum, mut acc) = (0.0, [0.0; 3]);
        let mut add = |w: f64, c: V| {
            if w > 0.0 && w.is_finite() {
                wsum += w;
                acc = [acc[0] + w * c[0], acc[1] + w * c[1], acc[2] + w * c[2]];
            }
        };
        match self.prep.mode[v] {
            Mode::Curve => {
                let x = pos[v];
                for &w in self.prep.nbrs(v) {
                    let q = pos[w as usize];
                    add(norm(sub(q, x)), [(q[0] + x[0]) * 0.5, (q[1] + x[1]) * 0.5, (q[2] + x[2]) * 0.5]);
                }
            }
            Mode::Surf if !self.prep.dim2 => {
                let co = self.prep.mesh.columns();
                for &f in self.prep.faces_of(v) {
                    let lp = &co.face_vertices[co.face_vertex_offsets[f as usize] as usize
                        ..co.face_vertex_offsets[f as usize + 1] as usize];
                    let pts: SmallVec<[V; 8]> = lp.iter().map(|w| pos[w.get()]).collect();
                    let c = mean(&pts);
                    let mut nrm = [0.0; 3];
                    for i in 0..pts.len() {
                        let (a, b) = (sub(pts[i], c), sub(pts[(i + 1) % pts.len()], c));
                        nrm = [
                            nrm[0] + a[1] * b[2] - a[2] * b[1],
                            nrm[1] + a[2] * b[0] - a[0] * b[2],
                            nrm[2] + a[0] * b[1] - a[1] * b[0],
                        ];
                    }
                    add(0.5 * norm(nrm), c);
                }
            }
            _ => {
                let mut buf = Pts::new();
                for &c in self.prep.cells_of(v) {
                    self.gather(c as usize, pos, None, &mut buf);
                    let a = self.metric(Metric::Volume, c as usize, &buf);
                    add(a, mean(&buf));
                }
            }
        }
        (wsum > 0.0).then(|| [acc[0] / wsum, acc[1] / wsum, acc[2] / wsum])
    }

    /// One node update from the positions `pos` and hints `hint`.
    pub fn propose(&self, v: usize, pos: &[V], hint: &[[f64; 2]], method: &Method) -> Prop {
        let x = pos[v];
        match *method {
            Method::Laplacian { relax }
            | Method::SmartLaplacian { relax }
            | Method::CentroidArea { relax }
            | Method::Winslow { relax } => {
                let target = match method {
                    Method::Winslow { .. } => match self.winslow_target(v, pos) {
                        Some(t) => t,
                        None => return Prop::Stay,
                    },
                    Method::CentroidArea { .. } => match self.centroid_area(v, pos) {
                        Some(t) => t,
                        None => return Prop::Stay,
                    },
                    _ => self.neighbour_mean(v, pos),
                };
                let raw = [x[0] + relax * (target[0] - x[0]), x[1] + relax * (target[1] - x[1]), x[2] + relax * (target[2] - x[2])];
                let Ok((p, h)) = self.project(v, raw, hint[v]) else { return Prop::RejGeom };
                if p == x {
                    return Prop::Stay;
                }
                let old = self.old_metrics(v, pos);
                if !self.guard(v, p, pos, &old) {
                    return Prop::RejGuard;
                }
                if matches!(method, Method::SmartLaplacian { .. }) {
                    let (a, b) = (self.min_shape(v, x, pos), self.min_shape(v, p, pos));
                    if a.is_nan() || b.is_nan() || b < a {
                        return Prop::RejGuard;
                    }
                }
                Prop::Move { p, h }
            }
            Method::Optimize { objective, p: pw } => self.optimize(v, pos, hint, objective, pw),
            #[allow(unreachable_patterns)]
            _ => Prop::Stay,
        }
    }

    fn optimize(&self, v: usize, pos: &[V], hint: &[[f64; 2]], obj: Objective, pw: f64) -> Prop {
        let l = self.prep.ell[v];
        let x = pos[v];
        if l.is_nan() || l <= 0.0 {
            return Prop::Stay;
        }
        let f0 = self.objective(v, x, pos, obj, pw);
        if !f0.is_finite() {
            return Prop::Stay;
        }
        let d = 1e-6 * l;
        let mut g = [0.0; 3];
        for (i, gi) in g.iter_mut().enumerate() {
            let (mut a, mut b) = (x, x);
            a[i] += d;
            b[i] -= d;
            let (fa, fb) = (self.objective(v, a, pos, obj, pw), self.objective(v, b, pos, obj, pw));
            if !(fa.is_finite() && fb.is_finite()) {
                return Prop::Stay;
            }
            *gi = (fa - fb) / (2.0 * d);
        }
        let Some(frame) = self.frame(v, x, hint[v]) else { return Prop::Stay };
        let gt = match frame {
            None => g,
            Some((true, t)) => {
                let s = dot(g, t);
                [t[0] * s, t[1] * s, t[2] * s]
            }
            Some((false, n)) => {
                let s = dot(g, n);
                [g[0] - n[0] * s, g[1] - n[1] * s, g[2] - n[2] * s]
            }
        };
        let gn = norm(gt);
        if (gn * l).is_nan() || gn * l <= 1e-12 * f0 {
            return Prop::Stay;
        }
        let dir = [-gt[0] / gn, -gt[1] / gn, -gt[2] / gn];
        let old = self.old_metrics(v, pos);
        let mut alpha = 0.25 * l;
        let mut first: Option<Prop> = None;
        for _ in 0..=10 {
            let raw = [x[0] + alpha * dir[0], x[1] + alpha * dir[1], x[2] + alpha * dir[2]];
            let fail = match self.project(v, raw, hint[v]) {
                Err(()) => Some(Prop::RejGeom),
                Ok((p, h)) => {
                    if !self.guard(v, p, pos, &old) {
                        Some(Prop::RejGuard)
                    } else if self.objective(v, p, pos, obj, pw) <= f0 - 1e-4 * alpha * gn {
                        return if p == x { Prop::Stay } else { Prop::Move { p, h } };
                    } else {
                        Some(Prop::Stay)
                    }
                }
            };
            first = first.or(fail);
            alpha *= 0.5;
        }
        first.unwrap_or(Prop::Stay)
    }
}
