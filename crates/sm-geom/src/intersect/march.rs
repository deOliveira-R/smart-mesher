//! General surface-surface intersection by marching.
//!
//! One surface `M` must be parametric; the other `T` is either its implicit form (two unknowns
//! `(u, v)` of `M`) or another parametric surface (four unknowns). Seeds are the hits of `M`'s
//! iso-curves with `T`; each seed is traced in both directions by a predictor along
//! `n_M x n_T` and a Newton corrector with a step-length constraint, with step control from the
//! turning angle (chord tolerance), closure detection, and boundary detection.

use super::curve_surface::curve_surface;
use super::ssi::{CurveGeometry, IntersectionCurve, MarchEnd, MarchedCurve, SsiOptions, SsiResult};
use super::CurveSurface;
use crate::nurbs::PreparedSurface;
use crate::roots::solve_dense;
use sm_core::{Point3, Vector3};
use sm_geom_api::{Circle, CurveDef, Domain1, Domain2, Ellipse, EllipticCone, EllipticCylinder, Frame, GeomError, ImplicitDef, ImplicitSurface, Interval, Line, ParametricSurface, SurfaceDef, Uv};
use std::collections::BTreeMap;

// ------------------------------------------------------------------ surface handles

/// A surface prepared for marching.
struct Surf<'a> {
    def: &'a SurfaceDef,
    par: Option<&'a dyn ParametricSurface>,
    imp: Option<ImplicitDef>,
    dom: Option<Domain2>,
}

fn make_surf<'a>(def: &'a SurfaceDef, prepared: &'a Option<PreparedSurface>) -> Surf<'a> {
    let par: Option<&dyn ParametricSurface> = match prepared {
        Some(p) => Some(p),
        None => def.as_surface(),
    };
    Surf { def, par, imp: def.implicit_def(), dom: par.map(|p| p.domain()) }
}

fn regular(s: &Surf) -> bool {
    s.dom.as_ref().is_some_and(|d| d.singularities.is_empty())
}

fn finite_extent(s: &Surf) -> bool {
    s.dom.as_ref().is_some_and(|d| (d.u.is_periodic() || d.u.range.is_bounded()) && (d.v.is_periodic() || d.v.range.is_bounded()))
}

fn unit(v: Vector3) -> Option<Vector3> {
    let n = v.norm();
    (n > 0.0 && n.is_finite()).then(|| v * (1.0 / n))
}

fn in_box(w: &Option<[f64; 6]>, p: Point3, pad: f64) -> bool {
    match w {
        None => true,
        Some(b) => p.x >= b[0] - pad && p.x <= b[3] + pad && p.y >= b[1] - pad && p.y <= b[4] + pad && p.z >= b[2] - pad && p.z <= b[5] + pad,
    }
}

// ------------------------------------------------------------------ iso-curves

/// The iso-curve `axis = value` of an analytic surface as a curve whose parameter equals
/// `other_param / scale`... returned as `(curve, scale)` with `other = t * scale`.
fn iso_curve(def: &SurfaceDef, fixed_u: bool, c: f64) -> Option<(CurveDef, f64)> {
    let e = |f: &Frame, a: f64| f.x * a.cos() + f.y * a.sin();
    let circle = |center: Point3, x: Vector3, y: Vector3, r: f64| -> Option<CurveDef> {
        (r > 1e-12).then(|| Circle::new(Frame { origin: center, x, y, z: x.cross(y) }, r).ok().map(CurveDef::Circle)).flatten()
    };
    match def {
        SurfaceDef::Plane(p) => {
            let f = &p.frame;
            let l = if fixed_u { Line::new(f.origin + f.x * c, f.y) } else { Line::new(f.origin + f.y * c, f.x) };
            l.ok().map(|l| (CurveDef::Line(l), 1.0))
        }
        SurfaceDef::Cylinder(s) => {
            let f = &s.frame;
            if fixed_u {
                Line::new(f.origin + e(f, c) * s.radius, f.z).ok().map(|l| (CurveDef::Line(l), 1.0))
            } else {
                circle(f.origin + f.z * c, f.x, f.y, s.radius).map(|d| (d, 1.0))
            }
        }
        SurfaceDef::Cone(s) => {
            let f = &s.frame;
            let (sn, cs) = s.half_angle.sin_cos();
            if fixed_u {
                let dir = e(f, c) * sn + f.z * cs;
                Line::new(f.origin + e(f, c) * s.radius, dir).ok().map(|l| (CurveDef::Line(l), 1.0))
            } else {
                circle(f.origin + f.z * (c * cs), f.x, f.y, s.radius + c * sn).map(|d| (d, 1.0))
            }
        }
        SurfaceDef::Sphere(s) => {
            let f = &s.frame;
            if fixed_u {
                // Meridian great circle; its parameter is the latitude.
                circle(f.origin, e(f, c), f.z, s.radius).map(|d| (d, 1.0))
            } else {
                circle(f.origin + f.z * (s.radius * c.sin()), f.x, f.y, s.radius * c.cos()).map(|d| (d, 1.0))
            }
        }
        SurfaceDef::Torus(s) => {
            let f = &s.frame;
            if fixed_u {
                let r = e(f, c);
                circle(f.origin + r * s.major, r, f.z, s.minor).map(|d| (d, 1.0))
            } else {
                circle(f.origin + f.z * (s.minor * c.sin()), f.x, f.y, s.major + s.minor * c.cos()).map(|d| (d, 1.0))
            }
        }
        SurfaceDef::EllipticCylinder(EllipticCylinder { frame: f, a, b }) => {
            if fixed_u {
                Line::new(f.origin + f.x * (a * c.cos()) + f.y * (b * c.sin()), f.z).ok().map(|l| (CurveDef::Line(l), 1.0))
            } else {
                Ellipse::new(Frame { origin: f.origin + f.z * c, ..*f }, *a, *b).ok().map(|e| (CurveDef::Ellipse(e), 1.0))
            }
        }
        SurfaceDef::EllipticCone(EllipticCone { frame: f, a, b, half_angle }) => {
            let (sn, cs) = half_angle.sin_cos();
            if fixed_u {
                let ring = f.x * (a * c.cos()) + f.y * (b * c.sin());
                let dir = ring * (sn / a) + f.z * cs;
                let n = dir.norm();
                Line::new(f.origin + ring, dir).ok().map(|l| (CurveDef::Line(l), 1.0 / n))
            } else {
                let k = 1.0 + c * sn / a;
                (k > 1e-12).then(|| Ellipse::new(Frame { origin: f.origin + f.z * (c * cs), ..*f }, a * k, b * k).ok().map(|e| (CurveDef::Ellipse(e), 1.0))).flatten()
            }
        }
        _ => None,
    }
}

// ------------------------------------------------------------------ parameter windows

fn uv_extent(s: &Surf, window: &Option<[f64; 6]>) -> Result<[Interval; 2], GeomError> {
    let d = s.dom.as_ref().ok_or(GeomError::Unsupported("surface has no parametrization"))?;
    let per = [d.u.is_periodic(), d.v.is_periodic()];
    let mut out = [d.u.range, d.v.range];
    let need = [!(per[0] || d.u.range.is_bounded()), !(per[1] || d.v.range.is_bounded())];
    if need[0] || need[1] {
        let w = window.ok_or(GeomError::Invalid("marching a surface with unbounded parameters needs a window"))?;
        let par = s.par.ok_or(GeomError::Unsupported("surface has no parametrization"))?;
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for i in 0..8 {
            let c = Point3::new(if i & 1 == 0 { w[0] } else { w[3] }, if i & 2 == 0 { w[1] } else { w[4] }, if i & 4 == 0 { w[2] } else { w[5] });
            if let Ok(pr) = par.project(c, None) {
                lo[0] = lo[0].min(pr.uv.u);
                hi[0] = hi[0].max(pr.uv.u);
                lo[1] = lo[1].min(pr.uv.v);
                hi[1] = hi[1].max(pr.uv.v);
            }
        }
        for k in 0..2 {
            if need[k] {
                if !(lo[k].is_finite() && hi[k] > lo[k]) {
                    return Err(GeomError::Invalid("window does not determine the parameter range"));
                }
                let m = 0.1 * (hi[k] - lo[k]) + 1e-9;
                out[k] = Interval::new(lo[k] - m, hi[k] + m);
                // Keep to the surface's own (half-)bounded domain, e.g. the cone nappe.
                let dom = if k == 0 { d.u.range } else { d.v.range };
                out[k] = Interval::new(out[k].lo.max(dom.lo), out[k].hi.min(dom.hi));
            }
        }
    }
    Ok(out)
}

// ------------------------------------------------------------------ marching engine

struct Engine<'a> {
    t: &'a Surf<'a>,
    /// `M` as a parametric surface (checked when the engine is built).
    mp: &'a dyn ParametricSurface,
    /// `T` as a parametric surface and its domain (present when marching in four unknowns).
    tp: Option<(&'a dyn ParametricSurface, Domain2)>,
    /// Marching in four unknowns (T is a NURBS surface) instead of two.
    four: bool,
    opts: &'a SsiOptions,
    mdom: Domain2,
    mbox: [Interval; 2],
    tbox: [Interval; 2],
    chord: f64,
    max_step: f64,
    min_step: f64,
}

#[derive(Clone, Copy)]
struct State {
    mu: Uv,
    tu: Uv,
    p: Point3,
}

enum Step {
    /// New state, its unit tangent (oriented along the travel direction) and `sin` of the angle
    /// between the surface normals there.
    Ok(State, Vector3, f64),
    Boundary(State),
    Singular,
    Fail,
}

impl<'a> Engine<'a> {
    fn mpar(&self) -> &dyn ParametricSurface {
        self.mp
    }

    /// Jet of `M` with a first-order extension beyond a bounded parameter range, so Newton can
    /// step across a boundary (a NURBS surface itself clamps) and the crossing is detected after.
    fn mj(&self, uv: Uv) -> sm_geom_api::SurfaceJet {
        extended_jet(self.mpar(), &self.mdom, uv)
    }

    fn tj(&self, uv: Uv) -> sm_geom_api::SurfaceJet {
        match &self.tp {
            Some((p, d)) => extended_jet(*p, d, uv),
            None => self.mj(uv),
        }
    }

    fn tangent_at(&self, st: &State) -> Option<(Vector3, f64)> {
        let nm = self.mpar().normal(st.mu).ok()?;
        let nt = if self.four {
            self.t.par?.normal(st.tu).ok()?
        } else {
            unit(self.t.imp.as_ref()?.gradient(st.p))?
        };
        let c = nm.cross(nt);
        let s = c.norm();
        // Surfaces tangent to within ~100 `tol.angular` (1e-7 rad by default) are singular.
        if s < (100.0 * self.opts.tol.angular).max(1e-12) { None } else { Some((c * (1.0 / s), s)) }
    }

    fn outside_m(&self, q: Uv) -> bool {
        let lim = |d: &Domain1, iv: Interval, x: f64| !d.is_periodic() && (x < iv.lo - 1e-12 || x > iv.hi + 1e-12);
        lim(&self.mdom.u, self.mbox[0], q.u) || lim(&self.mdom.v, self.mbox[1], q.v)
    }

    fn outside_t(&self, q: Uv) -> bool {
        if !self.four {
            return false;
        }
        let Some((_, d)) = &self.tp else { return false };
        let lim = |d: &Domain1, iv: Interval, x: f64| !d.is_periodic() && (x < iv.lo - 1e-12 || x > iv.hi + 1e-12);
        lim(&d.u, self.tbox[0], q.u) || lim(&d.v, self.tbox[1], q.v)
    }

    /// Newton corrector landing on both surfaces with `tau . (p - p0) = h`.
    fn correct(&self, start: &State, p0: Point3, tau: Vector3, h: f64, guess: State) -> Option<State> {
        let tol = self.opts.tol.linear;
        let mut st = guess;
        if !self.four {
            let imp = self.t.imp.as_ref()?;
            for _ in 0..12 {
                let j = self.mj(st.mu);
                let p = j.p;
                let f = imp.value(p);
                let g = imp.gradient(p);
                let gn = g.norm().max(1e-300);
                let f2 = tau.dot(p - p0) - h;
                if f.abs() / gn <= 1e-3 * tol && f2.abs() <= 1e-3 * tol {
                    return Some(State { mu: st.mu, tu: st.tu, p });
                }
                let a = [[g.dot(j.du), g.dot(j.dv)], [tau.dot(j.du), tau.dot(j.dv)]];
                let d = solve_dense(a, [-f, -f2])?;
                st.mu = Uv::new(st.mu.u + d[0], st.mu.v + d[1]);
                if !(st.mu.u.is_finite() && st.mu.v.is_finite()) {
                    return None;
                }
            }
            let p = self.mj(st.mu).p;
            let f = imp.value(p);
            let ok = f.abs() / imp.gradient(p).norm().max(1e-300) <= tol && (tau.dot(p - p0) - h).abs() <= 1e-3 * h.max(tol);
            let _ = start;
            ok.then_some(State { mu: st.mu, tu: st.tu, p })
        } else {
            for _ in 0..14 {
                let jm = self.mj(st.mu);
                let jt = self.tj(st.tu);
                let r = jm.p - jt.p;
                let f4 = tau.dot(jm.p - p0) - h;
                if r.norm() <= 1e-3 * tol && f4.abs() <= 1e-3 * tol {
                    return Some(State { mu: st.mu, tu: st.tu, p: jm.p });
                }
                let a = [
                    [jm.du.x, jm.dv.x, -jt.du.x, -jt.dv.x],
                    [jm.du.y, jm.dv.y, -jt.du.y, -jt.dv.y],
                    [jm.du.z, jm.dv.z, -jt.du.z, -jt.dv.z],
                    [tau.dot(jm.du), tau.dot(jm.dv), 0.0, 0.0],
                ];
                let d = solve_dense(a, [-r.x, -r.y, -r.z, -f4])?;
                st.mu = Uv::new(st.mu.u + d[0], st.mu.v + d[1]);
                st.tu = Uv::new(st.tu.u + d[2], st.tu.v + d[3]);
                if !(d.iter().all(|x| x.is_finite())) {
                    return None;
                }
            }
            let (pm, pt) = (self.mj(st.mu).p, self.tj(st.tu).p);
            ((pm - pt).norm() <= tol && (tau.dot(pm - p0) - h).abs() <= 1e-3 * h.max(tol)).then_some(State { mu: st.mu, tu: st.tu, p: pm })
        }
    }

    /// Predictor (least-squares in-plane displacement) and Newton corrector for a move of
    /// length `h` along `tau`, without any branch or boundary checks. `Err(true)` = a
    /// parametrization is singular here, `Err(false)` = the corrector failed.
    fn advance(&self, st: &State, tau: Vector3, h: f64) -> Result<State, bool> {
        let j = self.mj(st.mu);
        let (e, f, g) = (j.du.dot(j.du), j.du.dot(j.dv), j.dv.dot(j.dv));
        let det = e * g - f * f;
        if det <= 1e-24 * e.max(g).max(1e-300).powi(2) {
            return Err(true);
        }
        let (bu, bv) = (j.du.dot(tau) * h, j.dv.dot(tau) * h);
        let du = (g * bu - f * bv) / det;
        let dv = (e * bv - f * bu) / det;
        let mut guess = *st;
        guess.mu = Uv::new(st.mu.u + du, st.mu.v + dv);
        if self.four && self.t.par.is_some() {
            let jt = self.tj(st.tu);
            let (e2, f2, g2) = (jt.du.dot(jt.du), jt.du.dot(jt.dv), jt.dv.dot(jt.dv));
            let det2 = e2 * g2 - f2 * f2;
            if det2 <= 1e-24 * e2.max(g2).max(1e-300).powi(2) {
                return Err(true);
            }
            let (cu, cv) = (jt.du.dot(tau) * h, jt.dv.dot(tau) * h);
            guess.tu = Uv::new(st.tu.u + (g2 * cu - f2 * cv) / det2, st.tu.v + (e2 * cv - f2 * cu) / det2);
        }
        self.correct(st, st.p, tau, h, guess).ok_or(false)
    }

    /// One predictor-corrector step of length about `h` along `tau`.
    ///
    /// Branch jumps near tangency are rejected (`Fail`, the caller halves `h`): the corrector
    /// may not move the point farther from the straight prediction than the chord tolerance
    /// allows for the curvature the step control accepts, and the corrected midpoint of the
    /// step must lie within the chord tolerance of the chord.
    fn step(&self, st: &State, tau: Vector3, h: f64, chord: f64) -> Step {
        let new = match self.advance(st, tau, h) {
            Ok(n) => n,
            Err(true) => return Step::Singular,
            Err(false) => return Step::Fail,
        };
        // Reject a corrector that jumped to a different branch.
        if (new.p - st.p).norm() > 3.0 * h + 10.0 * self.opts.tol.linear {
            return Step::Fail;
        }
        let checked = h > self.min_step * 1.01;
        if self.outside_m(new.mu) || self.outside_t(new.tu) || !in_box(&self.opts.window, new.p, self.opts.tol.linear) {
            if checked && (new.p - (st.p + tau * h)).norm() > 3.0 * chord + 10.0 * self.opts.tol.linear {
                return Step::Fail;
            }
            return Step::Boundary(new);
        }
        let tan_new = self.tangent_at(&new).map(|(t2, sn)| (if t2.dot(tau) < 0.0 { t2 * -1.0 } else { t2 }, sn));
        if checked {
            // The tolerance for this step is the tightest local one over the step (start, middle,
            // end): a pair of branches closing in anywhere along it must not be bridged.
            let mut c = chord;
            if let Some((t2, _)) = tan_new {
                c = c.min(self.local_chord(&new, t2));
            }
            if (new.p - (st.p + tau * h)).norm() > 3.0 * c + 10.0 * self.opts.tol.linear {
                return Step::Fail;
            }
            match self.advance(st, tau, 0.5 * h) {
                Ok(mid) => {
                    c = c.min(self.local_chord(&mid, tau));
                    let chord_mid = Point3::new(0.5 * (st.p.x + new.p.x), 0.5 * (st.p.y + new.p.y), 0.5 * (st.p.z + new.p.z));
                    if (mid.p - chord_mid).norm() > c || (new.p - (st.p + tau * h)).norm() > 3.0 * c + 10.0 * self.opts.tol.linear {
                        return Step::Fail;
                    }
                }
                Err(_) => return Step::Fail,
            }
        }
        match tan_new {
            Some((t2, sn)) => Step::Ok(new, t2, sn),
            None => Step::Singular,
        }
    }

    /// Bisect the step to land on the domain/window boundary; returns the boundary point.
    fn to_boundary(&self, st: &State, tau: Vector3, mut hi: f64) -> Option<State> {
        let mut lo = 0.0;
        let mut best = None;
        for _ in 0..40 {
            let h = 0.5 * (lo + hi);
            if h <= self.opts.tol.linear * 0.01 {
                break;
            }
            match self.step(st, tau, h, self.chord) {
                Step::Ok(s, _, _) => {
                    lo = h;
                    best = Some(s);
                }
                Step::Boundary(s) => {
                    hi = h;
                    let _ = s;
                }
                _ => return best,
            }
            if hi - lo <= self.opts.tol.linear * 0.1 {
                break;
            }
        }
        best
    }

    /// Isolated tangent contacts of `M` with an implicit `T`: local extrema of `T`'s implicit
    /// function over `M` that touch zero (no sign change, so no iso-curve seed finds them).
    /// Grid minima of the distance estimate are polished by Newton on the stationarity of
    /// `phi(u, v) = f(M(u, v))`. Used only when the marcher found no curve at all.
    fn tangent_points(&self) -> (Vec<Point3>, Vec<State>) {
        let Some(imp) = self.t.imp.as_ref() else { return (Vec::new(), Vec::new()) };
        if self.four {
            return (Vec::new(), Vec::new());
        }
        const N: usize = 40;
        let tol = self.opts.tol.linear;
        let (bu, bv) = (self.mbox[0], self.mbox[1]);
        let at = |i: usize, j: usize| Uv::new(bu.lo + bu.len() * i as f64 / N as f64, bv.lo + bv.len() * j as f64 / N as f64);
        let sdist_at = |q: Uv| {
            let p = self.mj(q).p;
            imp.value(p) / imp.gradient(p).norm().max(1e-300)
        };
        let sgrid: Vec<f64> = (0..(N + 1) * (N + 1)).map(|k| sdist_at(at(k / (N + 1), k % (N + 1)))).collect();
        // Local minima of a grid function (ties go to the lower index so a plateau yields one).
        let minima = |grid: &dyn Fn(usize, usize) -> f64| -> Vec<(f64, Uv)> {
            let mut cands: Vec<(f64, Uv)> = Vec::new();
            for i in 0..=N {
                for j in 0..=N {
                    let v = grid(i, j);
                    let mut is_min = true;
                    for (di, dj) in [(-1i64, -1i64), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)] {
                        let (ni, nj) = (i as i64 + di, j as i64 + dj);
                        if ni < 0 || nj < 0 || ni > N as i64 || nj > N as i64 {
                            continue;
                        }
                        let w = grid(ni as usize, nj as usize);
                        if w < v || (w == v && (ni, nj) < (i as i64, j as i64)) {
                            is_min = false;
                            break;
                        }
                    }
                    if is_min {
                        cands.push((v, at(i, j)));
                    }
                }
            }
            cands.sort_by(|a, b| a.0.total_cmp(&b.0));
            cands.truncate(32);
            cands
        };
        let sg = |i: usize, j: usize| sgrid[i * (N + 1) + j];
        // Minima of the distance (a touching contact) and of the signed distance (a loop
        // that no seed line crossed).
        let mut cands = minima(&|i, j| sg(i, j).abs());
        cands.extend(minima(&sg));
        let (hu, hv) = (1e-6 * bu.len().max(1e-12), 1e-6 * bv.len().max(1e-12));
        // phi and its gradient (a, b) = (g . du, g . dv) at q.
        let eval = |q: Uv| {
            let j = self.mj(q);
            let gr = imp.gradient(j.p);
            (imp.value(j.p), gr.dot(j.du), gr.dot(j.dv), j.p)
        };
        let mut out: Vec<Point3> = Vec::new();
        let mut seeds: Vec<State> = Vec::new();
        for (_, q0) in cands {
            let mut q = q0;
            let (mut f, mut a, mut b, mut p) = eval(q);
            let cell = Uv::new(bu.len() / N as f64, bv.len() / N as f64);
            for _ in 0..40 {
                if a == 0.0 && b == 0.0 {
                    break;
                }
                let (_, a1, b1, _) = eval(Uv::new(q.u + hu, q.v));
                let (_, a2, b2, _) = eval(Uv::new(q.u, q.v + hv));
                let h = [[(a1 - a) / hu, (a2 - a) / hv], [(b1 - b) / hu, (b2 - b) / hv]];
                let Some(d) = solve_dense(h, [-a, -b]) else { break };
                let (du, dv) = (d[0].clamp(-cell.u, cell.u), d[1].clamp(-cell.v, cell.v));
                let nq = Uv::new(q.u + du, q.v + dv);
                let (nf, na, nb, np) = eval(nq);
                // Accept steps that reduce the stationarity residual.
                if na * na + nb * nb < a * a + b * b {
                    q = nq;
                    (f, a, b, p) = (nf, na, nb, np);
                } else {
                    break;
                }
                if du.abs() <= 1e-14 * bu.len() && dv.abs() <= 1e-14 * bv.len() {
                    break;
                }
            }
            let gn = imp.gradient(p).norm().max(1e-300);
            if self.outside_m(q) {
                continue;
            }
            if f.abs() / gn <= tol {
                if in_box(&self.opts.window, p, tol) && out.iter().all(|o| (*o - p).norm() > 20.0 * tol) {
                    out.push(p);
                }
            } else if f / gn < -tol && a.hypot(b) <= 1e-6 * gn * (self.mj(q).du.norm() + self.mj(q).dv.norm()) {
                // A penetration deeper than the tolerance: a small loop between the seed lines.
                // Walk out of the negative region along a coordinate direction to find a point
                // on its boundary, and seed the tracer there.
                let seed = [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)].iter().find_map(|&(du, dv)| {
                    let at_s = |s: f64| Uv::new(q.u + du * s * cell.u, q.v + dv * s * cell.v);
                    let (mut lo, mut hi) = (0.0, 0.0);
                    let mut s = 0.125;
                    while s <= 2.0 * N as f64 {
                        if sdist_at(at_s(s)) >= 0.0 {
                            hi = s;
                            break;
                        }
                        lo = s;
                        s *= 2.0;
                    }
                    if hi == 0.0 || self.outside_m(at_s(hi)) {
                        return None;
                    }
                    for _ in 0..80 {
                        let m = 0.5 * (lo + hi);
                        if sdist_at(at_s(m)) >= 0.0 { hi = m } else { lo = m }
                    }
                    let mu = at_s(0.5 * (lo + hi));
                    let sp = self.mj(mu).p;
                    in_box(&self.opts.window, sp, tol).then_some(State { mu, tu: Uv::default(), p: sp })
                });
                if let Some(sd) = seed {
                    seeds.push(sd);
                }
            }
        }
        (out, seeds)
    }

    /// Chord tolerance for steps from `st` along `tau`: `chord_tol`, reduced to a twelfth of the
    /// distance to the neighbouring branch of the intersection when that is closer than about
    /// `12 chord_tol`. The separation is read off the signed distance of `M` to `T` along the
    /// in-surface direction across the curve (`F(s) ~ F' s + F'' s^2 / 2`, branches at `s = 0`
    /// and `s = -2 F' / F''`). Only for an implicit `T`; with a NURBS `T` the tolerance is
    /// unchanged (documented limit).
    fn local_chord(&self, st: &State, tau: Vector3) -> f64 {
        let Some(imp) = self.t.imp.as_ref() else { return self.chord };
        if self.four {
            return self.chord;
        }
        let j = self.mj(st.mu);
        let Some(nm) = unit(j.du.cross(j.dv)) else { return self.chord };
        let Some(w) = unit(nm.cross(tau)) else { return self.chord };
        let (e, f, g) = (j.du.dot(j.du), j.du.dot(j.dv), j.dv.dot(j.dv));
        let det = e * g - f * f;
        if det <= 1e-24 * e.max(g).max(1e-300).powi(2) {
            return self.chord;
        }
        let d = self.chord;
        let (bu, bv) = (j.du.dot(w) * d, j.dv.dot(w) * d);
        let (du, dv) = ((g * bu - f * bv) / det, (e * bv - f * bu) / det);
        let val = |sgn: f64| {
            let p = self.mj(Uv::new(st.mu.u + sgn * du, st.mu.v + sgn * dv)).p;
            imp.value(p) / imp.gradient(p).norm().max(1e-300)
        };
        let (fp, fm) = (val(1.0), val(-1.0));
        let f0 = imp.value(st.p) / imp.gradient(st.p).norm().max(1e-300);
        let d1 = (fp - fm) / (2.0 * d);
        let d2 = (fp + fm - 2.0 * f0) / (d * d);
        if d2.abs() <= 1e-300 {
            return self.chord;
        }
        let sep = 2.0 * d1.abs() / d2.abs();
        self.chord.min(sep / 12.0).max(10.0 * self.opts.tol.linear.min(self.chord / 10.0))
    }

    /// Trace from `seed` in direction `dir` (+1/-1 relative to the seed tangent).
    /// Returns the points after the seed and how the branch ended.
    fn trace(&self, seed: &State, tau0: Vector3, dir: f64, start: Point3, allow_close: bool) -> (Vec<State>, MarchEnd) {
        let mut out: Vec<State> = Vec::new();
        let mut st = *seed;
        let mut tau = tau0 * dir;
        let mut h = (self.max_step * 0.25).max(self.min_step);
        let mut guard = 0usize;
        // Near tangency the branches of the intersection come close (about twice the normal
        // angle times the curvature radius): the step is proportional to sin(angle).
        let cap = |sn: f64| self.max_step * (4.0 * sn).min(1.0);
        // The chord tolerance in force: tightened where the two branches of the intersection
        // come close, so one step's deviation stays well under their separation.
        let mut last_sn: Option<f64> = None;
        let mut chord = self.local_chord(seed, tau);
        let mut hcap = self.tangent_at(seed).map_or(self.max_step, |(_, sn)| cap(sn));
        loop {
            guard += 1;
            if out.len() >= self.opts.max_points || guard > 8 * self.opts.max_points {
                return (out, MarchEnd::Limit);
            }
            // Closure: close the loop when we come back near the start moving toward it.
            if allow_close && out.len() >= 4 {
                let to_start = start - st.p;
                if to_start.norm() <= 1.2 * h && to_start.dot(tau) > 0.0 {
                    // The closing chord obeys the chord tolerance like any other.
                    let d = to_start.norm();
                    let ok = h <= self.min_step * 1.01
                        || self.advance(&st, tau, 0.5 * d).is_ok_and(|mid| (mid.p - (st.p + to_start * 0.5)).norm() <= chord);
                    if ok {
                        return (out, MarchEnd::Closed);
                    }
                    h = (h * 0.5).max(self.min_step);
                }
            }
            h = h.min(hcap).max(self.min_step);
            match self.step(&st, tau, h, chord) {
                Step::Ok(new, t2, sn) => {
                    // Chord control from the turning angle.
                    let cosang = tau.dot(t2).clamp(-1.0, 1.0);
                    let ang = cosang.acos();
                    if h * ang > 4.0 * chord && h > self.min_step * 1.01 {
                        h = (h * 0.5).max(self.min_step);
                        continue;
                    }
                    out.push(new);
                    st = new;
                    tau = t2;
                    hcap = cap(sn);
                    // Approaching a near-tangency (the normals' angle shrinking): do not step
                    // over it. The angle grows about linearly with distance from the closest
                    // approach, so `sn / rate` estimates the distance left.
                    if let Some(prev) = last_sn {
                        let rate = (prev - sn) / h.max(1e-300);
                        if rate > 0.0 {
                            hcap = hcap.min(0.5 * sn / rate);
                        }
                    }
                    last_sn = Some(sn);
                    chord = self.local_chord(&new, t2);
                    let kappa = ang / h.max(1e-300);
                    let want = if kappa > 0.0 { (8.0 * chord / kappa).sqrt() } else { f64::INFINITY };
                    h = (h * 2.0).min(want).min(self.max_step).max(self.min_step);
                }
                Step::Boundary(_) => {
                    if let Some(b) = self.to_boundary(&st, tau, h)
                        && (b.p - st.p).norm() > self.opts.tol.linear
                    {
                        out.push(b);
                    }
                    return (out, MarchEnd::Boundary);
                }
                Step::Singular => return (out, MarchEnd::Singular),
                Step::Fail => {
                    if h <= self.min_step * 1.0001 {
                        return (out, MarchEnd::Singular);
                    }
                    h = (h * 0.5).max(self.min_step);
                }
            }
        }
    }
}

/// Trace a branch from each seed not already covered by a traced curve (`snap` apart).
fn trace_seeds(engine: &Engine, seeds: Vec<State>, traced: &mut Traced, snap: f64, curves: &mut Vec<MarchedBranch>, isolated: &mut Vec<Point3>) {
    for seed in seeds {
        if traced.near(seed.p, snap) {
            continue;
        }
        // Refine the seed onto both surfaces, then get its tangent.
        let Some((tau0, _)) = engine.tangent_at(&seed) else {
            isolated.push(seed.p);
            continue;
        };
        let (fwd, fend) = engine.trace(&seed, tau0, 1.0, seed.p, true);
        let mut branch: Vec<State>;
        let (start_end, end_end, closed);
        if fend == MarchEnd::Closed {
            branch = std::iter::once(seed).chain(fwd).collect();
            branch.push(seed);
            closed = true;
            start_end = MarchEnd::Closed;
            end_end = MarchEnd::Closed;
        } else {
            let (bwd, bend) = engine.trace(&seed, tau0, -1.0, seed.p, false);
            branch = bwd.into_iter().rev().chain(std::iter::once(seed)).chain(fwd).collect();
            closed = false;
            start_end = bend;
            end_end = fend;
        }
        if branch.len() < 2 {
            isolated.push(seed.p);
            continue;
        }
        traced.add(branch.iter().map(|s| s.p).collect());
        curves.push(MarchedBranch { states: branch, closed, start: start_end, end: end_end });
    }
}

/// Jet at `uv`; outside a bounded, non-periodic range the position continues along the tangent
/// plane at the nearest in-range point.
fn extended_jet(s: &dyn ParametricSurface, dom: &Domain2, uv: Uv) -> sm_geom_api::SurfaceJet {
    let cl = |d: &Domain1, x: f64| if d.is_periodic() || !d.range.is_bounded() { x } else { d.range.clamp(x) };
    let c = Uv::new(cl(&dom.u, uv.u), cl(&dom.v, uv.v));
    let mut j = s.jet(c);
    let (du, dv) = (uv.u - c.u, uv.v - c.v);
    if du != 0.0 || dv != 0.0 {
        j.p = j.p + j.du * du + j.dv * dv;
    }
    j
}

// ------------------------------------------------------------------ spatial index of traced curves

struct Traced {
    cell: f64,
    /// cell -> (branch, index)
    cells: BTreeMap<(i64, i64, i64), Vec<(usize, usize)>>,
    branches: Vec<Vec<Point3>>,
}

impl Traced {
    fn new(cell: f64) -> Self {
        Self { cell: cell.max(1e-12), cells: BTreeMap::new(), branches: Vec::new() }
    }
    fn key(&self, p: Point3) -> (i64, i64, i64) {
        ((p.x / self.cell).floor() as i64, (p.y / self.cell).floor() as i64, (p.z / self.cell).floor() as i64)
    }
    fn add(&mut self, pts: Vec<Point3>) {
        let b = self.branches.len();
        for (i, p) in pts.iter().enumerate() {
            let k = self.key(*p);
            self.cells.entry(k).or_default().push((b, i));
        }
        self.branches.push(pts);
    }
    /// True if `p` lies within `d` of a traced segment.
    fn near(&self, p: Point3, d: f64) -> bool {
        let k = self.key(p);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(v) = self.cells.get(&(k.0 + dx, k.1 + dy, k.2 + dz)) {
                        for &(b, i) in v {
                            let pts = &self.branches[b];
                            let q = pts[i];
                            if (q - p).norm() <= d {
                                return true;
                            }
                            for j in [i.wrapping_sub(1), i + 1] {
                                if j < pts.len() && seg_dist(p, q, pts[j]) <= d {
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }
        false
    }
}

fn seg_dist(p: Point3, a: Point3, b: Point3) -> f64 {
    let ab = b - a;
    let l2 = ab.dot(ab);
    if l2 == 0.0 {
        return (p - a).norm();
    }
    let t = ((p - a).dot(ab) / l2).clamp(0.0, 1.0);
    (p - (a + ab * t)).norm()
}

// ------------------------------------------------------------------ entry point

/// Surface-surface intersection by marching (see the module docs).
pub(crate) fn intersect_general(a: &SurfaceDef, b: &SurfaceDef, o: &SsiOptions) -> Result<SsiResult, GeomError> {
    let pa = if let SurfaceDef::Nurbs(n) = a { Some(PreparedSurface::new(n)?) } else { None };
    let pb = if let SurfaceDef::Nurbs(n) = b { Some(PreparedSurface::new(n)?) } else { None };
    let sa = make_surf(a, &pa);
    let sb = make_surf(b, &pb);
    if sa.par.is_none() && sb.par.is_none() {
        return Err(GeomError::Unsupported("implicit-only against implicit-only intersection"));
    }
    // Choose the marching surface: parametric, regular, finite extent, in that order.
    let score = |s: &Surf| -> (bool, bool, bool) { (s.par.is_some(), regular(s), finite_extent(s)) };
    let a_is_m = score(&sa) >= score(&sb);
    let (m, t) = if a_is_m { (&sa, &sb) } else { (&sb, &sa) };
    let four = t.imp.is_none() || matches!(t.def, SurfaceDef::Nurbs(_));
    if four && t.par.is_none() {
        return Err(GeomError::Unsupported("surface has neither a parametrization nor an implicit form"));
    }
    let mdom = m.dom.clone().ok_or(GeomError::Unsupported("surface has no parametrization"))?;
    let mbox = uv_extent(m, &o.window)?;
    let tbox = if four { uv_extent(t, &o.window)? } else { [Interval::REAL, Interval::REAL] };
    let diag = |w: &[f64; 6]| ((w[3] - w[0]).powi(2) + (w[4] - w[1]).powi(2) + (w[5] - w[2]).powi(2)).sqrt();
    // Characteristic size for the default step: window, else the extent of M's finite patch.
    let size = match &o.window {
        Some(w) => diag(w),
        None => {
            let par = m.par.ok_or(GeomError::Unsupported("surface has no parametrization"))?;
            let mut lo = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
            let mut hi = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
            for i in 0..=8 {
                for j in 0..=8 {
                    let p = par.eval(Uv::new(mbox[0].lo + mbox[0].len() * i as f64 / 8.0, mbox[1].lo + mbox[1].len() * j as f64 / 8.0));
                    lo = Point3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
                    hi = Point3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
                }
            }
            (hi - lo).norm().max(1e-12)
        }
    };
    let chord = o.chord_tol.unwrap_or(1000.0 * o.tol.linear).max(10.0 * o.tol.linear);
    let max_step = o.max_step.unwrap_or(size / 20.0).max(1e-9 * size);
    let mp = m.par.ok_or(GeomError::Unsupported("surface has no parametrization"))?;
    let tp = if four { t.par.zip(t.dom.clone()) } else { None };
    if four && tp.is_none() {
        return Err(GeomError::Unsupported("surface has neither a parametrization nor an implicit form"));
    }
    let engine = Engine { t, mp, tp, four, opts: o, mdom: mdom.clone(), mbox, tbox, chord, max_step, min_step: (10.0 * o.tol.linear).min(max_step) };

    // ---- seeds from iso-curves of M against T
    let nlines = o.seed_lines.max(2);
    let mut seeds: Vec<State> = Vec::new();
    for fixed_u in [true, false] {
        let (fix_box, free_box) = if fixed_u { (mbox[0], mbox[1]) } else { (mbox[1], mbox[0]) };
        for i in 0..nlines {
            let c = fix_box.lo + fix_box.len() * (i as f64 + 0.5) / nlines as f64;
            let (curve, scale, range_free) = match (m.def, m.par) {
                (SurfaceDef::Nurbs(_), _) => {
                    let ps = match (a_is_m, &pa, &pb) {
                        (true, Some(p), _) => p,
                        (false, _, Some(p)) => p,
                        _ => continue,
                    };
                    match ps.iso_curve(fixed_u, c) {
                        Ok(nc) => (CurveDef::Nurbs(nc), 1.0, free_box),
                        Err(_) => continue,
                    }
                }
                (def, _) => match iso_curve(def, fixed_u, c) {
                    // Analytic iso-curve parameters are the free surface parameter (scaled).
                    Some((cv, scale)) => (cv, scale, free_box),
                    None => continue,
                },
            };
            let crange = Interval::new(range_free.lo / scale, range_free.hi / scale);
            let hits = match curve_surface(&curve, Some(crange), t.def, &o.tol) {
                Ok(CurveSurface::Points(h)) => h,
                _ => continue,
            };
            for h in hits {
                if !in_box(&o.window, h.point, o.tol.linear) {
                    continue;
                }
                let free = h.t * scale;
                let mu = if fixed_u { Uv::new(c, free) } else { Uv::new(free, c) };
                // Make the surface point agree with the hit (cheap sanity check).
                let mp = engine.mpar().eval(mu);
                if (mp - h.point).norm() > 1e3 * o.tol.linear.max(1e-9 * size) {
                    continue;
                }
                let tu = h.uv.unwrap_or_default();
                seeds.push(State { mu, tu, p: h.point });
            }
        }
    }

    // ---- trace
    let mut traced = Traced::new(max_step);
    let snap = chord + 20.0 * o.tol.linear;
    let mut curves: Vec<MarchedBranch> = Vec::new();
    let mut isolated: Vec<Point3> = Vec::new();
    trace_seeds(&engine, seeds, &mut traced, snap, &mut curves, &mut isolated);

    let mut curves = join_boundary_pieces(curves, 20.0 * o.tol.linear);
    if curves.is_empty() {
        if isolated.is_empty() {
            let (points, loops) = engine.tangent_points();
            isolated = points;
            // Loops between the seed lines (a penetration deeper than the tolerance).
            trace_seeds(&engine, loops, &mut traced, snap, &mut curves, &mut isolated);
            curves = join_boundary_pieces(curves, 20.0 * o.tol.linear);
        }
        if curves.is_empty() {
            isolated.dedup_by(|a, b| (*a - *b).norm() <= 20.0 * o.tol.linear);
            return Ok(match isolated.first() {
                Some(p) if isolated.len() == 1 => SsiResult::Point(*p),
                _ => SsiResult::Empty,
            });
        }
    }
    // ---- assemble (uv of T by projection when T is analytic and was treated as implicit)
    let mut out = Vec::with_capacity(curves.len());
    for br in curves {
        let points: Vec<Point3> = br.states.iter().map(|s| s.p).collect();
        let uv_m: Vec<Uv> = br.states.iter().map(|s| s.mu).collect();
        let mut uv_t: Vec<Uv> = Vec::new();
        if four {
            uv_t = br.states.iter().map(|s| s.tu).collect();
        } else if let Some(tp) = t.par {
            let mut hint = None;
            for s in &br.states {
                let uv = tp.project(s.p, hint).map(|p| p.uv).unwrap_or_default();
                // Keep periodic parameters continuous along the branch.
                let d = tp.domain();
                let uv = match hint {
                    Some(h) => Uv::new(d.u.nearest_to(uv.u, h.u), d.v.nearest_to(uv.v, h.v)),
                    None => uv,
                };
                hint = Some(uv);
                uv_t.push(uv);
            }
        }
        let (uv_a, uv_b) = if a_is_m { (uv_m, uv_t) } else { (uv_t, uv_m) };
        let mc = MarchedCurve { points, uv_a, uv_b, closed: br.closed, start: br.start, end: br.end };
        out.push(IntersectionCurve { geometry: CurveGeometry::Marched(mc), closed: br.closed });
    }
    Ok(SsiResult::Curves(out))
}

struct MarchedBranch {
    states: Vec<State>,
    closed: bool,
    start: MarchEnd,
    end: MarchEnd,
}

/// Join pieces that were cut by a parameter boundary (the seam of a closed but non-periodic
/// surface): a `Boundary` end meets exactly one other `Boundary` end at the same 3D point. The
/// parameters jump by the period across a joint. A joined piece whose two ends meet is closed.
fn join_boundary_pieces(mut pieces: Vec<MarchedBranch>, eps: f64) -> Vec<MarchedBranch> {
    loop {
        // (piece, at_end) for every open boundary end.
        let mut ends: Vec<(usize, bool, Point3)> = Vec::new();
        for (i, b) in pieces.iter().enumerate() {
            if b.closed {
                continue;
            }
            if b.start == MarchEnd::Boundary {
                ends.push((i, false, b.states[0].p));
            }
            if b.end == MarchEnd::Boundary {
                ends.push((i, true, b.states[b.states.len() - 1].p));
            }
        }
        let mut pair = None;
        'outer: for x in 0..ends.len() {
            let cands: Vec<usize> = (0..ends.len()).filter(|&y| y != x && (ends[x].2 - ends[y].2).norm() <= eps).collect();
            if cands.len() == 1 {
                let y = cands[0];
                // The partner must also see x as its only candidate.
                let back: Vec<usize> = (0..ends.len()).filter(|&z| z != y && (ends[y].2 - ends[z].2).norm() <= eps).collect();
                if back == [x] {
                    pair = Some((x.min(y), x.max(y)));
                    break 'outer;
                }
            }
        }
        let Some((x, y)) = pair else { return pieces };
        let (pi, pend, _) = ends[x];
        let (qi, qend, _) = ends[y];
        if pi == qi {
            // The two ends of one piece meet: it is a closed loop.
            let b = &mut pieces[pi];
            let first = b.states[0];
            b.states.push(first);
            b.closed = true;
            b.start = MarchEnd::Closed;
            b.end = MarchEnd::Closed;
            continue;
        }
        // Take both pieces out (higher index first), orient so `p` ends where `q` starts.
        let (hi_i, lo_i) = (pi.max(qi), pi.min(qi));
        let hi_piece = pieces.remove(hi_i);
        let lo_piece = pieces.remove(lo_i);
        let (mut p, mut q) = if pi == lo_i { (lo_piece, hi_piece) } else { (hi_piece, lo_piece) };
        if !pend {
            p.states.reverse();
            core::mem::swap(&mut p.start, &mut p.end);
        }
        if qend {
            q.states.reverse();
            core::mem::swap(&mut q.start, &mut q.end);
        }
        let start = p.start;
        let end = q.end;
        p.states.extend(q.states);
        pieces.push(MarchedBranch { states: p.states, closed: false, start, end });
    }
}
