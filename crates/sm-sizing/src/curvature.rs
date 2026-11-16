//! Geometry-aware size functions (opt-in): `curvature`, `point` and the `grade` wrapper, built by
//! [`size_field_geo`] from the topology view and the region's targets.
//!
//! | name | parameters | size at `p` |
//! |---|---|---|
//! | `"curvature"` | `tol`, `h_max`; optional `angle`, `h_min`, `grade` (default 0.5, or `"off"`), `samples` | chord-height rule of the local radius |
//! | `"point"` | `points: [[[x, y, z], h], ...]`, `grade`, `h_max` | `min(h_max, min_i h_i + grade \|p - x_i\|)` |
//! | `"grade"` | `grade`, `of: {name, params}` | the inner `curvature` or `point` field with that `grade` |
//!
//! **Curvature.** A chord of length `h` on a circle of radius `R` has sagitta `R - sqrt(R^2 -
//! h^2 / 4)`; requiring it to be at most `tol` gives `h = 2 sqrt(tol (2R - tol))` (every size is
//! fine once `tol >= R`, capped at the diameter `2R`). `R = 1 / k` with `k` the exact curvature
//! of the carrier: `|C' x C''| / |C'|^3` for an edge curve, the larger principal curvature
//! (from the two fundamental forms of the surface jet) for a face. `angle` adds the cap
//! `h <= R angle`; the result is clamped to `[h_min, h_max]` (`h_min` defaults to
//! `min(tol, h_max) / 1000`, which only matters at apexes where `k` is unbounded). Planes and lines give
//! `h_max`.
//!
//! `tol` bounds the sagitta of an *edge* (a chord of the boundary curve). It is not a bound on the
//! surface deviation of a whole element: the centre of a triangle inscribed in a circle sags up
//! to about `4/3 tol` from the surface (inscribed equilateral triangle).
//!
//! **With `grade = "off"`** the size at `p` is exact on the carriers only: the minimum over the
//! edge curves and face surfaces of the region that pass within the entity tolerance of `p`,
//! evaluated at the projection of `p`, and `h_max` elsewhere. That is the right field for curve
//! interval goals (edge points lie on the carrier) but not for unstructured schemes, which query
//! points slightly off the surface and would see `h_max`. A face counts through the foot point of
//! `p` clamped to the face's parameter box (so extensions of the carrier beyond the face do not
//! count; the trim inside the box is not re-checked). **Do not use `"off"` with a tet or tri
//! scheme**: the tolerance then silently breaks (measured on a cylinder r=1, h=2, tol 0.01,
//! h_max 0.5: 615 tets with a 2.44% volume deficit at `"off"`, 2135 tets and 0.89% at the default
//! grade); [`is_ungraded_curvature`] lets the caller warn or refuse. **Otherwise `grade = g`** (default 0.5) the size is the minimum of that exact on-carrier value
//! (`rule(foot) + g |p - foot|`, the foot being the projection clamped to the face or edge range, wherever it can be below `h_max`; so the rule holds **exactly on the
//! carriers** whatever the sample spacing) and
//! `min(h_max, min_q (h_q + g |p - q|))` over a fixed sample set (`4 samples + 1` points per
//! curved edge, `(samples + 1)^2` per curved face, trimmed by the face; a face whose trim cannot
//! be classified keeps all its samples), so it is `g`-Lipschitz (see [`Graded`]; the exact term is too
//! where the curvature is constant along the carrier) and fills the volume
//! interior smoothly. `samples` defaults to 16. Poles and apexes (singular first
//! fundamental form) carry no sample or exact value: the size there comes from their neighbours.
//!
//! Everything is a pure, order-independent function of the inputs: bit-identical for 1 and N
//! threads and across runs.

use serde_json::{Value, json};
use sm_core::{EntityId, Point3};
use sm_geom_api::{CurveDef, ParametricCurve, ParametricSurface, SurfaceDef, Uv};
use sm_scheme_api::{SchemeError, SchemeName, SizeField, SizingSpec};
use sm_topo_api::{Carrier, Dim, PointClass, TopologyView};

use crate::field::{num, pos, size_field};
use crate::grade::Graded;

/// Build the size field of a sizing spec with the model geometry in reach: `curvature`, `point`
/// and `grade` here (see the [module docs](self)), every other spec through
/// [`size_field`]. `targets` are the region's volumes, faces or edges (empty:
/// the whole model). Bad parameters are [`SchemeError::BadParams`] naming the key.
pub fn size_field_geo(
    view: &dyn TopologyView,
    targets: &[EntityId],
    spec: &SizingSpec,
) -> Result<Option<Box<dyn SizeField>>, SchemeError> {
    match spec {
        SizingSpec::Function { name, params } if matches!(name.as_str(), "curvature" | "point" | "grade") => {
            let bad = |reason: String| SchemeError::BadParams { scheme: SchemeName(format!("size_field:{name}")), reason };
            build(view, targets, name, &params.0).map(Some).map_err(bad)
        }
        _ => size_field(spec),
    }
}

/// True if `spec` is a `curvature` function with `grade: "off"` (the `grade` wrapper always
/// carries a numeric grade, so it never counts). That field is exact on the
/// carriers only, so a caller meshing with a tet or tri scheme should warn or refuse (see the
/// [module docs](self)); this crate does not know the scheme.
pub fn is_ungraded_curvature(spec: &SizingSpec) -> bool {
    matches!(spec, SizingSpec::Function { name, params }
        if name == "curvature" && matches!(params.0.get("grade"), Some(Value::String(s)) if s == "off"))
}

fn build(view: &dyn TopologyView, targets: &[EntityId], name: &str, p: &Value) -> Result<Box<dyn SizeField>, String> {
    match name {
        "curvature" => Ok(curvature_field(view, targets, p)?),
        "point" => Ok(point_field(p)?),
        _ => {
            let g = nonneg(p, "grade")?;
            let of = p.get("of").and_then(Value::as_object).ok_or("`of` must be {\"name\": ..., \"params\": {...}}")?;
            let inner = of.get("name").and_then(Value::as_str).ok_or("`of.name` must be a string")?;
            if !matches!(inner, "curvature" | "point") {
                return Err(format!("`of.name` must be `curvature` or `point`, got `{inner}`"));
            }
            let mut ip = of.get("params").cloned().unwrap_or_else(|| json!({}));
            let obj = ip.as_object_mut().ok_or("`of.params` must be an object")?;
            if obj.contains_key("grade") {
                return Err("`of.params.grade` is set twice (the wrapper's `grade` is the one used)".into());
            }
            obj.insert("grade".into(), json!(g));
            build(view, targets, inner, &ip)
        }
    }
}

fn nonneg(p: &Value, key: &str) -> Result<f64, String> {
    let x = num(p, key)?;
    if x >= 0.0 { Ok(x) } else { Err(format!("`{key}` must not be negative, got {x}")) }
}

fn point_field(p: &Value) -> Result<Box<dyn SizeField>, String> {
    let h_max = pos(p, "h_max")?;
    let grade = nonneg(p, "grade")?;
    let arr = p.get("points").and_then(Value::as_array).ok_or("`points` must be [[[x, y, z], h], ...]")?;
    let mut samples = Vec::with_capacity(arr.len());
    for (i, e) in arr.iter().enumerate() {
        let bad = || format!("`points[{i}]` must be [[x, y, z], h] with finite numbers and h > 0");
        let pair = e.as_array().filter(|a| a.len() == 2).ok_or_else(bad)?;
        let c: Vec<f64> = pair[0].as_array().ok_or_else(bad)?.iter().filter_map(Value::as_f64).filter(|x| x.is_finite()).collect();
        let h = pair[1].as_f64().filter(|h| h.is_finite() && *h > 0.0).ok_or_else(bad)?;
        if c.len() != 3 {
            return Err(bad());
        }
        samples.push((Point3::new(c[0], c[1], c[2]), h));
    }
    Ok(Box::new(Graded::new(&samples, grade, h_max)))
}

/// The graded field capped by the exact on-carrier rule: `min(local, graded)`. The graded part
/// fills the volume; the cap makes the carriers honour the chord-height rule exactly, however
/// sparse the samples are (between samples the graded value rises by up to `grade` times the
/// sample spacing).
struct Capped {
    local: Local,
    graded: Graded,
}

impl SizeField for Capped {
    fn size(&self, p: Point3) -> f64 {
        self.local.size(p).min(self.graded.size(p))
    }
}

/// The grade of `curvature` when the spec gives none.
pub const DEFAULT_GRADE: f64 = 0.5;

#[derive(Clone, Copy)]
struct Opts {
    tol: f64,
    h_max: f64,
    h_min: f64,
    angle: Option<f64>,
    /// The grade when graded (the exact term then reaches as far as it can matter), else `None`.
    grade: Option<f64>,
}

impl Opts {
    /// The size for curvature `k` (>= 0).
    fn size(&self, k: f64) -> f64 {
        if k.is_nan() || k <= 1e-300 {
            return self.h_max;
        }
        let r = 1.0 / k;
        let mut h = if self.tol >= r { 2.0 * r } else { 2.0 * (self.tol * (2.0 * r - self.tol)).sqrt() };
        if let Some(a) = self.angle {
            h = h.min(r * a);
        }
        h.min(self.h_max).max(self.h_min)
    }
}

fn curvature_field(view: &dyn TopologyView, targets: &[EntityId], p: &Value) -> Result<Box<dyn SizeField>, String> {
    let tol = pos(p, "tol")?;
    let h_max = pos(p, "h_max")?;
    let h_min = if p.get("h_min").is_some() { pos(p, "h_min")? } else { tol.min(h_max) * 1e-3 };
    if h_min > h_max {
        return Err("`h_min` exceeds `h_max`".into());
    }
    let angle = if p.get("angle").is_some() { Some(pos(p, "angle")?) } else { None };
    if angle.is_some_and(|a| a > std::f64::consts::PI) {
        return Err("`angle` must be at most pi".into());
    }
    let grade = match p.get("grade") {
        None => Some(DEFAULT_GRADE),
        Some(Value::String(s)) if s == "off" => None,
        Some(_) => Some(nonneg(p, "grade").map_err(|e| format!("{e} (or \"off\")"))?),
    };
    let n = match p.get("samples") {
        None => 16,
        Some(v) => v.as_u64().filter(|n| (2..=512).contains(n)).ok_or("`samples` must be an integer in 2..=512")? as usize,
    };
    let o = Opts { tol, h_max, h_min, angle, grade };
    let (edges, faces) = gather(view, targets);
    let mut feats = Vec::new();
    for e in edges {
        if let Some(f) = curve_feature(view, e, n, o) {
            feats.push(f);
        }
    }
    for f in faces {
        if let Some(ft) = surface_feature(view, f, n, o) {
            feats.push(ft);
        }
    }
    Ok(match grade {
        Some(g) => {
            let mut samples = Vec::new();
            for f in &feats {
                samples.extend_from_slice(&f.samples);
            }
            Box::new(Capped { local: Local { feats, h_max, o, grade: g }, graded: Graded::new(&samples, g, h_max) })
        }
        None => Box::new(Local { feats, h_max, o, grade: 0.0 }),
    })
}

/// The live edges and faces of the region, sorted and deduplicated.
fn gather(view: &dyn TopologyView, targets: &[EntityId]) -> (Vec<EntityId>, Vec<EntityId>) {
    let mut edges = Vec::new();
    let mut faces = Vec::new();
    if targets.is_empty() {
        edges.extend_from_slice(view.cells(Dim::Edge));
        faces.extend_from_slice(view.cells(Dim::Face));
    }
    for &t in targets {
        if !view.is_alive(t) {
            continue;
        }
        match view.dim_of(t) {
            Some(Dim::Volume) => faces.extend_from_slice(view.volume_faces(t)),
            Some(Dim::Face) => faces.push(t),
            Some(Dim::Edge) => edges.push(t),
            _ => {}
        }
    }
    for &f in &faces {
        edges.extend_from_slice(view.face_edges(f));
    }
    for v in [&mut edges, &mut faces] {
        v.sort_unstable();
        v.dedup();
        v.retain(|&id| view.is_alive(id));
    }
    (edges, faces)
}

enum Kind {
    Curve { def: CurveDef, lo: f64, hi: f64 },
    Surface { def: SurfaceDef, u: Axis, v: Axis },
}

/// The parameter range of a face along one axis, used to clamp the foot point.
#[derive(Clone, Copy)]
enum Axis {
    /// Bounded range.
    Range(f64, f64),
    /// Partial arc of a periodic axis: from `start`, length `len`, period `per`.
    Arc { start: f64, len: f64, per: f64 },
    /// Whole period starting at `lo`: no clamping.
    Wrap { lo: f64, per: f64 },
}

impl Axis {
    /// Parameter interval to sample (may exceed the domain by up to a period on periodic axes).
    fn span(self) -> (f64, f64) {
        match self {
            Axis::Range(a, b) => (a, b),
            Axis::Arc { start, len, .. } => (start, start + len),
            Axis::Wrap { lo, per } => (lo, lo + per),
        }
    }

    /// Nearest parameter of the range to `x`.
    fn clamp(self, x: f64) -> f64 {
        match self {
            Axis::Range(a, b) => x.clamp(a, b),
            Axis::Arc { start, len, per } => {
                let rel = (x - start).rem_euclid(per);
                if rel <= len {
                    x
                } else if rel - len < per - rel {
                    start + len
                } else {
                    start
                }
            }
            Axis::Wrap { .. } => x,
        }
    }
}

/// Candidate ranges of the projected boundary parameters `xs` along an axis of domain `d`: on a
/// periodic axis one arc per circular gap that could be the face's complement (ties give several).
fn axis_cands(d: &sm_geom_api::Domain1, xs: &[f64]) -> Vec<Axis> {
    if xs.is_empty() {
        return Vec::new();
    }
    if let Some(per) = d.period {
        let lo = d.range.lo;
        let mut r: Vec<f64> = xs.iter().map(|x| (x - lo).rem_euclid(per)).collect();
        r.sort_by(f64::total_cmp);
        let n = r.len();
        // Circular gaps: gap i runs from r[i-1] to r[i] (gap 0 wraps around the period).
        let gaps: Vec<f64> = (0..n).map(|i| if i == 0 { r[0] + per - r[n - 1] } else { r[i] - r[i - 1] }).collect();
        let big = gaps.iter().copied().fold(0.0, f64::max);
        // A boundary with no extent along the axis (a seam meridian, a pole) says nothing: whole period.
        let partial = big > 0.2 * per && per - big > 0.01 * per;
        if !partial {
            return vec![Axis::Wrap { lo, per }];
        }
        // Every gap within 1% of the largest is a candidate (a face bounded by two meridians has two).
        return (0..n)
            .filter(|&i| gaps[i] >= big - 0.01 * per)
            .map(|i| Axis::Arc { start: lo + r[i], len: per - gaps[i], per })
            .collect();
    }
    let a = xs.iter().copied().fold(f64::INFINITY, f64::min).max(d.range.lo);
    let b = xs.iter().copied().fold(f64::NEG_INFINITY, f64::max).min(d.range.hi);
    if a.is_finite() && b.is_finite() && b >= a { vec![Axis::Range(a, b)] } else { Vec::new() }
}

/// The range along one axis; `None` when empty or when the side of a periodic face is undecided
/// by `pick` (callers then fall back to the whole period).
#[cfg(test)]
fn axis_of(d: &sm_geom_api::Domain1, xs: &[f64]) -> Option<Axis> {
    let c = axis_cands(d, xs);
    (c.len() == 1).then(|| c[0])
}

/// Choose the parameter box among the candidates by classifying the box centre against the face
/// (a pole or meridian-only boundary leaves tied arcs). Undecided: whole period (finer, safe).
fn pick_axes(view: &dyn TopologyView, f: EntityId, dom: &sm_geom_api::Domain2, ucs: &[Axis], vcs: &[Axis]) -> Option<(Axis, Axis)> {
    if ucs.is_empty() || vcs.is_empty() {
        return None;
    }
    if ucs.len() == 1 && vcs.len() == 1 {
        return Some((ucs[0], vcs[0]));
    }
    let mid = |a: Axis| {
        let (x0, x1) = a.span();
        0.5 * (x0 + x1)
    };
    let mut hit = None;
    for &ua in ucs {
        for &va in vcs {
            if matches!(view.classify_point_on_face(f, Uv::new(mid(ua), mid(va))), Ok(PointClass::In)) {
                if hit.is_some() {
                    return Some((wrap_of(&dom.u, ucs[0]), wrap_of(&dom.v, vcs[0])));
                }
                hit = Some((ua, va));
            }
        }
    }
    Some(hit.unwrap_or_else(|| (wrap_of(&dom.u, ucs[0]), wrap_of(&dom.v, vcs[0]))))
}

/// The whole-period axis of a periodic domain, or the (single) range otherwise.
fn wrap_of(d: &sm_geom_api::Domain1, fallback: Axis) -> Axis {
    match d.period {
        Some(per) if matches!(fallback, Axis::Arc { .. }) => Axis::Wrap { lo: d.range.lo, per },
        _ => fallback,
    }
}

/// A curved edge or face of the region: its geometry, the entity tolerance, a padded box of the
/// curved samples and those samples with their sizes.
struct Feature {
    kind: Kind,
    eps: f64,
    lo: [f64; 3],
    hi: [f64; 3],
    samples: Vec<(Point3, f64)>,
}

fn finish(kind: Kind, eps: f64, o: Opts, samples: Vec<(Point3, f64)>) -> Option<Feature> {
    if samples.is_empty() {
        return None;
    }
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for (p, _) in &samples {
        for (k, x) in [p.x, p.y, p.z].into_iter().enumerate() {
            lo[k] = lo[k].min(x);
            hi[k] = hi[k].max(x);
        }
    }
    let diag = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt();
    let hmin = samples.iter().map(|s| s.1).fold(f64::INFINITY, f64::min);
    // Graded: the exact term `rule + g dist` matters until it reaches `h_max`.
    let eps = match o.grade {
        None => eps,
        Some(g) if g > 0.0 => eps.max((o.h_max - hmin) / g),
        Some(_) => f64::INFINITY,
    };
    let pad = if eps.is_finite() { 0.05 * diag + eps } else { f64::INFINITY };
    for k in 0..3 {
        lo[k] -= pad;
        hi[k] += pad;
    }
    Some(Feature { kind, eps, lo, hi, samples })
}

fn curve_k(c: &dyn ParametricCurve, t: f64) -> (Point3, f64) {
    let j = c.jet(t);
    let s = j.d1.norm();
    let k = if s > 0.0 { j.d1.cross(j.d2).norm() / (s * s * s) } else { 0.0 };
    (j.p, if k.is_finite() { k } else { 0.0 })
}

/// Larger principal curvature magnitude at `uv`, or `None` at a singular point.
fn surface_k(s: &dyn ParametricSurface, uv: Uv) -> Option<(Point3, f64)> {
    let j = s.jet(uv);
    let (e, f, g) = (j.du.dot(j.du), j.du.dot(j.dv), j.dv.dot(j.dv));
    let w = j.du.cross(j.dv);
    let wn = w.norm();
    let det = e * g - f * f;
    if !(wn > 0.0 && det > 1e-12 * e * g && det.is_finite()) {
        return None;
    }
    let n = w * (1.0 / wn);
    let (l, m, nn) = (j.duu.dot(n), j.duv.dot(n), j.dvv.dot(n));
    // Eigenvalues of the shape operator I^-1 II = [[a, b], [c, d]]. The difference form of the
    // discriminant has no cancellation at umbilic points (spheres), unlike H^2 - K.
    let a = (g * l - f * m) / det;
    let b = (g * m - f * nn) / det;
    let c = (e * m - f * l) / det;
    let d = (e * nn - f * m) / det;
    let hm = 0.5 * (a + d);
    let disc = (0.25 * (a - d) * (a - d) + b * c).max(0.0).sqrt();
    let k = (hm + disc).abs().max((hm - disc).abs());
    k.is_finite().then_some((j.p, k))
}

fn curve_feature(view: &dyn TopologyView, e: EntityId, n: usize, o: Opts) -> Option<Feature> {
    let Carrier::Curve(cc) = view.carrier(e) else { return None };
    let def = view.shapes().curve_def(cc.curve)?.clone();
    let (lo, hi) = (cc.range.lo, cc.range.hi);
    if !(lo.is_finite() && hi.is_finite() && hi > lo) {
        return None;
    }
    let c = def.as_curve();
    let m = 4 * n;
    let mut samples = Vec::new();
    for i in 0..=m {
        let (p, k) = curve_k(c, lo + (hi - lo) * (i as f64) / (m as f64));
        let h = o.size(k);
        if h < o.h_max {
            samples.push((p, h));
        }
    }
    finish(Kind::Curve { def, lo, hi }, view.tolerance_of(e), o, samples)
}

fn surface_feature(view: &dyn TopologyView, f: EntityId, n: usize, o: Opts) -> Option<Feature> {
    let sc = match view.carrier(f) {
        Carrier::Surface(sc) => Some(*sc),
        _ => view.composite_surface(f),
    }?;
    let def = view.shapes().surface_def(sc.surface)?.clone();
    let s = def.as_surface()?;
    let dom = s.domain();
    // Parameter box from the projected boundary samples (a periodic axis the boundary wraps
    // around keeps the whole period).
    let (mut us, mut vs) = (Vec::new(), Vec::new());
    for &e in view.face_edges(f) {
        for i in 0..=16 {
            let Ok(p) = sm_scheme_api::geom::edge_point(view, e, f64::from(i) / 16.0) else { continue };
            if let Ok(pr) = s.project(p, None) {
                us.push(pr.uv.u);
                vs.push(pr.uv.v);
            }
        }
    }
    let (ucs, vcs) = (axis_cands(&dom.u, &us), axis_cands(&dom.v, &vs));
    let (ua, va) = pick_axes(view, f, &dom, &ucs, &vcs)?;
    let ((u0, u1), (v0, v1)) = (ua.span(), va.span());
    let mut samples = Vec::new();
    for i in 0..=n {
        for j in 0..=n {
            let uv = Uv::new(u0 + (u1 - u0) * (i as f64) / (n as f64), v0 + (v1 - v0) * (j as f64) / (n as f64));
            // A face whose trim cannot be classified (periodic loops) keeps every sample: finer, safe.
            if matches!(view.classify_point_on_face(f, uv), Ok(PointClass::Out)) {
                continue;
            }
            if let Some((p, k)) = surface_k(s, uv) {
                let h = o.size(k);
                if h < o.h_max {
                    samples.push((p, h));
                }
            }
        }
    }
    finish(Kind::Surface { def, u: ua, v: va }, view.tolerance_of(f), o, samples)
}

/// The exact, ungraded field: the minimum over the curved features that pass through `p`.
struct Local {
    feats: Vec<Feature>,
    h_max: f64,
    o: Opts,
    /// Added per unit distance from the carrier (0 for the exact, ungraded field).
    grade: f64,
}

impl Local {
    fn at(&self, f: &Feature, p: Point3) -> Option<f64> {
        if p.x < f.lo[0] || p.x > f.hi[0] || p.y < f.lo[1] || p.y > f.hi[1] || p.z < f.lo[2] || p.z > f.hi[2] {
            return None;
        }
        match &f.kind {
            Kind::Curve { def, lo, hi } => {
                let c = def.as_curve();
                let pr = c.project(p, None).ok()?;
                let per = c.domain().period;
                let slack = 1e-9 * (hi - lo);
                let inside = |t: f64| t >= lo - slack && t <= hi + slack;
                let (t, d) = if inside(pr.t) || per.is_some_and(|q| inside(pr.t + q) || inside(pr.t - q)) {
                    (pr.t, pr.distance)
                } else {
                    // Beyond the arc: the nearer end point (the term stays continuous).
                    let (da, db) = ((p - curve_k(c, *lo).0).norm(), (p - curve_k(c, *hi).0).norm());
                    if da <= db { (*lo, da) } else { (*hi, db) }
                };
                if d > f.eps {
                    return None;
                }
                Some(self.o.size(curve_k(c, t).1) + self.grade * d)
            }
            Kind::Surface { def, u, v } => {
                let s = def.as_surface()?;
                let pr = s.project(p, None).ok()?;
                // Foot clamped to the face's parameter box: the carrier beyond the face is no feature.
                let uc = Uv::new(u.clamp(pr.uv.u), v.clamp(pr.uv.v));
                let (k, d) = if uc.u == pr.uv.u && uc.v == pr.uv.v {
                    (surface_k(s, pr.uv)?.1, pr.distance)
                } else {
                    let (q, k) = surface_k(s, uc)?;
                    (k, (p - q).norm())
                };
                (d <= f.eps).then(|| self.o.size(k) + self.grade * d)
            }
        }
    }
}

impl SizeField for Local {
    fn size(&self, p: Point3) -> f64 {
        let mut h = self.h_max;
        for f in &self.feats {
            if let Some(x) = self.at(f, p) {
                h = h.min(x);
            }
        }
        h
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_core::Vector3;
    use sm_geom_api::{Circle, Cylinder, Frame};
    use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI, TAU};

    const TOL: f64 = 0.01;
    const H_MAX: f64 = 2.0;

    fn frame() -> Frame {
        Frame::from_zx(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 0.0, 0.0)).expect("frame")
    }

    fn rule() -> f64 {
        2.0 * (TOL * (2.0 - TOL)).sqrt()
    }

    fn opts(g: f64) -> Opts {
        Opts { tol: TOL, h_max: H_MAX, h_min: TOL * 1e-3, angle: None, grade: Some(g) }
    }

    fn local(kind: Kind, g: f64) -> Local {
        let o = opts(g);
        let samples = vec![(Point3::new(1.0, 0.0, 0.0), rule()), (Point3::new(0.0, 1.0, 2.0), rule())];
        Local { feats: vec![finish(kind, 1e-6, o, samples).expect("feature")], h_max: H_MAX, o, grade: g }
    }

    fn cylinder_face(u: Axis) -> Kind {
        Kind::Surface { def: SurfaceDef::Cylinder(Cylinder::new(frame(), 1.0).expect("cyl")), u, v: Axis::Range(0.0, 2.0) }
    }

    /// Beyond a finite cylinder face the size is `min(h_max, rule + g d)` with `d` the axial
    /// distance (and `h_max` past the reach), for the full wrap and a quarter arc alike.
    #[test]
    fn carrier_extension_beyond_a_cylinder_face_grows_with_the_distance() {
        for g in [0.5, 0.1] {
            for u in [Axis::Wrap { lo: 0.0, per: TAU }, Axis::Arc { start: 0.0, len: FRAC_PI_2, per: TAU }] {
                let f = local(cylinder_face(u), g);
                for d in [0.0, 0.5, 1.0, 3.0, 10.0, 100.0] {
                    let want = (rule() + g * d).min(H_MAX);
                    for z in [2.0 + d, -d] {
                        let got = f.size(Point3::new(1.0, 0.0, z));
                        assert!((got - want).abs() < 1e-9, "g={g} d={d} z={z}: {got} vs {want}");
                    }
                }
            }
        }
    }

    /// A point on the carrier outside the quarter arc is measured from the arc end, and the
    /// whole-period face keeps the rule on the far side.
    #[test]
    fn partial_cylinder_clamps_the_angle_and_the_full_wrap_does_not() {
        let g = 0.5;
        let q = local(cylinder_face(Axis::Arc { start: 0.0, len: FRAC_PI_2, per: TAU }), g);
        let p = Point3::new(-1.0, 0.0, 1.0);
        let want = (rule() + g * 2.0_f64.sqrt()).min(H_MAX);
        assert!((q.size(p) - want).abs() < 1e-9, "{} vs {want}", q.size(p));
        let w = local(cylinder_face(Axis::Wrap { lo: 0.0, per: TAU }), g);
        assert!((w.size(p) - rule()).abs() < 1e-9);
    }

    /// A quarter circle edge: carrier points beyond the arc ends are measured from the nearer end.
    #[test]
    fn partial_arc_edge_clamps_to_the_end_points() {
        let g = 0.5;
        let kind = Kind::Curve { def: CurveDef::Circle(Circle::new(frame(), 1.0).expect("circle")), lo: 0.0, hi: FRAC_PI_2 };
        let f = local(kind, g);
        let on = f.size(Point3::new(FRAC_1_SQRT_2, FRAC_1_SQRT_2, 0.0));
        assert!((on - rule()).abs() < 1e-9, "{on}");
        for (t, end) in [(PI, Point3::new(0.0, 1.0, 0.0)), (-FRAC_PI_2, Point3::new(1.0, 0.0, 0.0))] {
            let p = Point3::new(t.cos(), t.sin(), 0.0);
            let want = (rule() + g * (p - end).norm()).min(H_MAX);
            assert!((f.size(p) - want).abs() < 1e-9, "t={t}: {} vs {want}", f.size(p));
        }
    }

    #[test]
    fn axis_detects_arcs_and_wraps() {
        let d = sm_geom_api::Domain1::periodic(0.0, TAU);
        let arc: Vec<f64> = (0..=16).map(|i| 5.5 + FRAC_PI_2 * f64::from(i) / 16.0).collect();
        match axis_of(&d, &arc) {
            Some(Axis::Arc { start, len, .. }) => assert!((start - 5.5).abs() < 1e-12 && (len - FRAC_PI_2).abs() < 1e-12, "{start} {len}"),
            _ => panic!("arc expected"),
        }
        let ring: Vec<f64> = (0..=16).map(|i| TAU * f64::from(i) / 16.0).collect();
        assert!(matches!(axis_of(&d, &ring), Some(Axis::Wrap { .. })));
        assert!(matches!(axis_of(&d, &[1.0, 1.0, 1.0]), Some(Axis::Wrap { .. })), "a seam alone wraps");
    }

    #[test]
    fn two_meridians_leave_two_tied_arcs() {
        let d = sm_geom_api::Domain1::periodic(0.0, TAU);
        let c = axis_cands(&d, &[0.0, std::f64::consts::PI, 0.0, std::f64::consts::PI]);
        assert_eq!(c.len(), 2);
        assert!(axis_of(&d, &[0.0, std::f64::consts::PI]).is_none(), "tie is undecided without the face");
        let starts: Vec<f64> = c.iter().map(|a| if let Axis::Arc { start, len, .. } = a { assert!((len - std::f64::consts::PI).abs() < 1e-12); *start } else { f64::NAN }).collect();
        assert!(starts.contains(&0.0) && starts.contains(&std::f64::consts::PI), "{starts:?}");
    }
}
