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
//! **With `grade = "off"`** the size at `p` is exact on the carriers only: the minimum over the
//! edge curves and face surfaces of the region that pass within the entity tolerance of `p`,
//! evaluated at the projection of `p`, and `h_max` elsewhere. That is the right field for curve
//! interval goals (edge points lie on the carrier) but not for unstructured schemes, which query
//! points slightly off the surface and would see `h_max`. A surface counts by its carrier (the
//! trim is not re-checked at query time). **Otherwise `grade = g`** (default 0.5) the size is
//! `min(h_max, min_q (h_q + g |p - q|))` over a fixed sample set (`4 samples + 1` points per
//! curved edge, `(samples + 1)^2` per curved face, trimmed by the face; a face whose trim cannot
//! be classified keeps all its samples), so it is `g`-Lipschitz everywhere (see [`Graded`]) and
//! fills the
//! volume interior smoothly. `samples` defaults to 16. Poles and apexes (singular first
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
/// [`size_field`](crate::size_field). `targets` are the region's volumes, faces or edges (empty:
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

/// The grade of `curvature` when the spec gives none.
pub const DEFAULT_GRADE: f64 = 0.5;

#[derive(Clone, Copy)]
struct Opts {
    tol: f64,
    h_max: f64,
    h_min: f64,
    angle: Option<f64>,
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
    let o = Opts { tol, h_max, h_min, angle };
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
            Box::new(Graded::new(&samples, g, h_max))
        }
        None => Box::new(Local { feats, h_max, o }),
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
    Surface { def: SurfaceDef },
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

fn finish(kind: Kind, eps: f64, samples: Vec<(Point3, f64)>) -> Option<Feature> {
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
    let pad = 0.05 * diag + 10.0 * eps;
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
    finish(Kind::Curve { def, lo, hi }, view.tolerance_of(e), samples)
}

fn surface_feature(view: &dyn TopologyView, f: EntityId, n: usize, o: Opts) -> Option<Feature> {
    let sc = match view.carrier(f) {
        Carrier::Surface(sc) => Some(*sc),
        _ => view.composite_surface(f),
    }?;
    let def = view.shapes().surface_def(sc.surface)?.clone();
    let s = def.as_surface()?;
    let dom = s.domain();
    // Parameter box: the whole period on a periodic axis, else the projected boundary samples.
    let mut ulo = f64::INFINITY;
    let mut uhi = f64::NEG_INFINITY;
    let mut vlo = f64::INFINITY;
    let mut vhi = f64::NEG_INFINITY;
    for &e in view.face_edges(f) {
        for i in 0..=16 {
            let Ok(p) = sm_scheme_api::geom::edge_point(view, e, f64::from(i) / 16.0) else { continue };
            if let Ok(pr) = s.project(p, None) {
                ulo = ulo.min(pr.uv.u);
                uhi = uhi.max(pr.uv.u);
                vlo = vlo.min(pr.uv.v);
                vhi = vhi.max(pr.uv.v);
            }
        }
    }
    let axis = |d: &sm_geom_api::Domain1, lo: f64, hi: f64| -> Option<(f64, f64)> {
        if let Some(per) = d.period {
            return Some((d.range.lo, d.range.lo + per));
        }
        let (a, b) = (lo.max(d.range.lo), hi.min(d.range.hi));
        (a.is_finite() && b.is_finite() && b >= a).then_some((a, b))
    };
    let (u0, u1) = axis(&dom.u, ulo, uhi)?;
    let (v0, v1) = axis(&dom.v, vlo, vhi)?;
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
    finish(Kind::Surface { def }, view.tolerance_of(f), samples)
}

/// The exact, ungraded field: the minimum over the curved features that pass through `p`.
struct Local {
    feats: Vec<Feature>,
    h_max: f64,
    o: Opts,
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
                if pr.distance > f.eps {
                    return None;
                }
                let per = c.domain().period;
                let slack = 1e-9 * (hi - lo);
                let inside = |t: f64| t >= lo - slack && t <= hi + slack;
                let ok = inside(pr.t) || per.is_some_and(|q| inside(pr.t + q) || inside(pr.t - q));
                ok.then(|| self.o.size(curve_k(c, pr.t).1))
            }
            Kind::Surface { def } => {
                let s = def.as_surface()?;
                let pr = s.project(p, None).ok()?;
                if pr.distance > f.eps {
                    return None;
                }
                surface_k(s, pr.uv).map(|(_, k)| self.o.size(k))
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
