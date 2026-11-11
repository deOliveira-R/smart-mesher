//! Layer counts, grading and the quality prediction of the adapter. Everything here is a pure
//! function of the shape, the circle count and the wanted size, so the applicability score and
//! `generate` agree: the score is the quality of the mesh `generate` would build.

use crate::kernels::{AnnulusQuadParams, DiskParams, RingParams, ogrid_annulus_quad, ogrid_disk, ring};
use crate::mesh::PlanarMesh;
use sm_core::EntityId;
use sm_scheme_api::{IntervalRequest, PlanCtx, SizingSpec};
use sm_topo_api::Dim;
use std::f64::consts::{PI, SQRT_2};

/// Predicted worst planar edge ratio above which `ogrid` scores below the unstructured
/// alternatives. A heuristic calibrated against pave on the W3c through pin and B5 (pave reaches
/// 1.5 on the pin faces at size 0.3): the pin's fuel disk at n = 8 predicts 1.91 and loses, the B5
/// annulus at n = 20 predicts 1.74 and wins; at B5 sizes 0.3 and 0.5 the ogrid mesh beats pave on
/// every metric, at 0.7 and 1.0 the prediction fails and pave stays. The measured 3D ratio also
/// carries the sweep height, so it is higher than this planar value.
pub(crate) const ER_LIMIT: f64 = 1.8;
/// Predicted minimum scaled Jacobian below which `ogrid` scores below the alternatives.
pub(crate) const SJ_LIMIT: f64 = 0.5;
/// A quad edge more than this many times longer, or shorter, than the wanted size breaks the size
/// intent.
pub(crate) const SIZE_DEV_LIMIT: f64 = 2.5;
/// Prediction meshes are built with at most this many circle nodes (quality depends on the ratios
/// of the shape, not on the count, once the layers scale with it).
const PREDICT_MAX_N: usize = 64;

/// Quality of a planar quad mesh.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Quality {
    /// Minimum scaled Jacobian over the quad corners.
    pub sj: f64,
    /// Largest per-quad longest over shortest edge.
    pub er: f64,
    /// Shortest quad edge.
    pub edge_min: f64,
    /// Longest quad edge.
    pub edge_max: f64,
}

impl Quality {
    /// Selection cost: lower is better.
    fn cost(&self) -> f64 {
        self.er + 2.0 * (1.0 - self.sj)
    }
}

/// Measure `mesh` (quads only; polygons are ignored).
pub(crate) fn quality(mesh: &PlanarMesh) -> Quality {
    let (mut er, mut lo, mut hi) = (1.0f64, f64::INFINITY, 0.0f64);
    for q in &mesh.quads {
        let (mut a, mut b) = (f64::INFINITY, 0.0f64);
        for i in 0..4 {
            let (p, r) = (mesh.nodes[q[i] as usize], mesh.nodes[q[(i + 1) % 4] as usize]);
            let l = (p[0] - r[0]).hypot(p[1] - r[1]);
            a = a.min(l);
            b = b.max(l);
        }
        er = er.max(if a > 0.0 { b / a } else { f64::INFINITY });
        lo = lo.min(a);
        hi = hi.max(b);
    }
    Quality { sj: mesh.min_scaled_jacobian(), er, edge_min: lo, edge_max: hi }
}

/// The size wanted on face `f`: the face's own request, then those of its volumes, then a constant
/// default sizing. `None` when the intent holds none (functions are only known to the driver).
pub(crate) fn wanted_size(cx: &PlanCtx<'_>, f: EntityId) -> Option<f64> {
    let of = |id: EntityId| match cx.intent.entities.get(&id)?.intervals?.value {
        IntervalRequest::Size(s) if s.is_finite() && s > 0.0 => Some(s),
        _ => None,
    };
    let topo = cx.input.topo;
    of(f)
        .or_else(|| {
            topo.coboundary(f).iter().filter(|i| topo.dim_of(i.cell) == Some(Dim::Volume)).find_map(|i| of(i.cell))
        })
        .or(match &cx.intent.defaults.sizing {
            Some(SizingSpec::Constant { size }) if size.is_finite() && *size > 0.0 => Some(*size),
            _ => None,
        })
}

/// Whether the user fixed an interval count on face `f`, on one of its edges or on a volume
/// around it. Then the scheme steers nothing: the counts are the user's.
pub(crate) fn has_user_counts(cx: &PlanCtx<'_>, f: EntityId) -> bool {
    let counts = |id: EntityId| {
        cx.intent
            .entities
            .get(&id)
            .and_then(|e| e.intervals)
            .is_some_and(|r| matches!(r.value, IntervalRequest::Count(_)))
    };
    let topo = cx.input.topo;
    counts(f)
        || cx.input.face_edges(f).into_iter().any(counts)
        || topo.coboundary(f).iter().filter(|i| topo.dim_of(i.cell) == Some(Dim::Volume)).any(|i| counts(i.cell))
}

/// The layer counts and ratios to try: `k` fixed by the user or around `k0`, `ratio` fixed or
/// one and the value that makes the radial thickness follow the tangential spacing (`rho` is the
/// outer over inner tangential spacing).
fn candidates(k0: usize, rho: f64, rings: Option<usize>, ratio: Option<f64>) -> Vec<(usize, f64)> {
    let ks: Vec<usize> = match rings {
        Some(k) => vec![k],
        None => (k0.saturating_sub(1).max(1)..=k0 + 1).collect(),
    };
    let mut v = Vec::new();
    for k in ks {
        match ratio {
            Some(r) => v.push((k, r)),
            None => {
                v.push((k, 1.0));
                if k >= 2 && (rho - 1.0).abs() > 0.05 {
                    v.push((k, rho.powf((k - 1) as f64 / k as f64).clamp(1e-3, 1e3)));
                }
            }
        }
    }
    v
}

fn best<T>(list: Vec<(T, PlanarMesh)>) -> Option<(T, PlanarMesh, Quality)> {
    let mut out: Option<(T, PlanarMesh, Quality)> = None;
    for (p, m) in list {
        let q = quality(&m);
        if q.sj > 0.0 && out.as_ref().is_none_or(|(_, _, b)| q.cost() < b.cost() - 1e-12) {
            out = Some((p, m, q));
        }
    }
    out
}

/// Free choices of the caller (`None`: select).
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Fixed {
    pub rings: Option<usize>,
    pub ratio: Option<f64>,
    pub phase_deg: Option<f64>,
}

/// Best disk of `n` circle nodes: layers, grading and core radius selected for the quality.
pub(crate) fn disk(n: usize, radius: f64, size: Option<f64>, fx: Fixed) -> Option<(DiskParams, PlanarMesh, Quality)> {
    let h = size.unwrap_or(2.0 * PI * radius / n as f64);
    let mut list = Vec::new();
    for core in [0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7] {
        let k0 = (((1.0 - core) * radius / h).round() as usize).clamp(1, 1000);
        for (k, ratio) in candidates(k0, 1.0, fx.rings, fx.ratio) {
            let mut p = DiskParams::balanced(n, k);
            p.core = core;
            p.ratio = ratio;
            p.radius = radius;
            p.phase_deg = fx.phase_deg.unwrap_or(45.0);
            if let Ok(m) = ogrid_disk(&p) {
                list.push((p, m));
            }
        }
    }
    best(list)
}

/// Best circle-in-square annulus with side counts `m` (circle count `sum m`).
pub(crate) fn annulus(
    m: [usize; 4],
    radius: f64,
    half: f64,
    phase_deg: f64,
    size: Option<f64>,
    fx: Fixed,
) -> Option<(AnnulusQuadParams, PlanarMesh, Quality)> {
    let n: usize = m.iter().sum();
    let h = size.unwrap_or(2.0 * PI * radius / n as f64);
    let depth = 0.5 * (1.0 + SQRT_2) * half - radius;
    let k0 = ((depth / h).round() as usize).clamp(1, 1000);
    let rho = (8.0 * half / n as f64) / (2.0 * PI * radius / n as f64);
    let list = candidates(k0, rho, fx.rings, fx.ratio)
        .into_iter()
        .filter_map(|(k, ratio)| {
            let p = AnnulusQuadParams { m, k, ratio, phase_deg, radius, side: 2.0 * half };
            ogrid_annulus_quad(&p).ok().map(|mesh| (p, mesh))
        })
        .collect();
    best(list)
}

/// Best two-circle ring of `n` nodes.
pub(crate) fn ring_of(
    n: usize,
    r_in: f64,
    r_out: f64,
    phase_deg: f64,
    size: Option<f64>,
    fx: Fixed,
) -> Option<(RingParams, PlanarMesh, Quality)> {
    let h = size.unwrap_or(2.0 * PI * r_out / n as f64);
    let k0 = (((r_out - r_in) / h).round() as usize).clamp(1, 1000);
    let list = candidates(k0, r_out / r_in, fx.rings, fx.ratio)
        .into_iter()
        .filter_map(|(k, ratio)| {
            let p = RingParams { n, k, r_in, r_out, ratio, phase_deg };
            ring(&p).ok().map(|mesh| (p, mesh))
        })
        .collect();
    best(list)
}

/// Reduce a prediction to at most [`PREDICT_MAX_N`] circle nodes by scaling the wanted size up
/// (the ratios of the shape stay), so that scoring a fine mesh stays cheap.
pub(crate) fn predict_scale(n: usize, size: Option<f64>) -> (usize, Option<f64>) {
    if n <= PREDICT_MAX_N {
        return (n, size);
    }
    let s = n as f64 / PREDICT_MAX_N as f64;
    (PREDICT_MAX_N, size.map(|h| h * s))
}

/// Whether a predicted mesh keeps the quality and the size intent: the verdict behind the score.
pub(crate) fn acceptable(q: &Quality, size: Option<f64>) -> Result<(), String> {
    if q.er > ER_LIMIT {
        return Err(format!("predicted edge ratio {:.2} exceeds {ER_LIMIT}", q.er));
    }
    if q.sj < SJ_LIMIT {
        return Err(format!("predicted scaled Jacobian {:.2} is below {SJ_LIMIT}", q.sj));
    }
    if let Some(h) = size
        && (q.edge_max > SIZE_DEV_LIMIT * h || q.edge_min < h / SIZE_DEV_LIMIT)
    {
        return Err(format!("predicted edges {:.3}..{:.3} are far from the wanted size {h}", q.edge_min, q.edge_max));
    }
    Ok(())
}
