//! The `ogrid` surface scheme: the topology adapter over the pure kernels. It recognises exact
//! disks, circle-in-square annuli and two-circle rings (see [`crate::fit`]), asks the interval
//! solver for even circle counts (disk) or one balance row (annulus, ring), and emits the kernel
//! mesh on the face with the boundary nodes taken from the earlier waves.

use crate::fit::{self, Frame, Shape};
use crate::kernels::{PolarParams, polar_disk};
use crate::mesh::{OgridError, PHASE_MAX_DEG, PlanarMesh, RATIO_MAX};
use crate::select::{self, Fixed};
use serde::Deserialize;
use sm_core::{EntityId, Point3};
use sm_scheme_api::geom;
use sm_scheme_api::{
    Applicability, Caps, ElemShape, FailureKind, GenCtx, GenerateFailure, IntervalCtx, IntervalSink, NodeKey, Params,
    Patch, PatchBuilder, PlanCtx, Rel, Requirements, Scheme, SchemeDim, SchemeError, Target, VertexRule,
};
use std::collections::BTreeMap;
use std::f64::consts::{FRAC_PI_4, PI, TAU};

/// Registered name of the scheme.
pub const NAME: &str = "ogrid";

/// Scheme parameters (JSON object, unknown keys rejected).
#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct OgridParams {
    /// `"ogrid"` (default; five-block disk) or `"polar"` (central n-gon plus rings, disk and
    /// two-circle ring only).
    pub mode: Option<String>,
    /// Radial layers; the default follows the size (layers about one size thick).
    pub rings: Option<u32>,
    /// Outermost over innermost layer thickness, within `1e-6..=1e6` (default: chosen with the
    /// layers for the best quad quality, 1 for a polar disk).
    pub ratio: Option<f64>,
    /// Angle in degrees, in the face frame (x axis = the global x axis projected on the face
    /// plane), of the first block corner of a disk (default 45) or of circle node 0 of a polar
    /// disk (default: the loop's first node). Ignored on annuli, whose corners are the square's.
    pub phase: Option<f64>,
}

/// Parsed, validated parameters.
#[derive(Clone, Copy, Debug)]
struct Opts {
    polar: bool,
    rings: Option<usize>,
    ratio: Option<f64>,
    phase: Option<f64>,
}

fn options(p: &Params) -> Result<Opts, String> {
    let q: OgridParams = if p.0.is_null() || p.0.as_object().is_some_and(|o| o.is_empty()) {
        OgridParams::default()
    } else {
        serde_json::from_value(p.0.clone()).map_err(|e| format!("ogrid parameters: {e}"))?
    };
    let polar = match q.mode.as_deref() {
        None | Some("ogrid") => false,
        Some("polar") => true,
        Some(m) => return Err(format!("ogrid: mode {m:?} is not \"ogrid\" or \"polar\"")),
    };
    if let Some(r) = q.rings
        && !(1..=1000).contains(&r)
    {
        return Err(format!("ogrid: rings {r} is outside 1..=1000"));
    }
    if let Some(ratio) = q.ratio
        && !(ratio.is_finite() && (1.0 / RATIO_MAX..=RATIO_MAX).contains(&ratio))
    {
        return Err(format!("ogrid: ratio {ratio} is outside 1e-6..=1e6"));
    }
    if let Some(ph) = q.phase
        && !(ph.is_finite() && ph.abs() <= PHASE_MAX_DEG)
    {
        return Err(format!("ogrid: phase {ph} must be finite with |phase| <= {PHASE_MAX_DEG}"));
    }
    Ok(Opts { polar, rings: q.rings.map(|r| r as usize), ratio: q.ratio, phase: q.phase })
}

/// The O-grid scheme: disks, circle-in-square annuli and two-circle rings as all-quad meshes.
/// Score 70 where applicable.
#[derive(Clone, Copy, Debug, Default)]
pub struct Ogrid;

/// Circle count nearest `length / size`, in multiples of `step` and at least `min`.
fn size_n(length: f64, size: Option<f64>, step: usize, min: usize) -> Option<usize> {
    let h = size?;
    let n = ((length / h / step as f64).round() as usize).saturating_mul(step);
    Some(n.max(min.div_ceil(step) * step))
}

/// Predict the quality of the mesh at circle count `n` (`None`: no size, so nothing to break).
fn predict(
    n: Option<usize>,
    size: Option<f64>,
    build: impl Fn(usize, Option<f64>) -> Option<select::Quality>,
) -> Result<(), String> {
    let Some(n) = n else {
        return Ok(());
    };
    let (n, h) = select::predict_scale(n, size);
    match build(n, h) {
        Some(q) => select::acceptable(&q, h),
        None => Err("no valid O-grid at the size's circle count".into()),
    }
}

/// Wrap an angle difference into `(-pi, pi]`.
fn wrap(d: f64) -> f64 {
    let mut x = d % TAU;
    if x > PI {
        x -= TAU;
    } else if x <= -PI {
        x += TAU;
    }
    x
}

/// Index of the angle nearest `target` (first on ties, with a small tolerance for round-off).
fn nearest(ang: &[f64], target: f64) -> usize {
    let mut best = (f64::INFINITY, 0);
    for (j, a) in ang.iter().enumerate() {
        let d = wrap(a - target).abs();
        if d < best.0 - 1e-9 {
            best = (d, j);
        }
    }
    best.1
}

/// Counts of the edge uses of loop `lp`, as `(edge, coefficient)` terms.
fn terms(lp: &[sm_topo_api::CoedgeKey], sign: i32, into: &mut BTreeMap<EntityId, i32>) {
    for k in lp {
        *into.entry(k.edge).or_insert(0) += sign;
    }
}

fn rows(m: BTreeMap<EntityId, i32>) -> Vec<(EntityId, i32)> {
    m.into_iter().filter(|(_, c)| *c != 0).collect()
}

impl Scheme for Ogrid {
    fn name(&self) -> &str {
        NAME
    }

    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }

    fn caps(&self, params: &Params) -> Caps {
        match options(params) {
            Ok(o) if o.polar => Caps::QUAD | Caps::POLY,
            _ => Caps::QUAD | Caps::STRUCTURED | Caps::BLOCK_STRUCTURED,
        }
    }

    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, params: &Params) -> Applicability {
        let Target::Surface(f) = target else {
            return Applicability::no("ogrid meshes surfaces");
        };        let o = match options(params) {
            Ok(o) => o,
            Err(e) => return Applicability::no(e),
        };
        if !cx.input.embedded_edges(f).is_empty() || !cx.input.embedded_vertices(f).is_empty() {
            return Applicability::no("ogrid cannot conform to embedded edges or vertices");
        }
        let shape = match fit::analyze(cx.input.topo, f) {
            Ok(Shape::SquareAnnulus { .. }) if o.polar => {
                return Applicability::no("polar mode has no circle-in-square annulus");
            }
            Ok(s) => s,
            Err(e) => return Applicability::no(e),
        };
        if o.polar {
            return Applicability::yes(70, "exact disk");
        }
        // Score the mesh `generate` would build at the size's circle count: 70 where it keeps the
        // quality and the size intent, below pave (40) where it does not.
        // Counts the user fixed are the user's resolution: nothing to predict or protect.
        if select::has_user_counts(cx, f) {
            return Applicability::yes(70, match shape {
                Shape::Disk { .. } => "exact disk",
                Shape::SquareAnnulus { .. } => "circle in a square",
                Shape::Ring { .. } => "two concentric circles",
            });
        }
        let size = select::wanted_size(cx, f);
        let fx = Fixed { rings: o.rings, ratio: o.ratio, phase_deg: o.phase };
        let (name, verdict) = match shape {
            Shape::Disk { radius, .. } => (
                "exact disk",
                predict(
                    size_n(2.0 * PI * radius, size, 2, 4),
                    size,
                    |n, h| select::disk(n, radius, h, fx).map(|b| b.2),
                ),
            ),
            Shape::SquareAnnulus { radius, half, .. } => (
                "circle in a square",
                predict(
                    size_n((2.0 * PI * radius * 8.0 * half).sqrt(), size, 4, 4),
                    size,
                    |n, h| select::annulus([n / 4; 4], radius, half, 45.0, h, fx).map(|b| b.2),
                ),
            ),
            Shape::Ring { r_in, r_out, .. } => (
                "two concentric circles",
                predict(
                    size_n(PI * (r_in + r_out), size, 1, 3),
                    size,
                    |n, h| select::ring_of(n, r_in, r_out, 0.0, h, fx).map(|b| b.2),
                ),
            ),
        };
        match verdict {
            Ok(()) => Applicability::yes(70, name),
            Err(why) => Applicability::yes(35, format!("{name}: {why}")),
        }
    }

    fn requirements(
        &self,
        cx: &PlanCtx<'_>,
        target: Target,
        _params: &Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        if let Target::Surface(f) = target {
            out.vertex_rule(f, VertexRule::Free);
            // The interval phase has no intent: hand it the wanted size, unless the user fixed
            // counts (then the scheme steers nothing).
            if let Some(h) = select::wanted_size(cx, f).filter(|_| !select::has_user_counts(cx, f)) {
                out.set_data(Params(serde_json::json!({ "size": h })));
            }
        }
        Ok(())
    }

    fn interval_constraints(
        &self,
        cx: &IntervalCtx<'_>,
        target: Target,
        params: &Params,
        out: &mut IntervalSink<'_>,
    ) -> sm_scheme_api::Result<()> {
        let Target::Surface(f) = target else {
            return Ok(());
        };
        let o = options(params)
            .map_err(|e| SchemeError::BadParams { scheme: sm_scheme_api::SchemeName::new(NAME), reason: e })?;
        let topo = cx.input.topo;
        let shape = fit::analyze(topo, f).map_err(SchemeError::Input)?;
        let loops = geom::face_loops(topo, f);
        let size = cx.data(target).and_then(|d| d.0.get("size")).and_then(serde_json::Value::as_f64);
        match shape {
            Shape::Disk { .. } => {
                let mut t = BTreeMap::new();
                terms(&loops[0], 1, &mut t);
                let all: Vec<EntityId> = loops[0].iter().map(|k| k.edge).collect();
                let t = rows(t);
                if o.polar {
                    out.linear(&t, Rel::Ge, 3);
                } else {
                    out.even(&all);
                    out.linear(&t, Rel::Ge, 4);
                }
            }
            Shape::SquareAnnulus { radius, half, .. } => {
                // The circle count at which the circle spacing and the square's spacing have the
                // wanted size as geometric mean: the sum of the sides is tied to the circle, so
                // sizing the two separately would pull the count between 8 and 16 and leave the
                // fuel disk over-resolved.
                if let Some(h) = size {
                    let n = (TAU * radius * 8.0 * half).sqrt() / h;
                    let side_edges: Vec<EntityId> = loops[0].iter().map(|k| k.edge).collect();
                    let per = (n / side_edges.len() as f64).max(1.0);
                    for e in &side_edges {
                        out.prefer(*e, per);
                        // The preference alone is averaged away by the neighbours' goals.
                        out.range(*e, per.floor() as u32, Some(per.ceil() as u32 + u32::from(per.fract() == 0.0)));
                    }
                    for e in loops[1].iter().map(|k| k.edge) {
                        out.prefer(e, (n / loops[1].len() as f64).max(1.0));
                    }
                }
                // n = m0 + m1 + m2 + m3 (the circle's evenness follows).
                let mut t = BTreeMap::new();
                terms(&loops[1], 1, &mut t);
                terms(&loops[0], -1, &mut t);
                out.linear(&rows(t), Rel::Eq, 0);
                // Equal sides: the radial lines then leave the square corners on the diagonals
                // (unequal sides twist them and cost the corner quads their Jacobian). Only when the
                // scheme steers (a size, no user counts), so a user's counts never become infeasible.
                let sides: Vec<EntityId> = loops[0].iter().map(|k| k.edge).collect();
                if size.is_some() && sides.len() == 4 {
                    for w in sides.windows(2) {
                        out.linear(&[(w[0], 1), (w[1], -1)], Rel::Eq, 0);
                    }
                }
            }
            Shape::Ring { .. } => {
                let mut t = BTreeMap::new();
                terms(&loops[0], 1, &mut t);
                terms(&loops[1], -1, &mut t);
                out.linear(&rows(t), Rel::Eq, 0);
                let mut inner = BTreeMap::new();
                terms(&loops[1], 1, &mut inner);
                out.linear(&rows(inner), Rel::Ge, 3);
            }
        }
        Ok(())
    }

    fn generate(&self, cx: &GenCtx<'_>, target: Target, params: &Params) -> Result<Patch, GenerateFailure> {
        let fail = |kind: FailureKind, r: String| GenerateFailure::new(target, NAME, kind, r);
        let Target::Surface(f) = target else {
            return Err(fail(FailureKind::Internal, "not a surface".into()));
        };
        let o = options(params).map_err(|e| fail(FailureKind::NotApplicable, e))?;
        let topo = cx.input.topo;
        let shape = fit::analyze(topo, f).map_err(|e| fail(FailureKind::NotApplicable, e))?;
        let loops = cx.loops(f).map_err(|e| fail(FailureKind::Internal, e.to_string()))?;
        let pts = |lp: &sm_scheme_api::BoundaryLoop| -> Result<Vec<[f64; 3]>, GenerateFailure> {
            lp.nodes
                .iter()
                .map(|k| {
                    cx.point(*k).map(|p| [p.x, p.y, p.z]).ok_or_else(|| {
                        fail(FailureKind::Internal, format!("boundary node {k:?} of an earlier wave is missing"))
                    })
                })
                .collect()
        };
        let geometry = |e: String| fail(FailureKind::Geometry, e);
        // Circle nodes of `lp`, counter-clockwise, with their angles.
        let circle = |frame: &Frame, lp: &sm_scheme_api::BoundaryLoop, r: f64| {
            let p = pts(lp)?;
            let (order, ang) = fit::ccw_circle_nodes(frame, &p, r).map_err(geometry)?;
            let keys: Vec<NodeKey> = order.iter().map(|&i| lp.nodes[i]).collect();
            Ok::<_, GenerateFailure>((keys, ang))
        };
        // Wanted size: the field at the first boundary node, else the size the circle count
        // implies (the solver balances the circle and square spacings around it).
        let field_size = |lp: &sm_scheme_api::BoundaryLoop| -> Option<f64> {
            let p = lp.nodes.first().and_then(|k| cx.point(*k))?;
            cx.size.map(|s| s.size(p)).filter(|h| h.is_finite() && *h > 0.0)
        };
        let fx = |phase_deg: f64| Fixed { rings: o.rings, ratio: o.ratio, phase_deg: Some(phase_deg) };
        let no_grid = |what: &str| fail(FailureKind::Quality, format!("no valid O-grid for {what}"));
        let kerr = |e: OgridError| {
            let kind =
                if matches!(e, OgridError::Inverted(_)) { FailureKind::Quality } else { FailureKind::Constraint };
            fail(kind, e.to_string())
        };

        // (kernel mesh, frame, rotation of the kernel into the frame, kernel node -> key).
        let (mesh, frame, rho, bound): (PlanarMesh, Frame, f64, Vec<(u32, NodeKey)>) = match shape {
            Shape::Disk { frame, radius } => {
                let (keys, ang) = circle(&frame, &loops[0], radius)?;
                let n = keys.len();
                if o.polar {
                    let s = o.phase.map_or(0, |ph| nearest(&ang, ph.to_radians()));
                    let k =
                        o.rings.unwrap_or_else(|| ((n as f64 / TAU).round() as usize).saturating_sub(1).clamp(1, 1000));
                    let mut p = PolarParams::new(n, k);
                    p.ratio = o.ratio.unwrap_or(1.0);
                    p.radius = radius;
                    p.phase_deg = ang[s].to_degrees();
                    let m = polar_disk(&p).map_err(kerr)?;
                    let b = (0..n).map(|j| ((k * n + j) as u32, keys[(s + j) % n])).collect();
                    (m, frame, 0.0, b)
                } else {
                    if n < 4 || n % 2 == 1 {
                        return Err(fail(
                            FailureKind::Constraint,
                            format!("{n} circle nodes; the O-grid needs an even count >= 4"),
                        ));
                    }
                    let s = nearest(&ang, o.phase.unwrap_or(45.0).to_radians());
                    let h = field_size(&loops[0]).or(Some(TAU * radius / n as f64));
                    let (_, m, _) = select::disk(n, radius, h, fx(ang[s].to_degrees())).ok_or_else(|| no_grid("the disk"))?;
                    let b = (0..n).map(|j| (j as u32, keys[(s + j) % n])).collect();
                    (m, frame, 0.0, b)
                }
            }
            Shape::SquareAnnulus { frame, radius, half, psi } => {
                let (keys, ang) = circle(&frame, &loops[1], radius)?;
                let n = keys.len();
                let outer = &loops[0];
                let m: Vec<usize> = outer.uses.iter().map(|u| u.segments as usize).collect();
                if m.len() != 4 || m.iter().sum::<usize>() != n || outer.nodes.len() != n {
                    return Err(fail(
                        FailureKind::Constraint,
                        format!("square sides {m:?} do not add up to the {n} circle nodes"),
                    ));
                }
                let s = nearest(&ang, psi);
                let skew = wrap(ang[s] - psi);
                let h = field_size(outer).or(Some((TAU * radius * 8.0 * half).sqrt() / n as f64));
                let phase = (FRAC_PI_4 + skew).to_degrees();
                let (_, mesh, _) = select::annulus([m[0], m[1], m[2], m[3]], radius, half, phase, h, fx(phase))
                    .ok_or_else(|| no_grid("the circle-in-square annulus"))?;
                let mut b: Vec<(u32, NodeKey)> = (0..n).map(|j| (j as u32, keys[(s + j) % n])).collect();
                b.extend((0..n).map(|i| ((n + i) as u32, outer.nodes[i])));
                (mesh, frame, psi - FRAC_PI_4, b)
            }
            Shape::Ring { frame, r_out, r_in } => {
                let (kin, ain) = circle(&frame, &loops[1], r_in)?;
                let (kout, aout) = circle(&frame, &loops[0], r_out)?;
                let n = kin.len();
                if kout.len() != n {
                    return Err(fail(
                        FailureKind::Constraint,
                        format!("rings with {n} and {} nodes; the circles need equal counts", kout.len()),
                    ));
                }
                let so = nearest(&aout, ain[0]);
                let h = field_size(&loops[0]).or(Some(PI * (r_in + r_out) / n as f64));
                let (p, mesh, _) = select::ring_of(n, r_in, r_out, ain[0].to_degrees(), h, fx(ain[0].to_degrees()))
                    .ok_or_else(|| no_grid("the ring"))?;
                let k = p.k;
                let mut b: Vec<(u32, NodeKey)> = (0..n).map(|j| (j as u32, kin[j])).collect();
                b.extend((0..n).map(|j| ((k * n + j) as u32, kout[(so + j) % n])));
                (mesh, frame, 0.0, b)
            }
        };

        // Kernel node -> existing key; everything else is a new node of the face.
        let mut key_of: Vec<Option<NodeKey>> = vec![None; mesh.nodes.len()];
        for (kn, key) in &bound {
            key_of[*kn as usize] = Some(*key);
        }
        let (c, s) = (rho.cos(), rho.sin());
        let world = |q: [f64; 2]| frame.world(c * q[0] - s * q[1], s * q[0] + c * q[1]);
        // The earlier waves' nodes must sit where the kernel puts its boundary.
        let scale = mesh.nodes.iter().fold(0.0f64, |m, q| m.max(q[0].abs()).max(q[1].abs())).max(f64::MIN_POSITIVE);
        for (kn, key) in &bound {
            let w = world(mesh.nodes[*kn as usize]);
            let p = cx.point(*key).ok_or_else(|| fail(FailureKind::Internal, format!("node {key:?} missing")))?;
            let d = ((w[0] - p.x).powi(2) + (w[1] - p.y).powi(2) + (w[2] - p.z).powi(2)).sqrt();
            if d > 1e-7 * scale {
                return Err(fail(
                    FailureKind::Quality,
                    format!("boundary node {key:?} is {d:.3e} from its O-grid position (edges not equally spaced?)"),
                ));
            }
        }
        let mut pb = PatchBuilder::new(target);
        let mut local = vec![0u32; mesh.nodes.len()];
        let mut fresh = 0u32;
        for (i, q) in mesh.nodes.iter().enumerate() {
            local[i] = match key_of[i] {
                Some(key) => pb.refer(key),
                None => {
                    let w = world(*q);
                    let (p, param) = geom::snap_to_face(topo, f, Point3::new(w[0], w[1], w[2]))
                        .map_err(|e| geometry(e.to_string()))?;
                    let key = NodeKey::new(f, fresh);
                    fresh += 1;
                    pb.node(key, p, param)
                }
            };
        }
        for q in &mesh.quads {
            pb.push(ElemShape::Quad, &q.map(|v| local[v as usize]));
        }
        for poly in &mesh.polys {
            let l: Vec<u32> = poly.iter().map(|&v| local[v as usize]).collect();
            pb.push(ElemShape::Polygon, &l);
        }
        Ok(pb.finish())
    }
}
