//! The surface `map` scheme: a transfinite (Coons) interpolation of a face with four logical
//! sides, evaluated in the face's parameter space and emitted as a single structured block
//! ([`sm_scheme_api::NodeGrid`]) of quadrilaterals.
//!
//! See `DESIGN.md` for the positioning, the blending, the choice of the parameter-space
//! frame and the fallbacks. Entry points: [`register`] and [`SurfaceMap`]; the pure
//! interpolation kernel is [`tfi`].

pub mod tfi;

use sm_geom_api::{ParametricSurface, Uv};
use sm_mesh_api::Param;
use sm_scheme_api::geom;
use sm_scheme_api::{
    Applicability, Caps, ElemShape, FailureKind, GenCtx, GenerateFailure, IntervalConstraint,
    IntervalCtx, IntervalSink, LogicalSide, NodeGrid, NodeKey, Params, Patch, PatchBuilder,
    PlanCtx, Requirements, Scheme, SchemeDim, SchemeError, SchemeRegistry, Target, VertexRule,
    VertexType,
};
use sm_topo_api::{Carrier, Side, TopologyView};

/// Registered name of the scheme.
pub const NAME: &str = "map";

/// Register the surface map scheme (`"map"`, dimension [`SchemeDim::Surface`]).
pub fn register(registry: &mut SchemeRegistry) -> sm_scheme_api::Result<()> {
    registry.register(Box::new(SurfaceMap))
}

/// Surface mapping: four logical sides, opposite sides with equal interval sums, interior by
/// transfinite interpolation. Produces `QUAD | STRUCTURED | BLOCK_STRUCTURED`. Takes no
/// parameters.
#[derive(Clone, Copy, Debug, Default)]
pub struct SurfaceMap;

/// The nominal angle classes of a face's corners decide the applicability score.
fn corner_classes(angles: impl Iterator<Item = f64>) -> [usize; 4] {
    let mut n = [0usize; 4];
    for a in angles {
        n[VertexType::nearest(a).element_count() as usize - 1] += 1;
    }
    n
}

fn params_empty(p: &Params) -> bool {
    p.0.is_null() || p.0.as_object().is_some_and(|o| o.is_empty())
}

impl Scheme for SurfaceMap {
    fn name(&self) -> &str {
        NAME
    }

    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }

    fn caps(&self, _params: &Params) -> Caps {
        Caps::QUAD | Caps::STRUCTURED | Caps::BLOCK_STRUCTURED
    }

    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, params: &Params) -> Applicability {
        let Target::Surface(f) = target else {
            return Applicability::no("map meshes surfaces");
        };
        if !params_empty(params) {
            return Applicability::no("map takes no parameters");
        }
        let topo = cx.input.topo;
        if geom::face_loops(topo, f).len() != 1 {
            return Applicability::no(
                "map needs exactly one boundary loop (inner loops cannot be mapped)",
            );
        }
        if !cx.input.embedded_edges(f).is_empty() || !cx.input.embedded_vertices(f).is_empty() {
            return Applicability::no("map cannot conform to embedded edges or vertices");
        }
        let corners = match geom::corners(topo, f) {
            Ok(c) => c,
            Err(e) => return Applicability::no(e.to_string()),
        };
        if corners.len() < 4 {
            return Applicability::no(format!(
                "{} corners; map needs at least four",
                corners.len()
            ));
        }
        let [ends, sides, bends, reversals] = corner_classes(corners.iter().map(|c| c.angle));
        if ends == 4 && ends + sides == corners.len() {
            Applicability::yes(100, "four End corners")
        } else if bends + reversals == 0 {
            Applicability::yes(50, "corner types need picking")
        } else {
            Applicability::yes(10, "re-entrant corners would be forced to Side or End")
        }
    }

    fn requirements(
        &self,
        _cx: &PlanCtx<'_>,
        target: Target,
        _params: &Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        if let Target::Surface(f) = target {
            out.vertex_rule(f, VertexRule::Map);
        }
        Ok(())
    }

    fn interval_constraints(
        &self,
        cx: &IntervalCtx<'_>,
        target: Target,
        _params: &Params,
        out: &mut IntervalSink<'_>,
    ) -> sm_scheme_api::Result<()> {
        let Target::Surface(f) = target else {
            return Ok(());
        };
        let layout = cx
            .layout(f)
            .ok_or_else(|| SchemeError::Input(format!("map face {f} has no layout")))?;
        if layout.sides.len() != 4 {
            return Err(SchemeError::Generate(GenerateFailure::new(
                target,
                NAME,
                FailureKind::NotApplicable,
                format!("{} logical sides; map needs four", layout.sides.len()),
            )));
        }
        let edges =
            |k: usize| -> Vec<_> { layout.sides[k].coedges.iter().map(|c| c.edge).collect() };
        for (a, b) in [(0, 2), (1, 3)] {
            let (lhs, rhs) = (edges(a), edges(b));
            // A seam used on both opposite sides cancels to an empty row.
            if matches!(IntervalConstraint::sum_equal(&lhs, &rhs), IntervalConstraint::Linear { terms, .. } if terms.is_empty())
            {
                continue;
            }
            out.equal(&lhs, &rhs);
        }
        Ok(())
    }

    fn generate(
        &self,
        cx: &GenCtx<'_>,
        target: Target,
        params: &Params,
    ) -> Result<Patch, GenerateFailure> {
        let fail = |kind: FailureKind, r: String| GenerateFailure::new(target, NAME, kind, r);
        if !params_empty(params) {
            return Err(fail(
                FailureKind::NotApplicable,
                "map takes no parameters".into(),
            ));
        }
        let Target::Surface(f) = target else {
            return Err(fail(FailureKind::Internal, "not a surface".into()));
        };
        let layout = cx
            .plan
            .layout(f)
            .ok_or_else(|| fail(FailureKind::Internal, "face has no layout".into()))?;
        if layout.sides.len() != 4 {
            return Err(fail(
                FailureKind::NotApplicable,
                format!("{} logical sides; map needs four", layout.sides.len()),
            ));
        }
        let side_keys = |k: usize| {
            cx.side_nodes(f, k)
                .ok_or_else(|| fail(FailureKind::Internal, format!("side {k} has no nodes")))
        };
        let (s0, s1, s2, s3) = (side_keys(0)?, side_keys(1)?, side_keys(2)?, side_keys(3)?);
        if s0.len() != s2.len() || s1.len() != s3.len() {
            return Err(fail(
                FailureKind::Internal,
                "opposite sides differ in interval sum (solver bug)".into(),
            ));
        }
        let (ni, nj) = (s0.len(), s1.len());
        if (ni as u64) * (nj as u64) >= u64::from(u32::MAX) {
            return Err(fail(
                FailureKind::Unsupported,
                format!("{ni} x {nj} nodes exceed the key range"),
            ));
        }

        // Boundary keys and 3-D positions.
        let pt = |k: NodeKey| {
            cx.point(k).map(|p| [p.x, p.y, p.z]).ok_or_else(|| {
                fail(
                    FailureKind::Internal,
                    format!("node {k:?} of an earlier wave is missing"),
                )
            })
        };
        let line =
            |keys: &mut dyn Iterator<Item = &NodeKey>| -> Result<Vec<[f64; 3]>, GenerateFailure> {
                keys.map(|k| pt(*k)).collect()
            };
        let bottom = line(&mut s0.iter())?;
        let right = line(&mut s1.iter())?;
        let top = line(&mut s2.iter().rev())?;
        let left = line(&mut s3.iter().rev())?;
        let w = tfi::Weights::from_lines(&bottom, &top, &left, &right);

        let mut keys = vec![NodeKey::new(f, u32::MAX); ni * nj];
        for i in 0..ni {
            keys[i] = s0[i];
            keys[(nj - 1) * ni + i] = s2[ni - 1 - i];
        }
        for j in 0..nj {
            keys[j * ni + ni - 1] = s1[j];
            keys[j * ni] = s3[nj - 1 - j];
        }

        let topo = cx.input.topo;
        let surface = surface_of(topo, f);
        let geometry = |e: String| fail(FailureKind::Geometry, e);
        let n_int = (ni - 2) * (nj - 2);
        let mut interior: Vec<([f64; 3], Param)> = Vec::with_capacity(n_int);
        // Full grid of positions in the space the orientation check runs in.
        let mut check_grid: Vec<[f64; 3]> = Vec::with_capacity(ni * nj);

        let uv_sides = surface.and_then(|_| side_uvs(cx, &layout.sides));
        if let (Some(surf), Some(uv)) = (surface, uv_sides) {
            // Parameter-space TFI, then evaluate on the surface.
            let (b, r) = (&uv[0], &uv[1]);
            let t: Vec<[f64; 2]> = uv[2].iter().rev().copied().collect();
            let l: Vec<[f64; 2]> = uv[3].iter().rev().copied().collect();
            let inner = tfi::coons(b, &t, &l, r, &w).map_err(|e| geometry(e.to_string()))?;
            let uvs: Vec<Uv> = inner.iter().map(|p| Uv::new(p[0], p[1])).collect();
            let mut pts = vec![sm_core::Point3::default(); uvs.len()];
            surf.eval_many(&uvs, &mut pts);
            for (p, q) in pts.iter().zip(&uvs) {
                interior.push(([p.x, p.y, p.z], Param::Surface([q.u, q.v])));
            }
            // Orientation is checked on the (u, v) grid.
            check_grid.resize(ni * nj, [0.0; 3]);
            for i in 0..ni {
                check_grid[i] = [b[i][0], b[i][1], 0.0];
                check_grid[(nj - 1) * ni + i] = [t[i][0], t[i][1], 0.0];
            }
            for j in 0..nj {
                check_grid[j * ni] = [l[j][0], l[j][1], 0.0];
                check_grid[j * ni + ni - 1] = [r[j][0], r[j][1], 0.0];
            }
            for j in 1..nj - 1 {
                for i in 1..ni - 1 {
                    let p = inner[(j - 1) * (ni - 2) + (i - 1)];
                    check_grid[j * ni + i] = [p[0], p[1], 0.0];
                }
            }
        } else {
            // No usable pcurves: 3-D TFI, projected onto the carrier when there is one.
            let inner = tfi::coons(&bottom, &top, &left, &right, &w)
                .map_err(|e| geometry(e.to_string()))?;
            check_grid.resize(ni * nj, [0.0; 3]);
            for i in 0..ni {
                check_grid[i] = bottom[i];
                check_grid[(nj - 1) * ni + i] = top[i];
            }
            for j in 0..nj {
                check_grid[j * ni] = left[j];
                check_grid[j * ni + ni - 1] = right[j];
            }
            let mut hint: Option<Uv> = None;
            for (n, p) in inner.iter().enumerate() {
                let (j, i) = (n / (ni - 2) + 1, n % (ni - 2) + 1);
                let q = match surface {
                    Some(surf) => {
                        let h = if i == 1 {
                            hint_row_start(&interior, ni).or(hint)
                        } else {
                            hint
                        };
                        let pr = surf
                            .project(sm_core::Point3::new(p[0], p[1], p[2]), h)
                            .map_err(|e| geometry(e.to_string()))?;
                        hint = Some(pr.uv);
                        interior.push((
                            [pr.point.x, pr.point.y, pr.point.z],
                            Param::Surface([pr.uv.u, pr.uv.v]),
                        ));
                        [pr.point.x, pr.point.y, pr.point.z]
                    }
                    None => {
                        interior.push((*p, Param::None));
                        *p
                    }
                };
                check_grid[j * ni + i] = q;
            }
        }
        orientation_ok(&check_grid, ni, nj).map_err(|r| fail(FailureKind::Quality, r))?;

        // The patch: boundary nodes by reference, interior nodes owned by the face.
        let mut pb = PatchBuilder::new(target);
        let mut idx = vec![0u32; ni * nj];
        for j in 0..nj {
            for i in 0..ni {
                let at = j * ni + i;
                if i == 0 || j == 0 || i == ni - 1 || j == nj - 1 {
                    idx[at] = pb.refer(keys[at]);
                } else {
                    let n = (j - 1) * (ni - 2) + (i - 1);
                    let key = NodeKey::new(f, n as u32);
                    keys[at] = key;
                    let (p, param) = interior[n];
                    idx[at] = pb.node(key, sm_core::Point3::new(p[0], p[1], p[2]), param);
                }
            }
        }
        for j in 0..nj - 1 {
            for i in 0..ni - 1 {
                pb.push(
                    ElemShape::Quad,
                    &[
                        idx[j * ni + i],
                        idx[j * ni + i + 1],
                        idx[(j + 1) * ni + i + 1],
                        idx[(j + 1) * ni + i],
                    ],
                );
            }
        }
        pb.grid(NodeGrid::new(ni as u32, nj as u32, keys));
        Ok(pb.finish())
    }
}

/// Hint for the first node of an interior row: the first node of the previous row.
fn hint_row_start(interior: &[([f64; 3], Param)], ni: usize) -> Option<Uv> {
    let row = ni - 2;
    if interior.len() < row {
        return None;
    }
    match interior[interior.len() - row].1 {
        Param::Surface([u, v]) => Some(Uv::new(u, v)),
        _ => None,
    }
}

/// The surface carrier of `f`.
fn surface_of(topo: &dyn TopologyView, f: sm_core::EntityId) -> Option<&dyn ParametricSurface> {
    match topo.carrier(f) {
        Carrier::Surface(sc) => topo.shapes().surface(sc.surface),
        _ => None,
    }
}

/// `(u, v)` of every node of the four logical sides (in side order, corner to corner), from the
/// pcurves of their coedges. `None` if any coedge lacks a pcurve, a node has no curve
/// parameter, or the pcurves disagree at a vertex (then the caller falls back to 3-D).
fn side_uvs(cx: &GenCtx<'_>, sides: &[LogicalSide]) -> Option<[Vec<[f64; 2]>; 4]> {
    let topo = cx.input.topo;
    let mut out: [Vec<[f64; 2]>; 4] = Default::default();
    for (k, side) in sides.iter().enumerate().take(4) {
        for ck in &side.coedges {
            let pc = topo.pcurve(*ck)?;
            let (_, cc) = geom::edge_curve(topo, ck.edge).ok()?;
            let nodes = cx.coedge_nodes(*ck)?;
            let n = nodes.len();
            let (s_start, s_end) = if ck.side == Side::Right {
                (1.0, 0.0)
            } else {
                (0.0, 1.0)
            };
            let mut seg: Vec<[f64; 2]> = Vec::with_capacity(n);
            for (m, key) in nodes.iter().enumerate() {
                let t = if m == 0 {
                    geom::edge_param(&cc, s_start)
                } else if m == n - 1 {
                    geom::edge_param(&cc, s_end)
                } else {
                    match cx.patches.node(*key)?.1 {
                        Param::Curve(t) => t,
                        _ => return None,
                    }
                };
                let uv = pc.eval(t);
                seg.push([uv.u, uv.v]);
            }
            if let Some(prev) = out[k].last() {
                if !close(*prev, seg[0]) {
                    return None;
                }
                out[k].extend_from_slice(&seg[1..]);
            } else {
                out[k].extend_from_slice(&seg);
            }
        }
    }
    // Consecutive sides share their corner, and the loop closes.
    for k in 0..4 {
        let (end, start) = (*out[k].last()?, *out[(k + 1) % 4].first()?);
        if !close(end, start) {
            return None;
        }
    }
    Some(out)
}

fn close(a: [f64; 2], b: [f64; 2]) -> bool {
    let scale = 1.0 + a[0].abs().max(a[1].abs()).max(b[0].abs()).max(b[1].abs());
    (a[0] - b[0]).abs() <= 1e-9 * scale && (a[1] - b[1]).abs() <= 1e-9 * scale
}

/// Every quad of the grid must turn the same way as the boundary outline (Newell normal), in
/// whatever space `grid` lives (`(u, v, 0)` or 3-D).
fn orientation_ok(grid: &[[f64; 3]], ni: usize, nj: usize) -> Result<(), String> {
    let mut outline: Vec<[f64; 3]> = Vec::with_capacity(2 * (ni + nj));
    outline.extend((0..ni).map(|i| grid[i]));
    outline.extend((1..nj).map(|j| grid[j * ni + ni - 1]));
    outline.extend((0..ni - 1).rev().map(|i| grid[(nj - 1) * ni + i]));
    outline.extend((1..nj - 1).rev().map(|j| grid[j * ni]));
    let mut n = [0.0; 3];
    for q in 0..outline.len() {
        let (a, b) = (outline[q], outline[(q + 1) % outline.len()]);
        n[0] += (a[1] - b[1]) * (a[2] + b[2]);
        n[1] += (a[2] - b[2]) * (a[0] + b[0]);
        n[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    if !(n.iter().all(|x| x.is_finite()) && n.iter().any(|x| *x != 0.0)) {
        return Err("the face outline has no area in its parameter space".into());
    }
    for j in 0..nj - 1 {
        for i in 0..ni - 1 {
            let (p0, p1, p2, p3) = (
                grid[j * ni + i],
                grid[j * ni + i + 1],
                grid[(j + 1) * ni + i + 1],
                grid[(j + 1) * ni + i],
            );
            let (d1, d2) = (
                [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]],
                [p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]],
            );
            let c = [
                d1[1] * d2[2] - d1[2] * d2[1],
                d1[2] * d2[0] - d1[0] * d2[2],
                d1[0] * d2[1] - d1[1] * d2[0],
            ];
            if c[0] * n[0] + c[1] * n[1] + c[2] * n[2] <= 0.0 {
                return Err(format!(
                    "the interpolation folds over at quad ({i}, {j}); the boundary is not mappable"
                ));
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::orientation_ok;

    fn lattice(ni: usize, nj: usize) -> Vec<[f64; 3]> {
        (0..nj)
            .flat_map(|j| (0..ni).map(move |i| [i as f64, j as f64, 0.0]))
            .collect()
    }

    #[test]
    fn orientation_accepts_a_regular_grid_and_rejects_a_fold() {
        let mut g = lattice(4, 4);
        assert!(orientation_ok(&g, 4, 4).is_ok());
        // Push an interior node across its neighbour: the quads around it flip.
        g[4 + 1] = [2.5, 2.5, 0.0];
        let err = orientation_ok(&g, 4, 4).expect_err("folded");
        assert!(err.contains("folds over"), "{err}");
        // A clockwise outline is the mirror image and equally valid.
        let mut mirrored = lattice(4, 4);
        for p in &mut mirrored {
            p[0] = -p[0];
        }
        assert!(orientation_ok(&mirrored, 4, 4).is_ok());
        // A collapsed outline has no area.
        let flat: Vec<[f64; 3]> = (0..16).map(|k| [k as f64 % 4.0, 0.0, 0.0]).collect();
        assert!(orientation_ok(&flat, 4, 4).is_err());
    }

    #[test]
    fn register_is_callable() {
        let mut reg = sm_scheme_api::SchemeRegistry::new();
        super::register(&mut reg).expect("register");
        assert!(
            super::register(&mut reg).is_err(),
            "double registration is rejected"
        );
    }
}
