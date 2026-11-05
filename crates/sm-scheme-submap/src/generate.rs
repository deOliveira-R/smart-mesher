//! The pure `generate` of the submap scheme: boundary lattice points from the solved sides,
//! cut nodes, per-block Coons interiors, canonical keys and one quad per lattice cell.

use crate::blocks::{Block, Cut, cuts, decompose};
use crate::lattice::{PointRows, lattice_polygon};
use crate::{NAME, tfi};
use sm_core::Point3;
use sm_geom_api::{ParametricSurface, Uv};
use sm_scheme_api::geom;
use sm_scheme_api::{
    ElemShape, FailureKind, GenCtx, GenerateFailure, NodeKey, Param, Patch, PatchBuilder, SideDir,
    Target, VertexType,
};

/// Lattice-point state while the patch is assembled.
#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    /// Interior point not placed yet.
    Pending,
    /// Boundary node of an earlier job (referenced, never created).
    Boundary,
    /// Interior node placed.
    Placed,
}

/// Surface parameters of the boundary nodes of every side (`side k`: one `[u, v]` per node but
/// the last), read from the pcurves through [`GenCtx::loop_uvs`], when the face lies on a
/// periodic surface. `None` (the 3-D path) for any other face or when the pcurves and the
/// sides do not line up.
fn side_uvs(
    cx: &GenCtx<'_>,
    f: sm_core::EntityId,
    side_keys: &[Vec<NodeKey>],
) -> Option<Vec<Vec<[f64; 2]>>> {
    let topo = cx.input.topo;
    let sm_topo_api::Carrier::Surface(sc) = topo.carrier(f) else {
        return None;
    };
    let dom = topo.shapes().surface(sc.surface)?.domain();
    if !(dom.u.is_periodic() || dom.v.is_periodic()) {
        return None;
    }
    let layout = cx.plan.layout(f)?;
    let lp = cx.loops(f).ok()?.into_iter().next()?;
    let uvs = cx.loop_uvs(f, &lp)?;
    let n = lp.nodes.len();
    let mut out = Vec::with_capacity(side_keys.len());
    for (side, keys) in layout.sides.iter().zip(side_keys) {
        let c0 = *side.coedges.first()?;
        let first = lp.uses.iter().find(|u| u.coedge == c0)?.first as usize;
        let m = keys.len().checked_sub(1)?;
        let mut row = Vec::with_capacity(m);
        for (q, key) in keys[..m].iter().enumerate() {
            let at = (first + q) % n;
            if lp.nodes[at] != *key {
                return None;
            }
            row.push(uvs[at]);
        }
        out.push(row);
    }
    Some(out)
}

/// Fractions along a cut of `m` interior nodes. Uniform unless the nearest placed lattice lines
/// on either side (a boundary line or an earlier cut) are graded, in which case the average of
/// their chord fractions over the cut's rows; a uniform reference gives the exact `s / (m + 1)`.
fn cut_fractions(m: i64, refs: &[Vec<f64>]) -> Vec<f64> {
    let uniform = |s: i64| s as f64 / (m + 1) as f64;
    let graded = !refs.is_empty()
        && refs.iter().any(|r| {
            r.len() == (m + 2) as usize
                && r.iter()
                    .enumerate()
                    .any(|(s, v)| (v - uniform(s as i64)).abs() > 1e-9)
        });
    (1..=m)
        .map(|s| {
            if !graded {
                return uniform(s);
            }
            let ok: Vec<&Vec<f64>> = refs.iter().filter(|r| r.len() == (m + 2) as usize).collect();
            ok.iter().map(|r| r[s as usize]).sum::<f64>() / ok.len() as f64
        })
        .collect()
}

fn eval_uv(surface: &Option<&dyn ParametricSurface>, w: [f64; 2]) -> [f64; 3] {
    // `side_uv` is only `Some` with a surface (checked in `run`).
    surface.map_or([0.0; 3], |s| {
        let p = s.eval(Uv::new(w[0], w[1]));
        [p.x, p.y, p.z]
    })
}

pub(crate) fn run(cx: &GenCtx<'_>, target: Target) -> Result<Patch, GenerateFailure> {
    let fail = |kind: FailureKind, r: String| GenerateFailure::new(target, NAME, kind, r);
    let internal = |r: String| fail(FailureKind::Internal, r);
    let Target::Surface(f) = target else {
        return Err(internal("not a surface".into()));
    };
    let layout = cx
        .plan
        .layout(f)
        .ok_or_else(|| internal("face has no layout".into()))?;
    if layout
        .corners
        .iter()
        .any(|c| c.vertex_type == VertexType::Reversal)
    {
        return Err(fail(
            FailureKind::NotApplicable,
            "reversal corners deferred".into(),
        ));
    }
    if layout.sides.len() < 4 {
        return Err(fail(
            FailureKind::NotApplicable,
            format!(
                "{} logical sides; submap needs at least four",
                layout.sides.len()
            ),
        ));
    }

    // 1. Boundary nodes per side and the lattice polygon they trace.
    let mut side_keys: Vec<Vec<NodeKey>> = Vec::with_capacity(layout.sides.len());
    let mut spec: Vec<(SideDir, u32)> = Vec::with_capacity(layout.sides.len());
    for (k, side) in layout.sides.iter().enumerate() {
        let keys = cx
            .side_nodes(f, k)
            .ok_or_else(|| internal(format!("side {k} has no nodes")))?;
        spec.push((side.dir, keys.len().saturating_sub(1) as u32));
        side_keys.push(keys);
    }
    let poly = lattice_polygon(&spec).map_err(|e| {
        let corners = side_keys
            .iter()
            .filter_map(|s| s.first().copied())
            .collect();
        fail(FailureKind::Constraint, e.to_string()).with_nodes(corners)
    })?;
    let rows = PointRows::build(&poly).ok_or_else(|| {
        fail(
            FailureKind::Unsupported,
            "lattice exceeds the node key range".into(),
        )
    })?;
    let (area, perimeter, n_inner) = poly.pick_area();
    if u64::from(rows.total()) != perimeter + n_inner {
        return Err(internal(format!(
            "point index has {} points, Pick gives {}",
            rows.total(),
            perimeter + n_inner
        )));
    }
    let total = rows.total() as usize;
    let ord = |a: i64, b: i64| -> Result<usize, GenerateFailure> {
        rows.ordinal(a, b)
            .map(|o| o as usize)
            .ok_or_else(|| internal(format!("lattice point ({a}, {b}) is outside the polygon")))
    };

    let mut state = vec![State::Pending; total];
    let mut pos = vec![[0.0f64; 3]; total];
    let mut param = vec![Param::None; total];
    let mut bkey = vec![NodeKey::new(f, u32::MAX); total];
    let side_uv = side_uvs(cx, f, &side_keys);
    let mut uv = vec![[0.0f64; 2]; total];
    for (k, keys) in side_keys.iter().enumerate() {
        let (dx, dy) = crate::lattice::step(poly.dirs[k]);
        let (x, y) = poly.verts[k];
        // The last node is the first of the next side.
        for (m, key) in keys[..keys.len() - 1].iter().enumerate() {
            let m = m as i64;
            let o = ord(x + dx * m, y + dy * m)?;
            if state[o] == State::Boundary {
                return Err(internal(format!("lattice point of side {k} visited twice")));
            }
            let p = cx
                .point(*key)
                .ok_or_else(|| internal(format!("node {key:?} of an earlier wave is missing")))?;
            state[o] = State::Boundary;
            pos[o] = [p.x, p.y, p.z];
            bkey[o] = *key;
            if let Some(su) = &side_uv {
                uv[o] = su[k][m as usize];
            }
        }
    }

    // Canonical interior numbering: rank among interior points in row-major order.
    let mut rank = vec![u32::MAX; total];
    let mut n_interior = 0u32;
    for (o, st) in state.iter().enumerate() {
        if *st == State::Pending {
            rank[o] = n_interior;
            n_interior += 1;
        }
    }
    if u64::from(n_interior) != n_inner {
        return Err(internal(format!(
            "{n_interior} interior points, Pick gives {n_inner}"
        )));
    }

    let topo = cx.input.topo;
    let geometry = |e: String| fail(FailureKind::Geometry, e);
    let snap = |p: [f64; 3]| -> Result<([f64; 3], Param), GenerateFailure> {
        let (q, prm) = geom::snap_to_face(topo, f, Point3::new(p[0], p[1], p[2]))
            .map_err(|e| geometry(e.to_string()))?;
        Ok(([q.x, q.y, q.z], prm))
    };

    let surface = match topo.carrier(f) {
        sm_topo_api::Carrier::Surface(sc) => topo.shapes().surface(sc.surface),
        _ => None,
    };
    if side_uv.is_some() && surface.is_none() {
        return Err(internal("periodic face lost its surface".into()));
    }

    // 2. Blocks and cut nodes.
    let blocks: Vec<Block> = decompose(&poly);
    let cut_lines: Vec<Cut> = cuts(&blocks);
    for c in &cut_lines {
        let (po, qo) = (ord(c.x, c.j0)?, ord(c.x, c.j1)?);
        if state[po] != State::Boundary || state[qo] != State::Boundary {
            return Err(internal(format!(
                "cut end of x = {} is not a boundary node",
                c.x
            )));
        }
        let (p, q) = (pos[po], pos[qo]);
        let m = c.j1 - c.j0 - 1;
        // Graded spacing from the nearest placed lines to the left and right of the cut.
        let mut refs: Vec<Vec<f64>> = Vec::new();
        for step in [-1i64, 1] {
            let mut x = c.x + step;
            while let Some(line) = (c.j0..=c.j1)
                .map(|b| rows.ordinal(x, b).map(|o| o as usize))
                .collect::<Option<Vec<usize>>>()
            {
                if line.iter().all(|&o| state[o] != State::Pending) {
                    let pts: Vec<[f64; 3]> = line.iter().map(|&o| pos[o]).collect();
                    refs.push(tfi::chord_fractions(&pts));
                    break;
                }
                x += step;
            }
        }
        let fr = cut_fractions(m, &refs);
        for s in 1..=m {
            let o = ord(c.x, c.j0 + s)?;
            if state[o] != State::Pending {
                return Err(internal("cut node placed twice".into()));
            }
            let t = fr[(s - 1) as usize];
            if side_uv.is_some() {
                let (a, b) = (uv[po], uv[qo]);
                let w: [f64; 2] = std::array::from_fn(|d| a[d] + (b[d] - a[d]) * t);
                (pos[o], uv[o]) = (eval_uv(&surface, w), w);
                (param[o], state[o]) = (Param::Surface(w), State::Placed);
            } else {
                let at: [f64; 3] = std::array::from_fn(|d| p[d] + (q[d] - p[d]) * t);
                let (pt, prm) = snap(at)?;
                (pos[o], param[o], state[o]) = (pt, prm, State::Placed);
            }
        }
    }

    // 3. Block interiors: Coons on the four lattice lines of each block.
    for blk in &blocks {
        if blk.width() < 2 || blk.height() < 2 {
            continue;
        }
        let line =
            |pts: &mut dyn Iterator<Item = (i64, i64)>| -> Result<Vec<[f64; 3]>, GenerateFailure> {
                pts.map(|(a, b)| ord(a, b).map(|o| pos[o])).collect()
            };
        let bottom = line(&mut (blk.i0..=blk.i1).map(|a| (a, blk.j0)))?;
        let top = line(&mut (blk.i0..=blk.i1).map(|a| (a, blk.j1)))?;
        let left = line(&mut (blk.j0..=blk.j1).map(|b| (blk.i0, b)))?;
        let right = line(&mut (blk.j0..=blk.j1).map(|b| (blk.i1, b)))?;
        let w = (blk.width() - 1) as usize;
        let place = |state: &[State], n: usize| -> Result<(usize, i64, i64), GenerateFailure> {
            let (a, b) = (blk.i0 + 1 + (n % w) as i64, blk.j0 + 1 + (n / w) as i64);
            let o = ord(a, b)?;
            if state[o] != State::Pending {
                return Err(internal(format!("block interior ({a}, {b}) placed twice")));
            }
            Ok((o, a, b))
        };
        if side_uv.is_some() {
            // Blend the parameters, weighted by the 3-D chord fractions of the same lines.
            let uv_line = |pts: &mut dyn Iterator<Item = (i64, i64)>| -> Result<Vec<[f64; 2]>, GenerateFailure> {
                pts.map(|(a, b)| ord(a, b).map(|o| uv[o])).collect()
            };
            let ub = uv_line(&mut (blk.i0..=blk.i1).map(|a| (a, blk.j0)))?;
            let ut = uv_line(&mut (blk.i0..=blk.i1).map(|a| (a, blk.j1)))?;
            let ul = uv_line(&mut (blk.j0..=blk.j1).map(|b| (blk.i0, b)))?;
            let ur = uv_line(&mut (blk.j0..=blk.j1).map(|b| (blk.i1, b)))?;
            let wts = [&bottom, &top, &left, &right].map(|l| tfi::chord_fractions(l));
            let inner = tfi::blend(&ub, &ut, &ul, &ur, &wts)
                .ok_or_else(|| internal("block boundary lines disagree in size".into()))?;
            for (n, w) in inner.iter().enumerate() {
                let (o, _, _) = place(&state, n)?;
                (pos[o], uv[o]) = (eval_uv(&surface, *w), *w);
                (param[o], state[o]) = (Param::Surface(*w), State::Placed);
            }
        } else {
            let inner = tfi::coons(&bottom, &top, &left, &right)
                .ok_or_else(|| internal("block boundary lines disagree in size".into()))?;
            for (n, p) in inner.iter().enumerate() {
                let (o, _, _) = place(&state, n)?;
                let (pt, prm) = snap(*p)?;
                (pos[o], param[o], state[o]) = (pt, prm, State::Placed);
            }
        }
    }
    if state.contains(&State::Pending) {
        return Err(internal("an interior lattice point is in no block".into()));
    }

    // 4. The patch: boundary by reference, interior owned by the face in rank order.
    let mut pb = PatchBuilder::new(target);
    let mut idx = vec![0u32; total];
    for o in 0..total {
        idx[o] = if state[o] == State::Boundary {
            pb.refer(bkey[o])
        } else {
            let p = pos[o];
            pb.node(
                NodeKey::new(f, rank[o]),
                Point3::new(p[0], p[1], p[2]),
                param[o],
            )
        };
    }
    let key_of = |o: usize| {
        if state[o] == State::Boundary {
            bkey[o]
        } else {
            NodeKey::new(f, rank[o])
        }
    };

    // 5. One quad per cell, row-major; 6. orientation against the face normal at the centroid.
    let (_, y_min, _, y_max) = poly.bbox();
    let (mut xs, mut spans) = (Vec::new(), Vec::new());
    let mut n_quads = 0u64;
    for b in y_min..y_max {
        poly.row_spans_into(b, &mut xs, &mut spans);
        for &(c0, c1) in &spans {
            for a in c0..c1 {
                let q = [
                    ord(a, b)?,
                    ord(a + 1, b)?,
                    ord(a + 1, b + 1)?,
                    ord(a, b + 1)?,
                ];
                let [p0, p1, p2, p3] = q.map(|o| pos[o]);
                let d1: [f64; 3] = std::array::from_fn(|k| p2[k] - p0[k]);
                let d2: [f64; 3] = std::array::from_fn(|k| p3[k] - p1[k]);
                let cross = [
                    d1[1] * d2[2] - d1[2] * d2[1],
                    d1[2] * d2[0] - d1[0] * d2[2],
                    d1[0] * d2[1] - d1[1] * d2[0],
                ];
                let c: [f64; 3] = std::array::from_fn(|k| (p0[k] + p1[k] + p2[k] + p3[k]) / 4.0);
                let n = geom::face_normal(topo, f, Point3::new(c[0], c[1], c[2]))
                    .map_err(|e| geometry(e.to_string()))?;
                if cross[0] * n.x + cross[1] * n.y + cross[2] * n.z <= 0.0 {
                    return Err(fail(
                        FailureKind::Quality,
                        format!(
                            "the interpolation folds over at lattice cell ({a}, {b}); the blocks are not mappable"
                        ),
                    )
                    .with_nodes(q.map(key_of).to_vec()));
                }
                pb.push(
                    ElemShape::Quad,
                    &[idx[q[0]], idx[q[1]], idx[q[2]], idx[q[3]]],
                );
                n_quads += 1;
            }
        }
    }
    if n_quads != area {
        return Err(internal(format!("{n_quads} quads, Pick gives {area}")));
    }
    Ok(pb.finish())
}
