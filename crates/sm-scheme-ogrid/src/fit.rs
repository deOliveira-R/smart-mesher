//! Recognise exact disks and annuli among planar faces: fit the circle(s) and the square from
//! the loops' geometry, in a face-oriented frame. Pure geometry, no meshing.

use sm_core::{EntityId, Point3};
use sm_scheme_api::geom;
use sm_topo_api::{CoedgeKey, TopologyView};
use std::f64::consts::{PI, TAU};

/// Relative tolerance for "exact" circles, squares and planes.
const TOL: f64 = 1e-9;
/// Samples per edge used to fit and verify.
const SAMPLES: usize = 8;

type V3 = [f64; 3];

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}
fn arr(p: Point3) -> V3 {
    [p.x, p.y, p.z]
}

/// Orthonormal frame of the face plane: origin, in-plane axes and the face normal
/// (`e2 = n x e1`), so angles measured from `e1` toward `e2` are counter-clockwise about `n`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Frame {
    pub o: V3,
    pub e1: V3,
    pub e2: V3,
}

impl Frame {
    /// Local planar coordinates of `p`.
    pub fn local(&self, p: V3) -> [f64; 2] {
        let d = sub(p, self.o);
        [dot(d, self.e1), dot(d, self.e2)]
    }
    /// The 3-D point of local `(x, y)`.
    pub fn world(&self, x: f64, y: f64) -> V3 {
        std::array::from_fn(|k| self.o[k] + x * self.e1[k] + y * self.e2[k])
    }
}

/// What a face is, for the O-grid scheme.
#[derive(Clone, Debug)]
pub(crate) enum Shape {
    /// One circular outer loop.
    Disk { frame: Frame, radius: f64 },
    /// Square outer loop (four straight edges) and a circular hole. `psi` is the angle of the
    /// first corner (start of the loop's first coedge) in the frame; `half` the half side.
    SquareAnnulus { frame: Frame, radius: f64, half: f64, psi: f64 },
    /// Two concentric circles (outer, inner).
    Ring { frame: Frame, r_out: f64, r_in: f64 },
}

struct Circle {
    c: V3,
    r: f64,
}

/// Points along a loop, each coedge sampled in its own direction.
fn loop_samples(t: &dyn TopologyView, lp: &[CoedgeKey]) -> Result<Vec<V3>, String> {
    let mut out = Vec::new();
    for k in lp {
        for j in 0..SAMPLES {
            let s = j as f64 / SAMPLES as f64;
            let p = geom::edge_point(t, k.edge, s).map_err(|e| e.to_string())?;
            out.push(arr(p));
        }
    }
    Ok(out)
}

/// Circle through samples spread around a loop, verified on all of them.
fn fit_circle(pts: &[V3], n: V3) -> Result<Circle, String> {
    if pts.len() < 3 {
        return Err("too few samples".into());
    }
    let (a, b, c) = (pts[0], pts[pts.len() / 3], pts[2 * pts.len() / 3]);
    let (u, v) = (sub(b, a), sub(c, a));
    let w = cross(u, v);
    let w2 = dot(w, w);
    if w2 <= 1e-24 * dot(u, u) * dot(v, v) {
        return Err("collinear samples".into());
    }
    // Circumcentre: a + (|u|^2 (v x w) ... ) / (2 |w|^2).
    let (uu, vv) = (dot(u, u), dot(v, v));
    let t1 = cross(w, u);
    let t2 = cross(v, w);
    let c0: V3 = std::array::from_fn(|k| a[k] + (vv * t1[k] + uu * t2[k]) / (2.0 * w2));
    let r = norm(sub(a, c0));
    if !(r.is_finite() && r > 0.0) {
        return Err("degenerate circle".into());
    }
    for p in pts {
        let d = sub(*p, c0);
        if (norm(d) - r).abs() > TOL * r || dot(d, n).abs() > TOL * r {
            return Err("loop is not an exact circle in the face plane".into());
        }
    }
    Ok(Circle { c: c0, r })
}

/// Classify `f`.
pub(crate) fn analyze(t: &dyn TopologyView, f: EntityId) -> Result<Shape, String> {
    let loops = geom::face_loops(t, f);
    if loops.is_empty() || loops.len() > 2 || loops.iter().any(|l| l.is_empty()) {
        return Err(format!("{} boundary loops; the O-grid needs a disk or an annulus", loops.len()));
    }
    let samples: Vec<Vec<V3>> = loops.iter().map(|l| loop_samples(t, l)).collect::<Result<_, _>>()?;
    let n = geom::face_normal(t, f, Point3::new(samples[0][0][0], samples[0][0][1], samples[0][0][2]))
        .map_err(|e| e.to_string())?;
    let n = arr(Point3::new(n.x, n.y, n.z));
    // Planar: the oriented face normal agrees everywhere on the loops.
    for p in samples.iter().flatten() {
        let q = geom::face_normal(t, f, Point3::new(p[0], p[1], p[2])).map_err(|e| e.to_string())?;
        if dot(arr(Point3::new(q.x, q.y, q.z)), n) < 1.0 - 1e-10 {
            return Err("the face is not planar".into());
        }
    }
    let on = |pts: &[V3], o: V3, scale: f64| pts.iter().all(|p| dot(sub(*p, o), n).abs() <= TOL * scale);
    let outer_circle = fit_circle(&samples[0], n);
    let inner = if loops.len() == 2 { Some(fit_circle(&samples[1], n)?) } else { None };
    let frame_at = |c: V3| {
        let x = [1.0, 0.0, 0.0];
        let mut e1 = sub(x, [n[0] * n[0], n[0] * n[1], n[0] * n[2]]);
        if norm(e1) < 1e-6 {
            let y = [0.0, 1.0, 0.0];
            e1 = sub(y, [n[1] * n[0], n[1] * n[1], n[1] * n[2]]);
        }
        let l = norm(e1);
        let e1 = [e1[0] / l, e1[1] / l, e1[2] / l];
        Frame { o: c, e1, e2: cross(n, e1) }
    };
    match (outer_circle, inner) {
        (Ok(c), None) => Ok(Shape::Disk { frame: frame_at(c.c), radius: c.r }),
        (Ok(co), Some(ci)) => {
            if norm(sub(co.c, ci.c)) > TOL * co.r.max(ci.r) {
                return Err("the two circles are not concentric".into());
            }
            if ci.r >= co.r {
                return Err("the hole is not inside the outer circle".into());
            }
            Ok(Shape::Ring { frame: frame_at(co.c), r_out: co.r, r_in: ci.r })
        }
        (Err(_), Some(ci)) => {
            let (half, q0) = square_annulus(t, &loops[0], &samples[0], n, &ci)?;
            let frame = frame_at(ci.c);
            let l = frame.local(q0);
            Ok(Shape::SquareAnnulus { frame, radius: ci.r, half, psi: l[1].atan2(l[0]) })
        }
        (Err(e), None) => Err(e),
    }
    .and_then(|s| {
        // The whole face lies in one plane through the fitted centre.
        let (o, scale) = match &s {
            Shape::Disk { frame, radius } => (frame.o, *radius),
            Shape::SquareAnnulus { frame, half, .. } => (frame.o, *half),
            Shape::Ring { frame, r_out, .. } => (frame.o, *r_out),
        };
        if samples.iter().all(|p| on(p, o, scale)) { Ok(s) } else { Err("the loops are not coplanar".into()) }
    })
}

/// A square outer loop of four distinct straight edges, centred on the hole. Returns the
/// half side and the first corner point.
fn square_annulus(
    t: &dyn TopologyView,
    lp: &[CoedgeKey],
    samples: &[V3],
    n: V3,
    hole: &Circle,
) -> Result<(f64, V3), String> {
    if lp.len() != 4 {
        return Err(format!("outer loop has {} edges; need a circle or four straight edges", lp.len()));
    }
    let mut edges: Vec<EntityId> = lp.iter().map(|k| k.edge).collect();
    edges.sort_unstable();
    edges.dedup();
    if edges.len() != 4 {
        return Err("the outer loop reuses an edge".into());
    }
    let mut q = [[0.0; 3]; 4];
    for (i, k) in lp.iter().enumerate() {
        let (a, _) = geom::coedge_ends(t, *k).ok_or("edge without ends")?;
        q[i] = arr(geom::vertex_point(t, a).map_err(|e| e.to_string())?);
    }
    let o: V3 = std::array::from_fn(|k| (q[0][k] + q[1][k] + q[2][k] + q[3][k]) / 4.0);
    let rc = norm(sub(q[0], o));
    if norm(sub(o, hole.c)) > TOL * rc {
        return Err("the square is not centred on the circle".into());
    }
    for i in 0..4 {
        let (a, b) = (sub(q[i], o), sub(q[(i + 1) % 4], o));
        let ok = (norm(a) - rc).abs() <= TOL * rc && dot(a, b).abs() <= TOL * rc * rc && dot(cross(a, b), n) > 0.0;
        if !ok {
            return Err("the outer loop is not a counter-clockwise square".into());
        }
    }
    // Straight sides: every sample lies on the line through its two corners.
    for (i, chunk) in samples.chunks(SAMPLES).enumerate() {
        let (a, b) = (q[i], q[(i + 1) % 4]);
        let d = sub(b, a);
        let l = norm(d);
        for p in chunk {
            if norm(cross(sub(*p, a), d)) > TOL * l * l {
                return Err("a side of the outer loop is not straight".into());
            }
        }
    }
    let half = rc / std::f64::consts::SQRT_2;
    if half <= hole.r * (1.0 + 1e-6) {
        return Err("the circle does not lie strictly inside the square".into());
    }
    Ok((half, q[0]))
}

/// Angles of loop nodes in `frame`, counter-clockwise about the normal, with the node order
/// reversed when the loop runs clockwise. Returns `(order, angles)`, where `order[i]` is the
/// loop index of the i-th counter-clockwise node. Fails unless the nodes lie on a circle of
/// `radius` and are equally spaced in angle.
pub(crate) fn ccw_circle_nodes(frame: &Frame, pts: &[V3], radius: f64) -> Result<(Vec<usize>, Vec<f64>), String> {
    let n = pts.len();
    if n < 3 {
        return Err(format!("{n} circle nodes"));
    }
    let loc: Vec<[f64; 2]> = pts.iter().map(|p| frame.local(*p)).collect();
    for l in &loc {
        if (l[0].hypot(l[1]) - radius).abs() > 1e-7 * radius {
            return Err("a circle node is off the circle".into());
        }
    }
    let ang: Vec<f64> = loc.iter().map(|l| l[1].atan2(l[0])).collect();
    let step = |a: f64, b: f64| {
        let mut d = (b - a) % TAU;
        if d > PI {
            d -= TAU;
        } else if d < -PI {
            d += TAU;
        }
        d
    };
    let dphi = TAU / n as f64;
    let ccw = step(ang[0], ang[1]) > 0.0;
    let order: Vec<usize> = if ccw { (0..n).collect() } else { (0..n).map(|i| (n - i) % n).collect() };
    let mut out = vec![ang[order[0]]];
    for w in 1..n {
        let d = step(ang[order[w - 1]], ang[order[w]]);
        if (d - dphi).abs() > 1e-6 * dphi.max(1e-3) + 1e-9 {
            return Err("circle nodes are not equally spaced in angle".into());
        }
        out.push(out[w - 1] + d);
    }
    // The loop closes after exactly one turn.
    if (step(ang[order[n - 1]], ang[order[0]]) - dphi).abs() > 1e-6 * dphi.max(1e-3) + 1e-9 {
        return Err("circle nodes do not close after one turn".into());
    }
    Ok((order, out))
}
