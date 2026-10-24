//! The scaling workload: refine the top face of a brick by repeated splits (edge subdivision plus a
//! chord), breadth first, so faces stay small and one volume ends up with `n + 6` faces.
#![allow(dead_code)]

use sm_core::{EntityId, Point3};
use sm_geom_api::{CurveDef, Interval, Line};
use sm_topo_api::fixtures::{self, BrickIds};
use sm_topo_api::{Carrier, CurveCarrier, FaceCut, InnerLoopAssign, OpId, Sense, Side, TopoError, TopologyEdit, TopologyViewExt};
use std::collections::VecDeque;

fn pos(t: &dyn TopologyEdit, v: EntityId) -> Result<Point3, TopoError> {
    match t.carrier(v) {
        Carrier::Point(p) => Ok(*p),
        _ => Err(TopoError::InvalidParameter("vertex without point")),
    }
}

fn axis(p: Point3, a: usize) -> f64 {
    [p.x, p.y, p.z][a]
}

fn lerp(p: Point3, q: Point3, s: f64) -> Point3 {
    Point3::new(p.x + s * (q.x - p.x), p.y + s * (q.y - p.y), p.z + s * (q.z - p.z))
}

/// Split the (axis-aligned rectangular) face `f` in two through its middle.
pub fn split_rect(t: &mut dyn TopologyEdit, f: EntityId) -> Result<[EntityId; 2], TopoError> {
    // Outer loop corners with their coedges.
    let entries: Vec<_> = t.loops(f).next().ok_or(TopoError::EmptyLoop)?.entries.to_vec();
    let mut corners = Vec::with_capacity(entries.len());
    for e in &entries {
        let (s, h) = t.edge_ends(e.cell).ok_or(TopoError::InvalidParameter("edge"))?;
        let (tail, head) = if e.side == Side::Right { (h, s) } else { (s, h) };
        corners.push((e.cell, tail, head));
    }
    let pts: Vec<Point3> = corners.iter().map(|c| pos(t, c.1)).collect::<Result<_, _>>()?;
    let ext = |a: usize| {
        let lo = pts.iter().map(|p| axis(*p, a)).fold(f64::INFINITY, f64::min);
        let hi = pts.iter().map(|p| axis(*p, a)).fold(f64::NEG_INFINITY, f64::max);
        (lo, hi)
    };
    let (xr, yr) = (ext(0), ext(1));
    let a = if xr.1 - xr.0 >= yr.1 - yr.0 { 0 } else { 1 };
    let (lo, hi) = ext(a);
    let c = lo + 0.5 * (hi - lo);
    // The two boundary points on the line axis = c: existing vertices or edges to split.
    let mut ends: Vec<EntityId> = Vec::new();
    let mut crossing: Vec<(usize, EntityId)> = Vec::new();
    // Vertices within `eps` of the cut line are the cut's ends (rounding makes "exactly on" unreliable).
    let eps = 1e-9 * (hi - lo);
    for (i, &(edge, tail, _head)) in corners.iter().enumerate() {
        let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
        let (pa, qa) = (axis(p, a), axis(q, a));
        if (pa - c).abs() <= eps {
            ends.push(tail);
        } else if (pa < c - eps && qa > c + eps) || (pa > c + eps && qa < c - eps) {
            crossing.push((i, edge));
        }
    }
    if ends.len() + crossing.len() != 2 {
        return Err(TopoError::InvalidParameter("rectangle cut does not meet the boundary twice"));
    }
    for (i, edge) in crossing {
        let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
        let s = (c - axis(p, a)) / (axis(q, a) - axis(p, a));
        let pt = lerp(p, q, s);
        let (es, _) = t.edge_ends(edge).ok_or(TopoError::InvalidParameter("edge"))?;
        let Carrier::Curve(cc) = t.carrier(edge).clone() else { return Err(TopoError::InvalidParameter("edge without curve")) };
        let param = cc.range.lo + (pt - pos(t, es)?).norm();
        let v = t.make_vertex(pt)?;
        t.split_edge(edge, v, param)?;
        ends.push(v);
    }
    let (pa, pb) = (pos(t, ends[0])?, pos(t, ends[1])?);
    let d = pb - pa;
    let curve = t.shapes_mut()?.add_curve(CurveDef::Line(Line::new(pa, d)?))?;
    let chord = t.make_edge(ends[0], ends[1], Some(CurveCarrier { curve, range: Interval::new(0.0, d.norm()), sense: Sense::Forward }))?;
    t.split_face(f, FaceCut::Chord { edge: chord }, InnerLoopAssign::None)
}

/// Build the unit brick (op 1) and return its ids.
pub fn setup(t: &mut dyn TopologyEdit) -> BrickIds {
    setup_at(t, [0.0; 3], [1.0; 3])
}

/// Build the brick `[min, min + size]` (op 1) and return its ids.
pub fn setup_at(t: &mut dyn TopologyEdit, min: [f64; 3], size: [f64; 3]) -> BrickIds {
    t.begin_op(OpId(1)).expect("begin");
    let b = fixtures::brick(t, Point3::new(min[0], min[1], min[2]), size).expect("brick");
    t.end_op().expect("end");
    b
}

/// Refine the top face with `n` splits. `per_op`: one operation per split (otherwise one for all).
/// Returns the live top-face pieces.
pub fn refine(t: &mut dyn TopologyEdit, b: &BrickIds, n: usize, per_op: bool) -> Vec<EntityId> {
    let mut queue: VecDeque<EntityId> = VecDeque::from([b.faces[5]]);
    let mut op = 2u64;
    if !per_op {
        t.begin_op(OpId(op)).expect("begin");
    }
    for _ in 0..n {
        let f = queue.pop_front().expect("a face to split");
        if per_op {
            t.begin_op(OpId(op)).expect("begin");
            op += 1;
        }
        let kids = split_rect(t, f).expect("split");
        if per_op {
            t.end_op().expect("end");
        }
        queue.extend(kids);
    }
    if !per_op {
        t.end_op().expect("end");
    }
    queue.into_iter().collect()
}
