//! Shared helpers for the integration tests: prisms built through the reference topology, and a
//! generic "give every boundary face a kind and a color" step.
#![allow(dead_code, missing_docs)]

use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{CurveDef, Frame, Interval, Line, Plane as GPlane, SurfaceDef};
use sm_model_api::{BoundaryKind, Domain, Model};
use sm_topo_api::fixtures::with_op;
use sm_topo_api::*;

pub struct PrismIds {
    pub bottom: EntityId,
    pub top: EntityId,
    pub sides: Vec<EntityId>,
    pub volume: EntityId,
}

/// A convex prism over a counter-clockwise polygon, `z0 <= z <= z1`. Sides[i] is the quad over
/// polygon edge `i -> i+1`. One op must be open.
pub fn prism(t: &mut CellComplex, poly: &[(f64, f64)], z0: f64, z1: f64) -> PrismIds {
    let n = poly.len();
    let pts: Vec<Point3> = poly.iter().map(|p| Point3::new(p.0, p.1, z0)).chain(poly.iter().map(|p| Point3::new(p.0, p.1, z1))).collect();
    let vs: Vec<EntityId> = pts.iter().map(|p| t.make_vertex(*p).unwrap()).collect();
    // (start, end, edge)
    let mut edges: Vec<(usize, usize, EntityId)> = Vec::new();
    let mk = |t: &mut CellComplex, a: usize, b: usize, edges: &mut Vec<(usize, usize, EntityId)>| {
        let d = pts[b] - pts[a];
        let len = d.norm();
        let curve = t.shapes_mut().unwrap().add_curve(CurveDef::Line(Line::new(pts[a], d * (1.0 / len)).unwrap())).unwrap();
        let e = t.make_edge(vs[a], vs[b], Some(CurveCarrier { curve, range: Interval::new(0.0, len), sense: Sense::Forward })).unwrap();
        edges.push((a, b, e));
    };
    for i in 0..n {
        mk(t, i, (i + 1) % n, &mut edges);
    }
    for i in 0..n {
        mk(t, n + i, n + (i + 1) % n, &mut edges);
    }
    for i in 0..n {
        mk(t, i, n + i, &mut edges);
    }
    let coedge = |a: usize, b: usize| -> CoedgeSpec {
        match edges.iter().find(|e| e.0 == a && e.1 == b) {
            Some(e) => CoedgeSpec { edge: e.2, sense: Sense::Forward },
            None => CoedgeSpec { edge: edges.iter().find(|e| e.0 == b && e.1 == a).unwrap().2, sense: Sense::Reversed },
        }
    };
    let face = |t: &mut CellComplex, cyc: &[usize], normal: Vector3| -> EntityId {
        let l: Vec<CoedgeSpec> = (0..cyc.len()).map(|i| coedge(cyc[i], cyc[(i + 1) % cyc.len()])).collect();
        let frame = Frame::from_z(pts[cyc[0]], normal).unwrap();
        let surface = t.shapes_mut().unwrap().add_surface(SurfaceDef::Plane(GPlane { frame })).unwrap();
        t.make_face(&[&l], Some(SurfaceCarrier { surface, sense: Sense::Forward })).unwrap()
    };
    let bottom_cyc: Vec<usize> = (0..n).rev().collect();
    let top_cyc: Vec<usize> = (n..2 * n).collect();
    let bottom = face(t, &bottom_cyc, Vector3::new(0.0, 0.0, -1.0));
    let top = face(t, &top_cyc, Vector3::new(0.0, 0.0, 1.0));
    let mut sides = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        let e = pts[j] - pts[i];
        let out = Vector3::new(e.y, -e.x, 0.0);
        sides.push(face(t, &[i, j, n + j, n + i], out));
    }
    let all: Vec<EntityId> = [bottom, top].into_iter().chain(sides.iter().copied()).collect();
    let uses: Vec<FaceUseSpec> = all.iter().map(|&f| FaceUseSpec { face: f, side: Side::Left }).collect();
    let volume = t.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &uses }]).unwrap();
    t.make_body(&[volume]).unwrap();
    PrismIds { bottom, top, sides, volume }
}

/// Build a prism in its own operation.
pub fn prism_op(t: &mut CellComplex, op: u64, poly: &[(f64, f64)], z0: f64, z1: f64) -> PrismIds {
    with_op(t, OpId(op), |t| Ok(prism(t, poly, z0, z1))).unwrap()
}

/// A regular polygon with `n` corners, circumradius `r`, first corner at angle `phase`.
pub fn regular_polygon(n: usize, r: f64, phase: f64) -> Vec<(f64, f64)> {
    (0..n)
        .map(|i| {
            let a = phase + core::f64::consts::TAU * i as f64 / n as f64;
            (r * a.cos(), r * a.sin())
        })
        .collect()
}

/// Add a response record "outer" and put every boundary face without a kind on it, coloring the
/// uncolored ones with a fresh face color (`label`) unless `color` is false.
pub fn finish_boundaries(m: &mut Model, t: &CellComplex, label: u32, color: bool) {
    let free: Vec<EntityId> = t.boundary_faces().filter(|f| m.face_boundary(*f).is_none()).collect();
    if free.is_empty() {
        return;
    }
    let name = format!("outer-{label}");
    let b = m.add_boundary(&name, BoundaryKind::Response).unwrap();
    m.assign_response(b, &free).unwrap();
    if color {
        let c = m.add_color(label, &format!("bc-{label}"), Domain::Face).unwrap();
        let uncolored: Vec<EntityId> = free.into_iter().filter(|f| m.face_color(*f).is_none()).collect();
        m.color_faces(c, &uncolored).unwrap();
    }
}
