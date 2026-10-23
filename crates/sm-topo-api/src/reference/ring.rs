//! Radial ring construction (reference algorithm).
//!
//! Around an edge with unit tangent `T`, every coedge contributes two edge uses (one per side of
//! its face) at the angle of the face's inward direction `n x t_coedge`. Angles are measured
//! counter-clockwise about `T` from the canonically smallest coedge. Within one face the two sides
//! are ordered by which one a counter-clockwise sweep meets first: the sweep direction at the face
//! is `r = T x inward`; if `n . r > 0` the sweep passes from the Negative side to the Positive side.
//! Wire uses are placed inside the wedge of their volume.

use super::complex::RefComplex;
use crate::ids::*;
use crate::model::*;
use crate::view::TopologyView;
use core::f64::consts::TAU;
use sm_core::{Point3, Vector3};
use sm_geom_api::vecs::unit;
use sm_geom_api::ShapeStore;

/// Face normal (including the face's surface sense) near point `p`, if it can be computed.
pub(crate) fn face_normal_at(shapes: &dyn ShapeStore, tol: &sm_geom_api::ModelTolerance, face: &FaceRec, p: Point3) -> Option<Vector3> {
    let n = if let Some(s) = shapes.surface(face.surface) {
        let pr = s.project(p, None, tol).ok()?;
        s.normal(pr.uv[0], pr.uv[1]).ok()?
    } else {
        let g = shapes.implicit(face.surface)?.gradient(p);
        unit(g)?
    };
    Some(n * face.surface_sense.sign())
}

struct Group {
    coedge: CoedgeRef,
    key: i64,
    first: FaceSide,
}

/// Build the ring of `e`.
pub(crate) fn build_ring(c: &RefComplex, e: EdgeId) -> Vec<EdgeUse> {
    let coedges = c.edge_coedges(e).to_vec();
    let mut groups: Vec<Group> = Vec::new();
    let edge = &c.edges_raw()[e.serial() as usize];
    let frame = c.shapes().curve(edge.curve).and_then(|cv| {
        let tm = 0.5 * (edge.t0 + edge.t1);
        Some((unit(cv.d1(tm))?, cv.eval(tm)))
    });
    if let Some((t, p)) = frame {
        let mut inward = Vec::new();
        for &cr in &coedges {
            let face = &c.faces_raw()[cr.face.serial() as usize];
            let sense = match face.loops[usize::from(cr.loop_index)].coedges[cr.index as usize].kind {
                CoedgeKind::Edge { sense, .. } => sense,
                CoedgeKind::Pole { .. } => continue,
            };
            let n = face_normal_at(c.shapes(), c.tolerance(), face, p);
            let dir = n.and_then(|n| unit(n.cross(t * sense.sign())).map(|i| (n, i)));
            inward.push((cr, dir));
        }
        let reference = inward.iter().find_map(|(_, d)| d.map(|(_, i)| i));
        if let Some(u0) = reference {
            let w0 = t.cross(u0);
            for (cr, dir) in inward {
                let (angle, first) = match dir {
                    Some((n, i)) => {
                        let mut a = i.dot(w0).atan2(i.dot(u0));
                        if a < 0.0 {
                            a += TAU;
                        }
                        if a > TAU - 1e-9 {
                            a = 0.0;
                        }
                        let r = t.cross(i);
                        (a, if n.dot(r) >= 0.0 { FaceSide::Negative } else { FaceSide::Positive })
                    }
                    None => (0.0, FaceSide::Negative),
                };
                groups.push(Group { coedge: cr, key: (angle / 1e-9).round() as i64, first });
            }
            groups.sort_by_key(|g| g.key); // stable: ties keep canonical coedge order
        }
    }
    if groups.is_empty() {
        groups = coedges.iter().map(|&cr| Group { coedge: cr, key: 0, first: FaceSide::Negative }).collect();
    }
    let mut ring: Vec<EdgeUse> = groups.iter().flat_map(|g| [g.first, g.first.opposite()].map(|side| EdgeUse::Coedge { coedge: g.coedge, side })).collect();

    // wires, ascending by owner: placed after the first use whose following wedge is their volume
    let mut wires: Vec<Option<VolumeId>> = Vec::new();
    for &vid in c.volumes() {
        if let Some(v) = c.volume(vid) {
            for s in &v.shells {
                wires.extend(s.wires.iter().filter(|&&w| w == e).map(|_| Some(vid)));
            }
        }
    }
    for &bid in c.bodies() {
        if let Some(b) = c.body(bid) {
            wires.extend(b.wires.iter().filter(|&&w| w == e).map(|_| None));
        }
    }
    for w in wires {
        let eu = EdgeUse::Wire { edge: e, volume: w };
        let n = ring.len();
        let pos = (0..n).find(|&i| {
            let mates = matches!((ring[i], ring[(i + 1) % n]), (EdgeUse::Coedge { coedge: a, .. }, EdgeUse::Coedge { coedge: b, .. }) if a == b);
            !mates && c.edge_use_side(ring[i]) == Some(w.map_or(Side::Exterior, Side::Volume))
        });
        match pos {
            Some(i) => ring.insert(i + 1, eu),
            None => ring.push(eu),
        }
    }
    ring
}
