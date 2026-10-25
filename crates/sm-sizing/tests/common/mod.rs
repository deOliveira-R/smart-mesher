//! Shared fixtures for the sm-sizing integration tests.
#![allow(dead_code)]

use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_geom_api::{CurveDef, Frame, Interval, Line, Plane, SurfaceDef};
use sm_scheme_api::{ConstraintId, IntervalConstraint, Origin, Record};
use sm_topo_api::fixtures::{BrickIds, brick, with_op};
use sm_topo_api::{
    CellComplex, CoedgeSpec, CurveCarrier, OpId, Sense, SurfaceCarrier, TopologyEdit,
};

/// A topological edge id.
pub fn edge(n: u64) -> EntityId {
    EntityId::new(EntityKind::TopoEdge, n).expect("edge id")
}

/// Constraint records numbered in list order.
pub fn records(items: Vec<IntervalConstraint>) -> Vec<Record<IntervalConstraint>> {
    items
        .into_iter()
        .enumerate()
        .map(|(i, item)| Record { id: ConstraintId(i as u32), origin: Origin::Driver, item })
        .collect()
}

/// A unit brick whose bottom x-edge (`edges[0]`) has a vertex in its middle.
pub struct SplitBrick {
    pub topo: CellComplex,
    pub ids: BrickIds,
    pub mid: EntityId,
}

pub fn split_brick() -> SplitBrick {
    let mut topo = CellComplex::new();
    let (ids, mid) = with_op(&mut topo, OpId(1), |t| {
        let b = brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])?;
        let m = t.make_vertex(Point3::new(0.5, 0.0, 0.0))?;
        t.split_edge(b.edges[0], m, 0.5)?;
        Ok((b, m))
    })
    .expect("fixture");
    SplitBrick { topo, ids, mid }
}

/// A standalone L-shaped planar face `[0,2]x[0,1] U [0,1]x[0,2]`, counter-clockwise, normal +z.
pub struct LFace {
    pub topo: CellComplex,
    pub face: EntityId,
    pub vertices: Vec<EntityId>,
}

pub fn l_face() -> LFace {
    let pts = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0), (1.0, 2.0), (0.0, 2.0)];
    let mut topo = CellComplex::new();
    let (face, vertices) = with_op(&mut topo, OpId(1), |t| {
        let vs: Vec<EntityId> = pts
            .iter()
            .map(|&(x, y)| t.make_vertex(Point3::new(x, y, 0.0)))
            .collect::<Result<_, _>>()?;
        let mut loop_: Vec<CoedgeSpec> = Vec::new();
        for i in 0..pts.len() {
            let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
            let d = Vector3::new(b.0 - a.0, b.1 - a.1, 0.0);
            let curve = t.shapes_mut()?.add_curve(CurveDef::Line(Line::new(Point3::new(a.0, a.1, 0.0), d)?))?;
            let e = t.make_edge(
                vs[i],
                vs[(i + 1) % pts.len()],
                Some(CurveCarrier { curve, range: Interval::new(0.0, d.norm()), sense: Sense::Forward }),
            )?;
            loop_.push(CoedgeSpec { edge: e, sense: Sense::Forward });
        }
        let frame = Frame::from_z(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0))?;
        let surface = t.shapes_mut()?.add_surface(SurfaceDef::Plane(Plane { frame }))?;
        let face = t.make_face(&[&loop_], Some(SurfaceCarrier { surface, sense: Sense::Forward }))?;
        Ok((face, vs))
    })
    .expect("l face");
    LFace { topo, face, vertices }
}
