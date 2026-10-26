//! Edge correspondence of surface copies: edges that share both ends (arc and chord of a
//! half-disk, a closed circle) are told apart geometrically, and closed curves take their
//! direction from the transformed tangent.

use core::f64::consts::PI;
use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_geom_api::{Circle, CurveDef, Frame, Interval, Line, Plane, SurfaceDef};
use sm_scheme_api::copy::{derive_vertex_map, edge_map};
use sm_scheme_api::{CopyConstraint, CopyReason, Target};
use sm_topo_api::fixtures::{cylinder, with_op};
use sm_topo_api::{
    CellComplex, CoedgeSpec, CurveCarrier, OpId, Sense, SurfaceCarrier, TopologyEdit,
};

struct Half {
    face: EntityId,
    arc: EntityId,
    line: EntityId,
}

/// A half disk of radius 1 at height `z`: arc from (1,0,z) to (-1,0,z) and the diameter back.
/// `line_first` creates the diameter before the arc (lower edge id).
fn half_disk(t: &mut CellComplex, z: f64, line_first: bool) -> Half {
    let a = t.make_vertex(Point3::new(1.0, 0.0, z)).expect("a");
    let b = t.make_vertex(Point3::new(-1.0, 0.0, z)).expect("b");
    let mk_line = |t: &mut CellComplex| {
        let curve = t
            .shapes_mut()
            .expect("shapes")
            .add_curve(CurveDef::Line(
                Line::new(Point3::new(1.0, 0.0, z), Vector3::new(-1.0, 0.0, 0.0)).expect("line"),
            ))
            .expect("curve");
        t.make_edge(
            a,
            b,
            Some(CurveCarrier {
                curve,
                range: Interval::new(0.0, 2.0),
                sense: Sense::Forward,
            }),
        )
        .expect("line edge")
    };
    let mk_arc = |t: &mut CellComplex| {
        let frame = Frame::from_zx(
            Point3::new(0.0, 0.0, z),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(1.0, 0.0, 0.0),
        )
        .expect("frame");
        let curve = t
            .shapes_mut()
            .expect("shapes")
            .add_curve(CurveDef::Circle(Circle::new(frame, 1.0).expect("circle")))
            .expect("curve");
        t.make_edge(
            a,
            b,
            Some(CurveCarrier {
                curve,
                range: Interval::new(0.0, PI),
                sense: Sense::Forward,
            }),
        )
        .expect("arc edge")
    };
    let (line, arc) = if line_first {
        let l = mk_line(t);
        (l, mk_arc(t))
    } else {
        let a = mk_arc(t);
        (mk_line(t), a)
    };
    let frame =
        Frame::from_z(Point3::new(0.0, 0.0, z), Vector3::new(0.0, 0.0, 1.0)).expect("frame");
    let surface = t
        .shapes_mut()
        .expect("shapes")
        .add_surface(SurfaceDef::Plane(Plane { frame }))
        .expect("surface");
    let lp = [
        CoedgeSpec {
            edge: arc,
            sense: Sense::Forward,
        },
        CoedgeSpec {
            edge: line,
            sense: Sense::Reversed,
        },
    ];
    let face = t
        .make_face(
            &[&lp],
            Some(SurfaceCarrier {
                surface,
                sense: Sense::Forward,
            }),
        )
        .expect("face");
    Half { face, arc, line }
}

fn copy_of(source: EntityId, target: EntityId, transform: Transform) -> CopyConstraint {
    CopyConstraint::new(
        Target::Surface(source),
        Target::Surface(target),
        transform,
        CopyReason::User,
    )
}

#[test]
fn edges_with_the_same_ends_map_by_geometry() {
    for line_first in [false, true] {
        let mut c = CellComplex::new();
        let (src, dst) = with_op(&mut c, OpId(1), |t| {
            Ok((half_disk(t, 0.0, false), half_disk(t, 1.0, line_first)))
        })
        .expect("faces");
        let mut cc = copy_of(
            src.face,
            dst.face,
            Transform::translation(Vector3::new(0.0, 0.0, 1.0)),
        );
        cc.vertex_map = derive_vertex_map(&c, &cc).expect("vertex map");
        let m = edge_map(&c, &cc).expect("edge map");
        assert!(m.contains(&(src.arc, dst.arc, true)), "arc to arc: {m:?}");
        assert!(
            m.contains(&(src.line, dst.line, true)),
            "diameter to diameter: {m:?}"
        );
        assert_eq!(m.len(), 2);
    }
}

/// A triangle at height `z` with the diameter of `half_disk` as one side and a third vertex at
/// (0, -1, z).
fn triangle(t: &mut CellComplex, z: f64) -> EntityId {
    let pts = [
        Point3::new(1.0, 0.0, z),
        Point3::new(-1.0, 0.0, z),
        Point3::new(0.0, -1.0, z),
    ];
    let vs: Vec<EntityId> = pts
        .iter()
        .map(|p| t.make_vertex(*p).expect("vertex"))
        .collect();
    let mut lp = Vec::new();
    for i in 0..3 {
        let (p, q) = (pts[i], pts[(i + 1) % 3]);
        let curve = t
            .shapes_mut()
            .expect("shapes")
            .add_curve(CurveDef::Line(Line::new(p, q - p).expect("line")))
            .expect("curve");
        let e = t
            .make_edge(
                vs[i],
                vs[(i + 1) % 3],
                Some(CurveCarrier {
                    curve,
                    range: Interval::new(0.0, (q - p).norm()),
                    sense: Sense::Forward,
                }),
            )
            .expect("edge");
        lp.push(CoedgeSpec {
            edge: e,
            sense: Sense::Forward,
        });
    }
    let frame =
        Frame::from_z(Point3::new(0.0, 0.0, z), Vector3::new(0.0, 0.0, -1.0)).expect("frame");
    let surface = t
        .shapes_mut()
        .expect("shapes")
        .add_surface(SurfaceDef::Plane(Plane { frame }))
        .expect("surface");
    t.make_face(
        &[&lp],
        Some(SurfaceCarrier {
            surface,
            sense: Sense::Forward,
        }),
    )
    .expect("face")
}

#[test]
fn two_source_edges_with_one_image_are_a_correspondence_error() {
    // The arc and the diameter of the half disk share their ends; the triangle has a single
    // edge between the images of those ends, so both would map onto it.
    let mut c = CellComplex::new();
    let (src, dst) = with_op(&mut c, OpId(1), |t| {
        Ok((half_disk(t, 0.0, false), triangle(t, 1.0)))
    })
    .expect("faces");
    let mut cc = copy_of(
        src.face,
        dst,
        Transform::translation(Vector3::new(0.0, 0.0, 1.0)),
    );
    cc.vertex_map = derive_vertex_map(&c, &cc).expect("vertex map");
    match edge_map(&c, &cc) {
        Err(sm_scheme_api::SchemeError::Correspondence { reason, .. }) => {
            assert!(reason.contains("two source edges map to"), "{reason}")
        }
        other => panic!("expected a correspondence error, got {other:?}"),
    }
}

#[test]
fn closed_curve_direction_follows_the_transformed_tangent() {
    let mut c = CellComplex::new();
    let cyl = with_op(&mut c, OpId(1), |t| cylinder(t, 1.0, 2.0)).expect("cylinder");
    let (bot, top) = (cyl.faces[0], cyl.faces[1]);
    let cases = [
        (
            Transform {
                m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                t: Vector3::new(0.0, 0.0, 2.0),
            },
            true,
        ),
        (
            Transform {
                m: [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]],
                t: Vector3::new(0.0, 0.0, 2.0),
            },
            false,
        ),
    ];
    for (g, expect_same) in cases {
        let mut cc = copy_of(bot, top, g);
        cc.vertex_map = derive_vertex_map(&c, &cc).expect("vertex map");
        let m = edge_map(&c, &cc).expect("edge map");
        assert_eq!(m, vec![(cyl.edges[0], cyl.edges[1], expect_same)]);
    }
}
