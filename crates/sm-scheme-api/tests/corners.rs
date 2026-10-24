//! Vertex types, corner picking and submap side directions on a brick whose bottom x-edge has
//! a vertex in its middle (so two faces have a 180-degree corner).

mod stub_schemes;

use core::f64::consts::PI;
use sm_core::{EntityId, Point3};
use sm_model_api::Model;
use sm_scheme_api::geom;
use sm_scheme_api::{
    Applicability, Caps, GenCtx, GenerateFailure, Params, Patch, PatchBuilder, PlanCtx, Scheme,
    SchemeDim, Sequential,
};
use sm_scheme_api::{
    CornerIntent, Driver, Firm, Firmness, MeshInput, MeshIntent, ReferenceCornerPicker,
    ReferenceIntervalSolver, SchemeChoice, SchemeError, SideDir, Target, VertexRule, VertexType,
    WholeFaces,
};
use sm_topo_api::fixtures::{BrickIds, brick, with_op};
use sm_topo_api::{CellComplex, CornerKey, OpId, TopologyEdit};
use stub_schemes::registry;

/// Volume stand-in that generates an empty patch, so face meshes can be inspected.
struct NoCells;

impl Scheme for NoCells {
    fn name(&self) -> &str {
        "nocells"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::TET
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(1, "test")
    }
    fn generate(&self, _cx: &GenCtx<'_>, t: Target, _p: &Params) -> Result<Patch, GenerateFailure> {
        Ok(PatchBuilder::new(t).finish())
    }
}

struct Fixture {
    topo: CellComplex,
    ids: BrickIds,
    mid: EntityId,
}

fn split_brick() -> Fixture {
    let mut topo = CellComplex::new();
    let (ids, mid) = with_op(&mut topo, OpId(1), |t| {
        let b = brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])?;
        let m = t.make_vertex(Point3::new(0.5, 0.0, 0.0))?;
        t.split_edge(b.edges[0], m, 0.5)?;
        Ok((b, m))
    })
    .expect("fixture");
    Fixture { topo, ids, mid }
}

fn corner_at(f: &Fixture, face: EntityId, v: EntityId) -> CornerKey {
    geom::corners(&f.topo, face)
        .expect("corners")
        .into_iter()
        .find(|c| c.vertex == v)
        .expect("corner")
        .key
}

#[test]
fn map_picks_vertex_types_per_face_use() {
    let f = split_brick();
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let model = Model::new();
    let sel = [f.ids.volume];
    let input = MeshInput {
        topo: &f.topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let mut intent = MeshIntent::default();
    intent.set_scheme(f.ids.volume, SchemeChoice::new("mapvol", Firmness::Hard));
    let plan = driver.plan(&input, &intent).expect("plan");
    // The middle vertex is a Side on the two faces that use the split edge and does not exist
    // on the others: the type belongs to the face use, not to the vertex.
    let face = f.ids.faces[4];
    let layout = plan.layout(face).expect("layout");
    assert_eq!(layout.rule, VertexRule::Map);
    assert_eq!(layout.corners.len(), 5);
    let ty = |l: &sm_scheme_api::FaceLayout, v| {
        l.corners
            .iter()
            .find(|c| c.corner == corner_at(&f, l.face, v))
            .map(|c| (c.vertex_type, c.firmness))
    };
    assert_eq!(
        ty(layout, f.mid),
        Some((VertexType::Side, Firmness::Default))
    );
    assert_eq!(
        ty(layout, f.ids.vertices[0]),
        Some((VertexType::End, Firmness::Default))
    );
    assert_eq!(
        layout
            .corners
            .iter()
            .filter(|c| c.vertex_type == VertexType::End)
            .count(),
        4
    );
    for c in &layout.corners {
        assert!(
            (c.angle - PI / 2.0).abs() < 1e-9 || (c.angle - PI).abs() < 1e-9,
            "angle {}",
            c.angle
        );
    }
    // Four logical sides; the one holding the middle vertex has two coedges.
    assert_eq!(layout.sides.len(), 4);
    assert_eq!(
        layout
            .sides
            .iter()
            .map(|s| s.coedges.len())
            .collect::<Vec<_>>()
            .iter()
            .filter(|n| **n == 2)
            .count(),
        1
    );
    // The side holding the split edge pairs with the opposite side: its two halves sum to the top edge.
    let other = plan.layout(f.ids.faces[5]).expect("+z layout");
    assert_eq!(other.corners.len(), 4);
    assert!(
        other
            .corners
            .iter()
            .all(|c| c.vertex_type == VertexType::End)
    );
    // Opposite sides carry equal counts (the submap-style sum for the split side).
    let count = |s: &sm_scheme_api::LogicalSide| {
        s.coedges
            .iter()
            .map(|c| plan.intervals(c.edge).expect("n"))
            .sum::<u32>()
    };
    assert_eq!(count(&layout.sides[0]), count(&layout.sides[2]));
    assert_eq!(count(&layout.sides[1]), count(&layout.sides[3]));
    assert_eq!(
        layout.sides.iter().map(|s| s.dir).collect::<Vec<_>>(),
        vec![
            SideDir::PlusI,
            SideDir::PlusJ,
            SideDir::MinusI,
            SideDir::MinusJ
        ]
    );
    let _ = Target::Surface(face);
}

#[test]
fn submap_honours_hard_corner_overrides_and_reports_a_bad_sum() {
    let f = split_brick();
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let model = Model::new();
    let sel = [f.ids.volume];
    let input = MeshInput {
        topo: &f.topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let face = f.ids.faces[4];
    // Only this face constrains its edges: the others are triangulated, the volume imposes nothing.
    let mut intent = MeshIntent::default();
    intent.set_scheme(f.ids.volume, SchemeChoice::new("tetvol", Firmness::Hard));
    for other in f.ids.faces.iter().filter(|x| **x != face) {
        intent.set_scheme(*other, SchemeChoice::new("tri", Firmness::Hard));
    }
    intent.set_scheme(face, SchemeChoice::new("submap", Firmness::Hard));
    // Default types: sum 4 (four Ends, one Side).
    let plan = driver.plan(&input, &intent).expect("plan");
    let base = plan.layout(face).expect("layout");
    assert_eq!(base.rule, VertexRule::Submap);
    assert_eq!(
        base.corners
            .iter()
            .map(|c| c.vertex_type.submap_weight())
            .sum::<i32>(),
        4
    );

    // Override: the geometric corner at the origin becomes a Side and the middle vertex an End.
    let x = corner_at(&f, face, f.ids.vertices[0]);
    let m = corner_at(&f, face, f.mid);
    intent.corners.push(CornerIntent {
        corner: x,
        vertex_type: Firm::hard(VertexType::Side),
    });
    intent.corners.push(CornerIntent {
        corner: m,
        vertex_type: Firm::hard(VertexType::End),
    });
    let plan = driver.plan(&input, &intent).expect("plan with overrides");
    let layout = plan.layout(face).expect("layout");
    let get = |k: CornerKey| {
        layout
            .corners
            .iter()
            .find(|c| c.corner == k)
            .map(|c| (c.vertex_type, c.firmness))
    };
    assert_eq!(get(x), Some((VertexType::Side, Firmness::Hard)));
    assert_eq!(get(m), Some((VertexType::End, Firmness::Hard)));
    assert_eq!(layout.sides.len(), 4);
    assert_eq!(
        layout.sides.iter().map(|s| s.dir).collect::<Vec<_>>(),
        vec![
            SideDir::PlusI,
            SideDir::PlusJ,
            SideDir::MinusI,
            SideDir::MinusJ
        ]
    );
    // The straight-through corner now sits inside a logical side with two coedges.
    assert!(layout.sides.iter().any(|s| s.coedges.len() == 2
        && s.coedges.iter().any(|c| {
            geom::coedge_ends(&f.topo, *c)
                .is_some_and(|(a, b)| a == f.ids.vertices[0] || b == f.ids.vertices[0])
        })));
    // The submap direction sums hold for the solved intervals.
    let sum = |d: SideDir| -> u32 {
        layout
            .sides
            .iter()
            .filter(|s| s.dir == d)
            .flat_map(|s| s.coedges.iter())
            .map(|c| plan.intervals(c.edge).expect("n"))
            .sum()
    };
    assert_eq!(sum(SideDir::PlusI), sum(SideDir::MinusI));
    assert_eq!(sum(SideDir::PlusJ), sum(SideDir::MinusJ));

    // The face with a straight-through corner and overridden vertex types actually
    // generates: an all-quad block mesh over the four logical sides (submap attaches no grid,
    // it is BLOCK_STRUCTURED), the straight-through vertex among the boundary nodes, and every
    // strip walks across.
    let mut reg2 = registry();
    reg2.register(Box::new(NoCells)).expect("nocells");
    let driver2 = Driver::new(&reg2, &solver, &picker);
    let mut gen_intent = intent.clone();
    gen_intent.set_scheme(f.ids.volume, SchemeChoice::new("nocells", Firmness::Hard));
    let gplan = driver2.plan(&input, &gen_intent).expect("plan");
    let ps = driver2
        .generate(&input, &gplan, &Sequential)
        .expect("generate with corner overrides");
    let patch = ps.get(Target::Surface(face)).expect("face patch");
    assert!(patch.grid.is_none(), "submap promises blocks, not a grid");
    let glayout = gplan.layout(face).expect("layout");
    let gsum = |d: SideDir| -> u32 {
        glayout
            .sides
            .iter()
            .filter(|s| s.dir == d)
            .flat_map(|s| s.coedges.iter())
            .map(|c| gplan.intervals(c.edge).expect("n"))
            .sum()
    };
    let (ni, nj) = (gsum(SideDir::PlusI), gsum(SideDir::PlusJ));
    assert_eq!(patch.elements.len(), (ni * nj) as usize);
    assert!(patch.elements.iter().all(|(s, _)| s == sm_scheme_api::ElemShape::Quad));
    let mid = sm_scheme_api::NodeKey::vertex(f.mid);
    assert!(patch.refs.contains(&mid), "the straight-through vertex is a boundary node");
    // Every non-corner boundary node starts a strip across the block; the four corners none.
    let mut starts = 0;
    for k in &patch.refs {
        if let Some(strip) = patch.strip_from(*k) {
            starts += 1;
            assert!(
                strip.len() == ni as usize + 1 || strip.len() == nj as usize + 1,
                "{} nodes for a {ni} x {nj} block",
                strip.len()
            );
            assert_eq!(strip.first(), Some(k));
            assert!(patch.refs.contains(strip.last().expect("end")), "ends on the boundary");
        }
    }
    assert_eq!(starts, 2 * (ni - 1) + 2 * (nj - 1));

    // One override alone breaks the corner sum (Ends 3, Side 2 -> 3): infeasible, naming the
    // request and the rule. The face's scheme is a hard intent, so nothing is blamed.
    intent.corners.truncate(1);
    match driver.plan(&input, &intent) {
        Err(SchemeError::Infeasible(inf)) => {
            assert!(
                inf.reason.contains("submap corner sum is 3"),
                "{}",
                inf.reason
            );
            assert!(inf.entities.contains(&face));
            assert!(inf.constraints.len() >= 2, "rule and request: {inf:?}");
            assert_eq!(inf.blame, None);
        }
        other => panic!("expected Infeasible, got {:?}", other.map(|_| ())),
    }
}

#[test]
fn vertex_type_tables_and_side_directions() {
    assert_eq!(VertexType::ALL.map(VertexType::element_count), [1, 2, 3, 4]);
    assert_eq!(
        VertexType::ALL.map(VertexType::submap_weight),
        [1, 0, -1, -2]
    );
    assert_eq!(VertexType::nearest(PI / 2.0 + 0.1), VertexType::End);
    assert_eq!(VertexType::nearest(PI + 0.1), VertexType::Side);
    assert_eq!(VertexType::nearest(1.4 * PI), VertexType::Corner);
    assert_eq!(VertexType::nearest(1.9 * PI), VertexType::Reversal);
    assert_eq!(SideDir::PlusI.turned(1), SideDir::PlusJ);
    assert_eq!(SideDir::PlusI.turned(-1), SideDir::MinusJ);
    assert_eq!(SideDir::PlusJ.reversed(), SideDir::MinusJ);
    assert!(SideDir::MinusI.along_i() && !SideDir::MinusJ.along_i());
    // A full left turn returns to the start after four Ends.
    assert_eq!(
        (0..4).fold(SideDir::PlusI, |d, _| d
            .turned(VertexType::End.submap_weight())),
        SideDir::PlusI
    );
}
