//! Acceptance tests of the surface `submap` scheme (plan W3-SUBMAP), through `Driver::mesh` into
//! a dimension-2 builder, with the stub `equal` curve scheme.
#![allow(clippy::needless_range_loop)]

mod common;

use common::*;
use sm_core::{EntityId, Point3};
use sm_mesh_api::{CellIx, MeshRead, validate};
use sm_scheme_api::stubs::brick_setup;
use sm_scheme_api::{
    Applicability, Attempt, CornerPicker, Driver, FailureKind, Firm, Firmness, IntervalSolver,
    MeshInput, MeshIntent, Meshed, Params, Phase, PlanCtx, ReferenceCornerPicker,
    ReferenceIntervalSolver, SchemeChoice, SchemeError, ScopedThreads, Sequential, SideDir, Target,
    VertexType, WholeFaces, geom,
};
use sm_scheme_submap::blocks::{cuts, decompose};
use sm_scheme_submap::lattice::{LatticePolygon, Location, lattice_polygon};
use sm_testkit::loops2d;

type Solvers = (Box<dyn IntervalSolver>, Box<dyn CornerPicker>, &'static str);

/// The reference solver/picker pair and the optimal pair of `sm-sizing`.
fn solver_pairs() -> Vec<Solvers> {
    vec![
        (
            Box::new(ReferenceIntervalSolver::default()),
            Box::new(ReferenceCornerPicker),
            "reference",
        ),
        (
            Box::new(sm_sizing::IiaSolver::default()),
            Box::new(sm_sizing::AnglePicker),
            "sizing",
        ),
    ]
}

fn mesh_pair(
    pair: &Solvers,
    f: &BareFace,
    intent: &MeshIntent,
) -> Result<Meshed<sm_mesh::Mesh>, SchemeError> {
    mesh_with(&registry(), &*pair.0, &*pair.1, f, intent, &Sequential)
}

/// The solved side directions and interval sums of `f`.
fn sides_of(out: &Meshed<sm_mesh::Mesh>, f: EntityId) -> Vec<(SideDir, u32)> {
    let layout = out.plan.layout(f).expect("layout");
    layout
        .sides
        .iter()
        .map(|s| {
            let n = s
                .coedges
                .iter()
                .map(|c| out.plan.intervals(c.edge).expect("n"))
                .sum();
            (s.dir, n)
        })
        .collect()
}

fn polygon_of(out: &Meshed<sm_mesh::Mesh>, f: EntityId) -> LatticePolygon {
    lattice_polygon(&sides_of(out, f)).expect("lattice")
}

/// Undirected edge count of the quad mesh.
fn edge_count(m: &dyn MeshRead) -> usize {
    let mut set = std::collections::BTreeSet::new();
    for c in 0..m.cell_count() {
        let cyc = m.cell_corners(CellIx::new(c)).expect("corners");
        let v: Vec<u32> = cyc.iter().map(|x| x.get() as u32).collect();
        for k in 0..v.len() {
            let (a, b) = (v[k], v[(k + 1) % v.len()]);
            set.insert((a.min(b), a.max(b)));
        }
    }
    set.len()
}

fn assert_quads_follow_normal(m: &dyn MeshRead, topo: &sm_topo_api::CellComplex) {
    for c in 0..m.cell_count() {
        let ci = CellIx::new(c);
        let cyc = m.cell_corners(ci).expect("corners");
        let p: Vec<_> = cyc.iter().map(|v| m.point(v.get())).collect();
        let n = (p[2] - p[0]).cross(p[3] - p[1]);
        let centroid = Point3::new(
            p.iter().map(|q| q.x).sum::<f64>() / 4.0,
            p.iter().map(|q| q.y).sum::<f64>() / 4.0,
            p.iter().map(|q| q.z).sum::<f64>() / 4.0,
        );
        let want = geom::face_normal(topo, m.cell_class(ci).entity, centroid).expect("normal");
        assert!(n.dot(want) > 0.0, "cell {c} against the normal");
    }
}

fn sum_of(out: &Meshed<sm_mesh::Mesh>, edges: &[EntityId]) -> u32 {
    edges
        .iter()
        .map(|e| out.plan.intervals(*e).expect("n"))
        .sum()
}

// ------------------------------------------------------------------------------ acceptance 1

#[test]
fn case1_l_shape_at_size_quarter_is_the_48_quad_lattice() {
    let l = face_of(&loops2d::l_shape());
    for pair in solver_pairs() {
        let out = mesh_pair(&pair, &l, &sized(0.25)).unwrap_or_else(|e| panic!("{}: {e}", pair.2));
        let tag = pair.2;
        for (e, n) in l.edges.iter().zip([8, 4, 4, 4, 4, 8]) {
            assert_eq!(out.plan.intervals(*e), Some(n), "{tag}");
        }
        // Layout: six sides, first side is geometric edge 1.
        let layout = out.plan.layout(l.face).expect("layout");
        assert_eq!(layout.sides.len(), 6, "{tag}");
        let dirs: Vec<_> = layout.sides.iter().map(|s| s.dir).collect();
        use SideDir::{MinusI, MinusJ, PlusI, PlusJ};
        assert_eq!(
            dirs,
            vec![PlusI, PlusJ, PlusI, PlusJ, MinusI, MinusJ],
            "{tag}"
        );
        assert_eq!(layout.sides[0].coedges[0].edge, l.edges[1], "{tag}");
        // Rows {e1, e3} = {e5} and {e2, e4} = {e0}.
        let e = &l.edges;
        assert_eq!(sum_of(&out, &[e[1], e[3]]), sum_of(&out, &[e[5]]));
        assert_eq!(sum_of(&out, &[e[2], e[4]]), sum_of(&out, &[e[0]]));
        let poly = polygon_of(&out, l.face);
        assert_eq!(poly.pick_area(), (48, 32, 33), "{tag}");
        assert_eq!(decompose(&poly).len(), 2, "{tag}");

        let m = &out.finished.mesh;
        assert!(validate(m).is_empty(), "{tag}: {:?}", validate(m));
        assert_eq!(m.cell_count(), 48, "{tag}");
        assert_eq!(m.vertex_count(), 65, "{tag}");
        assert_eq!(
            m.vertex_count() as i64 - edge_count(m) as i64 + m.cell_count() as i64,
            1,
            "{tag}: Euler"
        );
        assert!((area(m) - 3.0).abs() < 1e-12, "{tag}: {}", area(m));
        let (lo, hi) = scaled_jacobian_range(m);
        assert!(
            (lo - 1.0).abs() < 1e-12 && (hi - 1.0).abs() < 1e-12,
            "{tag}: {lo} {hi}"
        );
        assert_quads_follow_normal(m, &l.topo);
        // Every node is a point of the 0.25 lattice inside the L.
        for p in points(m) {
            assert!(p[2].abs() == 0.0);
            for c in &p[..2] {
                assert!((c * 4.0 - (c * 4.0).round()).abs() < 1e-12, "{p:?}");
            }
        }
    }
}

#[test]
fn both_pickers_give_the_same_layouts_and_meshes() {
    let cases = [
        (loops2d::l_shape(), sized(0.25)),
        (loops2d::five_sided_submap(), sized(0.5)),
        (loops2d::rectangle(3.0, 2.0), sized(0.5)),
    ];
    for (region, intent) in cases {
        let f = face_of(&region);
        let runs: Vec<_> = solver_pairs()
            .iter()
            .map(|p| mesh_pair(p, &f, &intent).expect("mesh"))
            .collect();
        let (a, b) = (&runs[0], &runs[1]);
        assert_eq!(
            a.plan.layout(f.face),
            b.plan.layout(f.face),
            "{}",
            region.name
        );
        assert_eq!(
            sm_testkit::check::mesh_hash(&a.finished.mesh),
            sm_testkit::check::mesh_hash(&b.finished.mesh),
            "{}",
            region.name
        );
    }
}

// ------------------------------------------------------------------------------ acceptance 2

/// Minimum scaled Jacobian measured on the first run of case 2 (see DESIGN.md); the test pins
/// 90% of it, never below 0.5.
const CASE2_MEASURED_MIN: f64 = 0.980_580_675_690_92;

#[test]
fn case2_l_shape_with_hard_counts_5_3_3_2_2_5() {
    let l = face_of(&loops2d::l_shape());
    let intent = counts(&[
        (l.edges[0], 5),
        (l.edges[1], 3),
        (l.edges[2], 3),
        (l.edges[3], 2),
        (l.edges[4], 2),
        (l.edges[5], 5),
    ]);
    for pair in solver_pairs() {
        let out = mesh_pair(&pair, &l, &intent).unwrap_or_else(|e| panic!("{}: {e}", pair.2));
        let tag = pair.2;
        let poly = polygon_of(&out, l.face);
        assert_eq!(
            poly.verts,
            vec![(0, 0), (3, 0), (3, 3), (5, 3), (5, 5), (0, 5)],
            "{tag}"
        );
        assert_eq!(poly.pick_area(), (19, 20, 10), "{tag}");
        let m = &out.finished.mesh;
        assert!(validate(m).is_empty(), "{tag}: {:?}", validate(m));
        assert_eq!(m.cell_count(), 19, "{tag}");
        assert_eq!(m.vertex_count(), 30, "{tag}");
        assert_eq!(decompose(&poly).len(), 2, "{tag}");
        assert!((area(m) - 3.0).abs() < 1e-12, "{tag}: {}", area(m));
        let (lo, _) = scaled_jacobian_range(m);
        eprintln!("case 2 ({tag}): min scaled Jacobian {lo}");
        assert!(
            lo > 0.0 && lo >= (0.9 * CASE2_MEASURED_MIN).max(0.5),
            "{tag}: {lo}"
        );
        assert_quads_follow_normal(m, &l.topo);
    }
}

// ------------------------------------------------------------------------------ acceptance 3

#[test]
fn case3_five_sided_face_is_four_logical_sides_with_exact_rows() {
    let r = face_of(&loops2d::five_sided_submap());
    for pair in solver_pairs() {
        let out = mesh_pair(&pair, &r, &sized(0.5)).unwrap_or_else(|e| panic!("{}: {e}", pair.2));
        let tag = pair.2;
        let layout = out.plan.layout(r.face).expect("layout");
        assert_eq!(layout.sides.len(), 4, "{tag}");
        assert_eq!(
            layout.sides.iter().filter(|s| s.coedges.len() == 2).count(),
            1,
            "{tag}"
        );
        let m = &out.finished.mesh;
        assert!(validate(m).is_empty(), "{tag}: {:?}", validate(m));
        assert_eq!(m.cell_count(), 32, "{tag}");
        assert_eq!(m.vertex_count(), 45, "{tag}");
        assert!((area(m) - 8.0).abs() < 1e-12, "{tag}");
        let (lo, hi) = scaled_jacobian_range(m);
        assert!(
            (lo - 1.0).abs() < 1e-12 && (hi - 1.0).abs() < 1e-12,
            "{tag}"
        );
    }
    // Hard goals derived from the layout: bottom = top exactly, read through the layout.
    let probe = mesh_face(&r, &sized(0.5), &Sequential).expect("probe");
    let layout = probe.plan.layout(r.face).expect("layout");
    let sums: Vec<u32> = layout
        .sides
        .iter()
        .map(|s| {
            s.coedges
                .iter()
                .map(|c| probe.plan.intervals(c.edge).expect("n"))
                .sum()
        })
        .collect();
    assert_eq!(sums[0], sums[2]);
    assert_eq!(sums[1], sums[3]);
    // Pin the two-edge side to 3 + 5 = 8 and the opposite side to 8 with a hard count.
    let two = layout
        .sides
        .iter()
        .position(|s| s.coedges.len() == 2)
        .expect("side");
    let opposite = (two + 2) % 4;
    let first = layout.sides[two].coedges[0].edge;
    let opp_edge = layout.sides[opposite].coedges[0].edge;
    let intent = counts(&[(first, 3), (opp_edge, 8)]);
    let out = mesh_face(&r, &intent, &Sequential).expect("hard goals");
    let layout = out.plan.layout(r.face).expect("layout");
    let on_side = |k: usize| -> u32 {
        layout.sides[k]
            .coedges
            .iter()
            .map(|c| out.plan.intervals(c.edge).expect("n"))
            .sum()
    };
    assert_eq!(on_side(two), on_side(opposite));
    assert_eq!(on_side(opposite), 8);
    assert!(validate(&out.finished.mesh).is_empty());
}

// ------------------------------------------------------------------------------ acceptance 4

#[test]
fn case4_rectangle_is_the_analytic_lattice_in_one_block() {
    let r = face_of(&loops2d::rectangle(3.0, 2.0));
    for pair in solver_pairs() {
        let out = mesh_pair(&pair, &r, &sized(0.5)).unwrap_or_else(|e| panic!("{}: {e}", pair.2));
        let m = &out.finished.mesh;
        assert_eq!(m.cell_count(), 24, "{}", pair.2);
        assert_eq!(decompose(&polygon_of(&out, r.face)).len(), 1);
        let mut got = points(m);
        got.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
        let mut want: Vec<[f64; 3]> = (0..=4)
            .flat_map(|j| (0..=6).map(move |i| [0.5 * f64::from(i), 0.5 * f64::from(j), 0.0]))
            .collect();
        want.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
        assert_eq!(got.len(), want.len());
        for (g, w) in got.iter().zip(&want) {
            // Bit-exact: every value is a representable multiple of 0.5.
            assert_eq!(
                g.map(f64::to_bits),
                w.map(f64::to_bits),
                "{} vs {}",
                g[0],
                w[0]
            );
        }
    }
}

// ------------------------------------------------------------------------------ acceptance 5

mod staircase_property {
    use super::*;
    use proptest::prelude::*;

    fn run(steps: &[(u32, u32)]) -> Result<(), TestCaseError> {
        let region = loops2d::staircase(steps);
        let face = face_of(&region);
        // Unit spacing: counts are the edge lengths.
        let n = region.outer.len();
        let mut pairs = Vec::new();
        for i in 0..n {
            let (a, b) = (region.outer[i], region.outer[(i + 1) % n]);
            pairs.push((
                face.edges[i],
                ((b.0 - a.0).abs() + (b.1 - a.1).abs()) as u32,
            ));
        }
        let out = mesh_face(&face, &counts(&pairs), &Sequential)
            .map_err(|e| TestCaseError::fail(format!("{steps:?}: {e}")))?;
        let poly = polygon_of(&out, face.face);
        let (a, b, i) = poly.pick_area();
        let m = &out.finished.mesh;
        prop_assert_eq!(m.cell_count() as u64, a);
        prop_assert_eq!(m.vertex_count() as u64, i + b);
        prop_assert!(validate(m).is_empty());
        prop_assert!((area(m) - region.area).abs() < 1e-12);
        let (lo, hi) = scaled_jacobian_range(m);
        prop_assert!(
            (lo - 1.0).abs() < 1e-12 && (hi - 1.0).abs() < 1e-12,
            "{lo} {hi}"
        );
        let blocks = decompose(&poly);
        prop_assert_eq!(blocks.len(), steps.len() + 1);
        prop_assert_eq!(blocks.iter().map(|b| b.area()).sum::<u64>(), a);
        // The node set is the set of integer lattice points of the polygon, up to the
        // (rotation, translation) of the frame: every node is an integer point of the
        // region itself, and the counts agree.
        let mut got: Vec<(i64, i64)> = points(m)
            .iter()
            .map(|p| {
                prop_assert!(
                    (p[0] - p[0].round()).abs() < 1e-12 && (p[1] - p[1].round()).abs() < 1e-12
                );
                Ok((p[0].round() as i64, p[1].round() as i64))
            })
            .collect::<Result<_, TestCaseError>>()?;
        got.sort_unstable();
        got.dedup();
        prop_assert_eq!(got.len(), m.vertex_count());
        let want = region_lattice_points(&region);
        prop_assert_eq!(got, want);
        Ok(())
    }

    /// Integer points inside or on the region, by the region's own even-odd test on the
    /// half-integer cells around each point.
    fn region_lattice_points(r: &loops2d::Region2) -> Vec<(i64, i64)> {
        let max = r
            .outer
            .iter()
            .fold((0.0f64, 0.0f64), |m, p| (m.0.max(p.0), m.1.max(p.1)));
        let mut out = Vec::new();
        for x in 0..=max.0 as i64 {
            for y in 0..=max.1 as i64 {
                let touches = [(-0.5, -0.5), (0.5, -0.5), (-0.5, 0.5), (0.5, 0.5)]
                    .iter()
                    .any(|d| r.contains((x as f64 + d.0, y as f64 + d.1)));
                if touches {
                    out.push((x, y));
                }
            }
        }
        out
    }

    proptest! {
        // Fixed seed, no failure persistence (CLAUDE.md: explicitly seeded randomness).
        #![proptest_config(sm_testkit::strategies::config(48))]
        #[test]
        fn random_staircases_mesh_to_the_integer_lattice(
            steps in proptest::collection::vec((1u32..=4, 1u32..=4), 1..=4)
        ) {
            // Failures print the minimal counterexample (`steps`) and proptest's seed.
            run(&steps)?;
        }
    }

    #[test]
    fn the_unit_l_staircase_is_the_testkit_l() {
        run(&[(1, 1)]).expect("L");
    }
}

// ------------------------------------------------------------------------------ acceptance 7

#[test]
fn case7_a_broken_closure_row_is_infeasible_in_both_solvers_naming_the_rows() {
    let l = face_of(&loops2d::l_shape());
    // e0 = 6 against e2 + e4 = 3 + 2 (all hard).
    let intent = counts(&[(l.edges[0], 6), (l.edges[2], 3), (l.edges[4], 2)]);
    for pair in solver_pairs() {
        let reg = registry();
        let driver = Driver::new(&reg, &*pair.0, &*pair.1);
        let sel = [l.face];
        let input = MeshInput::new(&l.topo, &l.model, &WholeFaces, &sel);
        match driver.plan(&input, &intent) {
            Err(SchemeError::Infeasible(inf)) => {
                assert!(!inf.constraints.is_empty(), "{}: rows named", pair.2);
                for e in [l.edges[0], l.edges[2], l.edges[4]] {
                    assert!(inf.entities.contains(&e), "{}: {inf:?}", pair.2);
                }
            }
            other => panic!(
                "{}: expected Infeasible, got {:?}",
                pair.2,
                other.map(|_| ())
            ),
        }
    }
}

#[test]
fn case7_a_seam_edge_on_both_sides_cancels_without_an_empty_row() {
    use sm_scheme_api::IntervalConstraint;
    let e = EntityId::new(sm_core::EntityKind::TopoEdge, 5).expect("id");
    let f = EntityId::new(sm_core::EntityKind::TopoEdge, 6).expect("id");
    match IntervalConstraint::sum_equal(&[e, f], &[e]) {
        IntervalConstraint::Linear { terms, .. } => assert_eq!(terms, vec![(f, 1)]),
        other => panic!("{other:?}"),
    }
    assert!(matches!(
        IntervalConstraint::sum_equal(&[e], &[e]),
        IntervalConstraint::Linear { terms, .. } if terms.is_empty()
    ));
}

/// The scheme's own empty-row skip: the cylinder's lateral face has the seam edge on both
/// `i` sides, so the `sum(+i) = sum(-i)` row cancels to an empty row and must be skipped.
#[test]
fn case7_the_seam_edge_on_both_sides_is_skipped_by_the_scheme_not_infeasible() {
    use sm_topo_api::fixtures::with_op;
    use sm_topo_api::{CellComplex, OpId};
    let mut topo = CellComplex::new();
    let ids = with_op(&mut topo, OpId(1), |t| sm_topo_api::fixtures::cylinder(t, 1.0, 2.0))
        .expect("cylinder");
    let lat = ids.faces[2];
    let seam = ids.edges[2];
    let mut intent = counts(&[(ids.edges[0], 4), (ids.edges[1], 4), (seam, 3)]);
    intent.set_scheme(lat, SchemeChoice::new("submap", Firmness::Hard));
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let model = sm_model_api::Model::new();
    let sel = [lat];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let plan = driver.plan(&input, &intent).expect("plan succeeds, no empty infeasible row");
    let layout = plan.layouts.iter().find(|l| l.face == lat).expect("layout");
    assert_eq!(layout.sides.len(), 4);
    let seam_sides = layout
        .sides
        .iter()
        .filter(|s| s.coedges.iter().any(|c| c.edge == seam))
        .count();
    assert_eq!(seam_sides, 2, "the seam is on two sides");
}

// ------------------------------------------------------------------------------ acceptance 8

/// A U whose closure rows hold (5 = 1 + 2 + 2 along i, 4 + 4 = 4 + 4 along j) but whose
/// lattice polygon touches itself.
fn touching_u() -> (BareFace, MeshIntent) {
    use loops2d::Region2;
    let outer = vec![
        (0.0, 0.0),
        (5.0, 0.0),
        (5.0, 4.0),
        (4.0, 4.0),
        (4.0, 1.0),
        (2.0, 1.0),
        (2.0, 4.0),
        (0.0, 4.0),
    ];
    let region = Region2 {
        name: "u",
        outer,
        holes: vec![],
        area: 20.0 - 6.0,
        outer_types: None,
    };
    let f = face_of(&region);
    let intent = counts(&[
        (f.edges[0], 5),
        (f.edges[1], 4),
        (f.edges[2], 1),
        (f.edges[3], 4),
        (f.edges[4], 2),
        (f.edges[5], 4),
        (f.edges[6], 2),
        (f.edges[7], 4),
    ]);
    (f, intent)
}

#[test]
fn case8_a_lattice_failure_falls_back_to_pave_and_is_recorded() {
    let (u, mut intent) = touching_u();
    let reg = registry_with_pave_fallback();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    // Imposed submap with the registry chain submap -> pave.
    intent.set_scheme(u.face, SchemeChoice::new("submap", Firmness::Soft));
    let out = mesh_with(&reg, &solver, &picker, &u, &intent, &Sequential).expect("falls back");
    let tp = out.plan.target(Target::Surface(u.face)).expect("target");
    assert_eq!(tp.scheme.as_str(), "pave");
    let a: &Attempt = out
        .attempts
        .iter()
        .find(|a| a.scheme.as_str() == "submap" && a.phase == Phase::Generate)
        .expect("submap attempt recorded");
    assert_eq!(a.kind, FailureKind::Constraint);
    assert!(
        a.message.starts_with("lattice:") || a.message.contains("lattice:"),
        "{}",
        a.message
    );
    assert!(a.message.contains("not simple"), "{}", a.message);
    assert!(validate(&out.finished.mesh).is_empty());

    // Without the chain (and with no other candidate) the failure reaches the caller.
    let bare = registry();
    let mut hard = intent.clone();
    hard.set_scheme(u.face, SchemeChoice::new("submap", Firmness::Hard));
    let err = mesh_with(&bare, &solver, &picker, &u, &hard, &Sequential)
        .err()
        .expect("error");
    match err {
        SchemeError::NoApplicableScheme { tried, .. } => {
            let a = tried
                .iter()
                .find(|a| a.scheme.as_str() == "submap")
                .expect("attempt");
            assert!(a.message.contains("Constraint"), "{}", a.message);
            assert!(
                a.message.contains("lattice: polygon is not simple"),
                "{}",
                a.message
            );
        }
        other => panic!("{other:?}"),
    }
}

// ------------------------------------------------------------------------------ acceptance 9

fn side_node_keys(
    out: &Meshed<sm_mesh::Mesh>,
    topo: &sm_topo_api::CellComplex,
    f: EntityId,
) -> Vec<Vec<sm_scheme_api::NodeKey>> {
    use sm_scheme_api::NodeKey;
    use sm_topo_api::{Side, TopologyViewExt};
    let layout = out.plan.layout(f).expect("layout");
    layout
        .sides
        .iter()
        .map(|s| {
            let mut keys: Vec<NodeKey> = Vec::new();
            for c in &s.coedges {
                let (a, b) = topo.edge_ends(c.edge).expect("ends");
                let n = out.plan.intervals(c.edge).expect("n");
                let mut v = vec![NodeKey::vertex(a)];
                v.extend((1..n).map(|i| NodeKey::new(c.edge, i)));
                v.push(NodeKey::vertex(b));
                if c.side == Side::Right {
                    v.reverse();
                }
                let skip = usize::from(!keys.is_empty());
                keys.extend_from_slice(&v[skip..]);
            }
            keys
        })
        .collect()
}

#[test]
fn case9_strips_cross_the_l_boundary_to_boundary() {
    let l = face_of(&loops2d::l_shape());
    let out = mesh_face(&l, &sized(0.25), &Sequential).expect("mesh");
    let patch = out.patches.get(Target::Surface(l.face)).expect("patch");
    assert!(patch.grid.is_none());
    let poly = polygon_of(&out, l.face);
    let keys = side_node_keys(&out, &l.topo, l.face);
    let left_of = |d: SideDir| match d {
        SideDir::PlusI => (0, 1),
        SideDir::PlusJ => (-1, 0),
        SideDir::MinusI => (0, -1),
        SideDir::MinusJ => (1, 0),
    };
    let boundary: std::collections::BTreeSet<_> = patch.refs.iter().copied().collect();
    let mut walked = 0;
    for (k, side) in keys.iter().enumerate() {
        let (x, y) = poly.verts[k];
        let (sx, sy) = sm_scheme_submap::lattice::step(poly.dirs[k]);
        let (nx, ny) = left_of(poly.dirs[k]);
        for (m, key) in side.iter().enumerate().take(side.len() - 1).skip(1) {
            let (a, b) = (x + sx * m as i64, y + sy * m as i64);
            // Lattice extent along the inward line to the first boundary point.
            let mut t = 1;
            while poly.inside(a + nx * t, b + ny * t) == Location::Inside {
                t += 1;
            }
            assert_eq!(poly.inside(a + nx * t, b + ny * t), Location::Boundary);
            let strip = patch
                .strip_from(*key)
                .unwrap_or_else(|| panic!("no strip from {key:?}"));
            assert_eq!(strip.len() as i64, t + 1, "side {k} node {m}");
            assert_eq!(strip[0], *key);
            assert!(
                boundary.contains(strip.last().expect("end")),
                "ends on the boundary"
            );
            walked += 1;
        }
    }
    // Every non-corner boundary node: 32 boundary points minus 6 corners.
    assert_eq!(walked, 26);
}

// ------------------------------------------------------------------------------ acceptance 10

fn applicability(
    f: &BareFace,
    reg: &sm_scheme_api::SchemeRegistry,
    name: &str,
    intent: &MeshIntent,
    params: &Params,
) -> Applicability {
    let sel = [f.face];
    let input = MeshInput::new(&f.topo, &f.model, &WholeFaces, &sel);
    let cx = PlanCtx::new(&input, intent);
    reg.get(sm_scheme_api::SchemeDim::Surface, name)
        .expect("scheme")
        .applicability(&cx, Target::Surface(f.face), params)
}

#[test]
fn case10_applicability_scores() {
    let reg = registry();
    let none = MeshIntent::default();
    let p = Params::default();
    let score = |a: Applicability| match a {
        Applicability::Applicable { score, .. } => Some(score),
        _ => None,
    };
    // Rectangle and L: corner sum 4.
    let rect = face_of(&loops2d::rectangle(3.0, 2.0));
    assert_eq!(
        score(applicability(&rect, &reg, "submap", &none, &p)),
        Some(80)
    );
    let l = face_of(&loops2d::l_shape());
    assert_eq!(
        score(applicability(&l, &reg, "submap", &none, &p)),
        Some(80)
    );
    // Map would beat it on the rectangle.
    let mut both = registry();
    sm_scheme_map::register(&mut both).expect("map");
    assert_eq!(
        score(applicability(&rect, &both, "map", &none, &p)),
        Some(100)
    );
    // A triangle's corner sum is 3: submap scores 20 for "needs picking"... but it has fewer
    // than four corners, so it is not applicable at all.
    let tri = face_of(&loops2d::circle(1.0, 3));
    assert!(matches!(
        applicability(&tri, &both, "submap", &none, &p),
        Applicability::NotApplicable { .. }
    ));
    // An L with one hard End on its re-entrant corner: sum 5, so 20.
    let corners = geom::corners(&l.topo, l.face).expect("corners");
    let reentrant = corners
        .iter()
        .find(|c| VertexType::nearest(c.angle) == VertexType::Corner)
        .expect("corner");
    let mut forced = MeshIntent::default();
    forced.corners.push(sm_scheme_api::CornerIntent::new(
        reentrant.key,
        Firm::hard(VertexType::End),
    ));
    assert_eq!(
        score(applicability(&l, &reg, "submap", &forced, &p)),
        Some(20)
    );
    // A hard Reversal is refused.
    let mut rev = MeshIntent::default();
    rev.corners.push(sm_scheme_api::CornerIntent::new(
        reentrant.key,
        Firm::hard(VertexType::Reversal),
    ));
    match applicability(&l, &reg, "submap", &rev, &p) {
        Applicability::NotApplicable { reason, .. } => {
            assert!(reason.contains("reversal"), "{reason}")
        }
        other => panic!("{other:?}"),
    }
    // A face with a hole.
    let holes = face_of(&loops2d::rectangle_with_holes());
    match applicability(&holes, &reg, "submap", &none, &p) {
        Applicability::NotApplicable { reason, .. } => assert!(reason.contains("loop"), "{reason}"),
        other => panic!("{other:?}"),
    }
    // Parameters: {} accepted, {"x":1} rejected.
    let empty = Params(serde_json::json!({}));
    assert_eq!(
        score(applicability(&l, &reg, "submap", &none, &empty)),
        Some(80)
    );
    let bad = Params(serde_json::json!({ "x": 1 }));
    assert!(matches!(
        applicability(&l, &reg, "submap", &none, &bad),
        Applicability::NotApplicable { .. }
    ));
    // Caps: no STRUCTURED, so a volume map does not accept it.
    let caps = reg
        .get(sm_scheme_api::SchemeDim::Surface, "submap")
        .expect("submap")
        .caps(&p);
    assert_eq!(
        caps,
        sm_scheme_api::Caps::QUAD | sm_scheme_api::Caps::BLOCK_STRUCTURED
    );
}

#[test]
fn a_surface_scheme_does_not_apply_to_a_non_surface_target() {
    let reg = registry();
    let l = face_of(&loops2d::l_shape());
    let sel = [l.face];
    let input = MeshInput::new(&l.topo, &l.model, &WholeFaces, &sel);
    let intent = MeshIntent::default();
    let cx = PlanCtx::new(&input, &intent);
    let s = reg
        .get(sm_scheme_api::SchemeDim::Surface, "submap")
        .expect("submap");
    assert!(matches!(
        s.applicability(&cx, Target::Curve(l.edges[0]), &Params::default()),
        Applicability::NotApplicable { .. }
    ));
}

// ------------------------------------------------------------------------------ acceptance 11

#[test]
fn case11_output_is_bit_identical_across_executors_runs_and_threads() {
    let l = face_of(&loops2d::l_shape());
    let hash = |threads: usize| {
        let out = if threads <= 1 {
            mesh_face(&l, &sized(0.25), &Sequential)
        } else {
            mesh_face(&l, &sized(0.25), &ScopedThreads(threads))
        }
        .expect("mesh");
        sm_testkit::check::mesh_hash(&out.finished.mesh)
    };
    sm_testkit::determinism::check(3, 4, hash).expect("deterministic");

    // Several faces in one wave: the six faces of a brick, all under submap.
    let (topo, ids, model) = brick_setup([1.0, 1.5, 2.0]);
    let mut intent = counts(&[(ids.edges[0], 2), (ids.edges[4], 3), (ids.edges[8], 4)]);
    for f in &ids.faces {
        intent.set_scheme(*f, SchemeChoice::new("submap", Firmness::Hard));
    }
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let brick_hash = |threads: usize| {
        let driver = Driver::new(&reg, &solver, &picker);
        let input = MeshInput::new(&topo, &model, &WholeFaces, &ids.faces);
        let mut alloc = sm_mesh_api::SerialAlloc::new();
        let out = if threads <= 1 {
            driver.mesh(
                &input,
                &intent,
                &Sequential,
                sm_mesh::MeshBuilder::surface(),
                &mut alloc,
            )
        } else {
            driver.mesh(
                &input,
                &intent,
                &ScopedThreads(threads),
                sm_mesh::MeshBuilder::surface(),
                &mut alloc,
            )
        }
        .expect("brick");
        assert_eq!(out.finished.mesh.cell_count(), 2 * (12 + 8 + 6));
        sm_testkit::check::mesh_hash(&out.finished.mesh)
    };
    sm_testkit::determinism::check(3, 4, brick_hash).expect("deterministic brick");

    // Cold replay: replanning from scratch with the stored rejections gives the same plan.
    let out = mesh_face(&l, &sized(0.25), &Sequential).expect("mesh");
    let reg = registry();
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [l.face];
    let input = MeshInput::new(&l.topo, &l.model, &WholeFaces, &sel);
    let replay = driver
        .plan_with(&input, &sized(0.25), &out.plan.rejections)
        .expect("replay");
    assert_eq!(replay, out.plan);
}

// ------------------------------------------------------------------------------ extras

#[test]
fn interior_node_keys_are_canonical_row_major_ranks() {
    let l = face_of(&loops2d::l_shape());
    let out = mesh_face(&l, &sized(0.25), &Sequential).expect("mesh");
    let patch = out.patches.get(Target::Surface(l.face)).expect("patch");
    let ranks: Vec<u32> = patch.nodes.iter().map(|n| n.key.index).collect();
    assert_eq!(ranks, (0..33).collect::<Vec<u32>>());
    // The first side runs along geometric edge 1, so the lattice `j` direction is the
    // geometric x axis: the cut extends the horizontal edge e2 through the re-entrant
    // vertex (1, 1) to the left boundary, with three interior points.
    let on_cut = patch
        .nodes
        .iter()
        .filter(|n| (n.point.y - 1.0).abs() < 1e-12 && n.point.x > 0.0 && n.point.x < 1.0)
        .count();
    assert_eq!(on_cut, 3, "one cut with three interior points");
}

#[test]
fn a_sized_u_face_meshes_with_four_valence_interior_nodes() {
    use loops2d::Region2;
    let outer = vec![
        (0.0, 0.0),
        (5.0, 0.0),
        (5.0, 4.0),
        (4.0, 4.0),
        (4.0, 1.0),
        (2.0, 1.0),
        (2.0, 4.0),
        (0.0, 4.0),
    ];
    let region = Region2 {
        name: "u",
        outer,
        holes: vec![],
        area: 14.0,
        outer_types: None,
    };
    let u = face_of(&region);
    let out = mesh_face(&u, &sized(0.5), &Sequential).expect("U");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert!((area(m) - 14.0).abs() < 1e-12);
    let (lo, hi) = scaled_jacobian_range(m);
    assert!((lo - 1.0).abs() < 1e-12 && (hi - 1.0).abs() < 1e-12);
    let poly = polygon_of(&out, u.face);
    assert_eq!(decompose(&poly).len(), 3);
    assert_eq!(cuts(&decompose(&poly)).len(), 2);
}

/// `cargo test -p sm-scheme-submap --release -- --ignored --nocapture perf`: wall time of the
/// whole driver run on a large L (recorded in W3_REPORT.md; not a gate).
#[test]
#[ignore = "timing measurement, run explicitly"]
fn perf_large_l_shape() {
    let l = face_of(&loops2d::l_shape());
    for size in [0.02, 0.01, 0.005] {
        let t = std::time::Instant::now();
        let out = mesh_face(&l, &sized(size), &Sequential).expect("mesh");
        let dt = t.elapsed();
        let m = &out.finished.mesh;
        eprintln!(
            "size {size}: {} quads, {} nodes, {dt:?}",
            m.cell_count(),
            m.vertex_count()
        );
    }
}
