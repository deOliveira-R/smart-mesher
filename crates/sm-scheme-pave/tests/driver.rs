//! Driver-level acceptance tests of the `pave` scheme (plan W3-PAVE items 5-10): meshes
//! through `Driver::mesh` into a dimension-2 builder with the real `pave`, the stub `equal`
//! curve scheme and the stub `tri` as fallback target.
#![allow(clippy::needless_range_loop)]

mod common;

use common::*;
use core::f64::consts::TAU;
use sm_core::{EntityId, Point3};
use sm_mesh_api::{CellIx, CellShape, FaceIx, FaceKind, MeshRead, validate};
use sm_scheme_api::geom;
use sm_scheme_api::stubs::brick_setup;
use sm_scheme_api::{
    Attempt, Driver, FailureKind, Firm, Firmness, IntervalRequest, MeshInput, MeshIntent, Phase,
    ReferenceCornerPicker, ReferenceIntervalSolver, SchemeChoice, SchemeError, ScopedThreads,
    Selection, Sequential, Target, WholeFaces,
};
use sm_testkit::loops2d::ngon_area;

/// Hard counts on the four edges of a brick z face: `nx` on the x edges, `ny` on the y edges.
fn brick_face_counts(topo: &sm_topo_api::CellComplex, face: EntityId, nx: u32, ny: u32) -> MeshIntent {
    let pairs: Vec<(EntityId, u32)> = face_edges(topo, face)
        .into_iter()
        .map(|e| {
            let l = edge_len(topo, e);
            (e, if (l - 4.0).abs() < 1e-9 { nx } else { ny })
        })
        .collect();
    hard(&pairs)
}

/// Every cell of the mesh faces the same way as the face normal at its centroid.
fn cells_follow_face_normal(m: &dyn MeshRead, topo: &sm_topo_api::CellComplex, face: EntityId) {
    for c in 0..m.cell_count() {
        let p = corners(m, c);
        let n = (p[2] - p[0]).cross(p[3] - p[1]);
        let centroid = Point3::new(
            p.iter().map(|q| q.x).sum::<f64>() / 4.0,
            p.iter().map(|q| q.y).sum::<f64>() / 4.0,
            p.iter().map(|q| q.z).sum::<f64>() / 4.0,
        );
        let want = geom::face_normal(topo, face, centroid).expect("face normal");
        assert!(n.dot(want) > 0.0, "cell {c} faces away from the face normal");
    }
}

fn all_quads(m: &dyn MeshRead) {
    assert!((0..m.cell_count()).all(|c| m.cell_shape(CellIx::new(c)) == CellShape::Quad));
}

#[test]
fn acceptance_5_planar_brick_face() {
    let (topo, ids, model) = brick_setup([4.0, 3.0, 1.0]);
    let face = ids.faces[5];
    let intent = brick_face_counts(&topo, face, 8, 6);
    let out = mesh_faces(&topo, &model, &[face], &intent, &Sequential).expect("mesh");
    let tp = out.plan.target(Target::Surface(face)).expect("face target");
    assert_eq!(tp.scheme.as_str(), "pave");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    all_quads(m);
    assert!((area(m) - 12.0).abs() < 1e-12, "area {}", area(m));
    // Boundary nodes are exactly the edge nodes: the 28 points on the rectangle outline,
    // every one a multiple of the interval length on its edge.
    let outline: Vec<Point3> = points(m).into_iter().filter(|p| p.x == 0.0 || p.x == 4.0 || p.y == 0.0 || p.y == 3.0).collect();
    assert_eq!(outline.len(), 28);
    for p in &outline {
        let on_x = p.y == 0.0 || p.y == 3.0;
        let s = if on_x { p.x / 0.5 } else { p.y / 0.5 };
        assert!((s - s.round()).abs() < 1e-12, "{p:?} is not an edge node");
    }
    let boundary_facets = (0..m.face_count()).filter(|f| m.face_kind(FaceIx::new(*f)) == FaceKind::Boundary).count();
    assert_eq!(boundary_facets, 28);
    cells_follow_face_normal(m, &topo, face);
    let (min, mean) = scaled_jacobian(m);
    eprintln!("planar 8x6: {} quads, scaled Jacobian min {min:.3} mean {mean:.3}", m.cell_count());
    assert!(min >= 0.40 && mean >= 0.85, "min {min} mean {mean}");
}

#[test]
fn acceptance_6_cylinder_lateral_face() {
    let (r, h) = (3.0, 1.0);
    let (topo, ids, model) = cylinder(r, h);
    let lateral = ids.faces[2];
    let [bottom, top, seam] = ids.edges;
    let intent = hard(&[(bottom, 72), (top, 72), (seam, 4)]);
    let out = mesh_faces(&topo, &model, &[lateral], &intent, &Sequential).expect("mesh");
    assert_eq!(out.plan.target(Target::Surface(lateral)).map(|t| t.scheme.as_str()), Some("pave"));
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    all_quads(m);
    for p in points(m) {
        assert!((p.x.hypot(p.y) - r).abs() < 1e-12, "{p:?} is off the cylinder");
        assert!(p.z > -1e-12 && p.z < h + 1e-12, "{p:?} is outside 0 <= z <= h");
    }
    // The seam closes: 72 nodes per circle, the seam's 3 interior nodes shared, no duplicates.
    let pts = points(m);
    for (i, a) in pts.iter().enumerate() {
        for b in &pts[i + 1..] {
            assert!((*a - *b).norm() > 1e-9, "duplicate vertices at {a:?}");
        }
    }
    // Area: between the inscribed bound for the widest quad and the full lateral area.
    let mut widest = 0.0f64;
    let (mut longest_ratio, mut cell_count) = (0.0f64, 0);
    for c in 0..m.cell_count() {
        let p = corners(m, c);
        let ang: Vec<f64> = p.iter().map(|q| q.y.atan2(q.x)).collect();
        for a in 0..4 {
            for b in 0..4 {
                let mut d = (ang[a] - ang[b]).abs();
                if d > TAU / 2.0 {
                    d = TAU - d;
                }
                widest = widest.max(d);
            }
        }
        let e: Vec<f64> = (0..4).map(|k| (p[(k + 1) % 4] - p[k]).norm()).collect();
        let (lo, hi) = (e.iter().copied().fold(f64::INFINITY, f64::min), e.iter().copied().fold(0.0, f64::max));
        longest_ratio = longest_ratio.max(hi / lo);
        cell_count += 1;
    }
    let full = TAU * r * h;
    let a = area(m);
    assert!(a <= full * (1.0 + 1e-12) && a >= full * (widest / 2.0).cos() * (1.0 - 1e-12), "area {a} outside [{}, {full}]", full * (widest / 2.0).cos());
    assert!(longest_ratio <= 2.5, "edge ratio {longest_ratio}");
    cells_follow_face_normal(m, &topo, lateral);
    let (min, mean) = scaled_jacobian(m);
    eprintln!("cylinder lateral: {cell_count} quads, widest {widest:.4} rad, edge ratio {longest_ratio:.3}, scaled Jacobian min {min:.3} mean {mean:.3}");
    assert!(min >= 0.40 && mean >= 0.85, "min {min} mean {mean}");
}

#[test]
fn acceptance_7a_cylinder_bottom_cap_is_a_disk_of_quads() {
    let (r, h) = (3.0, 1.0);
    let (topo, ids, model) = cylinder(r, h);
    let cap = ids.faces[0];
    let intent = hard(&[(ids.edges[0], 32)]);
    let out = mesh_faces(&topo, &model, &[cap], &intent, &Sequential).expect("mesh");
    assert_eq!(out.plan.target(Target::Surface(cap)).map(|t| t.scheme.as_str()), Some("pave"));
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    all_quads(m);
    let want = ngon_area(32, r);
    assert!((area(m) - want).abs() <= 1e-12 * want, "area {} vs {want}", area(m));
    // The cap's normal is -z.
    for c in 0..m.cell_count() {
        let p = corners(m, c);
        assert!((p[2] - p[0]).cross(p[3] - p[1]).z < 0.0, "cell {c} faces +z");
    }
    cells_follow_face_normal(m, &topo, cap);
    let (min, mean) = scaled_jacobian(m);
    eprintln!("cap: {} quads, scaled Jacobian min {min:.3} mean {mean:.3}", m.cell_count());
    assert!(min >= 0.40 && mean >= 0.85, "min {min} mean {mean}");
}

// ---------------------------------------------------------------------------- item 8

/// Soft counts `3, 2, 2, 2` (odd sum) on the four edges of the -z face.
fn odd_face(firm: fn(IntervalRequest) -> Firm<IntervalRequest>) -> (sm_topo_api::CellComplex, sm_topo_api::fixtures::BrickIds, sm_model_api::Model, MeshIntent) {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let face = ids.faces[4];
    let mut intent = MeshIntent::default();
    let edges = face_edges(&topo, face);
    for (e, n) in edges.iter().zip([3, 2, 2, 2]) {
        intent.set_intervals(*e, firm(IntervalRequest::Count(n)));
    }
    intent.set_scheme(face, SchemeChoice::new("pave", Firmness::Soft));
    (topo, ids, model, intent)
}

fn face_sum(plan: &sm_scheme_api::Plan, topo: &sm_topo_api::CellComplex, face: EntityId) -> u32 {
    face_edges(topo, face).iter().map(|e| plan.intervals(*e).expect("count")).sum()
}

#[test]
fn acceptance_8_soft_odd_counts_are_repaired_to_an_even_sum_and_mesh() {
    let (topo, ids, model, intent) = odd_face(Firm::soft);
    let face = ids.faces[4];
    let out = mesh_faces(&topo, &model, &[face], &intent, &Sequential).expect("mesh");
    assert_eq!(out.plan.target(Target::Surface(face)).map(|t| t.scheme.as_str()), Some("pave"));
    let n = face_sum(&out.plan, &topo, face);
    assert_eq!(n % 2, 0, "the even rule was honoured: sum {n}");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    all_quads(m);
    assert!((area(m) - 1.0).abs() < 1e-12);
}

#[test]
fn acceptance_8_hard_odd_counts_blame_pave_and_fall_back_to_tri() {
    let (topo, ids, model, mut intent) = odd_face(Firm::hard);
    let face = ids.faces[4];
    intent.entity_mut(face).fallbacks = vec![SchemeChoice::new("tri", Firmness::Default)];
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [face];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let target = Target::Surface(face);
    match driver.plan(&input, &intent) {
        Err(SchemeError::Infeasible(inf)) => {
            assert_eq!(inf.blame.as_ref().map(|b| (b.0, b.1.as_str())), Some((target, "pave")), "{inf:?}")
        }
        other => panic!("expected Infeasible, got {:?}", other.map(|_| ())),
    }
    let out = mesh_faces(&topo, &model, &[face], &intent, &Sequential).expect("mesh via the fallback");
    let tp = out.plan.target(target).expect("face");
    assert_eq!((tp.scheme.as_str(), &tp.selection), ("tri", &Selection::Intent { rank: 1 }));
    assert_eq!(face_sum(&out.plan, &topo, face), 9, "no parity constraint any more");
    let a: &Attempt = out.attempts.first().expect("history");
    assert_eq!((a.target, a.scheme.as_str(), a.phase, a.kind), (target, "pave", Phase::Intervals, FailureKind::Constraint));
    assert!(out.plan.rejections.contains(target, &a.scheme));
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.cell_count(), 9);
}

#[test]
fn acceptance_8_a_loop_with_sum_below_four_is_infeasible() {
    // The cap's single circle edge: a hard count of 2 is even but a loop needs four nodes.
    let (topo, ids, model) = cylinder(1.0, 1.0);
    let cap = ids.faces[0];
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [cap];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    // A soft scheme choice: the infeasible row is pave's, so pave is blamed (a hard choice
    // is never blamed by the planner, the system is just infeasible).
    for (firmness, blamed) in [(Firmness::Soft, true), (Firmness::Hard, false)] {
        let mut intent = hard(&[(ids.edges[0], 2)]);
        intent.set_scheme(cap, SchemeChoice::new("pave", firmness));
        match driver.plan(&input, &intent) {
            Err(SchemeError::Infeasible(inf)) => {
                let want = blamed.then_some((Target::Surface(cap), "pave"));
                assert_eq!(inf.blame.as_ref().map(|b| (b.0, b.1.as_str())), want, "{firmness:?}: {inf:?}");
            }
            other => panic!("expected Infeasible, got {:?}", other.map(|_| ())),
        }
    }
    // Four is fine: one quad.
    let ok = hard(&[(ids.edges[0], 4)]);
    let out = mesh_faces(&topo, &model, &[cap], &ok, &Sequential).expect("mesh");
    assert_eq!(out.finished.mesh.cell_count(), 1);
}

// ---------------------------------------------------------------------------- item 10

type Hashes = (u64, u64);

fn hashes(out: &sm_scheme_api::Meshed<sm_mesh::Mesh>) -> Hashes {
    (sm_mesh_api::topology_hash(&out.finished.mesh), sm_mesh_api::geometry_hash(&out.finished.mesh))
}

#[test]
fn acceptance_10_executors_give_bit_identical_meshes() {
    let (topo, ids, model) = brick_setup([4.0, 3.0, 1.0]);
    let planar = (ids.faces[5], brick_face_counts(&topo, ids.faces[5], 8, 6));
    let (ctopo, cids, cmodel) = cylinder(3.0, 1.0);
    let lateral = (cids.faces[2], hard(&[(cids.edges[0], 72), (cids.edges[1], 72), (cids.edges[2], 4)]));
    let cap = (cids.faces[0], hard(&[(cids.edges[0], 32)]));
    let cases: [(&sm_topo_api::CellComplex, &sm_model_api::Model, &(EntityId, MeshIntent)); 3] =
        [(&topo, &model, &planar), (&ctopo, &cmodel, &lateral), (&ctopo, &cmodel, &cap)];
    for (t, mo, (face, intent)) in cases {
        let seq = mesh_faces(t, mo, &[*face], intent, &Sequential).expect("sequential");
        let par = mesh_faces(t, mo, &[*face], intent, &ScopedThreads(4)).expect("threads");
        let again = mesh_faces(t, mo, &[*face], intent, &Sequential).expect("again");
        assert_eq!(hashes(&seq), hashes(&par), "{face}: Sequential vs ScopedThreads(4)");
        assert_eq!(hashes(&seq), hashes(&again), "{face}: two runs");
    }
}
