//! Commit rules of the ledger, schedule conflicts and the reference interval solver.

use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh_api::Param;
use sm_scheme_api::{
    ConstraintKind, DriverError, Firmness, Footprint, IntervalConstraint, IntervalSolver, JobId, JobKind, Ledger, MeshJob, MeshProblem, NodeKey, Origin, PatchBuilder, PatchError, ReferenceIntervalSolver, Relation, SchemeSpec, Target, Term,
};
use sm_topo_api::fixtures::{brick, with_op};
use sm_topo_api::{CellComplex, OpId, TopologyViewExt};
use std::collections::BTreeMap;

fn id(k: EntityKind, s: u64) -> EntityId {
    EntityId::new(k, s).expect("id")
}

fn job(i: u32, reads: &[Target], writes: &[Target]) -> MeshJob {
    MeshJob {
        id: JobId(i),
        kind: JobKind::Scheme(SchemeSpec::named("x")),
        targets: writes.to_vec(),
        group: None,
        footprint: Footprint::default().read(reads.iter().copied()).write(writes.iter().copied()),
        roles: BTreeMap::new(),
    }
}

#[test]
fn commit_rejects_bad_patches_atomically() {
    let mut cc = CellComplex::new();
    let ids = with_op(&mut cc, OpId(1), |t| brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])).expect("brick");
    let e = ids.edges[0];
    let (s, t) = cc.edge_ends(e).expect("ends");
    let mut ledger = Ledger::new();
    ledger.seed_vertex(s, Point3::new(0.0, 0.0, 0.0));
    ledger.seed_vertex(t, Point3::new(1.0, 0.0, 0.0));
    let j = job(0, &[], &[Target::Topo(e)]);

    // A node owned by a target outside the footprint.
    let mut b = PatchBuilder::new();
    let a = b.reference(NodeKey::vertex(s));
    let m = b.new_node(NodeKey::edge(ids.edges[1], 1), Point3::new(0.5, 0.0, 0.0), Param::None);
    let z = b.reference(NodeKey::vertex(t));
    b.curve(Target::Topo(e), &[a, m, z]);
    assert_eq!(ledger.commit(&j, &b.finish(), &cc), Err(PatchError::OutsideFootprint(Target::Topo(ids.edges[1]))));
    assert_eq!(ledger.node_count(), 2, "nothing applied");

    // An edge mesh that does not end at the end vertex.
    let mut b = PatchBuilder::new();
    let a = b.reference(NodeKey::vertex(s));
    let m = b.new_node(NodeKey::edge(e, 1), Point3::new(0.5, 0.0, 0.0), Param::Curve(0.5));
    b.curve(Target::Topo(e), &[a, m]);
    assert_eq!(ledger.commit(&j, &b.finish(), &cc), Err(PatchError::EdgeEnds(Target::Topo(e))));
    assert_eq!(ledger.node_count(), 2, "nothing applied");

    // A missing write target.
    assert_eq!(ledger.commit(&j, &PatchBuilder::new().finish(), &cc), Err(PatchError::Missing(Target::Topo(e))));

    // A good patch, then the same target again.
    let good = {
        let mut b = PatchBuilder::new();
        let a = b.reference(NodeKey::vertex(s));
        let m = b.new_node(NodeKey::edge(e, 1), Point3::new(0.5, 0.0, 0.0), Param::Curve(0.5));
        let z = b.reference(NodeKey::vertex(t));
        b.curve(Target::Topo(e), &[a, m, z]);
        b.finish()
    };
    ledger.commit(&j, &good, &cc).expect("commit");
    assert_eq!(ledger.curve(&Target::Topo(e)).map(|c| c.len()), Some(3));
    assert!(matches!(ledger.commit(&j, &good, &cc), Err(PatchError::DuplicateNode(_) | PatchError::AlreadyWritten(_))));
}

fn problem(jobs: Vec<MeshJob>) -> MeshProblem {
    MeshProblem { jobs, constraints: vec![], vars: vec![], vertices: vec![], vertex_types: BTreeMap::new(), impositions: BTreeMap::new(), deck_checks: vec![], attempts: vec![] }
}

#[test]
fn schedule_levels_and_conflicts() {
    let (e1, e2, f) = (Target::Topo(id(EntityKind::TopoEdge, 1)), Target::Topo(id(EntityKind::TopoEdge, 2)), Target::Topo(id(EntityKind::TopoFace, 1)));
    let p = problem(vec![job(0, &[], &[e1]), job(1, &[], &[e2]), job(2, &[e1, e2], &[f])]);
    assert_eq!(p.schedule().expect("schedule").waves, vec![vec![JobId(0), JobId(1)], vec![JobId(2)]]);
    let p = problem(vec![job(0, &[], &[e1]), job(1, &[], &[e1])]);
    assert!(matches!(p.schedule(), Err(DriverError::WriteConflict { .. })));
    let p = problem(vec![job(0, &[e2], &[e1])]);
    assert!(matches!(p.schedule(), Err(DriverError::MissingWriter { .. })));
    let p = problem(vec![job(0, &[e2], &[e1]), job(1, &[e1], &[e2])]);
    assert_eq!(p.schedule(), Err(DriverError::Cycle));
}

#[test]
fn reference_solver_classes_goals_parity() {
    let e: Vec<EntityId> = (1..=4).map(|s| id(EntityKind::TopoEdge, s)).collect();
    let o = Origin::DefaultSize;
    let mut cs = vec![
        IntervalConstraint::sums_equal(&[e[0]], &[e[1]], Firmness::Hard, o.clone()),
        IntervalConstraint::goal(e[0], 3.0, Firmness::Soft, o.clone()),
        IntervalConstraint::goal(e[1], 4.0, Firmness::Soft, o.clone()),
        IntervalConstraint::goal(e[2], 2.0, Firmness::Default, o.clone()),
    ];
    let sol = ReferenceIntervalSolver.solve(&e, &cs).expect("solve");
    // Class {e0, e1}: mean of 3 and 4 rounds to 4; e2 = 2; e3 has no goal = 1.
    assert_eq!((sol.get(e[0]), sol.get(e[1]), sol.get(e[2]), sol.get(e[3])), (Some(4), Some(4), Some(2), Some(1)));
    // e0 + e2 + e3 = 7 is odd: the lowest class occurring an odd number of times is bumped.
    cs.push(IntervalConstraint { kind: ConstraintKind::Even { edges: vec![e[0], e[2], e[3]] }, firmness: Firmness::Hard, origin: o.clone() });
    let sol = ReferenceIntervalSolver.solve(&e, &cs).expect("solve");
    assert_eq!((sol.get(e[0]), sol.get(e[1])), (Some(5), Some(5)));
    assert!(cs.iter().all(|c| c.holds(&sol)));
    // A constraint the reference solver cannot meet is reported, not ignored.
    cs.push(IntervalConstraint { kind: ConstraintKind::Linear { terms: [Term { edge: e[3], coeff: 1 }].into_iter().collect(), rel: Relation::Ge, rhs: 9 }, firmness: Firmness::Hard, origin: o });
    assert!(ReferenceIntervalSolver.solve(&e, &cs).is_err());
}
