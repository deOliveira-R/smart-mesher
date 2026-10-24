//! W0f gaps4: clusters (fills at explicit transforms, nestable inside any fill).

use sm_core::{EntityId, EntityKind, Point3, Transform, Vector3};
use sm_model_api::*;
use sm_topo_api::CellComplex;
use sm_topo_api::OpId;
use sm_topo_api::fixtures::{self, with_op};
use std::collections::BTreeMap;

fn recolor(from: EntityId, to: EntityId) -> ColorMap {
    ColorMap { volume: BTreeMap::from([(from, to)]), face: BTreeMap::new() }
}

fn member(fill: Fill, at: Transform) -> ClusterMember {
    ClusterMember::new(fill, at)
}

fn rot_z(deg: f64) -> Transform {
    let (s, c) = deg.to_radians().sin_cos();
    Transform { m: [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], t: Vector3::new(0.0, 0.0, 0.0) }
}

fn path(steps: &[PathStep]) -> InstancePath {
    steps.iter().copied().collect()
}

/// TRISO particles in a pebble, pebbles in a core: clusters nested in clusters, flattened in
/// member order with composed placements and the slot-like recolor order.
#[test]
fn nested_clusters_flatten_in_member_order() {
    let mut t = CellComplex::new();
    let kernel = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [0.1, 0.1, 0.1])).unwrap();
    let mut m = Model::new();
    let fuel = m.add_color(1, "fuel", Domain::Volume).unwrap();
    let poison = m.add_color(2, "poison", Domain::Volume).unwrap();
    m.color_volumes(fuel, &[kernel.volume]).unwrap();
    let triso = m.add_prototype(&[kernel.volume]).unwrap();
    let tr = |x: f64| Transform::translation(Vector3::new(x, 0.0, 0.0));
    let mut odd = member(Fill::Prototype(triso), tr(2.0));
    odd.recolor = recolor(fuel, poison);
    let pebble = m
        .add_cluster(Cluster::new(vec![member(Fill::Prototype(triso), tr(1.0)), odd, member(Fill::Void, tr(9.0)), member(Fill::Prototype(triso), tr(3.0))]))
        .unwrap();
    assert_eq!(pebble.kind(), EntityKind::Cluster);
    let core = m
        .add_cluster(Cluster::new(vec![member(Fill::Cluster(pebble), Transform::translation(Vector3::new(0.0, 10.0, 0.0))), member(Fill::Cluster(pebble), rot_z(90.0))]))
        .unwrap();
    let inst = m.add_instance(InstanceRow::new(Fill::Cluster(core), Transform::translation(Vector3::new(0.0, 0.0, 5.0)))).unwrap();
    let flat = flatten(&m).unwrap();
    assert_eq!(flat.entries.len(), 6, "2 pebbles x 3 particles (the void member is skipped)");
    let mut sorted = flat.paths.clone();
    sorted.sort();
    assert_eq!(sorted, flat.paths, "paths ascend");
    let p = |ci: u32, pi: u32| path(&[PathStep::Instance(inst), PathStep::Member { cluster: core, ix: ci }, PathStep::Member { cluster: pebble, ix: pi }]);
    assert_eq!(flat.paths, vec![p(0, 0), p(0, 1), p(0, 3), p(1, 0), p(1, 1), p(1, 3)]);
    // Placement: instance, then the core member, then the pebble member.
    let at = |f: &FlatModel, q: &InstancePath| f.placement_of(&f.entries[f.find(q).unwrap().start]).transform;
    let o = Point3::new(0.0, 0.0, 0.0);
    assert!((at(&flat, &p(0, 3)).apply_point(o) - Point3::new(3.0, 10.0, 5.0)).norm() < 1e-12);
    assert!((at(&flat, &p(1, 1)).apply_point(o) - Point3::new(0.0, 2.0, 5.0)).norm() < 1e-12, "rotated pebble");
    // Recolor: the member map applies to its subtree only.
    let color = |f: &FlatModel, q: &InstancePath| f.entries[f.find(q).unwrap().start].color;
    assert_eq!(color(&flat, &p(0, 1)), poison);
    assert_eq!(color(&flat, &p(1, 1)), poison);
    assert_eq!(color(&flat, &p(0, 0)), fuel);
    // An outer member's recolor wins over the inner one (slot-like order).
    let mut c = m.instancing().clusters[&core].clone();
    c.members[1].recolor = recolor(poison, fuel);
    m.set_cluster(core, c).unwrap();
    let flat = flatten(&m).unwrap();
    assert_eq!((color(&flat, &p(0, 1)), color(&flat, &p(1, 1))), (poison, fuel));
    // A path override reaches a member.
    m.add_path_override(PathOverride::new(path(&[PathStep::Instance(inst), PathStep::Member { cluster: core, ix: 0 }]), recolor(fuel, poison))).unwrap();
    let flat = flatten(&m).unwrap();
    assert_eq!(color(&flat, &p(0, 0)), poison);
    assert_eq!(m.resolve_face_color(&p(0, 0), kernel.faces[0]).unwrap(), None);
    m.check_integrity().unwrap();
    let r = validate(&m, &t, Tolerance::default());
    assert!(r.issues.iter().all(|i| i.code != IssueCode::InstanceCycle && !(i.code == IssueCode::UnknownEntity && i.entities.iter().any(|e| e.kind() == EntityKind::Cluster))), "{:#?}", r.issues);
    // Deterministic.
    assert_eq!(flatten(&m).unwrap().fingerprint(), flat.fingerprint());
}

#[test]
fn clusters_reject_cycles_and_refuse_removal_while_used() {
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3])).unwrap();
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[b.volume]).unwrap();
    let proto = m.add_prototype(&[b.volume]).unwrap();
    let inner = m.add_cluster(Cluster::new(vec![member(Fill::Prototype(proto), Transform::IDENTITY)])).unwrap();
    let outer = m.add_cluster(Cluster::new(vec![member(Fill::Cluster(inner), Transform::IDENTITY)])).unwrap();
    // A rotated lattice slot is a one-member cluster.
    let lat = m
        .add_lattice(Lattice::new(Layout::Cartesian { n: [2, 1, 1], pitch: Vector3::new(2.0, 2.0, 2.0) }, Transform::IDENTITY, Fill::Prototype(proto), BTreeMap::from([(SlotIx::new(1, 0, 0), Slot::new(Fill::Cluster(inner)))])))
        .unwrap();
    // inner -> outer -> inner is a cycle, and so is inner -> lattice -> inner.
    assert!(matches!(m.set_cluster(inner, Cluster::new(vec![member(Fill::Cluster(outer), Transform::IDENTITY)])), Err(ModelError::Cycle(_))));
    assert!(matches!(m.set_cluster(inner, Cluster::new(vec![member(Fill::Lattice(lat), Transform::IDENTITY)])), Err(ModelError::Cycle(_))));
    assert_eq!(m.instancing().clusters[&inner].members[0].fill, Fill::Prototype(proto), "unchanged on error");
    // Unknown fills, wrong clip kinds and non-finite placements are refused.
    assert!(m.add_cluster(Cluster::new(vec![member(Fill::Cluster(EntityId::new(EntityKind::Cluster, 99).unwrap()), Transform::IDENTITY)])).is_err());
    let mut bad = member(Fill::Prototype(proto), Transform::IDENTITY);
    bad.clip = Some(proto);
    assert!(m.add_cluster(Cluster::new(vec![bad])).is_err());
    assert!(m.add_cluster(Cluster::new(vec![member(Fill::Prototype(proto), Transform::translation(Vector3::new(f64::NAN, 0.0, 0.0)))])).is_err());
    // In use: the prototype by a member, inner by outer and by the lattice slot.
    assert!(matches!(m.remove_prototype(proto), Err(ModelError::InUse(_))));
    assert!(matches!(m.remove_cluster(inner), Err(ModelError::InUse(_))));
    let inst = m.add_instance(InstanceRow::new(Fill::Cluster(outer), Transform::IDENTITY)).unwrap();
    assert!(matches!(m.remove_cluster(outer), Err(ModelError::InUse(_))));
    assert_eq!(flatten(&m).unwrap().entries.len(), 1);
    m.remove_instance(inst).unwrap();
    m.remove_cluster(outer).unwrap();
    m.remove_lattice(lat).unwrap();
    m.remove_cluster(inner).unwrap();
    m.remove_prototype(proto).unwrap();
    m.check_integrity().unwrap();
    // A dead clip volume is reported by validate.
    let proto = m.add_prototype(&[b.volume]).unwrap();
    let mut clipped = member(Fill::Prototype(proto), Transform::IDENTITY);
    clipped.clip = Some(EntityId::new(EntityKind::TopoVolume, 77).unwrap());
    let c = m.add_cluster(Cluster::new(vec![clipped])).unwrap();
    let r = validate(&m, &t, Tolerance::default());
    assert!(r.issues.iter().any(|i| i.code == IssueCode::UnknownEntity && i.entities.contains(&c)), "{:#?}", r.issues);
}

#[test]
fn clusters_are_hashed_only_when_present() {
    let tol = sm_geom_api::ModelTolerance::default();
    let words = |m: &Model| {
        let mut v = Vec::new();
        m.canonical_visit(&tol, &mut |w| v.push(format!("{w:?}")));
        v
    };
    let mut a = Model::new();
    let mut b = Model::new();
    let empty = words(&a);
    let ca = a.add_cluster(Cluster::default()).unwrap();
    let cb = b.add_cluster(Cluster::default()).unwrap();
    assert_eq!(ca, cb);
    assert_eq!(words(&a), words(&b));
    assert_ne!(words(&a), empty);
    a.remove_cluster(ca).unwrap();
    // Only the id allocator remembers it: one extra (kind, next) pair.
    assert_eq!(words(&a).len(), empty.len() + 2);
}

#[cfg(feature = "serde")]
#[test]
fn clusters_round_trip_and_old_instancing_still_loads() {
    let mut m = Model::new();
    let c = m.add_cluster(Cluster::new(vec![member(Fill::Void, rot_z(30.0))])).unwrap();
    m.add_instance(InstanceRow::new(Fill::Cluster(c), Transform::IDENTITY)).unwrap();
    let ins = m.instancing().clone();
    let s = serde_json::to_string(&ins).unwrap();
    let back: Instancing = serde_json::from_str(&s).unwrap();
    assert_eq!(back, ins);
    // Data written before clusters existed has no `clusters` key.
    let mut old = serde_json::to_value(&ins).unwrap();
    old.as_object_mut().unwrap().remove("clusters");
    let v: Instancing = serde_json::from_value(old).unwrap();
    assert!(v.clusters.is_empty());
    assert_eq!(v.instances, ins.instances);
    let step: PathStep = serde_json::from_str(&serde_json::to_string(&PathStep::Member { cluster: c, ix: 3 }).unwrap()).unwrap();
    assert_eq!(step, PathStep::Member { cluster: c, ix: 3 });
}
