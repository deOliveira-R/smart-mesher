//! Contract-freeze checks: growable types stay constructible from outside the crate, and the
//! input guards added at freeze time hold.
#![cfg(feature = "stubs")]

use sm_core::{Aabb, Point3, Transform, Vector3};
use sm_rt_api::stub::{AnalyticWorld, Shape};
use sm_rt_api::{
    ComponentId, ComponentKind, FacetInput, InstanceId, InstancePolicy, Level, Placement, ResponseKind, RtError, Scratch,
    TrackBundle, Tracker, WorldSpec,
};

#[test]
fn growable_inputs_build_outside_the_crate() {
    let l = Level::new(ComponentId(1), InstanceId(2), [1, 2, 3]);
    assert_eq!(l.lattice, [1, 2, 3]);
    let mut f = FacetInput::default();
    f.vertices.push([0.0, 0.0, 0.0]);
    assert_eq!(f.vertices.len(), 1);
    assert_eq!(TrackBundle::default(), TrackBundle::new());
    assert_eq!(TrackBundle::default().offsets, vec![0]);
}

#[test]
fn negative_max_dist_is_rejected_and_state_unchanged() {
    let w = AnalyticWorld::simple(
        vec![Shape::Sphere { center: Point3::new(0.0, 0.0, 0.0), radius: 1.0 }],
        Aabb { lo: Point3::new(-2.0, -2.0, -2.0), hi: Point3::new(2.0, 2.0, 2.0) },
        ResponseKind::Vacuum,
    )
    .unwrap();
    let mut sc = Scratch::default();
    let mut st = w.init_state(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0), None, &mut sc).unwrap();
    let before = st.clone();
    assert!(matches!(w.step(&mut st, -0.5, &mut sc), Err(RtError::InvalidInput(_))));
    assert_eq!(st, before);
}

#[test]
fn non_finite_placement_is_reported() {
    let mut t = Transform::translation(Vector3::new(0.0, 0.0, 0.0));
    t.t.x = f64::NAN;
    let spec = WorldSpec::new(
        vec![ComponentKind::Union { placements: vec![Placement::new(ComponentId(1), t)] }, ComponentKind::Leaf { input: 0 }],
        ComponentId(0),
        InstancePolicy::Auto,
    );
    match spec.validate(1) {
        Err(RtError::Build(issues)) => assert!(issues.iter().any(|i| i.code == "world.placement_nonfinite"), "{issues:?}"),
        other => panic!("expected a build error, got {other:?}"),
    }
}
