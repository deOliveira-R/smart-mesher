//! Acceptance 12: every `RtError` variant sm-rt can return has a test.
mod common;
use common::*;
use sm_core::{Point3, Vector3};
use sm_geom_api::MemoryStore;
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::SerialAlloc;
use sm_rt::MeshTrackerBuilder;
use sm_rt_api::{
    BoundarySpec, BrepInput, BuildInput, BuildOptions, CellId, ComponentId, ComponentKind, FaceId, InstancePolicy, RegionId, ResponseKind, RtError, Scratch, StepEnd, SurfaceRefine,
    Tracker, TrackerBuilder, WorldSpec,
};
use sm_testkit::topo;

#[test]
fn acc12_unsupported_inputs_name_their_variant() {
    let f = tet_facets();
    let topo = topo::cube().unwrap();
    let geom = MemoryStore::new();
    let spec = BoundarySpec::default();
    let brep = BuildInput::Brep(BrepInput::new(&topo.complex, &geom, &f, SurfaceRefine::None, &spec));
    let world_spec = WorldSpec::new(vec![ComponentKind::Leaf { input: 0 }], ComponentId(0), InstancePolicy::default());
    let world = BuildInput::World(&world_spec);
    let o = BuildOptions::default();
    for (input, name) in [(brep, "Brep"), (world, "World")] {
        for r in [MeshTrackerBuilder.build(&input, &o).map(|_| ()), MeshTrackerBuilder.cache_key(&input, &o).map(|_| ())] {
            match r {
                Err(RtError::Unsupported(m)) => assert!(m.contains(name), "{m}"),
                other => panic!("{name}: {other:?}"),
            }
        }
    }
}

#[test]
fn acc12_open_shell_is_a_build_error_with_located_issues() {
    let mut f = tet_facets();
    f.triangles.pop();
    f.front.pop();
    f.back.pop();
    f.face.pop();
    match build(&f, &BuildOptions::default()) {
        Err(RtError::Build(issues)) => assert!(issues.iter().all(|i| i.code == "facet.open_edge" && i.location.is_some() && i.gap.is_some())),
        other => panic!("{:?}", other.map(|_| ())),
    }
}

fn tracker() -> impl Tracker {
    let mesh = hex_block([2; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    build_mesh_tracker(&mesh, &vacuum(), &opts_fail())
}

#[test]
fn acc12_query_errors_are_typed_and_leave_state_alone() {
    let t = tracker();
    let mut sc = Scratch::new();
    let ok = Point3::new(0.5, 0.5, 0.5);
    for d in [Vector3::new(0.0, 0.0, 0.0), Vector3::new(2.0, 0.0, 0.0), Vector3::new(f64::NAN, 0.0, 0.0), Vector3::new(1.0 + 1e-6, 0.0, 0.0)] {
        assert!(matches!(t.init_state(ok, d, None, &mut sc), Err(RtError::BadDirection { .. })), "{d:?}");
    }
    let unit = Vector3::new(1.0, 0.0, 0.0);
    for p in [Point3::new(f64::NAN, 0.0, 0.0), Point3::new(0.0, f64::INFINITY, 0.0)] {
        assert!(matches!(t.init_state(p, unit, None, &mut sc), Err(RtError::NonFinite(_))));
        assert!(matches!(t.locate(p, None, None, &mut sc), Err(RtError::NonFinite(_))));
        assert!(matches!(t.closest(p, None, &mut sc), Err(RtError::NonFinite(_))));
        assert!(matches!(t.face_normal(FaceId(0), p), Err(RtError::NonFinite(_))));
    }
    assert!(matches!(t.locate(ok, Some(Vector3::new(f64::NAN, 0.0, 0.0)), None, &mut sc), Err(RtError::NonFinite("direction"))));
    assert!(matches!(t.closest(ok, Some(RegionId(77)), &mut sc), Err(RtError::UnknownId { kind: "region", id: 77 })));
    assert!(matches!(t.face_normal(FaceId(9999), ok), Err(RtError::UnknownId { kind: "face", .. })));
    assert!(matches!(t.periodic_image(FaceId(9999)), Err(RtError::UnknownId { kind: "face", .. })));
    let mut st = t.init_state(ok, unit, None, &mut sc).unwrap();
    let before = st.clone();
    for bad in [-1.0, f64::NAN] {
        assert!(matches!(t.step(&mut st, bad, &mut sc), Err(RtError::InvalidInput(_))));
        assert_eq!(st, before);
    }
    st.cell = CellId::new(12345);
    assert!(matches!(t.step(&mut st, 1.0, &mut sc), Err(RtError::UnknownId { kind: "cell", .. })));
    let mut nan = before.clone();
    nan.pos = Point3::new(f64::NAN, 0.0, 0.0);
    assert!(matches!(t.step(&mut nan, 1.0, &mut sc), Err(RtError::NonFinite(_))));
    // Provided methods that report contract errors through the same type.
    let mut s2 = before.clone();
    let end = loop {
        let s = t.step(&mut s2, f64::INFINITY, &mut sc).unwrap();
        if let StepEnd::Boundary { face, .. } = s.end {
            break face;
        }
    };
    assert!(matches!(t.resolve_boundary(&mut s2, end, ResponseKind::Albedo(1.5), [0.0, 0.0]), Err(RtError::BadAlbedo { .. })));
    assert!(matches!(t.resolve_boundary(&mut s2, end, ResponseKind::White, [f64::NAN, 0.0]), Err(RtError::InvalidInput(_))));
    // A cell the tracker never had is not a region.
    assert_eq!(t.cell_region(CellId::new(999)), None);
    assert_eq!(t.cell_volume(CellId::new(999)), None);
    assert!(t.bin_map(&sm_rt_api::TallySpec::MeshSets(vec!["nope".into()])).is_err());
}

#[test]
fn acc12_a_point_outside_is_not_an_error() {
    let t = tracker();
    let mut sc = Scratch::new();
    let st = t.init_state(Point3::new(9.0, 9.0, 9.0), Vector3::new(1.0, 0.0, 0.0), None, &mut sc).unwrap();
    assert!(st.cell.is_none());
}
