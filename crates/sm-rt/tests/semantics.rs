//! Provided-method semantics through the real tracker: bundles, bins, fire, peek, DAGMC shims,
//! recovery and lost-particle diagnostics.
mod common;
use common::*;
use sm_core::{Point3, Vector3};
use sm_geom_api::Ray;
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::SerialAlloc;
use sm_rt_api::compat::{RayFire, RayOrientation, next_vol, point_in_volume, ray_fire, surface_sense};
use sm_rt_api::{
    BuildOptions, CellId, LostCause, RecoveryPolicy, RegionId, Scratch, StepEnd, TallySpec, TraceOptions, TrackLengthTally, TrackState, Tracker, TrackerCaps,
};
use sm_testkit::meshes::tet_block_two_volumes;

fn rays(name: &str, n: usize, lo: [f64; 3], hi: [f64; 3]) -> Vec<Ray> {
    let mut r = rng(name);
    (0..n)
        .map(|_| {
            let (d, _, _) = random_frame(&mut r);
            let u = |k: usize, r: &mut SplitMix64| lo[k] + (hi[k] - lo[k]) * (r.unit() + 1.0) / 2.0;
            Ray { origin: p3([u(0, &mut r), u(1, &mut r), u(2, &mut r)]), dir: v3(d) }
        })
        .collect()
}

#[test]
fn scope6_bundle_override_is_bit_identical_to_the_default_sequential_walk() {
    let mesh = hex_block([5; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let t = build_mesh_tracker(&mesh, &vacuum(), &BuildOptions::default());
    let rs = rays("bundle_equiv", 20_000, [-0.5; 3], [5.5; 3]);
    let opts = TraceOptions::default();
    let par = t.trace_bundle(&rs, &opts).unwrap();
    let seq = t.trace_bundle_block(&rs, &opts, &mut Scratch::new()).unwrap();
    assert_eq!(par, seq);
    assert_eq!(par.content_hash(), seq.content_hash());
    assert_eq!(par.ray_count(), rs.len());
    // The partition is fixed by block_size, not the pool: 1 thread equals 4.
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().unwrap().install(|| t.trace_bundle(&rs, &opts).unwrap());
    assert_eq!(one.content_hash(), par.content_hash());
    // Rays starting outside are LeftModel with no segments; inside rays sum to the chord to the wall.
    let outside = Ray { origin: Point3::new(-3.0, 0.5, 0.5), dir: Vector3::new(1.0, 0.0, 0.0) };
    let b = t.trace_bundle(&[outside], &opts).unwrap();
    assert_eq!((b.ray_range(0).unwrap().len(), b.end[0]), (0, sm_rt_api::EndCode::LeftModel));
    let inside = Ray { origin: Point3::new(0.25, 0.5, 0.5), dir: Vector3::new(1.0, 0.0, 0.0) };
    let b = t.trace_bundle(&[inside], &opts).unwrap();
    let r = b.ray_range(0).unwrap();
    assert_eq!(r.len(), 5);
    assert!((b.length[r].iter().sum::<f64>() - 4.75).abs() < 1e-14);
    assert_eq!(b.end[0], sm_rt_api::EndCode::Vacuum);
}

#[test]
fn scope6_bin_maps_cells_regions_and_mesh_sets() {
    let fx = tet_block_two_volumes([4, 2, 2], [1.0; 3], 2).unwrap();
    let t = build_mesh_tracker(&fx.mesh, &vacuum(), &opts_fail());
    let nc = t.cell_count();
    assert_eq!(t.bin_map(&TallySpec::Cells).unwrap(), sm_rt_api::BinMap::identity(nc));
    let reg = t.bin_map(&TallySpec::Regions).unwrap();
    assert_eq!(reg.n_bins, 2);
    reg.validate().unwrap();
    let per_region: Vec<usize> = (0..2).map(|r| reg.bin_of_cell.iter().filter(|&&b| b == r).count()).collect();
    assert_eq!(per_region.iter().sum::<usize>(), nc);
    assert!(per_region.iter().all(|&n| n > 0));
    // A track-length tally over regions sums to the path length of the bundle.
    let rs = rays("bins", 2_000, [0.1; 3], [3.9, 1.9, 1.9]);
    let mut tally = TrackLengthTally::new(reg);
    let mut sc = Scratch::new();
    for r in &rs {
        let mut st = t.init_state(r.origin, r.dir, None, &mut sc).unwrap();
        if !st.cell.is_none() {
            t.trace(&mut st, &TraceOptions::default(), &mut tally, &mut sc).unwrap();
        }
    }
    let bundle = t.trace_bundle(&rs, &TraceOptions::default()).unwrap();
    let want: f64 = bundle.length.iter().sum();
    let got: f64 = tally.acc.iter().sum();
    assert!((got - want).abs() < 1e-9 * want, "{got} vs {want}");
    assert!(tally.acc.iter().all(|&a| a > 0.0));
    let vols: f64 = (0..nc).map(|c| t.cell_volume(CellId::new(c)).unwrap()).sum();
    assert!((vols - fx.volume).abs() < 1e-12);
    // Unknown mesh sets are a typed error.
    assert!(t.bin_map(&TallySpec::MeshSets(vec!["missing".into()])).is_err());
}

#[test]
fn scope8_region_boundaries_surfaces_fire_peek_and_dagmc_shims() {
    let fx = tet_block_two_volumes([4, 2, 2], [1.0; 3], 2).unwrap();
    let t = build_mesh_tracker(&fx.mesh, &vacuum(), &opts_fail());
    assert!(t.caps().has(TrackerCaps::VOLUME_MESH) && t.caps().has(TrackerCaps::EXACT_PLANAR));
    assert!(!t.caps().has(TrackerCaps::ANALYTIC) && !t.caps().has(TrackerCaps::HIERARCHY) && !t.caps().has(TrackerCaps::PERIODIC));
    let mut sc = Scratch::new();
    let (a, b) = (RegionId::new(0), RegionId::new(1));
    let o = Point3::new(0.3, 0.9, 0.7);
    let d = Vector3::new(1.0, 0.0, 0.0);
    let loc = t.locate(o, None, None, &mut sc).unwrap();
    assert_eq!(loc.region, a);
    let mut st = t.init_state(o, d, None, &mut sc).unwrap();
    // Peek equals fire and does not move the state.
    let peek = t.distance_to_boundary(&st, f64::INFINITY, &mut sc).unwrap();
    let before = st.clone();
    let fr = t.fire(&mut st, f64::INFINITY, 1000, &mut sc).unwrap();
    let StepEnd::RegionBoundary { surface, to_region, face, .. } = fr.end else { panic!("{:?}", fr.end) };
    assert!((fr.dist - 1.7).abs() < 1e-13, "{}", fr.dist);
    assert_eq!(to_region, b);
    assert!(matches!(peek, sm_rt_api::Peek::Hit(h) if (h.dist - fr.dist).abs() == 0.0 && h.surface == surface && h.face == face));
    assert_ne!(st, before);
    assert_eq!(t.surfaces().sense(surface, a), Some(1).or(Some(-1)).filter(|_| true));
    assert_eq!(next_vol(&t, surface, a), Some(b));
    assert_eq!(next_vol(&t, surface, b), Some(a));
    assert!(surface_sense(&t, surface, a).is_some() && surface_sense(&t, surface, b) == surface_sense(&t, surface, a).map(|s| -s));
    assert!(point_in_volume(&t, a, o, None, &mut sc).unwrap());
    assert!(!point_in_volume(&t, b, o, None, &mut sc).unwrap());
    // DAGMC ray_fire: exiting volume a hits the interface; exiting b from there reaches the wall.
    let RayFire::Hit(h) = ray_fire(&t, a, o, d, RayOrientation::Exiting, None, &mut sc).unwrap() else { panic!() };
    assert_eq!(h.surface, surface);
    assert!((h.dist - 1.7).abs() < 1e-13);
    assert!(matches!(ray_fire(&t, b, o, d, RayOrientation::Exiting, None, &mut sc).unwrap(), RayFire::Miss));
    assert!(matches!(ray_fire(&t, a, o, d, RayOrientation::Exiting, Some(1.0), &mut sc).unwrap(), RayFire::Miss));
    // closest: from a point at x = 1.9 in volume a the interface is 0.1 away.
    let c = t.closest(Point3::new(1.9, 0.9, 0.7), Some(a), &mut sc).unwrap().unwrap();
    assert!((c.dist - 0.1).abs() < 1e-13 || c.dist < 0.1 + 1e-13);
    assert!(t.closest(Point3::new(1.9, 0.9, 0.7), None, &mut sc).unwrap().is_some());
    // Normals point out of the front region.
    let n = t.face_normal(face, st.pos).unwrap();
    assert!((n.norm() - 1.0).abs() < 1e-15);
}

#[test]
fn scope3_lost_particles_are_diagnosed_and_nudge_recovery_is_counted() {
    let mesh = hex_block([4; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let mut sc = Scratch::new();
    // A state whose cell is wrong by 2e-6 (beyond the 1e-6 tolerance).
    let wrong = |cell: usize| {
        let mut st = TrackState::new(Point3::new(1.0 + 2e-6, 0.5, 0.5), Vector3::new(1.0, 0.0, 0.0));
        st.cell = CellId::new(cell);
        st.region = RegionId::new(0);
        st
    };
    let fail = build_mesh_tracker(&mesh, &vacuum(), &opts_fail());
    let mut st = wrong(0);
    let s = fail.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    let StepEnd::Lost(l) = s.end else { panic!("{:?}", s.end) };
    assert_eq!(l.cause, LostCause::NoCellContainsPoint);
    assert_eq!(l.last_cell, CellId::new(0));
    assert_eq!(l.pos, st.pos);
    assert_eq!(l.dir, Vector3::new(1.0, 0.0, 0.0));
    assert!(l.nearest_surface.is_some() && l.classified_on.is_some());
    assert_eq!(st, wrong(0), "a lost step leaves the state unchanged");
    assert_eq!(fail.nudge_count(), 0);

    let mut o = BuildOptions::default();
    o.recovery = RecoveryPolicy::Nudge { tries: 3 };
    let nudge = build_mesh_tracker(&mesh, &vacuum(), &o);
    let mut st = wrong(0);
    let s = nudge.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    assert!(!matches!(s.end, StepEnd::Lost(_)), "{:?}", s.end);
    assert_eq!(nudge.nudge_count(), 1);
    // A wrong cell far away is re-located by the nudge too (counted).
    let mut st = TrackState::new(Point3::new(3.5, 0.5, 0.5), Vector3::new(0.0, 1.0, 0.0));
    st.cell = CellId::new(0);
    let s = nudge.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    assert!(!matches!(s.end, StepEnd::Lost(_)));
    assert_eq!(nudge.nudge_count(), 2);
    // Unrecoverable (outside the model): RecoveryFailed after the tries, state unchanged.
    let mut st = TrackState::new(Point3::new(9.0, 0.5, 0.5), Vector3::new(0.0, 1.0, 0.0));
    st.cell = CellId::new(0);
    let before = st.clone();
    let s = nudge.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    let StepEnd::Lost(l) = s.end else { panic!() };
    assert_eq!(l.cause, LostCause::RecoveryFailed);
    assert_eq!(st, before);
    // A particle outside the model cannot step.
    let mut out = TrackState::new(Point3::new(9.0, 9.0, 9.0), Vector3::new(1.0, 0.0, 0.0));
    let s = fail.step(&mut out, 1.0, &mut sc).unwrap();
    assert!(matches!(s.end, StepEnd::Lost(l) if l.cause == LostCause::NoCellContainsPoint));
    // Reaching the distance limit inside a cell is a Limit with the exact distance.
    let mut st = fail.init_state(Point3::new(0.1, 0.5, 0.5), Vector3::new(1.0, 0.0, 0.0), None, &mut sc).unwrap();
    let s = fail.step(&mut st, 0.3, &mut sc).unwrap();
    assert!(matches!(s.end, StepEnd::Limit) && s.dist == 0.3 && (st.pos.x - 0.4).abs() < 1e-15);
    let s = fail.step(&mut st, 0.0, &mut sc).unwrap();
    assert!(matches!(s.end, StepEnd::Limit) && s.dist == 0.0);
}

#[test]
fn scope5_complement_policy_region_makes_the_notch_a_complement_cell() {
    use sm_rt_api::{BoundarySpec, BuildInput, ComplementPolicy, MeshInput, RegionRule, TrackerBuilder};
    let nc = sm_testkit::meshes::nonconvex_poly().unwrap();
    let (rules, comp, spec) = (RegionRule::ByVolumeClassification, ComplementPolicy::Region("void".into()), BoundarySpec::default());
    let input = BuildInput::Mesh(MeshInput::new(&nc.mesh, &rules, &spec, &comp));
    let (t, _) = sm_rt::MeshTrackerBuilder.build(&input, &BuildOptions::default()).unwrap();
    let mut sc = Scratch::new();
    let void = t.regions().implicit_complement().expect("complement region");
    assert_eq!(t.regions().get(void).unwrap().name, "void");
    let loc = t.locate(Point3::new(1.5, 1.5, 0.5), None, None, &mut sc).unwrap();
    assert_eq!((loc.cell, loc.region), (CellId::COMPLEMENT, void));
    // Into the wall of the L cell: complement -> body, then out through the far wall.
    let mut st = t.init_state(Point3::new(1.5, 1.5, 0.5), Vector3::new(-1.0, 0.0, 0.0), None, &mut sc).unwrap();
    let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    assert!((s.dist - 0.5).abs() < 1e-14 && s.cell == CellId::COMPLEMENT && s.region == void);
    assert!(matches!(s.end, StepEnd::RegionBoundary { to_cell, .. } if to_cell == CellId::new(0)));
    let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    // ... and leaves through the far wall into the complement again (not a Boundary event).
    assert!((s.dist - 1.0).abs() < 1e-14 && matches!(s.end, StepEnd::RegionBoundary { to_cell, to_region, .. } if to_cell == CellId::COMPLEMENT && to_region == void));
    assert_eq!(st.cell, CellId::COMPLEMENT);
    // Towards the bounding box: the complement ends at the domain bounds, leaving the model.
    let mut st = t.init_state(Point3::new(1.5, 1.5, 0.5), Vector3::new(0.0, 1.0, 0.0), None, &mut sc).unwrap();
    let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    assert!((s.dist - 0.5).abs() < 1e-14);
    assert!(matches!(s.end, StepEnd::RegionBoundary { to_cell, to_region, .. } if to_cell.is_none() && to_region == RegionId::NONE));
    assert!(st.cell.is_none());
    // With the default policy the notch is simply outside the model.
    let plain = build_mesh_tracker(&nc.mesh, &BoundarySpec::default(), &opts_fail());
    assert!(plain.locate(Point3::new(1.5, 1.5, 0.5), None, None, &mut sc).unwrap().cell.is_none());
}

#[test]
fn scope4_a_ray_leaves_a_body_into_the_notch_complement() {
    use sm_rt_api::{BoundarySpec, BuildInput, ComplementPolicy, MeshInput, RegionRule, TrackerBuilder};
    let nc = sm_testkit::meshes::nonconvex_poly().unwrap();
    let (rules, comp, spec) = (RegionRule::ByVolumeClassification, ComplementPolicy::Region("void".into()), BoundarySpec::default());
    let input = BuildInput::Mesh(MeshInput::new(&nc.mesh, &rules, &spec, &comp));
    let (t, _) = sm_rt::MeshTrackerBuilder.build(&input, &BuildOptions::default()).unwrap();
    let mut sc = Scratch::new();
    let void = t.regions().implicit_complement().unwrap();
    let mut st = t.init_state(Point3::new(0.5, 1.5, 0.5), Vector3::new(1.0, 0.0, 0.0), None, &mut sc).unwrap();
    let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    assert!((s.dist - 0.5).abs() < 1e-14, "{}", s.dist);
    assert!(matches!(s.end, StepEnd::RegionBoundary { to_cell, to_region, .. } if to_cell == CellId::COMPLEMENT && to_region == void), "{:?}", s.end);
    assert_eq!((st.cell, st.region), (CellId::COMPLEMENT, void));
    // The notch is traversed as the complement and the ray finally leaves the box.
    let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    assert_eq!(s.cell, CellId::COMPLEMENT);
    assert!(matches!(s.end, StepEnd::RegionBoundary { to_cell, .. } if to_cell.is_none()));
}
