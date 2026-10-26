//! Behaviour of the C ABI over the stub `AnalyticWorld`, called as Rust `unsafe extern "C"` fns.
//! (The same checks run from a real C program in `c_abi.rs`.) Acceptance 1-3 semantics.
#![cfg(feature = "stubs")]

mod common;
use common::*;
use sm_rt_capi::ffi::test_hooks::*;
use sm_rt_capi::ffi::*;
use sm_rt_capi::*;

const R: f64 = 5.0;

fn chord(b: f64) -> f64 {
    2.0 * (R * R - b * b).sqrt()
}

#[test]
fn sphere_chords_match_analytic_values() {
    let t = sphere5();
    let c = ctx_of(t);
    for b in [0.0, 3.0, 4.9] {
        assert_eq!(init(c, [-9.0, b, 0.0], [1.0, 0.0, 0.0]), 0);
        let (r, s) = fire(c);
        assert_eq!((r, s.end, s.to_region), (0, SMRT_END_REGION_BOUNDARY, 0), "entering the sphere at b={b}");
        let (r, s) = fire(c);
        assert_eq!((r, s.end, s.to_region), (0, SMRT_END_REGION_BOUNDARY, SMRT_COMPLEMENT));
        assert!((s.dist - chord(b)).abs() < 1e-12, "b={b}: {} vs {}", s.dist, chord(b));
        assert_eq!(s.region, 0);
    }
}

#[test]
fn nested_shell_chords_sum_to_the_outer_chord() {
    let t = stub(STUB_SHELLS, R);
    let c = ctx_of(t);
    for b in [0.0, 0.7, 2.5, 4.3, 4.99] {
        assert_eq!(init(c, [-9.0, b, 0.3], [1.0, 0.0, 0.0]), 0);
        let (_, s) = fire(c); // into the outermost shell
        assert_eq!(s.end, SMRT_END_REGION_BOUNDARY);
        let mut sum = 0.0;
        loop {
            let (_, s) = fire(c);
            assert_eq!(s.end, SMRT_END_REGION_BOUNDARY);
            sum += s.dist;
            if s.to_region == SMRT_COMPLEMENT {
                break;
            }
        }
        let outer = 2.0 * (R * R - b * b - 0.09).sqrt();
        assert!((sum - outer).abs() < 1e-12, "b={b}: {sum} vs {outer}");
    }
}

#[test]
fn mirror_deck_reflects_bit_exactly() {
    let t = stub(STUB_MIRROR_BOX, 0.0);
    let c = ctx_of(t);
    let d = [0.6, 0.0, 0.8];
    assert_eq!(init(c, [0.0, 0.0, 0.0], d), 0);
    let (r, s) = step(c, f64::INFINITY);
    assert_eq!((r, s.end, s.action), (0, SMRT_END_BOUNDARY, SMRT_ACT_MIRROR));
    let st = state(c);
    // z is the nearer wall (10/0.8 = 12.5 vs 10/0.6): the mirror flips z only.
    assert_eq!(st.dir[0].to_bits(), d[0].to_bits());
    assert_eq!(st.dir[2].to_bits(), (-d[2]).to_bits());
    assert_eq!(st.dir[1].to_bits(), 0.0f64.to_bits());
}

/// Drive a particle to a wall of a response box: returns the face id.
fn to_wall(c: *mut SmrtCtx, action: i32) -> i64 {
    assert_eq!(init(c, [1.0, 2.0, 3.0], [1.0, 0.0, 0.0]), 0);
    let (r, s) = step(c, f64::INFINITY);
    assert_eq!((r, s.end, s.action), (0, SMRT_END_BOUNDARY, action));
    s.face
}

#[test]
fn white_boundary_returns_unit_inward_vectors() {
    let t = stub(STUB_WHITE_BOX, 0.0);
    let c = ctx_of(t);
    let face = to_wall(c, SMRT_ACT_WHITE);
    let mut rng = Rng::named("white-1e5");
    let mut out = [0.0f64; 3];
    for _ in 0..100_000 {
        let (u0, u1) = (rng.unit(), rng.unit());
        // SAFETY: live ctx, `out` holds three doubles.
        let r = unsafe { smrt_resolve_boundary(c, face, SMRT_RESP_WHITE, 0.0, u0, u1, out.as_mut_ptr()) };
        assert_eq!(r, 0);
        let n = (out[0] * out[0] + out[1] * out[1] + out[2] * out[2]).sqrt();
        assert!((n - 1.0).abs() < 1e-12, "|d| = {n}");
        assert!(out[0] <= 1e-12, "must point back into the box: {out:?}");
    }
    // Bad uniforms and kinds are errors, not panics.
    // SAFETY: as above.
    assert_eq!(unsafe { smrt_resolve_boundary(c, face, SMRT_RESP_WHITE, 0.0, 2.0, 0.5, out.as_mut_ptr()) }, SMRT_ERR_INVALID_INPUT);
    assert_eq!(unsafe { smrt_resolve_boundary(c, face, 77, 0.0, 0.5, 0.5, out.as_mut_ptr()) }, SMRT_ERR_BAD_ARG);
}

#[test]
fn reflective_albedo_and_vacuum_responses() {
    let t = stub(STUB_REFLECTIVE_BOX, 0.0);
    let c = ctx_of(t);
    let face = to_wall(c, SMRT_ACT_REFLECTIVE);
    let mut out = [0.0f64; 3];
    // SAFETY: live ctx, `out` holds three doubles.
    unsafe {
        assert_eq!(smrt_resolve_boundary(c, face, SMRT_RESP_REFLECTIVE, 0.0, 0.5, 0.5, out.as_mut_ptr()), 0);
        assert_eq!(out, [-1.0, 0.0, 0.0]);
        assert_eq!(smrt_resolve_boundary(c, face, SMRT_RESP_VACUUM, 0.0, 0.5, 0.5, out.as_mut_ptr()), 1);
    }
    let t = stub(STUB_ALBEDO_BOX, 0.0);
    let c = ctx_of(t);
    let face = to_wall(c, SMRT_ACT_ALBEDO);
    let (_, s) = step(c, 0.0);
    assert_eq!(s.end, SMRT_END_BOUNDARY);
    // The step reported the coefficient.
    assert_eq!(to_wall_albedo(c), 0.5);
    // SAFETY: as above.
    unsafe {
        assert_eq!(smrt_resolve_boundary(c, face, SMRT_RESP_ALBEDO, 0.5, 0.25, 0.0, out.as_mut_ptr()), 0);
        assert_eq!(smrt_resolve_boundary(c, face, SMRT_RESP_ALBEDO, 0.5, 0.75, 0.0, out.as_mut_ptr()), 1);
        assert_eq!(smrt_resolve_boundary(c, face, SMRT_RESP_ALBEDO, 1.5, 0.75, 0.0, out.as_mut_ptr()), SMRT_ERR_BAD_ALBEDO);
    }
}

fn to_wall_albedo(c: *mut SmrtCtx) -> f64 {
    init(c, [1.0, 2.0, 3.0], [1.0, 0.0, 0.0]);
    step(c, f64::INFINITY).1.albedo
}

#[test]
fn lattice_chord_sum_and_slot_change_skipping() {
    // Spheres of radius 1 at element centres (2, 6, 10) in a 3x3x3 lattice of pitch 4.
    let t = stub(STUB_LATTICE, 1.0);
    let c = ctx_of(t);
    for (dy, want_each) in [(0.0, 2.0), (0.6, 2.0 * (1.0f64 - 0.36).sqrt())] {
        assert_eq!(init(c, [0.25, 2.0 + dy, 2.0], [1.0, 0.0, 0.0]), 0);
        let (mut sum, mut slots) = (0.0, 0);
        loop {
            let (_, s) = step(c, f64::INFINITY);
            if s.region == 0 {
                sum += s.dist;
            }
            match s.end {
                SMRT_END_SLOT_CHANGE => slots += 1,
                SMRT_END_BOUNDARY => break,
                SMRT_END_LOST => panic!("lost"),
                _ => {}
            }
        }
        assert_eq!(slots, 2, "walls at x = 4 and 8");
        assert!((sum - 3.0 * want_each).abs() < 1e-12, "{sum} vs {}", 3.0 * want_each);
    }
    // From the background of element 0, just before the wall: `step` reports the slot change,
    // `fire` skips it and stops at the next region boundary (the sphere of element 1).
    assert_eq!(init(c, [3.5, 2.0, 2.0], [1.0, 0.0, 0.0]), 0);
    let (_, s) = step(c, f64::INFINITY);
    assert_eq!(s.end, SMRT_END_SLOT_CHANGE);
    assert!((s.dist - 0.5).abs() < 1e-12);
    assert_eq!(state(c).levels_total, 1);
    assert_eq!(init(c, [3.5, 2.0, 2.0], [1.0, 0.0, 0.0]), 0);
    let (r, s) = fire(c);
    assert_eq!((r, s.end, s.to_region), (0, SMRT_END_REGION_BOUNDARY, 0));
    assert!((s.dist - 1.5).abs() < 1e-12, "{}", s.dist);
}

#[test]
fn dag_shims_on_the_sphere() {
    let t = sphere5();
    let c = ctx_of(t);
    let (mut surf, mut dist, mut face) = (0i64, 0f64, 0i64);
    let mut fire_at = |region: i64, o: [f64; 3], orient: i32| {
        let d = [1.0, 0.0, 0.0];
        // SAFETY: live handles, arrays of three doubles, valid out slots.
        let r = unsafe { smrt_dag_ray_fire(t, c, region, o.as_ptr(), d.as_ptr(), orient, 0.0, &mut surf, &mut dist, &mut face) };
        (r, surf, dist)
    };
    // Exiting from inside: distance to the far side.
    assert_eq!(fire_at(0, [-2.0, 0.0, 0.0], SMRT_ORIENT_EXITING), (0, 0, 7.0));
    // Entering from outside.
    let (r, s, d) = fire_at(0, [-9.0, 0.0, 0.0], SMRT_ORIENT_ENTERING);
    assert_eq!((r, s), (0, 0));
    assert!((d - 4.0).abs() < 1e-12);
    // Any, from inside and outside the volume.
    assert_eq!(fire_at(0, [-2.0, 0.0, 0.0], SMRT_ORIENT_ANY), (0, 0, 7.0));
    let (r, s, d) = fire_at(0, [-9.0, 0.0, 0.0], SMRT_ORIENT_ANY);
    assert_eq!((r, s), (0, 0));
    assert!((d - 4.0).abs() < 1e-12);
    // Exiting from a point that is not in the volume: no hit.
    assert_eq!(fire_at(0, [-9.0, 0.0, 0.0], SMRT_ORIENT_EXITING), (SMRT_NO_HIT, SMRT_NONE, 0.0));
    // The complement volume exits through the sphere surface (entering it from the sphere side).
    let (r, s, d) = fire_at(SMRT_COMPLEMENT, [-9.0, 0.0, 0.0], SMRT_ORIENT_EXITING);
    assert_eq!((r, s), (0, 0));
    assert!((d - 4.0).abs() < 1e-12);
    // Bad orientation.
    let o = [0.0, 0.0, 0.0];
    let d = [1.0, 0.0, 0.0];
    // SAFETY: as above.
    assert_eq!(unsafe { smrt_dag_ray_fire(t, c, 0, o.as_ptr(), d.as_ptr(), 5, 0.0, &mut surf, &mut dist, std::ptr::null_mut()) }, SMRT_ERR_BAD_ARG);
}

#[test]
fn dag_point_in_volume_agrees_with_geometry_for_1e5_points() {
    let t = sphere5();
    let c = ctx_of(t);
    let mut rng = Rng::named("pin-1e5");
    let mut checked = 0;
    for _ in 0..100_000 {
        let p = [(rng.unit() - 0.5) * 18.0, (rng.unit() - 0.5) * 18.0, (rng.unit() - 0.5) * 18.0];
        let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
        if (r - R).abs() < 1e-9 {
            continue;
        }
        let (mut in_sphere, mut in_comp) = (-1i32, -1i32);
        // SAFETY: live handles, valid in/out pointers.
        unsafe {
            assert_eq!(smrt_dag_point_in_volume(t, c, 0, p.as_ptr(), std::ptr::null(), &mut in_sphere), 0);
            assert_eq!(smrt_dag_point_in_volume(t, c, SMRT_COMPLEMENT, p.as_ptr(), std::ptr::null(), &mut in_comp), 0);
        }
        assert_eq!(in_sphere, i32::from(r < R), "p={p:?}");
        assert_eq!(in_comp, i32::from(r > R), "p={p:?}");
        checked += 1;
    }
    assert!(checked > 99_000);
}

#[test]
fn surface_sense_is_consistent_with_next_vol() {
    let t = stub(STUB_SHELLS, R);
    // Shell k (surface k) separates region k (inside, forward/front) from region k+1.
    let (mut sense, mut next) = (0i32, 0i64);
    // SAFETY: live handle, valid out slots.
    unsafe {
        for k in 0..4i64 {
            assert_eq!(smrt_dag_surface_sense(t, k, k, &mut sense), 0);
            assert_eq!(sense, 1);
            assert_eq!(smrt_dag_next_vol(t, k, k, &mut next), 0);
            assert_eq!(next, k + 1);
            assert_eq!(smrt_dag_surface_sense(t, k, k + 1, &mut sense), 0);
            assert_eq!(sense, -1);
            assert_eq!(smrt_dag_next_vol(t, k, k + 1, &mut next), 0);
            assert_eq!(next, k);
        }
        // The outermost shell borders the complement, spelled SMRT_COMPLEMENT on output.
        assert_eq!(smrt_dag_next_vol(t, 4, 4, &mut next), 0);
        assert_eq!(next, SMRT_COMPLEMENT);
        assert_eq!(smrt_dag_surface_sense(t, 4, SMRT_COMPLEMENT, &mut sense), 0);
        assert_eq!(sense, -1);
        // A region that does not touch the surface, and an unknown surface.
        assert_eq!(smrt_dag_surface_sense(t, 0, 3, &mut sense), SMRT_NO_HIT);
        assert_eq!(smrt_dag_next_vol(t, 0, 3, &mut next), SMRT_NO_HIT);
        assert_eq!(smrt_dag_next_vol(t, 999, 0, &mut next), SMRT_ERR_UNKNOWN_ID);
    }
}

#[test]
fn locate_closest_state_and_normals() {
    let t = sphere5();
    let c = ctx_of(t);
    let mut loc = SmrtLocated::default();
    // SAFETY: live handles, valid pointers throughout this block.
    unsafe {
        assert_eq!(smrt_locate(c, [1.0, 1.0, 1.0].as_ptr(), std::ptr::null(), -1, &mut loc), 0);
        assert_eq!((loc.cell, loc.region), (0, 0));
        assert_eq!(smrt_locate(c, [9.0, 9.0, 9.0].as_ptr(), std::ptr::null(), -1, &mut loc), 0);
        assert_eq!(loc.region, SMRT_COMPLEMENT);
        assert_eq!(smrt_locate(c, [99.0, 0.0, 0.0].as_ptr(), std::ptr::null(), -1, &mut loc), SMRT_NO_HIT);
        assert_eq!(loc.cell, SMRT_NONE);
        let mut cl = SmrtClosest::default();
        assert_eq!(smrt_closest(c, [2.0, 0.0, 0.0].as_ptr(), SMRT_NONE, &mut cl), 0);
        assert!((cl.dist - 3.0).abs() < 1e-12 && cl.surface == 0, "{cl:?}");
        // Normal of the sphere surface at (5,0,0): outward.
        let mut n = [0.0f64; 3];
        assert_eq!(smrt_dag_get_angle(t, 0, [5.0, 0.0, 0.0].as_ptr(), n.as_mut_ptr()), 0);
        assert!((n[0] - 1.0).abs() < 1e-12 && n[1].abs() < 1e-12 && n[2].abs() < 1e-12, "{n:?}");
    }
    // State round trip, direction change, peek.
    assert_eq!(init(c, [-9.0, 0.0, 0.0], [1.0, 0.0, 0.0]), 0);
    let saved = state(c);
    assert_eq!((saved.cell, saved.region, saved.levels_total), (1, SMRT_COMPLEMENT, 0));
    let mut p = SmrtStep::default();
    // SAFETY: live ctx, valid out slot.
    unsafe {
        assert_eq!(smrt_peek(c, f64::INFINITY, &mut p), 0);
        assert_eq!((p.end, p.surface, p.to_region), (SMRT_END_REGION_BOUNDARY, 0, 0));
        assert!((p.dist - 4.0).abs() < 1e-12);
        assert_eq!(smrt_peek(c, 1.0, &mut p), SMRT_NO_HIT);
        assert_eq!(state(c), saved, "peek must not move the particle");
        assert_eq!(smrt_set_direction(c, 0.0, 1.0, 0.0), 0);
        assert_eq!(smrt_set_direction(c, 2.0, 0.0, 0.0), SMRT_ERR_BAD_DIRECTION);
        assert_eq!(smrt_set_state(c, &saved), 0);
        assert_eq!(state(c), saved);
        let mut bad = saved;
        bad.levels_total = 9;
        bad.levels_len = 8;
        assert_eq!(smrt_set_state(c, &bad), SMRT_ERR_BAD_ARG);
        // A cell id beyond the model is rejected up front, and the state is unchanged.
        let mut far = saved;
        far.cell = 123_456;
        assert_eq!(smrt_set_state(c, &far), SMRT_ERR_UNKNOWN_ID);
        assert_eq!(state(c), saved);
    }
}

#[test]
fn info_region_info_and_measures() {
    let t = sphere5();
    let mut info = SmrtInfo::default();
    // SAFETY: live handle, valid out slots.
    unsafe {
        assert_eq!(smrt_info(t, &mut info), 0);
    }
    assert_eq!((info.cells, info.regions, info.surfaces), (2, 2, 7));
    assert_eq!(info.complement_region, 1);
    assert_eq!((info.bounds_lo, info.bounds_hi), ([-10.0; 3], [10.0; 3]));
    let mut ri = SmrtRegionInfo::default();
    let vol = 4.0 / 3.0 * std::f64::consts::PI * 125.0;
    // SAFETY: as above.
    unsafe {
        assert_eq!(smrt_region_info(t, 0, &mut ri), 0);
        assert!((ri.measure - vol).abs() < 1e-9 && ri.implicit_complement == 0 && ri.color_id == -1);
        assert_eq!(smrt_region_info(t, SMRT_COMPLEMENT, &mut ri), 0);
        assert!((ri.measure - (8000.0 - vol)).abs() < 1e-9 && ri.implicit_complement == 1);
        let mut v = 0.0;
        assert_eq!(smrt_dag_measure_volume(t, 0, &mut v), 0);
        assert!((v - vol).abs() < 1e-9);
        assert_eq!(smrt_dag_measure_area(t, 0, &mut v), SMRT_ERR_UNSUPPORTED);
        assert_eq!(smrt_dag_measure_area(t, 99, &mut v), SMRT_ERR_UNKNOWN_ID);
        assert_eq!(smrt_region_info(t, 17, &mut ri), SMRT_ERR_UNKNOWN_ID);
    }
}

#[test]
fn open_without_a_backend_is_unsupported_and_free_order_is_safe() {
    let mut out: *mut SmrtTracker = std::ptr::null_mut();
    let path = c"model.smrt";
    // SAFETY: valid NUL-terminated path and out slot.
    assert_eq!(unsafe { smrt_open(path.as_ptr(), &mut out) }, SMRT_ERR_UNSUPPORTED);
    assert!(out.is_null());
    assert!(err_msg().contains("smrt_open"));
    assert_eq!(smrt_version(), SMRT_ABI_VERSION);
    // The context keeps the tracker alive: free the tracker first.
    let t = sphere5();
    let c = ctx_of(t);
    // SAFETY: each handle is freed once.
    unsafe {
        assert_eq!(smrt_tracker_free(t), 0);
    }
    assert_eq!(init(c, [0.0; 3], [1.0, 0.0, 0.0]), 0);
    // SAFETY: as above.
    unsafe {
        assert_eq!(smrt_ctx_free(c), 0);
    }
}
