//! Tests of the contract through the analytic stub: DAGMC-style queries and exact chords.

use sm_core::{Point3, Vector3};
use sm_rt_api_cand_dagmc_compat::*;
use std::ops::ControlFlow;

fn sphere(r: f64, parent: Option<u32>, boundary: Boundary) -> SolidSpec {
    SolidSpec { shape: AnalyticShape::Sphere { center: Point3::new(0.0, 0.0, 0.0), radius: r }, parent, tag: 0, boundary }
}

/// Vacuum box [-5,5]^3 (vol 0) holding sphere r=2 (vol 1) holding sphere r=1 (vol 2). Complement is vol 3.
fn world(box_bc: Boundary) -> AnalyticWorld {
    let spec = AnalyticSpec {
        solids: vec![
            SolidSpec {
                shape: AnalyticShape::Box { lo: Point3::new(-5.0, -5.0, -5.0), hi: Point3::new(5.0, 5.0, 5.0) },
                parent: None,
                tag: 1,
                boundary: box_bc,
            },
            sphere(2.0, Some(0), Boundary::Interior),
            sphere(1.0, Some(1), Boundary::Interior),
        ],
    };
    AnalyticWorld::new(&spec, &BuildParams::default()).unwrap()
}

fn unit(x: f64, y: f64, z: f64) -> Vector3 {
    let n = (x * x + y * y + z * z).sqrt();
    Vector3::new(x / n, y / n, z / n)
}

fn segments(w: &AnalyticWorld, start: VolHandle, o: Point3, d: Vector3) -> (Vec<Segment>, TraceOutcome) {
    let mut segs = Vec::new();
    let mut h = RayHistory::new();
    let out = w
        .trace(start, o, d, TraceOptions::default(), &mut h, &mut |s| {
            segs.push(*s);
            ControlFlow::Continue(())
        })
        .unwrap();
    (segs, out)
}

#[test]
fn chord_through_sphere_is_exact() {
    let w = world(Boundary::Vacuum);
    // Impact parameters from 0 up to almost tangent, along oblique directions.
    for (k, dir) in [unit(1.0, 0.0, 0.0), unit(1.0, 2.0, -0.5), unit(-0.3, 0.7, 0.9)].into_iter().enumerate() {
        // Any unit vector perpendicular to dir.
        let a = if dir.x.abs() < 0.9 { Vector3::new(1.0, 0.0, 0.0) } else { Vector3::new(0.0, 1.0, 0.0) };
        let perp = {
            let c = dir.cross(a);
            c * (1.0 / c.norm())
        };
        for i in 0..200 {
            let b = 1.999 * f64::from(i) / 199.0;
            let o = Point3::new(0.0, 0.0, 0.0) + perp * b + dir * -4.0;
            let (segs, out) = segments(&w, VolHandle(0), o, dir);
            let shell = 2.0 * ((4.0 - b * b).sqrt() - if b < 1.0 { (1.0 - b * b).sqrt() } else { 0.0 });
            let core = if b < 1.0 { 2.0 * (1.0 - b * b).sqrt() } else { 0.0 };
            let in_vol = |v: u32| segs.iter().filter(|s| s.volume == VolHandle(v)).map(Segment::length).sum::<f64>();
            assert!((in_vol(2) - core).abs() < 1e-12, "core k={k} b={b}");
            assert!((in_vol(1) - shell).abs() < 1e-12, "shell k={k} b={b}");
            // Segments tile the path with no gaps, and the ray ends on the vacuum box wall.
            for p in segs.windows(2) {
                assert_eq!(p[0].t1, p[1].t0);
            }
            assert!(matches!(out.end, TraceEnd::Boundary { kind: Boundary::Vacuum, .. }));
            assert_eq!(out.volume, VolHandle(0));
        }
    }
}

#[test]
fn dagmc_style_queries() {
    let w = world(Boundary::Vacuum);
    let (ic, shell, core) = (w.implicit_complement(), VolHandle(1), VolHandle(2));
    assert_eq!(ic, VolHandle(3));
    // ray_fire from the core center hits the r=1 sphere (surface 7) at distance 1.
    let o = Point3::new(0.0, 0.0, 0.0);
    let d = Vector3::new(0.0, 0.0, 1.0);
    let mut h = RayHistory::new();
    let hit = w.ray_fire(core, o, d, FireOptions::default(), Some(&mut h)).unwrap().unwrap();
    assert!((hit.dist - 1.0).abs() < 1e-15);
    assert_eq!(h.len(), 1);
    // Sense and next_vol: the r=1 sphere's normal points out of the core into the shell.
    assert_eq!(w.surface_sense(core, hit.surface).unwrap(), SurfSense::Forward);
    assert_eq!(w.surface_sense(shell, hit.surface).unwrap(), SurfSense::Reverse);
    assert_eq!(w.next_vol(hit.surface, core).unwrap(), shell);
    assert_eq!(w.surface_sense(ic, hit.surface), Err(RtError::NotIncident { surface: hit.surface, volume: ic }));
    // Restart on the surface with the history: the next hit is the r=2 sphere, not the same facet.
    let o2 = o + d * hit.dist;
    let hit2 = w.ray_fire(shell, o2, d, FireOptions::default(), Some(&mut h)).unwrap().unwrap();
    assert!((hit2.dist - 1.0).abs() < 1e-15 && hit2.surface != hit.surface);
    // Without the history a sphere facet at distance ~0 would be excluded only by orientation; Entering sees it.
    let ent = FireOptions { orientation: RayOrientation::Entering, ..Default::default() };
    let back = w.ray_fire(shell, o2, d, ent, None).unwrap();
    assert!(back.is_none() || back.is_some_and(|b| b.dist > 0.0));
    // dist_limit.
    let lim = FireOptions { dist_limit: Some(0.5), ..Default::default() };
    assert!(w.ray_fire(core, o, d, lim, None).unwrap().is_none());
    // Fire from outside everything toward the world: the complement is unbounded, so a ray
    // pointing away gets no hit, a ray pointing in hits the box (entering the box = exiting ic).
    let far = Point3::new(0.0, 0.0, 20.0);
    assert!(w.ray_fire(ic, far, d, FireOptions::default(), None).unwrap().is_none());
    let hit3 = w.ray_fire(ic, far, -d, FireOptions::default(), None).unwrap().unwrap();
    assert!((hit3.dist - 15.0).abs() < 1e-12);
    // point_in_volume partitions space; boundary points follow the direction.
    for (p, v) in [
        (Point3::new(0.0, 0.0, 0.5), core),
        (Point3::new(0.0, 0.0, 1.5), shell),
        (Point3::new(4.0, 0.0, 0.0), VolHandle(0)),
        (Point3::new(9.0, 0.0, 0.0), ic),
    ] {
        assert_eq!(w.find_volume(p, None).unwrap(), v);
        assert!(w.point_in_volume(v, p, None, None).unwrap());
    }
    let on = Point3::new(0.0, 0.0, 1.0);
    assert!(w.point_in_volume(core, on, Some(Vector3::new(0.0, 0.0, -1.0)), None).unwrap());
    assert!(w.point_in_volume(shell, on, Some(Vector3::new(0.0, 0.0, 1.0)), None).unwrap());
    // closest_to_location, get_angle, measures.
    let c = w.closest_to_location(shell, Point3::new(0.0, 0.0, 1.25)).unwrap();
    assert!((c.dist - 0.25).abs() < 1e-15);
    let n = w.get_angle(hit.surface, on, None).unwrap();
    assert_eq!(n, Vector3::new(0.0, 0.0, 1.0));
    let pi = std::f64::consts::PI;
    assert!((w.measure_volume(core).unwrap() - 4.0 / 3.0 * pi).abs() < 1e-12);
    assert!((w.measure_volume(shell).unwrap() - 4.0 / 3.0 * pi * 7.0).abs() < 1e-12);
    assert!((w.measure_volume(VolHandle(0)).unwrap() - (1000.0 - 4.0 / 3.0 * pi * 8.0)).abs() < 1e-9);
    assert!((w.measure_area(hit.surface).unwrap() - 4.0 * pi).abs() < 1e-12);
    assert_eq!(w.measure_volume(ic), Err(RtError::Unbounded(ic)));
    // Input errors are typed.
    assert_eq!(w.ray_fire(core, o, Vector3::new(0.0, 0.0, 2.0), FireOptions::default(), None), Err(RtError::NotUnit(2.0)));
    assert_eq!(w.measure_area(SurfHandle(999)), Err(RtError::BadSurface(SurfHandle(999))));
}

#[test]
fn boundaries_and_distance_limit() {
    let w = world(Boundary::Reflecting);
    let o = Point3::new(-4.0, 0.5, 0.0);
    let d = unit(1.0, 0.0, 0.0);
    let (_, out) = segments(&w, VolHandle(0), o, d);
    let TraceEnd::Boundary { surface, kind, t } = out.end else { panic!("{:?}", out.end) };
    assert_eq!(kind, Boundary::Reflecting);
    let pos = o + d * t;
    match w.apply_boundary(surface, VolHandle(0), pos, unit(1.0, 1.0, 0.0)).unwrap() {
        BoundaryAction::Reflect { dir } => assert!((dir - unit(-1.0, 1.0, 0.0)).norm() < 1e-15),
        a => panic!("{a:?}"),
    }
    // max_dist stops inside a volume and the partial segment is clipped.
    let mut h = RayHistory::new();
    let mut last = None;
    let opts = TraceOptions { max_dist: 6.5, ..Default::default() };
    let out = w
        .trace(VolHandle(0), o, d, opts, &mut h, &mut |s| {
            last = Some(*s);
            ControlFlow::Continue(())
        })
        .unwrap();
    assert_eq!(out.end, TraceEnd::Distance);
    assert_eq!(last.unwrap().t1, 6.5);
}

#[test]
fn escape_through_complement() {
    let w = world(Boundary::Vacuum);
    let ic = w.implicit_complement();
    let (segs, out) = segments(&w, ic, Point3::new(0.0, 0.0, 20.0), Vector3::new(0.0, 0.0, 1.0));
    assert!(segs.is_empty());
    assert_eq!(out.end, TraceEnd::Escaped);
}

#[test]
fn results_do_not_depend_on_thread_count() {
    let w = world(Boundary::Vacuum);
    let rays: Vec<(Point3, Vector3)> = (0..4000u64)
        .map(|i| {
            let f = |k: u64| ((i.wrapping_mul(6364136223846793005).wrapping_add(k * 1442695040888963407)) >> 11) as f64 / (1u64 << 53) as f64;
            (Point3::new(-4.0 + f(1) * 0.1, f(2) * 3.0 - 1.5, f(3) * 3.0 - 1.5), unit(1.0, f(4) - 0.5, f(5) - 0.5))
        })
        .collect();
    let run = |range: std::ops::Range<usize>| -> Vec<u64> {
        rays[range]
            .iter()
            .map(|&(o, d)| {
                let (segs, _) = segments(&w, VolHandle(0), o, d);
                let mut h = ContentHasher::new();
                for s in &segs {
                    h.write_u64(u64::from(s.volume.0));
                    h.write_f64(s.t0);
                    h.write_f64(s.t1);
                }
                h.finish().0 as u64
            })
            .collect()
    };
    let serial = run(0..rays.len());
    assert_eq!(serial, run(0..rays.len()));
    let mut par = Vec::new();
    std::thread::scope(|s| {
        let parts: Vec<_> = (0..4).map(|k| s.spawn(move || run(k * 1000..(k + 1) * 1000))).collect();
        for p in parts {
            par.extend(p.join().unwrap());
        }
    });
    assert_eq!(serial, par);
}

#[test]
fn cache_blob_round_trips_and_detects_damage() {
    let params = BuildParams::default();
    let input = BuildInput::Analytic(AnalyticSpec { solids: vec![sphere(1.0, None, Boundary::Vacuum)] });
    let key = input.content_hash();
    assert_eq!(key, input.content_hash());
    let blob = CacheBlob::from_sections("bvh", Precision::F64, key, &params, &[(*b"NODES   ", vec![1, 2, 3]), (*b"FACETS  ", vec![9; 100])]);
    let bytes = blob.to_bytes();
    assert_eq!(bytes.len() % 64, 0);
    let back = CacheBlob::from_bytes(&bytes).unwrap();
    assert_eq!(back, blob);
    assert_eq!(back.section(b"FACETS  ").unwrap(), &[9u8; 100][..]);
    assert!(back.header.matches("bvh", key, &params));
    assert!(!back.header.matches("obb", key, &params));
    let other = BuildParams { leaf_size: 8, ..params.clone() };
    assert!(!back.header.matches("bvh", key, &other));
    let mut bad = bytes.clone();
    let at = bad.len() - 192 + 64 + 5; // inside the FACETS section
    bad[at] ^= 1;
    assert!(matches!(CacheBlob::from_bytes(&bad).unwrap().section(b"FACETS  "), Err(RtError::Cache(_))));
    assert!(CacheBlob::from_bytes(&bytes[..20]).is_err());
}

#[cfg(feature = "capi")]
#[test]
fn c_abi_ray_fire() {
    use sm_rt_api_cand_dagmc_compat::capi::*;
    let w = world_into_raw(Box::new(world(Boundary::Vacuum)));
    let h = sm_rt_history_new();
    let (o, d) = ([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
    let mut hit = SmRtHit { surface: 0, facet: 0, dist: 0.0 };
    // SAFETY: all pointers are live and point to 3 doubles / one struct.
    unsafe {
        assert_eq!(sm_rt_ray_fire(w, 2, o.as_ptr(), d.as_ptr(), h, 0.0, 1, &mut hit), 0);
        assert!((hit.dist - 1.0).abs() < 1e-15);
        assert_eq!(sm_rt_point_in_volume(w, 2, o.as_ptr(), std::ptr::null()), 1);
        assert_eq!(sm_rt_ray_fire(w, 99, o.as_ptr(), d.as_ptr(), h, 0.0, 1, &mut hit), -1);
        assert_eq!(sm_rt_ray_fire(std::ptr::null(), 2, o.as_ptr(), d.as_ptr(), h, 0.0, 1, &mut hit), -100);
        sm_rt_history_free(h);
        sm_rt_world_free(w);
    }
}
