//! W3b-RT-ANALYTIC acceptance 7, 10 (flat worlds), 11 (single leaf), 12-15: disjoint cells and
//! overlap reporting, stub parity, sums and reversal, boundaries, errors, determinism, closest.
#[path = "common/csg.rs"]
mod csg;

use csg::*;
use sm_core::{Aabb, EntityId, Point3, Transform, Vector3};
use sm_geom_api::{Frame, GeomStore, ImplicitDef, Quadric, SurfaceDef};
use sm_rt::{AnalyticTrackerBuilder, check_overlaps};
use sm_rt_api::stub::{AnalyticWorld, Shape};
use sm_rt_api::{
    BoundaryDef, BoundarySelector, BoundarySpec, BuildInput, BuildOptions, CellId, ComplementPolicy, FaceAction, LostCause, RegionId, ResponseKind, RtError, Scratch, StepEnd, TraceEnd,
    TraceOptions, Tracker, TrackerBuilder,
};

// ---- 7: union cells, disjointness, overlap and gap -------------------------------------------

#[test]
fn union_cell_gives_inclusion_exclusion_chords() {
    let mut fx = Fx::new(40.0);
    let a = fx.sphere(p3(-1.0, 0.0, 0.0), 2.0);
    let b = fx.sphere(p3(1.5, 0.0, 0.0), 2.0);
    fx.cell("ab", vec![term(&[hs(a, true)]), term(&[hs(b, true)])]);
    let t = fx.build();
    let mut sc = Scratch::new();
    let mut r = rng("union");
    for _ in 0..200 {
        let y = 1.9 * r.unit();
        let z = 0.5 * r.unit();
        let tr = trace(&t, p3(-20.0, y, z), v3(1.0, 0.0, 0.0), f64::INFINITY, &mut sc);
        let iv = |c: f64, rad: f64| {
            let h2 = rad * rad - y * y - z * z;
            (h2 > 0.0).then(|| (c - h2.sqrt(), c + h2.sqrt()))
        };
        let (ia, ib) = (iv(-1.0, 2.0), iv(1.5, 2.0));
        let len = |i: Option<(f64, f64)>| i.map_or(0.0, |(a, b)| b - a);
        let overlap = match (ia, ib) {
            (Some(p), Some(q)) => (p.1.min(q.1) - p.0.max(q.0)).max(0.0),
            _ => 0.0,
        };
        let exact = len(ia) + len(ib) - overlap;
        assert!((tr.inside() - exact).abs() < 1e-12, "{} vs {exact}", tr.inside());
        // One chord when the spheres overlap on the line, the interior surface is no event.
        if overlap > 0.0 {
            assert_eq!(tr.chords().len(), 1);
        }
    }
}

/// Cells C (x < 0), A and B (x > 0, in two overlapping balls): A and B overlap in a lens.
fn lens_fixture() -> Fx {
    let mut fx = Fx::new(20.0);
    let p = fx.plane(p3(0.0, 0.0, 0.0), v3(1.0, 0.0, 0.0));
    let big = fx.sphere(p3(0.0, 0.0, 0.0), 15.0);
    let sa = fx.sphere(p3(0.0, 1.0, 0.0), 4.0);
    let sb = fx.sphere(p3(0.0, -1.0, 0.0), 4.0);
    fx.cell1("c", &[hs(p, true), hs(big, true)]);
    fx.cell1("a", &[hs(p, false), hs(sa, true)]);
    fx.cell1("b", &[hs(p, false), hs(sb, true)]);
    fx
}

#[test]
fn overlapping_cells_are_reported_and_never_resolved_by_order() {
    let fx = lens_fixture();
    let t = fx.build();
    let mut sc = Scratch::new();
    // A step that crosses into the lens ends Lost(Overlap) with the state unchanged.
    let mut st = t.init_state(p3(-5.0, 0.0, 0.0), v3(1.0, 0.0, 0.0), None, &mut sc).unwrap();
    assert_eq!(st.cell, CellId::new(0));
    let before = st.clone();
    let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    match s.end {
        StepEnd::Lost(l) => assert_eq!(l.cause, LostCause::Overlap),
        other => panic!("{other:?}"),
    }
    assert_eq!(st, before, "state unchanged");
    // A cold locate inside the lens finds no unique cell (the same path as no cell).
    let l = t.locate(p3(1.0, 0.0, 0.0), Some(v3(1.0, 0.0, 0.0)), None, &mut sc).unwrap();
    assert_eq!(l.cell, CellId::NONE);
    // A point in A only is fine.
    let l = t.locate(p3(1.0, 4.0, 0.0), Some(v3(1.0, 0.0, 0.0)), None, &mut sc).unwrap();
    assert_eq!(l.cell, CellId::new(1));
    // The sampled check reports the overlap with a point inside both balls.
    let issues = fx.with_hier(|h| check_overlaps(h, 20_000, 200, 7));
    let ov: Vec<_> = issues.iter().filter(|i| i.code == "csg.overlap").collect();
    assert!(!ov.is_empty());
    let q = ov[0].location.unwrap();
    assert!((q - p3(0.0, 1.0, 0.0)).norm() < 4.0 && (q - p3(0.0, -1.0, 0.0)).norm() < 4.0 && q.x > 0.0, "{q:?}");
    assert!(ov[0].message.contains("seed 7"));
}

#[test]
fn a_hole_is_a_gap() {
    let mut fx = Fx::new(5.0);
    fx.complement = ComplementPolicy::Outside;
    let lo = fx.plane(p3(-1.0, 0.0, 0.0), v3(1.0, 0.0, 0.0));
    let hi = fx.plane(p3(1.0, 0.0, 0.0), v3(1.0, 0.0, 0.0));
    fx.cell1("left", &[hs(lo, true)]);
    fx.cell1("right", &[hs(hi, false)]);
    let issues = fx.with_hier(|h| check_overlaps(h, 5_000, 50, 3));
    let gaps: Vec<_> = issues.iter().filter(|i| i.code == "csg.gap").collect();
    assert!(!gaps.is_empty());
    assert!(gaps.iter().all(|g| g.location.unwrap().x.abs() < 1.0));
    assert!(issues.iter().all(|i| i.code != "csg.overlap"));
}

#[test]
fn check_overlaps_is_deterministic_across_threads_and_runs() {
    let fx = lens_fixture();
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| fx.with_hier(|h| check_overlaps(h, 10_000, 100, 11)))
    };
    let base = run(1);
    assert!(!base.is_empty());
    assert_eq!(base, run(4));
    assert_eq!(base, run(1));
    assert_ne!(base, fx.with_hier(|h| check_overlaps(h, 10_000, 100, 12)));
}

// ---- 10: stub parity on flat worlds ------------------------------------------------------------

fn parity(reflect: bool) {
    let domain = Aabb { lo: p3(-30.0, -30.0, -30.0), hi: p3(30.0, 30.0, 30.0) };
    let sph = |c: [f64; 3], r: f64| Shape::Sphere { center: p3(c[0], c[1], c[2]), radius: r };
    let shapes = vec![
        sph([0.0, 0.0, 0.0], 10.0),
        sph([2.0, 1.0, 0.0], 3.0),
        sph([-4.0, 3.0, 1.0], 2.0),
        sph([20.0, 0.0, 0.0], 2.0),
        Shape::Box { lo: p3(-20.0, -3.0, -3.0), hi: p3(-15.0, 3.0, 3.0) },
    ];
    let stub = if reflect {
        AnalyticWorld::new(shapes.clone(), domain, [FaceAction::Reflect; 6]).unwrap()
    } else {
        AnalyticWorld::simple(shapes.clone(), domain, ResponseKind::Vacuum).unwrap()
    };
    let n = shapes.len();
    // Volumes: where the stub knows them.
    let mut fx = Fx::new(30.0);
    let sph_ids: Vec<EntityId> = (0..4).map(|i| if let Shape::Sphere { center, radius } = shapes[i] { fx.sphere(center, radius) } else { unreachable!() }).collect();
    let Shape::Box { lo, hi } = shapes[4] else { unreachable!() };
    let bx = fx.box_planes(lo, hi);
    fx.cell1("s0", &[hs(sph_ids[0], true), hs(sph_ids[1], false), hs(sph_ids[2], false)]);
    fx.cell1("s1", &[hs(sph_ids[1], true)]);
    fx.cell1("s2", &[hs(sph_ids[2], true)]);
    fx.cell1("s3", &[hs(sph_ids[3], true)]);
    let hb: Vec<_> = bx.iter().map(|&p| hs(p, true)).collect();
    fx.cell1("s4", &hb);
    for (i, c) in fx.cells.iter_mut().enumerate() {
        if let Some(v) = stub.cell_volume(CellId::new(i)) {
            c.volume = Some(v);
        }
    }
    fx.complement = ComplementPolicy::Region("bg".into());
    if reflect {
        fx.boundaries = BoundarySpec::new(BoundaryDef::Reflect, vec![]);
    }
    let t = fx.build();
    for i in 0..=n {
        assert_eq!(t.cell_volume(CellId::new(i)), stub.cell_volume(CellId::new(i)).filter(|_| i < n), "volume {i}");
    }
    let map = |c: CellId| if c == CellId::COMPLEMENT { CellId::new(n) } else { c };
    let mut r = rng(if reflect { "parity-reflect" } else { "parity-vacuum" });
    let (mut s1, mut s2) = (Scratch::new(), Scratch::new());
    let opts = TraceOptions::default().with_max_dist(150.0).with_max_steps(10_000);
    let count = if cfg!(debug_assertions) { 10_000 } else { 100_000 };
    for _ in 0..count {
        let o = p3(25.0 * r.unit(), 25.0 * r.unit(), 25.0 * r.unit());
        let d = random_dir(&mut r);
        // locate parity
        let (a, b) = (t.locate(o, Some(d), None, &mut s1).unwrap(), stub.locate(o, Some(d), None, &mut s2).unwrap());
        assert_eq!((map(a.cell), a.region), (b.cell, b.region), "{o:?}");
        let (mut st1, mut st2) = (t.init_state(o, d, None, &mut s1).unwrap(), stub.init_state(o, d, None, &mut s2).unwrap());
        let (mut g1, mut g2) = (Vec::new(), Vec::new());
        let e1 = t
            .trace(&mut st1, &opts, &mut |s: &sm_rt_api::Segment| {
                g1.push(*s);
                sm_rt_api::Flow::Continue
            }, &mut s1)
            .unwrap();
        let e2 = stub
            .trace(&mut st2, &opts, &mut |s: &sm_rt_api::Segment| {
                g2.push(*s);
                sm_rt_api::Flow::Continue
            }, &mut s2)
            .unwrap();
        assert_eq!(g1.len(), g2.len(), "{o:?} {d:?}");
        for (x, y) in g1.iter().zip(&g2) {
            assert_eq!((map(x.cell), x.region), (y.cell, y.region));
            assert!((x.t0 - y.t0).abs() < 1e-12 && (x.length - y.length).abs() < 1e-12, "{x:?} vs {y:?}");
            // Faces of sphere shapes agree (boxes and the domain walls are numbered differently).
            if let (Some(f1), Some(f2)) = (x.face_out, y.face_out)
                && f2.get() < 4
            {
                assert_eq!(f1, f2);
            }
        }
        match (e1, e2) {
            (TraceEnd::Limit, TraceEnd::Limit) => {}
            (TraceEnd::Boundary { action: a1, .. }, TraceEnd::Boundary { action: a2, .. }) => assert_eq!(a1, a2),
            (a, b) => panic!("ends differ: {a:?} vs {b:?}"),
        }
    }
}

#[test]
fn stub_parity_vacuum_and_reflective_flat_worlds() {
    parity(false);
    parity(true);
}

// ---- 11: sums, reversal, near-edge lines (single leaf) ----------------------------------------

fn shell_leaf(c0: Point3, radii: &[f64]) -> Fx {
    let mut fx = Fx::new(60.0);
    let s: Vec<_> = radii.iter().map(|&r| fx.sphere(c0, r)).collect();
    fx.cell1("s0", &[hs(s[0], true)]);
    for k in 1..radii.len() {
        fx.cell1(&format!("s{k}"), &[hs(s[k - 1], false), hs(s[k], true)]);
    }
    fx
}

fn box_exit(o: Point3, d: Vector3, half: f64) -> f64 {
    let (oa, da) = ([o.x, o.y, o.z], [d.x, d.y, d.z]);
    (0..3).filter(|&a| da[a] != 0.0).map(|a| ((if da[a] > 0.0 { half } else { -half }) - oa[a]) / da[a]).fold(f64::INFINITY, f64::min)
}

#[test]
fn sums_equal_hull_chords_reversed_lines_agree_and_edge_lines_survive() {
    let c0 = p3(3.0, -2.0, 1.0);
    let radii = [1.0, 2.0, 3.0, 4.0, 5.0];
    let fx = shell_leaf(c0, &radii);
    let t = fx.build();
    let mut sc = Scratch::new();
    let mut r = rng("sums");
    let n = n_lines() / 10;
    for i in 0..n {
        let d = random_dir(&mut r);
        let e = perp_to(&mut r, d);
        // Every few lines graze a shell within 1e-9.
        let b = if i % 5 == 0 { radii[i % 5 + (i / 5) % 4] + 1e-9 * r.unit() } else { 5.5 * r.unit().abs() };
        let o = c0 + (e * b - d * 30.0);
        let tr = trace(&t, o, d, f64::INFINITY, &mut sc);
        assert!(!matches!(tr.end, TraceEnd::Lost(_)), "lost at b {b}: {:?} {:?}", tr.end, tr.segs);
        let hull = box_exit(o, d, 60.0);
        assert!((tr.total() - hull).abs() < 1e-12 * (1.0 + hull), "{} vs {hull}", tr.total());
        // The reversed line from the far side of the shells gives the same chords reversed.
        let q = o + d * 60.0;
        let rev = trace(&t, q, -d, f64::INFINITY, &mut sc);
        assert!(!matches!(rev.end, TraceEnd::Lost(_)));
        let (f, g) = (tr.chords(), rev.chords());
        assert_eq!(f.len(), g.len(), "b {b}");
        for (x, y) in f.iter().zip(g.iter().rev()) {
            // Grazing lines amplify the rounding of the offset by 1/sqrt(r^2 - b^2) (documented).
            let tol = if i % 5 == 0 { 1e-7 } else { 1e-11 };
            assert!((x - y).abs() < tol, "{x} vs {y}");
        }
    }
}

// ---- 12: boundaries and decks ------------------------------------------------------------------

/// A pin in a box `[0, 4]^3` with the box planes' handles returned (outward normals).
fn pin_box(complement: ComplementPolicy) -> (Fx, [EntityId; 6]) {
    let mut fx = Fx::new(10.0);
    fx.complement = complement;
    fx.bounds = None;
    let bx = fx.box_planes(p3(0.0, 0.0, 0.0), p3(4.0, 4.0, 4.0));
    let cy = fx.cylinder(Frame::at(p3(2.0, 2.0, 0.0)), 1.0);
    let mut inside: Vec<_> = bx.iter().map(|&p| hs(p, true)).collect();
    let mut fuel = inside.clone();
    fuel.push(hs(cy, true));
    inside.push(hs(cy, false));
    fx.cell1("fuel", &fuel);
    fx.cell1("mod", &inside);
    (fx, bx)
}

#[test]
fn reflective_box_keeps_histories_inside() {
    for by_rule in [false, true] {
        let (mut fx, bx) = pin_box(ComplementPolicy::Outside);
        fx.boundaries = if by_rule {
            BoundarySpec::new(BoundaryDef::Respond(ResponseKind::Vacuum), bx.iter().map(|&p| (BoundarySelector::Entity(p), BoundaryDef::Reflect)).collect())
        } else {
            BoundarySpec::new(BoundaryDef::Reflect, vec![])
        };
        let t = fx.build();
        let mut sc = Scratch::new();
        let mut r = rng("reflect");
        let (hist, steps) = if cfg!(debug_assertions) { (400, 500) } else { (10_000, 1_000) };
        for _ in 0..hist {
            let o = p3(2.0 + 1.9 * r.unit(), 2.0 + 1.9 * r.unit(), 2.0 + 1.9 * r.unit());
            let mut st = t.init_state(o, random_dir(&mut r), None, &mut sc).unwrap();
            assert!(!st.cell.is_none());
            for _ in 0..steps {
                let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
                assert!(!matches!(s.end, StepEnd::Lost(_)), "{s:?}");
                for a in [st.pos.x, st.pos.y, st.pos.z] {
                    assert!((-1e-9..=4.0 + 1e-9).contains(&a), "left the box: {:?}", st.pos);
                }
            }
        }
    }
}

#[test]
fn translation_wrap_and_respond_in_step() {
    let (mut fx, bx) = pin_box(ComplementPolicy::Outside);
    let shift = |x: f64| Transform::translation(v3(x, 0.0, 0.0));
    fx.boundaries = BoundarySpec::new(
        BoundaryDef::Respond(ResponseKind::Vacuum),
        vec![
            (BoundarySelector::Entity(bx[0]), BoundaryDef::Wrap { transform: shift(4.0), mate: bx[1] }),
            (BoundarySelector::Entity(bx[1]), BoundaryDef::Wrap { transform: shift(-4.0), mate: bx[0] }),
        ],
    );
    let t = fx.build();
    assert!(t.caps().has(sm_rt_api::TrackerCaps::PERIODIC));
    let mut sc = Scratch::new();
    let mut st = t.init_state(p3(0.3, 3.5, 3.5), v3(-1.0, 0.0, 0.0), None, &mut sc).unwrap();
    let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    assert!((s.dist - 0.3).abs() < 1e-12);
    assert!(matches!(s.end, StepEnd::Boundary { action: FaceAction::Wrap { .. }, .. }));
    assert!((st.pos.x - 4.0).abs() < 1e-12 && st.dir.x == -1.0);
    assert!(!st.cell.is_none());
    let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    assert!(s.dist > 0.0 && !matches!(s.end, StepEnd::Lost(_)));
    // The y planes carry the default response: not applied in step, `resolve_boundary` does it.
    let mut st = t.init_state(p3(3.5, 3.5, 3.5), v3(0.0, 1.0, 0.0), None, &mut sc).unwrap();
    let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    let StepEnd::Boundary { face, action, .. } = s.end else { panic!("{s:?}") };
    assert_eq!(action, FaceAction::Respond(ResponseKind::Vacuum));
    assert_eq!(st.dir, v3(0.0, 1.0, 0.0), "Respond is not applied in step");
    let n = t.face_normal(face, st.pos).unwrap();
    let out = t.resolve_boundary(&mut st, face, ResponseKind::White, [0.3, 0.6]).unwrap();
    assert_eq!(out, sm_rt_api::BoundaryAction::Continue);
    assert!((st.dir.norm() - 1.0).abs() < 1e-12 && st.dir.dot(n) < 0.0, "inward unit vector");
    // The periodic image is a query-only view.
    let img = t.periodic_image(sm_rt_api::FaceId::new(0)).unwrap().unwrap();
    assert_eq!(img.face, sm_rt_api::FaceId::new(1));
    assert!(t.periodic_image(sm_rt_api::FaceId::new(2)).unwrap().is_none());
}

// ---- 13: errors, determinism ----------------------------------------------------------------------

#[test]
fn build_checks_and_errors() {
    // csg.unbounded (warning with the Outside policy; fatal with a complement region).
    let mut fx = Fx::new(5.0);
    fx.complement = ComplementPolicy::Outside;
    fx.bounds = None;
    let p = fx.plane(p3(0.0, 0.0, 0.0), v3(1.0, 0.0, 0.0));
    fx.cell1("half", &[hs(p, true)]);
    let (_, issues) = fx.build_with(&BuildOptions::default()).unwrap();
    assert!(issues.iter().any(|i| i.code == "csg.unbounded"));
    fx.complement = ComplementPolicy::Region("out".into());
    let Err(RtError::Build(v)) = fx.build_with(&BuildOptions::default()) else { panic!() };
    assert!(v.iter().any(|i| i.code == "csg.unbounded"));

    // csg.unknown_surface (fatal).
    let mut fx = Fx::new(5.0);
    let ghost = EntityId::new(sm_core::EntityKind::Surface, 99).unwrap();
    fx.cell1("x", &[hs(ghost, true)]);
    let Err(RtError::Build(v)) = fx.build_with(&BuildOptions::default()) else { panic!() };
    assert!(v.iter().any(|i| i.code == "csg.unknown_surface"));

    // csg.degenerate_surface (fatal): an all-zero quadric.
    let mut fx = Fx::new(5.0);
    let z = fx.store_add_raw(SurfaceDef::Implicit(ImplicitDef::Quadric(Quadric { coeffs: [0.0; 10] })));
    if let Some(z) = z {
        fx.cell1("x", &[hs(z, true)]);
        let Err(RtError::Build(v)) = fx.build_with(&BuildOptions::default()) else { panic!() };
        assert!(v.iter().any(|i| i.code == "csg.degenerate_surface"));
    }

    // csg.empty_cell and csg.unused_surface (warnings).
    let mut fx = Fx::new(5.0);
    let a = fx.sphere(p3(0.0, 0.0, 0.0), 1.0);
    let b = fx.sphere(p3(9.0, 0.0, 0.0), 1.0);
    let _unused = fx.sphere(p3(0.0, 4.0, 0.0), 1.0);
    fx.cell1("ok", &[hs(a, true)]);
    fx.cell1("empty", &[hs(a, true), hs(a, false)]);
    fx.cell1("far", &[hs(a, true), hs(b, true)]);
    let (_, issues) = fx.build_with(&BuildOptions::default()).unwrap();
    assert_eq!(issues.iter().filter(|i| i.code == "csg.empty_cell").count(), 2);
    assert!(issues.iter().any(|i| i.code == "csg.unused_surface"));

    // Unsupported inputs and bad arguments.
    let (fx, bx) = pin_box(ComplementPolicy::Outside);
    let t = fx.build();
    let mut sc = Scratch::new();
    let mut st = t.init_state(p3(1.0, 1.0, 1.0), v3(1.0, 0.0, 0.0), None, &mut sc).unwrap();
    assert!(matches!(t.step(&mut st, f64::NAN, &mut sc), Err(RtError::NonFinite(_))));
    assert!(matches!(t.step(&mut st, -1.0, &mut sc), Err(RtError::InvalidInput(_))));
    st.dir = v3(2.0, 0.0, 0.0);
    assert!(matches!(t.step(&mut st, 1.0, &mut sc), Err(RtError::BadDirection { .. })));
    assert!(matches!(t.locate(p3(f64::NAN, 0.0, 0.0), None, None, &mut sc), Err(RtError::NonFinite(_))));
    assert!(matches!(t.closest(p3(0.0, 0.0, 0.0), Some(RegionId::new(99)), &mut sc), Err(RtError::UnknownId { .. })));
    assert!(matches!(t.face_normal(sm_rt_api::FaceId::new(999), p3(0.0, 0.0, 0.0)), Err(RtError::UnknownId { .. })));
    let mut fx2 = fx;
    fx2.boundaries = BoundarySpec::new(BoundaryDef::Respond(ResponseKind::Albedo(2.0)), vec![]);
    assert!(matches!(fx2.build_with(&BuildOptions::default()), Err(RtError::Build(_))));
    fx2.boundaries = BoundarySpec::new(BoundaryDef::Reflect, vec![(BoundarySelector::FaceSet("x".into()), BoundaryDef::Reflect)]);
    assert!(matches!(fx2.build_with(&BuildOptions::default()), Err(RtError::Unsupported(_))));
    let ghost = EntityId::new(sm_core::EntityKind::Surface, 77).unwrap();
    fx2.boundaries = BoundarySpec::new(BoundaryDef::Reflect, vec![(BoundarySelector::Entity(bx[0]), BoundaryDef::Wrap { transform: Transform::IDENTITY, mate: ghost })]);
    let Err(RtError::Build(v)) = fx2.build_with(&BuildOptions::default()) else { panic!() };
    assert_eq!(v[0].code, "boundary.unknown_mate");
    // No cache image.
    let img = sm_rt_api::CacheImage { key: t.info().key.unwrap(), sections: vec![] };
    assert!(t.export_cache().is_none());
    assert!(matches!(AnalyticTrackerBuilder.load_image(&img), Err(RtError::Unsupported(_))));
    fx2.boundaries = BoundarySpec::default();
    assert!(fx2.with_hier(|h| matches!(AnalyticTrackerBuilder.load(&BuildInput::Hier(*h), &BuildOptions::default(), &img), Err(RtError::Unsupported(_)))));
    // A world that places or tiles CSG leaves is not hosted by the analytic tracker yet.
    let spec = sm_rt_api::WorldSpec::new(
        vec![
            sm_rt_api::ComponentKind::Leaf { input: 0 },
            sm_rt_api::ComponentKind::Union { placements: vec![sm_rt_api::Placement::new(sm_rt_api::ComponentId(0), Transform::IDENTITY)] },
        ],
        sm_rt_api::ComponentId(1),
        sm_rt_api::InstancePolicy::Auto,
    );
    let leaf = sm_rt_api::CsgLeafInput::new(&fx2.store, &fx2.cells, fx2.bounds, &fx2.complement);
    let leaves = [sm_rt_api::LeafInput::Csg(leaf)];
    let h = sm_rt_api::HierInput::new(&spec, &leaves, &fx2.boundaries);
    assert!(matches!(AnalyticTrackerBuilder.build(&BuildInput::Hier(h), &BuildOptions::default()), Err(RtError::Unsupported(_))));
    // The mesh builder dispatches CSG worlds to the analytic builder.
    assert!(matches!(sm_rt::MeshTrackerBuilder.build(&BuildInput::Hier(h), &BuildOptions::default()), Err(RtError::Unsupported(_))));
}

#[test]
fn determinism_and_handle_permutation_invariance() {
    let build = |swap: bool| {
        let mut fx = Fx::new(30.0);
        let (s, p) = if swap {
            let p = fx.plane(p3(0.0, 0.0, 0.0), v3(1.0, 0.0, 0.0));
            (fx.sphere(p3(2.0, 0.0, 0.0), 3.0), p)
        } else {
            let s = fx.sphere(p3(2.0, 0.0, 0.0), 3.0);
            (s, fx.plane(p3(0.0, 0.0, 0.0), v3(1.0, 0.0, 0.0)))
        };
        fx.cell1("a", &[hs(s, true), hs(p, true)]);
        fx.cell1("b", &[hs(s, true), hs(p, false)]);
        fx
    };
    let (f1, f2) = (build(false), build(true));
    let (t1, t2) = (f1.build(), f2.build());
    assert_eq!(t1.structure_hash(), t2.structure_hash(), "surface handle permutation");
    assert_eq!(t1.info().key.map(|k| k.topology_hash), t2.info().key.map(|k| k.topology_hash));
    // trace_bundle: same bits for 1 and N threads and repeats.
    let mut r = rng("bundle");
    let rays: Vec<sm_geom_api::Ray> = (0..20_000).map(|_| sm_geom_api::Ray { origin: p3(25.0 * r.unit(), 25.0 * r.unit(), 25.0 * r.unit()), dir: random_dir(&mut r) }).collect();
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| t1.trace_bundle(&rays, &TraceOptions::default().with_max_dist(500.0)).unwrap().content_hash())
    };
    sm_testkit::determinism::check(2, 4, run).unwrap();
    sm_testkit::determinism::check(2, 4, |_| t1.structure_hash()).unwrap();
}

// ---- 14: closest -------------------------------------------------------------------------------

#[test]
fn closest_distances_are_exact_and_on_the_surface() {
    let mut r = rng("closest");
    let fp = random_frame(&mut r, p3(1.0, 2.0, 3.0));
    let fs = random_frame(&mut r, p3(-3.0, 1.0, 0.5));
    let fc = random_frame(&mut r, p3(4.0, -1.0, 2.0));
    let fk = random_frame(&mut r, p3(-2.0, -4.0, 1.0));
    let ft = random_frame(&mut r, p3(0.0, 5.0, -2.0));
    let half = 0.6;
    let mut fx = Fx::new(50.0);
    let ids = [
        fx.plane(fp.origin, fp.z),
        fx.sphere(fs.origin, 2.5),
        fx.cylinder(fc, 1.5),
        fx.cone(fk, half),
        fx.torus(ft, 3.0, 1.0),
    ];
    for (i, &s) in ids.iter().enumerate() {
        fx.cell1(&format!("c{i}"), &[hs(s, true)]);
    }
    let t = fx.build();
    let mut sc = Scratch::new();
    let on_surface = |i: usize, q: Point3| {
        let imp = fx.store.implicit(ids[i]).unwrap();
        let g = imp.gradient(q).norm().max(1.0);
        (imp.value(q) / g).abs()
    };
    for _ in 0..10_000 {
        let p = p3(20.0 * r.unit(), 20.0 * r.unit(), 20.0 * r.unit());
        for (i, &rg) in (0..5).zip([0usize, 1, 2, 3, 4].iter()) {
            let c = t.closest(p, Some(RegionId::new(rg)), &mut sc).unwrap().unwrap();
            assert_eq!(c.surface.get(), i);
            let exact = match i {
                0 => fp.z.dot(p - fp.origin).abs(),
                1 => ((p - fs.origin).norm() - 2.5).abs(),
                2 => {
                    let q = p - fc.origin;
                    let rho = (q - fc.z * fc.z.dot(q)).norm();
                    (rho - 1.5).abs()
                }
                3 => {
                    let q = p - fk.origin;
                    let (z, rho) = (fk.z.dot(q), (q - fk.z * fk.z.dot(q)).norm());
                    let (sn, cs) = (half.sin(), half.cos());
                    // Distance to the nearer nappe: along the generator when its foot is ahead of the apex.
                    let nappe = |zz: f64| {
                        let s = rho * sn + zz * cs;
                        if s >= 0.0 { (rho * cs - zz * sn).abs() } else { rho.hypot(zz) }
                    };
                    nappe(z).min(nappe(-z))
                }
                _ => {
                    let q = p - ft.origin;
                    let (x, y, z) = (ft.x.dot(q), ft.y.dot(q), ft.z.dot(q));
                    ((x.hypot(y) - 3.0).hypot(z) - 1.0).abs()
                }
            };
            assert!((c.dist - exact).abs() < 1e-12, "kind {i}: {} vs {exact}", c.dist);
            assert!(on_surface(i, c.point) < 1e-12 * 50.0, "kind {i}: point off the surface by {}", on_surface(i, c.point));
            assert!(((c.point - p).norm() - c.dist).abs() < 1e-12);
        }
    }
    // The nearest over all surfaces.
    let all = t.closest(p3(0.0, 0.0, 0.0), None, &mut sc).unwrap().unwrap();
    assert!(all.dist >= 0.0);
    // A general quadric has no exact closest point.
    let mut fx = Fx::new(50.0);
    let q = fx.quadric(Quadric::from_local(&Frame::at(p3(0.0, 0.0, 0.0)), [1.0, 0.5, 0.25], [0.0; 3], -1.0));
    fx.cell1("e", &[hs(q, true)]);
    let t = fx.build();
    assert!(matches!(t.closest(p3(5.0, 5.0, 5.0), None, &mut sc), Err(RtError::Unsupported(_))));
}
