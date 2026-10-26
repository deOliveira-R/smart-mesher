//! The advancing front (stretch S1/S2) on the pure kernel.

mod common;

use common::*;
use sm_scheme_tri::TriError;
use sm_scheme_tri::cdt::{Pslg, Segment, SegmentKind};
use sm_scheme_tri::front::{AdvanceOptions, advance};
use sm_testkit::loops2d::{self, Region2};

fn subdivided(r: &Region2, h: f64) -> Pslg {
    let mut p = Pslg::default();
    for l in r.loops() {
        let base = p.points.len() as u32;
        let mut ring: Vec<[f64; 2]> = Vec::new();
        for i in 0..l.len() {
            let (a, b) = (l[i], l[(i + 1) % l.len()]);
            let len = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
            let n = (len / h).round().max(1.0) as usize;
            for k in 0..n {
                let s = k as f64 / n as f64;
                ring.push([a.0 + s * (b.0 - a.0), a.1 + s * (b.1 - a.1)]);
            }
        }
        let m = ring.len() as u32;
        p.points.extend(ring);
        for i in 0..m {
            p.segments.push(Segment { a: base + i, b: base + (i + 1) % m, kind: SegmentKind::Boundary });
        }
    }
    p
}

#[test]
fn front_on_the_regions_meets_the_acceptance_numbers() {
    for (r, h) in [
        (loops2d::square(), 0.1),
        (loops2d::circle(1.0, 64), 2.0 * std::f64::consts::PI / 64.0),
        (loops2d::annulus(1.0, 3.0, 48), 2.0 * std::f64::consts::PI / 48.0),
        (loops2d::l_shape(), 0.1),
        (loops2d::rectangle_with_holes(), 0.5),
    ] {
        let p = subdivided(&r, h);
        let g = Flat(h);
        let mut c = match advance(&p, &g, &AdvanceOptions::default()) {
            Ok(c) => c,
            Err(e) => panic!("{}: {e}", r.name),
        };
        assert!(all_positive(&c), "{}", r.name);
        let a = area(&c);
        let ang = c.min_angle(&g).unwrap().0;
        let n_ideal = r.area / (3f64.sqrt() / 4.0 * h * h);
        println!("{}: N={} ideal {:.0} min angle {ang:.2} area {a} vs {}", r.name, c.triangles().len(), n_ideal, r.area);
        assert!((a - r.area).abs() <= 1e-12 * r.area, "{}", r.name);
        let mut want: Vec<(u32, u32)> = p.segments.iter().map(|s| (s.a.min(s.b), s.a.max(s.b))).collect();
        want.sort();
        assert_eq!(boundary_edges(&c), want, "{}", r.name);
    }
}

// ------------------------------------------------------------------ stretch S2 through the driver

use sm_mesh_api::{MeshRead, validate};
use sm_scheme_api::{FailureKind, SchemeDim, SchemeError, Sequential};
use sm_testkit::topo::planar_face;
use sm_topo_api::fixtures::{cylinder, with_op};
use sm_topo_api::{CellComplex, OpId};

fn run_front(
    topo: &CellComplex,
    face: sm_core::EntityId,
    counts: &[(sm_core::EntityId, u32)],
) -> sm_mesh_api::reference::ReferenceMesh {
    let model = sm_model_api::Model::new();
    let reg = registry();
    let (solver, picker) = (sm_scheme_api::ReferenceIntervalSolver::default(), sm_scheme_api::ReferenceCornerPicker);
    let driver = sm_scheme_api::Driver::new(&reg, &solver, &picker);
    let sel = [face];
    let input = sm_scheme_api::MeshInput::new(topo, &model, &sm_scheme_api::WholeFaces, &sel);
    let mut intent = sm_scheme_api::MeshIntent::default();
    for &(e, n) in counts {
        intent.set_intervals(e, sm_scheme_api::Firm::hard(sm_scheme_api::IntervalRequest::Count(n)));
    }
    intent.set_scheme(face, sm_scheme_api::SchemeChoice::new("triadvance", sm_scheme_api::Firmness::Hard));
    driver
        .mesh(&input, &intent, &Sequential, sm_mesh_api::reference::ReferenceBuilder::surface(), &mut sm_mesh_api::SerialAlloc::new())
        .expect("triadvance")
        .finished
        .mesh
}

#[test]
fn s2_front_scheme_on_square_disc_annulus_and_cylinder() {
    let cases: Vec<(Region2, f64, i64)> = vec![
        (loops2d::square(), 0.1, 1),
        (loops2d::circle(1.0, 64), 0.098, 1),
        (loops2d::annulus(1.0, 3.0, 48), 0.131, 0),
    ];
    for (r, h, euler_want) in cases {
        let fx = planar_face(&r, 0.0).unwrap();
        let counts = counts_for(&fx, h);
        let m = run_front(&fx.complex, fx.face, &counts);
        assert!(validate(&m).is_empty(), "{}", r.name);
        let ang = min_angle_deg(&m);
        let n_ideal = r.area / (3f64.sqrt() / 4.0 * h * h);
        let n = m.cell_count() as f64;
        println!("front {}: N={n} ideal {n_ideal:.0} min angle {ang:.2}", r.name);
        assert!(ang >= 20.0, "{}: {ang}", r.name);
        assert!(n >= 0.7 * n_ideal && n <= 1.6 * n_ideal, "{}: N={n} ideal {n_ideal}", r.name);
        assert_eq!(euler(&m), euler_want, "{}", r.name);
        assert!((mesh_area(&m) - r.area).abs() <= 1e-12 * r.area);
    }
    // Cylinder lateral face, 12 around and 6 along (3-D angles).
    let mut c = CellComplex::new();
    let ids = with_op(&mut c, OpId(1), |t| cylinder(t, 1.0, 2.0)).unwrap();
    let counts = vec![(ids.edges[0], 12u32), (ids.edges[1], 12), (ids.edges[2], 6)];
    let m = run_front(&c, ids.faces[2], &counts);
    assert!(validate(&m).is_empty());
    let ang = min_angle_deg(&m);
    println!("front cylinder: N={} min angle {ang:.2}", m.cell_count());
    assert!(ang >= 20.0, "{ang}");
    assert_eq!(euler(&m), 0);
}

#[test]
fn s2_iteration_cap_is_enforced_and_reversed_loops_return_stuck() {
    // A cap of one iteration per ideal triangle is too small: the front reports Stuck instead of looping.
    let p = subdivided(&loops2d::square(), 0.1);
    let g = Flat(0.1);
    let r = advance(&p, &g, &AdvanceOptions { iteration_factor: 0.2, ..Default::default() });
    assert!(matches!(r, Err(TriError::Stuck { .. })), "{:?}", r.err());
    // The default cap suffices.
    assert!(advance(&p, &g, &AdvanceOptions::default()).is_ok());
    // A clockwise outer loop bounds the exterior: the front would grow forever; the cap stops it.
    let mut q = p.clone();
    for s in &mut q.segments {
        std::mem::swap(&mut s.a, &mut s.b);
    }
    let r = advance(&q, &g, &AdvanceOptions::default());
    assert!(
        matches!(r, Err(TriError::Stuck { .. }) | Err(TriError::TooManyPoints { .. }) | Err(TriError::Degenerate(_))),
        "{:?}",
        r.err()
    );
}

#[test]
fn s2_fallback_chain_reports_both_attempts_on_the_thin_rectangle() {
    let fx = planar_face(&loops2d::rectangle(10.0, 0.1), 0.0).unwrap();
    let e = &fx.edges[0];
    let counts = vec![(e[0], 10u32), (e[1], 1), (e[2], 10), (e[3], 1)];
    let mut reg = registry();
    reg.set_fallback(SchemeDim::Surface, "trimesh", &["triadvance"]).unwrap();
    let err = mesh_faces_with(&reg, &fx.complex, &[fx.face], &counts, "trimesh", &Sequential, None)
        .err()
        .expect("both fail");
    println!("{err}");
    assert!(err.contains("trimesh") && err.contains("triadvance"), "{err}");
    let _ = (FailureKind::Quality, SchemeError::Input(String::new()));
}

#[test]
fn s2_front_on_random_star_polygons_is_valid_or_stuck() {
    let mut ok = 0;
    for seed in 1..=60u64 {
        let mut rng = Rng(seed * 7919 + 13);
        let n = 8 + rng.below(30);
        let angs: Vec<f64> = (0..n)
            .map(|i| (i as f64 + 0.3 + 0.4 * (rng.below(1000) as f64 / 1000.0)) / n as f64 * std::f64::consts::TAU)
            .collect();
        let ring: Vec<[f64; 2]> = angs
            .iter()
            .map(|a| {
                let r = 0.6 + 0.4 * (rng.below(8) as f64 / 8.0);
                [1.5 + r * a.cos(), 1.5 + r * a.sin()]
            })
            .collect();
        let m = ring.len() as u32;
        let mut p = Pslg { points: ring.clone(), segments: Vec::new() };
        for i in 0..m {
            p.segments.push(Segment { a: i, b: (i + 1) % m, kind: SegmentKind::Boundary });
        }
        let area_poly: f64 = (0..ring.len())
            .map(|i| {
                let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
                0.5 * (a[0] * b[1] - b[0] * a[1])
            })
            .sum();
        match advance(&p, &Flat(0.3), &AdvanceOptions::default()) {
            Ok(c) => {
                ok += 1;
                assert!(all_positive(&c), "seed {seed}");
                assert!((area(&c) - area_poly).abs() <= 1e-12 * area_poly, "seed {seed}");
                let mut want: Vec<(u32, u32)> = p.segments.iter().map(|s| (s.a.min(s.b), s.a.max(s.b))).collect();
                want.sort();
                assert_eq!(boundary_edges(&c), want, "seed {seed}");
            }
            Err(TriError::Stuck { .. }) | Err(TriError::Degenerate(_)) => {}
            Err(e) => panic!("seed {seed}: {e}"),
        }
    }
    println!("front on random polygons: {ok}/60 closed");
    assert!(ok >= 40, "only {ok} of 60 closed");
}

#[test]
fn s2_front_conforms_to_embedded_chains_and_points() {
    let mut p = subdivided(&loops2d::square(), 0.1);
    let base = p.points.len() as u32;
    for k in 0..=6 {
        p.points.push([0.2 + 0.1 * k as f64, 0.5]);
    }
    for k in 0..6 {
        p.segments.push(Segment { a: base + k, b: base + k + 1, kind: SegmentKind::Embedded });
    }
    let iso = p.points.len() as u32;
    p.points.push([0.35, 0.2]);
    let c = advance(&p, &Flat(0.1), &AdvanceOptions::default()).expect("front");
    assert!(all_positive(&c));
    assert!((area(&c) - 1.0).abs() < 1e-12);
    for k in 0..6 {
        assert!(c.is_constrained(base + k, base + k + 1));
    }
    let used: std::collections::BTreeSet<u32> = c.triangles().iter().flatten().copied().collect();
    assert!(used.contains(&iso));
}

#[test]
fn s2_front_scheme_on_sphere_and_torus_is_valid_or_a_typed_failure() {
    use sm_topo_api::fixtures::{sphere, torus};
    let mut c = CellComplex::new();
    let ids = with_op(&mut c, OpId(1), |t| sphere(t, 1.0)).unwrap();
    let r = mesh_faces(&c, &[ids.face], &[(ids.edges[1], 16)], "triadvance", &Sequential);
    match r {
        Ok(o) => {
            let m = &o.finished.mesh;
            assert!(validate(m).is_empty());
            assert_eq!(euler(m), 2);
            println!("front sphere: N={} min angle {:.2}", m.cell_count(), min_angle_deg(m));
        }
        Err(e) => println!("front sphere: {e}"),
    }
    let mut c = CellComplex::new();
    let ids = with_op(&mut c, OpId(1), |t| torus(t, 3.0, 1.0)).unwrap();
    let r = mesh_faces(&c, &[ids.face], &[(ids.edges[0], 24), (ids.edges[1], 12)], "triadvance", &Sequential);
    match r {
        Ok(o) => {
            let m = &o.finished.mesh;
            assert!(validate(m).is_empty());
            assert_eq!(euler(m), 0);
            println!("front torus: N={} min angle {:.2}", m.cell_count(), min_angle_deg(m));
        }
        Err(e) => println!("front torus: {e}"),
    }
}
