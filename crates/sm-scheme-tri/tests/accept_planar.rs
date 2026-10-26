//! Acceptance 2-5 on planar faces, through the `Driver`.

mod common;

use common::*;
use core::f64::consts::PI;
use sm_mesh_api::{MeshRead, validate};
use sm_scheme_api::{FailureKind, SchemeError, Sequential};
use sm_testkit::loops2d::{self, Region2};
use sm_testkit::topo::{annulus_face, planar_face};

fn run_probe(
    fx: &sm_testkit::topo::FaceFixture,
    h: f64,
    params: serde_json::Value,
) -> (sm_scheme_api::Meshed<sm_mesh_api::reference::ReferenceMesh>, Seen) {
    let (reg, seen) = probe_registry();
    let counts = counts_for(fx, h);
    let out = mesh_faces_params(&reg, &fx.complex, &[fx.face], &counts, params, &Sequential, None)
        .expect("mesh");
    let s = seen.lock().unwrap().last().cloned().expect("seen");
    (out, s)
}

fn two_ring_annulus() -> Region2 {
    // Inner circle 48 points (r = 1), outer 96 points (r = 3).
    let ngon = |r: f64, n: usize| -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let a = 2.0 * PI * i as f64 / n as f64;
                (r * a.cos(), r * a.sin())
            })
            .collect()
    };
    let mut inner = ngon(1.0, 48);
    inner.reverse();
    Region2 {
        name: "annulus_48_96",
        outer: ngon(3.0, 96),
        holes: vec![inner],
        area: loops2d::ngon_area(96, 3.0) - loops2d::ngon_area(48, 1.0),
        outer_types: None,
    }
}

// ------------------------------------------------------------------ acceptance 2

#[test]
fn acc2_delaunay_without_optimize_on_square_annulus_l_shape() {
    for (r, h) in [
        (loops2d::square(), 0.1),
        (loops2d::annulus(1.0, 3.0, 48), 0.13),
        (loops2d::l_shape(), 0.1),
    ] {
        let fx = planar_face(&r, 0.0).unwrap();
        let (_, s) = run_probe(&fx, h, serde_json::json!({ "optimize": false }));
        assert_eq!(s.delaunay_violations, 0, "{}", r.name);
        assert!(s.triangles > 0);
    }
}

#[test]
fn acc2_delaunay_with_optimize_on_planar_faces() {
    for (r, h) in [
        (loops2d::square(), 0.1),
        (loops2d::annulus(1.0, 3.0, 48), 0.13),
        (loops2d::l_shape(), 0.1),
        (loops2d::rectangle_with_holes(), 0.5),
    ] {
        let fx = planar_face(&r, 0.0).unwrap();
        let (_, s) = run_probe(&fx, h, serde_json::json!({ "optimize": true }));
        assert_eq!(s.delaunay_violations, 0, "{}", r.name);
    }
}

// ------------------------------------------------------------------ acceptance 3

#[test]
fn acc3_conformity_euler_and_area_on_the_loop_fixtures() {
    let regions = [
        (loops2d::square(), 0.25),
        (loops2d::rectangle_with_holes(), 0.6),
        (loops2d::annulus(1.0, 2.0, 32), 0.2),
        (loops2d::circle(1.0, 64), 0.1),
        (loops2d::l_shape(), 0.25),
    ];
    for (r, h) in regions {
        let fx = planar_face(&r, 0.0).unwrap();
        let counts = counts_for(&fx, h);
        let out = mesh_faces(&fx.complex, &[fx.face], &counts, "trimesh", &Sequential).expect(r.name);
        let m = &out.finished.mesh;
        assert!(validate(m).is_empty(), "{}: {:?}", r.name, validate(m));
        // Boundary edges of the output are exactly the input segments.
        let want = expected_boundary(&out, &fx.complex, &all_edges(&fx));
        assert_eq!(boundary_pairs(m), want, "{}", r.name);
        // Euler: V - E + F = 1 - holes.
        assert_eq!(euler(m), 1 - r.holes.len() as i64, "{}", r.name);
        // Area = signed_area(outer) - sum signed_area(holes), 1e-12 relative.
        let a = mesh_area(m);
        let want_a = loops2d::signed_area(&r.outer)
            + r.holes.iter().map(|h| loops2d::signed_area(h)).sum::<f64>();
        assert!((a - want_a).abs() <= 1e-12 * want_a, "{}: {a} vs {want_a}", r.name);
    }
}

#[test]
fn acc3_embedded_chain_and_isolated_vertex_are_mesh_vertices_and_edges() {
    let mut fx = planar_face(&loops2d::square(), 0.0).unwrap();
    let (wires, pts) = embed_wires(&mut fx, 0.0, &[([0.2, 0.5], [0.8, 0.5])], &[[0.3, 0.2]]);
    let mut counts = counts_for(&fx, 0.1);
    counts.push((wires[0], 6));
    let out = mesh_faces(&fx.complex, &[fx.face], &counts, "trimesh", &Sequential).expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty());
    // Chain nodes: all 7 are vertices; all 6 chain segments are mesh edges.
    use sm_topo_api::TopologyViewExt;
    let (a, b) = fx.complex.edge_ends(wires[0]).unwrap();
    let mut keys = vec![sm_scheme_api::NodeKey::vertex(a)];
    keys.extend((1..6).map(|i| sm_scheme_api::NodeKey::new(wires[0], i)));
    keys.push(sm_scheme_api::NodeKey::vertex(b));
    let used = edge_use(m);
    for w in keys.windows(2) {
        let (x, y) = (out.vertex_of(w[0]).unwrap().0 as usize, out.vertex_of(w[1]).unwrap().0 as usize);
        assert!(used.contains_key(&(x.min(y), x.max(y))), "chain segment {w:?}");
    }
    // The isolated vertex is a mesh vertex used by at least one triangle.
    let v = out.vertex_of(sm_scheme_api::NodeKey::vertex(pts[0])).unwrap().0 as usize;
    assert!(tri_vertices(m).iter().any(|t| t.contains(&v)));
    // Embedded segments do not change area or Euler number of a disc.
    assert!((mesh_area(m) - 1.0).abs() < 1e-12);
    assert_eq!(euler(m), 1);
}

// ------------------------------------------------------------------ acceptance 4

#[test]
fn acc4_unit_square_ten_intervals() {
    let fx = planar_face(&loops2d::square(), 0.0).unwrap();
    let (out, s) = run_probe(&fx, 0.1, serde_json::json!({}));
    let m = &out.finished.mesh;
    let ang = min_angle_deg(m);
    println!("square: N={} min angle {ang:.3} unfixable {}", m.cell_count(), s.unfixable);
    assert!(ang >= 20.0, "{ang}");
    assert!((180..=330).contains(&m.cell_count()), "N = {}", m.cell_count());
    assert_eq!(s.unfixable, 0);
}

#[test]
fn acc4_twenty_degrees_on_disc_annulus_and_l_shape() {
    let cases: Vec<(Region2, f64)> = vec![
        (loops2d::circle(1.0, 64), 0.1),
        (two_ring_annulus(), 0.13),
        (loops2d::l_shape(), 0.1),
        (loops2d::annulus(1.0, 3.0, 48), 0.13),
        (loops2d::rectangle_with_holes(), 0.5),
    ];
    for (r, h) in cases {
        let fx = planar_face(&r, 0.0).unwrap();
        let (out, s) = run_probe(&fx, h, serde_json::json!({}));
        let ang = min_angle_deg(&out.finished.mesh);
        println!("{}: N={} min angle {ang:.3} unfixable {}", r.name, out.finished.mesh.cell_count(), s.unfixable);
        assert!(ang >= 20.0, "{}: {ang}", r.name);
    }
}

#[test]
fn acc4_analytic_annulus_face_with_circle_edges() {
    // True circles (not polygons): 48 intervals on the inner and 96 on the outer circle.
    let fx = annulus_face(1.0, 3.0).unwrap();
    let counts = vec![(fx.edges[0][0], 96u32), (fx.edges[1][0], 48u32)];
    let (reg, seen) = probe_registry();
    let out = mesh_faces_params(&reg, &fx.complex, &[fx.face], &counts, serde_json::json!({}), &Sequential, None)
        .expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty());
    let ang = min_angle_deg(m);
    let s = seen.lock().unwrap().last().cloned().unwrap();
    println!("annulus_face: N={} min angle {ang:.3} unfixable {}", m.cell_count(), s.unfixable);
    assert!(ang >= 20.0, "{ang}");
    assert_eq!(euler(m), 0);
    // Inscribed polygons: area below pi (9 - 1), above the two ngon areas' difference.
    let a = mesh_area(m);
    let lo = loops2d::ngon_area(96, 3.0) - loops2d::ngon_area(48, 1.0);
    assert!(a >= lo * (1.0 - 1e-12) && a <= fx.area, "{a} in [{lo}, {}]", fx.area);
}

// ------------------------------------------------------------------ acceptance 5

#[test]
fn acc5_gate_fails_on_the_thin_rectangle_with_the_worst_triangle_nodes() {
    let fx = planar_face(&loops2d::rectangle(10.0, 0.1), 0.0).unwrap();
    // Edges in loop order: bottom (long), right (short), top (long), left (short).
    let e = &fx.edges[0];
    let counts = vec![(e[0], 10u32), (e[1], 1), (e[2], 10), (e[3], 1)];
    let reg = registry();
    // The driver run reports the single attempt, with the achieved angle in its message.
    let err = mesh_faces_params(&reg, &fx.complex, &[fx.face], &counts, serde_json::json!({}), &Sequential, None)
        .err()
        .expect("must fail the gate");
    let SchemeError::NoApplicableScheme { tried, .. } = &err else {
        panic!("expected NoApplicableScheme, got {err:?}");
    };
    assert_eq!(tried.len(), 1, "{tried:?}");
    assert_eq!(tried[0].scheme.as_str(), "trimesh");
    assert!(tried[0].message.contains("5.7"), "achieved angle in the message: {}", tried[0].message);
    // The structured failure of the scheme itself carries the worst triangle's nodes.
    let gf = match generate_only(&reg, &fx.complex, &[fx.face], &counts, serde_json::json!({})) {
        Err(SchemeError::Generate(gf)) => gf,
        other => panic!("expected a generation failure, got {other:?}"),
    };
    assert_eq!(gf.kind, FailureKind::Quality);
    assert!(gf.reason.contains("deg"), "{}", gf.reason);
    assert_eq!(gf.nodes.len(), 3, "worst triangle nodes: {:?}", gf.nodes);
    assert_eq!(gf.scheme.as_str(), "trimesh");
}
