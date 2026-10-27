//! Acceptance 4 (cylinder, 3-D angles) and 6: curved faces, seams and poles.

mod common;

use common::*;
use core::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};
use sm_core::Point3;
use sm_math::sum::NeumaierSum;
use sm_mesh_api::{FaceIx, FaceKind, MeshRead, validate};
use sm_scheme_api::{NodeKey, Sequential};
use sm_testkit::topo::sphere_zone;
use sm_topo_api::fixtures::{cylinder, sphere, with_op};
use sm_topo_api::{CellComplex, OpId, TopologyView};

fn cyl() -> (CellComplex, sm_topo_api::fixtures::CylinderIds) {
    let mut c = CellComplex::new();
    let ids = with_op(&mut c, OpId(1), |t| cylinder(t, 1.0, 2.0)).unwrap();
    (c, ids)
}

fn surface_of(topo: &CellComplex, face: sm_core::EntityId) -> &dyn sm_geom_api::ParametricSurface {
    match topo.carrier(face) {
        sm_topo_api::Carrier::Surface(sc) => topo.shapes().surface(sc.surface).unwrap(),
        _ => panic!("no surface"),
    }
}

/// Largest distance of a mesh vertex to the surface of `face`.
fn off_surface(m: &sm_mesh_api::reference::ReferenceMesh, topo: &CellComplex, face: sm_core::EntityId) -> f64 {
    let s = surface_of(topo, face);
    (0..m.vertex_count())
        .map(|v| {
            let p = m.point(v);
            let pr = s.project(p, None).unwrap();
            (pr.point - p).norm()
        })
        .fold(0.0, f64::max)
}

#[test]
fn acc4_acc6_cylinder_lateral_face() {
    let (c, ids) = cyl();
    let face = ids.faces[2];
    let counts = vec![(ids.edges[0], 12u32), (ids.edges[1], 12), (ids.edges[2], 6)];
    let (reg, seen) = probe_registry();
    let out = mesh_faces_params(&reg, &c, &[face], &counts, serde_json::json!({ "optimize": false }), &Sequential, None)
        .expect("mesh");
    let m = &out.finished.mesh;
    let s = seen.lock().unwrap().last().cloned().unwrap();
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert!(s.isometric, "the cylinder chart is isometric");
    // Delaunay in the chart (acceptance 2) without optimize.
    assert_eq!(s.delaunay_violations, 0);
    // Angles measured in 3-D.
    let ang = min_angle_deg(m);
    println!("cylinder: N={} min angle {ang:.3} unfixable {}", m.cell_count(), s.unfixable);
    assert!(ang >= 20.0, "{ang}");
    // All nodes lie on the surface.
    let off = off_surface(m, &c, face);
    assert!(off <= 1e-12, "off surface by {off}");
    // The uv-area sum is exactly the rectangle 2 pi h.
    let want = TAU * 2.0;
    assert!((s.uv_area - want).abs() <= 1e-13 * want, "uv area {} vs {want}", s.uv_area);
    // 3-D area: inscribed triangles. Upper bound: the true area; lower bound: every triangle
    // with angular span <= phi keeps at least cos(phi/2) of its uv area (checked per triangle).
    let a3 = mesh_area(m);
    assert!(a3 <= want * (1.0 + 1e-12), "{a3} > {want}");
    let tris = tri_points(m);
    let mut phi_max = 0.0f64;
    let mut ratio_min = f64::INFINITY;
    for t in &tris {
        let th: Vec<f64> = t.iter().map(|p| p.y.atan2(p.x)).collect();
        // angular span on the circle (handle wrap)
        let mut span = 0.0f64;
        for i in 0..3 {
            let mut d = (th[i] - th[(i + 1) % 3]).abs();
            if d > PI {
                d = TAU - d;
            }
            span = span.max(d);
        }
        phi_max = phi_max.max(span);
        // uv area of the triangle: arc length x height via the unrolled coordinates around mid angle.
        let mid = th[0];
        let uv: Vec<[f64; 2]> = t
            .iter()
            .map(|p| {
                let mut d = p.y.atan2(p.x) - mid;
                while d > PI { d -= TAU; }
                while d < -PI { d += TAU; }
                [d, p.z]
            })
            .collect();
        let a_uv = 0.5 * ((uv[1][0] - uv[0][0]) * (uv[2][1] - uv[0][1]) - (uv[1][1] - uv[0][1]) * (uv[2][0] - uv[0][0])).abs();
        let a_3 = 0.5 * (t[1] - t[0]).cross(t[2] - t[0]).norm();
        ratio_min = ratio_min.min(a_3 / a_uv / (span / 2.0).cos());
    }
    assert!(ratio_min >= 1.0 - 1e-12, "per-triangle bound violated: {ratio_min}");
    assert!(a3 >= want * (phi_max / 2.0).cos(), "{a3} < {}", want * (phi_max / 2.0).cos());
    println!("cylinder: A3 = {a3:.6}, 2 pi h = {want:.6}, phi_max = {phi_max:.4}");
    // The cylinder lateral face alone is an open surface: its boundary is the two circles.
    let bnd = boundary_pairs(m);
    assert_eq!(bnd.len(), 24);
    // The seam is interior: every seam edge has two cells.
    assert!((0..m.face_count()).filter(|&f| m.face_kind(FaceIx::new(f)) == FaceKind::Interior).count() > 0);
}

#[test]
fn acc6_sphere_zone_twenty_degrees_and_on_surface() {
    let fx = sphere_zone(1.0, [0.0, FRAC_PI_2], [-FRAC_PI_4, FRAC_PI_4]).unwrap();
    // edges: bottom, right, top, left. Latitude arcs (u) and meridian arcs (v).
    let e = &fx.edges[0];
    let counts = vec![(e[0], 10u32), (e[1], 14), (e[2], 10), (e[3], 14)];
    let (reg, seen) = probe_registry();
    let out = mesh_faces_params(&reg, &fx.complex, &[fx.face], &counts, serde_json::json!({}), &Sequential, None)
        .expect("mesh");
    let m = &out.finished.mesh;
    let s = seen.lock().unwrap().last().cloned().unwrap();
    assert!(validate(m).is_empty());
    let ang = min_angle_deg(m);
    println!("zone: N={} min angle {ang:.3} unfixable {} iso {}", m.cell_count(), s.unfixable, s.isometric);
    assert!(ang >= 20.0, "{ang}");
    assert!(off_surface(m, &fx.complex, fx.face) <= 1e-12);
    // Area: inscribed triangles, between cos^2(phi) A and A with phi the largest angular
    // circumradius.
    let a = mesh_area(m);
    let rmax = tri_points(m).iter().map(circumradius).fold(0.0, f64::max);
    assert!(a <= fx.area * (1.0 + 1e-12), "{a} > {}", fx.area);
    assert!(a >= fx.area * (1.0 - rmax * rmax), "{a} < {}", fx.area * (1.0 - rmax * rmax));
}

#[test]
fn acc6_full_sphere_poles_seam_and_closed_surface() {
    let mut c = CellComplex::new();
    let ids = with_op(&mut c, OpId(1), |t| sphere(t, 1.0)).unwrap();
    let counts = vec![(ids.edges[1], 16u32)];
    let out = mesh_faces(&c, &[ids.face], &counts, "trimesh", &Sequential).expect("mesh");
    let m = &out.finished.mesh;
    // mesh_valid and closed: Euler 2, every facet interior.
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(euler(m), 2);
    assert!((0..m.face_count()).all(|f| m.face_kind(FaceIx::new(f)) == FaceKind::Interior));
    let used = edge_use(m);
    assert!(used.values().all(|&n| n == 2), "closed surface");
    // Exactly 6 triangles at each pole; every triangle holds three distinct vertices.
    for pole in ids.vertices {
        let v = out.vertex_of(NodeKey::vertex(pole)).unwrap().0 as usize;
        let n = tri_vertices(m).iter().filter(|t| t.contains(&v)).count();
        assert_eq!(n, 6, "triangles at pole {pole}");
    }
    for t in tri_vertices(m) {
        assert!(t[0] != t[1] && t[1] != t[2] && t[0] != t[2]);
    }
    // All nodes on the unit sphere.
    assert!(off_surface(m, &c, ids.face) <= 1e-12);
    // Area bound with R_max the largest 3-D circumradius.
    let a = mesh_area(m);
    let rmax = tri_points(m).iter().map(circumradius).fold(0.0, f64::max);
    let sphere_area = 4.0 * PI;
    let lo = sphere_area * (1.0 - rmax * rmax);
    println!("sphere: N={} A={a:.6} in [{lo:.6}, {sphere_area:.6}], R_max={rmax:.4}, min angle {:.2}", m.cell_count(), min_angle_deg(m));
    assert!(a <= sphere_area * (1.0 + 1e-12), "{a}");
    assert!(a >= lo, "{a} < {lo}");
    let _ = (NeumaierSum::new(), Point3::default());
}

#[test]
fn acc4_cylinder_with_default_params_reaches_twenty_degrees() {
    // Default parameters: optimize = true, min_angle = 20.
    let (c, ids) = cyl();
    let counts = vec![(ids.edges[0], 12u32), (ids.edges[1], 12), (ids.edges[2], 6)];
    let out = mesh_faces_params(&registry(), &c, &[ids.faces[2]], &counts, serde_json::json!({}), &Sequential, None)
        .expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    let ang = min_angle_deg(m);
    println!("cylinder (defaults): N={} min angle {ang:.3}", m.cell_count());
    assert!(ang >= 20.0, "{ang}");
}

#[test]
fn acc2_delaunay_on_the_cylinder_lateral_face_without_optimize() {
    let (c, ids) = cyl();
    let counts = vec![(ids.edges[0], 12u32), (ids.edges[1], 12), (ids.edges[2], 6)];
    let (reg, seen) = probe_registry();
    mesh_faces_params(&reg, &c, &[ids.faces[2]], &counts, serde_json::json!({ "optimize": false }), &Sequential, None)
        .expect("mesh");
    let s = seen.lock().unwrap().last().cloned().unwrap();
    assert!(s.isometric);
    assert_eq!(s.delaunay_violations, 0, "chart-space exact in-circle on the cylinder");
}
