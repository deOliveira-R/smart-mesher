//! T2 acceptance on the reference fixtures: counts, analytic areas with derived bounds,
//! watertightness, normal signs and determinism.

mod common;

use core::f64::consts::{PI, TAU};

use common::*;
use sm_core::{EntityId, Point3};
use sm_tess::{TessError, TessOptions, tessellate, tessellate_entities};
use sm_testkit::topo::all_topo_fixtures;
use sm_topo_api::fixtures::{brick, cylinder, sphere};
use sm_topo_api::{Dim, TopologyView};

fn opts(chord: f64) -> TessOptions {
    TessOptions { chord_tol: chord, angle_tol: 0.5, max_edge: f64::INFINITY }
}

#[test]
fn brick_has_12_triangles_and_area_exactly_600() {
    let (c, ids) = build(|t| brick(t, Point3::new(-5.0, -5.0, -5.0), [10.0; 3]));
    let t = tessellate(&c, &all_faces(&c), &TessOptions::default()).unwrap();
    assert_eq!(t.triangle_count(), 12);
    assert_eq!(t.vertex_count(), 8);
    assert_eq!(total_area(&t), 600.0);
    for f in ids.faces {
        assert_eq!(t.face_indices(f).unwrap().len(), 6);
        assert_eq!(face_area(&t, f), 100.0);
    }
    assert_eq!(t.edges.len(), 12);
    assert!(t.edges.iter().all(|r| r.count == 2));
}

#[test]
fn cylinder_lateral_area_within_derived_chord_bound() {
    let (r, h) = (2.0, 3.0);
    let (c, ids) = build(|t| cylinder(t, r, h));
    let o = opts(1e-3);
    let t = tessellate(&c, &all_faces(&c), &o).unwrap();
    // theta from the shared circle polyline.
    let n = t.edge_polyline(ids.edges[0]).unwrap().len() - 1;
    let theta = TAU / n as f64;
    assert!(r * (1.0 - (theta / 2.0).cos()) <= o.chord_tol + 1e-15, "the circle meets the chord tolerance");
    let exact = TAU * r * h;
    let area = face_area(&t, ids.faces[2]);
    let bound = TAU * h * (r - r * (theta / 2.0).cos());
    let f32_slack = 1e-5 * exact;
    assert!(area <= exact + f32_slack, "inscribed area {area} exceeds exact {exact}");
    assert!(exact - area <= bound + f32_slack, "area error {} exceeds bound {bound}", exact - area);
    // The caps: inscribed regular n-gon.
    let cap = 0.5 * n as f64 * r * r * theta.sin();
    for f in [ids.faces[0], ids.faces[1]] {
        assert!((face_area(&t, f) - cap).abs() < 1e-5 * cap, "cap area");
    }
}

#[test]
fn sphere_area_within_bound() {
    let r = 1.5;
    let (c, ids) = build(|t| sphere(t, r));
    let t = tessellate(&c, &all_faces(&c), &opts(1e-3)).unwrap();
    let area = face_area(&t, ids.face);
    let exact = 4.0 * PI * r * r;
    // A triangle with vertices on the sphere and circumradius R_c has flat area >= spherical area
    // * (1 - R_c^2 / r^2) (central projection from the centre); the spherical triangles tile the sphere.
    let worst = t.indices.chunks(3).map(|tri| circumradius(&t, tri)).fold(0.0, f64::max);
    let lower = exact * (1.0 - (worst / r).powi(2));
    let slack = 1e-5 * exact;
    assert!(area <= exact + slack && area >= lower - slack, "area {area} outside [{lower}, {exact}]");
    // The sampling really is fine: relative deficit well below the loose a-priori limit.
    let theta_chord = 2.0 * (1.0 - 1e-3 / r).acos();
    assert!((exact - area) / exact < theta_chord * theta_chord, "deficit {}", (exact - area) / exact);
    // Poles are single shared vertices: V - E + F style check on the closed surface.
    check_face_structure(&c, &t, ids.face);
}

#[test]
fn fixtures_are_watertight_and_consistently_oriented() {
    for fx in all_topo_fixtures().unwrap() {
        let c = &fx.complex;
        let t = tessellate(c, &all_faces(c), &opts(2e-3)).unwrap();
        for f in all_faces(c) {
            check_face_structure(c, &t, f);
        }
        // Adjacent faces share polyline vertices by index, hence bit-exactly.
        for e in c.cells(Dim::Edge) {
            let Some(poly) = t.edge_polyline(*e) else { continue };
            for r in c.coboundary(*e) {
                let used: std::collections::BTreeSet<u32> = t.face_indices(r.cell).unwrap().iter().copied().collect();
                assert!(poly.iter().all(|i| used.contains(i)), "{}: face {:?} misses a vertex of edge {e:?}", fx.name, r.cell);
            }
        }
        assert!((total_area(&t) - fx.area).abs() < 0.05 * fx.area, "{}: area {} vs {}", fx.name, total_area(&t), fx.area);
    }
}

#[test]
fn triangle_normals_agree_with_face_normals() {
    for fx in all_topo_fixtures().unwrap() {
        let c = &fx.complex;
        let t = tessellate(c, &all_faces(c), &opts(2e-3)).unwrap();
        for f in all_faces(c) {
            check_normal_signs(c, &t, f);
        }
    }
}

#[test]
fn output_is_identical_for_one_and_many_threads_and_repeats() {
    let fx = all_topo_fixtures().unwrap();
    let h = sm_testkit::determinism::check(2, 4, |threads| {
        fx.iter()
            .map(|f| {
                let c = &f.complex;
                pool(threads, || digest(&tessellate(c, &all_faces(c), &opts(1e-3)).unwrap()))
            })
            .collect::<Vec<u64>>()
    })
    .unwrap();
    assert_eq!(h.len(), fx.len());
}

#[test]
fn face_order_and_duplicates_do_not_change_the_output() {
    let (c, _) = build(|t| cylinder(t, 1.0, 2.0));
    let mut faces = all_faces(&c);
    let a = tessellate(&c, &faces, &opts(1e-3)).unwrap();
    faces.reverse();
    faces.push(faces[0]);
    let b = tessellate(&c, &faces, &opts(1e-3)).unwrap();
    assert_eq!(a, b);
}

#[test]
fn shared_edges_are_sampled_once_for_all_neighbours() {
    let (c, ids) = build(|t| cylinder(t, 1.0, 2.0));
    let t = tessellate(&c, &all_faces(&c), &opts(1e-3)).unwrap();
    // The bottom circle belongs to the bottom cap and the lateral face, closed: first == last.
    let poly = t.edge_polyline(ids.edges[0]).unwrap();
    assert_eq!(poly[0], poly[poly.len() - 1]);
    // Its end vertex index is the topological vertex (vertices come first, ascending id).
    assert!((poly[0] as usize) < 2);
    assert_eq!(t.position(poly[0]), [1.0, 0.0, 0.0]);
    // Top and bottom circles sample identically.
    assert_eq!(t.edge_polyline(ids.edges[1]).unwrap().len(), poly.len());
}

#[test]
fn max_edge_subdivides_edges_without_breaking_watertightness() {
    let (c, ids) = build(|t| brick(t, Point3::new(0.0, 0.0, 0.0), [10.0, 10.0, 10.0]));
    let o = TessOptions { chord_tol: 1.0, angle_tol: 1.0, max_edge: 2.5 };
    let t = tessellate(&c, &all_faces(&c), &o).unwrap();
    assert!(t.edges.iter().all(|r| r.count == 5));
    for f in ids.faces {
        check_face_structure(&c, &t, f);
        assert_eq!(t.face_indices(f).unwrap().len() / 3, 14, "16 boundary points -> 14 triangles");
    }
    assert_eq!(total_area(&t), 600.0);
}

#[test]
fn finer_chord_tolerance_gives_more_triangles() {
    let (c, _) = build(|t| sphere(t, 1.0));
    let coarse = tessellate(&c, &all_faces(&c), &opts(1e-2)).unwrap().triangle_count();
    let fine = tessellate(&c, &all_faces(&c), &opts(1e-4)).unwrap().triangle_count();
    assert!(fine > 4 * coarse, "{fine} vs {coarse}");
}

#[test]
fn volumes_bodies_and_edges_expand() {
    let (c, ids) = build(|t| brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3]));
    let o = TessOptions::default();
    let by_volume = tessellate_entities(&c, &[ids.volume], &o).unwrap();
    let by_body = tessellate_entities(&c, &[ids.body], &o).unwrap();
    let by_faces = tessellate(&c, &ids.faces, &o).unwrap();
    assert_eq!(by_volume, by_faces);
    assert_eq!(by_body, by_faces);
    let one_edge = tessellate_entities(&c, &[ids.edges[0]], &o).unwrap();
    assert_eq!((one_edge.triangle_count(), one_edge.edges.len(), one_edge.vertex_count()), (0, 1, 2));
}

#[test]
fn bad_inputs_are_typed_errors() {
    let (c, ids) = build(|t| brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3]));
    let o = TessOptions::default();
    assert!(matches!(tessellate(&c, &[ids.edges[0]], &o), Err(TessError::NotAFace(_))));
    let ghost = EntityId::new(sm_core::EntityKind::TopoFace, 999).unwrap();
    assert!(matches!(tessellate(&c, &[ghost], &o), Err(TessError::NotAFace(_))));
    assert!(matches!(tessellate_entities(&c, &[ids.vertices[0]], &o), Err(TessError::BadEntity(_))));
    for bad in [
        TessOptions { chord_tol: 0.0, ..o },
        TessOptions { chord_tol: f64::NAN, ..o },
        TessOptions { angle_tol: 0.0, ..o },
        TessOptions { angle_tol: 4.0, ..o },
        TessOptions { max_edge: -1.0, ..o },
    ] {
        assert!(matches!(tessellate(&c, &ids.faces, &bad), Err(TessError::BadOptions(_))), "{bad:?}");
    }
}

#[test]
fn empty_input_is_an_empty_tessellation() {
    let (c, _) = build(|t| brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3]));
    let t = tessellate(&c, &[], &TessOptions::default()).unwrap();
    assert_eq!((t.vertex_count(), t.triangle_count(), t.faces.len(), t.edges.len()), (0, 0, 0, 0));
}
