//! Acceptance 6: mirror decks slide, translation decks and interfaces are pinned.

mod common;

use common::*;
use sm_core::{EntityKind, Vector3};
use sm_mesh_api::{MeshEntityKind, MeshRead};
use sm_model_api::{BoundaryKind, Deck, InterfaceIntent};
use sm_smooth::{Method, NodeSelect, Schedule, SmoothInput, SmoothRequest, smooth};
use sm_testkit::lattice;
use sm_testkit::topo::mirror_chamber;
use sm_topo_api::{TopologyView, types::Dim};

#[test]
fn a6_mirror_deck_nodes_slide_in_the_plane() {
    let fx = mirror_chamber().expect("mirror chamber");
    let vol = fx.complex.cells(Dim::Volume)[0];
    let mut mesh = stub_mesh(&fx.complex, &fx.model, vol, 0.25);
    jitter(&mut mesh, &fx.complex, 5, 0.06, false);
    let deck_faces: Vec<_> = fx.model.deck_faces().map(|(f, _)| f).collect();
    assert_eq!(deck_faces.len(), 1);
    let cap = deck_faces[0];
    let before: Vec<[f64; 3]> = mesh.classified_on(cap).expect("cap nodes").vertices.iter().map(|v| {
        let p = mesh.point(v.get());
        [p.x, p.y, p.z]
    }).collect();
    assert!(!before.is_empty());
    let input = SmoothInput { mesh: &mesh, topo: &fx.complex, model: &fx.model };
    let r = SmoothRequest { method: Method::Laplacian { relax: 1.0 }, iterations: 30, fix_boundary: false, ..SmoothRequest::default() };
    let o = smooth(&input, &r).expect("smooth");
    assert!(!o.report.pinned.contains_key("deck"), "a mirror deck is not pinned");
    let set = mesh.classified_on(cap).expect("cap nodes");
    let mut moved = 0.0f64;
    for (k, v) in set.vertices.iter().enumerate() {
        let p = o.points[v.get()];
        assert!((p[2] - 0.5).abs() <= 1e-12, "cap node left the plane: z = {}", p[2]);
        let d = ((p[0] - before[k][0]).powi(2) + (p[1] - before[k][1]).powi(2)).sqrt();
        moved = moved.max(d);
    }
    assert!(moved > 1e-6, "a cap node slid in the plane (max move {moved:e})");
}

/// Find the two opposite brick faces at the extremes of `axis` from the classified nodes.
fn opposite_faces(f: &lattice::LatticeFixture, axis: usize) -> (sm_core::EntityId, sm_core::EntityId) {
    let faces: Vec<_> = f.mesh.classified_entities().iter().copied().filter(|e| e.kind() == EntityKind::TopoFace).collect();
    let lo = faces.iter().copied().find(|e| centroid_of(&f.mesh, *e)[axis].abs() < 1e-9).expect("low face");
    let hi = faces.iter().copied().find(|e| (centroid_of(&f.mesh, *e)[axis] - 1.0).abs() < 1e-9).expect("high face");
    (lo, hi)
}

fn pinned_entities(f: &lattice::LatticeFixture, faces: &[sm_core::EntityId]) -> Vec<sm_core::EntityId> {
    let mut v = faces.to_vec();
    for fc in faces {
        v.extend(f.topo.boundary(*fc).iter().map(|i| i.cell));
    }
    v.sort_unstable();
    v.dedup();
    v
}

fn assert_pinned_bit_identical(f: &lattice::LatticeFixture, start: &[[f64; 3]], pts: &[[f64; 3]], pinned: &[sm_core::EntityId], moved_elsewhere: bool) {
    let mut free_moved = false;
    for i in 0..start.len() {
        let e = f.mesh.classification(MeshEntityKind::Vertex, i as u32).entity;
        let same = (0..3).all(|k| start[i][k].to_bits() == pts[i][k].to_bits());
        if pinned.contains(&e) {
            assert!(same, "pinned node {i} on {e} moved");
        } else if !same {
            free_moved = true;
        }
    }
    assert_eq!(free_moved, moved_elsewhere);
}

#[test]
fn a6_translation_deck_pair_is_pinned_with_its_edges() {
    let f = lattice::hex([4, 4, 4], [1.0; 3]).expect("lattice");
    let (fa, fb) = opposite_faces(&f, 0);
    let mut p = f.perturbed(13, 0.2).expect("perturb");
    p.model.unassign(&[fa, fb]);
    let b = p.model.add_boundary("shift", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(1.0, 0.0, 0.0) })).expect("boundary");
    p.model.assign_pairs(b, &[(fa, fb)]).expect("pairs");
    let start = pts_of(&p.mesh);
    let o = run(&p, &req(Method::Laplacian { relax: 1.0 }, Schedule::Colored, 40, false));
    let pinned = pinned_entities(&p, &[fa, fb]);
    assert_pinned_bit_identical(&p, &start, &o.points, &pinned, true);
    assert!(o.report.pinned["deck"] > 0);
    assert_eq!(o.report.pinned_entities["deck"], { let mut v = vec![fa, fb]; v.sort_unstable(); v });
}

#[test]
fn a6_an_explicit_interface_face_is_pinned() {
    let f = lattice::hex([4, 4, 4], [1.0; 3]).expect("lattice");
    let (fa, _) = opposite_faces(&f, 2);
    let mut p = f.perturbed(13, 0.2).expect("perturb");
    p.model.unassign(&[fa]);
    let b = p.model.add_boundary("iface", BoundaryKind::Interface(InterfaceIntent::Conformal)).expect("boundary");
    p.model.assign_interface(b, &[fa], &[]).expect("interface");
    let start = pts_of(&p.mesh);
    let o = run(&p, &req(Method::Laplacian { relax: 1.0 }, Schedule::Jacobi, 40, false));
    let pinned = pinned_entities(&p, &[fa]);
    assert_pinned_bit_identical(&p, &start, &o.points, &pinned, true);
    assert!(o.report.pinned["interface"] > 0);
    assert_eq!(o.report.pinned_entities["interface"], vec![fa]);
}

#[test]
fn a6_a_response_boundary_is_not_pinned() {
    let f = lattice::hex([4, 4, 4], [1.0; 3]).expect("lattice");
    let p = f.perturbed(13, 0.2).expect("perturb");
    let o = run(&p, &req(Method::Laplacian { relax: 1.0 }, Schedule::Colored, 5, false));
    assert!(!o.report.pinned.contains_key("deck") && !o.report.pinned.contains_key("interface"));
    let _ = NodeSelect::All;
}
