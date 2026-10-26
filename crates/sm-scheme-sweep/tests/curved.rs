//! Acceptance 4, 5, 8 and the `strip_from` risk: cylinder, hollow cylinder and polyhedral
//! prism, with test-local source schemes.

mod common;

use common::{mesh_with, registry_with_sources, total_volume};
use core::f64::consts::PI;
use sm_core::{EntityId, Point3};
use sm_mesh_api::{CellIx, CellShape, MeshEntityKind, MeshRead, validate};
use sm_model_api::Model;
use sm_scheme_api::{
    Firm, Firmness, IntervalRequest, MeshIntent, NodeKey, SchemeChoice, Sequential, Target,
};
use sm_testkit::topo::{hollow_cylinder, regular_prism};
use sm_topo_api::fixtures::{CylinderIds, cylinder};
use sm_topo_api::{CellComplex, OpId};

fn sweep_params(source: EntityId, target: EntityId) -> SchemeChoice {
    SchemeChoice::with_params(
        "sweep",
        serde_json::json!({ "source": [source], "target": [target] }),
        Firmness::Hard,
    )
}

fn coords_of(m: &dyn MeshRead) -> Vec<[f64; 3]> {
    let c = m.coords();
    (0..c.x.len()).map(|i| [c.x[i], c.y[i], c.z[i]]).collect()
}

fn cylinder_setup(r: f64, h: f64) -> (CellComplex, CylinderIds) {
    let mut c = CellComplex::new();
    let ids =
        sm_topo_api::fixtures::with_op(&mut c, OpId(1), |t| cylinder(t, r, h)).expect("cylinder");
    (c, ids)
}

#[test]
fn axial_cylinder_is_64_wedges_on_the_circle() {
    let (topo, ids) = cylinder_setup(1.0, 2.0);
    let [bottom, top, _lateral] = ids.faces;
    let mut intent = MeshIntent::default();
    intent.set_intervals(ids.edges[0], Firm::hard(IntervalRequest::Count(16)));
    intent.set_intervals(ids.edges[1], Firm::hard(IntervalRequest::Count(16)));
    intent.set_intervals(ids.edges[2], Firm::hard(IntervalRequest::Count(4)));
    intent.set_scheme(ids.volume, sweep_params(bottom, top));
    intent.set_scheme(bottom, SchemeChoice::new("fan", Firmness::Hard));
    let out = mesh_with(
        &registry_with_sources(),
        &topo,
        &Model::new(),
        &[ids.volume],
        &intent,
        &Sequential,
    )
    .expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 64);
    for c in 0..64 {
        assert_eq!(m.cell_shape(CellIx(c)), CellShape::Wedge);
    }
    assert_eq!(m.count(MeshEntityKind::Vertex), 85);
    for p in coords_of(m) {
        let r = p[0].hypot(p[1]);
        assert!(r < 1e-12 || (r - 1.0).abs() < 1e-12, "node at r = {r}");
    }
    let want = 8.0 * (2.0 * PI / 16.0).sin() * 2.0;
    assert!((want - 6.122934918).abs() < 1e-9);
    assert!(
        (total_volume(m) - want).abs() / want < 1e-12,
        "{} vs {want}",
        total_volume(m)
    );
    // The target is meshed by copy.
    let tp = out.plan.target(Target::Surface(top)).expect("target");
    assert_eq!(tp.scheme.as_str(), "copy");
}

#[test]
fn hollow_cylinder_seam_strips_give_64_hexes() {
    let fx = hollow_cylinder(1.0, 2.0, 2.0).expect("hollow");
    let mut intent = MeshIntent::default();
    let topo = &fx.topo.complex;
    // Circles: 16 intervals each; seams: 4 layers.
    for e in sm_topo_api::TopologyView::face_edges(topo, fx.source).to_vec() {
        intent.set_intervals(e, Firm::hard(IntervalRequest::Count(16)));
    }
    for e in &fx.columns {
        intent.set_intervals(*e, Firm::hard(IntervalRequest::Count(4)));
    }
    intent.set_scheme(fx.volume, sweep_params(fx.source, fx.target));
    intent.set_scheme(fx.source, SchemeChoice::new("ring", Firmness::Hard));
    let out = mesh_with(
        &registry_with_sources(),
        topo,
        &Model::new(),
        &[fx.volume],
        &intent,
        &Sequential,
    )
    .expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 64);
    assert_eq!(m.count(MeshEntityKind::Vertex), 2 * 16 * 5);
    for c in 0..64 {
        assert_eq!(m.cell_shape(CellIx(c)), CellShape::Hex);
    }
    for p in coords_of(m) {
        let r = p[0].hypot(p[1]);
        assert!(
            (r - 1.0).abs() < 1e-12 || (r - 2.0).abs() < 1e-12,
            "node at r = {r}"
        );
    }
    let want = 8.0 * (2.0 * PI / 16.0).sin() * 3.0 * 2.0;
    assert!((want - 18.368804).abs() < 1e-5);
    assert!(
        (total_volume(m) - want).abs() / want < 1e-9,
        "{} vs {want}",
        total_volume(m)
    );
}

/// Risk 1: `strip_from` across a cylinder lateral face (the seam edge is used twice by the
/// face) reaches the matching node of the opposite circle for every circle node.
#[test]
fn strip_from_crosses_a_seam_face_to_the_opposite_circle() {
    let fx = hollow_cylinder(1.0, 2.0, 2.0).expect("hollow");
    let topo = &fx.topo.complex;
    let mut intent = MeshIntent::default();
    for e in sm_topo_api::TopologyView::face_edges(topo, fx.source).to_vec() {
        intent.set_intervals(e, Firm::hard(IntervalRequest::Count(16)));
    }
    for e in &fx.columns {
        intent.set_intervals(*e, Firm::hard(IntervalRequest::Count(4)));
    }
    intent.set_scheme(fx.volume, sweep_params(fx.source, fx.target));
    intent.set_scheme(fx.source, SchemeChoice::new("ring", Firmness::Hard));
    let out = mesh_with(
        &registry_with_sources(),
        topo,
        &Model::new(),
        &[fx.volume],
        &intent,
        &Sequential,
    )
    .expect("mesh");
    let ps = &out.patches;
    for &l in &fx.linking {
        let patch = ps.get(Target::Surface(l)).expect("linking patch");
        let circles: Vec<EntityId> = sm_topo_api::TopologyView::face_edges(topo, l)
            .iter()
            .copied()
            .filter(|e| !fx.columns.contains(e))
            .collect();
        assert_eq!(circles.len(), 2);
        // Bottom circle is the one at z = 0.
        let z_of = |e: EntityId| {
            let (a, _) = sm_topo_api::TopologyViewExt::edge_ends(topo, e).expect("ends");
            sm_scheme_api::geom::vertex_point(topo, a).expect("p").z
        };
        let (lo, hi) = if z_of(circles[0]) < z_of(circles[1]) {
            (circles[0], circles[1])
        } else {
            (circles[1], circles[0])
        };
        for k in 1..16u32 {
            let line = patch.strip_from(NodeKey::new(lo, k)).expect("strip");
            assert_eq!(line.len(), 5);
            assert_eq!(
                *line.last().expect("end"),
                NodeKey::new(hi, k),
                "circle node {k}"
            );
        }
    }
}

#[test]
fn polyhedral_prism_has_three_hexagonal_prisms() {
    let fx = regular_prism(6, 1.0, 3.0).expect("prism");
    let topo = &fx.topo.complex;
    let mut intent = MeshIntent::default();
    for e in sm_topo_api::TopologyView::face_edges(topo, fx.source).to_vec() {
        intent.set_intervals(e, Firm::hard(IntervalRequest::Count(1)));
    }
    for e in &fx.columns {
        intent.set_intervals(*e, Firm::hard(IntervalRequest::Count(3)));
    }
    intent.set_scheme(fx.volume, sweep_params(fx.source, fx.target));
    intent.set_scheme(fx.source, SchemeChoice::new("onepolygon", Firmness::Hard));
    let out = mesh_with(
        &registry_with_sources(),
        topo,
        &Model::new(),
        &[fx.volume],
        &intent,
        &Sequential,
    )
    .expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 3);
    for c in 0..3 {
        let c = CellIx(c);
        assert_eq!(m.cell_shape(c), CellShape::Polyhedron);
        let faces = m.cell_faces(c);
        assert_eq!(faces.len(), 8, "F");
        let mut verts: Vec<u32> = Vec::new();
        let mut loop_len = 0;
        for sf in faces {
            let fv = m.face_vertices(sf.face());
            loop_len += fv.len();
            verts.extend(fv.iter().map(|v| v.0));
        }
        verts.sort_unstable();
        verts.dedup();
        assert!(
            sm_mesh_api::cell_volume(m, c) > 0.0,
            "outward faces: positive volume"
        );
        assert_eq!(verts.len(), 12, "V");
        assert_eq!(loop_len / 2, 18, "E");
    }
    let want = 7.794228634059948;
    assert!(
        (total_volume(m) - want).abs() < 1e-12,
        "{}",
        total_volume(m)
    );
    let _ = Point3::default();
}
