//! Acceptance 3, 6, 7: extruded L, rotational and twisted sweeps from the sm-testkit fixtures.

mod common;

use common::{mesh_with, min_scaled_jacobian, registry, registry_with_sources, total_volume};
use core::f64::consts::{FRAC_PI_2, PI};
use sm_core::{Point3, Transform, Vector3};
use sm_mesh_api::{CellIx, MeshEntityKind, MeshRead, cell_volume, validate};
use sm_model_api::Model;
use sm_scheme_api::{
    Firm, Firmness, IntervalRequest, MeshIntent, SchemeChoice, Sequential, SizingSpec, Target,
};
use sm_testkit::topo::{
    SweepFixture, annular_sector, extruded_l, hollow_cylinder, regular_prism, twisted_prism,
};

fn sweep_choice(fx: &SweepFixture) -> SchemeChoice {
    SchemeChoice::with_params(
        "sweep",
        serde_json::json!({ "source": [fx.source], "target": [fx.target] }),
        Firmness::Hard,
    )
}

fn coords_of(m: &dyn MeshRead) -> Vec<[f64; 3]> {
    let c = m.coords();
    (0..c.x.len()).map(|i| [c.x[i], c.y[i], c.z[i]]).collect()
}

#[test]
fn fixture_invariants_are_as_documented() {
    let l = extruded_l(2.0, 1.0, 2.0).expect("L");
    assert_eq!(l.topo.counts, [12, 18, 8, 1, 1]);
    assert!((l.topo.volumes[0] - 6.0).abs() < 1e-14);
    assert_eq!(l.linking.len(), 6);
    let t = twisted_prism(2.0, 3.0, FRAC_PI_2).expect("twisted");
    assert!((t.topo.volumes[0] - 8.0).abs() < 1e-12);
    let a = annular_sector(1.0, 2.0, 1.0, FRAC_PI_2).expect("sector");
    assert!((a.topo.volumes[0] - 0.5 * FRAC_PI_2 * 3.0).abs() < 1e-12);
    assert_eq!(a.linking.len(), 4);
    let h = hollow_cylinder(1.0, 2.0, 2.0).expect("hollow");
    assert!((h.topo.volumes[0] - 6.0 * PI).abs() < 1e-12);
    let p = regular_prism(6, 1.0, 3.0).expect("prism");
    assert!((p.topo.volumes[0] - 7.794228634059948).abs() < 1e-12);
    assert_eq!(p.topo.counts[2], 8);
}

#[test]
fn extruded_l_has_48_hexes_and_105_nodes() {
    let fx = extruded_l(2.0, 1.0, 2.0).expect("L");
    let mut intent = MeshIntent::default();
    intent.defaults.sizing = Some(SizingSpec::Constant { size: 0.5 });
    intent.set_scheme(fx.volume, sweep_choice(&fx));
    // The frozen stub Submap meshes four-sided layouts only; the L uses a test-local lattice
    // quad scheme (see tests/common/surf.rs).
    intent.set_scheme(
        fx.source,
        SchemeChoice::with_params("gridface", serde_json::json!({ "h": 0.5 }), Firmness::Hard),
    );
    let out = mesh_with(
        &registry_with_sources(),
        &fx.topo.complex,
        &Model::new(),
        &[fx.volume],
        &intent,
        &Sequential,
    )
    .expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 48);
    assert_eq!(m.count(MeshEntityKind::Vertex), 21 * 5);
    assert!((total_volume(m) - 6.0).abs() < 1e-12);
    assert!((min_scaled_jacobian(m) - 1.0).abs() < 1e-12);
    // The column at the re-entrant corner (1, 1) is shared by two linking faces.
    let on_column = coords_of(m)
        .iter()
        .filter(|p| p[0] == 1.0 && p[1] == 1.0)
        .count();
    assert_eq!(on_column, 5);
}

/// Source rectangle 3x4 quads of the annular sector: radial 3, axial 4.
fn rotational(
    sweep: f64,
    layers: u32,
) -> (
    SweepFixture,
    sm_scheme_api::Meshed<sm_mesh_api::reference::ReferenceMesh>,
) {
    let fx = annular_sector(1.0, 2.0, 1.0, sweep).expect("sector");
    let mut intent = MeshIntent::default();
    // Source edges: radial (bottom/top) get 3, vertical get 4; columns (arcs) get `layers`.
    for e in &fx.columns {
        intent.set_intervals(*e, Firm::hard(IntervalRequest::Count(layers)));
    }
    intent.set_scheme(fx.volume, sweep_choice(&fx));
    intent.set_scheme(fx.source, SchemeChoice::new("map", Firmness::Hard));
    let topo = &fx.topo.complex;
    // Find the source edges by their geometry through the face loops.
    let edges = sm_topo_api::TopologyView::face_edges(topo, fx.source).to_vec();
    for e in edges {
        let (a, b) = sm_topo_api::TopologyViewExt::edge_ends(topo, e).expect("ends");
        let pa = sm_scheme_api::geom::vertex_point(topo, a).expect("a");
        let pb = sm_scheme_api::geom::vertex_point(topo, b).expect("b");
        let vertical = (pa.z - pb.z).abs() > 0.5;
        intent.set_intervals(
            e,
            Firm::hard(IntervalRequest::Count(if vertical { 4 } else { 3 })),
        );
    }
    let out = mesh_with(
        &registry(),
        topo,
        &Model::new(),
        &[fx.volume],
        &intent,
        &Sequential,
    )
    .expect("mesh");
    (fx, out)
}

fn check_rotational(sweep: f64, layers: u32) {
    let (fx, out) = rotational(sweep, layers);
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    let ncells = (12 * layers) as usize;
    assert_eq!(m.count(MeshEntityKind::Cell), ncells);
    // Every node: radius equals its source node's, angle = layer * sweep / L.
    let pts = coords_of(m);
    let step = sweep / f64::from(layers);
    let mut per_layer = vec![0usize; layers as usize + 1];
    for p in &pts {
        let r = p[0].hypot(p[1]);
        let a = p[1].atan2(p[0]).rem_euclid(2.0 * PI);
        let l = (a / step).round();
        assert!(
            (a - l * step).abs() < 1e-12 || (a - l * step - 2.0 * PI).abs() < 1e-12,
            "angle {a} not on a layer"
        );
        per_layer[l as usize] += 1;
        // Source radii: 1, 4/3, 5/3, 2.
        let rad = (r - 1.0) * 3.0;
        assert!(
            (rad - rad.round()).abs() < 1e-11,
            "radius {r} not on the source lattice"
        );
        assert!(
            (p[2] * 4.0 - (p[2] * 4.0).round()).abs() < 1e-11,
            "z {}",
            p[2]
        );
    }
    assert!(per_layer.iter().all(|n| *n == 4 * 5), "{per_layer:?}");
    // Volume of the inscribed sector prism.
    let l = f64::from(layers);
    let exact_discrete = 0.5 * l * (4.0 - 1.0) * 1.0 * step.sin();
    let v = total_volume(m);
    assert!(
        (v - exact_discrete).abs() / exact_discrete < 1e-12,
        "{v} vs {exact_discrete}"
    );
    let exact = 0.5 * sweep * 3.0;
    assert!(exact - v >= 0.0 && (exact - v) / exact <= step * step / 6.0 + 1e-15);
    assert!(
        (total_volume(m) - fx.topo.volumes[0]).abs() / fx.topo.volumes[0]
            <= step * step / 6.0 + 1e-15
    );
    // The detected copy transform is Rz(sweep).
    let copy = out
        .plan
        .problem
        .copy_into(Target::Surface(fx.target))
        .expect("copy constraint");
    let g = Transform::rotation(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        sweep,
    )
    .expect("rz");
    for i in 0..3 {
        for j in 0..3 {
            assert!((copy.item.transform.m[i][j] - g.m[i][j]).abs() < 1e-12);
        }
    }
    assert!(copy.item.transform.t.norm() < 1e-12);
    // Cell volumes are all positive.
    for c in 0..ncells {
        assert!(cell_volume(m, CellIx(c as u32)) > 0.0);
    }
}

#[test]
fn rotational_90_degrees_9_layers() {
    check_rotational(FRAC_PI_2, 9);
}

#[test]
fn rotational_270_degrees_18_layers() {
    check_rotational(1.5 * PI, 18);
}

#[test]
fn rotational_90_degree_mesh_volume_is_2_3442504() {
    let (_, out) = rotational(FRAC_PI_2, 9);
    assert!((total_volume(&out.finished.mesh) - 2.3442504).abs() < 1e-6);
}

#[test]
fn twisted_prism_layers_are_exact_affine_images() {
    let fx = twisted_prism(2.0, 3.0, FRAC_PI_2).expect("twisted");
    let layers = 6u32;
    let mut intent = MeshIntent::default();
    intent.set_scheme(fx.volume, sweep_choice(&fx));
    intent.set_scheme(fx.source, SchemeChoice::new("map", Firmness::Hard));
    let topo = &fx.topo.complex;
    for e in &fx.columns {
        intent.set_intervals(*e, Firm::hard(IntervalRequest::Count(layers)));
    }
    for e in sm_topo_api::TopologyView::face_edges(topo, fx.source).to_vec() {
        intent.set_intervals(e, Firm::hard(IntervalRequest::Count(4)));
    }
    let out = mesh_with(
        &registry(),
        topo,
        &Model::new(),
        &[fx.volume],
        &intent,
        &Sequential,
    )
    .expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 16 * 6);
    assert_eq!(m.count(MeshEntityKind::Vertex), 25 * 7);
    // Layer l of source node x is ((1-t) I + t Rz(pi/2)) x with z = 3 t, t = l / 6.
    let pts = coords_of(m);
    let mut hit = 0;
    for l in 0..=6 {
        let t = f64::from(l) / 6.0;
        for i in 0..5 {
            for j in 0..5 {
                let (x, y) = (-1.0 + 0.5 * f64::from(i), -1.0 + 0.5 * f64::from(j));
                // Rz(pi/2) (x, y) = (-y, x)
                let want = [(1.0 - t) * x + t * -y, (1.0 - t) * y + t * x, 3.0 * t];
                let found = pts
                    .iter()
                    .any(|p| (0..3).all(|k| (p[k] - want[k]).abs() < 1e-12));
                assert!(found, "no node at {want:?} (layer {l})");
                hit += 1;
            }
        }
    }
    assert_eq!(hit, 175);
    // Volume by exact 2x2x2 Gauss integration of the trilinear Jacobian.
    let (vol, least) = common::hex_gauss_volume(m);
    assert!(least > 0.0, "Gauss-point Jacobian {least}");
    assert!((vol - 8.0).abs() / 8.0 < 1e-12, "volume {vol}");
    assert!((total_volume(m) - 8.0).abs() / 8.0 < 0.1);
    let sj = min_scaled_jacobian(m);
    println!("twisted prism min scaled Jacobian = {sj:.6}");
    assert!(sj > 0.0);
}
