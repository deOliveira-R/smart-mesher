//! Acceptance 1 and 2: bricks and composites through `Driver::mesh` with individual stubs and
//! the real sweep; layer counts from intervals, sizes, bias and the `layers` row.

mod common;

use common::{
    mesh_with, min_scaled_jacobian, plan_only, registry, registry_with_bias, total_volume,
};
use sm_core::{EntityId, Vector3};
use sm_mesh_api::{MeshEntityKind, MeshRead, validate};
use sm_model_api::{Domain, PieceFace, PieceRef, PieceShape};
use sm_scheme_api::{
    Firm, Firmness, IntervalConstraint, IntervalRequest, MeshIntent, SchemeChoice, SchemeError,
    Sequential, SizingSpec, Target,
};
use sm_topo_api::fixtures::BrickIds;

const X: usize = 0;
const Y: usize = 4;
const Z: usize = 8;

fn sweep_z(ids: &BrickIds) -> SchemeChoice {
    SchemeChoice::with_params(
        "sweep",
        serde_json::json!({ "source": [ids.faces[4]], "target": [ids.faces[5]] }),
        Firmness::Hard,
    )
}

fn counts(ids: &BrickIds, n: [u32; 3]) -> MeshIntent {
    let mut intent = MeshIntent::default();
    intent.set_intervals(ids.edges[X], Firm::hard(IntervalRequest::Count(n[0])));
    intent.set_intervals(ids.edges[Y], Firm::hard(IntervalRequest::Count(n[1])));
    intent.set_intervals(ids.edges[Z], Firm::hard(IntervalRequest::Count(n[2])));
    intent
}

fn coords_of(m: &dyn MeshRead) -> Vec<[f64; 3]> {
    let c = m.coords();
    (0..c.x.len()).map(|i| [c.x[i], c.y[i], c.z[i]]).collect()
}

#[test]
fn brick_4x3x5_is_a_bit_exact_lattice() {
    let (topo, ids, model) = sm_scheme_api::stubs::brick_setup([4.0, 3.0, 5.0]);
    let mut intent = counts(&ids, [4, 3, 5]);
    intent.set_scheme(ids.volume, sweep_z(&ids));
    let out = mesh_with(&registry(), &topo, &model, &[ids.volume], &intent, &Sequential).expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 60);
    assert_eq!(m.count(MeshEntityKind::Vertex), 5 * 4 * 6);
    assert!((total_volume(m) - 60.0).abs() < 1e-12);
    for p in coords_of(m) {
        for c in p {
            assert_eq!(c, c.round(), "{p:?} is off the integer lattice");
        }
    }
    assert!((min_scaled_jacobian(m) - 1.0).abs() < 1e-12);
    // Hex cells, not polyhedra.
    for c in 0..60 {
        assert_eq!(m.cell_shape(sm_mesh_api::CellIx(c)), sm_mesh_api::CellShape::Hex);
    }
}

#[test]
fn a_column_edge_count_of_seven_gives_seven_layers() {
    let (topo, ids, model) = sm_scheme_api::stubs::brick_setup([1.0, 1.0, 1.0]);
    let mut intent = counts(&ids, [2, 2, 7]);
    intent.set_scheme(ids.volume, sweep_z(&ids));
    let out = mesh_with(&registry(), &topo, &model, &[ids.volume], &intent, &Sequential).expect("mesh");
    assert_eq!(out.finished.mesh.count(MeshEntityKind::Cell), 4 * 7);
    for e in 8..12 {
        assert_eq!(out.plan.intervals(ids.edges[e]), Some(7));
    }
}

#[test]
fn size_two_on_a_height_ten_brick_gives_five_layers() {
    let (topo, ids, model) = sm_scheme_api::stubs::brick_setup([2.0, 2.0, 10.0]);
    let mut intent = MeshIntent::default();
    intent.defaults.sizing = Some(SizingSpec::Constant { size: 2.0 });
    intent.set_scheme(ids.volume, sweep_z(&ids));
    let out = mesh_with(&registry(), &topo, &model, &[ids.volume], &intent, &Sequential).expect("mesh");
    let m = &out.finished.mesh;
    assert_eq!(m.count(MeshEntityKind::Cell), 5);
    assert!((total_volume(m) - 40.0).abs() < 1e-12);
}

/// Two merged bricks as one composite piece swept along x (the shape hint of the planner).
fn composite(
    counts: Option<(u32, u32)>,
    layers: Option<u32>,
) -> (
    sm_scheme_api::Result<sm_scheme_api::Meshed<sm_mesh_api::reference::ReferenceMesh>>,
    Vec<BrickIds>,
    EntityId,
) {
    let (topo, ids, mut model) = sm_scheme_api::stubs::row_setup(2);
    let mat = model.add_color(1, "mat", Domain::Volume).expect("color");
    model.color_volumes(mat, &[ids[0].volume, ids[1].volume]).expect("color volumes");
    let cell = model
        .composite(
            &[PieceRef::Whole(ids[0].volume), PieceRef::Whole(ids[1].volume)],
            &topo,
        )
        .expect("composite");
    model
        .set_shape(
            cell,
            PieceShape::sweep(
                vec![PieceFace::Whole(ids[0].faces[0])],
                vec![PieceFace::Whole(ids[1].faces[1])],
            ),
        )
        .expect("shape");
    let mut intent = MeshIntent::default();
    if let Some((a, b)) = counts {
        intent.set_intervals(ids[0].edges[0], Firm::hard(IntervalRequest::Count(a)));
        intent.set_intervals(ids[1].edges[0], Firm::hard(IntervalRequest::Count(b)));
    }
    if let Some(n) = layers {
        intent.set_scheme(
            cell,
            SchemeChoice::with_params(
                "sweep",
                serde_json::json!({
                    "source": [ids[0].faces[0]],
                    "target": [ids[1].faces[1]],
                    "layers": n,
                }),
                Firmness::Hard,
            ),
        );
    }
    let out = mesh_with(&registry(), &topo, &model, &[cell], &intent, &Sequential);
    (out, ids, cell)
}

#[test]
fn a_composite_of_two_bricks_with_columns_3_and_4_gives_seven_layers() {
    let (out, ids, cell) = composite(Some((3, 4)), None);
    let out = out.expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 7);
    assert!((total_volume(m) - 2.0).abs() < 1e-12);
    assert_eq!(out.plan.intervals(ids[0].edges[0]), Some(3));
    assert_eq!(out.plan.intervals(ids[1].edges[0]), Some(4));
    assert_eq!(out.plan.target(Target::Volume(cell)).map(|t| t.scheme.as_str()), Some("sweep"));
}

#[test]
fn layers_forces_a_linear_row_on_a_two_edge_column() {
    let (out, ids, cell) = composite(None, Some(6));
    let out = out.expect("mesh");
    let m = &out.finished.mesh;
    assert_eq!(m.count(MeshEntityKind::Cell), 6);
    let n0 = out.plan.intervals(ids[0].edges[0]).expect("a");
    let n1 = out.plan.intervals(ids[1].edges[0]).expect("b");
    assert_eq!(n0 + n1, 6);
    // The row is a scheme-origin linear constraint over the column's two edges.
    let rows: Vec<_> = out
        .plan
        .problem
        .intervals
        .iter()
        .filter(|r| matches!(r.item, IntervalConstraint::Linear { .. }))
        .filter(|r| {
            r.item.edges() == {
                let mut e = vec![ids[0].edges[0], ids[1].edges[0]];
                e.sort_unstable();
                e
            }
        })
        .collect();
    assert!(!rows.is_empty(), "no linear layer row in the plan for {cell}");
}

#[test]
fn conflicting_hard_counts_are_infeasible_and_name_both_edges() {
    let (topo, ids, model) = sm_scheme_api::stubs::brick_setup([1.0, 1.0, 1.0]);
    let mut intent = MeshIntent::default();
    intent.set_intervals(ids.edges[8], Firm::hard(IntervalRequest::Count(3)));
    intent.set_intervals(ids.edges[9], Firm::hard(IntervalRequest::Count(4)));
    intent.set_scheme(ids.volume, sweep_z(&ids));
    match plan_only(&registry(), &topo, &model, &[ids.volume], &intent) {
        Err(SchemeError::Infeasible(i)) => {
            assert!(i.entities.contains(&ids.edges[8]) && i.entities.contains(&ids.edges[9]), "{i}");
        }
        other => panic!("expected Infeasible, got {:?}", other.map(|_| ())),
    }
    // `layers` against a hard count.
    let mut intent = MeshIntent::default();
    intent.set_intervals(ids.edges[8], Firm::hard(IntervalRequest::Count(3)));
    intent.set_scheme(
        ids.volume,
        SchemeChoice::with_params(
            "sweep",
            serde_json::json!({
                "source": [ids.faces[4]], "target": [ids.faces[5]], "layers": 5
            }),
            Firmness::Hard,
        ),
    );
    match plan_only(&registry(), &topo, &model, &[ids.volume], &intent) {
        Err(SchemeError::Infeasible(i)) => assert!(i.entities.contains(&ids.edges[8]), "{i}"),
        other => panic!("expected Infeasible, got {:?}", other.map(|_| ())),
    }
}

#[test]
fn bias_on_the_column_edges_gives_a_geometric_layer_ratio_and_exact_interior_levels() {
    let (topo, ids, model) = sm_scheme_api::stubs::brick_setup([2.0, 2.0, 3.0]);
    let mut intent = counts(&ids, [2, 2, 6]);
    for e in 8..12 {
        intent.set_scheme(
            ids.edges[e],
            SchemeChoice::with_params("bias", serde_json::json!({ "ratio": 1.2 }), Firmness::Hard),
        );
    }
    intent.set_scheme(ids.volume, sweep_z(&ids));
    let out = mesh_with(&registry_with_bias(), &topo, &model, &[ids.volume], &intent, &Sequential)
        .expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    // Boundary column z levels: the nodes at x = y = 0.
    let pts = coords_of(m);
    let mut col: Vec<f64> = pts
        .iter()
        .filter(|p| p[0] == 0.0 && p[1] == 0.0)
        .map(|p| p[2])
        .collect();
    col.sort_by(f64::total_cmp);
    assert_eq!(col.len(), 7);
    let h: Vec<f64> = col.windows(2).map(|w| w[1] - w[0]).collect();
    // The column edge runs bottom to top or top to bottom: the ratio is 1.2 one way.
    let ratio: Vec<f64> = h.windows(2).map(|w| w[1] / w[0]).collect();
    let up = ratio.iter().all(|r| (r - 1.2).abs() < 1e-12);
    let down = ratio.iter().all(|r| (r - 1.0 / 1.2).abs() < 1e-12);
    assert!(up || down, "ratios {ratio:?}");
    // Every node (interior ones included) lies on one of the boundary column's levels.
    for p in &pts {
        assert!(col.iter().any(|z| (z - p[2]).abs() < 1e-12), "z {} is not a column level", p[2]);
    }
    let _ = Vector3::default();
}
