//! W0f gaps3: `record_merged`, `live_cover` on the complex and the overlay, `canonical_visit`.

// Scenario copied from sm-topo-api/tests/gaps3.rs and run against the production store (the
// reference's name is rebound so the assertions are untouched).
mod common;
use common::Dual as CellComplex;
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{CurveDef, Interval, Line};
use sm_topo_api::fixtures::{self, with_op};
use sm_topo_api::*;
use std::sync::Arc;

#[test]
fn record_merged_marks_a_retired_cell_merged_into_a_live_survivor() {
    let mut t = CellComplex::new();
    let [a, b, c] = with_op(&mut t, OpId(1), |t| {
        Ok([t.make_vertex(Point3::new(0.0, 0.0, 0.0))?, t.make_vertex(Point3::new(1.0, 0.0, 0.0))?, t.make_vertex(Point3::new(2.0, 0.0, 0.0))?])
    })
    .unwrap();
    let e = with_op(&mut t, OpId(2), |t| Ok(t.make_edge(a, c, None)?)).unwrap();
    // Outside an operation.
    assert!(matches!(t.record_merged(b, a, 0.0), Err(TopoError::NoOpenOp)));
    t.begin_op(OpId(3)).unwrap();
    // A live cell cannot be recorded as merged; nothing changes on error.
    assert!(matches!(t.record_merged(b, a, 0.0), Err(TopoError::InvalidParameter(_))));
    t.kill(b).unwrap();
    assert!(matches!(t.record_merged(b, b, 0.0), Err(TopoError::InvalidParameter(_))));
    assert!(matches!(t.record_merged(b, e, 0.0), Err(TopoError::InvalidParameter(_))), "rank mismatch");
    assert!(matches!(t.record_merged(b, a, f64::NAN), Err(TopoError::InvalidParameter(_))));
    assert!(matches!(t.record_merged(b, a, -1.0), Err(TopoError::InvalidParameter(_))));
    assert_eq!(t.state(b), Some(EntityState::Deleted(OpId(3))));
    t.record_merged(b, a, 0.25).unwrap();
    t.end_op().unwrap();
    assert_eq!(t.state(b), Some(EntityState::Merged(OpId(3))));
    assert_eq!(t.lineage().successors(b), vec![a]);
    assert!(t.lineage().of(b).any(|r| matches!(r.kind, LineageKind::Merged { into, gap } if into == a && gap == 0.25)));
    assert_eq!(t.live_cover(b), vec![a]);
    assert_eq!(t.live_cover(a), vec![a]);
}

#[test]
fn live_cover_on_the_complex_follows_splits() {
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3])).unwrap();
    let w = with_op(&mut t, OpId(2), |t| fixtures::webcut_brick(t, &b, 0.5)).unwrap();
    let _ = w;
    assert!(!t.is_alive(b.volume));
    let cover = t.live_cover(b.volume);
    assert_eq!(cover.len(), 2);
    assert!(cover.iter().all(|v| t.is_alive(*v)));
    assert!(cover.windows(2).all(|p| p[0] < p[1]));
}

#[test]
fn live_cover_on_the_overlay_maps_hidden_host_cells_to_every_live_piece() {
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3])).unwrap();
    let host: Arc<dyn TopologyView> = Arc::new(t);
    let mut ov = Overlay::new(host.clone());
    let y_edge = b.edges[6];
    let (mid, halves) = ov.partition_edge(y_edge, 0.5).unwrap();
    assert_eq!(ov.live_cover(y_edge), halves.to_vec());
    assert_eq!(ov.virtual_of(y_edge), Some(halves[0]), "virtual_of keeps first-child semantics");
    let cut = VirtualCut {
        from: mid,
        to: b.vertices[5],
        curve: CurveDef::Line(Line::new(Point3::new(0.0, 0.5, 1.0), Vector3::new(1.0, -0.5, 0.0)).unwrap()),
        range: Interval::new(0.0, (1.0f64 + 0.25).sqrt()),
    };
    let [c0, c1] = ov.partition_face(b.faces[5], cut).unwrap();
    assert_eq!(ov.live_cover(b.faces[5]), vec![c0, c1]);
    assert_eq!(ov.virtual_of(b.faces[5]), Some(c0));
    // Re-partition one half of the edge: the host edge now covers three live pieces.
    let (_, quarter) = ov.partition_edge(halves[0], 0.25).unwrap();
    let cover = ov.live_cover(y_edge);
    assert_eq!(cover.len(), 3);
    assert!(cover.contains(&halves[1]) && quarter.iter().all(|q| cover.contains(q)));
    assert!(cover.windows(2).all(|p| p[0] < p[1]));
    // Composite two adjacent host faces: each member and the interior edge map to it.
    let shared: Vec<EntityId> = ov.face_edges(b.faces[0]).iter().copied().filter(|e| ov.face_edges(b.faces[2]).contains(e)).collect();
    assert_eq!(shared.len(), 1);
    let vf = ov.composite_faces(&[b.faces[0], b.faces[2]]).unwrap();
    // The dissolved interior edge has no same-rank cover (it does not map to the face).
    assert!(ov.live_cover(shared[0]).is_empty());
    assert_eq!(ov.live_cover(b.faces[0]), vec![vf]);
    assert_eq!(ov.live_cover(b.faces[2]), vec![vf]);
    // Live cells map to themselves; undo restores the host cells.
    assert_eq!(ov.live_cover(b.faces[1]), vec![b.faces[1]]);
    ov.remove(vf).unwrap();
    assert_eq!(ov.live_cover(b.faces[0]), vec![b.faces[0]]);
    ov.remove(c0).unwrap();
    assert_eq!(ov.live_cover(b.faces[5]), vec![b.faces[5]]);
    // Persistence by replay rebuilds the cover table.
    let replayed = Overlay::from_data(host, ov.data()).unwrap();
    assert_eq!(replayed.live_cover(y_edge), ov.live_cover(y_edge));
}

#[test]
fn canonical_visit_folds_to_canonical_hash_and_separates_sub_quantum_changes() {
    let build = |dx: f64| {
        let mut t = CellComplex::new();
        with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(dx, 0.0, 0.0), [1.0; 3])).unwrap();
        t
    };
    let (a, b) = (build(0.0), build(1e-12));
    let words = |t: &CellComplex| {
        let mut v: Vec<HashWord> = Vec::new();
        t.canonical_visit(&mut |w| v.push(w));
        v
    };
    let (wa, wb) = (words(&a), words(&b));
    assert_eq!(wa.len(), wb.len());
    let ids: Vec<EntityId> = a.cells(Dim::Vertex).to_vec();
    assert!(!ids.is_empty());
    // Identity (quantized) words agree; raw values differ somewhere.
    let q = |w: &HashWord| match w {
        HashWord::Int(v) => *v as i64,
        HashWord::Float { q, .. } => *q,
    };
    assert!(wa.iter().zip(&wb).all(|(x, y)| q(x) == q(y)));
    assert!(wa.iter().zip(&wb).any(|(x, y)| x != y), "sub-quantum shift is visible in raw floats");
    assert_eq!(a.canonical_hash(), b.canonical_hash());
}
