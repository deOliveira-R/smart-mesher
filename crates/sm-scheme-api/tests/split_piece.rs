//! End to end through the decomposition: a brick is split by a realized plane cut
//! (`Model::split_realized`), and the two pieces are meshed through the overlay with the
//! `DecompositionFaces` resolver. Mesh entities on the cut are classified on the overlay's
//! `Partition*` cells and the cells on the pieces (W0e decisions 1 and 2).

mod stub_schemes;

use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{
    CellIx, FaceKind, MeshRead, SerialAlloc, cell_volume, geometry_hash, topology_hash, validate,
};
use sm_model_api::{Domain, Model, PieceRef, Plane};
use sm_scheme_api::{
    DecompositionFaces, Driver, Firm, Firmness, IntervalRequest, MeshInput, MeshIntent,
    ReferenceCornerPicker, ReferenceIntervalSolver, SchemeChoice, SchemeError, ScopedThreads,
    Sequential, WholeFaces,
};
use sm_topo_api::fixtures::{BrickIds, brick, with_op};
use sm_topo_api::{CellComplex, OpId, Overlay, TopologyView};
use std::sync::Arc;

struct Split {
    overlay: Overlay,
    model: Model,
    ids: BrickIds,
    pieces: [EntityId; 2],
    cut_face: EntityId,
}

/// A 2 x 1 x 1 brick cut at x = 0.8 into `[negative, positive]` pieces.
fn split() -> Split {
    let mut t = CellComplex::new();
    let ids = with_op(&mut t, OpId(1), |t| brick(t, Point3::new(0.0, 0.0, 0.0), [2.0, 1.0, 1.0])).unwrap();
    let host: Arc<dyn TopologyView> = Arc::new(t);
    let mut overlay = Overlay::new(host);
    let mut model = Model::new();
    let mat = model.add_color(1, "fuel", Domain::Volume).unwrap();
    model.color_volumes(mat, &[ids.volume]).unwrap();
    let pieces = model
        .split_realized(PieceRef::Whole(ids.volume), Plane::new(Vector3::new(1.0, 0.0, 0.0), 0.8).unwrap(), &mut overlay)
        .unwrap();
    let cut = model.decomposition().split_of(PieceRef::Whole(ids.volume)).unwrap().cut;
    let cut_face = model.decomposition().cut_cells(cut).unwrap().faces[0];
    Split { overlay, model, ids, pieces, cut_face }
}

fn intent(s: &Split, n: [u32; 3]) -> MeshIntent {
    let mut intent = MeshIntent::default();
    // y and z edges of the host brick that survive whole; x counts per piece on the halves.
    intent.set_intervals(s.ids.edges[4], Firm::hard(IntervalRequest::Count(n[1])));
    intent.set_intervals(s.ids.edges[8], Firm::hard(IntervalRequest::Count(n[2])));
    let cc = s.model.decomposition().cut_cells(sm_model_api::CutId(0)).unwrap();
    for (_, _, halves) in &cc.split_edges {
        for h in halves {
            intent.set_intervals(*h, Firm::hard(IntervalRequest::Count(n[0])));
        }
    }
    for p in s.pieces {
        intent.set_scheme(p, SchemeChoice::new("sweep", Firmness::Hard));
    }
    intent
}

fn mesh(s: &Split, sel: &[EntityId], threads: usize) -> sm_scheme_api::Meshed<sm_mesh_api::reference::ReferenceMesh> {
    let reg = stub_schemes::registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let input = MeshInput::new(&s.overlay, &s.model, &DecompositionFaces, sel);
    let it = intent(s, [2, 2, 3]);
    if threads > 1 {
        driver.mesh(&input, &it, &ScopedThreads(threads), ReferenceBuilder::new(), &mut SerialAlloc::new()).expect("mesh")
    } else {
        driver.mesh(&input, &it, &Sequential, ReferenceBuilder::new(), &mut SerialAlloc::new()).expect("mesh")
    }
}

#[test]
fn both_pieces_of_a_realized_split_mesh_end_to_end() {
    let s = split();
    let out = mesh(&s, &s.pieces, 1);
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    // Each piece: 2 x 2 x 3 hexes (x halves 2, y 2, z 3, by sweeping x or any direction).
    assert_eq!(m.cell_count(), 2 * 12);
    let mut vol = [0.0; 2];
    for c in 0..m.cell_count() {
        let ci = CellIx::new(c);
        let on = m.cell_class(ci).entity;
        let k = s.pieces.iter().position(|p| *p == on).expect("cells lie on the pieces");
        vol[k] += cell_volume(m, ci);
    }
    assert!((vol[0] - 0.8).abs() < 1e-12 && (vol[1] - 1.2).abs() < 1e-12, "{vol:?}");
    // The cut face is meshed once, shared by both pieces: interior faces on the PartitionFace.
    let cut = m.classified_on(s.cut_face).expect("faces on the cut");
    assert_eq!(cut.faces.len(), 6, "y (2) by z (3)");
    assert!(cut.faces.iter().all(|f| m.face_kind(*f) == FaceKind::Interior));
    assert!(cut.faces.iter().all(|f| m.face_vertices(*f).iter().all(|v| (m.point(v.get()).x - 0.8).abs() < 1e-12)));
    assert_eq!(cut.vertices.len(), 2, "two nodes inside the cut face");
    // Nodes on the cut's virtual edges and vertices are classified on them.
    let mut kinds: Vec<EntityKind> = m.classified_entities().iter().map(|e| e.kind()).collect();
    kinds.sort();
    kinds.dedup();
    for k in [EntityKind::PartitionVertex, EntityKind::PartitionEdge, EntityKind::PartitionFace, EntityKind::PartitionCell] {
        assert!(kinds.contains(&k), "{k:?} missing from {kinds:?}");
    }
    let cc = s.model.decomposition().cut_cells(sm_model_api::CutId(0)).unwrap();
    for v in &cc.vertices {
        assert_eq!(m.classified_on(*v).map(|c| c.vertices.len()), Some(1));
    }
    // A model face cut in two: its halves (PartitionFace) carry the boundary faces.
    for (_, [neg, pos]) in &cc.split_faces {
        let (a, b) = (m.classified_on(*neg).unwrap(), m.classified_on(*pos).unwrap());
        assert!(a.faces.iter().chain(&b.faces).all(|f| m.face_kind(*f) == FaceKind::Boundary));
        // x halves have 2 intervals each; the other direction is y (2) or z (3).
        assert_eq!(a.faces.len(), b.faces.len());
        assert!(a.faces.len() == 4 || a.faces.len() == 6, "{}", a.faces.len());
    }
}

#[test]
fn one_piece_alone_has_its_cut_face_on_the_boundary() {
    let s = split();
    let out = mesh(&s, &s.pieces[..1], 1);
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.cell_count(), 12);
    let cut = m.classified_on(s.cut_face).expect("faces on the cut");
    assert_eq!(cut.faces.len(), 6, "y (2) by z (3)");
    assert!(cut.faces.iter().all(|f| m.is_boundary_face(*f)));
    let total: f64 = (0..12).map(|c| cell_volume(m, CellIx::new(c))).sum();
    assert!((total - 0.8).abs() < 1e-12);
}

#[test]
fn the_whole_faces_resolver_refuses_cut_faces_with_a_clear_error() {
    let s = split();
    let reg = stub_schemes::registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let input = MeshInput::new(&s.overlay, &s.model, &WholeFaces, &s.pieces);
    match driver.plan(&input, &intent(&s, [2, 2, 3])) {
        Err(SchemeError::Input(r)) => assert!(r.contains("decomposition-overlay resolver"), "{r}"),
        other => panic!("expected an input error, got {:?}", other.map(|_| ())),
    }
}

#[test]
fn split_piece_meshing_is_deterministic() {
    let s = split();
    let first = mesh(&s, &s.pieces, 1);
    let key = |o: &sm_scheme_api::Meshed<sm_mesh_api::reference::ReferenceMesh>| {
        (topology_hash(&o.finished.mesh), geometry_hash(&o.finished.mesh), serde_json::to_string(&o.plan).unwrap())
    };
    let k0 = key(&first);
    assert_eq!(key(&mesh(&s, &s.pieces, 1)), k0);
    assert_eq!(key(&mesh(&s, &s.pieces, 4)), k0);
    let s2 = split();
    assert_eq!(key(&mesh(&s2, &s2.pieces, 1)), k0, "rebuilding model and overlay gives the same mesh");
}
