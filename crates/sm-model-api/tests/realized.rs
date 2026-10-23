//! Realized splits (W0e decision 2) and the W0d composite follow-ups: the overlay mints the cut
//! cells, the decomposition records `CutId -> overlay ids`, every piece face resolves to cells of
//! the overlay, composites drop interface faces, and `pieces_of_face` agrees with `piece_faces`.

use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_model_api::*;
use sm_topo_api::fixtures::{self, BrickIds, with_op};
use sm_topo_api::{CellComplex, OpId, Overlay, TopologyView, TopologyViewExt};
use std::collections::BTreeSet;
use std::sync::Arc;

fn plane(n: [f64; 3], offset: f64) -> Plane {
    Plane::new(Vector3::new(n[0], n[1], n[2]), offset).unwrap()
}

fn colored(t: &CellComplex, vols: &[EntityId]) -> Model {
    let _ = t;
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, vols).unwrap();
    m
}

fn brick() -> (Arc<dyn TopologyView>, BrickIds, Model) {
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [2.0, 1.0, 1.0])).unwrap();
    let m = colored(&t, &[b.volume]);
    (Arc::new(t), b, m)
}

fn merged_bricks() -> (Arc<dyn TopologyView>, BrickIds, BrickIds, Model) {
    let mut t = CellComplex::new();
    let (a, b) = with_op(&mut t, OpId(1), |t| {
        let (a, b) = fixtures::two_bricks(t)?;
        Ok((a, b))
    })
    .unwrap();
    with_op(&mut t, OpId(2), |t| fixtures::merge_touching_bricks(t, &a, &b)).unwrap();
    let m = colored(&t, &[a.volume, b.volume]);
    (Arc::new(t), a, b, m)
}

/// Every active piece's faces resolve, and their cells are exactly the overlay volume's faces.
fn assert_pieces_resolve(m: &Model, ov: &Overlay, vols: &[EntityId]) {
    for v in vols {
        for p in m.pieces_of_volume(*v) {
            let id = match p {
                PieceRef::Whole(v) => v,
                PieceRef::Cell(c) => c,
            };
            let mut cells: BTreeSet<EntityId> = BTreeSet::new();
            for pf in m.decomposition().piece_faces(p, ov.host()) {
                let c = m.decomposition().piece_face_cells(&pf, ov).unwrap_or_else(|| panic!("{pf:?} of {p:?} does not resolve"));
                assert!(!c.is_empty());
                cells.extend(c);
            }
            let want: BTreeSet<EntityId> = ov.volume_faces(id).iter().copied().collect();
            assert_eq!(cells, want, "{p:?}");
        }
    }
}

#[test]
fn realized_split_records_the_overlay_cells_and_every_piece_face_resolves() {
    let (host, b, mut m) = brick();
    let mut ov = Overlay::new(host.clone());
    let [n, p] = m.split_realized(PieceRef::Whole(b.volume), plane([1.0, 0.0, 0.0], 1.0), &mut ov).unwrap();
    assert_eq!((n.kind(), p.kind()), (EntityKind::PartitionCell, EntityKind::PartitionCell));
    assert_eq!(ov.cells(sm_topo_api::Dim::Volume), &[n, p], "piece ids are the overlay volumes");
    assert!(m.decomposition().is_realized(PieceRef::Whole(b.volume)));
    let cut = m.decomposition().split_of(PieceRef::Whole(b.volume)).unwrap().cut;
    let cc = m.decomposition().cut_cells(cut).unwrap().clone();
    assert_eq!(cc.faces.len(), 1);
    assert_eq!(cc.faces[0].kind(), EntityKind::PartitionFace);
    assert_eq!((cc.edges.len(), cc.vertices.len(), cc.split_faces.len(), cc.split_edges.len()), (4, 4, 4, 4));
    assert!(cc.vertices.iter().all(|v| v.kind() == EntityKind::PartitionVertex));
    // The cut face from both sides is the overlay cut face; the -x face is whole on n.
    let d = m.decomposition();
    assert_eq!(d.piece_face_cells(&PieceFace::Cut { cut, side: Keep::Negative }, &ov), Some(cc.faces.clone()));
    assert_eq!(d.piece_face_cells(&PieceFace::Cut { cut, side: Keep::Positive }, &ov), Some(cc.faces.clone()));
    assert_eq!(d.piece_face_cells(&PieceFace::Whole(b.faces[0]), &ov), Some(vec![b.faces[0]]));
    // A straddling face: each part is the overlay half on its side.
    let part_n = d.piece_face_cells(&PieceFace::Part { face: b.faces[2], piece: n }, &ov).unwrap();
    assert_eq!(part_n.len(), 1);
    assert!(ov.face_outline(part_n[0]).iter().all(|q| q.x <= 1.0 + 1e-9));
    assert_eq!(ov.face_sides(cc.faces[0]).map(|s| s.left), Some(sm_topo_api::Region::Volume(n)));
    // The model face resolves to both halves for a piece that holds it whole (no one here), and
    // to nothing stale.
    let whole_side = d.piece_face_cells(&PieceFace::Whole(b.faces[2]), &ov).unwrap();
    assert_eq!(whole_side.len(), 2);
    assert_pieces_resolve(&m, &ov, &[b.volume]);
    // Nested split of the positive piece; the sibling still resolves.
    let [p1, p2] = m.split_realized(PieceRef::Cell(p), plane([1.0, 0.0, 0.0], 1.5), &mut ov).unwrap();
    assert_eq!(ov.cells(sm_topo_api::Dim::Volume), &[n, p1, p2]);
    assert_pieces_resolve(&m, &ov, &[b.volume]);
    // A split along another axis: the first cut face straddles and becomes CutParts.
    let [n1, n2] = m.split_realized(PieceRef::Cell(n), plane([0.0, 1.0, 0.0], 0.5), &mut ov).unwrap();
    assert!(m.decomposition().piece(n1).unwrap().faces.iter().any(|f| matches!(f, PieceFace::CutPart { cut: c, .. } if *c == cut)));
    assert_pieces_resolve(&m, &ov, &[b.volume]);
    // The undivided sibling's Cut key now spans both halves of the first cut face.
    let both = m.decomposition().piece_face_cells(&PieceFace::Cut { cut, side: Keep::Positive }, &ov);
    assert_eq!(both.map(|v| v.len()), Some(2));
    let _ = n2;
}

#[test]
fn a_cut_through_an_interface_partitions_the_neighbour_face() {
    let (host, a, b, mut m) = merged_bricks();
    let mut ov = Overlay::new(host.clone());
    // Cut volume a by y = 0.5: its +x face (shared with b) is partitioned.
    m.split_realized(PieceRef::Whole(a.volume), plane([0.0, 1.0, 0.0], 0.5), &mut ov).unwrap();
    let shared = a.faces[1];
    assert!(!ov.is_alive(shared));
    assert_eq!(m.decomposition().piece_face_cells(&PieceFace::Whole(shared), &ov).map(|v| v.len()), Some(2));
    assert_pieces_resolve(&m, &ov, &[a.volume, b.volume]);
}

#[test]
fn realized_split_is_atomic_and_undoes_exactly() {
    let (host, b, mut m) = brick();
    let mut ov = Overlay::new(host.clone());
    let (h0, d0) = (ov.canonical_hash(), format!("{:?}", m.decomposition()));
    // A plane that misses the volume: nothing changes.
    assert!(m.split_realized(PieceRef::Whole(b.volume), plane([1.0, 0.0, 0.0], 9.0), &mut ov).is_err());
    assert_eq!((ov.canonical_hash(), format!("{:?}", m.decomposition())), (h0, d0.clone()));
    let [n, _] = m.split_realized(PieceRef::Whole(b.volume), plane([1.0, 0.0, 0.0], 1.0), &mut ov).unwrap();
    // A plain unsplit would leave the overlay stale.
    assert!(m.unsplit(n).is_err());
    // Nested children block the undo; the undo of a leaf pair restores everything.
    let [c1, _] = m.split_realized(PieceRef::Cell(n), plane([1.0, 0.0, 0.0], 0.5), &mut ov).unwrap();
    let before = ov.canonical_hash();
    assert!(m.unsplit_realized(n, &mut ov).is_err());
    assert_eq!(ov.canonical_hash(), before);
    assert_eq!(m.unsplit_realized(c1, &mut ov).unwrap(), PieceRef::Cell(n));
    assert_eq!(m.unsplit_realized(n, &mut ov).unwrap(), PieceRef::Whole(b.volume));
    assert_eq!(ov.canonical_hash(), host.canonical_hash(), "every overlay operation was removed");
    assert_eq!(m.pieces_of_volume(b.volume).as_slice(), &[PieceRef::Whole(b.volume)]);
    assert!(m.decomposition().cut_cells(CutId(0)).is_none());
    // Ids are never reused.
    let [n2, _] = m.split_realized(PieceRef::Whole(b.volume), plane([1.0, 0.0, 0.0], 1.0), &mut ov).unwrap();
    assert!(n2 > n);
}

#[test]
fn realized_splits_are_deterministic() {
    let run = || {
        let (host, b, mut m) = brick();
        let mut ov = Overlay::new(host);
        let [_, p] = m.split_realized(PieceRef::Whole(b.volume), plane([1.0, 0.0, 0.0], 0.8), &mut ov).unwrap();
        m.split_realized(PieceRef::Cell(p), plane([0.0, 0.0, 1.0], 0.25), &mut ov).unwrap();
        (ov.canonical_hash(), format!("{:?}", m.decomposition()), ov.data())
    };
    let first = run();
    assert_eq!(first, run());
    let hs: Vec<_> = (0..4).map(|_| std::thread::spawn(run)).collect();
    for h in hs {
        assert_eq!(h.join().unwrap(), first);
    }
}

#[test]
fn composites_drop_interface_faces_held_by_two_volumes_and_the_index_agrees() {
    let (host, a, b, mut m) = merged_bricks();
    let t = host.as_ref();
    let shared = a.faces[1];
    // b is split by y = 0.5, so the interface is a Part of each half of b.
    let all: Vec<EntityId> = t.volume_faces(b.volume).to_vec();
    assert!(all.contains(&shared));
    let [bn, bp] = m.split(PieceRef::Whole(b.volume), CutGeometry::Plane(plane([0.0, 1.0, 0.0], 0.5)), &all, &all, t).unwrap();
    assert!(m.decomposition().piece(bn).unwrap().faces.contains(&PieceFace::Part { face: shared, piece: bn }));
    let comp = m.composite(&[PieceRef::Whole(a.volume), PieceRef::Cell(bn)], t).unwrap();
    let faces = m.decomposition().piece_faces(PieceRef::Cell(comp), t);
    assert!(!faces.iter().any(|f| f.base() == Some(shared)), "interface (Whole + Part from two volumes) is internal: {faces:?}");
    assert!(!m.pieces_of_face(shared).contains(&PieceRef::Cell(comp)));
    assert!(m.pieces_of_face(shared).contains(&PieceRef::Cell(bp)));
    // pieces_of_face and piece_faces agree for every active piece and face.
    let mut pieces: Vec<PieceRef> = m.pieces_of_volume(a.volume).to_vec();
    pieces.extend(m.pieces_of_volume(b.volume));
    pieces.sort();
    pieces.dedup();
    for p in &pieces {
        for f in m.decomposition().piece_faces(*p, t).iter().filter_map(PieceFace::base) {
            assert!(m.pieces_of_face_in(f, t).contains(p), "{p:?} misses {f}");
        }
    }
    for f in t.faces() {
        for p in m.pieces_of_face(*f) {
            assert!(m.decomposition().piece_faces(p, t).iter().any(|x| x.base() == Some(*f)), "{p:?} listed for {f}");
        }
    }
    // Two whole volumes: the merged face is internal too.
    m.uncomposite(comp).unwrap();
    m.unsplit(bn).unwrap();
    let comp2 = m.composite(&[PieceRef::Whole(a.volume), PieceRef::Whole(b.volume)], t).unwrap();
    assert!(!m.decomposition().piece_faces(PieceRef::Cell(comp2), t).iter().any(|f| f.base() == Some(shared)));
    assert!(m.pieces_of_face(shared).is_empty());
}

#[cfg(feature = "serde")]
mod serde_checks {
    use super::*;

    #[test]
    fn realized_tables_round_trip_and_sweep_cut_ids_are_checked() {
        let (host, b, mut m) = brick();
        let mut ov = Overlay::new(host.clone());
        let [n, p] = m.split_realized(PieceRef::Whole(b.volume), plane([1.0, 0.0, 0.0], 1.0), &mut ov).unwrap();
        let s = serde_json::to_string(&m).unwrap();
        let back: Model = serde_json::from_str(&s).unwrap();
        assert_eq!(format!("{:?}", back.decomposition()), format!("{:?}", m.decomposition()));
        assert_eq!(back.decomposition().piece_face_cells(&PieceFace::Part { face: b.faces[2], piece: p }, &ov), m.decomposition().piece_face_cells(&PieceFace::Part { face: b.faces[2], piece: p }, &ov));
        // A sweep hint naming a cut id that was never issued is rejected on load.
        m.set_shape(n, PieceShape::Sweep { source: vec![PieceFace::Whole(b.faces[0])], target: vec![PieceFace::Cut { cut: CutId(41), side: Keep::Negative }] }).unwrap();
        let s = serde_json::to_string(&m).unwrap();
        let e = serde_json::from_str::<Model>(&s).unwrap_err().to_string();
        assert!(e.contains("cut serial"), "{e}");
    }
}

#[cfg(feature = "schemars")]
#[test]
fn piece_keys_have_json_schemas() {
    let s = format!("{:?}", schemars::schema_for!(PieceFace));
    assert!(s.contains("CutPart") && s.contains("piece"), "{s}");
    let s = format!("{:?}", schemars::schema_for!(PieceRef));
    assert!(s.contains("Whole"), "{s}");
}
