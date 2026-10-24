//! Virtual volume splits in the decomposition overlay (W0e decision 2): the cut face, its edges
//! and vertices are `Partition*` cells, the two volumes take caller-supplied `PartitionCell`
//! ids, the host is untouched, and the split replays, undoes and nests.

// Scenario copied from sm-topo-api/tests/overlay_volume.rs and run against the production store (the
// reference's name is rebound so the assertions are untouched).
mod common;
use common::Dual as CellComplex;
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_topo_api::fixtures::{self, BrickIds, with_op};
use sm_topo_api::*;
use std::sync::Arc;

fn cell(n: u64) -> EntityId {
    EntityId::new(EntityKind::PartitionCell, n).unwrap()
}

fn brick() -> (Arc<dyn TopologyView>, BrickIds) {
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 2.0, 3.0])).unwrap();
    (Arc::new(t), b)
}

fn newell(pts: &[Point3]) -> Vector3 {
    let mut n = Vector3::default();
    for i in 0..pts.len() {
        let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
        n = n + Vector3::new((a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y));
    }
    n
}

fn cut_at(ov: &mut Overlay, v: EntityId, z: f64, kids: [EntityId; 2]) -> Result<PlaneCut, TopoError> {
    ov.partition_volume_by_plane(v, Point3::new(0.0, 0.0, z), Vector3::new(0.0, 0.0, 1.0), kids)
}

#[test]
fn plane_split_creates_partition_cells_and_two_virtual_volumes() {
    let (host, b) = brick();
    let h0 = host.canonical_hash();
    let mut ov = Overlay::new(host.clone());
    let pc = cut_at(&mut ov, b.volume, 1.2, [cell(10), cell(11)]).unwrap();
    assert_eq!(host.canonical_hash(), h0, "host untouched");
    assert_eq!(pc.face.kind(), EntityKind::PartitionFace);
    assert_eq!(pc.children, [cell(10), cell(11)]);
    assert_eq!(pc.split_edges.len(), 4);
    assert_eq!(pc.split_faces.len(), 4);
    assert!(pc.split_edges.iter().all(|(e, v, h)| v.kind() == EntityKind::PartitionVertex && h.iter().all(|x| x.kind() == EntityKind::PartitionEdge) && b.edges.contains(e)));
    assert!(pc.split_faces.iter().all(|(_, c, h)| c.kind() == EntityKind::PartitionEdge && h.iter().all(|x| x.kind() == EntityKind::PartitionFace)));
    assert_eq!(pc.loop_edges.len(), 4);
    assert!(pc.loop_edges.iter().all(|e| e.kind() == EntityKind::PartitionEdge));
    assert_eq!(pc.loop_vertices.len(), 4);
    // Counts: 8 + 4 vertices, 12 - 4 + 8 + 4 edges, 6 - 4 + 8 + 1 faces, 2 volumes.
    assert_eq!(ov.cells(Dim::Vertex).len(), 12);
    assert_eq!(ov.cells(Dim::Edge).len(), 20);
    assert_eq!(ov.cells(Dim::Face).len(), 11);
    assert_eq!(ov.cells(Dim::Volume), &[cell(10), cell(11)]);
    assert_eq!(euler_characteristic(&ov), 3, "V - E + F = 1 + C for a ball split in two");
    assert!(!ov.is_alive(b.volume));
    assert_eq!(ov.dim_of(cell(10)), Some(Dim::Volume));
    assert_eq!(ov.resolve(cell(10)), &[b.volume]);
    assert_eq!(ov.virtual_of(b.volume), Some(cell(10)));
    assert_eq!(ov.shell_kind(cell(10), 0), Some(ShellKind::Outer));
    assert!(ov.volume_faces(b.volume).is_empty());
    assert!(matches!(ov.volume_moments(cell(10)), Err(QueryError::Unsupported(_))));
    // The cut face separates negative (left) from positive (right); its normal points +z.
    let sides = ov.face_sides(pc.face).unwrap();
    assert_eq!((sides.left, sides.right), (Region::Volume(cell(10)), Region::Volume(cell(11))));
    let n = newell(&ov.face_outline(pc.face));
    assert!(n.z > 0.0 && n.x.abs() < 1e-12 && n.y.abs() < 1e-12, "{n:?}");
    let outline = ov.face_outline(pc.face);
    assert!(outline.iter().all(|p| (p.z - 1.2).abs() < 1e-9));
    assert!(matches!(ov.carrier(pc.face), Carrier::Surface(_)));
    // Each child has the far cap, four side halves and the cut face; the bottom goes negative.
    let f0 = ov.volume_faces(cell(10)).to_vec();
    let f1 = ov.volume_faces(cell(11)).to_vec();
    assert_eq!((f0.len(), f1.len()), (6, 6));
    assert!(f0.contains(&b.faces[4]) && f1.contains(&b.faces[5]));
    // Host faces copied into the layer keep their face -> edge rows.
    assert_eq!(ov.face_edges(b.faces[4]), host.face_edges(b.faces[4]));
    assert!(f0.contains(&pc.face) && f1.contains(&pc.face));
    for (face, _, [neg, pos]) in &pc.split_faces {
        assert!(b.faces.contains(face));
        assert!(f0.contains(neg) && f1.contains(pos));
        assert!(ov.face_outline(*neg).iter().all(|p| p.z <= 1.2 + 1e-9));
        assert!(ov.face_outline(*pos).iter().all(|p| p.z >= 1.2 - 1e-9));
    }
    // Every face use of a child keeps its shell closed: each edge is used twice per child.
    for c in [cell(10), cell(11)] {
        let mut uses: std::collections::BTreeMap<EntityId, u32> = Default::default();
        for f in ov.volume_faces(c) {
            for e in ov.face_edges(*f) {
                *uses.entry(*e).or_default() += 1;
            }
        }
        assert!(uses.values().all(|n| *n == 2), "{c}: {uses:?}");
    }
    let issues = validate(&ov);
    assert!(issues.is_empty(), "{issues:#?}");
}

#[test]
fn volume_split_replays_undoes_and_nests() {
    let (host, b) = brick();
    let mut ov = Overlay::new(host.clone());
    let pc = cut_at(&mut ov, b.volume, 1.0, [cell(1), cell(2)]).unwrap();
    let after_first = ov.canonical_hash();
    // Nested split of the upper piece.
    let pc2 = cut_at(&mut ov, cell(2), 2.0, [cell(3), cell(4)]).unwrap();
    assert_eq!(ov.cells(Dim::Volume), &[cell(1), cell(3), cell(4)]);
    assert_eq!(ov.face_sides(pc2.face).map(|s| (s.left, s.right)), Some((Region::Volume(cell(3)), Region::Volume(cell(4)))));
    assert!(ov.volume_faces(cell(3)).contains(&pc.face), "the first cut face bounds the lower grandchild");
    // Persistence by replay is exact.
    let data = ov.data();
    let replay = Overlay::from_data(host.clone(), data.clone()).unwrap();
    assert_eq!(replay.canonical_hash(), ov.canonical_hash());
    assert_eq!(replay.data(), data);
    // Undo the nested volume split: the upper piece is back with its id.
    ov.remove(pc2.face).unwrap();
    assert_eq!(ov.cells(Dim::Volume), &[cell(1), cell(2)]);
    assert!(ov.is_alive(cell(2)));
    // A volume split that a later one depends on cannot be removed.
    let mut nested = replay.clone();
    assert!(matches!(nested.remove(pc.face), Err(TopoError::StillReferenced(_))));
    // Removing everything restores the host view; ids are never reused.
    let mut all = Overlay::from_data(host.clone(), ov.data()).unwrap();
    all.remove(pc.face).unwrap();
    assert_ne!(all.canonical_hash(), after_first);
    assert!(all.is_alive(b.volume));
    let again = cut_at(&mut all, b.volume, 1.0, [cell(5), cell(6)]).unwrap();
    assert_ne!(again.face, pc.face, "face ids are not reused");
}

#[test]
fn volume_split_is_atomic_on_error() {
    let (host, b) = brick();
    let mut ov = Overlay::new(host.clone());
    let h = ov.canonical_hash();
    let d = ov.data();
    // The plane misses the volume.
    assert!(cut_at(&mut ov, b.volume, 5.0, [cell(1), cell(2)]).is_err());
    // Child ids that are not fresh PartitionCell ids, or not ascending.
    assert!(cut_at(&mut ov, b.volume, 1.0, [cell(2), cell(1)]).is_err());
    assert!(cut_at(&mut ov, b.volume, 1.0, [EntityId::new(EntityKind::TopoVolume, 9).unwrap(), cell(1)]).is_err());
    // The plane contains a face.
    assert!(cut_at(&mut ov, b.volume, 0.0, [cell(1), cell(2)]).is_err());
    assert_eq!(ov.canonical_hash(), h);
    assert_eq!(ov.data(), d);
    // The primitive refuses a loop that does not separate the volume.
    let e = ov.partition_volume(b.volume, VolumeCut { edges: vec![b.edges[0]], surface: sm_geom_api::SurfaceDef::Plane(sm_geom_api::Plane { frame: sm_geom_api::Frame::at(Point3::default()) }), seed: b.faces[0] }, [cell(1), cell(2)]);
    assert!(e.is_err());
    assert_eq!(ov.canonical_hash(), h);
    // A used child id is refused.
    cut_at(&mut ov, b.volume, 1.0, [cell(1), cell(2)]).unwrap();
    assert!(cut_at(&mut ov, cell(2), 2.0, [cell(1), cell(3)]).is_err());
}

#[test]
fn volume_split_is_deterministic() {
    let run = || {
        let (host, b) = brick();
        let mut ov = Overlay::new(host);
        let pc = cut_at(&mut ov, b.volume, 0.7, [cell(1), cell(2)]).unwrap();
        cut_at(&mut ov, cell(1), 0.3, [cell(3), cell(4)]).unwrap();
        (ov.canonical_hash(), ov.data(), pc)
    };
    let first = run();
    assert_eq!(first, run());
    let hs: Vec<_> = (0..4).map(|_| std::thread::spawn(run)).collect();
    for h in hs {
        assert_eq!(h.join().unwrap(), first);
    }
}
