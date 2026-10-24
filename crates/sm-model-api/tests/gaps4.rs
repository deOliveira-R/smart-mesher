//! W0f gaps4: coloring iterators, overlay-aware face and volume colors.

use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_model_api::*;
use sm_topo_api::fixtures::{self, with_op};
use sm_topo_api::{CellComplex, OpId, Overlay, TopologyView};
use std::sync::Arc;

#[test]
fn coloring_iterators_list_entities_in_ascending_order() {
    let mut t = CellComplex::new();
    let (a, b) = with_op(&mut t, OpId(1), fixtures::two_bricks).unwrap();
    let mut m = Model::new();
    let steel = m.add_color(1, "steel", Domain::Volume).unwrap();
    let water = m.add_color(2, "water", Domain::Volume).unwrap();
    let wall = m.add_color(1, "wall", Domain::Face).unwrap();
    m.color_volumes(water, &[b.volume]).unwrap();
    m.color_volumes(steel, &[a.volume]).unwrap();
    m.color_faces(wall, &[b.faces[3], a.faces[0]]).unwrap();
    assert_eq!(m.volume_coloring().collect::<Vec<_>>(), vec![(a.volume, steel), (b.volume, water)]);
    assert_eq!(m.face_coloring().collect::<Vec<_>>(), vec![(a.faces[0], wall), (b.faces[3], wall)]);
}

#[test]
fn hidden_host_colors_reach_live_overlay_faces_and_pieces() {
    // Brick [0,2]x[0,1]x[0,1] split at x = 1 through the overlay.
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [2.0, 1.0, 1.0])).unwrap();
    let host: Arc<dyn TopologyView> = Arc::new(t);
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[b.volume]).unwrap();
    let wall = m.add_color(1, "wall", Domain::Face).unwrap();
    let cap = m.add_color(2, "cap", Domain::Face).unwrap();
    m.color_faces(wall, &b.faces).unwrap();
    let mut ov = Overlay::new(host.clone());
    let [n, p] = m.split_realized(PieceRef::Whole(b.volume), Plane::new(Vector3::new(1.0, 0.0, 0.0), 1.0).unwrap(), &mut ov).unwrap();
    let top = b.faces[5];
    assert!(!ov.is_alive(top));
    let halves = ov.live_cover(top);
    assert_eq!(halves.len(), 2);
    assert!(halves.iter().all(|f| f.kind() == EntityKind::PartitionFace));
    // Inherited through live_cover; the hidden host face is still enumerable.
    for f in &halves {
        assert_eq!(m.live_face_color(*f, &ov), Some(wall));
        assert_eq!(m.face_color(*f), None);
    }
    assert!(m.face_coloring().any(|(f, _)| f == top));
    assert_eq!(m.live_volume_color(n, &ov), Some(mat));
    assert_eq!(m.live_volume_color(p, &ov), Some(mat));
    // No false UnknownEntity / BoundaryFaceUncolored / ColoringNotPartition on the overlay.
    let r = validate(&m, &ov, Tolerance::default());
    for code in [IssueCode::BoundaryFaceUncolored, IssueCode::ColoringNotPartition] {
        assert!(!r.has(code), "{code:?}: {:#?}", r.issues);
    }
    let colored_unknown = |r: &Report| r.issues.iter().any(|i| i.code == IssueCode::UnknownEntity && i.detail.starts_with("colored"));
    assert!(!colored_unknown(&r), "{:#?}", r.issues);
    // An own color on a live overlay face overrides the inherited one.
    m.color_faces(cap, &[halves[0]]).unwrap();
    assert_eq!(m.live_face_color(halves[0], &ov), Some(cap));
    assert_eq!(m.live_face_color(halves[1], &ov), Some(wall));
    m.check_integrity().unwrap();
    m.uncolor_faces(&[halves[0]]);
    assert_eq!(m.live_face_color(halves[0], &ov), Some(wall));
    // Non-face kinds are still refused.
    assert!(matches!(m.color_faces(wall, &[b.volume]), Err(ModelError::WrongKind { .. })));
    // A colored face that is gone everywhere is still reported.
    let ghost = EntityId::new(EntityKind::TopoFace, 999).unwrap();
    m.color_faces(wall, &[ghost]).unwrap();
    assert!(colored_unknown(&validate(&m, &ov, Tolerance::default())));
}
