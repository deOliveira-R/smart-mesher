//! Third review round: primitive wedges, cut-id integrity, composite faces, deck checks on load.

use sm_core::{Point3, Vector3};
use sm_model_api::*;
use sm_topo_api::fixtures::{self, with_op};
use sm_topo_api::{CellComplex, CutOutcome, OpId};

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

fn colored_brick(size: [f64; 3]) -> (CellComplex, Model, fixtures::BrickIds) {
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, p3(0.0, 0.0, 0.0), size)).unwrap();
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[b.volume]).unwrap();
    (t, m, b)
}

fn plane_x(x: f64) -> CutGeometry {
    CutGeometry::Plane(Plane::new(Vector3::new(1.0, 0.0, 0.0), x).unwrap())
}

#[test]
fn a_wedge_must_be_a_primitive_one_over_n_turn() {
    let axis = Axis::new(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
    let rot = |n, d| CutSpec::new(Deck::rotation(axis, Turns::new(n, d).unwrap()), Keep::Positive, p3(1.0, 0.0, 0.0));
    assert!(matches!(plan_cut(&rot(2, 5)), Err(ModelError::Invalid(_))));
    assert!(plan_cut(&rot(1, 5)).is_ok());
    // A hand-built plan for the bad deck is refused by apply_cut.
    let mut forged = plan_cut(&rot(1, 5)).unwrap();
    forged.spec = rot(2, 5);
    let (t, mut m, _b) = colored_brick([1.0; 3]);
    let out = CutOutcome::default();
    assert!(matches!(m.apply_cut(&forged, &out, None, &t), Err(ModelError::Invalid(_))));
}

#[test]
fn internal_cut_parts_are_not_composite_faces() {
    let (t, mut m, b) = colored_brick([2.0, 1.0, 1.0]);
    let all = b.faces.to_vec();
    let [n1, p1] = m.split(PieceRef::Whole(b.volume), plane_x(1.0), &all, &all, &t).unwrap();
    let [n2, _] = m.split(PieceRef::Cell(n1), plane_x(0.5), &all, &all, &t).unwrap();
    let [n3, _] = m.split(PieceRef::Cell(p1), plane_x(1.5), &all, &all, &t).unwrap();
    let comp = m.composite(&[PieceRef::Cell(n2), PieceRef::Cell(n3)], &t).unwrap();
    let faces = m.decomposition().piece_faces(PieceRef::Cell(comp), &t);
    assert!(!faces.iter().any(|f| matches!(f, PieceFace::Cut { cut, .. } | PieceFace::CutPart { cut, .. } if *cut == CutId(0))), "{faces:?}");
}

#[cfg(feature = "serde")]
mod serde_tamper {
    use super::*;
    use serde_json::Value;

    fn split_model() -> (CellComplex, Model, fixtures::BrickIds) {
        let (t, mut m, b) = colored_brick([2.0, 1.0, 1.0]);
        let all = b.faces.to_vec();
        m.split(PieceRef::Whole(b.volume), plane_x(1.0), &all, &all, &t).unwrap();
        (t, m, b)
    }

    fn with_serial(m: &Model, serial: u64) -> std::result::Result<Model, serde_json::Error> {
        let mut v: Value = serde_json::to_value(m).unwrap();
        v["decomposition"]["cut_serial"] = Value::from(serial);
        serde_json::from_value(v)
    }

    #[test]
    fn a_cut_serial_that_would_reuse_an_id_is_refused() {
        let (_t, m, _b) = split_model();
        assert!(with_serial(&m, 1).is_ok());
        assert!(with_serial(&m, 0).is_err(), "serial 0 would reuse CutId(0)");
    }

    #[test]
    fn splitting_at_the_last_cut_id_is_an_error_not_a_panic() {
        let (t, m, b) = split_model();
        let mut m = with_serial(&m, u64::from(u32::MAX)).unwrap();
        let all = b.faces.to_vec();
        let piece = PieceRef::Cell(m.decomposition().active_cells()[0]);
        let before = m.ids().peek(sm_core::EntityKind::PartitionCell);
        let r = m.split(piece, plane_x(0.5), &all, &all, &t);
        assert!(r.is_err(), "{r:?}");
        assert_eq!(m.ids().peek(sm_core::EntityKind::PartitionCell), before);
    }

    #[test]
    fn a_stored_deck_that_fails_its_check_is_refused_on_load() {
        let (_t, mut m, _b) = split_model();
        let axis = Axis::new(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
        m.add_boundary("r", BoundaryKind::Deck(Deck::rotation(axis, Turns::new(1, 6).unwrap()))).unwrap();
        let s = serde_json::to_string(&m).unwrap();
        assert!(serde_json::from_str::<Model>(&s).is_ok());
        let bad = s.replace("\"den\":6", &format!("\"den\":{}", u32::MAX));
        assert_ne!(bad, s);
        assert!(serde_json::from_str::<Model>(&bad).is_err());
    }
}

#[test]
fn uncolor_volumes_hashes_like_a_state_that_never_had_the_color() {
    let (_t, mut m, b) = colored_brick([1.0; 3]);
    let mut never = Model::new();
    never.add_color(1, "mat", Domain::Volume).unwrap();
    let tol = sm_geom_api::ModelTolerance::default();
    let words = |m: &Model| {
        let mut v = Vec::new();
        m.canonical_visit(&tol, &mut |w| v.push(format!("{w:?}")));
        v
    };
    assert_ne!(words(&m), words(&never));
    let stray = sm_core::EntityId::new(sm_core::EntityKind::TopoFace, 99).unwrap();
    m.uncolor_volumes(&[b.volume, b.volume, stray]);
    assert_eq!(m.volume_color(b.volume), None);
    assert_eq!(words(&m), words(&never));
    m.uncolor_volumes(&[b.volume]); // already uncolored: ignored
    assert_eq!(words(&m), words(&never));
}
