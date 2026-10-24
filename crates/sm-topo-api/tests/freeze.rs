//! W0g freeze: CellFlags accessors, constructors of non-exhaustive records, golden serde shapes.

use sm_core::{EntityId, EntityKind};
use sm_topo_api::*;

#[test]
fn cell_flags_bits_roundtrip() {
    assert_eq!(CellFlags::NONE.bits(), 0);
    assert_eq!(CellFlags::DEGENERATE.bits(), 1);
    assert_eq!(CellFlags::from_bits(1), Some(CellFlags::DEGENERATE));
    assert_eq!(CellFlags::from_bits(0), Some(CellFlags::NONE));
    assert_eq!(CellFlags::from_bits(1 << 40), None);
    let f = CellFlags::from_bits(3).unwrap();
    assert!(f.contains(CellFlags::DEGENERATE));
    assert_eq!(CellFlags::NONE.union(CellFlags::DEGENERATE), CellFlags::DEGENERATE);
}

#[test]
fn constructors_match_fields() {
    let a = EntityId::new(EntityKind::TopoFace, 1).unwrap();
    let b = EntityId::new(EntityKind::TopoFace, 2).unwrap();
    let p = FacePair::new(b, a, Coincidence::Merged, OpId(3));
    assert_eq!((p.a, p.b, p.status, p.op), (b, a, Coincidence::Merged, OpId(3)));
    let s = OpSummary::new(OpId(1), vec![a], vec![b], 4, 2);
    assert_eq!((s.op, s.created.as_slice(), s.retired.as_slice(), s.lineage_start, s.lineage_len), (OpId(1), &[a][..], &[b][..], 4, 2));
    let c = CutOutcome::new(Vec::new(), vec![CapFace { face: a, from_surface: b }]);
    assert_eq!(c.caps.len(), 1);
    assert_eq!(CutOutcome::new(Vec::new(), Vec::new()), CutOutcome::default());
}

#[cfg(feature = "serde")]
#[test]
fn golden_serde_shapes() {
    use serde_json::json;
    // CellFlags is a bare integer on the wire, whatever its storage width.
    assert_eq!(serde_json::to_value(CellFlags::DEGENERATE).unwrap(), json!(1));
    assert_eq!(serde_json::from_value::<CellFlags>(json!(1)).unwrap(), CellFlags::DEGENERATE);
    // Unit variants stay plain strings; data variants stay externally tagged.
    assert_eq!(serde_json::to_value(EntityState::Live).unwrap(), json!("Live"));
    assert_eq!(serde_json::to_value(EntityState::Split(OpId(2))).unwrap(), json!({"Split": 2}));
    assert_eq!(serde_json::to_value(Coincidence::ImprintedNotMerged).unwrap(), json!("ImprintedNotMerged"));
}
