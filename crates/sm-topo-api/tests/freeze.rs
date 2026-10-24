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

// ---------------------------------------------------------------- W0g r2: pcurve hash words

fn pcurve_hash(pc: sm_geom_api::PCurve) -> u64 {
    use sm_topo_api::fixtures::{self, with_op};
    let mut t = CellComplex::new();
    let c = with_op(&mut t, OpId(1), |t| fixtures::cylinder(t, 1.0, 1.0)).unwrap();
    let key = CoedgeKey { face: c.faces[2], edge: c.edges[2], side: Side::Left };
    with_op(&mut t, OpId(2), |t| Ok(t.set_pcurve(key, Some(pc))?)).unwrap();
    // canonical_visit and canonical_hash cover the same words.
    let mut n = 0usize;
    t.canonical_visit(&mut |_| n += 1);
    assert!(n > 0);
    t.canonical_hash()
}

#[test]
fn pcurve_words_are_prefix_free_and_golden() {
    use sm_geom_api::{NurbsCurve2, PCurve, ShapeValue as W, Uv};
    let words = |pc: &PCurve| {
        let mut v = Vec::new();
        pc.for_each_word(|w| v.push(w));
        v
    };
    let poly = PCurve::Polyline2(vec![Uv::new(0.0, 0.5), Uv::new(1.0, 2.0)]);
    assert_eq!(words(&poly), vec![W::Int(2), W::Param(0.0), W::Param(0.5), W::Param(1.0), W::Param(2.0)]);
    let rat = NurbsCurve2::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![Uv::new(0.0, 0.0), Uv::new(1.0, 1.0)], Some(vec![1.0, 2.0])).unwrap();
    assert_eq!(
        words(&PCurve::Nurbs2(rat)),
        vec![
            W::Int(1),
            W::Int(4),
            W::Int(2),
            W::Int(1),
            W::Int(2),
            W::Param(0.0),
            W::Param(0.0),
            W::Param(1.0),
            W::Param(1.0),
            W::Param(0.0),
            W::Param(0.0),
            W::Param(1.0),
            W::Param(1.0),
            W::Unitless(1.0),
            W::Unitless(2.0),
        ]
    );
    let per = NurbsCurve2::new_periodic(1, vec![0.0, 1.0, 2.0, 3.0, 4.0], vec![Uv::new(0.0, 0.0), Uv::new(1.0, 0.0), Uv::new(0.0, 0.0)], None).unwrap();
    assert_eq!(words(&PCurve::Nurbs2(per))[..4], [W::Int(1 | 1 << 32), W::Int(5), W::Int(3), W::Int(0)]);
}

#[test]
fn pcurve_weights_hash_scale_free() {
    use sm_geom_api::{NurbsCurve2, PCurve, Uv};
    // Under the old absolute param quantum both weight sets saturated to the same code.
    let mk = |w: f64| PCurve::Nurbs2(NurbsCurve2::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![Uv::new(0.0, 0.0), Uv::new(1.0, 1.0)], Some(vec![w, 1.0])).unwrap());
    let tol = sm_geom_api::ModelTolerance::DEFAULT;
    let codes = |pc: &PCurve| {
        let mut v = Vec::new();
        pc.quantized_values(&tol, |c| v.push(c));
        v
    };
    assert_ne!(codes(&mk(1e12)), codes(&mk(2e12)));
    assert_ne!(pcurve_hash(mk(1e12)), pcurve_hash(mk(2e12)));
    // A relative change far below the 2^-40 step is invisible.
    assert_eq!(pcurve_hash(mk(3.0)), pcurve_hash(mk(3.0 * (1.0 + 1e-15))));
}

/// Golden canonical hashes of the cylinder fixture with a non-line pcurve (W0g r2 encoding).
/// A change here changes every stored topology hash with such pcurves.
#[test]
fn pcurve_canonical_hash_golden() {
    use sm_geom_api::{NurbsCurve2, PCurve, Uv};
    let poly = pcurve_hash(PCurve::Polyline2(vec![Uv::new(0.0, 0.0), Uv::new(1.0, 1.0)]));
    let nurbs = pcurve_hash(PCurve::Nurbs2(NurbsCurve2::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![Uv::new(0.0, 0.0), Uv::new(1.0, 1.0)], Some(vec![1.0, 2.0])).unwrap()));
    assert_eq!((poly, nurbs), (GOLDEN_POLY, GOLDEN_NURBS), "got ({poly:#018x}, {nurbs:#018x})");
}

const GOLDEN_POLY: u64 = 0x6dad260dd59272df;
const GOLDEN_NURBS: u64 = 0x06db58fff6673cd2;
