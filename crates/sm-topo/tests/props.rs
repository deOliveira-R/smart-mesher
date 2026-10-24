//! Property tests (sm-testkit strategies, fixed seed): random bricks and refinements agree with
//! the closed forms and with the reference implementation.

mod common;
#[path = "common/scenario.rs"]
mod scenario;

use common::same_state;
use proptest::prelude::*;
use sm_testkit::{check, strategies};
use sm_topo::TopoStore;
use sm_topo_api::fixtures::{self, with_op};
use sm_topo_api::{CellComplex, Dim, OpId, TopologyView, euler_characteristic, validate};

proptest! {
    #![proptest_config(strategies::config(24))]

    /// A random brick: counts, Euler, volume and area follow from the construction, and the store
    /// equals the testkit's reference complex exactly.
    #[test]
    fn random_bricks_match_their_closed_forms_and_the_reference(spec in strategies::brick_spec()) {
        let fixture = spec.build().unwrap();
        let mut t = TopoStore::new();
        t.set_paranoid(true);
        scenario::setup_at(&mut t, spec.min, spec.size);
        same_state(&fixture.complex, &t, "brick");
        prop_assert_eq!(Dim::ALL.map(|d| t.cells(d).len()), fixture.counts);
        prop_assert_eq!(euler_characteristic(&t), 2);
        prop_assert!(check::topo_valid(&t).is_ok());
        let v = t.volume_moments(t.cells(Dim::Volume)[0]).unwrap().volume;
        prop_assert!((v - spec.volume()).abs() <= 1e-9 * spec.volume());
        let area: f64 = t.cells(Dim::Face).iter().map(|f| t.face_moments(*f).unwrap().area).sum();
        prop_assert!((area - fixture.area).abs() <= 1e-9 * fixture.area);
    }

    /// Refine a random brick's top face and webcut another: ids, lineage and hashes equal the
    /// reference's; the volume is preserved (refinement only splits faces).
    #[test]
    fn refinement_and_webcut_agree_with_the_reference(spec in strategies::brick_spec(), n in 1usize..80, frac in 0.1f64..0.9, per_op in any::<bool>()) {
        let (mut a, mut b) = (CellComplex::new(), TopoStore::new());
        b.set_paranoid(true);
        let (ia, ib) = (scenario::setup_at(&mut a, spec.min, spec.size), scenario::setup_at(&mut b, spec.min, spec.size));
        prop_assert_eq!(&ia, &ib);
        scenario::refine(&mut a, &ia, n, per_op);
        scenario::refine(&mut b, &ib, n, per_op);
        prop_assert_eq!(a.canonical_hash(), b.canonical_hash());
        prop_assert!(validate(&b).is_empty());
        let vol = b.volume_moments(ib.volume).unwrap().volume;
        prop_assert!((vol - spec.volume()).abs() <= 1e-8 * spec.volume());
        // Another brick, webcut at `frac`, built after the refinement (ids interleave with it).
        let far = [spec.min[0] + 20.0, spec.min[1], spec.min[2]];
        let wa = with_op(&mut a, OpId(1_000_000), |t| fixtures::brick(t, sm_core::Point3::new(far[0], far[1], far[2]), spec.size)).unwrap();
        let wb = with_op(&mut b, OpId(1_000_000), |t| fixtures::brick(t, sm_core::Point3::new(far[0], far[1], far[2]), spec.size)).unwrap();
        prop_assert_eq!(&wa, &wb);
        let ca = with_op(&mut a, OpId(1_000_001), |t| fixtures::webcut_brick(t, &wa, frac)).unwrap();
        let cb = with_op(&mut b, OpId(1_000_001), |t| fixtures::webcut_brick(t, &wb, frac)).unwrap();
        prop_assert_eq!(format!("{ca:?}"), format!("{cb:?}"));
        prop_assert_eq!(a.canonical_hash(), b.canonical_hash());
        prop_assert_eq!(a.lineage().records(), b.lineage().records());
    }
}
