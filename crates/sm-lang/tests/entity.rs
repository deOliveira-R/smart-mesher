//! The set algebra and range resolution are the only algorithmic code: test them against simple oracles.

mod common;

use std::collections::BTreeSet;

use proptest::prelude::*;
use sm_core::{EntityId, EntityKind};
use sm_lang::EntitySet;
use sm_lang::entity::{all_of, live_range};
use sm_op_api::{Engine, QuietControl};
use sm_testkit::strategies::config;

use common::*;

fn ids(v: &[u32]) -> EntitySet {
    EntitySet::from_vec(v.iter().map(|s| EntityId::new(EntityKind::TopoFace, u64::from(*s) + 1).unwrap()).collect())
}

fn set(s: &EntitySet) -> BTreeSet<EntityId> {
    s.iter().copied().collect()
}

fn vec_strategy() -> impl Strategy<Value = Vec<u32>> {
    // Dense small ranges give plenty of overlap; the occasional wide values exercise chunk boundaries.
    proptest::collection::vec(prop_oneof![9 => 0u32..200, 1 => 0u32..100_000], 0..400)
}

proptest! {
    #![proptest_config(config(600))]

    #[test]
    fn chunked_algebra_matches_btreeset(a in vec_strategy(), b in vec_strategy()) {
        let (sa, sb) = (ids(&a), ids(&b));
        let (oa, ob) = (set(&sa), set(&sb));
        for chunk in [1usize, 2, 3, 7, 64, usize::MAX] {
            let u = sa.union_chunked(&sb, chunk);
            let i = sa.intersection_chunked(&sb, chunk);
            let d = sa.difference_chunked(&sb, chunk);
            prop_assert_eq!(set(&u), oa.union(&ob).copied().collect::<BTreeSet<_>>(), "union chunk {}", chunk);
            prop_assert_eq!(set(&i), oa.intersection(&ob).copied().collect::<BTreeSet<_>>(), "intersection chunk {}", chunk);
            prop_assert_eq!(set(&d), oa.difference(&ob).copied().collect::<BTreeSet<_>>(), "difference chunk {}", chunk);
            // Sorted and duplicate-free.
            for s in [&u, &i, &d] {
                prop_assert!(s.as_slice().windows(2).all(|w| w[0] < w[1]));
            }
        }
        // Algebraic identities.
        prop_assert_eq!(sa.union(&sb), sb.union(&sa));
        prop_assert_eq!(sa.intersection(&sb), sb.intersection(&sa));
        prop_assert_eq!(sa.difference(&sb).union(&sa.intersection(&sb)), sa.clone());
    }

    #[test]
    fn ranges_match_a_filter_over_the_live_set(lo in 0u64..60, len in 0u64..80, step in 1u64..9) {
        let engine = engine();
        let mut state = engine.registry().new_state();
        let q = QuietControl::new();
        engine.apply_record(&mut state, &sm_op_api::OpRecord::new("test.alloc_many", 1, serde_json::json!({"kind": "TopoFace", "n": 50})).unwrap(), &q.control()).unwrap();
        let hi = lo + len;
        let got = live_range(&state, EntityKind::TopoFace, lo, hi, step);
        let want: Vec<EntityId> = state.entities_of(EntityKind::TopoFace).filter(|id| id.serial() >= lo && id.serial() <= hi && (id.serial() - lo) % step == 0).collect();
        prop_assert_eq!(got.as_slice(), want.as_slice());
        prop_assert_eq!(all_of(&state, EntityKind::TopoFace).len(), 50);
    }
}

#[test]
fn ranges_skip_retired_entities_and_clip_to_allocated_serials() {
    let engine: Engine = engine();
    let mut state = engine.registry().new_state();
    let q = QuietControl::new();
    let apply = |state: &mut sm_op_api::State, name: &str, p: serde_json::Value| engine.apply_record(state, &sm_op_api::OpRecord::new(name, 1, p).unwrap(), &q.control()).unwrap();
    apply(&mut state, "model.create_brick", serde_json::json!({"x": 1, "y": 1, "z": 1}));
    apply(&mut state, "model.create_brick", serde_json::json!({"x": 1, "y": 1, "z": 1}));
    apply(&mut state, "model.delete_body", serde_json::json!({"id": "TopoBody#1"}));
    // Faces 1..=6 are retired, 7..=12 live. A huge range costs O(allocated), not O(range).
    let r = live_range(&state, EntityKind::TopoFace, 1, u64::MAX >> 9, 1);
    assert_eq!(r.len(), 6);
    assert_eq!(r.as_slice()[0].serial(), 7);
    assert_eq!(live_range(&state, EntityKind::TopoFace, 1, 6, 1).len(), 0);
    assert_eq!(live_range(&state, EntityKind::TopoFace, 8, 12, 2).iter().map(|i| i.serial()).collect::<Vec<_>>(), [8, 10, 12]);
    assert_eq!(all_of(&state, EntityKind::TopoVolume).len(), 1);
    assert!(all_of(&state, EntityKind::Color).is_empty());
}

#[test]
fn membership_and_single() {
    let s = ids(&[3, 1, 2, 3]);
    assert_eq!(s.len(), 3);
    assert!(s.contains(EntityId::new(EntityKind::TopoFace, 2).unwrap()));
    assert!(!s.contains(EntityId::new(EntityKind::TopoFace, 9).unwrap()));
    assert!(EntitySet::new().is_empty());
}
