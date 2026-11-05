//! W3b-0: plugin-supplied canonical records and bulk `retire_range`.

use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_op_api::*;

fn vol(n: u64) -> EntityId {
    EntityId::new(EntityKind::TopoVolume, n).unwrap()
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Make {
    n: u32,
}
impl TypedOp for Make {
    const NAME: &'static str = "t.make";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        for _ in 0..self.n {
            ctx.alloc(EntityKind::TopoVolume)?;
        }
        Ok(())
    }
}

/// Stores a rewritten record: allocates three volumes and stores `t.make {n: 3}`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Auto {}
impl TypedOp for Auto {
    const NAME: &'static str = "t.auto";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        for _ in 0..3 {
            ctx.alloc(EntityKind::TopoVolume)?;
        }
        ctx.set_canonical_record(OpRecord::of(&Make { n: 3 })?);
        Ok(())
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Kill {
    first: EntityId,
    n: u64,
    bulk: bool,
}
impl TypedOp for Kill {
    const NAME: &'static str = "t.kill";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if self.bulk {
            return ctx.retire_range(self.first, self.n);
        }
        for k in 0..self.n {
            ctx.retire(EntityId::new(self.first.kind(), self.first.serial() + k).unwrap())?;
        }
        Ok(())
    }
}

fn engine() -> Engine {
    let mut r = Registry::new();
    r.register::<Make>().unwrap();
    r.register::<Auto>().unwrap();
    r.register::<Kill>().unwrap();
    Engine::new(Arc::new(r), Services::new(), Arc::new(DerivedCache::default())).unwrap()
}

#[test]
fn canonical_record_replays_with_equal_hash() {
    let e = engine();
    let q = QuietControl::new();
    let mut st = e.registry().new_state();
    let (_, canon) = e
        .apply_record_canonical(&mut st, &OpRecord::of(&Auto {}).unwrap(), &q.control())
        .unwrap();
    let canon = canon.expect("op supplied a record");
    assert_eq!(canon.name, "t.make");
    let mut replay = e.registry().new_state();
    e.apply_record(&mut replay, &canon, &q.control()).unwrap();
    assert_eq!(replay.hash(), st.hash());
}

#[test]
fn retire_range_equals_per_id_retire() {
    let e = engine();
    let q = QuietControl::new();
    let run = |bulk: bool| {
        let mut st = e.registry().new_state();
        e.apply(&mut st, &Make { n: 40 }, &q.control()).unwrap();
        // A single id first (non-empty retired set), then runs.
        e.apply(&mut st, &Kill { first: vol(3), n: 1, bulk }, &q.control()).unwrap();
        let d1 = e.apply(&mut st, &Kill { first: vol(10), n: 20, bulk }, &q.control()).unwrap();
        let d2 = e.apply(&mut st, &Kill { first: vol(40), n: 1, bulk }, &q.control()).unwrap();
        (st.hash(), d1.removed, d2.removed, st.count_of(EntityKind::TopoVolume))
    };
    let (a, b) = (run(false), run(true));
    assert_eq!(a, b);
    assert_eq!(a.3, 40 - 22);
    // One big run from an empty retired set.
    let big = |bulk: bool| {
        let mut st = e.registry().new_state();
        e.apply(&mut st, &Make { n: 500 }, &q.control()).unwrap();
        e.apply(&mut st, &Kill { first: vol(1), n: 500, bulk }, &q.control()).unwrap();
        st.hash()
    };
    assert_eq!(big(false), big(true));
}

#[test]
fn retire_range_failures_change_nothing() {
    let e = engine();
    let q = QuietControl::new();
    let mut st = e.registry().new_state();
    e.apply(&mut st, &Make { n: 5 }, &q.control()).unwrap();
    e.apply(&mut st, &Kill { first: vol(3), n: 1, bulk: true }, &q.control()).unwrap();
    let h = st.hash();
    // Crossing a retired id, past `next`, starting past `next`.
    for (f, n) in [(2, 3), (4, 3), (6, 1)] {
        let r = e.apply(&mut st, &Kill { first: vol(f), n, bulk: true }, &q.control());
        assert!(r.is_err(), "{f}+{n}");
        assert_eq!(st.hash(), h);
    }
    // Empty range is a no-op.
    e.apply(&mut st, &Kill { first: vol(1), n: 0, bulk: true }, &q.control()).unwrap();
    assert_eq!(st.hash(), h);
    // Unknown kind.
    let other = EntityId::new(EntityKind::TopoFace, 1).unwrap();
    assert!(e.apply(&mut st, &Kill { first: other, n: 1, bulk: true }, &q.control()).is_err());
}
