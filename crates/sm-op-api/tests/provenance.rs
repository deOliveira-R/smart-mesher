//! Implementation provenance in node checks: `DIGEST_VERSION` / `IMPL_VERSION`,
//! `Check::versions`, `TimelineError::Upgraded`, `Timeline::replace_check` and
//! `ReplayPolicy::Rebaseline`.

use std::collections::BTreeMap;
use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_op_api::*;

#[derive(Clone, Default)]
struct Vals(BTreeMap<EntityId, u64>);

impl LayerType for Vals {
    const NAME: &'static str = "vals";
    const DIGEST_VERSION: u32 = 3;
    fn digest(&self, d: &mut Digest) {
        for (k, v) in &self.0 {
            d.id(*k).u64(*v);
        }
    }
    fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

/// Version 1 of the implementation: stores `x`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct ValV1 {
    x: u64,
}

impl TypedOp for ValV1 {
    const NAME: &'static str = "t.val";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let b = ctx.alloc(EntityKind::TopoBody)?;
        ctx.layer_mut::<Vals>()?.0.insert(b, self.x);
        Ok(())
    }
}

/// Version 2 (a "bug fix"): same payload, different result.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct ValV2 {
    x: u64,
}

impl TypedOp for ValV2 {
    const NAME: &'static str = "t.val";
    const IMPL_VERSION: u32 = 2;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let b = ctx.alloc(EntityKind::TopoBody)?;
        ctx.layer_mut::<Vals>()?.0.insert(b, self.x + 1);
        Ok(())
    }
}

fn engine(v2: bool) -> Engine {
    let mut r = Registry::new();
    r.register_layer::<Vals>().unwrap();
    if v2 {
        r.register::<ValV2>().unwrap();
    } else {
        r.register::<ValV1>().unwrap();
    }
    Engine::new(Arc::new(r), Services::new(), Arc::new(DerivedCache::default())).unwrap()
}

fn record(v2: bool) -> (Timeline, Hash256, Hash256) {
    let mut s = Session::new(engine(v2));
    let c = QuietControl::new();
    let (a, _) = s.apply(&ValV1 { x: 1 }, Meta::default(), &c.control()).unwrap();
    let (b, _) = s.apply(&ValV1 { x: 2 }, Meta::default(), &c.control()).unwrap();
    (s.timeline().clone(), a, b)
}

#[test]
fn versions_default_to_one_and_are_recorded_in_the_check() {
    let (t, a, _) = record(false);
    let n = t.get(a).unwrap();
    let want: BTreeMap<String, u32> =
        [("layer:vals".to_owned(), 3), ("op:t.val".to_owned(), 1)].into();
    assert_eq!(n.check.versions, want);
    let r = engine(true);
    assert_eq!(r.registry().op_impl_version("t.val"), Some(2));
    assert_eq!(r.registry().op_impl_version("core.batch"), Some(1));
    assert_eq!(r.registry().layer_digest_version("vals"), Some(3));
    assert_eq!(r.registry().layer_digest_version("nope"), None);
    // Text form: a sorted [check.versions] table with quoted keys; round trips.
    let (_, text) = t.node_file(a).unwrap();
    assert!(
        text.contains("\n[check.versions]\n\"layer:vals\" = 3\n\"op:t.val\" = 1\n"),
        "{text}"
    );
    let back = Timeline::from_files(&t.to_files().unwrap()).unwrap();
    assert_eq!(back.get(a).unwrap().check, n.check);
    // Not part of the node hash, and a file without the table still loads.
    let mut bare = n.clone();
    bare.check.versions.clear();
    assert_eq!(bare.hash().unwrap(), a);
    let stripped = text.replace("\n[check.versions]\n\"layer:vals\" = 3\n\"op:t.val\" = 1\n", "");
    let mut files = t.to_files().unwrap();
    files.insert(format!("ops/{a}.toml"), stripped);
    assert!(Timeline::from_files(&files).unwrap().get(a).unwrap().check.versions.is_empty());
}

#[test]
fn batch_checks_record_nested_op_versions() {
    let mut s = Session::new(engine(true));
    let c = QuietControl::new();
    let b = Batch {
        ops: vec![OpRecord::of(&ValV2 { x: 4 }).unwrap()],
    };
    let (h, _) = s.apply(&b, Meta::default(), &c.control()).unwrap();
    let v = &s.timeline().get(h).unwrap().check.versions;
    assert_eq!(v.get("op:core.batch"), Some(&1));
    assert_eq!(v.get("op:t.val"), Some(&2));
}

#[test]
fn replay_after_an_impl_upgrade_is_upgraded_not_diverged() {
    let (t, a, b) = record(false);
    let c = QuietControl::new();
    let e = Session::open(engine(true), t.clone(), Head::Ref("main".into()), &c.control())
        .err()
        .unwrap();
    match &e {
        TimelineError::Upgraded { node, changed } => {
            assert_eq!(*node, a);
            assert_eq!(changed, &vec!["op:t.val".to_owned()]);
        }
        e => panic!("{e:?}"),
    }
    assert_eq!(e.code(), "upgraded");
    // Without recorded versions the same mismatch is a plain Diverged.
    let mut old = t.clone();
    for h in [a, b] {
        let mut ch = old.get(h).unwrap().check.clone();
        ch.versions.clear();
        old.replace_check(h, ch).unwrap();
    }
    let e = Session::open(engine(true), old, Head::Ref("main".into()), &c.control())
        .err()
        .unwrap();
    assert!(matches!(e, TimelineError::Diverged { node, .. } if node == a), "{e:?}");
}

#[test]
fn rebaseline_rerecords_checks_in_place() {
    let (t, a, b) = record(false);
    let before = t.to_files().unwrap();
    let c = QuietControl::new();
    // Open without replaying (HEAD detached at the empty state), then re-baseline on checkout.
    let mut opened =
        Session::open(engine(true), t.clone(), Head::Detached(None), &c.control()).unwrap();
    opened.set_replay_policy(ReplayPolicy::Rebaseline);
    opened.checkout(Rev::Node(b), &c.control()).unwrap();
    assert_eq!(opened.take_rebaselined(), vec![a, b]);
    let w = opened.take_warnings();
    assert_eq!(w.len(), 2);
    assert!(w.iter().all(|e| matches!(e, TimelineError::Upgraded { .. })));
    // Hashes and file names unchanged; only check text differs.
    let after = opened.timeline().to_files().unwrap();
    assert_eq!(before.keys().collect::<Vec<_>>(), after.keys().collect::<Vec<_>>());
    assert_ne!(before[&format!("ops/{a}.toml")], after[&format!("ops/{a}.toml")]);
    let na = opened.timeline().get(a).unwrap();
    assert_eq!(na.hash().unwrap(), a);
    assert_eq!(na.check.versions["op:t.val"], 2);
    // The re-baselined timeline replays strictly with the new implementation.
    let back = Timeline::from_files(&after).unwrap();
    let mut strict = Session::open(engine(true), back, Head::Detached(Some(b)), &c.control()).unwrap();
    assert_eq!(strict.take_warnings().len(), 0);
    assert!(strict.state_at_from_root(b, &c.control()).is_ok());
}

#[test]
fn replace_check_keeps_the_body_and_hash() {
    let (mut t, a, b) = record(false);
    let body = t.get(b).unwrap().body.clone();
    let new = Check {
        state: Hash256::of(b"rebased"),
        ..Check::default()
    };
    t.replace_check(b, new.clone()).unwrap();
    let n = t.get(b).unwrap();
    assert_eq!(n.check, new);
    assert_eq!(n.body, body);
    assert_eq!(n.hash().unwrap(), b);
    assert_eq!(t.children(a), &[b]);
    assert!(matches!(
        t.replace_check(Hash256::of(b"x"), new),
        Err(TimelineError::UnknownNode(_))
    ));
}
