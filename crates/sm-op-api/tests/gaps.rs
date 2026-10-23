//! Contract tests for the W0f gap fixes: id adoption, succession hook, Delta wire form and
//! diagnostics, queries, per-kind state access, session branch management and seeding,
//! incremental timeline files, protocol DTO serde, per-layer check hashes, op-name length.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_op_api::*;

fn id(kind: EntityKind, s: u64) -> EntityId {
    EntityId::new(kind, s).unwrap()
}

// ------------------------------------------------------------------ layers

#[derive(Clone, Default)]
struct Bodies {
    ids: BTreeSet<EntityId>,
    /// Reaction log, to check ordering.
    log: Vec<String>,
}

impl LayerType for Bodies {
    const NAME: &'static str = "bodies";
    fn digest(&self, d: &mut Digest) {
        d.u64(self.ids.len() as u64);
        for i in &self.ids {
            d.id(*i);
        }
    }
    fn is_empty(&self) -> bool {
        self.ids.is_empty()
    }
    fn on_succession(&mut self, succ: &Successors) {
        self.log.push(format!("succ {}", succ.len()));
    }
    fn on_retired(&mut self, ids: &[EntityId]) {
        self.log.push(format!("ret {}", ids.len()));
        for i in ids {
            self.ids.remove(i);
        }
    }
}

/// Named groups that follow their members through splits and copies.
#[derive(Clone, Default)]
struct Groups {
    groups: BTreeMap<String, BTreeSet<EntityId>>,
    log: Vec<String>,
}

impl LayerType for Groups {
    const NAME: &'static str = "groups";
    const DEPENDS_ON: &'static [&'static str] = &["bodies"];
    fn digest(&self, d: &mut Digest) {
        for (n, m) in &self.groups {
            d.str(n).u64(m.len() as u64);
            for i in m {
                d.id(*i);
            }
        }
    }
    fn is_empty(&self) -> bool {
        self.groups.is_empty()
    }
    fn references_any(&self, ids: &[EntityId]) -> bool {
        self.groups.values().any(|m| ids.iter().any(|i| m.contains(i)))
    }
    fn on_succession(&mut self, succ: &Successors) {
        self.log.push("succ".into());
        for m in self.groups.values_mut() {
            for (old, new) in succ {
                if m.contains(old) {
                    m.remove(old);
                    m.extend(new.iter().copied());
                }
            }
        }
    }
    fn on_retired(&mut self, ids: &[EntityId]) {
        self.log.push("ret".into());
        for m in self.groups.values_mut() {
            for i in ids {
                m.remove(i);
            }
        }
    }
}

// ------------------------------------------------------------------ ops

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Make {}

impl TypedOp for Make {
    const NAME: &'static str = "t.make";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let b = ctx.alloc_as("body", EntityKind::TopoBody)?;
        ctx.layer_mut::<Bodies>()?.ids.insert(b);
        Ok(())
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Group {
    name: String,
    members: Vec<EntityId>,
}

impl TypedOp for Group {
    const NAME: &'static str = "t.group";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        for m in &self.members {
            ctx.require(*m)?;
        }
        ctx.layer_mut::<Groups>()?
            .groups
            .insert(self.name.clone(), self.members.iter().copied().collect());
        Ok(())
    }
}

/// Split a body into two: retire it, record the children as successors.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Split {
    body: EntityId,
}

impl TypedOp for Split {
    const NAME: &'static str = "t.split";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let b = ctx.resolve_kind(self.body, EntityKind::TopoBody)?;
        let l = ctx.alloc_as("child", EntityKind::TopoBody)?;
        let r = ctx.alloc_as("child", EntityKind::TopoBody)?;
        {
            let bodies = ctx.layer_mut::<Bodies>()?;
            bodies.ids.insert(l);
            bodies.ids.insert(r);
        }
        ctx.record_successors(b, &[l, r])?;
        ctx.retire(b)?;
        ctx.warn("t.split_lossy", "split drops nothing, really", &[b]);
        ctx.info("t.split_done", "split", &[l, r]);
        Ok(())
    }
}

/// Copy a body: `A -> [A, A']`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Copy {
    body: EntityId,
}

impl TypedOp for Copy {
    const NAME: &'static str = "t.copy";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let b = ctx.resolve(self.body)?;
        let c = ctx.alloc_as("copy", EntityKind::TopoBody)?;
        ctx.layer_mut::<Bodies>()?.ids.insert(c);
        ctx.record_successors(b, &[b, c])
    }
}

/// A downstream store minted `ids` itself; adopt them.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Adopt {
    ids: Vec<EntityId>,
    range: Option<(EntityId, u64)>,
}

impl TypedOp for Adopt {
    const NAME: &'static str = "t.adopt";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        ctx.adopt_as("adopted", &self.ids)?;
        if let Some((first, n)) = self.range {
            ctx.adopt_range(first, n)?;
        }
        Ok(())
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Retire {
    ids: Vec<EntityId>,
}

impl TypedOp for Retire {
    const NAME: &'static str = "t.retire";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        self.ids.iter().try_for_each(|i| ctx.retire(*i))
    }
}

// ------------------------------------------------------------------ queries

/// List live bodies.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct ListBodies {
    /// Only count.
    #[serde(default)]
    count_only: bool,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, PartialEq)]
struct BodyList {
    count: usize,
    ids: Vec<EntityId>,
}

impl TypedQuery for ListBodies {
    const NAME: &'static str = "t.list_bodies";
    type Output = BodyList;
    fn run(&self, state: &State, cx: &QueryCtx<'_>) -> Result<BodyList, OpError> {
        cx.step(0, 1, "list")?;
        let ids: Vec<EntityId> = state.entities_of(EntityKind::TopoBody).collect();
        Ok(BodyList {
            count: state.count_of(EntityKind::TopoBody),
            ids: if self.count_only { vec![] } else { ids },
        })
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct NeedsService {}

struct Kernel(u32);

impl TypedQuery for NeedsService {
    const NAME: &'static str = "t.kernel_version";
    type Output = u32;
    fn run(&self, _state: &State, cx: &QueryCtx<'_>) -> Result<u32, OpError> {
        Ok(cx.service::<Kernel>()?.0)
    }
}

static MAKES: AtomicUsize = AtomicUsize::new(0);

/// Counts applications, to check that seeded states skip replay.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct CountedMake {}

impl TypedOp for CountedMake {
    const NAME: &'static str = "t.counted_make";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        MAKES.fetch_add(1, Ordering::SeqCst);
        let b = ctx.alloc(EntityKind::TopoBody)?;
        ctx.layer_mut::<Bodies>()?.ids.insert(b);
        Ok(())
    }
}

fn registry() -> Registry {
    let mut r = Registry::new();
    r.register_layer::<Bodies>().unwrap();
    r.register_layer::<Groups>().unwrap();
    r.register::<Make>().unwrap();
    r.register::<Group>().unwrap();
    r.register::<Split>().unwrap();
    r.register::<Copy>().unwrap();
    r.register::<Adopt>().unwrap();
    r.register::<Retire>().unwrap();
    r.register::<CountedMake>().unwrap();
    r.register_query::<ListBodies>().unwrap();
    r.register_query::<NeedsService>().unwrap();
    r
}

fn engine() -> Engine {
    Engine::new(
        Arc::new(registry()),
        Services::new().with(Kernel(7)),
        Arc::new(DerivedCache::default()),
    )
    .unwrap()
}

fn session() -> Session {
    Session::new(engine())
}

fn q() -> QuietControl {
    QuietControl::new()
}

fn apply(s: &mut Session, op: &dyn Op) -> (Hash256, Delta) {
    let c = q();
    s.apply(op, Meta::default(), &c.control()).unwrap()
}

fn try_apply(s: &mut Session, op: &dyn Op) -> Result<(Hash256, Delta), TimelineError> {
    let c = q();
    s.apply(op, Meta::default(), &c.control())
}

// ------------------------------------------------------------------ tests

#[test]
fn ids_start_at_serial_one_and_adopt_checks_peek_next() {
    let mut s = session();
    let st = s.state().clone();
    assert_eq!(st.peek_next(EntityKind::TopoFace), Some(id(EntityKind::TopoFace, 1)));
    // In order, starting at 1.
    let ids = vec![id(EntityKind::TopoFace, 1), id(EntityKind::TopoFace, 2), id(EntityKind::TopoEdge, 1)];
    let (_, d) = apply(
        &mut s,
        &Adopt {
            ids: ids.clone(),
            range: Some((id(EntityKind::MeshCell, 1), 5)),
        },
    );
    assert_eq!(&d.created[..3], &ids[..]);
    assert_eq!(d.created.len(), 8);
    assert_eq!(d.created[7], id(EntityKind::MeshCell, 5));
    assert_eq!(d.outputs["adopted"], ids);
    assert!(s.state().contains(id(EntityKind::TopoFace, 2)));
    assert_eq!(s.state().count_of(EntityKind::MeshCell), 5);
    // Out of order, or a gap: IdMismatch, state untouched.
    let before = s.state().hash();
    let e = try_apply(&mut s, &Adopt { ids: vec![id(EntityKind::TopoFace, 4)], range: None }).unwrap_err();
    let TimelineError::Apply(ApplyError::Op { source, .. }) = &e else {
        panic!("{e:?}")
    };
    assert_eq!(
        *source,
        OpError::IdMismatch {
            expected: id(EntityKind::TopoFace, 3),
            got: id(EntityKind::TopoFace, 4)
        }
    );
    assert_eq!(e.code(), "id-mismatch");
    assert!(try_apply(&mut s, &Adopt { ids: vec![], range: Some((id(EntityKind::MeshCell, 1), 2)) }).is_err());
    assert_eq!(s.state().hash(), before);
}

#[test]
fn entities_of_and_count_of_are_per_kind() {
    let mut s = session();
    for _ in 0..4 {
        apply(&mut s, &Make {});
    }
    apply(&mut s, &Adopt { ids: vec![id(EntityKind::TopoFace, 1)], range: None });
    apply(&mut s, &Retire { ids: vec![id(EntityKind::TopoBody, 2)] });
    let st = s.state();
    assert_eq!(
        st.entities_of(EntityKind::TopoBody).collect::<Vec<_>>(),
        vec![id(EntityKind::TopoBody, 1), id(EntityKind::TopoBody, 3), id(EntityKind::TopoBody, 4)]
    );
    assert_eq!(st.count_of(EntityKind::TopoBody), 3);
    assert_eq!(st.count_of(EntityKind::TopoFace), 1);
    assert_eq!(st.count_of(EntityKind::MeshCell), 0);
    assert_eq!(st.entities_of(EntityKind::MeshCell).count(), 0);
    assert_eq!(st.entity_count(), 4);
}

#[test]
fn succession_runs_before_retire_and_carries_groups_to_children() {
    let mut s = session();
    let (_, d) = apply(&mut s, &Make {});
    let a = d.outputs["body"][0];
    apply(&mut s, &Make {});
    apply(&mut s, &Group { name: "g".into(), members: vec![a] });
    let (_, d) = apply(&mut s, &Split { body: a });
    let kids = d.outputs["child"].clone();
    assert_eq!(d.successors, BTreeMap::from([(a, kids.clone())]));
    assert!(d.touched_layers.iter().any(|l| l == "groups"));
    let g = s.state().layer::<Groups>().unwrap();
    assert_eq!(g.groups["g"], kids.iter().copied().collect());
    // After succession the group no longer references `a`, so `on_retired` is gated off.
    assert_eq!(g.log, vec!["succ"]);
    // Bodies does not override references_any (default true): both reactions ran, in order.
    assert_eq!(s.state().layer::<Bodies>().unwrap().log, vec!["succ 1", "ret 1"]);
    // Diagnostics come back in call order and do not enter any hash.
    assert_eq!(d.diagnostics.len(), 2);
    assert_eq!(d.diagnostics[0].severity, Severity::Warning);
    assert_eq!(d.diagnostics[0].code, "t.split_lossy");
    assert_eq!(d.diagnostics[1].severity, Severity::Info);
    assert_eq!(d.diagnostics[1].entities, kids);
}

#[test]
fn copy_succession_keeps_the_original_and_gating_skips_unrelated_layers() {
    let mut s = session();
    let (_, d) = apply(&mut s, &Make {});
    let a = d.outputs["body"][0];
    let (_, d) = apply(&mut s, &Make {});
    let b = d.outputs["body"][0];
    apply(&mut s, &Group { name: "g".into(), members: vec![a] });
    // Copy of b: the group layer does not reference b, so it is not touched.
    let (_, d) = apply(&mut s, &Copy { body: b });
    assert!(!d.touched_layers.iter().any(|l| l == "groups"));
    assert_eq!(d.successors[&b], vec![b, d.outputs["copy"][0]]);
    // Copy of a: the group gains the copy and keeps a.
    let (_, d) = apply(&mut s, &Copy { body: a });
    let c = d.outputs["copy"][0];
    assert_eq!(
        s.state().layer::<Groups>().unwrap().groups["g"],
        BTreeSet::from([a, c])
    );
}

#[test]
fn succession_is_dispatched_after_each_batch_sub_op() {
    let mut s = session();
    let (_, d) = apply(&mut s, &Make {});
    let a = d.outputs["body"][0];
    apply(&mut s, &Group { name: "g".into(), members: vec![a] });
    let rec = |name: &str, p: serde_json::Value| OpRecord::new(name, 1, p).unwrap();
    let batch = Batch {
        ops: vec![
            rec("t.split", serde_json::json!({"body": a.to_string()})),
            rec("t.split", serde_json::json!({"body": "$0.child[0]"})),
        ],
    };
    let (_, d) = apply(&mut s, &batch);
    let first = d.outputs["0.child"].clone();
    let second = d.outputs["1.child"].clone();
    assert_eq!(d.successors.len(), 2);
    assert_eq!(d.successors[&first[0]], second);
    let g = &s.state().layer::<Groups>().unwrap();
    assert_eq!(g.groups["g"], BTreeSet::from([first[1], second[0], second[1]]));
    // Two sub-ops, two rounds of succession, each delivering only its own entries.
    assert_eq!(g.log, vec!["succ", "succ"]);
    assert_eq!(s.state().layer::<Bodies>().unwrap().log, vec!["succ 1", "ret 1", "succ 1", "ret 1"]);
}

#[test]
fn record_successors_requires_live_successors() {
    #[derive(Debug, Serialize, Deserialize, JsonSchema)]
    struct Bad {}
    impl TypedOp for Bad {
        const NAME: &'static str = "t.bad_succ";
        fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
            let a = ctx.alloc(EntityKind::TopoBody)?;
            ctx.record_successors(a, &[EntityId::new(EntityKind::TopoBody, 99).unwrap()])
        }
    }
    let mut r = registry();
    r.register::<Bad>().unwrap();
    let e = Engine::new(Arc::new(r), Services::new(), Arc::new(DerivedCache::default())).unwrap();
    let mut st = e.registry().new_state();
    let c = q();
    assert!(e.apply(&mut st, &Bad {}, &c.control()).is_err());
}

#[test]
fn delta_round_trips_through_json_with_its_schema() {
    let mut s = session();
    let (_, d) = apply(&mut s, &Make {});
    let a = d.outputs["body"][0];
    let (_, d) = apply(&mut s, &Split { body: a });
    let v = serde_json::to_value(&d).unwrap();
    assert!(v["touched_layers"].is_array());
    assert_eq!(v["diagnostics"][0]["severity"], "warning");
    let back: Delta = serde_json::from_value(v).unwrap();
    assert_eq!(back, d);
    // Older replies without the new fields still decode.
    let old: Delta = serde_json::from_value(serde_json::json!({"created": [], "removed": []})).unwrap();
    assert_eq!(old, Delta::default());
    let schema = serde_json::to_value(schemars::schema_for!(Delta)).unwrap();
    for k in ["created", "removed", "outputs", "touched_layers", "successors", "diagnostics"] {
        assert!(schema["properties"].get(k).is_some(), "{k}");
    }
}

#[test]
fn queries_are_registered_run_and_catalogued() {
    let mut s = session();
    apply(&mut s, &Make {});
    apply(&mut s, &Make {});
    let e = s.engine().clone();
    let c = q();
    let out = e
        .run_query(s.state(), "t.list_bodies", &serde_json::json!({}), &c.control())
        .unwrap();
    let list: BodyList = serde_json::from_value(out).unwrap();
    assert_eq!(list.count, 2);
    assert_eq!(list.ids, vec![id(EntityKind::TopoBody, 1), id(EntityKind::TopoBody, 2)]);
    let out = e
        .run_query(s.state(), "t.kernel_version", &serde_json::json!({}), &c.control())
        .unwrap();
    assert_eq!(out, serde_json::json!(7));
    // Queries are not recorded and do not change the state.
    assert_eq!(s.timeline().len(), 2);
    // Errors.
    let err = e
        .run_query(s.state(), "t.nope", &serde_json::json!({}), &c.control())
        .unwrap_err();
    assert_eq!(err, RegistryError::UnknownQuery("t.nope".into()));
    assert_eq!(err.code(), "unknown-query");
    let err = e
        .run_query(s.state(), "t.list_bodies", &serde_json::json!({"count_only": 3}), &c.control())
        .unwrap_err();
    assert!(matches!(err, RegistryError::Query { .. }), "{err:?}");
    // A query name may equal an op name: separate namespace. Duplicates and bad names fail.
    let mut r = registry();
    assert_eq!(
        r.register_query::<ListBodies>().err(),
        Some(RegistryError::DuplicateQuery("t.list_bodies".into()))
    );
    #[derive(Debug, Serialize, Deserialize, JsonSchema)]
    struct Shadow {}
    impl TypedQuery for Shadow {
        const NAME: &'static str = "t.make";
        type Output = ();
        fn run(&self, _: &State, _: &QueryCtx<'_>) -> Result<(), OpError> {
            Ok(())
        }
    }
    assert!(r.register_query::<Shadow>().is_ok());
    assert_eq!(r.query_names(), vec!["t.kernel_version", "t.list_bodies", "t.make"]);
    // Catalog: sorted, input + output schemas.
    let cat: Vec<QueryInfo> = r.query_catalog().into_iter().map(Result::unwrap).collect();
    assert_eq!(cat[1].name, "t.list_bodies");
    assert_eq!(cat[1].schema.get("title").unwrap(), "t.list_bodies");
    assert!(cat[1].schema.get("properties").unwrap().get("count_only").is_some());
    let out = serde_json::to_value(&cat[1].output_schema).unwrap();
    assert!(out["properties"].get("ids").is_some());
    assert_eq!(cat[1].tool_name(), "t__list_bodies");
    serde_json::to_string(&cat[1]).unwrap();
}

#[test]
fn op_names_are_bounded_to_64_tool_name_bytes() {
    // 2 segments; tool name = a + "__" + b.
    let ok = format!("{}.{}", "a".repeat(31), "b".repeat(31)); // 31 + 2 + 31 = 64
    let long = format!("{}.{}", "a".repeat(31), "b".repeat(32)); // 65
    assert!(valid_op_name(&ok));
    assert!(!valid_op_name(&long));
    let three = format!("{}.{}.{}", "a".repeat(20), "b".repeat(20), "c".repeat(21)); // 61 + 4 = 65
    assert!(!valid_op_name(&three));
    assert_eq!(MAX_TOOL_NAME, 64);
}

#[test]
fn session_branch_management_is_logged() {
    let mut s = session();
    let (h1, _) = apply(&mut s, &Make {});
    let (h2, _) = apply(&mut s, &Make {});
    s.create_branch_at("old", h1).unwrap();
    assert_eq!(s.timeline().refs()["old"], h1);
    assert_eq!(s.head(), &Head::Ref("main".into()));
    assert!(matches!(s.create_branch_at("old", h2), Err(TimelineError::RefExists(_))));
    // Cannot delete the attached branch.
    assert_eq!(s.delete_branch("main"), Err(TimelineError::HeadOnBranch("main".into())));
    assert_eq!(s.delete_branch("main").unwrap_err().code(), "head-on-branch");
    // Rename the attached branch: HEAD follows.
    s.rename_branch("main", "trunk").unwrap();
    assert_eq!(s.head(), &Head::Ref("trunk".into()));
    assert_eq!(s.timeline().refs().get("trunk"), Some(&h2));
    assert!(!s.timeline().refs().contains_key("main"));
    // Rename into a directory of itself is fine (the old name goes away) ...
    s.rename_branch("old", "old/x").unwrap();
    // ... but colliding with another ref or using a bad name fails and changes nothing.
    assert!(s.rename_branch("trunk", "old").is_err());
    assert!(s.rename_branch("trunk", "old/x").is_err());
    assert!(s.rename_branch("trunk", "bad name").is_err());
    assert_eq!(s.timeline().refs().get("trunk"), Some(&h2));
    assert_eq!(s.head(), &Head::Ref("trunk".into()));
    s.delete_branch("old/x").unwrap();
    assert_eq!(s.timeline().refs().len(), 1);
    // The deleted branch's node stays checkout-able.
    assert!(s.timeline().get(h1).is_some());
    let actions: Vec<&str> = s.reflog().iter().map(|e| e.action.as_str()).collect();
    assert_eq!(
        actions,
        vec!["apply", "apply", "branch", "rename-branch", "rename-branch", "rename-branch", "rename-branch", "delete-branch"]
    );
}

#[test]
fn seed_state_accepts_only_matching_states_and_open_with_uses_seeds() {
    let mut s = session();
    let mut tips = Vec::new();
    for _ in 0..6 {
        tips.push(apply(&mut s, &CountedMake {}).0);
    }
    let tip = *tips.last().unwrap();
    let good = s.state().clone();
    let wrong = {
        let c = q();
        s.state_at(Some(tips[2]), &c.control()).unwrap()
    };
    let snap = good.snapshot();
    assert!(snap.is_err(), "Bodies has no snapshot hook; seeding uses live states here");
    // A state whose hash does not match the node's check is rejected.
    let e = s.seed_state(tip, wrong.clone()).unwrap_err();
    assert!(matches!(e, TimelineError::Diverged { ref layers, .. } if layers == &vec!["bodies".to_string()]), "{e:?}");
    assert!(matches!(s.seed_state(Hash256::of(b"x"), good.clone()), Err(TimelineError::UnknownNode(_))));
    // open_with: the seed at the tip means no replay at all.
    let timeline = s.timeline().clone();
    let reflog = s.reflog().to_vec();
    let before = MAKES.load(Ordering::SeqCst);
    let c = q();
    let s2 = Session::open_with(
        engine(),
        timeline.clone(),
        Head::Ref("main".into()),
        [(tip, good.clone())],
        reflog.clone(),
        &c.control(),
    )
    .unwrap();
    assert_eq!(s2.state().hash(), good.hash());
    assert_eq!(s2.reflog(), &reflog[..]);
    assert_eq!(MAKES.load(Ordering::SeqCst) - before, 0);
    // A bad seed fails the open.
    assert!(Session::open_with(engine(), timeline, Head::Ref("main".into()), [(tip, wrong)], vec![], &c.control()).is_err());
}

#[test]
fn node_and_ref_files_match_to_files() {
    let mut s = session();
    apply(&mut s, &Make {});
    let (h, _) = apply(&mut s, &Make {});
    s.branch("feature/x").unwrap();
    apply(&mut s, &Make {});
    let t = s.timeline();
    let mut union = BTreeMap::new();
    for (n, _) in t.nodes() {
        let (p, text) = t.node_file(n).unwrap();
        union.insert(p, text);
    }
    for name in t.refs().keys() {
        let (p, text) = t.ref_file(name).unwrap();
        union.insert(p, text);
    }
    assert_eq!(union, t.to_files().unwrap());
    assert!(t.ref_file("nope").is_none());
    assert!(matches!(t.node_file(Hash256::of(b"x")), Err(TimelineError::UnknownNode(_))));
    assert!(t.node_file(h).unwrap().0.starts_with("ops/"));
}

#[test]
fn check_carries_layer_hashes_outside_the_node_hash() {
    let mut s = session();
    let (h, _) = apply(&mut s, &Make {});
    let node = s.timeline().get(h).unwrap().clone();
    assert_eq!(node.check.state, s.state().hash());
    assert_eq!(node.check.layers.keys().collect::<Vec<_>>(), vec!["bodies"]);
    assert_eq!(node.check.layers["bodies"], s.state().layer_hash("bodies").unwrap());
    // Text form: a sorted [check.layers] table; round trips.
    let (_, text) = s.timeline().node_file(h).unwrap();
    assert!(text.contains("\n[check.layers]\nbodies = \""), "{text}");
    let back = Timeline::from_files(&s.timeline().to_files().unwrap()).unwrap();
    assert_eq!(back.get(h).unwrap().check, node.check);
    // Layer hashes are not part of the node identity, and a check without them still loads.
    let mut bare = node.clone();
    bare.check.layers.clear();
    assert_eq!(bare.hash().unwrap(), h);
    let stripped = text[..text.find("\n[check.layers]").unwrap()].to_owned() + "\n";
    let mut files = s.timeline().to_files().unwrap();
    files.insert(format!("ops/{h}.toml"), stripped);
    let t = Timeline::from_files(&files).unwrap();
    assert!(t.get(h).unwrap().check.layers.is_empty());
    // Re-inserting the same body with another layer map but the same state hash is not Diverged.
    let mut t2 = s.timeline().clone();
    assert_eq!(t2.insert(bare).unwrap(), h);
    // Different state: Diverged names the differing layers.
    let mut other = node.clone();
    other.check.state = Hash256::of(b"other");
    other.check.layers.insert("bodies".into(), Hash256::of(b"other"));
    other.check.layers.insert("groups".into(), Hash256::of(b"g"));
    match t2.insert(other) {
        Err(TimelineError::Diverged { layers, .. }) => assert_eq!(layers, vec!["bodies", "groups"]),
        r => panic!("{r:?}"),
    }
}

#[test]
fn protocol_dtos_serialize_like_the_node_file() {
    let mut s = session();
    apply(&mut s, &Make {});
    let (h, _) = apply(&mut s, &Make {});
    let node = s.timeline().get(h).unwrap().clone();
    let v = serde_json::to_value(&node).unwrap();
    assert!(v.get("parent").is_some() && v.get("op").is_some() && v.get("meta").is_some());
    assert!(v["check"].get("state").is_some());
    let back: Node = serde_json::from_value(v.clone()).unwrap();
    assert_eq!(back, node);
    // Same field set as the TOML node file.
    let (_, text) = s.timeline().node_file(h).unwrap();
    let file: serde_json::Value = toml::from_str::<toml::Value>(&text)
        .map(|t| serde_json::to_value(t).unwrap())
        .unwrap();
    assert_eq!(
        v.as_object().unwrap().keys().collect::<BTreeSet<_>>(),
        file.as_object().unwrap().keys().collect::<BTreeSet<_>>()
    );
    // A root omits `parent`.
    let root = s.timeline().get(s.timeline().roots()[0]).unwrap();
    assert!(serde_json::to_value(&root.body).unwrap().get("parent").is_none());
    let _ = schemars::schema_for!(Node);
    let _ = schemars::schema_for!(NodeBody);
    // Head and Rev: externally tagged.
    let hh = h.to_hex();
    for (head, rev, json) in [
        (Head::Ref("main".into()), Rev::Ref("main".into()), serde_json::json!({"ref": "main"})),
        (Head::Detached(Some(h)), Rev::Node(h), serde_json::json!({"node": hh})),
        (Head::Detached(None), Rev::Empty, serde_json::json!("empty")),
    ] {
        assert_eq!(serde_json::to_value(&head).unwrap(), json);
        assert_eq!(serde_json::to_value(&rev).unwrap(), json);
        assert_eq!(serde_json::from_value::<Head>(json.clone()).unwrap(), head);
        assert_eq!(serde_json::from_value::<Rev>(json).unwrap(), rev);
    }
    let hs = serde_json::to_value(schemars::schema_for!(Head)).unwrap();
    let rs = serde_json::to_value(schemars::schema_for!(Rev)).unwrap();
    assert_eq!(hs.get("oneOf"), rs.get("oneOf"));
    // OpInfo serializes.
    let cat = s.engine().registry().catalog();
    let info = cat.iter().find_map(|r| r.as_ref().ok()).unwrap();
    let v = serde_json::to_value(info).unwrap();
    assert!(v.get("schema").is_some() && v.get("output_schema").is_some());
}

#[test]
fn error_codes_are_stable_kebab_case() {
    let codes = [
        OpError::Invalid("x".into()).code(),
        OpError::Cancelled.code(),
        OpError::Cache(CacheError::TypeConflict { kind: "k".into() }).code(),
        RegistryError::BadOpName("x".into()).code(),
        ApplyError::Op { name: "x".into(), source: OpError::IdExhausted }.code(),
        TimelineError::NothingToUndo.code(),
        TimelineError::Apply(ApplyError::Registry(RegistryError::UnknownOp("x".into()))).code(),
    ];
    assert_eq!(
        codes,
        ["invalid-params", "cancelled", "cache-type-conflict", "bad-op-name", "id-exhausted", "nothing-to-undo", "unknown-op"]
    );
    for c in codes {
        assert!(c.bytes().all(|b| b.is_ascii_lowercase() || b == b'-'));
    }
}
