//! Contract tests. Everything here uses only the public API, the way a concept node would:
//! the api crate knows none of these layers or ops.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_op_api::*;

// ----------------------------------------------------------- sample "model" node

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Brick {
    x: f64,
    y: f64,
    z: f64,
    name: String,
}

#[derive(Clone, Default)]
struct ModelLayer {
    bricks: MapLayer<Brick>,
}

impl LayerType for ModelLayer {
    const NAME: &'static str = names::MODEL;
    fn digest(&self, d: &mut Digest) {
        self.bricks.digest_with(d, |d, b| {
            d.f64_quantized(b.x, 1e-9)
                .f64_quantized(b.y, 1e-9)
                .f64_quantized(b.z, 1e-9)
                .str(&b.name);
        });
    }
    fn is_empty(&self) -> bool {
        self.bricks.is_empty()
    }
    fn on_retired(&mut self, ids: &[EntityId]) {
        self.bricks.on_retired(ids);
    }
}

/// Mesh-size intent that refers to model entities.
#[derive(Clone, Default)]
struct IntentLayer {
    sizes: BTreeMap<EntityId, f64>,
}

impl LayerType for IntentLayer {
    const NAME: &'static str = names::INTENT;
    const DEPENDS_ON: &'static [&'static str] = &[names::MODEL];
    fn digest(&self, d: &mut Digest) {
        for (id, s) in &self.sizes {
            d.id(*id).f64_quantized(*s, 1e-9);
        }
    }
    fn is_empty(&self) -> bool {
        self.sizes.is_empty()
    }
    fn on_retired(&mut self, ids: &[EntityId]) {
        for id in ids {
            self.sizes.remove(id);
        }
    }
    fn references_any(&self, ids: &[EntityId]) -> bool {
        ids.iter().any(|id| self.sizes.contains_key(id))
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct CreateBrick {
    /// Extent along x.
    x: f64,
    y: f64,
    z: f64,
    /// Optional display name.
    name: Option<String>,
}

impl TypedOp for CreateBrick {
    const NAME: &'static str = "model.create_brick";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if self.x <= 0.0 || self.y <= 0.0 || self.z <= 0.0 {
            return Err(OpError::Invalid("extents must be positive".into()));
        }
        let id = ctx.alloc_as("body", EntityKind::TopoBody)?;
        let b = Brick {
            x: self.x,
            y: self.y,
            z: self.z,
            name: self.name.clone().unwrap_or_default(),
        };
        ctx.layer_mut::<ModelLayer>()?.bricks.insert(id, b);
        Ok(())
    }
}

fn brick(x: f64) -> CreateBrick {
    CreateBrick {
        x,
        y: 1.0,
        z: 1.0,
        name: None,
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct ScaleBrick {
    id: EntityId,
    factor: f64,
}

impl TypedOp for ScaleBrick {
    const NAME: &'static str = "model.scale_brick";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let id = ctx.resolve_kind(self.id, EntityKind::TopoBody)?;
        let b = ctx
            .layer_mut::<ModelLayer>()?
            .bricks
            .get_mut(id)
            .ok_or(OpError::UnknownEntity(id))?;
        b.x *= self.factor;
        Ok(())
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct DeleteBody {
    id: EntityId,
}

impl TypedOp for DeleteBody {
    const NAME: &'static str = "model.delete_body";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        ctx.retire(self.id)
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct SetSize {
    id: EntityId,
    size: f64,
}

impl TypedOp for SetSize {
    const NAME: &'static str = "intent.set_size";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let id = ctx.resolve(self.id)?;
        ctx.layer_mut::<IntentLayer>()?.sizes.insert(id, self.size);
        Ok(())
    }
}

/// Mutates, allocates, retires, then fails.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct BadOp {}

impl TypedOp for BadOp {
    const NAME: &'static str = "test.bad";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let id = ctx.alloc(EntityKind::TopoBody)?;
        ctx.layer_mut::<ModelLayer>()?.bricks.insert(
            id,
            Brick {
                x: 9.0,
                y: 9.0,
                z: 9.0,
                name: "ghost".into(),
            },
        );
        ctx.layer_mut::<IntentLayer>()?.sizes.insert(id, 1.0);
        Err(OpError::Failed("boom".into()))
    }
}

fn registry() -> Registry {
    let mut r = Registry::new();
    r.register_layer::<ModelLayer>().unwrap();
    r.register_layer::<IntentLayer>().unwrap();
    r.register::<CreateBrick>().unwrap();
    r.register::<ScaleBrick>().unwrap();
    r.register::<DeleteBody>().unwrap();
    r.register::<SetSize>().unwrap();
    r.register::<BadOp>().unwrap();
    r
}

fn engine_with(r: Registry, services: Services) -> Engine {
    Engine::new(Arc::new(r), services, Arc::new(DerivedCache::default())).unwrap()
}

fn session() -> Session {
    Session::new(engine_with(registry(), Services::new()))
}

fn meta(msg: &str) -> Meta {
    Meta {
        author: "test".into(),
        time: "2026-01-01T00:00:00Z".into(),
        message: msg.into(),
    }
}

fn apply(s: &mut Session, op: &dyn Op) -> (Hash256, Delta) {
    let q = QuietControl::new();
    s.apply(op, meta("m"), &q.control()).unwrap()
}

fn q() -> QuietControl {
    QuietControl::new()
}

fn bricks(s: &State) -> usize {
    s.layer::<ModelLayer>().unwrap().bricks.len()
}

// ------------------------------------------------------------------- acceptance tests

#[test]
fn create_then_reference_and_replay_twice_gives_equal_state_hashes() {
    let mut s = session();
    let (_, d) = apply(
        &mut s,
        &CreateBrick {
            x: 2.0,
            y: 3.0,
            z: 4.0,
            name: Some("a".into()),
        },
    );
    let id = d.outputs["body"][0];
    assert_eq!(id.kind(), EntityKind::TopoBody);
    assert_eq!(d.created, vec![id]);
    let (tip, _) = apply(&mut s, &ScaleBrick { id, factor: 0.5 });
    assert_eq!(
        s.state()
            .layer::<ModelLayer>()
            .unwrap()
            .bricks
            .get(id)
            .unwrap()
            .x,
        1.0
    );

    let q = q();
    let a = s.state_at_from_root(tip, &q.control()).unwrap();
    let b = s.state_at_from_root(tip, &q.control()).unwrap();
    assert_eq!(a.hash(), b.hash());
    assert_eq!(a.hash(), s.state().hash());

    // A second, independent session over the same files reproduces the state.
    let files = s.timeline().to_files().unwrap();
    let t2 = Timeline::from_files(&files).unwrap();
    let s2 = Session::open(
        engine_with(registry(), Services::new()),
        t2,
        s.head().clone(),
        &q.control(),
    )
    .unwrap();
    assert_eq!(s2.state().hash(), s.state().hash());
    assert_eq!(s2.position(), Some(tip));
}

#[test]
fn unknown_and_wrong_kind_references_are_rejected_without_effect() {
    let mut s = session();
    let (_, d) = apply(&mut s, &brick(1.0));
    let id = d.outputs["body"][0];
    let before = s.state().hash();
    let ghost = EntityId::new(EntityKind::TopoBody, 99).unwrap();
    let c = q();
    let e = s
        .apply(
            &ScaleBrick {
                id: ghost,
                factor: 2.0,
            },
            meta(""),
            &c.control(),
        )
        .unwrap_err();
    assert!(
        matches!(
            e,
            TimelineError::Apply(ApplyError::Op {
                source: OpError::UnknownEntity(_),
                ..
            })
        ),
        "{e:?}"
    );
    assert_eq!(id.kind(), EntityKind::TopoBody);
    assert_eq!(s.state().hash(), before);
}

#[test]
fn fork_from_node_3_of_10_and_switch_back() {
    let mut s = session();
    let mut nodes = Vec::new();
    for i in 1..=10 {
        nodes.push(apply(&mut s, &brick(i as f64)).0);
    }
    assert_eq!(bricks(s.state()), 10);
    let c = q();
    s.checkout(Rev::Node(nodes[2]), &c.control()).unwrap();
    assert_eq!(bricks(s.state()), 3);
    assert!(matches!(s.head(), Head::Detached(Some(_))));
    s.branch("exp").unwrap();
    // Fork allocates ids from the prefix state: serial 4 again, on a different branch.
    let (_, d) = apply(&mut s, &brick(100.0));
    assert_eq!(d.created[0].serial(), 4);
    apply(&mut s, &brick(101.0));
    assert_eq!(bricks(s.state()), 5);
    assert_eq!(s.timeline().refs()["main"], nodes[9]);
    assert_ne!(s.timeline().refs()["exp"], nodes[9]);
    let exp_hash = s.state().hash();

    s.checkout(Rev::Ref("main".into()), &c.control()).unwrap();
    assert_eq!(bricks(s.state()), 10);
    assert_eq!(s.position(), Some(nodes[9]));
    s.checkout(Rev::Ref("exp".into()), &c.control()).unwrap();
    assert_eq!(s.state().hash(), exp_hash);
    // Empty root.
    s.checkout(Rev::Empty, &c.control()).unwrap();
    assert_eq!(bricks(s.state()), 0);
    assert!(s.checkout(Rev::Ref("nope".into()), &c.control()).is_err());
    assert!(
        s.checkout(Rev::Node(Hash256::of(b"x")), &c.control())
            .is_err()
    );
}

// ------------------------------------------------------------------- plugins

#[derive(Clone, Default)]
struct TetLayer {
    tets: BTreeMap<u64, u64>,
}

impl LayerType for TetLayer {
    const NAME: &'static str = "scheme_tet";
    fn digest(&self, d: &mut Digest) {
        d.value(&self.tets);
    }
    fn is_empty(&self) -> bool {
        self.tets.is_empty()
    }
    fn snapshot(&self) -> Result<Option<Vec<u8>>, CodecError> {
        Ok(Some(canonical_bytes(&to_value(&self.tets)?)))
    }
    fn restore(bytes: &[u8]) -> Result<Self, CodecError> {
        // Test snapshot format: JSON text of the map with string keys.
        let m: BTreeMap<String, u64> =
            serde_json::from_slice(bytes).map_err(|e| CodecError::new(e.to_string()))?;
        Ok(Self {
            tets: m
                .into_iter()
                .map(|(k, v)| (k.parse().unwrap_or(0), v))
                .collect(),
        })
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct TetMesh {
    count: u64,
}

impl TypedOp for TetMesh {
    const NAME: &'static str = "scheme_tet.mesh";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let id = ctx.alloc(EntityKind::Mesh)?;
        ctx.layer_mut::<TetLayer>()?
            .tets
            .insert(id.serial(), self.count);
        Ok(())
    }
}

struct TetPlugin;

impl Plugin for TetPlugin {
    fn id(&self) -> &str {
        "scheme-tet"
    }
    fn register(&self, r: &mut Registry) -> Result<(), RegistryError> {
        r.register_layer::<TetLayer>()?;
        r.register::<TetMesh>()?;
        Ok(())
    }
}

#[test]
fn plugin_registers_its_own_op_and_layer_without_editing_the_api_crate() {
    let mut r = registry();
    r.install(&TetPlugin).unwrap();
    let mut s = Session::new(engine_with(r, Services::new()));
    apply(&mut s, &TetMesh { count: 12 });
    assert_eq!(s.state().layer::<TetLayer>().unwrap().tets.len(), 1);
    assert!(s.state().layer_names().any(|n| n == "scheme_tet"));
}

struct HalfBroken;

impl Plugin for HalfBroken {
    fn id(&self) -> &str {
        "half-broken"
    }
    fn register(&self, r: &mut Registry) -> Result<(), RegistryError> {
        r.register_layer::<TetLayer>()?;
        r.register::<TetMesh>()?;
        r.register::<CreateBrick>()?; // duplicate: fails after two successful registrations
        Ok(())
    }
}

#[test]
fn install_is_atomic() {
    let mut r = registry();
    let before = r.op_names().len();
    let layers_before = r.layer_roles().len();
    assert!(matches!(
        r.install(&HalfBroken),
        Err(RegistryError::DuplicateOp(_))
    ));
    assert_eq!(r.op_names().len(), before);
    assert_eq!(r.layer_roles().len(), layers_before);
    // The plugin's pieces are not half-installed: a clean install still works.
    r.install(&TetPlugin).unwrap();
}

#[test]
fn installing_a_new_concept_does_not_change_existing_hashes() {
    let mut s = session();
    apply(&mut s, &brick(1.0));
    apply(&mut s, &brick(2.0));
    let old_hash = s.state().hash();
    let tip = s.position().unwrap();

    let mut r = registry();
    r.install(&TetPlugin).unwrap();
    let s2 = Session::open(
        engine_with(r, Services::new()),
        Timeline::from_files(&s.timeline().to_files().unwrap()).unwrap(),
        s.head().clone(),
        &q().control(),
    )
    .unwrap();
    assert_eq!(s2.position(), Some(tip));
    assert_eq!(s2.state().hash(), old_hash);
}

#[test]
fn registration_order_does_not_matter() {
    let mut a = Registry::new();
    a.register_layer::<ModelLayer>().unwrap();
    a.register_layer::<IntentLayer>().unwrap();
    a.register::<CreateBrick>().unwrap();
    a.register::<ScaleBrick>().unwrap();
    a.install(&TetPlugin).unwrap();
    let mut b = Registry::new();
    b.install(&TetPlugin).unwrap();
    b.register::<ScaleBrick>().unwrap();
    b.register::<CreateBrick>().unwrap();
    b.register_layer::<IntentLayer>().unwrap();
    b.register_layer::<ModelLayer>().unwrap();
    let cat = |r: &Registry| -> Vec<String> {
        r.catalog()
            .into_iter()
            .map(|i| i.unwrap())
            .map(|i| format!("{} {} {}", i.name, i.version, i.schema.as_value()))
            .collect()
    };
    assert_eq!(cat(&a), cat(&b));
    assert_eq!(a.layer_order().unwrap(), b.layer_order().unwrap());
    assert_eq!(a.new_state().hash(), b.new_state().hash());
}

// ------------------------------------------------------------------- schemas / catalog

struct BrokenSchema;

impl OpType for BrokenSchema {
    fn name(&self) -> &'static str {
        "test.broken_schema"
    }
    fn version(&self) -> u32 {
        1
    }
    fn schema(&self) -> Result<schemars::Schema, String> {
        Err("cannot describe".into())
    }
    fn decode(&self, _: &Value) -> Result<Box<dyn Op>, CodecError> {
        Err(CodecError::new("no"))
    }
}

#[test]
fn json_schema_of_all_registered_ops() {
    let mut r = registry();
    r.install(&TetPlugin).unwrap();
    let cat: Vec<OpInfo> = r.catalog().into_iter().map(|i| i.unwrap()).collect();
    let names: Vec<&str> = cat.iter().map(|i| i.name.as_str()).collect();
    let mut sorted = names.clone();
    sorted.sort_unstable();
    assert_eq!(names, sorted, "catalog is sorted");
    for n in [
        "core.batch",
        "intent.set_size",
        "model.create_brick",
        "model.delete_body",
        "model.scale_brick",
        "scheme_tet.mesh",
        "test.bad",
    ] {
        assert!(names.contains(&n), "{n} missing from {names:?}");
    }
    for i in &cat {
        let v = i.schema.as_value();
        assert_eq!(v["title"], i.name.as_str(), "schema title is the op name");
        assert_eq!(v["type"], "object");
        assert!(!i.tool_name().contains('.'));
        assert!(
            i.tool_name()
                .bytes()
                .all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-')
        );
        assert!(i.output_schema.as_value().to_string().contains("created"));
    }
    let scale = cat.iter().find(|i| i.name == "model.scale_brick").unwrap();
    assert_eq!(scale.tool_name(), "model__scale_brick");
    // Entity ids are readable strings with a pattern, not uint64.
    let text = scale.schema.as_value().to_string();
    assert!(
        text.contains("Kind#serial") || text.contains("[A-Za-z]+#[0-9]+"),
        "{text}"
    );
    assert!(!text.contains("uint64"));
    // A broken schema does not hide the rest.
    r.register_dyn(Arc::new(BrokenSchema)).unwrap();
    let cat = r.catalog();
    assert!(cat.iter().any(
        |i| matches!(i, Err(RegistryError::Schema { name, .. }) if name == "test.broken_schema")
    ));
    assert!(cat.iter().filter(|i| i.is_ok()).count() >= 7);
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct MoveV2 {
    dx: f64,
    dy: f64,
}

impl TypedOp for MoveV2 {
    const NAME: &'static str = "test.move";
    const VERSION: u32 = 2;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let id = ctx.alloc(EntityKind::TopoBody)?;
        ctx.layer_mut::<ModelLayer>()?.bricks.insert(
            id,
            Brick {
                x: self.dx,
                y: self.dy,
                z: 0.0,
                name: String::new(),
            },
        );
        Ok(())
    }
}

#[test]
fn op_versions_migrate_step_by_step() {
    let mut r = registry();
    r.register::<MoveV2>().unwrap();
    // v1 had a single `d`; v2 splits it.
    r.register_migration("test.move", 1, |mut v| {
        let d = v.get("d").cloned().ok_or_else(|| CodecError::new("no d"))?;
        if let Some(m) = v.as_object_mut() {
            m.remove("d");
            m.insert("dx".into(), d.clone());
            m.insert("dy".into(), d);
        }
        Ok(v)
    })
    .unwrap();
    assert!(matches!(
        r.register_migration("test.move", 1, Ok),
        Err(RegistryError::DuplicateMigration { .. })
    ));
    assert!(matches!(
        r.register_migration("test.move", 2, Ok),
        Err(RegistryError::BadMigration { .. })
    ));
    assert!(matches!(
        r.register_migration("nope", 1, Ok),
        Err(RegistryError::UnknownOp(_))
    ));
    let old = OpRecord::new("test.move", 1, serde_json::json!({"d": 2.5})).unwrap();
    let op = r.decode(&old).unwrap();
    assert_eq!(op.version(), 2);
    assert_eq!(
        op.encode().unwrap(),
        serde_json::json!({"dx": 2.5, "dy": 2.5})
    );
    let future = OpRecord::new("test.move", 3, serde_json::json!({})).unwrap();
    assert!(matches!(
        r.decode(&future),
        Err(RegistryError::UnsupportedVersion { .. })
    ));
    let unknown = OpRecord::new("test.nope", 1, serde_json::json!({})).unwrap();
    assert!(matches!(
        r.decode(&unknown),
        Err(RegistryError::UnknownOp(_))
    ));
}

// ------------------------------------------------------------------- transactions

/// A long op: allocates per step, reports progress, honours cancellation.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct LongOp {
    steps: u64,
}

impl TypedOp for LongOp {
    const NAME: &'static str = "test.long";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let key = ctx
            .state()
            .cache_key("long.setup", &[], &serde_json::json!({}))?;
        ctx.derived()
            .get_or_compute::<u64, OpError>("long.setup", key, || Ok(7))?;
        for i in 0..self.steps {
            ctx.step(i, self.steps, "working")?;
            let id = ctx.alloc(EntityKind::TopoBody)?;
            ctx.layer_mut::<ModelLayer>()?.bricks.insert(
                id,
                Brick {
                    x: 1.0,
                    y: 1.0,
                    z: 1.0,
                    name: format!("s{i}"),
                },
            );
        }
        ctx.step(self.steps, self.steps, "done")
    }
}

struct CancelAfter {
    token: CancelToken,
    n: usize,
    seen: AtomicUsize,
}

impl Progress for CancelAfter {
    fn report(&self, _: f32, _: &str) {
        if self.seen.fetch_add(1, Ordering::SeqCst) + 1 >= self.n {
            self.token.cancel();
        }
    }
}

fn long_session() -> Session {
    let mut r = registry();
    r.register::<LongOp>().unwrap();
    Session::new(engine_with(r, Services::new()))
}

#[test]
fn long_op_reports_progress_and_completes() {
    let mut s = long_session();
    let rec = RecordingProgress::new();
    let tok = CancelToken::new();
    s.apply(&LongOp { steps: 10 }, meta(""), &Control::new(&rec, &tok))
        .unwrap();
    let ev = rec.events();
    assert_eq!(ev.len(), 11);
    assert!(ev.windows(2).all(|w| w[0].0 <= w[1].0));
    assert_eq!(ev.last().unwrap().0, 1.0);
    assert_eq!(bricks(s.state()), 10);
}

#[test]
fn cancelled_long_op_leaves_state_untouched_but_keeps_the_cache() {
    let mut s = long_session();
    apply(&mut s, &brick(1.0));
    let (hash, pos, nodes, next) = (
        s.state().hash(),
        s.position(),
        s.timeline().len(),
        s.state().peek_next(EntityKind::TopoBody),
    );
    let tok = CancelToken::new();
    let sink = CancelAfter {
        token: tok.clone(),
        n: 4,
        seen: AtomicUsize::new(0),
    };
    let e = s
        .apply(&LongOp { steps: 100 }, meta(""), &Control::new(&sink, &tok))
        .unwrap_err();
    assert!(
        matches!(e, TimelineError::Apply(ref a) if a.is_cancelled()),
        "{e:?}"
    );
    assert_eq!(s.state().hash(), hash);
    assert_eq!(s.position(), pos);
    assert_eq!(s.timeline().len(), nodes);
    assert_eq!(s.state().peek_next(EntityKind::TopoBody), next);
    assert_eq!(
        s.derived().stats().entries,
        1,
        "derived cache survives the cancel"
    );
    // Retrying without cancellation works and reuses the cache entry.
    let c = q();
    s.apply(&LongOp { steps: 3 }, meta(""), &c.control())
        .unwrap();
    assert!(s.derived().stats().hits >= 1);
}

#[test]
fn failing_op_leaves_state_untouched() {
    let mut s = session();
    apply(&mut s, &brick(1.0));
    let (hash, pos, nodes, next) = (
        s.state().hash(),
        s.position(),
        s.timeline().len(),
        s.state().peek_next(EntityKind::TopoBody),
    );
    let c = q();
    let e = s.apply(&BadOp {}, meta(""), &c.control()).unwrap_err();
    assert!(matches!(
        e,
        TimelineError::Apply(ApplyError::Op {
            source: OpError::Failed(_),
            ..
        })
    ));
    assert_eq!(s.state().hash(), hash);
    assert_eq!(s.position(), pos);
    assert_eq!(s.timeline().len(), nodes);
    assert_eq!(
        s.state().peek_next(EntityKind::TopoBody),
        next,
        "serials are not consumed by a failed op"
    );
    assert!(s.state().layer::<IntentLayer>().unwrap().sizes.is_empty());
    assert_eq!(s.reflog().len(), 1);
    // Engine-level: the caller's state is untouched too.
    let mut st = s.state().clone();
    let before = st.hash();
    let ctl = q();
    assert!(
        s.engine()
            .apply(&mut st, &BadOp {}, &ctl.control())
            .is_err()
    );
    assert_eq!(st.hash(), before);
}

#[test]
fn prepare_off_thread_and_commit_with_stale_base_detection() {
    let mut s = session();
    apply(&mut s, &brick(1.0));
    let base = s.prepare_base();
    let handle = std::thread::spawn(move || {
        let c = QuietControl::new();
        base.prepare(&brick(2.0), &c.control())
    });
    let prepared = handle.join().unwrap().unwrap();
    assert_eq!(prepared.delta().created.len(), 1);
    // The session moves on while the op was being prepared.
    apply(&mut s, &brick(3.0));
    let e = s.commit(prepared, meta("late")).unwrap_err();
    assert!(matches!(e, TimelineError::StaleBase { .. }), "{e:?}");
    assert_eq!(bricks(s.state()), 2);
    // A fresh prepare/commit succeeds.
    let c = q();
    let p = s.prepare(&brick(4.0), &c.control()).unwrap();
    s.commit(p, meta("ok")).unwrap();
    assert_eq!(bricks(s.state()), 3);
}

// ------------------------------------------------------------------- batch

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct MakeVolume {}

impl TypedOp for MakeVolume {
    const NAME: &'static str = "test.make_volume";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let id = ctx.alloc_as("volume", EntityKind::TopoVolume)?;
        ctx.layer_mut::<IntentLayer>()?.sizes.insert(id, 0.5);
        Ok(())
    }
}

#[test]
fn batch_is_one_atomic_node_with_intra_batch_references() {
    let mut r = registry();
    r.register::<MakeVolume>().unwrap();
    let mut s = Session::new(engine_with(r, Services::new()));
    let batch = Batch {
        ops: vec![
            OpRecord::new("test.make_volume", 1, serde_json::json!({})).unwrap(),
            OpRecord::new(
                "intent.set_size",
                1,
                serde_json::json!({"id": "$0.volume", "size": 2.0}),
            )
            .unwrap(),
        ],
    };
    let (_, d) = apply(&mut s, &batch);
    assert_eq!(s.timeline().len(), 1, "one node for the whole batch");
    let vol = d.outputs["0.volume"][0];
    assert_eq!(s.state().layer::<IntentLayer>().unwrap().sizes[&vol], 2.0);
    assert_eq!(d.created, vec![vol]);

    // A failing second op rolls back the first.
    let before = s.state().hash();
    let bad = Batch {
        ops: vec![
            OpRecord::new("test.make_volume", 1, serde_json::json!({})).unwrap(),
            OpRecord::new(
                "intent.set_size",
                1,
                serde_json::json!({"id": "$5.volume", "size": 2.0}),
            )
            .unwrap(),
        ],
    };
    let c = q();
    assert!(s.apply(&bad, meta(""), &c.control()).is_err());
    assert_eq!(s.state().hash(), before);

    // Replay of a batch node reproduces the state.
    let tip = s.position().unwrap();
    assert_eq!(
        s.state_at_from_root(tip, &c.control()).unwrap().hash(),
        before
    );
}

// ------------------------------------------------------------------- ids

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct AllocSorted {
    keys: Vec<u64>,
}

impl TypedOp for AllocSorted {
    const NAME: &'static str = "test.alloc_sorted";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let ids = ctx.alloc_sorted(EntityKind::MeshCell, &self.keys)?;
        for (id, k) in ids.iter().zip(&self.keys) {
            ctx.layer_mut::<TetLayer>()?.tets.insert(id.serial(), *k);
        }
        Ok(())
    }
}

#[test]
fn alloc_sorted_ids_do_not_depend_on_input_order() {
    let mut r = registry();
    r.install(&TetPlugin).unwrap();
    r.register::<AllocSorted>().unwrap();
    let run = |keys: Vec<u64>| {
        let mut s = Session::new(engine_with(r.clone(), Services::new()));
        let (_, d) = apply(&mut s, &AllocSorted { keys: keys.clone() });
        let tets = s.state().layer::<TetLayer>().unwrap().tets.clone();
        (tets, d.created.len())
    };
    let (a, n) = run(vec![30, 10, 20]);
    assert_eq!(n, 3);
    assert_eq!(
        a,
        BTreeMap::from([(1, 10), (2, 20), (3, 30)]),
        "serial order follows key order"
    );
    let (b, _) = run(vec![20, 30, 10]);
    assert_eq!(a, b);
}

#[test]
fn serials_are_never_reused_and_retired_ids_cannot_be_resolved() {
    let mut s = session();
    let (_, d) = apply(&mut s, &brick(1.0));
    let id = d.outputs["body"][0];
    apply(&mut s, &DeleteBody { id });
    assert!(!s.state().contains(id));
    let (_, d2) = apply(&mut s, &brick(1.0));
    assert_eq!(d2.created[0].serial(), id.serial() + 1);
    let c = q();
    assert!(
        s.apply(&ScaleBrick { id, factor: 2.0 }, meta(""), &c.control())
            .is_err()
    );
    assert_eq!(s.state().entity_count(), 1);
    assert_eq!(s.state().entities().collect::<Vec<_>>(), d2.created);
    assert!(s.state().kind_of(id).is_none());
}

#[test]
fn a_wrong_kind_reference_is_a_typed_error() {
    let mut r = registry();
    r.register::<MakeVolume>().unwrap();
    let mut s = Session::new(engine_with(r, Services::new()));
    let (_, d) = apply(&mut s, &MakeVolume {});
    let vol = d.outputs["volume"][0];
    assert_eq!(s.state().kind_of(vol), Some(EntityKind::TopoVolume));
    let c = q();
    let e = s
        .apply(
            &ScaleBrick {
                id: vol,
                factor: 1.0,
            },
            meta(""),
            &c.control(),
        )
        .unwrap_err();
    assert!(
        matches!(
            e,
            TimelineError::Apply(ApplyError::Op {
                source: OpError::WrongKind { .. },
                ..
            })
        ),
        "{e:?}"
    );
}

// ------------------------------------------------------------------- files

#[test]
fn to_files_from_files_round_trip_with_identical_hashes() {
    let mut s = session();
    for i in 1..=5 {
        apply(
            &mut s,
            &CreateBrick {
                x: i as f64,
                y: 0.1 * i as f64,
                z: 1e21 * i as f64,
                name: Some(format!("b{i}\nline two \"q\" \\ \u{1}")),
            },
        );
    }
    s.branch("side").unwrap();
    apply(&mut s, &brick(9.0));
    let files = s.timeline().to_files().unwrap();
    assert!(
        files
            .keys()
            .all(|k| k.starts_with("ops/") || k.starts_with("refs/heads/"))
    );
    assert!(!files.keys().any(|k| k.contains("HEAD")));
    assert_eq!(files.keys().filter(|k| k.starts_with("ops/")).count(), 6);
    assert_eq!(
        files["refs/heads/side"].trim(),
        s.position().unwrap().to_hex()
    );

    let t2 = Timeline::from_files(&files).unwrap();
    assert_eq!(t2.len(), s.timeline().len());
    assert_eq!(t2.refs(), s.timeline().refs());
    for (h, n) in s.timeline().nodes() {
        assert_eq!(t2.get(h), Some(n));
        assert_eq!(n.body.hash().unwrap(), h);
    }
    assert_eq!(t2.to_files().unwrap(), files, "serialization is canonical");

    let c = q();
    let s2 = Session::open(
        engine_with(registry(), Services::new()),
        t2,
        Head::Ref("side".into()),
        &c.control(),
    )
    .unwrap();
    assert_eq!(s2.state().hash(), s.state().hash());
}

#[test]
fn hashes_do_not_depend_on_text_format() {
    let mut s = session();
    apply(
        &mut s,
        &CreateBrick {
            x: 1.5,
            y: 2.0,
            z: 3.0,
            name: Some("a\r\nb".into()),
        },
    );
    let files = s.timeline().to_files().unwrap();
    // CRLF checkout (autocrlf) of every file.
    let crlf: BTreeMap<String, String> = files
        .iter()
        .map(|(k, v)| (k.clone(), v.replace('\n', "\r\n")))
        .collect();
    let t = Timeline::from_files(&crlf).unwrap();
    assert_eq!(t.len(), 1);
    assert_eq!(t.refs(), s.timeline().refs());
    // Reordered / reformatted keys also load.
    let (path, text) = files.iter().find(|(k, _)| k.starts_with("ops/")).unwrap();
    let reformatted = format!("# comment\n\n{}\n", text.replace(" = ", "   =   "));
    let mut f2 = files.clone();
    f2.insert(path.clone(), reformatted);
    assert!(Timeline::from_files(&f2).is_ok());
}

#[test]
fn golden_node_hash_is_stable() {
    let body = NodeBody {
        parent: None,
        op: OpRecord::new(
            "model.create_brick",
            1,
            serde_json::json!({"x": 1.0, "y": 2, "z": 3.5, "name": "g"}),
        )
        .unwrap(),
        meta: Meta {
            author: "golden".into(),
            time: "2026-01-01T00:00:00Z".into(),
            message: "m".into(),
        },
    };
    let h = body.hash().unwrap().to_hex();
    assert_eq!(
        h, GOLDEN_NODE,
        "node hash format changed; this is a breaking change to every stored project"
    );
    // The recorded result is not part of the identity.
    let child = NodeBody {
        parent: Some(body.hash().unwrap()),
        ..body
    };
    assert_ne!(child.hash().unwrap().to_hex(), GOLDEN_NODE);
}

const GOLDEN_NODE: &str = "abce6fb048507bb15d9816fdc65e5f467a46c23abcb7e7e501ce9b0c7ac58641";

#[test]
fn tampered_or_broken_files_are_rejected() {
    let mut s = session();
    apply(&mut s, &brick(1.0));
    apply(&mut s, &brick(2.0));
    let files = s.timeline().to_files().unwrap();
    // Edit a param in place: hash no longer matches the file name.
    let (path, text) = files
        .iter()
        .find(|(k, v)| k.starts_with("ops/") && v.contains("x = 2"))
        .unwrap();
    let mut bad = files.clone();
    bad.insert(path.clone(), text.replace("x = 2", "x = 3"));
    assert!(matches!(
        Timeline::from_files(&bad),
        Err(TimelineError::HashMismatch { .. })
    ));
    // Missing parent.
    let mut orphan = files.clone();
    let root = s.timeline().roots()[0];
    orphan.remove(&format!("ops/{root}.toml"));
    assert!(matches!(
        Timeline::from_files(&orphan),
        Err(TimelineError::UnknownParent(_))
    ));
    // Dangling ref.
    let mut dangling = files.clone();
    dangling.insert(
        "refs/heads/ghost".into(),
        format!("{}\n", Hash256::of(b"nothing")),
    );
    assert!(Timeline::from_files(&dangling).is_err());
    // Unknown field.
    let mut extra = files.clone();
    extra.insert(path.clone(), format!("{text}\n[bonus]\nx = 1\n"));
    assert!(matches!(
        Timeline::from_files(&extra),
        Err(TimelineError::Format(_))
    ));
    // Unrelated files (project.toml etc.) are ignored.
    let mut other = files.clone();
    other.insert("project.toml".into(), "name = 'x'".into());
    assert!(Timeline::from_files(&other).is_ok());
}

#[test]
fn two_branches_created_independently_produce_disjoint_file_sets() {
    let mut s = session();
    for i in 1..=3 {
        apply(&mut s, &brick(i as f64));
    }
    let base_files = s.timeline().to_files().unwrap();
    let base_tip = s.position().unwrap();
    let t = Timeline::from_files(&base_files).unwrap();

    let make = |name: &str, author: &str, x0: f64| -> BTreeMap<String, String> {
        let c = q();
        let mut w = Session::open(
            engine_with(registry(), Services::new()),
            t.clone(),
            Head::Ref("main".into()),
            &c.control(),
        )
        .unwrap();
        w.checkout(Rev::Node(base_tip), &c.control()).unwrap();
        w.branch(name).unwrap();
        for k in 0..3 {
            let m = Meta {
                author: author.into(),
                ..meta("work")
            };
            w.apply(&brick(x0 + k as f64), m, &c.control()).unwrap();
        }
        w.timeline().to_files().unwrap()
    };
    let a = make("alice", "alice", 10.0);
    let b = make("bob", "bob", 20.0);

    let changed = |f: &BTreeMap<String, String>| -> BTreeSet<String> {
        f.iter()
            .filter(|(k, v)| base_files.get(*k) != Some(*v))
            .map(|(k, _)| k.clone())
            .collect()
    };
    let (ca, cb) = (changed(&a), changed(&b));
    assert_eq!(ca.len(), 4, "3 new nodes + 1 ref: {ca:?}");
    assert!(
        ca.is_disjoint(&cb),
        "branches touch disjoint files: {:?}",
        ca.intersection(&cb).collect::<Vec<_>>()
    );
    assert!(
        !ca.iter().chain(&cb).any(|k| k.contains("HEAD")),
        "HEAD is never a committed file"
    );
    // Nothing pre-existing was modified or deleted.
    assert!(
        base_files
            .iter()
            .all(|(k, v)| a.get(k) == Some(v) && b.get(k) == Some(v))
    );
    // A git-style merge is the union, and it loads with both branches.
    let mut merged = a.clone();
    merged.extend(b.clone());
    let m = Timeline::from_files(&merged).unwrap();
    assert!(
        m.refs().contains_key("alice")
            && m.refs().contains_key("bob")
            && m.refs().contains_key("main")
    );
    assert_eq!(m.len(), 9);
}

#[test]
fn ref_names_are_validated_and_cannot_collide_as_file_and_directory() {
    let mut s = session();
    apply(&mut s, &brick(1.0));
    s.branch("feature").unwrap();
    for bad in [
        "",
        "/x",
        "x/",
        "a//b",
        "../x",
        "a b",
        "x.lock",
        "feature/sub",
    ] {
        assert!(s.timeline().check_ref_name(bad).is_err(), "{bad:?}");
    }
    apply(&mut s, &brick(1.0));
    // "main" does not exist yet as a ref (we created "feature" from unborn main's tip).
    let mut t = s.timeline().clone();
    let tip = s.position().unwrap();
    t.delete_ref("main").unwrap();
    t.create_ref("dir/x", tip).unwrap();
    assert!(t.create_ref("dir", tip).is_err());
    assert!(matches!(
        t.create_ref("dir/x", tip),
        Err(TimelineError::RefExists(_))
    ));
    assert_eq!(Timeline::new().default_head(), Head::Ref("main".into()));
    assert_eq!(
        t.default_head(),
        Head::Ref("dir/x".into()),
        "first branch by name when main is absent"
    );
    t.create_ref("main", tip).unwrap();
    assert_eq!(t.default_head(), Head::Ref("main".into()));
}

// ------------------------------------------------------------------- undo / redo

#[test]
fn undo_moves_the_ref_and_the_undone_node_stays_checkoutable() {
    let mut s = session();
    let n1 = apply(&mut s, &brick(1.0)).0;
    let n2 = apply(&mut s, &brick(2.0)).0;
    let n3 = apply(&mut s, &brick(3.0)).0;
    assert_eq!(s.timeline().refs()["main"], n3);
    s.undo().unwrap();
    assert_eq!(s.timeline().refs()["main"], n2, "ref moved to the parent");
    assert_eq!(s.position(), Some(n2));
    assert_eq!(bricks(s.state()), 2);
    assert!(
        matches!(s.head(), Head::Ref(n) if n == "main"),
        "still attached"
    );
    assert!(
        s.timeline().get(n3).is_some(),
        "undone node remains in the timeline"
    );
    assert!(
        s.timeline()
            .to_files()
            .unwrap()
            .contains_key(&format!("ops/{n3}.toml"))
    );

    // Check the undone experiment out, branch from it, come back.
    let c = q();
    s.checkout(Rev::Node(n3), &c.control()).unwrap();
    assert_eq!(bricks(s.state()), 3);
    s.branch("rescued").unwrap();
    assert_eq!(s.timeline().refs()["rescued"], n3);
    s.checkout(Rev::Ref("main".into()), &c.control()).unwrap();
    assert_eq!(s.position(), Some(n2));

    // Redo (checkout cleared the redo stack, so redo is gone).
    assert!(matches!(s.redo(), Err(TimelineError::NothingToRedo)));
    s.undo().unwrap();
    s.redo().unwrap();
    assert_eq!(s.position(), Some(n2));
    s.undo().unwrap();
    s.undo().unwrap();
    assert_eq!(s.position(), None);
    assert!(
        !s.timeline().refs().contains_key("main"),
        "undoing the root leaves an unborn branch"
    );
    assert!(matches!(s.undo(), Err(TimelineError::NothingToUndo)));
    s.redo().unwrap();
    assert_eq!(s.timeline().refs()["main"], n1);
    s.redo().unwrap();
    assert_eq!(s.timeline().refs()["main"], n2);
    // A new op after undo clears redo and creates a sibling node.
    s.undo().unwrap();
    let n2b = apply(&mut s, &brick(22.0)).0;
    assert_ne!(n2b, n2);
    assert!(matches!(s.redo(), Err(TimelineError::NothingToRedo)));
    assert_eq!(s.timeline().children(n1).len(), 2);
    // Re-applying the same op and meta reproduces the same node.
    s.undo().unwrap();
    let again = apply(&mut s, &brick(22.0)).0;
    assert_eq!(again, n2b);
    assert!(s.reflog().iter().any(|e| e.action == "undo"));
    // Detached head cannot undo.
    s.checkout(Rev::Node(n1), &c.control()).unwrap();
    assert!(matches!(s.undo(), Err(TimelineError::NotAttached)));
}

#[test]
fn reset_branch_moves_the_ref_like_git_reset() {
    let mut s = session();
    let n1 = apply(&mut s, &brick(1.0)).0;
    apply(&mut s, &brick(2.0));
    let c = q();
    s.reset_branch("main", n1, &c.control()).unwrap();
    assert_eq!(s.position(), Some(n1));
    assert_eq!(bricks(s.state()), 1);
    assert_eq!(s.timeline().refs()["main"], n1);
    assert_eq!(s.timeline().len(), 2);
    assert!(s.reset_branch("nope", n1, &c.control()).is_err());
}

#[test]
fn reset_of_a_detached_branch_logs_its_previous_tip() {
    let mut s = session();
    let n1 = apply(&mut s, &brick(1.0)).0;
    let n2 = apply(&mut s, &brick(2.0)).0;
    s.branch("side").unwrap();
    let n3 = apply(&mut s, &brick(3.0)).0;
    let c = q();
    // HEAD is on `side` (at n3); reset `main`, which is at n2.
    s.reset_branch("main", n1, &c.control()).unwrap();
    assert_eq!(s.position(), Some(n3));
    let e = s.reflog().last().unwrap();
    assert_eq!(e.action, "reset");
    assert_eq!(e.ref_name.as_deref(), Some("main"));
    assert_eq!(
        e.from,
        Some(n2),
        "the abandoned tip, not the session position"
    );
    assert_eq!(e.to, Some(n1));
}

#[test]
fn retire_only_touches_layers_that_reference_the_ids() {
    let mut s = session();
    let a = apply(&mut s, &brick(1.0)).1.outputs["body"][0];
    let b = apply(&mut s, &brick(2.0)).1.outputs["body"][0];
    apply(&mut s, &SetSize { id: a, size: 0.3 });
    let (_, d) = apply(&mut s, &DeleteBody { id: b });
    assert!(d.touched_layers.contains(&"model"));
    assert!(
        !d.touched_layers.contains(&"intent"),
        "intent does not reference the retired id"
    );
    assert_eq!(s.state().layer::<IntentLayer>().unwrap().sizes.len(), 1);
}

// ------------------------------------------------------------------- derived cache

#[derive(Default)]
struct Computations(AtomicUsize);

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Tessellate {}

impl TypedOp for Tessellate {
    const NAME: &'static str = "test.tessellate";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let key =
            ctx.state()
                .cache_key("tess", &[names::MODEL], &serde_json::json!({"tol": 0.1}))?;
        let n = ctx.service::<Computations>()?;
        let nb = ctx.layer::<ModelLayer>()?.bricks.len();
        let t = ctx
            .derived()
            .get_or_compute::<usize, OpError>("tess", key, || {
                n.0.fetch_add(1, Ordering::SeqCst);
                Ok(nb * 12)
            })?;
        let id = ctx.alloc(EntityKind::Mesh)?;
        ctx.layer_mut::<IntentLayer>()?.sizes.insert(id, *t as f64);
        Ok(())
    }
}

#[test]
fn derived_cache_is_keyed_by_layer_content_hashes() {
    let mut r = registry();
    r.register::<Tessellate>().unwrap();
    let eng = engine_with(r, Services::new().with(Computations::default()));
    let mut s = Session::new(eng);

    apply(&mut s, &brick(1.0));
    apply(&mut s, &Tessellate {});
    let comps = |s: &Session| s.engine().cache().stats();
    assert_eq!(comps(&s).misses, 1);
    assert_eq!(comps(&s).entries, 1);

    // Same model content: reused (the intent layer changed, but it is not an input).
    apply(&mut s, &Tessellate {});
    assert_eq!(comps(&s).entries, 1, "reused");
    assert_eq!(comps(&s).hits, 1);

    // The model changed: a new key, a second entry.
    apply(&mut s, &brick(2.0));
    apply(&mut s, &Tessellate {});
    assert_eq!(comps(&s).entries, 2, "invalidated by the model change");

    // Going back to an earlier state reuses the older entry (cache survives checkout).
    let c = q();
    let tess1 = s.timeline().children(s.timeline().roots()[0])[0];
    s.checkout(Rev::Node(tess1), &c.control()).unwrap();
    let hits = comps(&s).hits;
    s.apply(&Tessellate {}, meta("again"), &c.control())
        .unwrap();
    assert_eq!(comps(&s).entries, 2);
    assert_eq!(comps(&s).hits, hits + 1);

    // Keys: content-based, platform independent, typed dependencies.
    let st = s.state();
    let k1 = st
        .cache_key("tess", &[names::MODEL], &serde_json::json!({"tol": 0.1}))
        .unwrap();
    let k2 = st
        .cache_key("tess", &[names::MODEL], &serde_json::json!({"tol": 0.2}))
        .unwrap();
    let k3 = st
        .cache_key("other", &[names::MODEL], &serde_json::json!({"tol": 0.1}))
        .unwrap();
    assert!(k1 != k2 && k1 != k3);
    assert!(matches!(
        st.cache_key("tess", &["nope"], &serde_json::json!({})),
        Err(OpError::MissingLayer(_))
    ));
}

#[test]
fn derived_cache_is_bounded_and_rejects_type_confusion() {
    let cache = DerivedCache::with_capacity(100);
    for i in 0u8..10 {
        let key = Hash256::of(&[i]);
        cache.insert_sized("k", key, Arc::new(i), 30).unwrap();
    }
    let st = cache.stats();
    assert!(st.bytes <= 100 && st.entries <= 3, "{st:?}");
    // Most recent entry survives, oldest is gone.
    assert!(cache.get::<u8>("k", Hash256::of(&[9])).is_some());
    assert!(cache.get::<u8>("k", Hash256::of(&[0])).is_none());
    // Same key, other type: error, and `get` as the wrong type is a miss.
    let key = Hash256::of(&[9]);
    assert!(matches!(
        cache.insert_sized("k", key, Arc::new("s".to_owned()), 1),
        Err(CacheError::TypeConflict { .. })
    ));
    assert!(cache.get::<String>("k", key).is_none());
    let r: Result<Arc<String>, CacheError> = cache.get_or_compute("k", key, || Ok("s".to_owned()));
    assert!(r.is_err());
}

// ------------------------------------------------------------------- on_retired

static LOG: Mutex<Vec<&'static str>> = Mutex::new(Vec::new());

macro_rules! chain_layer {
    ($t:ident, $name:literal, [$($dep:literal),*]) => {
        #[derive(Clone, Default)]
        struct $t(BTreeSet<EntityId>);
        impl LayerType for $t {
            const NAME: &'static str = $name;
            const DEPENDS_ON: &'static [&'static str] = &[$($dep),*];
            fn digest(&self, d: &mut Digest) { for i in &self.0 { d.id(*i); } }
            fn is_empty(&self) -> bool { self.0.is_empty() }
            fn on_retired(&mut self, ids: &[EntityId]) {
                if let Ok(mut l) = LOG.lock() { l.push($name); }
                for i in ids { self.0.remove(i); }
            }
        }
    };
}

// Name order (aa < mm < zz) is the reverse of the dependency order zz <- mm <- aa.
chain_layer!(Base, "zz_base", []);
chain_layer!(Mid, "mm_mid", ["zz_base"]);
chain_layer!(Leaf, "aa_leaf", ["mm_mid"]);

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Seed {}

impl TypedOp for Seed {
    const NAME: &'static str = "chain.seed";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let id = ctx.alloc(EntityKind::Group)?;
        ctx.layer_mut::<Base>()?.0.insert(id);
        ctx.layer_mut::<Mid>()?.0.insert(id);
        ctx.layer_mut::<Leaf>()?.0.insert(id);
        Ok(())
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Unseed {
    id: EntityId,
}

impl TypedOp for Unseed {
    const NAME: &'static str = "chain.unseed";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        ctx.retire(self.id)
    }
}

#[test]
fn on_retired_runs_in_declared_dag_order_not_name_order() {
    let mut r = Registry::new();
    // Registration order is also scrambled.
    r.register_layer::<Mid>().unwrap();
    r.register_layer::<Leaf>().unwrap();
    r.register_layer::<Base>().unwrap();
    r.register::<Seed>().unwrap();
    r.register::<Unseed>().unwrap();
    assert_eq!(
        r.layer_order().unwrap(),
        vec!["zz_base", "mm_mid", "aa_leaf"]
    );
    let mut s = Session::new(engine_with(r, Services::new()));
    let (_, d) = apply(&mut s, &Seed {});
    let id = d.created[0];
    LOG.lock().unwrap().clear();
    let (_, d) = apply(&mut s, &Unseed { id });
    let log: Vec<&str> = LOG
        .lock()
        .unwrap()
        .iter()
        .copied()
        .filter(|n| ["zz_base", "mm_mid", "aa_leaf"].contains(n))
        .collect();
    assert_eq!(log, vec!["zz_base", "mm_mid", "aa_leaf"]);
    assert_eq!(d.touched_layers, vec!["aa_leaf", "mm_mid", "zz_base"]);
    assert!(s.state().layer::<Leaf>().unwrap().0.is_empty());
    assert!(s.state().layer::<Base>().unwrap().0.is_empty());
}

#[test]
fn layer_dependency_cycles_and_dangling_dependencies_are_detected() {
    chain_layer!(CycA, "cyc_a", ["cyc_b"]);
    chain_layer!(CycB, "cyc_b", ["cyc_a"]);
    let mut r = Registry::new();
    r.register_layer::<CycA>().unwrap();
    // Missing dependency tolerated until the order is demanded strictly...
    assert!(matches!(
        r.layer_order(),
        Err(RegistryError::UnknownLayerDependency { .. })
    ));
    // ...but a cycle is rejected at install time.
    struct P;
    impl Plugin for P {
        fn id(&self) -> &str {
            "cyc"
        }
        fn register(&self, r: &mut Registry) -> Result<(), RegistryError> {
            r.register_layer::<CycB>()?;
            Ok(())
        }
    }
    let before = r.layer_roles().len();
    assert!(matches!(r.install(&P), Err(RegistryError::LayerCycle(_))));
    assert_eq!(
        r.layer_roles().len(),
        before,
        "failed install leaves the registry unchanged"
    );
}

#[test]
fn deleting_a_body_cleans_downstream_layers_in_the_same_transaction() {
    let mut s = session();
    let (_, d) = apply(&mut s, &brick(1.0));
    let id = d.outputs["body"][0];
    apply(&mut s, &SetSize { id, size: 0.3 });
    assert_eq!(s.state().layer::<IntentLayer>().unwrap().sizes.len(), 1);
    let (_, d) = apply(&mut s, &DeleteBody { id });
    assert_eq!(d.removed, vec![id]);
    assert!(
        s.state().layer::<IntentLayer>().unwrap().sizes.is_empty(),
        "no dangling intent"
    );
    assert_eq!(bricks(s.state()), 0);
    assert!(d.touched_layers.contains(&"intent"));
    // Replay agrees.
    let c = q();
    let tip = s.position().unwrap();
    assert_eq!(
        s.state_at_from_root(tip, &c.control()).unwrap().hash(),
        s.state().hash()
    );
}

// ------------------------------------------------------------------- determinism / divergence

#[derive(Default)]
struct Entropy(AtomicU64);

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Flaky {}

impl TypedOp for Flaky {
    const NAME: &'static str = "test.flaky";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        // Injected nondeterminism: depends on how many times any op has run.
        let n = ctx.service::<Entropy>()?.0.fetch_add(1, Ordering::SeqCst);
        let id = ctx.alloc(EntityKind::TopoBody)?;
        ctx.layer_mut::<ModelLayer>()?.bricks.insert(
            id,
            Brick {
                x: n as f64 + 1.0,
                y: 1.0,
                z: 1.0,
                name: String::new(),
            },
        );
        Ok(())
    }
}

fn flaky_session() -> Session {
    let mut r = registry();
    r.register::<Flaky>().unwrap();
    Session::new(engine_with(r, Services::new().with(Entropy::default())))
}

#[test]
fn diverged_is_raised_when_an_op_is_nondeterministic() {
    let mut s = flaky_session();
    apply(&mut s, &brick(1.0));
    let (tip, _) = apply(&mut s, &Flaky {});
    let c = q();
    let e = s.state_at_from_root(tip, &c.control()).unwrap_err();
    assert!(
        matches!(e, TimelineError::Diverged { node, .. } if node == tip),
        "{e:?}"
    );
    // Checkout away and back: strict by default.
    s.checkout(Rev::Empty, &c.control()).unwrap();
    s.set_cache_policy(1000, 0); // forbid any cached shortcut
    let e = s.checkout(Rev::Node(tip), &c.control()).unwrap_err();
    assert!(matches!(e, TimelineError::Diverged { .. }));
    assert_eq!(
        s.position(),
        None,
        "failed checkout leaves the session unchanged"
    );
    // Warn policy proceeds and records the divergence.
    s.set_replay_policy(ReplayPolicy::Warn);
    s.checkout(Rev::Node(tip), &c.control()).unwrap();
    let w = s.take_warnings();
    assert_eq!(w.len(), 1);
    assert!(matches!(w[0], TimelineError::Diverged { .. }));
}

#[test]
fn inserting_a_node_with_a_different_recorded_result_is_diverged() {
    let mut s = session();
    let (h, _) = apply(&mut s, &brick(1.0));
    let mut t = s.timeline().clone();
    let mut n = t.get(h).unwrap().clone();
    assert_eq!(
        t.insert(n.clone()).unwrap(),
        h,
        "identical insert is idempotent"
    );
    n.check.state = Hash256::of(b"other platform float bits");
    assert!(matches!(t.insert(n), Err(TimelineError::Diverged { .. })));
}

#[test]
fn node_identity_ignores_the_recorded_result() {
    let mut s = session();
    let (h, _) = apply(&mut s, &brick(1.0));
    let mut n = s.timeline().get(h).unwrap().clone();
    n.check.state = Hash256::of(b"linux float bits");
    assert_eq!(n.hash().unwrap(), h);
    // And a file whose check differs still loads under its own name (replay is what compares).
    let mut files = s.timeline().to_files().unwrap();
    let path = format!("ops/{h}.toml");
    let text = files[&path].replace(&s.state().hash().to_hex(), &Hash256::of(b"x").to_hex());
    files.insert(path, text);
    let t = Timeline::from_files(&files).unwrap();
    let c = q();
    let s2 = Session::open(
        engine_with(registry(), Services::new()),
        t,
        Head::Ref("main".into()),
        &c.control(),
    );
    assert!(matches!(s2, Err(TimelineError::Diverged { .. })));
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct SumSquares {
    n: u64,
    threads: u64,
}

impl TypedOp for SumSquares {
    const NAME: &'static str = "test.sum_squares";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        // Fixed chunking independent of the thread count; chunk results reduced in chunk order.
        const CHUNK: u64 = 64;
        let nchunks = self.n.div_ceil(CHUNK);
        let scope = ctx.scope();
        let results: Vec<Mutex<f64>> = (0..nchunks).map(|_| Mutex::new(0.0)).collect();
        let next = AtomicU64::new(0);
        let failed = Mutex::new(None);
        std::thread::scope(|sc| {
            for _ in 0..self.threads.max(1) {
                sc.spawn(|| {
                    loop {
                        let c = next.fetch_add(1, Ordering::SeqCst);
                        if c >= nchunks {
                            break;
                        }
                        let (lo, hi) = (c * CHUNK, ((c + 1) * CHUNK).min(self.n));
                        let part: f64 = (lo..hi).map(|i| (i as f64).sqrt() * 0.1).sum();
                        if let Ok(mut r) = results[c as usize].lock() {
                            *r = part;
                        }
                        if let Err(e) = scope.step(c, nchunks, "chunk") {
                            if let Ok(mut f) = failed.lock() {
                                *f = Some(e);
                            }
                            break;
                        }
                    }
                });
            }
        });
        if let Some(e) = failed.into_inner().ok().flatten() {
            return Err(e);
        }
        let total: f64 = results
            .iter()
            .map(|m| m.lock().map(|g| *g).unwrap_or(0.0))
            .sum();
        let id = ctx.alloc(EntityKind::TopoBody)?;
        ctx.layer_mut::<ModelLayer>()?.bricks.insert(
            id,
            Brick {
                x: total,
                y: 1.0,
                z: 1.0,
                name: String::new(),
            },
        );
        Ok(())
    }
}

#[test]
fn state_hash_is_identical_for_one_thread_and_many_and_across_runs() {
    let run = |threads: u64| {
        let mut r = registry();
        r.register::<SumSquares>().unwrap();
        let mut s = Session::new(engine_with(r, Services::new()));
        let rec = RecordingProgress::new();
        let tok = CancelToken::new();
        s.apply(
            &SumSquares { n: 5000, threads },
            meta(""),
            &Control::new(&rec, &tok),
        )
        .unwrap();
        assert!(
            !rec.events().is_empty(),
            "workers reported progress through ApplyCtxScope"
        );
        s.state().hash()
    };
    let one = run(1);
    assert_eq!(one, run(1), "run twice");
    assert_eq!(one, run(8), "1 thread vs N threads");
    assert_eq!(one, run(3));
}

// ------------------------------------------------------------------- snapshot hook

#[test]
fn persisted_snapshot_hook_defaults_to_none_and_round_trips_when_implemented() {
    // Default: no snapshot support.
    let mut s = session();
    apply(&mut s, &brick(1.0));
    assert!(
        LayerType::snapshot(&ModelLayer::default())
            .unwrap()
            .is_none()
    );
    assert!(ModelLayer::restore(&[]).is_err());
    assert!(
        s.state().snapshot().is_err(),
        "a non-empty layer without the hook cannot be snapshotted"
    );

    // A layer implementing the hook: snapshot, restore, identical hash.
    #[derive(Clone, Default)]
    struct Big(BTreeMap<u64, u64>);
    impl LayerType for Big {
        const NAME: &'static str = "big";
        fn digest(&self, d: &mut Digest) {
            d.value(&self.0);
        }
        fn is_empty(&self) -> bool {
            self.0.is_empty()
        }
        fn snapshot(&self) -> Result<Option<Vec<u8>>, CodecError> {
            Ok(Some(
                serde_json::to_vec(
                    &self
                        .0
                        .iter()
                        .map(|(k, v)| (k.to_string(), *v))
                        .collect::<BTreeMap<_, _>>(),
                )
                .map_err(|e| CodecError::new(e.to_string()))?,
            ))
        }
        fn restore(bytes: &[u8]) -> Result<Self, CodecError> {
            let m: BTreeMap<String, u64> =
                serde_json::from_slice(bytes).map_err(|e| CodecError::new(e.to_string()))?;
            Ok(Self(
                m.into_iter()
                    .filter_map(|(k, v)| Some((k.parse().ok()?, v)))
                    .collect(),
            ))
        }
    }
    #[derive(Debug, Serialize, Deserialize, JsonSchema)]
    struct Fill {
        n: u64,
    }
    impl TypedOp for Fill {
        const NAME: &'static str = "big.fill";
        fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
            for i in 0..self.n {
                let id = ctx.alloc(EntityKind::MeshCell)?;
                ctx.layer_mut::<Big>()?.0.insert(id.serial(), i * i);
            }
            let first = ctx
                .state()
                .peek_next(EntityKind::MeshCell)
                .ok_or(OpError::IdExhausted)?;
            ctx.retire(
                EntityId::new(EntityKind::MeshCell, first.serial() - 1)
                    .ok_or(OpError::IdExhausted)?,
            )?;
            Ok(())
        }
    }
    let mut r = Registry::new();
    r.register_layer::<Big>().unwrap();
    r.register::<Fill>().unwrap();
    let mut s = Session::new(engine_with(r.clone(), Services::new()));
    apply(&mut s, &Fill { n: 100 });
    let snap = s.state().snapshot().unwrap();
    assert_eq!(snap.layers.len(), 1);
    let restored = r.restore(&snap).unwrap();
    assert_eq!(restored.hash(), s.state().hash());
    assert_eq!(
        restored.peek_next(EntityKind::MeshCell),
        s.state().peek_next(EntityKind::MeshCell)
    );
    assert_eq!(restored.entity_count(), 99);
    // The snapshot itself is serializable by the storage layer.
    let text = serde_json::to_string(&snap).unwrap();
    assert_eq!(serde_json::from_str::<StateSnapshot>(&text).unwrap(), snap);
}

// ------------------------------------------------------------------- value codec

#[test]
fn params_codec_rejects_unrepresentable_values_before_anything_is_recorded() {
    let mut s = session();
    let c = q();
    let e = s
        .apply(
            &CreateBrick {
                x: f64::NAN,
                y: 1.0,
                z: 1.0,
                name: None,
            },
            meta(""),
            &c.control(),
        )
        .unwrap_err();
    assert!(
        matches!(e, TimelineError::Apply(ApplyError::Encode { .. })),
        "{e:?}"
    );
    assert!(s.timeline().is_empty());
    // JSON clients: nulls are dropped, big integers are rejected.
    let rec = OpRecord::new(
        "model.create_brick",
        1,
        serde_json::json!({"x": 1, "y": 1, "z": 1, "name": null}),
    )
    .unwrap();
    s.apply_record(rec, meta(""), &c.control()).unwrap();
    assert!(OpRecord::new("model.create_brick", 1, serde_json::json!({"x": u64::MAX})).is_err());
    assert!(OpRecord::new("model.create_brick", 1, serde_json::json!([1])).is_err());
}

// ------------------------------------------------------------------- review fixes

#[test]
fn sub_quantum_change_invalidates_cache_key_but_not_state_hash() {
    let mk = |x: f64| {
        let mut s = session();
        apply(
            &mut s,
            &CreateBrick {
                x,
                y: 1.0,
                z: 1.0,
                name: None,
            },
        );
        s
    };
    let (a, b) = (mk(1.0), mk(1.0 + 1e-11)); // quantum is 1e-9
    assert_eq!(
        a.state().hash(),
        b.state().hash(),
        "identity hash is quantized"
    );
    let key = |s: &Session| {
        s.state()
            .cache_key("tess", &[names::MODEL], &serde_json::json!({}))
            .unwrap()
    };
    assert_ne!(key(&a), key(&b), "cache keys use exact content");
    assert_eq!(key(&a), key(&mk(1.0)));
}

#[test]
fn typed_and_json_record_paths_give_the_same_node() {
    let c = q();
    let typed = {
        let mut s = session();
        s.apply(
            &CreateBrick {
                x: 1.0,
                y: 1.0,
                z: 1.0,
                name: None,
            },
            meta("m"),
            &c.control(),
        )
        .unwrap()
        .0
    };
    for params in [
        serde_json::json!({"x": 1, "y": 1, "z": 1}),
        serde_json::json!({"x": 1.0, "y": 1.0, "z": 1.0, "name": null}),
        serde_json::json!({"x": 1.0, "y": 1.0, "z": 1.0, "junk": "zzz"}),
    ] {
        let mut s = session();
        let rec = OpRecord::new("model.create_brick", 1, params).unwrap();
        let h = s.apply_record(rec, meta("m"), &c.control()).unwrap().0;
        assert_eq!(h, typed);
        let stored = &s.timeline().get(h).unwrap().body.op;
        assert_eq!(
            stored,
            &OpRecord::of(&CreateBrick {
                x: 1.0,
                y: 1.0,
                z: 1.0,
                name: None
            })
            .unwrap()
        );
    }
}

#[test]
fn map_layer_digest_with_quantizes_and_cache_key_stays_exact() {
    let mk = |x: f64| {
        let mut s = session();
        apply(
            &mut s,
            &CreateBrick {
                x,
                y: 1.0,
                z: 1.0,
                name: None,
            },
        );
        s
    };
    let (a, b) = (mk(1.0), mk(f64::from_bits(1.0f64.to_bits() + 1)));
    assert_eq!(a.state().hash(), b.state().hash());
    let key = |s: &Session| {
        s.state()
            .cache_key("tess", &[names::MODEL], &serde_json::json!({}))
            .unwrap()
    };
    assert_ne!(key(&a), key(&b));
}

fn batch_of(recs: Vec<OpRecord>) -> Batch {
    Batch { ops: recs }
}

#[test]
fn batch_typed_and_json_record_paths_give_the_same_node() {
    let c = q();
    let typed_inner = OpRecord::of(&CreateBrick {
        x: 1.0,
        y: 1.0,
        z: 1.0,
        name: None,
    })
    .unwrap();
    let size = OpRecord::new(
        "intent.set_size",
        1,
        serde_json::json!({"id": "$0.body", "size": 0.5}),
    )
    .unwrap();
    let typed = {
        let mut s = session();
        s.apply(
            &batch_of(vec![typed_inner.clone(), size.clone()]),
            meta("m"),
            &c.control(),
        )
        .unwrap()
        .0
    };
    let raw = |p: serde_json::Value| OpRecord {
        name: "model.create_brick".into(),
        version: 1,
        params: p,
    };
    let raw_size = |p: serde_json::Value| OpRecord {
        name: "intent.set_size".into(),
        version: 1,
        params: p,
    };
    for (inner, sz) in [
        (
            serde_json::json!({"x": 1, "y": 1, "z": 1}),
            serde_json::json!({"id": "$0.body", "size": 0.5}),
        ),
        (
            serde_json::json!({"x": 1.0, "y": 1.0, "z": 1.0, "name": null}),
            serde_json::json!({"id": "$0.body", "size": 0.5, "z": null}),
        ),
        (
            serde_json::json!({"x": 1.0, "y": 1.0, "z": 1.0, "junk": "zzz"}),
            serde_json::json!({"id": "$0.body", "size": 0.5, "junk": 1}),
        ),
        (
            serde_json::json!({"x": 1.0, "y": 1.0, "z": 1.0}),
            serde_json::json!({"id": "$0.body", "size": 0.5, "junk": "$0.body"}),
        ),
    ] {
        let mut s = session();
        let (h, _) = s
            .apply(
                &batch_of(vec![raw(inner), raw_size(sz)]),
                meta("m"),
                &c.control(),
            )
            .unwrap();
        assert_eq!(h, typed);
        let stored = &s.timeline().get(h).unwrap().body.op;
        assert_eq!(
            stored,
            &OpRecord::of(&batch_of(vec![typed_inner.clone(), size.clone()])).unwrap()
        );
        let text = format!("{stored:?}");
        assert!(text.contains("$0.body") && !text.contains("junk"));
        // The stored record replays to the recorded state.
        let st = s.state_at_from_root(h, &c.control()).unwrap();
        assert_eq!(st.hash(), s.state().hash());
    }
}

#[test]
fn batch_stores_escaped_literals_and_rejects_nesting() {
    let c = q();
    let mut s = session();
    let b = batch_of(vec![
        OpRecord::new(
            "model.create_brick",
            1,
            serde_json::json!({"x": 1, "y": 1, "z": 1, "name": "$$0.price"}),
        )
        .unwrap(),
    ]);
    let (h, _) = s.apply(&b, meta(""), &c.control()).unwrap();
    assert!(format!("{:?}", s.timeline().get(h).unwrap().body.op).contains("$$0.price"));
    let inner = OpRecord::of(&batch_of(vec![])).unwrap();
    assert!(
        s.apply(&batch_of(vec![inner]), meta(""), &c.control())
            .is_err()
    );
}

#[test]
fn batch_progress_is_monotone() {
    let mut r = registry();
    r.register::<LongOp>().unwrap();
    let mut s = Session::new(engine_with(r, Services::new()));
    let long = || OpRecord::of(&LongOp { steps: 4 }).unwrap();
    let rec = RecordingProgress::new();
    let tok = CancelToken::new();
    s.apply(
        &batch_of(vec![long(), long()]),
        meta(""),
        &Control::new(&rec, &tok),
    )
    .unwrap();
    let ev = rec.events();
    assert!(ev.windows(2).all(|w| w[0].0 <= w[1].0), "{ev:?}");
    assert_eq!(ev.last().unwrap().0, 1.0);
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct NamedOp {}

macro_rules! named_op {
    ($t:ident, $n:literal) => {
        struct $t;
        impl OpType for $t {
            fn name(&self) -> &'static str {
                $n
            }
            fn version(&self) -> u32 {
                1
            }
            fn schema(&self) -> Result<schemars::Schema, String> {
                Ok(schemars::schema_for!(NamedOp))
            }
            fn decode(&self, _: &Value) -> Result<Box<dyn Op>, CodecError> {
                Err(CodecError::new("unused"))
            }
        }
    };
}
named_op!(N1, "a.b");
named_op!(N2, "a__b");
named_op!(N3, "A.b");
named_op!(N4, "a..b");
named_op!(N5, "a b.c");
named_op!(N6, "");
named_op!(N7, "plain");
named_op!(N8, "a.b_c.d9");

#[test]
fn op_names_are_validated_so_tool_names_are_injective() {
    let mut r = Registry::new();
    r.register_dyn(Arc::new(N1)).unwrap();
    r.register_dyn(Arc::new(N8)).unwrap();
    for t in [
        Arc::new(N2) as Arc<dyn OpType>,
        Arc::new(N3),
        Arc::new(N4),
        Arc::new(N5),
        Arc::new(N6),
        Arc::new(N7),
    ] {
        assert!(matches!(
            r.register_dyn(t),
            Err(RegistryError::BadOpName(_))
        ));
    }
    let names: Vec<String> = r
        .catalog()
        .into_iter()
        .map(|i| i.unwrap().tool_name())
        .collect();
    let set: BTreeSet<_> = names.iter().collect();
    assert_eq!(set.len(), names.len());
}

#[test]
fn batch_dollar_escape_passes_literal_strings() {
    let mut s = session();
    let c = q();
    let batch = Batch {
        ops: vec![
            OpRecord::new(
                "model.create_brick",
                1,
                serde_json::json!({"x": 1, "y": 1, "z": 1, "name": "$$0.price"}),
            )
            .unwrap(),
        ],
    };
    let (_, d) = s.apply(&batch, meta(""), &c.control()).unwrap();
    let id = d.outputs["0.body"][0];
    assert_eq!(
        s.state()
            .layer::<ModelLayer>()
            .unwrap()
            .bricks
            .get(id)
            .unwrap()
            .name,
        "$0.price"
    );
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct AssertNoSizes {}

impl TypedOp for AssertNoSizes {
    const NAME: &'static str = "test.assert_no_sizes";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if ctx.layer::<IntentLayer>()?.sizes.is_empty() {
            Ok(())
        } else {
            Err(OpError::Failed("dangling size".into()))
        }
    }
}

#[test]
fn batch_layers_react_after_each_sub_op() {
    let mut r = registry();
    r.register::<AssertNoSizes>().unwrap();
    let mut s = Session::new(engine_with(r, Services::new()));
    let c = q();
    let rec = |n: &str, p: serde_json::Value| OpRecord::new(n, 1, p).unwrap();
    let batch = Batch {
        ops: vec![
            rec(
                "model.create_brick",
                serde_json::json!({"x": 1, "y": 1, "z": 1}),
            ),
            rec(
                "intent.set_size",
                serde_json::json!({"id": "$0.body", "size": 0.5}),
            ),
            rec("model.delete_body", serde_json::json!({"id": "$0.body"})),
            rec("test.assert_no_sizes", serde_json::json!({})),
        ],
    };
    let (_, d) = s.apply(&batch, meta(""), &c.control()).unwrap();
    assert!(d.touched_layers.contains(&"intent"));
}

#[test]
fn engine_new_fails_fast_on_a_dangling_layer_dependency() {
    chain_layer!(Dang, "dang", ["nope"]);
    let mut r = Registry::new();
    r.register_layer::<Dang>().unwrap();
    let e = Engine::new(
        Arc::new(r),
        Services::new(),
        Arc::new(DerivedCache::default()),
    );
    assert!(matches!(
        e,
        Err(RegistryError::UnknownLayerDependency { .. })
    ));
}

#[test]
fn prepared_from_another_engine_cannot_be_committed() {
    let (mut a, b) = (session(), session());
    let c = q();
    let p = b.prepare(&brick(1.0), &c.control()).unwrap();
    assert!(matches!(
        a.commit(p, meta("")),
        Err(TimelineError::ForeignPrepared)
    ));
    assert!(a.timeline().is_empty());
    // Own prepare/commit still works.
    let p = a.prepare(&brick(1.0), &c.control()).unwrap();
    a.commit(p, meta("")).unwrap();
}

#[test]
fn fork_from_node_3_of_10_replays_without_recent_cache() {
    let mut s = session();
    s.set_cache_policy(4, 0); // no recent-state LRU; depth snapshots at 4 and 8
    let nodes: Vec<_> = (1..=10)
        .map(|i| apply(&mut s, &brick(i as f64)).0)
        .collect();
    let c = q();
    let h10 = s.state().hash();
    s.checkout(Rev::Node(nodes[2]), &c.control()).unwrap(); // replays from the root
    assert_eq!(bricks(s.state()), 3);
    s.checkout(Rev::Node(nodes[5]), &c.control()).unwrap(); // from the depth-4 snapshot
    assert_eq!(bricks(s.state()), 6);
    s.checkout(Rev::Node(nodes[9]), &c.control()).unwrap();
    assert_eq!(s.state().hash(), h10);
}

#[test]
fn engine_and_apply_ctx_are_send_and_sync() {
    fn both<T: Send + Sync>() {}
    both::<Engine>();
    both::<ApplyCtx<'static>>();
    both::<State>();
    both::<DerivedCache>();
}

/// v2 of a label op; v1 had `a` and `b` the other way round.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct LabelV2 {
    a: EntityId,
    b: EntityId,
    label: String,
}

impl TypedOp for LabelV2 {
    const NAME: &'static str = "test.label";
    const VERSION: u32 = 2;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let a = ctx.resolve(self.a)?;
        ctx.resolve(self.b)?;
        let layer = ctx.layer_mut::<ModelLayer>()?;
        let mut br = layer
            .bricks
            .get(a)
            .cloned()
            .ok_or(OpError::Failed("no brick".into()))?;
        br.name = self.label.clone();
        layer.bricks.insert(a, br);
        Ok(())
    }
}

#[test]
fn migrated_batch_sub_op_keeps_dollar_literals_escaped_and_replays() {
    let mut r = registry();
    r.register::<LabelV2>().unwrap();
    r.register_migration("test.label", 1, |mut v| {
        if let Some(m) = v.as_object_mut() {
            let a = m.remove("a");
            let b = m.remove("b");
            m.insert("a".into(), b.unwrap_or_default());
            m.insert("b".into(), a.unwrap_or_default());
        }
        Ok(v)
    })
    .unwrap();
    let c = q();
    let mk = || OpRecord::of(&brick(1.0)).unwrap();
    let label = |lit: &str| {
        OpRecord::new(
            "test.label",
            1,
            serde_json::json!({"a": "$0.body", "b": "$1.body", "label": lit}),
        )
        .unwrap()
    };
    for lit in ["$$0.body", "$$$x", "$$"] {
        let mut s = Session::new(engine_with(r.clone(), Services::new()));
        let expect = &lit[1..];
        let (h, d) = s
            .apply(
                &batch_of(vec![mk(), mk(), label(lit)]),
                meta(""),
                &c.control(),
            )
            .unwrap();
        // v1 `a` ($0) became v2 `b`, so the label lands on op 1's brick.
        let id = d.outputs["1.body"][0];
        let layer = s.state().layer::<ModelLayer>().unwrap();
        assert_eq!(layer.bricks.get(id).unwrap().name, expect);
        let text = format!("{:?}", s.timeline().get(h).unwrap().body.op);
        assert!(text.contains(&format!("\"{lit}\"")), "{text}");
        let st = s.state_at_from_root(h, &c.control()).unwrap();
        assert_eq!(st.hash(), s.state().hash());
    }
}
