//! Contract tests. Everything here uses only the public API, the way a concept node would:
//! the api crate knows none of these layers or ops.

use std::collections::BTreeMap;
use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_op_api::*;

// ----------------------------------------------------------- sample "model" node

/// A box-shaped body.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Brick {
    x: f64,
    y: f64,
    z: f64,
    name: String,
}

#[derive(Clone, Default)]
struct ModelLayer {
    bricks: BTreeMap<u64, Brick>,
}

impl LayerType for ModelLayer {
    const NAME: &'static str = names::MODEL;
    fn digest(&self, d: &mut Digest) {
        d.value(&self.bricks);
    }
}

/// Create an axis-aligned brick centred at the origin.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct CreateBrick {
    /// Extent along x.
    x: f64,
    /// Extent along y.
    y: f64,
    /// Extent along z.
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
        let id = ctx.alloc(EntityKind::TopoBody)?;
        let b = Brick { x: self.x, y: self.y, z: self.z, name: self.name.clone().unwrap_or_default() };
        ctx.layer_mut::<ModelLayer>()?.bricks.insert(id.to_raw(), b);
        Ok(())
    }
}

/// Rename a body created by an earlier op.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Rename {
    target: EntityRef,
    name: String,
}

impl TypedOp for Rename {
    const NAME: &'static str = "model.rename";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let id = self.target.resolve(ctx)?;
        let m = ctx.layer_mut::<ModelLayer>()?;
        m.bricks.get_mut(&id.to_raw()).ok_or(OpError::UnknownEntity(id))?.name = self.name.clone();
        Ok(())
    }
}

/// Mutates the layer and allocates an id, then fails: nothing may survive.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct ScaleThenFail {
    target: EntityRef,
}

impl TypedOp for ScaleThenFail {
    const NAME: &'static str = "model.scale_then_fail";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let id = self.target.resolve(ctx)?;
        ctx.layer_mut::<ModelLayer>()?.bricks.entry(id.to_raw()).and_modify(|b| b.x *= 2.0);
        ctx.alloc(EntityKind::TopoFace)?;
        Err(OpError::Failed("boom".into()))
    }
}

struct ModelPlugin;
impl Plugin for ModelPlugin {
    fn id(&self) -> &str {
        "model"
    }
    fn register(&self, r: &mut Registry) -> Result<(), RegistryError> {
        r.register_layer::<ModelLayer>()?;
        r.register::<CreateBrick>()?;
        r.register::<Rename>()?;
        r.register::<ScaleThenFail>()?;
        Ok(())
    }
}

// ----------------------------------------------------------- sample "meshes" node

#[derive(Clone, Default)]
struct MeshLayer {
    cells: u64,
}

impl LayerType for MeshLayer {
    const NAME: &'static str = names::MESHES;
    fn digest(&self, d: &mut Digest) {
        d.u64(self.cells);
    }
}

/// Long-running op: creates `steps` cells, reporting progress and honouring cancellation.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct MeshAll {
    steps: u32,
}

impl TypedOp for MeshAll {
    const NAME: &'static str = "meshing.mesh_all";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        // Derived cache keyed by the content of the model layer.
        let key = ctx
            .state()
            .cache_key("mesh_all", &[names::MODEL], &to_value(self).map_err(|e| OpError::Failed(e.0))?)
            .ok_or(OpError::MissingLayer(names::MODEL))?;
        let tess: Arc<u64> = match ctx.cache_get::<u64>("mesh_all", key) {
            Some(t) => t,
            None => {
                let n = ctx.layer::<ModelLayer>()?.bricks.len() as u64;
                let t = Arc::new(n * 10);
                ctx.cache_put("mesh_all", key, t.clone());
                t
            }
        };
        for i in 0..self.steps {
            ctx.check_cancelled()?;
            ctx.alloc(EntityKind::MeshCell)?;
            ctx.layer_mut::<MeshLayer>()?.cells += *tess;
            ctx.report((i + 1) as f32 / self.steps as f32, "meshing");
        }
        Ok(())
    }
}

struct MeshPlugin;
impl Plugin for MeshPlugin {
    fn id(&self) -> &str {
        "meshing"
    }
    fn register(&self, r: &mut Registry) -> Result<(), RegistryError> {
        r.register_layer::<MeshLayer>()?;
        r.register::<MeshAll>()
            .map(|_| ())
    }
}

// ------------------------------------------------------------------- helpers

fn registry() -> Arc<Registry> {
    let mut r = Registry::new();
    r.install(&ModelPlugin).expect("model");
    r.install(&MeshPlugin).expect("mesh");
    Arc::new(r)
}

fn meta(t: u64) -> Meta {
    Meta { author: "test".into(), time: t, message: String::new() }
}

fn brick(x: f64) -> CreateBrick {
    CreateBrick { x, y: 2.0, z: 3.0, name: None }
}

fn script(s: &mut Session) -> Vec<Hash256> {
    let mut hs = Vec::new();
    for i in 0..5u64 {
        let (h, d) = s.apply(&brick(1.0 + i as f64), meta(i)).expect("brick");
        hs.push(h);
        let id = d.created[0];
        let (h, _) = s
            .apply(&Rename { target: id.into(), name: format!("b{i}") }, meta(100 + i))
            .expect("rename");
        hs.push(h);
    }
    hs
}

fn tmpdir(tag: &str) -> std::path::PathBuf {
    let p = std::env::temp_dir().join(format!("sm-op-api-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&p);
    p
}

// --------------------------------------------------------------------- tests

#[test]
fn replay_is_deterministic() {
    let reg = registry();
    let mut a = Session::new(reg.clone());
    let mut b = Session::new(reg.clone());
    let ha = script(&mut a);
    let hb = script(&mut b);
    assert_eq!(ha, hb, "node hashes identical");
    assert_eq!(a.state().hash(), b.state().hash());

    // A third session replays the first one's timeline from the root.
    a.set_snapshot_interval(1_000);
    let c = Session::with_timeline(reg.clone(), a.timeline().clone()).expect("replay");
    assert_eq!(c.state().hash(), a.state().hash());
    // And from an empty snapshot cache after checkout back and forth.
    let mut d = Session::with_timeline(reg, a.timeline().clone()).expect("replay");
    d.checkout(Rev::Node(ha[2])).expect("co");
    d.checkout(Rev::Ref("main".into())).expect("co");
    assert_eq!(d.state().hash(), a.state().hash());
}

#[test]
fn fork_from_old_node() {
    let mut s = Session::new(registry());
    let hs = script(&mut s);
    let main_tip = s.position().expect("tip");
    let main_state = s.state().hash();

    s.checkout(Rev::Node(hs[3])).expect("checkout old");
    assert_eq!(s.state().entity_count(), 2);
    s.branch("try-pave").expect("branch");
    let (fork, _) = s.apply(&brick(9.0), meta(500)).expect("fork op");
    assert_eq!(s.timeline().get(fork).expect("node").parent, Some(hs[3]));
    assert_eq!(s.timeline().refs()["try-pave"], fork);
    assert_eq!(s.timeline().refs()["main"], main_tip, "main untouched");
    assert_ne!(s.state().hash(), main_state);

    s.checkout(Rev::Ref("main".into())).expect("back");
    assert_eq!(s.state().hash(), main_state);
    assert_eq!(s.timeline().children(Some(hs[3])).len(), 2, "two children at the fork point");
    assert!(matches!(s.checkout(Rev::Ref("nope".into())), Err(TimelineError::UnknownRef(_))));
}

#[test]
fn op_references_id_from_earlier_op() {
    let mut s = Session::new(registry());
    let (_, d) = s.apply(&brick(1.0), meta(0)).expect("create");
    let id = d.created[0];
    assert_eq!(id.kind(), EntityKind::TopoBody);
    assert_eq!(id.serial(), 1);
    s.apply(&Rename { target: id.into(), name: "fuel".into() }, meta(1)).expect("rename");
    assert_eq!(s.state().layer::<ModelLayer>().expect("layer").bricks[&id.to_raw()].name, "fuel");

    // An id nobody created is rejected.
    let ghost = EntityId::new(EntityKind::TopoBody, 99).expect("id");
    let err = s.apply(&Rename { target: ghost.into(), name: "x".into() }, meta(2)).unwrap_err();
    assert!(matches!(err, TimelineError::Apply(ApplyError::Op { source: OpError::UnknownEntity(_), .. })));
}

#[test]
fn failing_op_changes_nothing() {
    let mut s = Session::new(registry());
    let (h, d) = s.apply(&brick(1.0), meta(0)).expect("create");
    let before = s.state().hash();
    let len = s.timeline().len();
    let err = s.apply(&ScaleThenFail { target: d.created[0].into() }, meta(1)).unwrap_err();
    assert!(matches!(err, TimelineError::Apply(ApplyError::Op { source: OpError::Failed(_), .. })));
    assert_eq!(s.state().hash(), before);
    assert_eq!(s.timeline().len(), len);
    assert_eq!(s.position(), Some(h));
    // The face id the failed op allocated is not burnt: next alloc still starts at 1.
    assert_eq!(s.state().peek_next(EntityKind::TopoFace).map(|i| i.serial()), Some(1));

    // Engine-level: same guarantee on a bare state.
    let eng = Engine::new(registry());
    let mut st = eng.registry().new_state();
    let h0 = st.hash();
    let bad = CreateBrick { x: -1.0, y: 1.0, z: 1.0, name: None };
    assert!(eng.apply(&mut st, &bad, &NoProgress).is_err());
    assert_eq!(st.hash(), h0);
}

#[test]
fn toml_roundtrip_has_stable_hash() {
    let mut s = Session::new(registry());
    let hs = script(&mut s);
    for h in hs {
        let node = s.timeline().get(h).expect("node");
        let text = node.to_toml().expect("toml");
        let back = Node::from_toml(&text).expect("parse");
        assert_eq!(&back, node);
        assert_eq!(back.to_toml().expect("toml2"), text, "canonical text is a fixed point");
        assert_eq!(Hash256::of(text.as_bytes()), h);
    }
    // Golden: the root node's file.
    let root = s.timeline().get(s.timeline().path_to_root(s.position().expect("p")).expect("path")[0]).expect("root");
    let text = root.to_toml().expect("toml");
    assert!(text.starts_with("state = \""), "{text}");
    assert!(text.contains("[op]\nname = \"model.create_brick\"\nversion = 1\n\n[op.params]\nx = 1.0\ny = 2.0\nz = 3.0\n"), "{text}");
    assert!(!text.contains("parent"), "root has no parent key");
}

#[test]
fn filesystem_layout_roundtrip_and_tamper_detection() {
    let dir = tmpdir("fs").join("timeline");
    let mut s = Session::new(registry());
    let hs = script(&mut s);
    s.checkout(Rev::Node(hs[1])).expect("co");
    s.branch("try-pave").expect("branch");
    fs_save(&s, &dir);

    assert!(dir.join("HEAD").exists());
    assert!(dir.join("refs/heads/main").exists());
    assert!(dir.join(format!("ops/{}.toml", hs[0])).exists());
    assert_eq!(std::fs::read_to_string(dir.join("HEAD")).expect("head"), "ref: refs/heads/try-pave\n");

    let loaded = sm_op_api::fs::load(&dir).expect("load");
    assert_eq!(loaded.len(), s.timeline().len());
    assert_eq!(loaded.refs(), s.timeline().refs());
    assert_eq!(loaded.head(), s.timeline().head());
    let s2 = Session::with_timeline(registry(), loaded).expect("resume");
    assert_eq!(s2.state().hash(), s.state().hash());

    // Tamper with a node file: load must refuse.
    let p = dir.join(format!("ops/{}.toml", hs[0]));
    let text = std::fs::read_to_string(&p).expect("read").replace("x = 1.0", "x = 5.0");
    std::fs::write(&p, text).expect("write");
    assert!(matches!(sm_op_api::fs::load(&dir), Err(TimelineError::HashMismatch { .. })));
    let _ = std::fs::remove_dir_all(dir.parent().expect("parent"));
}

fn fs_save(s: &Session, dir: &std::path::Path) {
    sm_op_api::fs::save(s.timeline(), dir).expect("save");
}

#[test]
fn schema_generation() {
    let reg = registry();
    let json = reg.schema_json("model.create_brick").expect("schema");
    for needle in ["\"properties\"", "\"x\"", "\"y\"", "\"z\"", "\"name\"", "Create an axis-aligned brick", "Extent along x"] {
        assert!(json.contains(needle), "missing {needle} in {json}");
    }
    let cat = reg.catalog();
    let names: Vec<_> = cat.iter().map(|o| o.name.as_str()).collect();
    assert_eq!(names, ["meshing.mesh_all", "model.create_brick", "model.rename", "model.scale_then_fail"]);
    assert!(matches!(reg.schema("nope"), Err(RegistryError::UnknownOp(_))));
}

#[test]
fn progress_and_cancellation() {
    let mut s = Session::new(registry());
    s.apply(&brick(1.0), meta(0)).expect("brick");
    let before = s.state().hash();

    let rec = RecordingProgress::new();
    s.apply_with(&MeshAll { steps: 4 }, meta(1), &rec).expect("mesh");
    let ev = rec.events();
    assert_eq!(ev.len(), 4);
    assert_eq!(ev.last().map(|e| e.0), Some(1.0));
    let done = s.state().hash();
    assert_ne!(done, before);

    let cancel = RecordingProgress::cancel_after(2);
    let err = s.apply_with(&MeshAll { steps: 100 }, meta(2), &cancel).unwrap_err();
    assert!(matches!(err, TimelineError::Apply(ApplyError::Op { source: OpError::Cancelled, .. })));
    assert_eq!(s.state().hash(), done, "partial meshing discarded");

    let tok = CancelToken::new();
    tok.cancel();
    assert!(s.apply_with(&MeshAll { steps: 1 }, meta(3), &tok).is_err());
}

#[test]
fn derived_cache_does_not_affect_identity() {
    let reg = registry();
    let eng = Engine::new(reg.clone());
    let mut st = reg.new_state();
    eng.apply(&mut st, &brick(1.0), &NoProgress).expect("brick");
    let mut warm = st.clone();
    eng.apply(&mut warm, &MeshAll { steps: 0 }, &NoProgress).expect("warm cache");
    assert_eq!(warm.layer::<DerivedCache>().expect("cache").len(), 1);
    assert!(st.layer::<DerivedCache>().expect("cache").is_empty(), "clone is isolated");

    // Same content hash with and without the cache; later meshing from either state agrees.
    assert_eq!(warm.hash(), st.hash());
    let mut cold = st;
    eng.apply(&mut cold, &MeshAll { steps: 3 }, &NoProgress).expect("cold");
    eng.apply(&mut warm, &MeshAll { steps: 3 }, &NoProgress).expect("warm");
    assert_eq!(cold.hash(), warm.hash());

    // The cache key tracks input content.
    let k1 = warm.cache_key("k", &[names::MODEL], &Value::Int(1)).expect("key");
    eng.apply(&mut warm, &brick(7.0), &NoProgress).expect("brick");
    let k2 = warm.cache_key("k", &[names::MODEL], &Value::Int(1)).expect("key");
    assert_ne!(k1, k2);
    assert!(warm.cache_key("k", &["absent"], &Value::Int(1)).is_none());
}

#[test]
fn snapshots_speed_up_checkout_without_changing_results() {
    let reg = registry();
    let mut s = Session::new(reg.clone());
    s.set_snapshot_interval(3);
    let hs = script(&mut s);
    assert!(s.snapshot_count() >= 3);
    let tip_hash = s.state().hash();
    for (i, h) in hs.iter().enumerate() {
        s.checkout(Rev::Node(*h)).expect("co");
        let want = s.timeline().get(*h).expect("n").state;
        assert_eq!(s.state().hash(), want, "node {i}");
    }
    s.checkout(Rev::Ref("main".into())).expect("main");
    assert_eq!(s.state().hash(), tip_hash);
    s.checkout(Rev::Empty).expect("empty");
    assert_eq!(s.state().entity_count(), 0);
}

#[test]
fn registry_is_open_and_strict() {
    let mut r = Registry::new();
    r.install(&ModelPlugin).expect("first");
    assert!(matches!(r.install(&ModelPlugin), Err(RegistryError::DuplicateLayer(_))));
    assert!(matches!(r.register::<Rename>(), Err(RegistryError::DuplicateOp(_))));

    // Unknown op on replay is a typed error.
    let rec = OpRecord { name: "x.y".into(), version: 1, params: Value::Table(BTreeMap::new()) };
    assert!(matches!(r.decode(&rec), Err(RegistryError::UnknownOp(_))));
    // Layers are dynamic: a state only has what was registered.
    let st = r.new_state();
    assert_eq!(st.layer_names().collect::<Vec<_>>(), ["derived", "model"]);
    assert!(st.layer::<MeshLayer>().is_none());
}

/// Version 2 renamed `size` to `x`; v1 records are migrated on decode.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
struct Cube {
    x: f64,
}

impl TypedOp for Cube {
    const NAME: &'static str = "model.cube";
    const VERSION: u32 = 2;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        ctx.alloc(EntityKind::TopoBody).map(|_| ())
    }
}

#[test]
fn op_versions_migrate() {
    let mut r = Registry::new();
    r.register::<Cube>().expect("reg");
    r.register_migration("model.cube", 1, |v| match v {
        Value::Table(mut t) => {
            let s = t.remove("size").ok_or_else(|| CodecError::new("no size"))?;
            t.insert("x".into(), s);
            Ok(Value::Table(t))
        }
        _ => Err(CodecError::new("not a table")),
    });
    let old = OpRecord {
        name: "model.cube".into(),
        version: 1,
        params: Value::Table(BTreeMap::from([("size".to_owned(), Value::Float(2.0))])),
    };
    let op = r.decode(&old).expect("migrated");
    assert_eq!(op.as_any().downcast_ref::<Cube>().map(|c| c.x), Some(2.0));
    let future = OpRecord { version: 3, ..old.clone() };
    assert!(matches!(r.decode(&future), Err(RegistryError::UnsupportedVersion { .. })));
    let mut r2 = Registry::new();
    r2.register::<Cube>().expect("reg");
    assert!(matches!(r2.decode(&old), Err(RegistryError::MissingMigration { .. })));
}
