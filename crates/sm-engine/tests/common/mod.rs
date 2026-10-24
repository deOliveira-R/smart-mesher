//! Shared test support: a minimal demo plugin set (a counter layer, ops and queries split over
//! two plugins), temp directories, and a directory hash. Included by tests and benches.
#![allow(dead_code, missing_docs)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_engine::{ApplyOptions, Durability, FixedClock, Workspace, WorkspaceBuilder};
use sm_op_api::{
    ApplyCtx, CodecError, Digest, Hash256, LayerType, MapLayer, OpError, Plugin, QueryCtx,
    Registry, RegistryError, Services, State, TypedOp, TypedQuery,
};

// ------------------------------------------------------------------ layer

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Counter {
    pub name: String,
    pub value: i64,
    pub gain: f64,
}

#[derive(Clone, Default)]
pub struct CounterLayer {
    pub counters: MapLayer<Counter>,
}

impl LayerType for CounterLayer {
    const NAME: &'static str = "counter";
    fn digest(&self, d: &mut Digest) {
        self.counters.digest_with(d, |d, c| {
            d.str(&c.name).i64(c.value).f64_quantized(c.gain, 1e-9);
        });
    }
    fn is_empty(&self) -> bool {
        self.counters.is_empty()
    }
    fn on_retired(&mut self, ids: &[EntityId]) {
        self.counters.on_retired(ids);
    }
    fn references_any(&self, ids: &[EntityId]) -> bool {
        ids.iter().any(|i| self.counters.get(*i).is_some())
    }
    fn supports_snapshot(&self) -> bool {
        true
    }
    fn snapshot(&self) -> Result<Option<Vec<u8>>, CodecError> {
        let v: Vec<(EntityId, &Counter)> = self.counters.iter().collect();
        serde_json::to_vec(&v)
            .map(Some)
            .map_err(|e| CodecError::new(e.to_string()))
    }
    fn restore(bytes: &[u8]) -> Result<Self, CodecError> {
        let v: Vec<(EntityId, Counter)> =
            serde_json::from_slice(bytes).map_err(|e| CodecError::new(e.to_string()))?;
        let mut counters = MapLayer::new();
        for (id, c) in v {
            counters.insert(id, c);
        }
        Ok(Self { counters })
    }
}

// ------------------------------------------------------------------ services

/// Counts how many ops were applied (replay included), to prove snapshots skip replay.
#[derive(Clone, Default)]
pub struct ApplyCount(pub Arc<AtomicUsize>);

/// Source of nondeterminism for `counter.flaky`.
#[derive(Default)]
pub struct Flaky(pub AtomicU64);

fn tick(ctx: &ApplyCtx<'_>) -> Result<(), OpError> {
    ctx.service::<ApplyCount>()?
        .0
        .fetch_add(1, Ordering::SeqCst);
    Ok(())
}

// ------------------------------------------------------------------ ops

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct Create {
    pub name: String,
    pub initial: i64,
    pub gain: Option<f64>,
}

impl TypedOp for Create {
    const NAME: &'static str = "counter.create";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        tick(ctx)?;
        let id = ctx.alloc_as("counter", EntityKind::TopoBody)?;
        let c = Counter {
            name: self.name.clone(),
            value: self.initial,
            gain: self.gain.unwrap_or(1.0),
        };
        ctx.layer_mut::<CounterLayer>()?.counters.insert(id, c);
        Ok(())
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct Add {
    pub id: EntityId,
    pub delta: i64,
}

impl TypedOp for Add {
    const NAME: &'static str = "counter.add";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        tick(ctx)?;
        let id = ctx.resolve_kind(self.id, EntityKind::TopoBody)?;
        let c = ctx
            .layer_mut::<CounterLayer>()?
            .counters
            .get_mut(id)
            .ok_or(OpError::UnknownEntity(id))?;
        c.value += self.delta;
        Ok(())
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct AddAll {
    pub delta: i64,
}

impl TypedOp for AddAll {
    const NAME: &'static str = "counter.add_all";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        tick(ctx)?;
        let ids: Vec<EntityId> = ctx
            .layer::<CounterLayer>()?
            .counters
            .iter()
            .map(|(i, _)| i)
            .collect();
        let layer = ctx.layer_mut::<CounterLayer>()?;
        for id in ids {
            if let Some(c) = layer.counters.get_mut(id) {
                c.value += self.delta;
            }
        }
        Ok(())
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct ScaleGain {
    pub id: EntityId,
    pub factor: f64,
}

impl TypedOp for ScaleGain {
    const NAME: &'static str = "counter.scale_gain";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        tick(ctx)?;
        if !self.factor.is_finite() {
            return Err(OpError::Invalid("factor must be finite".into()));
        }
        let id = ctx.resolve_kind(self.id, EntityKind::TopoBody)?;
        let c = ctx
            .layer_mut::<CounterLayer>()?
            .counters
            .get_mut(id)
            .ok_or(OpError::UnknownEntity(id))?;
        c.gain *= self.factor;
        Ok(())
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct Delete {
    pub id: EntityId,
}

impl TypedOp for Delete {
    const NAME: &'static str = "counter.delete";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        tick(ctx)?;
        ctx.retire(self.id)
    }
}

/// Adds 1 and reports one warning (`demo.note`) and one info note (`demo.fyi`).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct Warn {
    pub id: EntityId,
}

impl TypedOp for Warn {
    const NAME: &'static str = "counter.warn";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        tick(ctx)?;
        let id = ctx.resolve_kind(self.id, EntityKind::TopoBody)?;
        let c = ctx
            .layer_mut::<CounterLayer>()?
            .counters
            .get_mut(id)
            .ok_or(OpError::UnknownEntity(id))?;
        c.value += 1;
        ctx.warn("demo.note", "value is odd", &[id]);
        ctx.info("demo.fyi", "for your information", &[id]);
        Ok(())
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct Fail {}

impl TypedOp for Fail {
    const NAME: &'static str = "counter.fail";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        tick(ctx)?;
        let id = ctx.alloc(EntityKind::TopoBody)?;
        ctx.layer_mut::<CounterLayer>()?.counters.insert(
            id,
            Counter {
                name: "ghost".into(),
                value: 9,
                gain: 1.0,
            },
        );
        Err(OpError::Failed("boom".into()))
    }
}

/// Result depends on how often it was applied: replay must report `Diverged`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct FlakyOp {}

impl TypedOp for FlakyOp {
    const NAME: &'static str = "counter.flaky";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        tick(ctx)?;
        let n = ctx.service::<Flaky>()?.0.fetch_add(1, Ordering::SeqCst);
        let id = ctx.alloc(EntityKind::TopoBody)?;
        ctx.layer_mut::<CounterLayer>()?.counters.insert(
            id,
            Counter {
                name: "flaky".into(),
                value: n as i64,
                gain: 1.0,
            },
        );
        Ok(())
    }
}

// ------------------------------------------------------------------ queries

#[derive(Debug, Deserialize, JsonSchema)]
pub struct Total {}

#[derive(Debug, Serialize, JsonSchema)]
pub struct TotalOut {
    pub total: i64,
    pub count: usize,
}

impl TypedQuery for Total {
    const NAME: &'static str = "counter.total";
    type Output = TotalOut;
    fn run(&self, state: &State, _: &QueryCtx<'_>) -> Result<TotalOut, OpError> {
        let l = state
            .layer::<CounterLayer>()
            .ok_or_else(|| OpError::MissingLayer("counter".into()))?;
        Ok(TotalOut {
            total: l.counters.iter().map(|(_, c)| c.value).sum(),
            count: l.counters.len(),
        })
    }
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct GetGain {
    pub id: EntityId,
}

impl TypedQuery for GetGain {
    const NAME: &'static str = "counter.gain";
    type Output = f64;
    fn run(&self, state: &State, _: &QueryCtx<'_>) -> Result<f64, OpError> {
        let l = state
            .layer::<CounterLayer>()
            .ok_or_else(|| OpError::MissingLayer("counter".into()))?;
        l.counters
            .get(self.id)
            .map(|c| c.gain)
            .ok_or(OpError::UnknownEntity(self.id))
    }
}

// ------------------------------------------------------------------ plugins

/// The layer, creation, mutation and the queries.
pub struct CounterPlugin;

impl Plugin for CounterPlugin {
    fn id(&self) -> &str {
        "demo.counter"
    }
    fn register(&self, r: &mut Registry) -> Result<(), RegistryError> {
        r.register_layer::<CounterLayer>()?;
        r.register::<Create>()?;
        r.register::<Add>()?;
        r.register::<AddAll>()?;
        r.register_query::<Total>()?;
        r.register_query::<GetGain>()?;
        Ok(())
    }
}

/// Extra ops, installed separately to prove registration order does not matter.
pub struct ExtrasPlugin;

impl Plugin for ExtrasPlugin {
    fn id(&self) -> &str {
        "demo.extras"
    }
    fn register(&self, r: &mut Registry) -> Result<(), RegistryError> {
        r.register::<ScaleGain>()?;
        r.register::<Delete>()?;
        r.register::<Warn>()?;
        r.register::<Fail>()?;
        r.register::<FlakyOp>()?;
        Ok(())
    }
}

// ------------------------------------------------------------------ helpers

pub const TIME: &str = "2026-01-01T00:00:00Z";

/// A builder with both plugins, a fixed clock, fast durability and the apply counter.
pub fn builder() -> (WorkspaceBuilder, Arc<AtomicUsize>) {
    let count = ApplyCount::default();
    let c = count.0.clone();
    let b = Workspace::builder()
        .plugin(CounterPlugin)
        .plugin(ExtrasPlugin)
        .services(Services::new().with(count).with(Flaky::default()))
        .clock(FixedClock(TIME.to_owned()))
        .author("tester")
        .durability(Durability::Fast);
    (b, c)
}

pub fn opts() -> ApplyOptions {
    ApplyOptions::default()
}

/// Create a counter and return its id.
pub fn create(ws: &mut Workspace, name: &str, initial: i64) -> EntityId {
    let r = ws
        .apply(
            &Create {
                name: name.into(),
                initial,
                gain: None,
            },
            &opts(),
        )
        .expect("create");
    r.delta.outputs["counter"][0]
}

pub fn add(ws: &mut Workspace, id: EntityId, delta: i64) -> Hash256 {
    ws.apply(&Add { id, delta }, &opts()).expect("add").node
}

/// Total of all counters at the current state, through the JSON query path.
pub fn total(ws: &Workspace) -> i64 {
    ws.query("counter.total", &serde_json::json!({}))
        .expect("query")["total"]
        .as_i64()
        .expect("i64")
}

// ------------------------------------------------------------------ temp dirs

static SEQ: AtomicU64 = AtomicU64::new(0);

pub struct TempDir(pub PathBuf);

impl TempDir {
    pub fn new(tag: &str) -> Self {
        let base =
            option_env!("CARGO_TARGET_TMPDIR").map_or_else(std::env::temp_dir, PathBuf::from);
        let p = base.join(format!(
            "sm-engine-{tag}-{}-{}",
            std::process::id(),
            SEQ.fetch_add(1, Ordering::Relaxed)
        ));
        let _ = std::fs::remove_dir_all(&p);
        std::fs::create_dir_all(&p).expect("mkdir");
        Self(p)
    }
    pub fn path(&self) -> &Path {
        &self.0
    }
    /// A not-yet-existing project path inside the temp dir.
    pub fn project(&self, name: &str) -> PathBuf {
        self.0.join(name)
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// All regular files under `root` as `relative path -> bytes`, optionally skipping `.cache`
/// and `.git`.
pub fn tree(root: &Path, skip_cache: bool) -> BTreeMap<String, Vec<u8>> {
    fn go(dir: &Path, rel: &str, skip: bool, out: &mut BTreeMap<String, Vec<u8>>) {
        let mut es: Vec<_> = std::fs::read_dir(dir)
            .expect("read_dir")
            .map(|e| e.expect("entry"))
            .collect();
        es.sort_by_key(|e| e.file_name());
        for e in es {
            let name = e.file_name().to_string_lossy().into_owned();
            if name == ".git" || (skip && name == ".cache") {
                continue;
            }
            let rel = if rel.is_empty() {
                name
            } else {
                format!("{rel}/{name}")
            };
            if e.file_type().expect("type").is_dir() {
                go(&e.path(), &rel, skip, out);
            } else {
                out.insert(rel, std::fs::read(e.path()).expect("read"));
            }
        }
    }
    let mut out = BTreeMap::new();
    go(root, "", skip_cache, &mut out);
    out
}

/// Hash of the project files (not `.cache`).
pub fn tree_hash(root: &Path) -> Hash256 {
    let mut buf = Vec::new();
    for (k, v) in tree(root, true) {
        buf.extend_from_slice(&(k.len() as u64).to_le_bytes());
        buf.extend_from_slice(k.as_bytes());
        buf.extend_from_slice(&(v.len() as u64).to_le_bytes());
        buf.extend_from_slice(&v);
    }
    Hash256::of(&buf)
}
