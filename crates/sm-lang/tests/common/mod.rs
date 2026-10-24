//! Demo ops, queries, layer and relations used by the integration tests (real geometry ops come later).
#![allow(dead_code)]

use std::collections::BTreeMap;
use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_lang::{EngineHost, Relations};
use sm_op_api::*;

#[derive(Clone, Default)]
pub struct DemoModel {
    pub bricks: BTreeMap<EntityId, (f64, f64, f64, String)>,
    pub volume_of: BTreeMap<EntityId, EntityId>,
    pub faces_of: BTreeMap<EntityId, Vec<EntityId>>,
    pub sizes: BTreeMap<EntityId, f64>,
    pub meshed: BTreeMap<EntityId, String>,
    pub notes: Vec<String>,
}

impl LayerType for DemoModel {
    const NAME: &'static str = names::MODEL;
    fn digest(&self, d: &mut Digest) {
        for (id, (x, y, z, n)) in &self.bricks {
            d.id(*id).f64_quantized(*x, 1e-9).f64_quantized(*y, 1e-9).f64_quantized(*z, 1e-9).str(n);
        }
        for (id, s) in &self.sizes {
            d.id(*id).f64_quantized(*s, 1e-9);
        }
        for (id, s) in &self.meshed {
            d.id(*id).str(s);
        }
        for n in &self.notes {
            d.str(n);
        }
    }
    fn is_empty(&self) -> bool {
        self.bricks.is_empty() && self.notes.is_empty() && self.sizes.is_empty() && self.meshed.is_empty()
    }
    fn on_retired(&mut self, ids: &[EntityId]) {
        for id in ids {
            self.bricks.remove(id);
            self.volume_of.remove(id);
            self.faces_of.remove(id);
            self.sizes.remove(id);
            self.meshed.remove(id);
        }
    }
}

/// Create a brick: a body with one volume and six faces.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct CreateBrick {
    /// Extent along x.
    pub x: f64,
    /// Extent along y.
    pub y: f64,
    /// Extent along z.
    pub z: f64,
    /// Optional display name.
    pub name: Option<String>,
}

impl TypedOp for CreateBrick {
    const NAME: &'static str = "model.create_brick";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if self.x <= 0.0 || self.y <= 0.0 || self.z <= 0.0 {
            return Err(OpError::Invalid("extents must be positive".into()));
        }
        let body = ctx.alloc_as("body", EntityKind::TopoBody)?;
        let vol = ctx.alloc_as("volume", EntityKind::TopoVolume)?;
        let mut faces = Vec::new();
        for _ in 0..6 {
            faces.push(ctx.alloc_as("faces", EntityKind::TopoFace)?);
        }
        let m = ctx.layer_mut::<DemoModel>()?;
        m.bricks.insert(body, (self.x, self.y, self.z, self.name.clone().unwrap_or_default()));
        m.volume_of.insert(body, vol);
        m.faces_of.insert(vol, faces);
        Ok(())
    }
}

/// Delete a body with its volume and faces.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct DeleteBody {
    /// The body to delete.
    pub id: EntityId,
}

impl TypedOp for DeleteBody {
    const NAME: &'static str = "model.delete_body";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let body = ctx.resolve_kind(self.id, EntityKind::TopoBody)?;
        let m = ctx.layer::<DemoModel>()?;
        let vol = m.volume_of.get(&body).copied();
        let faces = vol.and_then(|v| m.faces_of.get(&v).cloned()).unwrap_or_default();
        for f in faces {
            ctx.retire(f)?;
        }
        if let Some(v) = vol {
            ctx.retire(v)?;
        }
        ctx.retire(body)
    }
}

/// Multiply a brick's x extent.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct ScaleBrick {
    /// The body.
    pub id: EntityId,
    /// Scale factor.
    pub factor: f64,
}

impl TypedOp for ScaleBrick {
    const NAME: &'static str = "model.scale_brick";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let id = ctx.resolve_kind(self.id, EntityKind::TopoBody)?;
        let b = ctx.layer_mut::<DemoModel>()?.bricks.get_mut(&id).ok_or(OpError::UnknownEntity(id))?;
        b.0 *= self.factor;
        Ok(())
    }
}

/// Remember one entity (single required parameter: positional shorthand).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct Pick {
    /// The entity to remember.
    pub id: EntityId,
}

impl TypedOp for Pick {
    const NAME: &'static str = "model.pick";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        ctx.require(self.id)?;
        ctx.layer_mut::<DemoModel>()?.notes.push(self.id.to_string());
        Ok(())
    }
}

/// Store a text note.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct Note {
    /// The text.
    pub text: String,
}

impl TypedOp for Note {
    const NAME: &'static str = "model.note";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        ctx.layer_mut::<DemoModel>()?.notes.push(self.text.clone());
        Ok(())
    }
}

/// Set the target element size on entities.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct SetSize {
    /// Entities that get the size.
    pub ids: Vec<EntityId>,
    /// Target size.
    #[schemars(range(min = 0.001))]
    pub size: f64,
}

impl TypedOp for SetSize {
    const NAME: &'static str = "intent.set_size";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        for id in &self.ids {
            ctx.require(*id)?;
        }
        let m = ctx.layer_mut::<DemoModel>()?;
        for id in &self.ids {
            m.sizes.insert(*id, self.size);
        }
        Ok(())
    }
}

/// Meshing scheme.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Scheme {
    /// Structured mapping.
    Map,
    /// Paving.
    Pave,
}

/// Mesh volumes with a scheme.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct MeshRun {
    /// Volumes to mesh.
    pub targets: Vec<EntityId>,
    /// The scheme.
    pub scheme: Scheme,
    /// Merge coincident nodes afterwards.
    #[serde(default)]
    pub merge: bool,
    /// Element order (1 or 2).
    #[schemars(range(min = 1, max = 2))]
    pub order: Option<u32>,
}

impl TypedOp for MeshRun {
    const NAME: &'static str = "mesh.run";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        for id in &self.targets {
            ctx.resolve_kind(*id, EntityKind::TopoVolume)?;
        }
        let m = ctx.layer_mut::<DemoModel>()?;
        for id in &self.targets {
            m.meshed.insert(*id, format!("{:?}/{}/{}", self.scheme, self.merge, self.order.unwrap_or(1)));
        }
        Ok(())
    }
}

/// Allocate `n` entities of a kind in bulk (scale tests).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct AllocMany {
    /// Entity kind name, e.g. `TopoFace`.
    pub kind: String,
    /// How many.
    pub n: u64,
}

impl TypedOp for AllocMany {
    const NAME: &'static str = "test.alloc_many";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let kind = self.kind.parse::<EntityKind>().map_err(|e| OpError::Invalid(e.to_string()))?;
        let keys: Vec<u64> = (0..self.n).collect();
        ctx.alloc_sorted(kind, &keys)?;
        Ok(())
    }
}

/// Count entities (optionally of one kind).
#[derive(Debug, Deserialize, JsonSchema)]
pub struct Count {
    /// Entity kind name, e.g. `TopoVolume`.
    pub kind: Option<String>,
}

/// Result of `model.count`.
#[derive(Debug, Serialize, JsonSchema)]
pub struct CountOut {
    /// How many.
    pub count: u64,
}

impl TypedQuery for Count {
    const NAME: &'static str = "model.count";
    type Output = CountOut;
    fn run(&self, state: &State, _: &QueryCtx<'_>) -> Result<CountOut, OpError> {
        let n = match &self.kind {
            None => state.entity_count(),
            Some(k) => state.count_of(k.parse::<EntityKind>().map_err(|e| OpError::Invalid(e.to_string()))?),
        };
        Ok(CountOut { count: n as u64 })
    }
}

/// Entities of `kind` related to `within` (adjacency).
#[derive(Debug, Deserialize, JsonSchema)]
pub struct Related {
    /// Target kind name.
    pub kind: String,
    /// Entities to start from.
    pub within: Vec<EntityId>,
}

impl TypedQuery for Related {
    const NAME: &'static str = "topo.related";
    type Output = Vec<EntityId>;
    fn run(&self, state: &State, _: &QueryCtx<'_>) -> Result<Vec<EntityId>, OpError> {
        let kind = self.kind.parse::<EntityKind>().map_err(|e| OpError::Invalid(e.to_string()))?;
        Ok(related(state, kind, &self.within))
    }
}

pub fn related(state: &State, target: EntityKind, within: &[EntityId]) -> Vec<EntityId> {
    let Some(m) = state.layer::<DemoModel>() else { return Vec::new() };
    let mut out = Vec::new();
    for w in within {
        match (w.kind(), target) {
            (EntityKind::TopoVolume, EntityKind::TopoFace) => out.extend(m.faces_of.get(w).into_iter().flatten().copied()),
            (EntityKind::TopoBody, EntityKind::TopoVolume) => out.extend(m.volume_of.get(w).copied()),
            (EntityKind::TopoBody, EntityKind::TopoFace) => out.extend(m.volume_of.get(w).and_then(|v| m.faces_of.get(v)).into_iter().flatten().copied()),
            (EntityKind::TopoFace, EntityKind::TopoVolume) => out.extend(m.faces_of.iter().filter(|(_, fs)| fs.contains(w)).map(|(v, _)| *v)),
            _ => {}
        }
    }
    out
}

/// Direct adjacency over the demo layer.
pub struct DemoRelations;

impl Relations for DemoRelations {
    fn related(&self, state: &State, target: EntityKind, within: &[EntityId]) -> Result<Vec<EntityId>, String> {
        Ok(related(state, target, within))
    }
}

pub fn registry() -> Registry {
    let mut r = Registry::new();
    r.register_layer::<DemoModel>().unwrap();
    r.register::<CreateBrick>().unwrap();
    r.register::<DeleteBody>().unwrap();
    r.register::<ScaleBrick>().unwrap();
    r.register::<Pick>().unwrap();
    r.register::<Note>().unwrap();
    r.register::<SetSize>().unwrap();
    r.register::<MeshRun>().unwrap();
    r.register::<AllocMany>().unwrap();
    r.register_query::<Count>().unwrap();
    r.register_query::<Related>().unwrap();
    r
}

pub fn engine() -> Engine {
    Engine::new(Arc::new(registry()), Services::new(), Arc::new(DerivedCache::default())).unwrap()
}

pub fn host(engine: &Engine) -> EngineHost<'_> {
    EngineHost::new(engine, engine.registry().new_state()).with_relations(DemoRelations)
}
