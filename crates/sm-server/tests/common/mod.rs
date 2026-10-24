//! Demo plugins for the server tests: a small `demo` layer with ops that follow the W2 conventions
//! (`OUTPUTS`, `x-entity-kind`), a mesh op over `CoreLayers`, and the `topo.related` query.
#![allow(dead_code, missing_docs)]

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_engine::{FixedClock, WorkspaceBuilder};
use sm_layer_api::{CoreLayers, MeshEntry, MeshLayer, ModelLayer, insert_mesh};
use sm_mesh_api::reference::hex_block;
use sm_op_api::{
    ApplyCtx, Cardinality, Digest, LayerType, MapLayer, OpError, OutputRole, Plugin, QueryCtx, Registry,
    RegistryError, State, TypedOp, TypedQuery,
};

pub const AUTHOR: &str = "tester";
pub const TIME: &str = "2026-10-24T00:00:00Z";

#[derive(Clone, Default)]
pub struct DemoLayer {
    pub items: MapLayer<String>,
}

impl LayerType for DemoLayer {
    const NAME: &'static str = "demo";
    fn digest(&self, d: &mut Digest) {
        self.items.digest_with(d, |d, s| {
            d.str(s);
        });
    }
    fn is_empty(&self) -> bool {
        self.items.is_empty()
    }
    fn on_retired(&mut self, ids: &[EntityId]) {
        self.items.on_retired(ids);
    }
}

/// Create an item (a body-kind entity) with a name.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddItem {
    /// The item name.
    pub name: String,
}

impl TypedOp for AddItem {
    const NAME: &'static str = "demo.add_item";
    const OUTPUTS: &'static [OutputRole] =
        &[OutputRole::new("item", &[EntityKind::TopoBody], Cardinality::One, "the new item")];
    const POSITIONAL: Option<&'static str> = Some("name");
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let id = ctx.alloc_as("item", EntityKind::TopoBody)?;
        ctx.layer_mut::<DemoLayer>()?.items.insert(id, self.name.clone());
        Ok(())
    }
}

/// Rename an item.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Rename {
    /// The item.
    #[schemars(extend("x-entity-kind" = ["TopoBody"]))]
    pub item: EntityId,
    /// The new name.
    pub name: String,
}

impl TypedOp for Rename {
    const NAME: &'static str = "demo.rename";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let id = ctx.resolve_kind(self.item, EntityKind::TopoBody)?;
        *ctx.layer_mut::<DemoLayer>()?.items.get_mut(id).ok_or(OpError::UnknownEntity(id))? = self.name.clone();
        Ok(())
    }
}

/// Insert an `n^3` hex block as a mesh entry (needs `CoreLayers`).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddHex {
    /// Cells per side.
    pub n: u32,
}

impl TypedOp for AddHex {
    const NAME: &'static str = "demo.add_hex";
    const OUTPUTS: &'static [OutputRole] =
        &[OutputRole::new("mesh", &[EntityKind::Mesh], Cardinality::One, "the new mesh")];
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let quanta = ctx.layer::<ModelLayer>()?.hash_quanta();
        let before = ctx.layer::<MeshLayer>()?.alloc().clone();
        let mut alloc = before.clone();
        let mesh = hex_block([self.n; 3], [1.0; 3], &mut alloc).map_err(|e| OpError::Failed(e.to_string()))?;
        insert_mesh(ctx, MeshEntry::new(Box::new(mesh), Vec::new(), None, quanta), &before, alloc, "mesh")
    }
}

/// Entities of `kind` related to `within` (here: the ones of that kind in `within`).
#[derive(Debug, Deserialize, JsonSchema)]
pub struct Related {
    pub kind: String,
    #[schemars(extend("x-entity-kind" = ["TopoBody"]))]
    pub within: Vec<EntityId>,
}

impl TypedQuery for Related {
    const NAME: &'static str = "topo.related";
    type Output = Vec<EntityId>;
    fn run(&self, _: &State, _: &QueryCtx<'_>) -> Result<Vec<EntityId>, OpError> {
        let kind = self.kind.parse::<EntityKind>().map_err(|e| OpError::Invalid(e.to_string()))?;
        let mut v: Vec<EntityId> = self.within.iter().copied().filter(|i| i.kind() == kind).collect();
        v.sort();
        Ok(v)
    }
}

/// Number of items.
#[derive(Debug, Deserialize, JsonSchema)]
pub struct CountItems {}

impl TypedQuery for CountItems {
    const NAME: &'static str = "demo.count";
    type Output = u64;
    fn run(&self, state: &State, _: &QueryCtx<'_>) -> Result<u64, OpError> {
        Ok(state.layer::<DemoLayer>().map_or(0, |l| l.items.len() as u64))
    }
}

pub struct DemoPlugin;

impl Plugin for DemoPlugin {
    fn id(&self) -> &str {
        "demo"
    }
    fn register(&self, r: &mut Registry) -> Result<(), RegistryError> {
        r.register_layer::<DemoLayer>()?;
        r.register::<AddItem>()?;
        r.register::<Rename>()?;
        r.register::<AddHex>()?;
        r.register_query::<Related>()?;
        r.register_query::<CountItems>()?;
        Ok(())
    }
}

/// Demo plugins over `CoreLayers`, with a pinned clock and author (node hashes reproducible).
pub fn demo_builder() -> WorkspaceBuilder {
    WorkspaceBuilder::new()
        .plugin(CoreLayers::default())
        .plugin(DemoPlugin)
        .clock(FixedClock(TIME.to_owned()))
        .author(AUTHOR)
}

/// A unique scratch directory under the system temp dir, removed on drop.
pub struct TempDir(PathBuf);

impl TempDir {
    pub fn new(tag: &str) -> Self {
        static N: AtomicU64 = AtomicU64::new(0);
        let p = std::env::temp_dir().join(format!("sm-server-{tag}-{}-{}", std::process::id(), N.fetch_add(1, Ordering::Relaxed)));
        std::fs::create_dir_all(&p).expect("temp dir");
        Self(p)
    }
    pub fn path(&self) -> &Path {
        &self.0
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
