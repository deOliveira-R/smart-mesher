//! A small sample stack: the reference layers used by tests, docs and schema examples.
//!
//! These are *not* the real layers (those live in `sm-model`, `sm-sizing`, ...). They show the
//! pattern every concept node follows: a store type, a closed `#[serde(tag = "kind")]` op enum,
//! and `impl Layer`.

use std::collections::BTreeMap;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityKind;

use crate::ctx::ApplyCtx;
use crate::error::OpError;
use crate::id::Id;
use crate::layer::{Layer, Stack4};

/// The sample stack.
pub type SampleStack = Stack4<ModelStore, PartitionStore, IntentStore, MeshStore>;

fn finite3(v: [f64; 3]) -> bool {
    v.iter().all(|x| x.is_finite())
}

/// A box-shaped body with its single volume.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Brick {
    /// The brick's volume entity.
    pub volume: Id,
    /// Minimum corner.
    pub origin: [f64; 3],
    /// Edge lengths.
    pub size: [f64; 3],
}

/// Sample model layer: bricks keyed by body id.
#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct ModelStore {
    /// Bodies by id.
    pub bodies: BTreeMap<Id, Brick>,
}

impl ModelStore {
    /// The brick owning `volume`.
    pub fn brick_of_volume(&self, volume: Id) -> Option<&Brick> {
        self.bodies.values().find(|b| b.volume == volume)
    }
}

/// Model-layer ops.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "kind")]
pub enum ModelOp {
    /// Create an axis-aligned brick (one body and one volume).
    CreateBrick {
        /// Minimum corner `[x, y, z]`.
        origin: [f64; 3],
        /// Edge lengths `[x, y, z]`, all positive.
        size: [f64; 3],
    },
    /// Translate bodies by a vector.
    Translate {
        /// Body ids to move.
        targets: Vec<Id>,
        /// Displacement `[x, y, z]`.
        by: [f64; 3],
    },
    /// Delete bodies and their volumes.
    Delete {
        /// Body ids to delete.
        targets: Vec<Id>,
    },
}

impl Layer for ModelStore {
    const NAME: &'static str = "model";
    const OP_VERSION: u32 = 1;
    type Op = ModelOp;

    fn apply(&self, op: &ModelOp, ctx: &mut ApplyCtx<'_>) -> Result<Self, OpError> {
        let mut next = self.clone();
        match op {
            ModelOp::CreateBrick { origin, size } => {
                if !finite3(*origin) || !finite3(*size) || size.iter().any(|s| *s <= 0.0) {
                    return Err(OpError::Invalid(
                        "brick needs finite origin and positive size".into(),
                    ));
                }
                let body = ctx.alloc(EntityKind::TopoBody)?;
                let volume = ctx.alloc(EntityKind::TopoVolume)?;
                next.bodies.insert(
                    body,
                    Brick {
                        volume,
                        origin: *origin,
                        size: *size,
                    },
                );
            }
            ModelOp::Translate { targets, by } => {
                if !finite3(*by) {
                    return Err(OpError::Invalid("non-finite displacement".into()));
                }
                for t in targets {
                    let b = next.bodies.get_mut(t).ok_or(OpError::UnknownEntity(*t))?;
                    for (o, d) in b.origin.iter_mut().zip(by) {
                        *o += d;
                    }
                    ctx.modified(*t);
                }
            }
            ModelOp::Delete { targets } => {
                for t in targets {
                    let b = next.bodies.remove(t).ok_or(OpError::UnknownEntity(*t))?;
                    ctx.removed(*t);
                    ctx.removed(b.volume);
                }
            }
        }
        Ok(next)
    }
}

/// Sample partition layer: partition cells by the volume they cut.
#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct PartitionStore {
    /// Partition cell id to model volume id.
    pub cells: BTreeMap<Id, Id>,
}

/// Partition-layer ops.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "kind")]
pub enum PartitionOp {
    /// Virtually split a volume into `parts` partition cells.
    Split {
        /// The model volume to split.
        volume: Id,
        /// Number of cells, at least 2.
        parts: u32,
    },
}

impl Layer for PartitionStore {
    const NAME: &'static str = "partition";
    const OP_VERSION: u32 = 1;
    type Op = PartitionOp;

    fn apply(&self, op: &PartitionOp, ctx: &mut ApplyCtx<'_>) -> Result<Self, OpError> {
        let PartitionOp::Split { volume, parts } = op;
        let model = ctx.upstream.require::<ModelStore>("model")?;
        if model.brick_of_volume(*volume).is_none() {
            return Err(OpError::UnknownEntity(*volume));
        }
        if *parts < 2 {
            return Err(OpError::Invalid("a split needs at least 2 parts".into()));
        }
        let mut next = self.clone();
        for _ in 0..*parts {
            let cell = ctx.alloc(EntityKind::PartitionCell)?;
            next.cells.insert(cell, *volume);
        }
        Ok(next)
    }
}

/// Sample intent layer: target element size per volume or partition cell.
#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct IntentStore {
    /// Element size by entity id.
    pub sizes: BTreeMap<Id, f64>,
}

/// Intent-layer ops.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "kind")]
pub enum IntentOp {
    /// Set the element size on volumes or partition cells.
    SetSize {
        /// Volume or partition-cell ids.
        targets: Vec<Id>,
        /// Element size, positive.
        size: f64,
    },
}

impl Layer for IntentStore {
    const NAME: &'static str = "intent";
    const OP_VERSION: u32 = 1;
    type Op = IntentOp;

    fn apply(&self, op: &IntentOp, ctx: &mut ApplyCtx<'_>) -> Result<Self, OpError> {
        let IntentOp::SetSize { targets, size } = op;
        if !size.is_finite() || *size <= 0.0 {
            return Err(OpError::Invalid("size must be positive".into()));
        }
        let model = ctx.upstream.require::<ModelStore>("model")?;
        let part = ctx.upstream.require::<PartitionStore>("partition")?;
        let mut next = self.clone();
        for t in targets {
            let known = match t.kind() {
                EntityKind::TopoVolume => model.brick_of_volume(*t).is_some(),
                EntityKind::PartitionCell => part.cells.contains_key(t),
                _ => {
                    return Err(OpError::WrongKind {
                        id: *t,
                        expected: EntityKind::TopoVolume,
                    });
                }
            };
            if !known {
                return Err(OpError::UnknownEntity(*t));
            }
            next.sizes.insert(*t, *size);
            ctx.modified(*t);
        }
        Ok(next)
    }
}

/// A generated mesh of one volume.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct MeshRecord {
    /// The meshed model volume.
    pub volume: Id,
    /// Number of hexahedral cells.
    pub cells: u64,
}

/// Sample meshes layer.
#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct MeshStore {
    /// Meshes by id.
    pub meshes: BTreeMap<Id, MeshRecord>,
}

/// Meshes-layer ops.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "kind")]
pub enum MeshOp {
    /// Mesh volumes with structured hexes at the intent size (default 1.0). Long-running:
    /// reports progress per volume and honours cancellation.
    Generate {
        /// Model volume ids to mesh.
        volumes: Vec<Id>,
    },
}

impl Layer for MeshStore {
    const NAME: &'static str = "meshes";
    const OP_VERSION: u32 = 1;
    type Op = MeshOp;

    fn apply(&self, op: &MeshOp, ctx: &mut ApplyCtx<'_>) -> Result<Self, OpError> {
        let MeshOp::Generate { volumes } = op;
        let model = ctx.upstream.require::<ModelStore>("model")?;
        let intent = ctx.upstream.require::<IntentStore>("intent")?;
        let mut next = self.clone();
        let total = volumes.len() as u64;
        for (i, v) in volumes.iter().enumerate() {
            ctx.step(i as u64, total, "meshing volume")?;
            let brick = model
                .brick_of_volume(*v)
                .ok_or(OpError::UnknownEntity(*v))?;
            let h = intent.sizes.get(v).copied().unwrap_or(1.0);
            let cells: u64 = brick
                .size
                .iter()
                .map(|s| (s / h).ceil().max(1.0) as u64)
                .product();
            let mesh = ctx.alloc(EntityKind::Mesh)?;
            next.meshes.insert(mesh, MeshRecord { volume: *v, cells });
        }
        ctx.step(total, total, "done")?;
        Ok(next)
    }
}
