//! Classification: every mesh entity is owned by a model/partition entity (PUMI-style), with
//! parametric coordinates where they are defined.
//!
//! Rule: a mesh entity of dimension `d` may only be classified on a model entity of dimension
//! `>= d` (a vertex may sit on a model vertex, edge, face or volume; a cell only on a volume).

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};

use crate::error::{MeshError, MeshResult};
use crate::index::Dim;

/// Parametric coordinates of a vertex on its classifying entity.
#[derive(Clone, Copy, Debug, PartialEq, Default, Serialize, Deserialize)]
pub enum Param {
    /// No parametric coordinates (model vertex, volume, or not recorded).
    #[default]
    None,
    /// Curve parameter `u` on a topological edge.
    Curve(f64),
    /// Surface parameters `(u, v)` on a topological face.
    Surface([f64; 2]),
}

/// A classification: the owning model/partition entity plus parametric coordinates.
/// (Not serde-derivable because `EntityId` is frozen without serde; persist `entity.to_raw()`.)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Classification {
    /// Owning model or partition entity.
    pub entity: EntityId,
    /// Parametric coordinates (vertices only).
    pub param: Param,
}

impl Classification {
    /// Classification without parametric coordinates.
    pub const fn on(entity: EntityId) -> Self {
        Self { entity, param: Param::None }
    }
}

/// Dimension of a classifying entity kind, or `None` if the kind cannot classify mesh entities.
pub const fn model_dim(kind: EntityKind) -> Option<Dim> {
    match kind {
        EntityKind::TopoVertex => Some(Dim::Vertex),
        EntityKind::TopoEdge => Some(Dim::Edge),
        EntityKind::TopoFace => Some(Dim::Face),
        EntityKind::TopoVolume | EntityKind::PartitionCell => Some(Dim::Cell),
        _ => None,
    }
}

/// Check the dimension rule for classifying a `dim` mesh entity onto `target`.
pub fn check_classification(dim: Dim, target: EntityId) -> MeshResult<()> {
    match model_dim(target.kind()) {
        Some(md) if md >= dim => Ok(()),
        _ => Err(MeshError::BadClassification { dim, target }),
    }
}

/// Column of classifications for one dimension, with a deterministic reverse index
/// (`BTreeMap`) from model entity to the sorted list of mesh entities on it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ClassTable {
    entity: Vec<u64>,
    param: Vec<[f64; 2]>,
    reverse: BTreeMap<u64, Vec<u32>>,
}

impl ClassTable {
    /// A table of `len` unclassified rows.
    pub fn new(len: usize) -> Self {
        Self { entity: vec![0; len], param: vec![[f64::NAN; 2]; len], reverse: BTreeMap::new() }
    }
    /// Number of rows.
    pub fn len(&self) -> usize {
        self.entity.len()
    }
    /// True if there are no rows.
    pub fn is_empty(&self) -> bool {
        self.entity.is_empty()
    }
    /// Append an unclassified row.
    pub fn push_unclassified(&mut self) {
        self.entity.push(0);
        self.param.push([f64::NAN; 2]);
    }
    /// Classification of row `ix`.
    pub fn get(&self, ix: usize) -> Option<Classification> {
        let entity = EntityId::from_raw(*self.entity.get(ix)?)?;
        let p = self.param[ix];
        let param = match entity.kind() {
            EntityKind::TopoEdge if !p[0].is_nan() => Param::Curve(p[0]),
            EntityKind::TopoFace if !p[0].is_nan() && !p[1].is_nan() => Param::Surface(p),
            _ => Param::None,
        };
        Some(Classification { entity, param })
    }
    /// Set or clear the classification of row `ix`, keeping the reverse index in sync.
    pub fn set(&mut self, ix: usize, c: Option<Classification>) {
        if ix >= self.entity.len() {
            return;
        }
        let old = self.entity[ix];
        if old != 0
            && let Some(v) = self.reverse.get_mut(&old)
        {
            if let Ok(pos) = v.binary_search(&(ix as u32)) {
                v.remove(pos);
            }
            if v.is_empty() {
                self.reverse.remove(&old);
            }
        }
        match c {
            None => {
                self.entity[ix] = 0;
                self.param[ix] = [f64::NAN; 2];
            }
            Some(c) => {
                self.entity[ix] = c.entity.to_raw();
                self.param[ix] = match c.param {
                    Param::None => [f64::NAN; 2],
                    Param::Curve(u) => [u, f64::NAN],
                    Param::Surface(uv) => uv,
                };
                let v = self.reverse.entry(c.entity.to_raw()).or_default();
                if let Err(pos) = v.binary_search(&(ix as u32)) {
                    v.insert(pos, ix as u32);
                }
            }
        }
    }
    /// Mesh entities classified on `model`, ascending.
    pub fn on(&self, model: EntityId) -> &[u32] {
        self.reverse.get(&model.to_raw()).map_or(&[], Vec::as_slice)
    }
    /// All classifying entities in use, ascending by id.
    pub fn models(&self) -> Vec<EntityId> {
        self.reverse.keys().filter_map(|k| EntityId::from_raw(*k)).collect()
    }
    /// Raw entity column (`0` = unclassified), for hashing and serialization.
    pub fn raw_entities(&self) -> &[u64] {
        &self.entity
    }
}
