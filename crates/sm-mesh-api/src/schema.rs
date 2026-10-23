//! Schema half of the schema/instance split: what tags and sets a mesh declares.
//!
//! The schema is small, serializable and versioned; the instance (tables, columns, set members)
//! lives in the implementor. A validator checks an instance against its schema.

use serde::{Deserialize, Serialize};

use crate::index::Dim;

/// Current schema version of the mesh layer.
pub const SCHEMA_VERSION: u32 = 1;

/// Id of the default polyhedral geometry rule (addendum §7.5).
pub const FAN_VERTEX_MEAN_V1: &str = "smartmesh.fan_vertex_mean.v1";

/// Element type of a tag column.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TagDType {
    /// Signed 64-bit integers (material ids, flags, counters).
    I64,
    /// 64-bit floats.
    F64,
    /// Raw [`sm_core::EntityId`] values, `0` meaning none.
    Id,
}

/// Declaration of one typed tag column.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TagDef {
    /// Tag name, unique per dimension.
    pub name: String,
    /// Entity dimension the column is attached to.
    pub dim: Dim,
    /// Element type.
    pub dtype: TagDType,
    /// Components per entity (>= 1).
    pub width: usize,
}

/// Declaration of one named entity set.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SetDef {
    /// Set name, unique across the mesh.
    pub name: String,
    /// Dimension of the members.
    pub dim: Dim,
}

/// The declared schema of a mesh.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MeshSchema {
    /// Schema version ([`SCHEMA_VERSION`]).
    pub version: u32,
    /// Id of the face/cell geometry rule used for derived geometry.
    pub geometry_rule: String,
    /// Tag columns.
    pub tags: Vec<TagDef>,
    /// Sets.
    pub sets: Vec<SetDef>,
}

impl Default for MeshSchema {
    fn default() -> Self {
        Self {
            version: SCHEMA_VERSION,
            geometry_rule: FAN_VERTEX_MEAN_V1.to_string(),
            tags: Vec::new(),
            sets: Vec::new(),
        }
    }
}

impl MeshSchema {
    /// Find a tag declaration.
    pub fn tag(&self, dim: Dim, name: &str) -> Option<&TagDef> {
        self.tags.iter().find(|t| t.dim == dim && t.name == name)
    }
    /// Find a set declaration.
    pub fn set(&self, name: &str) -> Option<&SetDef> {
        self.sets.iter().find(|s| s.name == name)
    }
}
