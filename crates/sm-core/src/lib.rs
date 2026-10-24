//! Shared primitives for smart-mesh.
//!
//! This crate is a frozen contract: only the orchestrator edits it. See `CLAUDE.md`.

pub mod id;
pub mod geom;

pub use geom::{Aabb, Point3, Transform, Vector3};
pub use id::{ENTITY_ID_PATTERN, EntityId, EntityKind};
