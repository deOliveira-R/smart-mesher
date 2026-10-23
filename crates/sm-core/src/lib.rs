//! Shared primitives for smart-mesh.
//!
//! This crate is a frozen contract: only the orchestrator edits it. See `CLAUDE.md`.

pub mod id;
pub mod geom;

pub use geom::{Point3, Transform, Vector3};
pub use id::{EntityId, EntityKind};
