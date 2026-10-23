//! Contract for the mesh database ("Meshes" layer, docs/design/ir.md section 5).
//!
//! Approach: **face-first**. The backbone is the oriented cell<->face table and the face
//! `(left, right)` pair, with face->vertex loops. Shapes are derived tags, edges are an optional
//! derived table, and classification, high order, sets and hierarchy are side tables.
//!
//! * Consumers use [`MeshRead`] (and [`MeshCatalog`]); meshers use [`MeshBuild`]; modifiers use
//!   [`MeshEdit`] and [`CatalogEdit`].
//! * [`mod@reference`] is an in-memory implementation used as the oracle for the contract tests.
//!
//! See `DESIGN.md` for the model and trade-offs.

pub mod build;
pub mod classify;
pub mod edges;
pub mod error;
pub mod geometry;
pub mod hierarchy;
pub mod highorder;
pub mod index;
pub mod read;
pub mod reference;
pub mod sets;
pub mod shape;
pub mod validate;

pub use build::{MeshBuild, MeshEdit, SerialAlloc};
pub use classify::{ClassSet, Classification, Param};
pub use edges::{EdgeTable, FaceEdge};
pub use error::{MeshError, Result};
pub use hierarchy::{
    AppendMap, AppendOptions, CatalogEdit, CellMap, FlattenMap, Instance, InstancedView, MeshCatalog, append_transformed, flatten, unfold,
};
pub use highorder::{Basis, CellGeometry, HighOrderCell, NodeRef};
pub use index::{CellFace, CellIx, EdgeIx, FaceIx, MeshEntity, MeshEntityKind, Side, VertexIx};
pub use read::{Coords, MeshRead};
pub use sets::{MeshSet, SetMembers, TagColumn};
pub use shape::{CellShape, FaceShape, LoopRelation, loop_relation};
pub use validate::{ValidationIssue, ValidationReport, content_hash, validate, validate_geometry};
