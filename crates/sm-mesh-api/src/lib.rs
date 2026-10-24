//! Frozen contract for the mesh database ("Meshes" layer, docs/design/ir.md section 5).
//!
//! Approach: **face-first**. The backbone is the oriented cell<->face table (packed
//! [`SignedFace`]) and the face `(left, right)` pair, with face->vertex loops. Shapes are
//! derived tags with template corner lists, edges are an optional derived table, and
//! classification, high order, lineage, sets, tags and hierarchy are side tables.
//!
//! * Consumers use [`MeshRead`] (and [`MeshCatalog`]); meshers use [`MeshBuild`]; modifiers use
//!   [`MeshEdit`] and [`CatalogEdit`]. Geometry is in [`geometry`] (allocation-free free
//!   functions), selection in [`select`], checks and content hashes in [`mod@validate`].
//! * [`mod@reference`] is an in-memory implementation used as the oracle for the contract tests.
//!
//! Optional feature `serde` derives `Serialize`/`Deserialize` on the plain value types
//! (`EntityId` serializes as `Kind#serial`; binary backends store `EntityId::to_raw`).
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
pub mod instance;
pub mod read;
pub mod reference;
pub mod select;
pub mod sets;
pub mod shape;
pub mod validate;

pub use build::{CellAdded, Finished, MeshBuild, MeshEdit, SerialAlloc};
pub use classify::{ClassSet, Classification, Param, entity_dim};
pub use edges::{EdgeTable, FaceEdge};
pub use error::{MeshError, Result};
pub use geometry::{
    FAN_VERTEX_MEAN_V1, FaceGeometry, cell_volume, cell_volume_centroid, compute_geometry_cache,
    face_apex, face_area_vector, face_centroid, face_geometry,
};
pub use hierarchy::{CatalogEdit, ExtrusionFaceRole, ExtrusionMap, LevelMap, MeshCatalog};
pub use highorder::{
    Basis, CellGeometry, FaceCurve, HighOrderCell, NodeRef, NodeSlot, face_node_count, node_count,
    node_layout,
};
pub use index::{
    CellIx, EdgeIx, FaceIx, FaceKind, MAX_FACES, MeshDim, MeshEntity, MeshEntityKind, MeshEntityRef,
    NO_CELL, PointIx, Side, SignedFace, VertexIx,
};
pub use instance::{
    AppendMap, FlattenMap, FlattenOptions, Instance, InstancedView, LossKind, LossNote, MergeCtx,
    MergeRule, Weld, append_transformed, compose, finalize_welds, flatten, mirror_transform,
    unfold,
};
pub use read::{Coords, GeometryCache, MeshRead, TopologyColumns};
pub use sets::{MeshSet, SetMembers, TagColumn, TagValues};
pub use shape::{CellShape, FaceShape, LoopRelation, derive_corners, loop_relation};
pub use validate::{
    FaceNormals, GeomTol, Violation, ViolationCode, connectivity_hash, coordinate_hash, geometry_hash, geometry_hash_quantized, quantize_word,
    topology_hash, validate, validate_geometry, validate_geometry_with,
};
