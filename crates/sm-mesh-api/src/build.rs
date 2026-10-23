//! Builder and edit traits for producers (meshers, importers) and modifiers (smoothers,
//! transforms). Kept apart from the read-only queries so consumers cannot mutate.
//!
//! Topology is immutable once built: topological changes (refine, coarsen, remesh) are IR to IR
//! passes that read one mesh and build another. [`MeshEdit`] covers the non-topological edits.

use sm_core::{EntityId, Point3, Transform};

use crate::classify::Classification;
use crate::error::MeshResult;
use crate::index::{CellIx, Dim, VertexIx};
use crate::query::MeshRead;
use crate::schema::TagDef;
use crate::shape::{CellShape, CellSpec};
use crate::tag::TagRow;

/// Produces a mesh from cells. Faces (and edges) are derived: cells are added with their
/// *outward* face loops and the builder merges shared faces, applying the orientation rule.
///
/// Cells get ascending ids in insertion order, so the first cell to use a face becomes `left`.
/// Face ids are assigned by first appearance in `(cell, local face)` order (canonical).
pub trait MeshBuilder {
    /// The finished, read-only mesh type.
    type Mesh: MeshRead;

    /// Add a vertex (node); indices are dense in insertion order.
    fn add_vertex(&mut self, p: Point3) -> VertexIx;
    /// Add a straight-sided cell of a template shape from its reference-ordered corners.
    fn add_cell(&mut self, shape: CellShape, corners: &[VertexIx]) -> MeshResult<CellIx>;
    /// Add a high-order cell; `nodes` lists all geometry nodes in basis order, corners first.
    fn add_high_order_cell(&mut self, spec: CellSpec, nodes: &[VertexIx]) -> MeshResult<CellIx>;
    /// Add a general polyhedron from its face loops, each oriented *outward* (right-hand rule).
    fn add_polyhedron(&mut self, faces: &[&[VertexIx]]) -> MeshResult<CellIx>;

    /// Classify a vertex (with parametric coordinates where known).
    fn classify_vertex(&mut self, v: VertexIx, c: Classification) -> MeshResult<()>;
    /// Classify a cell onto a volume or partition cell.
    fn classify_cell(&mut self, cell: CellIx, entity: EntityId) -> MeshResult<()>;
    /// Classify the edge between two vertices (resolved at `finish`).
    fn classify_edge(&mut self, a: VertexIx, b: VertexIx, c: Classification) -> MeshResult<()>;
    /// Classify the face with this vertex set (resolved at `finish`).
    fn classify_face(&mut self, vertices: &[VertexIx], c: Classification) -> MeshResult<()>;

    /// Declare a tag column on vertices or cells (edge/face tags go through [`MeshEdit`]).
    fn define_tag(&mut self, def: TagDef) -> MeshResult<()>;
    /// Set a vertex or cell tag row.
    fn set_tag(&mut self, dim: Dim, name: &str, ix: u32, value: TagRow<'_>) -> MeshResult<()>;

    /// Merge faces, apply the orientation rule, derive edges, and return the mesh.
    fn finish(self) -> MeshResult<Self::Mesh>;
}

/// Non-topological edits of a finished mesh (for smoothers, transforms, tagging passes).
pub trait MeshEdit {
    /// Move a vertex.
    fn set_point(&mut self, v: VertexIx, p: Point3) -> MeshResult<()>;
    /// Set or clear a classification, keeping the reverse index consistent.
    fn set_classification(&mut self, dim: Dim, ix: u32, c: Option<Classification>) -> MeshResult<()>;
    /// Declare a tag column on any dimension (zero-filled).
    fn define_tag(&mut self, def: TagDef) -> MeshResult<()>;
    /// Set one tag row.
    fn set_tag(&mut self, dim: Dim, name: &str, ix: u32, value: TagRow<'_>) -> MeshResult<()>;
    /// Define (or replace the members of) a named set. Members are sorted and deduplicated.
    fn define_set(&mut self, name: &str, dim: Dim, members: Vec<u32>) -> MeshResult<()>;
    /// Apply an affine transform. For an improper transform (mirror) every face loop is
    /// reversed and templated cells get their corners permuted, so the orientation rule still
    /// holds on the mapped geometry. Errors with `Unsupported` for high-order or curved
    /// entities under an improper transform.
    fn apply_transform(&mut self, t: &Transform) -> MeshResult<()>;
}
