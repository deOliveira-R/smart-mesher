//! High-order geometry: a sparse per-cell `(shape, degree, basis)` table plus extra nodes.
//!
//! The coordinate arrays hold the `vertex_count` vertices first, then any extra (non-vertex)
//! nodes. A high-order cell lists its nodes as indices into that combined array, in the
//! reference-element order of `(shape, basis)`. Cells absent from the table are linear.

use crate::index::VertexIx;
use crate::shape::CellShape;

/// Polynomial basis family of a high-order cell's geometry.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum Basis {
    /// Lagrange with equispaced nodes.
    LagrangeEquispaced,
    /// Lagrange with Gauss-Lobatto-Legendre nodes.
    LagrangeGll,
    /// Bernstein-Bezier control points.
    Bernstein,
}

/// Geometric order of one cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct CellGeometry {
    /// Shape tag.
    pub shape: CellShape,
    /// Polynomial degree (1 = linear).
    pub degree: u8,
    /// Basis family (meaningful when `degree > 1`).
    pub basis: Basis,
}

/// Sparse high-order record of one cell.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HighOrderCell {
    /// Polynomial degree (`>= 2`).
    pub degree: u8,
    /// Basis family.
    pub basis: Basis,
    /// Node indices into the combined coordinate array, reference-element order.
    pub nodes: Vec<u32>,
}

/// A node reference used while building, before final point numbering exists.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NodeRef {
    /// An ordinary mesh vertex.
    Vertex(VertexIx),
    /// The n-th extra node added to the builder.
    Extra(u32),
}
