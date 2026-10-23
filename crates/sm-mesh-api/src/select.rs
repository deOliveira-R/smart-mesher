//! Selector algebra over entities (STK-style): compose sets, shapes, classification, tags and
//! boundary status with union / intersection / difference / complement.
//!
//! Evaluation is generic over [`MeshRead`] and always returns sorted, unique indices.

use sm_core::EntityId;

use crate::error::{MeshError, MeshResult};
use crate::index::{CellIx, Dim, FaceIx};
use crate::query::MeshRead;
use crate::shape::CellShape;
use crate::tag::{TagRow, sorted_difference, sorted_intersection, sorted_union};

/// A declarative entity selector. Evaluate with [`Selector::eval`].
#[derive(Clone, Debug, PartialEq)]
pub enum Selector {
    /// Every entity of the evaluated dimension.
    All,
    /// No entity.
    Nothing,
    /// A named set (must have the evaluated dimension).
    Set(String),
    /// Cells with this shape tag (cells only).
    Shape(CellShape),
    /// Entities classified directly on this model/partition entity.
    ClassifiedOn(EntityId),
    /// Faces with no `right` cell (faces only).
    Boundary,
    /// Faces with both sides (faces only).
    Interior,
    /// Entities whose tag (first component, integer) equals the value.
    TagEq(String, i64),
    /// Union of the operands.
    Union(Vec<Selector>),
    /// Intersection of the operands (empty list = all).
    Intersect(Vec<Selector>),
    /// First operand minus the second.
    Difference(Box<Selector>, Box<Selector>),
    /// Complement within the evaluated dimension.
    Complement(Box<Selector>),
}

impl Selector {
    /// Evaluate on entities of dimension `dim`.
    pub fn eval(&self, mesh: &dyn MeshRead, dim: Dim) -> MeshResult<Vec<u32>> {
        let n = mesh.len(dim) as u32;
        Ok(match self {
            Selector::All => (0..n).collect(),
            Selector::Nothing => Vec::new(),
            Selector::Set(name) => {
                let s = mesh.set(name).ok_or_else(|| MeshError::UnknownName(name.clone()))?;
                if s.dim != dim {
                    return Err(MeshError::TagMismatch(name.clone()));
                }
                s.members().to_vec()
            }
            Selector::Shape(shape) => {
                if dim != Dim::Cell {
                    return Err(MeshError::Unsupported("shape selector applies to cells"));
                }
                (0..n).filter(|&c| mesh.cell_shape(CellIx(c)) == *shape).collect()
            }
            Selector::ClassifiedOn(e) => mesh.entities_on(dim, *e).to_vec(),
            Selector::Boundary | Selector::Interior => {
                if dim != Dim::Face {
                    return Err(MeshError::Unsupported("boundary selector applies to faces"));
                }
                let want = matches!(self, Selector::Boundary);
                (0..n).filter(|&f| mesh.is_boundary_face(FaceIx(f)) == want).collect()
            }
            Selector::TagEq(name, value) => {
                let col = mesh.tag(dim, name).ok_or_else(|| MeshError::UnknownName(name.clone()))?;
                (0..n)
                    .filter(|&i| matches!(col.row(i as usize), Some(TagRow::I64(r)) if r.first() == Some(value)))
                    .collect()
            }
            Selector::Union(ops) => {
                let mut acc = Vec::new();
                for o in ops {
                    acc = sorted_union(&acc, &o.eval(mesh, dim)?);
                }
                acc
            }
            Selector::Intersect(ops) => {
                let mut acc: Vec<u32> = (0..n).collect();
                for o in ops {
                    acc = sorted_intersection(&acc, &o.eval(mesh, dim)?);
                }
                acc
            }
            Selector::Difference(a, b) => sorted_difference(&a.eval(mesh, dim)?, &b.eval(mesh, dim)?),
            Selector::Complement(a) => sorted_difference(&(0..n).collect::<Vec<_>>(), &a.eval(mesh, dim)?),
        })
    }
}
