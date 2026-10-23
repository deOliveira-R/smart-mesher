//! Boundary-kind tables: every boundary face is a deck boundary, a response boundary or an
//! interface (docs/design/ir.md §5.1).
//!
//! Two tables: `boundaries` (one row per boundary record: name + [`BoundaryKind`]) and
//! `face_boundary` (one row per face: which record it belongs to, which side of a pairing, and
//! its mate). The face-keyed table is the single source of truth for membership and pairing,
//! so the two directions cannot disagree.

use crate::action::{Axis, Plane, reflection, rotation, translation};
use crate::color::expect_kind;
use crate::error::{ModelError, Result};
use crate::model::Model;
use sm_core::{EntityId, EntityKind, Transform, Vector3};

/// A deck transformation. The parameters are the source of truth; the matrix is derived
/// ([`Deck::transform`]), so kind and transform cannot be inconsistent.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Deck {
    /// Reflection in a plane (improper). Self-paired: the faces lie on the plane.
    Mirror {
        /// The mirror plane.
        plane: Plane,
    },
    /// Pair `A <-> B = T(A)` with `T` a pure translation.
    Translation {
        /// Shift taking A to B.
        shift: Vector3,
    },
    /// Pair `A <-> B = R(A)`, `R` a rotation by `2π/order` about `axis`.
    Rotation {
        /// Rotation axis (direction gives the sense).
        axis: Axis,
        /// `n` in `2π/n`; at least 2.
        order: u32,
    },
}

impl Deck {
    /// The transform taking side A to side B (for a mirror: the reflection itself).
    pub fn transform(&self) -> Transform {
        match self {
            Deck::Mirror { plane } => reflection(plane),
            Deck::Translation { shift } => translation(*shift),
            Deck::Rotation { axis, order } => rotation(axis, core::f64::consts::TAU / f64::from(*order)),
        }
    }

    /// True for a mirror (self-paired boundary).
    pub fn is_self_paired(&self) -> bool {
        matches!(self, Deck::Mirror { .. })
    }

    /// Order of the generated cyclic group; `None` = infinite (translation).
    pub fn order(&self) -> Option<u32> {
        match self {
            Deck::Mirror { .. } => Some(2),
            Deck::Translation { .. } => None,
            Deck::Rotation { order, .. } => Some(*order),
        }
    }

    pub(crate) fn check(&self) -> Result<()> {
        match self {
            Deck::Rotation { order, .. } if *order < 2 => Err(ModelError::Invalid("rotation order must be at least 2".into())),
            Deck::Translation { shift } if shift.norm() == 0.0 => Err(ModelError::Invalid("zero translation".into())),
            _ => Ok(()),
        }
    }
}

/// Intent of an interface boundary.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum InterfaceIntent {
    /// Between volumes of different colors, a shared (merged) face; conformal by construction.
    Conformal,
    /// Explicitly non-conformal: side A and side B are separate face sets facing each other
    /// (sliding, mortar, different refinement), optionally related by a transform B = T(A).
    NonConformal {
        /// Relative motion/placement of B with respect to A, if any.
        transform: Option<Transform>,
    },
}

/// What a boundary record means for meshing and tracking.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum BoundaryKind {
    /// Deck transformation boundary (mirror / translation / rotation).
    Deck(Deck),
    /// Response boundary: a tag only (vacuum, reflective, source, ... are the consumer's business;
    /// the face color carries the label).
    Response,
    /// Interface between colors or an explicit non-conformal intent.
    Interface(InterfaceIntent),
}

impl BoundaryKind {
    /// The deck, if this is a deck boundary.
    pub fn as_deck(&self) -> Option<&Deck> {
        match self {
            BoundaryKind::Deck(d) => Some(d),
            _ => None,
        }
    }

    /// True if faces of this record come in A/B pairs (translation, rotation, non-conformal).
    pub fn is_paired(&self) -> bool {
        match self {
            BoundaryKind::Deck(d) => !d.is_self_paired(),
            BoundaryKind::Interface(InterfaceIntent::NonConformal { .. }) => true,
            _ => false,
        }
    }
}

/// Row of the boundary table (key: `EntityKind::Boundary` id).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BoundaryRow {
    /// Unique name.
    pub name: String,
    /// The kind.
    pub kind: BoundaryKind,
}

/// Which side of a pairing a face is on.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Side {
    /// The source side (the only side of mirrors, response boundaries, conformal interfaces).
    A,
    /// The image side `B = T(A)`.
    B,
}

/// Row of the face-boundary table (key: `EntityKind::TopoFace` id).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FaceBoundary {
    /// The boundary record this face belongs to.
    pub boundary: EntityId,
    /// Side within the record.
    pub side: Side,
    /// The face this one is paired with (`A <-> B`); `None` for unpaired faces.
    pub mate: Option<EntityId>,
}

impl Model {
    /// Add a boundary record (no faces yet).
    pub fn add_boundary(&mut self, name: &str, kind: BoundaryKind) -> Result<EntityId> {
        if self.boundaries.values().any(|b| b.name == name) {
            return Err(ModelError::Invalid(format!("duplicate boundary name {name:?}")));
        }
        if let BoundaryKind::Deck(d) = &kind {
            d.check()?;
        }
        let id = self.ids.alloc(EntityKind::Boundary)?;
        self.boundaries.insert(id, BoundaryRow { name: name.to_owned(), kind });
        Ok(id)
    }

    /// Put unpaired faces on side A of `boundary`. A face may belong to one record only.
    pub fn assign_faces(&mut self, boundary: EntityId, faces: &[EntityId]) -> Result<()> {
        self.boundary_row(boundary)?;
        for f in faces {
            self.check_unassigned(*f)?;
        }
        for f in faces {
            self.face_boundary.insert(*f, FaceBoundary { boundary, side: Side::A, mate: None });
        }
        Ok(())
    }

    /// Put face pairs `(a, b)` on `boundary`: `a` on side A, `b` on side B, mates of each other.
    /// The record must be a paired kind.
    pub fn assign_pairs(&mut self, boundary: EntityId, pairs: &[(EntityId, EntityId)]) -> Result<()> {
        let row = self.boundary_row(boundary)?;
        if !row.kind.is_paired() {
            return Err(ModelError::Invalid(format!("{boundary} is not a paired boundary kind")));
        }
        for (a, b) in pairs {
            self.check_unassigned(*a)?;
            self.check_unassigned(*b)?;
            if a == b {
                return Err(ModelError::Invalid("a face cannot be paired with itself".into()));
            }
        }
        for (a, b) in pairs {
            self.face_boundary.insert(*a, FaceBoundary { boundary, side: Side::A, mate: Some(*b) });
            self.face_boundary.insert(*b, FaceBoundary { boundary, side: Side::B, mate: Some(*a) });
        }
        Ok(())
    }

    fn boundary_row(&self, boundary: EntityId) -> Result<&BoundaryRow> {
        expect_kind(boundary, EntityKind::Boundary)?;
        self.boundaries.get(&boundary).ok_or(ModelError::Unknown(boundary))
    }

    fn check_unassigned(&self, face: EntityId) -> Result<()> {
        expect_kind(face, EntityKind::TopoFace)?;
        if self.face_boundary.contains_key(&face) { Err(ModelError::Duplicate(face)) } else { Ok(()) }
    }

    /// Faces of `boundary` on `side`, ascending.
    pub fn boundary_faces(&self, boundary: EntityId, side: Side) -> Vec<EntityId> {
        self.face_boundary.iter().filter(|(_, fb)| fb.boundary == boundary && fb.side == side).map(|(f, _)| *f).collect()
    }

    /// `(a, b)` pairs of `boundary`, ascending by `a`. Self-paired kinds give `(a, a)`.
    pub fn boundary_pairs(&self, boundary: EntityId) -> Vec<(EntityId, EntityId)> {
        self.face_boundary
            .iter()
            .filter(|(_, fb)| fb.boundary == boundary && fb.side == Side::A)
            .map(|(f, fb)| (*f, fb.mate.unwrap_or(*f)))
            .collect()
    }

    /// The kind of the boundary `face` belongs to, if any.
    pub fn face_kind(&self, face: EntityId) -> Option<&BoundaryKind> {
        let fb = self.face_boundary.get(&face)?;
        self.boundaries.get(&fb.boundary).map(|b| &b.kind)
    }

    /// Transform taking `face`'s side to its mate's side (inverse for side B faces).
    /// `None` if the face is not on a deck boundary.
    pub fn face_deck_transform(&self, face: EntityId) -> Option<Result<Transform>> {
        let fb = self.face_boundary.get(&face)?;
        let deck = self.boundaries.get(&fb.boundary)?.kind.as_deck()?;
        let t = deck.transform();
        Some(match fb.side {
            Side::A => Ok(t),
            Side::B => crate::action::inverse(&t),
        })
    }
}
