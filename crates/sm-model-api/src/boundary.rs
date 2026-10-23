//! Boundary kinds. Every boundary face is exactly one of:
//!
//! - **Deck**: part of a boundary identification under a deck generator `g`. Mirror: the face set is
//!   self-paired (`g` fixes the plane pointwise). Translation/rotation: face set `A` is paired with
//!   `B = g(A)`. Deck boundaries are *topological* information; they say nothing about physics.
//! - **Response**: a tag only. The consumer decides what the boundary does (vacuum, reflective,
//!   albedo...) by resolving the face color.
//! - **Interface**: between colors (conformal, shared face) or an explicit non-conformal intent
//!   (paired without sharing; optional relative transform).
//!
//! Sign convention for deck pairs `B = g(A)`: `g` maps the **outward** normal of an `A` face to the
//! **negative** of the outward normal of its `B` partner (the two faces look at each other across the
//! identification). For a mirror, `g(n) = -n` on the plane itself.

use crate::error::{ModelError, Result};
use crate::ids::IdAllocator;
use sm_core::{EntityId, EntityKind, Transform};
use std::collections::{BTreeMap, BTreeSet};

/// Which side of a deck identification a face is on.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum DeckSide {
    /// Mirror: the face is its own partner.
    SelfPaired,
    /// Source side `A`.
    A,
    /// Image side `B = g(A)`.
    B,
}

/// The face sets of one deck boundary.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum DeckSides {
    /// Mirror (or any involution): faces fixed by the generator.
    SelfPaired(BTreeSet<EntityId>),
    /// Translation / rotation pairing: `b = g(a)`.
    Paired {
        /// Source faces.
        a: BTreeSet<EntityId>,
        /// Image faces.
        b: BTreeSet<EntityId>,
    },
}

/// One deck boundary record: a generator and the faces it identifies.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DeckBoundary {
    /// `EntityKind::Boundary` id of this record.
    pub id: EntityId,
    /// Generator (a `Boundary` id owned by [`crate::symmetry::Symmetry`]).
    pub generator: EntityId,
    /// The identified faces.
    pub sides: DeckSides,
}

/// Intent of an interface boundary.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum InterfaceIntent {
    /// An interior face between two volumes (of different colors); conformal, shared.
    BetweenColors,
    /// Faces paired *without* sharing (sliding, mortar, different refinement).
    NonConformal {
        /// Partner faces (symmetric: the partner side should list this face).
        partner: BTreeSet<EntityId>,
        /// Relative transform applied to this side to land on the partner, if any (e.g. sliding).
        transform: Option<Transform>,
    },
}

/// The kind of one face.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum BoundaryKind {
    /// Deck boundary `boundary` (see [`DeckBoundary`]) and this face's side of it.
    Deck {
        /// The deck record.
        boundary: EntityId,
        /// Side of the identification.
        side: DeckSide,
    },
    /// Response boundary: tag only; the face color carries the identity.
    Response,
    /// Interface.
    Interface(InterfaceIntent),
}

/// Per-face kind table plus the deck records. Kept consistent by its mutators.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BoundaryTable {
    kinds: BTreeMap<EntityId, BoundaryKind>,
    decks: BTreeMap<EntityId, DeckBoundary>,
}

fn expect_face(f: EntityId) -> Result<()> {
    if f.kind() == EntityKind::TopoFace { Ok(()) } else { Err(ModelError::WrongKind { id: f, found: f.kind(), expected: "TopoFace" }) }
}

impl BoundaryTable {
    /// An empty table.
    pub fn new() -> Self {
        Self::default()
    }

    /// Kind of a face, if tagged.
    pub fn kind_of(&self, face: EntityId) -> Option<&BoundaryKind> {
        self.kinds.get(&face)
    }

    /// All tagged faces with their kinds, in face order.
    pub fn iter(&self) -> impl Iterator<Item = (EntityId, &BoundaryKind)> {
        self.kinds.iter().map(|(&f, k)| (f, k))
    }

    /// Deck record by id.
    pub fn deck(&self, id: EntityId) -> Option<&DeckBoundary> {
        self.decks.get(&id)
    }

    /// All deck records in id order.
    pub fn decks(&self) -> impl Iterator<Item = &DeckBoundary> {
        self.decks.values()
    }

    /// Tag faces as response boundaries (does not overwrite deck tags; returns an error for those).
    pub fn tag_response(&mut self, faces: &[EntityId]) -> Result<()> {
        for &f in faces {
            expect_face(f)?;
            if matches!(self.kinds.get(&f), Some(BoundaryKind::Deck { .. })) {
                return Err(ModelError::Invalid(format!("face {f} is already a deck boundary")));
            }
            self.kinds.insert(f, BoundaryKind::Response);
        }
        Ok(())
    }

    /// Tag a face as an interface.
    pub fn tag_interface(&mut self, face: EntityId, intent: InterfaceIntent) -> Result<()> {
        expect_face(face)?;
        if matches!(self.kinds.get(&face), Some(BoundaryKind::Deck { .. })) {
            return Err(ModelError::Invalid(format!("face {face} is already a deck boundary")));
        }
        self.kinds.insert(face, BoundaryKind::Interface(intent));
        Ok(())
    }

    /// Declare a deck boundary for `generator` over `sides`. Allocates its `Boundary` id and tags the
    /// faces. A face can belong to one deck boundary only.
    pub fn add_deck(&mut self, ids: &mut dyn IdAllocator, generator: EntityId, sides: DeckSides) -> Result<EntityId> {
        if generator.kind() != EntityKind::Boundary {
            return Err(ModelError::WrongKind { id: generator, found: generator.kind(), expected: "Boundary (generator)" });
        }
        let tagged: Vec<(EntityId, DeckSide)> = match &sides {
            DeckSides::SelfPaired(s) => s.iter().map(|&f| (f, DeckSide::SelfPaired)).collect(),
            DeckSides::Paired { a, b } => a.iter().map(|&f| (f, DeckSide::A)).chain(b.iter().map(|&f| (f, DeckSide::B))).collect(),
        };
        for &(f, _) in &tagged {
            expect_face(f)?;
            if matches!(self.kinds.get(&f), Some(BoundaryKind::Deck { .. })) {
                return Err(ModelError::Invalid(format!("face {f} already belongs to a deck boundary")));
            }
        }
        let id = ids.alloc(EntityKind::Boundary);
        for (f, side) in tagged {
            self.kinds.insert(f, BoundaryKind::Deck { boundary: id, side });
        }
        self.decks.insert(id, DeckBoundary { id, generator, sides });
        Ok(id)
    }

    /// Topology evolution: `new` takes over `old`'s kind (and deck side).
    pub fn inherit(&mut self, old: EntityId, new: EntityId) {
        if old == new {
            return;
        }
        let Some(k) = self.kinds.get(&old).cloned() else { return };
        if let BoundaryKind::Deck { boundary, side } = &k
            && let Some(d) = self.decks.get_mut(boundary) {
                match (&mut d.sides, side) {
                    (DeckSides::SelfPaired(s), _) => {
                        s.insert(new);
                    }
                    (DeckSides::Paired { a, .. }, DeckSide::A) => {
                        a.insert(new);
                    }
                    (DeckSides::Paired { b, .. }, _) => {
                        b.insert(new);
                    }
                }
            }
        self.kinds.entry(new).or_insert(k);
    }

    /// Remove a retired face from the table and from any deck record.
    pub fn retire(&mut self, face: EntityId) {
        self.kinds.remove(&face);
        for d in self.decks.values_mut() {
            match &mut d.sides {
                DeckSides::SelfPaired(s) => {
                    s.remove(&face);
                }
                DeckSides::Paired { a, b } => {
                    a.remove(&face);
                    b.remove(&face);
                }
            }
        }
    }
}
