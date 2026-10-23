//! Boundary kinds: what every boundary face of the domain *is* (ir.md §5, §5.1).
//!
//! A kind is attached to a face selection (a [`KindSet`]); kind sets are exclusive and must
//! cover every boundary face of the domain. Deck kinds additionally reference a
//! [`DeckPairing`] that carries the generator and the explicit face-to-face pairs.

use crate::action::{DeckKind, Generator};
use crate::selection::{Face, Selection};
use sm_core::EntityId;

/// Which side of a deck pairing a face set is on.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum DeckSide {
    /// Mirror: the face set is paired with itself.
    SelfPaired,
    /// Source side `A` (`B = g(A)`).
    A,
    /// Image side `B`.
    B,
}

/// Whether an interface is meant to be conformal.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Conformity {
    /// Shared faces after meshing (merge).
    Conformal,
    /// Declared non-conformal by design (sliding, mortar, different refinement).
    NonConformal {
        /// The face group on the other side, if the pairing is already known.
        partner: Option<EntityId>,
    },
}

/// Intent of an interface boundary. Tag only; no physics.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct InterfaceIntent {
    /// The two volume colors the interface separates, if it is a material interface.
    pub between: Option<(EntityId, EntityId)>,
    /// Conformal or declared non-conformal.
    pub conformity: Conformity,
}

/// The kind of a boundary face.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum BoundaryKind {
    /// Deck boundary: reduced-model cut where the missing part is a transformed copy.
    Deck {
        /// The [`DeckPairing`] (a `Boundary` id).
        pairing: EntityId,
        /// Which side of the pairing.
        side: DeckSide,
    },
    /// Response boundary: a tag the consumer resolves (vacuum, albedo, source...). No meaning here.
    Response {
        /// Free-form tag.
        tag: String,
    },
    /// Interface between colors or an explicitly non-conformal pair.
    Interface(InterfaceIntent),
}

/// Query form of a kind, used by selectors.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum KindFilter {
    /// Any deck face, or only deck faces of one kind.
    Deck(Option<DeckKind>),
    /// Any response face, or only those with this tag.
    Response(Option<String>),
    /// Any interface.
    Interface,
}

impl KindFilter {
    /// Does `kind` match this filter? `pairing_kind` resolves a pairing id to its kind.
    pub fn matches(&self, kind: &BoundaryKind, pairing_kind: impl Fn(EntityId) -> Option<DeckKind>) -> bool {
        match (self, kind) {
            (KindFilter::Deck(None), BoundaryKind::Deck { .. }) => true,
            (KindFilter::Deck(Some(k)), BoundaryKind::Deck { pairing, .. }) => pairing_kind(*pairing) == Some(*k),
            (KindFilter::Response(None), BoundaryKind::Response { .. }) => true,
            (KindFilter::Response(Some(t)), BoundaryKind::Response { tag }) => t == tag,
            (KindFilter::Interface, BoundaryKind::Interface(_)) => true,
            _ => false,
        }
    }
}

/// A set of faces sharing one boundary kind (a selection with a role).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct KindSet {
    /// Id (`Boundary` kind).
    pub id: EntityId,
    /// The kind.
    pub kind: BoundaryKind,
    /// The faces.
    pub faces: Selection<Face>,
}

/// A deck pairing: one generator and the explicit face-to-face correspondence.
///
/// For a mirror every pair is `(f, f)`. For translation and rotation `B = g(A)`, and the outward
/// normals satisfy `n_B = -g_lin(n_A)`.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DeckPairing {
    /// Id (`Boundary` kind).
    pub id: EntityId,
    /// The generator (carries the plane, vector or axis).
    pub generator: Generator,
    /// Pairs `(a, b)`, sorted by `a`.
    pub pairs: Vec<(EntityId, EntityId)>,
}

impl DeckPairing {
    /// Kind of the pairing.
    pub fn kind(&self) -> DeckKind {
        self.generator.kind()
    }
}
