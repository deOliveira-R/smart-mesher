//! Boundary behaviour: how a face acts on a particle that reaches it.
//!
//! Deck faces (mirror, translation, rotation pairs from `sm_model_api::Deck`) are handled **in
//! the structure**: the tracker applies the reflection or the wrap itself and keeps going, so a
//! reduced symmetric model is traced directly. Response boundaries (`BoundaryKind::Response`,
//! "a tag only") carry consumer semantics and are reported, never interpreted with randomness.

use sm_core::{EntityId, Transform};
use sm_model_api::{BoundaryKind, Deck, DeckSide, FaceBoundary, FaceRole};
use std::collections::BTreeMap;

/// What a response boundary asks the transport code to do.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum ResponseKind {
    /// Kill the particle (leakage).
    Vacuum,
    /// Isotropic (cosine) re-emission into the model; needs the caller's RNG.
    White,
    /// Specular reflection scaled by an albedo in `[0, 1]` (OpenMC `albedo`); caller scales weight.
    Albedo(f64),
    /// Perfect specular reflection, but owned by the consumer (use `FaceAction::Reflect` to have
    /// the tracker do it).
    Reflective,
}

/// What the tracker does when it reaches a face.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum FaceAction {
    /// Ordinary interior face: cross into the neighbouring region.
    Transmit,
    /// Specular reflection about the exact surface normal (mirror deck, or reflective response
    /// delegated to the tracker).
    Reflect,
    /// Map position and direction with `transform` and re-enter through `mate`. For a deck pair
    /// `B = g(A)`: a particle leaving through a face on side A uses `g`, through side B uses
    /// `g^-1` (see [`wrap_transform`]).
    Wrap {
        /// Applied to the hit point and (linear part only) to the direction.
        transform: Transform,
        /// The paired face.
        mate: EntityId,
    },
    /// Report to the consumer, state untouched.
    Respond(ResponseKind),
}

/// The wrap transform for a particle leaving through a deck face on `side` of `deck`:
/// `g` for side A, `g^-1` for side B. `None` for a mirror (self-paired: use [`FaceAction::Reflect`])
/// or a singular deck.
pub fn wrap_transform(deck: &Deck, side: DeckSide) -> Option<Transform> {
    match (deck, side) {
        (Deck::Mirror { .. }, _) | (_, DeckSide::SelfPaired) => None,
        (d, DeckSide::A) => Some(d.transform()),
        (d, DeckSide::B) => d.transform().inverse(),
    }
}

/// Per-face boundary actions for a tracker build. Faces not listed transmit if interior; an
/// unlisted face that has the implicit complement on its far side uses `exterior`.
#[derive(Clone, Debug, PartialEq)]
pub struct BoundarySpec {
    actions: BTreeMap<EntityId, FaceAction>,
    /// Action for unlisted faces that border the implicit complement. Default: vacuum.
    pub exterior: FaceAction,
    /// Response semantics per face *color* or face id for `BoundaryKind::Response` records:
    /// the model says "tag only", this table says what the tag means to the tracker.
    pub response_of: BTreeMap<EntityId, ResponseKind>,
}

impl Default for BoundarySpec {
    fn default() -> Self {
        Self { actions: BTreeMap::new(), exterior: FaceAction::Respond(ResponseKind::Vacuum), response_of: BTreeMap::new() }
    }
}

impl BoundarySpec {
    /// Empty spec: everything interior transmits, the exterior is vacuum.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the action of one face directly.
    pub fn set(&mut self, face: EntityId, action: FaceAction) {
        self.actions.insert(face, action);
    }

    /// Derive the action of `face` from its model records. `kind` is the record's
    /// [`BoundaryKind`]; `fb` the face's row. A response record is looked up in
    /// `response_of` by the face's color (`color`) and then by the face id; no entry means
    /// vacuum. Returns the action stored.
    pub fn add_model_face(&mut self, face: EntityId, fb: &FaceBoundary, kind: &BoundaryKind, color: Option<EntityId>) -> FaceAction {
        let act = match (&fb.role, kind) {
            (FaceRole::Deck { side, mate }, BoundaryKind::Deck(deck)) => match (wrap_transform(deck, *side), mate) {
                (Some(transform), Some(mate)) => FaceAction::Wrap { transform, mate: *mate },
                _ => FaceAction::Reflect,
            },
            (FaceRole::Response, _) => {
                let r = color.and_then(|c| self.response_of.get(&c)).or_else(|| self.response_of.get(&face)).copied();
                match r {
                    Some(ResponseKind::Reflective) => FaceAction::Reflect,
                    Some(r) => FaceAction::Respond(r),
                    None => FaceAction::Respond(ResponseKind::Vacuum),
                }
            }
            _ => FaceAction::Transmit,
        };
        self.actions.insert(face, act);
        act
    }

    /// The action of `face`, or `None` if unlisted (the implementor then decides between
    /// `Transmit` and `exterior` from the face sides).
    pub fn action_of(&self, face: EntityId) -> Option<FaceAction> {
        self.actions.get(&face).copied()
    }

    /// Listed faces in ascending id order (deterministic iteration for hashing).
    pub fn iter(&self) -> impl Iterator<Item = (EntityId, FaceAction)> + '_ {
        self.actions.iter().map(|(k, v)| (*k, *v))
    }
}
