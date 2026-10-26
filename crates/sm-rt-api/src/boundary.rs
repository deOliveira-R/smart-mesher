//! Boundary conditions: what a particle does when it reaches a boundary face.
//!
//! Two kinds of boundary, one rule (binding):
//!
//! * **Deck boundaries** (`sm_model_api::Deck`: mirror, translation, rotation) are *geometry*.
//!   The tracker applies them **inside** [`crate::Tracker::step`]: a mirror becomes
//!   [`FaceAction::Reflect`], a translation or rotation pair becomes [`FaceAction::Wrap`]. The
//!   step still reports the event ([`crate::StepEnd::Boundary`]) with the post-state already
//!   reflected or moved to the mate face, so tallies can count surface crossings.
//! * **Response boundaries** (`sm_model_api::BoundaryKind::Response`) are a *tag only* in the
//!   model: vacuum, white, albedo and reflective belong to the transport code. They become
//!   [`FaceAction::Respond`]; the step leaves the state **on the face** and the caller resolves
//!   it with [`crate::Tracker::resolve_boundary`], supplying its own random numbers. The tracker
//!   never draws a random number.
//!
//! So [`ResponseKind::Reflective`] (a caller-resolved response, exact specular reflection on
//! planar faces) and [`FaceAction::Reflect`] (a mirror deck applied in `step`) are different
//! things on purpose: one is a physics choice, the other is a symmetry of the model. No face
//! has both.
//!
//! [`BoundarySpec`] is the build-time form, derived from the model with
//! [`BoundarySpec::add_model_face`]; [`FaceAction`] is the resolved per-face form the tracker
//! carries.

use crate::cache::{ContentHash, ContentHasher};
use crate::error::BuildIssue;
use crate::ids::FaceId;
use sm_core::{EntityId, Transform};
use sm_model_api::{BoundaryKind, Deck, DeckSide, FaceBoundary, FaceRole};
use std::collections::BTreeMap;

/// What a transport code decides at a domain boundary (the model only tags such faces).
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum ResponseKind {
    /// The particle leaves the problem.
    Vacuum,
    /// Re-emitted with a cosine-law direction about the inward normal (two caller uniforms).
    White,
    /// Survives with probability `a` in `[0, 1]` (caller uniform `u[0] < a`) and is then
    /// reflected specularly; otherwise terminated. A code that prefers weight scaling treats the
    /// face as [`ResponseKind::Reflective`] and multiplies its weight by `a` itself.
    Albedo(f64),
    /// Exact specular reflection `d - 2 (d.n) n` (exact on planar faces).
    Reflective,
}

impl ResponseKind {
    /// True unless the kind is `Albedo(a)` with `a` outside `[0, 1]` or non-finite.
    pub fn is_valid(&self) -> bool {
        match *self {
            Self::Albedo(a) => a.is_finite() && (0.0..=1.0).contains(&a),
            _ => true,
        }
    }

    /// Stable wire code (for hashing and the C ABI): Vacuum 1, White 2, Albedo 3, Reflective 4.
    pub fn code(&self) -> u32 {
        match self {
            Self::Vacuum => 1,
            Self::White => 2,
            Self::Albedo(_) => 3,
            Self::Reflective => 4,
        }
    }

    pub(crate) fn hash_into(&self, h: &mut ContentHasher) {
        h.write_u64(u64::from(self.code()));
        if let Self::Albedo(a) = *self {
            h.write_f64(a);
        }
    }
}

/// What happens at a face, resolved for tracking.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum FaceAction {
    /// Interior face: the particle goes through (cell, region or slot change).
    Transmit,
    /// Mirror deck, applied inside `step`: direction reflected about the face plane.
    Reflect,
    /// Translation or rotation deck, applied inside `step`: the particle re-enters through
    /// `mate` after `transform` (a rigid map of position; its linear part maps the direction).
    Wrap {
        /// Map from this face's frame to the mate's: position `transform(p)`, direction
        /// `linear(transform) * d`. `g` for side A of the deck, `g^-1` for side B.
        transform: Transform,
        /// The face the particle re-emerges from.
        mate: FaceId,
    },
    /// Response boundary: the state stays on the face; call `resolve_boundary`.
    Respond(ResponseKind),
}

impl FaceAction {
    /// True if `step` already applied the action (`Reflect`, `Wrap`); false if the caller must
    /// resolve it (`Respond`) or nothing happens (`Transmit`).
    pub fn applied_in_step(&self) -> bool {
        matches!(self, Self::Reflect | Self::Wrap { .. })
    }

    /// Absorb this action into a hasher (for implementors' structure and cache-key hashes).
    pub fn hash_into(&self, h: &mut ContentHasher) {
        match self {
            Self::Transmit => h.write_u64(0),
            Self::Reflect => h.write_u64(1),
            Self::Wrap { transform, mate } => {
                h.write_u64(2);
                hash_transform(h, transform);
                h.write_u64(u64::from(mate.0));
            }
            Self::Respond(r) => {
                h.write_u64(3);
                r.hash_into(h);
            }
        }
    }
}

/// Absorb a transform into a hasher.
pub fn hash_transform(h: &mut ContentHasher, t: &Transform) {
    for row in &t.m {
        for &v in row {
            h.write_f64(v);
        }
    }
    h.write_f64(t.t.x);
    h.write_f64(t.t.y);
    h.write_f64(t.t.z);
}

/// The transform a deck applies to a particle leaving through a face on `side`: `g` (the deck's
/// A-to-B map) for side A, `g^-1` for side B. `None` for a mirror (it is [`FaceAction::Reflect`],
/// not a wrap), for a self-paired face, and for a singular transform.
pub fn wrap_transform(deck: &Deck, side: DeckSide) -> Option<Transform> {
    if deck.is_self_paired() {
        return None;
    }
    match side {
        DeckSide::A => Some(deck.transform()),
        DeckSide::B => deck.transform().inverse(),
        DeckSide::SelfPaired => None,
    }
}

/// Selects model surfaces for a boundary condition.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum BoundarySelector {
    /// A B-rep face (the mesh boundary faces classified on it).
    Entity(EntityId),
    /// A named face set of the mesh (a sideset).
    FaceSet(String),
}

impl BoundarySelector {
    pub(crate) fn hash_into(&self, h: &mut ContentHasher) {
        match self {
            Self::Entity(e) => {
                h.write_u64(1);
                h.write_u64(e.to_raw());
            }
            Self::FaceSet(s) => {
                h.write_u64(2);
                h.write_u64(s.len() as u64);
                h.write_bytes(s.as_bytes());
            }
        }
    }
}

/// Build-time form of a boundary condition (mates are model entities, not yet tracker faces).
/// [`BoundaryDef::resolve`] turns it into a [`FaceAction`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum BoundaryDef {
    /// Mirror deck: [`FaceAction::Reflect`].
    Reflect,
    /// Response boundary: [`FaceAction::Respond`].
    Respond(ResponseKind),
    /// Translation or rotation deck: [`FaceAction::Wrap`] to the face `mate` of the model.
    Wrap {
        /// Map from this face to the mate (see [`wrap_transform`]).
        transform: Transform,
        /// Model face the particle re-emerges from.
        mate: EntityId,
    },
}

impl BoundaryDef {
    /// Resolve to the tracker form. `mate_face` maps the model mate face to the tracker's
    /// `FaceId`; `None` if the mate is unknown.
    pub fn resolve(&self, mate_face: impl FnOnce(EntityId) -> Option<FaceId>) -> Option<FaceAction> {
        Some(match *self {
            Self::Reflect => FaceAction::Reflect,
            Self::Respond(r) => FaceAction::Respond(r),
            Self::Wrap { transform, mate } => FaceAction::Wrap { transform, mate: mate_face(mate)? },
        })
    }

    pub(crate) fn hash_into(&self, h: &mut ContentHasher) {
        match self {
            Self::Reflect => h.write_u64(1),
            Self::Respond(r) => {
                h.write_u64(2);
                r.hash_into(h);
            }
            Self::Wrap { transform, mate } => {
                h.write_u64(3);
                hash_transform(h, transform);
                h.write_u64(mate.to_raw());
            }
        }
    }
}

/// Response of each response-boundary record (a `sm_model_api` `EntityKind::Boundary` id): the
/// model tags such faces but leaves their meaning to the consumer, so the deck supplies it here.
pub type ResponseMap = BTreeMap<EntityId, ResponseKind>;

/// Boundary conditions from the deck. Deterministic: rules are an ordered `Vec`, the first
/// matching selector wins.
#[derive(Clone, Debug, PartialEq)]
pub struct BoundarySpec {
    /// Applied to boundary faces no selector claims.
    pub default: BoundaryDef,
    /// Explicit conditions; the first matching selector wins.
    pub rules: Vec<(BoundarySelector, BoundaryDef)>,
}

impl Default for BoundarySpec {
    fn default() -> Self {
        Self { default: BoundaryDef::Respond(ResponseKind::Vacuum), rules: Vec::new() }
    }
}

impl BoundarySpec {
    /// Add the condition of one model face, derived from the model's boundary tables
    /// (`face_boundary` row `fb` plus the `BoundaryKind` of its record `fb.boundary`).
    ///
    /// * deck mirror (self-paired) -> [`BoundaryDef::Reflect`];
    /// * deck translation or rotation, side A or B, with a mate -> [`BoundaryDef::Wrap`] using
    ///   [`wrap_transform`] (`g` for A, `g^-1` for B);
    /// * response -> [`BoundaryDef::Respond`] with the kind from `responses` (the record id is
    ///   the key); a missing entry is an issue, not a silent vacuum;
    /// * interface -> nothing is added (interior face, [`FaceAction::Transmit`]).
    ///
    /// A role that contradicts its deck (a mirror with a mate, a translation self-paired,
    /// a missing mate) is a `deck.mismatch` issue.
    pub fn add_model_face(&mut self, face: EntityId, fb: &FaceBoundary, kind: &BoundaryKind, responses: &ResponseMap) -> Result<(), BuildIssue> {
        let def = match (kind, fb.role) {
            (BoundaryKind::Interface(_), _) => return Ok(()),
            (BoundaryKind::Response, FaceRole::Response) => {
                let r = responses
                    .get(&fb.boundary)
                    .copied()
                    .ok_or_else(|| BuildIssue::new("boundary.unmapped_response", "response boundary has no ResponseKind in the response map").with_entity(face).suggest("add the record to the ResponseMap (vacuum, white, albedo or reflective)"))?;
                if !r.is_valid() {
                    return Err(BuildIssue::new("boundary.bad_albedo", "albedo must be in [0, 1]").with_entity(face));
                }
                BoundaryDef::Respond(r)
            }
            (BoundaryKind::Deck(deck), FaceRole::Deck { side, mate }) => {
                if deck.is_self_paired() {
                    if side != DeckSide::SelfPaired || mate.is_some() {
                        return Err(mismatch(face, "a mirror face must be self-paired without a mate"));
                    }
                    BoundaryDef::Reflect
                } else {
                    let Some(mate) = mate else { return Err(mismatch(face, "a translation or rotation face needs a mate")) };
                    let Some(transform) = wrap_transform(deck, side) else { return Err(mismatch(face, "deck face has no usable side or the deck transform is singular")) };
                    BoundaryDef::Wrap { transform, mate }
                }
            }
            _ => return Err(mismatch(face, "face role does not match its boundary record kind")),
        };
        self.rules.push((BoundarySelector::Entity(face), def));
        Ok(())
    }

    /// Content hash (cache key `boundary_hash`). Order of rules matters and is hashed.
    pub fn content_hash(&self) -> ContentHash {
        let mut h = ContentHasher::new();
        self.default.hash_into(&mut h);
        h.write_u64(self.rules.len() as u64);
        for (s, d) in &self.rules {
            s.hash_into(&mut h);
            d.hash_into(&mut h);
        }
        h.finish()
    }
}

fn mismatch(face: EntityId, msg: &'static str) -> BuildIssue {
    BuildIssue::new("deck.mismatch", msg).with_entity(face)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_core::{EntityKind, Point3, Vector3};
    use sm_model_api::{Axis, Plane, Turns};

    fn e(kind: EntityKind, n: u64) -> EntityId {
        EntityId::new(kind, n).unwrap()
    }

    #[test]
    fn wrap_transform_is_g_and_g_inverse_and_none_for_mirror() {
        let d = Deck::Translation { vector: Vector3::new(10.0, 0.0, 0.0) };
        let a = wrap_transform(&d, DeckSide::A).unwrap();
        let b = wrap_transform(&d, DeckSide::B).unwrap();
        let p = Point3::new(1.0, 2.0, 3.0);
        assert_eq!(a.apply_point(p), Point3::new(11.0, 2.0, 3.0));
        let back = b.apply_point(a.apply_point(p));
        assert!((back - p).norm() < 1e-15);
        let r = Deck::rotation(Axis { point: Point3::new(0.0, 0.0, 0.0), dir: Vector3::new(0.0, 0.0, 1.0) }, Turns { num: 1, den: 4 });
        let (ra, rb) = (wrap_transform(&r, DeckSide::A).unwrap(), wrap_transform(&r, DeckSide::B).unwrap());
        let q = Point3::new(1.0, 0.0, 0.0);
        assert!((ra.apply_point(q) - Point3::new(0.0, 1.0, 0.0)).norm() < 1e-12);
        assert!((rb.apply_point(ra.apply_point(q)) - q).norm() < 1e-12);
        let m = Deck::Mirror { plane: Plane { normal: Vector3::new(1.0, 0.0, 0.0), offset: 0.0 } };
        assert!(wrap_transform(&m, DeckSide::A).is_none() && wrap_transform(&m, DeckSide::SelfPaired).is_none());
    }

    #[test]
    fn model_faces_derive_actions() {
        let bnd = e(EntityKind::Boundary, 1);
        let f = |n| e(EntityKind::TopoFace, n);
        let mut spec = BoundarySpec::default();
        let mut resp = ResponseMap::new();
        resp.insert(bnd, ResponseKind::Albedo(0.5));
        // response
        let fb = FaceBoundary { boundary: bnd, role: FaceRole::Response };
        spec.add_model_face(f(1), &fb, &BoundaryKind::Response, &resp).unwrap();
        assert_eq!(spec.rules[0].1, BoundaryDef::Respond(ResponseKind::Albedo(0.5)));
        // missing response is an issue
        let err = spec.add_model_face(f(2), &FaceBoundary { boundary: e(EntityKind::Boundary, 9), role: FaceRole::Response }, &BoundaryKind::Response, &resp).unwrap_err();
        assert_eq!(err.code, "boundary.unmapped_response");
        // mirror
        let mirror = Deck::Mirror { plane: Plane { normal: Vector3::new(1.0, 0.0, 0.0), offset: 0.0 } };
        let fb = FaceBoundary { boundary: bnd, role: FaceRole::Deck { side: DeckSide::SelfPaired, mate: None } };
        spec.add_model_face(f(3), &fb, &BoundaryKind::Deck(mirror), &resp).unwrap();
        assert_eq!(spec.rules[1].1, BoundaryDef::Reflect);
        // translation A and B
        let tr = Deck::Translation { vector: Vector3::new(2.0, 0.0, 0.0) };
        for (n, side, mate) in [(4, DeckSide::A, 5), (5, DeckSide::B, 4)] {
            let fb = FaceBoundary { boundary: bnd, role: FaceRole::Deck { side, mate: Some(f(mate)) } };
            spec.add_model_face(f(n), &fb, &BoundaryKind::Deck(tr), &resp).unwrap();
        }
        let BoundaryDef::Wrap { transform: ta, .. } = spec.rules[2].1 else { panic!() };
        let BoundaryDef::Wrap { transform: tb, mate } = spec.rules[3].1 else { panic!() };
        assert_eq!(ta.apply_point(Point3::new(0.0, 0.0, 0.0)).x, 2.0);
        assert_eq!(tb.apply_point(Point3::new(0.0, 0.0, 0.0)).x, -2.0);
        assert_eq!(mate, f(4));
        assert_eq!(spec.rules[3].1.resolve(|m| (m == f(4)).then_some(FaceId(7))), Some(FaceAction::Wrap { transform: tb, mate: FaceId(7) }));
        // contradictions
        let bad = FaceBoundary { boundary: bnd, role: FaceRole::Deck { side: DeckSide::SelfPaired, mate: None } };
        assert_eq!(spec.add_model_face(f(8), &bad, &BoundaryKind::Deck(tr), &resp).unwrap_err().code, "deck.mismatch");
        assert_eq!(spec.add_model_face(f(8), &bad, &BoundaryKind::Response, &resp).unwrap_err().code, "deck.mismatch");
    }

    #[test]
    fn hash_is_order_and_value_sensitive() {
        let mut a = BoundarySpec::default();
        let mut b = BoundarySpec::default();
        a.rules.push((BoundarySelector::FaceSet("x".into()), BoundaryDef::Respond(ResponseKind::Albedo(0.5))));
        b.rules.push((BoundarySelector::FaceSet("x".into()), BoundaryDef::Respond(ResponseKind::Albedo(0.5))));
        assert_eq!(a.content_hash(), b.content_hash());
        b.rules[0].1 = BoundaryDef::Respond(ResponseKind::Albedo(0.6));
        assert_ne!(a.content_hash(), b.content_hash());
        assert!(!ResponseKind::Albedo(1.5).is_valid() && ResponseKind::White.is_valid());
    }
}
