//! Boundary kinds: every boundary face is a deck boundary, a response boundary or an interface
//! (docs/design/ir.md §5.1).
//!
//! The kind is a property of each **face**, not of a closed group: a unit cell with mirrors at
//! `x = 0` and `x = 1` is valid even though the two mirrors do not generate a finite group.
//! Two tables: `boundaries` (one row per record: name + [`BoundaryKind`]) and `face_boundary`
//! (one row per face: record, role, mate). The face-keyed table is the single source of truth
//! for membership and pairing, so the two directions cannot disagree.
//!
//! Sign rule for deck pairs (validated, `IssueCode::DeckOrientation`): with `B = g(A)` as
//! regions, the outward normals satisfy `n_B = -g(n_A)`; for a mirror `g(n) = -n` on the plane,
//! so the face is its own mate with the same outward normal.

use crate::error::{ModelError, Result, expect_kind};
use crate::model::Model;
use crate::xf::{Axis, GROUP_TOL, Plane, Turns, transforms_close};
use sm_core::{EntityId, EntityKind, Transform, Vector3};
use sm_topo_api::{Dim, FaceClass, Region, TopologyView, TopologyViewExt};

/// A deck transformation. The parameters are the source of truth; the matrix is derived, so kind
/// and transform cannot disagree.
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
        vector: Vector3,
    },
    /// Pair `A <-> B = R(A)`, `R` a rotation by `angle` (a rational number of turns) about `axis`.
    Rotation {
        /// Rotation axis (direction gives the sense).
        axis: Axis,
        /// Rotation angle; the group order is its denominator.
        angle: Turns,
    },
}

impl Deck {
    /// The transform taking side A to side B (for a mirror: the reflection itself). Decks that
    /// passed [`Deck::check`] (all decks stored in a [`Model`]) never fall back; a degenerate
    /// deck yields the identity.
    pub fn transform(&self) -> Transform {
        match self {
            Deck::Mirror { plane } => plane.reflection(),
            Deck::Translation { vector } => Transform::translation(*vector),
            Deck::Rotation { axis, angle } => Transform::rotation(axis.point, axis.dir, angle.radians()).unwrap_or(Transform::IDENTITY),
        }
    }

    /// True if the transform reverses orientation (mirror).
    pub fn is_improper(&self) -> bool {
        matches!(self, Deck::Mirror { .. })
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
            Deck::Rotation { angle, .. } => Some(angle.order()),
        }
    }

    /// Reject degenerate parameters (zero or non-unit normals and axes, zero translation or
    /// angle, non-finite numbers).
    pub fn check(&self) -> Result<()> {
        let unit = |v: Vector3| v.norm().is_finite() && (v.norm() - 1.0).abs() <= 1e-9;
        match self {
            Deck::Mirror { plane } if !(unit(plane.normal) && plane.offset.is_finite()) => Err(ModelError::Invalid("mirror plane needs a unit normal".into())),
            Deck::Translation { vector } if !(vector.norm().is_finite() && vector.norm() > 0.0) => Err(ModelError::Invalid("translation must be finite and non-zero".into())),
            Deck::Rotation { axis, angle } if !unit(axis.dir) || angle.is_zero() || angle.den < 2 => Err(ModelError::Invalid("rotation needs a unit axis and a non-zero angle".into())),
            _ => Ok(()),
        }
    }

    /// True if both decks are the same variant with the same transform within [`GROUP_TOL`].
    pub fn same_as(&self, o: &Deck) -> bool {
        std::mem::discriminant(self) == std::mem::discriminant(o) && transforms_close(&self.transform(), &o.transform(), GROUP_TOL)
    }
}

/// Intent of an interface boundary.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum InterfaceIntent {
    /// A shared (merged) face between volumes of different colors.
    Conformal,
    /// Explicitly non-conformal: two face sets facing each other (sliding, mortar, different
    /// refinement), optionally related by a transform `B = T(A)`.
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
    /// Response boundary: a tag only (vacuum, reflective, source, ... belong to the consumer;
    /// the face color carries the label).
    Response,
    /// Interface between colors, or an explicit non-conformal intent.
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
}

/// Row of the boundary table.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BoundaryRow {
    /// The `EntityKind::Boundary` id.
    pub id: EntityId,
    /// Unique name.
    pub name: String,
    /// The kind.
    pub kind: BoundaryKind,
}

/// Which side of a pairing a face is on. `A` is the mesh here, `B` is the copy of the mesh under
/// the deck transform `g`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum DeckSide {
    /// Source side.
    A,
    /// Image side `B = g(A)`.
    B,
    /// The face is its own mate (mirror).
    SelfPaired,
}

/// What a face does within its boundary record.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum FaceRole {
    /// Deck face. `mate` is `None` exactly when `side` is [`DeckSide::SelfPaired`].
    Deck {
        /// Side of the pairing.
        side: DeckSide,
        /// The paired face.
        mate: Option<EntityId>,
    },
    /// Response face.
    Response,
    /// Interface face; `side` is `A` or `B` (the two face sets of an M:N interface).
    Interface {
        /// Which face set.
        side: DeckSide,
    },
}

/// Row of the face-boundary table: the single source of truth, keyed by face.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FaceBoundary {
    /// The boundary record this face belongs to.
    pub boundary: EntityId,
    /// Role within it.
    pub role: FaceRole,
}

/// The effective kind of a face, with derivation for interior faces.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum FaceKindView {
    /// Deck face.
    Deck {
        /// The record.
        boundary: EntityId,
        /// The deck.
        deck: Deck,
        /// Side of the pairing.
        side: DeckSide,
        /// The mate (`None` for a mirror face).
        mate: Option<EntityId>,
    },
    /// Response face.
    Response {
        /// The record.
        boundary: EntityId,
    },
    /// Interface: explicit (`boundary` is `Some`) or derived `Conformal` for an interior face
    /// whose two side colors differ (`boundary` is `None`).
    Interface {
        /// The record, if explicit.
        boundary: Option<EntityId>,
        /// The intent.
        intent: InterfaceIntent,
    },
    /// Interior face needing no kind (same color on both sides, or a baffle).
    Interior,
    /// A boundary or free face with no kind yet.
    Unassigned,
}

/// What happens when a ray or particle crosses a deck face: the face it re-emerges from and the
/// transform of position (and direction, by the linear part).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DeckHit {
    /// The mate face (`face` itself for a mirror).
    pub mate: EntityId,
    /// Transform from this face's side to the mate's side (inverse for B faces).
    pub to_mate: Transform,
    /// True if `to_mate` reverses orientation (mirror).
    pub improper: bool,
}

/// Dense per-face deck table, built once per topology revision. `hit_by_ordinal` is the O(1)
/// path for tracking (the ordinal is the face's index in `cells(Dim::Face)`); `hit` pays one
/// binary search.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DeckLookup {
    faces: Vec<EntityId>,
    hits: Vec<Option<DeckHit>>,
}

impl DeckLookup {
    /// The deck hit of `face`, if it is a deck face (binary search for the ordinal).
    pub fn hit(&self, face: EntityId) -> Option<&DeckHit> {
        let i = self.faces.binary_search(&face).ok()?;
        self.hits[i].as_ref()
    }

    /// The deck hit of the face at `ordinal` (index into `cells(Dim::Face)` at build time), O(1).
    pub fn hit_by_ordinal(&self, ordinal: usize) -> Option<&DeckHit> {
        self.hits.get(ordinal)?.as_ref()
    }

    /// Number of faces covered (all live faces at build time).
    pub fn len(&self) -> usize {
        self.faces.len()
    }

    /// True if there are no faces.
    pub fn is_empty(&self) -> bool {
        self.faces.is_empty()
    }

    /// Number of deck faces.
    pub fn deck_face_count(&self) -> usize {
        self.hits.iter().flatten().count()
    }
}

impl Model {
    /// Add a boundary record (no faces yet). Deck parameters are validated.
    pub fn add_boundary(&mut self, name: &str, kind: BoundaryKind) -> Result<EntityId> {
        if self.boundaries.values().any(|b| b.name == name) {
            return Err(ModelError::NameTaken(name.to_owned()));
        }
        if let BoundaryKind::Deck(d) = &kind {
            d.check()?;
        }
        let id = self.ids.alloc(EntityKind::Boundary)?;
        self.boundaries.insert(id, BoundaryRow { id, name: name.to_owned(), kind });
        Ok(id)
    }

    /// The boundary record `id`.
    pub fn boundary(&self, id: EntityId) -> Option<&BoundaryRow> {
        self.boundaries.get(&id)
    }

    /// All boundary records, ascending by id.
    pub fn boundaries(&self) -> impl Iterator<Item = &BoundaryRow> {
        self.boundaries.values()
    }

    /// The face's membership row.
    pub fn face_boundary(&self, f: EntityId) -> Option<&FaceBoundary> {
        self.face_boundary.get(&f)
    }

    /// Faces of record `b` with their roles, ascending by face.
    pub fn boundary_faces(&self, b: EntityId) -> impl Iterator<Item = (EntityId, &FaceBoundary)> {
        self.face_boundary.iter().filter(move |(_, fb)| fb.boundary == b).map(|(f, fb)| (*f, fb))
    }

    fn boundary_row(&self, boundary: EntityId) -> Result<&BoundaryRow> {
        expect_kind(boundary, EntityKind::Boundary)?;
        self.boundaries.get(&boundary).ok_or(ModelError::Unknown(boundary))
    }

    fn check_free(&self, face: EntityId) -> Result<()> {
        expect_kind(face, EntityKind::TopoFace)?;
        if self.face_boundary.contains_key(&face) { Err(ModelError::Duplicate(face)) } else { Ok(()) }
    }

    /// Put faces on a mirror record: each is its own mate.
    pub fn assign_self_paired(&mut self, b: EntityId, faces: &[EntityId]) -> Result<()> {
        if !matches!(self.boundary_row(b)?.kind, BoundaryKind::Deck(Deck::Mirror { .. })) {
            return Err(ModelError::Invalid(format!("{b} is not a mirror deck")));
        }
        self.check_distinct_free(faces)?;
        for f in faces {
            self.face_boundary.insert(*f, FaceBoundary { boundary: b, role: FaceRole::Deck { side: DeckSide::SelfPaired, mate: None } });
        }
        Ok(())
    }

    fn check_distinct_free(&self, faces: &[EntityId]) -> Result<()> {
        for (i, f) in faces.iter().enumerate() {
            self.check_free(*f)?;
            if faces[..i].contains(f) {
                return Err(ModelError::Duplicate(*f));
            }
        }
        Ok(())
    }

    /// Put face pairs `(a, b)` on a translation or rotation record: `a` on side A, `b` on side B,
    /// reciprocal mates.
    pub fn assign_pairs(&mut self, b: EntityId, pairs: &[(EntityId, EntityId)]) -> Result<()> {
        match self.boundary_row(b)?.kind {
            BoundaryKind::Deck(Deck::Translation { .. } | Deck::Rotation { .. }) => {}
            _ => return Err(ModelError::Invalid(format!("{b} is not a translation or rotation deck"))),
        }
        let all: Vec<EntityId> = pairs.iter().flat_map(|(x, y)| [*x, *y]).collect();
        self.check_distinct_free(&all)?;
        for (x, y) in pairs {
            self.face_boundary.insert(*x, FaceBoundary { boundary: b, role: FaceRole::Deck { side: DeckSide::A, mate: Some(*y) } });
            self.face_boundary.insert(*y, FaceBoundary { boundary: b, role: FaceRole::Deck { side: DeckSide::B, mate: Some(*x) } });
        }
        Ok(())
    }

    /// Put faces on a response record.
    pub fn assign_response(&mut self, b: EntityId, faces: &[EntityId]) -> Result<()> {
        if !matches!(self.boundary_row(b)?.kind, BoundaryKind::Response) {
            return Err(ModelError::Invalid(format!("{b} is not a response record")));
        }
        self.check_distinct_free(faces)?;
        for f in faces {
            self.face_boundary.insert(*f, FaceBoundary { boundary: b, role: FaceRole::Response });
        }
        Ok(())
    }

    /// Put two face sets on an interface record (M:N; sides `A` and `B`). A conformal record
    /// takes its faces on side A only.
    pub fn assign_interface(&mut self, b: EntityId, a: &[EntityId], bside: &[EntityId]) -> Result<()> {
        match self.boundary_row(b)?.kind {
            BoundaryKind::Interface(InterfaceIntent::Conformal) if !bside.is_empty() => {
                return Err(ModelError::Invalid("a conformal interface has one face set".into()));
            }
            BoundaryKind::Interface(_) => {}
            _ => return Err(ModelError::Invalid(format!("{b} is not an interface record"))),
        }
        let all: Vec<EntityId> = a.iter().chain(bside).copied().collect();
        self.check_distinct_free(&all)?;
        for f in a {
            self.face_boundary.insert(*f, FaceBoundary { boundary: b, role: FaceRole::Interface { side: DeckSide::A } });
        }
        for f in bside {
            self.face_boundary.insert(*f, FaceBoundary { boundary: b, role: FaceRole::Interface { side: DeckSide::B } });
        }
        Ok(())
    }

    /// Remove faces from their records. Removing one member of a deck pair removes its mate too.
    pub fn unassign(&mut self, faces: &[EntityId]) {
        for f in faces {
            if let Some(fb) = self.face_boundary.remove(f)
                && let FaceRole::Deck { mate: Some(m), .. } = fb.role
            {
                self.face_boundary.remove(&m);
            }
        }
    }

    /// The effective kind of a face; interior faces between different volume colors derive
    /// `Interface(Conformal)`. `None` if the face is not alive.
    pub fn face_kind(&self, f: EntityId, t: &dyn TopologyView) -> Option<FaceKindView> {
        let class = t.face_class(f)?;
        if let Some(fb) = self.face_boundary.get(&f) {
            let row = self.boundaries.get(&fb.boundary)?;
            return Some(match (row.kind, fb.role) {
                (BoundaryKind::Deck(deck), FaceRole::Deck { side, mate }) => FaceKindView::Deck { boundary: fb.boundary, deck, side, mate },
                (BoundaryKind::Interface(intent), _) => FaceKindView::Interface { boundary: Some(fb.boundary), intent },
                _ => FaceKindView::Response { boundary: fb.boundary },
            });
        }
        Some(match class {
            FaceClass::Interface => {
                let s = t.face_sides(f)?;
                let col = |r: Region| match r {
                    Region::Volume(v) => self.volume_color.get(&v).copied(),
                    Region::Exterior => None,
                };
                if col(s.left) != col(s.right) { FaceKindView::Interface { boundary: None, intent: InterfaceIntent::Conformal } } else { FaceKindView::Interior }
            }
            FaceClass::Baffle => FaceKindView::Interior,
            FaceClass::Boundary | FaceClass::Free => FaceKindView::Unassigned,
        })
    }

    /// Deck faces with their rows, ascending by face.
    pub fn deck_faces(&self) -> impl Iterator<Item = (EntityId, &FaceBoundary)> {
        self.face_boundary.iter().filter(|(_, fb)| matches!(fb.role, FaceRole::Deck { .. })).map(|(f, fb)| (*f, fb))
    }

    /// Build the O(1) deck table for tracking. Rebuild when the topology revision changes.
    pub fn deck_lookup(&self, t: &dyn TopologyView) -> DeckLookup {
        let faces: Vec<EntityId> = t.cells(Dim::Face).to_vec();
        let mut hits: Vec<Option<DeckHit>> = vec![None; faces.len()];
        for (f, fb) in self.deck_faces() {
            let (Some(i), Some(row), FaceRole::Deck { side, mate }) = (faces.binary_search(&f).ok(), self.boundaries.get(&fb.boundary), fb.role) else { continue };
            let Some(deck) = row.kind.as_deck() else { continue };
            let g = deck.transform();
            let (to_mate, mate_id) = match side {
                DeckSide::A | DeckSide::SelfPaired => (g, mate.unwrap_or(f)),
                DeckSide::B => (g.inverse().unwrap_or(Transform::IDENTITY), mate.unwrap_or(f)),
            };
            hits[i] = Some(DeckHit { mate: mate_id, to_mate, improper: to_mate.is_improper() });
        }
        DeckLookup { faces, hits }
    }
}
