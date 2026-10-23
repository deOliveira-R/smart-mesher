//! Unfold: the full model `union g F` as a **view** over the fundamental domain `F`.
//!
//! Nothing is copied and no ids are minted: a copy of a base entity is a [`CopyRef`] `(base, elem)`
//! where `elem` indexes [`UnfoldedModel::elements`] (0 is the identity). Colors, kinds and
//! orientation are answered by queries. The glue rule is `A@e ~ B@(e . g^-1)` for a pair
//! `B = g(A)`, and `a@e ~ a@(e . g)` for a mirror; faces whose partner copy is outside the window
//! stay exposed. [`UnfoldedModel::refold`] returns the exact [`Symmetry`] it came from, and
//! [`UnfoldedModel::origin_of`] follows the recorded cut lineage back to the pre-cut entity.
//!
//! Element order: translation shifts (first generator slowest, exponents `0..n`) times the point
//! group (breadth-first closure). Translations are assumed to commute with the point group; the
//! validator reports `TranslationNotNormalized` otherwise.

use crate::boundary::{DeckSide, FaceKindView};
use crate::error::{ModelError, Result};
use crate::instancing::Placement;
use crate::model::Model;
use crate::symmetry::{MAX_GROUP_ORDER, Symmetry, point_group};
use crate::xf::{GROUP_TOL, TransformIndex, transforms_close};
use sm_core::{EntityId, Transform};
use sm_topo_api::{Sense, TopologyView};
use std::collections::{BTreeMap, BTreeSet};

/// Parameters of an unfold.
#[derive(Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct UnfoldSpec {
    /// Repeat count (window) per translation generator; required for each of them.
    pub translation_repeats: BTreeMap<EntityId, u32>,
}

/// An entity of the fundamental domain seen in one copy.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CopyRef {
    /// The base entity (volume or face of the fundamental domain).
    pub base: EntityId,
    /// Index into [`UnfoldedModel::elements`]; 0 is the identity.
    pub elem: u32,
}

/// Two deck faces identified into one interior face of the unfolded model.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Glue {
    /// One side.
    pub a: CopyRef,
    /// The other side (the same base face for a mirror).
    pub b: CopyRef,
    /// The deck record responsible.
    pub boundary: EntityId,
}

/// The derived full model. Owns everything it answers from, so it is serializable.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct UnfoldedModel {
    elements: Vec<Placement>,
    domain: Vec<EntityId>,
    faces: Vec<EntityId>,
    volume_color: BTreeMap<EntityId, Option<EntityId>>,
    face_info: BTreeMap<EntityId, (Option<EntityId>, Option<FaceKindView>)>,
    glued: Vec<Glue>,
    glued_set: BTreeSet<CopyRef>,
    exposed: Vec<CopyRef>,
    symmetry: Symmetry,
    parent: BTreeMap<EntityId, EntityId>,
}

impl UnfoldedModel {
    /// The copies' placements; index 0 is the identity.
    pub fn elements(&self) -> &[Placement] {
        &self.elements
    }

    /// Every volume copy, ascending by `(elem, base)`.
    pub fn volumes(&self) -> impl Iterator<Item = CopyRef> + '_ {
        (0..self.elements.len() as u32).flat_map(move |elem| self.domain.iter().map(move |b| CopyRef { base: *b, elem }))
    }

    /// Every face copy of the domain's faces, ascending by `(elem, base)`.
    pub fn faces(&self) -> impl Iterator<Item = CopyRef> + '_ {
        (0..self.elements.len() as u32).flat_map(move |elem| self.faces.iter().map(move |b| CopyRef { base: *b, elem }))
    }

    /// The volume color of a copy (unchanged by the group action).
    pub fn color_of(&self, c: CopyRef) -> Option<EntityId> {
        if (c.elem as usize) >= self.elements.len() {
            return None;
        }
        self.volume_color.get(&c.base).copied().flatten()
    }

    /// The face color of a copy.
    pub fn face_color_of(&self, c: CopyRef) -> Option<EntityId> {
        if (c.elem as usize) >= self.elements.len() {
            return None;
        }
        self.face_info.get(&c.base).and_then(|x| x.0)
    }

    /// The kind of a face copy: a glued deck face is an interior face.
    pub fn kind_of(&self, face: CopyRef) -> Option<FaceKindView> {
        if (face.elem as usize) >= self.elements.len() {
            return None;
        }
        let k = self.face_info.get(&face.base)?.1?;
        Some(match k {
            FaceKindView::Deck { .. } if self.glued_set.contains(&face) => FaceKindView::Interior,
            other => other,
        })
    }

    /// The loop sense of a face copy relative to the transformed base loop: `Reversed` exactly
    /// when the copy's placement is improper. An improper copy reverses the loop of each image
    /// face so the stored normal follows the right-hand rule again, then left/right is re-derived
    /// from the copy's volume ids (ir.md §6); sense is never mixed with that swap.
    pub fn sense(&self, face: CopyRef) -> Sense {
        match self.elements.get(face.elem as usize) {
            Some(p) if p.orientation_reversed => Sense::Reversed,
            _ => Sense::Forward,
        }
    }

    /// The pre-cut entity this copy's base entity came from, through the recorded cut lineage
    /// (the base entity itself if no cut changed it).
    pub fn origin_of(&self, c: CopyRef) -> Option<EntityId> {
        if (c.elem as usize) >= self.elements.len() {
            return None;
        }
        let mut cur = c.base;
        for _ in 0..1024 {
            match self.parent.get(&cur) {
                Some(p) if *p != cur => cur = *p,
                _ => break,
            }
        }
        Some(cur)
    }

    /// Identified deck face pairs.
    pub fn glued(&self) -> &[Glue] {
        &self.glued
    }

    /// Deck faces with no neighbour copy in the window.
    pub fn exposed(&self) -> &[CopyRef] {
        &self.exposed
    }

    /// The exact symmetry this view was unfolded from.
    pub fn refold(&self) -> &Symmetry {
        &self.symmetry
    }
}

/// Unfold the declared symmetry of `m` over topology `t` (the fundamental domain).
pub fn unfold(m: &Model, t: &dyn TopologyView, spec: &UnfoldSpec) -> Result<UnfoldedModel> {
    let sym = m.symmetry().ok_or(ModelError::NoSymmetry)?;
    let mut point = Vec::new();
    let mut tiles: Vec<(EntityId, Transform, u32)> = Vec::new();
    let mut gens: Vec<(EntityId, crate::boundary::Deck)> = Vec::new();
    for b in &sym.generators {
        let deck = *m.boundary(*b).and_then(|r| r.kind.as_deck()).ok_or(ModelError::Unknown(*b))?;
        match deck.order() {
            Some(_) => point.push(deck.transform()),
            None => {
                let n = *spec.translation_repeats.get(b).ok_or(ModelError::MissingRepeats(*b))?;
                if n == 0 {
                    return Err(ModelError::Invalid("a translation window needs at least one copy".into()));
                }
                tiles.push((*b, deck.transform(), n));
            }
        }
        gens.push((*b, deck));
    }
    let pg = point_group(&point, MAX_GROUP_ORDER)?;
    let mut shifts = vec![Transform::IDENTITY];
    for (_, tr, n) in &tiles {
        // Bound the element count before allocating (u32::MAX repeats must not allocate).
        let total = shifts.len().checked_mul(*n as usize).and_then(|c| c.checked_mul(pg.len()));
        if total.is_none_or(|c| c > MAX_GROUP_ORDER) {
            return Err(ModelError::GroupNotFinite { limit: MAX_GROUP_ORDER });
        }
        let mut next = Vec::with_capacity(shifts.len() * *n as usize);
        for s in &shifts {
            let mut cur = *s;
            for _ in 0..*n {
                next.push(cur);
                cur = tr.compose(&cur);
            }
        }
        shifts = next;
    }
    let mut elems: Vec<Transform> = Vec::with_capacity(shifts.len() * pg.len());
    for s in &shifts {
        for p in &pg {
            elems.push(s.compose(p));
        }
    }
    let mut index = TransformIndex::default();
    for (i, e) in elems.iter().enumerate() {
        index.insert(e, i as u32);
    }
    let find = |x: &Transform| index.candidates(x).into_iter().find(|&c| transforms_close(&elems[c as usize], x, GROUP_TOL));

    let domain: Vec<EntityId> = sym.domain.iter().copied().collect();
    let mut faces: BTreeSet<EntityId> = BTreeSet::new();
    for v in &domain {
        faces.extend(t.volume_faces(*v).iter().copied());
    }
    let faces: Vec<EntityId> = faces.into_iter().collect();

    let (mut glued, mut exposed) = (Vec::new(), Vec::new());
    for (b, deck) in &gens {
        let g = deck.transform();
        let ginv = g.inverse().ok_or(ModelError::Singular)?;
        for (f, fb) in m.boundary_faces(*b) {
            let crate::boundary::FaceRole::Deck { side, mate } = fb.role else { continue };
            for (e, el) in elems.iter().enumerate() {
                let e = e as u32;
                match side {
                    DeckSide::A => {
                        let mate = mate.unwrap_or(f);
                        match find(&el.compose(&ginv)) {
                            Some(j) => glued.push(Glue { a: CopyRef { base: f, elem: e }, b: CopyRef { base: mate, elem: j }, boundary: *b }),
                            None => exposed.push(CopyRef { base: f, elem: e }),
                        }
                    }
                    DeckSide::B => {
                        if find(&el.compose(&g)).is_none() {
                            exposed.push(CopyRef { base: f, elem: e });
                        }
                    }
                    DeckSide::SelfPaired => match find(&el.compose(&g)) {
                        Some(j) if j > e => glued.push(Glue { a: CopyRef { base: f, elem: e }, b: CopyRef { base: f, elem: j }, boundary: *b }),
                        Some(_) => {}
                        None => exposed.push(CopyRef { base: f, elem: e }),
                    },
                }
            }
        }
    }
    glued.sort_by_key(|g| (g.a, g.b));
    exposed.sort();
    let glued_set: BTreeSet<CopyRef> = glued.iter().flat_map(|g| [g.a, g.b]).collect();
    let mut parent = BTreeMap::new();
    for l in m.cut_lineage().iter().flatten() {
        for n in &l.new {
            if *n != l.old {
                parent.insert(*n, l.old);
            }
        }
    }
    Ok(UnfoldedModel {
        elements: elems.iter().map(|e| Placement::of(*e)).collect(),
        volume_color: domain.iter().map(|v| (*v, m.volume_color(*v))).collect(),
        face_info: faces.iter().map(|f| (*f, (m.face_color(*f), m.face_kind(*f, t)))).collect(),
        domain,
        faces,
        glued,
        glued_set,
        exposed,
        symmetry: sym.clone(),
        parent,
    })
}

impl UnfoldedModel {
    pub(crate) fn element_transform(&self, e: u32) -> Option<Transform> {
        self.elements.get(e as usize).map(|p| p.transform)
    }
}
