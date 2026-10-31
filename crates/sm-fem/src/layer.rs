//! The `fem` layer: element types per block, sideset memberships and side preferences per face.

use std::collections::BTreeMap;

use sm_core::EntityId;
use sm_op_api::{Digest, LayerType, Successors, names};

use crate::element::ElementType;

/// Layer name of [`FemLayer`].
pub const FEM_LAYER: &str = "fem";

/// FE-specific data that the `model` tables cannot hold.
///
/// Blocks are volume [`Color`](sm_model_api::ColorRow)s and sidesets are face colors, so the
/// partition of volumes into blocks lives in the `model` layer. The model gives a face only one
/// color, so this layer also holds what a sideset needs beyond that:
/// - the element type declared for a block color,
/// - the sideset memberships of faces (a face may be in several sidesets), and
/// - for each membership, the volume the face is taken `wrt` (which side of an interior face
///   the sideset lowers to).
///
/// # Digest (version 2)
/// The element-type count, then `(color id, type code)` ascending; then the membership count and
/// `(face id, color id, volume count, volume ids)` ascending by face then color. Everything is
/// integer data, so the hash is exact on every platform.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FemLayer {
    element_types: BTreeMap<EntityId, ElementType>,
    /// `(face, sideset color)` to the volumes the face may be taken `wrt`: empty when none was
    /// given, one volume as added, several after the volume split (the lowering picks the one
    /// the face borders). Sorted and distinct.
    members: BTreeMap<(EntityId, EntityId), Vec<EntityId>>,
}

impl FemLayer {
    /// The element type declared for the block color `color`.
    pub fn element_type(&self, color: EntityId) -> Option<ElementType> {
        self.element_types.get(&color).copied()
    }

    /// All declared element types, ascending by block color.
    pub fn element_types(&self) -> impl Iterator<Item = (EntityId, ElementType)> + '_ {
        self.element_types.iter().map(|(c, t)| (*c, *t))
    }

    /// The volumes whose side of `face` the sideset `color` takes, ascending; empty when none
    /// was given.
    ///
    /// One volume as added by `fem.sideset_add`. When that volume is later split, the face keeps
    /// all its pieces as candidates (the face borders at most one of them on a given side).
    pub fn side_wrt(&self, face: EntityId, color: EntityId) -> &[EntityId] {
        self.members.get(&(face, color)).map_or(&[], Vec::as_slice)
    }

    /// All memberships `(face, sideset color, wrt volumes)`, ascending by face then color.
    ///
    /// A face can appear under several colors; the model's own single face color is not
    /// repeated here unless `fem.sideset_add` named it.
    pub fn memberships(&self) -> impl Iterator<Item = (EntityId, EntityId, &[EntityId])> + '_ {
        self.members
            .iter()
            .map(|((f, c), vs)| (*f, *c, vs.as_slice()))
    }

    /// All side preferences `(face, volume)`, ascending by face, then color, then volume.
    pub fn side_prefs(&self) -> impl Iterator<Item = (EntityId, EntityId)> + '_ {
        self.members
            .iter()
            .flat_map(|((f, _), vs)| vs.iter().map(move |v| (*f, *v)))
    }

    pub(crate) fn set_element_type(&mut self, color: EntityId, ty: ElementType) {
        self.element_types.insert(color, ty);
    }

    /// Put `face` in sideset `color`, replacing the `wrt` an earlier add of the pair stored.
    pub(crate) fn set_side_wrt(&mut self, face: EntityId, color: EntityId, wrt: Option<EntityId>) {
        self.members.insert((face, color), wrt.into_iter().collect());
    }
}

impl LayerType for FemLayer {
    const NAME: &'static str = FEM_LAYER;
    const DEPENDS_ON: &'static [&'static str] = &[names::MODEL];

    fn digest(&self, d: &mut Digest) {
        d.u64(self.element_types.len() as u64);
        for (c, t) in &self.element_types {
            d.id(*c);
            d.u64(t.code());
        }
        d.u64(self.members.len() as u64);
        for ((f, c), vs) in &self.members {
            d.id(*f);
            d.id(*c);
            d.u64(vs.len() as u64);
            for v in vs {
                d.id(*v);
            }
        }
    }

    fn is_empty(&self) -> bool {
        self.element_types.is_empty() && self.members.is_empty()
    }

    /// Carries memberships over splits and merges: a split face passes its memberships to
    /// every child, and a split volume is replaced by all its pieces in every list. (The layer
    /// has no topology access, so it cannot tell which piece owns which child face; the
    /// lowering resolves that against the mesh.)
    fn on_succession(&mut self, succ: &Successors) {
        let mut next: BTreeMap<(EntityId, EntityId), Vec<EntityId>> = BTreeMap::new();
        for ((f, c), vs) in std::mem::take(&mut self.members) {
            let faces = match succ.get(&f) {
                Some(new) => new.as_slice(),
                None => std::slice::from_ref(&f),
            };
            let mut mapped: Vec<EntityId> = Vec::with_capacity(vs.len());
            for v in vs {
                match succ.get(&v) {
                    Some(new) => mapped.extend_from_slice(new),
                    None => mapped.push(v),
                }
            }
            for g in faces {
                next.entry((*g, c)).or_default().extend_from_slice(&mapped);
            }
        }
        for vs in next.values_mut() {
            vs.sort_unstable();
            vs.dedup();
        }
        self.members = next;
    }

    /// Drops the entries whose block color, sideset color or face was retired, and retired
    /// volumes from the preferences (a membership whose every preferred volume is gone is
    /// dropped; one that never had a preference stays).
    fn on_retired(&mut self, ids: &[EntityId]) {
        let mut sorted = ids.to_vec();
        sorted.sort_unstable();
        let gone = |id: &EntityId| sorted.binary_search(id).is_ok();
        self.element_types.retain(|c, _| !gone(c));
        self.members.retain(|(f, c), vs| {
            if gone(f) || gone(c) {
                return false;
            }
            let had = !vs.is_empty();
            vs.retain(|v| !gone(v));
            !(had && vs.is_empty())
        });
    }

    fn references_any(&self, ids: &[EntityId]) -> bool {
        let mut sorted = ids.to_vec();
        sorted.sort_unstable();
        let hit = |id: &EntityId| sorted.binary_search(id).is_ok();
        self.element_types.keys().any(hit)
            || self
                .members
                .iter()
                .any(|((f, c), vs)| hit(f) || hit(c) || vs.iter().any(hit))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_core::EntityKind;

    fn id(kind: EntityKind, n: u64) -> EntityId {
        EntityId::new(kind, n).expect("id")
    }

    #[test]
    fn succession_maps_faces_to_children_and_volumes_to_pieces() {
        let (f, g, h) = (
            id(EntityKind::TopoFace, 1),
            id(EntityKind::TopoFace, 2),
            id(EntityKind::TopoFace, 3),
        );
        let (v, v1, v2) = (
            id(EntityKind::TopoVolume, 1),
            id(EntityKind::TopoVolume, 2),
            id(EntityKind::TopoVolume, 3),
        );
        let c = id(EntityKind::Color, 1);
        let mut l = FemLayer::default();
        l.set_side_wrt(f, c, Some(v));
        // The face splits, then the volume splits.
        let mut s = Successors::new();
        s.insert(f, vec![g, h]);
        assert!(l.references_any(&[f]));
        l.on_succession(&s);
        assert_eq!(l.side_wrt(g, c), &[v]);
        assert_eq!(l.side_wrt(h, c), &[v]);
        assert!(l.side_wrt(f, c).is_empty());
        let mut s = Successors::new();
        s.insert(v, vec![v1, v2]);
        assert!(l.references_any(&[v]));
        l.on_succession(&s);
        assert_eq!(l.side_wrt(g, c), &[v1, v2]);
        // Retiring the old ids leaves the pieces; retiring both pieces drops the entry.
        l.on_retired(&[f, v]);
        assert_eq!(l.memberships().count(), 2);
        l.on_retired(&[v1, v2]);
        assert!(l.is_empty());
    }

    #[test]
    fn references_any_matches_faces_volumes_and_colors() {
        let f = id(EntityKind::TopoFace, 1);
        let v = id(EntityKind::TopoVolume, 1);
        let c = id(EntityKind::Color, 1);
        let other = id(EntityKind::TopoVolume, 9);
        let mut l = FemLayer::default();
        l.set_side_wrt(f, c, Some(v));
        assert!(l.references_any(&[other, v]) && l.references_any(&[f, c]));
        assert!(l.references_any(&[c]));
        assert!(!l.references_any(&[other]));
    }

    #[test]
    fn a_face_is_in_several_sidesets() {
        let f = id(EntityKind::TopoFace, 1);
        let (v1, v2) = (id(EntityKind::TopoVolume, 1), id(EntityKind::TopoVolume, 2));
        let (c1, c2) = (id(EntityKind::Color, 1), id(EntityKind::Color, 2));
        let mut l = FemLayer::default();
        l.set_side_wrt(f, c1, Some(v1));
        l.set_side_wrt(f, c2, Some(v2));
        assert_eq!(l.side_wrt(f, c1), &[v1]);
        assert_eq!(l.side_wrt(f, c2), &[v2]);
        // Re-adding without wrt clears only that pair's preference.
        l.set_side_wrt(f, c1, None);
        assert!(l.side_wrt(f, c1).is_empty());
        assert_eq!(l.memberships().count(), 2);
    }
}
