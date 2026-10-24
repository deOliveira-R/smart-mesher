//! The `fem` layer: element types per block, side preferences per sideset face.

use std::collections::BTreeMap;

use sm_core::EntityId;
use sm_op_api::{Digest, LayerType, names};

use crate::element::ElementType;

/// Layer name of [`FemLayer`].
pub const FEM_LAYER: &str = "fem";

/// FE-specific data that the `model` tables cannot hold.
///
/// Blocks are volume [`Color`](sm_model_api::ColorRow)s and sidesets are face colors, so the
/// partition of volumes into blocks lives in the `model` layer. This layer adds only
/// - the element type declared for a block color, and
/// - the volume a sideset face is taken `wrt` (which side of an interior face the sideset lowers
///   to).
///
/// # Digest (version 1)
/// The element-type count, then `(color id, type code)` ascending; then the side-preference
/// count and `(face id, volume id)` ascending. Everything is integer data, so the hash is exact
/// on every platform.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FemLayer {
    element_types: BTreeMap<EntityId, ElementType>,
    side_wrt: BTreeMap<EntityId, EntityId>,
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

    /// The volume whose side of `face` a sideset takes, if one was given.
    pub fn side_wrt(&self, face: EntityId) -> Option<EntityId> {
        self.side_wrt.get(&face).copied()
    }

    /// All side preferences `(face, volume)`, ascending by face.
    pub fn side_prefs(&self) -> impl Iterator<Item = (EntityId, EntityId)> + '_ {
        self.side_wrt.iter().map(|(f, v)| (*f, *v))
    }

    pub(crate) fn set_element_type(&mut self, color: EntityId, ty: ElementType) {
        self.element_types.insert(color, ty);
    }

    pub(crate) fn set_side_wrt(&mut self, face: EntityId, wrt: Option<EntityId>) {
        match wrt {
            Some(v) => {
                self.side_wrt.insert(face, v);
            }
            None => {
                self.side_wrt.remove(&face);
            }
        }
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
        d.u64(self.side_wrt.len() as u64);
        for (f, v) in &self.side_wrt {
            d.id(*f);
            d.id(*v);
        }
    }

    fn is_empty(&self) -> bool {
        self.element_types.is_empty() && self.side_wrt.is_empty()
    }

    /// Drops the entries whose block color, face or volume was retired.
    fn on_retired(&mut self, ids: &[EntityId]) {
        let mut sorted = ids.to_vec();
        sorted.sort_unstable();
        let gone = |id: &EntityId| sorted.binary_search(id).is_ok();
        self.element_types.retain(|c, _| !gone(c));
        self.side_wrt.retain(|f, v| !gone(f) && !gone(v));
    }

    fn references_any(&self, ids: &[EntityId]) -> bool {
        ids.iter().any(|id| {
            self.element_types.contains_key(id)
                || self.side_wrt.contains_key(id)
                || self.side_wrt.values().any(|v| v == id)
        })
    }
}
