//! The `intent` layer: what the user asked of the mesher, keyed by model entity.

use sm_core::EntityId;
use sm_op_api::{Digest, LayerType, names};
use sm_scheme_api::MeshIntent;

/// The `intent` layer ([`names::INTENT`], depends on `model`): sizes, interval counts and scheme
/// choices recorded by `mesh.size`, `mesh.intervals`, `mesh.scheme` and `mesh.default_size`.
///
/// It wraps [`MeshIntent`] (a `BTreeMap` keyed by entity id), so the state does not depend on the
/// order requests were made in for different entities; a second request on the same entity
/// replaces the first (subject to the firmness ladder, see [`MeshPlugin`](crate::MeshPlugin)).
///
/// # Digest (version 1)
/// `Digest::value(&intent)`, i.e. the canonical encoding of the whole intent. Its floats are user
/// input (sizes, or the value `auto` resolved to at apply time) and are hashed unquantized.
/// `mesh.size {auto}` therefore stores a computed float: it is bit-identical across runs of one
/// build, and can differ across platforms only if the configured auto-size hook does.
///
/// # Retirement
/// [`on_retired`](LayerType::on_retired) drops every request on a retired entity: entity rows,
/// corner overrides on a retired face, retired group members and transform targets (a group or
/// transform left empty is dropped) and interval rows naming a retired edge.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct IntentLayer {
    intent: MeshIntent,
}

impl IntentLayer {
    /// The recorded intent.
    pub fn intent(&self) -> &MeshIntent {
        &self.intent
    }

    /// Mutable intent for the ops of this crate (they re-canonicalize and check afterwards).
    pub(crate) fn intent_mut(&mut self) -> &mut MeshIntent {
        &mut self.intent
    }

    /// True when retiring `ids` would change the layer. Mirrors [`prune`](Self::prune).
    fn affected(&self, ids: &Ids) -> bool {
        let i = &self.intent;
        i.entities.keys().any(|k| ids.has(*k))
            || i.corners.iter().any(|c| ids.has(c.corner.face))
            || i.groups.iter().any(|g| g.members.iter().any(|m| ids.has(*m)))
            || i.transforms.iter().any(|t| t.targets.iter().any(|m| ids.has(*m)))
            || i.intervals.iter().any(|r| r.edges().into_iter().any(|e| ids.has(e)))
    }

    /// Remove every request that names a retired entity.
    fn prune(&mut self, ids: &Ids) {
        let i = &mut self.intent;
        i.entities.retain(|k, _| !ids.has(*k));
        i.corners.retain(|c| !ids.has(c.corner.face));
        for g in &mut i.groups {
            g.members.retain(|m| !ids.has(*m));
        }
        i.groups.retain(|g| !g.members.is_empty());
        for t in &mut i.transforms {
            t.targets.retain(|m| !ids.has(*m));
        }
        i.transforms.retain(|t| !t.targets.is_empty());
        i.intervals
            .retain(|r| !r.edges().into_iter().any(|e| ids.has(e)));
    }
}

/// A sorted id set (borrowed when the input is already ascending, as `Delta.removed` is).
struct Ids<'a>(std::borrow::Cow<'a, [EntityId]>);

impl<'a> Ids<'a> {
    fn new(ids: &'a [EntityId]) -> Self {
        if ids.is_sorted() {
            return Self(std::borrow::Cow::Borrowed(ids));
        }
        let mut v = ids.to_vec();
        v.sort_unstable();
        v.dedup();
        Self(std::borrow::Cow::Owned(v))
    }

    fn has(&self, id: EntityId) -> bool {
        self.0.binary_search(&id).is_ok()
    }
}

impl LayerType for IntentLayer {
    const NAME: &'static str = names::INTENT;
    const DEPENDS_ON: &'static [&'static str] = &[names::MODEL];

    fn digest(&self, d: &mut Digest) {
        d.value(&self.intent);
    }

    fn is_empty(&self) -> bool {
        self.intent == MeshIntent::default()
    }

    fn on_retired(&mut self, ids: &[EntityId]) {
        self.prune(&Ids::new(ids));
    }

    fn references_any(&self, ids: &[EntityId]) -> bool {
        self.affected(&Ids::new(ids))
    }
}
