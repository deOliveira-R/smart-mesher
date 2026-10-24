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
///
/// # Known gap
/// Scheme parameters that name entities (for example sweep source and target faces) are opaque
/// JSON owned by the scheme and are **not** pruned. A scheme choice on a live volume whose params
/// name faces of a deleted volume survives, and `mesh.run` then fails with "not applicable"
/// until `mesh.scheme` is issued again.
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
            || i.groups
                .iter()
                .any(|g| g.members.iter().any(|m| ids.has(*m)))
            || i.transforms
                .iter()
                .any(|t| t.targets.iter().any(|m| ids.has(*m)))
            || i.intervals
                .iter()
                .any(|r| r.edges().into_iter().any(|e| ids.has(e)))
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

#[cfg(test)]
mod tests {
    use super::*;
    use sm_core::EntityKind;
    use sm_scheme_api::{
        Firm, Firmness, GroupId, IntervalConstraint, IntervalRequest, MeshGroup, SchemeChoice,
        TransformId, TransformIntent,
    };

    fn id(kind: EntityKind, n: u64) -> EntityId {
        EntityId::new(kind, n).expect("id")
    }

    fn sample() -> IntentLayer {
        let mut l = IntentLayer::default();
        let i = l.intent_mut();
        for n in 1..=4 {
            i.set_intervals(
                id(EntityKind::TopoEdge, n),
                Firm::hard(IntervalRequest::Count(3)),
            );
        }
        i.set_scheme(
            id(EntityKind::TopoVolume, 1),
            SchemeChoice::new("map", Firmness::Hard),
        );
        i.groups.push(MeshGroup::new(
            GroupId(1),
            vec![id(EntityKind::TopoVolume, 1), id(EntityKind::TopoVolume, 2)],
        ));
        i.groups.push(MeshGroup::new(
            GroupId(2),
            vec![id(EntityKind::TopoVolume, 3)],
        ));
        i.transforms.push(TransformIntent::new(
            TransformId(1),
            vec![id(EntityKind::TopoFace, 1)],
            SchemeChoice::new("noop", Firmness::Soft),
        ));
        i.intervals.push(IntervalConstraint::Linear {
            terms: vec![
                (id(EntityKind::TopoEdge, 1), 1),
                (id(EntityKind::TopoEdge, 2), -1),
            ],
            rel: sm_scheme_api::Rel::Eq,
            rhs: 0,
        });
        i.canonicalize();
        l
    }

    /// `references_any` must hold exactly when `on_retired` changes the layer.
    #[test]
    fn references_any_matches_on_retired() {
        let all: Vec<EntityId> = (1..=4)
            .map(|n| id(EntityKind::TopoEdge, n))
            .chain((1..=3).map(|n| id(EntityKind::TopoVolume, n)))
            .chain([id(EntityKind::TopoFace, 1), id(EntityKind::TopoFace, 9)])
            .collect();
        // Every subset of the candidate ids (2^9 of them), in a scrambled order.
        for mask in 0u32..(1 << all.len()) {
            let mut ids: Vec<EntityId> = (0..all.len())
                .filter(|b| mask >> b & 1 == 1)
                .map(|b| all[b])
                .collect();
            ids.reverse();
            let base = sample();
            let mut after = base.clone();
            LayerType::on_retired(&mut after, &ids);
            assert_eq!(
                LayerType::references_any(&base, &ids),
                after != base,
                "mask {mask:#b}"
            );
        }
    }

    #[test]
    fn retirement_prunes_every_kind_of_reference() {
        let mut l = sample();
        LayerType::on_retired(
            &mut l,
            &[
                id(EntityKind::TopoVolume, 1),
                id(EntityKind::TopoVolume, 3),
                id(EntityKind::TopoFace, 1),
                id(EntityKind::TopoEdge, 2),
            ],
        );
        let i = l.intent();
        assert_eq!(
            i.entities.len(),
            3,
            "edges 1, 3, 4 remain; volume 1 is gone"
        );
        assert_eq!(i.groups.len(), 1);
        assert_eq!(i.groups[0].members, vec![id(EntityKind::TopoVolume, 2)]);
        assert!(i.transforms.is_empty(), "its only target was retired");
        assert!(i.intervals.is_empty(), "the row named edge 2");
        assert!(!LayerType::is_empty(&l));
        LayerType::on_retired(
            &mut l,
            &(1..=4)
                .map(|n| id(EntityKind::TopoEdge, n))
                .chain([id(EntityKind::TopoVolume, 2)])
                .collect::<Vec<_>>(),
        );
        assert!(LayerType::is_empty(&l));
    }

    #[test]
    fn digest_depends_on_content_only() {
        use sm_op_api::Layer;
        let (a, b) = (sample(), sample());
        assert_eq!(a.content_hash(), b.content_hash());
        assert_eq!(a.exact_hash(), b.exact_hash());
        let mut c = sample();
        c.intent_mut().set_intervals(
            id(EntityKind::TopoEdge, 1),
            Firm::hard(IntervalRequest::Count(4)),
        );
        assert_ne!(a.content_hash(), c.content_hash());
        assert_ne!(IntentLayer::default().content_hash(), a.content_hash());
    }
}
