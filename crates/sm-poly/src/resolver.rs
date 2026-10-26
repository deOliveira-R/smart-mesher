//! Mapping a pair of model faces to the feature curve they share.

use sm_core::{EntityId, EntityKind};
use sm_mesh_api::Classification;
use sm_topo_api::{TopologyView, TopologyViewExt};

/// Finds the `TopoEdge` that a mesh edge between two different `TopoFace`s lies on.
///
/// Called for every region-boundary mesh edge whose two triangles lie on different model faces
/// (and for which the source has no edge table). `a < b` always.
pub trait EdgeResolver: Send + Sync {
    /// The feature curve shared by faces `a` and `b` that the mesh edge with end-vertex
    /// classifications `ends` lies on, or `None` if there is no unique answer (the caller then
    /// reports [`PolyError::AmbiguousEdge`](crate::PolyError::AmbiguousEdge)).
    fn resolve(&self, a: EntityId, b: EntityId, ends: [Classification; 2]) -> Option<EntityId>;
}

/// Resolves from the end vertices only: an end on a `TopoEdge` names the curve.
///
/// Needs at least two mesh intervals per curve: an edge that joins two `TopoVertex` ends is
/// ambiguous. Two ends on different `TopoEdge`s are ambiguous as well.
#[derive(Clone, Copy, Debug, Default)]
pub struct VertexClassResolver;

impl EdgeResolver for VertexClassResolver {
    fn resolve(&self, _a: EntityId, _b: EntityId, ends: [Classification; 2]) -> Option<EntityId> {
        let mut found: Option<EntityId> = None;
        for c in ends {
            if c.entity.kind() == EntityKind::TopoEdge {
                match found {
                    Some(e) if e != c.entity => return None,
                    _ => found = Some(c.entity),
                }
            }
        }
        found
    }
}

/// Resolves from the model topology: the edges common to both faces, filtered by what the
/// mesh edge's end vertices are classified on.
#[derive(Clone, Copy)]
pub struct TopologyResolver<'a>(pub &'a dyn TopologyView);

impl EdgeResolver for TopologyResolver<'_> {
    fn resolve(&self, a: EntityId, b: EntityId, ends: [Classification; 2]) -> Option<EntityId> {
        let (ea, eb) = (self.0.face_edges(a), self.0.face_edges(b));
        let (mut i, mut j) = (0, 0);
        let mut found: Option<EntityId> = None;
        while i < ea.len() && j < eb.len() {
            match ea[i].cmp(&eb[j]) {
                core::cmp::Ordering::Less => i += 1,
                core::cmp::Ordering::Greater => j += 1,
                core::cmp::Ordering::Equal => {
                    let edge = ea[i];
                    i += 1;
                    j += 1;
                    if !ends.iter().all(|c| self.end_fits(edge, c.entity)) {
                        continue;
                    }
                    if found.is_some() {
                        return None;
                    }
                    found = Some(edge);
                }
            }
        }
        found
    }
}

impl TopologyResolver<'_> {
    /// True if an end vertex classified on `class` can lie on `edge`.
    fn end_fits(&self, edge: EntityId, class: EntityId) -> bool {
        match class.kind() {
            EntityKind::TopoEdge => class == edge,
            EntityKind::TopoVertex => self
                .0
                .edge_ends(edge)
                .is_some_and(|(p, q)| p == class || q == class),
            _ => false,
        }
    }
}
