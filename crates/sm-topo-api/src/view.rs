//! The read-only query trait used by meshers, semantics and tracking.

use crate::data::*;
use crate::ids::*;
use crate::lineage::LineageLog;
use crate::location::{InstanceData, Location};
use crate::overlay::VirtualTopology;
use sm_core::{EntityId, EntityKind};
use sm_geom_api::TolerancePolicy;

/// Read-only access to a cellular non-manifold complex. Object-safe: consumers take
/// `&dyn TopologyView`. Implementations must be deterministic: every `Vec` returned is in a
/// documented, input-independent order (ascending id unless stated otherwise).
///
/// Adjacency queries (`edge_coedges`, `vertex_edges`, `face_uses`) are required methods so an
/// implementation can serve them from reverse indices; everything else is derived.
pub trait TopologyView: Send + Sync {
    /// The model's tolerance policy (bounds for per-entity tolerances).
    fn policy(&self) -> &TolerancePolicy;
    /// True if `id` exists and has not been retired by a split, merge or delete.
    fn is_alive(&self, id: EntityId) -> bool;
    /// All live entities of `kind`, ascending by id.
    fn entity_ids(&self, kind: EntityKind) -> Vec<EntityId>;

    /// Vertex record.
    fn vertex(&self, id: VertexId) -> Option<&VertexData>;
    /// Edge record.
    fn edge(&self, id: EdgeId) -> Option<&EdgeData>;
    /// Face record.
    fn face(&self, id: FaceId) -> Option<&FaceData>;
    /// Volume record.
    fn volume(&self, id: VolumeId) -> Option<&VolumeData>;
    /// Body record.
    fn body(&self, id: BodyId) -> Option<&BodyData>;
    /// Instance record.
    fn instance(&self, id: InstanceId) -> Option<&InstanceData>;

    /// Every coedge using `e`, ordered by owner id then ring then index. More than two entries
    /// means a non-manifold edge.
    fn edge_coedges(&self, e: EdgeId) -> Vec<CoedgeRef>;
    /// Edges incident to `v`, ascending.
    fn vertex_edges(&self, v: VertexId) -> Vec<EdgeId>;
    /// The (at most two) shell uses of `f`, ordered by volume id.
    fn face_uses(&self, f: FaceId) -> Vec<FaceUseRef>;

    /// Imprint/merge status records (ordered by `(a, b)`).
    fn coincidences(&self) -> &[CoincidenceRecord];
    /// The lineage log.
    fn lineage(&self) -> &LineageLog;
    /// Virtual topology overlay (composites/partitions), if one is attached. Host entities are
    /// never modified by an overlay.
    fn virtual_overlay(&self) -> Option<&dyn VirtualTopology> {
        None
    }

    /// Number of live entities of `kind`.
    fn count(&self, kind: EntityKind) -> usize {
        self.entity_ids(kind).len()
    }

    /// Resolve a coedge address.
    fn coedge(&self, r: CoedgeRef) -> Option<&Coedge> {
        let (ring, ix) = (r.ring as usize, r.index as usize);
        match r.owner {
            CoedgeOwner::Face(f) => self.face(f)?.loops.get(ring)?.coedges.get(ix),
            CoedgeOwner::Volume(v) => self.volume(v)?.wires.get(ring)?.coedges.get(ix),
            CoedgeOwner::Body(b) => self.body(b)?.free_wires.get(ring)?.coedges.get(ix),
        }
    }

    /// `(first, last)` vertex of a coedge in its traversal direction.
    fn coedge_vertices(&self, c: &Coedge) -> Option<(VertexId, VertexId)> {
        let e = self.edge(c.edge)?;
        Some(match c.sense {
            Sense::Forward => (e.start, e.end),
            Sense::Reversed => (e.end, e.start),
        })
    }

    /// Resolve a face-use address.
    fn face_use(&self, r: FaceUseRef) -> Option<&FaceUse> {
        self.volume(r.volume)?.shells.get(r.shell as usize)?.uses.get(r.index as usize)
    }

    /// The two sides of a face (`Exterior` where no volume uses that side).
    fn face_sides(&self, f: FaceId) -> Option<FaceSides> {
        self.face(f)?;
        let mut sides = FaceSides { front: Side::Exterior, back: Side::Exterior };
        for r in self.face_uses(f) {
            let u = self.face_use(r)?;
            match u.sense {
                Sense::Forward => sides.back = Side::Volume(r.volume),
                Sense::Reversed => sides.front = Side::Volume(r.volume),
            }
        }
        Some(sides)
    }

    /// Role of a face derived from its uses.
    fn face_role(&self, f: FaceId) -> Option<FaceRole> {
        let uses = self.face_uses(f);
        let s = self.face_sides(f)?;
        let senses: Vec<Sense> = uses.iter().filter_map(|r| self.face_use(*r)).map(|u| u.sense).collect();
        Some(if uses.is_empty() {
            FaceRole::Free
        } else if uses.len() > 2 || (uses.len() == 2 && senses[0] == senses[1]) {
            FaceRole::Invalid
        } else if s.front == s.back {
            FaceRole::Baffle
        } else if s.front == Side::Exterior || s.back == Side::Exterior {
            FaceRole::Boundary
        } else {
            FaceRole::Interface
        })
    }

    /// Distinct faces using `e`, ascending.
    fn edge_faces(&self, e: EdgeId) -> Vec<FaceId> {
        let mut v: Vec<FaceId> = self
            .edge_coedges(e)
            .into_iter()
            .filter_map(|c| if let CoedgeOwner::Face(f) = c.owner { Some(f) } else { None })
            .collect();
        v.dedup();
        v
    }

    /// Distinct faces in the shells of `v`, in shell order (first occurrence).
    fn volume_faces(&self, v: VolumeId) -> Vec<FaceId> {
        let mut out: Vec<FaceId> = Vec::new();
        if let Some(vol) = self.volume(v) {
            for u in vol.shells.iter().flat_map(|s| s.uses.iter()) {
                if !out.contains(&u.face) {
                    out.push(u.face);
                }
            }
        }
        out
    }

    /// Distinct edges bounding `f`, in first-occurrence order.
    fn face_edges(&self, f: FaceId) -> Vec<EdgeId> {
        let mut out: Vec<EdgeId> = Vec::new();
        if let Some(face) = self.face(f) {
            for c in face.loops.iter().flat_map(|l| l.coedges.iter()) {
                if !out.contains(&c.edge) {
                    out.push(c.edge);
                }
            }
        }
        out
    }

    /// Status record for a face pair (either order).
    fn coincidence(&self, a: FaceId, b: FaceId) -> Option<&CoincidenceRecord> {
        let (a, b) = if a <= b { (a, b) } else { (b, a) };
        self.coincidences().iter().find(|r| r.a == a && r.b == b)
    }

    /// Effective placement of an instance: the composition of its location with its parents'.
    fn instance_location(&self, id: InstanceId) -> Option<Location> {
        let mut cur = self.instance(id)?;
        let mut loc = cur.target.location;
        for _ in 0..64 {
            match cur.parent {
                None => return Some(loc),
                Some(p) => {
                    cur = self.instance(p)?;
                    loc = cur.target.location.compose(&loc);
                }
            }
        }
        None
    }
}
