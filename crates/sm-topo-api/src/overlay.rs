//! Virtual topology: composites and partitions layered over host entities without changing them.
//!
//! A *composite* presents several host entities as one (merge coplanar faces, chain edges); a
//! *partition* presents one host entity as several (split a face or volume into meshable
//! pieces). Pieces have their own ids of kind `PartitionCell` and are never written into the
//! host complex. The host's content hash is identical with or without an overlay.

use crate::data::*;
use crate::ids::*;
use crate::lineage::LineageLog;
use crate::location::InstanceData;
use crate::view::TopologyView;
use smallvec::SmallVec;
use sm_core::{EntityId, EntityKind};
use sm_geom_api::TolerancePolicy;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// What a virtual piece does to its hosts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[non_exhaustive]
pub enum VirtualKind {
    /// Several host faces presented as one face.
    CompositeFace,
    /// Several host edges (a chain) presented as one edge.
    CompositeEdge,
    /// Part of one host face.
    PartitionFace,
    /// Part of one host volume.
    PartitionVolume,
    /// A host entity hidden (collapsed) from meshing: small edge/face/vertex removal.
    Collapsed,
}

/// One virtual entity.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct VirtualPiece {
    /// Id (`EntityKind::PartitionCell`).
    pub id: EntityId,
    /// What it is.
    pub kind: VirtualKind,
    /// Host entities covered (composites: all members in chain order; partitions: the one host).
    pub hosts: SmallVec<[EntityId; 4]>,
    /// Entities defining the cut (other virtual pieces or host curves) for partitions; empty otherwise.
    pub defined_by: SmallVec<[EntityId; 4]>,
}

/// Read-only virtual topology hook. Object-safe.
pub trait VirtualTopology: Send + Sync {
    /// All pieces, ascending by id.
    fn pieces(&self) -> &[VirtualPiece];

    /// Pieces that cover `host` (as a member or as the partitioned entity), ascending.
    fn covering(&self, host: EntityId) -> Vec<&VirtualPiece> {
        self.pieces().iter().filter(|p| p.hosts.contains(&host)).collect()
    }

    /// Look up a piece.
    fn piece(&self, id: EntityId) -> Option<&VirtualPiece> {
        self.pieces().iter().find(|p| p.id == id)
    }
}

/// Reference overlay: a plain table with deterministic ids.
#[derive(Clone, Debug, Default)]
pub struct OverlayTable {
    alloc: IdAllocator,
    pieces: Vec<VirtualPiece>,
}

/// Why an overlay edit or check failed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum OverlayError {
    /// A host is missing from the complex.
    #[error("host {0} not found")]
    MissingHost(EntityId),
    /// A host already belongs to another composite.
    #[error("host {0} already in a composite")]
    AlreadyComposite(EntityId),
    /// Hosts are of the wrong kind for the requested piece.
    #[error("hosts of wrong kind")]
    WrongKind,
    /// Id space exhausted.
    #[error("id space exhausted")]
    IdSpace,
}

impl OverlayTable {
    /// Empty overlay.
    pub fn new() -> Self {
        Self::default()
    }

    fn push(&mut self, kind: VirtualKind, hosts: &[EntityId], defined_by: &[EntityId]) -> Result<EntityId, OverlayError> {
        let id = self.alloc.alloc(EntityKind::PartitionCell).ok_or(OverlayError::IdSpace)?;
        self.pieces.push(VirtualPiece { id, kind, hosts: hosts.iter().copied().collect(), defined_by: defined_by.iter().copied().collect() });
        Ok(id)
    }

    /// Present `faces` as one virtual face. Each face may be in at most one composite.
    pub fn composite_faces(&mut self, host: &dyn TopologyView, faces: &[FaceId]) -> Result<EntityId, OverlayError> {
        let hosts: Vec<EntityId> = faces.iter().map(|f| f.id()).collect();
        for h in &hosts {
            if !host.is_alive(*h) {
                return Err(OverlayError::MissingHost(*h));
            }
            if self.pieces.iter().any(|p| p.kind == VirtualKind::CompositeFace && p.hosts.contains(h)) {
                return Err(OverlayError::AlreadyComposite(*h));
            }
        }
        self.push(VirtualKind::CompositeFace, &hosts, &[])
    }

    /// Present `volume` as several pieces cut by `cutters` (host faces or other pieces). Returns the
    /// ids of `n` new pieces in order.
    pub fn partition_volume(&mut self, host: &dyn TopologyView, volume: VolumeId, cutters: &[EntityId], n: usize) -> Result<Vec<EntityId>, OverlayError> {
        if !host.is_alive(volume.id()) {
            return Err(OverlayError::MissingHost(volume.id()));
        }
        (0..n).map(|_| self.push(VirtualKind::PartitionVolume, &[volume.id()], cutters)).collect()
    }
}

impl VirtualTopology for OverlayTable {
    fn pieces(&self) -> &[VirtualPiece] {
        &self.pieces
    }
}

/// A host view with an overlay attached. Every query delegates to the host unchanged;
/// only [`TopologyView::virtual_overlay`] differs.
pub struct WithOverlay<'a> {
    /// The unmodified host.
    pub host: &'a dyn TopologyView,
    /// The overlay.
    pub overlay: &'a dyn VirtualTopology,
}

impl TopologyView for WithOverlay<'_> {
    fn policy(&self) -> &TolerancePolicy {
        self.host.policy()
    }
    fn is_alive(&self, id: EntityId) -> bool {
        self.host.is_alive(id)
    }
    fn entity_ids(&self, kind: EntityKind) -> Vec<EntityId> {
        self.host.entity_ids(kind)
    }
    fn vertex(&self, id: VertexId) -> Option<&VertexData> {
        self.host.vertex(id)
    }
    fn edge(&self, id: EdgeId) -> Option<&EdgeData> {
        self.host.edge(id)
    }
    fn face(&self, id: FaceId) -> Option<&FaceData> {
        self.host.face(id)
    }
    fn volume(&self, id: VolumeId) -> Option<&VolumeData> {
        self.host.volume(id)
    }
    fn body(&self, id: BodyId) -> Option<&BodyData> {
        self.host.body(id)
    }
    fn instance(&self, id: InstanceId) -> Option<&InstanceData> {
        self.host.instance(id)
    }
    fn edge_coedges(&self, e: EdgeId) -> Vec<CoedgeRef> {
        self.host.edge_coedges(e)
    }
    fn vertex_edges(&self, v: VertexId) -> Vec<EdgeId> {
        self.host.vertex_edges(v)
    }
    fn face_uses(&self, f: FaceId) -> Vec<FaceUseRef> {
        self.host.face_uses(f)
    }
    fn coincidences(&self) -> &[CoincidenceRecord] {
        self.host.coincidences()
    }
    fn lineage(&self) -> &LineageLog {
        self.host.lineage()
    }
    fn virtual_overlay(&self) -> Option<&dyn VirtualTopology> {
        Some(self.overlay)
    }
}
