//! Virtual topology overlay (composite / partition) that never touches host entities.
//!
//! An overlay is a separate object that *refers* to host entities by id. Virtual entities get ids
//! in a reserved serial range ([`VIRTUAL_SERIAL_BIT`]) so they can never collide with host ids,
//! and every consumer sees the union "visible host entities + virtual entities" through
//! [`visible_faces`] / [`visible_edges`]. Removing the overlay restores the host view exactly
//! (reversibility), and host state hashes are unaffected by construction (the overlay only takes
//! `&dyn TopologyView`).

use crate::error::TopoError;
use crate::ids::*;
use crate::model::StableHasher;
use crate::view::TopologyView;
use sm_core::{EntityId, EntityKind};
use std::collections::{BTreeMap, BTreeSet};

/// Kind of virtual operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VirtualKind {
    /// Several adjacent host entities presented as one (hides the interior boundaries).
    Composite,
    /// One host entity presented as several pieces.
    Partition,
}

/// A virtual entity and the host entities it stands for.
#[derive(Clone, Debug, PartialEq)]
pub struct VirtualEntity {
    /// Id in the reserved virtual range; the kind equals the hosts' kind.
    pub id: EntityId,
    /// Composite or partition piece.
    pub kind: VirtualKind,
    /// Composite: all merged hosts. Partition: the single host.
    pub hosts: Vec<EntityId>,
    /// Partition: piece ordinal within the host (0 for composites).
    pub piece: u32,
}

/// Read side of an overlay. Object-safe; implemented by `sm-virtual-topo`.
pub trait VirtualOverlay: Send + Sync {
    /// Overlay name (for diagnostics).
    fn name(&self) -> &str;
    /// All virtual entities, ascending by id.
    fn entities(&self) -> &[VirtualEntity];
    /// Look up a virtual entity.
    fn entity(&self, id: EntityId) -> Option<&VirtualEntity> {
        self.entities().iter().find(|e| e.id == id)
    }
    /// True if the host entity is replaced by virtual entities and should not be shown.
    fn is_hidden(&self, host: EntityId) -> bool;
    /// The virtual entities that stand for a host (empty if shown as itself).
    fn virtual_of(&self, host: EntityId) -> Vec<EntityId>;
    /// Hash of the overlay content.
    fn content_hash(&self) -> u64;
}

/// Reference overlay.
#[derive(Clone, Debug, Default)]
pub struct SimpleOverlay {
    name: String,
    entities: Vec<VirtualEntity>,
    hidden: BTreeSet<EntityId>,
    next: BTreeMap<u8, u64>,
}

impl SimpleOverlay {
    /// Empty overlay.
    pub fn new(name: impl Into<String>) -> Self {
        Self { name: name.into(), ..Self::default() }
    }

    fn mint(&mut self, kind: EntityKind) -> Result<EntityId, TopoError> {
        let n = self.next.entry(kind as u8).or_insert(0);
        let id = EntityId::new(kind, VIRTUAL_SERIAL_BIT | *n).ok_or(TopoError::IdSpaceExhausted(kind))?;
        *n += 1;
        Ok(id)
    }

    fn check_host(view: &dyn TopologyView, h: EntityId) -> bool {
        match h.kind() {
            EntityKind::TopoFace => FaceId::from_entity(h).and_then(|f| view.face(f)).is_some_and(|r| r.state.is_active()),
            EntityKind::TopoEdge => EdgeId::from_entity(h).and_then(|f| view.edge(f)).is_some_and(|r| r.state.is_active()),
            _ => false,
        }
    }

    /// Present adjacent host faces (or edges) as one virtual entity. Hosts must be active, of one
    /// kind, not already hidden, and connected through shared edges (faces) or vertices (edges).
    pub fn composite(&mut self, view: &dyn TopologyView, hosts: &[EntityId]) -> Result<EntityId, TopoError> {
        let first = *hosts.first().ok_or(TopoError::Unsupported("composite of nothing"))?;
        if hosts.len() < 2 {
            return Err(TopoError::Unsupported("composite needs at least two hosts"));
        }
        for &h in hosts {
            if h.kind() != first.kind() || !Self::check_host(view, h) {
                return Err(TopoError::NotActive(h));
            }
            if self.hidden.contains(&h) {
                return Err(TopoError::InUse(h));
            }
        }
        // connectivity: union-find over shared sub-entities
        let keys: Vec<BTreeSet<u64>> = hosts
            .iter()
            .map(|&h| match h.kind() {
                EntityKind::TopoFace => FaceId::from_entity(h).and_then(|f| view.face(f)).map(|f| f.loops.iter().flat_map(|l| l.coedges.iter()).filter_map(|c| c.kind.edge()).map(|e| e.serial()).collect()).unwrap_or_default(),
                _ => EdgeId::from_entity(h).and_then(|e| view.edge(e)).map(|e| BTreeSet::from([e.start.serial(), e.end.serial()])).unwrap_or_default(),
            })
            .collect();
        let mut comp = vec![0usize];
        let mut grew = true;
        while grew {
            grew = false;
            for i in 0..hosts.len() {
                if !comp.contains(&i) && comp.iter().any(|&j| !keys[i].is_disjoint(&keys[j])) {
                    comp.push(i);
                    grew = true;
                }
            }
        }
        if comp.len() != hosts.len() {
            return Err(TopoError::NotCoincident("composite hosts are not connected".into()));
        }
        let mut sorted = hosts.to_vec();
        sorted.sort();
        let id = self.mint(first.kind())?;
        self.hidden.extend(sorted.iter().copied());
        self.entities.push(VirtualEntity { id, kind: VirtualKind::Composite, hosts: sorted, piece: 0 });
        Ok(id)
    }

    /// Present a host face or edge as `pieces` virtual entities (geometry of the pieces is the
    /// business of the caller; the overlay records identity only).
    pub fn partition(&mut self, view: &dyn TopologyView, host: EntityId, pieces: u32) -> Result<Vec<EntityId>, TopoError> {
        if pieces < 2 {
            return Err(TopoError::Unsupported("partition needs at least two pieces"));
        }
        if !Self::check_host(view, host) {
            return Err(TopoError::NotActive(host));
        }
        if self.hidden.contains(&host) {
            return Err(TopoError::InUse(host));
        }
        let mut ids = Vec::new();
        for piece in 0..pieces {
            let id = self.mint(host.kind())?;
            self.entities.push(VirtualEntity { id, kind: VirtualKind::Partition, hosts: vec![host], piece });
            ids.push(id);
        }
        self.hidden.insert(host);
        Ok(ids)
    }
}

impl VirtualOverlay for SimpleOverlay {
    fn name(&self) -> &str {
        &self.name
    }
    fn entities(&self) -> &[VirtualEntity] {
        &self.entities
    }
    fn is_hidden(&self, host: EntityId) -> bool {
        self.hidden.contains(&host)
    }
    fn virtual_of(&self, host: EntityId) -> Vec<EntityId> {
        self.entities.iter().filter(|e| e.hosts.contains(&host)).map(|e| e.id).collect()
    }
    fn content_hash(&self) -> u64 {
        let mut h = StableHasher::default();
        for e in &self.entities {
            h.write_u64(e.id.to_raw());
            h.write_u64(e.kind as u64);
            e.hosts.iter().for_each(|x| h.write_u64(x.to_raw()));
            h.write_u64(u64::from(e.piece));
        }
        h.finish()
    }
}

/// Faces as consumers see them through an overlay: visible host faces plus virtual faces, ascending.
pub fn visible_faces(view: &dyn TopologyView, overlay: &dyn VirtualOverlay) -> Vec<EntityId> {
    let mut v: Vec<EntityId> = view.faces().iter().map(|f| f.entity()).filter(|&e| !overlay.is_hidden(e)).collect();
    v.extend(overlay.entities().iter().map(|e| e.id).filter(|id| id.kind() == EntityKind::TopoFace));
    v.sort();
    v
}

/// Edges as consumers see them through an overlay.
pub fn visible_edges(view: &dyn TopologyView, overlay: &dyn VirtualOverlay) -> Vec<EntityId> {
    let mut v: Vec<EntityId> = view.edges().iter().map(|f| f.entity()).filter(|&e| !overlay.is_hidden(e)).collect();
    v.extend(overlay.entities().iter().map(|e| e.id).filter(|id| id.kind() == EntityKind::TopoEdge));
    v.sort();
    v
}
