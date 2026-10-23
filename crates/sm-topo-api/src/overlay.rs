//! Virtual topology overlay: composite/partition cells layered over a host complex without
//! changing a single host entity (Cubit's virtual topology; docs/design/ir.md §5 "decomposition").
//!
//! An overlay is itself a [`TopologyView`], so meshers, semantics and tracking consume it exactly
//! like the host. Virtual cells have kind `PartitionCell`; their dimension is reported by
//! [`TopologyView::dim_of`]. Host cells that a virtual cell absorbs are *hidden* (not alive in the
//! overlay) but remain untouched in the host. Dropping the overlay restores the host view.

use crate::complex::canonical_rotate;
use crate::derived::TopologyViewExt;
use crate::error::TopoError;
use crate::types::{Carrier, CellFlags, Dim, FacePair, Incidence, LineageRecord, OpId, Side};
use crate::view::TopologyView;
use indexmap::IndexMap;
use sm_core::{EntityId, EntityKind};
use sm_geom_api::Tolerance;
use std::collections::{BTreeMap, BTreeSet};

/// Extra queries of a view that layers virtual cells over a host.
pub trait TopologyOverlay: TopologyView {
    /// The untouched host view.
    fn host(&self) -> &dyn TopologyView;
    /// Host cells a virtual cell stands for, ascending; empty for non-virtual ids.
    fn virtual_members(&self, id: EntityId) -> &[EntityId];
    /// The virtual cell covering a host cell, or the cell itself if it is visible.
    fn resolve(&self, host_cell: EntityId) -> EntityId;
}

#[derive(Clone, Debug)]
struct VirtualCell {
    dim: Dim,
    members: Vec<EntityId>,
    down: Vec<Incidence>,
    up: Vec<Incidence>,
    carrier: Carrier,
}

/// Reference overlay supporting composite faces (merge adjacent faces into one virtual face).
///
/// Partition (split) cells and edge composites follow the same pattern: a virtual cell, a hidden
/// set, and patched rows.
pub struct Overlay<'h> {
    host: &'h dyn TopologyView,
    virt: IndexMap<EntityId, VirtualCell>,
    hidden: BTreeSet<EntityId>,
    covering: BTreeMap<EntityId, EntityId>,
    patched_down: BTreeMap<EntityId, Vec<Incidence>>,
    patched_up: BTreeMap<EntityId, Vec<Incidence>>,
    live: [Vec<EntityId>; 5],
}

impl<'h> Overlay<'h> {
    /// An empty overlay over `host` (a view identical to the host).
    pub fn new(host: &'h dyn TopologyView) -> Self {
        let mut o = Self {
            host,
            virt: IndexMap::new(),
            hidden: BTreeSet::new(),
            covering: BTreeMap::new(),
            patched_down: BTreeMap::new(),
            patched_up: BTreeMap::new(),
            live: Default::default(),
        };
        o.rebuild_live();
        o
    }

    /// Drop all virtual cells, restoring the host view.
    pub fn clear(&mut self) {
        *self = Self::new(self.host);
    }

    fn rebuild_live(&mut self) {
        for dim in Dim::ALL {
            let mut l: Vec<EntityId> = self.host.cells(dim).iter().copied().filter(|c| !self.hidden.contains(c)).collect();
            l.extend(self.virt.iter().filter(|(_, v)| v.dim == dim).map(|(id, _)| *id));
            l.sort();
            self.live[dim.index()] = l;
        }
    }

    fn down_of(&self, id: EntityId) -> Vec<Incidence> {
        self.boundary(id).to_vec()
    }

    fn up_of(&self, id: EntityId) -> Vec<Incidence> {
        self.coboundary(id).to_vec()
    }

    /// Merge `faces` (host faces, at least two, connected through shared edges) into one virtual
    /// face. Edges shared between members disappear from the overlay; the virtual face's loops
    /// are the remaining coedges, chained head-to-tail (ring 0 holds the smallest edge id; a
    /// carrier-aware caller may reorder outer/inner). All members must have the same side and
    /// shell in each volume. Pinched boundaries and non-manifold interiors are `Unsupported`.
    pub fn composite_faces(&mut self, faces: &[EntityId]) -> Result<EntityId, TopoError> {
        let mut members: Vec<EntityId> = faces.to_vec();
        members.sort();
        members.dedup();
        if members.len() < 2 {
            return Err(TopoError::InvalidParameter("a composite needs at least two faces"));
        }
        for &f in &members {
            if self.host.dim_of(f) != Some(Dim::Face) || !self.is_alive(f) || self.virt.contains_key(&f) {
                return Err(TopoError::WrongDim { id: f, expected: Dim::Face });
            }
        }
        // Net coedges of the union; an edge shared by two members cancels.
        let mut uses: BTreeMap<EntityId, Vec<(EntityId, Side)>> = BTreeMap::new();
        for &f in &members {
            for c in self.host.coedges(f) {
                uses.entry(c.edge).or_default().push((f, c.sense.side()));
            }
        }
        let mut interior: Vec<EntityId> = Vec::new();
        let mut boundary: Vec<(EntityId, Side)> = Vec::new();
        for (edge, us) in &uses {
            let faces_here: BTreeSet<EntityId> = us.iter().map(|u| u.0).collect();
            if faces_here.len() >= 2 {
                let l = us.iter().filter(|u| u.1 == Side::Left).count();
                if l * 2 != us.len() {
                    return Err(TopoError::Unsupported("non-manifold composite interior"));
                }
                interior.push(*edge);
            } else {
                boundary.extend(us.iter().map(|u| (*edge, u.1)));
            }
        }
        if interior.is_empty() {
            return Err(TopoError::InvalidParameter("faces share no edge"));
        }
        // Chain the remaining coedges into loops.
        let ends = |e: EntityId, s: Side| -> Option<(EntityId, EntityId)> {
            let (a, b) = self.host.edge_ends(e)?;
            Some(if s == Side::Right { (b, a) } else { (a, b) })
        };
        let mut by_tail: BTreeMap<EntityId, usize> = BTreeMap::new();
        for (i, (e, s)) in boundary.iter().enumerate() {
            let (tail, _) = ends(*e, *s).ok_or(TopoError::WrongDim { id: *e, expected: Dim::Edge })?;
            if by_tail.insert(tail, i).is_some() {
                return Err(TopoError::Unsupported("composite boundary pinches at a vertex"));
            }
        }
        let mut used = vec![false; boundary.len()];
        let mut order: Vec<usize> = (0..boundary.len()).collect();
        order.sort_by_key(|&i| boundary[i]);
        let mut down: Vec<Incidence> = Vec::new();
        let mut ring = 0u16;
        for &start in &order {
            if used[start] {
                continue;
            }
            let mut cur = start;
            let mut loop_entries = Vec::new();
            loop {
                used[cur] = true;
                let (e, s) = boundary[cur];
                loop_entries.push(Incidence::new(e, s, ring));
                let (_, head) = ends(e, s).ok_or(TopoError::WrongDim { id: e, expected: Dim::Edge })?;
                let Some(&next) = by_tail.get(&head) else { return Err(TopoError::OpenLoop { ring: usize::from(ring), at: loop_entries.len() }) };
                if next == start {
                    break;
                }
                if used[next] {
                    return Err(TopoError::Unsupported("composite boundary is not a set of simple loops"));
                }
                cur = next;
            }
            canonical_rotate(&mut loop_entries);
            down.extend(loop_entries);
            ring += 1;
        }
        // Parents (volumes, bodies): same side/ring for every member.
        let mut parent_rows: BTreeMap<EntityId, (Side, u16)> = BTreeMap::new();
        for &f in &members {
            for p in self.coboundary(f) {
                match parent_rows.insert(p.cell, (p.side, p.ring)) {
                    Some(prev) if prev != (p.side, p.ring) => return Err(TopoError::Unsupported("members sit on different sides of a volume")),
                    _ => {}
                }
            }
        }
        // Commit.
        let serial = self.virt.len() as u64;
        let vid = EntityId::new(EntityKind::PartitionCell, serial).ok_or(TopoError::IdExhausted)?;
        let carrier = {
            let mut cs = members.iter().map(|&f| self.host.carrier(f));
            let first = cs.next().unwrap_or(Carrier::None);
            if cs.all(|c| c == first) { first } else { Carrier::None }
        };
        let vup: Vec<Incidence> = parent_rows.iter().map(|(p, (s, r))| Incidence::new(*p, *s, *r)).collect();
        for (&p, &(side, ring)) in &parent_rows {
            let mut row: Vec<Incidence> = self.down_of(p).into_iter().filter(|e| !members.contains(&e.cell)).collect();
            row.push(Incidence::new(vid, side, ring));
            row.sort_by_key(|e| (e.ring, e.cell, e.side));
            self.patched_down.insert(p, row);
        }
        for (e, s) in &boundary {
            let mut row: Vec<Incidence> = self.up_of(*e).into_iter().filter(|x| !members.contains(&x.cell)).collect();
            let ring_of = down.iter().find(|d| d.cell == *e && d.side == *s).map_or(0, |d| d.ring);
            row.push(Incidence::new(vid, *s, ring_of));
            row.sort();
            row.dedup();
            self.patched_up.insert(*e, row);
        }
        for &e in &interior {
            self.hidden.insert(e);
        }
        for &f in &members {
            self.hidden.insert(f);
            self.covering.insert(f, vid);
        }
        for &e in &interior {
            self.covering.insert(e, vid);
            for vx in [self.host.edge_ends(e).map(|x| x.0), self.host.edge_ends(e).map(|x| x.1)].into_iter().flatten() {
                let remaining: Vec<Incidence> = self.up_of(vx).into_iter().filter(|x| !self.hidden.contains(&x.cell)).collect();
                if remaining.is_empty() {
                    self.hidden.insert(vx);
                    self.covering.insert(vx, vid);
                } else {
                    self.patched_up.insert(vx, remaining);
                }
            }
        }
        self.virt.insert(vid, VirtualCell { dim: Dim::Face, members, down, up: vup, carrier });
        self.rebuild_live();
        Ok(vid)
    }
}

impl TopologyView for Overlay<'_> {
    fn model_tolerance(&self) -> Tolerance {
        self.host.model_tolerance()
    }
    fn dim_of(&self, id: EntityId) -> Option<Dim> {
        self.virt.get(&id).map(|v| v.dim).or_else(|| self.host.dim_of(id))
    }
    fn is_alive(&self, id: EntityId) -> bool {
        self.virt.contains_key(&id) || (!self.hidden.contains(&id) && self.host.is_alive(id))
    }
    fn cells(&self, dim: Dim) -> &[EntityId] {
        &self.live[dim.index()]
    }
    fn boundary(&self, id: EntityId) -> &[Incidence] {
        if let Some(v) = self.virt.get(&id) {
            return &v.down;
        }
        if self.hidden.contains(&id) {
            return &[];
        }
        self.patched_down.get(&id).map_or_else(|| self.host.boundary(id), |v| v.as_slice())
    }
    fn coboundary(&self, id: EntityId) -> &[Incidence] {
        if let Some(v) = self.virt.get(&id) {
            return &v.up;
        }
        if self.hidden.contains(&id) {
            return &[];
        }
        self.patched_up.get(&id).map_or_else(|| self.host.coboundary(id), |v| v.as_slice())
    }
    fn carrier(&self, id: EntityId) -> Carrier {
        self.virt.get(&id).map_or_else(|| self.host.carrier(id), |v| v.carrier)
    }
    fn flags(&self, id: EntityId) -> CellFlags {
        self.host.flags(id)
    }
    fn tolerance_override(&self, id: EntityId) -> Option<f64> {
        self.host.tolerance_override(id)
    }
    fn coincidences(&self) -> &[FacePair] {
        self.host.coincidences()
    }
    fn lineage(&self) -> &[LineageRecord] {
        self.host.lineage()
    }
    fn born(&self, id: EntityId) -> Option<OpId> {
        self.host.born(id)
    }
    fn revision(&self) -> OpId {
        self.host.revision()
    }
}

impl TopologyOverlay for Overlay<'_> {
    fn host(&self) -> &dyn TopologyView {
        self.host
    }
    fn virtual_members(&self, id: EntityId) -> &[EntityId] {
        self.virt.get(&id).map_or(&[], |v| v.members.as_slice())
    }
    fn resolve(&self, host_cell: EntityId) -> EntityId {
        self.covering.get(&host_cell).copied().unwrap_or(host_cell)
    }
}
