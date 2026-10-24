//! Invariant checks: the reference local checks (`end_op`) ported to work on rows directly, an
//! incremental shell-closure computation, and a parallel full `validate`.

use crate::store::TopoStore;
use rayon::prelude::*;
use sm_core::EntityId;
use sm_topo_api::{CellFlags, Carrier, CoedgeKind, Dim, Incidence, Region, Rule, Sense, Severity, ShellKind, Side, TopoIssue, TopologyView, TopologyViewExt};
use smallvec::SmallVec;
use std::collections::BTreeMap;

/// For every `(volume, ring)` that is not closed: its lowest edge with a non-zero net use, and
/// that net use.
pub(crate) type OpenShells = BTreeMap<(EntityId, u16), (EntityId, i64)>;

fn v(cell: EntityId, rule: Rule, message: impl Into<String>) -> TopoIssue {
    TopoIssue::new(cell, rule, Severity::Error, message)
}

/// The local checks on the given cells (ascending). With `open` the shell closure comes from the
/// incremental computation; without it every shell is walked (the reference algorithm).
pub(crate) fn check_cells(view: &dyn TopologyView, ids: &[EntityId], open: Option<&OpenShells>) -> Vec<TopoIssue> {
    let mut out = Vec::new();
    for &c in ids {
        let Some(dim) = view.dim_of(c) else { continue };
        check_row(view, dim, c, &mut out);
        match dim {
            Dim::Edge => check_edge(view, c, &mut out),
            Dim::Face => check_face(view, c, &mut out),
            Dim::Volume => check_shells(view, c, open, &mut out),
            _ => {}
        }
    }
    out
}

fn check_row(view: &dyn TopologyView, dim: Dim, c: EntityId, out: &mut Vec<TopoIssue>) {
    for e in view.boundary(c) {
        let Some(cd) = view.dim_of(e.cell) else {
            out.push(v(c, Rule::DeadReference, format!("unknown cell {:?}", e.cell)));
            continue;
        };
        if !view.is_alive(e.cell) {
            out.push(v(c, Rule::DeadReference, format!("dead cell {:?}", e.cell)));
            continue;
        }
        let embedded = e.ring == Incidence::EMBEDDED;
        let ok = match dim {
            Dim::Vertex => false,
            Dim::Edge => cd == Dim::Vertex && !embedded,
            Dim::Face => (cd == Dim::Edge && (embedded && e.side == Side::Both || !embedded && e.side != Side::Both)) || (cd == Dim::Vertex && embedded && e.side == Side::Both),
            Dim::Volume => (cd == Dim::Face && !embedded) || (cd <= Dim::Face && embedded && e.side == Side::Both),
            Dim::Body => cd < Dim::Body && e.side == Side::Both,
        };
        if !ok {
            out.push(v(c, Rule::BadIncidence, format!("{dim:?} -> {cd:?} with side {:?}, ring {}", e.side, e.ring)));
        }
    }
}

fn check_edge(view: &dyn TopologyView, e: EntityId, out: &mut Vec<TopoIssue>) {
    let Some((s, t)) = view.edge_ends(e) else {
        out.push(v(e, Rule::EdgeEnds, "edge row is neither [start, end] nor a single Both"));
        return;
    };
    if view.flags(e).contains(CellFlags::DEGENERATE) && (s != t || matches!(view.carrier(e), Carrier::Curve(_))) {
        out.push(v(e, Rule::EdgeEnds, "a degenerate (pole) edge must be closed and curve-less"));
    }
}

fn tail_head(view: &dyn TopologyView, edge: EntityId, side: Side) -> Option<(EntityId, EntityId)> {
    let (s, e) = view.edge_ends(edge)?;
    Some(if side == Side::Right { (e, s) } else { (s, e) })
}

fn check_face(view: &dyn TopologyView, face: EntityId, out: &mut Vec<TopoIssue>) {
    let loops: Vec<_> = view.loops(face).collect();
    if loops.is_empty() {
        out.push(v(face, Rule::RingOrder, "face has no loop"));
    }
    for (i, l) in loops.iter().enumerate() {
        if usize::from(l.ring) != i {
            out.push(v(face, Rule::RingOrder, format!("loop {i} has ring {}", l.ring)));
        }
        for (k, c) in l.entries.iter().enumerate() {
            let next = &l.entries[(k + 1) % l.entries.len()];
            match (tail_head(view, c.cell, c.side), tail_head(view, next.cell, next.side)) {
                (Some((_, head)), Some((tail, _))) if head == tail => {}
                (Some(_), Some(_)) => out.push(v(face, Rule::LoopOpen, format!("loop {i} breaks after coedge {k}"))),
                _ => {} // EdgeEnds reports it
            }
        }
    }
    let (mut lefts, mut rights) = (0, 0);
    for e in view.coboundary(face) {
        if view.dim_of(e.cell) == Some(Dim::Volume) {
            lefts += usize::from(e.side != Side::Right);
            rights += usize::from(e.side != Side::Left);
        }
    }
    if lefts > 1 || rights > 1 {
        out.push(v(face, Rule::FaceSides, format!("{lefts} cells on the left, {rights} on the right")));
    }
    if let Some(s) = view.face_sides(face) {
        match (s.left, s.right) {
            (Region::Exterior, Region::Volume(_)) => out.push(v(face, Rule::OrientationRule, "boundary face has its volume on the right (normal points into it)")),
            (Region::Volume(l), Region::Volume(r)) if l > r => out.push(v(face, Rule::OrientationRule, "interface face must have the lower volume id on its left")),
            _ => {}
        }
    }
}

fn check_shells(view: &dyn TopologyView, vol: EntityId, open: Option<&OpenShells>, out: &mut Vec<TopoIssue>) {
    for shell in view.shells(vol) {
        match view.shell_kind(vol, shell.ring) {
            None => out.push(v(vol, Rule::ShellKind, format!("shell {} has no kind", shell.ring))),
            Some(ShellKind::Outer) if shell.ring != 0 => out.push(v(vol, Rule::ShellKind, "only shell 0 can be Outer")),
            Some(ShellKind::Outer | ShellKind::Void) if shell.entries.iter().any(|e| e.side == Side::Both) => {
                out.push(v(vol, Rule::ShellKind, format!("baffle in closed shell {}", shell.ring)));
            }
            _ => {}
        }
        if let Some(open) = open {
            if let Some((edge, n)) = open.get(&(vol, shell.ring)) {
                out.push(v(vol, Rule::ShellOpen, format!("shell {} leaves edge {edge:?} with net use {n}", shell.ring)));
            }
            continue;
        }
        let mut count: BTreeMap<EntityId, i64> = BTreeMap::new();
        for u in shell.entries {
            let use_sign = match u.side {
                Side::Left => 1,
                Side::Right => -1,
                Side::Both => 0,
            };
            for c in view.coedges(u.cell) {
                if c.kind == CoedgeKind::Pole {
                    continue;
                }
                let sign = if c.sense() == Sense::Forward { 1 } else { -1 };
                *count.entry(c.key.edge).or_insert(0) += use_sign * sign;
            }
        }
        if let Some((edge, n)) = count.iter().find(|(_, n)| **n != 0) {
            out.push(v(vol, Rule::ShellOpen, format!("shell {} leaves edge {edge:?} with net use {n}", shell.ring)));
        }
    }
}

fn check_coboundary_of(view: &dyn TopologyView, dim: Dim, ids: &[EntityId], out: &mut Vec<TopoIssue>) {
    let _ = dim;
    for &p in ids {
        for e in view.boundary(p) {
            let back = Incidence { cell: p, side: e.side, ring: e.ring };
            if !view.coboundary(e.cell).contains(&back) {
                out.push(v(p, Rule::CoboundaryMismatch, format!("{:?} does not list parent {p:?}", e.cell)));
            }
        }
        for e in view.coboundary(p) {
            let fwd = Incidence { cell: p, side: e.side, ring: e.ring };
            if !view.boundary(e.cell).contains(&fwd) {
                out.push(v(p, Rule::CoboundaryMismatch, format!("lists parent {:?} whose row lacks it", e.cell)));
            }
        }
    }
}

impl TopoStore {
    /// Shells left open by the edges whose net use may have changed in the open operation: for
    /// each such edge, the net face-use count per `(volume, ring)`, read from the coboundary
    /// rows of the edge and of its faces. Consumes the closure marks.
    pub(crate) fn open_shells(&mut self) -> OpenShells {
        let slots = self.closure_edges.take_sorted();
        let mut open = OpenShells::new();
        for s in slots {
            let slot = s as usize;
            if slot >= self.t[Dim::Edge.index()].state.len() || self.t[Dim::Edge.index()].state[slot] != sm_topo_api::EntityState::Live {
                continue;
            }
            if self.t[Dim::Edge.index()].flags[slot].contains(CellFlags::DEGENERATE) {
                continue;
            }
            let Some(edge) = EntityId::new(Dim::Edge.kind(), u64::from(s) + 1) else { continue };
            let mut nets: SmallVec<[((EntityId, u16), i64); 4]> = SmallVec::new();
            for fe in self.t[Dim::Edge.index()].up.row(slot) {
                if fe.cell.kind() != Dim::Face.kind() || fe.ring == Incidence::EMBEDDED {
                    continue;
                }
                let sign = if fe.side == Side::Right { -1 } else { 1 };
                for ve in self.up(fe.cell) {
                    if ve.cell.kind() != Dim::Volume.kind() || ve.ring == Incidence::EMBEDDED {
                        continue;
                    }
                    let use_sign = match ve.side {
                        Side::Left => 1,
                        Side::Right => -1,
                        Side::Both => 0,
                    };
                    let key = (ve.cell, ve.ring);
                    match nets.iter_mut().find(|(k, _)| *k == key) {
                        Some((_, n)) => *n += use_sign * sign,
                        None => nets.push((key, use_sign * sign)),
                    }
                }
            }
            for (key, n) in nets {
                if n != 0 {
                    open.entry(key).or_insert((edge, n));
                }
            }
        }
        open
    }

    /// The `end_op` local checks with the incremental shell closure.
    pub(crate) fn check_local(&self, ids: &[EntityId], open: &OpenShells) -> Vec<TopoIssue> {
        check_cells(self, ids, Some(open))
    }

    /// The same result as [`sm_topo_api::validate`] (issues sorted by `(cell, rule)`), computed
    /// in parallel over chunks of cells in a fixed order, so it is identical for any thread count.
    pub fn validate_par(&self) -> Vec<TopoIssue> {
        const CHUNK: usize = 512;
        let mut per_dim: Vec<&[EntityId]> = Vec::new();
        let mut ids: Vec<EntityId> = Vec::new();
        for dim in Dim::ALL {
            per_dim.push(self.cells(dim));
            ids.extend_from_slice(self.cells(dim));
        }
        let mut out: Vec<TopoIssue> = ids.par_chunks(CHUNK).map(|c| check_cells(self, c, None)).collect::<Vec<_>>().into_iter().flatten().collect();
        for (d, cells) in Dim::ALL.into_iter().zip(per_dim) {
            let parts: Vec<Vec<TopoIssue>> = cells
                .par_chunks(CHUNK)
                .map(|c| {
                    let mut o = Vec::new();
                    check_coboundary_of(self, d, c, &mut o);
                    o
                })
                .collect();
            out.extend(parts.into_iter().flatten());
        }
        out.sort_by_key(|a| (a.cell, a.rule));
        out
    }
}
