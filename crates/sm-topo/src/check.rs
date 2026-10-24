//! Invariant checks: the reference local checks (`sm_topo_api::check_cells_with`) fed by an
//! incremental shell-closure computation, a long-row pre-filter, and a parallel full `validate`.

use crate::store::TopoStore;
use rayon::prelude::*;
use sm_core::EntityId;
use sm_topo_api::{CellFlags, Dim, Incidence, Rule, Severity, ShellKind, Side, TopoIssue, TopologyView, check_cells, check_cells_with, check_row_entries};
use smallvec::SmallVec;
use std::collections::BTreeMap;

/// For every `(volume, ring)` that is not closed: its lowest edge with a non-zero net use, and
/// that net use.
pub(crate) type OpenShells = BTreeMap<(EntityId, u16), (EntityId, i64)>;

/// For long volume/body rows: the entries inserted in the open operation that are still in the
/// row, in row order. Rows with at least `min_len` entries are checked through these only (the
/// rest were checked when they were inserted).
pub(crate) struct Inserted {
    pub(crate) min_len: usize,
    pub(crate) map: BTreeMap<EntityId, Vec<Incidence>>,
}
fn v(cell: EntityId, rule: Rule, message: impl Into<String>) -> TopoIssue {
    TopoIssue::new(cell, rule, Severity::Error, message)
}

/// The entries of a long sorted volume/body row that need checking: those inserted in this
/// operation. `None` for any other cell, which goes through [`check_cells_with`] unchanged.
fn changed_entries<'a>(view: &'a dyn TopologyView, dim: Dim, c: EntityId, ins: &'a Inserted) -> Option<&'a [Incidence]> {
    (matches!(dim, Dim::Volume | Dim::Body) && view.boundary(c).len() >= ins.min_len).then(|| ins.map.get(&c).map_or(&[][..], Vec::as_slice))
}

/// The `end_op` local checks on the given cells (ascending) with the incremental shell closure.
///
/// Every cell goes through [`sm_topo_api::check_cells_with`], with `open` answering the shell
/// closure test, except long volume/body rows: those are a pre-filter that checks only the
/// entries inserted in this operation. Its results must equal `check_cells_with` on the same
/// cell (the paranoid cross-check in `end_op` and the differential tests enforce that), and it
/// emits a cell's row issues before its shell issues, like the full check.
pub(crate) fn check_local(view: &dyn TopologyView, ids: &[EntityId], open: &OpenShells, ins: &Inserted) -> Vec<TopoIssue> {
    let open_shell = |vol: EntityId, ring: u16| open.get(&(vol, ring)).copied();
    let mut out = Vec::new();
    let mut run = 0;
    for (i, &c) in ids.iter().enumerate() {
        let Some(dim) = view.dim_of(c) else { continue };
        let Some(changed) = changed_entries(view, dim, c, ins) else { continue };
        out.extend(check_cells_with(view, &ids[run..i], &open_shell));
        run = i + 1;
        out.extend(check_row_entries(view, dim, c, changed));
        if dim == Dim::Volume {
            check_shells_long(view, c, open, changed, &mut out);
        }
    }
    out.extend(check_cells_with(view, &ids[run..], &open_shell));
    out
}

/// `check_shells` for a long row: ring groups by binary search, the baffle test on the entries
/// inserted in this operation. Invariant: every place that sets a shell kind (`make_volume`,
/// `add_shell`, `split_volume`) also splices that ring's entries into the row, and every splice
/// into a volume/body row is appended to `ins_log`, so a ring whose kind changed has all its
/// entries in `changed`.
fn check_shells_long(view: &dyn TopologyView, vol: EntityId, open: &OpenShells, changed: &[Incidence], out: &mut Vec<TopoIssue>) {
    let row = view.boundary(vol);
    let mut pos = 0;
    while pos < row.len() {
        let ring = row[pos].ring;
        let end = pos + row[pos..].partition_point(|e| e.ring == ring);
        if ring != Incidence::EMBEDDED {
            match view.shell_kind(vol, ring) {
                None => out.push(v(vol, Rule::ShellKind, format!("shell {ring} has no kind"))),
                Some(ShellKind::Outer) if ring != 0 => out.push(v(vol, Rule::ShellKind, "only shell 0 can be Outer")),
                Some(ShellKind::Outer | ShellKind::Void) if changed.iter().any(|e| e.ring == ring && e.side == Side::Both) => {
                    out.push(v(vol, Rule::ShellKind, format!("baffle in closed shell {ring}")));
                }
                _ => {}
            }
            if let Some((edge, n)) = open.get(&(vol, ring)) {
                out.push(v(vol, Rule::ShellOpen, format!("shell {ring} leaves edge {edge:?} with net use {n}")));
            }
        }
        pos = end;
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
    ///
    /// The candidate edges are this operation's marks plus `open_edges`, the persistent carry-over
    /// of edges that had a non-zero net use at the last commit. Flag edits are checked at `end_op`,
    /// so committed states are closed and the carry-over is empty; it stays as a safeguard. Returns the
    /// open shells and the new carry-over, to be stored only if the operation commits.
    pub(crate) fn open_shells(&mut self) -> (OpenShells, std::collections::BTreeSet<u32>) {
        let mut slots = self.closure_edges.take_sorted();
        slots.extend(self.open_edges.iter().copied());
        slots.sort_unstable();
        slots.dedup();
        let mut carry = std::collections::BTreeSet::new();
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
                    carry.insert(s);
                    open.entry(key).or_insert((edge, n));
                }
            }
        }
        (open, carry)
    }

    /// The entries inserted into volume/body rows in the open operation that are still present,
    /// per cell in row order.
    pub(crate) fn inserted_entries(&self) -> Inserted {
        let mut map: BTreeMap<EntityId, Vec<Incidence>> = BTreeMap::new();
        for (id, e) in &self.ins_log {
            map.entry(*id).or_default().push(*e);
        }
        for (id, v) in &mut map {
            v.sort_by_key(|e| (e.ring, e.cell, e.side));
            v.dedup();
            let row = self.row(*id);
            v.retain(|e| row.binary_search_by_key(&(e.ring, e.cell, e.side), |x| (x.ring, x.cell, x.side)).is_ok());
        }
        Inserted { min_len: self.long_row, map }
    }

    /// The same result as [`sm_topo_api::validate()`] (issues sorted by `(cell, rule)`), computed
    /// in parallel over chunks of cells in a fixed order, so it is identical for any thread count.
    pub fn validate_par(&self) -> Vec<TopoIssue> {
        const CHUNK: usize = 512;
        let mut per_dim: Vec<&[EntityId]> = Vec::new();
        let mut ids: Vec<EntityId> = Vec::new();
        for dim in Dim::ALL {
            per_dim.push(self.cells(dim));
            ids.extend_from_slice(self.cells(dim));
        }
        let mut out: Vec<TopoIssue> = ids.par_chunks(CHUNK).map(|c| check_cells(self, c)).collect::<Vec<_>>().into_iter().flatten().collect();
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
