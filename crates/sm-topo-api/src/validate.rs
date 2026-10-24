//! Invariant checks that any [`TopologyView`] implementor (and any consumer) can run.

use crate::derived::TopologyViewExt;
use crate::types::{Carrier, CellFlags, Dim, Incidence, Region, Sense, ShellKind, Side};
use crate::view::TopologyView;
use sm_core::EntityId;
use std::collections::BTreeMap;

/// Which rule an issue breaks.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum Rule {
    /// An incidence has the wrong rank pair or side tag for its relation.
    BadIncidence,
    /// A row references a dead or unknown cell.
    DeadReference,
    /// An edge row is not `[start, end]` or a single `Both`, or a degenerate edge is malformed.
    EdgeEnds,
    /// Loop rings are not `0..n` in order, or a loop has `Both`/no coedges.
    RingOrder,
    /// A loop's coedges do not chain head-to-tail.
    LoopOpen,
    /// A face has two cells on the same side.
    FaceSides,
    /// The orientation rule (ir.md §6) is violated: a lone volume must be on the face's left,
    /// two volumes must be `left < right`.
    OrientationRule,
    /// A shell does not close: `boundary(boundary) != 0` (pole edges excluded).
    ShellOpen,
    /// A baffle (`Both`) sits in an `Outer` or `Void` shell, or a shell has no kind.
    ShellKind,
    /// Boundary and coboundary rows disagree.
    CoboundaryMismatch,
    /// A cell lacks the carrier its rank needs.
    MissingGeometry,
    /// Carrier geometry contradicts the topology beyond tolerance.
    GeometryMismatch,
}

/// How serious an issue is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Severity {
    /// The complex is invalid.
    Error,
    /// Suspicious but allowed.
    Warning,
}

/// One broken invariant.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub struct TopoIssue {
    /// The offending cell.
    pub cell: EntityId,
    /// The rule broken.
    pub rule: Rule,
    /// Severity.
    pub severity: Severity,
    /// Human-readable detail.
    pub message: String,
}

impl TopoIssue {
    /// An issue from its parts.
    pub fn new(cell: EntityId, rule: Rule, severity: Severity, message: impl Into<String>) -> Self {
        Self { cell, rule, severity, message: message.into() }
    }
}

fn v(cell: EntityId, rule: Rule, message: impl Into<String>) -> TopoIssue {
    TopoIssue { cell, rule, severity: Severity::Error, message: message.into() }
}

/// Check the combinatorial invariants of a complex. Empty result = valid.
///
/// Verified: incidence ranks and tags, live references, edge ends (and degenerate pole edges),
/// loop ring order and closure, at most one cell per face side, the orientation rule
/// (boundary faces have their volume on the left, interfaces are `left < right`), shell kinds
/// and shell closure (`boundary(boundary) = 0`, with baffles cancelling and pole edges
/// excluded), and boundary/coboundary agreement.
pub fn validate(view: &dyn TopologyView) -> Vec<TopoIssue> {
    let mut ids: Vec<EntityId> = Vec::new();
    for dim in Dim::ALL {
        ids.extend_from_slice(view.cells(dim));
    }
    let mut out = check_cells(view, &ids);
    check_coboundary(view, &mut out);
    out.sort_by_key(|a| (a.cell, a.rule));
    out
}

/// The local checks, on the given cells only (used by `end_op` on the touched cells).
pub(crate) fn check_cells(view: &dyn TopologyView, ids: &[EntityId]) -> Vec<TopoIssue> {
    let mut out = Vec::new();
    for &c in ids {
        let Some(dim) = view.dim_of(c) else { continue };
        check_row(view, dim, c, &mut out);
        match dim {
            Dim::Edge => check_edge(view, c, &mut out),
            Dim::Face => check_face(view, c, &mut out),
            Dim::Volume => check_shells(view, c, &mut out),
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

fn check_shells(view: &dyn TopologyView, vol: EntityId, out: &mut Vec<TopoIssue>) {
    for shell in view.shells(vol) {
        match view.shell_kind(vol, shell.ring) {
            None => out.push(v(vol, Rule::ShellKind, format!("shell {} has no kind", shell.ring))),
            Some(ShellKind::Outer) if shell.ring != 0 => out.push(v(vol, Rule::ShellKind, "only shell 0 can be Outer")),
            Some(ShellKind::Outer | ShellKind::Void) if shell.entries.iter().any(|e| e.side == Side::Both) => {
                out.push(v(vol, Rule::ShellKind, format!("baffle in closed shell {}", shell.ring)));
            }
            _ => {}
        }
        let mut count: BTreeMap<EntityId, i64> = BTreeMap::new();
        for u in shell.entries {
            let use_sign = match u.side {
                Side::Left => 1,
                Side::Right => -1,
                Side::Both => 0,
            };
            for c in view.coedges(u.cell) {
                if c.kind == crate::types::CoedgeKind::Pole {
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

fn check_coboundary(view: &dyn TopologyView, out: &mut Vec<TopoIssue>) {
    for dim in Dim::ALL {
        for &p in view.cells(dim) {
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
}

/// Check carriers against shape: vertices have points, edges have curves whose end points match
/// their vertices, edge samples lie on the surfaces of the faces that use them (all within the
/// per-entity tolerances), and CSG volumes only reference surfaces in the store. Virtual
/// (composite) faces and bodies are skipped.
pub fn validate_geometry(view: &dyn TopologyView) -> Vec<TopoIssue> {
    let store = view.shapes();
    let mut out = Vec::new();
    for &vx in view.cells(Dim::Vertex) {
        if !matches!(view.carrier(vx), Carrier::Point(_)) {
            out.push(v(vx, Rule::MissingGeometry, "vertex has no point"));
        }
    }
    for &e in view.cells(Dim::Edge) {
        if view.flags(e).contains(CellFlags::DEGENERATE) {
            continue;
        }
        let Carrier::Curve(cc) = view.carrier(e) else {
            out.push(v(e, Rule::MissingGeometry, "edge has no curve"));
            continue;
        };
        let Some(curve) = store.curve(cc.curve) else {
            out.push(v(e, Rule::MissingGeometry, format!("curve {:?} not in store", cc.curve)));
            continue;
        };
        let Some((s, t)) = view.edge_ends(e) else { continue };
        let (p_start, p_end) = if cc.sense == Sense::Forward { (cc.range.lo, cc.range.hi) } else { (cc.range.hi, cc.range.lo) };
        for (vx, par) in [(s, p_start), (t, p_end)] {
            if let Carrier::Point(p) = view.carrier(vx) {
                let d = (curve.eval(par) - *p).norm();
                if !view.tolerance().coincident(d, view.tolerance_of(e), view.tolerance_of(vx)) && d > view.tolerance_of(e) + view.tolerance_of(vx) {
                    out.push(v(e, Rule::GeometryMismatch, format!("curve end is {d:e} from vertex {vx:?}")));
                }
            }
        }
    }
    for &f in view.cells(Dim::Face) {
        let sc = match view.carrier(f) {
            Carrier::Surface(sc) => *sc,
            Carrier::Composite(_) => continue,
            _ => {
                out.push(v(f, Rule::MissingGeometry, "face has no surface"));
                continue;
            }
        };
        let Some(surf) = store.surface(sc.surface) else {
            if store.implicit(sc.surface).is_none() {
                out.push(v(f, Rule::MissingGeometry, format!("surface {:?} not in store", sc.surface)));
            }
            continue;
        };
        for c in view.coedges(f) {
            let Carrier::Curve(cc) = view.carrier(c.key.edge) else { continue };
            let Some(curve) = store.curve(cc.curve) else { continue };
            for k in 0..3 {
                let t = cc.range.lo + 0.5 * k as f64 * cc.range.len();
                let Ok(pr) = surf.project(curve.eval(t), None) else { continue };
                if pr.distance > view.tolerance_of(f) + view.tolerance_of(c.key.edge) {
                    out.push(v(f, Rule::GeometryMismatch, format!("edge {:?} is {:e} off the surface", c.key.edge, pr.distance)));
                    break;
                }
            }
        }
    }
    for &vol in view.cells(Dim::Volume) {
        if let Carrier::Csg(cell) = view.carrier(vol) {
            for h in &cell.halfspaces {
                if store.implicit(h.surface).is_none() {
                    out.push(v(vol, Rule::MissingGeometry, format!("CSG half-space surface {:?} has no implicit form", h.surface)));
                }
            }
        }
    }
    out
}

/// Euler characteristic `V - E + F` over the live cells, with degenerate (pole) edges left out,
/// so a sphere gives 2 and a solid brick 2.
pub fn euler_characteristic(view: &dyn TopologyView) -> i64 {
    let e = view.cells(Dim::Edge).iter().filter(|e| !view.flags(**e).contains(CellFlags::DEGENERATE)).count();
    view.cells(Dim::Vertex).len() as i64 - e as i64 + view.cells(Dim::Face).len() as i64
}
