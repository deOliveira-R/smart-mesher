//! Invariant checks that any [`TopologyView`] implementor (and any consumer) can run.

use crate::derived::TopologyViewExt;
use crate::types::{Carrier, CellFlags, Dim, Incidence, Sense, Side};
use crate::view::TopologyView;
use sm_core::EntityId;
use sm_geom_api::GeometryStore;
use std::collections::BTreeMap;

/// Which rule a [`Violation`] breaks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Rule {
    /// An incidence has the wrong rank pair or side tag for its relation.
    BadIncidence,
    /// A row references a dead or unknown cell.
    DeadReference,
    /// An edge row is not `[start, end]` or a single `Both`.
    EdgeEnds,
    /// Loop rings are not `0..n` in order, or a loop has `Both`/no coedges.
    RingOrder,
    /// A loop's coedges do not chain head-to-tail.
    LoopOpen,
    /// A face has two cells on the same side.
    FaceSides,
    /// A shell does not close: `boundary(boundary) != 0`.
    ShellOpen,
    /// Boundary and coboundary rows disagree.
    CoboundaryMismatch,
    /// A cell lacks the carrier its rank needs.
    MissingGeometry,
    /// Carrier geometry contradicts the topology beyond tolerance.
    GeometryMismatch,
}

/// One broken invariant.
#[derive(Clone, Debug, PartialEq)]
pub struct Violation {
    /// The offending cell.
    pub cell: EntityId,
    /// The rule broken.
    pub rule: Rule,
    /// Human-readable detail.
    pub message: String,
}

fn v(cell: EntityId, rule: Rule, message: impl Into<String>) -> Violation {
    Violation { cell, rule, message: message.into() }
}

/// Check the combinatorial invariants of a complex. Empty result = valid.
///
/// Verified: incidence ranks and tags, live references, edge ends, loop ring order and closure,
/// at most one cell per face side, shell closure (`boundary(boundary) = 0`, with baffles
/// cancelling), and boundary/coboundary agreement.
pub fn validate(view: &dyn TopologyView) -> Vec<Violation> {
    let mut out = Vec::new();
    for dim in Dim::ALL {
        for &c in view.cells(dim) {
            check_row(view, dim, c, &mut out);
        }
    }
    for &f in view.cells(Dim::Face) {
        check_face(view, f, &mut out);
    }
    for &e in view.cells(Dim::Edge) {
        if view.edge_ends(e).is_none() {
            out.push(v(e, Rule::EdgeEnds, "edge row is neither [start, end] nor a single Both"));
        }
    }
    for &vol in view.cells(Dim::Volume) {
        check_shells(view, vol, &mut out);
    }
    check_coboundary(view, &mut out);
    out
}

fn check_row(view: &dyn TopologyView, dim: Dim, c: EntityId, out: &mut Vec<Violation>) {
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
            Dim::Volume => (cd == Dim::Face && !embedded) || (cd < Dim::Face && embedded && e.side == Side::Both),
            Dim::Body => cd < Dim::Body && e.side == Side::Both,
        };
        if !ok {
            out.push(v(c, Rule::BadIncidence, format!("{dim:?} -> {cd:?} with side {:?}, ring {}", e.side, e.ring)));
        }
    }
}

fn tail_head(view: &dyn TopologyView, edge: EntityId, side: Side) -> Option<(EntityId, EntityId)> {
    let (s, e) = view.edge_ends(edge)?;
    Some(if side == Side::Right { (e, s) } else { (s, e) })
}

fn check_face(view: &dyn TopologyView, face: EntityId, out: &mut Vec<Violation>) {
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
}

fn check_shells(view: &dyn TopologyView, vol: EntityId, out: &mut Vec<Violation>) {
    for shell in view.shells(vol) {
        let mut count: BTreeMap<EntityId, i64> = BTreeMap::new();
        for u in shell.entries {
            let use_sign = match u.side {
                Side::Left => 1,
                Side::Right => -1,
                Side::Both => 0,
            };
            for c in view.coedges(u.cell) {
                let sign = if c.sense == Sense::Forward { 1 } else { -1 };
                *count.entry(c.edge).or_insert(0) += use_sign * sign;
            }
        }
        if let Some((edge, n)) = count.iter().find(|(_, n)| **n != 0) {
            out.push(v(vol, Rule::ShellOpen, format!("shell {} leaves edge {edge:?} with net use {n}", shell.ring)));
        }
    }
}

fn check_coboundary(view: &dyn TopologyView, out: &mut Vec<Violation>) {
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
/// their vertices, and edge samples lie on the surfaces of the faces that use them, all within
/// the per-entity tolerances.
pub fn validate_geometry(view: &dyn TopologyView, store: &dyn GeometryStore) -> Vec<Violation> {
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
                let d = (curve.eval(par) - p).norm();
                if d > view.tolerance(e) + view.tolerance(vx) {
                    out.push(v(e, Rule::GeometryMismatch, format!("curve end is {d:e} from vertex {vx:?}")));
                }
            }
        }
    }
    for &f in view.cells(Dim::Face) {
        let Carrier::Surface(sc) = view.carrier(f) else {
            out.push(v(f, Rule::MissingGeometry, "face has no surface"));
            continue;
        };
        let Some(surf) = store.surface(sc.surface) else {
            out.push(v(f, Rule::MissingGeometry, format!("surface {:?} not in store", sc.surface)));
            continue;
        };
        for c in view.coedges(f) {
            let Carrier::Curve(cc) = view.carrier(c.edge) else { continue };
            let Some(curve) = store.curve(cc.curve) else { continue };
            for k in 0..3 {
                let t = cc.range.lo + 0.5 * k as f64 * cc.range.len();
                let Ok(pr) = surf.project(curve.eval(t), None) else { continue };
                if pr.distance > view.tolerance(f) + view.tolerance(c.edge) {
                    out.push(v(f, Rule::GeometryMismatch, format!("edge {:?} is {:e} off the surface", c.edge, pr.distance)));
                    break;
                }
            }
        }
    }
    out
}
