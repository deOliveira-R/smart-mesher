//! Implementation-independent validation of any [`TopologyView`].
//!
//! The checks recompute expectations from the primary records (faces, shells, edges) and compare
//! them with the derived indices (rings, coedge lists), so a buggy cache cannot vouch for itself.

use crate::error::{Violation, ViolationCode};
use crate::ids::*;
use crate::model::*;
use crate::view::{TopologyView, coedge_ends};
use sm_core::EntityId;
use sm_geom_api::Sense;
use std::collections::BTreeMap;

fn v(code: ViolationCode, entity: impl Into<Option<EntityId>>, message: String) -> Violation {
    Violation { code, entity: entity.into(), message }
}

/// Check every invariant of the radial-edge model. Empty result means valid.
///
/// Checked: references are live; edge geometry matches its vertices within the model resolution;
/// loops are closed chains; face-use bookkeeping (owner, sense, shell membership) is consistent in
/// both directions; every shell is closed (each edge traversed equally often in both directions);
/// radial rings equal the expected multiset of edge uses and every wedge between neighbouring uses
/// has one consistent volume; body/volume membership; coincidence records.
pub fn validate(view: &dyn TopologyView) -> Vec<Violation> {
    let mut out = Vec::new();
    let tol = view.tolerance().resolution;

    for &eid in view.edges() {
        let Some(e) = view.edge(eid) else { continue };
        let (Some(sv), Some(ev)) = (view.vertex(e.start), view.vertex(e.end)) else {
            out.push(v(ViolationCode::DanglingReference, eid.entity(), "edge vertex missing".into()));
            continue;
        };
        if !sv.state.is_active() || !ev.state.is_active() {
            out.push(v(ViolationCode::DanglingReference, eid.entity(), "edge vertex retired".into()));
        }
        match view.shapes().curve(e.curve) {
            None => out.push(v(ViolationCode::DanglingReference, eid.entity(), "curve handle does not resolve".into())),
            Some(c) => {
                let (a, b) = (c.eval(e.t0), c.eval(e.t1));
                if (a - sv.point).norm() > tol || (b - ev.point).norm() > tol {
                    out.push(v(ViolationCode::EdgeGeometry, eid.entity(), "curve endpoints differ from vertices beyond resolution".into()));
                }
            }
        }
    }

    let mut expected_coedges: BTreeMap<EdgeId, Vec<CoedgeRef>> = BTreeMap::new();
    for &fid in view.faces() {
        let Some(f) = view.face(fid) else { continue };
        if view.shapes().surface(f.surface).is_none() && view.shapes().implicit(f.surface).is_none() {
            out.push(v(ViolationCode::DanglingReference, fid.entity(), "surface handle does not resolve".into()));
        }
        for (li, l) in f.loops.iter().enumerate() {
            let mut ends = Vec::new();
            for (ci, c) in l.coedges.iter().enumerate() {
                if let CoedgeKind::Edge { edge, .. } = c.kind {
                    if view.edge(edge).is_none_or(|e| !e.state.is_active()) {
                        out.push(v(ViolationCode::DanglingReference, fid.entity(), format!("loop {li} uses missing or retired edge")));
                    }
                    expected_coedges.entry(edge).or_default().push(CoedgeRef { face: fid, loop_index: li as u16, index: ci as u32 });
                }
                match coedge_ends(view, c) {
                    Some(p) => ends.push(p),
                    None => {
                        ends.clear();
                        break;
                    }
                }
            }
            let closed = !ends.is_empty() && (0..ends.len()).all(|i| ends[i].1 == ends[(i + 1) % ends.len()].0);
            if !closed {
                out.push(v(ViolationCode::LoopNotClosed, fid.entity(), format!("loop {li} is not a closed chain")));
            }
        }
        for side in FaceSide::BOTH {
            let u = f.uses[side.index()];
            if u.sense != side.sense() {
                out.push(v(ViolationCode::FaceUseMismatch, fid.entity(), format!("{side:?} use has wrong sense")));
            }
            if let UseOwner::Shell(sr) = u.owner {
                let fu = FaceUse { face: fid, side };
                let ok = view.volume(sr.volume).is_some_and(|vr| vr.state.is_active()) && view.shell(sr).is_some_and(|s| s.face_uses.contains(&fu));
                if !ok {
                    out.push(v(ViolationCode::FaceUseMismatch, fid.entity(), format!("{side:?} use owner shell does not list it")));
                }
            }
        }
    }

    // shells: reverse membership and closure
    let mut expected_wires: BTreeMap<EdgeId, Vec<EdgeUse>> = BTreeMap::new();
    for &vid in view.volumes() {
        let Some(vr) = view.volume(vid) else { continue };
        match view.body(vr.body) {
            Some(b) if b.state.is_active() && b.volumes.contains(&vid) => {}
            _ => out.push(v(ViolationCode::Membership, vid.entity(), "volume not listed by an active body".into())),
        }
        for (si, sh) in vr.shells.iter().enumerate() {
            let sref = ShellRef { volume: vid, index: si as u16 };
            let mut balance: BTreeMap<EdgeId, i32> = BTreeMap::new();
            for fu in &sh.face_uses {
                let Some(f) = view.face(fu.face).filter(|f| f.state.is_active()) else {
                    out.push(v(ViolationCode::DanglingReference, vid.entity(), format!("shell {si} lists a retired face")));
                    continue;
                };
                if f.uses[fu.side.index()].owner != UseOwner::Shell(sref) {
                    out.push(v(ViolationCode::FaceUseMismatch, vid.entity(), format!("shell {si} lists a use it does not own")));
                }
                for l in &f.loops {
                    for c in &l.coedges {
                        if let CoedgeKind::Edge { edge, sense } = c.kind {
                            let d = if fu.side == FaceSide::Negative { sense } else { sense.flip() };
                            *balance.entry(edge).or_insert(0) += if d == Sense::Forward { 1 } else { -1 };
                        }
                    }
                }
            }
            if let Some((e, _)) = balance.iter().find(|(_, b)| **b != 0) {
                out.push(v(ViolationCode::ShellNotClosed, vid.entity(), format!("shell {si}: edge {e:?} is not traversed equally in both directions")));
            }
            for &w in &sh.wires {
                expected_wires.entry(w).or_default().push(EdgeUse::Wire { edge: w, volume: Some(vid) });
            }
        }
    }
    for &bid in view.bodies() {
        let Some(b) = view.body(bid) else { continue };
        for &vid in &b.volumes {
            if view.volume(vid).is_none_or(|x| x.body != bid || !x.state.is_active()) {
                out.push(v(ViolationCode::Membership, bid.entity(), "body lists a volume that does not point back".into()));
            }
        }
        for &w in &b.wires {
            expected_wires.entry(w).or_default().push(EdgeUse::Wire { edge: w, volume: None });
        }
    }

    // rings
    for &eid in view.edges() {
        let mut want_c = expected_coedges.get(&eid).cloned().unwrap_or_default();
        want_c.sort();
        if view.edge_coedges(eid) != want_c.as_slice() {
            out.push(v(ViolationCode::RadialRing, eid.entity(), "edge_coedges disagrees with the faces".into()));
        }
        let mut want: Vec<EdgeUse> = want_c.iter().flat_map(|&c| FaceSide::BOTH.map(|side| EdgeUse::Coedge { coedge: c, side })).collect();
        want.extend(expected_wires.get(&eid).cloned().unwrap_or_default());
        want.sort();
        let ring = view.radial_ring(eid);
        let mut have = ring.to_vec();
        have.sort();
        if have != want {
            out.push(v(ViolationCode::RadialRing, eid.entity(), format!("ring has {} uses, expected {}", have.len(), want.len())));
            continue;
        }
        let n = ring.len();
        for i in 0..n {
            let (a, b) = (ring[i], ring[(i + 1) % n]);
            let mates = matches!((a, b), (EdgeUse::Coedge { coedge: x, .. }, EdgeUse::Coedge { coedge: y, side: _ }) if x == y);
            if !mates && n > 1 && view.edge_use_side(a) != view.edge_use_side(b) {
                out.push(v(ViolationCode::RadialRing, eid.entity(), format!("wedge between ring positions {i} and {} has two different owners", (i + 1) % n)));
            }
        }
    }

    for r in view.coincidences() {
        let ok = match r.status {
            Coincidence::Merged => view.face(r.a).is_some_and(|f| f.state.is_active()) && view.face(r.b).is_some_and(|f| matches!(f.state, EntityState::Merged(_))),
            _ => view.face(r.a).is_some() && view.face(r.b).is_some() && r.a < r.b,
        };
        if !ok {
            out.push(v(ViolationCode::Coincidence, r.a.entity(), format!("record ({:?}, {:?}) is inconsistent with the faces", r.a, r.b)));
        }
    }
    out
}
