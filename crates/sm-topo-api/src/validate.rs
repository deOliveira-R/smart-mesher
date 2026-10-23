//! Implementation-independent checks and the canonical content hash.

use crate::data::*;
use crate::ids::*;
use crate::lineage::LineageKind;
use crate::view::TopologyView;
use sm_core::{EntityId, EntityKind};
use sm_geom_api::{GeomStore, ParametricCurve};
use std::collections::BTreeMap;

/// A consistency problem found by [`validate`].
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum TopoIssue {
    /// `from` references `to`, which does not exist or was retired.
    Dangling {
        /// Referencing entity.
        from: EntityId,
        /// Missing target.
        to: EntityId,
    },
    /// A loop's coedges do not chain.
    LoopOpen {
        /// The face.
        face: FaceId,
        /// Loop index.
        loop_ix: usize,
        /// Coedge position where the chain breaks.
        at: usize,
    },
    /// A face has no loops or its first loop is not `Outer`.
    NoOuterLoop(FaceId),
    /// A face has more than two uses or two uses of equal sense.
    BadFaceUses(FaceId),
    /// A tolerance lies outside the policy range.
    ToleranceRange {
        /// The entity.
        entity: EntityId,
        /// Its tolerance.
        tol: f64,
    },
    /// A shell is not closed: the signed use count of `edge` is not zero.
    ShellOpen {
        /// The volume.
        volume: VolumeId,
        /// Shell index.
        shell: usize,
        /// The unbalanced edge.
        edge: EdgeId,
    },
    /// A volume's `body` does not list it.
    BodyMismatch(VolumeId),
}

/// Check structural invariants of any [`TopologyView`]: references alive, loops closed, outer
/// loop first, face uses sane, tolerances in range, shells closed (signed edge count zero,
/// which holds for baffles and cylinder seams). Result is ordered by entity id; empty = valid.
pub fn validate(v: &dyn TopologyView) -> Vec<TopoIssue> {
    let mut out = Vec::new();
    let pol = *v.policy();
    let tol_ok = |t: f64| t >= pol.linear_min && t <= pol.linear_max;
    let dangling = |from: EntityId, to: EntityId, alive: bool, out: &mut Vec<TopoIssue>| {
        if !alive {
            out.push(TopoIssue::Dangling { from, to });
        }
    };
    for id in v.entity_ids(EntityKind::TopoVertex) {
        if let Some(d) = VertexId::new(id).and_then(|i| v.vertex(i)).filter(|d| !tol_ok(d.tol)) {
            out.push(TopoIssue::ToleranceRange { entity: id, tol: d.tol });
        }
    }
    for id in v.entity_ids(EntityKind::TopoEdge) {
        let Some(e) = EdgeId::new(id).and_then(|i| v.edge(i)) else { continue };
        if !tol_ok(e.tol) {
            out.push(TopoIssue::ToleranceRange { entity: id, tol: e.tol });
        }
        for vx in [e.start, e.end] {
            dangling(id, vx.id(), v.is_alive(vx.id()), &mut out);
        }
    }
    for id in v.entity_ids(EntityKind::TopoFace) {
        let Some(fid) = FaceId::new(id) else { continue };
        let Some(f) = v.face(fid) else { continue };
        if !tol_ok(f.tol) {
            out.push(TopoIssue::ToleranceRange { entity: id, tol: f.tol });
        }
        if f.loops.first().map(|l| l.kind) != Some(LoopKind::Outer) || f.loops.iter().skip(1).any(|l| l.kind != LoopKind::Inner) {
            out.push(TopoIssue::NoOuterLoop(fid));
        }
        for (li, l) in f.loops.iter().enumerate() {
            for c in &l.coedges {
                dangling(id, c.edge.id(), v.is_alive(c.edge.id()), &mut out);
            }
            for i in 0..l.coedges.len() {
                let next = &l.coedges[(i + 1) % l.coedges.len()];
                let ok = match (v.coedge_vertices(&l.coedges[i]), v.coedge_vertices(next)) {
                    (Some((_, end)), Some((start, _))) => end == start,
                    _ => false,
                };
                if !ok {
                    out.push(TopoIssue::LoopOpen { face: fid, loop_ix: li, at: i + 1 });
                    break;
                }
            }
        }
        if v.face_role(fid) == Some(FaceRole::Invalid) {
            out.push(TopoIssue::BadFaceUses(fid));
        }
    }
    for id in v.entity_ids(EntityKind::TopoVolume) {
        let Some(vid) = VolumeId::new(id) else { continue };
        let Some(vol) = v.volume(vid) else { continue };
        if !v.body(vol.body).is_some_and(|b| b.volumes.contains(&vid)) {
            out.push(TopoIssue::BodyMismatch(vid));
        }
        for (si, sh) in vol.shells.iter().enumerate() {
            let mut balance: BTreeMap<EdgeId, i32> = BTreeMap::new();
            for u in &sh.uses {
                dangling(id, u.face.id(), v.is_alive(u.face.id()), &mut out);
                let Some(face) = v.face(u.face) else { continue };
                for c in face.loops.iter().flat_map(|l| l.coedges.iter()) {
                    if v.edge(c.edge).is_some_and(|e| !e.degenerate) {
                        *balance.entry(c.edge).or_insert(0) += c.sense.sign() * u.sense.sign();
                    }
                }
            }
            for (edge, n) in balance {
                if n != 0 {
                    out.push(TopoIssue::ShellOpen { volume: vid, shell: si, edge });
                }
            }
        }
    }
    out
}

/// Tolerance consistency against geometry: every edge end must lie within the edge's tolerance
/// plus both vertex tolerances of the curve end point. Returns offending edges.
pub fn check_edge_geometry(v: &dyn TopologyView, geom: &dyn GeomStore) -> Vec<EdgeId> {
    let mut bad = Vec::new();
    for id in v.entity_ids(EntityKind::TopoEdge) {
        let Some(e) = EdgeId::new(id).and_then(|i| v.edge(i)) else { continue };
        let Some(c) = e.curve.and_then(|c| geom.curve(c)) else { continue };
        for (t, vx) in [(e.range.lo, e.start), (e.range.hi, e.end)] {
            if let Some(vd) = v.vertex(vx) {
                let d = (c.eval(t) - vd.point).norm();
                if d > e.tol + vd.tol {
                    bad.push(e.id);
                    break;
                }
            }
        }
    }
    bad
}

/// FNV-1a accumulator.
struct Fnv(u64);

impl Fnv {
    fn bytes(&mut self, b: &[u8]) {
        for x in b {
            self.0 = (self.0 ^ u64::from(*x)).wrapping_mul(0x0100_0000_01b3);
        }
    }
    fn u64(&mut self, x: u64) {
        self.bytes(&x.to_le_bytes());
    }
    fn i64(&mut self, x: i64) {
        self.bytes(&x.to_le_bytes());
    }
    fn id(&mut self, x: impl Into<EntityId>) {
        self.u64(x.into().to_raw());
    }
    fn sense(&mut self, s: Sense) {
        self.u64(s.sign() as u64);
    }
}

/// Canonical content hash of a topology: ids, structure, senses, lineage, and quantized
/// coordinates/tolerances (quantum = `10 * linear_min` for coordinates, `linear_min` for
/// tolerances), so it matches across platforms. Same build -> same hash.
pub fn topology_hash(v: &dyn TopologyView) -> u64 {
    let pol = *v.policy();
    let qc = |x: f64| (x / (10.0 * pol.linear_min)).round() as i64;
    let qt = |x: f64| (x / pol.linear_min).round() as i64;
    let mut h = Fnv(0xcbf2_9ce4_8422_2325);
    for id in v.entity_ids(EntityKind::TopoVertex) {
        let Some(d) = VertexId::new(id).and_then(|i| v.vertex(i)) else { continue };
        h.id(id);
        [d.point.x, d.point.y, d.point.z].into_iter().for_each(|x| h.i64(qc(x)));
        h.i64(qt(d.tol));
    }
    for id in v.entity_ids(EntityKind::TopoEdge) {
        let Some(d) = EdgeId::new(id).and_then(|i| v.edge(i)) else { continue };
        h.id(id);
        h.u64(d.curve.map_or(u64::MAX, |c| c.id().to_raw()));
        h.i64(qc(d.range.lo));
        h.i64(qc(d.range.hi));
        h.id(d.start);
        h.id(d.end);
        h.i64(qt(d.tol));
        h.u64(u64::from(d.degenerate));
    }
    let coedges = |h: &mut Fnv, cs: &[Coedge]| {
        h.u64(cs.len() as u64);
        for c in cs {
            h.id(c.edge);
            h.sense(c.sense);
            h.u64(u64::from(c.pcurve.is_some()));
        }
    };
    for id in v.entity_ids(EntityKind::TopoFace) {
        let Some(d) = FaceId::new(id).and_then(|i| v.face(i)) else { continue };
        h.id(id);
        h.u64(d.surface.map_or(u64::MAX, |s| s.id().to_raw()));
        h.sense(d.surface_sense);
        h.i64(qt(d.tol));
        for l in &d.loops {
            h.u64(l.kind as u64);
            coedges(&mut h, &l.coedges);
        }
        d.embedded_edges.iter().for_each(|e| h.id(*e));
    }
    for id in v.entity_ids(EntityKind::TopoVolume) {
        let Some(d) = VolumeId::new(id).and_then(|i| v.volume(i)) else { continue };
        h.id(id);
        h.id(d.body);
        for s in &d.shells {
            h.u64(s.kind as u64);
            for u in &s.uses {
                h.id(u.face);
                h.sense(u.sense);
            }
        }
        d.wires.iter().for_each(|w| coedges(&mut h, &w.coedges));
    }
    for id in v.entity_ids(EntityKind::TopoBody) {
        let Some(d) = BodyId::new(id).and_then(|i| v.body(i)) else { continue };
        h.id(id);
        d.volumes.iter().for_each(|x| h.id(*x));
        d.free_wires.iter().for_each(|w| coedges(&mut h, &w.coedges));
    }
    for id in v.entity_ids(EntityKind::Instance) {
        let Some(d) = InstanceId::new(id).and_then(|i| v.instance(i)) else { continue };
        h.id(id);
        h.id(d.target.id);
        h.sense(d.target.sense);
        let t = &d.target.location.0;
        t.m.iter().flatten().for_each(|x| h.i64((x * 1e9).round() as i64));
        [t.t.x, t.t.y, t.t.z].into_iter().for_each(|x| h.i64(qc(x)));
        h.u64(d.parent.map_or(u64::MAX, |p| p.id().to_raw()));
    }
    for r in v.coincidences() {
        h.id(r.a);
        h.id(r.b);
        h.u64(r.status as u64);
        h.i64(r.overlap_area.map_or(-1, |a| (a / (pol.linear_min * pol.linear_min)).round() as i64));
    }
    for r in v.lineage().records() {
        h.u64(r.op.0);
        match &r.kind {
            LineageKind::Created { entity } => {
                h.u64(1);
                h.id(*entity);
            }
            LineageKind::Generated { from, into } => {
                h.u64(2);
                h.id(*from);
                into.iter().for_each(|x| h.id(*x));
            }
            LineageKind::Split { from, into } => {
                h.u64(3);
                h.id(*from);
                into.iter().for_each(|x| h.id(*x));
            }
            LineageKind::Merged { from, into } => {
                h.u64(4);
                from.iter().for_each(|x| h.id(*x));
                h.id(*into);
            }
            LineageKind::Modified { entity } => {
                h.u64(5);
                h.id(*entity);
            }
            LineageKind::Deleted { entity } => {
                h.u64(6);
                h.id(*entity);
            }
        }
    }
    h.0
}
