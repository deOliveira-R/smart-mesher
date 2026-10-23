//! Copy jobs (deck pairs, copy-mesh, mirror): entity correspondence, the built-in copy
//! execution, and the post-commit check for deck faces written by another job.
//!
//! Orientation: the model's deck sign rule is `n_B = -g(n_A)` (sm-model-api decision 3), so
//! under a proper transform the image of an A loop runs against B's normal and is reversed; under
//! an improper transform the reflection already reverses it.

use crate::error::{FailureKind, SchemeFailure};
use crate::job::{CopySpec, EntityMap};
use crate::ledger::Ledger;
use crate::patch::{MeshPatch, PatchBuilder};
use crate::target::{NodeKey, Target};
use sm_core::{EntityId, EntityKind, Point3, Transform};
use sm_mesh_api::Param;
use sm_topo_api::{Carrier, Sense, TopologyView, TopologyViewExt};
use std::collections::BTreeSet;

const COPY: &str = "copy";

fn point_of(topo: &dyn TopologyView, v: EntityId) -> Option<Point3> {
    match topo.carrier(v) {
        Carrier::Point(p) => Some(*p),
        _ => None,
    }
}

fn dist(a: Point3, b: Point3) -> f64 {
    (a - b).norm()
}

/// Vertices of a face (ends of its edges), ascending.
pub fn face_vertices(topo: &dyn TopologyView, f: EntityId) -> Vec<EntityId> {
    let mut v: BTreeSet<EntityId> = BTreeSet::new();
    for &e in topo.face_edges(f) {
        if let Some((s, t)) = topo.edge_ends(e) {
            v.insert(s);
            v.insert(t);
        }
    }
    v.into_iter().collect()
}

/// Correspondence of face `a` to face `b` under `t`: vertices by position (within `tol`),
/// edges by mapped end vertices (`Reversed` if `b`'s edge runs the other way), and `a -> b`.
/// Returns `None` if some vertex or edge of `a` has no image in `b`. When several edges of `b`
/// share the mapped ends, the lowest id is taken.
pub fn entity_map(topo: &dyn TopologyView, a: EntityId, b: EntityId, t: &Transform, tol: f64) -> Option<EntityMap> {
    let mut m = EntityMap::default();
    let vb = face_vertices(topo, b);
    for va in face_vertices(topo, a) {
        let p = t.apply_point(point_of(topo, va)?);
        let img = vb.iter().copied().find(|&v| point_of(topo, v).is_some_and(|q| dist(p, q) <= tol))?;
        m.pairs.insert(va, (img, Sense::Forward));
    }
    for &ea in topo.face_edges(a) {
        let (s, e) = topo.edge_ends(ea)?;
        let (s2, e2) = (m.get(s)?.0, m.get(e)?.0);
        let hit = topo.face_edges(b).iter().copied().find_map(|eb| match topo.edge_ends(eb) {
            Some((x, y)) if x == s2 && y == e2 => Some((eb, Sense::Forward)),
            Some((x, y)) if x == e2 && y == s2 => Some((eb, Sense::Reversed)),
            _ => None,
        })?;
        m.pairs.insert(ea, hit);
    }
    m.pairs.insert(a, (b, Sense::Forward));
    Some(m)
}

/// Image of a source node key under a copy, given the source edge lengths from the ledger.
fn map_key(ledger: &Ledger, spec: &CopySpec, k: &NodeKey) -> Option<NodeKey> {
    if k.owner == spec.source {
        return Some(NodeKey::interior(spec.target, k.local));
    }
    let id = k.owner.topo()?;
    let (img, sense) = spec.map.get(id)?;
    match id.kind() {
        EntityKind::TopoVertex | EntityKind::PartitionVertex => Some(NodeKey::vertex(img)),
        EntityKind::TopoEdge | EntityKind::PartitionEdge => {
            let n = ledger.curve(&k.owner)?.len() as u32 - 1;
            Some(NodeKey::edge(img, if sense == Sense::Forward { k.local } else { n - k.local }))
        }
        _ => None,
    }
}

/// Execute a copy job: images of the claimed edges, then of the face (or edge) mesh.
pub fn copy_patch(ledger: &Ledger, spec: &CopySpec) -> Result<MeshPatch, SchemeFailure> {
    let fail = |m: String| SchemeFailure::fatal(COPY, &[spec.target], FailureKind::Topology, m);
    let mut b = PatchBuilder::new();
    let t = &spec.transform;
    // Claimed edges: build them in the target edge's own direction.
    for &eb in &spec.edges {
        let Some((&ea, &(_, sense))) = spec.map.pairs.iter().find(|(_, (img, _))| *img == eb) else {
            return Err(fail(format!("no source edge for {eb:?}")));
        };
        let src = ledger.curve(&Target::Topo(ea)).ok_or_else(|| fail(format!("source edge {ea:?} not meshed")))?;
        let n = src.len() - 1;
        let mut list = Vec::with_capacity(src.len());
        for j in 0..=n {
            let sj = if sense == Sense::Forward { j } else { n - j };
            let sk = &src[sj];
            let key = map_key(ledger, spec, sk).ok_or_else(|| fail(format!("no image for {sk:?}")))?;
            let i = if key.owner == Target::Topo(eb) {
                let p = ledger.point(sk).ok_or_else(|| fail(format!("{sk:?} not committed")))?;
                b.new_node(key, t.apply_point(p), Param::None)
            } else {
                b.reference(key)
            };
            list.push(i);
        }
        b.curve(Target::Topo(eb), &list);
    }
    if let Some(fm) = ledger.face(&spec.source) {
        b.begin_face(spec.target);
        let reverse = !t.is_improper();
        let mut el: Vec<u32> = Vec::with_capacity(4);
        for k in 0..fm.len() {
            el.clear();
            for sk in fm.element(k) {
                let key = map_key(ledger, spec, sk).ok_or_else(|| fail(format!("no image for {sk:?}")))?;
                let i = match b.local(&key) {
                    Some(i) => i,
                    None if key.owner == spec.target => {
                        let p = ledger.point(sk).ok_or_else(|| fail(format!("{sk:?} not committed")))?;
                        b.new_node(key, t.apply_point(p), Param::None)
                    }
                    None => b.reference(key),
                };
                el.push(i);
            }
            if reverse {
                el.reverse();
            }
            b.face(&el);
        }
    }
    Ok(b.finish())
}

/// True if the committed mesh of `target` is the image of `source` under `t`: same element
/// count, and every element's mapped point set matches an element of the target within `tol`.
pub fn verify_copy(ledger: &Ledger, source: &Target, target: &Target, t: &Transform, tol: f64) -> bool {
    let (Some(a), Some(b)) = (ledger.face(source), ledger.face(target)) else { return false };
    if a.len() != b.len() {
        return false;
    }
    let q = |p: Point3| -> [i64; 3] { [(p.x / tol).round() as i64, (p.y / tol).round() as i64, (p.z / tol).round() as i64] };
    let centroid = |el: &[NodeKey], map: bool| -> Option<[i64; 3]> {
        let mut c = [0.0; 3];
        for k in el {
            let p = ledger.point(k)?;
            let p = if map { t.apply_point(p) } else { p };
            c[0] += p.x;
            c[1] += p.y;
            c[2] += p.z;
        }
        let n = el.len() as f64;
        Some(q(Point3::new(c[0] / n, c[1] / n, c[2] / n)))
    };
    let mut ca: Vec<[i64; 3]> = Vec::with_capacity(a.len());
    let mut cb: Vec<[i64; 3]> = Vec::with_capacity(b.len());
    for k in 0..a.len() {
        match (centroid(a.element(k), true), centroid(b.element(k), false)) {
            (Some(x), Some(y)) => {
                ca.push(x);
                cb.push(y);
            }
            _ => return false,
        }
    }
    ca.sort_unstable();
    cb.sort_unstable();
    ca == cb
}
