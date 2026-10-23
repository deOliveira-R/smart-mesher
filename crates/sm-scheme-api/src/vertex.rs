//! Shared helpers for structured schemes: loop corners ([`CornerKey`]), logical sides from vertex
//! types, and edge meshes read in loop direction. Corner *picking* (choosing types globally) is a
//! service of `sm-sizing`; these helpers only interpret the types.

use crate::intent::VertexType;
use crate::ledger::Ledger;
use crate::target::{NodeKey, Target};
use sm_core::EntityId;
use sm_topo_api::{CoedgeKey, CornerKey, Side, TopologyView, TopologyViewExt};

/// One corner (vertex use) of a face loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoopCorner {
    /// Stable name (face + arriving coedge).
    pub key: CornerKey,
    /// The vertex.
    pub vertex: EntityId,
    /// The coedge leaving the corner.
    pub outgoing: CoedgeKey,
}

/// The corners of loop `ring` of `face`, in loop order: corner `i` is the tail of coedge `i`.
/// Pole coedges are skipped.
pub fn loop_corners(topo: &dyn TopologyView, face: EntityId, ring: u16) -> Vec<LoopCorner> {
    let coedges: Vec<CoedgeKey> = topo.coedges(face).filter(|c| c.ring == ring && c.kind == sm_topo_api::CoedgeKind::Edge).map(|c| c.key).collect();
    let n = coedges.len();
    let mut out = Vec::with_capacity(n);
    for (i, &out_key) in coedges.iter().enumerate() {
        let Some((s, t)) = topo.edge_ends(out_key.edge) else { continue };
        let vertex = if out_key.side == Side::Right { t } else { s };
        let incoming = coedges[(i + n - 1) % n];
        out.push(LoopCorner { key: CornerKey { face, incoming }, vertex, outgoing: out_key });
    }
    out
}

/// Logical sides of a loop: maximal runs of coedges between `End` corners (the corners of a
/// mapped face). Returns the sides in loop order, the first starting at the first `End` corner;
/// one side holding the whole loop if there is no `End` corner.
pub fn logical_sides(corners: &[LoopCorner], ty: impl Fn(&CornerKey) -> VertexType) -> Vec<Vec<CoedgeKey>> {
    let n = corners.len();
    let Some(first) = corners.iter().position(|c| ty(&c.key) == VertexType::End) else {
        return vec![corners.iter().map(|c| c.outgoing).collect()];
    };
    let mut sides: Vec<Vec<CoedgeKey>> = Vec::new();
    for k in 0..n {
        let c = &corners[(first + k) % n];
        if ty(&c.key) == VertexType::End || sides.is_empty() {
            sides.push(Vec::new());
        }
        if let Some(s) = sides.last_mut() {
            s.push(c.outgoing);
        }
    }
    sides
}

/// Sum of the quarter turns of a loop's vertex types (4 for an outer loop of a mapped face).
pub fn turn_sum(corners: &[LoopCorner], ty: impl Fn(&CornerKey) -> VertexType) -> i32 {
    corners.iter().map(|c| ty(&c.key).quarter_turns()).sum()
}

/// The committed nodes of a coedge's edge, in the coedge's direction (reversed for a `Right`
/// coedge), endpoints included.
pub fn coedge_nodes(ledger: &Ledger, k: CoedgeKey) -> Option<Vec<NodeKey>> {
    let mut v = ledger.curve(&Target::Topo(k.edge))?.to_vec();
    if k.side == Side::Right {
        v.reverse();
    }
    Some(v)
}

/// The committed nodes along a chain of coedges, endpoints included, without repeating the shared
/// vertex between consecutive coedges.
pub fn chain_nodes(ledger: &Ledger, chain: &[CoedgeKey]) -> Option<Vec<NodeKey>> {
    let mut out: Vec<NodeKey> = Vec::new();
    for &k in chain {
        let v = coedge_nodes(ledger, k)?;
        let skip = usize::from(!out.is_empty());
        out.extend_from_slice(&v[skip..]);
    }
    Some(out)
}
