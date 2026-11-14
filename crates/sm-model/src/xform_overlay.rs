//! Carrying the overlay's recorded geometry through a similarity (W4-VTOPO-FIX2).
//!
//! A virtual op stores absolute geometry: the chord curve of a face partition, the carrier
//! surface of a volume cut, the parameter of an edge partition. `Overlay::rebase` replays those
//! unchanged on the moved host, which would leave the virtual cells behind. The transform ops
//! therefore rewrite the recorded ops of every virtual op that touches a transformed cell
//! (directly, or through a virtual cell an earlier touching op created) before the overlay is
//! rebuilt on the transformed host.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use sm_core::EntityId;
use sm_geom_api::{CurveDef, Interval};
use sm_layer_api::ModelLayer;
use sm_op_api::OpError;
use sm_topo_api::{Carrier, Overlay, OverlayData, TopoError, TopologyView, VirtualOp};

use crate::xform::Xf;

/// Rewritten ops, the overlay id counters, and, per rewritten face partition (by op index),
/// the boundary edges that its first half had before the transform.
pub(crate) struct Rewritten {
    ops: Vec<VirtualOp>,
    serial: [u64; 3],
    first_half: BTreeMap<usize, BTreeSet<EntityId>>,
    /// The collapsed edges whose recorded gap was carried through the map.
    collapsed: Vec<EntityId>,
}

/// The edges on the boundary of a cell.
fn edges_of(view: &dyn TopologyView, cell: EntityId) -> BTreeSet<EntityId> {
    view.boundary(cell).iter().map(|i| i.cell).collect()
}

impl Rewritten {
    /// Build the overlay on the transformed `host`. A face partition gives its first half the
    /// loop arc that starts first in the host face's loop order; a transform that reverses or
    /// rotates the loop (a reflection) can hand that arc to the other half, so ids would name
    /// the mirror of the wrong half. Each partition is replayed and its halves swapped when the
    /// first half no longer has the edges it had before.
    pub(crate) fn build(self, host: Arc<dyn TopologyView>) -> Result<Overlay, OpError> {
        let collapsed = self.collapsed.clone();
        self.build_inner(host).map_err(|e| match (e, collapsed.first()) {
            // A scaled collapse whose gap now exceeds the tolerance cap.
            (TopoError::ToleranceTooLarge { wanted, max }, Some(edge)) => OpError::Invalid(format!(
                "the collapsed edge {edge} cannot follow the transform: its gap would grow to {wanted:e}, above linear_max {max:e}; remove the collapse first with model.remove_virtual"
            )),
            (e, _) => OpError::Failed(format!("the virtual topology cannot follow the transform: {e}")),
        })
    }

    fn build_inner(mut self, host: Arc<dyn TopologyView>) -> Result<Overlay, TopoError> {
        let rev = host.revision();
        for (&k, want) in &self.first_half {
            let prefix = OverlayData::new(rev, self.ops[..=k].to_vec(), self.serial);
            let ov = Overlay::from_data(host.clone(), prefix)?;
            let VirtualOp::PartitionFace { halves, .. } = &mut self.ops[k] else { continue };
            if edges_of(&ov, halves[0]) != *want {
                halves.swap(0, 1);
            }
        }
        Overlay::from_data(host, OverlayData::new(rev, self.ops, self.serial))
    }
}

/// The overlay's ops with the geometry of the ones that touch `closure` transformed by `xf`
/// (plus the id counters), or `None` if no op touches it (the plain rebase then applies).
pub(crate) fn transformed_ops(layer: &ModelLayer, closure: &[EntityId], xf: &Xf) -> Result<Option<Rewritten>, OpError> {
    let Some(ov) = layer.overlay() else { return Ok(None) };
    let view = layer.topo();
    let mut moved: BTreeSet<EntityId> = closure.iter().copied().collect();
    // Whether an edge's curve is a line (its parameter is arc length). A hidden edge has no
    // carrier in the view, so the host answers for it, and edges the recorded ops made are
    // followed through `lines`.
    let mut lines: BTreeMap<EntityId, bool> = BTreeMap::new();
    let host = ov.host();
    let carrier_line = |e: EntityId| {
        let c = match view.carrier(e) {
            Carrier::None => host.carrier(e),
            c => c,
        };
        matches!(c, Carrier::Curve(cc) if matches!(view.shapes().curve_def(cc.curve), Some(CurveDef::Line(_))))
    };
    let s = xf.scale;
    let mut any = false;
    let mut first_half = BTreeMap::new();
    let mut collapsed = Vec::new();
    let mut out = Vec::with_capacity(ov.ops().len());
    for op in ov.ops() {
        let touches = match op {
            VirtualOp::Composite { faces, .. } => faces.iter().any(|f| moved.contains(f)),
            VirtualOp::PartitionEdge { edge, .. } => moved.contains(edge),
            VirtualOp::PartitionFace { face, .. } => moved.contains(face),
            VirtualOp::PartitionVolume { volume, .. } => moved.contains(volume),
            VirtualOp::CompositeEdges { edges, .. } => edges.iter().any(|e| moved.contains(e)),
            VirtualOp::CollapseEdge { edge, .. } => moved.contains(edge),
            _ => false,
        };
        // Record the line-ness of the edges this op makes, touching or not.
        match op {
            VirtualOp::PartitionEdge { edge, halves, .. } => {
                let l = lines.get(edge).copied().unwrap_or_else(|| carrier_line(*edge));
                lines.extend(halves.iter().map(|&h| (h, l)));
            }
            VirtualOp::PartitionFace { cut, chord, .. } => {
                lines.insert(*chord, matches!(cut.curve, CurveDef::Line(_)));
            }
            VirtualOp::CollapseEdge { repointed, .. } => {
                for &(orig, copy) in repointed {
                    let l = lines.get(&orig).copied().unwrap_or_else(|| carrier_line(orig));
                    lines.insert(copy, l);
                }
            }
            _ => {}
        }
        if !touches {
            out.push(op.clone());
            continue;
        }
        any = true;
        if let VirtualOp::PartitionFace { halves, .. } = op {
            // The half's edges as they were when the op ran (later ops may hide it).
            let prefix = OverlayData::new(ov.data().host_revision, ov.ops()[..=out.len()].to_vec(), ov.data().next_serial);
            let before = Overlay::from_data(ov.host_arc(), prefix).map_err(|e| OpError::Failed(format!("virtual topology replay: {e}")))?;
            first_half.insert(out.len(), edges_of(&before, halves[0]));
        }
        moved.extend(op.created());
        out.push(match op.clone() {
            VirtualOp::PartitionEdge { edge, param, vertex, halves } => {
                // A line's parameter is arc length: it scales with the map.
                let param = if lines.get(&edge).copied().unwrap_or_else(|| carrier_line(edge)) { param * s } else { param };
                VirtualOp::PartitionEdge { edge, param, vertex, halves }
            }
            VirtualOp::PartitionFace { face, mut cut, chord, halves } => {
                let line = matches!(cut.curve, CurveDef::Line(_));
                cut.curve = cut.curve.transformed(&xf.t).map_err(|e| OpError::Failed(format!("virtual chord {chord}: {e}")))?;
                if line {
                    cut.range = Interval::new(cut.range.lo * s, cut.range.hi * s);
                }
                VirtualOp::PartitionFace { face, cut, chord, halves }
            }
            VirtualOp::PartitionVolume { volume, mut cut, face, children } => {
                cut.surface = cut.surface.transformed(&xf.t).map_err(|e| OpError::Failed(format!("virtual cut face {face}: {e}")))?.0;
                VirtualOp::PartitionVolume { volume, cut, face, children }
            }
            // A collapse's recorded gap is a length: it scales with the map (replay compares
            // it with the recomputed gap, so a stale gap would be refused).
            VirtualOp::CollapseEdge { edge, into_vertex, gap, repointed } => {
                collapsed.push(edge);
                VirtualOp::CollapseEdge { edge, into_vertex, gap: gap * s, repointed }
            }
            // Composites carry no absolute geometry.
            other => other,
        });
    }
    Ok(any.then(|| Rewritten { ops: out, serial: ov.data().next_serial, first_half, collapsed }))
}
