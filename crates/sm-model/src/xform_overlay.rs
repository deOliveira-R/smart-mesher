//! Carrying the overlay's recorded geometry through a similarity (W4-VTOPO-FIX2).
//!
//! A virtual op stores absolute geometry: the chord curve of a face partition, the carrier
//! surface of a volume cut, the parameter of an edge partition. `Overlay::rebase` replays those
//! unchanged on the moved host, which would leave the virtual cells behind. The transform ops
//! therefore rewrite the recorded ops of every virtual op that touches a transformed cell
//! (directly, or through a virtual cell an earlier touching op created) before the overlay is
//! rebuilt on the transformed host.

use std::collections::{BTreeMap, BTreeSet};

use sm_core::EntityId;
use sm_geom_api::{CurveDef, Interval};
use sm_layer_api::ModelLayer;
use sm_op_api::OpError;
use sm_topo_api::{Carrier, VirtualOp};

use crate::xform::Xf;

/// Rewritten ops and the overlay id counters.
pub(crate) type Rewritten = (Vec<VirtualOp>, [u64; 3]);

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
            // Composites and collapses carry no absolute geometry (a collapse's gap is
            // recomputed on replay).
            other => other,
        });
    }
    Ok(any.then(|| (out, ov.data().next_serial)))
}
