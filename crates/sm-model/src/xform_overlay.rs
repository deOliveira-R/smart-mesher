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
    /// The collapsed edges with their recorded gap carried through the map (the scaled gap).
    collapsed: Vec<(EntityId, f64)>,
}

/// The edges on the boundary of a cell.
fn edges_of(view: &dyn TopologyView, cell: EntityId) -> BTreeSet<EntityId> {
    view.boundary(cell).iter().map(|i| i.cell).collect()
}

/// Replays an op record one op at a time on a fresh overlay, through the overlay's own ops, so
/// the state after each op can be read: one pass of `n` ops instead of a replay of every prefix
/// (which is quadratic in the number of touched partitions, and each replay is itself
/// superlinear). The overlay mints ids from its counters, so the pass is valid only while the
/// record's ids are the ones the counters give (no gaps from removed ops); a mismatch, a refused
/// op or an op kind it does not know makes [`Stepper::step`] fail and the caller falls back to
/// the prefix replays.
struct Stepper {
    ov: Overlay,
    /// Face ids of the record -> face ids the stepper minted (they differ where a partition's
    /// halves are not in the order the counters mint them).
    rename: BTreeMap<EntityId, EntityId>,
}

/// What a step minted that the caller needs.
enum Minted {
    /// The two halves of a face partition, in the order the overlay minted them (the first one
    /// has the loop arc that starts first).
    Halves([EntityId; 2]),
    Other,
}

impl Stepper {
    fn new(host: Arc<dyn TopologyView>) -> Self {
        Self { ov: Overlay::new(host), rename: BTreeMap::new() }
    }

    fn name(&self, id: EntityId) -> EntityId {
        self.rename.get(&id).copied().unwrap_or(id)
    }

    fn step(&mut self, op: &VirtualOp) -> Result<Minted, ()> {
        let mut minted = Minted::Other;
        match op {
            VirtualOp::Composite { faces, .. } => {
                let faces: Vec<EntityId> = faces.iter().map(|&f| self.name(f)).collect();
                self.ov.composite_faces(&faces).map_err(drop)?;
            }
            VirtualOp::PartitionEdge { edge, param, .. } => {
                self.ov.partition_edge(*edge, *param).map_err(drop)?;
            }
            VirtualOp::PartitionFace { face, cut, .. } => {
                minted = Minted::Halves(self.ov.partition_face(self.name(*face), cut.clone()).map_err(drop)?);
            }
            VirtualOp::PartitionVolume { volume, cut, children, .. } => {
                let mut cut = cut.clone();
                cut.seed = self.name(cut.seed);
                self.ov.partition_volume(self.name(*volume), cut, *children).map_err(drop)?;
            }
            VirtualOp::CompositeEdges { edges, .. } => {
                self.ov.composite_edges(edges).map_err(drop)?;
            }
            VirtualOp::CollapseEdge { edge, into_vertex, .. } => {
                self.ov.collapse_edge(*edge, *into_vertex).map_err(drop)?;
            }
            _ => return Err(()),
        }
        // The ids the overlay minted must be the record's (as sets: a partition's halves may be
        // recorded in the other order).
        let sorted = |op: &VirtualOp| {
            let mut v = op.created().to_vec();
            v.sort();
            v
        };
        match self.ov.ops().last() {
            Some(done) if sorted(done) == sorted(op) => Ok(minted),
            _ => Err(()),
        }
    }

    /// The record names `recorded` for the halves the overlay minted as `minted` (`recorded` in
    /// that order, or crossed when `crossed`).
    fn rename_halves(&mut self, recorded: [EntityId; 2], minted: [EntityId; 2], crossed: bool) {
        let [m0, m1] = if crossed { [minted[1], minted[0]] } else { minted };
        self.rename.insert(recorded[0], m0);
        self.rename.insert(recorded[1], m1);
    }
}

/// The edges the first half of each face partition in `at` had when its op ran, by op index.
fn first_halves(ov: &Overlay, at: &[usize]) -> Result<BTreeMap<usize, BTreeSet<EntityId>>, OpError> {
    let Some(&last) = at.last() else { return Ok(BTreeMap::new()) };
    let want: BTreeSet<usize> = at.iter().copied().collect();
    let mut out = BTreeMap::new();
    let mut stepper = Stepper::new(ov.host_arc());
    let mut stepped = true;
    for (i, op) in ov.ops().iter().enumerate().take(last + 1) {
        match stepper.step(op) {
            Ok(Minted::Halves(m)) => {
                if let VirtualOp::PartitionFace { halves, .. } = op {
                    stepper.rename_halves(*halves, m, false);
                }
                if want.contains(&i) {
                    // The record's first half has the first loop arc, which the overlay mints first.
                    out.insert(i, edges_of(&stepper.ov, m[0]));
                }
            }
            Ok(Minted::Other) => {}
            Err(()) => {
                stepped = false;
                break;
            }
        }
    }
    if stepped {
        return Ok(out);
    }
    // Fallback: replay each prefix (the half's edges as they were when the op ran; later ops
    // may hide it).
    let mut out = BTreeMap::new();
    for &k in at {
        let VirtualOp::PartitionFace { halves, .. } = &ov.ops()[k] else { continue };
        let prefix = OverlayData::new(ov.data().host_revision, ov.ops()[..=k].to_vec(), ov.data().next_serial);
        let before = Overlay::from_data(ov.host_arc(), prefix).map_err(|e| OpError::Failed(format!("virtual topology replay: {e}")))?;
        out.insert(k, edges_of(&before, halves[0]));
    }
    Ok(out)
}

impl Rewritten {
    /// Build the overlay on the transformed `host`. A face partition gives its first half the
    /// loop arc that starts first in the host face's loop order; a transform that reverses or
    /// rotates the loop (a reflection) can hand that arc to the other half, so ids would name
    /// the mirror of the wrong half. The ops are replayed once, step by step, and a partition's
    /// halves are swapped when the first half no longer has the edges it had before.
    pub(crate) fn build(self, host: Arc<dyn TopologyView>) -> Result<Overlay, OpError> {
        let collapsed = self.collapsed.clone();
        self.build_inner(host).map_err(|e| match e {
            // A scaled collapse whose gap now exceeds the tolerance cap: name that collapse.
            TopoError::ToleranceTooLarge { wanted, max } => {
                let named = collapsed
                    .iter()
                    .filter(|&&(_, gap)| gap > max)
                    .min_by(|a, b| (a.1 - wanted).abs().total_cmp(&(b.1 - wanted).abs()))
                    .or_else(|| collapsed.iter().min_by(|a, b| (a.1 - wanted).abs().total_cmp(&(b.1 - wanted).abs())));
                match named {
                    Some(&(edge, _)) => OpError::Invalid(format!(
                        "the collapsed edge {edge} cannot follow the transform: its gap would grow to {wanted:e}, above linear_max {max:e}; remove the collapse first with model.remove_virtual"
                    )),
                    None => OpError::Failed(format!("the virtual topology cannot follow the transform: tolerance {wanted:e} above linear_max {max:e}")),
                }
            }
            e => OpError::Failed(format!("the virtual topology cannot follow the transform: {e}")),
        })
    }

    /// The swaps of one pass over the ops, or `None` if the pass cannot be done (see
    /// [`Stepper`]).
    fn swaps_by_stepping(&self, host: &Arc<dyn TopologyView>) -> Option<Vec<usize>> {
        let last = *self.first_half.keys().next_back()?;
        let mut stepper = Stepper::new(host.clone());
        let mut swaps = Vec::new();
        for (k, op) in self.ops.iter().enumerate().take(last + 1) {
            let Minted::Halves(m) = stepper.step(op).ok()? else { continue };
            let VirtualOp::PartitionFace { halves, .. } = op else { return None };
            let crossed = self.first_half.get(&k).is_some_and(|want| edges_of(&stepper.ov, m[0]) != *want);
            stepper.rename_halves(*halves, m, crossed);
            if crossed {
                swaps.push(k);
            }
        }
        Some(swaps)
    }

    fn build_inner(mut self, host: Arc<dyn TopologyView>) -> Result<Overlay, TopoError> {
        let rev = host.revision();
        if let Some(swaps) = self.swaps_by_stepping(&host) {
            for k in swaps {
                if let VirtualOp::PartitionFace { halves, .. } = &mut self.ops[k] {
                    halves.swap(0, 1);
                }
            }
        } else {
            for (&k, want) in &self.first_half {
                let prefix = OverlayData::new(rev, self.ops[..=k].to_vec(), self.serial);
                let ov = Overlay::from_data(host.clone(), prefix)?;
                let VirtualOp::PartitionFace { halves, .. } = &mut self.ops[k] else { continue };
                if edges_of(&ov, halves[0]) != *want {
                    halves.swap(0, 1);
                }
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
    let mut partitions: Vec<usize> = Vec::new();
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
        if matches!(op, VirtualOp::PartitionFace { .. }) {
            partitions.push(out.len());
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
                collapsed.push((edge, gap * s));
                VirtualOp::CollapseEdge { edge, into_vertex, gap: gap * s, repointed }
            }
            // Composites carry no absolute geometry.
            other => other,
        });
    }
    if !any {
        return Ok(None);
    }
    let first_half = first_halves(ov, &partitions)?;
    Ok(Some(Rewritten { ops: out, serial: ov.data().next_serial, first_half, collapsed }))
}
