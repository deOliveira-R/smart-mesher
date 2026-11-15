//! `model.heal`: tolerant healing, tolerance first.
//!
//! The fixes run in a fixed order and each one only changes what its finding names:
//!
//! 1. **weld** (`gap_near`): for every near gap of [`mod@crate::heal_check`] up to `tol`, raise the
//!    per-entity tolerance of the paired cells to half the measured gap each (two cells coincide
//!    when `gap <= tol_a + tol_b <= linear_max`), then
//!    [`merge`] the cells within it. The lowest id of a merged pair survives, so the other
//!    side's boundary moves by the gap, within the raised tolerance; every move is a
//!    `Merged { gap }` lineage record and a [`MergedCell`] in the outcome. A gap whose closure
//!    has an edge no longer than twice the gap is skipped (the merge would collapse it): that is
//!    a collapse, which is virtual and not done here.
//! 2. **orient**: a volume whose outer shell is inside out (negative `volume_moments`) has its
//!    faces reversed, with the uses taken off and put back (the improper-map sequence of
//!    `sm-model`'s `DESIGN.md`).
//! 3. **close**: after the weld, every remaining chain of free edges that forms one simple loop,
//!    planar within `tol`, is closed by a plane face whose coedges run against the neighbours'.
//!    A loop that is the whole boundary of a lone face is left alone (it would make a pillow), and so is a
//!    loop whose faces all lie in the cap plane (a split planar sheet: the cap would double-cover it).
//!    The new face is a free face (it joins no body: the topology has no body-member edit).
//!
//! Geometry moves only through the merge (within the raised tolerance); `heal` never moves a
//! vertex. Running it twice changes nothing the second time.

use std::collections::{BTreeMap, BTreeSet};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{Frame, Plane, SurfaceDef};
use sm_topo_api::{
    CoedgeSpec, Dim, FaceUseSpec, Sense, ShellKind, Side, SurfaceCarrier, TopologyEdit,
    TopologyView, TopologyViewExt,
};

use crate::error::BooleanError;
use crate::geo;
use crate::heal_check::{edge_length, near_gaps, open_chains};
use crate::merge::MergeStatus;
use crate::merge::{MergeKind, MergeKinds, MergeOptions, MergedCell, merge, merge_check};

/// Which fixes [`heal`] applies (all by default).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct HealFixes {
    /// Raise tolerances and merge near cells (`gap_near`).
    pub weld: bool,
    /// Re-orient inside-out volumes.
    pub orient: bool,
    /// Close planar open loops.
    pub close: bool,
}

impl Default for HealFixes {
    fn default() -> Self {
        Self {
            weld: true,
            orient: true,
            close: true,
        }
    }
}

/// Options of [`heal`].
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct HealApply {
    /// Heal only the closure of these cells (default: everything).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub entities: Option<Vec<EntityId>>,
    /// The tolerance to heal within (default and maximum: the model's `linear_max`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tol: Option<f64>,
    /// The fixes to run.
    #[serde(default)]
    pub fixes: HealFixes,
}

/// A per-entity tolerance that [`heal`] raised.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct RaisedTolerance {
    /// The cell.
    pub cell: EntityId,
    /// Its new tolerance.
    pub tol: f64,
}

/// A hole that [`heal`] closed.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct ClosedHole {
    /// The new face.
    pub face: EntityId,
    /// The free edges it closes, in loop order.
    pub edges: Vec<EntityId>,
    /// The largest distance of the loop from the new plane.
    pub flatness: f64,
}

/// Something [`heal`] found and left alone, with the reason.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct HealSkip {
    /// The fix that declined: `weld`, `orient` or `close`.
    pub fix: String,
    /// The entities involved.
    pub entities: Vec<EntityId>,
    /// Why.
    pub reason: String,
}

/// What [`heal`] did, in the order it did it.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct HealOutcome {
    /// Tolerances raised before merging.
    pub raised: Vec<RaisedTolerance>,
    /// Merges, with the gaps they discarded.
    pub merged: Vec<MergedCell>,
    /// Volumes whose faces were reversed.
    pub reoriented: Vec<EntityId>,
    /// Holes closed by a new face.
    pub closed: Vec<ClosedHole>,
    /// Findings left alone.
    pub skipped: Vec<HealSkip>,
}

impl HealOutcome {
    /// True if nothing was changed.
    pub fn is_empty(&self) -> bool {
        self.raised.is_empty()
            && self.merged.is_empty()
            && self.reoriented.is_empty()
            && self.closed.is_empty()
    }
}

fn skip(out: &mut HealOutcome, fix: &str, entities: Vec<EntityId>, reason: impl Into<String>) {
    let s = HealSkip {
        fix: fix.to_owned(),
        entities,
        reason: reason.into(),
    };
    if !out.skipped.contains(&s) {
        out.skipped.push(s);
    }
}

fn check_tol(topo: &dyn TopologyEdit, o: &HealApply) -> Result<f64, BooleanError> {
    let max = topo.tolerance().linear_max;
    let tol = o.tol.unwrap_or(max);
    if !(tol.is_finite() && tol > 0.0 && tol <= max) {
        return Err(BooleanError::Invalid(format!(
            "heal tolerance {tol:e} is outside (0, linear_max = {max:e}]"
        )));
    }
    Ok(tol)
}

/// Run the fixes of `o.fixes` on `topo`, inside an open operation (the caller aborts it on
/// `Err`).
///
/// # Errors
/// `Invalid` for a bad tolerance or a dead entity; `Topo` if an edit the analysis predicted valid
/// is refused.
pub fn heal(topo: &mut dyn TopologyEdit, o: &HealApply) -> Result<HealOutcome, BooleanError> {
    let tol = check_tol(topo, o)?;
    let mut out = HealOutcome::default();
    if o.fixes.weld {
        weld_gaps(topo, &o.entities, tol, &mut out)?;
    }
    if o.fixes.orient {
        let vols = scope_volumes(topo, &o.entities);
        orient_volumes(topo, &vols, &mut out)?;
    }
    if o.fixes.close {
        close_open_loops(topo, &o.entities, tol, &mut out)?;
    }
    Ok(out)
}

fn scope_volumes(view: &dyn TopologyView, entities: &Option<Vec<EntityId>>) -> Vec<EntityId> {
    let all = view.cells(Dim::Volume).to_vec();
    match entities {
        None => all,
        Some(list) => {
            let set: BTreeSet<EntityId> = list.iter().copied().collect();
            all.into_iter()
                .filter(|v| {
                    set.contains(v) || view.volume_faces(*v).iter().any(|f| set.contains(f))
                })
                .collect()
        }
    }
}

fn raise(
    topo: &mut dyn TopologyEdit,
    cell: EntityId,
    to: f64,
    out: &mut HealOutcome,
) -> Result<(), BooleanError> {
    if topo.tolerance_of(cell) < to {
        topo.raise_tolerance(cell, to)?;
        out.raised.push(RaisedTolerance { cell, tol: to });
    }
    Ok(())
}

/// Weld the near gaps up to `tol`: raise the paired cells' tolerances to the gap, then merge.
/// Runs inside an open operation.
///
/// # Errors
/// As [`heal`].
pub fn weld_gaps(
    topo: &mut dyn TopologyEdit,
    entities: &Option<Vec<EntityId>>,
    tol: f64,
    out: &mut HealOutcome,
) -> Result<(), BooleanError> {
    let max = topo.tolerance().linear_max;
    // Each pass merges what is mergeable; a merge can make a neighbouring pair mergeable.
    for _ in 0..8 {
        let mut findings = near_gaps(&*topo, entities);
        findings.retain(|f| f.entities.len() == 2 && f.value <= tol);
        findings.sort_by(|a, b| a.entities.cmp(&b.entities));
        let mut progress = false;
        for f in findings {
            let (a, b) = (f.entities[0], f.entities[1]);
            if !topo.is_alive(a) || !topo.is_alive(b) {
                progress = true; // rescan with the merged ids
                continue;
            }
            let scope = [a, b];
            let mut closure: BTreeSet<EntityId> = BTreeSet::new();
            closure.extend(topo.closure(a));
            closure.extend(topo.closure(b));
            // A closure edge this short would collapse: not a weld.
            let short = closure
                .iter()
                .copied()
                .filter(|&c| topo.dim_of(c) == Some(Dim::Edge) && !geo::is_degenerate(&*topo, c))
                .find(|&e| edge_length(&*topo, e).is_some_and(|l| l <= 2.0 * f.value));
            if let Some(e) = short {
                skip(
                    out,
                    "weld",
                    vec![a, b],
                    format!(
                        "{e} is no longer than twice the gap {:e}: merging would collapse it (a virtual collapse, not a weld)",
                        f.value
                    ),
                );
                continue;
            }
            let rep = merge_check(&*topo, &scope, &[], &MergeOptions::default());
            let mut pairs: Vec<_> = rep
                .pairs
                .iter()
                .filter(|p| p.status == MergeStatus::Near && p.gap <= tol)
                .map(|p| (p.a, p.b, p.gap))
                .collect();
            pairs.sort_by_key(|x| (x.0, x.1));
            for (x, y, gap) in pairs {
                // Cells coincide when `gap <= tol_x + tol_y`: half the gap each (rounded up).
                let to = (gap * 0.5 * (1.0 + 1e-9)).min(max);
                raise(topo, x, to, out)?;
                raise(topo, y, to, out)?;
            }
            let r = merge(
                topo,
                &scope,
                &[],
                &MergeOptions {
                    tol: None,
                    kinds: MergeKinds::default(),
                },
            )?;
            // The surviving face keeps the larger tolerance: its boundary moved by the gap.
            for m in r.merged.iter().filter(|m| m.kind == MergeKind::Face) {
                let to = (m.gap * (1.0 + 1e-9)).min(max);
                raise(topo, m.keep, to, out)?;
            }
            if !r.merged.is_empty() {
                progress = true;
            }
            out.merged.extend(r.merged);
        }
        if !progress {
            break;
        }
    }
    // Anything still near and within tol could not be merged (a conflict or an overlap).
    for f in near_gaps(&*topo, entities) {
        if f.entities.len() == 2 && f.value <= tol {
            skip(
                out,
                "weld",
                f.entities.clone(),
                format!(
                    "the gap {:e} could not be merged (see model.merge_check)",
                    f.value
                ),
            );
        }
    }
    Ok(())
}

/// Re-orient the inside-out volumes among `volumes`: negative `volume_moments`, one outer shell,
/// faces used by that volume alone. Runs inside an open operation.
///
/// # Errors
/// As [`heal`].
pub fn orient_volumes(
    topo: &mut dyn TopologyEdit,
    volumes: &[EntityId],
    out: &mut HealOutcome,
) -> Result<(), BooleanError> {
    for &v in volumes {
        if !topo.is_alive(v) {
            continue;
        }
        let Ok(m) = topo.volume_moments(v) else {
            continue;
        };
        if m.volume >= 0.0 {
            continue;
        }
        let faces = topo.volume_faces(v).to_vec();
        let alone = faces.iter().all(|&f| {
            let uses = topo.face_uses_of(f);
            uses.len() == 1 && uses[0].volume == v && uses[0].sense == Sense::Forward
        });
        let one_shell =
            topo.shells(v).count() == 1 && topo.shell_kind(v, 0) == Some(ShellKind::Outer);
        if !alone || !one_shell {
            skip(
                out,
                "orient",
                vec![v],
                "the volume has other shells, shares a face or uses one on its right: not re-oriented",
            );
            continue;
        }
        for &f in &faces {
            topo.remove_face_use(v, f, Side::Left)?;
        }
        for &f in &faces {
            topo.reverse_face(f)?;
        }
        for &f in &faces {
            topo.add_face_use(
                v,
                0,
                FaceUseSpec {
                    face: f,
                    side: Side::Left,
                },
            )?;
        }
        out.reoriented.push(v);
    }
    Ok(())
}

/// One free edge of a loop to close: the sense the new face traverses it with.
struct Rail {
    edge: EntityId,
    sense: Sense,
}

fn free_face_use(view: &dyn TopologyView, e: EntityId) -> Option<(EntityId, Sense)> {
    let uses: Vec<_> = view
        .coboundary(e)
        .iter()
        .filter(|i| view.dim_of(i.cell) == Some(Dim::Face))
        .collect();
    match uses[..] {
        [u] if u.side != Side::Both => Some((u.cell, u.side.sense())),
        _ => None,
    }
}

/// Order the chain `edges` into one simple loop for the new face, or say why it is not one.
fn order_loop(view: &dyn TopologyView, edges: &[EntityId]) -> Result<Vec<Rail>, String> {
    let mut by_tail: BTreeMap<EntityId, Vec<(EntityId, Sense, EntityId)>> = BTreeMap::new();
    let mut degree: BTreeMap<EntityId, usize> = BTreeMap::new();
    for &e in edges {
        let (_, existing) =
            free_face_use(view, e).ok_or_else(|| format!("{e} is not a free edge"))?;
        let (s, t) = view
            .edge_ends(e)
            .ok_or_else(|| format!("{e} has no end vertices"))?;
        // The new face runs against the neighbour: reversed sense.
        let sense = existing.flip();
        let (tail, head) = if sense == Sense::Forward {
            (s, t)
        } else {
            (t, s)
        };
        by_tail.entry(tail).or_default().push((e, sense, head));
        *degree.entry(s).or_default() += 1;
        *degree.entry(t).or_default() += 1;
    }
    if degree.values().any(|&d| d != 2) {
        return Err(
            "the free edges branch or do not close (a vertex is not on exactly two)".into(),
        );
    }
    if by_tail.values().any(|v| v.len() != 1) {
        return Err("the free edges are not consistently oriented".into());
    }
    let first = *by_tail.keys().next().ok_or("no free edges")?;
    let mut rails = Vec::new();
    let mut tail = first;
    loop {
        let &(e, sense, head) = by_tail
            .get(&tail)
            .and_then(|v| v.first())
            .ok_or("the free edges do not chain head to tail")?;
        rails.push(Rail { edge: e, sense });
        tail = head;
        if tail == first {
            break;
        }
        if rails.len() > edges.len() {
            return Err("the free edges do not chain head to tail".into());
        }
    }
    if rails.len() != edges.len() {
        return Err("the free edges form more than one loop".into());
    }
    Ok(rails)
}

/// Close each chain of free edges that is one simple loop, planar within `tol`, with a plane
/// face. Runs inside an open operation.
///
/// # Errors
/// As [`heal`].
pub fn close_open_loops(
    topo: &mut dyn TopologyEdit,
    entities: &Option<Vec<EntityId>>,
    tol: f64,
    out: &mut HealOutcome,
) -> Result<(), BooleanError> {
    let chains = open_chains(&*topo, entities)?;
    for ch in chains {
        let edges = &ch.entities;
        let rails = match order_loop(&*topo, edges) {
            Ok(r) => r,
            Err(why) => {
                skip(out, "close", edges.clone(), why);
                continue;
            }
        };
        // A lone face's whole boundary: closing it would make a pillow, not heal a hole.
        let users: BTreeSet<EntityId> = edges
            .iter()
            .filter_map(|&e| free_face_use(&*topo, e).map(|u| u.0))
            .collect();
        if users.len() == 1
            && users
                .iter()
                .all(|&f| topo.face_edges(f).len() == edges.len())
        {
            skip(
                out,
                "close",
                edges.clone(),
                "the loop is the whole boundary of a lone face (a sheet, not a hole)",
            );
            continue;
        }
        // Sample the loop in traversal order.
        let mut pts: Vec<Point3> = Vec::new();
        for r in &rails {
            for k in 0..8 {
                let f = k as f64 / 8.0;
                let f = if r.sense == Sense::Forward {
                    f
                } else {
                    1.0 - f
                };
                if let Some(p) = geo::edge_sample(&*topo, r.edge, f) {
                    pts.push(p);
                }
            }
        }
        if pts.len() < 3 {
            skip(out, "close", edges.clone(), "too few points to fit a plane");
            continue;
        }
        let n = pts.len() as f64;
        let c = pts.iter().fold(Vector3::new(0.0, 0.0, 0.0), |a, p| {
            a + Vector3::new(p.x, p.y, p.z)
        }) * (1.0 / n);
        let c = Point3::new(c.x, c.y, c.z);
        // Newell's area vector, about the centroid.
        let mut area = Vector3::new(0.0, 0.0, 0.0);
        for i in 0..pts.len() {
            let (p, q) = (pts[i] - c, pts[(i + 1) % pts.len()] - c);
            area = area + p.cross(q);
        }
        let Some(normal) = geo::unit(area) else {
            skip(out, "close", edges.clone(), "the loop has no area");
            continue;
        };
        let flat = pts
            .iter()
            .map(|p| ((*p - c).dot(normal)).abs())
            .fold(0.0, f64::max);
        if flat > tol {
            skip(
                out,
                "close",
                edges.clone(),
                format!("the loop is {flat:e} from a plane, above the tolerance {tol:e}"),
            );
            continue;
        }
        // A planar sheet of several coplanar faces: the cap would lie on the sheet, a zero-volume
        // double cover. Skip when every face using a loop edge lies within tol of the cap plane.
        let coplanar = users.iter().all(|&f| {
            topo.face_edges(f).iter().all(|&e| {
                (0..=4).all(|k| {
                    geo::edge_sample(&*topo, e, k as f64 / 4.0)
                        .is_some_and(|p| ((p - c).dot(normal)).abs() <= tol)
                })
            })
        });
        if coplanar {
            skip(
                out,
                "close",
                edges.clone(),
                "every face at the loop lies in the cap plane (a planar sheet, not a hole)",
            );
            continue;
        }
        let frame = Frame::from_z(c, normal)?;
        let surface = topo
            .shapes_mut()?
            .add_surface(SurfaceDef::Plane(Plane { frame }))?;
        let lp: Vec<CoedgeSpec> = rails
            .iter()
            .map(|r| CoedgeSpec {
                edge: r.edge,
                sense: r.sense,
            })
            .collect();
        let face = topo.make_face(
            &[&lp],
            Some(SurfaceCarrier {
                surface,
                sense: Sense::Forward,
            }),
        )?;
        topo.record_created(face)?;
        out.closed.push(ClosedHole {
            face,
            edges: rails.iter().map(|r| r.edge).collect(),
            flatness: flat,
        });
    }
    Ok(())
}
