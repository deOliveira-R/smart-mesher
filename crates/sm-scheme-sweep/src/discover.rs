//! Column discovery: which edges join the source vertices to the target vertices, and which
//! faces link the two.

use crate::error::SweepError;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, Vector3};
use sm_scheme_api::geom;
use sm_topo_api::{CellFlags, TopologyView, TopologyViewExt};
use std::collections::BTreeSet;

/// Longest column (edges) the walk follows before giving up.
const MAX_STEPS: usize = 64;
/// Minimum cosine between consecutive column edges at an interior vertex of a composite stack.
const STRAIGHT: f64 = 0.9;

/// One column: the path of edges from a source vertex to its target vertex.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Column {
    /// The source vertex.
    pub vertex: EntityId,
    /// `(edge, runs along its own direction)` from the source vertex to the target vertex.
    pub path: Vec<(EntityId, bool)>,
}

/// The analysis of a sweep: source, target, linking faces and the column of every source vertex.
/// It is computed while planning and stored as scheme data for the interval and generation
/// phases.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Columns {
    /// Member volumes in stacking order (one for a plain sweep or a composite piece).
    pub members: Vec<EntityId>,
    /// The source face.
    pub source: EntityId,
    /// The target face.
    pub target: EntityId,
    /// Faces between consecutive members (empty for a single piece).
    pub interfaces: Vec<EntityId>,
    /// Linking faces, ascending.
    pub linking: Vec<EntityId>,
    /// One column per source vertex, ascending by vertex.
    pub columns: Vec<Column>,
    /// The member owning each step of a column.
    pub step_member: Vec<usize>,
}

impl Columns {
    /// The end (target) vertex of a column.
    pub fn end_vertex(&self, t: &dyn TopologyView, c: &Column) -> Option<EntityId> {
        let (e, fwd) = c.path.last()?;
        let (a, b) = t.edge_ends(*e)?;
        Some(if *fwd { b } else { a })
    }

    /// `(source vertex, target vertex)` for every column, ascending by source.
    pub fn vertex_map(&self, t: &dyn TopologyView) -> Vec<(EntityId, EntityId)> {
        self.columns
            .iter()
            .filter_map(|c| Some((c.vertex, self.end_vertex(t, c)?)))
            .collect()
    }
}

fn face_vertices(t: &dyn TopologyView, f: EntityId) -> Vec<EntityId> {
    let mut v: Vec<EntityId> = t
        .face_edges(f)
        .iter()
        .filter(|e| !t.flags(**e).contains(CellFlags::DEGENERATE))
        .filter_map(|e| t.edge_ends(*e))
        .flat_map(|(a, b)| [a, b])
        .collect();
    v.sort_unstable();
    v.dedup();
    v
}

/// Number of real (non-pole) coedges of the loops of `f`.
fn coedge_count(t: &dyn TopologyView, f: EntityId) -> usize {
    t.loops(f)
        .map(|lp| {
            lp.keys()
                .filter(|k| !t.flags(k.edge).contains(CellFlags::DEGENERATE))
                .count()
        })
        .sum()
}

fn no(reason: impl Into<String>) -> SweepError {
    SweepError::NoColumns(reason.into())
}

/// Find the columns of a stack of `members` (volumes with their faces) between `source` and
/// `target`.
///
/// For each source vertex the walk follows the single edge that is not an edge of the source,
/// the target or an interface face; at a vertex with several candidates (an interior vertex of
/// a composite stack) the one continuing straight on (cosine above 0.9) is taken. Handles a
/// single volume, a composite piece (merged volumes; the layer count is the sum of the step
/// counts) and a source with several loops.
///
/// # Errors
/// [`SweepError::NoColumns`] naming why: the faces touch, the vertex counts differ (many-to-one
/// is not supported), a vertex has no column or no straight continuation, two columns end at
/// one target vertex, the columns differ in length, or a linking face has fewer than four
/// coedges.
pub fn discover_columns(
    t: &dyn TopologyView,
    members: &[(EntityId, Vec<EntityId>)],
    source: EntityId,
    target: EntityId,
) -> Result<Columns, SweepError> {
    let all: BTreeSet<EntityId> = members.iter().flat_map(|m| m.1.iter().copied()).collect();
    if !all.contains(&source) || !all.contains(&target) || source == target {
        return Err(no(format!(
            "source {source} and target {target} must be two distinct faces of the piece"
        )));
    }
    let interfaces_set: BTreeSet<EntityId> = all
        .iter()
        .copied()
        .filter(|f| members.iter().filter(|m| m.1.contains(f)).count() >= 2)
        .collect();
    let (vs, vg) = (face_vertices(t, source), face_vertices(t, target));
    if vs.iter().any(|v| vg.contains(v)) {
        return Err(no("source and target share a vertex"));
    }
    if vs.len() != vg.len() {
        return Err(no(format!(
            "source has {} vertices, target {} (many-to-one sweeps are not supported)",
            vs.len(),
            vg.len()
        )));
    }
    let mut excluded: BTreeSet<EntityId> = BTreeSet::new();
    for f in [source, target].iter().chain(interfaces_set.iter()) {
        excluded.extend(t.face_edges(*f).iter().copied());
    }
    let piece_edges: BTreeSet<EntityId> = all
        .iter()
        .flat_map(|f| t.face_edges(*f).iter().copied())
        .collect();
    let pos = |v: EntityId| geom::vertex_point(t, v).map_err(|e| no(e.to_string()));
    let mut columns = Vec::with_capacity(vs.len());
    for &v in &vs {
        let mut path: Vec<(EntityId, bool)> = Vec::new();
        let (mut cur, mut last, mut dir): (EntityId, Option<EntityId>, Option<Vector3>) =
            (v, None, None);
        for _ in 0..MAX_STEPS {
            if vg.contains(&cur) {
                break;
            }
            let cands: Vec<(EntityId, EntityId)> = piece_edges
                .iter()
                .filter(|e| !excluded.contains(e) && Some(**e) != last)
                .filter_map(|e| {
                    let (a, b) = t.edge_ends(*e)?;
                    if a == cur {
                        Some((*e, b))
                    } else if b == cur {
                        Some((*e, a))
                    } else {
                        None
                    }
                })
                .collect();
            let pick = match (cands.as_slice(), dir) {
                ([one], _) => *one,
                (many, Some(d)) if !many.is_empty() => {
                    let p0 = pos(cur)?;
                    let mut best: Option<(f64, (EntityId, EntityId))> = None;
                    for c in many {
                        let q = pos(c.1)? - p0;
                        let u = q * (1.0 / q.norm().max(1e-300));
                        let dot = u.dot(d);
                        if best.is_none_or(|b| dot > b.0) {
                            best = Some((dot, *c));
                        }
                    }
                    match best {
                        Some((dot, c)) if dot > STRAIGHT => c,
                        _ => {
                            return Err(no(format!(
                                "column at {v}: no straight continuation at {cur}"
                            )));
                        }
                    }
                }
                _ => {
                    return Err(no(format!(
                        "vertex {cur} needs exactly one linking edge, found {}",
                        cands.len()
                    )));
                }
            };
            let (e, next) = pick;
            let (a, _) = t.edge_ends(e).ok_or_else(|| no("edge without ends"))?;
            let q = pos(next)? - pos(cur)?;
            dir = Some(q * (1.0 / q.norm().max(1e-300)));
            path.push((e, a == cur));
            last = Some(e);
            cur = next;
        }
        if !vg.contains(&cur) {
            return Err(no(format!("column at {v} does not reach the target")));
        }
        columns.push(Column { vertex: v, path });
    }
    let ends: BTreeSet<EntityId> = columns
        .iter()
        .filter_map(|c| {
            let (e, fwd) = c.path.last()?;
            let (a, b) = t.edge_ends(*e)?;
            Some(if *fwd { b } else { a })
        })
        .collect();
    if ends.len() != columns.len() {
        return Err(no("two columns end at one target vertex"));
    }
    let steps = columns.first().map_or(0, |c| c.path.len());
    if steps == 0 || columns.iter().any(|c| c.path.len() != steps) {
        return Err(no("columns have different lengths"));
    }
    let linking_of = |m: &(EntityId, Vec<EntityId>)| -> Vec<EntityId> {
        m.1.iter()
            .copied()
            .filter(|f| *f != source && *f != target && !interfaces_set.contains(f))
            .collect()
    };
    let mut step_member = Vec::with_capacity(steps);
    for j in 0..steps {
        let e = columns[0].path[j].0;
        let ms: Vec<usize> = (0..members.len())
            .filter(|&i| {
                linking_of(&members[i])
                    .iter()
                    .any(|f| t.face_edges(*f).contains(&e))
            })
            .collect();
        match ms.as_slice() {
            [m] => step_member.push(*m),
            _ => {
                return Err(no(format!(
                    "step {j}: edge {e} does not belong to exactly one member"
                )));
            }
        }
    }
    let mut order: Vec<usize> = Vec::new();
    for m in &step_member {
        if !order.contains(m) {
            order.push(*m);
        }
    }
    if order.len() != members.len() {
        return Err(no("a member is not on the column"));
    }
    let step_member: Vec<usize> = step_member
        .iter()
        .map(|m| order.iter().position(|o| o == m).unwrap_or(0))
        .collect();
    let mut interfaces = Vec::new();
    for w in order.windows(2) {
        let shared: Vec<EntityId> = members[w[0]]
            .1
            .iter()
            .copied()
            .filter(|f| members[w[1]].1.contains(f))
            .collect();
        match shared.as_slice() {
            [f] => interfaces.push(*f),
            _ => return Err(no("consecutive members must share exactly one face")),
        }
    }
    let linking: Vec<EntityId> = all
        .iter()
        .copied()
        .filter(|f| *f != source && *f != target && !interfaces_set.contains(f))
        .collect();
    for l in &linking {
        if coedge_count(t, *l) < 4 {
            return Err(no(format!("linking face {l} has fewer than four coedges")));
        }
    }
    Ok(Columns {
        members: order.iter().map(|&i| members[i].0).collect(),
        source,
        target,
        interfaces,
        linking,
        columns,
        step_member,
    })
}
