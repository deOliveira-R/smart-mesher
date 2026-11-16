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
    /// The source face (the first source face of a set sweep).
    pub source: EntityId,
    /// The target face (the image of `source`).
    pub target: EntityId,
    /// Further `[source, target]` face pairs of a many-to-many sweep over imprinted faces,
    /// ascending by source; empty (and absent from the serialised form) for a 1-to-1 sweep.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub more: Vec<[EntityId; 2]>,
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
    /// Every `[source, target]` face pair, ascending by source (one pair for a 1-to-1 sweep).
    pub fn pairs(&self) -> Vec<[EntityId; 2]> {
        let mut v = Vec::with_capacity(1 + self.more.len());
        v.push([self.source, self.target]);
        v.extend_from_slice(&self.more);
        v
    }

    /// `(source vertex, target vertex)` of the columns that start on the vertices of `face`.
    pub fn face_vertex_map(
        &self,
        t: &dyn TopologyView,
        face: EntityId,
    ) -> Vec<(EntityId, EntityId)> {
        let vs = face_vertices(t, face);
        self.vertex_map(t)
            .into_iter()
            .filter(|(a, _)| vs.binary_search(a).is_ok())
            .collect()
    }

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

/// Work counters for tests (feature `test-hooks`): how often the full column walk ran.
#[cfg(feature = "test-hooks")]
pub mod counters {
    use std::cell::Cell;
    thread_local! {
        static DISCOVER_CALLS: Cell<u64> = const { Cell::new(0) };
    }
    /// Reset this thread's `discover_columns` call counter.
    pub fn reset_discover_calls() {
        DISCOVER_CALLS.with(|c| c.set(0));
    }
    /// Number of `discover_columns` calls on this thread since the last reset.
    pub fn discover_calls() -> u64 {
        DISCOVER_CALLS.with(Cell::get)
    }
    pub(crate) fn bump() {
        DISCOVER_CALLS.with(|c| c.set(c.get() + 1));
    }
}

/// Cheap necessary conditions on a source/target pair given their sorted vertex lists: no
/// shared vertex and equal vertex counts. Returns the exact error `discover_columns` gives.
pub(crate) fn prefilter(vs: &[EntityId], vg: &[EntityId]) -> Result<(), SweepError> {
    if vs.iter().any(|v| vg.binary_search(v).is_ok()) {
        return Err(no("source and target share a vertex"));
    }
    if vs.len() != vg.len() {
        return Err(no(format!(
            "source has {} vertices, target {} (many-to-one sweeps are not supported)",
            vs.len(),
            vg.len()
        )));
    }
    Ok(())
}

/// Sorted vertex lists of faces (for pair prefiltering).
pub(crate) fn vertex_lists(t: &dyn TopologyView, faces: &[EntityId]) -> Vec<Vec<EntityId>> {
    faces.iter().map(|f| face_vertices(t, *f)).collect()
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
/// one target vertex, the columns differ in length, a linking face has fewer than four
/// coedges, or a linking face is not a strip (inner loops, or fewer than two column coedges).
pub fn discover_columns(
    t: &dyn TopologyView,
    members: &[(EntityId, Vec<EntityId>)],
    source: EntityId,
    target: EntityId,
) -> Result<Columns, SweepError> {
    discover_column_sets(t, members, &[source], &[target])
}

/// [`discover_columns`] for sets of faces: many-to-many over imprinted faces (a volume whose
/// bottom and top are each split into several faces by imprinted lines, with no internal
/// face). Every source face must have a target face whose vertices are exactly the ends of
/// its columns; the pairs come out ascending by source face. One face on each side is the
/// plain 1-to-1 analysis, unchanged. True many-to-one (a face set against a different number
/// of faces) is not supported.
///
/// # Errors
/// As [`discover_columns`]; a set that has no face-to-face correspondence names the source and
/// target face ids and the vertices that find no image.
pub fn discover_column_sets(
    t: &dyn TopologyView,
    members: &[(EntityId, Vec<EntityId>)],
    sources: &[EntityId],
    targets: &[EntityId],
) -> Result<Columns, SweepError> {
    #[cfg(feature = "test-hooks")]
    counters::bump();
    let all: BTreeSet<EntityId> = members.iter().flat_map(|m| m.1.iter().copied()).collect();
    let mut sources: Vec<EntityId> = sources.to_vec();
    let mut targets: Vec<EntityId> = targets.to_vec();
    sources.sort_unstable();
    targets.sort_unstable();
    let distinct = |v: &[EntityId]| v.windows(2).all(|w| w[0] != w[1]);
    if sources.is_empty()
        || targets.is_empty()
        || !distinct(&sources)
        || !distinct(&targets)
        || sources.iter().chain(&targets).any(|f| !all.contains(f))
        || sources.iter().any(|f| targets.contains(f))
    {
        return Err(no(match (sources.as_slice(), targets.as_slice()) {
            ([s], [g]) => {
                format!("source {s} and target {g} must be two distinct faces of the piece")
            }
            _ => format!(
                "source faces {sources:?} and target faces {targets:?} must be non-empty sets of distinct faces of the piece, disjoint from each other"
            ),
        }));
    }
    if sources.len() != targets.len() {
        return Err(no(format!(
            "source faces {sources:?} against target faces {targets:?}: the sets differ in size, so there is no face-to-face correspondence (many-to-one sweeps are not supported)"
        )));
    }
    let (source, target) = (sources[0], targets[0]);
    let interfaces_set: BTreeSet<EntityId> = all
        .iter()
        .copied()
        .filter(|f| members.iter().filter(|m| m.1.contains(f)).count() >= 2)
        .collect();
    let union_vertices = |fs: &[EntityId]| {
        let mut v: Vec<EntityId> = fs.iter().flat_map(|f| face_vertices(t, *f)).collect();
        v.sort_unstable();
        v.dedup();
        v
    };
    let (vs, vg) = (union_vertices(&sources), union_vertices(&targets));
    prefilter(&vs, &vg).map_err(|e| {
        if sources.len() == 1 {
            e
        } else {
            no(format!(
                "source faces {sources:?} against target faces {targets:?}: {e}; there is no face-to-face correspondence"
            ))
        }
    })?;
    let mut excluded: BTreeSet<EntityId> = BTreeSet::new();
    for f in sources.iter().chain(&targets).chain(interfaces_set.iter()) {
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
            .filter(|f| !sources.contains(f) && !targets.contains(f) && !interfaces_set.contains(f))
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
        .filter(|f| !sources.contains(f) && !targets.contains(f) && !interfaces_set.contains(f))
        .collect();
    // Face pairing: the ends of the columns of one source face are the vertices of one target.
    let mut pairs: Vec<[EntityId; 2]> = Vec::with_capacity(sources.len());
    if sources.len() == 1 {
        pairs.push([source, target]);
    } else {
        let mut taken: Vec<EntityId> = Vec::new();
        for &s in &sources {
            let sv = face_vertices(t, s);
            let mut ends: Vec<EntityId> = columns
                .iter()
                .filter(|c| sv.binary_search(&c.vertex).is_ok())
                .filter_map(|c| {
                    let (e, fwd) = c.path.last()?;
                    let (a, b) = t.edge_ends(*e)?;
                    Some(if *fwd { b } else { a })
                })
                .collect();
            ends.sort_unstable();
            ends.dedup();
            let hits: Vec<EntityId> = targets
                .iter()
                .copied()
                .filter(|g| face_vertices(t, *g) == ends)
                .collect();
            match hits.as_slice() {
                [g] if !taken.contains(g) => {
                    taken.push(*g);
                    pairs.push([s, *g]);
                }
                _ => {
                    return Err(no(format!(
                        "source face {s}: its columns end at vertices {ends:?}, which are not the vertices of exactly one unused face of the target set {targets:?}; no face-to-face correspondence (many-to-one sweeps are not supported)"
                    )));
                }
            }
        }
    }
    let column_edges: BTreeSet<EntityId> = columns
        .iter()
        .flat_map(|c| c.path.iter().map(|(e, _)| *e))
        .collect();
    for l in &linking {
        if coedge_count(t, *l) < 4 {
            return Err(no(format!("linking face {l} has fewer than four coedges")));
        }
        // A linking face must be a strip: one loop, and at least two column coedges
        // (a seam counts twice). Otherwise the pair is not a prism.
        if t.loops(*l).count() != 1 {
            return Err(no(format!(
                "linking face {l} has inner loops (not a strip)"
            )));
        }
        let n = t
            .coedges(*l)
            .filter(|c| column_edges.contains(&c.key.edge))
            .count();
        if n < 2 {
            return Err(no(format!("linking face {l} has {n} column coedges")));
        }
    }
    Ok(Columns {
        members: order.iter().map(|&i| members[i].0).collect(),
        source: pairs[0][0],
        target: pairs[0][1],
        more: pairs[1..].to_vec(),
        interfaces,
        linking,
        columns,
        step_member,
    })
}
