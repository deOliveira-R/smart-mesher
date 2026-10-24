//! The volume `map` scheme: a six-faced volume whose faces are mapped quad grids becomes one
//! structured hexahedral block, interior by 3-D transfinite interpolation. See `DESIGN.md`.
//!
//! * `requirements`: exactly six faces, each imposed `QUAD | STRUCTURED` (hard); map writes no
//!   face mesh (no claims).
//! * `interval_constraints`: the twelve logical edges fall into three parallel classes of four,
//!   and the interval counts are equal within a class.
//! * `generate`: reads the six face grids ([`sm_scheme_api::GenCtx::grid`]), orients them into
//!   one `(i, j, k)` lattice by keys alone, interpolates the interior and emits hexes.

mod block;
#[cfg(test)]
mod block_tests;

use sm_scheme_api::{
    Applicability, Caps, FailureKind, Firmness, GenCtx, GenerateFailure, IntervalCtx, IntervalSink, Params, Patch, PlanCtx,
    Requirements, Scheme, SchemeDim, SchemeError, SchemeName, SchemeRegistry, Target,
};
use sm_core::EntityId;
use sm_topo_api::{TopologyView, TopologyViewExt};
use std::collections::{BTreeMap, BTreeSet};

/// Registered name of the scheme (dimension `Volume`).
pub const NAME: &str = "map";

/// The volume map scheme (`"map"`, dimension `Volume`, `HEX | STRUCTURED | BLOCK_STRUCTURED`).
#[derive(Clone, Copy, Debug, Default)]
pub struct VolumeMap;

/// Register the volume map scheme.
pub fn register(registry: &mut SchemeRegistry) -> sm_scheme_api::Result<()> {
    registry.register(Box::new(VolumeMap))
}

fn not_applicable(target: Target, reason: impl Into<String>) -> SchemeError {
    SchemeError::NotApplicable { target, scheme: SchemeName::new(NAME), reason: reason.into() }
}

/// Reason the piece cannot be a map block from topology alone, if any.
fn topology_problem(cx: &PlanCtx<'_>, target: Target) -> Option<String> {
    let Target::Volume(vol) = target else {
        return Some("not a volume".into());
    };
    let faces = match cx.input.piece_faces(target) {
        Ok(f) => f,
        Err(e) => return Some(e.to_string()),
    };
    if faces.len() != 6 {
        return Some(format!("{} faces; map needs exactly 6", faces.len()));
    }
    let topo: &dyn TopologyView = cx.input.topo;
    if !cx.input.embedded_edges(vol).is_empty() || !cx.input.embedded_vertices(vol).is_empty() {
        return Some("embedded members cannot be conformed to by a block".into());
    }
    let mut uses: BTreeMap<EntityId, u32> = BTreeMap::new();
    for f in &faces {
        if topo.loops(*f).count() != 1 {
            return Some(format!("face {f} has inner loops"));
        }
        for e in cx.input.face_edges(*f) {
            *uses.entry(e).or_default() += 1;
        }
    }
    if uses.values().any(|n| *n != 2) {
        return Some("an edge is not shared by exactly two of the six faces".into());
    }
    let verts: BTreeSet<EntityId> = uses.keys().filter_map(|e| topo.edge_ends(*e)).flat_map(|(a, b)| [a, b]).collect();
    if verts.len() + 6 != uses.len() + 2 {
        return Some("the faces do not close a sphere-like shell (Euler characteristic)".into());
    }
    None
}

impl Scheme for VolumeMap {
    fn name(&self) -> &str {
        NAME
    }

    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }

    fn caps(&self, _params: &Params) -> Caps {
        Caps::HEX | Caps::STRUCTURED | Caps::BLOCK_STRUCTURED
    }

    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, _params: &Params) -> Applicability {
        match topology_problem(cx, target) {
            Some(why) => Applicability::no(why),
            None => Applicability::yes(100, "six faces: one structured hexahedral block"),
        }
    }

    fn requirements(&self, cx: &PlanCtx<'_>, target: Target, _params: &Params, out: &mut Requirements<'_>) -> sm_scheme_api::Result<()> {
        if let Some(why) = topology_problem(cx, target) {
            return Err(not_applicable(target, why));
        }
        for f in cx.input.piece_faces(target)? {
            out.require_caps(Target::Surface(f), Caps::QUAD | Caps::STRUCTURED, Firmness::Hard);
        }
        Ok(())
    }

    fn interval_constraints(&self, cx: &IntervalCtx<'_>, target: Target, _params: &Params, out: &mut IntervalSink<'_>) -> sm_scheme_api::Result<()> {
        let faces = cx.input.piece_faces(target)?;
        let classes = parallel_classes(cx, &faces).map_err(|why| not_applicable(target, why))?;
        for class in &classes {
            let Some((first, rest)) = class.split_first() else { continue };
            for other in rest {
                out.equal(first, other);
            }
        }
        Ok(())
    }

    fn generate(&self, cx: &GenCtx<'_>, target: Target, _params: &Params) -> Result<Patch, GenerateFailure> {
        let err = |kind: FailureKind, reason: String| GenerateFailure::new(target, NAME, kind, reason);
        let Some(owner) = target.entity() else {
            return Err(err(FailureKind::Unsupported, "groups are not supported".into()));
        };
        let faces = cx.input.piece_faces(target).map_err(|e| err(FailureKind::Internal, e.to_string()))?;
        let mut grids = Vec::with_capacity(faces.len());
        for f in &faces {
            let g = cx.grid(*f).ok_or_else(|| err(FailureKind::Unsupported, format!("face {f} has no structured grid")))?;
            grids.push(g);
        }
        let b = block::assemble(owner, &grids, &|k| cx.point(k)).map_err(|f| err(f.kind, f.reason))?;
        Ok(block::patch(target, &b))
    }
}

/// The three classes of parallel logical edges, each as the edge sets (sorted ids) of its four
/// members, derived from the solved face layouts.
fn parallel_classes(cx: &IntervalCtx<'_>, faces: &[EntityId]) -> Result<Vec<Vec<Vec<EntityId>>>, String> {
    let mut sides: Vec<Vec<Vec<EntityId>>> = Vec::with_capacity(faces.len());
    for f in faces {
        let layout = cx.layout(*f).ok_or_else(|| format!("face {f} has no solved layout"))?;
        if layout.sides.len() != 4 {
            return Err(format!("face {f} has {} logical sides", layout.sides.len()));
        }
        sides.push(
            layout
                .sides
                .iter()
                .map(|s| {
                    let mut e: Vec<EntityId> = s.coedges.iter().map(|k| k.edge).collect();
                    e.sort_unstable();
                    e.dedup();
                    e
                })
                .collect(),
        );
    }
    let mut ids: BTreeMap<&Vec<EntityId>, (usize, u32)> = BTreeMap::new();
    for s in sides.iter().flatten() {
        ids.entry(s).or_insert((0, 0)).1 += 1;
    }
    if ids.len() != 12 || ids.values().any(|v| v.1 != 2) {
        return Err(format!("the logical sides of the faces do not form 12 shared block edges ({} found)", ids.len()));
    }
    for (i, v) in ids.values_mut().enumerate() {
        v.0 = i;
    }
    let mut parent: Vec<usize> = (0..12).collect();
    fn root(p: &mut [usize], mut x: usize) -> usize {
        while p[x] != x {
            p[x] = p[p[x]];
            x = p[x];
        }
        x
    }
    for f in &sides {
        for s in 0..2 {
            let (a, b) = (ids[&f[s]].0, ids[&f[s + 2]].0);
            let (ra, rb) = (root(&mut parent, a), root(&mut parent, b));
            parent[ra.max(rb)] = ra.min(rb);
        }
    }
    let mut classes: BTreeMap<usize, Vec<Vec<EntityId>>> = BTreeMap::new();
    for (key, (i, _)) in &ids {
        let r = root(&mut parent, *i);
        classes.entry(r).or_default().push((*key).clone());
    }
    if classes.len() != 3 || classes.values().any(|c| c.len() != 4) {
        return Err(format!("{} classes of parallel edges (3 of 4 expected)", classes.len()));
    }
    Ok(classes.into_values().collect())
}
