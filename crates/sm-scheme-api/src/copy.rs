//! Copy constraints: correspondence construction and the built-in copy generator (user copy,
//! mirror, deck pairs, sweep targets), at curve and surface level.

use crate::error::{Result, SchemeError};
use crate::geom;
use crate::patch::{ElemShape, Patch, PatchNode, PatchSet};
use crate::problem::{CopyConstraint, Record};
use crate::target::{NodeKey, Target};
use sm_core::{EntityId, Point3, Transform};
use sm_mesh_api::Param;
use sm_topo_api::{Dim, TopologyView, TopologyViewExt};
use std::collections::BTreeMap;

fn corr(c: &CopyConstraint, reason: impl Into<String>) -> SchemeError {
    SchemeError::Correspondence { source_target: c.source, target: c.target, reason: reason.into() }
}

/// Vertices of a curve or surface target, ascending.
pub fn target_vertices(t: &dyn TopologyView, target: Target) -> Vec<EntityId> {
    let mut v: Vec<EntityId> = match target {
        Target::Curve(e) => t.edge_ends(e).map(|(a, b)| vec![a, b]).unwrap_or_default(),
        Target::Surface(f) => t.face_edges(f).iter().filter_map(|e| t.edge_ends(*e)).flat_map(|(a, b)| [a, b]).collect(),
        _ => Vec::new(),
    };
    v.sort_unstable();
    v.dedup();
    v
}

/// Build the vertex correspondence of a copy geometrically: every source vertex maps to the
/// target vertex at `transform(p)` within the summed vertex tolerances. Errors if a vertex
/// has no image or two share one.
pub fn derive_vertex_map(t: &dyn TopologyView, c: &CopyConstraint) -> Result<Vec<(EntityId, EntityId)>> {
    let src = target_vertices(t, c.source);
    let dst = target_vertices(t, c.target);
    let mut pts: Vec<(EntityId, Point3)> = Vec::with_capacity(dst.len());
    for w in &dst {
        pts.push((*w, geom::vertex_point(t, *w)?));
    }
    let mut out = Vec::with_capacity(src.len());
    for v in &src {
        let p = c.transform.apply_point(geom::vertex_point(t, *v)?);
        let hit: Vec<EntityId> = pts.iter().filter(|(w, q)| (*q - p).norm() <= t.tolerance_of(*v) + t.tolerance_of(*w)).map(|x| x.0).collect();
        match hit.as_slice() {
            [w] => out.push((*v, *w)),
            [] => return Err(corr(c, format!("vertex {v} has no image"))),
            _ => return Err(corr(c, format!("vertex {v} has several images"))),
        }
    }
    let mut imgs: Vec<EntityId> = out.iter().map(|x| x.1).collect();
    imgs.sort_unstable();
    if imgs.windows(2).any(|w| w[0] == w[1]) {
        return Err(corr(c, "two vertices map to one"));
    }
    Ok(out)
}

/// For each edge of the source face, the target-face edge whose ends are the images of its
/// ends, and whether it runs the same way.
pub fn edge_map(t: &dyn TopologyView, c: &CopyConstraint) -> Result<Vec<(EntityId, EntityId, bool)>> {
    let (Target::Surface(s), Target::Surface(d)) = (c.source, c.target) else {
        return Err(corr(c, "edge maps exist for surface copies only"));
    };
    let vm: BTreeMap<EntityId, EntityId> = c.vertex_map.iter().copied().collect();
    let mut out = Vec::new();
    for e in t.face_edges(s) {
        let Some((a, b)) = t.edge_ends(*e) else { continue };
        let (Some(&ma), Some(&mb)) = (vm.get(&a), vm.get(&b)) else {
            return Err(corr(c, format!("edge {e}: ends not in the vertex map")));
        };
        let mut found = None;
        for f in t.face_edges(d) {
            if let Some((x, y)) = t.edge_ends(*f) {
                if (x, y) == (ma, mb) {
                    found = Some((*f, true));
                } else if (x, y) == (mb, ma) {
                    found = Some((*f, false));
                }
                if found.is_some() {
                    break;
                }
            }
        }
        let (f, same) = found.ok_or_else(|| corr(c, format!("edge {e} has no image")))?;
        out.push((*e, f, same));
    }
    Ok(out)
}

/// Generate the target patch of a copy constraint from the source patch.
///
/// Keys: vertices map through `vertex_map`; nodes of a source edge map to the corresponding
/// target edge (index `i` or `n - i` when the edge runs the other way); nodes owned by the
/// source map to the target with the same index. Points are `transform(p)`; parameters are
/// dropped (`Param::None`). Surface elements are reversed when the mapped orientation
/// disagrees with the target face normal.
pub fn copy_patch(t: &dyn TopologyView, ps: &PatchSet, rec: &Record<CopyConstraint>, intervals: &dyn Fn(EntityId) -> Option<u32>) -> Result<Patch> {
    let c = &rec.item;
    let src = ps.get(c.source).ok_or_else(|| corr(c, "source patch not generated"))?;
    let (Some(s_ent), Some(d_ent)) = (c.source.entity(), c.target.entity()) else {
        return Err(corr(c, "groups cannot be copied"));
    };
    let vm: BTreeMap<EntityId, EntityId> = c.vertex_map.iter().copied().collect();
    // Edge map: source edge -> (target edge, same direction).
    let mut em: BTreeMap<EntityId, (EntityId, bool)> = BTreeMap::new();
    match (c.source, c.target) {
        (Target::Curve(a), Target::Curve(b)) => {
            let (Some((sa, _)), Some((ta, tb))) = (t.edge_ends(a), t.edge_ends(b)) else {
                return Err(corr(c, "edge without ends"));
            };
            let img = vm.get(&sa).copied().ok_or_else(|| corr(c, "start vertex not mapped"))?;
            let same = img != tb || ta == tb;
            em.insert(a, (b, same));
        }
        (Target::Surface(_), Target::Surface(_)) => {
            for (e, f, same) in edge_map(t, c)? {
                em.insert(e, (f, same));
            }
        }
        _ => return Err(corr(c, "only curve and surface copies are supported")),
    }
    let map_key = |k: NodeKey| -> Result<NodeKey> {
        if k.owner == s_ent && !matches!(c.source, Target::Curve(_)) {
            return Ok(NodeKey::new(d_ent, k.index));
        }
        match t.dim_of(k.owner) {
            Some(Dim::Vertex) => vm.get(&k.owner).map(|w| NodeKey::vertex(*w)).ok_or_else(|| corr(c, format!("vertex {} not mapped", k.owner))),
            Some(Dim::Edge) => {
                let (f, same) = *em.get(&k.owner).ok_or_else(|| corr(c, format!("edge {} not mapped", k.owner)))?;
                let n = intervals(f).ok_or_else(|| corr(c, format!("edge {f} has no intervals")))?;
                Ok(NodeKey::new(f, if same { k.index } else { n - k.index }))
            }
            _ => Err(corr(c, format!("node owner {} is outside the source", k.owner))),
        }
    };
    let mut out = Patch::new(c.target);
    for n in &src.nodes {
        out.nodes.push(PatchNode { key: map_key(n.key)?, point: c.transform.apply_point(n.point), param: Param::None });
    }
    let flip = match (c.source, c.target) {
        (Target::Surface(sf), Target::Surface(df)) => orientation_flip(t, sf, df, &c.transform)?,
        _ => false,
    };
    let mut buf: Vec<NodeKey> = Vec::with_capacity(8);
    for (shape, nodes) in src.elements.iter() {
        buf.clear();
        for k in nodes {
            buf.push(map_key(*k)?);
        }
        if flip && matches!(shape, ElemShape::Tri | ElemShape::Quad | ElemShape::Polygon) {
            buf[1..].reverse();
        }
        out.elements.push(shape, &buf);
    }
    if let Some(g) = &src.grid {
        let mut ng = g.clone();
        for k in &mut ng.nodes {
            *k = map_key(*k)?;
        }
        out.grid = Some(ng);
    }
    Ok(out)
}

/// True if elements oriented with the source normal, once mapped, point against the target
/// normal (`det(M) M n_s . n_t < 0`).
fn orientation_flip(t: &dyn TopologyView, s: EntityId, d: EntityId, g: &Transform) -> Result<bool> {
    let ps = t.face_outline(s).first().copied().ok_or_else(|| SchemeError::Input(format!("face {s} has no outline")))?;
    let pd = t.face_outline(d).first().copied().ok_or_else(|| SchemeError::Input(format!("face {d} has no outline")))?;
    let ns = geom::face_normal(t, s, ps)?;
    let nd = geom::face_normal(t, d, pd)?;
    let m = g.apply_vector(ns) * g.det().signum();
    Ok(m.dot(nd) < 0.0)
}
