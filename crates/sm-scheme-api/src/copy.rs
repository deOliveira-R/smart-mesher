//! Copy constraints: correspondence construction, transform algebra, the built-in copy
//! generator (user copy, mirror, deck pairs, sweep targets) at curve and surface level, and the
//! post-generation verification of members that another job writes.

use crate::error::{Result, SchemeError};
use crate::geom;
use crate::patch::{ElemShape, Elements, NodeGrid, Patch, PatchNode, PatchSet};
use crate::plan::{CopyClass, CopyMember};
use crate::problem::{ConstraintId, CopyConstraint};
use crate::target::{NodeKey, Target};
use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_mesh_api::Param;
use sm_topo_api::{TopologyView, TopologyViewExt};
use std::collections::BTreeMap;

fn corr(source: Target, target: Target, reason: impl Into<String>) -> SchemeError {
    SchemeError::Correspondence {
        source_target: source,
        target,
        reason: reason.into(),
    }
}

/// `a` after `b` (apply `b` first): `p -> A (B p + tb) + ta`.
pub fn compose(a: &Transform, b: &Transform) -> Transform {
    let mut m = [[0.0; 3]; 3];
    for (i, row) in m.iter_mut().enumerate() {
        for (j, e) in row.iter_mut().enumerate() {
            *e = (0..3).map(|k| a.m[i][k] * b.m[k][j]).sum();
        }
    }
    let t = a.apply_vector(b.t) + a.t;
    Transform { m, t }
}

/// The inverse of a transform, or `None` if its linear part is singular.
pub fn inverse(a: &Transform) -> Option<Transform> {
    let m = &a.m;
    let det = a.det();
    if !det.is_finite() || det.abs() < 1e-300 {
        return None;
    }
    let c =
        |r0: usize, r1: usize, c0: usize, c1: usize| m[r0][c0] * m[r1][c1] - m[r0][c1] * m[r1][c0];
    let adj = [
        [c(1, 2, 1, 2), -c(0, 2, 1, 2), c(0, 1, 1, 2)],
        [-c(1, 2, 0, 2), c(0, 2, 0, 2), -c(0, 1, 0, 2)],
        [c(1, 2, 0, 1), -c(0, 2, 0, 1), c(0, 1, 0, 1)],
    ];
    let mut inv = [[0.0; 3]; 3];
    for (i, row) in inv.iter_mut().enumerate() {
        for (j, e) in row.iter_mut().enumerate() {
            *e = adj[i][j] / det;
        }
    }
    let linear = Transform {
        m: inv,
        t: Vector3::new(0.0, 0.0, 0.0),
    };
    let t = linear.apply_vector(a.t);
    Some(Transform {
        m: inv,
        t: Vector3::new(-t.x, -t.y, -t.z),
    })
}

/// Compose two sorted correspondences: `(a, b)` then `(b, c)` gives `(a, c)`.
pub fn compose_maps(
    first: &[(EntityId, EntityId)],
    second: &[(EntityId, EntityId)],
) -> Vec<(EntityId, EntityId)> {
    let s: BTreeMap<EntityId, EntityId> = second.iter().copied().collect();
    let mut out: Vec<(EntityId, EntityId)> = first
        .iter()
        .filter_map(|(a, b)| s.get(b).map(|c| (*a, *c)))
        .collect();
    out.sort_unstable();
    out
}

/// Swap the two sides of a correspondence (sorted by the new source).
pub fn invert_map(m: &[(EntityId, EntityId)]) -> Vec<(EntityId, EntityId)> {
    let mut out: Vec<(EntityId, EntityId)> = m.iter().map(|(a, b)| (*b, *a)).collect();
    out.sort_unstable();
    out
}

/// Vertices of a curve or surface target, ascending.
pub fn target_vertices(t: &dyn TopologyView, target: Target) -> Vec<EntityId> {
    let mut v: Vec<EntityId> = match target {
        Target::Curve(e) => t.edge_ends(e).map(|(a, b)| vec![a, b]).unwrap_or_default(),
        Target::Surface(f) => t
            .face_edges(f)
            .iter()
            .filter_map(|e| t.edge_ends(*e))
            .flat_map(|(a, b)| [a, b])
            .collect(),
        _ => Vec::new(),
    };
    v.sort_unstable();
    v.dedup();
    v
}

/// Build the vertex correspondence of a copy geometrically: every source vertex maps to the
/// target vertex at `transform(p)` within the summed vertex tolerances. Errors if a vertex
/// has no image or two share one.
pub fn derive_vertex_map(
    t: &dyn TopologyView,
    c: &CopyConstraint,
) -> Result<Vec<(EntityId, EntityId)>> {
    let src = target_vertices(t, c.source);
    let dst = target_vertices(t, c.target);
    let mut pts: Vec<(EntityId, Point3)> = Vec::with_capacity(dst.len());
    for w in &dst {
        pts.push((*w, geom::vertex_point(t, *w)?));
    }
    let mut out = Vec::with_capacity(src.len());
    for v in &src {
        let p = c.transform.apply_point(geom::vertex_point(t, *v)?);
        let hit: Vec<EntityId> = pts
            .iter()
            .filter(|(w, q)| (*q - p).norm() <= t.tolerance_of(*v) + t.tolerance_of(*w))
            .map(|x| x.0)
            .collect();
        match hit.as_slice() {
            [w] => out.push((*v, *w)),
            [] => return Err(corr(c.source, c.target, format!("vertex {v} has no image"))),
            _ => {
                return Err(corr(
                    c.source,
                    c.target,
                    format!("vertex {v} has several images"),
                ));
            }
        }
    }
    let mut imgs: Vec<EntityId> = out.iter().map(|x| x.1).collect();
    imgs.sort_unstable();
    if imgs.windows(2).any(|w| w[0] == w[1]) {
        return Err(corr(c.source, c.target, "two vertices map to one"));
    }
    out.sort_unstable();
    Ok(out)
}

/// For each edge of the source face, the target-face edge whose ends are the images of its
/// ends under the constraint's vertex map, and whether it runs the same way. Edges sharing
/// the same ends (an arc and its chord, two arcs of a circle) are told apart by the edge
/// midpoint against `transform(source midpoint)`.
pub fn edge_map(
    t: &dyn TopologyView,
    c: &CopyConstraint,
) -> Result<Vec<(EntityId, EntityId, bool)>> {
    edge_map_for(t, c.source, c.target, &c.vertex_map, &c.transform)
}

/// True if `transform` maps the direction of edge `e` onto that of edge `f` (tangents at the
/// midpoints). Decides the direction of closed curves, whose ends cannot.
fn same_direction(t: &dyn TopologyView, e: EntityId, f: EntityId, g: &Transform) -> Result<bool> {
    Ok(g.apply_vector(geom::edge_tangent(t, e, 0.5)?)
        .dot(geom::edge_tangent(t, f, 0.5)?)
        >= 0.0)
}

/// [`edge_map`] for an explicit pair of faces, vertex correspondence and transform (class
/// members).
pub fn edge_map_for(
    t: &dyn TopologyView,
    source: Target,
    target: Target,
    vertex_map: &[(EntityId, EntityId)],
    transform: &Transform,
) -> Result<Vec<(EntityId, EntityId, bool)>> {
    let (Target::Surface(s), Target::Surface(d)) = (source, target) else {
        return Err(corr(
            source,
            target,
            "edge maps exist for surface copies only",
        ));
    };
    let vm: BTreeMap<EntityId, EntityId> = vertex_map.iter().copied().collect();
    let mut out: Vec<(EntityId, EntityId, bool)> = Vec::new();
    for e in t.face_edges(s) {
        let Some((a, b)) = t.edge_ends(*e) else {
            continue;
        };
        let (Some(&ma), Some(&mb)) = (vm.get(&a), vm.get(&b)) else {
            return Err(corr(
                source,
                target,
                format!("edge {e}: ends not in the vertex map"),
            ));
        };
        let mut cands: Vec<(EntityId, bool)> = Vec::new();
        for f in t.face_edges(d) {
            if let Some((x, y)) = t.edge_ends(*f) {
                if (x, y) == (ma, mb) {
                    cands.push((*f, true));
                } else if (x, y) == (mb, ma) {
                    cands.push((*f, false));
                }
            }
        }
        cands.dedup_by_key(|c| c.0);
        let (f, mut same) = match cands.as_slice() {
            [] => return Err(corr(source, target, format!("edge {e} has no image"))),
            [one] => *one,
            many => {
                let want = transform.apply_point(geom::edge_point(t, *e, 0.5)?);
                let mut best: Option<(f64, (EntityId, bool))> = None;
                for &(f, same) in many {
                    if out.iter().any(|o| o.1 == f) {
                        continue;
                    }
                    let dist = (geom::edge_point(t, f, 0.5)? - want).norm();
                    if dist <= t.tolerance_of(*e) + t.tolerance_of(f)
                        && best.is_none_or(|b| dist < b.0)
                    {
                        best = Some((dist, (f, same)));
                    }
                }
                best.ok_or_else(|| {
                    corr(
                        source,
                        target,
                        format!(
                            "edge {e}: several candidate images, none at the transformed midpoint"
                        ),
                    )
                })?
                .1
            }
        };
        if ma == mb {
            same = same_direction(t, *e, f, transform)?;
        }
        out.push((*e, f, same));
    }
    Ok(out)
}

/// Maps node keys of a source face or curve to the target's, through the vertex and edge
/// correspondences. Nodes owned by the source entity keep their index.
struct KeyMapper {
    source: Target,
    target: Target,
    s_ent: EntityId,
    d_ent: EntityId,
    vm: BTreeMap<EntityId, EntityId>,
    em: BTreeMap<EntityId, (EntityId, bool, u32)>,
}

impl KeyMapper {
    fn new(
        t: &dyn TopologyView,
        source: Target,
        target: Target,
        vertex_map: &[(EntityId, EntityId)],
        transform: &Transform,
        intervals: &dyn Fn(EntityId) -> Option<u32>,
    ) -> Result<Self> {
        let (Some(s_ent), Some(d_ent)) = (source.entity(), target.entity()) else {
            return Err(corr(source, target, "groups cannot be copied"));
        };
        let vm: BTreeMap<EntityId, EntityId> = vertex_map.iter().copied().collect();
        let mut em: BTreeMap<EntityId, (EntityId, bool, u32)> = BTreeMap::new();
        let n_of = |f: EntityId| {
            intervals(f).ok_or_else(|| corr(source, target, format!("edge {f} has no intervals")))
        };
        match (source, target) {
            (Target::Curve(a), Target::Curve(b)) => {
                let (Some((sa, _)), Some((ta, tb))) = (t.edge_ends(a), t.edge_ends(b)) else {
                    return Err(corr(source, target, "edge without ends"));
                };
                let img = vm
                    .get(&sa)
                    .copied()
                    .ok_or_else(|| corr(source, target, "start vertex not mapped"))?;
                let same = if ta == tb {
                    same_direction(t, a, b, transform)?
                } else {
                    img != tb
                };
                em.insert(a, (b, same, n_of(b)?));
            }
            (Target::Surface(_), Target::Surface(_)) => {
                for (e, f, same) in edge_map_for(t, source, target, vertex_map, transform)? {
                    em.insert(e, (f, same, n_of(f)?));
                }
            }
            _ => {
                return Err(corr(
                    source,
                    target,
                    "only curve and surface copies are supported",
                ));
            }
        }
        Ok(Self {
            source,
            target,
            s_ent,
            d_ent,
            vm,
            em,
        })
    }

    fn map(&self, k: NodeKey) -> Result<NodeKey> {
        if k.owner == self.s_ent && !matches!(self.source, Target::Curve(_)) {
            return Ok(NodeKey::new(self.d_ent, k.index));
        }
        if let Some(w) = self.vm.get(&k.owner) {
            return Ok(NodeKey::vertex(*w));
        }
        match self.em.get(&k.owner) {
            Some((f, same, n)) => Ok(NodeKey::new(
                *f,
                if *same {
                    k.index
                } else {
                    n.saturating_sub(k.index)
                },
            )),
            None => Err(corr(
                self.source,
                self.target,
                format!("node owner {} is outside the source", k.owner),
            )),
        }
    }
}

/// Generate the target patch of a copy from the source patch: `transform` places the nodes,
/// keys map through the vertex and edge correspondences (nodes of the source entity keep their
/// index), parameters are dropped (`Param::None`) and surface elements are reversed when the
/// mapped orientation disagrees with the target face normal.
pub fn copy_patch(
    t: &dyn TopologyView,
    ps: &PatchSet,
    source: Target,
    target: Target,
    transform: &Transform,
    vertex_map: &[(EntityId, EntityId)],
    intervals: &dyn Fn(EntityId) -> Option<u32>,
) -> Result<Patch> {
    let src = ps
        .get(source)
        .ok_or_else(|| corr(source, target, "source patch not generated"))?;
    let km = KeyMapper::new(t, source, target, vertex_map, transform, intervals)?;
    let mut out = Patch::new(target);
    out.nodes.reserve(src.nodes.len());
    for n in &src.nodes {
        out.nodes.push(PatchNode {
            key: km.map(n.key)?,
            point: transform.apply_point(n.point),
            param: Param::None,
        });
    }
    out.refs.reserve(src.refs.len());
    for r in &src.refs {
        out.refs.push(km.map(*r)?);
    }
    let flip = match (source, target) {
        (Target::Surface(sf), Target::Surface(df)) => orientation_flip(t, sf, df, transform)?,
        _ => false,
    };
    out.elements = Elements::with_capacity(src.elements.len(), 4);
    let mut buf: Vec<u32> = Vec::with_capacity(8);
    for (i, (shape, nodes)) in src.elements.iter().enumerate() {
        if shape == ElemShape::Polyhedron {
            let faces: Vec<&[u32]> = src
                .elements
                .polyhedron_faces(i)
                .into_iter()
                .flatten()
                .collect();
            out.elements.push_polyhedron(&faces);
            continue;
        }
        buf.clear();
        buf.extend_from_slice(nodes);
        if flip
            && matches!(shape, ElemShape::Tri | ElemShape::Quad | ElemShape::Polygon)
            && buf.len() > 1
        {
            buf[1..].reverse();
        }
        out.elements.push(shape, &buf);
    }
    if let Some(g) = &src.grid {
        let mut nodes = Vec::with_capacity(g.nodes.len());
        for k in &g.nodes {
            nodes.push(km.map(*k)?);
        }
        out.grid = Some(NodeGrid {
            ni: g.ni,
            nj: g.nj,
            nodes,
        });
    }
    Ok(out)
}

/// True if elements oriented with the source normal, once mapped, point against the target
/// normal (`det(M) M n_s . n_t < 0`).
fn orientation_flip(t: &dyn TopologyView, s: EntityId, d: EntityId, g: &Transform) -> Result<bool> {
    let ps = t
        .face_outline(s)
        .first()
        .copied()
        .ok_or_else(|| SchemeError::Input(format!("face {s} has no outline")))?;
    let ns = geom::face_normal(t, s, ps)?;
    // Target normal at the image of the same point (matters for curved faces).
    let nd = geom::face_normal(t, d, g.apply_point(ps))?;
    let m = g.apply_vector(ns) * g.det().signum();
    Ok(m.dot(nd) < 0.0)
}

/// Canonical element list: flat key storage and element ranges sorted by their key lists.
type Canon = (Vec<NodeKey>, Vec<(usize, usize)>);

fn canonical_elements(p: &Patch, map: &dyn Fn(NodeKey) -> Result<NodeKey>) -> Result<Canon> {
    // Flat: keys of all elements back to back, each element sorted, plus its end offset.
    let mut keys: Vec<NodeKey> = Vec::new();
    let mut ends: Vec<usize> = Vec::with_capacity(p.elements.len());
    for (_, nodes) in p.elements.iter() {
        let start = keys.len();
        for &l in nodes {
            let k = p.key(l).ok_or_else(|| SchemeError::InvalidPatch {
                target: p.target,
                reason: format!("local index {l} out of range"),
            })?;
            keys.push(map(k)?);
        }
        keys[start..].sort_unstable();
        ends.push(keys.len());
    }
    let mut ranges: Vec<(usize, usize)> = ends
        .iter()
        .scan(0, |s, &e| {
            let r = (*s, e);
            *s = e;
            Some(r)
        })
        .collect();
    ranges.sort_unstable_by(|a, b| keys[a.0..a.1].cmp(&keys[b.0..b.1]));
    Ok((keys, ranges))
}

/// Check that the patch of `m.target` equals the image of the class root's patch: the same
/// node keys at `from_root(p)` within `tol`, and the same elements by key. Used for members
/// that a parent writes (claimed) instead of the driver. The constraint reported on failure is
/// the last one on the member's path.
pub fn verify_copy(
    t: &dyn TopologyView,
    ps: &PatchSet,
    class: &CopyClass,
    m: &CopyMember,
    tol: f64,
) -> Result<()> {
    let last = m.constraints.last().copied().unwrap_or(ConstraintId(0));
    let mismatch = |reason: String| SchemeError::CopyMismatch {
        constraint: last,
        reason,
    };
    let src = ps
        .get(class.root)
        .ok_or_else(|| mismatch(format!("root {:?} has no patch", class.root)))?;
    let dst = ps
        .get(m.target)
        .ok_or_else(|| mismatch(format!("{:?} has no patch", m.target)))?;
    let n_of = |e: EntityId| ps.get(Target::Curve(e)).map(|p| p.nodes.len() as u32 + 1);
    let km = KeyMapper::new(t, class.root, m.target, &m.vertex_map, &m.from_root, &n_of)?;
    for n in &src.nodes {
        let k = km.map(n.key)?;
        let want = m.from_root.apply_point(n.point);
        let got = ps
            .point(k)
            .ok_or_else(|| mismatch(format!("image node {k:?} of {:?} does not exist", n.key)))?;
        if (got - want).norm() > tol {
            return Err(mismatch(format!(
                "node {k:?} is {:.3e} away from the image of {:?}",
                (got - want).norm(),
                n.key
            )));
        }
    }
    if src.nodes.len() != dst.nodes.len() {
        return Err(mismatch(format!(
            "{} nodes against {} in the root",
            dst.nodes.len(),
            src.nodes.len()
        )));
    }
    let a = canonical_elements(src, &|k| km.map(k))?;
    let b = canonical_elements(dst, &|k| Ok(k))?;
    let same = a.1.len() == b.1.len()
        && a.1
            .iter()
            .zip(&b.1)
            .all(|(x, y)| a.0[x.0..x.1] == b.0[y.0..y.1]);
    if !same {
        return Err(mismatch(format!(
            "element connectivity differs ({} elements against {})",
            b.1.len(),
            a.1.len()
        )));
    }
    Ok(())
}
