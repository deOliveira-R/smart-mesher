//! Copy equivalence classes (defect D2). Copy constraints are edges of an equivalence relation
//! between faces (or between curves), each carrying its transform and vertex correspondence.
//! A class is meshed from one root; every other member derives from it by the composition of
//! the constraints along a spanning tree, and every redundant constraint is only checked.

use crate::copy::{compose_maps, derive_vertex_map, invert_map, target_vertices};
use crate::error::{Result, SchemeError};
use crate::geom;
use crate::plan::{CopyClass, CopyMember, CopyMode};
use crate::problem::{ConstraintId, CopyConstraint, Record};
use crate::target::Target;
use sm_core::{EntityId, Transform};
use sm_topo_api::TopologyView;
use std::collections::{BTreeMap, BTreeSet};

/// What the root preference and the redundancy check need to know.
pub(crate) struct ClassInput<'a> {
    pub topo: &'a dyn TopologyView,
    pub copies: &'a [Record<CopyConstraint>],
    /// Targets written by a parent (claimed).
    pub claimed: &'a BTreeSet<Target>,
    /// Targets already planned with a scheme of their own.
    pub resolved: &'a BTreeSet<Target>,
    /// Targets with `Role::SweepSource`.
    pub sweep_sources: &'a BTreeSet<Target>,
    /// Targets with a hard (non-copy) scheme intent.
    pub hard: &'a BTreeSet<Target>,
    /// Targets whose intent is the `copy` scheme (never roots).
    pub copy_intent: &'a BTreeSet<Target>,
    /// Tolerance of the redundancy checks.
    pub tol: f64,
}

struct Uf(Vec<usize>);

impl Uf {
    fn find(&mut self, mut i: usize) -> usize {
        while self.0[i] != i {
            self.0[i] = self.0[self.0[i]];
            i = self.0[i];
        }
        i
    }
    fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            self.0[ra.max(rb)] = ra.min(rb);
        }
    }
}

/// Group the copy constraints into classes (ascending by root) and check every redundant
/// constraint against the composed one.
pub(crate) fn build_classes(inp: &ClassInput<'_>) -> Result<Vec<CopyClass>> {
    let nodes: Vec<Target> = inp
        .copies
        .iter()
        .flat_map(|r| [r.item.source, r.item.target])
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    let ix = |t: &Target| nodes.binary_search(t).ok();
    let mut uf = Uf((0..nodes.len()).collect());
    let mut adj: Vec<Vec<(usize, usize, bool)>> = vec![Vec::new(); nodes.len()];
    for (ri, r) in inp.copies.iter().enumerate() {
        let (Some(a), Some(b)) = (ix(&r.item.source), ix(&r.item.target)) else {
            continue;
        };
        uf.union(a, b);
        adj[a].push((b, ri, true));
        adj[b].push((a, ri, false));
    }
    let mut comps: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for i in 0..nodes.len() {
        let r = uf.find(i);
        comps.entry(r).or_default().push(i);
    }
    let mut out = Vec::new();
    for members in comps.into_values() {
        out.push(build_one(inp, &nodes, &adj, &members)?);
    }
    out.sort_by_key(|c| c.root);
    Ok(out)
}

fn root_key(inp: &ClassInput<'_>, t: Target) -> (u8, u8, u8, u8, u8, Target) {
    (
        u8::from(inp.copy_intent.contains(&t)),
        u8::from(!inp.resolved.contains(&t)),
        u8::from(!inp.sweep_sources.contains(&t)),
        u8::from(!inp.claimed.contains(&t)),
        u8::from(!inp.hard.contains(&t)),
        t,
    )
}

struct Placed {
    from_root: Transform,
    vm: Vec<(EntityId, EntityId)>,
    path: Vec<ConstraintId>,
}

fn build_one(
    inp: &ClassInput<'_>,
    nodes: &[Target],
    adj: &[Vec<(usize, usize, bool)>],
    members: &[usize],
) -> Result<CopyClass> {
    let root_ix = members
        .iter()
        .copied()
        .min_by_key(|&i| root_key(inp, nodes[i]))
        .ok_or_else(|| SchemeError::Input("empty copy class".into()))?;
    let root = nodes[root_ix];
    let root_vs = target_vertices(inp.topo, root);
    let mut placed: BTreeMap<usize, Placed> = BTreeMap::new();
    placed.insert(
        root_ix,
        Placed {
            from_root: Transform::IDENTITY,
            vm: root_vs.iter().map(|v| (*v, *v)).collect(),
            path: Vec::new(),
        },
    );
    let mut tree: BTreeSet<usize> = BTreeSet::new();
    let mut queue = std::collections::VecDeque::from([root_ix]);
    while let Some(u) = queue.pop_front() {
        for &(v, ri, forward) in &adj[u] {
            if placed.contains_key(&v) {
                continue;
            }
            let r = &inp.copies[ri];
            let pu = &placed[&u];
            let (from_root, vm) = if forward {
                (
                    r.item.transform.compose(&pu.from_root),
                    compose_maps(&pu.vm, &r.item.vertex_map),
                )
            } else {
                let inv =
                    r.item.transform.inverse().ok_or_else(|| SchemeError::Correspondence {
                        source_target: r.item.source,
                        target: r.item.target,
                        reason: "singular copy transform".into(),
                    })?;
                (
                    inv.compose(&pu.from_root),
                    compose_maps(&pu.vm, &invert_map(&r.item.vertex_map)),
                )
            };
            let mut path = pu.path.clone();
            path.push(r.id);
            tree.insert(ri);
            placed.insert(
                v,
                Placed {
                    from_root,
                    vm,
                    path,
                },
            );
            queue.push_back(v);
        }
    }
    // Redundant constraints must agree with the composed placement.
    for (ri, r) in inp.copies.iter().enumerate() {
        if tree.contains(&ri) {
            continue;
        }
        let (Ok(a), Ok(b)) = (
            nodes.binary_search(&r.item.source),
            nodes.binary_search(&r.item.target),
        ) else {
            continue;
        };
        let (Some(ps), Some(pd)) = (placed.get(&a), placed.get(&b)) else {
            continue;
        };
        let first = pd.path.last().or(ps.path.last()).copied().unwrap_or(r.id);
        let conflict = |reason: String| SchemeError::CopyConflict {
            target: r.item.target,
            first,
            second: r.id,
            reason,
        };
        let vmap: BTreeMap<EntityId, EntityId> = r.item.vertex_map.iter().copied().collect();
        let d_of: BTreeMap<EntityId, EntityId> = pd.vm.iter().copied().collect();
        for (rv, sv) in &ps.vm {
            let Some(want) = vmap.get(sv) else { continue };
            match d_of.get(rv) {
                Some(have) if have == want => {}
                other => {
                    return Err(conflict(format!(
                        "vertex {rv} of the root maps to {other:?} along one path and to {want} along the other"
                    )));
                }
            }
            let p = geom::vertex_point(inp.topo, *rv)?;
            let via_s = r.item.transform.apply_point(ps.from_root.apply_point(p));
            let via_d = pd.from_root.apply_point(p);
            if (via_s - via_d).norm() > inp.tol {
                return Err(conflict(format!(
                    "the two paths place vertex {rv} {:.3e} apart (tolerance {:.1e})",
                    (via_s - via_d).norm(),
                    inp.tol
                )));
            }
        }
    }
    let mut ms = Vec::new();
    for (&i, pl) in &placed {
        if i == root_ix {
            continue;
        }
        let t = nodes[i];
        if inp.resolved.contains(&t) {
            let last = pl.path.last().copied().unwrap_or(ConstraintId(0));
            return Err(SchemeError::CopyConflict {
                target: t,
                first: last,
                second: last,
                reason: "the target already has a scheme of its own".into(),
            });
        }
        let mode = if inp.claimed.contains(&t) {
            CopyMode::Verify
        } else {
            CopyMode::Generate
        };
        ms.push(CopyMember {
            target: t,
            from_root: pl.from_root,
            vertex_map: pl.vm.clone(),
            mode,
            constraints: pl.path.clone(),
        });
    }
    ms.sort_by_key(|m| m.target);
    Ok(CopyClass { root, members: ms })
}

/// Fill the vertex correspondence of every copy that has none (geometrically, from its
/// transform). Returns the number filled.
pub(crate) fn fill_vertex_maps(
    topo: &dyn TopologyView,
    copies: &mut [Record<CopyConstraint>],
) -> Result<usize> {
    let mut n = 0;
    for r in copies.iter_mut() {
        if r.item.vertex_map.is_empty() {
            r.item.vertex_map = derive_vertex_map(topo, &r.item)?;
            n += 1;
        }
    }
    Ok(n)
}
