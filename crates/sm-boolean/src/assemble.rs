//! Grouping face uses into shells and volumes, shared by the booleans and the webcut.

use std::collections::BTreeMap;

use sm_core::{EntityId, Point3};
use sm_topo_api::{CoedgeKind, FaceUseSpec, Sense, ShellKind, ShellSpec, Side, TopologyEdit, TopologyView, TopologyViewExt};

use crate::error::BooleanError;
use crate::faceclass;
use crate::geo;
use crate::locate::{self, PointLocation};

/// One result volume: its outer shell and its cavity shells, as face uses.
#[derive(Clone, Debug)]
pub(crate) struct ShellGroup {
    pub outer: Vec<(EntityId, Side)>,
    pub voids: Vec<Vec<(EntityId, Side)>>,
}

fn sense_of(side: Side) -> Sense {
    if side == Side::Left { Sense::Forward } else { Sense::Reversed }
}

/// Group `uses` (a face and the side the new region occupies) into closed shells by flood fill
/// across shared edges, tell the outer shells from the cavity walls by probing just inside each
/// shell against its normals, and give every cavity to the innermost outer shell around it.
/// Groups come ordered by the lowest face id of the outer shell. `probe` and `rule` only label
/// the error.
pub(crate) fn group_shells<V: TopologyView + ?Sized>(view: &V, uses: &[(EntityId, Side)], probe: EntityId, rule: &'static str) -> Result<Vec<ShellGroup>, BooleanError> {
    let invalid = || BooleanError::ResultInvalid { volume: probe, rule };
    let cache = faceclass::FaceCache::new();
    let lin = view.tolerance().linear;
    let tol = *view.tolerance();
    let n = uses.len();
    // Edge -> uses.
    let mut edge_uses: BTreeMap<EntityId, Vec<usize>> = BTreeMap::new();
    for (i, (f, _)) in uses.iter().enumerate() {
        for ce in view.coedges(*f) {
            if ce.kind == CoedgeKind::Pole {
                continue;
            }
            edge_uses.entry(ce.key.edge).or_default().push(i);
        }
    }
    if edge_uses.values().any(|us| us.len() != 2) {
        return Err(invalid());
    }
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(p: &mut [usize], x: usize) -> usize {
        let mut r = x;
        while p[r] != r {
            r = p[r];
        }
        let mut c = x;
        while p[c] != r {
            let nx = p[c];
            p[c] = r;
            c = nx;
        }
        r
    }
    for us in edge_uses.values() {
        let (a, b) = (find(&mut parent, us[0]), find(&mut parent, us[1]));
        if a != b {
            parent[a.max(b)] = a.min(b);
        }
    }
    let mut shells: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for i in 0..n {
        let r = find(&mut parent, i);
        shells.entry(r).or_default().push(i);
    }
    let mut shell_list: Vec<Vec<usize>> = shells.into_values().collect();
    for s in &mut shell_list {
        s.sort_by_key(|&i| uses[i].0);
    }
    shell_list.sort_by_key(|s| uses[s[0]].0);
    let shell_uses: Vec<Vec<locate::Use>> = shell_list.iter().map(|s| s.iter().map(|&i| (uses[i].0, sense_of(uses[i].1))).collect()).collect();
    let mut is_outer: Vec<bool> = Vec::new();
    for su in &shell_uses {
        let mut bbox = sm_core::Aabb::empty();
        for &(f, _) in su {
            bbox = bbox.union(&geo::face_aabb(view, f, 0.0));
        }
        let diag = geo::dist(bbox.lo, bbox.hi);
        let delta = (1e-3 * diag).max(100.0 * lin);
        let mut verdict: Option<bool> = None;
        for &(f, sense) in su {
            let Some(p) = faceclass::interior_point(view, &cache, f)? else { continue };
            let (nf, _, _) = geo::face_normal_near(view, f, p).ok_or(BooleanError::Unsupported("cannot evaluate a face normal"))?;
            let n_out = nf * sense.sign();
            let q = Point3::new(p.x - delta * n_out.x, p.y - delta * n_out.y, p.z - delta * n_out.z);
            match locate::locate_in_uses(view, &cache, su, q, &tol) {
                Ok(PointLocation::In) => {
                    verdict = Some(true);
                    break;
                }
                Ok(PointLocation::Out) => {
                    verdict = Some(false);
                    break;
                }
                _ => continue,
            }
        }
        is_outer.push(verdict.ok_or_else(invalid)?);
    }
    let outers: Vec<usize> = (0..shell_list.len()).filter(|&i| is_outer[i]).collect();
    if outers.is_empty() {
        return Err(invalid());
    }
    let mut void_of: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for i in 0..shell_list.len() {
        if is_outer[i] {
            continue;
        }
        let f = shell_uses[i][0].0;
        let p = faceclass::interior_point(view, &cache, f)?.ok_or_else(invalid)?;
        let mut best: Option<(f64, usize)> = None;
        for &o in &outers {
            if locate::locate_in_uses(view, &cache, &shell_uses[o], p, &tol)? == PointLocation::In {
                let mut bbox = sm_core::Aabb::empty();
                for &(g, _) in &shell_uses[o] {
                    bbox = bbox.union(&geo::face_aabb(view, g, 0.0));
                }
                let size = (bbox.hi.x - bbox.lo.x) * (bbox.hi.y - bbox.lo.y) * (bbox.hi.z - bbox.lo.z);
                if best.is_none_or(|b| size < b.0) {
                    best = Some((size, o));
                }
            }
        }
        let (_, o) = best.ok_or_else(invalid)?;
        void_of.entry(o).or_default().push(i);
    }
    let spec = |idx: usize| -> Vec<(EntityId, Side)> { shell_list[idx].iter().map(|&i| uses[i]).collect() };
    Ok(outers.iter().map(|&o| ShellGroup { outer: spec(o), voids: void_of.get(&o).map(|vs| vs.iter().map(|&v| spec(v)).collect()).unwrap_or_default() }).collect())
}

/// Make one volume per group.
pub(crate) fn make_volumes<E: TopologyEdit + ?Sized>(topo: &mut E, groups: &[ShellGroup]) -> Result<Vec<EntityId>, BooleanError> {
    let mut volumes = Vec::new();
    for g in groups {
        let to_specs = |u: &[(EntityId, Side)]| -> Vec<FaceUseSpec> { u.iter().map(|&(face, side)| FaceUseSpec { face, side }).collect() };
        let outer = to_specs(&g.outer);
        let voids: Vec<Vec<FaceUseSpec>> = g.voids.iter().map(|v| to_specs(v)).collect();
        let mut shells: Vec<ShellSpec<'_>> = vec![ShellSpec { kind: ShellKind::Outer, uses: &outer }];
        for v in &voids {
            shells.push(ShellSpec { kind: ShellKind::Void, uses: v });
        }
        volumes.push(topo.make_volume(&shells)?);
    }
    Ok(volumes)
}
