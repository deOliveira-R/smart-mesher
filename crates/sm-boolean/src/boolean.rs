//! Booleans: unite, subtract, intersect.
//!
//! One pair of volumes at a time (lists fold): imprint the two boundaries on each other with the
//! shared kernel, classify every fragment against the other operand, select fragments by rule,
//! weld the coincident cells of the selected fragments, assemble the result volumes, and
//! regularize. See `DESIGN.md` for the classification table.

use std::collections::{BTreeMap, BTreeSet};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, Point3, Vector3};
use sm_topo_api::{CoedgeKind, Dim, FaceUseSpec, Sense, ShellKind, ShellSpec, Side, TopologyEdit, TopologyView, TopologyViewExt};

use crate::arrange;
use crate::bodies;
use crate::error::BooleanError;
use crate::faceclass;
use crate::geo;
use crate::locate::{self, PointLocation};
use crate::merge::{MergeKinds, MergeOptions, merge};
use crate::pieces::{self, Other, PairSpec, Share};

/// The boolean rule.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum BoolRule {
    /// `A u B`.
    Unite,
    /// `A - B`.
    Subtract,
    /// `A n B`.
    Intersect,
}

impl BoolRule {
    fn name(self) -> &'static str {
        match self {
            BoolRule::Unite => "unite",
            BoolRule::Subtract => "subtract",
            BoolRule::Intersect => "intersect",
        }
    }
}

/// Options of [`boolean`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct BoolOptions {
    /// Keep the operands unchanged: the kernel works on deep copies (which share the curve and
    /// surface handles with the originals; a later `replace_surface` on one affects both).
    pub keep: bool,
    /// Merge the coplanar and co-surface fragments of the result (`join_faces`, `join_edges`).
    pub regularize: bool,
    /// Weld the coincident vertices and edges of the selected fragments (default true; without
    /// it the result is not conformal where the operands' boundaries meet).
    pub merge: bool,
    /// Merge tolerance of the weld (see `MergeOptions::tol`).
    pub max_gap: Option<f64>,
}

impl Default for BoolOptions {
    fn default() -> Self {
        Self { keep: false, regularize: true, merge: true, max_gap: None }
    }
}

/// What [`boolean`] produced; every id is alive.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BoolOutcome {
    /// The result bodies, one per result volume.
    pub bodies: Vec<EntityId>,
    /// The result volumes.
    pub volumes: Vec<EntityId>,
    /// Result faces that lie on the other operand's surfaces (the faces the operation created).
    pub new_faces: Vec<EntityId>,
    /// The remaining faces of the result volumes.
    pub faces: Vec<EntityId>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Class {
    In,
    Out,
    OnSame,
    OnOpposite,
}

/// Outward unit normal of volume `own` at the face point nearest `p`.
fn outward_normal<V: TopologyView + ?Sized>(view: &V, f: EntityId, own: EntityId, p: Point3) -> Result<Vector3, BooleanError> {
    let (n, _, _) = geo::face_normal_near(view, f, p).ok_or(BooleanError::Unsupported("cannot evaluate a face normal"))?;
    let sense = view.face_use_in(f, own).first().copied().unwrap_or(Sense::Forward);
    Ok(n * sense.sign())
}

fn use_side<V: TopologyView + ?Sized>(view: &V, f: EntityId, vol: EntityId) -> Option<Side> {
    view.coboundary(f).iter().find(|e| e.cell == vol).map(|e| e.side)
}

fn classify_frag<V: TopologyView + ?Sized>(view: &V, f: EntityId, own: EntityId, other: EntityId, other_uses: &[locate::Use]) -> Result<Class, BooleanError> {
    let tol = *view.tolerance();
    let lin = tol.linear;
    let p = faceclass::interior_point(view, f)?.ok_or(BooleanError::Unsupported("cannot find a point inside a fragment"))?;
    let n_own = outward_normal(view, f, own, p)?;
    for &(g, _) in other_uses {
        if locate::on_face(view, g, p, lin)? {
            let n_g = outward_normal(view, g, other, p)?;
            return Ok(if n_own.dot(n_g) > 0.0 { Class::OnSame } else { Class::OnOpposite });
        }
    }
    match locate::locate_in_uses(view, other_uses, p, &tol)? {
        PointLocation::In => Ok(Class::In),
        PointLocation::Out => Ok(Class::Out),
        PointLocation::On => Err(BooleanError::Ambiguous { face: f, point: p }),
    }
}

/// One kept face use of the result.
#[derive(Clone, Copy, Debug)]
struct Kept {
    face: EntityId,
    side: Side,
    from_b: bool,
}

struct Stage {
    volumes: Vec<EntityId>,
    bodies: Vec<EntityId>,
    new_faces: BTreeSet<EntityId>,
}

/// Apply `rule` to the volume sets `a` and `b` (volumes or bodies).
///
/// Unite and intersect fold over all operands of `a` then `b`; subtract takes every volume of `a`
/// as a target and removes every volume of `b` from each piece in turn. Each result volume is its
/// own body; the lineage declares what became what (`record_merged` for unite and intersect,
/// `record_split` for subtract) so colors, groups and blocks follow.
///
/// Runs inside an open operation; on `Err` the caller aborts it. Inputs must be manifold solids
/// ([`BooleanError::NonManifoldInput`]); unite of bodies that do not touch is
/// `Invalid("bodies do not touch")` and an empty intersection is [`BooleanError::EmptyResult`].
///
/// # Errors
/// See [`BooleanError`]; notably `Ambiguous` when a fragment cannot be classified,
/// `Unsupported` for tangent-only contacts and for geometry the arrangement cannot express, and
/// `ResultInvalid` when an assembled volume is open or has no positive volume.
pub fn boolean<E: TopologyEdit + ?Sized>(topo: &mut E, rule: BoolRule, a: &[EntityId], b: &[EntityId], o: &BoolOptions) -> Result<BoolOutcome, BooleanError> {
    let va = bodies::volumes_of(&*topo, a)?;
    let vb = bodies::volumes_of(&*topo, b)?;
    if va.is_empty() {
        return Err(BooleanError::Invalid("the first operand list is empty".into()));
    }
    if rule != BoolRule::Unite && vb.is_empty() || (rule == BoolRule::Unite && va.len() + vb.len() < 2) {
        return Err(BooleanError::Invalid("a boolean needs at least two operands".into()));
    }
    if va.iter().any(|v| vb.contains(v)) {
        return Err(BooleanError::Invalid("a volume cannot be both operands".into()));
    }
    for &v in va.iter().chain(&vb) {
        bodies::check_manifold(&*topo, v)?;
    }
    let (va, vb) = if o.keep { (copy_all(topo, &va)?, copy_all(topo, &vb)?) } else { (va, vb) };
    let mut out = BoolOutcome::default();
    let mut new_faces: BTreeSet<EntityId> = BTreeSet::new();
    match rule {
        BoolRule::Unite | BoolRule::Intersect => {
            let mut all: Vec<EntityId> = va.clone();
            all.extend(vb.iter().copied());
            let mut cur = all[0];
            let mut stage: Option<Stage> = None;
            for &next in &all[1..] {
                let s = fuse2(topo, rule, cur, next, o)?;
                new_faces = s.new_faces.clone();
                if s.volumes.len() != 1 {
                    return Err(BooleanError::Unsupported("the result has several volumes"));
                }
                cur = s.volumes[0];
                stage = Some(s);
            }
            let s = stage.ok_or(BooleanError::EmptyResult)?;
            out.volumes = s.volumes;
            out.bodies = s.bodies;
        }
        BoolRule::Subtract => {
            let mut current: Vec<EntityId> = va.clone();
            let mut bodies_now: BTreeMap<EntityId, EntityId> = BTreeMap::new();
            for &tool in &vb {
                let mut next: Vec<EntityId> = Vec::new();
                let mut any_overlap = false;
                for &t in &current {
                    let s = fuse2(topo, rule, t, tool, o)?;
                    any_overlap = true;
                    for (v, nb) in s.volumes.iter().zip(&s.bodies) {
                        bodies_now.insert(*v, *nb);
                    }
                    new_faces.extend(s.new_faces.iter().copied());
                    next.extend(s.volumes);
                }
                let _ = any_overlap;
                if next.is_empty() {
                    return Err(BooleanError::EmptyResult);
                }
                current = next;
            }
            out.bodies = current.iter().filter_map(|v| bodies_now.get(v).copied()).collect();
            out.volumes = current;
        }
    }
    // Roles.
    let mut faces: BTreeSet<EntityId> = BTreeSet::new();
    for &v in &out.volumes {
        faces.extend(topo.volume_faces(v).iter().copied());
    }
    out.new_faces = new_faces.iter().copied().filter(|f| faces.contains(f)).collect();
    out.faces = faces.iter().copied().filter(|f| !new_faces.contains(f)).collect();
    out.volumes.sort();
    out.bodies.sort();
    Ok(out)
}

fn copy_all<E: TopologyEdit + ?Sized>(topo: &mut E, vols: &[EntityId]) -> Result<Vec<EntityId>, BooleanError> {
    let mut out = Vec::new();
    for &v in vols {
        out.push(crate::copy::copy_volume(topo, v)?);
    }
    Ok(out)
}

fn fuse2<E: TopologyEdit + ?Sized>(topo: &mut E, rule: BoolRule, va: EntityId, vb: EntityId, o: &BoolOptions) -> Result<Stage, BooleanError> {
    let tol = *topo.tolerance();
    let lin = tol.linear;
    let body_of = |t: &E, v: EntityId| -> Option<EntityId> { t.coboundary(v).iter().find(|e| t.dim_of(e.cell) == Some(Dim::Body)).map(|e| e.cell) };
    let (body_a, body_b) = (body_of(&*topo, va), body_of(&*topo, vb));
    for (b, v) in [(body_a, va), (body_b, vb)] {
        if let Some(b) = b {
            let members: Vec<EntityId> = topo.boundary(b).iter().map(|e| e.cell).collect();
            if members.iter().any(|m| *m != v) {
                return Err(BooleanError::Unsupported("a boolean operand shares its body with other cells: separate them first"));
            }
        }
    }
    // 1-4. Imprint the boundaries on each other.
    let fa: Vec<EntityId> = topo.volume_faces(va).to_vec();
    let fb: Vec<EntityId> = topo.volume_faces(vb).to_vec();
    let pairs: Vec<PairSpec> = pieces::candidate_pairs(&*topo, &fa, &fb, tol.linear_max).into_iter().map(|(a, b)| PairSpec { a, b: Other::Face(b) }).collect();
    let computed = pieces::compute(&*topo, &pairs, Share::Shared, &|_| 0, 10.0 * tol.linear_max)?;
    if computed.pieces.is_empty() && computed.coincident.is_empty() && !computed.tangent_points.is_empty() {
        return Err(BooleanError::Unsupported("tangent-only contact between the operands: perturb one of them"));
    }
    if !computed.pieces.is_empty() {
        let ps: Vec<&pieces::Piece> = computed.pieces.iter().collect();
        let order: Vec<EntityId> = fa.iter().chain(fb.iter()).copied().collect();
        arrange::apply(topo, &ps, lin, &order)?;
    }
    // 5. Classification.
    let fa_now: Vec<EntityId> = topo.volume_faces(va).to_vec();
    let fb_now: Vec<EntityId> = topo.volume_faces(vb).to_vec();
    let uses_a = locate::volume_uses(&*topo, va);
    let uses_b = locate::volume_uses(&*topo, vb);
    let mut class_a: Vec<(EntityId, Class)> = Vec::new();
    for &f in &fa_now {
        class_a.push((f, classify_frag(&*topo, f, va, vb, &uses_b)?));
    }
    let mut class_b: Vec<(EntityId, Class)> = Vec::new();
    for &f in &fb_now {
        class_b.push((f, classify_frag(&*topo, f, vb, va, &uses_a)?));
    }
    let all_out = class_a.iter().chain(&class_b).all(|(_, c)| *c == Class::Out);
    if all_out && computed.pieces.is_empty() {
        match rule {
            BoolRule::Unite => return Err(BooleanError::Invalid("bodies do not touch: unite needs operands that overlap or share a face".into())),
            BoolRule::Intersect => return Err(BooleanError::EmptyResult),
            BoolRule::Subtract => {
                // Nothing to remove: the target stays, the tool is consumed.
                if let Some(b) = body_b {
                    topo.kill_cascade(b)?;
                } else {
                    topo.kill_cascade(vb)?;
                }
                return Ok(Stage { volumes: vec![va], bodies: body_a.into_iter().collect(), new_faces: BTreeSet::new() });
            }
        }
    }
    // 6. Selection.
    let mut kept: Vec<Kept> = Vec::new();
    for &(f, c) in &class_a {
        let keep = match rule {
            BoolRule::Unite => matches!(c, Class::Out | Class::OnSame),
            BoolRule::Intersect => matches!(c, Class::In | Class::OnSame),
            BoolRule::Subtract => matches!(c, Class::Out | Class::OnOpposite),
        };
        if keep {
            let side = use_side(&*topo, f, va).ok_or(BooleanError::Unsupported("a fragment is not used by its volume"))?;
            kept.push(Kept { face: f, side, from_b: false });
        }
    }
    for &(f, c) in &class_b {
        let (keep, flip) = match rule {
            BoolRule::Unite => (c == Class::Out, false),
            BoolRule::Intersect => (c == Class::In, false),
            BoolRule::Subtract => (c == Class::In, true),
        };
        if keep {
            let side = use_side(&*topo, f, vb).ok_or(BooleanError::Unsupported("a fragment is not used by its volume"))?;
            kept.push(Kept { face: f, side: if flip { side.flip() } else { side }, from_b: true });
        }
    }
    if kept.is_empty() {
        return Err(BooleanError::EmptyResult);
    }
    let kept_faces: BTreeSet<EntityId> = kept.iter().map(|k| k.face).collect();
    let b_faces: BTreeSet<EntityId> = kept.iter().filter(|k| k.from_b).map(|k| k.face).collect();
    // 7. Take the operands apart: free the face sides, delete the volumes and the unused faces.
    let old_faces: Vec<EntityId> = fa_now.iter().chain(&fb_now).copied().collect();
    for &v in &[va, vb] {
        let fs: Vec<EntityId> = topo.volume_faces(v).to_vec();
        for f in fs {
            topo.remove_face_use(v, f, Side::Both)?;
        }
    }
    topo.kill(va)?;
    topo.kill(vb)?;
    for f in old_faces.iter().copied().filter(|f| !kept_faces.contains(f)) {
        if topo.is_alive(f) && topo.coboundary(f).iter().all(|e| topo.dim_of(e.cell) != Some(Dim::Volume)) {
            topo.kill_cascade(f)?;
        }
    }
    // 8. Weld the coincident vertices and edges of the kept fragments.
    if o.merge {
        let scope: Vec<EntityId> = kept_faces.iter().copied().collect();
        let opts = MergeOptions { tol: o.max_gap, kinds: MergeKinds { vertex: true, edge: true, face: false } };
        merge(topo, &scope, &[], &opts)?;
    }
    // 9. Shells and volumes.
    let new_volumes = assemble(topo, &kept, rule, va)?;
    // 10. Bodies and lineage.
    for b in [body_a, body_b].into_iter().flatten() {
        if topo.is_alive(b) {
            topo.kill(b)?;
        }
    }
    let mut new_bodies = Vec::new();
    for &v in &new_volumes {
        new_bodies.push(topo.make_body(&[v])?);
    }
    match rule {
        BoolRule::Subtract => {
            topo.record_split(va, &new_volumes)?;
            if let Some(b) = body_a {
                topo.record_split(b, &new_bodies)?;
            }
        }
        _ => {
            let (v0, nb0) = (new_volumes[0], new_bodies[0]);
            topo.record_merged(va, v0, 0.0)?;
            topo.record_merged(vb, v0, 0.0)?;
            if let Some(b) = body_a {
                topo.record_merged(b, nb0, 0.0)?;
            }
            if let Some(b) = body_b {
                topo.record_merged(b, nb0, 0.0)?;
            }
        }
    }
    // 11. Regularize.
    if o.regularize {
        crate::regularize::regularize(topo, &new_volumes, &b_faces)?;
    }
    // 12. Final checks.
    for &v in &new_volumes {
        let m = topo.volume_moments(v)?;
        if !(m.volume > 0.0) {
            return Err(BooleanError::ResultInvalid { volume: v, rule: rule.name() });
        }
    }
    let mut nf: BTreeSet<EntityId> = BTreeSet::new();
    for &v in &new_volumes {
        for &f in topo.volume_faces(v) {
            if b_faces.contains(&f) || topo.face_edges(f).is_empty() {
                nf.insert(f);
            }
        }
    }
    Ok(Stage { volumes: new_volumes, bodies: new_bodies, new_faces: nf.into_iter().filter(|f| topo.is_alive(*f)).collect() })
}

/// Group the kept face uses into shells and make the volumes. Returns the new volumes in
/// ascending creation order (by lowest face id of the outer shell).
fn assemble<E: TopologyEdit + ?Sized>(topo: &mut E, kept: &[Kept], rule: BoolRule, probe_volume: EntityId) -> Result<Vec<EntityId>, BooleanError> {
    let lin = topo.tolerance().linear;
    let n = kept.len();
    let index: BTreeMap<EntityId, usize> = kept.iter().enumerate().map(|(i, k)| (k.face, i)).collect();
    // Edge -> kept faces using it (with multiplicity).
    let mut edge_uses: BTreeMap<EntityId, Vec<usize>> = BTreeMap::new();
    for (i, k) in kept.iter().enumerate() {
        for ce in topo.coedges(k.face) {
            if ce.kind == CoedgeKind::Pole {
                continue;
            }
            edge_uses.entry(ce.key.edge).or_default().push(i);
        }
    }
    for (e, us) in &edge_uses {
        if us.len() != 2 {
            let _ = e;
            return Err(BooleanError::ResultInvalid { volume: probe_volume, rule: rule.name() });
        }
    }
    // Union-find over faces joined by edges.
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(p: &mut Vec<usize>, x: usize) -> usize {
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
            let (lo, hi) = (a.min(b), a.max(b));
            parent[hi] = lo;
        }
    }
    let mut shells: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for i in 0..n {
        let r = find(&mut parent, i);
        shells.entry(r).or_default().push(i);
    }
    // Order shells by their lowest face id.
    let mut shell_list: Vec<Vec<usize>> = shells.into_values().collect();
    for s in &mut shell_list {
        s.sort_by_key(|&i| kept[i].face);
    }
    shell_list.sort_by_key(|s| kept[s[0]].face);
    let _ = index;
    // Outer or void, by probing just inside the shell (against the normals).
    let tol = *topo.tolerance();
    let mut is_outer: Vec<bool> = Vec::new();
    let mut shell_uses: Vec<Vec<locate::Use>> = Vec::new();
    for s in &shell_list {
        let uses: Vec<locate::Use> = s.iter().map(|&i| (kept[i].face, if kept[i].side == Side::Left { Sense::Forward } else { Sense::Reversed })).collect();
        let mut bbox = sm_core::Aabb::empty();
        for &(f, _) in &uses {
            bbox = bbox.union(&geo::face_aabb(&*topo, f, 0.0));
        }
        let diag = geo::dist(bbox.lo, bbox.hi);
        let delta = (1e-3 * diag).max(100.0 * lin);
        let mut verdict: Option<bool> = None;
        for &(f, sense) in &uses {
            let Some(p) = faceclass::interior_point(&*topo, f)? else { continue };
            let n_out = outward_normal_use(&*topo, f, sense, p)?;
            let q = Point3::new(p.x - delta * n_out.x, p.y - delta * n_out.y, p.z - delta * n_out.z);
            match locate::locate_in_uses(&*topo, &uses, q, &tol) {
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
        let Some(v) = verdict else { return Err(BooleanError::ResultInvalid { volume: probe_volume, rule: rule.name() }) };
        is_outer.push(v);
        shell_uses.push(uses);
    }
    let outers: Vec<usize> = (0..shell_list.len()).filter(|&i| is_outer[i]).collect();
    if outers.is_empty() {
        return Err(BooleanError::ResultInvalid { volume: probe_volume, rule: rule.name() });
    }
    // Voids go to the innermost outer shell that contains them.
    let mut void_of: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for i in 0..shell_list.len() {
        if is_outer[i] {
            continue;
        }
        let f = shell_uses[i][0].0;
        let Some(p) = faceclass::interior_point(&*topo, f)? else { return Err(BooleanError::ResultInvalid { volume: probe_volume, rule: rule.name() }) };
        let mut best: Option<(f64, usize)> = None;
        for &o in &outers {
            if locate::locate_in_uses(&*topo, &shell_uses[o], p, &tol)? == PointLocation::In {
                let mut bbox = sm_core::Aabb::empty();
                for &(g, _) in &shell_uses[o] {
                    bbox = bbox.union(&geo::face_aabb(&*topo, g, 0.0));
                }
                let size = (bbox.hi.x - bbox.lo.x) * (bbox.hi.y - bbox.lo.y) * (bbox.hi.z - bbox.lo.z);
                if best.is_none_or(|b| size < b.0) {
                    best = Some((size, o));
                }
            }
        }
        let Some((_, o)) = best else { return Err(BooleanError::ResultInvalid { volume: probe_volume, rule: rule.name() }) };
        void_of.entry(o).or_default().push(i);
    }
    let spec_of = |idx: usize| -> Vec<FaceUseSpec> { shell_list[idx].iter().map(|&i| FaceUseSpec { face: kept[i].face, side: kept[i].side }).collect() };
    let mut volumes = Vec::new();
    for &o in &outers {
        let outer_spec = spec_of(o);
        let void_specs: Vec<Vec<FaceUseSpec>> = void_of.get(&o).map(|vs| vs.iter().map(|&v| spec_of(v)).collect()).unwrap_or_default();
        let mut shells: Vec<ShellSpec<'_>> = vec![ShellSpec { kind: ShellKind::Outer, uses: &outer_spec }];
        for vs in &void_specs {
            shells.push(ShellSpec { kind: ShellKind::Void, uses: vs });
        }
        volumes.push(topo.make_volume(&shells)?);
    }
    Ok(volumes)
}

fn outward_normal_use<V: TopologyView + ?Sized>(view: &V, f: EntityId, sense: Sense, p: Point3) -> Result<Vector3, BooleanError> {
    let (n, _, _) = geo::face_normal_near(view, f, p).ok_or(BooleanError::Unsupported("cannot evaluate a face normal"))?;
    Ok(n * sense.sign())
}
