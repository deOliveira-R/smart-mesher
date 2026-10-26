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

/// Class of a fragment, with its interior probe point and its outward normal there.
fn classify_frag<V: TopologyView + ?Sized>(view: &V, f: EntityId, own: EntityId, other: EntityId, other_uses: &[locate::Use]) -> Result<(Class, Point3, Vector3), BooleanError> {
    let tol = *view.tolerance();
    let lin = tol.linear;
    let p = faceclass::interior_point(view, f)?.ok_or(BooleanError::Unsupported("cannot find a point inside a fragment"))?;
    let n_own = outward_normal(view, f, own, p)?;
    for &(g, _) in other_uses {
        if locate::on_face(view, g, p, lin)? {
            let n_g = outward_normal(view, g, other, p)?;
            return Ok((if n_own.dot(n_g) > 0.0 { Class::OnSame } else { Class::OnOpposite }, p, n_own));
        }
    }
    match locate::locate_in_uses(view, other_uses, p, &tol)? {
        PointLocation::In => Ok((Class::In, p, n_own)),
        PointLocation::Out => Ok((Class::Out, p, n_own)),
        PointLocation::On => Err(BooleanError::Ambiguous { face: f, point: p }),
    }
}

/// One kept face use of the result.
#[derive(Clone, Copy, Debug)]
struct Kept {
    face: EntityId,
    /// A point inside the fragment and the normal pointing out of the result region there. The
    /// side the result occupies is derived from them at assembly, because freeing the operands'
    /// uses may reverse a face (the orientation rule).
    pt: Point3,
    n_res: Vector3,
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
    let mut class_a: Vec<(EntityId, Class, Point3, Vector3)> = Vec::new();
    for &f in &fa_now {
        let (c, p, n) = classify_frag(&*topo, f, va, vb, &uses_b)?;
        class_a.push((f, c, p, n));
    }
    let mut class_b: Vec<(EntityId, Class, Point3, Vector3)> = Vec::new();
    for &f in &fb_now {
        let (c, p, n) = classify_frag(&*topo, f, vb, va, &uses_a)?;
        class_b.push((f, c, p, n));
    }
    let all_out = class_a.iter().chain(&class_b).all(|(_, c, _, _)| *c == Class::Out);
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
    for &(f, c, pt, n) in &class_a {
        let keep = match rule {
            BoolRule::Unite => matches!(c, Class::Out | Class::OnSame),
            BoolRule::Intersect => matches!(c, Class::In | Class::OnSame),
            BoolRule::Subtract => matches!(c, Class::Out | Class::OnOpposite),
        };
        if keep {
            kept.push(Kept { face: f, pt, n_res: n, from_b: false });
        }
    }
    for &(f, c, pt, n) in &class_b {
        let (keep, flip) = match rule {
            BoolRule::Unite => (c == Class::Out, false),
            BoolRule::Intersect => (c == Class::In, false),
            BoolRule::Subtract => (c == Class::In, true),
        };
        if keep {
            kept.push(Kept { face: f, pt, n_res: if flip { n * -1.0 } else { n }, from_b: true });
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
        crate::regularize::regularize(topo, &new_volumes)?;
    }
    // Pcurves of the new and moved coedges on curved faces.
    let mut result_faces: BTreeSet<EntityId> = BTreeSet::new();
    for &v in &new_volumes {
        result_faces.extend(topo.volume_faces(v).iter().copied());
    }
    let rf: Vec<EntityId> = result_faces.into_iter().collect();
    crate::pcurves::fix_faces(topo, &rf)?;
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
    // The side the result occupies, from the face's current orientation.
    let mut uses: Vec<(EntityId, Side)> = Vec::with_capacity(kept.len());
    for k in kept {
        let (nf, _, _) = geo::face_normal_near(&*topo, k.face, k.pt).ok_or(BooleanError::Unsupported("cannot evaluate a face normal"))?;
        uses.push((k.face, if nf.dot(k.n_res) > 0.0 { Side::Left } else { Side::Right }));
    }
    let groups = crate::assemble::group_shells(&*topo, &uses, probe_volume, rule.name())?;
    crate::assemble::make_volumes(topo, &groups)
}
