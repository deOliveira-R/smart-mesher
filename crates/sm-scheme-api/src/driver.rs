//! The reference driver: plan top-down, solve intervals once, apply bottom-up, commit in a fixed
//! order, fall back by re-planning, and emit through `sm_mesh_api::MeshBuild`.
//!
//! 1. **Closure.** The requested targets (volumes, pieces, faces or curves) and everything below
//!    them, from [`ShapeView`].
//! 2. **Decks.** Every deck pair inside the closure imposes `CopyOf` on the B face and on each
//!    paired curve, an interval equality per curve pair and `Forbid(BOUNDARY_LAYER)` on both
//!    faces; a mirror face gets `OnPlane` and `Forbid(BOUNDARY_LAYER)`.
//! 3. **Plan, top-down** (atomic groups, then dimension 3, 2, 1, ascending keys, copies after
//!    their sources): candidates from the registry, filtered by the requirements imposed so
//!    far (capabilities, forbidden capabilities, allowed names), gated or ranked by
//!    `applicability`, then `plan`. A plan's claims are owned by it, its requirements land on
//!    the children, its interval constraints join the global problem.
//! 4. **Solve** sizes plus all constraints with the [`IntervalSolver`].
//! 5. **Apply, bottom-up** (dimension 1, 2, 3). Inside a dimension, jobs are levelled by
//!    `Plan::after`; each level runs on the [`Executor`] (jobs read only the committed mesh) and
//!    is committed in job order, so numbering never depends on threads.
//! 6. **Fallback.** A failed `apply` whose error allows it excludes that scheme for that entity
//!    and re-plans from scratch (the requirements and intervals may change).
//! 7. **Emit** ([`emit`]): nodes with classification, cells, boundary faces classified on the
//!    faces whose patches they match. Nothing reaches the builder before everything succeeded.

use crate::builtin::{CURVE_COPY, SURFACE_COPY};
use crate::caps::Caps;
use crate::error::{Attempt, DriveError, FailureKind, Phase, SchemeError};
use crate::intent::{EntityIntent, Firmness, MeshIntent, SchemeChoice, SchemeId, SchemeSpec, SizeSpec};
use crate::interval::{IntervalConstraint, IntervalProblem, IntervalSolution, IntervalSolver, SourcedConstraint};
use crate::key::ShapeKey;
use crate::mesh::CommittedMesh;
use crate::patch::{FaceList, Grid, NodeId, PatchRef, SurfacePatch, VolumePatch};
use crate::plan::{ChildRequirement, Plan, Requirement};
use crate::registry::{Candidate, Origin, SchemeImpl, SchemeRegistry};
use crate::scheme::{BoundaryLoop, BoundaryUse, CurveInput, PlanCtx, SurfaceInput, VolumeInput};
use crate::shape::{DeckLink, FaceUseRef, LoopUse, ShapeView};
use crate::size::SizeField;
use crate::vertex::CornerPicker;
use smallvec::SmallVec;
use sm_core::{EntityId, Point3, Transform};
use sm_mesh_api::{CellIx, Classification, MeshBuild, MeshEntityRef, Param, VertexIx};
use sm_topo_api::{Dim, Sense};
use std::collections::{BTreeMap, BTreeSet};

/// Runs independent jobs. Results come back in index order whatever the scheduling.
pub trait Executor: Sync {
    /// `(0..n).map(f)`, possibly in parallel.
    fn map<R: Send, F: Fn(usize) -> R + Sync>(&self, n: usize, f: F) -> Vec<R>;
}

/// Runs jobs one after another.
#[derive(Clone, Copy, Debug, Default)]
pub struct Sequential;

impl Executor for Sequential {
    fn map<R: Send, F: Fn(usize) -> R + Sync>(&self, n: usize, f: F) -> Vec<R> {
        (0..n).map(f).collect()
    }
}

/// Runs jobs on up to `n` scoped `std` threads in contiguous chunks (API crates use no rayon;
/// `sm-engine` can plug a rayon executor in).
#[derive(Clone, Copy, Debug)]
pub struct Threads(pub usize);

impl Executor for Threads {
    fn map<R: Send, F: Fn(usize) -> R + Sync>(&self, n: usize, f: F) -> Vec<R> {
        let t = self.0.clamp(1, n.max(1));
        if t == 1 {
            return (0..n).map(f).collect();
        }
        let chunk = n.div_ceil(t);
        std::thread::scope(|s| {
            let f = &f;
            let handles: Vec<_> = (0..t).map(|k| s.spawn(move || (k * chunk..((k + 1) * chunk).min(n)).map(f).collect::<Vec<R>>())).collect();
            handles
                .into_iter()
                .flat_map(|h| match h.join() {
                    Ok(v) => v,
                    Err(p) => std::panic::resume_unwind(p),
                })
                .collect()
        })
    }
}

/// Driver settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DriverOptions {
    /// Size for curves with no size anywhere in the intent (and no `MeshIntent::default_size`).
    pub default_size: f64,
    /// Bound on fallback re-plans.
    pub max_replans: usize,
}

impl Default for DriverOptions {
    fn default() -> Self {
        DriverOptions { default_size: 1.0, max_replans: 16 }
    }
}

/// One scheduled scheme call.
#[derive(Clone, Debug, PartialEq)]
pub struct Job {
    /// Dimension of its targets.
    pub dim: Dim,
    /// The plan.
    pub plan: Plan,
    /// Capabilities of the result (`INHERIT` resolved).
    pub caps: Caps,
}

/// The planned, solved problem (a "mesh preview": nothing is meshed yet).
#[derive(Clone, Debug, PartialEq)]
pub struct MeshPlan {
    /// Jobs in apply order (dimension ascending, then first target).
    pub jobs: Vec<Job>,
    /// Owning job of every target and claim.
    pub owner: BTreeMap<ShapeKey, usize>,
    /// Requirements per entity (from parents and decks).
    pub imposed: BTreeMap<ShapeKey, Vec<ChildRequirement>>,
    /// The interval problem.
    pub problem: IntervalProblem,
    /// Its solution (empty until solved).
    pub intervals: IntervalSolution,
    /// Failed attempts during planning.
    pub attempts: Vec<Attempt>,
    /// Vertices of the closure, ascending.
    pub vertices: Vec<ShapeKey>,
    /// Face uses per volume of the closure.
    pub faces_of: BTreeMap<ShapeKey, Vec<FaceUseRef>>,
    /// Loops per face of the closure.
    pub loops_of: BTreeMap<ShapeKey, Vec<LoopUse>>,
}

impl MeshPlan {
    /// The job that meshes `key` (target or claim).
    pub fn job_of(&self, key: ShapeKey) -> Option<&Job> {
        self.owner.get(&key.canonical()).map(|&i| &self.jobs[i])
    }
    /// Requirements imposed on `key`.
    pub fn imposed_on(&self, key: ShapeKey) -> &[ChildRequirement] {
        self.imposed.get(&key.canonical()).map_or(&[], Vec::as_slice)
    }
}

/// Result of [`Driver::run`].
#[derive(Clone, Debug)]
pub struct MeshOutcome {
    /// The meshed scratch store (emit it with [`emit`]).
    pub mesh: CommittedMesh,
    /// The plan that succeeded.
    pub plan: MeshPlan,
    /// Every failed attempt, including apply failures that triggered a re-plan.
    pub attempts: Vec<Attempt>,
}

enum Prepared {
    Curve { ends: (NodeId, NodeId), interior: Vec<(f64, Point3, f64)> },
    Surface(SurfacePatch),
    Volume(VolumePatch),
}

enum ApplyFail {
    Job { target: ShapeKey, scheme: SchemeId, error: SchemeError },
    Drive(DriveError),
}

impl From<DriveError> for ApplyFail {
    fn from(e: DriveError) -> Self {
        ApplyFail::Drive(e)
    }
}

/// The reference meshing driver.
#[derive(Clone, Copy)]
pub struct Driver<'a> {
    /// Scheme code.
    pub registry: &'a SchemeRegistry,
    /// Shape queries.
    pub shape: &'a dyn ShapeView,
    /// The intent.
    pub intent: &'a MeshIntent,
    /// Global interval solver.
    pub solver: &'a dyn IntervalSolver,
    /// Shared corner picker.
    pub corners: &'a dyn CornerPicker,
    /// Optional size field.
    pub size: Option<&'a dyn SizeField>,
    /// Settings.
    pub options: DriverOptions,
}

static NO_INTENT: EntityIntent = EntityIntent { scheme: SchemeChoice::Auto, size: None, vertex_types: Vec::new() };

fn dim_u8(d: Dim) -> u8 {
    d as u8
}

fn infer_caps(faces: &FaceList<PatchRef>, grid: bool) -> Caps {
    let mut c = Caps::NONE;
    for l in faces.iter() {
        c |= match l.len() {
            3 => Caps::TRI,
            4 => Caps::QUAD,
            _ => Caps::POLY,
        };
    }
    if grid {
        c |= Caps::STRUCTURED;
    }
    c
}

impl<'a> Driver<'a> {
    fn attempt(target: ShapeKey, scheme: &SchemeId, phase: Phase, kind: FailureKind, message: impl Into<String>) -> Attempt {
        Attempt { target, scheme: scheme.clone(), phase, kind, message: message.into() }
    }

    fn intent_of(&self, key: ShapeKey) -> &EntityIntent {
        self.intent.get(key).unwrap_or(&NO_INTENT)
    }

    /// Resolve `INHERIT` against the source of a `CopyOf` requirement.
    fn resolve_caps(caps: Caps, reqs: &[ChildRequirement], jobs: &[Job], owner: &BTreeMap<ShapeKey, usize>) -> Caps {
        if !caps.contains(Caps::INHERIT) {
            return caps;
        }
        let src = reqs.iter().find_map(|r| match &r.req {
            Requirement::CopyOf { source, .. } => owner.get(source).map(|&j| jobs[j].caps),
            _ => None,
        });
        caps.without(Caps::INHERIT) | src.unwrap_or(Caps::NONE)
    }

    /// Check hard requirements; return the soft score factor or the violation.
    fn check_reqs(id: &SchemeId, caps: Caps, reqs: &[ChildRequirement]) -> Result<f64, String> {
        let mut factor = 1.0;
        for r in reqs {
            match &r.req {
                Requirement::Caps { need, firmness } if !caps.contains(*need) => {
                    if *firmness == Firmness::Hard {
                        return Err(format!("{} requires capabilities {:?}, {id} has {:?}", r.by, need, caps));
                    }
                    factor *= 0.5;
                }
                Requirement::Forbid { caps: f } if caps.intersects(*f) => return Err(format!("{} forbids capabilities {:?}", r.by, f)),
                Requirement::Scheme { allowed, firmness } if !allowed.contains(id) => {
                    if *firmness == Firmness::Hard {
                        return Err(format!("{} allows only {:?}", r.by, allowed));
                    }
                    factor *= 0.5;
                }
                _ => {}
            }
        }
        Ok(factor)
    }

    #[allow(clippy::too_many_arguments)]
    fn plan_entity(
        &self,
        targets: &[ShapeKey],
        dim: Dim,
        cands: Vec<Candidate>,
        imposed: &BTreeMap<ShapeKey, Vec<ChildRequirement>>,
        excluded: &BTreeSet<(ShapeKey, SchemeId)>,
        jobs: &[Job],
        owner: &BTreeMap<ShapeKey, usize>,
        attempts: &mut Vec<Attempt>,
    ) -> Result<(Plan, Caps), DriveError> {
        let key = targets[0];
        let reqs: Vec<ChildRequirement> = targets.iter().flat_map(|t| imposed.get(t).into_iter().flatten().cloned()).collect();
        let vtypes = self.intent_of(key).vertex_types.clone();
        let mut ranked: Vec<(Origin, f64, usize, Candidate, Caps)> = Vec::new();
        let mut local: Vec<Attempt> = Vec::new();
        for (order, c) in cands.into_iter().enumerate() {
            let id = c.spec.scheme.clone();
            if excluded.contains(&(key, id.clone())) {
                continue;
            }
            let Some(imp) = self.registry.get(&id) else {
                local.push(Self::attempt(key, &id, Phase::Requirements, FailureKind::NotApplicable, "unknown scheme"));
                continue;
            };
            if imp.core().dim() != dim {
                local.push(Self::attempt(key, &id, Phase::Requirements, FailureKind::NotApplicable, format!("a {:?} scheme cannot mesh a {dim:?}", imp.core().dim())));
                continue;
            }
            let caps = Self::resolve_caps(imp.core().caps(), &reqs, jobs, owner);
            let factor = match Self::check_reqs(&id, caps, &reqs) {
                Ok(f) => f,
                Err(m) => {
                    local.push(Self::attempt(key, &id, Phase::Requirements, FailureKind::Rejected, m));
                    continue;
                }
            };
            let cx = PlanCtx { shape: self.shape, targets, params: &c.spec.params, imposed: &reqs, corners: self.corners, vertex_types: &vtypes };
            let app = imp.core().applicability(&cx);
            if !app.is_applicable() {
                local.push(Self::attempt(key, &id, Phase::Applicability, FailureKind::NotApplicable, app.reason));
                continue;
            }
            let score = if c.origin == Origin::Auto { app.score * factor } else { 1.0 };
            ranked.push((if c.origin == Origin::Auto { Origin::Auto } else { Origin::Explicit }, score, order, c, caps));
        }
        // Explicit and fallback candidates keep their order; auto ones go by score, then list order.
        ranked.sort_by(|a, b| a.0.cmp(&b.0).then(b.1.total_cmp(&a.1)).then(a.2.cmp(&b.2)));
        for (_, _, _, c, caps) in ranked {
            let id = c.spec.scheme.clone();
            let Some(imp) = self.registry.get(&id) else { continue };
            let cx = PlanCtx { shape: self.shape, targets, params: &c.spec.params, imposed: &reqs, corners: self.corners, vertex_types: &vtypes };
            match imp.core().plan(&cx) {
                Ok(mut p) => {
                    p.targets.sort();
                    if p.targets.as_slice() != targets || p.scheme != id {
                        local.push(Self::attempt(key, &id, Phase::Plan, FailureKind::Internal, "plan names other targets or another scheme"));
                        continue;
                    }
                    if let Some(bad) = p.claims.iter().chain(p.requires.iter().map(|r| &r.target)).find(|k| self.shape.dim(**k).is_none_or(|d| d >= dim)) {
                        local.push(Self::attempt(key, &id, Phase::Plan, FailureKind::Internal, format!("{bad} is not a lower-dimensional child")));
                        continue;
                    }
                    attempts.extend(local);
                    return Ok((p, caps));
                }
                Err(e) => local.push(Self::attempt(key, &id, Phase::Plan, e.kind(), e.to_string())),
            }
        }
        let mine = local.clone();
        attempts.extend(local);
        Err(DriveError::NoScheme { target: key, attempts: mine })
    }

    fn candidates_for(&self, key: ShapeKey, dim: Dim, reqs: &[ChildRequirement]) -> Result<Vec<Candidate>, DriveError> {
        let intent = self.intent_of(key);
        if reqs.iter().any(|r| matches!(r.req, Requirement::CopyOf { .. })) {
            let copy = match dim {
                Dim::Edge => CURVE_COPY,
                Dim::Face => SURFACE_COPY,
                _ => return Err(DriveError::Conflict { target: key, message: "copy requirement on a volume".into() }),
            };
            if let SchemeChoice::Chain { schemes, firmness: Firmness::Hard } = &intent.scheme
                && !schemes.iter().any(|s| s.scheme.as_str() == copy)
            {
                return Err(DriveError::Conflict { target: key, message: format!("hard scheme {:?} conflicts with a deck/copy requirement", schemes.iter().map(|s| s.scheme.to_string()).collect::<Vec<_>>()) });
            }
            return Ok(vec![Candidate { spec: SchemeSpec::new(copy), origin: Origin::Explicit }]);
        }
        Ok(self.registry.candidates(dim_u8(dim), &intent.scheme))
    }

    /// Plan everything below `targets` (top-down), without solving intervals.
    pub fn plan(&self, targets: &[ShapeKey], excluded: &BTreeSet<(ShapeKey, SchemeId)>) -> Result<MeshPlan, DriveError> {
        let shape = self.shape;
        let mut targets: Vec<ShapeKey> = targets.iter().map(|t| t.canonical()).collect();
        targets.sort();
        targets.dedup();
        // 1. Closure.
        let mut sets: [BTreeSet<ShapeKey>; 4] = Default::default();
        for t in &targets {
            match shape.dim(*t) {
                Some(d @ (Dim::Edge | Dim::Face | Dim::Volume)) => {
                    sets[d as usize].insert(*t);
                }
                _ => return Err(SchemeError::shape(*t, "not a meshable curve, face, volume or piece").into()),
            }
        }
        let mut faces_of = BTreeMap::new();
        let mut face_parents: BTreeMap<ShapeKey, Vec<ShapeKey>> = BTreeMap::new();
        for v in sets[3].clone() {
            let fu = shape.volume_faces(v)?;
            for u in &fu {
                sets[2].insert(u.face);
                face_parents.entry(u.face).or_default().push(v);
            }
            faces_of.insert(v, fu);
        }
        let mut loops_of = BTreeMap::new();
        let mut curve_parents: BTreeMap<ShapeKey, Vec<ShapeKey>> = BTreeMap::new();
        for f in sets[2].clone() {
            let loops = shape.face_loops(f)?;
            for u in loops.iter().flat_map(|l| &l.uses) {
                sets[1].insert(u.curve);
                let p = curve_parents.entry(u.curve).or_default();
                if !p.contains(&f) {
                    p.push(f);
                }
            }
            loops_of.insert(f, loops);
        }
        for c in sets[1].clone() {
            let (a, b) = shape.curve_ends(c)?;
            sets[0].insert(a);
            sets[0].insert(b);
        }
        // 2. Decks.
        let mut imposed: BTreeMap<ShapeKey, Vec<ChildRequirement>> = BTreeMap::new();
        let mut constraints: Vec<SourcedConstraint> = Vec::new();
        let push = |imposed: &mut BTreeMap<ShapeKey, Vec<ChildRequirement>>, target: ShapeKey, req: Requirement, by: ShapeKey| imposed.entry(target).or_default().push(ChildRequirement { target, req, by });
        for link in shape.deck_links() {
            match link {
                DeckLink::Pair { a, b, transform, .. } if sets[2].contains(&a) && sets[2].contains(&b) => {
                    push(&mut imposed, b, Requirement::CopyOf { source: a, transform, sense: Sense::Forward }, b);
                    for f in [a, b] {
                        push(&mut imposed, f, Requirement::Forbid { caps: Caps::BOUNDARY_LAYER }, b);
                    }
                    for (ca, cb, s) in shape.corresponding_curves(a, b, &transform)? {
                        constraints.push(SourcedConstraint { constraint: IntervalConstraint::equal(ca, cb), by: b, scheme: None, why: "deck pair".into() });
                        let has = imposed.get(&cb).is_some_and(|v| v.iter().any(|r| matches!(r.req, Requirement::CopyOf { .. })));
                        if !has {
                            push(&mut imposed, cb, Requirement::CopyOf { source: ca, transform, sense: s }, b);
                        }
                    }
                }
                DeckLink::Mirror { face, normal, offset, .. } if sets[2].contains(&face) => {
                    push(&mut imposed, face, Requirement::Forbid { caps: Caps::BOUNDARY_LAYER }, face);
                    push(&mut imposed, face, Requirement::OnPlane { normal, offset }, face);
                }
                _ => {}
            }
        }
        // 3. Plan top-down.
        let mut jobs: Vec<Job> = Vec::new();
        let mut owner: BTreeMap<ShapeKey, usize> = BTreeMap::new();
        let mut attempts = Vec::new();
        let accept = |p: Plan, caps: Caps, dim: Dim, jobs: &mut Vec<Job>, owner: &mut BTreeMap<ShapeKey, usize>, imposed: &mut BTreeMap<ShapeKey, Vec<ChildRequirement>>, constraints: &mut Vec<SourcedConstraint>| -> Result<(), DriveError> {
            let idx = jobs.len();
            for k in p.targets.iter().chain(&p.claims) {
                if owner.insert(*k, idx).is_some() {
                    return Err(DriveError::Conflict { target: *k, message: format!("claimed twice (second claim by {})", p.scheme) });
                }
            }
            for r in &p.requires {
                imposed.entry(r.target).or_default().push(r.clone());
            }
            for c in &p.intervals {
                constraints.push(SourcedConstraint { constraint: c.clone(), by: p.targets[0], scheme: Some(p.scheme.clone()), why: format!("{} plan", p.scheme) });
            }
            jobs.push(Job { dim, plan: p, caps });
            Ok(())
        };
        for g in &self.intent.groups {
            if !g.targets.iter().all(|t| sets[3].contains(t)) {
                continue;
            }
            let cands = vec![Candidate { spec: g.scheme.clone(), origin: Origin::Explicit }];
            let (p, caps) = self.plan_entity(&g.targets, Dim::Volume, cands, &imposed, excluded, &jobs, &owner, &mut attempts)?;
            accept(p, caps, Dim::Volume, &mut jobs, &mut owner, &mut imposed, &mut constraints)?;
        }
        for dim in [Dim::Volume, Dim::Face, Dim::Edge] {
            let mut pending: Vec<ShapeKey> = sets[dim as usize].iter().copied().collect();
            // Copies wait for their source's job; repeat until no progress.
            while !pending.is_empty() {
                let mut next = Vec::new();
                let mut progressed = false;
                for key in pending {
                    if owner.contains_key(&key) {
                        progressed = true;
                        continue;
                    }
                    let reqs: Vec<ChildRequirement> = imposed.get(&key).cloned().unwrap_or_default();
                    let src = reqs.iter().find_map(|r| match &r.req {
                        Requirement::CopyOf { source, .. } => Some(*source),
                        _ => None,
                    });
                    if let Some(s) = src
                        && !owner.contains_key(&s)
                        && sets[dim as usize].contains(&s)
                    {
                        next.push(key);
                        continue;
                    }
                    let cands = self.candidates_for(key, dim, &reqs)?;
                    let (p, caps) = self.plan_entity(&[key], dim, cands, &imposed, excluded, &jobs, &owner, &mut attempts)?;
                    accept(p, caps, dim, &mut jobs, &mut owner, &mut imposed, &mut constraints)?;
                    progressed = true;
                }
                if !progressed {
                    return Err(DriveError::Cycle(next));
                }
                pending = next;
            }
        }
        // 4. Sizes.
        let default = self.intent.default_size.unwrap_or(self.options.default_size);
        for &c in &sets[1] {
            let mut spec = self.intent_of(c).size;
            if spec.is_none() {
                let parents = curve_parents.get(&c).cloned().unwrap_or_default();
                spec = parents.iter().find_map(|f| self.intent_of(*f).size);
                if spec.is_none() {
                    let mut vols: Vec<ShapeKey> = parents.iter().flat_map(|f| face_parents.get(f).cloned().unwrap_or_default()).collect();
                    vols.sort();
                    spec = vols.iter().find_map(|v| self.intent_of(*v).size);
                }
            }
            let len = shape.curve_length(c)?;
            let constraint = match spec {
                Some(SizeSpec::Intervals { n, firmness }) => IntervalConstraint::Count { curve: c, n, firmness },
                Some(SizeSpec::Size { size, .. }) => IntervalConstraint::Goal { curve: c, goal: len / size },
                None => IntervalConstraint::Goal { curve: c, goal: len / default },
            };
            constraints.push(SourcedConstraint { constraint, by: c, scheme: None, why: "size".into() });
            constraints.push(SourcedConstraint { constraint: IntervalConstraint::Min { curve: c, n: 1 }, by: c, scheme: None, why: "at least one interval".into() });
        }
        // Apply order: dimension ascending, then first target.
        let mut order: Vec<usize> = (0..jobs.len()).collect();
        order.sort_by_key(|&i| (jobs[i].dim, jobs[i].plan.targets[0]));
        let mut remap = vec![0usize; jobs.len()];
        for (new, &old) in order.iter().enumerate() {
            remap[old] = new;
        }
        let mut slots: Vec<Option<Job>> = jobs.into_iter().map(Some).collect();
        let jobs: Vec<Job> = order.iter().filter_map(|&i| slots[i].take()).collect();
        let owner = owner.into_iter().map(|(k, i)| (k, remap[i])).collect();
        Ok(MeshPlan {
            jobs,
            owner,
            imposed,
            problem: IntervalProblem { curves: sets[1].iter().copied().collect(), constraints },
            intervals: IntervalSolution::default(),
            attempts,
            vertices: sets[0].iter().copied().collect(),
            faces_of,
            loops_of,
        })
    }

    /// Solve the interval problem of `plan` in place.
    pub fn solve(&self, plan: &mut MeshPlan) -> Result<(), DriveError> {
        plan.intervals = self.solver.solve(&plan.problem)?;
        Ok(())
    }

    fn run_job(&self, plan: &MeshPlan, job: &Job, mesh: &CommittedMesh) -> Result<Prepared, SchemeError> {
        let key = job.plan.targets[0];
        let imp = self.registry.get(&job.plan.scheme).ok_or_else(|| SchemeError::failed(FailureKind::Internal, "scheme vanished from the registry"))?;
        let imposed = plan.imposed_on(key);
        match imp {
            SchemeImpl::Curve(s) => {
                let n = plan.intervals.get(key).ok_or_else(|| SchemeError::failed(FailureKind::Internal, "curve without interval count"))?;
                let (a, b) = self.shape.curve_ends(key)?;
                let node = |v| mesh.vertex(v).ok_or_else(|| SchemeError::failed(FailureKind::Internal, format!("vertex {v} has no node")));
                let ends = (node(a)?, node(b)?);
                let input = CurveInput { shape: self.shape, mesh, curve: key, intervals: n, length: self.shape.curve_length(key)?, closed: a == b, plan: &job.plan, imposed, size: self.size };
                let patch = s.apply(&input)?;
                let f = &patch.fractions;
                if f.len() + 1 != n as usize || f.iter().any(|x| !(*x > 0.0 && *x < 1.0)) || f.windows(2).any(|w| w[1] <= w[0]) {
                    return Err(SchemeError::failed(FailureKind::Internal, format!("curve patch needs {} increasing fractions in (0, 1)", n.saturating_sub(1))));
                }
                let mut interior = Vec::with_capacity(f.len());
                for &x in f {
                    let (p, t) = self.shape.curve_point(key, x)?;
                    interior.push((x, p, t));
                }
                Ok(Prepared::Curve { ends, interior })
            }
            SchemeImpl::Surface(s) => {
                let loops_src = plan.loops_of.get(&key).ok_or_else(|| SchemeError::failed(FailureKind::Internal, "face outside the closure"))?;
                let mut loops = Vec::with_capacity(loops_src.len());
                for l in loops_src {
                    let mut nodes = Vec::new();
                    let mut uses = Vec::with_capacity(l.uses.len());
                    for u in &l.uses {
                        let c = mesh.curve(u.curve).ok_or_else(|| SchemeError::failed(FailureKind::Constraint, format!("boundary curve {} is not meshed", u.curve)))?;
                        let first = nodes.len() as u32;
                        let seg = c.nodes.len() - 1;
                        match u.sense {
                            Sense::Forward => nodes.extend_from_slice(&c.nodes[..seg]),
                            Sense::Reversed => nodes.extend(c.nodes[1..].iter().rev()),
                        }
                        uses.push(BoundaryUse { curve: u.curve, sense: u.sense, first, segments: seg as u32 });
                    }
                    loops.push(BoundaryLoop { ring: l.ring, nodes, uses });
                }
                let input = SurfaceInput { shape: self.shape, mesh, face: key, loops: &loops, plan: &job.plan, imposed, intervals: &plan.intervals, size: self.size };
                Ok(Prepared::Surface(s.apply(&input)?))
            }
            SchemeImpl::Volume(s) => {
                let faces: Vec<Vec<FaceUseRef>> = job.plan.targets.iter().map(|t| plan.faces_of.get(t).cloned().unwrap_or_default()).collect();
                let imposed_all: Vec<ChildRequirement> = job.plan.targets.iter().flat_map(|t| plan.imposed_on(*t).iter().cloned()).collect();
                let input = VolumeInput { shape: self.shape, mesh, targets: &job.plan.targets, faces: &faces, plan: &job.plan, imposed: &imposed_all, intervals: &plan.intervals, size: self.size };
                let patch = s.apply(&input)?;
                for c in &job.plan.claims {
                    if self.shape.dim(*c) == Some(Dim::Face) && !patch.surfaces.iter().any(|s| s.face == *c) {
                        return Err(SchemeError::failed(FailureKind::Internal, format!("claimed face {c} missing from the volume patch")));
                    }
                }
                Ok(Prepared::Volume(patch))
            }
        }
    }

    fn commit(mesh: &mut CommittedMesh, job: &Job, p: Prepared) -> Result<(), DriveError> {
        let key = job.plan.targets[0];
        let bad = |message: String| DriveError::BadPatch { target: key, scheme: job.plan.scheme.clone(), message };
        match p {
            Prepared::Curve { ends, interior } => mesh.commit_curve(key, ends, &interior, &job.plan.scheme).map_err(bad),
            Prepared::Surface(s) => mesh.commit_surface(key, &s, &job.plan.scheme, job.caps).map_err(bad),
            Prepared::Volume(v) => {
                let caps = v.surfaces.iter().fold(Caps::NONE, |c, s| c | infer_caps(&s.faces, s.grid.is_some()));
                mesh.commit_volume(&job.plan.targets, &job.plan.claims, &v, &job.plan.scheme, caps).map_err(bad)
            }
        }
    }

    fn verify_copy(&self, mesh: &CommittedMesh, b: ShapeKey, a: ShapeKey, t: &Transform) -> Result<(), String> {
        let (Some(sb), Some(sa)) = (mesh.surface(b), mesh.surface(a)) else { return Err("copy pair not meshed".into()) };
        if sa.faces.len() != sb.faces.len() {
            return Err(format!("{} elements on {a}, {} on {b}", sa.faces.len(), sb.faces.len()));
        }
        let mut img: Vec<Point3> = sa.faces.nodes.iter().map(|n| t.apply_point(mesh.point(*n))).collect();
        img.sort_by(|p, q| p.x.total_cmp(&q.x));
        let tol = 16.0 * self.shape.linear_tolerance();
        for n in &sb.faces.nodes {
            let p = mesh.point(*n);
            let scale = tol * (1.0 + p.x.abs().max(p.y.abs()).max(p.z.abs()));
            let lo = img.partition_point(|q| q.x < p.x - scale);
            if !img[lo..].iter().take_while(|q| q.x <= p.x + scale).any(|q| (*q - p).norm() <= scale) {
                return Err(format!("node {n:?} of {b} is not the image of a node of {a}"));
            }
        }
        Ok(())
    }

    fn apply_inner<E: Executor>(&self, plan: &MeshPlan, exec: &E) -> Result<CommittedMesh, ApplyFail> {
        let mut mesh = CommittedMesh::new();
        for &v in &plan.vertices {
            let p = self.shape.vertex_point(v).map_err(DriveError::from)?;
            mesh.add_vertex(v, p);
        }
        for dim in [Dim::Edge, Dim::Face, Dim::Volume] {
            let idx: Vec<usize> = (0..plan.jobs.len()).filter(|&j| plan.jobs[j].dim == dim).collect();
            // Level by same-dimension `after` dependencies.
            let mut level: BTreeMap<usize, usize> = BTreeMap::new();
            let mut remaining = idx.clone();
            let mut lvl = 0;
            while !remaining.is_empty() {
                let (ready, rest): (Vec<usize>, Vec<usize>) = remaining.iter().partition(|&&j| plan.jobs[j].plan.after.iter().filter_map(|a| plan.owner.get(a)).filter(|d| plan.jobs[**d].dim == dim && **d != j).all(|d| level.contains_key(d)));
                if ready.is_empty() {
                    return Err(DriveError::Cycle(rest.iter().map(|&j| plan.jobs[j].plan.targets[0]).collect()).into());
                }
                for j in ready {
                    level.insert(j, lvl);
                }
                remaining = rest;
                lvl += 1;
            }
            for l in 0..lvl {
                let batch: Vec<usize> = idx.iter().copied().filter(|j| level.get(j) == Some(&l)).collect();
                let mesh_ref = &mesh;
                let results = exec.map(batch.len(), |i| self.run_job(plan, &plan.jobs[batch[i]], mesh_ref));
                for (j, r) in batch.iter().zip(results) {
                    let job = &plan.jobs[*j];
                    match r {
                        Ok(p) => Self::commit(&mut mesh, job, p)?,
                        Err(error) => return Err(ApplyFail::Job { target: job.plan.targets[0], scheme: job.plan.scheme.clone(), error }),
                    }
                }
            }
        }
        for (key, reqs) in &plan.imposed {
            if self.shape.dim(*key) != Some(Dim::Face) {
                continue;
            }
            for r in reqs {
                if let Requirement::CopyOf { source, transform, .. } = &r.req
                    && let Err(m) = self.verify_copy(&mesh, *key, *source, transform)
                {
                    let scheme = plan.job_of(*key).map_or_else(|| SchemeId::new("?"), |j| j.plan.scheme.clone());
                    return Err(ApplyFail::Job { target: *key, scheme, error: SchemeError::failed(FailureKind::Constraint, format!("deck copy violated: {m}")) });
                }
            }
        }
        Ok(mesh)
    }

    /// Apply a solved plan once (no fallback).
    pub fn apply<E: Executor>(&self, plan: &MeshPlan, exec: &E) -> Result<CommittedMesh, DriveError> {
        self.apply_inner(plan, exec).map_err(|f| match f {
            ApplyFail::Job { target, scheme, error } => DriveError::Failed { target, scheme, error, attempts: Vec::new() },
            ApplyFail::Drive(e) => e,
        })
    }

    /// Plan, solve and apply, re-planning without a failed scheme while fallback is allowed.
    pub fn run<E: Executor>(&self, targets: &[ShapeKey], exec: &E) -> Result<MeshOutcome, DriveError> {
        self.intent.check()?;
        let mut excluded: BTreeSet<(ShapeKey, SchemeId)> = BTreeSet::new();
        let mut history: Vec<Attempt> = Vec::new();
        for _ in 0..=self.options.max_replans {
            let mut plan = match self.plan(targets, &excluded) {
                Ok(p) => p,
                Err(DriveError::NoScheme { target, mut attempts }) => {
                    let mut all: Vec<Attempt> = history.iter().filter(|a| a.target == target).cloned().collect();
                    all.append(&mut attempts);
                    return Err(DriveError::NoScheme { target, attempts: all });
                }
                Err(e) => return Err(e),
            };
            self.solve(&mut plan)?;
            match self.apply_inner(&plan, exec) {
                Ok(mesh) => {
                    history.extend(plan.attempts.iter().cloned());
                    return Ok(MeshOutcome { mesh, plan, attempts: history });
                }
                Err(ApplyFail::Job { target, scheme, error }) if error.allows_fallback() => {
                    history.push(Self::attempt(target, &scheme, Phase::Apply, error.kind(), error.to_string()));
                    excluded.insert((target, scheme));
                }
                Err(ApplyFail::Job { target, scheme, error }) => return Err(DriveError::Failed { target, scheme, error, attempts: history }),
                Err(ApplyFail::Drive(e)) => return Err(e),
            }
        }
        Err(DriveError::TooManyReplans { attempts: history })
    }

    /// Convert meshed volumes with a registered [`crate::MeshTransform`], all at once.
    pub fn transform(&self, mesh: &mut CommittedMesh, id: &SchemeId, targets: &[ShapeKey]) -> Result<(), DriveError> {
        let t = self.registry.transform(id).ok_or_else(|| DriveError::Registry(format!("unknown transform {id}")))?;
        let mut targets: Vec<ShapeKey> = targets.iter().map(|k| k.canonical()).collect();
        targets.sort();
        targets.dedup();
        let cx = PlanCtx { shape: self.shape, targets: &targets, params: &Default::default(), imposed: &[], corners: self.corners, vertex_types: &[] };
        let key = *targets.first().ok_or_else(|| DriveError::Registry("transform without targets".into()))?;
        let app = t.applicability(&cx);
        if !app.is_applicable() {
            return Err(DriveError::Failed { target: key, scheme: id.clone(), error: SchemeError::NotApplicable(app.reason), attempts: Vec::new() });
        }
        let patch = t.apply(self.shape, mesh, &targets).map_err(|error| DriveError::Failed { target: key, scheme: id.clone(), error, attempts: Vec::new() })?;
        mesh.apply_transform(&targets, &patch, id, t.caps()).map_err(|message| DriveError::BadPatch { target: key, scheme: id.clone(), message })
    }
}

/// What [`emit`] wrote.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EmitMap {
    /// Builder vertex of committed node `i` is `first_vertex + i`.
    pub first_vertex: usize,
    /// Builder cells in commit order.
    pub cells: Vec<CellIx>,
}

/// Write a committed mesh into a [`MeshBuild`]: every node with its classification (parameter
/// kept when it fits the entity kind), every cell classified on its volume or piece, and each
/// boundary face classified on the face whose patch element it matches. Interior faces are left
/// to `finish`, which derives them from their cells.
pub fn emit<B: MeshBuild>(mesh: &CommittedMesh, shape: &dyn ShapeView, b: &mut B) -> Result<EmitMap, DriveError> {
    let base = b.vertex_count();
    let mut class: BTreeMap<ShapeKey, EntityId> = BTreeMap::new();
    let mut classify = |k: ShapeKey| -> Result<EntityId, DriveError> {
        if let Some(e) = class.get(&k) {
            return Ok(*e);
        }
        let e = shape.classify_on(k)?;
        class.insert(k, e);
        Ok(e)
    };
    for (p, o) in mesh.points().iter().zip(mesh.origins()) {
        let v = b.add_vertex(*p)?;
        let entity = classify(o.on)?;
        let param = if o.param.fits(entity.kind()) { o.param } else { Param::None };
        b.set_class(MeshEntityRef::Vertex(v), Classification { entity, param })?;
    }
    let mut on_face: BTreeMap<SmallVec<[u32; 4]>, EntityId> = BTreeMap::new();
    for (k, s) in mesh.surfaces() {
        let e = classify(*k)?;
        for l in s.faces.iter() {
            let mut key: SmallVec<[u32; 4]> = l.iter().map(|n| n.0).collect();
            key.sort_unstable();
            on_face.entry(key).or_insert(e);
        }
    }
    let mut cells = Vec::with_capacity(mesh.cell_count());
    let mut verts: Vec<VertexIx> = Vec::with_capacity(8);
    for c in mesh.volumes() {
        let ents: Vec<EntityId> = c.targets.iter().map(|t| classify(*t)).collect::<Result<_, _>>()?;
        for i in 0..c.len() {
            verts.clear();
            verts.extend(c.cell(i).iter().map(|n| VertexIx::new(base + n.get())));
            let added = b.add_cell_from_vertices(c.shape[i], &verts)?;
            b.set_class(MeshEntityRef::Cell(added.cell), Classification::on(ents[c.target[i] as usize]))?;
            for (sf, reused) in added.faces.iter().zip(&added.reused) {
                if *reused {
                    continue;
                }
                let mut key: SmallVec<[u32; 4]> = b.face_loop(sf.face()).iter().map(|v| (v.get() - base) as u32).collect();
                key.sort_unstable();
                if let Some(e) = on_face.get(&key) {
                    b.set_class(MeshEntityRef::Face(sf.face()), Classification::on(*e))?;
                }
            }
            cells.push(added.cell);
        }
    }
    Ok(EmitMap { first_vertex: base, cells })
}

/// The grid of a committed structured face as stored (helper for structured volume schemes).
pub fn grid_of(mesh: &CommittedMesh, face: ShapeKey) -> Option<&Grid<NodeId>> {
    mesh.surface(face)?.grid.as_ref()
}
