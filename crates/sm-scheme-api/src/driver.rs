//! Reference driver: intent -> jobs -> interval solve -> schedule -> waves -> ledger.
//!
//! The phases are separate public steps so the real `mesh-driver` crate (and tests) can reuse or
//! replace any of them:
//!
//! 1. [`MeshProblem::build`]: closure of the requested targets; volume/group jobs planned first
//!    (they claim the faces they write and impose requirements on the rest); model deck rules;
//!    surface jobs (copy jobs for deck side B); curve jobs; goals from intervals and sizes; vertex
//!    types merged by firmness. Job ids are assigned by sorting jobs by their first write target.
//! 2. [`MeshProblem::solve`]: one global interval solve over all edges.
//! 3. [`MeshProblem::schedule`]: dependency levels of the read/write graph; each level is a wave
//!    of independent jobs.
//!    Transform jobs (THex & co.) are not claimed writers: they depend on the original writer of
//!    every target they rewrite, and two transforms may not rewrite the same target.
//! 4. [`MeshProblem::execute`]: each wave runs through an [`Executor`] (sequential or threaded)
//!    against the ledger as of the previous wave; patches are committed in job-id order.
//! 5. [`run`]: the loop that turns a `Retry::NextInChain` execution failure into an exclusion and
//!    re-plans, so fallback re-solves intervals with the next scheme's constraints.

use crate::copy::{copy_patch, entity_map, verify_copy};
use crate::error::{Attempt, DriverError, Retry, SchemeFailure};
use crate::intent::{Firmness, GroupKind, IntervalSpec, MeshIntent, SchemeChoice, SchemeIntent, SchemeSpec};
use crate::interval::{IntervalConstraint, IntervalSolution, IntervalSolver, Origin};
use crate::job::{CopyReason, CopySpec, Footprint, JobId, JobKind, MeshJob};
use crate::ledger::Ledger;
use crate::patch::MeshPatch;
use crate::scheme::{ExecCtx, Imposition, JobPlan, JobRequest, PlanCtx, Requirement, SchemeDim, SchemeRegistry, SizeField, VertexTypes};
use crate::target::Target;
use sm_core::{EntityId, Transform};
use sm_model_api::{BoundaryKind, Deck, DeckSide, FaceRole, Model, PieceRef};
use sm_topo_api::{Carrier, Dim, TopologyView, TopologyViewExt};
use std::collections::{BTreeMap, BTreeSet};

/// Everything a run reads.
#[derive(Clone, Copy)]
pub struct ProblemInput<'a> {
    /// Topology (or an overlay over it).
    pub topo: &'a dyn TopologyView,
    /// Model semantics and decomposition.
    pub model: &'a Model,
    /// Intent.
    pub intent: &'a MeshIntent,
    /// Registered schemes.
    pub registry: &'a SchemeRegistry,
    /// Optional size field.
    pub size: Option<&'a dyn SizeField>,
}

impl<'a> ProblemInput<'a> {
    fn plan_ctx(&self) -> PlanCtx<'a> {
        PlanCtx { topo: self.topo, model: self.model, intent: self.intent, size: self.size }
    }
}

/// Runs the jobs of one wave. Results must be returned in input order; the driver commits them
/// in that order, so any executor gives the same ledger.
pub trait Executor: Sync {
    /// Run `task(0..n)` and return the results in index order.
    fn run_all(&self, n: usize, task: &(dyn Fn(usize) -> Result<MeshPatch, SchemeFailure> + Sync)) -> Vec<Result<MeshPatch, SchemeFailure>>;
}

/// One job after another on the calling thread.
#[derive(Clone, Copy, Debug, Default)]
pub struct SequentialExecutor;

impl Executor for SequentialExecutor {
    fn run_all(&self, n: usize, task: &(dyn Fn(usize) -> Result<MeshPatch, SchemeFailure> + Sync)) -> Vec<Result<MeshPatch, SchemeFailure>> {
        (0..n).map(task).collect()
    }
}

/// Waves of independent jobs, in execution order.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Schedule {
    /// Each wave lists job ids ascending.
    pub waves: Vec<Vec<JobId>>,
}

/// The planned problem.
#[derive(Clone, Debug, PartialEq)]
pub struct MeshProblem {
    /// Jobs, indexed by `JobId`.
    pub jobs: Vec<MeshJob>,
    /// All interval constraints (plans, decks, goals).
    pub constraints: Vec<IntervalConstraint>,
    /// Interval variables: every edge of the closure, ascending.
    pub vars: Vec<EntityId>,
    /// Model vertices of the closure (seeded as nodes).
    pub vertices: Vec<EntityId>,
    /// Resolved vertex types.
    pub vertex_types: VertexTypes,
    /// Requirements per target.
    pub impositions: BTreeMap<Target, Vec<Imposition>>,
    /// Deck pairs written by a non-copy job, checked after commit: `(source, target, transform)`.
    pub deck_checks: Vec<(Target, Target, Transform)>,
    /// Rejected scheme candidates (diagnostics).
    pub attempts: Vec<Attempt>,
}

struct Planned {
    targets: Vec<Target>,
    kind: JobKind,
    group: Option<GroupKind>,
    plan: JobPlan,
}

#[derive(Default)]
struct Closure {
    vols: BTreeSet<Target>,
    faces: BTreeSet<Target>,
    edges: BTreeSet<EntityId>,
    verts: BTreeSet<EntityId>,
}

impl Closure {
    fn add(&mut self, t: Target, inp: &ProblemInput<'_>) -> Result<(), DriverError> {
        if let Target::Topo(id) = t
            && !inp.topo.is_alive(id)
        {
            return Err(DriverError::Intent(format!("{id:?} is not a live topology cell")));
        }
        match t.dim() {
            Some(3) => {
                self.vols.insert(t);
                let faces: Vec<Target> = match t {
                    Target::Topo(v) | Target::Piece(PieceRef::Whole(v)) => inp.topo.volume_faces(v).iter().map(|&f| Target::Topo(f)).collect(),
                    Target::Piece(p) => inp.model.decomposition().piece_faces(p, inp.topo).into_iter().map(Target::piece_face).collect(),
                    Target::PieceFace(_) => Vec::new(),
                };
                for f in faces {
                    self.add(f, inp)?;
                }
            }
            Some(2) => {
                self.faces.insert(t);
                if let Target::Topo(f) = t {
                    for &e in inp.topo.face_edges(f) {
                        self.add(Target::Topo(e), inp)?;
                    }
                }
            }
            Some(1) => {
                if let Target::Topo(e) = t {
                    self.edges.insert(e);
                    if let Some((s, u)) = inp.topo.edge_ends(e) {
                        self.verts.insert(s);
                        self.verts.insert(u);
                    }
                }
            }
            Some(0) => {
                if let Target::Topo(v) = t {
                    self.verts.insert(v);
                }
            }
            _ => return Err(DriverError::Intent(format!("{t:?} cannot be meshed"))),
        }
        Ok(())
    }
}

fn edge_length(topo: &dyn TopologyView, e: EntityId) -> f64 {
    if let Carrier::Curve(c) = topo.carrier(e)
        && let Some(curve) = topo.shapes().curve(c.curve)
        && let Ok(l) = curve.length(c.range, topo.tolerance().linear)
    {
        return l;
    }
    match topo.edge_ends(e) {
        Some((s, t)) => match (topo.carrier(s), topo.carrier(t)) {
            (Carrier::Point(a), Carrier::Point(b)) => (*a - *b).norm(),
            _ => 1.0,
        },
        None => 1.0,
    }
}

impl MeshProblem {
    /// Plan every job of the intent.
    pub fn build(inp: &ProblemInput<'_>) -> Result<Self, DriverError> {
        let intent = inp.intent;
        let mut cl = Closure::default();
        for &t in &intent.mesh {
            cl.add(t, inp)?;
        }
        for g in &intent.groups {
            if g.members.is_empty() {
                return Err(DriverError::Intent("empty group".into()));
            }
            for &m in &g.members {
                cl.add(m, inp)?;
            }
        }
        let mut b = Builder { inp, imposed: BTreeMap::new(), claimed: BTreeMap::new(), planned: Vec::new(), attempts: Vec::new() };

        // 1. Volume jobs: groups, then single volumes.
        let mut grouped: BTreeSet<Target> = BTreeSet::new();
        for g in &intent.groups {
            let mut members = g.members.clone();
            members.sort_unstable();
            members.dedup();
            grouped.extend(members.iter().copied());
            b.plan(members, Some(g.scheme.clone()), Some(g.kind), SchemeDim::Volume)?;
        }
        for &v in &cl.vols {
            if !grouped.contains(&v) {
                let choice = intent.target(&v).scheme;
                b.plan(vec![v], choice, None, SchemeDim::Volume)?;
            }
        }

        // 2. Model deck rules on faces.
        let mut constraints: Vec<IntervalConstraint> = Vec::new();
        let tol = inp.topo.tolerance().linear * 16.0;
        let mut copy_of: BTreeMap<Target, (Target, Transform, EntityId)> = BTreeMap::new();
        for &f in &cl.faces {
            let Some(fid) = f.topo() else { continue };
            let Some(fb) = inp.model.face_boundary(fid) else { continue };
            let Some(BoundaryKind::Deck(deck)) = inp.model.boundary(fb.boundary).map(|r| r.kind) else { continue };
            b.impose(Imposition { target: f, requirement: Requirement::NoBoundaryLayer, firmness: Firmness::Hard });
            if let Deck::Mirror { plane } = deck {
                b.impose(Imposition { target: f, requirement: Requirement::OnPlane(plane), firmness: Firmness::Hard });
            }
            if let FaceRole::Deck { side: DeckSide::B, mate: Some(a) } = fb.role
                && cl.faces.contains(&Target::Topo(a))
            {
                let t = deck.transform();
                b.impose(Imposition { target: f, requirement: Requirement::CopyOf { source: Target::Topo(a), transform: t }, firmness: Firmness::Hard });
                if let Some(map) = entity_map(inp.topo, a, fid, &t, tol) {
                    for &ea in inp.topo.face_edges(a) {
                        if let Some((eb, _)) = map.get(ea)
                            && eb != ea
                        {
                            constraints.push(IntervalConstraint::sums_equal(&[ea], &[eb], Firmness::Hard, Origin::Deck { a: ea, b: eb }));
                        }
                    }
                }
                copy_of.insert(f, (Target::Topo(a), t, fb.boundary));
            }
        }

        // 3. Surface jobs (copy jobs for deck side B).
        let mut deck_checks = Vec::new();
        for &f in &cl.faces {
            if let Some((src, t, _)) = copy_of.get(&f)
                && b.claimed.contains_key(&f)
            {
                deck_checks.push((*src, f, *t));
            }
            if b.claimed.contains_key(&f) {
                continue;
            }
            if let (Some((src, t, boundary)), Some(fid), Some(aid)) = (copy_of.get(&f), f.topo(), copy_of.get(&f).and_then(|c| c.0.topo())) {
                let map = entity_map(inp.topo, aid, fid, t, tol).ok_or(DriverError::DeckMismatch { source_target: *src, target: f })?;
                let a_edges: BTreeSet<EntityId> = inp.topo.face_edges(aid).iter().copied().collect();
                let edges: Vec<EntityId> = inp.topo.face_edges(fid).iter().copied().filter(|e| !a_edges.contains(e) && !b.claimed.contains_key(&Target::Topo(*e))).collect();
                let footprint = Footprint::default()
                    .read([*src])
                    .read(a_edges.iter().map(|&e| Target::Topo(e)))
                    .read(inp.topo.face_edges(fid).iter().filter(|e| !edges.contains(e)).map(|&e| Target::Topo(e)))
                    .write([f])
                    .write(edges.iter().map(|&e| Target::Topo(e)));
                let spec = CopySpec { source: *src, target: f, transform: *t, map, edges, reason: CopyReason::Deck { boundary: *boundary } };
                b.push(vec![f], JobKind::Copy(Box::new(spec)), None, JobPlan { footprint, ..JobPlan::default() })?;
                continue;
            }
            let choice = b.resolve(&f)?;
            b.plan(vec![f], choice, None, SchemeDim::Surface)?;
        }

        // 4. Curve jobs.
        for &e in &cl.edges {
            let t = Target::Topo(e);
            if b.claimed.contains_key(&t) {
                continue;
            }
            let choice = b.resolve(&t)?;
            b.plan(vec![t], choice, None, SchemeDim::Curve)?;
        }

        // 5. Goals from intervals and sizes (edge, then its faces, then their volumes, then default).
        for &e in &cl.edges {
            let te = Target::Topo(e);
            let mut spec: Option<((IntervalSpec, Firmness), Origin)> = intent.target(&te).intervals.map(|s| (s, Origin::User(te)));
            if spec.is_none() {
                'outer: for fi in inp.topo.coboundary(e) {
                    let tf = Target::Topo(fi.cell);
                    if let Some(s) = intent.target(&tf).intervals {
                        spec = Some((s, Origin::User(tf)));
                        break;
                    }
                    for vi in inp.topo.coboundary(fi.cell) {
                        let tv = Target::Topo(vi.cell);
                        if let Some(s) = intent.target(&tv).intervals {
                            spec = Some((s, Origin::User(tv)));
                            break 'outer;
                        }
                    }
                }
            }
            if spec.is_none() {
                spec = intent.default_size.map(|s| ((IntervalSpec::Size(s), Firmness::Default), Origin::DefaultSize));
            }
            if let Some(((s, firm), origin)) = spec {
                let count = match s {
                    IntervalSpec::Count(n) => f64::from(n),
                    IntervalSpec::Size(h) if h > 0.0 && h.is_finite() => edge_length(inp.topo, e) / h,
                    IntervalSpec::Size(h) => return Err(DriverError::Intent(format!("bad size {h}"))),
                };
                constraints.push(IntervalConstraint::goal(e, count, firm, origin));
            }
        }

        // 6. Vertex types: intent first, then plans by firmness.
        let mut vertex_types: VertexTypes = intent.vertex_types.clone();
        for p in &b.planned {
            for &(k, s) in &p.plan.vertex_types {
                match vertex_types.get(&k) {
                    Some(old) if old.firmness > s.firmness => {}
                    Some(old) if old.firmness == s.firmness && old.ty != s.ty => {
                        return Err(DriverError::Conflict { target: Target::Topo(k.face), detail: format!("vertex type {:?} vs {:?}", old.ty, s.ty) });
                    }
                    _ => {
                        vertex_types.insert(k, s);
                    }
                }
            }
        }

        // 7. Transforms (THex, QTri, boundary layers): one job per intent, over all its targets,
        // run after the targets' writers and allowed to replace their meshes.
        for ti in &intent.transforms {
            let mut targets = ti.targets.clone();
            targets.sort_unstable();
            targets.dedup();
            if targets.is_empty() {
                return Err(DriverError::Intent("transform without targets".into()));
            }
            b.plan_transform(targets, &ti.scheme)?;
        }

        // 8. Jobs, sorted by (transform last, first write target).
        let Builder { imposed, planned, attempts, .. } = b;
        let mut planned = planned;
        planned.sort_by_key(|p| (matches!(p.kind, JobKind::Transform(_)), p.plan.footprint.writes.first().copied()));
        let mut jobs = Vec::with_capacity(planned.len());
        for (i, p) in planned.into_iter().enumerate() {
            constraints.extend(p.plan.constraints);
            jobs.push(MeshJob { id: JobId(i as u32), kind: p.kind, targets: p.targets, group: p.group, footprint: p.plan.footprint, roles: p.plan.roles });
        }
        Ok(MeshProblem { jobs, constraints, vars: cl.edges.into_iter().collect(), vertices: cl.verts.into_iter().collect(), vertex_types, impositions: imposed, deck_checks, attempts })
    }

    /// Solve the intervals of every edge.
    pub fn solve(&self, solver: &dyn IntervalSolver) -> Result<IntervalSolution, DriverError> {
        Ok(solver.solve(&self.vars, &self.constraints)?)
    }

    /// Dependency waves: job `j` depends on the writer of each target it reads.
    pub fn schedule(&self) -> Result<Schedule, DriverError> {
        let is_transform = |j: &MeshJob| matches!(j.kind, JobKind::Transform(_));
        let mut writer: BTreeMap<Target, JobId> = BTreeMap::new();
        let mut rewriter: BTreeMap<Target, JobId> = BTreeMap::new();
        for j in &self.jobs {
            let map = if is_transform(j) { &mut rewriter } else { &mut writer };
            for &w in &j.footprint.writes {
                if let Some(a) = map.insert(w, j.id) {
                    return Err(DriverError::WriteConflict { target: w, a, b: j.id });
                }
            }
        }
        let n = self.jobs.len();
        let mut deps: Vec<BTreeSet<usize>> = vec![BTreeSet::new(); n];
        let mut users: Vec<BTreeSet<usize>> = vec![BTreeSet::new(); n];
        for j in &self.jobs {
            // A transform reads what it rewrites: it runs after the original writer.
            let rewrites = if is_transform(j) { j.footprint.writes.iter().filter(|w| writer.contains_key(w)).collect() } else { Vec::new() };
            for r in j.footprint.reads.iter().chain(rewrites) {
                match writer.get(r) {
                    Some(i) if *i != j.id => {
                        deps[j.id.0 as usize].insert(i.0 as usize);
                        users[i.0 as usize].insert(j.id.0 as usize);
                    }
                    Some(_) => {}
                    None if r.dim() == Some(0) => {}
                    None => return Err(DriverError::MissingWriter { job: j.id, target: *r }),
                }
            }
        }
        let mut level = vec![0usize; n];
        let mut indeg: Vec<usize> = deps.iter().map(|d| d.len()).collect();
        let mut ready: BTreeSet<usize> = (0..n).filter(|&i| indeg[i] == 0).collect();
        let mut done = 0;
        while let Some(i) = ready.pop_first() {
            done += 1;
            for &u in &users[i] {
                level[u] = level[u].max(level[i] + 1);
                indeg[u] -= 1;
                if indeg[u] == 0 {
                    ready.insert(u);
                }
            }
        }
        if done != n {
            return Err(DriverError::Cycle);
        }
        let depth = level.iter().copied().max().map_or(0, |m| m + 1);
        let mut waves = vec![Vec::new(); depth];
        for (i, &l) in level.iter().enumerate() {
            waves[l].push(JobId(i as u32));
        }
        Ok(Schedule { waves })
    }

    /// Run the waves and return the committed ledger. Nothing outside the returned ledger
    /// changes, so a failed run leaves no trace.
    pub fn execute(&self, inp: &ProblemInput<'_>, intervals: &IntervalSolution, schedule: &Schedule, exec: &dyn Executor) -> Result<Ledger, DriverError> {
        let mut ledger = Ledger::new();
        for &v in &self.vertices {
            if let Carrier::Point(p) = inp.topo.carrier(v) {
                ledger.seed_vertex(v, *p);
            }
        }
        for wave in &schedule.waves {
            let jobs: Vec<&MeshJob> = wave.iter().map(|id| &self.jobs[id.0 as usize]).collect();
            let results = {
                let cx = ExecCtx { topo: inp.topo, model: inp.model, ledger: &ledger, intervals, vertex_types: &self.vertex_types, size: inp.size };
                let task = |k: usize| -> Result<MeshPatch, SchemeFailure> { run_job(inp.registry, &cx, jobs[k]) };
                exec.run_all(jobs.len(), &task)
            };
            for (job, res) in jobs.iter().zip(results) {
                let patch = res.map_err(|failure| DriverError::JobFailed { job: Some(job.id), failure })?;
                ledger.commit(job, &patch, inp.topo).map_err(|error| DriverError::Patch { job: job.id, error })?;
            }
        }
        let tol = inp.topo.tolerance().linear * 16.0;
        for (src, dst, t) in &self.deck_checks {
            if !verify_copy(&ledger, src, dst, t, tol) {
                return Err(DriverError::DeckMismatch { source_target: *src, target: *dst });
            }
        }
        Ok(ledger)
    }
}

fn run_job(reg: &SchemeRegistry, cx: &ExecCtx<'_>, job: &MeshJob) -> Result<MeshPatch, SchemeFailure> {
    match &job.kind {
        JobKind::Scheme(s) | JobKind::Transform(s) => match reg.get(&s.name) {
            Some(scheme) => scheme.execute(cx, job),
            None => Err(SchemeFailure::fatal(s.name.as_str(), &job.targets, crate::error::FailureKind::Internal, "scheme not registered")),
        },
        JobKind::Copy(spec) => copy_patch(cx.ledger, spec),
    }
}

struct Builder<'a, 'b> {
    inp: &'b ProblemInput<'a>,
    imposed: BTreeMap<Target, Vec<Imposition>>,
    claimed: BTreeMap<Target, usize>,
    planned: Vec<Planned>,
    attempts: Vec<Attempt>,
}

impl Builder<'_, '_> {
    fn impose(&mut self, i: Imposition) {
        self.imposed.entry(i.target).or_default().push(i);
    }

    /// The scheme choice of a non-volume target: user intent vs imposed `Scheme` requirements,
    /// highest firmness wins; equal firmness with different schemes is a conflict unless the
    /// firmness is `Default`/`Soft` and the user spoke (the user wins ties).
    fn resolve(&self, t: &Target) -> Result<Option<SchemeIntent>, DriverError> {
        let user = self.inp.intent.target(t).scheme;
        let mut best: Option<(Firmness, &SchemeSpec)> = None;
        for i in self.imposed.get(t).into_iter().flatten() {
            if let Requirement::Scheme(s) = &i.requirement {
                match best {
                    Some((f, b)) if f == i.firmness && b != s && f == Firmness::Hard => {
                        return Err(DriverError::Conflict { target: *t, detail: format!("imposed '{}' vs '{}'", b.name, s.name) });
                    }
                    Some((f, _)) if f >= i.firmness => {}
                    _ => best = Some((i.firmness, s)),
                }
            }
        }
        Ok(match (user, best) {
            (Some(u), Some((f, s))) if f > u.firmness => Some(SchemeIntent { choice: SchemeChoice::Fixed(s.clone()), firmness: f }),
            (Some(u), Some((f, s))) if f == Firmness::Hard && u.firmness == Firmness::Hard && u.choice != SchemeChoice::Fixed(s.clone()) => {
                return Err(DriverError::Conflict { target: *t, detail: format!("user scheme vs imposed '{}'", s.name) });
            }
            (Some(u), _) => Some(u),
            (None, Some((f, s))) => Some(SchemeIntent { choice: SchemeChoice::Fixed(s.clone()), firmness: f }),
            (None, None) => None,
        })
    }

    fn push(&mut self, targets: Vec<Target>, kind: JobKind, group: Option<GroupKind>, mut plan: JobPlan) -> Result<(), DriverError> {
        let idx = self.planned.len();
        plan.footprint.writes.extend(targets.iter().copied());
        for &w in &plan.footprint.writes {
            if let Some(&a) = self.claimed.get(&w) {
                return Err(DriverError::WriteConflict { target: w, a: JobId(a as u32), b: JobId(idx as u32) });
            }
        }
        for &w in &plan.footprint.writes {
            self.claimed.insert(w, idx);
        }
        for i in plan.impositions.clone() {
            self.impose(i);
        }
        self.planned.push(Planned { targets, kind, group, plan });
        Ok(())
    }

    fn plan_transform(&mut self, targets: Vec<Target>, spec: &SchemeSpec) -> Result<(), DriverError> {
        let scheme = self.inp.registry.get(&spec.name).ok_or_else(|| DriverError::UnknownScheme(spec.name.clone()))?;
        if scheme.dim() != SchemeDim::Transform {
            return Err(DriverError::Intent(format!("'{}' is not a transform scheme", spec.name)));
        }
        let cx = self.inp.plan_ctx();
        let req = JobRequest { targets: &targets, params: &spec.params, imposed: &[], group: Some(GroupKind::Cluster) };
        if let Some(reason) = match scheme.applicability(&cx, &req) {
            crate::scheme::Applicability::NotApplicable { reason } => Some(reason),
            _ => None,
        } {
            self.attempts.push(Attempt { target: targets[0], scheme: spec.name.clone(), reason });
            let attempts = self.attempts.iter().filter(|a| a.target == targets[0]).cloned().collect();
            return Err(DriverError::NoApplicableScheme { target: targets[0], attempts });
        }
        let mut plan = scheme.plan(&cx, &req).map_err(|failure| DriverError::JobFailed { job: None, failure })?;
        plan.footprint.writes.extend(targets.iter().copied());
        // Rewrites do not claim: the original writer keeps its claim.
        for i in plan.impositions.clone() {
            self.impose(i);
        }
        self.planned.push(Planned { targets, kind: JobKind::Transform(spec.clone()), group: Some(GroupKind::Cluster), plan });
        Ok(())
    }

    fn plan(&mut self, targets: Vec<Target>, choice: Option<SchemeIntent>, group: Option<GroupKind>, dim: SchemeDim) -> Result<(), DriverError> {
        let reg = self.inp.registry;
        let key = targets[0];
        let excluded = self.inp.intent.excluded.get(&key).cloned().unwrap_or_default();
        let cx = self.inp.plan_ctx();
        let imposed: Vec<Imposition> = targets.iter().flat_map(|t| self.imposed.get(t).cloned().unwrap_or_default()).collect();
        let (mut cands, auto) = match choice.map(|c| c.choice) {
            Some(SchemeChoice::Fixed(s)) => (vec![s], false),
            Some(SchemeChoice::Chain(v)) => (v, false),
            Some(SchemeChoice::Auto { candidates }) if !candidates.is_empty() => (candidates.into_iter().map(|n| SchemeSpec { name: n, params: Default::default() }).collect(), true),
            _ => (reg.names_of(dim).into_iter().map(|n| SchemeSpec { name: n, params: Default::default() }).collect(), true),
        };
        cands.retain(|c| !excluded.contains(&c.name));
        // Score every candidate; auto selection orders by score (desc), then name.
        let mut scored: Vec<(u32, SchemeSpec)> = Vec::new();
        for spec in cands {
            spec.params.check().map_err(|k| DriverError::Intent(format!("parameter '{k}' of '{}' is not finite", spec.name)))?;
            let scheme = reg.get(&spec.name).ok_or_else(|| DriverError::UnknownScheme(spec.name.clone()))?;
            if scheme.dim() != dim {
                self.attempts.push(Attempt { target: key, scheme: spec.name.clone(), reason: format!("is a {:?} scheme", scheme.dim()) });
                continue;
            }
            let req = JobRequest { targets: &targets, params: &spec.params, imposed: &imposed, group };
            match scheme.applicability(&cx, &req).score() {
                Some(s) => scored.push((s, spec)),
                None => {
                    let reason = scheme.applicability(&cx, &req).reason().to_owned();
                    self.attempts.push(Attempt { target: key, scheme: spec.name.clone(), reason });
                }
            }
        }
        if auto {
            scored.sort_by(|a, b| b.0.cmp(&a.0).then_with(|| a.1.name.cmp(&b.1.name)));
        }
        for (_, spec) in scored {
            let Some(scheme) = reg.get(&spec.name) else { continue };
            let req = JobRequest { targets: &targets, params: &spec.params, imposed: &imposed, group };
            match scheme.plan(&cx, &req) {
                Ok(plan) => return self.push(targets, JobKind::Scheme(spec), group, plan),
                Err(f) if f.retry == Retry::Fatal => return Err(DriverError::JobFailed { job: None, failure: f }),
                Err(f) => self.attempts.push(Attempt { target: key, scheme: spec.name.clone(), reason: f.message }),
            }
        }
        let attempts = self.attempts.iter().filter(|a| a.target == key).cloned().collect();
        Err(DriverError::NoApplicableScheme { target: key, attempts })
    }
}

/// Result of [`run`].
#[derive(Clone, Debug)]
pub struct RunOutput {
    /// The intent as finally used (with fallback exclusions recorded for replay).
    pub intent: MeshIntent,
    /// The final problem.
    pub problem: MeshProblem,
    /// Its interval solution.
    pub intervals: IntervalSolution,
    /// Its schedule.
    pub schedule: Schedule,
    /// The committed meshes.
    pub ledger: Ledger,
}

/// Build, solve, schedule and execute; on an execution failure that allows fallback, exclude the
/// failed scheme for the job's first target and start over (at most `max_attempts` times).
pub fn run(inp: &ProblemInput<'_>, solver: &dyn IntervalSolver, exec: &dyn Executor, max_attempts: usize) -> Result<RunOutput, DriverError> {
    let mut intent = inp.intent.clone();
    for _ in 0..max_attempts {
        let cur = ProblemInput { intent: &intent, ..*inp };
        let attempt = (|| {
            let problem = MeshProblem::build(&cur)?;
            let intervals = problem.solve(solver)?;
            let schedule = problem.schedule()?;
            let ledger = problem.execute(&cur, &intervals, &schedule, exec)?;
            Ok::<_, DriverError>((problem, intervals, schedule, ledger))
        })();
        match attempt {
            Ok((problem, intervals, schedule, ledger)) => return Ok(RunOutput { intent, problem, intervals, schedule, ledger }),
            Err(DriverError::JobFailed { job: Some(_), failure }) if failure.retry == Retry::NextInChain => {
                let Some(&key) = failure.targets.first() else { return Err(DriverError::JobFailed { job: None, failure }) };
                let list = intent.excluded.entry(key).or_default();
                if list.contains(&failure.scheme) {
                    return Err(DriverError::JobFailed { job: None, failure });
                }
                list.push(failure.scheme.clone());
            }
            Err(e) => return Err(e),
        }
    }
    Err(DriverError::FallbackLimit(max_attempts))
}

/// Dimension of a topology id as a `Dim`, for schemes that need it.
pub fn dim_of(topo: &dyn TopologyView, id: EntityId) -> Option<Dim> {
    topo.dim_of(id)
}
