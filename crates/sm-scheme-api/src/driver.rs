//! The driver: plan (see `planner`), generate patches wave by wave (concurrently inside a
//! wave), verify copies, and commit in a fixed order into a [`MeshBuild`]; `mesh` re-plans
//! around failures.

use crate::copy;
use crate::error::{Attempt, FailureKind, Phase, Result, SchemeError};
use crate::exec::Executor;
use crate::geom;
use crate::intent::{MeshIntent, SchemeName};
use crate::patch::{ElemShape, Patch, PatchBuilder, PatchSet};
use crate::plan::{CopyMode, Plan};
use crate::problem::{MeshConstraint, Selection};
use crate::scheme::{GenCtx, MeshInput, SchemeRegistry};
use crate::sizing::{SizeField, builtin_field};
use crate::solve::{CornerPicker, IntervalSolver};
use crate::target::{Job, NodeKey, SchemeDim, Target};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use sm_core::EntityId;
use sm_mesh_api::{CellShape, Classification, Finished, MeshBuild, MeshEntityRef, SerialAlloc, VertexIx};
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Mutex;

/// Driver options.
#[derive(Clone, Copy, Debug)]
pub struct DriverOptions {
    /// Maximum number of re-plans after failures a different scheme could avoid.
    pub max_replans: usize,
    /// Also classify mesh edges on curves (creates the mesh edge table).
    pub classify_edges: bool,
    /// Tolerance of copy redundancy checks and verification (default: the model's linear
    /// tolerance).
    pub copy_tolerance: Option<f64>,
}

impl Default for DriverOptions {
    fn default() -> Self {
        Self { max_replans: 16, classify_edges: false, copy_tolerance: None }
    }
}

/// `(target, scheme)` pairs rejected after a failure, with the attempt that caused each.
///
/// Serializes as the list of attempts (JSON cannot key a map by a pair). Stored in
/// [`Plan::rejections`], not in the intent: replaying `(intent, rejections)` through
/// [`Driver::plan_with`] reproduces a plan, and the user's Op payload never holds solver state.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Rejections(pub BTreeMap<(Target, SchemeName), Attempt>);

impl Rejections {
    /// Reject the attempt's `(target, scheme)`.
    pub fn insert(&mut self, a: Attempt) {
        self.0.insert((a.target, a.scheme.clone()), a);
    }
    /// True if `(target, scheme)` is rejected.
    pub fn contains(&self, t: Target, scheme: &SchemeName) -> bool {
        self.0.contains_key(&(t, scheme.clone()))
    }
    /// True if nothing is rejected.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
    /// Number of rejected pairs.
    pub fn len(&self) -> usize {
        self.0.len()
    }
    /// The attempts, in `(target, scheme)` order.
    pub fn attempts(&self) -> impl Iterator<Item = &Attempt> {
        self.0.values()
    }
}

impl Serialize for Rejections {
    fn serialize<S: Serializer>(&self, s: S) -> core::result::Result<S::Ok, S::Error> {
        s.collect_seq(self.0.values())
    }
}

impl<'de> Deserialize<'de> for Rejections {
    fn deserialize<D: Deserializer<'de>>(d: D) -> core::result::Result<Self, D::Error> {
        let v = Vec::<Attempt>::deserialize(d)?;
        let mut r = Rejections::default();
        for a in v {
            r.insert(a);
        }
        Ok(r)
    }
}

/// The result of [`Driver::mesh`].
pub struct Meshed<M> {
    /// The plan that succeeded.
    pub plan: Plan,
    /// Its patches.
    pub patches: PatchSet,
    /// The finished mesh.
    pub finished: Finished<M>,
    /// Every attempt that was rejected or failed on the way, in order: failures that caused a
    /// re-plan first, then the passed-over candidates of the final plan.
    pub attempts: Vec<Attempt>,
}

/// The meshing driver. Holds the registry and the solvers; stateless otherwise, so one driver
/// can mesh many models (and the same input always gives the same output).
pub struct Driver<'r> {
    /// Schemes.
    pub registry: &'r SchemeRegistry,
    /// Interval solver.
    pub solver: &'r dyn IntervalSolver,
    /// Corner picker.
    pub picker: &'r dyn CornerPicker,
    /// Size field override (else built from `intent.defaults.sizing`).
    pub size: Option<&'r dyn SizeField>,
    /// Options.
    pub options: DriverOptions,
}

/// What one job produced.
enum JobOut {
    Patch(Patch),
    Transformed(Vec<Patch>),
}

impl<'r> Driver<'r> {
    /// A driver with default options and no size override.
    pub fn new(registry: &'r SchemeRegistry, solver: &'r dyn IntervalSolver, picker: &'r dyn CornerPicker) -> Self {
        Self { registry, solver, picker, size: None, options: DriverOptions::default() }
    }

    /// Generate every patch of the plan, wave by wave. Inside a wave the executor may run jobs
    /// concurrently; results are merged in job order, so the output is identical for every
    /// executor. The size field is `self.size`, else rebuilt from `plan.sizing`.
    pub fn generate(&self, input: &MeshInput<'_>, plan: &Plan, exec: &dyn Executor) -> Result<PatchSet> {
        let owned = if self.size.is_none() { plan.sizing.as_ref().and_then(builtin_field) } else { None };
        let field: Option<&dyn SizeField> = self.size.or(owned.as_deref());
        self.generate_with(input, plan, field, exec)
    }

    fn generate_with(&self, input: &MeshInput<'_>, plan: &Plan, size: Option<&dyn SizeField>, exec: &dyn Executor) -> Result<PatchSet> {
        let mut ps = PatchSet::new();
        for &v in &plan.problem.vertices {
            if !ps.add_vertex_node(NodeKey::vertex(v), geom::vertex_point(input.topo, v)?) {
                return Err(SchemeError::Input(format!("vertex {v} listed twice")));
            }
        }
        let ctx = CheckCtx::new(plan);
        for wave in &plan.waves {
            let slots: Vec<Mutex<Option<Result<JobOut>>>> = wave.iter().map(|_| Mutex::new(None)).collect();
            {
                let ps_ref = &ps;
                exec.run(wave.len(), &|i| {
                    let r = self.run_job(input, plan, ps_ref, size, wave[i]);
                    *slots[i].lock().unwrap_or_else(std::sync::PoisonError::into_inner) = Some(r);
                });
            }
            // Merge in job order. The first error in job order is the one reported.
            let mut replaced: Vec<Patch> = Vec::new();
            for (job, slot) in wave.iter().zip(slots) {
                let out = slot.into_inner().unwrap_or_else(std::sync::PoisonError::into_inner).ok_or_else(|| SchemeError::Input(format!("{job:?} did not run")))??;
                match (job, out) {
                    (Job::Generate(t), JobOut::Patch(patch)) => {
                        ctx.check_patch(&ps, *t, &patch)?;
                        let (main, children) = ctx.split_claims(*t, patch)?;
                        for c in children {
                            let target = c.target;
                            ps.insert(c).map_err(|k| SchemeError::InvalidPatch { target, reason: format!("duplicate node {k:?}") })?;
                        }
                        ps.insert(main).map_err(|k| SchemeError::InvalidPatch { target: *t, reason: format!("duplicate node {k:?}") })?;
                    }
                    (Job::Transform(_), JobOut::Transformed(ps2)) => replaced.extend(ps2),
                    _ => return Err(SchemeError::Input(format!("{job:?} returned the wrong kind of result"))),
                }
            }
            if !replaced.is_empty() {
                let targets: Vec<Target> = replaced.iter().map(|p| p.target).collect();
                let mut next = ps.without(&targets);
                for patch in replaced {
                    ctx.check_patch(&next, patch.target, &patch)?;
                    let target = patch.target;
                    next.insert(patch).map_err(|k| SchemeError::InvalidPatch { target, reason: format!("duplicate node {k:?}") })?;
                }
                ps = next;
            }
        }
        // Members that a parent writes must equal the image of their root.
        let tol = self.options.copy_tolerance.unwrap_or(input.topo.tolerance().linear);
        for class in &plan.copies {
            for m in class.members.iter().filter(|m| m.mode == CopyMode::Verify) {
                copy::verify_copy(input.topo, &ps, class, m, tol)?;
            }
        }
        Ok(ps)
    }

    fn run_job(&self, input: &MeshInput<'_>, plan: &Plan, ps: &PatchSet, size: Option<&dyn SizeField>, job: Job) -> Result<JobOut> {
        match job {
            Job::Generate(t) => {
                let tp = plan.target(t).ok_or_else(|| SchemeError::Input(format!("{t:?} is not in the plan")))?;
                if let Selection::Copy { .. } = tp.selection {
                    let (class, m) = plan.copy_of(t).ok_or_else(|| SchemeError::Input(format!("{t:?}: copy without a class")))?;
                    let p = copy::copy_patch(input.topo, ps, class.root, t, &m.from_root, &m.vertex_map, &|e| plan.intervals(e))?;
                    return Ok(JobOut::Patch(p));
                }
                let s = self.registry.get(t.dim(), tp.scheme.as_str()).ok_or_else(|| SchemeError::UnknownScheme { name: tp.scheme.clone(), target: t })?;
                let cx = GenCtx { input, plan, patches: ps, size };
                Ok(JobOut::Patch(s.generate(&cx, t, &tp.params)?))
            }
            Job::Transform(id) => {
                let tp = plan.transforms.iter().find(|x| x.id == id).ok_or_else(|| SchemeError::Input(format!("transform {id:?} is not planned")))?;
                let tr = self.registry.transform(tp.transform.name.as_str()).ok_or_else(|| SchemeError::UnknownScheme { name: tp.transform.name.clone(), target: tp.targets[0] })?;
                let mut patches: Vec<Patch> = Vec::with_capacity(tp.targets.len());
                for t in &tp.targets {
                    patches.push(ps.get(*t).cloned().ok_or_else(|| SchemeError::Input(format!("transform {id:?}: {t:?} has no patch")))?);
                }
                let cx = GenCtx { input, plan, patches: ps, size };
                tr.apply(&cx, &tp.targets, &tp.transform.params, &mut patches)?;
                if patches.len() != tp.targets.len() || patches.iter().zip(&tp.targets).any(|(p, t)| p.target != *t) {
                    return Err(SchemeError::InvalidPatch { target: tp.targets[0], reason: "a transform must keep one patch per target, in order".into() });
                }
                Ok(JobOut::Transformed(patches))
            }
        }
    }

    /// Write every patch into `b` in a fixed order (nodes in patch-set order, then volume
    /// cells, then surface classifications, then optionally curve segments) and finish.
    /// All-or-nothing: on error nothing was finished and `alloc` is unchanged
    /// (`MeshBuild::finish` contract).
    pub fn commit<B: MeshBuild>(&self, input: &MeshInput<'_>, ps: &PatchSet, mut b: B, alloc: &mut SerialAlloc) -> Result<Finished<B::Output>> {
        let classify = |owner: EntityId| input.pieces.classify(owner).ok_or(SchemeError::Unclassifiable { owner });
        let mut verts: Vec<VertexIx> = Vec::with_capacity(ps.node_count());
        for (k, pt, param) in ps.nodes() {
            let v = b.add_vertex(pt)?;
            let ent = classify(k.owner)?;
            let param = if ent == k.owner && param.fits(ent.kind()) { param } else { sm_mesh_api::Param::None };
            b.set_class(MeshEntityRef::Vertex(v), Classification { entity: ent, param })?;
            verts.push(v);
        }
        // Local index -> mesh vertex, one lookup per node or reference of a patch.
        let table = |patch: &Patch| -> Result<Vec<VertexIx>> {
            let mut t = Vec::with_capacity(patch.local_len());
            for n in &patch.nodes {
                t.push(*ps.index_of(n.key).and_then(|i| verts.get(i as usize)).ok_or_else(|| SchemeError::InvalidPatch { target: patch.target, reason: format!("node {:?} missing", n.key) })?);
            }
            for k in &patch.refs {
                t.push(*ps.index_of(*k).and_then(|i| verts.get(i as usize)).ok_or_else(|| SchemeError::InvalidPatch { target: patch.target, reason: format!("node {k:?} missing") })?);
            }
            Ok(t)
        };
        let bad = |t: Target, reason: String| SchemeError::InvalidPatch { target: t, reason };
        let mut vs: Vec<VertexIx> = Vec::with_capacity(8);
        let mut faces: Vec<Vec<VertexIx>> = Vec::new();
        for patch in ps.patches().filter(|p| p.target.dim() == SchemeDim::Volume) {
            let t = patch.target;
            let tab = table(patch)?;
            for (i, (shape, nodes)) in patch.elements.iter().enumerate() {
                let map = |l: u32| tab.get(l as usize).copied().ok_or_else(|| bad(t, format!("local index {l} out of range")));
                let added = match shape {
                    ElemShape::Cell(cs) => {
                        if cs == CellShape::Polyhedron {
                            return Err(bad(t, "use ElemShape::Polyhedron for polyhedral cells".into()));
                        }
                        vs.clear();
                        for &l in nodes {
                            vs.push(map(l)?);
                        }
                        b.add_cell_from_vertices(cs, &vs)?
                    }
                    ElemShape::Polyhedron => {
                        faces.clear();
                        for f in patch.elements.polyhedron_faces(i).into_iter().flatten() {
                            faces.push(f.iter().map(|&l| map(l)).collect::<Result<Vec<_>>>()?);
                        }
                        let refs: Vec<&[VertexIx]> = faces.iter().map(Vec::as_slice).collect();
                        b.add_polyhedron(&refs)?
                    }
                    other => return Err(bad(t, format!("volume patch element {other:?}"))),
                };
                let owner = match (t.entity(), patch.owners.get(i)) {
                    (_, Some(o)) => *o,
                    (Some(e), None) => e,
                    (None, None) => return Err(bad(t, "group cell without owner".into())),
                };
                b.set_class(MeshEntityRef::Cell(added.cell), Classification::on(classify(owner)?))?;
            }
        }
        for patch in ps.patches().filter(|p| p.target.dim() == SchemeDim::Surface) {
            let t = patch.target;
            let Some(face) = t.entity() else { continue };
            let class = Classification::on(classify(face)?);
            let tab = table(patch)?;
            for (_, nodes) in patch.elements.iter() {
                vs.clear();
                for &l in nodes {
                    vs.push(tab.get(l as usize).copied().ok_or_else(|| bad(t, format!("local index {l} out of range")))?);
                }
                let (f, _) = b.find_face(&vs).ok_or_else(|| bad(t, "surface element bounds no cell (surface-only meshes are not representable in sm-mesh-api)".into()))?;
                b.set_class(MeshEntityRef::Face(f), class)?;
            }
        }
        if self.options.classify_edges {
            for patch in ps.patches().filter(|p| p.target.dim() == SchemeDim::Curve) {
                let t = patch.target;
                let Some(edge) = t.entity() else { continue };
                let class = Classification::on(classify(edge)?);
                let tab = table(patch)?;
                for (_, nodes) in patch.elements.iter() {
                    if let [x, y] = nodes {
                        let (x, y) = (tab.get(*x as usize).copied(), tab.get(*y as usize).copied());
                        let (Some(x), Some(y)) = (x, y) else { return Err(bad(t, "segment index out of range".into())) };
                        b.set_class(MeshEntityRef::Edge(x, y), class)?;
                    }
                }
            }
        }
        Ok(b.finish(alloc)?)
    }

    /// Plan, generate and commit, re-planning around failures a different scheme could avoid:
    /// the error's [`SchemeError::blame`] (an infeasible interval system blames the cheapest
    /// scheme-origin constraint, a child with no applicable scheme blames its imposing parent,
    /// a fallback-eligible generation failure blames its scheme) is rejected and the whole
    /// problem is re-planned, up to `options.max_replans` times.
    pub fn mesh<B: MeshBuild>(&self, input: &MeshInput<'_>, intent: &MeshIntent, exec: &dyn Executor, builder: B, alloc: &mut SerialAlloc) -> Result<Meshed<B::Output>> {
        self.mesh_with(input, intent, &Rejections::default(), exec, builder, alloc)
    }

    /// [`Driver::mesh`] starting from known rejections (replaying a stored plan).
    pub fn mesh_with<B: MeshBuild>(&self, input: &MeshInput<'_>, intent: &MeshIntent, rej: &Rejections, exec: &dyn Executor, builder: B, alloc: &mut SerialAlloc) -> Result<Meshed<B::Output>> {
        let owned = if self.size.is_none() { intent.defaults.sizing.as_ref().and_then(builtin_field) } else { None };
        let field: Option<&dyn SizeField> = self.size.or(owned.as_deref());
        let mut rej = rej.clone();
        let mut history: Vec<Attempt> = Vec::new();
        for _ in 0..=self.options.max_replans {
            let step = match self.plan_with(input, intent, &rej) {
                Err(e) => {
                    let blame = e.blame();
                    Err((e, blame))
                }
                Ok(plan) => match self.generate_with(input, &plan, field, exec) {
                    Ok(ps) => Ok((plan, ps)),
                    Err(e) => Err(self.annotate(&plan, e)),
                },
            };
            match step {
                Ok((plan, patches)) => {
                    let finished = self.commit(input, &patches, builder, alloc)?;
                    let mut attempts = history;
                    for a in &plan.attempts {
                        if !attempts.contains(a) {
                            attempts.push(a.clone());
                        }
                    }
                    return Ok(Meshed { plan, patches, finished, attempts });
                }
                Err((e, blame)) => match blame {
                    Some((t, s)) if s.as_str() != SchemeName::COPY && !rej.contains(t, &s) => {
                        let a = Attempt { target: t, scheme: s, phase: phase_of(&e), kind: kind_of(&e), message: e.to_string() };
                        history.push(a.clone());
                        rej.insert(a);
                    }
                    _ => return Err(e),
                },
            }
        }
        Err(SchemeError::ReplanLimit { attempts: history })
    }

    /// Attach the blame of a generation-time error that needs the plan to be found.
    fn annotate(&self, plan: &Plan, e: SchemeError) -> (SchemeError, Option<(Target, SchemeName)>) {
        let blame = match &e {
            SchemeError::CopyMismatch { constraint, .. } => {
                // The scheme that wrote the member (its claimer), else the one that asked for the copy.
                let member = plan.copies.iter().flat_map(|c| c.members.iter()).find(|m| m.constraints.last() == Some(constraint)).map(|m| m.target);
                let writer = member.and_then(|m| plan.claimed_by(m));
                writer
                    .and_then(|w| plan.target(w).map(|tp| (w, tp.scheme.clone())))
                    .or_else(|| match plan.problem.record_origin(*constraint) {
                        Some(crate::problem::Origin::Scheme { scheme, target }) => Some((*target, scheme.clone())),
                        _ => None,
                    })
            }
            other => other.blame(),
        };
        (e, blame)
    }
}

fn phase_of(e: &SchemeError) -> Phase {
    match e {
        SchemeError::Infeasible(_) => Phase::Intervals,
        SchemeError::NoApplicableScheme { .. } | SchemeError::NotApplicable { .. } | SchemeError::ImpositionConflict { .. } => Phase::Resolve,
        SchemeError::CopyMismatch { .. } => Phase::Verify,
        SchemeError::Generate(_) => Phase::Generate,
        _ => Phase::Generate,
    }
}

fn kind_of(e: &SchemeError) -> FailureKind {
    match e {
        SchemeError::Generate(g) => g.kind,
        SchemeError::NoApplicableScheme { .. } | SchemeError::NotApplicable { .. } => FailureKind::NotApplicable,
        _ => FailureKind::Constraint,
    }
}

/// Patch-contract checks that need the plan: owners and claims.
struct CheckCtx<'p> {
    plan: &'p Plan,
    members: BTreeMap<Target, Vec<EntityId>>,
    claimed: BTreeMap<Target, Vec<Target>>,
}

impl<'p> CheckCtx<'p> {
    fn new(plan: &'p Plan) -> Self {
        let mut members: BTreeMap<Target, Vec<EntityId>> = BTreeMap::new();
        for r in &plan.problem.constraints {
            if let MeshConstraint::Atomic { group, members: ms } = &r.item {
                members.insert(Target::Group(*group), ms.iter().filter_map(Target::entity).collect());
            }
        }
        let mut claimed: BTreeMap<Target, Vec<Target>> = BTreeMap::new();
        for c in &plan.problem.claims {
            claimed.entry(c.item.by).or_default().push(c.item.child);
        }
        Self { plan, members, claimed }
    }

    /// The patch contract: right target; nodes owned by the target, a group member or a
    /// claimed child (curves: exactly the indices `1..=n-1`); unique keys; parameters that fit;
    /// element shapes with the right node counts; every reference already known; owners empty
    /// or one per element; surface elements in a volume patch only for claimed faces.
    fn check_patch(&self, ps: &PatchSet, t: Target, patch: &Patch) -> Result<()> {
        let bad = |reason: String| SchemeError::InvalidPatch { target: t, reason };
        if patch.target != t {
            return Err(bad(format!("patch is for {:?}", patch.target)));
        }
        let mut owners: Vec<EntityId> = match t {
            Target::Group(_) => self.members.get(&t).cloned().unwrap_or_default(),
            _ => t.entity().into_iter().collect(),
        };
        let claimed: Vec<EntityId> = self.claimed.get(&t).into_iter().flatten().filter_map(Target::entity).collect();
        owners.extend(claimed.iter().copied());
        let mut keys: BTreeSet<NodeKey> = BTreeSet::new();
        for n in &patch.nodes {
            if !owners.contains(&n.key.owner) {
                return Err(bad(format!("node {:?} is not owned by the target", n.key)));
            }
            if !keys.insert(n.key) || ps.index_of(n.key).is_some() {
                return Err(bad(format!("duplicate node {:?}", n.key)));
            }
            if !n.param.fits(n.key.owner.kind()) {
                return Err(bad(format!("node {:?}: parameter does not fit its owner", n.key)));
            }
        }
        let mut ref_set: BTreeSet<NodeKey> = BTreeSet::new();
        for r in &patch.refs {
            if ps.index_of(*r).is_none() {
                return Err(bad(format!("element references unknown node {r:?}")));
            }
            if !ref_set.insert(*r) {
                return Err(bad(format!("reference {r:?} listed twice")));
            }
        }
        if let Target::Curve(e) = t {
            let n = self.plan.intervals(e).ok_or_else(|| bad("no interval count".into()))?;
            let want: BTreeSet<NodeKey> = (1..n).map(|i| NodeKey::new(e, i)).collect();
            if keys != want {
                return Err(bad(format!("a curve with {n} intervals needs nodes 1..={}", n.saturating_sub(1))));
            }
        }
        let locals = patch.local_len();
        if let Some(&l) = patch.elements.indices().iter().find(|&&l| l as usize >= locals) {
            return Err(bad(format!("element index {l} out of range ({locals} locals)")));
        }
        if !patch.owners.is_empty() && patch.owners.len() != patch.elements.len() {
            return Err(bad("owners must be empty or one per element".into()));
        }
        for (i, (shape, nodes)) in patch.elements.iter().enumerate() {
            let n = nodes.len();
            let owner = patch.owners.get(i).copied();
            let ok = match shape {
                ElemShape::Segment => n == 2,
                ElemShape::Tri => n == 3,
                ElemShape::Quad => n == 4,
                ElemShape::Polygon => n >= 3,
                ElemShape::Cell(cs) => cs != CellShape::Polyhedron && cs.vertex_count() == Some(n),
                ElemShape::Polyhedron => patch.elements.polyhedron_faces(i).is_some_and(|mut f| f.all(|l| l.len() >= 3)),
            };
            if !ok {
                return Err(bad(format!("element {i}: {shape:?} with {n} nodes")));
            }
            let want_dim = match shape {
                ElemShape::Segment => SchemeDim::Curve,
                ElemShape::Tri | ElemShape::Quad | ElemShape::Polygon => SchemeDim::Surface,
                ElemShape::Cell(_) | ElemShape::Polyhedron => SchemeDim::Volume,
            };
            match (t.dim(), want_dim) {
                (a, b) if a == b => {}
                (SchemeDim::Volume, SchemeDim::Surface) => {
                    // Surface elements of a volume patch belong to claimed faces.
                    if !owner.is_some_and(|o| claimed.contains(&o)) {
                        return Err(bad(format!("element {i}: a surface element in a volume patch needs a claimed face as owner")));
                    }
                }
                (SchemeDim::Volume, SchemeDim::Curve) => {
                    if !owner.is_some_and(|o| claimed.contains(&o)) {
                        return Err(bad(format!("element {i}: a segment in a volume patch needs a claimed edge as owner")));
                    }
                }
                (SchemeDim::Surface, SchemeDim::Curve) => {
                    if !owner.is_some_and(|o| claimed.contains(&o)) {
                        return Err(bad(format!("element {i}: a segment in a surface patch needs a claimed edge as owner")));
                    }
                }
                (a, b) => return Err(bad(format!("element {i}: {b:?} element in a {a:?} patch"))),
            }
            if let Some(o) = owner
                && !owners.contains(&o)
            {
                return Err(bad(format!("element {i}: owner {o} is neither a member nor a claimed child")));
            }
        }
        if let Some(g) = &patch.grid
            && g.nodes.len() != g.ni as usize * g.nj as usize
        {
            return Err(bad("grid size does not match ni * nj".into()));
        }
        Ok(())
    }

    /// Move the nodes and elements owned by claimed children out of a claimer's patch into
    /// patches of their own (target = child), so claimed faces and curves are classified and
    /// read like any other. Returns the remainder and the child patches (child patches merge
    /// first: the remainder may reference their nodes).
    fn split_claims(&self, t: Target, patch: Patch) -> Result<(Patch, Vec<Patch>)> {
        let Some(children) = self.claimed.get(&t) else { return Ok((patch, Vec::new())) };
        let child_of = |o: EntityId| children.iter().copied().find(|c| c.entity() == Some(o));
        let touched = patch.nodes.iter().any(|n| child_of(n.key.owner).is_some()) || patch.owners.iter().any(|o| child_of(*o).is_some());
        if !touched {
            return Ok((patch, Vec::new()));
        }
        let mut kids: BTreeMap<Target, PatchBuilder> = children.iter().map(|c| (*c, PatchBuilder::new(*c))).collect();
        let mut main = PatchBuilder::new(t);
        // Nodes first, in order, so numbering follows the original.
        for n in &patch.nodes {
            match child_of(n.key.owner) {
                Some(c) => {
                    if let Some(b) = kids.get_mut(&c) {
                        b.node(n.key, n.point, n.param);
                    }
                }
                None => {
                    main.node(n.key, n.point, n.param);
                }
            }
        }
        let key_of = |l: u32| patch.key(l).ok_or_else(|| SchemeError::InvalidPatch { target: t, reason: format!("local index {l} out of range") });
        // Local index of `key` inside builder `b`: own node or reference.
        fn local(b: &mut PatchBuilder, own: &BTreeMap<NodeKey, u32>, key: NodeKey) -> u32 {
            own.get(&key).copied().unwrap_or_else(|| b.refer(key))
        }
        let mut main_own: BTreeMap<NodeKey, u32> = BTreeMap::new();
        let mut kid_own: BTreeMap<Target, BTreeMap<NodeKey, u32>> = BTreeMap::new();
        {
            let (mut mi, mut ki): (u32, BTreeMap<Target, u32>) = (0, BTreeMap::new());
            for n in &patch.nodes {
                match child_of(n.key.owner) {
                    Some(c) => {
                        let i = ki.entry(c).or_insert(0);
                        kid_own.entry(c).or_default().insert(n.key, *i);
                        *i += 1;
                    }
                    None => {
                        main_own.insert(n.key, mi);
                        mi += 1;
                    }
                }
            }
        }
        let empty = BTreeMap::new();
        for (i, (shape, nodes)) in patch.elements.iter().enumerate() {
            let owner = patch.owners.get(i).copied();
            let kid = owner.and_then(child_of).filter(|_| !matches!(shape, ElemShape::Cell(_) | ElemShape::Polyhedron));
            match kid {
                Some(c) => {
                    let own = kid_own.get(&c).unwrap_or(&empty);
                    let b = kids.get_mut(&c).ok_or_else(|| SchemeError::InvalidPatch { target: t, reason: "claimed child lost".into() })?;
                    let mut ls: Vec<u32> = Vec::with_capacity(nodes.len());
                    for &l in nodes {
                        ls.push(local(b, own, key_of(l)?));
                    }
                    b.push(shape, &ls);
                }
                None => {
                    if let Some(o) = owner {
                        main.owner(o);
                    }
                    if shape == ElemShape::Polyhedron {
                        let mut fs: Vec<Vec<u32>> = Vec::new();
                        for f in patch.elements.polyhedron_faces(i).into_iter().flatten() {
                            let mut ls = Vec::with_capacity(f.len());
                            for &l in f {
                                ls.push(local(&mut main, &main_own, key_of(l)?));
                            }
                            fs.push(ls);
                        }
                        let refs: Vec<&[u32]> = fs.iter().map(Vec::as_slice).collect();
                        main.push_polyhedron(&refs);
                    } else {
                        let mut ls: Vec<u32> = Vec::with_capacity(nodes.len());
                        for &l in nodes {
                            ls.push(local(&mut main, &main_own, key_of(l)?));
                        }
                        main.push(shape, &ls);
                    }
                }
            }
        }
        if let Some(g) = patch.grid {
            main.grid(g);
        }
        let mut out: Vec<Patch> = kids.into_values().map(PatchBuilder::finish).filter(|p| !p.nodes.is_empty() || !p.elements.is_empty()).collect();
        out.sort_by_key(|p| p.target);
        Ok((main.finish(), out))
    }
}
