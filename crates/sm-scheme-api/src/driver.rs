//! The driver: assemble the [`MeshingProblem`], solve it into a [`Plan`], generate patches
//! wave by wave (concurrently inside a wave), and commit them in a fixed order into a
//! [`MeshBuild`].

use crate::copy;
use crate::error::{Infeasible, Result, SchemeError};
use crate::exec::Executor;
use crate::firm::{Firm, Firmness};
use crate::geom;
use crate::intent::{IntervalRequest, MeshIntent, Params, SchemeChoice, SchemeName};
use crate::patch::{ElemShape, Patch, PatchSet};
use crate::plan::Plan;
use crate::problem::{ConstraintId, CopyConstraint, CopyReason, IntervalConstraint, IntervalSink, MeshConstraint, MeshingProblem, Origin, Requirements, Role, Selection, TargetPlan};
use crate::scheme::{Applicability, GenCtx, IntervalCtx, MeshInput, PlanCtx, SchemeRegistry};
use crate::sizing::{SizeField, builtin_field};
use crate::solve::{CornerPicker, IntervalSolver};
use crate::target::{GroupId, NodeKey, SchemeDim, Target, piece_of};
use crate::vertex::{FaceLayout, VertexRule, VertexType};
use sm_core::{EntityId, Transform};
use sm_mesh_api::{CellShape, Classification, Finished, MeshBuild, MeshEntityRef, Param, SerialAlloc, VertexIx};
use sm_model_api::{Deck, FaceKindView, PieceRef, PieceShape};
use sm_topo_api::{CornerKey, Dim, TopologyViewExt};
use std::collections::{BTreeMap, BTreeSet};

/// Driver options.
#[derive(Clone, Copy, Debug)]
pub struct DriverOptions {
    /// Maximum number of re-plans after fallback-eligible generation failures.
    pub max_replans: usize,
    /// Also classify mesh edges on curves (creates the mesh edge table).
    pub classify_edges: bool,
}

impl Default for DriverOptions {
    fn default() -> Self {
        Self { max_replans: 16, classify_edges: false }
    }
}

/// `(target, scheme)` pairs rejected after a generation failure, with the reason.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Rejections(pub BTreeMap<(Target, SchemeName), String>);

/// The result of [`Driver::mesh`].
pub struct Meshed<M> {
    /// The plan that succeeded.
    pub plan: Plan,
    /// Its patches.
    pub patches: PatchSet,
    /// The finished mesh.
    pub finished: Finished<M>,
    /// Rejections accumulated by fallbacks.
    pub rejections: Rejections,
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

struct Region {
    volumes: Vec<EntityId>,
    groups: Vec<(GroupId, Vec<EntityId>)>,
    grouped: BTreeMap<EntityId, GroupId>,
    faces: Vec<EntityId>,
    edges: Vec<EntityId>,
    vertices: Vec<EntityId>,
}

fn sorted<T: Ord>(mut v: Vec<T>) -> Vec<T> {
    v.sort();
    v.dedup();
    v
}

impl<'r> Driver<'r> {
    /// A driver with default options and no size override.
    pub fn new(registry: &'r SchemeRegistry, solver: &'r dyn IntervalSolver, picker: &'r dyn CornerPicker) -> Self {
        Self { registry, solver, picker, size: None, options: DriverOptions::default() }
    }

    fn region(&self, input: &MeshInput<'_>, intent: &MeshIntent) -> Result<Region> {
        let volumes = sorted(input.selection.to_vec());
        if let Some(v) = volumes.iter().find(|v| piece_of(**v).is_none()) {
            return Err(SchemeError::Input(format!("{v} is not a TopoVolume or PartitionCell")));
        }
        let mut grouped = BTreeMap::new();
        let mut groups = Vec::new();
        for g in &intent.groups {
            let members = sorted(g.members.clone());
            for m in &members {
                if volumes.binary_search(m).is_err() {
                    return Err(SchemeError::Input(format!("group {:?}: member {m} is not selected", g.id)));
                }
                if grouped.insert(*m, g.id).is_some() {
                    return Err(SchemeError::Input(format!("{m} is in two groups")));
                }
            }
            groups.push((g.id, members));
        }
        groups.sort();
        let mut faces = BTreeSet::new();
        for v in &volumes {
            faces.extend(input.piece_faces(Target::Volume(*v))?);
        }
        let faces: Vec<EntityId> = faces.into_iter().collect();
        let edges = sorted(faces.iter().flat_map(|f| input.face_edges(*f)).collect());
        let vertices = sorted(edges.iter().filter_map(|e| input.topo.edge_ends(*e)).flat_map(|(a, b)| [a, b]).collect());
        Ok(Region { volumes, groups, grouped, faces, edges, vertices })
    }

    /// Assemble and solve the problem.
    pub fn plan(&self, input: &MeshInput<'_>, intent: &MeshIntent) -> Result<Plan> {
        self.plan_with(input, intent, &Rejections::default())
    }

    /// Assemble and solve the problem, skipping rejected `(target, scheme)` pairs.
    pub fn plan_with(&self, input: &MeshInput<'_>, intent: &MeshIntent, rej: &Rejections) -> Result<Plan> {
        let topo = input.topo;
        let region = self.region(input, intent)?;
        let cx = PlanCtx { input, intent };
        let mut p = MeshingProblem::default();
        p.vertices = region.vertices.clone();
        let mut plans: BTreeMap<Target, TargetPlan> = BTreeMap::new();

        // Deck boundaries (model layer): copies, no boundary layers, plane locks.
        for &f in &region.faces {
            if let Some(FaceKindView::Deck { boundary, deck, side, mate }) = input.model.face_kind(f, topo) {
                if intent.entities.get(&f).is_some_and(|e| e.boundary_layer.is_some()) {
                    return Err(SchemeError::BoundaryLayerOnDeck { face: f, boundary });
                }
                let origin = Origin::Deck { boundary };
                p.push_constraint(origin.clone(), MeshConstraint::NoBoundaryLayer { face: f });
                match (deck, side, mate) {
                    (Deck::Mirror { plane }, _, _) => {
                        p.push_constraint(origin, MeshConstraint::PlaneLock { face: f, normal: plane.normal, offset: plane.offset });
                    }
                    (_, sm_model_api::DeckSide::A, Some(m)) if region.faces.binary_search(&m).is_ok() => {
                        let c = CopyConstraint { source: Target::Surface(f), target: Target::Surface(m), transform: deck.transform(), vertex_map: Vec::new(), reason: CopyReason::Deck { boundary } };
                        p.push_copy(origin, c);
                    }
                    _ => {}
                }
            }
        }

        // Volumes, then groups (top-down: their requirements constrain faces and curves).
        for &v in &region.volumes {
            if region.grouped.contains_key(&v) {
                continue;
            }
            let t = Target::Volume(v);
            let tp = self.resolve(&cx, &p, t, rej)?;
            self.require(&cx, &mut p, &tp)?;
            plans.insert(t, tp);
        }
        for (id, members) in &region.groups {
            let t = Target::Group(*id);
            let mt: Vec<Target> = members.iter().map(|m| Target::Volume(*m)).collect();
            p.push_constraint(Origin::Intent, MeshConstraint::Atomic { group: *id, members: mt });
            let tp = self.resolve(&cx, &p, t, rej)?;
            self.require(&cx, &mut p, &tp)?;
            plans.insert(t, tp);
        }

        // User copies on faces and curves.
        for (id, ei) in &intent.entities {
            let Some(ch) = ei.scheme.as_ref().filter(|c| c.name.as_str() == SchemeName::COPY) else { continue };
            let t = match topo.dim_of(*id) {
                Some(Dim::Face) if region.faces.binary_search(id).is_ok() => Target::Surface(*id),
                Some(Dim::Edge) if region.edges.binary_search(id).is_ok() => Target::Curve(*id),
                _ => continue,
            };
            let source: EntityId = ch.params.get(SchemeName::COPY, "source")?.ok_or_else(|| SchemeError::BadParams { scheme: ch.name.clone(), reason: "missing `source`".into() })?;
            let transform: Transform = ch.params.get(SchemeName::COPY, "transform")?.unwrap_or(Transform::IDENTITY);
            let vertex_map: Vec<(EntityId, EntityId)> = sorted(ch.params.get(SchemeName::COPY, "vertex_map")?.unwrap_or_default());
            let st = match t {
                Target::Surface(_) => Target::Surface(source),
                _ => Target::Curve(source),
            };
            p.push_copy(Origin::Intent, CopyConstraint { source: st, target: t, transform, vertex_map, reason: CopyReason::User });
        }
        self.copy_closure(input, &region, &mut p)?;

        // Faces, then curves.
        for &f in &region.faces {
            let t = Target::Surface(f);
            let tp = match p.copy_into(t) {
                Some(rec) => copy_plan(intent, t, rec)?,
                None => {
                    let tp = self.resolve(&cx, &p, t, rej)?;
                    self.require(&cx, &mut p, &tp)?;
                    tp
                }
            };
            plans.insert(t, tp);
        }
        for &e in &region.edges {
            let t = Target::Curve(e);
            let tp = match p.copy_into(t) {
                Some(rec) => copy_plan(intent, t, rec)?,
                None => {
                    let tp = self.resolve(&cx, &p, t, rej)?;
                    self.require(&cx, &mut p, &tp)?;
                    tp
                }
            };
            plans.insert(t, tp);
        }

        // Corner requests from intent, then corner picking per face with a rule.
        for c in &intent.corners {
            if region.faces.binary_search(&c.corner.face).is_ok() {
                p.push_vertex_type(Origin::Intent, c.corner, c.vertex_type);
            }
        }
        let layouts = self.pick_corners(input, &p)?;

        // Intervals: intent and sizing goals, then scheme equations.
        self.sizing_constraints(input, intent, &region, &mut p)?;
        let roles: Vec<(Target, Role)> = p.roles.iter().map(|r| r.item).collect();
        let icx = IntervalCtx { input, layouts: &layouts, roles: &roles };
        for tp in plans.values() {
            if tp.scheme.as_str() == SchemeName::COPY {
                continue;
            }
            let s = self.registry.get(tp.target.dim(), tp.scheme.as_str()).ok_or_else(|| SchemeError::UnknownScheme { name: tp.scheme.clone(), target: tp.target })?;
            s.interval_constraints(&icx, tp.target, &tp.params, &mut IntervalSink::new(&mut p, &tp.scheme, tp.target))?;
        }
        let intervals = self.solver.solve(&region.edges, &p.intervals)?;

        p.targets = plans.into_values().collect();
        let waves = waves(&p)?;
        Ok(Plan { problem: p, intervals, layouts, waves })
    }

    fn require(&self, cx: &PlanCtx<'_>, p: &mut MeshingProblem, tp: &TargetPlan) -> Result<()> {
        let s = self.registry.get(tp.target.dim(), tp.scheme.as_str()).ok_or_else(|| SchemeError::UnknownScheme { name: tp.scheme.clone(), target: tp.target })?;
        s.requirements(cx, tp.target, &tp.params, &mut Requirements::new(p, &tp.scheme, tp.target))
    }

    /// Choose the scheme of `target`: intent chain (scheme, fallbacks), else the piece shape
    /// hint, else the best applicable candidate among the imposed set or the auto policy.
    fn resolve(&self, cx: &PlanCtx<'_>, p: &MeshingProblem, target: Target, rej: &Rejections) -> Result<TargetPlan> {
        let dim = target.dim();
        // Impositions: intersect every allowed set.
        let mut allowed: Option<(Vec<SchemeName>, Target)> = None;
        for r in p.impositions.iter().filter(|r| r.item.child == target) {
            allowed = Some(match allowed {
                None => (r.item.allowed.clone(), r.item.by),
                Some((a, by)) => {
                    let both: Vec<SchemeName> = a.iter().filter(|n| r.item.allowed.contains(n)).cloned().collect();
                    if both.is_empty() {
                        return Err(SchemeError::ImpositionConflict { child: target, imposed_by: r.item.by, required: r.item.allowed.clone(), conflict: format!("{by:?} requires one of {a:?}") });
                    }
                    (both, by)
                }
            });
        }
        // Explicit chain.
        let mut chain: Vec<(SchemeChoice, Selection)> = Vec::new();
        let ent = match target {
            Target::Group(g) => cx.intent.groups.iter().find(|x| x.id == g).and_then(|x| x.scheme.clone()).map(|s| (s, Vec::new())),
            _ => target.entity().and_then(|e| cx.intent.entities.get(&e)).and_then(|ei| ei.scheme.clone().map(|s| (s, ei.fallbacks.clone()))),
        };
        if let Some((s, fb)) = ent {
            chain.push((s, Selection::Intent { rank: 0 }));
            for (i, f) in fb.into_iter().enumerate() {
                chain.push((f, Selection::Intent { rank: i as u32 + 1 }));
            }
        } else if let Some(PieceRef::Cell(c)) = target.piece()
            && let Some(piece) = cx.input.model.decomposition().piece(c)
        {
            match &piece.shape {
                PieceShape::Sweep { source, target: tg } => {
                    let map = |v: &[sm_model_api::PieceFace]| -> Vec<EntityId> { v.iter().filter_map(|pf| cx.input.pieces.face_of(cx.input.topo, cx.input.model, pf)).collect() };
                    let params = serde_json::json!({ "source": map(source), "target": map(tg) });
                    chain.push((SchemeChoice::with_params("sweep", params, Firmness::Default), Selection::PieceShape));
                }
                PieceShape::Map => chain.push((SchemeChoice::new("map", Firmness::Default), Selection::PieceShape)),
                PieceShape::Any => {}
            }
        }
        let mut rejected: Vec<(SchemeName, String)> = Vec::new();
        let any_hard = chain.iter().any(|c| c.0.firmness == Firmness::Hard);
        for (choice, sel) in chain {
            if let Some((a, by)) = &allowed
                && !a.contains(&choice.name)
            {
                if choice.firmness == Firmness::Hard {
                    return Err(SchemeError::ImpositionConflict { child: target, imposed_by: *by, required: a.clone(), conflict: format!("the intent requires `{}` (hard)", choice.name) });
                }
                rejected.push((choice.name.clone(), format!("overridden by imposition from {by:?}")));
                continue;
            }
            if let Some(why) = rej.0.get(&(target, choice.name.clone())) {
                rejected.push((choice.name.clone(), format!("rejected after failure: {why}")));
                continue;
            }
            let s = self.registry.get(dim, choice.name.as_str()).ok_or_else(|| SchemeError::UnknownScheme { name: choice.name.clone(), target })?;
            if matches!(target, Target::Group(_)) && !s.supports_groups() {
                rejected.push((choice.name.clone(), "does not mesh groups".into()));
                continue;
            }
            match s.applicability(cx, target, &choice.params) {
                Applicability::Applicable { .. } => return Ok(TargetPlan { target, scheme: choice.name, params: choice.params, selection: sel, rejected }),
                Applicability::NotApplicable { reason } => rejected.push((choice.name, reason)),
            }
        }
        if any_hard {
            return Err(SchemeError::NoApplicableScheme { target, tried: rejected });
        }
        // Imposed set or auto policy.
        let policy = match dim {
            SchemeDim::Curve => &cx.intent.defaults.auto.curve,
            SchemeDim::Surface => &cx.intent.defaults.auto.surface,
            SchemeDim::Volume => &cx.intent.defaults.auto.volume,
        };
        let names: Vec<SchemeName> = if policy.is_empty() { self.registry.of_dim(dim).map(|s| SchemeName::new(s.name())).collect() } else { policy.clone() };
        let names: Vec<SchemeName> = match &allowed {
            Some((a, _)) => {
                let mut v: Vec<SchemeName> = names.iter().filter(|n| a.contains(n)).cloned().collect();
                v.extend(a.iter().filter(|n| !names.contains(n)).cloned());
                v
            }
            None => names,
        };
        let mut best: Option<(u32, SchemeName)> = None;
        for n in names {
            if rej.0.contains_key(&(target, n.clone())) {
                rejected.push((n, "rejected after failure".into()));
                continue;
            }
            let Some(s) = self.registry.get(dim, n.as_str()) else { continue };
            if matches!(target, Target::Group(_)) && !s.supports_groups() {
                continue;
            }
            match s.applicability(cx, target, &Params::empty()) {
                Applicability::Applicable { score, .. } => {
                    if best.as_ref().is_none_or(|b| score > b.0) {
                        best = Some((score, n));
                    }
                }
                Applicability::NotApplicable { reason } => rejected.push((n, reason)),
            }
        }
        match best {
            Some((score, name)) => {
                let selection = if allowed.is_some() { Selection::Imposed } else { Selection::Auto { score } };
                Ok(TargetPlan { target, scheme: name, params: Params::empty(), selection, rejected })
            }
            None => Err(SchemeError::NoApplicableScheme { target, tried: rejected }),
        }
    }

    /// Fill vertex maps of surface copies, add the implied curve copies of their edges, check
    /// that no target is copied twice and add the interval equalities of curve copies.
    fn copy_closure(&self, input: &MeshInput<'_>, region: &Region, p: &mut MeshingProblem) -> Result<()> {
        let topo = input.topo;
        for i in 0..p.copies.len() {
            let c = &p.copies[i].item;
            let inside = |t: Target| match t {
                Target::Surface(f) => region.faces.binary_search(&f).is_ok(),
                Target::Curve(e) => region.edges.binary_search(&e).is_ok(),
                _ => false,
            };
            if !inside(c.source) || !inside(c.target) || c.source == c.target {
                return Err(SchemeError::Correspondence { source_target: c.source, target: c.target, reason: "source and target must be distinct entities of the meshed region".into() });
            }
            if c.vertex_map.is_empty() {
                let vm = copy::derive_vertex_map(topo, c)?;
                p.copies[i].item.vertex_map = vm;
            }
        }
        let n0 = p.copies.len();
        for i in 0..n0 {
            if !matches!(p.copies[i].item.target, Target::Surface(_)) {
                continue;
            }
            let parent = p.copies[i].id;
            let c = p.copies[i].item.clone();
            let vm: BTreeMap<EntityId, EntityId> = c.vertex_map.iter().copied().collect();
            for (e, f, _) in copy::edge_map(topo, &c)? {
                if e == f || p.copy_into(Target::Curve(f)).is_some() {
                    continue;
                }
                let Some((a, b)) = topo.edge_ends(e) else { continue };
                let mut evm: Vec<(EntityId, EntityId)> = [a, b].iter().filter_map(|v| vm.get(v).map(|w| (*v, *w))).collect();
                evm.sort_unstable();
                evm.dedup();
                p.push_copy(Origin::Driver, CopyConstraint { source: Target::Curve(e), target: Target::Curve(f), transform: c.transform, vertex_map: evm, reason: CopyReason::Closure { parent } });
            }
        }
        let mut seen: BTreeMap<Target, ConstraintId> = BTreeMap::new();
        for r in &p.copies {
            if let Some(first) = seen.insert(r.item.target, r.id) {
                return Err(SchemeError::CopyConflict { target: r.item.target, first, second: r.id });
            }
        }
        let eqs: Vec<(EntityId, EntityId)> = p
            .copies
            .iter()
            .filter_map(|r| match (r.item.source, r.item.target) {
                (Target::Curve(a), Target::Curve(b)) => Some((a, b)),
                _ => None,
            })
            .collect();
        for (a, b) in eqs {
            p.push_interval(Origin::Driver, IntervalConstraint::SumEqual { lhs: vec![a], rhs: vec![b] });
        }
        Ok(())
    }

    fn pick_corners(&self, input: &MeshInput<'_>, p: &MeshingProblem) -> Result<Vec<FaceLayout>> {
        let mut rules: BTreeMap<EntityId, (VertexRule, Vec<ConstraintId>)> = BTreeMap::new();
        for r in &p.vertex_rules {
            let (face, rule) = r.item;
            match rules.get_mut(&face) {
                None => {
                    rules.insert(face, (rule, vec![r.id]));
                }
                Some((have, ids)) => {
                    ids.push(r.id);
                    if *have != rule {
                        return Err(Infeasible { reason: format!("conflicting corner rules {have:?} and {rule:?}"), constraints: ids.clone(), entities: vec![face] }.into());
                    }
                }
            }
        }
        let mut out = Vec::with_capacity(rules.len());
        for (face, (rule, mut ids)) in rules {
            let reqs: Vec<(CornerKey, Firm<VertexType>)> = p.vertex_types.iter().filter(|r| r.item.0.face == face).map(|r| r.item).collect();
            ids.extend(p.vertex_types.iter().filter(|r| r.item.0.face == face).map(|r| r.id));
            out.push(self.picker.pick(input.topo, face, rule, &reqs, &ids)?);
        }
        Ok(out)
    }

    fn sizing_constraints(&self, input: &MeshInput<'_>, intent: &MeshIntent, region: &Region, p: &mut MeshingProblem) -> Result<()> {
        let topo = input.topo;
        // Requests cascade volume -> face -> edge; the lowest id wins within a level.
        let mut req: BTreeMap<EntityId, Firm<IntervalRequest>> = BTreeMap::new();
        for v in region.volumes.iter().rev() {
            if let Some(r) = intent.entities.get(v).and_then(|e| e.intervals) {
                for f in input.piece_faces(Target::Volume(*v))? {
                    for e in input.face_edges(f) {
                        req.insert(e, r);
                    }
                }
            }
        }
        for f in region.faces.iter().rev() {
            if let Some(r) = intent.entities.get(f).and_then(|e| e.intervals) {
                for e in input.face_edges(*f) {
                    req.insert(e, r);
                }
            }
        }
        for e in &region.edges {
            if let Some(r) = intent.entities.get(e).and_then(|x| x.intervals) {
                req.insert(*e, r);
            }
        }
        let owned = if self.size.is_none() { intent.defaults.sizing.as_ref().and_then(builtin_field) } else { None };
        let field: Option<&dyn SizeField> = self.size.or(owned.as_deref());
        for &e in &region.edges {
            let c = match req.get(&e) {
                Some(Firm { value: IntervalRequest::Count(n), firmness }) => (Origin::Intent, IntervalConstraint::Fixed { edge: e, count: *n, firmness: *firmness }),
                Some(Firm { value: IntervalRequest::Size(h), .. }) => (Origin::Intent, IntervalConstraint::Goal { edge: e, goal: geom::edge_length(topo, e)? / h }),
                None => match field {
                    Some(f) => {
                        let mid = geom::edge_point(topo, e, 0.5)?;
                        (Origin::Sizing, IntervalConstraint::Goal { edge: e, goal: geom::edge_length(topo, e)? / f.size(mid) })
                    }
                    None => (Origin::Sizing, IntervalConstraint::Goal { edge: e, goal: 1.0 }),
                },
            };
            p.push_interval(c.0, c.1);
        }
        Ok(())
    }

    /// Generate every patch of the plan, wave by wave. Inside a wave the executor may run
    /// targets concurrently; results are merged in target order, so the output is identical
    /// for every executor.
    pub fn generate<E: Executor>(&self, input: &MeshInput<'_>, intent: &MeshIntent, plan: &Plan, exec: &E) -> Result<PatchSet> {
        let owned = if self.size.is_none() { intent.defaults.sizing.as_ref().and_then(builtin_field) } else { None };
        let field: Option<&dyn SizeField> = self.size.or(owned.as_deref());
        let mut ps = PatchSet::default();
        for &v in &plan.problem.vertices {
            ps.nodes.insert(NodeKey::vertex(v), (geom::vertex_point(input.topo, v)?, Param::None));
        }
        for wave in &plan.waves {
            let results = exec.map(wave.len(), &|i| self.generate_one(input, intent, plan, &ps, field, wave[i]));
            for (t, r) in wave.iter().zip(results) {
                let patch = r?;
                check_patch(input, plan, &ps, *t, &patch)?;
                for n in &patch.nodes {
                    ps.nodes.insert(n.key, (n.point, n.param));
                }
                ps.patches.insert(*t, patch);
            }
        }
        Ok(ps)
    }

    fn generate_one(&self, input: &MeshInput<'_>, intent: &MeshIntent, plan: &Plan, ps: &PatchSet, size: Option<&dyn SizeField>, t: Target) -> Result<Patch> {
        let tp = plan.target(t).ok_or_else(|| SchemeError::Input(format!("{t:?} is not in the plan")))?;
        if tp.scheme.as_str() == SchemeName::COPY {
            let rec = plan.copy_into(t).ok_or_else(|| SchemeError::Input(format!("{t:?}: copy without constraint")))?;
            return copy::copy_patch(input.topo, ps, rec, &|e| plan.intervals(e));
        }
        let s = self.registry.get(t.dim(), tp.scheme.as_str()).ok_or_else(|| SchemeError::UnknownScheme { name: tp.scheme.clone(), target: t })?;
        let cx = GenCtx { input, plan, patches: ps, size };
        let mut patch = s.generate(&cx, t, &tp.params)?;
        if let Target::Group(g) = t
            && let Some(conv) = intent.groups.iter().find(|x| x.id == g).and_then(|x| x.conversion.as_ref())
        {
            let tr = self.registry.transform(conv.name.as_str()).ok_or_else(|| SchemeError::UnknownScheme { name: conv.name.clone(), target: t })?;
            tr.apply(&cx, t, &conv.params, &mut patch)?;
        }
        Ok(patch)
    }

    /// Write every patch into `b` in a fixed order (nodes by key, then cells by target, then
    /// surface classifications) and finish. All-or-nothing: on error nothing was finished
    /// and `alloc` is unchanged (`MeshBuild::finish` contract).
    pub fn commit<B: MeshBuild>(&self, input: &MeshInput<'_>, plan: &Plan, ps: &PatchSet, mut b: B, alloc: &mut SerialAlloc) -> Result<Finished<B::Output>> {
        let classify = |owner: EntityId| input.pieces.classify(owner).ok_or(SchemeError::Unclassifiable { owner });
        let mut verts: BTreeMap<NodeKey, VertexIx> = BTreeMap::new();
        for (k, pt, param) in ps.nodes() {
            let v = b.add_vertex(pt)?;
            let ent = classify(k.owner)?;
            let param = if ent == k.owner && param.fits(ent.kind()) { param } else { Param::None };
            b.set_class(MeshEntityRef::Vertex(v), Classification { entity: ent, param })?;
            verts.insert(k, v);
        }
        let mut vs: Vec<VertexIx> = Vec::with_capacity(8);
        let lookup = |verts: &BTreeMap<NodeKey, VertexIx>, k: NodeKey, t: Target| verts.get(&k).copied().ok_or_else(|| SchemeError::InvalidPatch { target: t, reason: format!("node {k:?} missing") });
        for patch in ps.patches().filter(|p| p.target.dim() == SchemeDim::Volume) {
            let t = patch.target;
            for (i, (shape, nodes)) in patch.elements.iter().enumerate() {
                let ElemShape::Cell(cs) = shape else {
                    return Err(SchemeError::InvalidPatch { target: t, reason: format!("volume patch element {shape:?}") });
                };
                if cs == CellShape::Polyhedron {
                    return Err(SchemeError::InvalidPatch { target: t, reason: "polyhedral cells need face lists (not in the patch contract yet)".into() });
                }
                vs.clear();
                for k in nodes {
                    vs.push(lookup(&verts, *k, t)?);
                }
                let added = b.add_cell_from_vertices(cs, &vs)?;
                let owner = match (t.entity(), patch.owners.get(i)) {
                    (_, Some(o)) => *o,
                    (Some(e), None) => e,
                    (None, None) => return Err(SchemeError::InvalidPatch { target: t, reason: "group cell without owner".into() }),
                };
                b.set_class(MeshEntityRef::Cell(added.cell), Classification::on(classify(owner)?))?;
            }
        }
        for patch in ps.patches().filter(|p| p.target.dim() == SchemeDim::Surface) {
            let t = patch.target;
            let Some(face) = t.entity() else { continue };
            let class = Classification::on(classify(face)?);
            for (_, nodes) in patch.elements.iter() {
                vs.clear();
                for k in nodes {
                    vs.push(lookup(&verts, *k, t)?);
                }
                let (f, _) = b.find_face(&vs).ok_or_else(|| SchemeError::InvalidPatch { target: t, reason: "surface element bounds no cell (surface-only meshes are not representable in sm-mesh-api)".into() })?;
                b.set_class(MeshEntityRef::Face(f), class)?;
            }
        }
        if self.options.classify_edges {
            for patch in ps.patches().filter(|p| p.target.dim() == SchemeDim::Curve) {
                let t = patch.target;
                let Some(edge) = t.entity() else { continue };
                let class = Classification::on(classify(edge)?);
                for (_, nodes) in patch.elements.iter() {
                    if let [a, c] = nodes {
                        b.set_class(MeshEntityRef::Edge(lookup(&verts, *a, t)?, lookup(&verts, *c, t)?), class)?;
                    }
                }
            }
        }
        let _ = plan;
        Ok(b.finish(alloc)?)
    }

    /// Plan, generate and commit, falling back on fallback-eligible generation failures:
    /// the failing `(target, scheme)` is rejected and the whole problem is re-planned (a
    /// different scheme changes the constraints), up to `options.max_replans` times.
    pub fn mesh<B: MeshBuild, E: Executor>(&self, input: &MeshInput<'_>, intent: &MeshIntent, exec: &E, builder: B, alloc: &mut SerialAlloc) -> Result<Meshed<B::Output>> {
        let mut rej = Rejections::default();
        for _ in 0..=self.options.max_replans {
            let plan = self.plan_with(input, intent, &rej)?;
            match self.generate(input, intent, &plan, exec) {
                Ok(patches) => {
                    let finished = self.commit(input, &plan, &patches, builder, alloc)?;
                    return Ok(Meshed { plan, patches, finished, rejections: rej });
                }
                Err(e) => match e.as_fallback() {
                    Some(g) if g.scheme.as_str() != SchemeName::COPY => {
                        rej.0.insert((g.target, g.scheme.clone()), g.reason.clone());
                    }
                    _ => return Err(e),
                },
            }
        }
        Err(SchemeError::ReplanLimit(self.options.max_replans))
    }
}

/// The plan of a copy target. A hard intent for another scheme on it is an
/// `ImpositionConflict` (the copy source imposes `copy`); a soft one is recorded as overridden.
fn copy_plan(intent: &MeshIntent, t: Target, rec: &crate::problem::Record<CopyConstraint>) -> Result<TargetPlan> {
    let mut rejected = Vec::new();
    if let Some(ch) = t.entity().and_then(|e| intent.entities.get(&e)).and_then(|ei| ei.scheme.as_ref()).filter(|c| c.name.as_str() != SchemeName::COPY) {
        if ch.firmness == Firmness::Hard {
            return Err(SchemeError::ImpositionConflict { child: t, imposed_by: rec.item.source, required: vec![SchemeName::new(SchemeName::COPY)], conflict: format!("the intent requires `{}` (hard)", ch.name) });
        }
        rejected.push((ch.name.clone(), format!("overridden by copy constraint {:?}", rec.id)));
    }
    Ok(TargetPlan { target: t, scheme: SchemeName::new(SchemeName::COPY), params: Params::empty(), selection: Selection::Copy { constraint: rec.id }, rejected })
}

/// Group targets into waves: by dimension (groups after volumes), then by dependency depth
/// within a dimension (copy source before copy target, `Precedes`). Sorted inside a wave.
fn waves(p: &MeshingProblem) -> Result<Vec<Vec<Target>>> {
    let mut deps: BTreeMap<Target, Vec<Target>> = BTreeMap::new();
    for r in &p.copies {
        deps.entry(r.item.target).or_default().push(r.item.source);
    }
    for r in &p.constraints {
        if let MeshConstraint::Precedes { before, after } = r.item {
            deps.entry(after).or_default().push(before);
        }
    }
    let rank = |t: &Target| (t.dim().rank(), matches!(t, Target::Group(_)));
    let mut level: BTreeMap<Target, u32> = BTreeMap::new();
    fn depth(t: Target, deps: &BTreeMap<Target, Vec<Target>>, level: &mut BTreeMap<Target, u32>, stack: &mut Vec<Target>, rank: &dyn Fn(&Target) -> (u8, bool)) -> Result<u32> {
        if let Some(l) = level.get(&t) {
            return Ok(*l);
        }
        if stack.contains(&t) {
            return Err(SchemeError::Input(format!("dependency cycle through {t:?}")));
        }
        stack.push(t);
        let mut l = 0;
        for d in deps.get(&t).map(Vec::as_slice).unwrap_or(&[]) {
            if rank(d) > rank(&t) {
                return Err(SchemeError::Input(format!("{t:?} depends on higher-dimensional {d:?}")));
            }
            if rank(d) == rank(&t) {
                l = l.max(depth(*d, deps, level, stack, rank)? + 1);
            }
        }
        stack.pop();
        level.insert(t, l);
        Ok(l)
    }
    let mut buckets: BTreeMap<(u8, bool, u32), Vec<Target>> = BTreeMap::new();
    for tp in &p.targets {
        let l = depth(tp.target, &deps, &mut level, &mut Vec::new(), &rank)?;
        let (d, g) = rank(&tp.target);
        buckets.entry((d, g, l)).or_default().push(tp.target);
    }
    Ok(buckets.into_values().collect())
}

/// The patch contract: right target, nodes owned by the target (curves: exactly the indices
/// `1..n`), unique keys, every referenced node known.
fn check_patch(input: &MeshInput<'_>, plan: &Plan, ps: &PatchSet, t: Target, patch: &Patch) -> Result<()> {
    let bad = |reason: String| SchemeError::InvalidPatch { target: t, reason };
    if patch.target != t {
        return Err(bad(format!("patch is for {:?}", patch.target)));
    }
    let owners: Vec<EntityId> = match t {
        Target::Group(g) => plan
            .problem
            .constraints
            .iter()
            .find_map(|r| match &r.item {
                MeshConstraint::Atomic { group, members } if *group == g => Some(members.iter().filter_map(Target::entity).collect()),
                _ => None,
            })
            .unwrap_or_default(),
        _ => t.entity().into_iter().collect(),
    };
    let mut keys: BTreeSet<NodeKey> = BTreeSet::new();
    for n in &patch.nodes {
        if !owners.contains(&n.key.owner) {
            return Err(bad(format!("node {:?} is not owned by the target", n.key)));
        }
        if !keys.insert(n.key) || ps.nodes.contains_key(&n.key) {
            return Err(bad(format!("duplicate node {:?}", n.key)));
        }
        if !n.param.fits(n.key.owner.kind()) {
            return Err(bad(format!("node {:?}: parameter does not fit its owner", n.key)));
        }
    }
    if let Target::Curve(e) = t {
        let n = plan.intervals(e).ok_or_else(|| bad("no interval count".into()))?;
        let want: BTreeSet<NodeKey> = (1..n).map(|i| NodeKey::new(e, i)).collect();
        if keys != want {
            return Err(bad(format!("a curve with {n} intervals needs nodes 1..{n}")));
        }
    }
    for k in patch.elements.node_refs() {
        if !keys.contains(k) && !ps.nodes.contains_key(k) {
            return Err(bad(format!("element references unknown node {k:?}")));
        }
    }
    if !patch.owners.is_empty() && patch.owners.len() != patch.elements.len() {
        return Err(bad("owners must be empty or one per element".into()));
    }
    let _ = input;
    Ok(())
}
