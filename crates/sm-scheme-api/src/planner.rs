//! Planning: assemble the [`MeshingProblem`] from intent, model and schemes, resolve schemes
//! (claims, copy classes, intent chain, registry fallbacks, piece hints, impositions, auto
//! scoring), solve corners and intervals, and schedule the jobs.
//!
//! Planning order (see `DESIGN.md`):
//! 1. region, deck constraints, boundary-layer requests;
//! 2. volumes, then groups (their requirements constrain faces and curves);
//! 3. user copies, copy closure and classes;
//! 4. faces (closure and classes again after each requirement that adds copies);
//! 5. curves (likewise);
//! 6. final classes, copy-member caps check;
//! 7. corners, sizing and scheme intervals, solve;
//! 8. footprints and waves.

use crate::caps::Caps;
use crate::classes::{ClassInput, build_classes, fill_vertex_maps};
use crate::copy;
use crate::driver::{Driver, Rejections};
use crate::error::{Attempt, FailureKind, Infeasible, Phase, Result, SchemeError};
use crate::firm::{Firm, Firmness};
use crate::geom;
use crate::intent::{IntervalRequest, MeshIntent, Params, SchemeChoice, SchemeName};
use crate::plan::{CopyClass, CopyMode, Plan, TransformPlan};
use crate::problem::{
    ConstraintId, CopyConstraint, CopyReason, Imposition, IntervalConstraint, IntervalSink,
    MeshConstraint, MeshingProblem, Need, Origin, Requirements, Role, Scope, Selection, TargetPlan,
};
use crate::schedule::{ScheduleInput, schedule};
use crate::scheme::{Applicability, IntervalCtx, MeshInput, PlanCtx};
use crate::sizing::{SizeField, field_for};
use crate::target::{GroupId, Job, SchemeDim, Target, TransformId, piece_of};
use crate::vertex::{FaceLayout, VertexRule, VertexType};
use sm_core::{EntityId, EntityKind, Transform};
use sm_model_api::{Deck, FaceKindView, PieceRef, PieceShape};
use sm_topo_api::{CornerKey, Dim, TopologyViewExt};
use std::cmp::Reverse;
use std::collections::{BTreeMap, BTreeSet};

pub(crate) struct Region {
    pub volumes: Vec<EntityId>,
    pub groups: Vec<(GroupId, Vec<EntityId>)>,
    pub grouped: BTreeMap<EntityId, GroupId>,
    pub vol_faces: BTreeMap<EntityId, Vec<EntityId>>,
    pub faces: Vec<EntityId>,
    pub edges: Vec<EntityId>,
    pub vertices: Vec<EntityId>,
}

impl Region {
    /// Is `t` a target of the region (sorted tables, binary search)?
    pub(crate) fn contains(&self, t: Target) -> bool {
        match t {
            Target::Curve(e) => self.edges.binary_search(&e).is_ok(),
            Target::Surface(f) => self.faces.binary_search(&f).is_ok(),
            Target::Volume(v) => self.volumes.binary_search(&v).is_ok(),
            Target::Group(g) => self.groups.iter().any(|(id, _)| *id == g),
        }
    }
}

/// The strict sub-entity targets of `t` inside the region: edges (boundary and embedded) of a
/// face; faces, their edges and embedded edges of a piece or of every member of a group.
fn children_in(input: &MeshInput<'_>, region: &Region, t: Target) -> Result<BTreeSet<Target>> {
    let mut out = BTreeSet::new();
    // Boundary edges and embedded (wire) edges of a face are its curve children.
    let add_face = |f: EntityId, out: &mut BTreeSet<Target>| {
        out.insert(Target::Surface(f));
        for e in input.face_edges(f).into_iter().chain(input.embedded_edges(f)) {
            out.insert(Target::Curve(e));
        }
    };
    let add_embedded = |v: EntityId, out: &mut BTreeSet<Target>| {
        for e in input.embedded_edges(v) {
            out.insert(Target::Curve(e));
        }
    };
    let add_piece = |v: EntityId, out: &mut BTreeSet<Target>| {
        add_embedded(v, out);
        for f in region.vol_faces.get(&v).map(Vec::as_slice).unwrap_or(&[]) {
            add_face(*f, out);
        }
    };
    match t {
        Target::Curve(_) => {}
        Target::Surface(f) => {
            for e in input.face_edges(f).into_iter().chain(input.embedded_edges(f)) {
                out.insert(Target::Curve(e));
            }
        }
        Target::Volume(v) => add_piece(v, &mut out),
        Target::Group(g) => {
            for (id, members) in &region.groups {
                if *id == g {
                    for m in members {
                        add_piece(*m, &mut out);
                    }
                }
            }
        }
    }
    Ok(out)
}

fn sorted<T: Ord>(mut v: Vec<T>) -> Vec<T> {
    v.sort();
    v.dedup();
    v
}

/// A requirement a candidate scheme must satisfy.
#[derive(Clone, Debug)]
struct NeedRef {
    child: Target,
    need: Need,
    firmness: Firmness,
    by: Target,
    id: ConstraintId,
    from_intent: bool,
}

fn violates(n: &Need, name: &SchemeName, caps: Caps) -> bool {
    match n {
        Need::Schemes(names) => !names.contains(name),
        Need::Caps(c) => !caps.contains(*c),
        Need::Forbid(c) => caps.intersects(*c),
    }
}

/// Caps a claimed child shows to impositions: what its claimer produces on surfaces.
fn child_caps(claimer: Caps) -> Caps {
    claimer & (Caps::TRI | Caps::QUAD | Caps::STRUCTURED | Caps::BLOCK_STRUCTURED | Caps::POLY)
}

enum Cand {
    Chain(SchemeChoice, Selection),
}

impl<'r> Driver<'r> {
    pub(crate) fn region(&self, input: &MeshInput<'_>, intent: &MeshIntent) -> Result<Region> {
        let selected = sorted(input.selection.to_vec());
        let is_face = |id: &EntityId| {
            matches!(id.kind(), EntityKind::TopoFace | EntityKind::PartitionFace)
        };
        // Surface-only selection: faces of the view, no pieces.
        let surface_only = !selected.is_empty() && selected.iter().all(is_face);
        let (volumes, picked_faces) = if surface_only {
            for f in &selected {
                if input.topo.dim_of(*f) != Some(Dim::Face) || !input.topo.is_alive(*f) {
                    return Err(SchemeError::Input(format!("{f} is not a live face of the view")));
                }
            }
            (Vec::new(), selected)
        } else {
            if let Some(v) = selected.iter().find(|v| piece_of(**v).is_none()) {
                return Err(SchemeError::Input(format!(
                    "{v} is not a TopoVolume or PartitionCell (a selection is all pieces or all faces)"
                )));
            }
            (selected, Vec::new())
        };
        let mut grouped = BTreeMap::new();
        let mut groups = Vec::new();
        for g in &intent.groups {
            if g.id.0 >= TransformId::GROUP_BASE {
                return Err(SchemeError::Input(format!(
                    "group id {:?} is reserved",
                    g.id
                )));
            }
            let members = sorted(g.members.clone());
            for m in &members {
                if volumes.binary_search(m).is_err() {
                    return Err(SchemeError::Input(format!(
                        "group {:?}: member {m} is not selected",
                        g.id
                    )));
                }
                if grouped.insert(*m, g.id).is_some() {
                    return Err(SchemeError::Input(format!("{m} is in two groups")));
                }
            }
            groups.push((g.id, members));
        }
        groups.sort();
        let mut faces: BTreeSet<EntityId> = picked_faces.into_iter().collect();
        let mut vol_faces = BTreeMap::new();
        for v in &volumes {
            let f = input.piece_faces(Target::Volume(*v))?;
            faces.extend(f.iter().copied());
            vol_faces.insert(*v, f);
        }
        let faces: Vec<EntityId> = faces.into_iter().collect();
        // Embedded members (W0f gaps 2): the wire edges and points of every selected face and
        // piece get curve targets and vertex nodes, so the parent's mesh can conform to them.
        let parents = || faces.iter().chain(volumes.iter());
        let edges = sorted(
            faces
                .iter()
                .flat_map(|f| input.face_edges(*f))
                .chain(parents().flat_map(|p| input.embedded_edges(*p)))
                .collect(),
        );
        let vertices = sorted(
            edges
                .iter()
                .filter_map(|e| input.topo.edge_ends(*e))
                .flat_map(|(a, b)| [a, b])
                .chain(parents().flat_map(|p| input.embedded_vertices(*p)))
                .collect(),
        );
        Ok(Region {
            volumes,
            groups,
            grouped,
            vol_faces,
            faces,
            edges,
            vertices,
        })
    }

    /// Assemble and solve the problem.
    pub fn plan(&self, input: &MeshInput<'_>, intent: &MeshIntent) -> Result<Plan> {
        self.plan_with(input, intent, &Rejections::default())
    }

    /// Assemble and solve the problem, skipping rejected `(target, scheme)` pairs. The same
    /// `(intent, rejections)` always gives the same plan.
    pub fn plan_with(
        &self,
        input: &MeshInput<'_>,
        intent: &MeshIntent,
        rej: &Rejections,
    ) -> Result<Plan> {
        intent.check()?;
        let region = self.region(input, intent)?;
        Planner::new(self, input, intent, rej, region).run()
    }
}

struct Planner<'a, 'r> {
    drv: &'a Driver<'r>,
    input: &'a MeshInput<'a>,
    intent: &'a MeshIntent,
    rej: &'a Rejections,
    cx: PlanCtx<'a>,
    region: Region,
    p: MeshingProblem,
    plans: BTreeMap<Target, TargetPlan>,
    /// Targets whose chosen scheme is a hard intent.
    hard: BTreeSet<Target>,
    /// Targets whose intent is a hard non-copy scheme (known before they are planned).
    intent_hard: BTreeSet<Target>,
    copy_intent: BTreeSet<Target>,
    claimed: BTreeMap<Target, Target>,
    attempts: Vec<Attempt>,
    classes: Vec<CopyClass>,
    class_ix: BTreeMap<Target, (usize, Option<usize>)>,
    closed: BTreeSet<ConstraintId>,
    checked: usize,
    tol: f64,
    /// Impositions by constrained child (append-only, synced before each resolve).
    imp_ix: BTreeMap<Target, Vec<usize>>,
    imp_seen: usize,
}

impl<'a, 'r> Planner<'a, 'r> {
    fn new(
        drv: &'a Driver<'r>,
        input: &'a MeshInput<'a>,
        intent: &'a MeshIntent,
        rej: &'a Rejections,
        region: Region,
    ) -> Self {
        let mut intent_hard = BTreeSet::new();
        let mut copy_intent = BTreeSet::new();
        for (id, ei) in &intent.entities {
            let t = match id.kind() {
                EntityKind::TopoEdge => Target::Curve(*id),
                EntityKind::TopoFace => Target::Surface(*id),
                EntityKind::TopoVolume | EntityKind::PartitionCell => Target::Volume(*id),
                _ => continue,
            };
            if let Some(ch) = &ei.scheme {
                if ch.name.as_str() == SchemeName::COPY {
                    copy_intent.insert(t);
                } else if ch.firmness == Firmness::Hard {
                    intent_hard.insert(t);
                }
            }
        }
        for g in &intent.groups {
            if g.scheme
                .as_ref()
                .is_some_and(|c| c.firmness == Firmness::Hard)
            {
                intent_hard.insert(Target::Group(g.id));
            }
        }
        let tol = drv
            .options
            .copy_tolerance
            .unwrap_or(input.topo.tolerance().linear);
        Self {
            drv,
            input,
            intent,
            rej,
            cx: PlanCtx { input, intent },
            region,
            p: MeshingProblem::default(),
            plans: BTreeMap::new(),
            hard: BTreeSet::new(),
            intent_hard,
            copy_intent,
            claimed: BTreeMap::new(),
            attempts: Vec::new(),
            classes: Vec::new(),
            class_ix: BTreeMap::new(),
            closed: BTreeSet::new(),
            checked: 0,
            tol,
            imp_ix: BTreeMap::new(),
            imp_seen: 0,
        }
    }

    fn run(mut self) -> Result<Plan> {
        self.p.vertices = self.region.vertices.clone();
        self.deck()?;
        self.boundary_layers()?;
        self.plan_volumes()?;
        self.user_copies()?;
        self.refresh_copies()?;
        let faces: Vec<Target> = self
            .region
            .faces
            .iter()
            .map(|f| Target::Surface(*f))
            .collect();
        self.plan_all(&faces)?;
        let edges: Vec<Target> = self
            .region
            .edges
            .iter()
            .map(|e| Target::Curve(*e))
            .collect();
        self.plan_all(&edges)?;
        self.finish_copies()?;

        // Corner requests from intent, then corner picking per face with a rule.
        for c in &self.intent.corners {
            if self.region.faces.binary_search(&c.corner.face).is_ok() {
                self.p
                    .push_vertex_type(Origin::Intent, c.corner, c.vertex_type);
            }
        }
        let layouts = self.pick_corners().map_err(|e| self.blame(e))?;

        // Intervals: intent and sizing goals, curve-copy equalities, then scheme equations.
        self.sizing_constraints()?;
        let eqs: Vec<(EntityId, EntityId)> = self
            .p
            .copies
            .iter()
            .filter_map(|r| match (r.item.source, r.item.target) {
                (Target::Curve(a), Target::Curve(b)) => Some((a, b)),
                _ => None,
            })
            .collect();
        for (a, b) in eqs {
            self.p
                .push_interval(Origin::Driver, IntervalConstraint::sum_equal(&[a], &[b]));
        }
        let roles: Vec<(Target, Role)> = self.p.roles.iter().map(|r| r.item).collect();
        let tps: Vec<TargetPlan> = self.plans.values().cloned().collect();
        let icx = IntervalCtx {
            input: self.input,
            layouts: &layouts,
            roles: &roles,
            targets: &tps,
        };
        for tp in &tps {
            if tp.scheme.as_str() == SchemeName::COPY
                || matches!(tp.selection, Selection::Claimed { .. })
            {
                continue;
            }
            let s = self
                .drv
                .registry
                .get(tp.target.dim(), tp.scheme.as_str())
                .ok_or_else(|| SchemeError::UnknownScheme {
                    name: tp.scheme.clone(),
                    target: tp.target,
                })?;
            s.interval_constraints(
                &icx,
                tp.target,
                &tp.params,
                &mut IntervalSink::new(&mut self.p, &tp.scheme, tp.target),
            )?;
        }
        let intervals = match self.drv.solver.solve(&self.region.edges, &self.p.intervals) {
            Ok(v) => v,
            Err(e) => return Err(self.blame(e.into())),
        };

        let transforms = self.plan_transforms()?;
        self.p.targets = std::mem::take(&mut self.plans).into_values().collect();
        self.p.invalidate();
        let groups: BTreeMap<GroupId, Vec<EntityId>> = self.region.groups.iter().cloned().collect();
        let plans: BTreeMap<Target, TargetPlan> = self
            .p
            .targets
            .iter()
            .map(|t| (t.target, t.clone()))
            .collect();
        let scheduled = schedule(&ScheduleInput {
            input: self.input,
            plans: &plans,
            problem: &self.p,
            classes: &self.classes,
            transforms: &transforms,
            groups: &groups,
            grouped: &self.region.grouped,
        });
        let (footprints, waves) = match scheduled {
            Err(SchemeError::Schedule { jobs, constraints, reason }) => return Err(self.blame_cycle(jobs, constraints, reason)),
            other => other?,
        };
        Ok(Plan {
            problem: self.p,
            intervals,
            layouts,
            copies: self.classes,
            transforms,
            footprints,
            waves,
            rejections: self.rej.clone(),
            attempts: self.attempts,
            sizing: self.intent.defaults.sizing.clone(),
            index: Default::default(),
        })
    }

    // ------------------------------------------------------------------ blame

    fn blame(&self, mut e: SchemeError) -> SchemeError {
        if let SchemeError::Infeasible(inf) = &mut e
            && inf.blame.is_none()
        {
            inf.blame = self.pick_blame(&inf.constraints);
        }
        e
    }

    /// A schedule error stays a schedule error unless it is a dependency cycle through a
    /// scheme-origin `Precedes` (a scheme's [`Requirements::reads`]): that becomes an
    /// [`Infeasible`] blaming the reading `(target, scheme)` (the later such constraint, if the
    /// scheme is blameable), so the driver rejects it and re-plans.
    fn blame_cycle(&self, jobs: Vec<Job>, constraints: Vec<ConstraintId>, reason: String) -> SchemeError {
        let blame = (reason == "dependency cycle")
            .then(|| {
                constraints.iter().rev().find_map(|id| {
                    let is_read = self.p.constraints.iter().any(|r| r.id == *id && matches!(r.item, MeshConstraint::Precedes { .. }));
                    match self.p.record_origin(*id)? {
                        Origin::Scheme { scheme, target }
                            if is_read && scheme.as_str() != SchemeName::COPY && !self.hard.contains(target) && !self.rej.0.contains_key(&(*target, scheme.clone())) =>
                        {
                            Some((*target, scheme.clone()))
                        }
                        _ => None,
                    }
                })
            })
            .flatten();
        match blame {
            Some(b) => {
                let mut entities: Vec<EntityId> = jobs
                    .iter()
                    .filter_map(|j| match j {
                        Job::Generate(t) => t.entity(),
                        Job::Transform(_) => None,
                    })
                    .collect();
                entities.sort_unstable();
                entities.dedup();
                Infeasible { reason: "dependency cycle through a scheme read".into(), constraints, entities, blame: Some(b) }.into()
            }
            None => SchemeError::Schedule { jobs, constraints, reason },
        }
    }

    /// Among the involved scheme-origin constraints whose selection is not a hard intent, the
    /// lowest-dimension one; ties go to the larger (later) constraint id.
    fn pick_blame(&self, cids: &[ConstraintId]) -> Option<(Target, SchemeName)> {
        cids.iter()
            .filter_map(|id| match self.p.record_origin(*id)? {
                Origin::Scheme { scheme, target }
                    if scheme.as_str() != SchemeName::COPY
                        && !self.hard.contains(target)
                        && !self.rej.0.contains_key(&(*target, scheme.clone())) =>
                {
                    Some((*id, *target, scheme.clone()))
                }
                _ => None,
            })
            .min_by_key(|(id, t, _)| (t.dim().rank(), Reverse(*id)))
            .map(|(_, t, s)| (t, s))
    }

    // ------------------------------------------------------------------ model-level constraints

    fn deck(&mut self) -> Result<()> {
        let topo = self.input.topo;
        for f in self.region.faces.clone() {
            let Some(FaceKindView::Deck {
                boundary,
                deck,
                side,
                mate,
            }) = self.input.model.face_kind(f, topo)
            else {
                continue;
            };
            if self
                .intent
                .entities
                .get(&f)
                .is_some_and(|e| e.boundary_layer.is_some())
            {
                return Err(SchemeError::BoundaryLayerOnDeck { face: f, boundary });
            }
            let origin = Origin::Deck { boundary };
            self.p.push_imposition(
                origin.clone(),
                Imposition {
                    child: Target::Surface(f),
                    need: Need::Forbid(Caps::BOUNDARY_LAYER),
                    firmness: Firmness::Hard,
                    by: Target::Surface(f),
                },
            );
            match (deck, side, mate) {
                (Deck::Mirror { plane }, _, _) => {
                    self.p.push_constraint(
                        origin,
                        MeshConstraint::PlaneLock {
                            face: f,
                            normal: plane.normal,
                            offset: plane.offset,
                        },
                    );
                }
                (_, sm_model_api::DeckSide::A, Some(m))
                    if self.region.faces.binary_search(&m).is_ok() =>
                {
                    let c = CopyConstraint {
                        source: Target::Surface(f),
                        target: Target::Surface(m),
                        transform: deck.transform(),
                        vertex_map: Vec::new(),
                        reason: CopyReason::Deck { boundary },
                    };
                    self.p.push_copy(origin, c);
                }
                _ => {}
            }
        }
        Ok(())
    }

    /// Boundary-layer requests: a constraint per adjacent volume target, plus a soft need for
    /// a volume scheme with the boundary-layer capability.
    fn boundary_layers(&mut self) -> Result<()> {
        // A request on anything but a region face would be silently dropped: refuse it (curve
        // boundary layers do not exist yet; `MeshIntent::check` refuses non-face kinds too).
        for (id, e) in &self.intent.entities {
            if e.boundary_layer.is_some() && self.region.faces.binary_search(id).is_err() && !matches!(id.kind(), EntityKind::TopoFace | EntityKind::PartitionFace) {
                return Err(SchemeError::Input(format!("{id}: boundary layers are only supported on faces")));
            }
        }
        for f in self.region.faces.clone() {
            let Some(spec) = self.intent.entities.get(&f).and_then(|e| e.boundary_layer.clone()) else {
                continue;
            };
            let mut targets: Vec<Target> = Vec::new();
            let mut adjacent: Vec<EntityId> = Vec::new();
            for (v, faces) in &self.region.vol_faces {
                if faces.binary_search(&f).is_err() {
                    continue;
                }
                adjacent.push(*v);
                if !spec.into.is_empty() && !spec.into.contains(v) {
                    continue;
                }
                targets.push(match self.region.grouped.get(v) {
                    Some(g) => Target::Group(*g),
                    None => Target::Volume(*v),
                });
            }
            if let Some(v) = spec.into.iter().find(|v| !adjacent.contains(v)) {
                return Err(SchemeError::Input(format!("{f}: boundary layer `into` volume {v} is not adjacent to the face")));
            }
            for vt in sorted(targets) {
                self.p.push_constraint(
                    Origin::Intent,
                    MeshConstraint::BoundaryLayer {
                        face: f,
                        volume: vt,
                        spec: spec.clone(),
                    },
                );
                self.p.push_imposition(
                    Origin::Intent,
                    Imposition {
                        child: vt,
                        need: Need::Caps(Caps::BOUNDARY_LAYER),
                        firmness: Firmness::Hard,
                        by: Target::Surface(f),
                    },
                );
            }
        }
        Ok(())
    }

    fn plan_volumes(&mut self) -> Result<()> {
        for v in self.region.volumes.clone() {
            if self.region.grouped.contains_key(&v) {
                continue;
            }
            self.plan_target(Target::Volume(v))?;
        }
        for (id, members) in self.region.groups.clone() {
            let mt: Vec<Target> = members.iter().map(|m| Target::Volume(*m)).collect();
            self.p.push_constraint(
                Origin::Intent,
                MeshConstraint::Atomic {
                    group: id,
                    members: mt,
                },
            );
            self.plan_target(Target::Group(id))?;
        }
        Ok(())
    }

    /// `copy` schemes in the intent become copy constraints.
    fn user_copies(&mut self) -> Result<()> {
        let topo = self.input.topo;
        for (id, ei) in &self.intent.entities {
            let Some(ch) = ei
                .scheme
                .as_ref()
                .filter(|c| c.name.as_str() == SchemeName::COPY)
            else {
                continue;
            };
            let t = match topo.dim_of(*id) {
                Some(Dim::Face) if self.region.faces.binary_search(id).is_ok() => {
                    Target::Surface(*id)
                }
                Some(Dim::Edge) if self.region.edges.binary_search(id).is_ok() => {
                    Target::Curve(*id)
                }
                _ => continue,
            };
            let source: EntityId = ch.params.get(SchemeName::COPY, "source")?.ok_or_else(|| {
                SchemeError::BadParams {
                    scheme: ch.name.clone(),
                    reason: "missing `source`".into(),
                }
            })?;
            let transform: Transform = ch
                .params
                .get(SchemeName::COPY, "transform")?
                .unwrap_or(Transform::IDENTITY);
            let vertex_map: Vec<(EntityId, EntityId)> = sorted(
                ch.params
                    .get(SchemeName::COPY, "vertex_map")?
                    .unwrap_or_default(),
            );
            let st = match t {
                Target::Surface(_) => Target::Surface(source),
                _ => Target::Curve(source),
            };
            self.p.push_copy(
                Origin::Intent,
                CopyConstraint {
                    source: st,
                    target: t,
                    transform,
                    vertex_map,
                    reason: CopyReason::User,
                },
            );
        }
        Ok(())
    }

    // ------------------------------------------------------------------ copy closure and classes

    /// Validate and complete the new copy constraints (vertex maps, implied edge copies), then
    /// rebuild the classes. Runs again after every requirement that adds copies (defect D4).
    fn refresh_copies(&mut self) -> Result<()> {
        let topo = self.input.topo;
        loop {
            let n0 = self.p.copies.len();
            for i in self.checked..n0 {
                let c = &self.p.copies[i].item;
                let inside = |t: Target| match t {
                    Target::Surface(f) => self.region.faces.binary_search(&f).is_ok(),
                    Target::Curve(e) => self.region.edges.binary_search(&e).is_ok(),
                    _ => false,
                };
                if !inside(c.source)
                    || !inside(c.target)
                    || c.source == c.target
                    || c.source.dim() != c.target.dim()
                {
                    return Err(SchemeError::Correspondence { source_target: c.source, target: c.target, reason: "source and target must be distinct entities of one dimension inside the meshed region".into() });
                }
            }
            self.checked = n0;
            fill_vertex_maps(topo, &mut self.p.copies)?;
            let mut pairs: BTreeSet<(Target, Target)> = self
                .p
                .copies
                .iter()
                .map(|r| (r.item.source, r.item.target))
                .collect();
            // Edge copies implied by face copies.
            for i in 0..n0 {
                if !matches!(self.p.copies[i].item.target, Target::Surface(_))
                    || !self.closed.insert(self.p.copies[i].id)
                {
                    continue;
                }
                let parent = self.p.copies[i].id;
                let c = self.p.copies[i].item.clone();
                let vm: BTreeMap<EntityId, EntityId> = c.vertex_map.iter().copied().collect();
                for (e, f, _) in copy::edge_map(topo, &c)? {
                    if e == f {
                        continue;
                    }
                    if pairs.contains(&(Target::Curve(e), Target::Curve(f)))
                        || pairs.contains(&(Target::Curve(f), Target::Curve(e)))
                    {
                        continue;
                    }
                    pairs.insert((Target::Curve(e), Target::Curve(f)));
                    let Some((a, b)) = topo.edge_ends(e) else {
                        continue;
                    };
                    let mut evm: Vec<(EntityId, EntityId)> = [a, b]
                        .iter()
                        .filter_map(|v| vm.get(v).map(|w| (*v, *w)))
                        .collect();
                    evm.sort_unstable();
                    evm.dedup();
                    self.p.push_copy(
                        Origin::Driver,
                        CopyConstraint {
                            source: Target::Curve(e),
                            target: Target::Curve(f),
                            transform: c.transform,
                            vertex_map: evm,
                            reason: CopyReason::Closure { parent },
                        },
                    );
                }
            }
            if self.p.copies.len() == n0 {
                break;
            }
        }
        self.p.invalidate();
        self.rebuild_classes()
    }

    fn rebuild_classes(&mut self) -> Result<()> {
        let claimed: BTreeSet<Target> = self.claimed.keys().copied().collect();
        let resolved: BTreeSet<Target> = self
            .plans
            .values()
            .filter(|t| {
                !matches!(
                    t.selection,
                    Selection::Claimed { .. } | Selection::Copy { .. }
                )
            })
            .map(|t| t.target)
            .filter(|t| matches!(t, Target::Surface(_) | Target::Curve(_)))
            .collect();
        let sweep_sources: BTreeSet<Target> = self
            .p
            .roles
            .iter()
            .filter(|r| matches!(r.item.1, Role::SweepSource { .. }))
            .map(|r| r.item.0)
            .collect();
        let hard: BTreeSet<Target> = self.intent_hard.union(&self.hard).copied().collect();
        self.classes = build_classes(&ClassInput {
            topo: self.input.topo,
            copies: &self.p.copies,
            claimed: &claimed,
            resolved: &resolved,
            sweep_sources: &sweep_sources,
            hard: &hard,
            copy_intent: &self.copy_intent,
            tol: self.tol,
        })?;
        self.class_ix.clear();
        for (ci, c) in self.classes.iter().enumerate() {
            self.class_ix.insert(c.root, (ci, None));
            for (mi, m) in c.members.iter().enumerate() {
                self.class_ix.insert(m.target, (ci, Some(mi)));
            }
        }
        Ok(())
    }

    fn plan_all(&mut self, targets: &[Target]) -> Result<()> {
        for &t in targets {
            let before = self.p.copies.len();
            self.plan_target(t)?;
            if self.p.copies.len() != before {
                self.refresh_copies()?;
            }
        }
        self.refresh_copies()
    }

    /// Final class checks: every planned copy is still a member, copy members get the class
    /// index and the caps of their root, and impositions on members hold (defect D3).
    fn finish_copies(&mut self) -> Result<()> {
        for (ci, class) in self.classes.iter().enumerate() {
            let root_plan = self.plans.get(&class.root).ok_or_else(|| {
                SchemeError::Input(format!("copy root {:?} was not planned", class.root))
            })?;
            if matches!(root_plan.selection, Selection::Copy { .. }) {
                return Err(SchemeError::Input(format!(
                    "copy root {:?} was planned as a copy",
                    class.root
                )));
            }
            let rcaps = if matches!(root_plan.selection, Selection::Claimed { .. }) {
                root_plan.caps
            } else {
                root_plan.caps.without(Caps::INHERIT)
            };
            for m in &class.members {
                let tp = self.plans.get_mut(&m.target).ok_or_else(|| {
                    SchemeError::Input(format!("copy member {:?} was not planned", m.target))
                })?;
                match tp.selection {
                    Selection::Copy { .. } => {
                        tp.selection = Selection::Copy { class: ci as u32 };
                        tp.caps = rcaps;
                    }
                    Selection::Claimed { .. } => {}
                    _ => {
                        return Err(SchemeError::Input(format!(
                            "copy member {:?} was planned with its own scheme",
                            m.target
                        )));
                    }
                }
            }
        }
        // Impositions on members are checked against what actually meshes them.
        let mut soft: Vec<Attempt> = Vec::new();
        for imp in &self.p.impositions {
            let Some(&(ci, Some(mi))) = self.class_ix.get(&imp.item.child) else {
                continue;
            };
            let class = &self.classes[ci];
            let m = &class.members[mi];
            let root = self.plans.get(&class.root).ok_or_else(|| {
                SchemeError::Input(format!("copy root {:?} was not planned", class.root))
            })?;
            let (name, caps) = match self.plans.get(&m.target).map(|t| &t.selection) {
                Some(Selection::Claimed { by }) => {
                    let bp = self.plans.get(by).ok_or_else(|| {
                        SchemeError::Input(format!("claimer {by:?} was not planned"))
                    })?;
                    (bp.scheme.clone(), child_caps(bp.caps))
                }
                _ => (
                    root.scheme.clone(),
                    if matches!(root.selection, Selection::Claimed { .. }) {
                        root.caps
                    } else {
                        root.caps.without(Caps::INHERIT)
                    },
                ),
            };
            if violates(&imp.item.need, &name, caps) {
                if imp.item.firmness != Firmness::Hard {
                    soft.push(Attempt {
                        target: m.target,
                        scheme: name.clone(),
                        phase: Phase::Resolve,
                        kind: FailureKind::Rejected,
                        message: format!(
                            "soft imposition from {:?} not met: meshed as a copy of {:?}",
                            imp.item.by, class.root
                        ),
                    });
                    continue;
                }
                return Err(SchemeError::ImpositionConflict {
                    child: m.target,
                    imposed_by: imp.item.by,
                    required: imp.item.need.clone(),
                    conflict: format!("it is meshed as a copy of {:?} by `{name}`", class.root),
                });
            }
        }
        // Impositions on claimed children (not class members) hold against what the claimer
        // writes there. The claimer's own impositions are its business.
        for imp in &self.p.impositions {
            if matches!(self.class_ix.get(&imp.item.child), Some(&(_, Some(_)))) {
                continue;
            }
            let Some(Selection::Claimed { by }) =
                self.plans.get(&imp.item.child).map(|p| &p.selection)
            else {
                continue;
            };
            if imp.item.by == *by {
                continue;
            }
            let bp = self
                .plans
                .get(by)
                .ok_or_else(|| SchemeError::Input(format!("claimer {by:?} was not planned")))?;
            let (name, caps) = (bp.scheme.clone(), child_caps(bp.caps));
            if violates(&imp.item.need, &name, caps) {
                if imp.item.firmness != Firmness::Hard {
                    soft.push(Attempt {
                        target: imp.item.child,
                        scheme: name.clone(),
                        phase: Phase::Resolve,
                        kind: FailureKind::Rejected,
                        message: format!(
                            "soft imposition from {:?} not met: written by {by:?}",
                            imp.item.by
                        ),
                    });
                    continue;
                }
                return Err(SchemeError::ImpositionConflict {
                    child: imp.item.child,
                    imposed_by: imp.item.by,
                    required: imp.item.need.clone(),
                    conflict: format!("it is written by {by:?} with `{name}`"),
                });
            }
        }
        self.attempts.extend(soft);
        // A copy plan must belong to a class.
        for tp in self.plans.values() {
            if matches!(tp.selection, Selection::Copy { .. })
                && !self.class_ix.contains_key(&tp.target)
            {
                return Err(SchemeError::Input(format!(
                    "{:?} was planned as a copy but no copy constraint remains",
                    tp.target
                )));
            }
        }
        Ok(())
    }

    // ------------------------------------------------------------------ scheme resolution

    fn plan_target(&mut self, t: Target) -> Result<()> {
        let tp = if let Some(by) = self.claimed.get(&t).copied() {
            self.claimed_plan(t, by)?
        } else if let Some(&(ci, Some(mi))) = self.class_ix.get(&t) {
            self.copy_plan(t, ci, mi)?
        } else {
            let (mut tp, hard) = self.resolve(t)?;
            if hard {
                self.hard.insert(t);
            }
            self.require(&mut tp)?;
            tp
        };
        self.plans.insert(t, tp);
        Ok(())
    }

    fn intent_choice(&self, t: Target) -> Option<(&SchemeChoice, &[SchemeChoice])> {
        match t {
            Target::Group(g) => self
                .intent
                .groups
                .iter()
                .find(|x| x.id == g)
                .and_then(|x| x.scheme.as_ref().map(|s| (s, x.fallbacks.as_slice()))),
            _ => t
                .entity()
                .and_then(|e| self.intent.entities.get(&e))
                .and_then(|ei| ei.scheme.as_ref().map(|s| (s, ei.fallbacks.as_slice()))),
        }
    }

    /// The plan of a claimed child: written by the claimer, so the claimer's scheme and caps.
    fn claimed_plan(&mut self, t: Target, by: Target) -> Result<TargetPlan> {
        let bp = self.plans.get(&by).ok_or_else(|| {
            SchemeError::Input(format!("{t:?} is claimed by {by:?}, which is not planned"))
        })?;
        let (scheme, caps) = (bp.scheme.clone(), child_caps(bp.caps));
        if let Some((ch, _)) = self.intent_choice(t).filter(|(c, _)| c.name != scheme) {
            if ch.firmness == Firmness::Hard {
                return Err(SchemeError::ImpositionConflict {
                    child: t,
                    imposed_by: by,
                    required: Need::Schemes(vec![scheme]),
                    conflict: format!("the intent requires `{}` (hard)", ch.name),
                });
            }
            self.attempts.push(Attempt {
                target: t,
                scheme: ch.name.clone(),
                phase: Phase::Resolve,
                kind: FailureKind::Rejected,
                message: format!("overridden: {t:?} is written by {by:?}"),
            });
        }
        Ok(TargetPlan {
            target: t,
            scheme,
            params: Params::empty(),
            data: Params::empty(),
            caps,
            selection: Selection::Claimed { by },
            reason: format!("written by {by:?}"),
        })
    }

    /// The plan of a non-root class member. A hard intent for another scheme is an
    /// `ImpositionConflict` (the root imposes `copy`); a soft one is recorded as overridden.
    fn copy_plan(&mut self, t: Target, ci: usize, mi: usize) -> Result<TargetPlan> {
        let class = &self.classes[ci];
        let m = &class.members[mi];
        if m.mode == CopyMode::Verify {
            return Err(SchemeError::Input(format!(
                "{t:?} is a verify member but not claimed"
            )));
        }
        let root = class.root;
        let ch = self
            .intent_choice(t)
            .map(|(c, _)| c.clone())
            .filter(|c| c.name.as_str() != SchemeName::COPY);
        if let Some(ch) = ch {
            if ch.firmness == Firmness::Hard {
                return Err(SchemeError::ImpositionConflict {
                    child: t,
                    imposed_by: root,
                    required: Need::Schemes(vec![SchemeName::new(SchemeName::COPY)]),
                    conflict: format!("the intent requires `{}` (hard)", ch.name),
                });
            }
            self.attempts.push(Attempt {
                target: t,
                scheme: ch.name.clone(),
                phase: Phase::Resolve,
                kind: FailureKind::Rejected,
                message: format!("overridden: {t:?} is a copy of {root:?}"),
            });
        }
        Ok(TargetPlan {
            target: t,
            scheme: SchemeName::new(SchemeName::COPY),
            params: Params::empty(),
            data: Params::empty(),
            caps: Caps::INHERIT,
            selection: Selection::Copy { class: ci as u32 },
            reason: format!("copy of {root:?}"),
        })
    }

    /// Impositions on `t`, and, when `t` is a class root, on the members it generates.
    fn needs_for(&self, t: Target) -> Vec<NeedRef> {
        let mut subjects = vec![t];
        if let Some(&(ci, None)) = self.class_ix.get(&t) {
            subjects.extend(
                self.classes[ci]
                    .members
                    .iter()
                    .filter(|m| m.mode == CopyMode::Generate)
                    .map(|m| m.target),
            );
        }
        subjects
            .iter()
            .flat_map(|s| self.imp_ix.get(s).into_iter().flatten().copied())
            .collect::<BTreeSet<usize>>()
            .into_iter()
            .map(|i| {
                let r = &self.p.impositions[i];
                NeedRef {
                    child: r.item.child,
                    need: r.item.need.clone(),
                    firmness: r.item.firmness,
                    by: r.item.by,
                    id: r.id,
                    from_intent: r.origin == Origin::Intent,
                }
            })
            .collect()
    }

    fn sync_impositions(&mut self) {
        for i in self.imp_seen..self.p.impositions.len() {
            self.imp_ix
                .entry(self.p.impositions[i].item.child)
                .or_default()
                .push(i);
        }
        self.imp_seen = self.p.impositions.len();
    }

    fn att(
        &self,
        t: Target,
        name: &SchemeName,
        kind: FailureKind,
        msg: impl Into<String>,
    ) -> Attempt {
        Attempt {
            target: t,
            scheme: name.clone(),
            phase: Phase::Resolve,
            kind,
            message: msg.into(),
        }
    }

    /// Evaluate one candidate. `Some((score, reason))` if it can win; `None` if it was passed
    /// over (the reason is recorded in `local`).
    fn try_candidate(
        &self,
        t: Target,
        choice: &SchemeChoice,
        needs: &[NeedRef],
        from_intent: bool,
        honour_soft: bool,
        local: &mut Vec<Attempt>,
    ) -> Result<Option<(u32, String)>> {
        let name = &choice.name;
        let Some(s) = self.drv.registry.get(t.dim(), name.as_str()) else {
            if from_intent {
                return Err(SchemeError::UnknownScheme {
                    name: name.clone(),
                    target: t,
                });
            }
            local.push(self.att(t, name, FailureKind::NotApplicable, "not registered"));
            return Ok(None);
        };
        if matches!(t, Target::Group(_)) && !s.supports_groups() {
            local.push(self.att(t, name, FailureKind::NotApplicable, "does not mesh groups"));
            return Ok(None);
        }
        if let Some(prev) = self.rej.0.get(&(t, name.clone())) {
            local.push(self.att(
                t,
                name,
                FailureKind::Rejected,
                format!(
                    "rejected after a {:?} failure in {:?}: {}",
                    prev.kind, prev.phase, prev.message
                ),
            ));
            return Ok(None);
        }
        let caps = s.caps(&choice.params).without(Caps::INHERIT);
        // A hard intent beats soft impositions; soft ones are preferences, dropped in the
        // fallback pass (`honour_soft == false`).
        let hard_intent = choice.firmness == Firmness::Hard && from_intent;
        for n in needs
            .iter()
            .filter(|n| n.firmness != Firmness::Hard && violates(&n.need, name, caps))
        {
            if hard_intent {
                local.push(self.att(
                    t,
                    name,
                    FailureKind::Rejected,
                    format!(
                        "soft imposition from {:?} ignored: the intent requires `{name}` (hard)",
                        n.by
                    ),
                ));
            }
        }
        if let Some(n) = needs.iter().find(|n| {
            violates(&n.need, name, caps)
                && (n.firmness == Firmness::Hard || (honour_soft && !hard_intent))
        }) {
            if hard_intent {
                return Err(if n.from_intent {
                    SchemeError::NotApplicable {
                        target: t,
                        scheme: name.clone(),
                        reason: format!("requested {:?} is not provided by the scheme", n.need),
                    }
                } else {
                    SchemeError::ImpositionConflict {
                        child: n.child,
                        imposed_by: n.by,
                        required: n.need.clone(),
                        conflict: format!("the intent requires `{name}` (hard)"),
                    }
                });
            }
            local.push(self.att(
                t,
                name,
                FailureKind::Rejected,
                format!("overridden by imposition from {:?}: {:?}", n.by, n.need),
            ));
            return Ok(None);
        }
        match s.applicability(&self.cx, t, &choice.params) {
            Applicability::Applicable { score, reason } => Ok(Some((score, reason))),
            Applicability::NotApplicable { reason } => {
                local.push(self.att(t, name, FailureKind::NotApplicable, reason));
                Ok(None)
            }
        }
    }

    fn tp(
        &self,
        t: Target,
        choice: SchemeChoice,
        selection: Selection,
        reason: String,
    ) -> TargetPlan {
        let caps = self
            .drv
            .registry
            .get(t.dim(), choice.name.as_str())
            .map_or(Caps::NONE, |s| s.caps(&choice.params));
        TargetPlan {
            target: t,
            scheme: choice.name,
            params: choice.params,
            data: Params::empty(),
            caps,
            selection,
            reason,
        }
    }

    /// Choose the scheme of `t`. Order: the intent chain, the registry fallbacks of its last
    /// choice, else the piece-shape hint, else the best applicable candidate among the imposed
    /// set or the auto list. Returns the plan and whether the winner is a hard intent.
    fn resolve(&mut self, t: Target) -> Result<(TargetPlan, bool)> {
        self.sync_impositions();
        match self.resolve_pass(t, true) {
            // Soft impositions are preferences: when nothing satisfies them, plan without.
            Err(
                SchemeError::NoApplicableScheme { .. } | SchemeError::ImpositionConflict { .. },
            ) if self
                .needs_for(t)
                .iter()
                .any(|n| n.firmness != Firmness::Hard) =>
            {
                self.resolve_pass(t, false)
            }
            r => r,
        }
    }

    fn resolve_pass(&mut self, t: Target, honour_soft: bool) -> Result<(TargetPlan, bool)> {
        let dim = t.dim();
        let needs: Vec<NeedRef> = self
            .needs_for(t)
            .into_iter()
            .filter(|n| honour_soft || n.firmness == Firmness::Hard)
            .collect();
        // Impositions by name intersect; an empty intersection is a conflict between parents.
        let mut allowed: Option<(Vec<SchemeName>, Target)> = None;
        for n in &needs {
            if let Need::Schemes(names) = &n.need {
                allowed = Some(match allowed.take() {
                    None => (names.clone(), n.by),
                    Some((a, by)) => {
                        let both: Vec<SchemeName> =
                            a.iter().filter(|x| names.contains(x)).cloned().collect();
                        if both.is_empty() {
                            return Err(SchemeError::ImpositionConflict {
                                child: n.child,
                                imposed_by: n.by,
                                required: n.need.clone(),
                                conflict: format!("{by:?} requires one of {a:?}"),
                            });
                        }
                        (both, by)
                    }
                });
            }
        }
        // Explicit chain, registry fallbacks of its last choice, else the piece hint.
        let mut chain: Vec<Cand> = Vec::new();
        if let Some((s, fb)) = self.intent_choice(t) {
            chain.push(Cand::Chain(s.clone(), Selection::Intent { rank: 0 }));
            for (i, f) in fb.iter().enumerate() {
                chain.push(Cand::Chain(
                    f.clone(),
                    Selection::Intent { rank: i as u32 + 1 },
                ));
            }
        }
        let n_intent = chain.len();
        let any_hard = chain
            .iter()
            .any(|Cand::Chain(c, _)| c.firmness == Firmness::Hard);
        if let Some(Cand::Chain(last, _)) = chain.last() {
            let names: Vec<SchemeName> = self
                .drv
                .registry
                .fallbacks(dim, last.name.as_str())
                .to_vec();
            for (i, n) in names.into_iter().enumerate() {
                chain.push(Cand::Chain(
                    SchemeChoice::new(n.as_str(), Firmness::Default),
                    Selection::Registry { rank: i as u32 },
                ));
            }
        }
        // Then the piece-shape hint (never past a hard intent, which must not be replaced).
        if !any_hard
            && let Some(PieceRef::Cell(c)) = t.piece()
            && let Some(piece) = self.input.model.decomposition().piece(c)
        {
            match &piece.shape {
                PieceShape::Sweep { source, target: tg } => {
                    let map = |v: &[sm_model_api::PieceFace]| -> Vec<EntityId> {
                        v.iter()
                            .filter_map(|pf| {
                                self.input
                                    .pieces
                                    .face_of(self.input.topo, self.input.model, pf)
                            })
                            .collect()
                    };
                    let params = serde_json::json!({ "source": map(source), "target": map(tg) });
                    chain.push(Cand::Chain(
                        SchemeChoice::with_params("sweep", params, Firmness::Default),
                        Selection::PieceShape,
                    ));
                }
                PieceShape::Map => chain.push(Cand::Chain(
                    SchemeChoice::new("map", Firmness::Default),
                    Selection::PieceShape,
                )),
                PieceShape::Any => {}
            }
        }
        let mut local: Vec<Attempt> = Vec::new();
        let mut tried: BTreeSet<SchemeName> = BTreeSet::new();
        for (i, Cand::Chain(choice, sel)) in chain.into_iter().enumerate() {
            if !tried.insert(choice.name.clone()) {
                continue;
            }
            if let Some((_, reason)) =
                self.try_candidate(t, &choice, &needs, i < n_intent, honour_soft, &mut local)?
            {
                let hard = i < n_intent && choice.firmness == Firmness::Hard;
                self.attempts.extend(local);
                return Ok((self.tp(t, choice, sel, reason), hard));
            }
        }
        if any_hard {
            let tried_n = local.clone();
            self.attempts.extend(local);
            if let [only] = tried_n.as_slice()
                && n_intent == 1
                && only.kind == FailureKind::NotApplicable
            {
                return Err(SchemeError::NotApplicable {
                    target: t,
                    scheme: only.scheme.clone(),
                    reason: only.message.clone(),
                });
            }
            return Err(SchemeError::NoApplicableScheme {
                target: t,
                tried: tried_n,
                imposed_by: self.blameable_imposers(t, &needs),
            });
        }
        // Imposed set or auto list.
        let policy = match dim {
            SchemeDim::Curve => &self.intent.defaults.auto.curve,
            SchemeDim::Surface => &self.intent.defaults.auto.surface,
            SchemeDim::Volume => &self.intent.defaults.auto.volume,
        };
        let names: Vec<SchemeName> = if !policy.is_empty() {
            policy.clone()
        } else if !self.drv.registry.auto(dim).is_empty() {
            self.drv.registry.auto(dim).to_vec()
        } else {
            self.drv
                .registry
                .of_dim(dim)
                .map(|s| SchemeName::new(s.name()))
                .collect()
        };
        // A parent's requirement must stay satisfiable under a narrow auto list: with
        // impositions, the remaining registered schemes follow the list (name order).
        let names: Vec<SchemeName> = if needs.is_empty() {
            names
        } else {
            let mut v = names;
            let rest: Vec<SchemeName> = self
                .drv
                .registry
                .of_dim(dim)
                .map(|s| SchemeName::new(s.name()))
                .filter(|n| !v.contains(n))
                .collect();
            v.extend(rest);
            v
        };
        let names: Vec<SchemeName> = match &allowed {
            Some((a, _)) => {
                let mut v: Vec<SchemeName> =
                    names.iter().filter(|n| a.contains(n)).cloned().collect();
                v.extend(a.iter().filter(|n| !names.contains(n)).cloned());
                v
            }
            None => names,
        };
        let mut scored: Vec<(u32, SchemeName, String)> = Vec::new();
        for n in names {
            if !tried.insert(n.clone()) {
                continue;
            }
            let choice = SchemeChoice::new(n.as_str(), Firmness::Default);
            if let Some((score, reason)) =
                self.try_candidate(t, &choice, &needs, false, honour_soft, &mut local)?
            {
                scored.push((score, n, reason));
            }
        }
        let best = scored
            .iter()
            .enumerate()
            .fold(None::<usize>, |b, (i, s)| match b {
                Some(j) if scored[j].0 >= s.0 => Some(j),
                _ => Some(i),
            });
        match best {
            Some(w) => {
                let (score, name, reason) = scored[w].clone();
                for (i, (sc, n, _)) in scored.iter().enumerate() {
                    if i != w {
                        local.push(self.att(
                            t,
                            n,
                            FailureKind::Rejected,
                            format!("outscored by `{name}` ({sc} < {score})"),
                        ));
                    }
                }
                self.attempts.extend(local);
                let selection = if needs.is_empty() {
                    Selection::Auto { score }
                } else {
                    Selection::Imposed { score }
                };
                Ok((
                    self.tp(
                        t,
                        SchemeChoice::new(name.as_str(), Firmness::Default),
                        selection,
                        reason,
                    ),
                    false,
                ))
            }
            None => {
                let tried_n = local.clone();
                self.attempts.extend(local);
                Err(SchemeError::NoApplicableScheme {
                    target: t,
                    tried: tried_n,
                    imposed_by: self.blameable_imposers(t, &needs),
                })
            }
        }
    }

    /// Imposing parents that a re-plan could change: planned, not hard, not copies, not `t` itself.
    fn blameable_imposers(&self, t: Target, needs: &[NeedRef]) -> Vec<(Target, SchemeName)> {
        let mut out: Vec<(Target, SchemeName)> = Vec::new();
        for n in needs {
            if n.by == t
                || n.firmness == Firmness::Hard
                    && self
                        .p
                        .record_origin(n.id)
                        .is_some_and(|o| !matches!(o, Origin::Scheme { .. }))
            {
                continue;
            }
            if let Some(bp) = self.plans.get(&n.by)
                && !self.hard.contains(&n.by)
                && bp.scheme.as_str() != SchemeName::COPY
                && !matches!(bp.selection, Selection::Claimed { .. })
                && !self.rej.0.contains_key(&(n.by, bp.scheme.clone()))
                && !out.iter().any(|x| x.0 == n.by)
            {
                out.push((n.by, bp.scheme.clone()));
            }
        }
        out.sort();
        out
    }

    /// The strict sub-entities of `t` a requirement may be about.
    fn children_of(&self, t: Target) -> Result<BTreeSet<Target>> {
        children_in(self.input, &self.region, t)
    }

    fn require(&mut self, tp: &mut TargetPlan) -> Result<()> {
        let s = self
            .drv
            .registry
            .get(tp.target.dim(), tp.scheme.as_str())
            .ok_or_else(|| SchemeError::UnknownScheme {
                name: tp.scheme.clone(),
                target: tp.target,
            })?;
        let (input, region) = (self.input, &self.region);
        let in_region = |t: Target| region.contains(t);
        let within = |parent: Target, child: Target| children_in(input, region, parent).is_ok_and(|c| c.contains(&child));
        let scope = Scope {
            children: self.children_of(tp.target)?,
            resolved: &self.plans,
            in_region: &in_region,
            within: &within,
        };
        let n_claims = self.p.claims.len();
        let mut req = Requirements::new(&mut self.p, &tp.scheme, tp.target, &scope);
        let r = s.requirements(&self.cx, tp.target, &tp.params, &mut req);
        let f = req.finish();
        r?;
        if let Some(d) = f? {
            tp.data = d;
        }
        for c in &self.p.claims[n_claims..] {
            self.claimed.entry(c.item.child).or_insert(c.item.by);
        }
        self.p.invalidate();
        Ok(())
    }

    // ------------------------------------------------------------------ corners and intervals

    fn pick_corners(&self) -> Result<Vec<FaceLayout>> {
        let p = &self.p;
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
                        return Err(Infeasible {
                            reason: format!("conflicting corner rules {have:?} and {rule:?}"),
                            constraints: ids.clone(),
                            entities: vec![face],
                            blame: None,
                        }
                        .into());
                    }
                }
            }
        }
        let mut out = Vec::with_capacity(rules.len());
        for (face, (rule, mut ids)) in rules {
            let reqs: Vec<(CornerKey, Firm<VertexType>)> = p
                .vertex_types
                .iter()
                .filter(|r| r.item.0.face == face)
                .map(|r| r.item)
                .collect();
            ids.extend(
                p.vertex_types
                    .iter()
                    .filter(|r| r.item.0.face == face)
                    .map(|r| r.id),
            );
            out.push(
                self.drv
                    .picker
                    .pick(self.input.topo, face, rule, &reqs, &ids)?,
            );
        }
        Ok(out)
    }

    fn sizing_constraints(&mut self) -> Result<()> {
        let topo = self.input.topo;
        let intent = self.intent;
        // Requests cascade volume -> face -> edge; the lowest id wins within a level.
        let mut req: BTreeMap<EntityId, Firm<IntervalRequest>> = BTreeMap::new();
        for v in self.region.volumes.iter().rev() {
            if let Some(r) = intent.entities.get(v).and_then(|e| e.intervals) {
                for f in self
                    .region
                    .vol_faces
                    .get(v)
                    .map(Vec::as_slice)
                    .unwrap_or(&[])
                {
                    for e in self.input.face_edges(*f) {
                        req.insert(e, r);
                    }
                }
            }
        }
        for f in self.region.faces.iter().rev() {
            if let Some(r) = intent.entities.get(f).and_then(|e| e.intervals) {
                for e in self.input.face_edges(*f) {
                    req.insert(e, r);
                }
            }
        }
        for e in &self.region.edges {
            if let Some(r) = intent.entities.get(e).and_then(|x| x.intervals) {
                req.insert(*e, r);
            }
        }
        let owned = if self.drv.size.is_none() {
            field_for(intent.defaults.sizing.as_ref())?
        } else {
            None
        };
        let field: Option<&dyn SizeField> = self.drv.size.or(owned.as_deref());
        for &e in &self.region.edges {
            let c = match req.get(&e) {
                Some(Firm {
                    value: IntervalRequest::Count(n),
                    firmness,
                }) => (
                    Origin::Intent,
                    IntervalConstraint::Fixed {
                        edge: e,
                        count: *n,
                        firmness: *firmness,
                    },
                ),
                Some(Firm {
                    value: IntervalRequest::Size(h),
                    firmness,
                }) => {
                    let goal = geom::edge_length(topo, e)? / h;
                    match firmness {
                        // A firm size is a firm count: keep its firmness (it used to be dropped).
                        Firmness::Hard | Firmness::Soft => (
                            Origin::Intent,
                            IntervalConstraint::Fixed {
                                edge: e,
                                count: size_count(goal),
                                firmness: *firmness,
                            },
                        ),
                        Firmness::Default => (Origin::Intent, IntervalConstraint::Goal { edge: e, goal }),
                    }
                }
                None => match field {
                    Some(f) => (
                        Origin::Sizing,
                        IntervalConstraint::Goal {
                            edge: e,
                            goal: geom::edge_size_goal(topo, e, f)?,
                        },
                    ),
                    None => (
                        Origin::Sizing,
                        IntervalConstraint::Goal { edge: e, goal: 1.0 },
                    ),
                },
            };
            self.p.push_interval(c.0, c.1);
        }
        // User relations between edges, in intent order.
        for r in &intent.intervals {
            if let Some(e) = r.edges().into_iter().find(|e| self.region.edges.binary_search(e).is_err()) {
                return Err(SchemeError::Input(format!("interval row names {e}, which is not an edge of the region")));
            }
            let mut r = r.clone();
            r.canonicalize();
            self.p.push_interval(Origin::Intent, r);
        }
        Ok(())
    }

    // ------------------------------------------------------------------ transforms

    fn plan_transforms(&self) -> Result<Vec<TransformPlan>> {
        let mut out: Vec<TransformPlan> = Vec::new();
        for t in &self.intent.transforms {
            let mut targets: Vec<Target> = Vec::new();
            for e in &t.targets {
                // Pieces (their group's patch for a grouped one), and faces and curves inside
                // the region (W0f gaps 2: smoothing a surface before a sweep, surface meshes).
                let target = if self.region.volumes.binary_search(e).is_ok() {
                    match self.region.grouped.get(e) {
                        Some(g) => Target::Group(*g),
                        None => Target::Volume(*e),
                    }
                } else if self.region.faces.binary_search(e).is_ok() {
                    Target::Surface(*e)
                } else if self.region.edges.binary_search(e).is_ok() {
                    Target::Curve(*e)
                } else {
                    return Err(SchemeError::Input(format!(
                        "transform {:?}: {e} is not a selected piece, nor a face or curve of the region",
                        t.id
                    )));
                };
                targets.push(target);
            }
            out.push(TransformPlan {
                id: t.id,
                targets: sorted(targets),
                transform: t.transform.clone(),
            });
        }
        for g in &self.intent.groups {
            if let Some(conv) = &g.conversion {
                out.push(TransformPlan {
                    id: TransformId(TransformId::GROUP_BASE | g.id.0),
                    targets: vec![Target::Group(g.id)],
                    transform: conv.clone(),
                });
            }
        }
        out.sort_by_key(|t| t.id);
        for t in &out {
            if self
                .drv
                .registry
                .transform(t.transform.name.as_str())
                .is_none()
            {
                return Err(SchemeError::UnknownScheme {
                    name: t.transform.name.clone(),
                    target: t.targets[0],
                });
            }
        }
        Ok(out)
    }
}

/// Interval count of a firm size request: `round(length / size)`, at least 1.
fn size_count(goal: f64) -> u32 {
    if goal.is_finite() { goal.round().clamp(1.0, f64::from(u32::MAX)) as u32 } else { 1 }
}
