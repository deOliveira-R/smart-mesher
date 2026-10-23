//! Footprints and waves (defects D6, grafts G1 and G10). A job's footprint is derived from the
//! plan, never written by hand; waves are Kahn levels of the dependency graph, so "face after
//! volume" (a deck copy of a claimed face) is as expressible as "face before volume".

use crate::error::{Result, SchemeError};
use crate::plan::{CopyMode, CopyClass, Footprint, TransformPlan};
use crate::problem::{ConstraintId, MeshConstraint, MeshingProblem, Selection, TargetPlan};
use crate::scheme::MeshInput;
use crate::target::{GroupId, Job, Target};
use sm_core::EntityId;
use std::collections::{BTreeMap, BTreeSet};

pub(crate) struct ScheduleInput<'a> {
    pub input: &'a MeshInput<'a>,
    pub plans: &'a BTreeMap<Target, TargetPlan>,
    pub problem: &'a MeshingProblem,
    pub classes: &'a [CopyClass],
    pub transforms: &'a [TransformPlan],
    /// Members of every group.
    pub groups: &'a BTreeMap<GroupId, Vec<EntityId>>,
    /// Group of every grouped volume.
    pub grouped: &'a BTreeMap<EntityId, GroupId>,
}

fn sorted(mut v: Vec<Target>) -> Vec<Target> {
    v.sort_unstable();
    v.dedup();
    v
}

/// The boundary children a target reads: faces of a piece or group, edges of a face.
fn children(inp: &ScheduleInput<'_>, t: Target) -> Result<Vec<Target>> {
    Ok(match t {
        Target::Curve(_) => Vec::new(),
        Target::Surface(f) => inp.input.face_edges(f).into_iter().map(Target::Curve).collect(),
        Target::Volume(_) => inp.input.piece_faces(t)?.into_iter().map(Target::Surface).collect(),
        Target::Group(g) => {
            let mut v = Vec::new();
            for m in inp.groups.get(&g).map(Vec::as_slice).unwrap_or(&[]) {
                v.extend(inp.input.piece_faces(Target::Volume(*m))?.into_iter().map(Target::Surface));
            }
            v
        }
    })
}

/// Footprints (in job order) and waves.
pub(crate) fn schedule(inp: &ScheduleInput<'_>) -> Result<(Vec<Footprint>, Vec<Vec<Job>>)> {
    let claims: BTreeMap<Target, Target> = inp.problem.claims.iter().rev().map(|c| (c.item.child, c.item.by)).collect();
    let mut claims_by: BTreeMap<Target, Vec<Target>> = BTreeMap::new();
    for (c, by) in &claims {
        claims_by.entry(*by).or_default().push(*c);
    }
    // Jobs: every planned target that is not claimed, then the transforms.
    let mut jobs: Vec<Job> = inp.plans.values().filter(|p| !matches!(p.selection, Selection::Claimed { .. })).map(|p| Job::Generate(p.target)).collect();
    jobs.extend(inp.transforms.iter().map(|t| Job::Transform(t.id)));
    jobs.sort_unstable();

    let writer = |x: Target| -> Option<Job> {
        if let Some(by) = claims.get(&x) {
            return Some(Job::Generate(*by));
        }
        if let Target::Volume(v) = x
            && let Some(g) = inp.grouped.get(&v)
        {
            return Some(Job::Generate(Target::Group(*g)));
        }
        inp.plans.contains_key(&x).then_some(Job::Generate(x))
    };
    let precedes: Vec<(Target, Target, ConstraintId)> = inp
        .problem
        .constraints
        .iter()
        .filter_map(|r| match &r.item {
            MeshConstraint::Precedes { before, after } => Some((*before, *after, r.id)),
            _ => None,
        })
        .collect();
    let class_of: BTreeMap<Target, (usize, usize)> = inp
        .classes
        .iter()
        .enumerate()
        .flat_map(|(ci, c)| c.members.iter().enumerate().map(move |(mi, m)| (m.target, (ci, mi))))
        .collect();

    let mut footprints: Vec<Footprint> = Vec::with_capacity(jobs.len());
    let mut deps: BTreeMap<Job, BTreeSet<Job>> = BTreeMap::new();
    let mut written_by: BTreeMap<Target, Job> = BTreeMap::new();
    for &job in &jobs {
        let (reads, writes) = match job {
            Job::Generate(t) => {
                let mine: BTreeSet<Target> = claims_by.get(&t).into_iter().flatten().copied().collect();
                let mut reads: Vec<Target> = children(inp, t)?.into_iter().filter(|c| !mine.contains(c)).collect();
                if let Some(&(ci, mi)) = class_of.get(&t)
                    && inp.classes[ci].members[mi].mode == CopyMode::Generate
                {
                    reads.push(inp.classes[ci].root);
                }
                reads.extend(precedes.iter().filter(|p| p.1 == t).map(|p| p.0));
                let mut writes = vec![t];
                writes.extend(mine);
                (sorted(reads), sorted(writes))
            }
            Job::Transform(id) => {
                let tp = inp.transforms.iter().find(|x| x.id == id).ok_or_else(|| SchemeError::Input(format!("transform {id:?} is not planned")))?;
                (sorted(tp.targets.clone()), sorted(tp.targets.clone()))
            }
        };
        if matches!(job, Job::Generate(_)) {
            for w in &writes {
                if let Some(prev) = written_by.insert(*w, job) {
                    let cids: Vec<ConstraintId> = inp.problem.claims.iter().filter(|c| c.item.child == *w).map(|c| c.id).collect();
                    return Err(SchemeError::Schedule { jobs: vec![prev, job], constraints: cids, reason: format!("{w:?} is written by two jobs") });
                }
            }
        }
        footprints.push(Footprint { job, reads, writes });
    }
    for fp in &footprints {
        let mut d: BTreeSet<Job> = BTreeSet::new();
        for r in &fp.reads {
            match writer(*r) {
                Some(w) if w != fp.job => {
                    d.insert(w);
                }
                Some(_) => {}
                None => return Err(SchemeError::Schedule { jobs: vec![fp.job], constraints: Vec::new(), reason: format!("{:?} reads {r:?}, which no job writes", fp.job) }),
            }
        }
        if let Job::Transform(id) = fp.job {
            // Transforms over the same targets run in id order.
            for other in &footprints {
                if let Job::Transform(oid) = other.job
                    && oid < id
                    && other.writes.iter().any(|w| fp.writes.contains(w))
                {
                    d.insert(other.job);
                }
            }
        }
        deps.insert(fp.job, d);
    }
    // Kahn levelling.
    let mut done: BTreeSet<Job> = BTreeSet::new();
    let mut remaining: Vec<Job> = jobs.clone();
    let mut waves: Vec<Vec<Job>> = Vec::new();
    while !remaining.is_empty() {
        let (ready, rest): (Vec<Job>, Vec<Job>) = remaining.iter().partition(|j| deps[*j].iter().all(|d| done.contains(d)));
        if ready.is_empty() {
            let cids: Vec<ConstraintId> = inp
                .problem
                .copies
                .iter()
                .map(|c| c.id)
                .chain(inp.problem.claims.iter().map(|c| c.id))
                .chain(precedes.iter().map(|p| p.2))
                .filter(|id| {
                    let t = inp.problem.copies.iter().find(|c| c.id == *id).map(|c| (c.item.source, c.item.target)).or_else(|| inp.problem.claims.iter().find(|c| c.id == *id).map(|c| (c.item.child, c.item.by)));
                    t.is_some_and(|(a, b)| remaining.contains(&Job::Generate(a)) || remaining.contains(&Job::Generate(b)))
                })
                .collect();
            return Err(SchemeError::Schedule { jobs: remaining, constraints: cids, reason: "dependency cycle".into() });
        }
        done.extend(ready.iter().copied());
        waves.push(ready);
        remaining = rest;
    }
    Ok((footprints, waves))
}
