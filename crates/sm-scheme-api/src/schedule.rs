//! Footprints and waves (defects D6, grafts G1 and G10). A job's footprint is derived from the
//! plan, never written by hand; waves are Kahn levels of the dependency graph, so "face after
//! volume" (a deck copy of a claimed face) is as expressible as "face before volume".

use crate::error::{Result, SchemeError};
use crate::plan::{CopyClass, CopyMode, Footprint, TransformPlan};
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

/// The children a target reads: faces of a piece or group, edges of a face, and the embedded
/// (wire) edges of a face or piece (W0f gaps 2), which are generated before their parent.
fn children(inp: &ScheduleInput<'_>, t: Target) -> Result<Vec<Target>> {
    let embedded = |p: EntityId| inp.input.embedded_edges(p).into_iter().map(Target::Curve);
    Ok(match t {
        Target::Curve(_) => Vec::new(),
        Target::Surface(f) => inp
            .input
            .face_edges(f)
            .into_iter()
            .map(Target::Curve)
            .chain(embedded(f))
            .collect(),
        Target::Volume(v) => inp
            .input
            .piece_faces(t)?
            .into_iter()
            .map(Target::Surface)
            .chain(embedded(v))
            .collect(),
        Target::Group(g) => {
            let mut v = Vec::new();
            for m in inp.groups.get(&g).map(Vec::as_slice).unwrap_or(&[]) {
                v.extend(
                    inp.input
                        .piece_faces(Target::Volume(*m))?
                        .into_iter()
                        .map(Target::Surface),
                );
                v.extend(embedded(*m));
            }
            v
        }
    })
}

/// Footprints (in job order) and waves.
pub(crate) fn schedule(inp: &ScheduleInput<'_>) -> Result<(Vec<Footprint>, Vec<Vec<Job>>)> {
    let mut first_claim: BTreeMap<Target, (Target, ConstraintId)> = BTreeMap::new();
    for c in &inp.problem.claims {
        match first_claim.get(&c.item.child) {
            Some((by, id)) if *by != c.item.by => {
                return Err(SchemeError::Schedule {
                    jobs: vec![Job::Generate(*by), Job::Generate(c.item.by)],
                    constraints: vec![*id, c.id],
                    reason: format!("{:?} is written by two jobs", c.item.child),
                });
            }
            Some(_) => {}
            None => {
                first_claim.insert(c.item.child, (c.item.by, c.id));
            }
        }
    }
    let claims: BTreeMap<Target, Target> =
        first_claim.iter().map(|(c, (by, _))| (*c, *by)).collect();
    let mut claims_by: BTreeMap<Target, Vec<Target>> = BTreeMap::new();
    for (c, by) in &claims {
        claims_by.entry(*by).or_default().push(*c);
    }
    // Jobs: every planned target that is not claimed, then the transforms.
    let mut jobs: Vec<Job> = inp
        .plans
        .values()
        .filter(|p| !matches!(p.selection, Selection::Claimed { .. }))
        .map(|p| Job::Generate(p.target))
        .collect();
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
        .flat_map(|(ci, c)| {
            c.members
                .iter()
                .enumerate()
                .map(move |(mi, m)| (m.target, (ci, mi)))
        })
        .collect();

    let mut footprints: Vec<Footprint> = Vec::with_capacity(jobs.len());
    let mut deps: BTreeMap<Job, BTreeSet<Job>> = BTreeMap::new();
    let mut written_by: BTreeMap<Target, Job> = BTreeMap::new();
    for &job in &jobs {
        let (reads, writes) = match job {
            Job::Generate(t) => {
                let mine: BTreeSet<Target> =
                    claims_by.get(&t).into_iter().flatten().copied().collect();
                let mut reads: Vec<Target> = children(inp, t)?
                    .into_iter()
                    .filter(|c| !mine.contains(c))
                    .collect();
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
                let tp = inp.transforms.iter().find(|x| x.id == id).ok_or_else(|| {
                    SchemeError::Input(format!("transform {id:?} is not planned"))
                })?;
                (sorted(tp.targets.clone()), sorted(tp.targets.clone()))
            }
        };
        if matches!(job, Job::Generate(_)) {
            for w in &writes {
                if let Some(prev) = written_by.insert(*w, job) {
                    let cids: Vec<ConstraintId> = inp
                        .problem
                        .claims
                        .iter()
                        .filter(|c| c.item.child == *w)
                        .map(|c| c.id)
                        .collect();
                    return Err(SchemeError::Schedule {
                        jobs: vec![prev, job],
                        constraints: cids,
                        reason: format!("{w:?} is written by two jobs"),
                    });
                }
            }
        }
        footprints.push(Footprint { job, reads, writes });
    }
    // The last transform over a target (by id) is the writer its readers depend on (W0f gaps 2):
    // a volume reads its smoothed faces, a copy member its transformed root.
    let mut last_transform: BTreeMap<Target, Job> = BTreeMap::new();
    for tp in inp.transforms {
        for t in &tp.targets {
            let j = Job::Transform(tp.id);
            match last_transform.get(t) {
                Some(Job::Transform(prev)) if *prev > tp.id => {}
                _ => {
                    last_transform.insert(*t, j);
                }
            }
        }
    }
    for fp in &footprints {
        let mut d: BTreeSet<Job> = BTreeSet::new();
        for r in &fp.reads {
            let w = match fp.job {
                Job::Generate(_) => last_transform.get(r).copied().or_else(|| writer(*r)),
                Job::Transform(_) => writer(*r),
            };
            match w {
                Some(w) if w != fp.job => {
                    d.insert(w);
                }
                Some(_) => {}
                None => {
                    return Err(SchemeError::Schedule {
                        jobs: vec![fp.job],
                        constraints: Vec::new(),
                        reason: format!("{:?} reads {r:?}, which no job writes", fp.job),
                    });
                }
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
        let (ready, rest): (Vec<Job>, Vec<Job>) = remaining
            .iter()
            .partition(|j| deps[*j].iter().all(|d| done.contains(d)));
        if ready.is_empty() {
            let involved = |t: Target| writer(t).is_some_and(|w| remaining.contains(&w));
            let mut cids: Vec<ConstraintId> = Vec::new();
            cids.extend(
                inp.problem
                    .copies
                    .iter()
                    .filter(|c| involved(c.item.source) && involved(c.item.target))
                    .map(|c| c.id),
            );
            cids.extend(
                inp.problem
                    .claims
                    .iter()
                    .filter(|c| involved(c.item.child) && involved(c.item.by))
                    .map(|c| c.id),
            );
            cids.extend(
                precedes
                    .iter()
                    .filter(|p| involved(p.0) && involved(p.1))
                    .map(|p| p.2),
            );
            cids.sort_unstable();
            return Err(SchemeError::Schedule {
                jobs: remaining,
                constraints: cids,
                reason: "dependency cycle".into(),
            });
        }
        done.extend(ready.iter().copied());
        waves.push(ready);
        remaining = rest;
    }
    Ok((footprints, waves))
}
