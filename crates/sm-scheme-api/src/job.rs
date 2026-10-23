//! Mesh jobs: the unit of work, scheduling and atomic commit.
//!
//! A [`MeshJob`] covers a **set** of targets (one volume, a multisweep group, a merged-volume
//! cluster, a deck-paired face) and declares a [`Footprint`]: what it reads (must be committed
//! before it runs) and what it writes (exactly once, by exactly one job). Jobs whose footprints do
//! not conflict run concurrently; their [`crate::MeshPatch`]es are committed in `JobId` order.

use crate::intent::{GroupKind, SchemeSpec};
use crate::target::Target;
use sm_core::{EntityId, Transform};
use sm_topo_api::Sense;
use std::collections::{BTreeMap, BTreeSet};

/// Dense job id: the index in [`crate::MeshProblem::jobs`]. Jobs are sorted by their first write
/// target, so ids are a pure function of the intent and the model.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct JobId(pub u32);

/// Read and write sets of a job.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Footprint {
    /// Targets whose committed meshes the job reads.
    pub reads: BTreeSet<Target>,
    /// Targets the job creates meshes for (all or nothing).
    pub writes: BTreeSet<Target>,
}

impl Footprint {
    /// Builder: add reads.
    pub fn read(mut self, ts: impl IntoIterator<Item = Target>) -> Self {
        self.reads.extend(ts);
        self
    }
    /// Builder: add writes.
    pub fn write(mut self, ts: impl IntoIterator<Item = Target>) -> Self {
        self.writes.extend(ts);
        self
    }
    /// True if two jobs may run in the same wave: neither writes what the other reads or writes.
    pub fn independent(&self, o: &Footprint) -> bool {
        self.writes.is_disjoint(&o.writes) && self.writes.is_disjoint(&o.reads) && o.writes.is_disjoint(&self.reads)
    }
}

/// Correspondence between the entities of a copy source and its target.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct EntityMap {
    /// Source entity -> (target entity, `Reversed` if the target edge runs against the image of
    /// the source edge). Vertices, edges and the face itself.
    pub pairs: BTreeMap<EntityId, (EntityId, Sense)>,
}

impl EntityMap {
    /// The image of a source entity.
    pub fn get(&self, e: EntityId) -> Option<(EntityId, Sense)> {
        self.pairs.get(&e).copied()
    }
}

/// Why a copy job exists.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CopyReason {
    /// A translation/rotation deck pair: side B must be the image of side A.
    Deck {
        /// The boundary record.
        boundary: EntityId,
    },
    /// A user copy-mesh request.
    User,
}

/// A copy (or mirror) job: the target's mesh is the image of the committed source mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct CopySpec {
    /// Source face (or edge).
    pub source: Target,
    /// Target face (or edge).
    pub target: Target,
    /// Transform taking source points to target points (improper = mirror).
    pub transform: Transform,
    /// Entity correspondence.
    pub map: EntityMap,
    /// Target edges this job writes as images of source edges (edges not shared with the source
    /// and not written by another job).
    pub edges: Vec<EntityId>,
    /// Origin.
    pub reason: CopyReason,
}

/// What a job does.
#[derive(Clone, Debug, PartialEq)]
pub enum JobKind {
    /// Run a registered scheme.
    Scheme(SchemeSpec),
    /// Copy a committed mesh under a transform (built into the driver).
    Copy(Box<CopySpec>),
    /// Rewrite already committed targets (THex, QTri, boundary layers); may replace their meshes.
    Transform(SchemeSpec),
}

/// Role of a face in a sweep job.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum SweepRole {
    /// Meshed before the sweep (read).
    Source,
    /// Meshed by the sweep as the image of the source (written).
    Target,
    /// Meshed by the sweep layer by layer; must admit a mapped/submapped mesh (written).
    Linking,
}

/// One job.
#[derive(Clone, Debug, PartialEq)]
pub struct MeshJob {
    /// Id (index in the problem's job list).
    pub id: JobId,
    /// What runs.
    pub kind: JobKind,
    /// The targets the job was created for, ascending (the set).
    pub targets: Vec<Target>,
    /// Group kind, for multi-target jobs.
    pub group: Option<GroupKind>,
    /// Read and write sets.
    pub footprint: Footprint,
    /// Roles the planning scheme assigned to targets (sweep source/target/linking).
    pub roles: BTreeMap<Target, SweepRole>,
}

impl MeshJob {
    /// The scheme name, for `Scheme` and `Transform` jobs.
    pub fn scheme(&self) -> Option<&SchemeSpec> {
        match &self.kind {
            JobKind::Scheme(s) | JobKind::Transform(s) => Some(s),
            JobKind::Copy(_) => None,
        }
    }
    /// First write target (the sort key of job ids).
    pub fn key(&self) -> Option<Target> {
        self.footprint.writes.first().copied()
    }
}
