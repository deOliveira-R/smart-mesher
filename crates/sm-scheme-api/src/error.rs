//! Typed errors. [`SchemeFailure`] is what a scheme returns; its [`Retry`] tells the driver
//! whether to fall back. [`DriverError`] is what the driver returns.

use crate::intent::SchemeName;
use crate::interval::IntervalError;
use crate::job::JobId;
use crate::target::{NodeKey, Target};
use sm_mesh_api::MeshError;
use sm_topo_api::QueryError;

/// What went wrong inside a scheme.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FailureKind {
    /// The input does not fit the scheme (wrong corner count, not sweepable, ...).
    NotApplicable,
    /// Topology the scheme cannot handle (missing edge meshes, unexpected loops).
    Topology,
    /// A geometry query failed (projection, evaluation, unsupported carrier).
    Geometry,
    /// The solved intervals do not fit the scheme.
    Intervals,
    /// The result failed a quality or validity check (inverted elements).
    Quality,
    /// A feature this scheme does not implement yet.
    Unsupported,
    /// A bug or broken invariant.
    Internal,
}

/// What the driver may do after a failure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Retry {
    /// Exclude this scheme for the job's targets and re-plan (next in the chain, or the next auto
    /// candidate). Interval counts are re-solved, because the next scheme brings its own
    /// constraints.
    NextInChain,
    /// Stop: no other scheme can help (bad input, internal error).
    Fatal,
}

/// A scheme's structured failure, carried up so the driver can fall back.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[error("scheme '{scheme}' failed on {targets:?} ({kind:?}): {message}")]
pub struct SchemeFailure {
    /// The scheme.
    pub scheme: SchemeName,
    /// The job's targets.
    pub targets: Vec<Target>,
    /// Category.
    pub kind: FailureKind,
    /// Whether falling back makes sense.
    pub retry: Retry,
    /// Human-readable detail.
    pub message: String,
}

impl SchemeFailure {
    /// A failure that allows fallback.
    pub fn fallback(scheme: &str, targets: &[Target], kind: FailureKind, message: impl Into<String>) -> Self {
        Self { scheme: SchemeName::new(scheme), targets: targets.to_vec(), kind, retry: Retry::NextInChain, message: message.into() }
    }
    /// A failure that stops meshing.
    pub fn fatal(scheme: &str, targets: &[Target], kind: FailureKind, message: impl Into<String>) -> Self {
        Self { scheme: SchemeName::new(scheme), targets: targets.to_vec(), kind, retry: Retry::Fatal, message: message.into() }
    }
}

/// One rejected scheme candidate, kept for diagnostics.
#[derive(Clone, Debug, PartialEq)]
pub struct Attempt {
    /// The targets.
    pub target: Target,
    /// The candidate.
    pub scheme: SchemeName,
    /// Why it was rejected.
    pub reason: String,
}

/// A patch broke the commit rules (see [`crate::Ledger::commit`]).
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum PatchError {
    /// A part or a new node writes a target outside the job's write footprint.
    #[error("job writes {0:?} outside its footprint")]
    OutsideFootprint(Target),
    /// A target in the write footprint got no part.
    #[error("job did not write {0:?}")]
    Missing(Target),
    /// The target was already written by an earlier job.
    #[error("{0:?} already written")]
    AlreadyWritten(Target),
    /// A referenced node is neither new in the patch nor committed.
    #[error("unknown node {0:?}")]
    UnknownNode(NodeKey),
    /// A new node collides with a committed one, or a key repeats in the patch.
    #[error("node {0:?} defined twice")]
    DuplicateNode(NodeKey),
    /// Malformed CSR, out-of-range index, non-finite point or wrong corner count.
    #[error("malformed patch: {0}")]
    Malformed(String),
    /// An edge mesh does not start and end at the edge's vertex nodes.
    #[error("edge mesh of {0:?} does not run from its start vertex to its end vertex")]
    EdgeEnds(Target),
}

/// Errors of the driver.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum DriverError {
    /// A scheme name is not registered.
    #[error("unknown scheme '{0}'")]
    UnknownScheme(SchemeName),
    /// No candidate was applicable and planned for a target.
    #[error("no applicable scheme for {target:?} ({} rejected)", attempts.len())]
    NoApplicableScheme {
        /// The target.
        target: Target,
        /// Rejected candidates.
        attempts: Vec<Attempt>,
    },
    /// Two requirements of equal (hard) firmness disagree.
    #[error("conflicting requirements on {target:?}: {detail}")]
    Conflict {
        /// The target.
        target: Target,
        /// What disagrees.
        detail: String,
    },
    /// Two jobs declare the same write target.
    #[error("{target:?} written by jobs {a:?} and {b:?}")]
    WriteConflict {
        /// The target.
        target: Target,
        /// First job.
        a: JobId,
        /// Second job.
        b: JobId,
    },
    /// A job reads a target nobody writes.
    #[error("job {job:?} reads {target:?}, which no job writes")]
    MissingWriter {
        /// The job.
        job: JobId,
        /// The target.
        target: Target,
    },
    /// The read/write graph has a cycle.
    #[error("job dependency cycle")]
    Cycle,
    /// The interval solve failed.
    #[error(transparent)]
    Intervals(#[from] IntervalError),
    /// A scheme failed while planning or executing.
    #[error("job {job:?}: {failure}")]
    JobFailed {
        /// The job (`None` while planning).
        job: Option<JobId>,
        /// The failure.
        failure: SchemeFailure,
    },
    /// The fallback loop ran out of attempts.
    #[error("fallback limit reached after {0} attempts")]
    FallbackLimit(usize),
    /// A committed patch broke the rules.
    #[error("job {job:?}: {error}")]
    Patch {
        /// The job.
        job: JobId,
        /// The broken rule.
        error: PatchError,
    },
    /// A deck pair's meshes are not images of each other.
    #[error("deck copy violated: {target:?} is not the image of {source_target:?}")]
    DeckMismatch {
        /// Side-A target.
        source_target: Target,
        /// Side-B target.
        target: Target,
    },
    /// A topology query failed.
    #[error(transparent)]
    Topology(#[from] QueryError),
    /// Writing to the mesh database failed.
    #[error(transparent)]
    Mesh(#[from] MeshError),
    /// A target or node cannot be expressed in the mesh database (e.g. classification on an
    /// overlay `Partition*` id).
    #[error("cannot write {0:?}: {1}")]
    Unwritable(Target, String),
    /// Malformed intent (non-finite parameter, empty group, unknown target).
    #[error("invalid intent: {0}")]
    Intent(String),
}
