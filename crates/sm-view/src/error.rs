//! Typed errors of the view queries.

use sm_core::EntityId;
use sm_op_api::OpError;

/// Why a view query or [`skin`](crate::skin) call failed. Messages name ids and a fix.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum ViewError {
    /// The state has no mesh.
    #[error("the state has no mesh: run `mesh.run` first")]
    NoMesh,
    /// `mesh` was omitted and the state has several meshes.
    #[error("`mesh` is required, the state has {} meshes: {}", .0.len(), join_ids(.0))]
    AmbiguousMesh(Vec<EntityId>),
    /// A named mesh is not in the state.
    #[error("unknown mesh {0}: list meshes with `view.scene`")]
    UnknownMesh(EntityId),
    /// The metric name is not a cell metric of `sm-quality`.
    #[error("unknown metric `{name}`; valid names: {}", .valid.join(", "))]
    UnknownMetric {
        /// The name that was asked for.
        name: String,
        /// Every valid metric name.
        valid: Vec<String>,
    },
    /// The skin would exceed `max_tris`; nothing is decimated silently.
    #[error("the skin has {tris} triangles, above max_tris {max}: {hint}")]
    TooLarge {
        /// Triangles the skin would have.
        tris: u64,
        /// The limit that applied.
        max: u32,
        /// What to do about it.
        hint: &'static str,
    },
    /// The clip plane is not usable (zero or non-finite normal, non-finite origin).
    #[error("bad clip plane: {0}")]
    BadClip(String),
    /// The metric is defined for no cell shape of the mesh.
    #[error("metric `{0}` is not defined for any cell shape of this mesh")]
    Unsupported(String),
    /// A mesh or model layer failure.
    #[error("{0}")]
    Mesh(String),
}

/// Hint carried by [`ViewError::TooLarge`].
pub const TOO_LARGE_HINT: &str = "clip, pass `volumes`, or raise max_tris";

fn join_ids(ids: &[EntityId]) -> String {
    ids.iter().map(ToString::to_string).collect::<Vec<_>>().join(", ")
}

impl From<ViewError> for OpError {
    fn from(e: ViewError) -> Self {
        match e {
            ViewError::UnknownMesh(id) => OpError::UnknownEntity(id),
            ViewError::TooLarge { .. } | ViewError::Mesh(_) => OpError::Failed(e.to_string()),
            _ => OpError::Invalid(e.to_string()),
        }
    }
}
