//! Error type of the engine.

use std::path::PathBuf;

use sm_op_api::{Diagnostic, Hash256, RegistryError, TimelineError};

/// Everything the engine can fail with.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum EngineError {
    /// A file-system operation failed.
    #[error("i/o error at {path}: {source}")]
    Io {
        /// The path involved.
        path: PathBuf,
        /// The underlying error.
        #[source]
        source: std::io::Error,
    },
    /// Timeline, replay or node-format error from `sm-op-api`.
    #[error(transparent)]
    Timeline(#[from] TimelineError),
    /// Registry, plugin or query error from `sm-op-api`.
    #[error(transparent)]
    Registry(#[from] RegistryError),
    /// A project file is malformed.
    #[error("project format: {0}")]
    Format(String),
    /// The project was written by a newer, incompatible version.
    #[error("project format version {found} is newer than the supported version {supported}")]
    UnsupportedFormat {
        /// Version found in `project.toml`.
        found: u32,
        /// Highest version this build reads.
        supported: u32,
    },
    /// The directory has no `project.toml`.
    #[error("not a smart-mesh project: {0}")]
    NotAProject(PathBuf),
    /// `create` found an existing project or a non-empty directory.
    #[error("{0} already exists and is not an empty directory")]
    AlreadyExists(PathBuf),
    /// Another engine holds the project's write lock.
    #[error("project {0} is open in another engine")]
    Locked(PathBuf),
    /// The workspace was opened read-only.
    #[error("workspace is read-only")]
    ReadOnly,
    /// The operation needs a project directory but the workspace is in-memory.
    #[error("workspace has no project directory")]
    NoProject,
    /// The diagnostics policy refused to record the op because it warned. Nothing was recorded
    /// and the state is unchanged; repeat with `accept_warnings` to record it anyway.
    #[error("op stopped by policy: {} warning(s), first: {}", .diagnostics.len(), .diagnostics.first().map_or("", |d| d.code.as_str()))]
    StoppedOnWarning {
        /// The warnings that blocked the op (suppressed codes excluded).
        diagnostics: Vec<Diagnostic>,
    },
    /// The op is in the timeline but saving it to disk failed. State and timeline are advanced;
    /// [`Workspace::save`](crate::Workspace::save) retries.
    #[error("recorded in memory (node {}) but not saved: {source}", .node.map_or_else(|| "-".to_owned(), |h| h.to_string()))]
    NotPersisted {
        /// The recorded node (when the failing step recorded one).
        node: Option<Hash256>,
        /// Why saving failed.
        #[source]
        source: Box<EngineError>,
    },
    /// A request (JSON from a front end) is malformed.
    #[error("bad request: {0}")]
    BadRequest(String),
}

impl EngineError {
    /// Stable kebab-case code for protocols and front ends (inner codes for wrapped errors).
    pub fn code(&self) -> &'static str {
        match self {
            EngineError::Io { .. } => "io",
            EngineError::Timeline(e) => e.code(),
            EngineError::Registry(e) => e.code(),
            EngineError::Format(_) => "project-format",
            EngineError::UnsupportedFormat { .. } => "unsupported-format",
            EngineError::NotAProject(_) => "not-a-project",
            EngineError::AlreadyExists(_) => "already-exists",
            EngineError::Locked(_) => "locked",
            EngineError::ReadOnly => "read-only",
            EngineError::NoProject => "no-project",
            EngineError::StoppedOnWarning { .. } => "stopped-on-warning",
            EngineError::NotPersisted { .. } => "not-persisted",
            EngineError::BadRequest(_) => "bad-request",
        }
    }

    pub(crate) fn io(path: impl Into<PathBuf>, source: std::io::Error) -> Self {
        EngineError::Io {
            path: path.into(),
            source,
        }
    }
}

impl From<sm_op_api::ApplyError> for EngineError {
    fn from(e: sm_op_api::ApplyError) -> Self {
        EngineError::Timeline(TimelineError::Apply(e))
    }
}

/// Result alias.
pub type Result<T> = std::result::Result<T, EngineError>;
