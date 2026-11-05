//! The benchmark's typed error.

/// Everything that can go wrong in the harness. Library code returns this instead of panicking
/// on input paths (case files, driver JSON, command line).
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum BenchError {
    /// File system error with the path it concerns.
    #[error("{path}: {source}")]
    Io {
        /// The path.
        path: String,
        /// The cause.
        source: std::io::Error,
    },
    /// A binary case file is malformed or truncated.
    #[error("bad {what}: {why}")]
    Format {
        /// Which format (`smfacets v1`, ...).
        what: &'static str,
        /// What is wrong.
        why: String,
    },
    /// JSON could not be read or written.
    #[error("json: {0}")]
    Json(#[from] serde_json::Error),
    /// Invalid command line or argument.
    #[error("usage: {0}")]
    Usage(String),
    /// Geometry construction or tessellation failed.
    #[error("geometry: {0}")]
    Geometry(String),
    /// The tracker under test failed.
    #[error("tracker: {0}")]
    Tracker(String),
    /// A fairness rule forbids the requested output (mixed machines, repo path for `--model`, ...).
    #[error("fairness rule: {0}")]
    Fairness(String),
    /// The requested tracker, case or mode is not available in this task (RTBENCH-B or Docker).
    #[error("unavailable: {0}")]
    Unavailable(String),
    /// A generated file differs from its recorded hash.
    #[error("hash check failed: {0}")]
    Check(String),
}

impl BenchError {
    /// Wrap an I/O error with its path.
    pub fn io(path: impl AsRef<std::path::Path>, source: std::io::Error) -> Self {
        Self::Io { path: path.as_ref().display().to_string(), source }
    }
}

/// Result alias of the crate.
pub type Result<T> = core::result::Result<T, BenchError>;
