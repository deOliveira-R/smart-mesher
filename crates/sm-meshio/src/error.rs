//! Typed errors for the exporters and the `.smsh` reader.

use sm_mesh_api::MeshError;

/// Errors from writing or reading mesh files.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum MeshioError {
    /// An I/O error from the underlying writer or file.
    #[error("i/o error: {0}")]
    Io(#[from] std::io::Error),
    /// The JSON header could not be produced or parsed.
    #[error("header json: {0}")]
    Json(#[from] serde_json::Error),
    /// The input is not a valid `.smsh` file (bad magic, truncated, bad offsets, bad checksum).
    #[error("malformed .smsh: {0}")]
    Malformed(String),
    /// The file's format version is newer than this reader understands.
    #[error("unsupported .smsh version {major}.{minor}")]
    UnsupportedVersion {
        /// Major version found in the file.
        major: u32,
        /// Minor version found in the file.
        minor: u32,
    },
    /// The mesh uses a feature the format version cannot represent (for example a shape or
    /// parameter variant added to the mesh API after this crate was written).
    #[error("unsupported: {0}")]
    Unsupported(String),
    /// Attached metadata (decks, hierarchy maps) is inconsistent with the mesh.
    #[error("inconsistent metadata: {0}")]
    Inconsistent(String),
    /// An OpenFOAM export, parse or check error.
    #[error(transparent)]
    Foam(#[from] crate::foam::FoamError),
    /// A mesh-API call failed.
    #[error(transparent)]
    Mesh(#[from] MeshError),
}

/// Result alias.
pub type Result<T> = core::result::Result<T, MeshioError>;
