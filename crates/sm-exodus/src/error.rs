//! Typed errors of the Exodus writer, the netCDF codec and the internal reader.

use sm_mesh_api::MeshError;

/// Errors from writing or decoding netCDF / Exodus files.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum ExodusError {
    /// An I/O error from the underlying writer.
    #[error("i/o error: {0}")]
    Io(#[from] std::io::Error),
    /// A mesh-API call failed, or the mesh is internally inconsistent (for example a side whose
    /// face loop matches no face of its cell).
    #[error(transparent)]
    Mesh(#[from] MeshError),
    /// The mesh or the input uses a feature this crate does not write or read.
    #[error("unsupported: {0}")]
    Unsupported(String),
    /// A quantity does not fit the file format (or the chosen id width).
    #[error("{what} too large: {value} exceeds {limit}")]
    TooLarge {
        /// What overflowed.
        what: String,
        /// The offending value.
        value: u64,
        /// The largest value the format allows.
        limit: u64,
    },
    /// The mesh has no vertices or no active cells.
    #[error("mesh has no vertices or no active cells")]
    EmptyMesh,
    /// A cell belongs to two block sets.
    #[error("cell {cell} is in both block sets '{a}' and '{b}'")]
    OverlappingBlocks {
        /// Dense cell index.
        cell: u32,
        /// First set name.
        a: String,
        /// Second set name.
        b: String,
    },
    /// A set label cannot be used as an Exodus id (not positive, or used twice).
    #[error("set '{set}' has an unusable label {label}")]
    BadLabel {
        /// Set name.
        set: String,
        /// The label.
        label: i64,
    },
    /// A name exceeds the 255 bytes Exodus allows (or a QA string its 32).
    #[error("name of {} bytes is too long", name.len())]
    NameTooLong {
        /// The offending name.
        name: String,
    },
    /// The netCDF model is inconsistent, or the input bytes are not a valid file.
    #[error("malformed: {0}")]
    Malformed(String),
}

/// Result alias.
pub type Result<T> = core::result::Result<T, ExodusError>;
