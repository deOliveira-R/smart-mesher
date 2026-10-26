//! Exodus II writer in pure Rust (own netCDF classic / 64-bit-offset encoder).
pub mod error;
pub mod netcdf;
pub mod options;
pub mod plan;
pub mod read;
pub mod tables;
pub mod write;
pub use error::ExodusError;
pub use options::{BlockSelection, ExodusOptions};
pub use read::{ExodusData, read_exodus};
pub use plan::{ExodusPlan, plan};
pub use tables::{exodus_element_type, exodus_side, local_face};
pub use write::{BlockSummary, ExodusReport, SetSummary, exodus_bytes, exodus_nc, write_exodus};
