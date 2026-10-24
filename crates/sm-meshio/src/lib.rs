//! Mesh export for smart-mesh: the native `.smsh` container and VTU (VTK XML unstructured grid).
//!
//! Both writers read the mesh through [`sm_mesh_api::MeshRead`] only and produce output that is
//! a pure function of the mesh: byte-identical across runs and thread counts.
//!
//! * [`smsh`]: the native format (`SMSH_FORMAT.md`), writer and reader.
//! * [`vtu`]: ParaView / VTK export with polyhedra, mixed cells and region/color cell data.
//!
//! See `DESIGN.md`.

mod crc;
pub mod error;
mod le;
pub mod smsh;
pub mod vtu;

pub use error::{MeshioError, Result};
pub use smsh::{DeckParams, DeckRecord, SmshFile, SmshMesh, SmshWriter};
pub use vtu::{VtuEncoding, VtuOptions, write_vtu, vtu_bytes};
