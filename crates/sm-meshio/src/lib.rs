//! Mesh export for smart-mesh: the native `.smsh` container and VTU (VTK XML unstructured grid).
//!
//! Both writers read the mesh through [`sm_mesh_api::MeshRead`] only and produce output that is
//! a pure function of the mesh: byte-identical across runs and thread counts.
//!
//! * [`smsh`]: the native format (`SMSH_FORMAT.md`), writer and reader.
//! * [`foam`]: OpenFOAM polyMesh (ASCII) writer, parser and checker.
//! * [`dagmc`]: DAGMC `.h5m` (MOAB HDF5) faceted B-rep writer (`H5M_FORMAT.md`).
//! * [`gmsh`]: Gmsh MSH 4.1 writer, ASCII and binary, and a round-trip parser.
//! * [`report`]: the loss report every exporter shares.
//! * [`vtu`]: ParaView / VTK export with polyhedra, mixed cells and region/color cell data.
//!
//! See `DESIGN.md`.

mod crc;
pub mod dagmc;
pub mod error;
pub mod foam;
pub mod gmsh;
#[cfg(feature = "testing")]
#[doc(hidden)]
pub mod h5;
#[cfg(not(feature = "testing"))]
mod h5;
mod le;
pub mod report;
pub mod select;
pub mod smsh;
pub mod vtu;

pub use error::{MeshioError, Result};
pub use report::{LossItem, LossReport, LossSeverity, vtu_loss_report};
pub use smsh::{DeckParams, DeckRecord, ElementTypeInfo, MeshStats, SmshFile, SmshMesh, SmshWriter, element_type_shape};
pub use vtu::{VtuEncoding, VtuOptions, write_vtu, vtu_bytes};
