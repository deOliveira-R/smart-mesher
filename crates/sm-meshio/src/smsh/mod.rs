//! The native `.smsh` container: a JSON header followed by 64-byte-aligned little-endian
//! arrays (see `SMSH_FORMAT.md` for the byte-level specification).

mod format;
mod reader;
mod writer;

pub use format::{
    ALIGN, ArrayInfo, DeckParams, DeckRecord, Dtype, ExtrusionInfo, GeometryCacheInfo, HashInfo, Header, LevelMapInfo, MAGIC, MeshInfo, PREAMBLE,
    Provenance, SetInfo, TagInfo, VERSION_MAJOR, VERSION_MINOR, Version, ELEMENT_TYPES, ElementTypeInfo, MeshStats, element_type_shape,
};
#[cfg(feature = "mmap")]
pub use reader::SmshMap;
pub use reader::{SmshExtrusion, SmshFile, SmshLevelMap, SmshMesh};
pub(crate) use writer::check_columns;
pub use writer::SmshWriter;
