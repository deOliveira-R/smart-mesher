//! OpenMC `geometry.xml` / `materials.xml` reader (stub until W3c-CSG-OPENMC).

use crate::{CsgIr, ImportError};

/// Import an OpenMC geometry from the text of its `geometry.xml` and, when there is one, of its
/// `materials.xml`.
///
/// # Errors
/// Always [`ImportError::Unsupported`] until the reader lands.
pub fn import(_geometry_xml: &str, _materials_xml: Option<&str>) -> Result<CsgIr, ImportError> {
    Err(ImportError::Unsupported { line: None, what: "the OpenMC reader is not implemented yet".into() })
}
