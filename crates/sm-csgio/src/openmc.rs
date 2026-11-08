//! OpenMC `geometry.xml` reader (stub until W3c-CSG-OPENMC).

use crate::ImportError;

/// Import an OpenMC geometry from its `geometry.xml` text.
///
/// Always returns [`ImportError::Unsupported`] until the reader lands.
pub fn import(_geometry_xml: &str) -> Result<(), ImportError> {
    Err(ImportError::Unsupported("the OpenMC reader is not implemented yet".into()))
}
