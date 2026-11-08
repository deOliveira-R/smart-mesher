//! CSG model import: reads constructive-solid-geometry decks into a deterministic IR.
//!
//! Skeleton (W3c-0.5). Only OpenMC is a target (ruling C6); the IR, instantiation and the
//! `smcsg` tool arrive with W3c-CSG-CORE, and the OpenMC reader with W3c-CSG-OPENMC.

pub mod openmc;

/// Errors of CSG import.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum ImportError {
    /// The requested format or feature is not implemented yet.
    #[error("unsupported: {0}")]
    Unsupported(String),
}
