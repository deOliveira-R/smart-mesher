//! Errors of the deck to B-rep import.

use sm_op_api::OpError;

/// Why a deck could not be built into B-rep volumes. Every message names the deck element.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum CsgbrepError {
    /// A feature the builder does not implement (always names the deck surface or element).
    #[error("unsupported{}: {what}", .line.map(|l| format!(" (line {l})")).unwrap_or_default())]
    Unsupported {
        /// Line of the offending element in the deck, when known.
        line: Option<u32>,
        /// What is refused.
        what: String,
    },
    /// The deck or an option is inconsistent.
    #[error("{0}")]
    Invalid(String),
    /// The model has no finite bounds; give `options.bounds`.
    #[error("the deck is not bounded: {0}")]
    Unbounded(String),
    /// Two cells own the same piece of space.
    #[error("cells {a} and {b} overlap in a piece of volume {volume:e}")]
    Overlap {
        /// First cell (deck path).
        a: String,
        /// Second cell (deck path).
        b: String,
        /// Volume of the shared piece.
        volume: f64,
    },
    /// The geometry kernel refused a cut or a union.
    #[error("kernel: {0}")]
    Kernel(String),
}

impl From<CsgbrepError> for OpError {
    fn from(e: CsgbrepError) -> Self {
        match e {
            CsgbrepError::Unsupported { line, what } => OpError::Unsupported(format!("csg.unsupported: {what}{}", line.map(|l| format!(" (deck line {l})")).unwrap_or_default())),
            CsgbrepError::Invalid(m) => OpError::Invalid(format!("csg.invalid: {m}")),
            CsgbrepError::Unbounded(m) => OpError::Invalid(format!("csg.unbounded: the deck is not bounded: {m}")),
            CsgbrepError::Overlap { a, b, volume } => OpError::Failed(format!("csg.overlap: cells {a} and {b} overlap in a piece of volume {volume:e}")),
            CsgbrepError::Kernel(m) => OpError::Failed(format!("csg.kernel: {m}")),
        }
    }
}

impl From<sm_csgio::ImportError> for CsgbrepError {
    fn from(e: sm_csgio::ImportError) -> Self {
        match e {
            sm_csgio::ImportError::Unsupported { line, what } => Self::Unsupported { line, what },
            other => Self::Invalid(other.to_string()),
        }
    }
}
