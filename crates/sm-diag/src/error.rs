//! Typed errors of the diagnostics.

use sm_op_api::OpError;
use sm_quality::QualityError;

/// Why a diagnostic could not run.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum DiagError {
    /// An option is out of range or names something unknown.
    #[error("invalid argument: {0}")]
    BadArgument(String),
    /// A metric driver failed.
    #[error("quality: {0}")]
    Quality(#[from] QualityError),
}

impl From<DiagError> for OpError {
    fn from(e: DiagError) -> Self {
        match e {
            DiagError::BadArgument(m) => OpError::Invalid(m),
            DiagError::Quality(q) => OpError::Failed(q.to_string()),
        }
    }
}
