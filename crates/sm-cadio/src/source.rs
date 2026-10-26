//! The `source` parameter of the import op and query: a content hash or inline bytes.

use std::sync::Arc;

use base64::Engine as _;
use base64::engine::general_purpose::STANDARD;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_op_api::OpError;

use crate::error::StepError;
use crate::inputs::{StepInputs, is_sha256_hex, sha256_hex};

/// Largest inline file (decoded): 8 MiB. Bigger files are staged and named by `sha256`.
pub const MAX_INLINE_BYTES: usize = 8 << 20;

/// A file named by the sha256 of its bytes, read from the injected [`StepInputs`] service.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Sha256Source {
    /// 64 lowercase hex digits.
    pub sha256: String,
}

/// A file carried inside the op, base64 encoded (standard alphabet, padded).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct InlineSource {
    /// Base64 of the file; at most [`MAX_INLINE_BYTES`] decoded.
    pub inline: String,
}

/// Where the STEP bytes come from. Never a path: a path is neither replayable nor forkable.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum Source {
    /// `{"sha256": "<64 hex>"}`.
    Sha256(Sha256Source),
    /// `{"inline": "<base64>"}`.
    Inline(InlineSource),
}

impl Source {
    /// A hash source.
    pub fn sha256(hex: impl Into<String>) -> Self {
        Self::Sha256(Sha256Source { sha256: hex.into() })
    }

    /// An inline source encoding `bytes`.
    pub fn inline(bytes: &[u8]) -> Self {
        Self::Inline(InlineSource { inline: STANDARD.encode(bytes) })
    }
}

/// Resolve `source` to its bytes. `service` is asked for the [`StepInputs`] only when the source
/// is a hash. Every failure is a deterministic [`OpError::Invalid`] (a missing service is
/// [`OpError::MissingService`]).
pub(crate) fn load<'a>(source: &Source, service: impl FnOnce() -> Result<&'a StepInputs, OpError>) -> Result<Arc<[u8]>, OpError> {
    match source {
        Source::Inline(i) => {
            // Reject on the encoded size first: never decode more than the cap allows.
            let max_encoded = MAX_INLINE_BYTES.div_ceil(3) * 4;
            if i.inline.len() > max_encoded {
                return Err(StepError::Inline(format!("inline source is {} base64 characters; the limit is {MAX_INLINE_BYTES} bytes decoded (stage the file and use sha256)", i.inline.len())).into());
            }
            let bytes = STANDARD.decode(i.inline.as_bytes()).map_err(|e| StepError::Inline(format!("inline source is not valid base64: {e}")))?;
            if bytes.len() > MAX_INLINE_BYTES {
                return Err(StepError::Inline(format!("inline source is {} bytes; the limit is {MAX_INLINE_BYTES} (stage the file and use sha256)", bytes.len())).into());
            }
            Ok(Arc::from(bytes))
        }
        Source::Sha256(s) => {
            if !is_sha256_hex(&s.sha256) {
                return Err(OpError::Invalid(format!("step.bad_sha256: `{}` is not 64 lowercase hex digits", s.sha256)));
            }
            let store = service()?;
            let Some(bytes) = store.0.get(&s.sha256) else {
                return Err(OpError::Invalid(format!("step.input_missing: input blob {} not found; stage it first", s.sha256)));
            };
            let got = sha256_hex(&bytes);
            if got != s.sha256 {
                return Err(OpError::Invalid(format!("step.input_mismatch: the staged blob for {} hashes to {got}", s.sha256)));
            }
            Ok(bytes)
        }
    }
}
