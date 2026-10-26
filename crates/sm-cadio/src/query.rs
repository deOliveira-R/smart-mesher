//! `cad.step_info`: what a STEP file contains, without importing it.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_op_api::{OpError, QueryCtx, State, TypedQuery};

use crate::inputs::{StepInputs, sha256_hex};
use crate::inspect::{StepInfo, inspect};
use crate::part21;
use crate::source::{Source, load};

/// `cad.step_info {source, name?}`: schema, hash, units, uncertainty, per-solid counts and
/// surface kinds, unsupported entities and whether the file is an assembly. Pure: it reads the
/// bytes named by `source` (inline, or through the [`StepInputs`] service) and nothing else.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct StepInfoQuery {
    /// The file: `{sha256}` or `{inline}`.
    pub source: Source,
    /// A label for the file (echoed in errors only).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
}

impl TypedQuery for StepInfoQuery {
    const NAME: &'static str = "cad.step_info";
    type Output = StepInfo;

    fn run(&self, _state: &State, cx: &QueryCtx<'_>) -> Result<StepInfo, OpError> {
        let bytes = load(&self.source, || cx.service::<StepInputs>())?;
        let file = part21::parse(&bytes)?;
        Ok(inspect(&file, sha256_hex(&bytes), bytes.len() as u64)?)
    }
}
