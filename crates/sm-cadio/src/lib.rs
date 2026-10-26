//! STEP (AP203/AP214/AP242) B-rep import into the cell complex, content-addressed.
//!
//! See `DESIGN.md`. The pieces:
//!
//! | Module | What |
//! |---|---|
//! | [`part21`] | tokenizer and parser (`parse`) |
//! | [`read_step`] | STEP bytes to a plain [`BrepModel`], independent of any topology store |
//! | [`edge_range`], [`weld_vertices`] | edge parameter ranges and reader-side stitching |
//! | [`realize`] | put a [`BrepModel`] into any `TopologyEdit` host inside an open operation |
//! | [`ImportStep`], [`StepInfoQuery`], [`CadioPlugin`] | the `model.import_step` op and `cad.step_info` query |
//! | [`inputs`] | content-addressed blobs: [`StepInputs`], [`MemoryInputStore`], [`DirInputStore`], [`stage_file`] |

mod args;
mod assembly;
mod decode;
mod edge;
pub mod error;
mod gcodec;
pub mod inputs;
mod inspect;
pub mod model;
mod op;
pub mod part21;
mod query;
mod realize;
mod source;
mod units;
mod weld;
mod write;

pub use decode::read_step;
pub use edge::edge_range;
pub use error::{OpenEdge, StepError};
pub use inputs::{DirInputStore, InputRef, InputStore, MemoryInputStore, StepInputs, sha256_hex, stage_file};
pub use inspect::{SolidInfo, StepInfo, UnsupportedInfo};
pub use model::*;
pub use op::ImportStep;
pub use query::StepInfoQuery;
pub use realize::{Inverted, RealizeOptions, Realized, realize};
pub use source::{InlineSource, MAX_INLINE_BYTES, Sha256Source, Source};
pub use weld::weld_vertices;
pub use write::{Loss, StepSchema, WriteOptions, WriteReport, export_step, write_step};

use sm_op_api::{Plugin, Registry, RegistryError};

/// The `cadio` plugin: registers `model.import_step` and `cad.step_info`, no layers
/// (`sm_layer_api::CoreLayers` registers them). Needs the [`StepInputs`] service for
/// `{sha256}` sources.
#[derive(Clone, Copy, Debug, Default)]
pub struct CadioPlugin;

impl Plugin for CadioPlugin {
    fn id(&self) -> &str {
        "cadio"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        registry.register::<ImportStep>()?;
        registry.register_query::<StepInfoQuery>()?;
        Ok(())
    }
}
