//! Booleans (unite, subtract, intersect), imprint, merge and the webcut family on the cell complex.
//!
//! One fragment kernel serves every op: candidate face pairs, surface-surface intersection,
//! clipping to both faces, splitting with the Euler operators of `sm_topo_api::TopologyEdit`,
//! classification of the fragments against the other operand, volume assembly and
//! regularization. See `DESIGN.md`.
//!
//! The library functions ([`boolean`], [`imprint_bodies`], [`merge`], [`webcut`]) run **inside an
//! already open operation** (`begin_op` .. `end_op`), exactly like `sm_topo_api::fixtures`: they
//! never open, close or abort one. Atomicity comes from the caller: `sm_layer_api::topo_txn`
//! aborts the operation when a function returns `Err`, and tests use
//! `sm_topo_api::fixtures::with_op`. [`merge_check`] and [`locate_point`] are read-only.

mod arrange;
mod assemble;
mod bodies;
pub mod boolean;
mod cc;
mod copy;
pub mod error;
mod faceclass;
mod geo;
pub mod imprint;
pub mod locate;
pub mod merge;
pub mod ops;
mod pieces;
mod poly2;
mod regularize;
mod registry;
pub mod webcut;

pub use error::BooleanError;
pub use boolean::{BoolOptions, BoolOutcome, BoolRule, boolean};
pub use imprint::{ImprintOptions, ImprintOutcome, imprint_bodies};
pub use webcut::{CutTool, KeepSide, WebcutOptions, WebcutOutcome, webcut};
pub use locate::{PointLocation, locate_point};
pub use merge::{MergeKind, MergeKinds, MergeOptions, MergePair, MergeReport, MergeStatus, MergedCell, SkipReason, merge, merge_check};

use sm_op_api::{Plugin, Registry, RegistryError};

/// The `boolean` plugin: booleans, imprint, merge and webcut as ops and queries. It registers no
/// layers (assembly installs it next to `sm_layer_api::CoreLayers`).
#[derive(Clone, Copy, Debug, Default)]
pub struct BooleanPlugin;

impl Plugin for BooleanPlugin {
    fn id(&self) -> &str {
        "boolean"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        registry.register::<ops::Unite>()?;
        registry.register::<ops::Subtract>()?;
        registry.register::<ops::Intersect>()?;
        registry.register::<ops::Imprint>()?;
        registry.register::<ops::Merge>()?;
        registry.register::<ops::WebcutPlane>()?;
        registry.register::<ops::WebcutSurface>()?;
        registry.register::<ops::WebcutCylinder>()?;
        registry.register_query::<ops::MergeCheck>()?;
        Ok(())
    }
}
