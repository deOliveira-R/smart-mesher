//! Mesh validation and quality diagnostics as queries and ops.
//!
//! [`DiagPlugin`] (id `"diag"`) registers the queries `mesh.validate` ([`MeshValidate`]) and
//! `mesh.quality` ([`MeshQuality`]) and the op `mesh.flag_cells` ([`MeshFlagCells`]). It
//! registers no layer and needs `sm_layer_api::CoreLayers`. The pure cores
//! ([`validate_mesh`], [`quality_report`], [`flagged_cells`]) work on any `MeshRead`.
//!
//! Wire rules: no NaN or inf in JSON (undefined is `null`, the Verdict sentinel 1e30 stays a
//! number), entities are `EntityId` strings with their dense index, every reply has
//! `schema_version: 1`, all maps are ordered. Suggestions name ops by string; whether an op is
//! available is read from the [`OpNames`] service the host injects. See `DESIGN.md`.

mod analysis;
mod common;
mod error;
mod flag;
mod locate;
mod quality;
mod suggest;
mod validate;

use std::collections::BTreeSet;

use sm_op_api::{Plugin, Registry, RegistryError};

pub use analysis::{SkippedMetric, ThresholdOverride};
pub use common::SCHEMA_VERSION;
pub use error::DiagError;
pub use flag::{Level, MAX_GROW, MeshFlagCells, flagged_cells, grow_cells};
pub use locate::{EntityRef, NEAR_CAP};
pub use quality::{
    Cluster, HistogramOut, MAX_CLUSTERS, MAX_WORST, MeshQuality, MetricReport, ProfileName, QualityOptions, QualityReport, SummaryOut, ThresholdOut, ValiditySummary,
    Verdict, WorstEntry, quality_report,
};
pub use suggest::{MIN_ENRICHMENT, MIN_FRACTION, OP_INTERVALS, OP_SCHEME, OP_SIZE, OP_SMOOTH, SPREAD_ENRICHMENT, Suggestion};
pub use validate::{
    Finding, GeometryStatus, MAX_LIMIT, MeshValidate, OwnerCount, Severity, ValidateOptions, ValidateReport, Violation, build_report, validate_mesh,
};

/// Names of the registered ops, as a service: `mesh.quality` reads it to mark suggestions
/// available. The host (sm-server) fills it from `Registry::op_names()`; without it every
/// suggestion that names an op is `available: false` and says the registry was not provided.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct OpNames(pub BTreeSet<String>);

impl OpNames {
    /// The names from any iterator of strings.
    pub fn new<I: IntoIterator<Item = S>, S: Into<String>>(names: I) -> Self {
        Self(names.into_iter().map(Into::into).collect())
    }
}

/// The `diag` plugin: queries `mesh.validate` and `mesh.quality`, op `mesh.flag_cells`.
///
/// Registers no layer; install `sm_layer_api::CoreLayers` first.
#[derive(Clone, Copy, Debug, Default)]
pub struct DiagPlugin;

impl Plugin for DiagPlugin {
    fn id(&self) -> &str {
        "diag"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        registry.register_query::<MeshValidate>()?;
        registry.register_query::<MeshQuality>()?;
        registry.register::<MeshFlagCells>()?;
        Ok(())
    }
}
