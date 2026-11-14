//! The intent and meshes plugin and the mesh-driver node (W2 task T7).
//!
//! | Item | What |
//! |---|---|
//! | [`MeshPlugin`] | registers [`IntentLayer`], the `mesh.*` ops and the `mesh.info` query. It does **not** register the `meshes` layer (`sm_layer_api::CoreLayers` does) |
//! | [`MesherConfig`] | the service: scheme registry, interval solver, corner picker, size-field and auto-size hooks, injected by the distribution |
//! | [`IntentLayer`] | the `intent` layer (depends on `model`) |
//! | [`RayonExec`] | the driver [`Executor`](sm_scheme_api::Executor) on the rayon pool |
//! | ops | `mesh.size`, `mesh.intervals`, `mesh.scheme`, `mesh.default_size`, `mesh.auto` (intent), `mesh.run` (volumes or faces), `mesh.delete` (meshes), `mesh.preserve_volume` (equal-area circle nodes) |
//! | queries | `mesh.info`, `mesh.plan` (explain what ran or will run, and why), `mesh.auto_strategies` |
//!
//! This crate names no scheme crate and no sizing crate: schemes, the solver and the picker all
//! arrive through [`MesherConfig`]. See `DESIGN.md`.
//!
//! # Install order
//! The `intent` and `meshes` layers depend on `model`, so install `CoreLayers` before
//! [`MeshPlugin`], and inject a [`MesherConfig`] service
//! (`Services::new().with_versioned("mesher", 1, cfg)`).

mod auto;
mod census;
mod config;
mod decompose_query;
mod exec;
mod info;
mod intent_layer;
mod intent_ops;
mod mesh_ops;
mod plan_query;
mod polyscheme;
mod volfit;

pub use auto::{
    AutoInput, AutoLists, AutoStrategy, DEFAULT_STRATEGY, DefaultStrategy, HEX_ONLY_STRATEGY,
    MeshAuto, MeshAutoStrategiesQuery, SchemeAssignment, StrategiesInfo, StrategyInfo,
    StrategyRegistry, TET_STRATEGY, replan_budget,
};
pub use census::{CensusOutcome, CensusReport, CensusRow, census};
pub use decompose_query::{DecomposePlanReport, DecomposeStatus, MeshDecomposePlanQuery, VolumeCuts};
pub use config::{AutoSizeFn, DEFAULT_MAX_CELLS, MesherConfig, SizeFieldFn};
pub use exec::RayonExec;
pub use polyscheme::POLYHEDRON;
pub use volfit::MeshPreserveVolume;
pub use info::{MeshInfo, MeshInfoQuery};
pub use intent_layer::IntentLayer;
pub use intent_ops::{FirmnessArg, MeshDefaultSize, MeshIntervalRelation, MeshIntervals, MeshScheme, MeshSize, RelationArg};
pub use mesh_ops::{MeshDelete, MeshRun};
pub use plan_query::{
    Alternative, ElementsInfo, EntityPlan, IntervalRequestRow, IntervalWhy, MeshPlanQuery, PlanDim,
    PlanFailure, PlanNote, PlanReport, PlanStatus, WhyKind, explain_plan,
};

use sm_op_api::{Plugin, Registry, RegistryError};

/// The mesher plugin: registers [`IntentLayer`], the `mesh.*` ops and the `mesh.info` query.
#[derive(Clone, Copy, Debug, Default)]
pub struct MeshPlugin;

impl Plugin for MeshPlugin {
    fn id(&self) -> &str {
        "mesher"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        registry.register_layer::<IntentLayer>()?;
        registry.register::<MeshSize>()?;
        registry.register::<MeshIntervals>()?;
        registry.register::<MeshIntervalRelation>()?;
        registry.register::<MeshScheme>()?;
        registry.register::<MeshDefaultSize>()?;
        registry.register::<MeshRun>()?;
        registry.register::<MeshDelete>()?;
        registry.register::<MeshAuto>()?;
        registry.register::<MeshPreserveVolume>()?;
        registry.register_query::<MeshInfoQuery>()?;
        registry.register_query::<MeshPlanQuery>()?;
        registry.register_query::<MeshDecomposePlanQuery>()?;
        registry.register_query::<MeshAutoStrategiesQuery>()?;
        Ok(())
    }
}
