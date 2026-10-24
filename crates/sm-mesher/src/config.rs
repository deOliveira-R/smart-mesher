//! The mesher service injected by the distribution.

use std::sync::Arc;

use sm_core::EntityId;
use sm_scheme_api::{
    CornerPicker, IntervalSolver, ReferenceCornerPicker, ReferenceIntervalSolver, SchemeError,
    SchemeRegistry, SizeField, SizingSpec,
};
use sm_topo_api::TopologyView;

/// Builds a size field from a sizing spec (`None` for no field).
pub type SizeFieldFn =
    Arc<dyn Fn(&SizingSpec) -> Result<Option<Box<dyn SizeField>>, SchemeError> + Send + Sync>;

/// Resolves `mesh.size {auto}` to a size for the targets at a factor (1..=10).
pub type AutoSizeFn =
    Arc<dyn Fn(&dyn TopologyView, &[EntityId], u32) -> Result<f64, String> + Send + Sync>;

/// The mesher service: scheme registry, solvers and sizing hooks, injected by the distribution.
#[derive(Clone)]
pub struct MesherConfig {
    registry: Arc<SchemeRegistry>,
    solver: Arc<dyn IntervalSolver>,
    picker: Arc<dyn CornerPicker>,
    size_fields: Option<SizeFieldFn>,
    auto_size: Option<AutoSizeFn>,
}

impl MesherConfig {
    /// A config over `registry` with the reference solver and picker and no sizing hooks.
    pub fn new(registry: SchemeRegistry) -> Self {
        Self {
            registry: Arc::new(registry),
            solver: Arc::new(ReferenceIntervalSolver::default()),
            picker: Arc::new(ReferenceCornerPicker),
            size_fields: None,
            auto_size: None,
        }
    }

    /// Replace the interval solver.
    #[must_use]
    pub fn with_solver(mut self, solver: Box<dyn IntervalSolver>) -> Self {
        self.solver = Arc::from(solver);
        self
    }

    /// Replace the corner picker.
    #[must_use]
    pub fn with_picker(mut self, picker: Box<dyn CornerPicker>) -> Self {
        self.picker = Arc::from(picker);
        self
    }

    /// Set the size-field builder.
    #[must_use]
    pub fn with_size_fields(mut self, f: SizeFieldFn) -> Self {
        self.size_fields = Some(f);
        self
    }

    /// Set the automatic-size hook.
    #[must_use]
    pub fn with_auto_size(mut self, f: AutoSizeFn) -> Self {
        self.auto_size = Some(f);
        self
    }

    /// The scheme registry.
    pub fn registry(&self) -> &SchemeRegistry {
        &self.registry
    }

    /// The interval solver.
    pub fn solver(&self) -> &dyn IntervalSolver {
        &*self.solver
    }

    /// The corner picker.
    pub fn picker(&self) -> &dyn CornerPicker {
        &*self.picker
    }

    /// The size-field builder, if set.
    pub fn size_fields(&self) -> Option<&SizeFieldFn> {
        self.size_fields.as_ref()
    }

    /// The automatic-size hook, if set.
    pub fn auto_size(&self) -> Option<&AutoSizeFn> {
        self.auto_size.as_ref()
    }
}
