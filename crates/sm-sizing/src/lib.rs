//! Interval matching, corner picking and size fields. Skeleton: see W2 plan §2 task T3
//! (`docs/design/w2/plan.md`); that task fills in the bodies.

use sm_core::EntityId;
use sm_scheme_api::{
    ConstraintId, CornerPicker, Firm, IntervalSolver, ReferenceCornerPicker,
    ReferenceIntervalSolver, SchemeError, SchemeName, SizeField, SizingSpec,
};
use sm_topo_api::{CornerKey, TopologyView};

/// Optimal interval solver. For now it delegates to [`ReferenceIntervalSolver`].
#[derive(Clone, Copy, Debug, Default)]
pub struct IiaSolver {
    inner: ReferenceIntervalSolver,
}

impl IntervalSolver for IiaSolver {
    fn solve(
        &self,
        edges: &[EntityId],
        constraints: &[sm_scheme_api::Record<sm_scheme_api::IntervalConstraint>],
    ) -> Result<Vec<(EntityId, u32)>, sm_scheme_api::Infeasible> {
        self.inner.solve(edges, constraints)
    }
}

/// Cubit-style corner picker. For now it delegates to [`ReferenceCornerPicker`].
#[derive(Clone, Copy, Debug, Default)]
pub struct AnglePicker {
    inner: ReferenceCornerPicker,
}

impl CornerPicker for AnglePicker {
    fn pick(
        &self,
        topo: &dyn TopologyView,
        face: EntityId,
        rule: sm_scheme_api::VertexRule,
        requests: &[(CornerKey, Firm<sm_scheme_api::VertexType>)],
        ids: &[ConstraintId],
    ) -> sm_scheme_api::Result<sm_scheme_api::FaceLayout> {
        self.inner.pick(topo, face, rule, requests, ids)
    }
}

/// Build the size field of a sizing spec (`None` for no field). Not implemented yet: `Constant`
/// is served by `sm_scheme_api::sizing::field_for`, functions report `BadParams`.
pub fn size_field(spec: &SizingSpec) -> Result<Option<Box<dyn SizeField>>, SchemeError> {
    match spec {
        SizingSpec::Constant { .. } => sm_scheme_api::sizing::field_for(Some(spec)),
        _ => Err(SchemeError::BadParams {
            scheme: SchemeName("size_field".into()),
            reason: "not implemented yet (W2 T3)".into(),
        }),
    }
}

/// Errors of [`auto_size`].
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum SizingError {
    /// The function has no body yet (W2 T3).
    NotImplemented,
}

impl std::fmt::Display for SizingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotImplemented => f.write_str("auto_size is not implemented yet (W2 T3)"),
        }
    }
}

impl std::error::Error for SizingError {}

/// A deterministic automatic element size for `targets`, monotone in `factor` (1..=10). Not
/// implemented yet.
pub fn auto_size(
    _view: &dyn TopologyView,
    _targets: &[EntityId],
    _factor: u32,
) -> Result<f64, SizingError> {
    Err(SizingError::NotImplemented)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_scheme_api::stubs::brick_setup;

    #[test]
    fn entry_points_are_callable() {
        let _ = (IiaSolver::default(), AnglePicker::default());
        let solver: &dyn IntervalSolver = &IiaSolver::default();
        assert!(solver.solve(&[], &[]).is_ok());
        assert!(size_field(&SizingSpec::Constant { size: 1.0 }).expect("constant").is_some());
        let (cc, _, _) = brick_setup([1.0; 3]);
        assert_eq!(auto_size(&cc, &[], 5), Err(SizingError::NotImplemented));
    }
}
