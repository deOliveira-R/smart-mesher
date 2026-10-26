//! Typed scheme parameters.

use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_scheme_api::{Params, SchemeError, SchemeName};

/// How the target face is produced.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum TargetMode {
    /// A copy of the source when the vertex correspondence is affine
    /// ([`crate::fit_target_transform`] accepts), else the scheme is not applicable.
    #[default]
    Auto,
    /// Always register a copy constraint (the best-effort transform if the strict fit fails).
    Copy,
    /// Mesh the target by projecting the last layer. Parsed, but not implemented: it is rejected
    /// with `BadParams`.
    Project,
}

/// How interior layer nodes are placed from the boundary columns.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Placement {
    /// Affine map plus Shepard residual; the interpolating placement if the Jacobian check
    /// of the affine one fails.
    #[default]
    Auto,
    /// Affine map plus Shepard residual only.
    Affine,
    /// Shepard interpolation of the column displacements only.
    Interpolate,
}

fn default_power() -> f64 {
    3.0
}

fn default_support() -> u32 {
    16
}

/// Parameters of the `"sweep"` scheme. Unknown fields are rejected.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SweepParams {
    /// The source face; give both `source` and `target`, or neither.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<[EntityId; 1]>,
    /// The target face.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target: Option<[EntityId; 1]>,
    /// How the target face is produced (default `auto`).
    #[serde(default)]
    pub target_mode: TargetMode,
    /// Interior placement (default `auto`).
    #[serde(default)]
    pub placement: Placement,
    /// Shepard distance exponent `p` in `w ~ d^-p` (default 3, must be positive).
    #[serde(default = "default_power")]
    pub shepard_power: f64,
    /// Number of nearest boundary nodes in the Shepard sums (default 16; 0 means all).
    #[serde(default = "default_support")]
    pub support: u32,
    /// Every corner scaled Jacobian must exceed this (default 0: strictly positive).
    #[serde(default)]
    pub min_scaled_jacobian: f64,
    /// Total layer count along a column; adds a linear row on the first column's edges.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub layers: Option<u32>,
    /// Test hook: bend the boundary columns by this angle (radians) over the sweep, to force
    /// inverted layers.
    #[cfg(feature = "test-hooks")]
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bend: Option<f64>,
}

impl Default for SweepParams {
    fn default() -> Self {
        Self {
            source: None,
            target: None,
            target_mode: TargetMode::Auto,
            placement: Placement::Auto,
            shepard_power: default_power(),
            support: default_support(),
            min_scaled_jacobian: 0.0,
            layers: None,
            #[cfg(feature = "test-hooks")]
            bend: None,
        }
    }
}

impl SweepParams {
    /// Parse and validate scheme parameters.
    ///
    /// # Errors
    /// [`SchemeError::BadParams`] for a wrong type, an unknown field, only one of
    /// `source`/`target`, a non-positive `shepard_power`, `layers: 0`, a non-finite
    /// `min_scaled_jacobian` (or one of at least 1), or `target_mode: "project"`.
    pub fn from_params(p: &Params) -> Result<Self, SchemeError> {
        let bad = |reason: String| SchemeError::BadParams {
            scheme: SchemeName::new(crate::NAME),
            reason,
        };
        let value = if p.0.is_null() {
            serde_json::Value::Object(serde_json::Map::new())
        } else {
            p.0.clone()
        };
        let s: SweepParams = serde_json::from_value(value).map_err(|e| bad(e.to_string()))?;
        if s.source.is_some() != s.target.is_some() {
            return Err(bad("give both `source` and `target`, or neither".into()));
        }
        if !(s.shepard_power.is_finite() && s.shepard_power > 0.0) {
            return Err(bad(format!(
                "shepard_power must be positive, got {}",
                s.shepard_power
            )));
        }
        if !(s.min_scaled_jacobian.is_finite() && s.min_scaled_jacobian < 1.0) {
            return Err(bad(format!(
                "min_scaled_jacobian must be below 1, got {}",
                s.min_scaled_jacobian
            )));
        }
        if s.layers == Some(0) {
            return Err(bad("layers must be at least 1".into()));
        }
        if s.target_mode == TargetMode::Project {
            return Err(bad("target_mode project is not implemented".into()));
        }
        Ok(s)
    }
}
