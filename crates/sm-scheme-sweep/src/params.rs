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
    /// Always register a copy constraint with the best-effort fitted transform; if its residual
    /// exceeds the copy tolerance the analysis fails (no silent bad copy) until `project` exists.
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

/// A face list that also accepts a bare id (`"source": 7` is `"source": [7]`).
fn face_list<'de, D: serde::Deserializer<'de>>(d: D) -> Result<Option<Vec<EntityId>>, D::Error> {
    #[derive(Deserialize)]
    #[serde(untagged)]
    enum One {
        List(Vec<EntityId>),
        Single(EntityId),
    }
    Ok(Option::<One>::deserialize(d)?.map(|o| match o {
        One::List(v) => v,
        One::Single(f) => vec![f],
    }))
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
    /// The source faces; give both `source` and `target`, or neither. One face is the 1-to-1
    /// sweep; several are a many-to-many sweep over imprinted faces (each source face has a
    /// target face with the vertices its columns end at). A bare id is read as a one-item list.
    #[serde(
        default,
        skip_serializing_if = "Option::is_none",
        deserialize_with = "face_list"
    )]
    pub source: Option<Vec<EntityId>>,
    /// The target faces (as many as source faces).
    #[serde(
        default,
        skip_serializing_if = "Option::is_none",
        deserialize_with = "face_list"
    )]
    pub target: Option<Vec<EntityId>>,
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
    /// `source`/`target`, source and target lists of different lengths, a non-positive `shepard_power`, `layers: 0`, a non-finite
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
        if s.source.as_ref().is_some_and(Vec::is_empty)
            || s.target.as_ref().is_some_and(Vec::is_empty)
        {
            return Err(bad("`source` and `target` need at least one face".into()));
        }
        if let (Some(a), Some(b)) = (&s.source, &s.target)
            && a.len() != b.len()
        {
            return Err(bad(format!(
                "source faces {a:?} against target faces {b:?}: the sets differ in size, so there is no face-to-face correspondence (many-to-one sweeps are not supported)"
            )));
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
