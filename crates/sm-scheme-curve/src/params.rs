//! Typed parameters of the curve schemes, carried as JSON inside `SchemeChoice::params`.

use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_scheme_api::Params;

/// Default `max_ratio` of [`StretchParams`].
pub const DEFAULT_MAX_RATIO: f64 = 2.0;

/// Parameters of `bias`: `{ratio}`, `{first_size}`, `{first_size, last_size}` or
/// `{ratio, first_size}`; `start` names the vertex the series starts at (default: the edge's
/// own start vertex).
///
/// - `ratio`: factor between successive edge lengths (`e_{k+1} = ratio * e_k`); the interval
///   count comes from the plan.
/// - `first_size`: first edge length; the ratio is solved for the plan's count.
/// - `first_size` + `last_size`: the count is derived from both sizes (soft), then as `first_size`.
/// - `ratio` + `first_size`: the count is derived (soft), then the series is rebuilt from the
///   ratio over the plan's count.
#[derive(Clone, Debug, PartialEq, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BiasParams {
    /// Edge-length ratio.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ratio: Option<f64>,
    /// First edge length.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub first_size: Option<f64>,
    /// Last edge length (needs `first_size`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_size: Option<f64>,
    /// Vertex (`TopoVertex`) the series starts at; must be an end of the edge.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub start: Option<EntityId>,
}

/// A validated [`BiasParams`] combination.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BiasMode {
    /// Ratio only.
    Ratio(f64),
    /// First size only.
    First(f64),
    /// First and last size.
    FirstLast {
        /// First edge.
        first: f64,
        /// Last edge.
        last: f64,
    },
    /// Ratio and first size.
    RatioFirst {
        /// Ratio.
        ratio: f64,
        /// First edge.
        first: f64,
    },
}

fn pos(what: &str, v: f64) -> Result<f64, String> {
    if v.is_finite() && v > 0.0 { Ok(v) } else { Err(format!("{what} must be finite and positive, got {v}")) }
}

impl BiasParams {
    /// Validate the combination.
    pub fn mode(&self) -> Result<BiasMode, String> {
        let r = self.ratio.map(|v| pos("ratio", v)).transpose()?;
        let f = self.first_size.map(|v| pos("first_size", v)).transpose()?;
        let l = self.last_size.map(|v| pos("last_size", v)).transpose()?;
        match (r, f, l) {
            (Some(ratio), None, None) => Ok(BiasMode::Ratio(ratio)),
            (None, Some(first), None) => Ok(BiasMode::First(first)),
            (None, Some(first), Some(last)) => Ok(BiasMode::FirstLast { first, last }),
            (Some(ratio), Some(first), None) => Ok(BiasMode::RatioFirst { ratio, first }),
            (None, None, None) => Err("bias needs `ratio` or `first_size`".into()),
            (_, None, Some(_)) => Err("`last_size` needs `first_size`".into()),
            (Some(_), _, Some(_)) => Err("`ratio` and `last_size` over-determine the series".into()),
        }
    }
}

/// Parameters of `dualbias`: `{ratio}` or `{first_size}` (the end-edge length). `ratio > 1` is
/// coarse in the middle.
#[derive(Clone, Debug, PartialEq, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DualbiasParams {
    /// Edge-length ratio growing from both ends toward the middle.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ratio: Option<f64>,
    /// Length of the two end edges.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub first_size: Option<f64>,
}

/// A validated [`DualbiasParams`] combination.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DualMode {
    /// Ratio.
    Ratio(f64),
    /// End-edge size.
    First(f64),
}

impl DualbiasParams {
    /// Validate the combination.
    pub fn mode(&self) -> Result<DualMode, String> {
        match (self.ratio, self.first_size) {
            (Some(r), None) => Ok(DualMode::Ratio(pos("ratio", r)?)),
            (None, Some(f)) => Ok(DualMode::First(pos("first_size", f)?)),
            (None, None) => Err("dualbias needs `ratio` or `first_size`".into()),
            (Some(_), Some(_)) => Err("give `ratio` or `first_size`, not both".into()),
        }
    }
}

/// Parameters of `stretch`. Without `last_size` this is a geometric series from `first_size`;
/// with it, the end edges are pinned and the interior varies linearly between them.
/// `max_ratio` (default [`DEFAULT_MAX_RATIO`]) bounds the ratio of consecutive edges; a spacing
/// that needs more fails instead of being relaxed.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StretchParams {
    /// First edge length.
    pub first_size: f64,
    /// Last edge length.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_size: Option<f64>,
    /// Largest ratio of consecutive edges.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_ratio: Option<f64>,
}

impl StretchParams {
    /// Validated `(first, last, max_ratio)`.
    pub fn resolved(&self) -> Result<(f64, Option<f64>, f64), String> {
        let first = pos("first_size", self.first_size)?;
        let last = self.last_size.map(|v| pos("last_size", v)).transpose()?;
        let max = self.max_ratio.unwrap_or(DEFAULT_MAX_RATIO);
        if !(max.is_finite() && max >= 1.0) {
            return Err(format!("max_ratio must be finite and at least 1, got {max}"));
        }
        Ok((first, last, max))
    }
}

/// Parameters of `curvature`: `factor` `c`. `c = 0` is equal spacing, `c > 0` makes the edges
/// smallest where curvature is largest, `c < 0` largest there.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CurvatureParams {
    /// Adaptation factor.
    pub factor: f64,
}

/// Read typed parameters from a `SchemeChoice` payload (`null` reads as `{}`).
pub fn parse<T: serde::de::DeserializeOwned>(params: &Params) -> Result<T, String> {
    let v = if params.0.is_null() { serde_json::Value::Object(Default::default()) } else { params.0.clone() };
    serde_json::from_value(v).map_err(|e| e.to_string())
}
