//! Ops that edit the `intent` layer: `mesh.size`, `mesh.intervals`, `mesh.scheme`,
//! `mesh.default_size`. None of them touches the model or the meshes.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::ModelLayer;
use sm_op_api::{ApplyCtx, OpError, TypedOp};
use sm_scheme_api::{
    Firm, Firmness, IntervalRequest, Params, SchemeChoice, SchemeDim, SizingSpec,
};

use crate::config::MesherConfig;
use crate::intent_layer::IntentLayer;

/// Firmness as spelled in op parameters (`"default" | "soft" | "hard"`).
///
/// Cubit's ladder is `default < soft < hard`: a request never overwrites a firmer one on the
/// same entity (the op keeps the firmer request and emits a `mesh.firmness_kept` warning).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum FirmnessArg {
    /// Set automatically; the solver treats a size as a real-valued goal.
    Default,
    /// A preference, kept unless it conflicts with a hard request.
    Soft,
    /// A requirement, never changed.
    Hard,
}

impl From<FirmnessArg> for Firmness {
    fn from(f: FirmnessArg) -> Self {
        match f {
            FirmnessArg::Default => Firmness::Default,
            FirmnessArg::Soft => Firmness::Soft,
            FirmnessArg::Hard => Firmness::Hard,
        }
    }
}

const SIZE_KINDS: [EntityKind; 3] = [
    EntityKind::TopoVolume,
    EntityKind::TopoFace,
    EntityKind::TopoEdge,
];

/// Live targets of one of `kinds`, sorted and de-duplicated (so parameter order never matters).
pub(crate) fn live_targets(
    ctx: &ApplyCtx<'_>,
    ids: &[EntityId],
    kinds: &[EntityKind],
) -> Result<Vec<EntityId>, OpError> {
    if ids.is_empty() {
        return Err(OpError::Invalid("`targets` is empty".into()));
    }
    for &id in ids {
        ctx.require(id)?;
        if !kinds.contains(&id.kind()) {
            return Err(OpError::Invalid(format!(
                "{id} is a {:?}; expected one of {kinds:?}",
                id.kind()
            )));
        }
    }
    let mut v = ids.to_vec();
    v.sort_unstable();
    v.dedup();
    Ok(v)
}

fn dim_of(kind: EntityKind) -> SchemeDim {
    match kind {
        EntityKind::TopoEdge => SchemeDim::Curve,
        EntityKind::TopoFace => SchemeDim::Surface,
        _ => SchemeDim::Volume,
    }
}

fn scheme_err(e: sm_scheme_api::SchemeError) -> OpError {
    OpError::Invalid(e.to_string())
}

/// Install `req` on `id` unless a firmer request is already there. Returns whether it applied.
fn set_intervals(layer: &mut IntentLayer, id: EntityId, req: Firm<IntervalRequest>) -> bool {
    let i = layer.intent_mut();
    if i.entities
        .get(&id)
        .and_then(|e| e.intervals)
        .is_some_and(|old| old.firmness > req.firmness)
    {
        return false;
    }
    i.set_intervals(id, req);
    true
}

fn finish(layer: &mut IntentLayer) -> Result<(), OpError> {
    let i = layer.intent_mut();
    i.canonicalize();
    i.check().map_err(scheme_err)
}

fn warn_kept(ctx: &mut ApplyCtx<'_>, op: &str, kept: &[EntityId]) {
    if !kept.is_empty() {
        ctx.warn(
            "mesh.firmness_kept",
            format!("{op}: a firmer request is already set; kept it"),
            kept,
        );
    }
}

/// `mesh.size`: ask for an element size on volumes, faces or curves.
///
/// Recorded as `IntervalRequest::Size` on each target. On a volume or face it applies to every
/// edge of it; an edge's own request wins over a face's, a face's over a volume's. Default
/// firmness is `default`: the size is a real goal `length / size` that the interval solver
/// approaches while matching opposite sides. `soft` and `hard` fix the count
/// `max(1, round(length / size))` instead. `auto` (1 fine .. 10 coarse) is resolved *now* through
/// the configured auto-size hook and recorded as a plain size, so replay never re-runs it.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshSize {
    /// Volumes, faces or curves to size.
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoFace", "TopoEdge"]))]
    pub targets: Vec<EntityId>,
    /// Element size (> 0). Exactly one of `size` and `auto`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub size: Option<f64>,
    /// Automatic size factor, 1 (fine) to 10 (coarse). Exactly one of `size` and `auto`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub auto: Option<u32>,
    /// Firmness (default `default`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub firmness: Option<FirmnessArg>,
}

impl TypedOp for MeshSize {
    const NAME: &'static str = "mesh.size";

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let targets = live_targets(ctx, &self.targets, &SIZE_KINDS)?;
        let size = match (self.size, self.auto) {
            (Some(s), None) => s,
            (None, Some(k)) => {
                if !(1..=10).contains(&k) {
                    return Err(OpError::Invalid("`auto` must be in 1..=10".into()));
                }
                let hook = ctx
                    .service::<MesherConfig>()?
                    .auto_size()
                    .cloned()
                    .ok_or_else(|| OpError::Invalid("no auto-size hook is configured".into()))?;
                hook(ctx.layer::<ModelLayer>()?.topo(), &targets, k).map_err(OpError::Invalid)?
            }
            _ => {
                return Err(OpError::Invalid(
                    "give exactly one of `size` and `auto`".into(),
                ));
            }
        };
        if !(size.is_finite() && size > 0.0) {
            return Err(OpError::Invalid(format!(
                "size {size} must be positive and finite"
            )));
        }
        let firmness = self.firmness.map_or(Firmness::Default, Firmness::from);
        let layer = ctx.layer_mut::<IntentLayer>()?;
        let mut kept = Vec::new();
        for &t in &targets {
            let req = Firm {
                value: IntervalRequest::Size(size),
                firmness,
            };
            if !set_intervals(layer, t, req) {
                kept.push(t);
            }
        }
        finish(layer)?;
        warn_kept(ctx, Self::NAME, &kept);
        Ok(())
    }
}

/// `mesh.intervals`: ask for an interval count on volumes, faces or curves.
///
/// On a curve the default firmness is `hard`; on a face or volume the count applies to the owned
/// curves and defaults to `soft` (Cubit: a count on a curve is hard, on a surface or volume soft).
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshIntervals {
    /// Volumes, faces or curves.
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoFace", "TopoEdge"]))]
    pub targets: Vec<EntityId>,
    /// Number of intervals (>= 1).
    pub count: u32,
    /// Firmness (default: `hard` on curves, `soft` otherwise).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub firmness: Option<FirmnessArg>,
}

impl TypedOp for MeshIntervals {
    const NAME: &'static str = "mesh.intervals";

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let targets = live_targets(ctx, &self.targets, &SIZE_KINDS)?;
        if self.count == 0 {
            return Err(OpError::Invalid("`count` must be >= 1".into()));
        }
        let layer = ctx.layer_mut::<IntentLayer>()?;
        let mut kept = Vec::new();
        for &t in &targets {
            let firmness = self.firmness.map_or_else(
                || {
                    if t.kind() == EntityKind::TopoEdge {
                        Firmness::Hard
                    } else {
                        Firmness::Soft
                    }
                },
                Firmness::from,
            );
            let req = Firm {
                value: IntervalRequest::Count(self.count),
                firmness,
            };
            if !set_intervals(layer, t, req) {
                kept.push(t);
            }
        }
        finish(layer)?;
        warn_kept(ctx, Self::NAME, &kept);
        Ok(())
    }
}

/// `mesh.scheme`: choose the meshing scheme of volumes, faces or curves.
///
/// The scheme must be registered for the target's dimension (or be the reserved `copy`). The
/// choice defaults to `hard`: the user named it, so a conflicting imposition is an error rather
/// than a silent override. `fallbacks` are tried in order, as `soft` choices without parameters,
/// when the scheme fails to apply or to generate.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshScheme {
    /// Volumes, faces or curves.
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoFace", "TopoEdge"]))]
    pub targets: Vec<EntityId>,
    /// Registered scheme name (`"map"`, `"equal"`, `"sweep"`, ...).
    pub scheme: String,
    /// Scheme parameters, an object owned by the scheme.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub params: Option<Params>,
    /// Scheme names tried in order after `scheme` fails.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub fallbacks: Vec<String>,
    /// Firmness of `scheme` (default `hard`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub firmness: Option<FirmnessArg>,
}

impl TypedOp for MeshScheme {
    const NAME: &'static str = "mesh.scheme";

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let targets = live_targets(ctx, &self.targets, &SIZE_KINDS)?;
        let cfg = ctx.service::<MesherConfig>()?.clone();
        let known = |dim: SchemeDim, name: &str| {
            name == sm_scheme_api::SchemeName::COPY || cfg.registry().get(dim, name).is_some()
        };
        for &t in &targets {
            let dim = dim_of(t.kind());
            for name in std::iter::once(&self.scheme).chain(&self.fallbacks) {
                if !known(dim, name) {
                    return Err(OpError::Invalid(format!(
                        "no {dim:?} scheme named `{name}` (target {t})"
                    )));
                }
            }
        }
        let firmness = self.firmness.map_or(Firmness::Hard, Firmness::from);
        let choice = match &self.params {
            Some(p) if !p.is_empty() => {
                p.check().map_err(scheme_err)?;
                SchemeChoice::with_params(&self.scheme, p.0.clone(), firmness)
            }
            _ => SchemeChoice::new(&self.scheme, firmness),
        };
        let fallbacks: Vec<SchemeChoice> = self
            .fallbacks
            .iter()
            .map(|n| SchemeChoice::new(n, Firmness::Soft))
            .collect();
        let layer = ctx.layer_mut::<IntentLayer>()?;
        let mut kept = Vec::new();
        for &t in &targets {
            let i = layer.intent_mut();
            if i.entities
                .get(&t)
                .and_then(|e| e.scheme.as_ref())
                .is_some_and(|old| old.firmness > firmness)
            {
                kept.push(t);
                continue;
            }
            let e = i.entity_mut(t);
            e.scheme = Some(choice.clone());
            e.fallbacks = fallbacks.clone();
        }
        finish(layer)?;
        warn_kept(ctx, Self::NAME, &kept);
        Ok(())
    }
}

/// `mesh.default_size`: the model-wide default sizing, used where no request reaches an edge.
///
/// Either a constant `size`, or a named sizing `function` with `params` that the configured
/// size-field builder ([`MesherConfig::with_size_fields`]) understands. Replaces any earlier
/// default.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshDefaultSize {
    /// A constant element size (> 0). Exactly one of `size` and `function`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub size: Option<f64>,
    /// A named sizing function. Exactly one of `size` and `function`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub function: Option<String>,
    /// Parameters of `function`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub params: Option<Params>,
}

impl TypedOp for MeshDefaultSize {
    const NAME: &'static str = "mesh.default_size";

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let spec = match (self.size, &self.function) {
            (Some(size), None) => {
                if !(size.is_finite() && size > 0.0) {
                    return Err(OpError::Invalid(format!(
                        "size {size} must be positive and finite"
                    )));
                }
                SizingSpec::Constant { size }
            }
            (None, Some(name)) => SizingSpec::Function {
                name: name.clone(),
                params: self.params.clone().unwrap_or_default(),
            },
            _ => {
                return Err(OpError::Invalid(
                    "give exactly one of `size` and `function`".into(),
                ));
            }
        };
        if matches!(spec, SizingSpec::Function { .. }) {
            // Build it once to validate the name and parameters now, not at `mesh.run`.
            let cfg = ctx.service::<MesherConfig>()?;
            let build = cfg
                .size_fields()
                .ok_or_else(|| OpError::Invalid("no size-field builder is configured".into()))?;
            if build(&spec).map_err(scheme_err)?.is_none() {
                return Err(OpError::Invalid("the sizing function builds no field".into()));
            }
        }
        let layer = ctx.layer_mut::<IntentLayer>()?;
        layer.intent_mut().defaults.sizing = Some(spec);
        finish(layer)
    }
}
