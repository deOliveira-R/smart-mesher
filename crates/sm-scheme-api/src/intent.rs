//! Serializable meshing intent: what the user (or an Op) asked for. Pure data, stored in an Op
//! and in git; the driver turns it into a [`crate::MeshingProblem`].

use crate::error::SchemeError;
use crate::firm::{Firm, Firmness};
use crate::target::{GroupId, TransformId};
use crate::vertex::VertexType;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_topo_api::CornerKey;
use std::collections::BTreeMap;

/// Registered name of a scheme (`"equal"`, `"map"`, `"sweep"`, ...). Names are unique per
/// dimension in a [`crate::SchemeRegistry`].
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[serde(transparent)]
pub struct SchemeName(pub String);

impl SchemeName {
    /// The reserved name of the copy scheme, executed by the driver at every dimension (user
    /// copy, mirror, deck copies, sweep targets).
    pub const COPY: &'static str = "copy";

    /// Build from a string.
    pub fn new(s: &str) -> Self {
        Self(s.to_owned())
    }
    /// The name as a string.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl core::fmt::Display for SchemeName {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(&self.0)
    }
}

/// Scheme parameters: a JSON object owned by the scheme. The value model is exactly
/// `sm_op_api::codec::Value` (`serde_json::Value` restricted to what TOML can represent: no
/// `null`, finite floats), so intents and plans cross the Op boundary without conversion.
/// Entity references are [`EntityId`] strings (`Kind#serial`) read with [`Params::get`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Params(pub serde_json::Value);

#[cfg(feature = "schemars")]
impl schemars::JsonSchema for Params {
    fn schema_name() -> std::borrow::Cow<'static, str> {
        "Params".into()
    }
    fn json_schema(_: &mut schemars::SchemaGenerator) -> schemars::Schema {
        schemars::json_schema!({
            "type": "object",
            "description": "Scheme parameters (owned by the scheme): an object of TOML-representable values; entity references are `Kind#serial` strings."
        })
    }
}

impl Default for Params {
    fn default() -> Self {
        Self::empty()
    }
}

impl Params {
    /// Empty parameters: the empty object `{}` (never `null`, which TOML cannot hold).
    pub fn empty() -> Self {
        Self(serde_json::Value::Object(serde_json::Map::new()))
    }

    /// True for the empty object (and for `null`, read as empty).
    pub fn is_empty(&self) -> bool {
        match &self.0 {
            serde_json::Value::Null => true,
            serde_json::Value::Object(m) => m.is_empty(),
            _ => false,
        }
    }

    /// Check that the value is an object the Op codec accepts (TOML-representable, no `null`,
    /// finite floats). Run when an intent is loaded (see [`MeshIntent::check`]).
    pub fn check(&self) -> Result<(), SchemeError> {
        sm_op_api::codec::to_params(&self.0)
            .map(|_| ())
            .map_err(|e| SchemeError::BadParams {
                scheme: SchemeName::new(""),
                reason: e.to_string(),
            })
    }

    /// Typed field `key`; `Ok(None)` if absent. A field of the wrong type is an error naming
    /// `scheme`.
    pub fn get<T: DeserializeOwned>(
        &self,
        scheme: &str,
        key: &str,
    ) -> Result<Option<T>, SchemeError> {
        match self.0.get(key) {
            None | Some(serde_json::Value::Null) => Ok(None),
            Some(v) => {
                serde_json::from_value(v.clone())
                    .map(Some)
                    .map_err(|e| SchemeError::BadParams {
                        scheme: SchemeName::new(scheme),
                        reason: format!("{key}: {e}"),
                    })
            }
        }
    }
}

/// A scheme request: name, parameters and firmness.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct SchemeChoice {
    /// Scheme name.
    pub name: SchemeName,
    /// Scheme parameters.
    #[serde(default, skip_serializing_if = "Params::is_empty")]
    pub params: Params,
    /// Firmness: a `Hard` choice that conflicts with an imposition is an error; a `Soft` or
    /// `Default` one is overridden.
    #[serde(default)]
    pub firmness: Firmness,
}

impl SchemeChoice {
    /// A choice with no parameters.
    pub fn new(name: &str, firmness: Firmness) -> Self {
        Self {
            name: SchemeName::new(name),
            params: Params::empty(),
            firmness,
        }
    }
    /// A choice with parameters.
    pub fn with_params(name: &str, params: serde_json::Value, firmness: Firmness) -> Self {
        Self {
            name: SchemeName::new(name),
            params: Params(params),
            firmness,
        }
    }
}

/// An interval request on a curve.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub enum IntervalRequest {
    /// Exactly this many intervals (`Hard`: fixed; `Soft` / `Default`: a preferred count).
    Count(u32),
    /// Target element size. `Hard` / `Soft`: a fixed count `round(length / size).max(1)` with
    /// that firmness; `Default`: a real goal `length / size`.
    Size(f64),
}

/// Boundary-layer request on a face (into the adjacent volumes). Refused on deck faces and on
/// anything that is not a face (curve boundary layers do not exist yet).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct BoundaryLayerIntent {
    /// First layer thickness.
    pub first: f64,
    /// Growth ratio.
    pub growth: f64,
    /// Number of layers.
    pub layers: u32,
    /// Volumes (`TopoVolume` / `PartitionCell` pieces) the layer grows into; empty = every
    /// adjacent volume. Each listed volume must be adjacent to the face.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub into: Vec<EntityId>,
    /// Extra parameters for the boundary-layer generator (owned by the volume scheme).
    #[serde(default, skip_serializing_if = "Params::is_empty")]
    pub params: Params,
}

impl BoundaryLayerIntent {
    /// A layer into every adjacent volume, without extra parameters.
    pub fn new(first: f64, growth: f64, layers: u32) -> Self {
        Self {
            first,
            growth,
            layers,
            into: Vec::new(),
            params: Params::empty(),
        }
    }
}

/// Everything asked for one entity (edge, face, volume or piece).
#[derive(Clone, Debug, PartialEq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct EntityIntent {
    /// The scheme, if chosen.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scheme: Option<SchemeChoice>,
    /// Fallback chain tried in order after `scheme` (e.g. tridelaunay, triadvance, qtri).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub fallbacks: Vec<SchemeChoice>,
    /// Interval request (curves; on a face or volume it applies to every edge of it).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub intervals: Option<Firm<IntervalRequest>>,
    /// Boundary layer request (faces).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub boundary_layer: Option<BoundaryLayerIntent>,
}

impl EntityIntent {
    /// An empty request.
    pub fn new() -> Self {
        Self::default()
    }
    /// Set the scheme.
    pub fn with_scheme(mut self, choice: SchemeChoice) -> Self {
        self.scheme = Some(choice);
        self
    }
    /// Set the fallback chain.
    pub fn with_fallbacks(mut self, fallbacks: Vec<SchemeChoice>) -> Self {
        self.fallbacks = fallbacks;
        self
    }
    /// Set the interval request.
    pub fn with_intervals(mut self, req: Firm<IntervalRequest>) -> Self {
        self.intervals = Some(req);
        self
    }
    /// Set the boundary-layer request.
    pub fn with_boundary_layer(mut self, bl: BoundaryLayerIntent) -> Self {
        self.boundary_layer = Some(bl);
        self
    }
}

/// A user vertex-type override on one face corner.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct CornerIntent {
    /// The corner.
    pub corner: CornerKey,
    /// Its requested type (usually `Hard`).
    pub vertex_type: Firm<VertexType>,
}

impl CornerIntent {
    /// A vertex-type override on `corner`.
    pub fn new(corner: CornerKey, vertex_type: Firm<VertexType>) -> Self {
        Self { corner, vertex_type }
    }
}

/// Volumes meshed together by one scheme invocation and committed all-or-nothing:
/// multisweep, polyhedral assemblies.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct MeshGroup {
    /// Group id (unique within the intent).
    pub id: GroupId,
    /// Member pieces (`TopoVolume` or `PartitionCell` ids).
    pub members: Vec<EntityId>,
    /// The group scheme (a volume scheme that supports groups).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scheme: Option<SchemeChoice>,
    /// Fallback chain tried in order after `scheme`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub fallbacks: Vec<SchemeChoice>,
    /// **Deprecated (W0f).** A topology-preserving [`TransformIntent`] over the whole group
    /// (driver transform id `TransformId::GROUP_BASE | group id`). Element conversions (THex,
    /// HTet, QTri, TQuad) cannot be expressed here: they create nodes the group does not own
    /// (rejected by `check_patch`) and are IR -> IR mesh passes instead (see `DESIGN.md`).
    /// Kept for format compatibility; new intents should not use it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub conversion: Option<SchemeChoice>,
}

impl MeshGroup {
    /// A group of `members` with no scheme chosen yet.
    pub fn new(id: GroupId, members: Vec<EntityId>) -> Self {
        Self { id, members, scheme: None, fallbacks: Vec::new(), conversion: None }
    }
    /// Set the group scheme.
    pub fn with_scheme(mut self, choice: SchemeChoice) -> Self {
        self.scheme = Some(choice);
        self
    }
    /// Set the fallback chain.
    pub fn with_fallbacks(mut self, fallbacks: Vec<SchemeChoice>) -> Self {
        self.fallbacks = fallbacks;
        self
    }
}

/// A standalone mesh transform: a **topology-preserving patch edit** (smoothing, node
/// snapping, reclassification of a generated patch) through a registered
/// [`crate::MeshTransform`]. Targets are pieces and, W0f gaps 2, faces and curves inside the
/// region. Runs after every writer of its targets (transforms over a target chain in id
/// order); the last transform over a target is the writer its readers depend on, so a volume
/// meshed after a surface smoothing reads the smoothed patch and a copy of a transformed root
/// is generated from the transformed patch. A transform cannot cover a target together with
/// one of its readers (a dependency cycle). The driver swaps in the rewritten patches only if
/// all succeed. Element conversions (THex, HTet, QTri, TQuad) are not
/// transforms: they are IR -> IR passes over `MeshRead -> MeshBuild` (see `DESIGN.md`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct TransformIntent {
    /// Id (unique within the intent, below [`TransformId::GROUP_BASE`]).
    pub id: TransformId,
    /// The transformed targets: pieces (`TopoVolume` or `PartitionCell` ids; a member of a
    /// group transforms the group's patch), faces (`TopoFace` / `PartitionFace`) and curves of
    /// the region.
    pub targets: Vec<EntityId>,
    /// The registered [`crate::MeshTransform`] and its parameters.
    pub transform: SchemeChoice,
}

impl TransformIntent {
    /// A transform over `targets`.
    pub fn new(id: TransformId, targets: Vec<EntityId>, transform: SchemeChoice) -> Self {
        Self { id, targets, transform }
    }
}

/// A sizing function (Cubit: constant, curvature, linear, interval, inverse, super, bias,
/// field-from-file, geometry-adaptive). Built into a [`crate::SizeField`] by the sizing layer.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub enum SizingSpec {
    /// The same size everywhere.
    Constant {
        /// Element size.
        size: f64,
    },
    /// A named sizing function with its own parameters (curvature, linear, bias, field, ...).
    Function {
        /// Function name.
        name: String,
        /// Its parameters.
        #[serde(default)]
        params: Params,
    },
}

/// Candidate schemes for automatic selection, in priority order per dimension (Cubit auto:
/// surfaces from {pave, submap, triprimitive, map}, volumes from {map, submap, sweep}). An
/// empty list means "the registry's auto list, else every registered scheme of that
/// dimension".
#[derive(Clone, Debug, PartialEq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct AutoPolicy {
    /// Curve candidates.
    #[serde(default)]
    pub curve: Vec<SchemeName>,
    /// Surface candidates.
    #[serde(default)]
    pub surface: Vec<SchemeName>,
    /// Volume candidates.
    #[serde(default)]
    pub volume: Vec<SchemeName>,
}

impl AutoPolicy {
    /// Candidate lists per dimension (empty = registry auto list).
    pub fn new(curve: Vec<SchemeName>, surface: Vec<SchemeName>, volume: Vec<SchemeName>) -> Self {
        Self { curve, surface, volume }
    }
}

/// Model-wide defaults.
#[derive(Clone, Debug, PartialEq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct Defaults {
    /// Default sizing (none: every curve gets a goal of one interval).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sizing: Option<SizingSpec>,
    /// Auto-selection candidates (a non-empty list overrides the registry's auto list).
    #[serde(default)]
    pub auto: AutoPolicy,
}

impl Defaults {
    /// No sizing, registry auto lists.
    pub fn new() -> Self {
        Self::default()
    }
    /// Set the default sizing.
    pub fn with_sizing(mut self, sizing: SizingSpec) -> Self {
        self.sizing = Some(sizing);
        self
    }
    /// Set the auto-selection candidates.
    pub fn with_auto(mut self, auto: AutoPolicy) -> Self {
        self.auto = auto;
        self
    }
}

/// The whole meshing intent of a model (the Intent layer of `docs/design/ir.md` section 5).
///
/// Keyed by entity id: edges and faces of the topology view, `TopoVolume` ids for whole
/// volumes and `PartitionCell` ids for decomposition pieces. `BTreeMap` keeps the serialized
/// form canonical. The solver state of a fallback (`Rejections`) is deliberately **not** part
/// of the intent; it is stored in the `Plan`.
#[derive(Clone, Debug, PartialEq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct MeshIntent {
    /// Per-entity requests.
    #[serde(default)]
    pub entities: BTreeMap<EntityId, EntityIntent>,
    /// Vertex-type overrides (canonical order: sorted by corner, see [`MeshIntent::canonicalize`]).
    #[serde(default)]
    pub corners: Vec<CornerIntent>,
    /// Atomic groups.
    #[serde(default)]
    pub groups: Vec<MeshGroup>,
    /// Standalone topology-preserving mesh transforms over volume targets (and, W0f gaps 2,
    /// over faces and curves inside the region).
    #[serde(default)]
    pub transforms: Vec<TransformIntent>,
    /// User interval relations between edges (W0f gaps 2; PARITY C P1): match intervals
    /// (`sum_equal`), relative intervals and sums over a chain, as rows of the IIA system. The
    /// planner adds each row with `Origin::Intent`; every edge must be in the region.
    #[serde(default)]
    pub intervals: Vec<crate::problem::IntervalConstraint>,
    /// Defaults.
    #[serde(default)]
    pub defaults: Defaults,
}

impl MeshIntent {
    /// An empty intent.
    pub fn new() -> Self {
        Self::default()
    }

    /// The request for `id`, creating an empty one.
    pub fn entity_mut(&mut self, id: EntityId) -> &mut EntityIntent {
        self.entities.entry(id).or_default()
    }

    /// Request a scheme on `id`.
    pub fn set_scheme(&mut self, id: EntityId, choice: SchemeChoice) {
        self.entity_mut(id).scheme = Some(choice);
    }

    /// Request intervals on `id`.
    pub fn set_intervals(&mut self, id: EntityId, req: Firm<IntervalRequest>) {
        self.entity_mut(id).intervals = Some(req);
    }

    /// Sort corner overrides, group members and transform targets (later entries for the same
    /// corner win) and groups and transforms by id, so equal intents serialize identically.
    pub fn canonicalize(&mut self) {
        let mut by: BTreeMap<CornerKey, Firm<VertexType>> = BTreeMap::new();
        for c in &self.corners {
            by.insert(c.corner, c.vertex_type);
        }
        self.corners = by
            .into_iter()
            .map(|(corner, vertex_type)| CornerIntent {
                corner,
                vertex_type,
            })
            .collect();
        for g in &mut self.groups {
            g.members.sort_unstable();
            g.members.dedup();
        }
        self.groups.sort_by_key(|g| g.id);
        for t in &mut self.transforms {
            t.targets.sort_unstable();
            t.targets.dedup();
        }
        self.transforms.sort_by_key(|t| t.id);
        for r in &mut self.intervals {
            r.canonicalize();
        }
    }

    /// Validate what the type system cannot: parameters are Op-codec values, group and
    /// transform ids are unique and in range, sizes and counts are sane.
    pub fn check(&self) -> Result<(), SchemeError> {
        let bad = |what: String| Err(SchemeError::Input(what));
        let params = |who: &str, c: &SchemeChoice| {
            c.params
                .check()
                .map_err(|e| SchemeError::Input(format!("{who}: `{}`: {e}", c.name)))
        };
        for (id, e) in &self.entities {
            for c in e.scheme.iter().chain(e.fallbacks.iter()) {
                params(&id.to_string(), c)?;
            }
            match e.intervals.map(|r| r.value) {
                Some(IntervalRequest::Count(0)) => {
                    return bad(format!("{id}: interval count must be >= 1"));
                }
                Some(IntervalRequest::Size(h)) if !(h.is_finite() && h > 0.0) => {
                    return bad(format!("{id}: interval size must be positive and finite"));
                }
                _ => {}
            }
            if let Some(b) = &e.boundary_layer {
                if !matches!(id.kind(), sm_core::EntityKind::TopoFace | sm_core::EntityKind::PartitionFace) {
                    return bad(format!(
                        "{id}: boundary layers are only supported on faces (curve boundary layers do not exist yet)"
                    ));
                }
                if !(b.first.is_finite()
                    && b.first > 0.0
                    && b.growth.is_finite()
                    && b.growth >= 1.0
                    && b.layers >= 1)
                {
                    return bad(format!(
                        "{id}: boundary layer needs first > 0, growth >= 1 and layers >= 1"
                    ));
                }
                if let Some(v) = b.into.iter().find(|v| !matches!(v.kind(), sm_core::EntityKind::TopoVolume | sm_core::EntityKind::PartitionCell)) {
                    return bad(format!("{id}: boundary layer `into` lists {v}, which is not a volume piece"));
                }
                b.params
                    .check()
                    .map_err(|e| SchemeError::Input(format!("{id}: boundary layer params: {e}")))?;
            }
        }
        let mut gids: Vec<GroupId> = Vec::new();
        for g in &self.groups {
            if gids.contains(&g.id) {
                return bad(format!("group {:?} listed twice", g.id));
            }
            gids.push(g.id);
            if g.members.is_empty() {
                return bad(format!("group {:?} has no members", g.id));
            }
            for c in g
                .scheme
                .iter()
                .chain(g.fallbacks.iter())
                .chain(g.conversion.iter())
            {
                params(&format!("group {:?}", g.id), c)?;
            }
        }
        let mut tids: Vec<TransformId> = Vec::new();
        for t in &self.transforms {
            if t.id.0 >= TransformId::GROUP_BASE || tids.contains(&t.id) {
                return bad(format!("transform {:?} is reserved or listed twice", t.id));
            }
            tids.push(t.id);
            if t.targets.is_empty() {
                return bad(format!("transform {:?} has no targets", t.id));
            }
            params(&format!("transform {:?}", t.id), &t.transform)?;
        }
        for r in &self.intervals {
            r.check()?;
        }
        if let Some(SizingSpec::Function { params: p, .. }) = &self.defaults.sizing {
            p.check()
                .map_err(|e| SchemeError::Input(format!("sizing: {e}")))?;
        }
        Ok(())
    }
}
