//! Serializable meshing intent: what the user (or an Op) asked for. Pure data, stored in an Op
//! and in git; the driver turns it into a [`crate::MeshingProblem`].

use crate::error::SchemeError;
use crate::firm::{Firm, Firmness};
use crate::target::GroupId;
use crate::vertex::VertexType;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_topo_api::CornerKey;
use std::collections::BTreeMap;

/// Registered name of a scheme (`"equal"`, `"map"`, `"sweep"`, ...). Names are unique per
/// dimension in a [`crate::SchemeRegistry`].
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
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

/// Scheme parameters: a JSON object owned by the scheme (same value model as `sm-op-api`
/// params). Schemes read typed fields with [`Params::get`].
#[derive(Clone, Debug, PartialEq, Default, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Params(pub serde_json::Value);

impl Params {
    /// Empty parameters.
    pub fn empty() -> Self {
        Self(serde_json::Value::Null)
    }

    /// Typed field `key`; `Ok(None)` if absent. A field of the wrong type is an error naming
    /// `scheme`.
    pub fn get<T: DeserializeOwned>(&self, scheme: &str, key: &str) -> Result<Option<T>, SchemeError> {
        match self.0.get(key) {
            None | Some(serde_json::Value::Null) => Ok(None),
            Some(v) => serde_json::from_value(v.clone())
                .map(Some)
                .map_err(|e| SchemeError::BadParams { scheme: SchemeName::new(scheme), reason: format!("{key}: {e}") }),
        }
    }
}

/// A scheme request: name, parameters and firmness.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SchemeChoice {
    /// Scheme name.
    pub name: SchemeName,
    /// Scheme parameters.
    #[serde(default, skip_serializing_if = "is_null")]
    pub params: Params,
    /// Firmness: a `Hard` choice that conflicts with an imposition is an error; a `Soft` or
    /// `Default` one is overridden.
    #[serde(default)]
    pub firmness: Firmness,
}

fn is_null(p: &Params) -> bool {
    p.0.is_null()
}

impl SchemeChoice {
    /// A choice with no parameters.
    pub fn new(name: &str, firmness: Firmness) -> Self {
        Self { name: SchemeName::new(name), params: Params::empty(), firmness }
    }
    /// A choice with parameters.
    pub fn with_params(name: &str, params: serde_json::Value, firmness: Firmness) -> Self {
        Self { name: SchemeName::new(name), params: Params(params), firmness }
    }
}

/// An interval request on a curve.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum IntervalRequest {
    /// Exactly this many intervals (a goal if not `Hard`).
    Count(u32),
    /// Target element size; the goal is `length / size`.
    Size(f64),
}

/// Boundary-layer request on a face (into the adjacent volumes). Refused on deck faces.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct BoundaryLayerIntent {
    /// First layer thickness.
    pub first: f64,
    /// Growth ratio.
    pub growth: f64,
    /// Number of layers.
    pub layers: u32,
}

/// Everything asked for one entity (edge, face, volume or piece).
#[derive(Clone, Debug, PartialEq, Default, Serialize, Deserialize)]
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

/// A user vertex-type override on one face corner.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CornerIntent {
    /// The corner.
    pub corner: CornerKey,
    /// Its requested type (usually `Hard`).
    pub vertex_type: Firm<VertexType>,
}

/// Volumes meshed together by one scheme invocation and committed all-or-nothing:
/// multisweep, merged volumes that must be converted in one call (THex), polyhedral assemblies.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MeshGroup {
    /// Group id (unique within the intent).
    pub id: GroupId,
    /// Member pieces (`TopoVolume` or `PartitionCell` ids).
    pub members: Vec<EntityId>,
    /// The group scheme (a volume scheme that supports groups).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scheme: Option<SchemeChoice>,
    /// Optional conversion applied to the whole group after generation (thex, htet, qtri).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub conversion: Option<SchemeChoice>,
}

/// A sizing function (Cubit: constant, curvature, linear, interval, inverse, super, bias,
/// field-from-file, geometry-adaptive). Built into a [`crate::SizeField`] by the sizing layer.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
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
/// empty list means "every registered scheme of that dimension".
#[derive(Clone, Debug, PartialEq, Default, Serialize, Deserialize)]
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

/// Model-wide defaults.
#[derive(Clone, Debug, PartialEq, Default, Serialize, Deserialize)]
pub struct Defaults {
    /// Default sizing (none: every curve gets a goal of one interval).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sizing: Option<SizingSpec>,
    /// Auto-selection candidates.
    #[serde(default)]
    pub auto: AutoPolicy,
}

/// The whole meshing intent of a model (the Intent layer of `docs/design/ir.md` §5).
///
/// Keyed by entity id: edges and faces of the topology view, `TopoVolume` ids for whole
/// volumes and `PartitionCell` ids for decomposition pieces. `BTreeMap` keeps the serialized
/// form canonical.
#[derive(Clone, Debug, PartialEq, Default, Serialize, Deserialize)]
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
    /// Defaults.
    #[serde(default)]
    pub defaults: Defaults,
}

impl MeshIntent {
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

    /// Sort corner overrides and group members (later entries for the same corner win) and
    /// groups by id, so equal intents serialize identically.
    pub fn canonicalize(&mut self) {
        let mut by: BTreeMap<CornerKey, Firm<VertexType>> = BTreeMap::new();
        for c in &self.corners {
            by.insert(c.corner, c.vertex_type);
        }
        self.corners = by.into_iter().map(|(corner, vertex_type)| CornerIntent { corner, vertex_type }).collect();
        for g in &mut self.groups {
            g.members.sort_unstable();
            g.members.dedup();
        }
        self.groups.sort_by_key(|g| g.id);
    }
}
