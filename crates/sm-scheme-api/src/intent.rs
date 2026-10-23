//! Serializable meshing intent: scheme choices, parameters, interval specs, vertex types and job
//! groups. This is what a `SetScheme` / `Mesh` op stores in the timeline (serde feature); the
//! driver turns it into jobs.

use crate::target::Target;
use sm_core::EntityId;
use sm_topo_api::CornerKey;
use std::collections::BTreeMap;
use std::fmt;

/// How binding a setting is. Ordered: `Default < Soft < Hard`.
///
/// Cubit semantics: `Default` = nobody chose; `Soft` = chosen automatically (auto selection,
/// imposed by a parent scheme, corner picking) and may be overridden; `Hard` = chosen by the user
/// and must hold, or meshing fails.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Firmness {
    /// Nobody chose.
    #[default]
    Default,
    /// Chosen automatically; may be overridden.
    Soft,
    /// Chosen by the user; must hold.
    Hard,
}

/// Registered name of a scheme (`"equal"`, `"map"`, `"sweep"`, ...). Names are unique per
/// [`crate::SchemeRegistry`] across all dimensions.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct SchemeName(pub String);

impl SchemeName {
    /// Build from a string.
    pub fn new(s: &str) -> Self {
        Self(s.to_owned())
    }
    /// The name.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for SchemeName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

/// One parameter value. Floats must be finite (checked by [`Params::check`]) so the intent
/// stays representable in the op codec (TOML).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ParamValue {
    /// Boolean.
    Bool(bool),
    /// Integer.
    Int(i64),
    /// Finite float.
    Float(f64),
    /// String (an enum option such as `"translate"`).
    Str(String),
    /// Entity ids (sources, targets, hard points).
    Ids(Vec<EntityId>),
    /// Floats (bias tables, pinpoint parameters).
    Floats(Vec<f64>),
}

/// Scheme parameters: a sorted string map, so serialization and hashing are deterministic.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct Params(pub BTreeMap<String, ParamValue>);

impl Params {
    /// Empty parameters.
    pub fn new() -> Self {
        Self::default()
    }
    /// Builder-style insert.
    pub fn with(mut self, key: &str, v: ParamValue) -> Self {
        self.0.insert(key.to_owned(), v);
        self
    }
    /// A raw value.
    pub fn get(&self, key: &str) -> Option<&ParamValue> {
        self.0.get(key)
    }
    /// A float (an `Int` converts).
    pub fn float(&self, key: &str) -> Option<f64> {
        match self.0.get(key)? {
            ParamValue::Float(x) => Some(*x),
            ParamValue::Int(i) => Some(*i as f64),
            _ => None,
        }
    }
    /// An integer.
    pub fn int(&self, key: &str) -> Option<i64> {
        match self.0.get(key)? {
            ParamValue::Int(i) => Some(*i),
            _ => None,
        }
    }
    /// A string.
    pub fn str(&self, key: &str) -> Option<&str> {
        match self.0.get(key)? {
            ParamValue::Str(s) => Some(s),
            _ => None,
        }
    }
    /// Entity ids.
    pub fn ids(&self, key: &str) -> Option<&[EntityId]> {
        match self.0.get(key)? {
            ParamValue::Ids(v) => Some(v),
            _ => None,
        }
    }
    /// A boolean.
    pub fn bool(&self, key: &str) -> Option<bool> {
        match self.0.get(key)? {
            ParamValue::Bool(b) => Some(*b),
            _ => None,
        }
    }
    /// Reject non-finite floats; returns the offending key.
    pub fn check(&self) -> Result<(), String> {
        for (k, v) in &self.0 {
            let ok = match v {
                ParamValue::Float(x) => x.is_finite(),
                ParamValue::Floats(xs) => xs.iter().all(|x| x.is_finite()),
                _ => true,
            };
            if !ok {
                return Err(k.clone());
            }
        }
        Ok(())
    }
}

/// A scheme with its parameters.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SchemeSpec {
    /// Registered scheme name.
    pub name: SchemeName,
    /// Scheme-specific parameters.
    pub params: Params,
}

impl SchemeSpec {
    /// A scheme without parameters.
    pub fn named(name: &str) -> Self {
        Self { name: SchemeName::new(name), params: Params::new() }
    }
    /// Builder-style parameter.
    pub fn with(mut self, key: &str, v: ParamValue) -> Self {
        self.params = self.params.with(key, v);
        self
    }
}

/// Which scheme a target (or job group) uses.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SchemeChoice {
    /// Exactly this scheme.
    Fixed(SchemeSpec),
    /// Try in order; the first that is applicable, plans and executes wins (Cubit: TriMesh =
    /// TriDelaunay -> TriAdvance -> QTri).
    Chain(Vec<SchemeSpec>),
    /// Auto selection: the highest [`crate::Applicability`] score among `candidates` (all
    /// registered schemes of the target's dimension if empty); ties break by scheme name.
    Auto {
        /// Candidate scheme names (empty = all of the dimension).
        candidates: Vec<SchemeName>,
    },
}

impl SchemeChoice {
    /// `Fixed` shorthand.
    pub fn fixed(name: &str) -> Self {
        SchemeChoice::Fixed(SchemeSpec::named(name))
    }
}

/// A scheme choice with its firmness.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SchemeIntent {
    /// The choice.
    pub choice: SchemeChoice,
    /// Firmness (a user `Hard` choice beats a parent's `Soft` imposition).
    pub firmness: Firmness,
}

/// Interval request on an edge (or every edge of a face/volume).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum IntervalSpec {
    /// An interval count.
    Count(u32),
    /// A target element size (count = length / size, rounded by the solver).
    Size(f64),
}

/// Cubit surface vertex type: how many elements meet at a boundary vertex of a structured face
/// (End 1 ≈ 90°, Side 2 ≈ 180°, Corner 3 ≈ 270°, Reversal 4 ≈ 360°). Stored per vertex **use**
/// ([`CornerKey`]), because one vertex can have different types on different faces.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum VertexType {
    /// One element: a logical corner of a mapped face (interior angle ≈ 90°).
    End,
    /// Two elements: lies inside a logical side (≈ 180°).
    Side,
    /// Three elements (≈ 270°).
    Corner,
    /// Four elements (≈ 360°, e.g. the tip of a slit).
    Reversal,
}

impl VertexType {
    /// The turn contributed to the logical boundary, in quarter turns (End +1, Side 0, Corner -1,
    /// Reversal -2). A mapped face needs a sum of 4 over its loop.
    pub const fn quarter_turns(self) -> i32 {
        match self {
            VertexType::End => 1,
            VertexType::Side => 0,
            VertexType::Corner => -1,
            VertexType::Reversal => -2,
        }
    }

    /// Nearest type for an interior angle in radians (the rule auto vertex typing starts from;
    /// corner pickers refine it globally).
    pub fn from_angle(angle: f64) -> Self {
        let q = angle / core::f64::consts::FRAC_PI_2;
        if q < 1.5 {
            VertexType::End
        } else if q < 2.5 {
            VertexType::Side
        } else if q < 3.5 {
            VertexType::Corner
        } else {
            VertexType::Reversal
        }
    }
}

/// A vertex type with firmness (user = `Hard`, picked = `Soft`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VertexTypeSetting {
    /// The type.
    pub ty: VertexType,
    /// Firmness.
    pub firmness: Firmness,
}

/// Per-target intent.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TargetIntent {
    /// Scheme choice (none = auto, `Default` firmness).
    pub scheme: Option<SchemeIntent>,
    /// Interval request; on a face or volume it applies to every edge below without its own.
    pub intervals: Option<(IntervalSpec, Firmness)>,
}

/// What kind of atomic multi-target job a group is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum GroupKind {
    /// Volumes swept together (many-to-one / many-to-many through a volume chain).
    Multisweep,
    /// Merged volumes that must be meshed in one call (THex, polyhedron scheme setup).
    Cluster,
}

/// A set of targets meshed by **one** job (one patch, committed all-or-nothing).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GroupIntent {
    /// Kind.
    pub kind: GroupKind,
    /// Members (sorted and deduplicated by the driver).
    pub members: Vec<Target>,
    /// The scheme of the group.
    pub scheme: SchemeIntent,
}

/// A mesh transform to run after the targets are meshed (THex, QTri, HTet, boundary layers).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TransformIntent {
    /// Registered transform scheme.
    pub scheme: SchemeSpec,
    /// The targets it rewrites (all of them atomically).
    pub targets: Vec<Target>,
}

/// The whole meshing intent of a model: what the timeline stores.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MeshIntent {
    /// Targets to mesh (with their closure). Usually volumes or pieces; faces alone are allowed
    /// (surface meshes stay in the ledger; only volume cells reach `MeshBuild`).
    pub mesh: Vec<Target>,
    /// Per-target settings.
    #[cfg_attr(feature = "serde", serde(with = "crate::serde_pairs"))]
    pub targets: BTreeMap<Target, TargetIntent>,
    /// Vertex types per vertex use.
    #[cfg_attr(feature = "serde", serde(with = "crate::serde_pairs"))]
    pub vertex_types: BTreeMap<CornerKey, VertexTypeSetting>,
    /// Atomic multi-target groups.
    pub groups: Vec<GroupIntent>,
    /// Post-meshing transforms.
    pub transforms: Vec<TransformIntent>,
    /// Default element size when nothing more specific is set.
    pub default_size: Option<f64>,
    /// Schemes excluded per target (written by the driver's fallback loop, kept for replay).
    #[cfg_attr(feature = "serde", serde(with = "crate::serde_pairs"))]
    pub excluded: BTreeMap<Target, Vec<SchemeName>>,
}

impl MeshIntent {
    /// The intent of one target (default if none).
    pub fn target(&self, t: &Target) -> TargetIntent {
        self.targets.get(t).cloned().unwrap_or_default()
    }
    /// Set the scheme of a target.
    pub fn set_scheme(&mut self, t: Target, choice: SchemeChoice, firmness: Firmness) {
        self.targets.entry(t).or_default().scheme = Some(SchemeIntent { choice, firmness });
    }
    /// Set the intervals of a target.
    pub fn set_intervals(&mut self, t: Target, spec: IntervalSpec, firmness: Firmness) {
        self.targets.entry(t).or_default().intervals = Some((spec, firmness));
    }
}
