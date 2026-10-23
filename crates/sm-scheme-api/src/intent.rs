//! Serializable meshing intent: scheme choices, parameters, sizes and vertex types per entity.
//!
//! Everything here is plain serde data (no trait objects, no floats that TOML cannot hold once
//! [`Params::check`] passed), so a `SetScheme`/`Mesh` op can carry it and the timeline can store it
//! in git. The registry turns a [`SchemeId`] back into code.

use crate::error::SchemeError;
use crate::key::ShapeKey;
use crate::vertex::CornerAssignment;
use core::fmt;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// How strongly a setting holds (Cubit's default / soft / hard).
///
/// `Default` is the driver's own choice, `Soft` was assigned automatically (another scheme or
/// a corner picker may override it), `Hard` was set by the user and is never overridden; a
/// conflict between two `Hard` settings is an error.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Firmness {
    /// Driver default.
    #[default]
    Default,
    /// Automatically assigned; may be overridden.
    Soft,
    /// User-specified; never overridden.
    Hard,
}

/// Stable scheme name, e.g. `"curve.equal"`, `"surface.map"`, `"volume.sweep"`.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct SchemeId(pub String);

impl SchemeId {
    /// Build from a name.
    pub fn new(name: &str) -> Self {
        SchemeId(name.to_owned())
    }
    /// The name.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl From<&str> for SchemeId {
    fn from(s: &str) -> Self {
        SchemeId::new(s)
    }
}

impl fmt::Display for SchemeId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

/// A scheme parameter value: a closed tree that maps 1:1 to TOML/JSON (no null).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ParamValue {
    /// Boolean.
    Bool(bool),
    /// Integer.
    Int(i64),
    /// Finite float (checked by [`Params::check`]).
    Float(f64),
    /// String.
    Str(String),
    /// An entity, e.g. a sweep source.
    Key(ShapeKey),
    /// A list.
    List(Vec<ParamValue>),
    /// A nested table, ordered by key.
    Map(BTreeMap<String, ParamValue>),
}

impl ParamValue {
    fn check(&self, path: &str) -> Result<(), SchemeError> {
        match self {
            ParamValue::Float(x) if !x.is_finite() => Err(SchemeError::BadParams(format!("{path}: non-finite float"))),
            ParamValue::List(v) => v.iter().enumerate().try_for_each(|(i, x)| x.check(&format!("{path}[{i}]"))),
            ParamValue::Map(m) => m.iter().try_for_each(|(k, x)| x.check(&format!("{path}.{k}"))),
            _ => Ok(()),
        }
    }

    /// The keys of a `Key` or a list of `Key`s.
    pub fn keys(&self) -> Option<Vec<ShapeKey>> {
        match self {
            ParamValue::Key(k) => Some(vec![*k]),
            ParamValue::List(v) => v
                .iter()
                .map(|x| match x {
                    ParamValue::Key(k) => Some(*k),
                    _ => None,
                })
                .collect(),
            _ => None,
        }
    }
}

/// Scheme parameters by name (ordered, so serialization is canonical).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Params(pub BTreeMap<String, ParamValue>);

impl Params {
    /// No parameters.
    pub fn new() -> Self {
        Self::default()
    }
    /// Builder: set `name` to `v`.
    #[must_use]
    pub fn with(mut self, name: &str, v: ParamValue) -> Self {
        self.0.insert(name.to_owned(), v);
        self
    }
    /// The value of `name`.
    pub fn get(&self, name: &str) -> Option<&ParamValue> {
        self.0.get(name)
    }
    /// An integer parameter.
    pub fn int(&self, name: &str) -> Option<i64> {
        match self.get(name)? {
            ParamValue::Int(i) => Some(*i),
            _ => None,
        }
    }
    /// A float parameter (integers widen).
    pub fn float(&self, name: &str) -> Option<f64> {
        match self.get(name)? {
            ParamValue::Float(x) => Some(*x),
            ParamValue::Int(i) => Some(*i as f64),
            _ => None,
        }
    }
    /// A key or key-list parameter.
    pub fn keys(&self, name: &str) -> Option<Vec<ShapeKey>> {
        self.get(name)?.keys()
    }
    /// Reject values TOML cannot hold (non-finite floats).
    pub fn check(&self) -> Result<(), SchemeError> {
        self.0.iter().try_for_each(|(k, v)| v.check(k))
    }
}

/// A scheme with its parameters.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SchemeSpec {
    /// The scheme.
    pub scheme: SchemeId,
    /// Its parameters.
    #[serde(default)]
    pub params: Params,
}

impl SchemeSpec {
    /// A scheme without parameters.
    pub fn new(name: &str) -> Self {
        SchemeSpec { scheme: SchemeId::new(name), params: Params::new() }
    }
    /// Builder: add a parameter.
    #[must_use]
    pub fn with(mut self, name: &str, v: ParamValue) -> Self {
        self.params = self.params.with(name, v);
        self
    }
}

/// Which scheme(s) to try on an entity.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum SchemeChoice {
    /// Automatic selection: every registered auto scheme of the entity's dimension, ranked by
    /// `applicability` score.
    #[default]
    Auto,
    /// An explicit chain, tried in order (each entry followed by its registry fallback chain).
    /// With `Hard` firmness auto selection is never reached; otherwise it follows the chain.
    Chain {
        /// The schemes in preference order.
        schemes: Vec<SchemeSpec>,
        /// How firmly the chain holds.
        firmness: Firmness,
    },
}

impl SchemeChoice {
    /// A single hard scheme.
    pub fn hard(spec: SchemeSpec) -> Self {
        SchemeChoice::Chain { schemes: vec![spec], firmness: Firmness::Hard }
    }
}

/// Size intent of an entity. Set on a face or volume, it applies to every curve below it that
/// has no closer setting (Cubit's size propagation).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum SizeSpec {
    /// An interval count (meaningful on curves).
    Intervals {
        /// The count.
        n: u32,
        /// `Hard` = fixed; otherwise a goal the interval solve may move.
        firmness: Firmness,
    },
    /// A target element size.
    Size {
        /// Target edge length.
        size: f64,
        /// How firmly the size holds (only `Hard` counts become fixed).
        firmness: Firmness,
    },
}

/// Intent on one entity.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct EntityIntent {
    /// Scheme choice.
    #[serde(default)]
    pub scheme: SchemeChoice,
    /// Size or interval count.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub size: Option<SizeSpec>,
    /// Vertex types on a face's corners (per surface, Cubit "surface vertex type").
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub vertex_types: Vec<CornerAssignment>,
}

/// Several volumes (or pieces) meshed by one scheme call, atomically (multisweep, merged
/// volumes meshed together).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GroupIntent {
    /// The members (canonical keys, sorted, unique).
    pub targets: Vec<ShapeKey>,
    /// The scheme for the whole group.
    pub scheme: SchemeSpec,
}

/// The whole meshing intent: per-entity settings, atomic groups and global defaults.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MeshIntent {
    /// Per-entity intent, sorted by key, unique (a list of pairs so JSON/TOML keys stay strings).
    #[serde(default)]
    pub entities: Vec<(ShapeKey, EntityIntent)>,
    /// Atomic multi-volume groups.
    #[serde(default)]
    pub groups: Vec<GroupIntent>,
    /// Size used for curves without any size setting (`None` = driver option).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub default_size: Option<f64>,
}

impl MeshIntent {
    /// Empty intent.
    pub fn new() -> Self {
        Self::default()
    }

    /// The intent of `key` (canonicalized), if any.
    pub fn get(&self, key: ShapeKey) -> Option<&EntityIntent> {
        let key = key.canonical();
        self.entities.binary_search_by(|(k, _)| k.cmp(&key)).ok().map(|i| &self.entities[i].1)
    }

    /// The intent of `key`, created empty if missing (keeps the list sorted).
    pub fn entry(&mut self, key: ShapeKey) -> &mut EntityIntent {
        let key = key.canonical();
        let i = match self.entities.binary_search_by(|(k, _)| k.cmp(&key)) {
            Ok(i) => i,
            Err(i) => {
                self.entities.insert(i, (key, EntityIntent::default()));
                i
            }
        };
        &mut self.entities[i].1
    }

    /// Set the scheme choice of `key`.
    pub fn set_scheme(&mut self, key: ShapeKey, choice: SchemeChoice) {
        self.entry(key).scheme = choice;
    }

    /// Set the size of `key`.
    pub fn set_size(&mut self, key: ShapeKey, size: SizeSpec) {
        self.entry(key).size = Some(size);
    }

    /// Check structural rules after deserialization: sorted unique keys, finite numbers,
    /// sorted unique group members, no entity in two groups.
    pub fn check(&self) -> Result<(), SchemeError> {
        if self.entities.windows(2).any(|w| w[0].0 >= w[1].0) || self.entities.iter().any(|(k, _)| *k != k.canonical()) {
            return Err(SchemeError::BadParams("intent entities must be canonical, sorted and unique".into()));
        }
        for (k, e) in &self.entities {
            if let SchemeChoice::Chain { schemes, .. } = &e.scheme {
                for s in schemes {
                    s.params.check().map_err(|err| SchemeError::BadParams(format!("{k}: {err}")))?;
                }
            }
            if let Some(SizeSpec::Size { size, .. }) = e.size
                && !(size.is_finite() && size > 0.0)
            {
                return Err(SchemeError::BadParams(format!("{k}: size must be finite and positive")));
            }
        }
        let mut seen = std::collections::BTreeSet::new();
        for g in &self.groups {
            if g.targets.is_empty() || g.targets.windows(2).any(|w| w[0] >= w[1]) {
                return Err(SchemeError::BadParams("group targets must be non-empty, sorted and unique".into()));
            }
            for t in &g.targets {
                if !seen.insert(*t) {
                    return Err(SchemeError::BadParams(format!("{t} is in two groups")));
                }
            }
            g.scheme.params.check()?;
        }
        if let Some(s) = self.default_size
            && !(s.is_finite() && s > 0.0)
        {
            return Err(SchemeError::BadParams("default size must be finite and positive".into()));
        }
        Ok(())
    }
}
