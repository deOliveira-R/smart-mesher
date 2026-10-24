//! `project.toml`: format version, units, default tolerances and metadata.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::error::{EngineError, Result};

/// Highest `project.toml` format version this build reads and the one it writes.
pub const FORMAT_VERSION: u32 = 1;

/// Default model tolerances recorded with the project.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Tolerances {
    /// Linear tolerance in project units.
    pub linear: f64,
    /// Angular tolerance in radians.
    pub angular: f64,
}

/// Contents of `project.toml`.
///
/// Unknown keys written by a newer minor version are kept in [`extra`](Self::extra) and written
/// back, so an older tool never silently drops them.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ProjectConfig {
    /// Format version (see [`FORMAT_VERSION`]).
    pub format: u32,
    /// Project name.
    pub name: String,
    /// Free-text description (no control characters).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Length unit of the model, e.g. `"cm"`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub units: Option<String>,
    /// Default tolerances.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tolerances: Option<Tolerances>,
    /// Free-form string metadata, key-sorted.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub metadata: BTreeMap<String, String>,
    /// Keys this build does not know.
    #[serde(flatten)]
    pub extra: BTreeMap<String, toml::Value>,
}

impl ProjectConfig {
    /// A config with the current format version and the given name.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            format: FORMAT_VERSION,
            name: name.into(),
            description: None,
            units: None,
            tolerances: None,
            metadata: BTreeMap::new(),
            extra: BTreeMap::new(),
        }
    }

    /// Check the values that must survive a text round trip and CRLF checkouts: no control
    /// characters in strings, finite positive tolerances.
    pub fn validate(&self) -> Result<()> {
        let plain = |what: &str, s: &str| {
            if s.chars().any(char::is_control) {
                Err(EngineError::Format(format!(
                    "{what} contains a control character"
                )))
            } else {
                Ok(())
            }
        };
        plain("name", &self.name)?;
        if let Some(d) = &self.description {
            plain("description", d)?;
        }
        if let Some(u) = &self.units {
            plain("units", u)?;
        }
        for (k, v) in &self.metadata {
            plain("metadata key", k)?;
            plain("metadata value", v)?;
        }
        if let Some(t) = &self.tolerances
            && !(t.linear.is_finite() && t.linear > 0.0 && t.angular.is_finite() && t.angular > 0.0)
        {
            return Err(EngineError::Format(
                "tolerances must be finite and positive".into(),
            ));
        }
        Ok(())
    }

    /// Render as TOML text.
    pub fn to_toml(&self) -> Result<String> {
        self.validate()?;
        toml::to_string(self).map_err(|e| EngineError::Format(format!("project.toml: {e}")))
    }

    /// Parse TOML text, rejecting formats newer than [`FORMAT_VERSION`].
    pub fn from_toml(text: &str) -> Result<Self> {
        let cfg: ProjectConfig =
            toml::from_str(text).map_err(|e| EngineError::Format(format!("project.toml: {e}")))?;
        if cfg.format > FORMAT_VERSION {
            return Err(EngineError::UnsupportedFormat {
                found: cfg.format,
                supported: FORMAT_VERSION,
            });
        }
        cfg.validate()?;
        Ok(cfg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_is_stable_text() {
        let mut c = ProjectConfig::new("reactor");
        c.units = Some("cm".into());
        c.tolerances = Some(Tolerances {
            linear: 1e-7,
            angular: 1e-9,
        });
        c.metadata.insert("b".into(), "2".into());
        c.metadata.insert("a".into(), "1".into());
        let t = c.to_toml().unwrap();
        assert_eq!(ProjectConfig::from_toml(&t).unwrap(), c);
        assert_eq!(ProjectConfig::from_toml(&t).unwrap().to_toml().unwrap(), t);
        assert!(
            t.find("a = ").unwrap() < t.find("b = ").unwrap(),
            "metadata is key-sorted"
        );
    }

    #[test]
    fn rejects_bad_values() {
        let mut c = ProjectConfig::new("x");
        c.tolerances = Some(Tolerances {
            linear: f64::NAN,
            angular: 1.0,
        });
        assert!(c.to_toml().is_err());
        c.tolerances = Some(Tolerances {
            linear: 0.0,
            angular: 1.0,
        });
        assert!(c.validate().is_err());
        assert!(ProjectConfig::from_toml("name = 3").is_err());
        assert!(matches!(
            ProjectConfig::from_toml("format = 2\nname = \"n\""),
            Err(EngineError::UnsupportedFormat {
                found: 2,
                supported: 1
            })
        ));
    }
}
