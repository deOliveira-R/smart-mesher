//! Diagnostics policy and clocks: front-end behavior the contract leaves to the engine.
//!
//! Ops report [`Diagnostic`]s in their `Delta`; they are never part of a state or node hash.
//! The policy decides what the caller sees and whether a warning stops an op from being recorded.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};
use sm_op_api::{Diagnostic, Severity};

/// Which diagnostics are reported back to the caller.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Verbosity {
    /// Report none.
    Quiet,
    /// Report warnings only (default).
    #[default]
    Warnings,
    /// Report warnings and informational notes.
    Info,
}

/// What to do with the diagnostics of an applied op.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct DiagnosticPolicy {
    /// What is reported in the [`ApplyReport`](crate::ApplyReport).
    pub verbosity: Verbosity,
    /// Refuse to record an op that produced a (non-suppressed) warning. The op's effects are
    /// discarded and the call fails with [`EngineError::StoppedOnWarning`](crate::EngineError);
    /// independent of `verbosity`.
    pub stop_on_warning: bool,
    /// Diagnostic codes that are neither reported nor able to stop an op.
    pub suppress: BTreeSet<String>,
}

impl DiagnosticPolicy {
    /// The diagnostics to show (in op order) and how many were hidden by verbosity or suppression.
    pub fn report(&self, all: &[Diagnostic]) -> (Vec<Diagnostic>, usize) {
        let shown: Vec<Diagnostic> = all
            .iter()
            .filter(|d| !self.suppress.contains(&d.code))
            .filter(|d| match d.severity {
                Severity::Warning => self.verbosity >= Verbosity::Warnings,
                Severity::Info => self.verbosity >= Verbosity::Info,
            })
            .cloned()
            .collect();
        let hidden = all.len() - shown.len();
        (shown, hidden)
    }

    /// The warnings that stop the op under this policy (empty unless `stop_on_warning`).
    pub fn blockers(&self, all: &[Diagnostic]) -> Vec<Diagnostic> {
        if !self.stop_on_warning {
            return Vec::new();
        }
        all.iter()
            .filter(|d| d.severity == Severity::Warning && !self.suppress.contains(&d.code))
            .cloned()
            .collect()
    }
}

/// Source of node timestamps (RFC 3339, UTC). Node metadata is part of a node's hash, so tests
/// that compare node hashes across runs use [`FixedClock`].
pub trait Clock: Send + Sync {
    /// The current time as `YYYY-MM-DDTHH:MM:SSZ`.
    fn now(&self) -> String;
}

/// The system clock.
#[derive(Clone, Copy, Debug, Default)]
pub struct SystemClock;

impl Clock for SystemClock {
    fn now(&self) -> String {
        let secs = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_secs());
        rfc3339_utc(secs)
    }
}

/// A clock that always returns the same time.
#[derive(Clone, Debug)]
pub struct FixedClock(pub String);

impl Clock for FixedClock {
    fn now(&self) -> String {
        self.0.clone()
    }
}

/// Format seconds since the Unix epoch as `YYYY-MM-DDTHH:MM:SSZ` (proleptic Gregorian).
pub fn rfc3339_utc(secs: u64) -> String {
    let (days, rem) = (secs / 86_400, secs % 86_400);
    // Civil-from-days (H. Hinnant), valid for the whole u64 range we can meet.
    let z = days as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!(
        "{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}Z",
        rem / 3600,
        rem % 3600 / 60,
        rem % 60
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rfc3339_known_instants() {
        assert_eq!(rfc3339_utc(0), "1970-01-01T00:00:00Z");
        assert_eq!(rfc3339_utc(951_782_400), "2000-02-29T00:00:00Z");
        assert_eq!(rfc3339_utc(1_700_000_000), "2023-11-14T22:13:20Z");
        assert_eq!(rfc3339_utc(4_102_444_799), "2099-12-31T23:59:59Z");
    }

    fn d(sev: Severity, code: &str) -> Diagnostic {
        Diagnostic::new(sev, code, "m", vec![])
    }

    #[test]
    fn policy_filters_and_blocks() {
        let all = [d(Severity::Info, "a.info"), d(Severity::Warning, "a.warn"), d(Severity::Warning, "b.warn")];
        let mut p = DiagnosticPolicy::default();
        assert_eq!(p.report(&all).0.len(), 2);
        assert!(p.blockers(&all).is_empty());
        p.verbosity = Verbosity::Info;
        assert_eq!(p.report(&all), (all.to_vec(), 0));
        p.verbosity = Verbosity::Quiet;
        assert_eq!(p.report(&all), (vec![], 3));
        p.stop_on_warning = true;
        assert_eq!(p.blockers(&all).len(), 2);
        p.suppress.insert("a.warn".into());
        assert_eq!(p.blockers(&all).len(), 1);
        assert_eq!(p.blockers(&all)[0].code, "b.warn");
    }
}
