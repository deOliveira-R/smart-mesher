//! The topology conformance suite: every scenario that pins the behavior of
//! `sm_topo_api::TopologyEdit` implementations, written once and generic over the implementation.
//!
//! The reference `CellComplex` runs it from `sm-topo-api/tests`, and every other implementation
//! (the production store, the paranoid twin that runs both in lock step) calls
//! [`run_topology_conformance`] with a factory. Scenarios use `sm_topo_api::fixtures`, so they
//! share one definition of "a brick" and cannot drift between crates.
//!
//! Timing assertions are not part of the suite (a twin would measure two stores plus the
//! comparison); implementations keep those in their own tests.

use sm_geom_api::ModelTolerance;
use sm_topo_api::TopologyEdit;
use std::panic::{AssertUnwindSafe, catch_unwind};

mod bridge_chain;
mod flags;
mod freeze;
mod gaps3;
mod gaps4;
mod join;
mod overlay;
mod overlay_volume;
mod reference;
mod shells;

/// What a conformance scenario needs from an implementation: the edit trait, `Clone` (overlay
/// scenarios snapshot the complex as a host), and thread safety (hosts are shared in an `Arc`).
pub trait Sys: TopologyEdit + Clone + Send + Sync + 'static {}

impl<T: TopologyEdit + Clone + Send + Sync + 'static> Sys for T {}

/// Factory handed to each scenario: a fresh empty complex, with the given model tolerance or the
/// default one for `None`.
pub(crate) type Mk<'a, E> = dyn Fn(Option<ModelTolerance>) -> E + Sync + 'a;

type Scenario<E> = (&'static str, fn(&Mk<E>));

/// Every scenario as `(group, [(name, fn)])`.
fn groups<E: Sys>() -> Vec<(&'static str, Vec<Scenario<E>>)> {
    vec![
        ("reference", reference::scenarios::<E>()),
        ("bridge_chain", bridge_chain::scenarios::<E>()),
        ("freeze", freeze::scenarios::<E>()),
        ("gaps3", gaps3::scenarios::<E>()),
        ("gaps4", gaps4::scenarios::<E>()),
        ("join", join::scenarios::<E>()),
        ("overlay", overlay::scenarios::<E>()),
        ("overlay_volume", overlay_volume::scenarios::<E>()),
        ("shells", shells::scenarios::<E>()),
        ("flags", flags::scenarios::<E>()),
    ]
}

/// The number of scenarios in the suite.
pub fn scenario_count() -> usize {
    // The count does not depend on the implementation.
    groups::<sm_topo_api::CellComplex>().iter().map(|(_, s)| s.len()).sum()
}

fn message(e: &(dyn std::any::Any + Send)) -> String {
    e.downcast_ref::<String>().cloned().or_else(|| e.downcast_ref::<&str>().map(|s| (*s).to_string())).unwrap_or_else(|| "non-string panic".into())
}

/// Run every conformance scenario against the implementation `make` builds.
///
/// `make` is `Sync` because some scenarios build models on several threads.
/// `make(None)` must return a fresh empty complex with the default tolerance and
/// `make(Some(tol))` one with model tolerance `tol`. All scenarios run even if some fail;
/// the final panic names each failed scenario (`group::scenario`) with its message.
///
/// # Panics
/// If any scenario fails.
pub fn run_topology_conformance<E: Sys>(name: &str, make: &(dyn Fn(Option<ModelTolerance>) -> E + Sync)) {
    let mut failed: Vec<String> = Vec::new();
    let mut total = 0usize;
    for (group, scenarios) in groups::<E>() {
        for (scenario, f) in scenarios {
            total += 1;
            if let Err(e) = catch_unwind(AssertUnwindSafe(|| f(make))) {
                failed.push(format!("{group}::{scenario}: {}", message(&*e)));
            }
        }
    }
    assert!(failed.is_empty(), "{name}: {} of {total} topology conformance scenarios failed:\n{}", failed.len(), failed.iter().map(|f| format!("  - {f}")).collect::<Vec<_>>().join("\n"));
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_topo_api::CellComplex;

    #[test]
    fn the_reference_passes_every_scenario() {
        run_topology_conformance("reference", &|t| t.map_or_else(CellComplex::new, CellComplex::with_tolerance));
        assert!(scenario_count() > 50);
    }

    #[test]
    fn a_failing_implementation_is_reported_by_scenario_name() {
        // An implementation that cannot honor a model tolerance fails the scenarios that ask for one.
        let r = catch_unwind(|| run_topology_conformance("broken", &|t| if t.is_some() { panic!("no tolerance support") } else { CellComplex::new() }));
        let msg = message(&*r.expect_err("the suite must fail"));
        assert!(msg.starts_with("broken: ") && msg.contains("topology conformance scenarios failed"), "{msg}");
        assert!(msg.contains("no tolerance support") && msg.contains("  - reference::"), "{msg}");
    }
}
