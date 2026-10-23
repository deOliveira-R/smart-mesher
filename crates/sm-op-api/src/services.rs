//! Injected services: a typed map of long-lived objects (geometry kernel, mesher) that ops reach
//! through `ApplyCtx::service` instead of globals.

use std::any::{Any, TypeId};
use std::collections::BTreeMap;
use std::sync::Arc;

/// A type-indexed bag of shared services, supplied to `Engine::new`.
///
/// Services must behave as pure functions of their inputs from the op's point of view;
/// otherwise replay diverges (and the engine reports `Diverged`).
#[derive(Clone, Default)]
pub struct Services {
    map: BTreeMap<TypeId, (&'static str, Arc<dyn Any + Send + Sync>)>,
}

impl Services {
    /// An empty bag.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add (or replace) the service of type `S`.
    #[must_use]
    pub fn with<S: Any + Send + Sync>(mut self, s: S) -> Self {
        self.map
            .insert(TypeId::of::<S>(), (std::any::type_name::<S>(), Arc::new(s)));
        self
    }

    /// The service of type `S`, if present.
    pub fn get<S: Any + Send + Sync>(&self) -> Option<&S> {
        self.map.get(&TypeId::of::<S>())?.1.downcast_ref::<S>()
    }
}

impl std::fmt::Debug for Services {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_list()
            .entries(self.map.values().map(|(n, _)| n))
            .finish()
    }
}
