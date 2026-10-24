//! Injected services: a typed map of long-lived objects (geometry kernel, mesher) that ops reach
//! through `ApplyCtx::service` instead of globals.

use std::any::{Any, TypeId};
use std::collections::BTreeMap;
use std::sync::Arc;

/// A type-indexed bag of shared services, supplied to `Engine::new`.
///
/// Services must behave as pure functions of their inputs from the op's point of view;
/// otherwise replay diverges (and the engine reports `Diverged`). A service whose behaviour can
/// change between releases (a kernel, a scheme registry) should be added with
/// [`with_versioned`](Self::with_versioned): its version is recorded in every node's
/// `Check::versions` as `service:<name>` (see `Check::of_with`), so a replay mismatch after the
/// service changed reports `Upgraded` rather than `Diverged`.
#[derive(Clone, Default)]
pub struct Services {
    map: BTreeMap<TypeId, (&'static str, Arc<dyn Any + Send + Sync>)>,
    /// Provenance name -> (version, the service's type).
    versions: BTreeMap<&'static str, (u32, TypeId)>,
}

impl Services {
    /// An empty bag.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add (or replace) the service of type `S`, unversioned. Replacing a versioned service of
    /// the same type drops its version entry.
    #[must_use]
    pub fn with<S: Any + Send + Sync>(mut self, s: S) -> Self {
        let t = TypeId::of::<S>();
        self.versions.retain(|_, (_, ty)| *ty != t);
        self.map
            .insert(t, (std::any::type_name::<S>(), Arc::new(s)));
        self
    }

    /// Add (or replace) the service of type `S` under provenance `name` with implementation
    /// `version`. Bump `version` whenever the service's results can change.
    ///
    /// Last wins: a later call with the same `name` replaces the earlier version entry (and a
    /// later call for the same type `S` under another name drops the old name), so each name
    /// and each service type appears at most once in [`versions`](Self::versions).
    #[must_use]
    pub fn with_versioned<S: Any + Send + Sync>(
        self,
        name: &'static str,
        version: u32,
        s: S,
    ) -> Self {
        let mut me = self.with(s);
        me.versions.insert(name, (version, TypeId::of::<S>()));
        me
    }

    /// The service of type `S`, if present.
    pub fn get<S: Any + Send + Sync>(&self) -> Option<&S> {
        self.map.get(&TypeId::of::<S>())?.1.downcast_ref::<S>()
    }

    /// Versioned services as `(name, version)`, in name order.
    pub fn versions(&self) -> impl Iterator<Item = (&'static str, u32)> + '_ {
        self.versions.iter().map(|(n, (v, _))| (*n, *v))
    }
}

impl std::fmt::Debug for Services {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_list()
            .entries(self.map.values().map(|(n, _)| n))
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Kernel(u8);
    struct Schemes;

    #[test]
    fn versions_are_name_ordered_and_last_wins() {
        let s = Services::new()
            .with_versioned("kernel", 1, Kernel(1))
            .with_versioned("schemes", 3, Schemes)
            .with_versioned("kernel", 2, Kernel(2));
        assert_eq!(
            s.versions().collect::<Vec<_>>(),
            vec![("kernel", 2), ("schemes", 3)]
        );
        assert_eq!(s.get::<Kernel>().map(|k| k.0), Some(2));
        // Same type under a new name: the old name goes.
        let s = s.with_versioned("geom", 5, Kernel(3));
        assert_eq!(
            s.versions().collect::<Vec<_>>(),
            vec![("geom", 5), ("schemes", 3)]
        );
        // Unversioned replacement drops the entry.
        let s = s.with(Schemes);
        assert_eq!(s.versions().collect::<Vec<_>>(), vec![("geom", 5)]);
    }
}
