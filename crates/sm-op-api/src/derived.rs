//! Derived caches keyed by content hash.

use std::any::Any;
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, PoisonError};

use crate::hash::Hash256;

type Entry = Arc<dyn Any + Send + Sync>;

/// Rebuildable results (tracking structures, geometry caches, sweep graphs) keyed by
/// `(kind, content hash of the inputs)`.
///
/// It is shared between all states of a session, so a checkout back to an old node hits the cache.
/// It never takes part in the state hash: dropping it only costs time.
#[derive(Default)]
pub struct DerivedCache {
    entries: Mutex<BTreeMap<(String, Hash256), Entry>>,
}

impl DerivedCache {
    /// Return the cached `T` for `(kind, key)` or compute, store and return it.
    ///
    /// The lock is not held while `compute` runs. If an entry of another type is stored under the
    /// same key it is replaced.
    pub fn get_or_compute<T, E>(
        &self,
        kind: &str,
        key: Hash256,
        compute: impl FnOnce() -> Result<T, E>,
    ) -> Result<Arc<T>, E>
    where
        T: Any + Send + Sync,
    {
        let map_key = (kind.to_string(), key);
        let hit = self.lock().get(&map_key).cloned();
        if let Some(arc) = hit.and_then(|e| e.downcast::<T>().ok()) {
            return Ok(arc);
        }
        let value = Arc::new(compute()?);
        self.lock().insert(map_key, value.clone());
        Ok(value)
    }

    /// Number of cached entries.
    pub fn len(&self) -> usize {
        self.lock().len()
    }

    /// Whether the cache is empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Drop everything (always safe).
    pub fn clear(&self) {
        self.lock().clear();
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, BTreeMap<(String, Hash256), Entry>> {
        self.entries.lock().unwrap_or_else(PoisonError::into_inner)
    }
}
