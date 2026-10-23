//! The derived cache: results (tessellations, overlap tables, tracking structures, ...) keyed
//! by the content hash of their inputs.
//!
//! It is owned by the session and shared by every state and every op, so it survives
//! checkout, failure and cancellation. That is safe because an entry is a pure function of
//! its key: a hit must never change an op's output. It never enters the state hash.

use std::any::{Any, TypeId};
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, MutexGuard};

use crate::error::CacheError;
use crate::hash::Hash256;

struct Entry {
    kind: String,
    value: Arc<dyn Any + Send + Sync>,
    ty: TypeId,
    bytes: usize,
    last_used: u64,
}

/// Keyed by the key hash alone (it already mixes in the kind, see `State::cache_key`), so
/// lookups allocate nothing; `lru` orders keys by last use (ticks are unique) for O(log n)
/// eviction.
#[derive(Default)]
struct Inner {
    map: BTreeMap<Hash256, Entry>,
    lru: BTreeMap<u64, Hash256>,
    bytes: usize,
    tick: u64,
    hits: u64,
    misses: u64,
}

/// Counters for tests and diagnostics.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CacheStats {
    /// Lookups that found an entry.
    pub hits: u64,
    /// Lookups that did not.
    pub misses: u64,
    /// Entries currently held.
    pub entries: usize,
    /// Estimated bytes currently held.
    pub bytes: usize,
}

/// Bounded LRU cache, `Sync`, with interior mutability. Eviction order depends only on the
/// access sequence; since entries are pure functions of their key, eviction never changes
/// results.
pub struct DerivedCache {
    inner: Mutex<Inner>,
    capacity_bytes: usize,
}

impl Default for DerivedCache {
    fn default() -> Self {
        Self::with_capacity(256 << 20)
    }
}

impl DerivedCache {
    /// A cache that evicts least-recently-used entries beyond `capacity_bytes` (estimated).
    pub fn with_capacity(capacity_bytes: usize) -> Self {
        Self { inner: Mutex::default(), capacity_bytes }
    }

    fn lock(&self) -> MutexGuard<'_, Inner> {
        self.inner.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// Fetch a cached value of type `T` for `(kind, key)`. A value of another type or kind
    /// counts as a miss.
    pub fn get<T: Any + Send + Sync>(&self, kind: &str, key: Hash256) -> Option<Arc<T>> {
        let mut g = self.lock();
        g.tick += 1;
        let tick = g.tick;
        let inner = &mut *g;
        let found = inner.map.get_mut(&key).and_then(|e| {
            if e.kind != kind {
                return None;
            }
            let v = Arc::clone(&e.value).downcast::<T>().ok()?;
            inner.lru.remove(&e.last_used);
            e.last_used = tick;
            inner.lru.insert(tick, key);
            Some(v)
        });
        if found.is_some() {
            g.hits += 1;
        } else {
            g.misses += 1;
        }
        found
    }

    /// Store a value with an explicit size estimate. Replacing an entry that holds a different
    /// type (or was stored under a different kind) is an error; replacing one of the same type
    /// keeps the new value.
    pub fn insert_sized<T: Any + Send + Sync>(&self, kind: &str, key: Hash256, value: Arc<T>, bytes: usize) -> Result<(), CacheError> {
        let mut g = self.lock();
        g.tick += 1;
        let tick = g.tick;
        if let Some(old) = g.map.get(&key) {
            if old.ty != TypeId::of::<T>() || old.kind != kind {
                return Err(CacheError::TypeConflict { kind: kind.to_owned() });
            }
            let (ob, ot) = (old.bytes, old.last_used);
            g.bytes -= ob;
            g.lru.remove(&ot);
        }
        g.bytes += bytes;
        g.lru.insert(tick, key);
        g.map.insert(key, Entry { kind: kind.to_owned(), value, ty: TypeId::of::<T>(), bytes, last_used: tick });
        // Evict least-recently-used until within capacity, keeping the newest.
        while g.bytes > self.capacity_bytes && g.map.len() > 1 {
            let Some((_, victim)) = g.lru.pop_first() else { break };
            if let Some(e) = g.map.remove(&victim) {
                g.bytes -= e.bytes;
            }
        }
        Ok(())
    }

    /// Return the cached value or compute, store and return it. The size estimate is
    /// `size_of::<T>()`; use [`get_or_compute_sized`](Self::get_or_compute_sized) for heap-heavy values.
    pub fn get_or_compute<T, E>(&self, kind: &str, key: Hash256, f: impl FnOnce() -> Result<T, E>) -> Result<Arc<T>, E>
    where
        T: Any + Send + Sync,
        E: From<CacheError>,
    {
        self.get_or_compute_sized(kind, key, || f().map(|v| (v, std::mem::size_of::<T>())))
    }

    /// Like [`get_or_compute`](Self::get_or_compute) with `f` returning `(value, estimated_bytes)`.
    /// `f` runs without the cache lock held, so it may itself use the cache.
    pub fn get_or_compute_sized<T, E>(&self, kind: &str, key: Hash256, f: impl FnOnce() -> Result<(T, usize), E>) -> Result<Arc<T>, E>
    where
        T: Any + Send + Sync,
        E: From<CacheError>,
    {
        if let Some(v) = self.get::<T>(kind, key) {
            return Ok(v);
        }
        let (v, bytes) = f()?;
        let v = Arc::new(v);
        self.insert_sized(kind, key, Arc::clone(&v), bytes)?;
        Ok(v)
    }

    /// Drop every entry.
    pub fn clear(&self) {
        let mut g = self.lock();
        g.map.clear();
        g.lru.clear();
        g.bytes = 0;
    }

    /// Current counters.
    pub fn stats(&self) -> CacheStats {
        let g = self.lock();
        CacheStats { hits: g.hits, misses: g.misses, entries: g.map.len(), bytes: g.bytes }
    }
}

impl std::fmt::Debug for DerivedCache {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DerivedCache").field("stats", &self.stats()).finish()
    }
}
