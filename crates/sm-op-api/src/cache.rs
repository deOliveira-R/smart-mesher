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
    capacity_bytes: usize,
    disabled: bool,
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
///
/// Every entry counts at least 1 byte toward the capacity, so a zero-byte estimate is still
/// evictable. The capacity can change at run time ([`set_capacity`](Self::set_capacity)), and
/// [`disabled`](Self::disabled) builds a cache that never holds anything (determinism tests
/// compare a run on it against a warm run).
pub struct DerivedCache {
    inner: Mutex<Inner>,
}

impl Default for DerivedCache {
    fn default() -> Self {
        Self::with_capacity(256 << 20)
    }
}

impl DerivedCache {
    /// A cache that evicts least-recently-used entries beyond `capacity_bytes` (estimated).
    pub fn with_capacity(capacity_bytes: usize) -> Self {
        Self {
            inner: Mutex::new(Inner {
                capacity_bytes,
                ..Inner::default()
            }),
        }
    }

    /// A cache that stores nothing: [`get`](Self::get) always misses (counted in
    /// [`stats`](Self::stats)) and [`insert_sized`](Self::insert_sized) validates nothing,
    /// stores nothing and returns `Ok`, so `get_or_compute*` always computes. Determinism tests
    /// replay on `Engine::new(.., Arc::new(DerivedCache::disabled()))` and compare against a
    /// warm run.
    pub fn disabled() -> Self {
        Self {
            inner: Mutex::new(Inner {
                disabled: true,
                ..Inner::default()
            }),
        }
    }

    /// Whether this cache was built with [`disabled`](Self::disabled).
    pub fn is_disabled(&self) -> bool {
        self.lock().disabled
    }

    /// The current capacity bound in (estimated) bytes.
    pub fn capacity(&self) -> usize {
        self.lock().capacity_bytes
    }

    /// Change the capacity bound and evict least-recently-used entries until within it, with
    /// the same rule as [`insert_sized`](Self::insert_sized): the most recently used entry is
    /// always kept. No effect on a [`disabled`](Self::disabled) cache, which holds nothing.
    pub fn set_capacity(&self, capacity_bytes: usize) {
        let mut g = self.lock();
        g.capacity_bytes = capacity_bytes;
        g.evict();
    }

    fn lock(&self) -> MutexGuard<'_, Inner> {
        self.inner.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// Fetch a cached value of type `T` for `(kind, key)`. A value of another type or kind
    /// counts as a miss.
    pub fn get<T: Any + Send + Sync>(&self, kind: &str, key: Hash256) -> Option<Arc<T>> {
        let mut g = self.lock();
        if g.disabled {
            g.misses += 1;
            return None;
        }
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

    /// Store a value with an explicit size estimate (at least 1 byte is counted). Replacing an
    /// entry that holds a different type (or was stored under a different kind) is an error;
    /// replacing one of the same type keeps the new value. On a [`disabled`](Self::disabled)
    /// cache this does nothing and returns `Ok`.
    pub fn insert_sized<T: Any + Send + Sync>(
        &self,
        kind: &str,
        key: Hash256,
        value: Arc<T>,
        bytes: usize,
    ) -> Result<(), CacheError> {
        let mut g = self.lock();
        if g.disabled {
            return Ok(());
        }
        let bytes = bytes.max(1);
        g.tick += 1;
        let tick = g.tick;
        if let Some(old) = g.map.get(&key) {
            if old.ty != TypeId::of::<T>() || old.kind != kind {
                return Err(CacheError::TypeConflict {
                    kind: kind.to_owned(),
                });
            }
            let (ob, ot) = (old.bytes, old.last_used);
            g.bytes = g.bytes.saturating_sub(ob);
            g.lru.remove(&ot);
        }
        g.bytes += bytes;
        g.lru.insert(tick, key);
        g.map.insert(
            key,
            Entry {
                kind: kind.to_owned(),
                value,
                ty: TypeId::of::<T>(),
                bytes,
                last_used: tick,
            },
        );
        g.evict();
        Ok(())
    }

    /// Return the cached value or compute, store and return it. The size estimate is
    /// `size_of::<T>()`; use [`get_or_compute_sized`](Self::get_or_compute_sized) for heap-heavy values.
    pub fn get_or_compute<T, E>(
        &self,
        kind: &str,
        key: Hash256,
        f: impl FnOnce() -> Result<T, E>,
    ) -> Result<Arc<T>, E>
    where
        T: Any + Send + Sync,
        E: From<CacheError>,
    {
        self.get_or_compute_sized(kind, key, || f().map(|v| (v, std::mem::size_of::<T>())))
    }

    /// Like [`get_or_compute`](Self::get_or_compute) with `f` returning `(value, estimated_bytes)`.
    /// `f` runs without the cache lock held, so it may itself use the cache.
    pub fn get_or_compute_sized<T, E>(
        &self,
        kind: &str,
        key: Hash256,
        f: impl FnOnce() -> Result<(T, usize), E>,
    ) -> Result<Arc<T>, E>
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
        CacheStats {
            hits: g.hits,
            misses: g.misses,
            entries: g.map.len(),
            bytes: g.bytes,
        }
    }
}

impl Inner {
    /// Evict least-recently-used entries until within capacity, keeping the newest.
    fn evict(&mut self) {
        while self.bytes > self.capacity_bytes && self.map.len() > 1 {
            let Some((_, victim)) = self.lru.pop_first() else {
                break;
            };
            if let Some(e) = self.map.remove(&victim) {
                self.bytes = self.bytes.saturating_sub(e.bytes);
            }
        }
    }
}

impl std::fmt::Debug for DerivedCache {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DerivedCache")
            .field("stats", &self.stats())
            .finish()
    }
}
