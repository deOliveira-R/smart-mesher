//! Layer stores: the independent, dynamically registered parts of a [`State`](crate::State).
//!
//! The api crate knows no concrete layer. A concept node implements [`LayerType`] for its
//! store, registers it with `Registry::register_layer`, and ops reach it by type.

use std::any::Any;
use std::collections::BTreeMap;
use std::sync::Arc;

use serde::Serialize;

use crate::hash::{Hash256, Sha256};
use crate::toml::to_toml;
use crate::value::to_value;

/// Conventional layer names (docs/design/ir.md §5). Concept nodes should use these for the
/// corresponding store; any other name is equally valid.
pub mod names {
    /// Model layer: topology, shape, coloring, boundary kinds, instancing.
    pub const MODEL: &str = "model";
    /// Partition layer: virtual decomposition.
    pub const PARTITION: &str = "partition";
    /// Intent layer: sizes, intervals, schemes.
    pub const INTENT: &str = "intent";
    /// Mesh layer: meshes and hierarchy.
    pub const MESHES: &str = "meshes";
    /// Derived layer: caches keyed by content hash (provided by this crate).
    pub const DERIVED: &str = "derived";
}

/// Whether a layer's content is part of the state identity.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LayerRole {
    /// Part of the state hash; replay must reproduce it exactly.
    Authoritative,
    /// A cache rebuilt on demand; excluded from the state hash.
    Derived,
}

/// Incremental canonical digest builder handed to [`LayerType::digest`].
///
/// Every write is length- or type-framed so concatenations cannot collide.
pub struct Digest(Sha256);

impl Digest {
    pub(crate) fn new() -> Self {
        Self(Sha256::new())
    }
    pub(crate) fn finish(self) -> Hash256 {
        self.0.finish()
    }
    /// Mix in raw bytes (length-prefixed).
    pub fn bytes(&mut self, b: &[u8]) -> &mut Self {
        self.0.update(&(b.len() as u64).to_le_bytes());
        self.0.update(b);
        self
    }
    /// Mix in a string.
    pub fn str(&mut self, s: &str) -> &mut Self {
        self.bytes(s.as_bytes())
    }
    /// Mix in an unsigned integer.
    pub fn u64(&mut self, v: u64) -> &mut Self {
        self.0.update(&v.to_le_bytes());
        self
    }
    /// Mix in a signed integer.
    pub fn i64(&mut self, v: i64) -> &mut Self {
        self.u64(v as u64)
    }
    /// Mix in a float quantised to multiples of `quantum` (docs/ARCHITECTURE.md §3: the CI
    /// state hash covers *quantized* coordinates so it is platform independent).
    pub fn f64_quantized(&mut self, v: f64, quantum: f64) -> &mut Self {
        let q = (v / quantum).round();
        // Saturating cast; NaN maps to 0, which is fine because NaN is a validator error.
        self.i64(q as i64)
    }
    /// Mix in another hash.
    pub fn hash(&mut self, h: &Hash256) -> &mut Self {
        self.0.update(&h.0);
        self
    }
    /// Mix in any serializable value via its canonical TOML form. Convenient default for
    /// small layers; large layers should hash their arrays directly.
    pub fn value<T: Serialize + ?Sized>(&mut self, v: &T) -> &mut Self {
        match to_value(v) {
            Ok(val) => {
                let t = match val {
                    crate::Value::Table(t) => t,
                    other => BTreeMap::from([("v".to_owned(), other)]),
                };
                self.str(&to_toml(&t))
            }
            Err(e) => self.str(&format!("<unserializable: {e}>")),
        }
    }
}

/// Implemented by a concept node for each store it contributes to the state.
pub trait LayerType: Clone + Default + Send + Sync + 'static {
    /// Unique layer name (see [`names`]).
    const NAME: &'static str;
    /// Authoritative (default) or derived.
    const ROLE: LayerRole = LayerRole::Authoritative;
    /// Feed the canonical content into the digest. Must be deterministic and independent of
    /// allocation addresses, thread count and hash-map order.
    fn digest(&self, d: &mut Digest);
}

/// Object-safe view of a [`LayerType`]; implemented automatically.
pub trait Layer: Any + Send + Sync {
    /// Layer name.
    fn name(&self) -> &'static str;
    /// Layer role.
    fn role(&self) -> LayerRole;
    /// Content digest of this layer.
    fn content_hash(&self) -> Hash256;
    /// Deep copy (cheap for persistent/Arc-based stores).
    fn clone_box(&self) -> Box<dyn Layer>;
    /// Upcast for downcasting by type.
    fn as_any(&self) -> &dyn Any;
    /// Mutable upcast.
    fn as_any_mut(&mut self) -> &mut dyn Any;
}

impl<T: LayerType> Layer for T {
    fn name(&self) -> &'static str {
        T::NAME
    }
    fn role(&self) -> LayerRole {
        T::ROLE
    }
    fn content_hash(&self) -> Hash256 {
        let mut d = Digest::new();
        d.str(T::NAME);
        self.digest(&mut d);
        d.finish()
    }
    fn clone_box(&self) -> Box<dyn Layer> {
        Box::new(self.clone())
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

/// Factory producing an empty layer for a fresh state.
pub(crate) type LayerFactory = fn() -> Arc<dyn Layer>;

/// The derived layer: results (tessellations, overlap tables, ...) keyed by the content hash
/// of their inputs. Excluded from the state hash. Installed by `Registry::new`.
#[derive(Clone, Default)]
pub struct DerivedCache {
    entries: BTreeMap<(String, Hash256), Arc<dyn Any + Send + Sync>>,
}

impl DerivedCache {
    /// Fetch a cached value of type `T` for `(kind, key)`.
    pub fn get<T: Any + Send + Sync>(&self, kind: &str, key: Hash256) -> Option<Arc<T>> {
        let e = self.entries.get(&(kind.to_owned(), key))?;
        Arc::clone(e).downcast::<T>().ok()
    }
    /// Store a value. Replaces any previous entry.
    pub fn insert<T: Any + Send + Sync>(&mut self, kind: &str, key: Hash256, value: Arc<T>) {
        self.entries.insert((kind.to_owned(), key), value);
    }
    /// Drop all entries (derived data is always rebuildable).
    pub fn clear(&mut self) {
        self.entries.clear();
    }
    /// Number of cached entries.
    pub fn len(&self) -> usize {
        self.entries.len()
    }
    /// True when empty.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

impl LayerType for DerivedCache {
    const NAME: &'static str = names::DERIVED;
    const ROLE: LayerRole = LayerRole::Derived;
    fn digest(&self, _: &mut Digest) {}
}
