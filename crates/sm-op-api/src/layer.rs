//! Layer stores: the independent, dynamically registered parts of a [`State`](crate::State).
//!
//! The api crate knows no concrete layer. A concept node implements [`LayerType`] for its
//! store, registers it with `Registry::register_layer`, and ops reach it by type.

use std::any::Any;
use std::collections::BTreeMap;

/// Successor map handed to [`LayerType::on_succession`]: old id to its successors.
pub type Successors = BTreeMap<EntityId, Vec<EntityId>>;

use serde::Serialize;
use sm_core::EntityId;

use crate::codec::{CodecError, canonical_bytes, to_value};
use crate::hash::{Hash256, Hasher};

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
}

/// Whether a layer's content is part of the state identity.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LayerRole {
    /// Part of the state hash; replay must reproduce it exactly.
    Authoritative,
    /// Rebuildable from authoritative layers; excluded from the state hash.
    Derived,
}

/// Incremental canonical digest builder handed to [`LayerType::digest`].
///
/// Every write is type- and length-framed so concatenations cannot collide.
///
/// A digest runs in one of two modes, invisible to implementors: *identity* (floats quantized,
/// used for the CI state hash) or *exact* (floats hashed bit-for-bit, used for derived-cache
/// keys so a cache hit can never change op output).
pub struct Digest(Hasher, bool);

impl Digest {
    pub(crate) fn new(domain: &str, exact: bool) -> Self {
        Self(Hasher::new(domain), exact)
    }
    pub(crate) fn finish(self) -> Hash256 {
        self.0.finish()
    }
    /// Mix in raw bytes (length-prefixed).
    pub fn bytes(&mut self, b: &[u8]) -> &mut Self {
        self.0.update(&[1]);
        self.0.update(&(b.len() as u64).to_le_bytes());
        self.0.update(b);
        self
    }
    /// Mix in a string.
    pub fn str(&mut self, s: &str) -> &mut Self {
        self.0.update(&[2]);
        self.0.update(&(s.len() as u64).to_le_bytes());
        self.0.update(s.as_bytes());
        self
    }
    /// Mix in an unsigned integer.
    pub fn u64(&mut self, v: u64) -> &mut Self {
        self.0.update(&[3]);
        self.0.update(&v.to_le_bytes());
        self
    }
    /// Mix in a signed integer.
    pub fn i64(&mut self, v: i64) -> &mut Self {
        self.0.update(&[4]);
        self.0.update(&v.to_le_bytes());
        self
    }
    /// Mix in an entity id.
    pub fn id(&mut self, id: EntityId) -> &mut Self {
        self.u64(id.to_raw())
    }
    /// Mix in a float quantised to multiples of `quantum` (docs/ARCHITECTURE.md §3: the CI
    /// state hash covers *quantized* coordinates so it is platform independent). `NaN` and
    /// infinities hash to distinct markers; a non-positive or non-finite `quantum` hashes as an
    /// error marker (deterministic, and a validator problem). Magnitudes too large for the
    /// quantized `i64` (`|v / quantum| >= 2^63`) hash their raw bits instead of saturating, so
    /// they do not all collide. Rounding is inherent to quantization: a value within one ulp of
    /// a half-quantum boundary (`(k + 0.5) * quantum`) can round to different integers on
    /// different platforms or after harmless float reassociation, which shows up as a spurious
    /// `Diverged`. Choose quanta well away from the model's tolerances and from values that
    /// land on such boundaries. In exact mode (cache keys) the raw bits are always hashed.
    pub fn f64_quantized(&mut self, v: f64, quantum: f64) -> &mut Self {
        if !(quantum.is_finite() && quantum > 0.0) {
            self.0.update(&[5, 0]);
        } else if v.is_nan() {
            self.0.update(&[5, 1]);
        } else if v == f64::INFINITY {
            self.0.update(&[5, 2]);
        } else if v == f64::NEG_INFINITY {
            self.0.update(&[5, 3]);
        } else {
            let q = (v / quantum).round();
            if self.1 || q.abs() >= 9.2e18 {
                self.0.update(&[5, 5]);
                self.0.update(&v.to_bits().to_le_bytes());
            } else {
                self.0.update(&[5, 4]);
                self.0.update(&(q as i64).to_le_bytes());
            }
        }
        self
    }
    /// Mix in a float whose identity quantization the caller already computed (`quantized`),
    /// for layers whose quantum varies per field (`sm-model-api`'s Q2 rule, `sm-topo-api`'s
    /// `HashWord::Float`). Identity mode writes `quantized`; exact mode writes `v.to_bits()`.
    /// `NaN` and infinities use the same markers as [`f64_quantized`](Self::f64_quantized) in
    /// both modes. Rounding to `quantized` is the caller's job and must be deterministic.
    pub fn f64_with(&mut self, v: f64, quantized: i64) -> &mut Self {
        if v.is_nan() {
            self.0.update(&[5, 1]);
        } else if v == f64::INFINITY {
            self.0.update(&[5, 2]);
        } else if v == f64::NEG_INFINITY {
            self.0.update(&[5, 3]);
        } else if self.1 {
            self.0.update(&[5, 5]);
            self.0.update(&v.to_bits().to_le_bytes());
        } else {
            self.0.update(&[5, 4]);
            self.0.update(&quantized.to_le_bytes());
        }
        self
    }
    /// Mix in another hash.
    pub fn hash(&mut self, h: &Hash256) -> &mut Self {
        self.0.update(&[6]);
        self.0.update(&h.0);
        self
    }
    /// Mix in any serializable value via its canonical encoding (key order and text form do
    /// not matter). Convenient for small layers; large layers should hash their arrays
    /// directly. A value that cannot be encoded (NaN, ...) hashes its encoding error text, so
    /// distinct failures differ, but values failing the same way still collide: keep such values
    /// out of layers. Floats inside are *not* quantized.
    pub fn value<T: Serialize + ?Sized>(&mut self, v: &T) -> &mut Self {
        match to_value(v) {
            Ok(val) => {
                self.0.update(&[7]);
                let b = canonical_bytes(&val);
                self.0.update(&(b.len() as u64).to_le_bytes());
                self.0.update(&b);
            }
            Err(e) => {
                let m = e.to_string();
                self.0.update(&[7, 0xFF]);
                self.0.update(&(m.len() as u64).to_le_bytes());
                self.0.update(m.as_bytes());
            }
        }
        self
    }
}

/// Implemented by a concept node for each store it contributes to the state.
pub trait LayerType: Clone + Send + Sync + 'static {
    /// Unique layer name (see [`names`]).
    const NAME: &'static str;
    /// Authoritative (default) or derived.
    const ROLE: LayerRole = LayerRole::Authoritative;
    /// Names of layers this layer's data refers to (its upstream concept nodes), mirroring the
    /// edges of the concept DAG in `docs/graph/concepts.yaml`. [`on_retired`](Self::on_retired)
    /// reactions run upstream-first in the topological order of these edges (ties by name).
    const DEPENDS_ON: &'static [&'static str] = &[];
    /// Version of this layer's [`digest`](Self::digest) encoding. Bump it whenever the digest
    /// of unchanged content changes (a field added to the hash, a new quantum, ...). Recorded
    /// in every node's `Check::versions` as `layer:<NAME>`, so replay over an older timeline
    /// reports `TimelineError::Upgraded` instead of a bare `Diverged`.
    const DIGEST_VERSION: u32 = 1;
    /// Feed the canonical content into the digest. Must be deterministic and independent of
    /// allocation addresses, thread count and hash-map order. Write floats via
    /// [`Digest::f64_quantized`], never pre-round: a layer that rounds itself and writes
    /// `i64`/`u64` collapses sub-quantum differences in cache keys too.
    fn digest(&self, d: &mut Digest);
    /// True when the layer holds nothing. Empty layers do not enter the state hash, so
    /// installing a new concept leaves the hashes of existing histories unchanged.
    fn is_empty(&self) -> bool;
    /// Called after an op recorded successors (`ApplyCtx::record_successors`): old id to new
    /// ids (split children, merge survivor; a copy is `A -> [A, A']`). Runs in the same
    /// dependency order as [`on_retired`](Self::on_retired) and before it, on every non-empty
    /// authoritative layer whose [`references_any`](Self::references_any) holds for the keys,
    /// so a downstream layer can carry groups or names over before the old ids are retired.
    /// Must be deterministic. Default: nothing. (v1 gives no upstream state access.)
    fn on_succession(&mut self, _succ: &Successors) {}
    /// Called after an op retired `ids`, on every non-empty authoritative layer, so references
    /// to deleted entities do not dangle. Must be deterministic. Default: nothing.
    fn on_retired(&mut self, _ids: &[EntityId]) {}
    /// Whether [`on_retired`](Self::on_retired) would change this layer for `ids`. Checked
    /// through `&self` before the layer is copied-on-write, so a retire only clones, re-digests
    /// and reports (in `Delta::touched_layers`) layers that actually refer to the ids. Must
    /// not return `false` when `on_retired` would change anything. Default: `true`.
    fn references_any(&self, _ids: &[EntityId]) -> bool {
        true
    }
    /// Persisted-snapshot hook for very large layers (mesh arrays). `Ok(None)` (the default)
    /// means "replay to rebuild"; `Some(bytes)` lets a storage backend persist the layer and
    /// restore it via [`restore`](Self::restore) instead of replaying.
    fn snapshot(&self) -> Result<Option<Vec<u8>>, CodecError> {
        Ok(None)
    }
    /// Whether [`snapshot`](Self::snapshot) is implemented: return `true` iff it returns
    /// `Some` for non-empty states. Lets callers check ([`State::can_snapshot`]) before
    /// attempting a snapshot. Default: `false`.
    ///
    /// [`State::can_snapshot`]: crate::state::State::can_snapshot
    fn supports_snapshot(&self) -> bool {
        false
    }
    /// Rebuild a layer from [`snapshot`](Self::snapshot) bytes. Default: unsupported.
    fn restore(_bytes: &[u8]) -> Result<Self, CodecError>
    where
        Self: Sized,
    {
        Err(CodecError::new("layer does not support snapshot restore"))
    }
}

/// Object-safe view of a [`LayerType`]; implemented automatically.
pub trait Layer: Any + Send + Sync {
    /// Layer name.
    fn name(&self) -> &'static str;
    /// Layer role.
    fn role(&self) -> LayerRole;
    /// See [`LayerType::DIGEST_VERSION`].
    fn digest_version(&self) -> u32;
    /// Content digest of this layer (not memoized; `State` memoizes).
    fn content_hash(&self) -> Hash256;
    /// Exact content digest: like [`content_hash`](Self::content_hash) but floats written via
    /// [`Digest::f64_quantized`] are hashed bit-for-bit. Used for derived-cache keys.
    fn exact_hash(&self) -> Hash256;
    /// See [`LayerType::is_empty`].
    fn is_empty(&self) -> bool;
    /// See [`LayerType::on_succession`].
    fn on_succession(&mut self, succ: &Successors);
    /// See [`LayerType::on_retired`].
    fn on_retired(&mut self, ids: &[EntityId]);
    /// See [`LayerType::references_any`].
    fn references_any(&self, ids: &[EntityId]) -> bool;
    /// See [`LayerType::snapshot`].
    fn snapshot(&self) -> Result<Option<Vec<u8>>, CodecError>;
    /// See [`LayerType::supports_snapshot`]. Default: `false`.
    fn supports_snapshot(&self) -> bool {
        false
    }
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
    fn digest_version(&self) -> u32 {
        T::DIGEST_VERSION
    }
    fn content_hash(&self) -> Hash256 {
        let mut d = Digest::new("smart-mesh/layer/v1", false);
        d.str(T::NAME);
        self.digest(&mut d);
        d.finish()
    }
    fn exact_hash(&self) -> Hash256 {
        let mut d = Digest::new("smart-mesh/layer-exact/v1", true);
        d.str(T::NAME);
        self.digest(&mut d);
        d.finish()
    }
    fn is_empty(&self) -> bool {
        LayerType::is_empty(self)
    }
    fn on_succession(&mut self, succ: &Successors) {
        LayerType::on_succession(self, succ);
    }
    fn on_retired(&mut self, ids: &[EntityId]) {
        LayerType::on_retired(self, ids);
    }
    fn references_any(&self, ids: &[EntityId]) -> bool {
        LayerType::references_any(self, ids)
    }
    fn snapshot(&self) -> Result<Option<Vec<u8>>, CodecError> {
        LayerType::snapshot(self)
    }
    fn supports_snapshot(&self) -> bool {
        LayerType::supports_snapshot(self)
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

/// Reference store for concept layers: an ordered map from entity id to payload. Wrap it in a
/// newtype that implements [`LayerType`] and forward `digest_with` (quantizing floats),
/// `is_empty` and `on_retired`.
#[derive(Clone, Debug)]
pub struct MapLayer<T> {
    items: BTreeMap<EntityId, T>,
}

impl<T> Default for MapLayer<T> {
    fn default() -> Self {
        Self {
            items: BTreeMap::new(),
        }
    }
}

impl<T> MapLayer<T> {
    /// An empty map.
    pub fn new() -> Self {
        Self::default()
    }
    /// Look up a payload.
    pub fn get(&self, id: EntityId) -> Option<&T> {
        self.items.get(&id)
    }
    /// Mutable lookup.
    pub fn get_mut(&mut self, id: EntityId) -> Option<&mut T> {
        self.items.get_mut(&id)
    }
    /// Insert or replace; returns the previous payload.
    pub fn insert(&mut self, id: EntityId, v: T) -> Option<T> {
        self.items.insert(id, v)
    }
    /// Remove and return a payload.
    pub fn remove(&mut self, id: EntityId) -> Option<T> {
        self.items.remove(&id)
    }
    /// Entries in id order.
    pub fn iter(&self) -> impl Iterator<Item = (EntityId, &T)> {
        self.items.iter().map(|(k, v)| (*k, v))
    }
    /// Number of entries.
    pub fn len(&self) -> usize {
        self.items.len()
    }
    /// True when empty.
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }
    /// Drop entries for retired ids.
    pub fn on_retired(&mut self, ids: &[EntityId]) {
        for id in ids {
            self.items.remove(id);
        }
    }
}

impl<T> MapLayer<T> {
    /// Feed every entry into the digest in id order, length-prefixed: the id, then whatever
    /// `f` writes for the payload. This is the documented way to digest a `MapLayer`: write
    /// float fields with [`Digest::f64_quantized`] inside `f` so the state hash is quantized
    /// and platform independent.
    pub fn digest_with(&self, d: &mut Digest, f: impl Fn(&mut Digest, &T)) {
        d.u64(self.items.len() as u64);
        for (id, v) in &self.items {
            d.id(*id);
            f(d, v);
        }
    }
}

impl<T: Serialize> MapLayer<T> {
    /// Digest through the payload's canonical serialization. **Only for payloads without
    /// floats**: [`Digest::value`] does not quantize, so float bits would leak into the state
    /// hash. Use [`digest_with`](Self::digest_with) for anything holding an `f64`.
    pub fn digest_float_free(&self, d: &mut Digest) {
        self.digest_with(d, |d, v| {
            d.value(v);
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn both(f: impl Fn(&mut Digest)) -> (Hash256, Hash256) {
        let mut a = Digest::new("t", false);
        let mut b = Digest::new("t", true);
        f(&mut a);
        f(&mut b);
        (a.finish(), b.finish())
    }

    #[test]
    fn f64_with_hashes_the_given_quantum_in_identity_mode_and_bits_in_exact_mode() {
        // Same quantized value, sub-quantum difference: identity equal, exact different.
        let (i1, e1) = both(|d| {
            d.f64_with(1.0, 1024);
        });
        let (i2, e2) = both(|d| {
            d.f64_with(1.0 + 1e-9, 1024);
        });
        assert_eq!(i1, i2);
        assert_ne!(e1, e2);
        // It matches f64_quantized when the caller rounds the same way.
        let (iq, eq) = both(|d| {
            d.f64_quantized(1.0, 1.0 / 1024.0);
        });
        assert_eq!(iq, i1);
        assert_eq!(eq, e1);
        // Non-finite values use the f64_quantized markers in both modes.
        for v in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let (a, b) = both(|d| {
                d.f64_with(v, 0);
            });
            let (c, e) = both(|d| {
                d.f64_quantized(v, 1.0);
            });
            assert_eq!((a, b), (c, e));
        }
    }
}
