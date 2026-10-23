//! The composable state: independent layer stores plus the id allocator and derived cache.

use std::any::Any;
use std::sync::Arc;

use crate::ctx::{ApplyCtx, Control, Delta, Upstream};
use crate::derived::DerivedCache;
use crate::error::OpError;
use crate::hash::{Hash256, content_hash};
use crate::id::IdAllocator;
use crate::layer::{Layer, OpOf, Stack, StackOp};

/// Everything an op can read and change: one store per layer, the id allocator, and a shared
/// derived cache.
///
/// Stores sit behind `Arc`, so cloning a `State` (for snapshots and for the transactional apply)
/// copies pointers, and an op touching one layer shares the other three with the old state.
pub struct State<S: Stack> {
    model: Arc<S::Model>,
    partition: Arc<S::Partition>,
    intent: Arc<S::Intent>,
    meshes: Arc<S::Meshes>,
    ids: IdAllocator,
    derived: Arc<DerivedCache>,
}

impl<S: Stack> Clone for State<S> {
    fn clone(&self) -> Self {
        Self {
            model: self.model.clone(),
            partition: self.partition.clone(),
            intent: self.intent.clone(),
            meshes: self.meshes.clone(),
            ids: self.ids.clone(),
            derived: self.derived.clone(),
        }
    }
}

impl<S: Stack> Default for State<S> {
    fn default() -> Self {
        Self {
            model: Arc::default(),
            partition: Arc::default(),
            intent: Arc::default(),
            meshes: Arc::default(),
            ids: IdAllocator::default(),
            derived: Arc::default(),
        }
    }
}

impl<S: Stack> State<S> {
    /// An empty state that shares an existing derived cache.
    pub fn with_derived(derived: Arc<DerivedCache>) -> Self {
        Self {
            derived,
            ..Self::default()
        }
    }

    /// The model layer store.
    pub fn model(&self) -> &S::Model {
        &self.model
    }
    /// The partition layer store.
    pub fn partition(&self) -> &S::Partition {
        &self.partition
    }
    /// The intent layer store.
    pub fn intent(&self) -> &S::Intent {
        &self.intent
    }
    /// The meshes layer store.
    pub fn meshes(&self) -> &S::Meshes {
        &self.meshes
    }
    /// The id allocator.
    pub fn ids(&self) -> &IdAllocator {
        &self.ids
    }
    /// The derived cache (shared by all states of a session; excluded from the state hash).
    pub fn derived(&self) -> &Arc<DerivedCache> {
        &self.derived
    }

    /// Apply `op` and return the new state and what changed. `self` is never modified, so a
    /// failing (or cancelled) op trivially changes nothing.
    pub fn apply(&self, op: &OpOf<S>, control: &dyn Control) -> Result<(Self, Delta), OpError> {
        let mut next = self.clone();
        let mut delta = Delta::default();
        let mut ids = self.ids.clone();
        match op {
            StackOp::Model(o) => {
                delta.layer = <S::Model as Layer>::NAME;
                let mut ctx = ApplyCtx::new(&mut ids, &mut delta, control, Upstream::new(&[]));
                next.model = Arc::new(self.model.apply(o, &mut ctx)?);
            }
            StackOp::Partition(o) => {
                delta.layer = <S::Partition as Layer>::NAME;
                let up: [&dyn Any; 1] = [&*self.model];
                let mut ctx = ApplyCtx::new(&mut ids, &mut delta, control, Upstream::new(&up));
                next.partition = Arc::new(self.partition.apply(o, &mut ctx)?);
            }
            StackOp::Intent(o) => {
                delta.layer = <S::Intent as Layer>::NAME;
                let up: [&dyn Any; 2] = [&*self.model, &*self.partition];
                let mut ctx = ApplyCtx::new(&mut ids, &mut delta, control, Upstream::new(&up));
                next.intent = Arc::new(self.intent.apply(o, &mut ctx)?);
            }
            StackOp::Meshes(o) => {
                delta.layer = <S::Meshes as Layer>::NAME;
                let up: [&dyn Any; 3] = [&*self.model, &*self.partition, &*self.intent];
                let mut ctx = ApplyCtx::new(&mut ids, &mut delta, control, Upstream::new(&up));
                next.meshes = Arc::new(self.meshes.apply(o, &mut ctx)?);
            }
        }
        next.ids = ids;
        Ok((next, delta))
    }

    /// Content hash of one layer store, by layer name. `None` for an unknown name.
    pub fn layer_hash(&self, name: &str) -> Option<Result<Hash256, OpError>> {
        let h = |r: Result<Hash256, _>| Some(r.map_err(OpError::from));
        if name == <S::Model as Layer>::NAME {
            h(content_hash(name, &*self.model))
        } else if name == <S::Partition as Layer>::NAME {
            h(content_hash(name, &*self.partition))
        } else if name == <S::Intent as Layer>::NAME {
            h(content_hash(name, &*self.intent))
        } else if name == <S::Meshes as Layer>::NAME {
            h(content_hash(name, &*self.meshes))
        } else {
            None
        }
    }

    /// Content hash of the whole state: the four layer hashes plus the allocator. The derived
    /// cache is excluded. Equal hashes mean equal states.
    pub fn state_hash(&self) -> Result<Hash256, OpError> {
        let layer = |n: &str| self.layer_hash(n).unwrap_or(Ok(Hash256::default()));
        let parts = (
            layer(<S::Model as Layer>::NAME)?,
            layer(<S::Partition as Layer>::NAME)?,
            layer(<S::Intent as Layer>::NAME)?,
            layer(<S::Meshes as Layer>::NAME)?,
            &self.ids,
        );
        Ok(content_hash("sm-op-api/state/v1", &parts)?)
    }

    /// Get or compute a derived value keyed by the content hashes of the named layers (`deps`).
    /// The result is valid exactly as long as those layers are unchanged.
    pub fn derived_or_compute<T, E>(
        &self,
        kind: &str,
        deps: &[&str],
        compute: impl FnOnce(&Self) -> Result<T, E>,
    ) -> Result<Arc<T>, E>
    where
        T: Any + Send + Sync,
        E: From<OpError>,
    {
        let mut hashes = Vec::with_capacity(deps.len());
        for d in deps {
            let h = self
                .layer_hash(d)
                .ok_or_else(|| OpError::Invalid(format!("unknown layer `{d}`")))??;
            hashes.push(h);
        }
        let key = content_hash("sm-op-api/derived-key/v1", &hashes).map_err(OpError::from)?;
        self.derived.get_or_compute(kind, key, || compute(self))
    }
}
