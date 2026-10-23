//! The plugin registry: ops by name, layers by type, migrations by version.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use indexmap::IndexMap;
use schemars::{Schema, schema_for};

use crate::error::RegistryError;
use crate::layer::{DerivedCache, Layer, LayerFactory, LayerType};
use crate::op::{Op, OpRecord, TypedOp};
use crate::state::State;
use crate::value::{CodecError, Value, from_value};

/// Type-erased registration of one op payload type.
pub trait OpType: Send + Sync {
    /// Registry name.
    fn name(&self) -> &'static str;
    /// Current payload version.
    fn version(&self) -> u32;
    /// JSON schema of the params, for MCP tools, language help and Python stubs.
    fn schema(&self) -> Schema;
    /// Decode current-version params into an op.
    fn decode(&self, params: &Value) -> Result<Box<dyn Op>, CodecError>;
}

struct Typed<T>(std::marker::PhantomData<fn() -> T>);

impl<T: TypedOp> OpType for Typed<T> {
    fn name(&self) -> &'static str {
        T::NAME
    }
    fn version(&self) -> u32 {
        T::VERSION
    }
    fn schema(&self) -> Schema {
        schema_for!(T)
    }
    fn decode(&self, params: &Value) -> Result<Box<dyn Op>, CodecError> {
        Ok(Box::new(from_value::<T>(params)?))
    }
}

/// Rewrites params from version `n` to `n + 1`.
pub type Migration = fn(Value) -> Result<Value, CodecError>;

/// A plugin contributes ops, layers and migrations (one per concept node).
pub trait Plugin {
    /// Human-readable id, e.g. the concept node name.
    fn id(&self) -> &str;
    /// Register everything this plugin provides.
    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError>;
}

/// Description of one registered op, for tool/help/stub generation.
#[derive(Clone, Debug)]
pub struct OpInfo {
    /// Op name.
    pub name: String,
    /// Current version.
    pub version: u32,
    /// Params schema.
    pub schema: Schema,
}

/// Open registry of ops and layers. Build once, then share behind an `Arc`.
pub struct Registry {
    ops: IndexMap<String, Box<dyn OpType>>,
    layers: BTreeMap<&'static str, LayerFactory>,
    migrations: BTreeMap<(String, u32), Migration>,
}

impl Default for Registry {
    fn default() -> Self {
        Self::new()
    }
}

impl Registry {
    /// A registry with only the built-in derived-cache layer.
    pub fn new() -> Self {
        let mut r = Self { ops: IndexMap::new(), layers: BTreeMap::new(), migrations: BTreeMap::new() };
        r.layers.insert(DerivedCache::NAME, || Arc::new(DerivedCache::default()));
        r
    }

    /// Run a plugin's registration.
    pub fn install(&mut self, plugin: &dyn Plugin) -> Result<&mut Self, RegistryError> {
        plugin.register(self)?;
        Ok(self)
    }

    /// Register a typed op.
    pub fn register<T: TypedOp>(&mut self) -> Result<&mut Self, RegistryError> {
        self.register_dyn(Box::new(Typed::<T>(std::marker::PhantomData)))
    }

    /// Register a hand-written [`OpType`].
    pub fn register_dyn(&mut self, t: Box<dyn OpType>) -> Result<&mut Self, RegistryError> {
        let name = t.name().to_owned();
        if self.ops.contains_key(&name) {
            return Err(RegistryError::DuplicateOp(name));
        }
        self.ops.insert(name, t);
        Ok(self)
    }

    /// Register the step that upgrades `name` params from version `from` to `from + 1`.
    pub fn register_migration(&mut self, name: &str, from: u32, m: Migration) -> &mut Self {
        self.migrations.insert((name.to_owned(), from), m);
        self
    }

    /// Register a layer type; fresh states get a `Default` instance.
    pub fn register_layer<L: LayerType>(&mut self) -> Result<&mut Self, RegistryError> {
        if self.layers.contains_key(L::NAME) {
            return Err(RegistryError::DuplicateLayer(L::NAME));
        }
        self.layers.insert(L::NAME, || Arc::new(L::default()) as Arc<dyn Layer>);
        Ok(self)
    }

    /// An empty state with every registered layer instantiated.
    pub fn new_state(&self) -> State {
        State::with_layers(self.layers.iter().map(|(k, f)| (*k, f())).collect())
    }

    /// Names of registered ops, sorted.
    pub fn op_names(&self) -> BTreeSet<&str> {
        self.ops.keys().map(String::as_str).collect()
    }

    /// Schema of the named op's params.
    pub fn schema(&self, name: &str) -> Result<Schema, RegistryError> {
        self.ops
            .get(name)
            .map(|t| t.schema())
            .ok_or_else(|| RegistryError::UnknownOp(name.to_owned()))
    }

    /// Schema of the named op's params, rendered as compact JSON text.
    pub fn schema_json(&self, name: &str) -> Result<String, RegistryError> {
        Ok(self.schema(name)?.as_value().to_string())
    }

    /// All ops with schemas, sorted by name: the single source for MCP tool lists, language
    /// help and Python stub generation.
    pub fn catalog(&self) -> Vec<OpInfo> {
        let mut v: Vec<OpInfo> = self
            .ops
            .values()
            .map(|t| OpInfo { name: t.name().to_owned(), version: t.version(), schema: t.schema() })
            .collect();
        v.sort_by(|a, b| a.name.cmp(&b.name));
        v
    }

    /// Decode a stored record, running migrations from its version up to the current one.
    pub fn decode(&self, rec: &OpRecord) -> Result<Box<dyn Op>, RegistryError> {
        let t = self.ops.get(&rec.name).ok_or_else(|| RegistryError::UnknownOp(rec.name.clone()))?;
        let cur = t.version();
        if rec.version > cur {
            return Err(RegistryError::UnsupportedVersion {
                name: rec.name.clone(),
                found: rec.version,
                supported: cur,
            });
        }
        let mut params = rec.params.clone();
        for v in rec.version..cur {
            let m = self
                .migrations
                .get(&(rec.name.clone(), v))
                .ok_or_else(|| RegistryError::MissingMigration { name: rec.name.clone(), from: v })?;
            params = m(params).map_err(|source| RegistryError::Decode { name: rec.name.clone(), source })?;
        }
        t.decode(&params).map_err(|source| RegistryError::Decode { name: rec.name.clone(), source })
    }
}
