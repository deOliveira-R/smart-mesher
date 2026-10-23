//! The plugin registry: ops by name, layers by type, migrations by version.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use schemars::{Schema, schema_for};

use crate::batch::Batch;
use crate::codec::{CodecError, Value, from_value};
use crate::error::RegistryError;
use crate::layer::{Layer, LayerRole, LayerType};
use crate::op::{Delta, Op, OpRecord, TypedOp};
use crate::query::{QueryCtx, QueryInfo, QueryType, TypedQ, TypedQuery};
use crate::state::{State, StateSnapshot};

/// Type-erased registration of one op payload type.
pub trait OpType: Send + Sync {
    /// Registry name.
    fn name(&self) -> &'static str;
    /// Current payload version.
    fn version(&self) -> u32;
    /// JSON schema of the params, for MCP tools, language help and Python stubs.
    fn schema(&self) -> Result<Schema, String>;
    /// Decode current-version params into an op.
    fn decode(&self, params: &Value) -> Result<Box<dyn Op>, CodecError>;
    /// Implementation version (see `TypedOp::IMPL_VERSION`). Default 1.
    fn impl_version(&self) -> u32 {
        1
    }
}

struct Typed<T>(std::marker::PhantomData<fn() -> T>);

impl<T: TypedOp> OpType for Typed<T> {
    fn name(&self) -> &'static str {
        T::NAME
    }
    fn version(&self) -> u32 {
        T::VERSION
    }
    fn schema(&self) -> Result<Schema, String> {
        let mut s = schema_for!(T);
        s.insert("title".to_owned(), T::NAME.into());
        Ok(s)
    }
    fn decode(&self, params: &Value) -> Result<Box<dyn Op>, CodecError> {
        Ok(Box::new(from_value::<T>(params)?))
    }
    fn impl_version(&self) -> u32 {
        T::IMPL_VERSION
    }
}

/// Rewrites params from version `n` to `n + 1`.
pub type Migration = Arc<dyn Fn(Value) -> Result<Value, CodecError> + Send + Sync>;

/// A plugin contributes ops, queries, layers and migrations (one per concept node).
pub trait Plugin {
    /// Human-readable id, e.g. the concept node name.
    fn id(&self) -> &str;
    /// Register everything this plugin provides.
    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError>;
}

/// Description of one registered op, for tool/help/stub generation.
#[derive(Clone, Debug, serde::Serialize)]
pub struct OpInfo {
    /// Op name, e.g. `model.create_brick`.
    pub name: String,
    /// Current version.
    pub version: u32,
    /// Params schema; `title` is the op name.
    pub schema: Schema,
    /// Schema of the result (`Delta`: created, removed, named outputs, touched layers).
    pub output_schema: Schema,
}

impl OpInfo {
    /// Name usable as an MCP/LLM tool name (`^[a-zA-Z0-9_-]+$`): `.` becomes `__`. Injective,
    /// because registered op names never contain `__`.
    pub fn tool_name(&self) -> String {
        self.name.replace('.', "__")
    }
}

/// Longest allowed tool name (`name` with `.` mangled to `__`), in bytes.
pub const MAX_TOOL_NAME: usize = 64;

/// Op-name rule: `^[a-z][a-z0-9_]*(\.[a-z][a-z0-9_]*)+$` with no `__` (so `tool_name` is
/// injective), and at most [`MAX_TOOL_NAME`] bytes once `.` is mangled to `__` (MCP / LLM tool
/// name limit). Names are stored in every node, so the rule is part of the format. Query names
/// follow the same rule.
pub fn valid_op_name(name: &str) -> bool {
    if name.replace('.', "__").len() > MAX_TOOL_NAME {
        return false;
    }
    let mut segs = 0;
    for seg in name.split('.') {
        let mut c = seg.chars();
        if !c.next().is_some_and(|f| f.is_ascii_lowercase())
            || !c.all(|x| x.is_ascii_lowercase() || x.is_ascii_digit() || x == '_')
        {
            return false;
        }
        segs += 1;
    }
    segs >= 2 && !name.contains("__")
}

type RestoreFn = fn(&[u8]) -> Result<Arc<dyn Layer>, CodecError>;

#[derive(Clone)]
struct LayerEntry {
    name: &'static str,
    role: LayerRole,
    digest_version: u32,
    deps: &'static [&'static str],
    factory: Arc<dyn Fn() -> Arc<dyn Layer> + Send + Sync>,
    restore: RestoreFn,
}

/// Open registry of ops and layers. Build once, then share behind an `Arc`.
///
/// Everything is held in ordered maps and behind `Arc`s: the registry is cheap to clone, and its
/// observable behaviour (catalog, layer order, new states) does not depend on registration order.
#[derive(Clone)]
pub struct Registry {
    ops: BTreeMap<String, Arc<dyn OpType>>,
    queries: BTreeMap<String, Arc<dyn QueryType>>,
    layers: BTreeMap<&'static str, LayerEntry>,
    migrations: BTreeMap<(String, u32), Migration>,
}

impl Default for Registry {
    fn default() -> Self {
        Self::new()
    }
}

impl Registry {
    /// A registry with only the built-in `core.batch` op.
    pub fn new() -> Self {
        let mut r = Self {
            ops: BTreeMap::new(),
            queries: BTreeMap::new(),
            layers: BTreeMap::new(),
            migrations: BTreeMap::new(),
        };
        r.ops.insert(
            Batch::NAME.to_owned(),
            Arc::new(Typed::<Batch>(std::marker::PhantomData)),
        );
        r
    }

    /// Run a plugin's registration atomically: if it fails, the registry is unchanged.
    pub fn install(&mut self, plugin: &dyn Plugin) -> Result<&mut Self, RegistryError> {
        let mut staged = self.clone();
        plugin.register(&mut staged)?;
        staged.layer_order_impl(false)?;
        *self = staged;
        Ok(self)
    }

    /// Register a typed op.
    pub fn register<T: TypedOp>(&mut self) -> Result<&mut Self, RegistryError> {
        self.register_dyn(Arc::new(Typed::<T>(std::marker::PhantomData)))
    }

    /// Register a hand-written [`OpType`].
    pub fn register_dyn(&mut self, t: Arc<dyn OpType>) -> Result<&mut Self, RegistryError> {
        let name = t.name().to_owned();
        if !valid_op_name(&name) {
            return Err(RegistryError::BadOpName(name));
        }
        if self.ops.contains_key(&name) {
            return Err(RegistryError::DuplicateOp(name));
        }
        self.ops.insert(name, t);
        Ok(self)
    }

    /// Register a typed query (same name rule as ops; queries have their own namespace).
    pub fn register_query<Q: TypedQuery>(&mut self) -> Result<&mut Self, RegistryError> {
        self.register_query_dyn(Arc::new(TypedQ::<Q>(std::marker::PhantomData)))
    }

    /// Register a hand-written [`QueryType`].
    pub fn register_query_dyn(
        &mut self,
        t: Arc<dyn QueryType>,
    ) -> Result<&mut Self, RegistryError> {
        let name = t.name().to_owned();
        if !valid_op_name(&name) {
            return Err(RegistryError::BadOpName(name));
        }
        if self.queries.contains_key(&name) {
            return Err(RegistryError::DuplicateQuery(name));
        }
        self.queries.insert(name, t);
        Ok(self)
    }

    /// Names of registered queries, sorted.
    pub fn query_names(&self) -> Vec<&str> {
        self.queries.keys().map(String::as_str).collect()
    }

    /// Run a query against `state`: decode `input`, run, encode the reply. Read-only; nothing
    /// is recorded. `cx` supplies services, the derived cache and progress (see
    /// `Engine::run_query` for the usual entry point).
    pub fn run_query(
        &self,
        state: &State,
        name: &str,
        input: &Value,
        cx: &QueryCtx<'_>,
    ) -> Result<Value, RegistryError> {
        let q = self
            .queries
            .get(name)
            .ok_or_else(|| RegistryError::UnknownQuery(name.to_owned()))?;
        q.run(state, input, cx)
            .map_err(|e| RegistryError::Query {
                name: name.to_owned(),
                source: Box::new(e),
            })
    }

    /// All queries with input and output schemas, sorted by name. One query with a broken
    /// schema yields an `Err` entry without hiding the others.
    pub fn query_catalog(&self) -> Vec<Result<QueryInfo, RegistryError>> {
        self.queries
            .values()
            .map(|t| {
                let name = t.name().to_owned();
                let err = |reason| RegistryError::Schema {
                    name: name.clone(),
                    reason,
                };
                let schema = t.input_schema().map_err(err)?;
                let output_schema = t.output_schema().map_err(err)?;
                if schema.as_object().is_none() {
                    return Err(err("schema is not an object".to_owned()));
                }
                Ok(QueryInfo {
                    name: name.clone(),
                    schema,
                    output_schema,
                })
            })
            .collect()
    }

    /// Register the step that upgrades `name` params from version `from` to `from + 1`.
    /// The op must already be registered and `from` must be below its current version.
    pub fn register_migration(
        &mut self,
        name: &str,
        from: u32,
        m: impl Fn(Value) -> Result<Value, CodecError> + Send + Sync + 'static,
    ) -> Result<&mut Self, RegistryError> {
        let current = self
            .ops
            .get(name)
            .ok_or_else(|| RegistryError::UnknownOp(name.to_owned()))?
            .version();
        if from >= current {
            return Err(RegistryError::BadMigration {
                name: name.to_owned(),
                from,
                current,
            });
        }
        let key = (name.to_owned(), from);
        if self.migrations.contains_key(&key) {
            return Err(RegistryError::DuplicateMigration {
                name: name.to_owned(),
                from,
            });
        }
        self.migrations.insert(key, Arc::new(m));
        Ok(self)
    }

    /// Register a layer type; fresh states get a `Default` instance.
    pub fn register_layer<L: LayerType + Default>(&mut self) -> Result<&mut Self, RegistryError> {
        self.register_layer_with(L::default)
    }

    /// Register a layer type built by `factory` (for layers that need configuration).
    pub fn register_layer_with<L: LayerType>(
        &mut self,
        factory: impl Fn() -> L + Send + Sync + 'static,
    ) -> Result<&mut Self, RegistryError> {
        if self.layers.contains_key(L::NAME) {
            return Err(RegistryError::DuplicateLayer(L::NAME));
        }
        self.layers.insert(
            L::NAME,
            LayerEntry {
                name: L::NAME,
                role: L::ROLE,
                digest_version: L::DIGEST_VERSION,
                deps: L::DEPENDS_ON,
                factory: Arc::new(move || Arc::new(factory()) as Arc<dyn Layer>),
                restore: |b| Ok(Arc::new(L::restore(b)?) as Arc<dyn Layer>),
            },
        );
        Ok(self)
    }

    /// An empty state with every registered layer instantiated.
    pub fn new_state(&self) -> State {
        State::with_layers(
            self.layers
                .iter()
                .map(|(k, e)| (*k, (e.factory)()))
                .collect(),
        )
    }

    /// Rebuild a state from a persisted snapshot (see [`State::snapshot`]).
    pub fn restore(&self, snap: &StateSnapshot) -> Result<State, RegistryError> {
        let mut st = self.new_state();
        st.restore_entities(&snap.kinds)
            .map_err(|source| RegistryError::Snapshot {
                layer: "<entities>".to_owned(),
                source,
            })?;
        for (name, bytes) in &snap.layers {
            let e = self
                .layers
                .get(name.as_str())
                .ok_or_else(|| RegistryError::MissingLayer(name.clone()))?;
            let layer = (e.restore)(bytes).map_err(|source| RegistryError::Snapshot {
                layer: name.clone(),
                source,
            })?;
            st.replace_layer(e.name, layer);
        }
        Ok(st)
    }

    /// Registered layer names with their roles, sorted by name.
    pub fn layer_roles(&self) -> Vec<(&'static str, LayerRole)> {
        self.layers.values().map(|e| (e.name, e.role)).collect()
    }

    /// Digest version of a registered layer (`LayerType::DIGEST_VERSION`).
    pub fn layer_digest_version(&self, name: &str) -> Option<u32> {
        self.layers.get(name).map(|e| e.digest_version)
    }

    /// Implementation version of a registered op (`OpType::impl_version`).
    pub fn op_impl_version(&self, name: &str) -> Option<u32> {
        self.ops.get(name).map(|t| t.impl_version())
    }

    /// Layers in dependency order: every layer after the layers it `DEPENDS_ON`; ties by name.
    /// This is the order `on_retired` reactions run in. Fails on a cycle or an unregistered dependency.
    pub fn layer_order(&self) -> Result<Vec<&'static str>, RegistryError> {
        self.layer_order_impl(true)
    }

    fn layer_order_impl(&self, strict: bool) -> Result<Vec<&'static str>, RegistryError> {
        let mut pending: BTreeMap<&'static str, BTreeSet<&'static str>> = BTreeMap::new();
        for e in self.layers.values() {
            let mut deps = BTreeSet::new();
            for d in e.deps {
                if self.layers.contains_key(d) {
                    deps.insert(*d);
                } else if strict {
                    return Err(RegistryError::UnknownLayerDependency {
                        layer: e.name,
                        dep: d,
                    });
                }
            }
            pending.insert(e.name, deps);
        }
        let mut order = Vec::with_capacity(pending.len());
        while !pending.is_empty() {
            let next = pending.iter().find(|(_, d)| d.is_empty()).map(|(n, _)| *n);
            let Some(n) = next else {
                return Err(RegistryError::LayerCycle(pending.keys().copied().collect()));
            };
            pending.remove(n);
            pending.values_mut().for_each(|d| {
                d.remove(n);
            });
            order.push(n);
        }
        Ok(order)
    }

    /// Names of registered ops, sorted.
    pub fn op_names(&self) -> Vec<&str> {
        self.ops.keys().map(String::as_str).collect()
    }

    /// Current version of the named op.
    pub fn op_version(&self, name: &str) -> Result<u32, RegistryError> {
        self.ops
            .get(name)
            .map(|t| t.version())
            .ok_or_else(|| RegistryError::UnknownOp(name.to_owned()))
    }

    fn info(t: &dyn OpType) -> Result<OpInfo, RegistryError> {
        let name = t.name().to_owned();
        let schema = t.schema().map_err(|reason| RegistryError::Schema {
            name: name.clone(),
            reason,
        })?;
        if schema.as_object().is_none() {
            return Err(RegistryError::Schema {
                name,
                reason: "schema is not an object".to_owned(),
            });
        }
        Ok(OpInfo {
            name,
            version: t.version(),
            schema,
            output_schema: schema_for!(Delta),
        })
    }

    /// Schema of the named op's params.
    pub fn schema(&self, name: &str) -> Result<Schema, RegistryError> {
        let t = self
            .ops
            .get(name)
            .ok_or_else(|| RegistryError::UnknownOp(name.to_owned()))?;
        Ok(Self::info(t.as_ref())?.schema)
    }

    /// All ops with schemas, sorted by name: the single source for MCP tool lists, language
    /// help and Python stub generation. One op with a broken schema yields an `Err` entry
    /// without hiding the others.
    pub fn catalog(&self) -> Vec<Result<OpInfo, RegistryError>> {
        self.ops.values().map(|t| Self::info(t.as_ref())).collect()
    }

    /// Decode a stored record, running migrations from its version up to the current one.
    pub fn decode(&self, rec: &OpRecord) -> Result<Box<dyn Op>, RegistryError> {
        let t = self
            .ops
            .get(&rec.name)
            .ok_or_else(|| RegistryError::UnknownOp(rec.name.clone()))?;
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
            let m = self.migrations.get(&(rec.name.clone(), v)).ok_or_else(|| {
                RegistryError::MissingMigration {
                    name: rec.name.clone(),
                    from: v,
                }
            })?;
            params = m(params).map_err(|source| RegistryError::Decode {
                name: rec.name.clone(),
                source,
            })?;
        }
        t.decode(&params).map_err(|source| RegistryError::Decode {
            name: rec.name.clone(),
            source,
        })
    }
}
