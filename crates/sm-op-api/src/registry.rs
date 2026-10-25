//! The plugin registry: ops by name, layers by type, migrations by version.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use schemars::{Schema, schema_for};

use crate::batch::Batch;
use crate::codec::{CodecError, Value, from_value};
use crate::error::RegistryError;
use crate::layer::{Layer, LayerRole, LayerType};
use crate::op::{Delta, Op, OpRecord, TypedOp};
use sm_core::EntityKind;
use crate::query::{QueryCtx, QueryInfo, QueryType, TypedQ, TypedQuery};
use crate::state::{State, StateSnapshot};

mod batch_schema {
    //! Assembly of the typed `core.batch` schema.

    use schemars::Schema;
    use serde_json::{Map, Value, json};

    use super::{OpInfo, OpType};
    use crate::batch::{Batch, ESCAPED_LITERAL_PATTERN, REF_PATTERN};
    use crate::error::RegistryError;
    use crate::op::TypedOp;

    /// sm-core's `EntityId` pattern.
    const ENTITY_PATTERN: &str = "^[A-Za-z]+#[0-9]+$";
    /// Name of the shared id-or-reference def.
    pub(super) const BATCH_ENTITY_ID: &str = "BatchEntityId";

    fn rewrite(v: &mut Value, prefix: &str) {
        match v {
            Value::Object(m) => {
                let inline_id = m.get("type").and_then(Value::as_str) == Some("string")
                    && m.get("pattern").and_then(Value::as_str) == Some(ENTITY_PATTERN);
                if inline_id {
                    // Keep `x-*` annotations (`x-entity-kind`, `x-positional`) on the property.
                    let mut r = Map::new();
                    r.insert("$ref".to_owned(), json!(format!("#/$defs/{BATCH_ENTITY_ID}")));
                    for (k, x) in m.iter().filter(|(k, _)| k.starts_with("x-")) {
                        r.insert(k.clone(), x.clone());
                    }
                    *v = Value::Object(r);
                    return;
                }
                if let Some(Value::String(r)) = m.get_mut("$ref")
                    && let Some(k) = r.strip_prefix("#/$defs/")
                {
                    *r = if k == "EntityId" {
                        format!("#/$defs/{BATCH_ENTITY_ID}")
                    } else {
                        format!("#/$defs/{prefix}{k}")
                    };
                }
                m.values_mut().for_each(|x| rewrite(x, prefix));
            }
            Value::Array(a) => a.iter_mut().for_each(|x| rewrite(x, prefix)),
            _ => {}
        }
    }

    pub(super) fn build<'a>(
        ops: impl Iterator<Item = &'a dyn OpType>,
        info: fn(&dyn OpType) -> Result<OpInfo, RegistryError>,
    ) -> Result<Schema, RegistryError> {
        let mut defs = Map::new();
        defs.insert(
            BATCH_ENTITY_ID.to_owned(),
            json!({
                "description": "Entity id `Kind#serial`, a batch reference `$i.role` / `$i.role[k]` / `$i.role[*]` (`[*]` only as an array element), or a `$$`-escaped literal",
                "anyOf": [
                    { "type": "string", "pattern": ENTITY_PATTERN },
                    { "type": "string", "pattern": REF_PATTERN },
                    { "type": "string", "pattern": ESCAPED_LITERAL_PATTERN },
                ],
            }),
        );
        let mut variants = Vec::new();
        for t in ops {
            if t.name() == Batch::NAME {
                continue;
            }
            let name = t.name();
            // A broken op schema is reported by its own catalog entry; the batch omits it.
            let Ok(i) = info(t) else { continue };
            let mut v = i.schema.to_value();
            let prefix = format!("{name}.");
            let local = v.as_object_mut().and_then(|m| {
                m.remove("$schema");
                m.remove("$defs")
            });
            if let Some(Value::Object(local)) = local {
                for (k, mut d) in local {
                    if k == "EntityId" {
                        continue;
                    }
                    rewrite(&mut d, &prefix);
                    defs.insert(format!("{prefix}{k}"), d);
                }
            }
            rewrite(&mut v, &prefix);
            defs.insert(format!("{name}.params"), v);
            variants.push(json!({
                "type": "object",
                "properties": {
                    "name": { "const": name },
                    "version": { "const": t.version() },
                    "params": { "$ref": format!("#/$defs/{name}.params") },
                },
                "required": ["name", "version", "params"],
                "additionalProperties": false,
            }));
        }
        let s = json!({
            "$schema": "https://json-schema.org/draft/2020-12/schema",
            "title": Batch::NAME,
            "description": "Several ops applied atomically as one node; later ops may reference ids allocated by earlier ones (`$i.role`).",
            "type": "object",
            "properties": {
                "ops": { "type": "array", "items": { "oneOf": variants } },
            },
            "required": ["ops"],
            "$defs": defs,
        });
        Schema::try_from(s).map_err(|e| RegistryError::Schema {
            name: Batch::NAME.to_owned(),
            reason: e.to_string(),
        })
    }
}

/// How many ids an op output role holds.
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize, schemars::JsonSchema,
)]
#[serde(rename_all = "snake_case")]
#[non_exhaustive]
pub enum Cardinality {
    /// Exactly one id.
    One,
    /// Zero or one id.
    Optional,
    /// Any number of ids, including none.
    Many,
}

impl Cardinality {
    /// Whether `n` ids satisfy this cardinality.
    pub fn allows(self, n: usize) -> bool {
        match self {
            Self::One => n == 1,
            Self::Optional => n <= 1,
            Self::Many => true,
        }
    }
}

/// Declaration of one named output role of an op (`ApplyCtx::alloc_as` / `adopt_as`).
/// Read by the engine (validation), sm-lang (static role checks), MCP and stub generators.
#[derive(Clone, Copy, Debug, serde::Serialize)]
#[non_exhaustive]
pub struct OutputRole {
    /// Role name, as passed to `alloc_as` / `adopt_as` and used in `$i.role` references.
    pub name: &'static str,
    /// Allowed entity kinds; empty means any kind.
    pub kinds: &'static [EntityKind],
    /// How many ids the role holds.
    pub cardinality: Cardinality,
    /// One-line description.
    pub doc: &'static str,
}

impl OutputRole {
    /// Declare a role.
    pub const fn new(
        name: &'static str,
        kinds: &'static [EntityKind],
        cardinality: Cardinality,
        doc: &'static str,
    ) -> Self {
        Self {
            name,
            kinds,
            cardinality,
            doc,
        }
    }
}

/// Owned, deserializable mirror of [`OutputRole`] with the same JSON shape.
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
pub struct OutputRoleOwned {
    /// Role name.
    pub name: String,
    /// Allowed entity kinds; empty means any kind.
    pub kinds: Vec<EntityKind>,
    /// How many ids the role holds.
    pub cardinality: Cardinality,
    /// One-line description.
    pub doc: String,
}

impl From<&OutputRole> for OutputRoleOwned {
    fn from(r: &OutputRole) -> Self {
        Self {
            name: r.name.to_owned(),
            kinds: r.kinds.to_vec(),
            cardinality: r.cardinality,
            doc: r.doc.to_owned(),
        }
    }
}

/// Whether `s` is a valid output role name: `[a-z_][a-z0-9_]*`.
pub fn valid_role_name(s: &str) -> bool {
    let mut c = s.chars();
    c.next().is_some_and(|f| f.is_ascii_lowercase() || f == '_')
        && c.all(|x| x.is_ascii_lowercase() || x.is_ascii_digit() || x == '_')
}

/// Schema annotation on an id property (or, on an array, its items): the list of canonical
/// [`EntityKind`] names (as serialized, e.g. `"TopoVolume"`) the id may have. Write it with
/// `#[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoFace"]))]` on the field.
pub const X_ENTITY_KIND: &str = "x-entity-kind";

/// Schema annotation `true` on the params property that a front end may pass positionally
/// (injected by the registry from `TypedOp::POSITIONAL`).
pub const X_POSITIONAL: &str = "x-positional";

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
    /// Declared output roles (see `TypedOp::OUTPUTS`). Default: none declared.
    fn outputs(&self) -> &'static [OutputRole] {
        &[]
    }
    /// Whether the op declares it writes no ids (see `TypedOp::NO_OUTPUTS`). Default: false.
    fn no_outputs(&self) -> bool {
        false
    }
    /// Name of the positional params property (see `TypedOp::POSITIONAL`). Default: none.
    fn positional(&self) -> Option<&'static str> {
        None
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
    fn outputs(&self) -> &'static [OutputRole] {
        T::OUTPUTS
    }
    fn no_outputs(&self) -> bool {
        T::NO_OUTPUTS
    }
    fn positional(&self) -> Option<&'static str> {
        T::POSITIONAL
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

fn is_false(b: &bool) -> bool {
    !*b
}

/// Description of one registered op, for tool/help/stub generation.
/// Built only by the [`Registry`]; it may gain fields.
#[derive(Clone, Debug, serde::Serialize)]
#[non_exhaustive]
pub struct OpInfo {
    /// Op name, e.g. `model.create_brick`.
    pub name: String,
    /// MCP-safe tool name (`name` with `.` mangled to `__`); clients never recompute it.
    pub tool_name: String,
    /// Current version.
    pub version: u32,
    /// Params schema; `title` is the op name.
    pub schema: Schema,
    /// Schema of the result (`Delta`: created, removed, named outputs, touched layers).
    pub output_schema: Schema,
    /// Declared output roles; empty when the op declares none (then roles are unchecked).
    pub outputs: Vec<OutputRole>,
    /// The op explicitly declares that it writes no ids (`TypedOp::NO_OUTPUTS`). Omitted from
    /// JSON when false.
    #[serde(skip_serializing_if = "is_false")]
    pub no_outputs: bool,
    /// Params property that may be given positionally (also marked `x-positional` in `schema`).
    pub positional: Option<String>,
}

impl OpInfo {
    /// Name usable as an MCP/LLM tool name (`^[a-zA-Z0-9_-]+$`): `.` becomes `__`. Injective,
    /// because registered op names never contain `__`.
    pub fn tool_name(&self) -> String {
        self.name.replace('.', "__")
    }
}

/// Owned, deserializable mirror of [`OpInfo`]: serializes to exactly the same JSON, so a
/// client can read a catalog reply into it.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
pub struct OpInfoOwned {
    /// Op name.
    pub name: String,
    /// MCP-safe tool name.
    pub tool_name: String,
    /// Current version.
    pub version: u32,
    /// Params schema.
    pub schema: Schema,
    /// Result schema.
    pub output_schema: Schema,
    /// Declared output roles.
    pub outputs: Vec<OutputRoleOwned>,
    /// The op explicitly declares that it writes no ids (absent in older catalogs = false).
    #[serde(default, skip_serializing_if = "is_false")]
    pub no_outputs: bool,
    /// Positional params property, if any.
    pub positional: Option<String>,
}

impl From<&OpInfo> for OpInfoOwned {
    fn from(i: &OpInfo) -> Self {
        Self {
            name: i.name.clone(),
            tool_name: i.tool_name.clone(),
            version: i.version,
            schema: i.schema.clone(),
            output_schema: i.output_schema.clone(),
            outputs: i.outputs.iter().map(OutputRoleOwned::from).collect(),
            no_outputs: i.no_outputs,
            positional: i.positional.clone(),
        }
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

    /// Register a hand-written [`OpType`]. Fails with [`RegistryError::DuplicateOp`] if the name
    /// is already an op or a query (one namespace, see [`register_query`](Self::register_query)).
    pub fn register_dyn(&mut self, t: Arc<dyn OpType>) -> Result<&mut Self, RegistryError> {
        let name = t.name().to_owned();
        if !valid_op_name(&name) {
            return Err(RegistryError::BadOpName(name));
        }
        // Ops and queries share one namespace: their MCP tool names (`tool_name`) do.
        if self.ops.contains_key(&name) || self.queries.contains_key(&name) {
            return Err(RegistryError::DuplicateOp(name));
        }
        // A registered op must always appear in the catalog and the batch schema, so reject
        // here whatever `info` would later reject.
        Self::info(t.as_ref())?;
        let mut seen = BTreeSet::new();
        for r in t.outputs() {
            if !valid_role_name(r.name) {
                return Err(RegistryError::BadOutputRole {
                    op: name,
                    role: r.name.to_owned(),
                });
            }
            if !seen.insert(r.name) {
                return Err(RegistryError::DuplicateOutputRole {
                    op: name,
                    role: r.name.to_owned(),
                });
            }
        }
        self.ops.insert(name, t);
        Ok(self)
    }

    /// Register a typed query (same name rule as ops). Op and query names share one namespace,
    /// because their MCP tool names (`OpInfo::tool_name`, `QueryInfo::tool_name`) do: registering a query
    /// under an op's name is [`RegistryError::DuplicateQuery`], and the reverse
    /// [`RegistryError::DuplicateOp`].
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
        if self.queries.contains_key(&name) || self.ops.contains_key(&name) {
            return Err(RegistryError::DuplicateQuery(name));
        }
        // Keep the query visible in `query_catalog`.
        let bad = |reason| RegistryError::Schema {
            name: name.clone(),
            reason,
        };
        for s in [
            t.input_schema().map_err(bad)?,
            t.output_schema().map_err(bad)?,
        ] {
            if s.as_object().is_none() {
                return Err(bad("schema is not an object".to_owned()));
            }
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
                    tool_name: name.replace('.', "__"),
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

    /// Declared output roles of a registered op (empty if unknown or undeclared).
    pub fn op_outputs(&self, name: &str) -> &'static [OutputRole] {
        self.ops.get(name).map_or(&[], |t| t.outputs())
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
        let mut schema = t.schema().map_err(|reason| RegistryError::Schema {
            name: name.clone(),
            reason,
        })?;
        if schema.as_object().is_none() {
            return Err(RegistryError::Schema {
                name,
                reason: "schema is not an object".to_owned(),
            });
        }
        if let Some(p) = t.positional() {
            let prop = schema
                .get_mut("properties")
                .and_then(|v| v.as_object_mut())
                .and_then(|m| m.get_mut(p))
                .and_then(|v| v.as_object_mut());
            let Some(prop) = prop else {
                return Err(RegistryError::Schema {
                    name,
                    reason: format!("positional property `{p}` is not in the params schema"),
                });
            };
            prop.insert(X_POSITIONAL.to_owned(), true.into());
        }
        if t.no_outputs() && !t.outputs().is_empty() {
            return Err(RegistryError::Schema {
                name,
                reason: "NO_OUTPUTS is set but OUTPUTS is not empty".to_owned(),
            });
        }
        Ok(OpInfo {
            tool_name: name.replace('.', "__"),
            name,
            version: t.version(),
            schema,
            output_schema: schema_for!(Delta),
            outputs: t.outputs().to_vec(),
            no_outputs: t.no_outputs(),
            positional: t.positional().map(str::to_owned),
        })
    }

    /// Schema of the named op's params. For `core.batch` this is [`batch_schema`](Self::batch_schema).
    pub fn schema(&self, name: &str) -> Result<Schema, RegistryError> {
        let t = self
            .ops
            .get(name)
            .ok_or_else(|| RegistryError::UnknownOp(name.to_owned()))?;
        Ok(self.info_full(t.as_ref())?.schema)
    }

    /// All ops with schemas, sorted by name: the single source for MCP tool lists, language
    /// help and Python stub generation. One op with a broken schema yields an `Err` entry
    /// without hiding the others (`core.batch` embeds the schemas of the ops that have one).
    pub fn catalog(&self) -> Vec<Result<OpInfo, RegistryError>> {
        self.ops.values().map(|t| self.info_full(t.as_ref())).collect()
    }

    fn info_full(&self, t: &dyn OpType) -> Result<OpInfo, RegistryError> {
        let mut info = Self::info(t)?;
        if info.name == Batch::NAME {
            info.schema = self.batch_schema()?;
        }
        Ok(info)
    }

    /// The typed schema of `core.batch` params over the ops registered now.
    ///
    /// `ops.items` is a `oneOf` with one variant per registered op except `core.batch` itself
    /// (nesting is rejected): `{name: const, version: const (current), params: {$ref:
    /// "#/$defs/<op>.params"}}`. Each op's params schema is stored once under `$defs` as
    /// `<op>.params`, and its own `$defs` entries as `<op>.<def>`. Inside these batch-local
    /// defs every `EntityId` (by `$ref` or inline) is replaced by a reference to the shared
    /// def `BatchEntityId`: `anyOf` the `EntityId` pattern,
    /// [`REF_PATTERN`](crate::batch::REF_PATTERN) and the `$$`-escaped literal pattern. Schemas
    /// are not hashed, so this never changes the node format.
    ///
    /// An op whose own schema is broken is left out (its [`catalog`](Self::catalog) entry is
    /// the `Err`), so one broken op does not hide the batch.
    pub fn batch_schema(&self) -> Result<Schema, RegistryError> {
        batch_schema::build(self.ops.values().map(|t| t.as_ref()), Self::info)
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
