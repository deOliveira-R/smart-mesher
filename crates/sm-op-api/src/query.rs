//! Read-only queries (`list`, `measure`, ...): registered by name like ops, in a separate
//! namespace, with input and output schemas. A query never mutates state and is never recorded
//! in the timeline.

use std::any::Any;

use schemars::{JsonSchema, Schema, schema_for};
use serde::Serialize;
use serde::de::DeserializeOwned;

use crate::cache::DerivedCache;
use crate::codec::{CodecError, Value, from_value, to_value};
use crate::error::OpError;
use crate::progress::Control;
use crate::services::Services;
use crate::state::State;

/// Read-only context a query runs in: injected services, the derived cache and progress /
/// cancellation. There is no mutation handle.
#[derive(Clone, Copy)]
pub struct QueryCtx<'a> {
    services: &'a Services,
    cache: &'a DerivedCache,
    control: Control<'a>,
}

impl<'a> QueryCtx<'a> {
    /// Build a context (the engine does this in `Engine::run_query`).
    pub fn new(services: &'a Services, cache: &'a DerivedCache, control: Control<'a>) -> Self {
        Self {
            services,
            cache,
            control,
        }
    }

    /// Look up an injected service.
    pub fn service<S: Any + Send + Sync>(&self) -> Result<&'a S, OpError> {
        self.services
            .get::<S>()
            .ok_or(OpError::MissingService(std::any::type_name::<S>()))
    }

    /// The session's derived cache.
    pub fn derived(&self) -> &'a DerivedCache {
        self.cache
    }

    /// Progress and cancellation.
    pub fn control(&self) -> Control<'a> {
        self.control
    }

    /// Check cancellation, then report `done / total`.
    pub fn step(&self, done: u64, total: u64, msg: &str) -> Result<(), OpError> {
        if self.control.cancel.is_cancelled() {
            return Err(OpError::Cancelled);
        }
        let f = if total == 0 {
            1.0
        } else {
            (done as f64 / total as f64).clamp(0.0, 1.0) as f32
        };
        self.control.progress.report(f, msg);
        Ok(())
    }
}

/// A typed query: its input is the payload, [`Output`](Self::Output) the reply.
pub trait TypedQuery: DeserializeOwned + JsonSchema + Send + Sync + 'static {
    /// Registry name (same rule as op names; separate namespace).
    const NAME: &'static str;
    /// Reply type.
    type Output: Serialize + JsonSchema;
    /// Run against a state. Must be deterministic.
    fn run(&self, state: &State, cx: &QueryCtx<'_>) -> Result<Self::Output, OpError>;
}

/// Type-erased registration of one query type.
pub trait QueryType: Send + Sync {
    /// Registry name.
    fn name(&self) -> &'static str;
    /// Input schema (`title` = query name).
    fn input_schema(&self) -> Result<Schema, String>;
    /// Output schema.
    fn output_schema(&self) -> Result<Schema, String>;
    /// Decode the input, run, encode the reply.
    fn run(&self, state: &State, input: &Value, cx: &QueryCtx<'_>) -> Result<Value, OpError>;
}

pub(crate) struct TypedQ<Q>(pub(crate) std::marker::PhantomData<fn() -> Q>);

impl<Q: TypedQuery> QueryType for TypedQ<Q> {
    fn name(&self) -> &'static str {
        Q::NAME
    }
    fn input_schema(&self) -> Result<Schema, String> {
        let mut s = schema_for!(Q);
        s.insert("title".to_owned(), Q::NAME.into());
        Ok(s)
    }
    fn output_schema(&self) -> Result<Schema, String> {
        Ok(schema_for!(Q::Output))
    }
    fn run(&self, state: &State, input: &Value, cx: &QueryCtx<'_>) -> Result<Value, OpError> {
        let q: Q = from_value(input)?;
        let out = q.run(state, cx)?;
        to_value(&out).map_err(|e: CodecError| OpError::Codec(e))
    }
}

/// Description of one registered query, for tool/help generation.
#[derive(Clone, Debug, Serialize)]
pub struct QueryInfo {
    /// Query name.
    pub name: String,
    /// Input schema; `title` is the query name.
    pub schema: Schema,
    /// Output schema.
    pub output_schema: Schema,
}

impl QueryInfo {
    /// Name usable as an MCP/LLM tool name: `.` becomes `__`.
    pub fn tool_name(&self) -> String {
        self.name.replace('.', "__")
    }
}
