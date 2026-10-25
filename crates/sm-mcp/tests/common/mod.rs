//! A mock `Endpoint`: a real op `Registry` for the catalog, canned results, a request log.
#![allow(dead_code, missing_docs)]

use std::io::Cursor;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sm_core::{EntityId, EntityKind};
use sm_op_api::{
    ApplyCtx, Cardinality, Delta, Hash256, OpError, OutputRole, QueryCtx, Registry, State,
    TypedOp, TypedQuery,
};
use sm_protocol::{
    ApplyResult, CatalogResult, Endpoint, ExportFormat, ExportResult, HashResult, PROTOCOL_VERSION,
    Request, RpcError, ScriptResult, StatusResult,
};

pub fn h(n: u8) -> Hash256 {
    Hash256([n; 32])
}

/// Makes a brick (the mock never applies it).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct CreateBrick {
    pub x: f64,
    pub y: Option<f64>,
    pub z: Option<f64>,
}
impl TypedOp for CreateBrick {
    const NAME: &'static str = "model.create_brick";
    const OUTPUTS: &'static [OutputRole] = &[
        OutputRole::new("volume", &[EntityKind::TopoVolume], Cardinality::One, "the brick volume"),
        OutputRole::new("faces", &[EntityKind::TopoFace], Cardinality::Many, "the six faces"),
    ];
    const POSITIONAL: Option<&'static str> = Some("x");
    fn apply(&self, _: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        Ok(())
    }
}

/// Meshes volumes.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct MeshRun {
    #[schemars(extend("x-entity-kind" = ["TopoVolume"]))]
    pub targets: Vec<EntityId>,
}
impl TypedOp for MeshRun {
    const NAME: &'static str = "mesh.run";
    fn apply(&self, _: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        Ok(())
    }
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct Count {}
impl TypedQuery for Count {
    const NAME: &'static str = "demo.count";
    type Output = u32;
    fn run(&self, _: &State, _: &QueryCtx<'_>) -> Result<u32, OpError> {
        Ok(0)
    }
}

/// Related entities of `within`.
#[derive(Debug, Deserialize, JsonSchema)]
pub struct Related {
    #[schemars(extend("x-entity-kind" = ["TopoVolume"]))]
    pub within: EntityId,
}
impl TypedQuery for Related {
    const NAME: &'static str = "topo.related";
    type Output = Vec<EntityId>;
    fn run(&self, _: &State, _: &QueryCtx<'_>) -> Result<Vec<EntityId>, OpError> {
        Ok(vec![])
    }
}


macro_rules! unit_query {
    ($t:ident, $name:literal) => {
        #[derive(Debug, Deserialize, JsonSchema)]
        pub struct $t {}
        impl TypedQuery for $t {
            const NAME: &'static str = $name;
            type Output = Vec<String>;
            fn run(&self, _: &State, _: &QueryCtx<'_>) -> Result<Vec<String>, OpError> {
                Ok(vec![])
            }
        }
    };
}
unit_query!(Big, "demo.big");
unit_query!(Internal, "demo.internal");
unit_query!(BadParams, "demo.badparams");

/// How a faulty mock catalog fails.
#[derive(Clone, Copy)]
pub enum CatalogFault {
    EndpointError,
    Unreadable,
}

pub struct Mock {
    reg: Registry,
    /// Every request received, in order.
    pub log: Vec<Request>,
    /// Extra raw catalog op entries (to test hostile catalogs).
    pub extra_ops: Vec<Value>,
    /// Extra raw catalog query entries.
    pub extra_queries: Vec<Value>,
    /// Make the catalog fail: `Err` from the endpoint, or an unreadable payload.
    pub catalog_fault: Option<CatalogFault>,
    /// Replaces the `demo.big` payload size (number of array elements).
    pub big: usize,
}

impl Mock {
    pub fn new() -> Self {
        let mut reg = Registry::new();
        reg.register::<CreateBrick>().unwrap();
        reg.register::<MeshRun>().unwrap();
        reg.register_query::<Count>().unwrap();
        reg.register_query::<Related>().unwrap();
        reg.register_query::<Big>().unwrap();
        reg.register_query::<Internal>().unwrap();
        reg.register_query::<BadParams>().unwrap();
        Self { reg, log: Vec::new(), extra_ops: Vec::new(), extra_queries: Vec::new(), catalog_fault: None, big: 0 }
    }

    pub fn catalog_calls(&self) -> usize {
        self.log.iter().filter(|r| matches!(r, Request::Catalog)).count()
    }

    fn catalog(&self) -> Value {
        let ops = self
            .reg
            .catalog()
            .into_iter()
            .map(|i| sm_op_api::OpInfoOwned::from(&i.unwrap()))
            .collect();
        let queries = self
            .reg
            .query_catalog()
            .into_iter()
            .map(|q| sm_op_api::QueryInfoOwned::from(&q.unwrap()))
            .collect();
        let batch_schema = serde_json::to_value(self.reg.batch_schema().unwrap()).unwrap();
        let c = CatalogResult { protocol: PROTOCOL_VERSION, ops, queries, batch_schema };
        let mut v = serde_json::to_value(&c).unwrap();
        for e in &self.extra_ops {
            v["ops"].as_array_mut().unwrap().push(e.clone());
        }
        for e in &self.extra_queries {
            v["queries"].as_array_mut().unwrap().push(e.clone());
        }
        v
    }
}

fn volume(n: u64) -> EntityId {
    EntityId::new(EntityKind::TopoVolume, n).unwrap()
}

impl Endpoint for Mock {
    fn call(&mut self, req: Request) -> Result<Value, RpcError> {
        self.log.push(req.clone());
        match req {
            Request::Catalog => match self.catalog_fault {
                Some(CatalogFault::EndpointError) => Err(RpcError::internal("catalog down")),
                Some(CatalogFault::Unreadable) => Ok(json!({"ops": "nope"})),
                None => Ok(self.catalog()),
            },
            Request::Apply { op, .. } => match op.as_str() {
                "mesh.run" => Err(RpcError::application(
                    "op-invalid",
                    "TopoVolume#1 has no scheme; run mesh.scheme first",
                )),
                _ => {
                    let mut d = Delta::default();
                    d.created.push(volume(1));
                    d.outputs.insert("volume".into(), vec![volume(1)]);
                    d.touched_layers.push("model".into());
                    Ok(serde_json::to_value(ApplyResult::new(h(1), h(2), d)).unwrap())
                }
            },
            Request::Query { name, input, .. } => match name.as_str() {
                "demo.count" => Ok(json!(7)),
                "topo.related" => {
                    // Typed parse, like a real endpoint: a rejected id is invalid params.
                    serde_json::from_value::<Related>(input)
                        .map_err(|e| RpcError::invalid_params(e.to_string()))?;
                    Ok(json!(["TopoFace#1", "TopoFace#2"]))
                }
                "demo.big" => {
                    Ok(Value::Array((0..self.big).map(|i| json!(format!("a\nb{i}"))).collect()))
                }
                "demo.internal" => Err(RpcError::internal("boom")),
                "demo.badparams" => Err(RpcError::invalid_params("input does not fit")),
                _ => Err(RpcError::application("unknown-query", format!("no query `{name}`"))),
            },
            Request::ScriptRun { text, .. } => {
                if text.contains("fail") {
                    Err(RpcError::application("script-error", "line 1: TopoFace#3 is not a volume")
                        .with_data(json!({"line": 1})))
                } else {
                    Ok(serde_json::to_value(ScriptResult::new(h(3), "ok\n", vec![], json!([])))
                        .unwrap())
                }
            }
            Request::Status | Request::Checkout { .. } | Request::Branch { .. }
            | Request::Undo | Request::Redo | Request::Save | Request::Open { .. } => {
                Ok(serde_json::to_value(StatusResult::new(
                    sm_op_api::Head::Ref("main".into()),
                    h(2),
                ))
                .unwrap())
            }
            Request::Log { limit, .. } => Ok(json!({"limit": limit, "nodes": []})),
            Request::Hash { .. } => {
                Ok(serde_json::to_value(HashResult::new(h(2), Default::default())).unwrap())
            }
            Request::Export { mesh: None, .. } => {
                Err(RpcError::application("no-mesh", "the state holds no mesh"))
            }
            Request::Export { path, .. } => Ok(serde_json::to_value(ExportResult::new(
                path,
                ExportFormat::Smsh,
                12,
                h(7),
            ))
            .unwrap()),
            _ => Err(RpcError::method_not_found("?")),
        }
    }
}

/// Run `lines` (one JSON text per line) through `run_stdio`; the raw output text.
pub fn run_raw(mock: &mut Mock, input: &str) -> String {
    let mut out = Vec::new();
    sm_mcp::run_stdio(mock, Cursor::new(input.as_bytes().to_vec()), &mut out).unwrap();
    String::from_utf8(out).unwrap()
}

/// Run requests; the replies, parsed.
pub fn run(mock: &mut Mock, msgs: &[Value]) -> Vec<Value> {
    let input: String = msgs.iter().map(|m| format!("{m}\n")).collect();
    run_raw(mock, &input)
        .lines()
        .map(|l| serde_json::from_str(l).unwrap())
        .collect()
}

pub fn init(version: &str) -> Value {
    json!({"jsonrpc":"2.0","id":0,"method":"initialize","params":{
        "protocolVersion": version, "capabilities": {}, "clientInfo": {"name":"t","version":"0"}}})
}

pub fn call(id: u64, name: &str, args: Value) -> Value {
    json!({"jsonrpc":"2.0","id":id,"method":"tools/call","params":{"name":name,"arguments":args}})
}

pub fn list(id: u64) -> Value {
    json!({"jsonrpc":"2.0","id":id,"method":"tools/list"})
}
