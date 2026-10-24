//! Session host, protocol endpoint, transports and the standard distribution. Skeleton: see W2
//! plan §2 task T10 (`docs/design/w2/plan.md`); that task fills in the bodies.
//!
//! [`Server`] implements [`sm_protocol::Endpoint`]. Only `catalog` and `session.hash` work in the
//! skeleton; every other method answers a typed `method-not-implemented` error.

use std::sync::Arc;

use serde_json::Value;
use sm_engine::{EngineError, Workspace, WorkspaceBuilder};
use sm_layer_api::CoreLayers;
use sm_mesher::{MeshPlugin, MesherConfig};
use sm_model::ModelPlugin;
use sm_op_api::{OpInfoOwned, QueryInfoOwned, Services};
use sm_protocol::{CatalogResult, Endpoint, HashResult, METHOD_NOT_FOUND, PROTOCOL_VERSION, Request, RpcError};
use sm_scheme_api::{SchemeDim, SchemeRegistry};
use sm_sizing::{AnglePicker, IiaSolver};
use sm_tess::TessPlugin;

/// Errors of [`Server::new`].
#[derive(Debug)]
#[non_exhaustive]
pub enum ServerError {
    /// The workspace could not be built.
    Engine(EngineError),
}

impl std::fmt::Display for ServerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Engine(e) => write!(f, "workspace: {e}"),
        }
    }
}

impl std::error::Error for ServerError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Engine(e) => Some(e),
        }
    }
}

impl From<EngineError> for ServerError {
    fn from(e: EngineError) -> Self {
        Self::Engine(e)
    }
}

/// A single-writer protocol server over one workspace.
pub struct Server {
    ws: Workspace,
}

impl Server {
    /// A server over an in-memory workspace built from `builder`.
    pub fn new(builder: WorkspaceBuilder) -> Result<Self, ServerError> {
        Ok(Self { ws: builder.memory()? })
    }

    /// The workspace.
    pub fn workspace(&self) -> &Workspace {
        &self.ws
    }
}

fn engine_err(e: &EngineError) -> RpcError {
    RpcError::application(e.code(), e.to_string())
}

macro_rules! to_value {
    ($v:expr) => {
        serde_json::to_value($v).map_err(|e| RpcError::internal(e.to_string()))
    };
}

impl Endpoint for Server {
    fn call(&mut self, req: Request) -> Result<Value, RpcError> {
        match req {
            Request::Catalog => {
                let reg = self.ws.registry();
                let ops = reg
                    .catalog()
                    .into_iter()
                    .collect::<Result<Vec<_>, _>>()
                    .map_err(|e| RpcError::internal(e.to_string()))?;
                let queries = reg
                    .query_catalog()
                    .into_iter()
                    .collect::<Result<Vec<_>, _>>()
                    .map_err(|e| RpcError::internal(e.to_string()))?;
                let batch_schema = to_value!(&reg.batch_schema().map_err(|e| RpcError::internal(e.to_string()))?)?;
                let ops = ops.iter().map(OpInfoOwned::from).collect();
                let queries = queries.iter().map(QueryInfoOwned::from).collect();
                to_value!(&CatalogResult { protocol: PROTOCOL_VERSION, ops, queries, batch_schema })
            }
            Request::Hash { at } => {
                let state = match at {
                    None => self.ws.state().clone(),
                    Some(rev) => self.ws.state_at(&rev).map_err(|e| engine_err(&e))?,
                };
                to_value!(&HashResult::new(state.hash(), state.layer_hashes()))
            }
            other => Err(RpcError::new(
                METHOD_NOT_FOUND,
                "method-not-implemented",
                format!("`{}` is not implemented yet", other.method()),
            )),
        }
    }
}

/// The standard distribution: core layers, the four plugins and the mesher service with the
/// reference-compatible solver, picker and sizing hooks.
pub fn standard() -> WorkspaceBuilder {
    let mut registry = SchemeRegistry::new();
    // Skeleton registers fail only on duplicates, which cannot happen here.
    for r in [sm_scheme_curve::register, sm_scheme_map::register, sm_scheme_mapvol::register] {
        let _ = r(&mut registry);
    }
    // The skeleton schemes register nothing yet, so only set lists whose members exist.
    for (dim, name) in [(SchemeDim::Curve, "equal"), (SchemeDim::Surface, "map"), (SchemeDim::Volume, "map")] {
        if registry.get(dim, name).is_some() {
            let _ = registry.set_auto(dim, &[name]);
        }
    }
    let mesher = MesherConfig::new(registry)
        .with_solver(Box::new(IiaSolver::default()))
        .with_picker(Box::new(AnglePicker::default()))
        .with_size_fields(Arc::new(sm_sizing::size_field))
        .with_auto_size(Arc::new(|view, targets, factor| {
            sm_sizing::auto_size(view, targets, factor).map_err(|e| e.to_string())
        }));
    WorkspaceBuilder::new()
        .plugin(CoreLayers::new(|| Box::new(sm_topo::TopoStore::new())))
        .plugin(ModelPlugin)
        .plugin(MeshPlugin)
        .plugin(sm_fem::FemPlugin)
        .plugin(TessPlugin)
        .services(Services::new().with_versioned("mesher", 1, mesher))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn catalog_and_hash_work_and_the_rest_is_typed() {
        let mut s = Server::new(standard()).expect("server");
        let cat = s.call(Request::Catalog).expect("catalog");
        assert_eq!(cat["protocol"], PROTOCOL_VERSION);
        assert!(cat["ops"].as_array().is_some_and(|o| !o.is_empty()), "core.batch is there");
        let h = s.call(Request::Hash { at: None }).expect("hash");
        assert!(h["layers"].is_object() && h["state"].as_str().is_some_and(|s| s.len() == 64), "{h}");
        let e = s.call(Request::Status).expect_err("stub");
        assert_eq!(e.kind, "method-not-implemented");
    }
}
