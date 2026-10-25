//! [`SessionHost`]: the command-language [`Host`] over a [`Workspace`], and the session
//! operations (export, save, status, timeline control) that both the language and the protocol
//! [`Server`](crate::Server) use, so each exists exactly once.

use std::collections::BTreeMap;
use std::path::{Component, Path, PathBuf};

use serde_json::Value;
use sm_core::{EntityId, EntityKind};
use sm_engine::{ApplyOptions, Workspace};
use sm_fem::{ExportFormat as FemFormat, ExportRequest, FemError};
use sm_lang::{Host, HostError, QueryRelations, Relations, SessionCommand, TimelineCommand, TimelineHead};
use sm_layer_api::MeshLayer;
use sm_op_api::{Delta, Engine, Hash256, Head, OpRecord, Registry, Rev, State};
use sm_protocol::{ExportFormat, ExportResult, StatusResult};

use crate::b64;
use crate::error::SessionError;

/// Name of the adjacency query the language's `kind in X` filters call.
pub const RELATIONS_QUERY: &str = "topo.related";

/// Largest include file the host loads, in bytes.
const MAX_INCLUDE_BYTES: u64 = 16 << 20;

/// What `session.export` asks for, in protocol terms.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ExportParams {
    /// Output path (see [`SessionHost::resolve_path`]); none means inline-only.
    pub path: Option<String>,
    /// Format; inferred from the path extension when absent.
    pub format: Option<ExportFormat>,
    /// Mesh to export; the only mesh when absent.
    pub mesh: Option<EntityId>,
    /// Embed the current timeline node as provenance.
    pub provenance: bool,
    /// Return the bytes base64-encoded in the result.
    pub inline: bool,
}

/// Adjacency from the registered `topo.related` query, owning a handle to the engine so the host
/// needs no self-reference.
struct EngineRelations {
    engine: Engine,
}

impl Relations for EngineRelations {
    fn related(&self, state: &State, target: EntityKind, within: &[EntityId]) -> Result<Vec<EntityId>, String> {
        QueryRelations::new(&self.engine, RELATIONS_QUERY).related(state, target, within)
    }
}

/// A [`Host`] over `&mut Workspace`: one timeline node per applied record, timeline control on
/// the workspace, session commands (export, save, hash, status) and include loading.
///
/// * `apply` is [`Workspace::apply_json`]: params are re-encoded canonically, so a script, an
///   MCP call and an `apply` request that mean the same op make the same node.
/// * `timeline(Branch)` is [`Workspace::fork`] (creates the branch **and** attaches HEAD).
/// * `load` reads the include map first, then files below the project root (never outside it).
pub struct SessionHost<'w> {
    ws: &'w mut Workspace,
    relations: Option<EngineRelations>,
    includes: BTreeMap<String, String>,
}

impl<'w> SessionHost<'w> {
    /// A host over `ws`. Adjacency is available when the registry has the `topo.related` query.
    pub fn new(ws: &'w mut Workspace) -> Self {
        let relations = ws
            .registry()
            .query_names()
            .contains(&RELATIONS_QUERY)
            .then(|| EngineRelations { engine: ws.session().engine().clone() });
        Self { ws, relations, includes: BTreeMap::new() }
    }

    /// A host for the session operations only (hash, export, save): no adjacency lookup, so no
    /// registry scan per request.
    pub(crate) fn plain(ws: &'w mut Workspace) -> Self {
        Self { ws, relations: None, includes: BTreeMap::new() }
    }

    /// In-memory include files, by the name a script gives to `include`.
    #[must_use]
    pub fn with_includes(mut self, includes: BTreeMap<String, String>) -> Self {
        self.includes = includes;
        self
    }

    /// The workspace.
    pub fn workspace(&self) -> &Workspace {
        self.ws
    }

    /// Resolve a user path: absolute as is; relative against the project root when a project is
    /// attached, else against the process working directory.
    pub fn resolve_path(&self, path: &str) -> Result<PathBuf, SessionError> {
        resolve_path(self.ws.root(), path)
    }

    /// `timeline.status` as the protocol's [`StatusResult`].
    pub fn status_result(&self) -> StatusResult {
        status_result(self.ws)
    }

    /// Execute a timeline command on the workspace.
    pub fn timeline_command(&mut self, cmd: &TimelineCommand) -> Result<(), SessionError> {
        match cmd {
            TimelineCommand::Branch { name, at } => self.ws.fork(name, *at)?,
            TimelineCommand::Checkout(rev) => self.ws.checkout(rev.clone())?,
            TimelineCommand::Undo => self.ws.undo()?,
            TimelineCommand::Redo => self.ws.redo()?,
            other => {
                return Err(SessionError::new(
                    "timeline-unsupported",
                    format!("`timeline.{}` is not supported by this server", other.verb()),
                ));
            }
        }
        Ok(())
    }

    /// Persist pending state (needs a project).
    pub fn save(&mut self) -> Result<(), SessionError> {
        Ok(self.ws.save()?)
    }

    /// The state hash at `at` (the current state when absent), with its per-layer hashes.
    pub fn hash_at(&mut self, at: Option<&Rev>) -> Result<(Hash256, BTreeMap<String, Hash256>), SessionError> {
        let state = match at {
            None => self.ws.state().clone(),
            Some(rev) => self.ws.state_at(rev)?,
        };
        Ok((state.hash(), state.layer_hashes()))
    }

    /// Export a mesh of the current state: build the bytes with [`sm_fem::export`], then write
    /// them atomically (temp file and rename) when a path is given.
    ///
    /// With no mesh named, [`sm_fem::export`] picks the only mesh; none is `no-mesh`, several is
    /// `ambiguous-mesh` (with `data.meshes`), a named id that is not in the state is
    /// `unknown-mesh`. Other failures carry [`FemError::code`]. An explicit `format` that
    /// disagrees with a `.smsh`/`.vtu` path extension, and `provenance` with a format that cannot
    /// record it (VTU), are `bad-request`. The bytes are a pure function of the state and the
    /// request.
    pub fn export(&mut self, p: &ExportParams) -> Result<ExportResult, SessionError> {
        let path = p.path.as_deref().map(|s| self.resolve_path(s)).transpose()?;
        if path.is_none() && !p.inline {
            return Err(SessionError::bad_request("export needs a `path`, or `inline: true`"));
        }
        let format = match (p.format, &path) {
            (Some(f), Some(path)) => {
                if let Some(by_ext) = known_format_of(path)
                    && by_ext != f
                {
                    return Err(SessionError::bad_request(format!(
                        "format `{}` disagrees with the extension of {}",
                        format_name(f),
                        path.display()
                    )));
                }
                f
            }
            (Some(f), None) => f,
            (None, Some(path)) => format_of(path)?,
            (None, None) => return Err(SessionError::bad_request("inline export needs an explicit `format`")),
        };
        let fem_format = match format {
            ExportFormat::Smsh => FemFormat::Smsh,
            ExportFormat::Vtu => FemFormat::Vtu,
            _ => return Err(SessionError::bad_request("unsupported export format")),
        };
        if p.provenance && format != ExportFormat::Smsh {
            return Err(SessionError::bad_request(format!(
                "`provenance` is only recorded in `smsh` exports, not `{}`",
                format_name(format)
            )));
        }
        let state = self.ws.state();
        let mut req = ExportRequest::new(fem_format);
        req.mesh = p.mesh;
        if p.provenance {
            req.provenance = provenance_of(self.ws.status().position);
        }
        let mut bytes = Vec::new();
        sm_fem::export(state, &req, &mut bytes).map_err(|e| export_error(&e, state, p.mesh))?;
        if let Some(path) = &path {
            write_atomic(path, &bytes)?;
        }
        let mut out = ExportResult::new(
            path.as_ref().map(|p| p.to_string_lossy().into_owned()),
            format,
            bytes.len() as u64,
            Hash256::of(&bytes),
        );
        if p.inline {
            out = out.with_base64(b64::encode(&bytes));
        }
        Ok(out)
    }

    fn session_value(&mut self, cmd: &SessionCommand) -> Result<Value, SessionError> {
        match cmd {
            SessionCommand::Export { path, format, mesh, provenance } => {
                let format = format.as_deref().map(parse_format).transpose()?;
                let r = self.export(&ExportParams {
                    path: Some(path.clone()),
                    format,
                    mesh: *mesh,
                    provenance: *provenance,
                    inline: false,
                })?;
                to_json(&r)
            }
            SessionCommand::Save => {
                self.save()?;
                Ok(strip_nulls(to_json(&self.status_result())?))
            }
            SessionCommand::Hash => Ok(Value::String(self.ws.state_hash().to_hex())),
            SessionCommand::Status => Ok(strip_nulls(to_json(&self.status_result())?)),
            other => Err(SessionError::new(
                "session-unsupported",
                format!("`session.{}` is not supported by this server", other.verb()),
            )),
        }
    }

    fn load_include(&self, path: &str) -> Result<String, String> {
        if let Some(text) = self.includes.get(path) {
            return Ok(text.clone());
        }
        let Some(root) = self.ws.root() else {
            return Err(format!("include {path:?} is not in the request and there is no project to read it from"));
        };
        let rel = Path::new(path);
        if rel.is_absolute() || rel.components().any(|c| !matches!(c, Component::Normal(_) | Component::CurDir)) {
            return Err(format!("include {path:?} must be a relative path below the project root"));
        }
        let root = root.canonicalize().map_err(|e| format!("project root: {e}"))?;
        let full = root.join(rel).canonicalize().map_err(|e| format!("include {path:?}: {e}"))?;
        if !full.starts_with(&root) {
            return Err(format!("include {path:?} resolves outside the project root"));
        }
        let meta = std::fs::metadata(&full).map_err(|e| format!("include {path:?}: {e}"))?;
        if !meta.is_file() || meta.len() > MAX_INCLUDE_BYTES {
            return Err(format!("include {path:?} is not a regular file of at most {MAX_INCLUDE_BYTES} bytes"));
        }
        std::fs::read_to_string(&full).map_err(|e| format!("include {path:?}: {e}"))
    }
}

impl Host for SessionHost<'_> {
    fn registry(&self) -> &Registry {
        self.ws.registry()
    }

    fn state(&self) -> &State {
        self.ws.state()
    }

    fn relations(&self) -> Option<&dyn Relations> {
        self.relations.as_ref().map(|r| r as &dyn Relations)
    }

    fn apply(&mut self, rec: &OpRecord) -> Result<Delta, HostError> {
        let opts = ApplyOptions { version: Some(rec.version), ..ApplyOptions::default() };
        match self.ws.apply_json(&rec.name, rec.params.clone(), &opts) {
            Ok(report) => Ok(report.delta),
            Err(e) => Err(SessionError::from(e).into()),
        }
    }

    fn query(&mut self, name: &str, params: &Value) -> Result<Value, HostError> {
        self.ws.query(name, params).map_err(|e| SessionError::from(e).into())
    }

    fn timeline(&mut self, cmd: &TimelineCommand) -> Result<(), HostError> {
        self.timeline_command(cmd).map_err(Into::into)
    }

    fn timeline_head(&mut self) -> Result<TimelineHead, HostError> {
        let st = self.ws.status();
        let branch = match st.head {
            Head::Ref(name) => Some(name),
            _ => None,
        };
        Ok(TimelineHead::new(branch, st.position.map(|h| h.to_hex())))
    }

    fn session(&mut self, cmd: &SessionCommand) -> Result<Value, HostError> {
        self.session_value(cmd).map_err(Into::into)
    }

    fn load(&mut self, path: &str) -> Result<String, String> {
        self.load_include(path)
    }
}

/// The protocol's status of `ws`.
pub(crate) fn status_result(ws: &Workspace) -> StatusResult {
    let st = ws.status();
    StatusResult::new(st.head, st.state)
        .with_position(st.position, st.depth)
        .with_branches(st.nodes, st.branches)
        .with_redo(st.redo)
        .with_dirty(st.dirty)
        .with_project(st.project)
}

pub(crate) fn resolve_path(root: Option<&Path>, path: &str) -> Result<PathBuf, SessionError> {
    if path.is_empty() {
        return Err(SessionError::bad_request("empty path"));
    }
    let p = Path::new(path);
    if p.is_absolute() {
        return Ok(p.to_owned());
    }
    match root {
        Some(root) => Ok(root.join(p)),
        None => std::env::current_dir()
            .map(|cwd| cwd.join(p))
            .map_err(|e| SessionError::new("io", format!("working directory: {e}"))),
    }
}

fn format_of(path: &Path) -> Result<ExportFormat, SessionError> {
    known_format_of(path).ok_or_else(|| {
        SessionError::bad_request(format!(
            "cannot infer the export format from {}: use `.smsh` or `.vtu`, or pass `format`",
            path.display()
        ))
    })
}

fn parse_format(s: &str) -> Result<ExportFormat, SessionError> {
    match s {
        "smsh" => Ok(ExportFormat::Smsh),
        "vtu" => Ok(ExportFormat::Vtu),
        other => Err(SessionError::bad_request(format!("unknown export format `{other}` (expected `smsh` or `vtu`)"))),
    }
}

/// The format a `.smsh`/`.vtu` extension names; none for any other extension.
fn known_format_of(path: &Path) -> Option<ExportFormat> {
    match path.extension().and_then(|e| e.to_str()).map(str::to_ascii_lowercase).as_deref() {
        Some("smsh") => Some(ExportFormat::Smsh),
        Some("vtu") => Some(ExportFormat::Vtu),
        _ => None,
    }
}

fn format_name(f: ExportFormat) -> &'static str {
    match f {
        ExportFormat::Vtu => "vtu",
        _ => "smsh",
    }
}

/// The provenance string of an export: the current node hash, none in the empty state.
fn provenance_of(position: Option<Hash256>) -> Option<String> {
    position.map(Hash256::to_hex)
}

/// A failed [`sm_fem::export`] as a session error, with [`FemError::code`] as the code. A state
/// without a `meshes` layer holds no mesh (`no-mesh`), or, when `named` asks for one,
/// does not hold that one (`unknown-mesh`); an ambiguous pick lists the meshes in `data.meshes`.
fn export_error(e: &FemError, state: &State, named: Option<EntityId>) -> SessionError {
    match e {
        FemError::NoMeshLayer if named.is_some() => SessionError::new("unknown-mesh", e.to_string()),
        FemError::NoMeshLayer => SessionError::new("no-mesh", e.to_string()),
        FemError::AmbiguousMesh(_) => {
            let meshes: Vec<String> =
                state.layer::<MeshLayer>().map(|l| l.meshes().map(|(id, _)| id.to_string()).collect()).unwrap_or_default();
            SessionError::new(e.code(), e.to_string()).with_data(serde_json::json!({ "meshes": meshes }))
        }
        other => SessionError::from(other),
    }
}

/// Write `bytes` to `path` through a sibling temp file and a rename, so a reader never sees a
/// half-written export.
fn write_atomic(path: &Path, bytes: &[u8]) -> Result<(), SessionError> {
    let io = |what: &str, e: std::io::Error| SessionError::new("io", format!("{what} {}: {e}", path.display()));
    let Some(name) = path.file_name() else {
        return Err(SessionError::bad_request(format!("{} has no file name", path.display())));
    };
    let mut tmp_name = name.to_owned();
    tmp_name.push(".part");
    let tmp = path.with_file_name(tmp_name);
    std::fs::write(&tmp, bytes).map_err(|e| io("writing", e))?;
    std::fs::rename(&tmp, path).map_err(|e| {
        let _ = std::fs::remove_file(&tmp);
        io("renaming into", e)
    })
}

/// `v` as JSON; a serialization failure is an `internal` error, never a null success.
pub(crate) fn to_json<T: serde::Serialize>(v: &T) -> Result<Value, SessionError> {
    serde_json::to_value(v).map_err(|e| SessionError::internal(e.to_string()))
}

/// The command language has no null: an absent field is simply missing.
fn strip_nulls(v: Value) -> Value {
    match v {
        Value::Object(m) => Value::Object(m.into_iter().filter(|(_, v)| !v.is_null()).map(|(k, v)| (k, strip_nulls(v))).collect()),
        Value::Array(a) => Value::Array(a.into_iter().map(strip_nulls).collect()),
        other => other,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_protocol::RpcError;

    struct Unserializable;

    impl serde::Serialize for Unserializable {
        fn serialize<S: serde::Serializer>(&self, _: S) -> Result<S::Ok, S::Error> {
            Err(serde::ser::Error::custom("boom"))
        }
    }

    #[test]
    fn a_serialization_failure_is_an_internal_error_not_a_null_result() {
        let e = to_json(&Unserializable).expect_err("must fail");
        assert_eq!(e.code, "internal");
        let rpc = RpcError::from(e);
        assert_eq!(rpc.code, sm_protocol::INTERNAL_ERROR);
        assert_eq!(rpc.kind, "internal");
        assert!(rpc.message.contains("boom"), "{rpc}");
    }

    #[test]
    fn provenance_is_the_node_hash_and_absent_in_the_empty_state() {
        let h = Hash256::of(b"node");
        assert_eq!(provenance_of(Some(h)), Some(h.to_hex()));
        assert_eq!(provenance_of(None), None);
    }
}
