//! Pure-Rust core of the bindings: [`Host`] (a `Mutex` over the session server) and the mesh table
//! snapshot [`MeshTables`]. No PyO3 types here, so `cargo test` covers it without a Python.

use std::path::Path;
use std::sync::{Mutex, MutexGuard};

use serde_json::{Value, json};
use sm_core::EntityId;
use sm_layer_api::MeshLayer;
use sm_mesh_api::{CellIx, FaceIx, MeshEntityKind, MeshRead, NO_CELL};
use sm_protocol::{Endpoint, RpcError};
use sm_server::{InputRef, Server, ServerError};

/// Why a [`Host`] call failed outside the protocol.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum HostError {
    /// The server could not be built, opened or created, or staging failed.
    #[error("{0}")]
    Server(#[from] ServerError),
    /// The state holds no mesh.
    #[error("the state holds no mesh")]
    NoMesh,
    /// No mesh was named and the state holds several.
    #[error("the state holds {count} meshes; name the mesh to export")]
    AmbiguousMesh {
        /// Number of meshes.
        count: usize,
        /// Their ids, ascending.
        meshes: Vec<EntityId>,
    },
    /// The named mesh is not in the state.
    #[error("unknown mesh {0}")]
    UnknownMesh(EntityId),
    /// An earlier panic poisoned the session lock; the session is no longer trusted.
    #[error("the session lock is poisoned by an earlier panic; open a new session")]
    Poisoned,
}

impl HostError {
    /// The error as a protocol error: `kind` is the contract code string.
    pub fn to_rpc(&self) -> RpcError {
        match self {
            HostError::NoMesh => RpcError::application("no-mesh", self.to_string()),
            HostError::AmbiguousMesh { meshes, .. } => {
                let ids: Vec<String> = meshes.iter().map(ToString::to_string).collect();
                let mut e = RpcError::application("ambiguous-mesh", self.to_string());
                e.data = json!({ "meshes": ids });
                e
            }
            HostError::UnknownMesh(_) => RpcError::application("unknown-mesh", self.to_string()),
            HostError::Poisoned => RpcError::internal(self.to_string()),
            HostError::Server(e) => RpcError::application("server", e.to_string()),
        }
    }
}

/// A session: one [`Server`] behind a mutex. Callers serialize on the lock, so ops stay
/// sequential and deterministic.
pub struct Host {
    server: Mutex<Server>,
}

impl Host {
    /// An in-memory session over the standard distribution.
    pub fn memory() -> Result<Self, HostError> {
        Self::memory_with(None, None)
    }

    /// [`memory`](Self::memory) with an author and a fixed RFC 3339 timestamp (reproducible node
    /// hashes, like `smesh --author --fixed-time`).
    pub fn memory_with(author: Option<String>, fixed_time: Option<String>) -> Result<Self, HostError> {
        let server = Server::from_factory(move || sm_server::standard_with(author.clone(), fixed_time.clone()))?;
        Ok(Self { server: Mutex::new(server) })
    }

    /// The project at `dir`; `create = Some(name)` makes a new project of that name instead.
    pub fn open(dir: &Path, create: Option<&str>) -> Result<Self, HostError> {
        let server = match create {
            Some(name) => Server::create(sm_server::standard(), dir, name)?,
            None => Server::open(sm_server::standard(), dir)?,
        };
        Ok(Self { server: Mutex::new(server.with_opener(sm_server::standard)) })
    }

    fn lock(&self) -> Result<MutexGuard<'_, Server>, HostError> {
        self.server.lock().map_err(|_| HostError::Poisoned)
    }

    /// Run `f` with the server locked (test hooks and the binding glue).
    pub fn with_server<R>(&self, f: impl FnOnce(&mut Server) -> R) -> Result<R, HostError> {
        Ok(f(&mut *self.lock()?))
    }

    /// Call a protocol method. The request goes through [`Endpoint::handle_text`] as JSON-RPC
    /// text, exactly as `InProcess` and remote clients do, so errors and hashes match them.
    pub fn call(&self, method: &str, params: &Value) -> Result<Value, RpcError> {
        let text = json!({ "jsonrpc": "2.0", "id": 1, "method": method, "params": params }).to_string();
        let resp = self.lock().map_err(|e| e.to_rpc())?.handle_text(&text);
        match (resp.result, resp.error) {
            (_, Some(e)) => Err(e),
            (Some(v), None) => Ok(v),
            (None, None) => Err(RpcError::internal("reply carries neither result nor error")),
        }
    }

    /// Stage `bytes` in the session's content-addressed input store.
    pub fn stage(&self, bytes: &[u8]) -> Result<InputRef, HostError> {
        Ok(self.lock()?.stage_input(bytes)?)
    }

    /// Ids of the meshes in the current state, ascending.
    pub fn mesh_ids(&self) -> Result<Vec<EntityId>, HostError> {
        let layer = self.mesh_layer()?;
        Ok(layer.map(|l| l.meshes().map(|(id, _)| id).collect()).unwrap_or_default())
    }

    fn mesh_layer(&self) -> Result<Option<MeshLayer>, HostError> {
        // Cloning the layer copies only `Arc`s, so the lock is released before any table is built.
        Ok(self.lock()?.workspace().state().layer::<MeshLayer>().cloned())
    }

    /// Snapshot the tables of a mesh. `None` means the only mesh; none or several is
    /// [`HostError::NoMesh`] / [`HostError::AmbiguousMesh`].
    pub fn mesh_tables(&self, mesh: Option<EntityId>) -> Result<MeshTables, HostError> {
        let layer = self.mesh_layer()?.ok_or_else(|| mesh.map_or(HostError::NoMesh, HostError::UnknownMesh))?;
        let id = match mesh {
            Some(id) => id,
            None => {
                let ids: Vec<EntityId> = layer.meshes().map(|(id, _)| id).collect();
                match ids.as_slice() {
                    [] => return Err(HostError::NoMesh),
                    [one] => *one,
                    _ => return Err(HostError::AmbiguousMesh { count: ids.len(), meshes: ids }),
                }
            }
        };
        let entry = layer.get(id).ok_or(HostError::UnknownMesh(id))?;
        Ok(tables_of(entry.mesh()))
    }
}

/// One table column with its `.smsh` dtype.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum Column {
    /// `u8`.
    U8(Vec<u8>),
    /// `u32`.
    U32(Vec<u32>),
    /// `u64`.
    U64(Vec<u64>),
    /// `f64`.
    F64(Vec<f64>),
}

impl Column {
    /// Number of elements.
    pub fn len(&self) -> usize {
        match self {
            Column::U8(v) => v.len(),
            Column::U32(v) => v.len(),
            Column::U64(v) => v.len(),
            Column::F64(v) => v.len(),
        }
    }

    /// True when the column has no element.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The bytes of the column (native endianness; the `.smsh` writer stores little endian).
    pub fn bytes(&self) -> &[u8] {
        match self {
            Column::U8(v) => v,
            Column::U32(v) => bytemuck::cast_slice(v),
            Column::U64(v) => bytemuck::cast_slice(v),
            Column::F64(v) => bytemuck::cast_slice(v),
        }
    }

    /// The `.smsh` dtype name.
    pub fn dtype(&self) -> &'static str {
        match self {
            Column::U8(_) => "u8",
            Column::U32(_) => "u32",
            Column::U64(_) => "u64",
            Column::F64(_) => "f64",
        }
    }
}

/// Owned snapshot of the always-present `.smsh` arrays of a mesh (`SMSH_FORMAT.md` 3.1), in file
/// order, with the file's names and dtypes.
#[derive(Clone, Debug, PartialEq)]
pub struct MeshTables {
    /// `(name, column)` in `.smsh` order.
    pub columns: Vec<(&'static str, Column)>,
}

impl MeshTables {
    /// A column by name.
    pub fn get(&self, name: &str) -> Option<&Column> {
        self.columns.iter().find(|(n, _)| *n == name).map(|(_, c)| c)
    }
}

/// Raw ids of a kind, ascending with the dense index.
fn raw_ids(m: &dyn MeshRead, k: MeshEntityKind) -> Vec<u64> {
    (0..m.count(k)).map(|i| m.id(k, i as u32).to_raw()).collect()
}

/// Classification index columns against the sorted unique entity table.
fn class_columns(m: &dyn MeshRead) -> (Vec<u64>, [Vec<u32>; 3]) {
    let kinds = [MeshEntityKind::Vertex, MeshEntityKind::Face, MeshEntityKind::Cell];
    let raw: [Vec<u64>; 3] =
        kinds.map(|k| (0..m.count(k)).map(|i| m.classification(k, i as u32).entity.to_raw()).collect());
    // The `.smsh` writer folds the edge table's classifications into the entity table too.
    let mut table: Vec<u64> = raw.iter().flatten().copied().collect();
    if let Some(t) = m.edges() {
        table.extend(t.class.iter().map(|c| c.entity.to_raw()));
    }
    table.sort_unstable();
    table.dedup();
    let idx = |v: &[u64]| -> Vec<u32> { v.iter().map(|r| table.binary_search(r).map_or(0, |i| i as u32)).collect() };
    let cols = [idx(&raw[0]), idx(&raw[1]), idx(&raw[2])];
    (table, cols)
}

/// Snapshot the tables of `m` (one copy per table; nothing aliases the mesh).
pub fn tables_of(m: &dyn MeshRead) -> MeshTables {
    let (nf, nc) = (m.count(MeshEntityKind::Face), m.count(MeshEntityKind::Cell));
    let co = m.coords();
    let cols = m.columns();
    let (class_entities, [vclass, fclass, cclass]) = class_columns(m);
    let face_kind: Vec<u8> = (0..nf).map(|i| m.face_kind(FaceIx::new(i)).code()).collect();
    let cell_shape: Vec<u8> = (0..nc).map(|c| m.cell_shape(CellIx::new(c)).code()).collect();
    let mut corner_off = Vec::with_capacity(nc + 1);
    let mut corners: Vec<u32> = Vec::new();
    corner_off.push(0u64);
    for c in 0..nc {
        if let Some(cs) = m.cell_corners(CellIx::new(c)) {
            corners.extend(cs.iter().map(|v| v.0));
        }
        corner_off.push(corners.len() as u64);
    }
    let parent: Vec<u32> = (0..nc).map(|c| m.cell_parent(CellIx::new(c)).map_or(NO_CELL, |p| p.0)).collect();
    let active: Vec<u8> = (0..nc).map(|c| u8::from(m.cell_active(CellIx::new(c)))).collect();
    let columns = vec![
        ("x", Column::F64(co.x.to_vec())),
        ("y", Column::F64(co.y.to_vec())),
        ("z", Column::F64(co.z.to_vec())),
        ("vertex_ids", Column::U64(raw_ids(m, MeshEntityKind::Vertex))),
        ("face_ids", Column::U64(raw_ids(m, MeshEntityKind::Face))),
        ("cell_ids", Column::U64(raw_ids(m, MeshEntityKind::Cell))),
        ("face_offsets", Column::U64(cols.face_vertex_offsets.to_vec())),
        ("face_vertices", Column::U32(bytemuck::cast_slice(cols.face_vertices).to_vec())),
        ("face_left", Column::U32(bytemuck::cast_slice(cols.face_left).to_vec())),
        ("face_right", Column::U32(cols.face_right.to_vec())),
        ("face_kind", Column::U8(face_kind)),
        ("cell_face_offsets", Column::U64(cols.cell_face_offsets.to_vec())),
        ("cell_faces", Column::U32(bytemuck::cast_slice(cols.cell_faces).to_vec())),
        ("cell_shape", Column::U8(cell_shape)),
        ("cell_corner_offsets", Column::U64(corner_off)),
        ("cell_corners", Column::U32(corners)),
        ("cell_parent", Column::U32(parent)),
        ("cell_active", Column::U8(active)),
        ("class_entities", Column::U64(class_entities)),
        ("vertex_class", Column::U32(vclass)),
        ("face_class", Column::U32(fclass)),
        ("cell_class", Column::U32(cclass)),
    ];
    MeshTables { columns }
}
