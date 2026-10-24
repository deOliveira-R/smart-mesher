//! Blocks, sidesets, deck lowering and export. Skeleton: see W2 plan §2 task T8
//! (`docs/design/w2/plan.md`); that task fills in the bodies.
//!
//! The skeleton [`export`] already writes a mesh of the `meshes` layer with identity lowering (no
//! sets), so `sm-server` can test the export path before the real lowering exists.

use std::io::Write;

use sm_core::EntityId;
use sm_layer_api::MeshLayer;
use sm_meshio::{MeshioError, SmshWriter, VtuOptions, write_vtu};
use sm_op_api::{Plugin, Registry, RegistryError, State};

/// The FEM plugin (`FemLayer`, `fem.*` ops). Registers nothing yet.
#[derive(Clone, Copy, Debug, Default)]
pub struct FemPlugin;

impl Plugin for FemPlugin {
    fn id(&self) -> &str {
        "fem"
    }

    fn register(&self, _registry: &mut Registry) -> Result<(), RegistryError> {
        Ok(())
    }
}

/// Output file format.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ExportFormat {
    /// The native `.smsh` container.
    Smsh,
    /// VTK XML unstructured grid.
    Vtu,
}

/// What to export.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExportRequest {
    /// Output format.
    pub format: ExportFormat,
    /// The mesh to write; the only mesh (or the first by id) when absent.
    pub mesh: Option<EntityId>,
    /// Timeline hash to record as provenance (`.smsh` only).
    pub provenance: Option<String>,
}

impl ExportRequest {
    /// A request for `format` with defaults for everything else.
    pub fn new(format: ExportFormat) -> Self {
        Self { format, mesh: None, provenance: None }
    }
}

/// What an export wrote.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct ExportReport {
    /// The format written.
    pub format: ExportFormat,
    /// The mesh written.
    pub mesh: EntityId,
    /// Number of bytes written.
    pub bytes: u64,
}

/// Errors of [`export`].
#[derive(Debug)]
#[non_exhaustive]
pub enum FemError {
    /// The state has no `meshes` layer.
    NoMeshLayer,
    /// The state holds no mesh.
    NoMesh,
    /// The requested mesh is not in the layer.
    UnknownMesh(EntityId),
    /// The writer failed.
    Meshio(MeshioError),
    /// Writing to the sink failed.
    Io(std::io::Error),
}

impl std::fmt::Display for FemError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoMeshLayer => f.write_str("the state has no `meshes` layer"),
            Self::NoMesh => f.write_str("the state holds no mesh to export"),
            Self::UnknownMesh(id) => write!(f, "unknown mesh {id}"),
            Self::Meshio(e) => write!(f, "mesh writer: {e}"),
            Self::Io(e) => write!(f, "i/o: {e}"),
        }
    }
}

impl std::error::Error for FemError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Meshio(e) => Some(e),
            Self::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<MeshioError> for FemError {
    fn from(e: MeshioError) -> Self {
        Self::Meshio(e)
    }
}

impl From<std::io::Error> for FemError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

/// Counts bytes passing through a writer.
struct Counting<'a> {
    inner: &'a mut dyn Write,
    n: u64,
}

impl Write for Counting<'_> {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let k = self.inner.write(buf)?;
        self.n += k as u64;
        Ok(k)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.inner.flush()
    }
}

/// Write the requested mesh of `state` to `w`. A pure function of `(state, req)`.
///
/// Skeleton: the lowering is the identity (no blocks, no sets).
pub fn export(
    state: &State,
    req: &ExportRequest,
    w: &mut dyn Write,
) -> Result<ExportReport, FemError> {
    let layer = state.layer::<MeshLayer>().ok_or(FemError::NoMeshLayer)?;
    let (id, entry) = match req.mesh {
        Some(id) => (id, layer.get(id).ok_or(FemError::UnknownMesh(id))?),
        None => layer.meshes().next().ok_or(FemError::NoMesh)?,
    };
    let mut out = Counting { inner: w, n: 0 };
    match req.format {
        ExportFormat::Smsh => {
            let mut writer = SmshWriter::new(entry.mesh());
            if let Some(p) = &req.provenance {
                writer = writer.provenance(p.clone());
            }
            writer.write_to(&mut out)?;
        }
        ExportFormat::Vtu => write_vtu(entry.mesh(), &VtuOptions::default(), &mut out)?,
    }
    out.flush()?;
    Ok(ExportReport { format: req.format, mesh: id, bytes: out.n })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    use schemars::JsonSchema;
    use serde::{Deserialize, Serialize};
    use sm_layer_api::{CoreLayers, MeshEntry, ModelLayer, adopt_minted};
    use sm_mesh_api::reference::hex_block;
    use sm_mesh_api::MeshRead;
    use sm_op_api::{
        ApplyCtx, DerivedCache, Engine, Meta, OpError, QuietControl, Services, Session, TypedOp,
    };
    use sm_meshio::SmshFile;

    /// Test op: insert an `n^3` hex block as a mesh entry.
    #[derive(Debug, Serialize, Deserialize, JsonSchema)]
    #[serde(deny_unknown_fields)]
    struct AddHex {
        n: u32,
    }

    impl TypedOp for AddHex {
        const NAME: &'static str = "test.add_hex";
        fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
            let quanta = ctx.layer::<ModelLayer>()?.hash_quanta();
            let before = ctx.layer::<MeshLayer>()?.alloc().clone();
            let mut alloc = before.clone();
            let mesh = hex_block([self.n; 3], [1.0; 3], &mut alloc)
                .map_err(|e| OpError::Failed(e.to_string()))?;
            let id = mesh.mesh_id();
            adopt_minted(ctx, &before, &alloc, "mesh")?;
            let entry = MeshEntry::new(Box::new(mesh), Vec::new(), None, quanta);
            ctx.layer_mut::<MeshLayer>()?.insert(id, entry, alloc)?;
            Ok(())
        }
    }

    fn state_with_hex(n: u32) -> State {
        let mut reg = Registry::new();
        reg.install(&CoreLayers::default()).expect("layers");
        reg.install(&FemPlugin).expect("fem");
        reg.register::<AddHex>().expect("op");
        let engine = Engine::new(Arc::new(reg), Services::new(), Arc::new(DerivedCache::default()))
            .expect("engine");
        let mut s = Session::new(engine);
        if n > 0 {
            let q = QuietControl::new();
            s.apply(&AddHex { n }, Meta::new("test", "", ""), &q.control()).expect("add hex");
        }
        s.state().clone()
    }

    #[test]
    fn exports_smsh_and_vtu() {
        let st = state_with_hex(2);
        let mut buf = Vec::new();
        let rep = export(&st, &ExportRequest::new(ExportFormat::Smsh), &mut buf).expect("smsh");
        assert_eq!(rep.bytes, buf.len() as u64);
        let file = SmshFile::parse(&buf).expect("parse");
        let m = file.decode().expect("decode");
        assert_eq!(m.cell_ids.len(), 8);
        let mut again = Vec::new();
        export(&st, &ExportRequest::new(ExportFormat::Smsh), &mut again).expect("again");
        assert_eq!(buf, again, "byte-identical");

        let mut vtu = Vec::new();
        let rep = export(&st, &ExportRequest::new(ExportFormat::Vtu), &mut vtu).expect("vtu");
        assert!(rep.bytes > 0 && String::from_utf8_lossy(&vtu).contains("VTKFile"));
    }

    #[test]
    fn errors_are_typed() {
        let st = state_with_hex(0);
        let mut sink = Vec::new();
        let req = ExportRequest::new(ExportFormat::Smsh);
        assert!(matches!(export(&st, &req, &mut sink), Err(FemError::NoMesh)));
        let st = state_with_hex(1);
        let bogus = EntityId::new(sm_core::EntityKind::Mesh, 99).expect("id");
        let req = ExportRequest { mesh: Some(bogus), ..req };
        assert!(matches!(export(&st, &req, &mut sink), Err(FemError::UnknownMesh(_))));
    }
}
