//! Export: pick a mesh, lower the model onto it, write `.smsh` or VTU.

use std::io::Write;

use sm_core::EntityId;
use sm_layer_api::{MeshLayer, ModelLayer};
use sm_meshio::{SmshWriter, VtuOptions, write_vtu};
use sm_op_api::State;

use crate::error::FemError;
use crate::layer::FemLayer;
use crate::lower::{Lowered, SetSummary, lower};

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
    /// The mesh to write. When absent the state must hold exactly one mesh.
    pub mesh: Option<EntityId>,
    /// Timeline hash to record as provenance (`.smsh` only).
    pub provenance: Option<String>,
}

impl ExportRequest {
    /// A request for `format` with defaults for everything else.
    pub fn new(format: ExportFormat) -> Self {
        Self {
            format,
            mesh: None,
            provenance: None,
        }
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
    /// The sets lowered from blocks, sidesets and decks, in file order.
    pub sets: Vec<SetSummary>,
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

/// Write the requested mesh of `state` to `w`. A pure function of `(state, req)`: byte-identical
/// across runs and thread counts.
///
/// The mesh is lowered first ([`lower`]): volume colors become cell sets, face colors become
/// side sets and deck boundaries become face sets plus `.smsh` deck records. A state without a
/// `fem` layer lowers with no declared element types; a state without a `model` layer writes
/// the mesh unchanged.
///
/// With `req.mesh == None` the state must hold exactly one mesh.
///
/// # Errors
/// [`FemError::NoMeshLayer`], [`FemError::NoMesh`], [`FemError::AmbiguousMesh`],
/// [`FemError::UnknownMesh`], [`FemError::StaleMesh`], the lowering errors, and writer or sink
/// failures.
pub fn export(
    state: &State,
    req: &ExportRequest,
    w: &mut dyn Write,
) -> Result<ExportReport, FemError> {
    let layer = state.layer::<MeshLayer>().ok_or(FemError::NoMeshLayer)?;
    let (id, entry) = match req.mesh {
        Some(id) => (id, layer.get(id).ok_or(FemError::UnknownMesh(id))?),
        None => {
            let mut it = layer.meshes();
            let first = it.next().ok_or(FemError::NoMesh)?;
            if it.next().is_some() {
                return Err(FemError::AmbiguousMesh(layer.len()));
            }
            first
        }
    };
    if entry.stale {
        return Err(FemError::StaleMesh(id));
    }
    let lowered = match state.layer::<ModelLayer>() {
        Some(model) => {
            let none = FemLayer::default();
            let fem = state.layer::<FemLayer>().unwrap_or(&none);
            lower(model, entry, fem, layer.alloc())?
        }
        None => Lowered::identity(entry),
    };
    let mesh = (*lowered.mesh).as_read();
    let mut out = Counting { inner: w, n: 0 };
    match req.format {
        ExportFormat::Smsh => {
            let mut writer = SmshWriter::new(mesh);
            if let Some(p) = &req.provenance {
                writer = writer.provenance(p.clone());
            }
            for d in &lowered.decks {
                writer = writer.deck(d.clone());
            }
            writer.write_to(&mut out)?;
        }
        ExportFormat::Vtu => write_vtu(mesh, &VtuOptions::default(), &mut out)?,
    }
    out.flush()?;
    Ok(ExportReport {
        format: req.format,
        mesh: id,
        bytes: out.n,
        sets: lowered.sets,
    })
}
