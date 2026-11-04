//! Export: pick a mesh, lower the model onto it, write `.smsh` or VTU.

use std::io::Write;

use sm_core::EntityId;
use sm_exodus::{ExodusOptions, write_exodus};
use sm_layer_api::{MeshLayer, ModelLayer};
use sm_meshio::foam::{FoamOptions, build_polymesh};
use sm_meshio::{LossReport, SmshWriter, VtuOptions, vtu_loss_report, write_vtu};
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
    /// Exodus II (netCDF classic, 64-bit offset).
    Exodus,
    /// OpenFOAM `constant/polyMesh` (several files; see [`export_files`]).
    OpenFoam,
}

impl ExportFormat {
    /// The lower-case format name (`"smsh"`, `"vtu"`, `"exodus"`, `"openfoam"`).
    pub const fn name(self) -> &'static str {
        match self {
            Self::Smsh => "smsh",
            Self::Vtu => "vtu",
            Self::Exodus => "exodus",
            Self::OpenFoam => "openfoam",
        }
    }

    /// True when the format writes several files and only [`export_files`] can produce it.
    pub const fn is_multi_file(self) -> bool {
        matches!(self, Self::OpenFoam)
    }
}

/// What to export. Build with [`ExportRequest::new`] and the builder methods.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct ExportRequest {
    /// Output format.
    pub format: ExportFormat,
    /// The mesh to write. When absent the state must hold exactly one mesh.
    pub mesh: Option<EntityId>,
    /// Timeline hash to record as provenance (`.smsh` and Exodus).
    pub provenance: Option<String>,
    /// OpenFOAM only: split cells that fail checkMesh's cell-determinant check (ruling INT-8;
    /// 2D cases with `empty` or `wedge` patches are judged by OpenFOAM's 2D rule). On by default
    /// ([`ExportRequest::new`]); `false` writes the internal cells as they are.
    pub split_small_determinant: bool,
}

impl ExportRequest {
    /// A request for `format` with defaults for everything else.
    pub fn new(format: ExportFormat) -> Self {
        Self {
            format,
            mesh: None,
            provenance: None,
            split_small_determinant: true,
        }
    }

    /// Export the named mesh instead of the state's only one.
    #[must_use]
    pub fn mesh(mut self, id: EntityId) -> Self {
        self.mesh = Some(id);
        self
    }

    /// Record `hash` as provenance (`.smsh` and Exodus).
    #[must_use]
    pub fn provenance(mut self, hash: impl Into<String>) -> Self {
        self.provenance = Some(hash.into());
        self
    }

    /// Switch the OpenFOAM small-determinant split on or off (default on).
    #[must_use]
    pub fn split_small_determinant(mut self, on: bool) -> Self {
        self.split_small_determinant = on;
        self
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
    /// What the format could not carry, one line per loss item (`code: detail xcount`).
    pub notes: Vec<String>,
}

/// One file of a multi-file export.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExportFile {
    /// Path relative to the output root (`constant/polyMesh/points`).
    pub name: String,
    /// File contents.
    pub bytes: Vec<u8>,
}

/// What [`export_files`] produced.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExportBundle {
    /// The mesh written.
    pub mesh: EntityId,
    /// The files, sorted by name.
    pub files: Vec<ExportFile>,
    /// The sets lowered from blocks, sidesets and decks, in file order.
    pub sets: Vec<SetSummary>,
    /// What the format could not carry, one line per loss item (`code: detail xcount`).
    pub notes: Vec<String>,
}

fn notes_of(loss: &LossReport) -> Vec<String> {
    loss.items
        .iter()
        .map(|i| format!("{}: {} x{}", i.code, i.detail, i.count))
        .collect()
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

/// Select the mesh of `req` and lower the model onto it.
fn prepare(state: &State, req: &ExportRequest) -> Result<(EntityId, Lowered), FemError> {
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
    Ok((id, lowered))
}

/// Write `lowered` in a single-stream format; returns the loss notes.
fn write_stream<W: Write>(
    format: ExportFormat,
    provenance: Option<&String>,
    lowered: &Lowered,
    out: &mut W,
) -> Result<Vec<String>, FemError> {
    let mesh = (*lowered.mesh).as_read();
    let mut notes = Vec::new();
    match format {
        ExportFormat::Smsh => {
            let mut writer = SmshWriter::new(mesh);
            if let Some(p) = provenance {
                writer = writer.provenance(p.clone());
            }
            for d in &lowered.decks {
                writer = writer.deck(d.clone());
            }
            for (set, ty) in &lowered.block_types {
                writer = writer.element_type(set.clone(), ty.name());
            }
            writer.write_to(out)?;
        }
        ExportFormat::Vtu => {
            let opts = VtuOptions::default();
            notes = notes_of(&vtu_loss_report(mesh, &opts));
            write_vtu(mesh, &opts, out)?;
        }
        ExportFormat::Exodus => {
            let mut opts = ExodusOptions::default();
            if let Some(p) = provenance {
                opts = opts.provenance(p.clone());
            }
            let rep = write_exodus(mesh, &opts, out)?;
            if rep.skipped_sides > 0 {
                notes.push(format!(
                    "exodus.skipped_sides: side-set entries on inactive cells x{}",
                    rep.skipped_sides
                ));
            }
        }
        ExportFormat::OpenFoam => return Err(FemError::MultiFile(format.name())),
    }
    Ok(notes)
}

/// Write the requested mesh of `state` to `w`. A pure function of `(state, req)`: byte-identical
/// across runs and thread counts.
///
/// The mesh is lowered first ([`lower`]): volume colors become cell sets, face colors become
/// side sets and deck boundaries become face sets plus `.smsh` deck records. A state without a
/// `fem` layer lowers with no declared element types; a state without a `model` layer writes
/// the mesh unchanged. Declared block element types are written to the `.smsh` `element_types`
/// header. Exodus writes volume colors as blocks and side sets and deck faces as side sets.
///
/// With `req.mesh == None` the state must hold exactly one mesh.
///
/// # Errors
/// [`FemError::MultiFile`] for a multi-file format (use [`export_files`]),
/// [`FemError::NoMeshLayer`], [`FemError::NoMesh`], [`FemError::AmbiguousMesh`],
/// [`FemError::UnknownMesh`], [`FemError::StaleMesh`], the lowering errors, and writer or sink
/// failures.
pub fn export(
    state: &State,
    req: &ExportRequest,
    w: &mut dyn Write,
) -> Result<ExportReport, FemError> {
    if req.format.is_multi_file() {
        return Err(FemError::MultiFile(req.format.name()));
    }
    let (id, lowered) = prepare(state, req)?;
    let mut out = Counting { inner: w, n: 0 };
    let notes = write_stream(req.format, req.provenance.as_ref(), &lowered, &mut out)?;
    out.flush()?;
    Ok(ExportReport {
        format: req.format,
        mesh: id,
        bytes: out.n,
        sets: lowered.sets,
        notes,
    })
}

/// Write the requested mesh of `state` as a set of files, sorted by name. A pure function of
/// `(state, req)`.
///
/// OpenFOAM gives `constant/polyMesh/{points,faces,owner,neighbour,boundary}` plus `cellZones`
/// and `faceZones` only when non-empty. A single-stream format gives one file named
/// `mesh.smsh`, `mesh.vtu` or `mesh.exo`, the same bytes [`export`] writes.
///
/// # Errors
/// As [`export`], minus `MultiFile`, plus writer failures of the OpenFOAM path.
pub fn export_files(state: &State, req: &ExportRequest) -> Result<ExportBundle, FemError> {
    let (id, lowered) = prepare(state, req)?;
    let (files, notes) = match req.format {
        ExportFormat::OpenFoam => {
            let opts = FoamOptions::new()
                .decks(lowered.decks.clone())
                .split_small_determinant(req.split_small_determinant);
            let build = build_polymesh((*lowered.mesh).as_read(), &opts)?;
            let files = build
                .mesh
                .files()?
                .into_iter()
                .map(|f| ExportFile {
                    name: format!("constant/polyMesh/{}", f.name),
                    bytes: f.bytes,
                })
                .collect();
            (files, notes_of(&build.report.loss))
        }
        f => {
            let mut bytes = Vec::new();
            let notes = write_stream(f, req.provenance.as_ref(), &lowered, &mut bytes)?;
            let name = match f {
                ExportFormat::Smsh => "mesh.smsh",
                ExportFormat::Vtu => "mesh.vtu",
                _ => "mesh.exo",
            };
            (
                vec![ExportFile {
                    name: name.to_string(),
                    bytes,
                }],
                notes,
            )
        }
    };
    let mut files: Vec<ExportFile> = files;
    files.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(ExportBundle {
        mesh: id,
        files,
        sets: lowered.sets,
        notes,
    })
}
