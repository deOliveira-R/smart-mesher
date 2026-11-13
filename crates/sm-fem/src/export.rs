//! Export: pick a mesh, lower the model onto it, write `.smsh` or VTU.

use std::io::Write;

use sm_core::EntityId;
use sm_exodus::{ExodusOptions, write_exodus};
use sm_layer_api::{MeshLayer, ModelLayer};
use sm_mesh_api::{LevelMap, MeshCatalog};
use sm_meshio::select::Select;
use sm_meshio::foam::{FoamOptions, FoamPatchType, build_polymesh};
use sm_meshio::{LossReport, SmshWriter, VtuOptions, vtu_loss_report, write_vtu};
use sm_op_api::State;

use crate::error::FemError;
use crate::layer::FemLayer;
use crate::lower::{Lowered, SetSummary, lower};
use crate::select::{check_select, select_lowered};

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
    /// OpenFOAM only: names of face (side) sets written as `empty` patches (2D cases).
    pub empty_patches: Vec<String>,
    /// OpenFOAM only: names of face (side) sets written as `wedge` patches (axisymmetric cases).
    pub wedge_patches: Vec<String>,
    /// OpenFOAM only: split warped faces into planar triangles (off by default: it makes
    /// checkMesh's concave-cell check fail on warped hex meshes, see `FOAM_FORMAT.md` section 12).
    /// Planar meshes export byte for byte as without the pass.
    pub split_warped_faces: bool,
    /// Export only part of the mesh (`.smsh`, VTU and Exodus; see [`sm_meshio::select`]).
    /// `None` and a selection that keeps everything write the same bytes as before. A real
    /// selection writes a new mesh, so the level maps of the mesh layer are left out of a `.smsh`
    /// (`export.select_levelmap_dropped`). OpenFOAM refuses it.
    pub select: Option<Select>,
}

impl ExportRequest {
    /// A request for `format` with defaults for everything else.
    pub fn new(format: ExportFormat) -> Self {
        Self {
            format,
            mesh: None,
            provenance: None,
            split_small_determinant: true,
            empty_patches: Vec::new(),
            wedge_patches: Vec::new(),
            split_warped_faces: false,
            select: None,
        }
    }

    /// OpenFOAM: write these face (side) sets as `empty` patches. With empty or wedge patches
    /// the case is 2D and the small-determinant split does not apply.
    #[must_use]
    pub fn empty_patches(mut self, names: Vec<String>) -> Self {
        self.empty_patches = names;
        self
    }

    /// OpenFOAM: write these face (side) sets as `wedge` patches.
    #[must_use]
    pub fn wedge_patches(mut self, names: Vec<String>) -> Self {
        self.wedge_patches = names;
        self
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

    /// Export only the part of the mesh `select` names.
    #[must_use]
    pub fn select(mut self, select: Select) -> Self {
        self.select = Some(select);
        self
    }

    /// Switch the OpenFOAM warped-face split on or off (default off).
    #[must_use]
    pub fn split_warped_faces(mut self, on: bool) -> Self {
        self.split_warped_faces = on;
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

/// A lowered mesh ready to write: the mesh id of the state, the lowering (after the selection),
/// the notes of the selection and whether the selection cut the mesh.
struct Prepared {
    id: EntityId,
    lowered: Lowered,
    notes: Vec<String>,
    cut: bool,
}

/// The level maps a `.smsh` of `prep` carries, and the note when a cut mesh has to leave them
/// out (it is a new mesh: the maps name the whole one).
fn maps_for<'a>(state: &'a State, prep: &Prepared) -> (Vec<&'a LevelMap>, Option<String>) {
    let maps = level_maps_of(state, prep.id);
    if prep.cut && !maps.is_empty() {
        let note = format!(
            "export.select_levelmap_dropped: level maps of the whole mesh do not apply to the selected one x{}",
            maps.len()
        );
        return (Vec::new(), Some(note));
    }
    (maps, None)
}

/// Pick the mesh of `req`, lower the model onto it and apply the selection.
fn prepare(state: &State, req: &ExportRequest) -> Result<Prepared, FemError> {
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
    let Some(sel) = &req.select else {
        return Ok(Prepared { id, lowered, notes: Vec::new(), cut: false });
    };
    if req.format.is_multi_file() {
        return Err(FemError::Unsupported(format!(
            "`select` does not apply to {} (it keeps its patches and zones)",
            req.format.name()
        )));
    }
    let cut = !sel.is_all();
    if !cut {
        check_select(&lowered, sel)?;
        return Ok(Prepared { id, lowered, notes: Vec::new(), cut });
    }
    let (lowered, notes) = select_lowered(lowered, sel)?;
    Ok(Prepared { id, lowered, notes, cut })
}

/// The level maps of the mesh layer that name mesh `id` (as fine mesh first, then as coarse
/// mesh), in ascending order of the other mesh. A `.smsh` of that mesh carries them, so the
/// hierarchy reaches disk.
pub fn level_maps_of(state: &State, id: EntityId) -> Vec<&LevelMap> {
    let Some(layer) = state.layer::<MeshLayer>() else {
        return Vec::new();
    };
    let mut maps: Vec<&LevelMap> = layer.maps_from(id).into_iter().map(|(_, m)| m).collect();
    maps.extend(layer.maps_to(id).into_iter().map(|(_, m)| m));
    maps
}

/// The `.smsh` writer of a lowered mesh: provenance, deck records, declared element types and
/// the level maps `maps` (see [`level_maps_of`]; every map must be current for the mesh).
pub fn smsh_writer<'a>(
    lowered: &'a Lowered,
    provenance: Option<&str>,
    maps: &[&'a LevelMap],
) -> SmshWriter<'a> {
    let mut writer = SmshWriter::new((*lowered.mesh).as_read());
    if let Some(p) = provenance {
        writer = writer.provenance(p);
    }
    for d in &lowered.decks {
        writer = writer.deck(d.clone());
    }
    for (set, ty) in &lowered.block_types {
        writer = writer.element_type(set.clone(), ty.name());
    }
    for m in maps {
        writer = writer.level_map(m);
    }
    writer
}

/// Write `lowered` in a single-stream format; returns the loss notes.
fn write_stream<W: Write>(
    format: ExportFormat,
    provenance: Option<&String>,
    lowered: &Lowered,
    maps: &[&LevelMap],
    out: &mut W,
) -> Result<Vec<String>, FemError> {
    let mesh = (*lowered.mesh).as_read();
    let mut notes = Vec::new();
    match format {
        ExportFormat::Smsh => {
            smsh_writer(lowered, provenance.map(String::as_str), maps).write_to(out)?;
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
    let prep = prepare(state, req)?;
    let mut out = Counting { inner: w, n: 0 };
    let (maps, dropped) = maps_for(state, &prep);
    let mut notes = prep.notes.clone();
    if req.format == ExportFormat::Smsh {
        notes.extend(dropped);
    }
    notes.extend(write_stream(req.format, req.provenance.as_ref(), &prep.lowered, &maps, &mut out)?);
    out.flush()?;
    Ok(ExportReport {
        format: req.format,
        mesh: prep.id,
        bytes: out.n,
        sets: prep.lowered.sets.clone(),
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
    let prep = prepare(state, req)?;
    let (id, lowered) = (prep.id, &prep.lowered);
    let (files, notes) = match req.format {
        ExportFormat::OpenFoam => {
            let mut opts = FoamOptions::new()
                .decks(lowered.decks.clone())
                .split_small_determinant(req.split_small_determinant)
                .split_warped_faces(req.split_warped_faces);
            for n in &req.empty_patches {
                opts = opts.patch_type(n.clone(), FoamPatchType::Empty);
            }
            for n in &req.wedge_patches {
                opts = opts.patch_type(n.clone(), FoamPatchType::Wedge);
            }
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
            let (maps, dropped) = maps_for(state, &prep);
            let mut notes = prep.notes.clone();
            if f == ExportFormat::Smsh {
                notes.extend(dropped);
            }
            notes.extend(write_stream(f, req.provenance.as_ref(), lowered, &maps, &mut bytes)?);
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
        sets: prep.lowered.sets.clone(),
        notes,
    })
}
