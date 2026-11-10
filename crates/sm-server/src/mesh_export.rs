//! Mesh export formats that are not in `sm-fem`: DAGMC `.h5m`, Gmsh `.msh` 4.1 and the `.smrt`
//! tracking image. Each is a pure function of the state and the options.

use serde_json::{Map, Value};
use sm_core::EntityId;
use sm_fem::{FemError, FemLayer, Lowered, SetOrigin, lower};
use sm_layer_api::{MeshEntry, MeshLayer, ModelLayer};
use sm_meshio::dagmc::{DagmcError, DagmcOptions, Graveyard, export_h5m};
use sm_meshio::gmsh::{GmshError, GmshOptions, PolyPolicy, write_msh4};
use sm_meshio::report::LossReport;
use sm_model_api::BoundaryKind;
use sm_op_api::State;
use sm_protocol::{ExportFormat, ExportNote};
use sm_rt::MeshTrackerBuilder;
use sm_rt_api::{
    BoundarySpec, BuildInput, BuildOptions, ComplementPolicy, MeshInput, PartitionOptions, RecoveryPolicy, RegionRule, ResponseKind, ResponseMap, RtError, Tracker,
};

use crate::error::SessionError;
use crate::host::export_error;

/// The formats this module writes.
pub(crate) fn handles(format: ExportFormat) -> bool {
    matches!(format, ExportFormat::Dagmc | ExportFormat::Gmsh | ExportFormat::Smrt)
}

/// Bytes and loss notes (`code: detail xcount`) of one export.
pub(crate) struct Written {
    pub bytes: Vec<u8>,
    pub notes: Vec<ExportNote>,
}

pub(crate) fn notes_of(loss: &LossReport) -> Vec<ExportNote> {
    loss.items.iter().map(|i| ExportNote::parse(&format!("{}: {} x{}", i.code, i.detail, i.count))).collect()
}

/// The options object of `format`: absent or `null` is empty; anything else must be an object
/// whose keys are all in `allowed`.
fn options_map<'a>(name: &str, options: Option<&'a Value>, allowed: &[&str]) -> Result<Option<&'a Map<String, Value>>, SessionError> {
    let listing = || if allowed.is_empty() { "no options".to_owned() } else { format!("options: {}", allowed.join(", ")) };
    match options {
        None | Some(Value::Null) => Ok(None),
        Some(Value::Object(m)) => {
            if let Some(k) = m.keys().find(|k| !allowed.contains(&k.as_str())) {
                return Err(SessionError::bad_request(format!("unknown `{name}` option `{k}` ({})", listing())));
            }
            Ok(Some(m))
        }
        Some(_) => Err(SessionError::bad_request(format!("`options` of `{name}` must be an object ({})", listing()))),
    }
}

fn bool_option(name: &str, m: Option<&Map<String, Value>>, key: &str) -> Result<Option<bool>, SessionError> {
    match m.and_then(|m| m.get(key)) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::Bool(b)) => Ok(Some(*b)),
        Some(other) => Err(SessionError::bad_request(format!("`{name}` option `{key}` must be a boolean, found {other}"))),
    }
}

fn gmsh_options(options: Option<&Value>) -> Result<GmshOptions, SessionError> {
    let m = options_map("gmsh", options, &["binary", "polyhedra"])?;
    let mut o = GmshOptions::default();
    if let Some(b) = bool_option("gmsh", m, "binary")? {
        o = o.with_binary(b);
    }
    if let Some(v) = m.and_then(|m| m.get("polyhedra")).filter(|v| !v.is_null()) {
        let p: PolyPolicy = serde_json::from_value(v.clone())
            .map_err(|_| SessionError::bad_request(format!("`gmsh` option `polyhedra` must be \"error\", \"decompose\" or \"drop\", found {v}")))?;
        o = o.with_polyhedra(p);
    }
    Ok(o)
}

/// The `dagmc` options. `chord_tol` (a positive finite number) only applies to `source: model`
/// (`model` true); with a mesh it is refused rather than ignored.
pub(crate) fn dagmc_options(options: Option<&Value>, model: bool) -> Result<(DagmcOptions, Option<f64>), SessionError> {
    let m = options_map("dagmc", options, &["graveyard", "chord_tol"])?;
    let mut o = DagmcOptions::default();
    match m.and_then(|m| m.get("graveyard")) {
        None | Some(Value::Null | Value::Bool(false)) => {}
        Some(Value::Bool(true)) => o.graveyard = Some(Graveyard::default()),
        Some(v @ Value::Object(_)) => {
            let g: Graveyard = serde_json::from_value(v.clone())
                .map_err(|e| SessionError::bad_request(format!("`dagmc` option `graveyard`: {e} (keys: scale, thickness_frac)")))?;
            o.graveyard = Some(g);
        }
        Some(other) => {
            return Err(SessionError::bad_request(format!(
                "`dagmc` option `graveyard` must be a boolean or {{scale, thickness_frac}}, found {other}"
            )));
        }
    }
    let chord = match m.and_then(|m| m.get("chord_tol")) {
        None | Some(Value::Null) => None,
        Some(v) => {
            if !model {
                return Err(SessionError::bad_request("`dagmc` option `chord_tol` only applies to `source: model` (a mesh is not re-faceted)"));
            }
            match v.as_f64() {
                Some(x) if x.is_finite() && x > 0.0 => Some(x),
                _ => return Err(SessionError::bad_request(format!("`dagmc` option `chord_tol` must be a positive number, found {v}"))),
            }
        }
    };
    Ok((o, chord))
}

/// Pick the mesh as `sm_fem::export` does: the named one, else the only one.
fn select(state: &State, named: Option<EntityId>) -> Result<&MeshEntry, FemError> {
    let layer = state.layer::<MeshLayer>().ok_or(FemError::NoMeshLayer)?;
    let (id, entry) = match named {
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
    Ok(entry)
}

/// Write `format` (one of [`handles`]) of the selected mesh of `state`. Options are checked
/// before the mesh is touched, so a bad key is a request error whatever the state.
pub(crate) fn write(state: &State, format: ExportFormat, named: Option<EntityId>, options: Option<&Value>) -> Result<Written, SessionError> {
    match format {
        ExportFormat::Dagmc => {
            let (o, _) = dagmc_options(options, false)?;
            let lowered = lowered(state, named)?;
            let mut bytes = Vec::new();
            let rep = export_h5m((*lowered.mesh).as_read(), &o, &mut bytes).map_err(dagmc_error)?;
            Ok(Written { bytes, notes: notes_of(&rep.loss) })
        }
        ExportFormat::Gmsh => {
            let o = gmsh_options(options)?;
            let lowered = lowered(state, named)?;
            let mut bytes = Vec::new();
            let rep = write_msh4((*lowered.mesh).as_read(), &o, &mut bytes).map_err(gmsh_error)?;
            Ok(Written { bytes, notes: notes_of(&rep.loss) })
        }
        _ => {
            options_map("smrt", options, &[])?;
            smrt(state, named)
        }
    }
}

/// The selected mesh with the model's colors lowered onto it as sets (`mat:<set>` in DAGMC,
/// physical groups in Gmsh).
fn lowered(state: &State, named: Option<EntityId>) -> Result<Lowered, SessionError> {
    let entry = select(state, named).map_err(|e| export_error(&e, state, named))?;
    match state.layer::<ModelLayer>() {
        Some(model) => {
            let none = FemLayer::default();
            let fem = state.layer::<FemLayer>().unwrap_or(&none);
            let alloc = state.layer::<MeshLayer>().map(MeshLayer::alloc).ok_or_else(|| SessionError::internal("mesh layer vanished"))?;
            lower(model, entry, fem, alloc).map_err(|e| SessionError::from(&e))
        }
        None => Ok(Lowered::identity(entry)),
    }
}

/// The `.smrt` image of the mesh tracker for the selected mesh (default responses, vacuum
/// boundaries), built with the majorant partition ([`smrt_build_options`]). `rt.info` builds
/// the same tracker without the partition, so the structure hashes of the two differ.
fn smrt(state: &State, named: Option<EntityId>) -> Result<Written, SessionError> {
    let entry = select(state, named).map_err(|e| export_error(&e, state, named))?;
    let model = state
        .layer::<ModelLayer>()
        .ok_or_else(|| SessionError::new("no-model", "`smrt` export needs the model layer (region names come from the model)"))?;
    let spec = boundary_spec(model)?;
    let complement = ComplementPolicy::Outside;
    let (rules, lowered, notes) = smrt_regions(state, named);
    let host = match &lowered {
        Some(l) => (*l.mesh).as_read(),
        None => entry.mesh(),
    };
    let input = BuildInput::Mesh(MeshInput::new(host, &rules, &spec, &complement));
    let opts = smrt_build_options();
    let (tracker, _) = MeshTrackerBuilder.build_concrete(&input, &opts).map_err(rt_error)?;
    let image = tracker.export_cache().ok_or_else(|| SessionError::new("smrt-unavailable", "the tracker exports no cache image"))?;
    let mut bytes = Vec::new();
    sm_rt::write_smrt(&mut bytes, &image).map_err(|e| SessionError::new("io", format!("encoding the tracking image: {e}")))?;
    Ok(Written { bytes, notes })
}

/// Region rule of the `.smrt` (Q2): regions are named after the blocks (the material groups that
/// become `mat:<name>` in `.h5m`) when every cell lies in a block; the lowered mesh that
/// carries the block sets is returned with the rule. Otherwise (no blocks, a cell in no block, or a
/// lowering error) regions are the volume ids and a warning note says why (R10).
///
/// Invariant: `ByCellSets` resolves names with `set_by_name`, the first match. `sm_fem::lower`
/// rejects block names that collide with mesh sets and emits blocks before sidesets, so a block
/// name is unique among the lowered sets. The ops `fem.block_add` and `fem.sideset_add` also
/// refuse a name already in use, so no two lowered sets share a name and the lookup cannot mis-pick.
fn smrt_regions(state: &State, named: Option<EntityId>) -> (RegionRule, Option<Lowered>, Vec<ExportNote>) {
    let fallback = |why: String| {
        let detail = format!("regions are named by volume id ({why}); fix the model so every volume is in a block (fem.block_add)");
        (RegionRule::ByVolumeClassification, None, vec![ExportNote::warning("smrt-region-names", 1, detail)])
    };
    let lowered = match lowered(state, named) {
        Ok(l) => l,
        Err(e) => return fallback(format!("block lowering failed: {e}")),
    };
    let names: Vec<String> = lowered.sets.iter().filter(|s| s.origin == SetOrigin::Block).map(|s| s.name.clone()).collect();
    if names.is_empty() {
        return fallback("the model has no blocks".to_owned());
    }
    let mesh = (*lowered.mesh).as_read();
    // Each cell has one volume with one color, so a cell is in at most one block set: the
    // blocks cover the mesh exactly when their sizes sum to the cell count.
    let covered: usize = lowered.sets.iter().filter(|s| s.origin == SetOrigin::Block).map(|s| s.len).sum();
    let total = mesh.cell_count();
    if covered != total {
        return fallback(format!("{} of {total} cells are in no block", total - covered.min(total)));
    }
    (RegionRule::ByCellSets(names), Some(lowered), Vec::new())
}

/// Build options of the `.smrt` handover image. The majorant partition is on, so the file carries
/// the node sets and node bounds a transport code needs for delta tracking (`smrt_n_nodes`,
/// `smrt_node_set`); without it the opened tracker answers `SMRT_ERR_UNSUPPORTED` to those calls.
pub(crate) fn smrt_build_options() -> BuildOptions {
    let mut part = PartitionOptions::default();
    part.enabled = true;
    let mut opts = BuildOptions::default().with_partition(part);
    opts.recovery = RecoveryPolicy::Fail;
    opts
}

/// Boundary conditions as `rt.info` takes them with no response bindings: every response record
/// is vacuum.
fn boundary_spec(model: &ModelLayer) -> Result<BoundarySpec, SessionError> {
    let m = model.model();
    let mut responses = ResponseMap::new();
    let mut spec = BoundarySpec::default();
    for row in m.boundaries() {
        if matches!(row.kind, BoundaryKind::Response) {
            responses.entry(row.id).or_insert(ResponseKind::Vacuum);
        }
        for (face, fb) in m.boundary_faces(row.id) {
            spec.add_model_face(face, fb, &row.kind, &responses)
                .map_err(|i| SessionError::new("rt-build", format!("{}: {}", i.code, i.message)))?;
        }
    }
    Ok(spec)
}

fn rt_error(e: RtError) -> SessionError {
    match e {
        RtError::Build(issues) => {
            let first = issues.first().map(|i| format!("{}: {}", i.code, i.message)).unwrap_or_default();
            SessionError::new("rt-build", format!("tracker input has {} issue(s); first: {first} (run rt.check)", issues.len()))
        }
        other => SessionError::new("rt-build", other.to_string()),
    }
}

fn gmsh_error(e: GmshError) -> SessionError {
    match &e {
        GmshError::PolyhedraUnsupported { count, hint } => {
            SessionError::new("polyhedra-unsupported", format!("{e}; set the `polyhedra` option to \"decompose\" or \"drop\""))
                .with_data(serde_json::json!({ "count": count, "hint": hint }))
        }
        GmshError::NonconformingDecomposition { .. } => SessionError::new("nonconforming-decomposition", e.to_string()),
        GmshError::BadLabel { .. } => SessionError::new("bad-set-label", e.to_string()),
        GmshError::BadName(_) => SessionError::new("bad-set-name", e.to_string()),
        GmshError::Unsupported(_) => SessionError::new("unsupported", e.to_string()),
        GmshError::Mesh(_) => SessionError::new("mesh", e.to_string()),
        GmshError::Io(_) => SessionError::new("io", e.to_string()),
        _ => SessionError::new("export-failed", e.to_string()),
    }
}

pub(crate) fn dagmc_error(e: DagmcError) -> SessionError {
    match &e {
        DagmcError::OpenSurface { volume, edge, at } => {
            SessionError::new("open-surface", e.to_string()).with_data(serde_json::json!({ "volume": volume, "edge": edge, "at": at }))
        }
        DagmcError::NameTooLong { name } => SessionError::new("name-too-long", e.to_string()).with_data(serde_json::json!({ "name": name })),
        DagmcError::Unclassified { .. } => SessionError::new("unclassified-face", e.to_string()),
        DagmcError::InconsistentSense { .. } => SessionError::new("inconsistent-sense", e.to_string()),
        DagmcError::NonManifoldSurface { .. } => SessionError::new("non-manifold-surface", e.to_string()),
        DagmcError::Unsupported(_) => SessionError::new("unsupported", e.to_string()),
        DagmcError::Mesh(_) => SessionError::new("mesh", e.to_string()),
        DagmcError::Io(_) => SessionError::new("io", e.to_string()),
        _ => SessionError::new("export-failed", e.to_string()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn option_keys_are_checked_per_format() {
        assert!(gmsh_options(Some(&json!({"binary": true, "polyhedra": "drop"}))).is_ok());
        assert!(gmsh_options(Some(&json!({"graveyard": true}))).is_err());
        assert!(dagmc_options(Some(&json!({"binary": true})), false).is_err());
        assert!(dagmc_options(Some(&json!({"graveyard": {"scale": 1.2}})), false).expect("object").0.graveyard.is_some());
        assert!(dagmc_options(Some(&json!({"graveyard": false})), false).expect("false").0.graveyard.is_none());
        assert!(gmsh_options(Some(&json!({"binary": "yes"}))).is_err());
        assert!(gmsh_options(Some(&json!({"polyhedra": "fix"}))).is_err());
        assert_eq!(dagmc_options(Some(&json!({"chord_tol": 0.25})), true).expect("model").1, Some(0.25));
        assert!(dagmc_options(Some(&json!({"chord_tol": 0.25})), false).is_err(), "a mesh is not re-faceted");
        assert!(dagmc_options(Some(&json!({"chord_tol": 0.0})), true).is_err());
        assert!(dagmc_options(Some(&json!({"chord_tol": "fine"})), true).is_err());
        assert!(options_map("smrt", Some(&json!({"x": 1})), &[]).is_err());
    }
}
