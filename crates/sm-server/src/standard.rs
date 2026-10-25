//! The standard distribution: core layers, the four plugins and the mesher service.

use std::sync::Arc;

use sm_engine::WorkspaceBuilder;
use sm_layer_api::CoreLayers;
use sm_mesher::{MeshPlugin, MesherConfig};
use sm_model::ModelPlugin;
use sm_op_api::Services;
use sm_scheme_api::{SchemeDim, SchemeRegistry};
use sm_sizing::{AnglePicker, IiaSolver};
use sm_tess::TessPlugin;

use crate::error::StandardError;

/// The scheme registry of the standard distribution: every scheme crate registered, then the
/// strict auto lists `curve: [equal]`, `surface: [map]`, `volume: [map]`.
pub fn standard_schemes() -> Result<SchemeRegistry, StandardError> {
    let mut registry = SchemeRegistry::new();
    type Register = fn(&mut SchemeRegistry) -> sm_scheme_api::Result<()>;
    let crates: [(&'static str, Register); 3] = [
        ("sm-scheme-curve", sm_scheme_curve::register),
        ("sm-scheme-map", sm_scheme_map::register),
        ("sm-scheme-mapvol", sm_scheme_mapvol::register),
    ];
    for (crate_name, register) in crates {
        register(&mut registry).map_err(|e| StandardError::Register { crate_name, message: e.to_string() })?;
    }
    for (dim, label, scheme) in [
        (SchemeDim::Curve, "curve", "equal"),
        (SchemeDim::Surface, "surface", "map"),
        (SchemeDim::Volume, "volume", "map"),
    ] {
        if registry.get(dim, scheme).is_none() {
            return Err(StandardError::MissingScheme { dim: label, scheme });
        }
        registry
            .set_auto(dim, &[scheme])
            .map_err(|e| StandardError::Register { crate_name: "auto list", message: e.to_string() })?;
    }
    Ok(registry)
}

/// The standard plugins and services around a given scheme registry. [`try_standard`] passes
/// [`standard_schemes`]; tests pass the stub registry.
pub fn distribution(schemes: SchemeRegistry) -> WorkspaceBuilder {
    let mesher = MesherConfig::new(schemes)
        .with_solver(Box::new(IiaSolver::default()))
        .with_picker(Box::new(AnglePicker))
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

/// The standard distribution, or why it cannot be assembled (a scheme crate failed to register,
/// or an auto list names a scheme nobody registered).
pub fn try_standard() -> Result<WorkspaceBuilder, StandardError> {
    Ok(distribution(standard_schemes()?))
}

/// The standard distribution: [`CoreLayers`] over `TopoStore`, `ModelPlugin`, `MeshPlugin`,
/// `FemPlugin`, `TessPlugin`, and the `mesher` service (IIA solver, angle picker, size fields,
/// auto size, the three scheme crates, strict auto lists).
///
/// # Panics
/// If the distribution is misassembled, i.e. [`try_standard`] fails. Scheme registration errors
/// are never discarded.
pub fn standard() -> WorkspaceBuilder {
    match try_standard() {
        Ok(b) => b,
        Err(e) => panic!("the standard distribution is misassembled: {e}"),
    }
}
