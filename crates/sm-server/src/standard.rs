//! The standard distribution: core layers, every plugin, the mesher service and the input store.

use std::cell::RefCell;
use std::path::Path;
use std::sync::{Arc, PoisonError, RwLock, Weak};

use sm_cadio::{CadioPlugin, DirInputStore, InputRef, InputStore, MemoryInputStore, StepInputs};
use sm_engine::WorkspaceBuilder;
use sm_layer_api::CoreLayers;
use sm_mesher::{MeshPlugin, MesherConfig};
use sm_model::ModelPlugin;
use sm_op_api::{Plugin, Registry, Services};
use sm_scheme_api::{SchemeDim, SchemeRegistry};
use sm_sizing::{AnglePicker, IiaSolver};
use sm_tess::TessPlugin;

use crate::error::StandardError;

/// The scheme registry of the standard distribution: every scheme crate registered (strict: a failure is an error), then the
/// strict auto lists `curve: [equal]`, `surface: [map]`, `volume: [map]`.
pub fn standard_schemes() -> Result<SchemeRegistry, StandardError> {
    let mut registry = SchemeRegistry::new();
    type Register = fn(&mut SchemeRegistry) -> sm_scheme_api::Result<()>;
    let crates: [(&'static str, Register); 8] = [
        ("sm-scheme-curve", sm_scheme_curve::register),
        ("sm-scheme-map", sm_scheme_map::register),
        ("sm-scheme-mapvol", sm_scheme_mapvol::register),
        ("sm-scheme-submap", sm_scheme_submap::register),
        ("sm-scheme-pave", sm_scheme_pave::register),
        ("sm-scheme-tri", sm_scheme_tri::register),
        ("sm-scheme-sweep", sm_scheme_sweep::register),
        ("sm-scheme-tet", sm_scheme_tet::register),
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
    distribution_with_core(CoreLayers::new(|| Box::new(sm_topo::TopoStore::new())), schemes)
}

/// The content-addressed store of staged inputs (STEP files), bound to a project directory.
///
/// Unbound (in-memory sessions, `reference_standard`, a fresh [`standard`]) it keeps blobs in a
/// [`MemoryInputStore`]. [`Server::open`](crate::Server::open), [`Server::create`](crate::Server::create)
/// and `session.open` bind it to `<project>/inputs/` before the project replays, so the project
/// directory alone carries everything a replay needs. Binding is done by this crate for the
/// builder it just built on the same thread; a caller that opens a project with
/// `standard().open(dir)` directly gets the in-memory store (a STEP import then replays only if
/// the blob was staged in this process).
#[derive(Debug, Default)]
pub struct ProjectInputs {
    dir: RwLock<Option<DirInputStore>>,
    mem: MemoryInputStore,
}

impl ProjectInputs {
    /// An unbound store.
    pub fn new() -> Self {
        Self::default()
    }

    /// Bind to `<project>/inputs/` (blobs there become visible; new blobs go there).
    pub fn bind(&self, project: &Path) {
        *self.dir.write().unwrap_or_else(PoisonError::into_inner) = Some(DirInputStore::new(project.join("inputs")));
    }

    /// Drop the project binding; new blobs go to memory again.
    pub fn unbind(&self) {
        *self.dir.write().unwrap_or_else(PoisonError::into_inner) = None;
    }

    /// The `inputs/` directory this store is bound to, if any.
    pub fn bound_dir(&self) -> Option<std::path::PathBuf> {
        self.dir.read().unwrap_or_else(PoisonError::into_inner).as_ref().map(|d| d.root().to_owned())
    }

    /// Stage `bytes` (in the project's `inputs/` when bound, else in memory).
    pub fn stage(&self, bytes: &[u8]) -> std::io::Result<InputRef> {
        match self.dir.read().unwrap_or_else(PoisonError::into_inner).as_ref() {
            Some(d) => d.put(bytes),
            None => Ok(self.mem.put(bytes)),
        }
    }
}

impl InputStore for ProjectInputs {
    fn get(&self, sha256: &str) -> Option<Arc<[u8]>> {
        let from_dir = self.dir.read().unwrap_or_else(PoisonError::into_inner).as_ref().and_then(|d| d.get(sha256));
        from_dir.or_else(|| self.mem.get(sha256))
    }
}

/// Service handle that lets the server stage bytes (the [`StepInputs`] service only reads).
#[derive(Clone, Debug)]
pub struct InputsHandle(pub Arc<ProjectInputs>);

thread_local! {
    /// The stores of the builders assembled on this thread and not yet consumed by a server, each
    /// awaiting its project. Weak, so a dropped builder leaves nothing behind.
    static PENDING: RefCell<Vec<Weak<ProjectInputs>>> = const { RefCell::new(Vec::new()) };
}

/// Forget every store awaiting a project (called before building a builder to open).
pub(crate) fn clear_pending_inputs() {
    PENDING.with(|p| p.borrow_mut().clear());
}

/// Bind every live pending store of this thread to `project` for a replay and return them. The
/// builder being opened is not identifiable from outside the engine, so its store is among them;
/// the caller must [`release_pending_inputs`] afterwards so the others stay unbound.
pub(crate) fn bind_pending_inputs(project: &Path) -> Vec<Arc<ProjectInputs>> {
    let stores: Vec<_> = PENDING.with(|p| std::mem::take(&mut *p.borrow_mut())).iter().filter_map(Weak::upgrade).collect();
    for s in &stores {
        s.bind(project);
    }
    stores
}

/// Unbind the stores returned by [`bind_pending_inputs`], except `keep` (the opened workspace's own).
pub(crate) fn release_pending_inputs(bound: &[Arc<ProjectInputs>], keep: Option<&Arc<ProjectInputs>>) {
    for s in bound {
        if !keep.is_some_and(|k| Arc::ptr_eq(k, s)) {
            s.unbind();
        }
    }
}

/// Every plugin of the distribution, in install order.
fn each_plugin(core: &CoreLayers, mut f: impl FnMut(&dyn Plugin)) {
    f(core);
    f(&ModelPlugin);
    f(&MeshPlugin);
    f(&sm_fem::FemPlugin);
    f(&TessPlugin);
    f(&sm_boolean::BooleanPlugin);
    f(&CadioPlugin);
    f(&sm_smooth::SmoothPlugin);
    f(&sm_poly::PolyPlugin);
    f(&sm_diag::DiagPlugin);
    f(&sm_rt::RtPlugin);
}

/// The names of every op the plugins register (INT-3). A plugin that fails to install here fails
/// the same way when the workspace builds, which reports it.
fn op_names(core: &CoreLayers) -> sm_diag::OpNames {
    let mut reg = Registry::new();
    each_plugin(core, |p| {
        let _ = reg.install(p);
    });
    sm_diag::OpNames::new(reg.op_names())
}

/// [`distribution`] over a caller-chosen topology core (e.g. the reference `CellComplex`).
pub fn distribution_with_core(core: CoreLayers, schemes: SchemeRegistry) -> WorkspaceBuilder {
    let mesher = MesherConfig::new(schemes)
        .with_solver(Box::new(IiaSolver::default()))
        .with_picker(Box::new(AnglePicker))
        .with_size_fields(Arc::new(sm_sizing::size_field))
        .with_auto_size(Arc::new(|view, targets, factor| {
            sm_sizing::auto_size(view, targets, factor).map_err(|e| e.to_string())
        }));
    let inputs = Arc::new(ProjectInputs::new());
    PENDING.with(|p| {
        let mut p = p.borrow_mut();
        p.retain(|w| w.strong_count() > 0);
        p.push(Arc::downgrade(&inputs));
    });
    let names = op_names(&core);
    let services = Services::new()
        .with_versioned("mesher", 1, mesher)
        .with(StepInputs(inputs.clone()))
        .with(InputsHandle(inputs))
        .with(names);
    WorkspaceBuilder::new()
        .plugin(core)
        .plugin(ModelPlugin)
        .plugin(MeshPlugin)
        .plugin(sm_fem::FemPlugin)
        .plugin(TessPlugin)
        .plugin(sm_boolean::BooleanPlugin)
        .plugin(CadioPlugin)
        .plugin(sm_smooth::SmoothPlugin)
        .plugin(sm_poly::PolyPlugin)
        .plugin(sm_diag::DiagPlugin)
        .plugin(sm_rt::RtPlugin)
        .services(services)
}

/// The standard distribution over the reference [`sm_topo_api::CellComplex`] core instead of
/// `TopoStore`, with the standard schemes. For differential tests (same ops, same hashes).
///
/// # Panics
/// If the distribution is misassembled, like [`standard`].
pub fn reference_standard() -> WorkspaceBuilder {
    let schemes = match standard_schemes() {
        Ok(s) => s,
        Err(e) => panic!("the standard distribution is misassembled: {e}"),
    };
    distribution_with_core(CoreLayers::new(|| Box::new(sm_topo_api::CellComplex::new())), schemes)
}

/// [`standard`] with an optional author and a fixed RFC 3339 timestamp (reproducible node hashes).
///
/// # Panics
/// If the distribution is misassembled, like [`standard`].
pub fn standard_with(author: Option<String>, fixed_time: Option<String>) -> WorkspaceBuilder {
    let mut b = standard();
    if let Some(a) = author {
        b = b.author(a);
    }
    if let Some(t) = fixed_time {
        b = b.clock(sm_engine::FixedClock(t));
    }
    b
}

/// The standard distribution, or why it cannot be assembled (a scheme crate failed to register,
/// or an auto list names a scheme nobody registered).
pub fn try_standard() -> Result<WorkspaceBuilder, StandardError> {
    Ok(distribution(standard_schemes()?))
}

/// The standard distribution: [`CoreLayers`] over `TopoStore`, the model, mesh, fem, tess,
/// boolean, cadio, smooth, poly, diag and rt plugins, and the services: `mesher` (IIA solver,
/// angle picker, size fields, auto size, the eight scheme crates, strict auto lists), the
/// [`ProjectInputs`] store behind `StepInputs`, and `sm_diag::OpNames`.
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
