//! The standard distribution: core layers, every plugin, the mesher service and the input store.

use std::cell::RefCell;
use std::path::Path;
use std::sync::{Arc, PoisonError, RwLock};

use sm_cadio::{CadioPlugin, DirInputStore, InputRef, InputStore, MemoryInputStore, StepInputs};
use sm_engine::WorkspaceBuilder;
use sm_layer_api::CoreLayers;
use sm_mesher::{MeshPlugin, MesherConfig};
use sm_model::ModelPlugin;
use sm_op_api::{Plugin, Registry, RegistryError, Services};
use sm_scheme_api::{SchemeDim, SchemeRegistry};
use sm_sizing::{AnglePicker, IiaSolver};
use sm_tess::TessPlugin;

use crate::error::StandardError;

/// The scheme registry of the standard distribution: every scheme crate registered (strict: a failure is an error), then the
/// strict auto lists `curve: [equal]`, `surface: [map]`, `volume: [map]`.
pub fn standard_schemes() -> Result<SchemeRegistry, StandardError> {
    let mut registry = SchemeRegistry::new();
    type Register = fn(&mut SchemeRegistry) -> sm_scheme_api::Result<()>;
    let crates: [(&'static str, Register); 9] = [
        ("sm-scheme-curve", sm_scheme_curve::register),
        ("sm-scheme-map", sm_scheme_map::register),
        ("sm-scheme-mapvol", sm_scheme_mapvol::register),
        ("sm-scheme-submap", sm_scheme_submap::register),
        ("sm-scheme-pave", sm_scheme_pave::register),
        ("sm-scheme-tri", sm_scheme_tri::register),
        ("sm-scheme-sweep", sm_scheme_sweep::register),
        ("sm-scheme-tet", sm_scheme_tet::register),
        ("sm-scheme-ogrid", sm_scheme_ogrid::register),
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
/// builder being opened (each builder owns its store; nothing is shared between sessions or
/// threads); a caller that opens a project with
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
    /// The project directory of the `open` call running on this thread, if any. It is set only
    /// for the duration of that synchronous call, so it never outlives it and never leaks to
    /// another builder, server or thread.
    static OPENING: RefCell<Option<std::path::PathBuf>> = const { RefCell::new(None) };
}

/// Run `f` (a builder's `open`) with `project` as the directory the builder's own input store
/// binds to while the engine installs its plugins, i.e. before the project replays.
pub(crate) fn with_opening<T>(project: &Path, f: impl FnOnce() -> T) -> T {
    struct Restore(Option<std::path::PathBuf>);
    impl Drop for Restore {
        fn drop(&mut self) {
            OPENING.with(|o| *o.borrow_mut() = self.0.take());
        }
    }
    let prev = OPENING.with(|o| o.borrow_mut().replace(project.to_owned()));
    let _restore = Restore(prev);
    f()
}

/// Registers nothing; when installed inside [`with_opening`] it binds its own store to the
/// project being opened. This is how a builder finds its store without any global registry.
struct InputsPlugin(Arc<ProjectInputs>);

impl Plugin for InputsPlugin {
    fn id(&self) -> &str {
        "sm-server.inputs"
    }

    fn register(&self, _registry: &mut Registry) -> Result<(), RegistryError> {
        if let Some(dir) = OPENING.with(|o| o.borrow().clone()) {
            self.0.bind(&dir);
        }
        Ok(())
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
    f(&sm_view::ViewPlugin);
    f(&sm_csgbrep::CsgbrepPlugin);
    f(&sm_modify::ModifyPlugin);
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
    let names = op_names(&core);
    let services = Services::new()
        .with_versioned("mesher", 1, mesher)
        .with(StepInputs(inputs.clone()))
        .with(InputsHandle(inputs.clone()))
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
        .plugin(sm_view::ViewPlugin)
        .plugin(sm_csgbrep::CsgbrepPlugin)
        .plugin(sm_modify::ModifyPlugin)
        .plugin(InputsPlugin(inputs))
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

#[cfg(test)]
mod opening_tests {
    use std::path::PathBuf;
    use std::sync::Mutex;
    use std::sync::mpsc::{Receiver, Sender, channel};
    use std::time::Duration;

    use sm_op_api::{Plugin, Registry, RegistryError};

    use super::*;
    use crate::Server;

    /// Meets the other open while `OPENING` is set, so two opens overlap for certain. Unlike a
    /// barrier, a partner that never comes fails the open after a timeout instead of hanging.
    struct Rendezvous(Sender<()>, Mutex<Receiver<()>>);

    impl Plugin for Rendezvous {
        fn id(&self) -> &str {
            "test.rendezvous"
        }
        fn register(&self, _: &mut Registry) -> Result<(), RegistryError> {
            // `RegistryError` has no variant for a test-local failure and a new one is an `sm-op-api`
            // ACR, so `BadOpName` stands in and carries the message (it is not a bad op name).
            self.0.send(()).map_err(|_| RegistryError::BadOpName("rendezvous: partner gone".into()))?;
            let rx = self.1.lock().unwrap_or_else(PoisonError::into_inner);
            rx.recv_timeout(Duration::from_secs(10)).map_err(|_| RegistryError::BadOpName("rendezvous: timed out".into()))
        }
    }

    /// A scratch directory removed on drop, also when the test panics.
    struct Scratch(PathBuf);

    impl std::ops::Deref for Scratch {
        type Target = Path;
        fn deref(&self) -> &Path {
            &self.0
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// `<target>/tmp`, found from the test executable (`<target>/<profile>/deps/<exe>`): unit
    /// tests do not get `CARGO_TARGET_TMPDIR`. Falls back to the system temp dir.
    fn target_tmp() -> PathBuf {
        let exe = std::env::current_exe().unwrap_or_default();
        match exe.ancestors().nth(3) {
            Some(target) if target.join("CACHEDIR.TAG").exists() => target.join("tmp"),
            _ => std::env::temp_dir(),
        }
    }

    fn scratch(tag: &str) -> Scratch {
        let p = target_tmp().join(format!("sm-server-opening-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&p);
        std::fs::create_dir_all(&p).expect("scratch dir");
        Scratch(p)
    }

    fn opening() -> Option<PathBuf> {
        OPENING.with(|o| o.borrow().clone())
    }

    fn files_under(dir: &Path) -> Vec<String> {
        let mut out = Vec::new();
        let mut stack = vec![dir.to_owned()];
        while let Some(d) = stack.pop() {
            for e in std::fs::read_dir(&d).into_iter().flatten().flatten() {
                let p = e.path();
                if p.is_dir() {
                    stack.push(p);
                } else {
                    out.push(p.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default());
                }
            }
        }
        out
    }

    /// Two threads open two projects at the same time; each stages into its own project only,
    /// and `OPENING` is clear again on each thread afterwards.
    #[test]
    fn concurrent_opens_bind_their_own_project() {
        let root = scratch("two");
        let dirs = [root.join("a"), root.join("b")];
        for (i, d) in dirs.iter().enumerate() {
            drop(Server::create(standard_with(Some("t".into()), Some("2026-10-24T00:00:00Z".into())), d, &format!("p{i}")).expect("create"));
        }
        let ((tx_a, rx_a), (tx_b, rx_b)) = (channel(), channel());
        let mut ends = [Some(Rendezvous(tx_a, Mutex::new(rx_b))), Some(Rendezvous(tx_b, Mutex::new(rx_a)))];
        let payloads: [&[u8]; 2] = [b"payload for project a", b"payload for project b"];
        let sha = std::thread::scope(|s| {
            let hs: Vec<_> = (0..2)
                .map(|i| {
                    let (dir, end) = (dirs[i].clone(), ends[i].take().expect("one end per thread"));
                    s.spawn(move || {
                        let b = standard_with(Some("t".into()), Some("2026-10-24T00:00:00Z".into())).plugin(end);
                        let server = Server::open(b, &dir).expect("open");
                        assert_eq!(opening(), None, "OPENING is clear after open on this thread");
                        server.stage_input(payloads[i]).expect("stage").sha256
                    })
                })
                .collect();
            hs.into_iter().map(|h| h.join().expect("thread")).collect::<Vec<_>>()
        });
        assert_ne!(sha[0], sha[1]);
        for i in 0..2 {
            let (own, other) = (files_under(&dirs[i].join("inputs")), files_under(&dirs[1 - i].join("inputs")));
            assert!(own.iter().any(|f| f.contains(&sha[i])), "project {i} holds its own blob: {own:?}");
            assert!(!other.iter().any(|f| f.contains(&sha[i])), "project {} does not hold project {i}'s blob: {other:?}", 1 - i);
        }
    }

    /// An `open` that fails (no such project) still leaves `OPENING` clear.
    #[test]
    fn a_failed_open_clears_opening() {
        let root = scratch("fail");
        let missing = root.join("no-such-project");
        assert!(Server::open(standard_with(None, None), &missing).is_err());
        assert_eq!(opening(), None);
        // The guard also restores an outer binding when opens nest.
        with_opening(&root, || {
            with_opening(&missing, || ());
            assert_eq!(opening().as_deref(), Some(&*root));
        });
        assert_eq!(opening(), None);
    }
}
