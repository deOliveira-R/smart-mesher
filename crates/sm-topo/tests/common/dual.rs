//! `Dual`: the reference `CellComplex` and `TopoStore` behind one `TopologyEdit`. Every edit runs
//! on both and the results must be equal; at every `end_op` / `abort_op` (and when dropped outside
//! an operation) the complete observable state is compared with `same_state`. Reads come from
//! the reference. This lets the copied reference scenarios run unchanged in lock step.
#![allow(dead_code, clippy::too_many_arguments)]

use super::same_state;
use smallvec::SmallVec;
use sm_core::{EntityId, EntityKind, Point3};
use sm_geom_api::{CurveDef, GeomAppend, GeomStore, MemoryStore, ModelTolerance, PCurve, SurfaceDef};
use sm_topo::TopoStore;
use sm_topo_api::*;

/// Both implementations in lock step (see the module docs).
#[derive(Clone)]
pub struct Dual {
    pub a: CellComplex,
    pub b: TopoStore,
    open: bool,
    /// Geometry appended through `shapes_mut`, replayed on both at the next call.
    pend: Option<MemoryStore>,
}

fn same<T: PartialEq + core::fmt::Debug>(what: &str, a: T, b: T) -> T {
    assert_eq!(a, b, "{what}: reference vs sm-topo");
    a
}

impl Dual {
    pub fn new() -> Self {
        let mut b = TopoStore::new();
        b.set_paranoid(true);
        b.set_row_thresholds(6, 6);
        Self { a: CellComplex::new(), b, open: false, pend: None }
    }

    pub fn with_tolerance(tol: ModelTolerance) -> Self {
        let mut b = TopoStore::with_tolerance(tol);
        b.set_paranoid(true);
        b.set_row_thresholds(6, 6);
        Self { a: CellComplex::with_tolerance(tol), b, open: false, pend: None }
    }

    /// Replay geometry appended through `shapes_mut` onto both stores.
    fn flush(&mut self) {
        let Some(p) = self.pend.take() else { return };
        for n in self.a.shapes().curve_count()..p.curve_count() {
            let def = p.curve_def(EntityId::new(EntityKind::Curve, n as u64).expect("curve id")).expect("pending curve").clone();
            let ia = self.a.shapes_mut().expect("open op").add_curve(def.clone());
            let ib = self.b.shapes_mut().expect("open op").add_curve(def);
            assert_eq!(ia, ib, "curve handles");
        }
        for n in self.a.shapes().surface_count()..p.surface_count() {
            let def = p.surface_def(EntityId::new(EntityKind::Surface, n as u64).expect("surface id")).expect("pending surface").clone();
            let ia = self.a.shapes_mut().expect("open op").add_surface(def.clone());
            let ib = self.b.shapes_mut().expect("open op").add_surface(def);
            assert_eq!(ia, ib, "surface handles");
        }
    }

    /// The open operation's id (equal on both).
    pub fn open_op(&self) -> Option<OpId> {
        same("open_op", self.a.open_op(), self.b.open_op())
    }

    pub fn compare(&self, ctx: &str) {
        same_state(&self.a, &self.b, ctx);
        assert_eq!(self.b.validate_par(), validate(&self.b), "{ctx}: validate_par vs validate");
    }
}

impl Drop for Dual {
    fn drop(&mut self) {
        if !std::thread::panicking() && !self.open {
            self.compare("final state");
        }
    }
}

impl TopologyView for Dual {
    fn tolerance(&self) -> &ModelTolerance {
        self.a.tolerance()
    }
    fn shapes(&self) -> &dyn GeomStore {
        match &self.pend {
            Some(p) => p,
            None => self.a.shapes(),
        }
    }
    fn dim_of(&self, id: EntityId) -> Option<Dim> {
        self.a.dim_of(id)
    }
    fn state(&self, id: EntityId) -> Option<EntityState> {
        self.a.state(id)
    }
    fn cells(&self, dim: Dim) -> &[EntityId] {
        self.a.cells(dim)
    }
    fn boundary(&self, id: EntityId) -> &[Incidence] {
        self.a.boundary(id)
    }
    fn coboundary(&self, id: EntityId) -> &[Incidence] {
        self.a.coboundary(id)
    }
    fn carrier(&self, id: EntityId) -> &Carrier {
        self.a.carrier(id)
    }
    fn flags(&self, id: EntityId) -> CellFlags {
        self.a.flags(id)
    }
    fn pcurve(&self, k: CoedgeKey) -> Option<&PCurve> {
        self.a.pcurve(k)
    }
    fn shell_kind(&self, volume: EntityId, ring: u16) -> Option<ShellKind> {
        self.a.shell_kind(volume, ring)
    }
    fn radial_uses(&self, edge: EntityId) -> &[EdgeUse] {
        self.a.radial_uses(edge)
    }
    fn tolerance_override(&self, id: EntityId) -> Option<f64> {
        self.a.tolerance_override(id)
    }
    fn coincidences(&self) -> &[FacePair] {
        self.a.coincidences()
    }
    fn coincident_faces(&self, f: EntityId) -> &[FacePair] {
        self.a.coincident_faces(f)
    }
    fn lineage(&self) -> &LineageLog {
        self.a.lineage()
    }
    fn born(&self, id: EntityId) -> Option<OpId> {
        self.a.born(id)
    }
    fn revision(&self) -> OpId {
        self.a.revision()
    }
    fn volume_faces(&self, v: EntityId) -> &[EntityId] {
        self.a.volume_faces(v)
    }
    fn face_edges(&self, f: EntityId) -> &[EntityId] {
        self.a.face_edges(f)
    }
}

/// An edit that takes `Copy` arguments and returns a comparable result.
macro_rules! both {
    ($($name:ident($($arg:ident: $ty:ty),*) -> $ret:ty;)*) => {$(
        fn $name(&mut self $(, $arg: $ty)*) -> $ret {
            self.flush();
            let ra = self.a.$name($($arg),*);
            let rb = self.b.$name($($arg),*);
            same(stringify!($name), ra, rb)
        }
    )*};
}

impl TopologyEdit for Dual {
    fn begin_op(&mut self, op: OpId) -> Result<(), TopoError> {
        self.flush();
        let r = same("begin_op", self.a.begin_op(op), self.b.begin_op(op));
        self.open = r.is_ok() || self.open;
        r
    }

    fn end_op(&mut self) -> Result<OpSummary, TopoError> {
        self.flush();
        let ra = self.a.end_op();
        let rb = self.b.end_op();
        let r = same("end_op", ra, rb);
        // Success commits; failure rolls the operation back. Either way it is closed.
        if !matches!(r, Err(TopoError::NoOpenOp)) {
            self.open = false;
        }
        self.compare("after end_op");
        r
    }

    fn abort_op(&mut self) {
        self.flush();
        self.a.abort_op();
        self.b.abort_op();
        self.open = false;
        self.compare("after abort_op");
    }

    fn shapes_mut(&mut self) -> Result<&mut dyn GeomAppend, TopoError> {
        self.flush();
        same("shapes_mut", self.a.shapes_mut().map(|_| ()), self.b.shapes_mut().map(|_| ()))?;
        let mut m = MemoryStore::default();
        for n in 0..self.a.shapes().curve_count() {
            let def = EntityId::new(EntityKind::Curve, n as u64).and_then(|id| self.a.shapes().curve_def(id)).cloned();
            m.add_curve(def.ok_or(TopoError::InvalidParameter("curve def"))?)?;
        }
        for n in 0..self.a.shapes().surface_count() {
            let def = EntityId::new(EntityKind::Surface, n as u64).and_then(|id| self.a.shapes().surface_def(id)).cloned();
            m.add_surface(def.ok_or(TopoError::InvalidParameter("surface def"))?)?;
        }
        Ok(self.pend.insert(m))
    }

    both! {
        set_model_tolerance(tol: ModelTolerance) -> Result<(), TopoError>;
        make_vertex(p: Point3) -> Result<EntityId, TopoError>;
        make_edge(start: EntityId, end: EntityId, curve: Option<CurveCarrier>) -> Result<EntityId, TopoError>;
        make_face(loops: &[&[CoedgeSpec]], surface: Option<SurfaceCarrier>) -> Result<EntityId, TopoError>;
        make_volume(shells: &[ShellSpec<'_>]) -> Result<EntityId, TopoError>;
        make_body(members: &[EntityId]) -> Result<EntityId, TopoError>;
        add_shell(volume: EntityId, shell: ShellSpec<'_>) -> Result<u16, TopoError>;
        remove_shell(volume: EntityId, ring: u16) -> Result<(), TopoError>;
        add_face_use(volume: EntityId, ring: u16, use_: FaceUseSpec) -> Result<(), TopoError>;
        remove_face_use(volume: EntityId, face: EntityId, side: Side) -> Result<(), TopoError>;
        embed(parent: EntityId, child: EntityId) -> Result<(), TopoError>;
        unembed(parent: EntityId, child: EntityId) -> Result<(), TopoError>;
        split_edge(edge: EntityId, at: EntityId, param: f64) -> Result<[EntityId; 2], TopoError>;
        split_face(face: EntityId, cut: FaceCut<'_>, inner: InnerLoopAssign<'_>) -> Result<[EntityId; 2], TopoError>;
        split_volume(volume: EntityId, cut: &[EntityId], assign: ShellAssign<'_>) -> Result<[EntityId; 2], TopoError>;
        bridge_loops(face: EntityId, edge: EntityId) -> Result<(), TopoError>;
        make_inner_loop(face: EntityId, lp: &[CoedgeSpec]) -> Result<u16, TopoError>;
        kill_inner_loop(face: EntityId, ring: u16) -> Result<(), TopoError>;
        merge_vertices(keep: EntityId, drop: EntityId, max_gap: Option<f64>) -> Result<(), TopoError>;
        merge_edges(keep: EntityId, drop: EntityId, relative: Sense) -> Result<(), TopoError>;
        merge_faces(keep: EntityId, drop: EntityId, relative: Sense) -> Result<(), TopoError>;
        join_edges(first: EntityId, second: EntityId) -> Result<EntityId, TopoError>;
        join_faces(keep: EntityId, drop: EntityId) -> Result<EntityId, TopoError>;
        unmerge_vertex(v: EntityId, edges: &[EntityId]) -> Result<EntityId, TopoError>;
        unmerge_edge(edge: EntityId, faces: &[EntityId]) -> Result<EntityId, TopoError>;
        unmerge_face(face: EntityId, side: Side) -> Result<EntityId, TopoError>;
        split_degenerate_edge(edge: EntityId, param: f64) -> Result<[EntityId; 2], TopoError>;
        scale_tolerances(cells: &[EntityId], factor: f64) -> Result<(), TopoError>;
        reverse_face(face: EntityId) -> Result<(), TopoError>;
        kill(cell: EntityId) -> Result<(), TopoError>;
        kill_cascade(cell: EntityId) -> Result<SmallVec<[EntityId; 16]>, TopoError>;
        set_vertex_point(v: EntityId, p: Point3) -> Result<(), TopoError>;
        raise_tolerance(cell: EntityId, tol: f64) -> Result<(), TopoError>;
        set_flags(cell: EntityId, flags: CellFlags) -> Result<(), TopoError>;
        record_coincidence(a: EntityId, b: EntityId, status: Coincidence) -> Result<(), TopoError>;
        record_split(parent: EntityId, children: &[EntityId]) -> Result<(), TopoError>;
        record_merged(drop: EntityId, into: EntityId, gap: f64) -> Result<(), TopoError>;
        record_generated(cell: EntityId, from_surface: EntityId) -> Result<(), TopoError>;
        record_created(cell: EntityId) -> Result<(), TopoError>;
    }

    fn set_carrier(&mut self, cell: EntityId, carrier: Carrier) -> Result<(), TopoError> {
        self.flush();
        let ra = self.a.set_carrier(cell, carrier.clone());
        let rb = self.b.set_carrier(cell, carrier);
        same("set_carrier", ra, rb)
    }

    fn set_pcurve(&mut self, key: CoedgeKey, pcurve: Option<PCurve>) -> Result<(), TopoError> {
        self.flush();
        let ra = self.a.set_pcurve(key, pcurve.clone());
        let rb = self.b.set_pcurve(key, pcurve);
        same("set_pcurve", ra, rb)
    }

    fn replace_curve(&mut self, id: EntityId, def: CurveDef) -> Result<(), TopoError> {
        self.flush();
        let ra = self.a.replace_curve(id, def.clone());
        let rb = self.b.replace_curve(id, def);
        same("replace_curve", ra, rb)
    }

    fn replace_surface(&mut self, id: EntityId, def: SurfaceDef) -> Result<(), TopoError> {
        self.flush();
        let ra = self.a.replace_surface(id, def.clone());
        let rb = self.b.replace_surface(id, def);
        same("replace_surface", ra, rb)
    }
}
