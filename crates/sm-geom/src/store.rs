//! [`KernelStore`]: the performance-oriented [`GeomStore`].
//!
//! It keeps the serializable definitions (the source of truth, identical handles and semantics
//! to the reference `MemoryStore`) plus a *prepared* evaluator for every NURBS entry, so
//! `curve(id)` / `surface(id)` hand out the fast allocation-free evaluators with Bezier culling
//! caches. Analytic entries resolve to the contract's exact closed-form implementations.

use crate::nurbs::{PreparedCurve, PreparedSurface};
use sm_core::{EntityId, EntityKind};
use sm_geom_api::{CurveDef, GeomAppend, GeomError, GeomStore, GeomStoreMut, ImplicitDef, ImplicitSurface, MemoryStore, ParametricCurve, ParametricSurface, StoreData, SurfaceDef};

/// Geometry store with prepared NURBS evaluators. Deterministic: the same insertion order gives
/// the same handles, and all lookups are O(1) slot reads.
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize), serde(from = "StoreData", into = "StoreData"))]
pub struct KernelStore {
    curves: Vec<CurveDef>,
    curve_prep: Vec<Option<PreparedCurve>>,
    surfaces: Vec<SurfaceDef>,
    surface_prep: Vec<Option<PreparedSurface>>,
    implicits: Vec<Option<ImplicitDef>>,
}

fn prep_curve(d: &CurveDef) -> Result<Option<PreparedCurve>, GeomError> {
    match d {
        CurveDef::Nurbs(n) => PreparedCurve::new(n).map(Some),
        _ => Ok(None),
    }
}

fn prep_surface(d: &SurfaceDef) -> Result<Option<PreparedSurface>, GeomError> {
    match d {
        SurfaceDef::Nurbs(n) => PreparedSurface::new(n).map(Some),
        _ => Ok(None),
    }
}

impl PartialEq for KernelStore {
    fn eq(&self, o: &Self) -> bool {
        self.curves == o.curves && self.surfaces == o.surfaces
    }
}

impl KernelStore {
    /// An empty store.
    pub fn new() -> Self {
        Self::default()
    }

    /// Build from definitions (validates each; the first invalid definition is the error).
    pub fn from_data(d: StoreData) -> Result<Self, GeomError> {
        let mut s = Self::new();
        for c in d.curves {
            s.add_curve(c)?;
        }
        for sf in d.surfaces {
            s.add_surface(sf)?;
        }
        Ok(s)
    }

    /// Copy into the reference store (used by differential tests and exports).
    pub fn to_memory_store(&self) -> MemoryStore {
        MemoryStore::from(StoreData::new(self.curves.clone(), self.surfaces.clone()))
    }

    /// Drop every handle with serial `>= curves` / `>= surfaces` (rollback of recent additions;
    /// same semantics as `MemoryStore::truncate`).
    pub fn truncate(&mut self, curves: usize, surfaces: usize) {
        self.curves.truncate(curves);
        self.curve_prep.truncate(curves);
        self.surfaces.truncate(surfaces);
        self.surface_prep.truncate(surfaces);
        self.implicits.truncate(surfaces);
    }

    /// The prepared evaluator of a NURBS curve handle (also gives access to knot editing).
    pub fn prepared_curve(&self, id: EntityId) -> Option<&PreparedCurve> {
        Self::slot(id, EntityKind::Curve, self.curves.len()).and_then(|s| self.curve_prep[s].as_ref())
    }

    /// The prepared evaluator of a NURBS surface handle.
    pub fn prepared_surface(&self, id: EntityId) -> Option<&PreparedSurface> {
        Self::slot(id, EntityKind::Surface, self.surfaces.len()).and_then(|s| self.surface_prep[s].as_ref())
    }

    fn slot(id: EntityId, kind: EntityKind, len: usize) -> Option<usize> {
        if id.kind() != kind {
            return None;
        }
        usize::try_from(id.serial()).ok().filter(|s| *s < len)
    }
}

impl From<StoreData> for KernelStore {
    /// Infallible conversion for already-validated data; entries that fail validation are
    /// dropped from the prepared layer only (evaluation then falls back to the reference
    /// evaluator). Use [`KernelStore::from_data`] to validate.
    fn from(d: StoreData) -> Self {
        let curve_prep = d.curves.iter().map(|c| prep_curve(c).ok().flatten()).collect();
        let surface_prep = d.surfaces.iter().map(|s| prep_surface(s).ok().flatten()).collect();
        let implicits = d.surfaces.iter().map(SurfaceDef::implicit_def).collect();
        Self { curves: d.curves, curve_prep, surfaces: d.surfaces, surface_prep, implicits }
    }
}

impl From<KernelStore> for StoreData {
    fn from(s: KernelStore) -> Self {
        StoreData::new(s.curves, s.surfaces)
    }
}

impl GeomStore for KernelStore {
    fn curve(&self, id: EntityId) -> Option<&dyn ParametricCurve> {
        let s = Self::slot(id, EntityKind::Curve, self.curves.len())?;
        Some(match &self.curve_prep[s] {
            Some(p) => p,
            None => self.curves[s].as_curve(),
        })
    }
    fn surface(&self, id: EntityId) -> Option<&dyn ParametricSurface> {
        let s = Self::slot(id, EntityKind::Surface, self.surfaces.len())?;
        match &self.surface_prep[s] {
            Some(p) => Some(p),
            None => self.surfaces[s].as_surface(),
        }
    }
    fn implicit(&self, id: EntityId) -> Option<&dyn ImplicitSurface> {
        let s = Self::slot(id, EntityKind::Surface, self.surfaces.len())?;
        self.implicits[s].as_ref().map(|d| d as &dyn ImplicitSurface)
    }
    fn curve_def(&self, id: EntityId) -> Option<&CurveDef> {
        Self::slot(id, EntityKind::Curve, self.curves.len()).map(|s| &self.curves[s])
    }
    fn surface_def(&self, id: EntityId) -> Option<&SurfaceDef> {
        Self::slot(id, EntityKind::Surface, self.surfaces.len()).map(|s| &self.surfaces[s])
    }
    fn curve_count(&self) -> usize {
        self.curves.len()
    }
    fn surface_count(&self) -> usize {
        self.surfaces.len()
    }
}

impl GeomAppend for KernelStore {
    fn add_curve(&mut self, def: CurveDef) -> Result<EntityId, GeomError> {
        def.validate()?;
        let prep = prep_curve(&def)?;
        let id = EntityId::new(EntityKind::Curve, self.curves.len() as u64).ok_or(GeomError::Invalid("id space exhausted"))?;
        self.curves.push(def);
        self.curve_prep.push(prep);
        Ok(id)
    }
    fn add_surface(&mut self, def: SurfaceDef) -> Result<EntityId, GeomError> {
        def.validate()?;
        let prep = prep_surface(&def)?;
        let id = EntityId::new(EntityKind::Surface, self.surfaces.len() as u64).ok_or(GeomError::Invalid("id space exhausted"))?;
        self.implicits.push(def.implicit_def());
        self.surfaces.push(def);
        self.surface_prep.push(prep);
        Ok(id)
    }
}

impl GeomStoreMut for KernelStore {
    fn replace_curve(&mut self, id: EntityId, def: CurveDef) -> Result<CurveDef, GeomError> {
        def.validate()?;
        let s = Self::slot(id, EntityKind::Curve, self.curves.len()).ok_or(GeomError::Invalid("unknown curve handle"))?;
        let prep = prep_curve(&def)?;
        self.curve_prep[s] = prep;
        Ok(core::mem::replace(&mut self.curves[s], def))
    }
    fn replace_surface(&mut self, id: EntityId, def: SurfaceDef) -> Result<SurfaceDef, GeomError> {
        def.validate()?;
        let s = Self::slot(id, EntityKind::Surface, self.surfaces.len()).ok_or(GeomError::Invalid("unknown surface handle"))?;
        let prep = prep_surface(&def)?;
        self.implicits[s] = def.implicit_def();
        self.surface_prep[s] = prep;
        Ok(core::mem::replace(&mut self.surfaces[s], def))
    }
}
