//! Shape handles: how topology refers to geometry.
//!
//! Topology stores only `EntityId`s of kind `Curve` / `Surface`. A [`ShapeStore`] resolves them.
//! Many entities may share one handle (e.g. all faces cut from one plane), and a handle is never
//! reused for different geometry.

use crate::analytic_surf::AnalyticSurface;
use crate::GeomError;
use crate::csg::ImplicitSurface;
use crate::curve::ParametricCurve;
use crate::surface::ParametricSurface;
use sm_core::{EntityId, EntityKind};
use std::sync::Arc;

/// Resolves shape handles to geometry. Object-safe and thread-safe.
pub trait ShapeStore: Send + Sync {
    /// Curve for a `Curve` id.
    fn curve(&self, id: EntityId) -> Option<&dyn ParametricCurve>;
    /// Parametric view of a `Surface` id (absent for pure CSG surfaces without a parametrization).
    fn surface(&self, id: EntityId) -> Option<&dyn ParametricSurface>;
    /// Implicit (CSG) view of a `Surface` id, if it has one.
    fn implicit(&self, id: EntityId) -> Option<&dyn ImplicitSurface>;
}

#[derive(Clone, Debug)]
struct SurfaceEntry {
    param: Option<Arc<dyn ParametricSurface>>,
    implicit: Option<Arc<dyn ImplicitSurface>>,
}

/// Reference in-memory [`ShapeStore`]. Ids are dense serials in insertion order.
#[derive(Clone, Debug, Default)]
pub struct ShapeLibrary {
    curves: Vec<Arc<dyn ParametricCurve>>,
    surfaces: Vec<SurfaceEntry>,
}

impl ShapeLibrary {
    /// Empty library.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a curve; returns its handle.
    pub fn add_curve<C: ParametricCurve + 'static>(&mut self, c: C) -> Result<EntityId, GeomError> {
        let id = EntityId::new(EntityKind::Curve, self.curves.len() as u64).ok_or(GeomError::Degenerate("shape id space exhausted"))?;
        self.curves.push(Arc::new(c));
        Ok(id)
    }

    /// Add an analytic surface exposing both the parametric and the implicit view.
    pub fn add_analytic_surface(&mut self, s: AnalyticSurface) -> Result<EntityId, GeomError> {
        self.push_surface(SurfaceEntry { param: Some(Arc::new(s)), implicit: Some(Arc::new(s)) })
    }

    /// Add a parametric-only surface.
    pub fn add_parametric_surface(&mut self, s: Arc<dyn ParametricSurface>) -> Result<EntityId, GeomError> {
        self.push_surface(SurfaceEntry { param: Some(s), implicit: None })
    }

    /// Add an implicit-only (CSG) surface.
    pub fn add_implicit_surface(&mut self, s: Arc<dyn ImplicitSurface>) -> Result<EntityId, GeomError> {
        self.push_surface(SurfaceEntry { param: None, implicit: Some(s) })
    }

    fn push_surface(&mut self, e: SurfaceEntry) -> Result<EntityId, GeomError> {
        let id = EntityId::new(EntityKind::Surface, self.surfaces.len() as u64).ok_or(GeomError::Degenerate("shape id space exhausted"))?;
        self.surfaces.push(e);
        Ok(id)
    }

    fn idx(id: EntityId, kind: EntityKind) -> Option<usize> {
        (id.kind() == kind).then(|| id.serial() as usize)
    }
}

impl ShapeStore for ShapeLibrary {
    fn curve(&self, id: EntityId) -> Option<&dyn ParametricCurve> {
        self.curves.get(Self::idx(id, EntityKind::Curve)?).map(|c| c.as_ref())
    }
    fn surface(&self, id: EntityId) -> Option<&dyn ParametricSurface> {
        self.surfaces.get(Self::idx(id, EntityKind::Surface)?)?.param.as_deref()
    }
    fn implicit(&self, id: EntityId) -> Option<&dyn ImplicitSurface> {
        self.surfaces.get(Self::idx(id, EntityKind::Surface)?)?.implicit.as_deref()
    }
}
