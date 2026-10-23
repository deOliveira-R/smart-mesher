//! Geometry handles and the read-only geometry store topology refers to.

use crate::defs::{CurveDef, SurfaceDef};
use crate::error::GeomError;
use sm_core::{EntityId, EntityKind};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

macro_rules! geom_id {
    ($(#[$m:meta])* $name:ident, $kind:expr) => {
        $(#[$m])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
        #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
        pub struct $name(EntityId);
        impl $name {
            /// Wrap an id; `None` if its kind is wrong.
            pub fn new(id: EntityId) -> Option<Self> {
                (id.kind() == $kind).then_some(Self(id))
            }
            /// The underlying entity id.
            pub fn id(self) -> EntityId {
                self.0
            }
        }
    };
}

geom_id!(
    /// Handle of a curve in a [`GeomStore`] (`EntityKind::Curve`).
    CurveId,
    EntityKind::Curve
);
geom_id!(
    /// Handle of a surface in a [`GeomStore`] (`EntityKind::Surface`).
    SurfaceId,
    EntityKind::Surface
);

/// Read-only geometry lookup. Topology holds only ids; this resolves them. Object-safe, so
/// meshers take `&dyn GeomStore`. Many topological entities may share one geometry entry
/// (shared handles), and a geometry entry never knows who uses it.
pub trait GeomStore: Send + Sync {
    /// Resolve a curve.
    fn curve(&self, id: CurveId) -> Option<&CurveDef>;
    /// Resolve a surface.
    fn surface(&self, id: SurfaceId) -> Option<&SurfaceDef>;
}

/// Reference in-memory store. Ids are `serial = insertion index`, so the same build order
/// yields the same ids.
#[derive(Clone, Debug, Default)]
pub struct SimpleGeomStore {
    curves: Vec<CurveDef>,
    surfaces: Vec<SurfaceDef>,
}

impl SimpleGeomStore {
    /// Empty store.
    pub fn new() -> Self {
        Self::default()
    }

    /// Insert a curve.
    pub fn add_curve(&mut self, c: CurveDef) -> Result<CurveId, GeomError> {
        let id = EntityId::new(EntityKind::Curve, self.curves.len() as u64).ok_or(GeomError::Invalid("id space exhausted"))?;
        self.curves.push(c);
        CurveId::new(id).ok_or(GeomError::Invalid("curve id kind"))
    }

    /// Insert a surface.
    pub fn add_surface(&mut self, s: SurfaceDef) -> Result<SurfaceId, GeomError> {
        let id = EntityId::new(EntityKind::Surface, self.surfaces.len() as u64).ok_or(GeomError::Invalid("id space exhausted"))?;
        self.surfaces.push(s);
        SurfaceId::new(id).ok_or(GeomError::Invalid("surface id kind"))
    }
}

impl GeomStore for SimpleGeomStore {
    fn curve(&self, id: CurveId) -> Option<&CurveDef> {
        self.curves.get(id.id().serial() as usize)
    }
    fn surface(&self, id: SurfaceId) -> Option<&SurfaceDef> {
        self.surfaces.get(id.id().serial() as usize)
    }
}
