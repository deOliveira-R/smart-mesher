//! Geometry by handle: closed serializable definitions and the store that resolves them.
//!
//! Topology never owns shape. It holds `EntityId` handles (kinds `Curve` / `Surface`) and asks a
//! [`GeomStore`] to resolve them; many faces can share one surface. [`CurveDef`] and
//! [`SurfaceDef`] are closed enums, so every stored shape serializes and clones; the object-safe
//! traits are implemented by delegation. [`SurfaceDef::Implicit`] stores a surface that has no
//! parametrization (a general quadric, the torus quartic): `surface(id)` is `None` for it and
//! `implicit(id)` is `Some`.

use crate::implicit::ImplicitSurface;
use crate::{Circle, Cone, Cylinder, Ellipse, GeomError, ImplicitDef, Line, NurbsCurve, NurbsSurface, ParametricCurve, ParametricSurface, Plane, Sphere, Torus};
use sm_core::{EntityId, EntityKind};

/// Serializable description of a curve.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum CurveDef {
    /// Line.
    Line(Line),
    /// Circle.
    Circle(Circle),
    /// Ellipse.
    Ellipse(Ellipse),
    /// NURBS (reference evaluator in this crate).
    Nurbs(NurbsCurve),
}

/// Serializable description of a surface.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum SurfaceDef {
    /// Plane.
    Plane(Plane),
    /// Cylinder.
    Cylinder(Cylinder),
    /// Cone (one nappe parametrically).
    Cone(Cone),
    /// Sphere.
    Sphere(Sphere),
    /// Ring torus (`R > r`).
    Torus(Torus),
    /// NURBS (reference evaluator in this crate).
    Nurbs(NurbsSurface),
    /// An implicit-only surface (general quadric, torus quartic). No parametrization.
    Implicit(ImplicitDef),
}

fn finite_pos(v: f64) -> bool {
    v.is_finite() && v > 0.0
}

impl CurveDef {
    /// The evaluable curve.
    pub fn as_curve(&self) -> &dyn ParametricCurve {
        match self {
            Self::Line(c) => c,
            Self::Circle(c) => c,
            Self::Ellipse(c) => c,
            Self::Nurbs(c) => c,
        }
    }

    /// Check the data (positive radii, valid NURBS).
    pub fn validate(&self) -> Result<(), GeomError> {
        match self {
            Self::Line(l) if !(l.dir.norm() > 0.0 && l.dir.norm().is_finite()) => Err(GeomError::Invalid("line direction")),
            Self::Circle(c) if !finite_pos(c.radius) => Err(GeomError::Invalid("circle radius")),
            Self::Ellipse(e) if !(finite_pos(e.a) && finite_pos(e.b)) => Err(GeomError::Invalid("ellipse radii")),
            Self::Nurbs(n) => n.validate(),
            _ => Ok(()),
        }
    }
}

impl SurfaceDef {
    /// The evaluable parametric surface, or `None` for [`SurfaceDef::Implicit`].
    pub fn as_surface(&self) -> Option<&dyn ParametricSurface> {
        Some(match self {
            Self::Plane(s) => s,
            Self::Cylinder(s) => s,
            Self::Cone(s) => s,
            Self::Sphere(s) => s,
            Self::Torus(s) => s,
            Self::Nurbs(s) => s,
            Self::Implicit(_) => return None,
        })
    }

    /// The implicit form, if the surface has one (every analytic kind and `Implicit`; not NURBS).
    pub fn implicit_def(&self) -> Option<ImplicitDef> {
        match self {
            Self::Implicit(d) => Some(*d),
            Self::Nurbs(_) => None,
            other => other.as_surface().and_then(|s| s.implicit()),
        }
    }

    /// Check the data (positive radii, ring torus, valid NURBS, finite quadric).
    pub fn validate(&self) -> Result<(), GeomError> {
        match self {
            Self::Cylinder(c) if !finite_pos(c.radius) => Err(GeomError::Invalid("cylinder radius")),
            Self::Sphere(s) if !finite_pos(s.radius) => Err(GeomError::Invalid("sphere radius")),
            Self::Cone(c) if !(c.radius.is_finite() && c.radius >= 0.0 && c.half_angle > 0.0 && c.half_angle < core::f64::consts::FRAC_PI_2) => Err(GeomError::Invalid("cone")),
            Self::Torus(t) if !(finite_pos(t.minor) && t.major > t.minor && t.major.is_finite()) => Err(GeomError::Invalid("torus must satisfy major > minor > 0")),
            Self::Nurbs(n) => n.validate(),
            Self::Implicit(ImplicitDef::Quadric(q)) if q.coeffs.iter().any(|c| !c.is_finite()) => Err(GeomError::Invalid("quadric coefficients")),
            _ => Ok(()),
        }
    }
}

/// Resolves curve and surface handles to shape objects. Object-safe; implementors must be
/// deterministic (same handle, same object).
pub trait GeomStore: Send + Sync {
    /// The curve behind a `Curve` handle.
    fn curve(&self, id: EntityId) -> Option<&dyn ParametricCurve>;
    /// The parametric surface behind a `Surface` handle (`None` for implicit-only surfaces).
    fn surface(&self, id: EntityId) -> Option<&dyn ParametricSurface>;
    /// The implicit form of a surface, for every surface that has one.
    fn implicit(&self, id: EntityId) -> Option<&dyn ImplicitSurface>;
    /// The curve definition (serializable data).
    fn curve_def(&self, id: EntityId) -> Option<&CurveDef>;
    /// The surface definition (serializable data).
    fn surface_def(&self, id: EntityId) -> Option<&SurfaceDef>;
    /// Number of curve handles allocated (serials are `0..count`).
    fn curve_count(&self) -> usize;
    /// Number of surface handles allocated (serials are `0..count`).
    fn surface_count(&self) -> usize;
}

/// Mutable geometry store. Handles come from a deterministic per-kind counter: the same insertion
/// order gives the same ids.
pub trait GeomStoreMut: GeomStore {
    /// Insert a curve; returns its handle (`Curve#n`, `n` = insertion count).
    fn add_curve(&mut self, def: CurveDef) -> Result<EntityId, GeomError>;
    /// Insert a surface; returns its handle (`Surface#n`).
    fn add_surface(&mut self, def: SurfaceDef) -> Result<EntityId, GeomError>;
    /// Replace the definition behind an existing curve handle; returns the old definition.
    fn replace_curve(&mut self, id: EntityId, def: CurveDef) -> Result<CurveDef, GeomError>;
    /// Replace the definition behind an existing surface handle; returns the old definition.
    fn replace_surface(&mut self, id: EntityId, def: SurfaceDef) -> Result<SurfaceDef, GeomError>;
    /// Drop every handle with serial `>= curves` / `>= surfaces` (rollback of recent additions).
    fn truncate(&mut self, curves: usize, surfaces: usize);
}

/// Reference in-memory store.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize), serde(from = "StoreData", into = "StoreData"))]
pub struct MemoryStore {
    curves: Vec<CurveDef>,
    surfaces: Vec<SurfaceDef>,
    /// Cached implicit forms, parallel to `surfaces`, so `implicit(id)` can hand out a borrow.
    implicits: Vec<Option<ImplicitDef>>,
}

/// The persisted form of a [`MemoryStore`] (definitions only).
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StoreData {
    /// Curve definitions, serial order.
    pub curves: Vec<CurveDef>,
    /// Surface definitions, serial order.
    pub surfaces: Vec<SurfaceDef>,
}

impl From<StoreData> for MemoryStore {
    fn from(d: StoreData) -> Self {
        let implicits = d.surfaces.iter().map(SurfaceDef::implicit_def).collect();
        Self { curves: d.curves, surfaces: d.surfaces, implicits }
    }
}

impl From<MemoryStore> for StoreData {
    fn from(s: MemoryStore) -> Self {
        Self { curves: s.curves, surfaces: s.surfaces }
    }
}

impl MemoryStore {
    /// An empty store.
    pub fn new() -> Self {
        Self::default()
    }

    fn slot(id: EntityId, kind: EntityKind, len: usize) -> Option<usize> {
        if id.kind() != kind {
            return None;
        }
        usize::try_from(id.serial()).ok().filter(|s| *s < len)
    }
}

impl GeomStore for MemoryStore {
    fn curve(&self, id: EntityId) -> Option<&dyn ParametricCurve> {
        self.curve_def(id).map(CurveDef::as_curve)
    }
    fn surface(&self, id: EntityId) -> Option<&dyn ParametricSurface> {
        self.surface_def(id).and_then(SurfaceDef::as_surface)
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

impl GeomStoreMut for MemoryStore {
    fn add_curve(&mut self, def: CurveDef) -> Result<EntityId, GeomError> {
        def.validate()?;
        let id = EntityId::new(EntityKind::Curve, self.curves.len() as u64).ok_or(GeomError::Invalid("id space exhausted"))?;
        self.curves.push(def);
        Ok(id)
    }
    fn add_surface(&mut self, def: SurfaceDef) -> Result<EntityId, GeomError> {
        def.validate()?;
        let id = EntityId::new(EntityKind::Surface, self.surfaces.len() as u64).ok_or(GeomError::Invalid("id space exhausted"))?;
        self.implicits.push(def.implicit_def());
        self.surfaces.push(def);
        Ok(id)
    }
    fn replace_curve(&mut self, id: EntityId, def: CurveDef) -> Result<CurveDef, GeomError> {
        def.validate()?;
        let s = Self::slot(id, EntityKind::Curve, self.curves.len()).ok_or(GeomError::Invalid("unknown curve handle"))?;
        Ok(core::mem::replace(&mut self.curves[s], def))
    }
    fn replace_surface(&mut self, id: EntityId, def: SurfaceDef) -> Result<SurfaceDef, GeomError> {
        def.validate()?;
        let s = Self::slot(id, EntityKind::Surface, self.surfaces.len()).ok_or(GeomError::Invalid("unknown surface handle"))?;
        self.implicits[s] = def.implicit_def();
        Ok(core::mem::replace(&mut self.surfaces[s], def))
    }
    fn truncate(&mut self, curves: usize, surfaces: usize) {
        self.curves.truncate(curves);
        self.surfaces.truncate(surfaces);
        self.implicits.truncate(surfaces);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Frame, Quadric};
    use sm_core::{Point3, Vector3};

    #[test]
    fn deterministic_handles_and_lookup() {
        let build = || {
            let mut s = MemoryStore::new();
            let a = s.add_surface(SurfaceDef::Plane(Plane { frame: Frame::at(Point3::default()) })).unwrap();
            let b = s.add_surface(SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::default()), 1.0).unwrap())).unwrap();
            (s, a, b)
        };
        let (s1, a1, b1) = build();
        let (s2, a2, b2) = build();
        assert_eq!((a1, b1), (a2, b2));
        assert_eq!(s1, s2);
        assert_eq!(a1.to_string(), "Surface#0");
        assert!(s1.surface(b1).is_some() && s1.curve(a1).is_none());
        assert!(s1.implicit(b1).is_some());
        let g: &dyn GeomStore = &s1;
        assert!(g.surface(a1).is_some());
        assert_eq!(g.surface_count(), 2);
    }

    #[test]
    fn implicit_only_and_nurbs_and_validation() {
        let mut s = MemoryStore::new();
        let q = s.add_surface(SurfaceDef::Implicit(ImplicitDef::Quadric(Quadric::sphere(&Frame::at(Point3::default()), 2.0)))).unwrap();
        assert!(s.surface(q).is_none());
        assert!(s.implicit(q).is_some());
        assert!(matches!(s.surface_def(q), Some(SurfaceDef::Implicit(_))));
        let c = s.add_curve(CurveDef::Line(Line::new(Point3::default(), Vector3::new(1.0, 0.0, 0.0)).unwrap())).unwrap();
        assert!(s.curve(c).is_some());
        assert!(s.add_curve(CurveDef::Circle(Circle { frame: Frame::at(Point3::default()), radius: -1.0 })).is_err());
        let n = NurbsCurve { degree: 1, knots: vec![0.0, 0.0, 1.0, 1.0], control: vec![Point3::default(), Point3::new(1.0, 0.0, 0.0)], weights: None };
        let nc = s.add_curve(CurveDef::Nurbs(n)).unwrap();
        assert!((s.curve(nc).unwrap().eval(0.5) - Point3::new(0.5, 0.0, 0.0)).norm() < 1e-14, "NURBS is evaluable through the store");
        let bad = NurbsCurve { degree: 1, knots: vec![0.0, 1.0], control: vec![Point3::default(), Point3::default()], weights: None };
        assert!(s.add_curve(CurveDef::Nurbs(bad)).is_err());
        // Replace and truncate.
        let old = s.replace_curve(c, CurveDef::Line(Line::new(Point3::default(), Vector3::new(0.0, 1.0, 0.0)).unwrap())).unwrap();
        assert!(matches!(old, CurveDef::Line(_)));
        s.truncate(1, 0);
        assert_eq!((s.curve_count(), s.surface_count()), (1, 0));
        assert!(s.implicit(q).is_none());
    }
}
