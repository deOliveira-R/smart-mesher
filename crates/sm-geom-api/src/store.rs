//! Handle resolution: the GMI-like seam between topology and shape.
//!
//! Topology never owns geometry. It holds `EntityId` handles (kinds `Curve` / `Surface`) and asks a
//! [`GeometryStore`] to resolve them. One surface may carry many faces.

use crate::{Circle, Cone, Cylinder, Ellipse, GeomError, Line, NurbsCurve, NurbsSurface, ParametricCurve, ParametricSurface, Plane, Sphere, Torus};
use sm_core::{EntityId, EntityKind};
use std::collections::BTreeMap;

/// Resolves curve and surface handles to shape objects. Object-safe; implementors must be
/// deterministic (same handle, same object).
pub trait GeometryStore: Send + Sync {
    /// The curve behind a `Curve` handle, if present and evaluable.
    fn curve(&self, id: EntityId) -> Option<&dyn ParametricCurve>;
    /// The surface behind a `Surface` handle, if present and evaluable.
    fn surface(&self, id: EntityId) -> Option<&dyn ParametricSurface>;
}

/// Serializable description of a curve: an analytic kind or NURBS data.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CurveData {
    /// Line.
    Line(Line),
    /// Circle.
    Circle(Circle),
    /// Ellipse.
    Ellipse(Ellipse),
    /// NURBS (needs `sm-geom` to become evaluable).
    Nurbs(NurbsCurve),
}

/// Serializable description of a surface: an analytic kind or NURBS data.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SurfaceData {
    /// Plane.
    Plane(Plane),
    /// Cylinder.
    Cylinder(Cylinder),
    /// Cone.
    Cone(Cone),
    /// Sphere.
    Sphere(Sphere),
    /// Torus.
    Torus(Torus),
    /// NURBS (needs `sm-geom` to become evaluable).
    Nurbs(NurbsSurface),
}

impl CurveData {
    /// Turn the description into an evaluable object. NURBS is [`GeomError::Unsupported`] here.
    pub fn into_dyn(self) -> Result<Box<dyn ParametricCurve>, GeomError> {
        Ok(match self {
            Self::Line(c) => Box::new(c),
            Self::Circle(c) => Box::new(c),
            Self::Ellipse(c) => Box::new(c),
            Self::Nurbs(n) => {
                n.validate()?;
                return Err(GeomError::Unsupported("NURBS evaluation lives in sm-geom"));
            }
        })
    }
}

impl SurfaceData {
    /// Turn the description into an evaluable object. NURBS is [`GeomError::Unsupported`] here.
    pub fn into_dyn(self) -> Result<Box<dyn ParametricSurface>, GeomError> {
        Ok(match self {
            Self::Plane(s) => Box::new(s),
            Self::Cylinder(s) => Box::new(s),
            Self::Cone(s) => Box::new(s),
            Self::Sphere(s) => Box::new(s),
            Self::Torus(s) => Box::new(s),
            Self::Nurbs(n) => {
                n.validate()?;
                return Err(GeomError::Unsupported("NURBS evaluation lives in sm-geom"));
            }
        })
    }
}

/// Reference in-memory [`GeometryStore`]. Handles are allocated sequentially per kind, so the
/// same insertion order gives the same ids.
#[derive(Debug, Default)]
pub struct MemoryStore {
    curves: BTreeMap<EntityId, Box<dyn ParametricCurve>>,
    surfaces: BTreeMap<EntityId, Box<dyn ParametricSurface>>,
}

impl MemoryStore {
    /// An empty store.
    pub fn new() -> Self {
        Self::default()
    }

    /// Insert a curve; returns its handle (`Curve#n`, `n` = insertion count).
    pub fn add_curve(&mut self, c: Box<dyn ParametricCurve>) -> Result<EntityId, GeomError> {
        let id = EntityId::new(EntityKind::Curve, self.curves.len() as u64).ok_or(GeomError::Invalid("id space exhausted"))?;
        self.curves.insert(id, c);
        Ok(id)
    }

    /// Insert a surface; returns its handle (`Surface#n`).
    pub fn add_surface(&mut self, s: Box<dyn ParametricSurface>) -> Result<EntityId, GeomError> {
        let id = EntityId::new(EntityKind::Surface, self.surfaces.len() as u64).ok_or(GeomError::Invalid("id space exhausted"))?;
        self.surfaces.insert(id, s);
        Ok(id)
    }

    /// Number of stored curves.
    pub fn curve_count(&self) -> usize {
        self.curves.len()
    }

    /// Number of stored surfaces.
    pub fn surface_count(&self) -> usize {
        self.surfaces.len()
    }
}

impl GeometryStore for MemoryStore {
    fn curve(&self, id: EntityId) -> Option<&dyn ParametricCurve> {
        self.curves.get(&id).map(|b| b.as_ref())
    }
    fn surface(&self, id: EntityId) -> Option<&dyn ParametricSurface> {
        self.surfaces.get(&id).map(|b| b.as_ref())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Frame;
    use sm_core::Point3;

    #[test]
    fn deterministic_handles_and_lookup() {
        let build = || {
            let mut s = MemoryStore::new();
            let a = s.add_surface(Box::new(Plane { frame: Frame::at(Point3::default()) })).unwrap();
            let b = s.add_surface(Box::new(Sphere::new(Frame::at(Point3::default()), 1.0).unwrap())).unwrap();
            (s, a, b)
        };
        let (s1, a1, b1) = build();
        let (_, a2, b2) = build();
        assert_eq!((a1, b1), (a2, b2));
        assert_eq!(a1.to_string(), "Surface#0");
        assert!(s1.surface(b1).is_some() && s1.curve(a1).is_none());
        let g: &dyn GeometryStore = &s1;
        assert!(g.surface(a1).is_some());
    }

    #[test]
    fn data_to_dyn() {
        let c = CurveData::Line(Line::new(Point3::default(), sm_core::Vector3::new(1.0, 0.0, 0.0)).unwrap());
        assert!(c.into_dyn().is_ok());
        let n = NurbsCurve { degree: 1, knots: vec![0.0, 0.0, 1.0, 1.0], control: vec![Point3::default(), Point3::new(1.0, 0.0, 0.0)], weights: None };
        assert!(matches!(CurveData::Nurbs(n).into_dyn(), Err(GeomError::Unsupported(_))));
    }
}
