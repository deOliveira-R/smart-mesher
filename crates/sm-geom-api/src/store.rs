//! Geometry by handle: closed serializable definitions and the store that resolves them.
//!
//! Topology never owns shape. It holds `EntityId` handles (kinds `Curve` / `Surface`) and asks a
//! [`GeomStore`] to resolve them; many faces can share one surface. [`CurveDef`] and
//! [`SurfaceDef`] are closed enums, so every stored shape serializes and clones; the object-safe
//! traits are implemented by delegation. [`SurfaceDef::Implicit`] stores a surface that has no
//! parametrization (a general quadric, the torus quartic): `surface(id)` is `None` for it and
//! `implicit(id)` is `Some`.

use crate::implicit::ImplicitSurface;
use crate::{Circle, Cone, Frame, Cylinder, Ellipse, GeomError, ImplicitDef, Line, NurbsCurve, NurbsSurface, ParametricCurve, ParametricSurface, Plane, Sphere, Torus};
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

/// One number of a shape definition, tagged with how it is quantized for hashing (see
/// [`crate::ModelTolerance`]). Produced by [`CurveDef::for_each_value`] and [`SurfaceDef::for_each_value`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ShapeValue {
    /// A discriminant, count or degree: hashed as is.
    Int(u64),
    /// A length (origins, radii, control points): `linear * 2^-10`.
    Length(f64),
    /// An angle or a unit-vector component (cone half-angle, frame axes): `angular * 2^-10`.
    Angle(f64),
    /// A scale-free value (knots, weights, quadric coefficients, line direction): quantized
    /// relatively (40-bit mantissa, never saturates), see
    /// [`crate::ModelTolerance::quantize_scale_free`].
    Unitless(f64),
}

fn frame_values(fr: &Frame, f: &mut impl FnMut(ShapeValue)) {
    for v in [fr.origin.x, fr.origin.y, fr.origin.z] {
        f(ShapeValue::Length(v));
    }
    for a in [fr.x, fr.y, fr.z] {
        for v in [a.x, a.y, a.z] {
            f(ShapeValue::Angle(v));
        }
    }
}

fn points_values(pts: &[sm_core::Point3], f: &mut impl FnMut(ShapeValue)) {
    f(ShapeValue::Int(pts.len() as u64));
    for p in pts {
        for v in [p.x, p.y, p.z] {
            f(ShapeValue::Length(v));
        }
    }
}

fn floats_values(xs: &[f64], f: &mut impl FnMut(ShapeValue)) {
    f(ShapeValue::Int(xs.len() as u64));
    xs.iter().for_each(|x| f(ShapeValue::Unitless(*x)));
}

fn weights_values(w: &Option<Vec<f64>>, f: &mut impl FnMut(ShapeValue)) {
    match w {
        None => f(ShapeValue::Int(0)),
        Some(w) => {
            f(ShapeValue::Int(1));
            floats_values(w, f);
        }
    }
}

impl ImplicitDef {
    /// Visit every number of the definition in a fixed order (variant tag first).
    pub fn for_each_value(&self, mut f: impl FnMut(ShapeValue)) {
        match self {
            Self::Quadric(q) => {
                f(ShapeValue::Int(1));
                q.coeffs.iter().for_each(|c| f(ShapeValue::Unitless(*c)));
            }
            Self::Torus(t) => {
                f(ShapeValue::Int(2));
                frame_values(&t.frame, &mut f);
                f(ShapeValue::Length(t.major));
                f(ShapeValue::Length(t.minor));
            }
        }
    }
}

fn finite_pos(v: f64) -> bool {
    v.is_finite() && v > 0.0
}

impl CurveDef {
    /// Visit every number of the definition in a fixed order (variant tag first), each tagged
    /// with its quantization class. Content hashes are built from this.
    pub fn for_each_value(&self, mut f: impl FnMut(ShapeValue)) {
        match self {
            Self::Line(l) => {
                f(ShapeValue::Int(1));
                for v in [l.origin.x, l.origin.y, l.origin.z] {
                    f(ShapeValue::Length(v));
                }
                for v in [l.dir.x, l.dir.y, l.dir.z] {
                    f(ShapeValue::Unitless(v));
                }
            }
            Self::Circle(c) => {
                f(ShapeValue::Int(2));
                frame_values(&c.frame, &mut f);
                f(ShapeValue::Length(c.radius));
            }
            Self::Ellipse(e) => {
                f(ShapeValue::Int(3));
                frame_values(&e.frame, &mut f);
                f(ShapeValue::Length(e.a));
                f(ShapeValue::Length(e.b));
            }
            Self::Nurbs(n) => {
                f(ShapeValue::Int(4));
                f(ShapeValue::Int(n.degree as u64));
                floats_values(&n.knots, &mut f);
                points_values(&n.control, &mut f);
                weights_values(&n.weights, &mut f);
            }
        }
    }

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
    /// Visit every number of the definition in a fixed order (variant tag first), each tagged
    /// with its quantization class. Content hashes are built from this.
    pub fn for_each_value(&self, mut f: impl FnMut(ShapeValue)) {
        match self {
            Self::Plane(p) => {
                f(ShapeValue::Int(1));
                frame_values(&p.frame, &mut f);
            }
            Self::Cylinder(c) => {
                f(ShapeValue::Int(2));
                frame_values(&c.frame, &mut f);
                f(ShapeValue::Length(c.radius));
            }
            Self::Cone(c) => {
                f(ShapeValue::Int(3));
                frame_values(&c.frame, &mut f);
                f(ShapeValue::Length(c.radius));
                f(ShapeValue::Angle(c.half_angle));
            }
            Self::Sphere(s) => {
                f(ShapeValue::Int(4));
                frame_values(&s.frame, &mut f);
                f(ShapeValue::Length(s.radius));
            }
            Self::Torus(t) => {
                f(ShapeValue::Int(5));
                frame_values(&t.frame, &mut f);
                f(ShapeValue::Length(t.major));
                f(ShapeValue::Length(t.minor));
            }
            Self::Nurbs(n) => {
                f(ShapeValue::Int(6));
                f(ShapeValue::Int(n.degree_u as u64));
                f(ShapeValue::Int(n.degree_v as u64));
                f(ShapeValue::Int(n.nu as u64));
                f(ShapeValue::Int(n.nv as u64));
                floats_values(&n.knots_u, &mut f);
                floats_values(&n.knots_v, &mut f);
                points_values(&n.control, &mut f);
                weights_values(&n.weights, &mut f);
            }
            Self::Implicit(d) => {
                f(ShapeValue::Int(7));
                d.for_each_value(f);
            }
        }
    }

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

/// Append-only geometry store. Handles come from a deterministic per-kind counter: the same
/// insertion order gives the same ids. This is all that topology hands out through
/// `TopologyEdit::shapes_mut`, so an operation can always roll its additions back.
pub trait GeomAppend: GeomStore {
    /// Insert a curve; returns its handle (`Curve#n`, `n` = insertion count).
    fn add_curve(&mut self, def: CurveDef) -> Result<EntityId, GeomError>;
    /// Insert a surface; returns its handle (`Surface#n`).
    fn add_surface(&mut self, def: SurfaceDef) -> Result<EntityId, GeomError>;
}

/// Mutable geometry store: appending plus in-place replacement. Replacement is journaled and
/// lineage-logged only through `TopologyEdit::replace_curve` / `replace_surface`.
pub trait GeomStoreMut: GeomAppend {
    /// Replace the definition behind an existing curve handle; returns the old definition.
    fn replace_curve(&mut self, id: EntityId, def: CurveDef) -> Result<CurveDef, GeomError>;
    /// Replace the definition behind an existing surface handle; returns the old definition.
    fn replace_surface(&mut self, id: EntityId, def: SurfaceDef) -> Result<SurfaceDef, GeomError>;
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

    /// Drop every handle with serial `>= curves` / `>= surfaces` (rollback of recent additions).
    /// Not on a trait: only an owner that tracks which handles are referenced may call it.
    pub fn truncate(&mut self, curves: usize, surfaces: usize) {
        self.curves.truncate(curves);
        self.surfaces.truncate(surfaces);
        self.implicits.truncate(surfaces);
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

impl GeomAppend for MemoryStore {
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
}

impl GeomStoreMut for MemoryStore {
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
    fn value_visitor_sees_definitions() {
        let vals = |d: &SurfaceDef| {
            let mut v = Vec::new();
            d.for_each_value(|x| v.push(x));
            v
        };
        let a = SurfaceDef::Implicit(ImplicitDef::Quadric(Quadric { coeffs: [0.25, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5] }));
        let b = SurfaceDef::Implicit(ImplicitDef::Quadric(Quadric { coeffs: [4.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5] }));
        assert_ne!(vals(&a), vals(&b));
        let s1 = SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::default()), 1.0).unwrap());
        let s7 = SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::default()), 7.0).unwrap());
        assert_ne!(vals(&s1), vals(&s7));
        let mut c = Vec::new();
        CurveDef::Line(Line::new(Point3::default(), Vector3::new(1.0, 0.0, 0.0)).unwrap()).for_each_value(|x| c.push(x));
        assert_eq!(c.len(), 7);
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
