//! Projection of new boundary nodes onto the model geometry (the rule of `sm-smooth`:
//! `project` with no hint, refused when it leaves the curve range or the trimmed face).

use sm_core::{EntityKind, Point3};
use sm_geom_api::{Domain1, Interval, ParametricCurve, ParametricSurface};
use sm_mesh_api::{Classification, Param};
use sm_topo_api::{Carrier, PointClass, TopologyView};

/// Where a projected node ended up.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Projected {
    /// The point on the entity.
    pub point: [f64; 3],
    /// Its parameter on the classified entity.
    pub param: Param,
}

/// Looks up the analytic carrier of a classified model entity.
pub(crate) struct Projector<'a> {
    topo: &'a dyn TopologyView,
}

enum Geo<'a> {
    Curve(&'a dyn ParametricCurve, Interval, Domain1),
    Surf(&'a dyn ParametricSurface),
}

impl<'a> Projector<'a> {
    pub(crate) fn new(topo: &'a dyn TopologyView) -> Self {
        Self { topo }
    }

    /// True if nodes classified like `c` are projected at all (an edge or a face of the model).
    pub(crate) fn applies(&self, c: Classification) -> bool {
        matches!(c.entity.kind(), EntityKind::TopoEdge | EntityKind::TopoFace) && self.topo.is_alive(c.entity)
    }

    fn geo(&self, c: Classification) -> Option<Geo<'a>> {
        match (c.entity.kind(), self.topo.carrier(c.entity)) {
            (EntityKind::TopoEdge, Carrier::Curve(cc)) => {
                let cv = self.topo.shapes().curve(cc.curve)?;
                Some(Geo::Curve(cv, cc.range, cv.domain()))
            }
            (EntityKind::TopoFace, Carrier::Surface(sc)) => Some(Geo::Surf(self.topo.shapes().surface(sc.surface)?)),
            _ => None,
        }
    }

    /// Project `p` onto the entity `c` is classified on. `None` when there is no carrier, the
    /// projection fails, or it leaves the curve range or the trimmed face.
    pub(crate) fn project(&self, c: Classification, p: [f64; 3]) -> Option<Projected> {
        let q = Point3::new(p[0], p[1], p[2]);
        match self.geo(c)? {
            Geo::Curve(curve, range, domain) => {
                let r = curve.project(q, None).ok()?;
                let mut t = r.t;
                if let Some(per) = domain.period {
                    t += ((range.lo - t) / per).ceil() * per;
                }
                if !(t >= range.lo && t <= range.hi) {
                    return None;
                }
                Some(Projected { point: [r.point.x, r.point.y, r.point.z], param: Param::Curve(t) })
            }
            Geo::Surf(surf) => {
                let r = surf.project(q, None).ok()?;
                match self.topo.classify_point_on_face(c.entity, r.uv) {
                    Ok(PointClass::Out) | Err(_) => None,
                    Ok(_) => Some(Projected { point: [r.point.x, r.point.y, r.point.z], param: Param::Surface([r.uv.u, r.uv.v]) }),
                }
            }
        }
    }
}
