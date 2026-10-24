//! W0g freeze: constructors of the non-exhaustive records, serde shapes of the persisted ones.

use sm_core::{Point3, Vector3};
use sm_geom_api::*;

#[test]
fn constructors_match_fields() {
    let h = RayHit::new(2.0, Point3::new(1.0, 2.0, 3.0), Vector3::new(0.0, 0.0, 1.0), Some(Uv::new(0.5, 0.25)), Crossing::AlongNormal);
    assert_eq!((h.t, h.point, h.normal, h.uv, h.crossing), (2.0, Point3::new(1.0, 2.0, 3.0), Vector3::new(0.0, 0.0, 1.0), Some(Uv::new(0.5, 0.25)), Crossing::AlongNormal));
    let c = CurveProjection::new(0.5, Point3::new(1.0, 0.0, 0.0), 0.1);
    assert_eq!((c.t, c.point, c.distance), (0.5, Point3::new(1.0, 0.0, 0.0), 0.1));
    let s = SurfaceProjection::new(Uv::new(1.0, 2.0), Point3::default(), 0.0);
    assert_eq!((s.uv, s.point, s.distance), (Uv::new(1.0, 2.0), Point3::default(), 0.0));
    let pole = Singularity::new(ParamAxis::V, 1.5, Point3::new(0.0, 0.0, 1.0));
    assert_eq!((pole.along, pole.at, pole.point), (ParamAxis::V, 1.5, Point3::new(0.0, 0.0, 1.0)));
    let d = Domain2::new(Domain1::periodic(0.0, 6.0), Domain1::open(-1.5, 1.5), [pole]);
    assert_eq!(d.singularities.as_slice(), &[pole]);
    assert_eq!(Domain2::new(d.u, d.v, []), Domain2::regular(d.u, d.v));
    let t = ModelTolerance::new(1e-5, 1e-8, 1e-10, 1e-2);
    assert_eq!((t.linear, t.angular, t.parametric, t.linear_max), (1e-5, 1e-8, 1e-10, 1e-2));
    let u = ModelTolerance::DEFAULT.with_linear(1e-5).with_angular(1e-8).with_parametric(1e-10).with_linear_max(1e-2);
    assert_eq!(t, u);
    let n2 = NurbsCurve2::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![Uv::new(0.0, 0.0), Uv::new(1.0, 1.0)], None).unwrap();
    assert!(!n2.periodic);
    assert!(NurbsCurve2::new(1, vec![0.0, 1.0], vec![Uv::new(0.0, 0.0)], None).is_err(), "validates");
    let p2 = NurbsCurve2::new_periodic(1, vec![0.0, 1.0, 2.0, 3.0, 4.0], vec![Uv::new(0.0, 0.0), Uv::new(1.0, 0.0), Uv::new(0.0, 0.0)], None).unwrap();
    assert!(p2.periodic);
    let ns = NurbsSurface::new(1, 1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0], 2, 2, vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0), Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)], None).unwrap();
    assert_eq!(ns.periodic, [false, false]);
    assert!(NurbsSurface::new(1, 1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0], 2, 2, vec![Point3::default()], None).is_err());
    let sd = StoreData::new(vec![CurveDef::Nurbs(NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![Point3::default(), Point3::new(1.0, 0.0, 0.0)], None).unwrap())], Vec::new());
    assert_eq!((sd.curves.len(), sd.surfaces.len()), (1, 0));
}

#[test]
fn crossing_matches_need_a_wildcard() {
    let entered = |c: Crossing| match c {
        Crossing::AlongNormal => Some(true),
        Crossing::AgainstNormal => Some(false),
        _ => None,
    };
    assert_eq!(entered(Crossing::Tangent), None);
}

#[cfg(feature = "serde")]
#[test]
fn golden_serde_shapes() {
    use serde_json::json;
    // Field names of the persisted records are frozen.
    assert_eq!(serde_json::to_value(ModelTolerance::DEFAULT).unwrap(), json!({"linear": 1e-6, "angular": 1e-9, "parametric": 1e-9, "linear_max": 1e-3}));
    let n2 = NurbsCurve2::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![Uv::new(0.0, 0.0), Uv::new(1.0, 1.0)], None).unwrap();
    let v = serde_json::to_value(&n2).unwrap();
    assert_eq!(v, json!({"degree": 1, "knots": [0.0, 0.0, 1.0, 1.0], "control": [{"u": 0.0, "v": 0.0}, {"u": 1.0, "v": 1.0}], "weights": null, "periodic": false}));
    // Data written before `periodic` existed still reads.
    let old = json!({"degree": 1, "knots": [0.0, 0.0, 1.0, 1.0], "control": [{"u": 0.0, "v": 0.0}, {"u": 1.0, "v": 1.0}], "weights": null});
    assert_eq!(serde_json::from_value::<NurbsCurve2>(old).unwrap(), n2);
    // Round trips.
    let ns = NurbsSurface::new_periodic(1, 1, vec![0.0, 1.0, 2.0, 3.0, 4.0], vec![0.0, 0.0, 1.0, 1.0], 3, 2, vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 1.0), Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 1.0), Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 1.0)], None, [true, false]).unwrap();
    assert_eq!(serde_json::from_value::<NurbsSurface>(serde_json::to_value(&ns).unwrap()).unwrap(), ns);
    let sd = StoreData::new(vec![CurveDef::Nurbs(NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![Point3::default(), Point3::new(1.0, 0.0, 0.0)], None).unwrap())], vec![SurfaceDef::Nurbs(ns)]);
    let sv = serde_json::to_value(&sd).unwrap();
    assert_eq!(sv.as_object().unwrap().keys().collect::<Vec<_>>(), ["curves", "surfaces"]);
    assert_eq!(serde_json::from_value::<StoreData>(sv).unwrap(), sd);
    let d = Domain2::new(Domain1::periodic(0.0, 6.0), Domain1::open(-1.5, 1.5), [Singularity::new(ParamAxis::V, 1.5, Point3::new(0.0, 0.0, 1.0))]);
    assert_eq!(serde_json::from_value::<Domain2>(serde_json::to_value(&d).unwrap()).unwrap(), d);
    let h = RayHit::new(2.0, Point3::new(1.0, 2.0, 3.0), Vector3::new(0.0, 0.0, 1.0), None, Crossing::Tangent);
    assert_eq!(serde_json::to_value(h.crossing).unwrap(), json!("Tangent"));
    assert_eq!(serde_json::from_value::<RayHit>(serde_json::to_value(h).unwrap()).unwrap(), h);
}
