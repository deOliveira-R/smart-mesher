//! Multi-generator deck groups.

use sm_core::{Point3, Vector3};
use sm_model_api::action::Plane;
use sm_model_api::boundary::{BoundaryKind, Deck};
use sm_model_api::color::ColorDomain;
use sm_model_api::mock::MockTopology;
use sm_model_api::symmetry::{SymmetryOrigin, UnfoldSpec};
use sm_model_api::validate::{validate, validate_unfolded};
use sm_model_api::{Model, Tolerance, TopologyView};

const TOL: Tolerance = Tolerance { linear: 1e-8, angular: 1e-9 };

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

#[test]
fn two_mirrors_generate_the_klein_four_group() {
    let mut t = MockTopology::new();
    let (v, f) = t.add_box(p(0.0, 0.0, 0.0), p(1.0, 1.0, 1.0));
    let mut m = Model::new();
    let c = m.add_color("fuel", ColorDomain::Volume).unwrap();
    m.color_volume(v, c).unwrap();
    let plane = |n: Vector3| Plane::new(p(0.0, 0.0, 0.0), n).unwrap();
    let mx = m.add_boundary("mx", BoundaryKind::Deck(Deck::Mirror { plane: plane(Vector3::new(1.0, 0.0, 0.0)) })).unwrap();
    let my = m.add_boundary("my", BoundaryKind::Deck(Deck::Mirror { plane: plane(Vector3::new(0.0, 1.0, 0.0)) })).unwrap();
    m.assign_faces(mx, &[f[0]]).unwrap();
    m.assign_faces(my, &[f[2]]).unwrap();
    let walls = m.add_boundary("walls", BoundaryKind::Response).unwrap();
    let rest: Vec<_> = t.faces().into_iter().filter(|x| !m.face_boundary.contains_key(x)).collect();
    m.assign_faces(walls, &rest).unwrap();
    m.declare_symmetry(&[mx, my], &[v], SymmetryOrigin::Declared).unwrap();
    assert!(validate(&m, &t, &TOL).is_ok());

    let u = m.unfold(&UnfoldSpec::default()).unwrap();
    assert_eq!(u.copies.len(), 4);
    assert_eq!(u.glued.len(), 4, "2 seams per mirror plane");
    assert!(u.exposed.is_empty());
    assert_eq!(u.copies.iter().filter(|c| c.improper).count(), 2, "two single reflections; their product is proper");
    assert!(validate_unfolded(&t, &u, &TOL).is_ok());
}
