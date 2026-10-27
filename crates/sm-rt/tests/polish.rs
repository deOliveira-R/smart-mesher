//! Polish: default boundary at the complement's box exit, touching unmerged shells.
mod common;
use common::*;
use sm_core::Point3;
use sm_rt::check_facets;
use sm_rt_api::{BoundaryDef, BoundarySpec, BuildInput, CellId, FaceAction, FacetInput, ResponseKind, RtError, Scratch, StepEnd, TrackerBuilder};
use sm_topo_api::Region;

fn two_tets() -> FacetInput {
    let mut f = FacetInput::default();
    let pts = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let tris = [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
    add_shell(&mut f, &pts, &tris, vol(1), Region::Exterior, 1);
    let moved = pts.map(|p| [p[0] + 3.0, p[1], p[2]]);
    add_shell(&mut f, &moved, &tris, vol(2), Region::Exterior, 2);
    f
}

#[test]
fn default_applies_at_the_complement_box_exit_with_a_warning() {
    let f = two_tets();
    let b = sm_rt::MeshTrackerBuilder;
    let opts = opts_fail();
    // Vacuum default: no warning, the ray leaves the model at the box.
    let (_, rep) = b.build(&BuildInput::Facets(&f, &BoundarySpec::default()), &opts).unwrap();
    assert!(rep.issues.is_empty(), "{:?}", rep.issues);
    // Reflective default: unclaimed faces lead into the complement, the box reflects.
    let spec = BoundarySpec::new(BoundaryDef::Respond(ResponseKind::Reflective), vec![]);
    let (t, rep) = b.build(&BuildInput::Facets(&f, &spec), &opts).unwrap();
    assert!(rep.issues.iter().any(|i| i.code == "boundary.default_at_complement_exit"), "{:?}", rep.issues);
    let mut sc = Scratch::new();
    let mut st = t.init_state(Point3::new(0.1, 0.1, 0.1), v3([1.0, 0.0, 0.0]), None, &mut sc).unwrap();
    let mut regions = 0;
    let mut hit_box = false;
    for _ in 0..16 {
        let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
        match s.end {
            StepEnd::Boundary { action, face, .. } => {
                assert!(matches!(action, FaceAction::Respond(ResponseKind::Reflective)));
                assert_eq!(s.cell, CellId::COMPLEMENT);
                assert!((st.pos.x - 4.0).abs() < 1e-12, "{:?}", st.pos);
                assert_eq!(t.face_normal(face, st.pos).unwrap(), v3([1.0, 0.0, 0.0]));
                t.resolve_boundary(&mut st, face, ResponseKind::Reflective, [0.5, 0.5]).unwrap();
                assert!((st.dir.x + 1.0).abs() < 1e-14);
                hit_box = true;
                break;
            }
            StepEnd::Lost(_) => panic!("lost"),
            _ => regions += 1,
        }
    }
    assert!(hit_box && regions == 3, "{regions} steps before the box");
    // The default survives the cache.
    let img = sm_rt::read_smrt(&t.export_cache().unwrap().encode()).unwrap();
    let (u, _) = b.load(&BuildInput::Facets(&f, &spec), &opts, &img).unwrap();
    assert_eq!(u.info(), t.info());
    assert_eq!(u.structure_hash(), t.structure_hash());
}

#[test]
fn touching_unmerged_shells_are_reported_and_refused() {
    let mut f = FacetInput::default();
    let pts = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let tris = [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
    add_shell(&mut f, &pts, &tris, vol(1), Region::Exterior, 1);
    // A second tetrahedron on the other side of the x+y+z = 1 facet, with its own vertices.
    let q = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 1.0, 1.0]];
    let tq = [[0, 1, 2], [0, 3, 1], [1, 3, 2], [2, 3, 0]];
    add_shell(&mut f, &q, &tq, vol(2), Region::Exterior, 2);
    let issues = check_facets(&f, &BoundarySpec::default());
    let c: Vec<_> = issues.iter().filter(|i| i.code == "facet.coincident").collect();
    assert_eq!(c.len(), 1, "{issues:?}");
    assert!(c[0].location.is_some());
    assert!(matches!(build(&f, &opts_fail()), Err(RtError::Build(_))));
    // Separate bodies are not flagged.
    assert!(check_facets(&two_tets(), &BoundarySpec::default()).is_empty());
}
