//! W3b-PRIM acceptance 8: `model.copy`.

mod common;

use common::*;
use core::f64::consts::PI;
use sm_core::{EntityId, EntityKind};
use sm_layer_api::MeshLayer;
use sm_model::{AxisSpec, CopyXform, CreateBrick, CreateCylinder, CreateFrustum, CreateSphere, CreateSpherePart, CreateTorus, HalfSpace, ModelCopy, Octants};
use sm_op_api::{Op, Session};

fn vol(n: u64) -> EntityId {
    id(EntityKind::TopoVolume, n)
}

fn brick10() -> CreateBrick {
    CreateBrick { x: 10.0, y: None, z: None, center: None }
}

fn centroid(s: &Session, v: EntityId) -> [f64; 3] {
    let c = model(s).topo().volume_moments(v).expect("moments").centroid().expect("centroid");
    [c.x, c.y, c.z]
}

fn copy(entities: Vec<EntityId>, count: Option<u32>, xform: Option<CopyXform>) -> ModelCopy {
    ModelCopy { entities, count, xform, lattice: None }
}

fn mv(d: [f64; 3]) -> Option<CopyXform> {
    Some(CopyXform::Move { delta: d })
}

#[test]
fn brick_copy_count_3_move() {
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &brick10()).expect("brick");
        let before_pts = vertex_points(&s);
        let c0 = centroid(&s, vol(1));
        let (_, d) = run(&mut s, &copy(vec![vol(1)], Some(3), mv([20.0, 0.0, 0.0]))).expect("copy");
        assert_eq!(counts(&s), vec![32, 48, 24, 4, 4]);
        assert_eq!(d.outputs["volumes"], vec![vol(2), vol(3), vol(4)]);
        assert_eq!(d.outputs["bodies"], vec![id(EntityKind::TopoBody, 2), id(EntityKind::TopoBody, 3), id(EntityKind::TopoBody, 4)]);
        for (k, v) in volumes(&s).iter().enumerate() {
            assert!(near(*v, 1000.0, 1e-12), "volume {k}: {v}");
        }
        for k in 1..=3u64 {
            let c = centroid(&s, vol(1 + k));
            assert!((c[0] - c0[0] - 20.0 * k as f64).abs() < 1e-11 && (c[1] - c0[1]).abs() < 1e-11 && (c[2] - c0[2]).abs() < 1e-11, "copy {k}: {c:?}");
        }
        // The source is untouched, bit for bit.
        assert_eq!(&vertex_points(&s)[..8], &before_pts[..]);
        clean(&s);
        assert!(worst_outwardness(&s) > 0.0);
        // The copy equals a freshly built brick moved the same way (canonical hash of the cells).
        let mut r = session(factory);
        run(&mut r, &brick10()).expect("brick");
        run(&mut r, &sm_model::Move { entities: vec![vol(1)], delta: [20.0, 0.0, 0.0] }).expect("move");
        let want = vertex_points(&r);
        assert_eq!(&vertex_points(&s)[8..16], &want[..]);
    });
}

#[test]
fn copy_without_xform_makes_coincident_copies() {
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &brick10()).expect("brick");
        run(&mut s, &copy(vec![id(EntityKind::TopoBody, 1)], None, None)).expect("copy");
        let p = vertex_points(&s);
        assert_eq!(&p[..8], &p[8..16]);
        assert_eq!(counts(&s), vec![16, 24, 12, 2, 2]);
        clean(&s);
    });
}

#[test]
fn reflect_copies_have_positive_volume_and_outward_normals() {
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &CreateBrick { x: 2.0, y: Some(3.0), z: Some(4.0), center: Some([5.0, 0.0, 0.0]) }).expect("brick");
        run(&mut s, &copy(vec![vol(1)], Some(2), Some(CopyXform::Reflect { normal: [1.0, 0.0, 0.0], origin: None }))).expect("copy");
        let v = volumes(&s);
        assert!(v.iter().all(|x| near(*x, 24.0, 1e-12)), "{v:?}");
        let c1 = centroid(&s, vol(2));
        assert!((c1[0] + 5.0).abs() < 1e-12, "mirrored: {c1:?}");
        // The second copy is the mirror of the mirror: back on the source.
        let c2 = centroid(&s, vol(3));
        assert!((c2[0] - 5.0).abs() < 1e-12, "{c2:?}");
        clean(&s);
        assert!(worst_outwardness(&s) > 0.0);
        // A curved solid mirrored once.
        let mut s = session(factory);
        run(&mut s, &CreateCylinder { height: 2.0, radius: 1.0, center: Some([3.0, 1.0, 0.0]) }).expect("cyl");
        run(&mut s, &copy(vec![vol(1)], None, Some(CopyXform::Reflect { normal: [1.0, 1.0, 0.0], origin: None }))).expect("copy");
        clean(&s);
        assert!(volumes(&s).iter().all(|x| near(*x, 2.0 * PI, 1e-12)));
        assert!(worst_outwardness(&s) > 0.0);
    });
}

#[test]
fn every_curved_primitive_copies_cleanly_with_equal_volume_and_area() {
    use HalfSpace::Positive as P;
    let ops: Vec<(&str, Box<dyn Op>)> = vec![
        ("cylinder", Box::new(CreateCylinder { height: 3.0, radius: 1.5, center: None })),
        ("sphere", Box::new(CreateSphere { radius: 2.0, center: None })),
        ("torus", Box::new(CreateTorus { major: 5.0, minor: 1.0, center: None })),
        ("cone", Box::new(CreateFrustum { height: 3.0, radius: 2.0, minor: None, top: None, center: None })),
        ("frustum", Box::new(CreateFrustum { height: 3.0, radius: 2.0, minor: Some(1.0), top: Some(1.0), center: None })),
        ("hemisphere", Box::new(CreateSpherePart { radius: 2.0, inner: None, octants: Some(Octants { x: None, y: None, z: Some(P) }), center: None })),
        ("octant", Box::new(CreateSpherePart { radius: 2.0, inner: None, octants: Some(Octants { x: Some(P), y: Some(P), z: Some(P) }), center: None })),
        ("hollow sphere", Box::new(CreateSpherePart { radius: 2.0, inner: Some(1.0), octants: None, center: None })),
        ("hollow octant", Box::new(CreateSpherePart { radius: 2.0, inner: Some(1.0), octants: Some(Octants { x: Some(P), y: Some(P), z: Some(P) }), center: None })),
    ];
    let xforms: Vec<Option<CopyXform>> = vec![
        None,
        mv([30.0, 0.0, 0.0]),
        Some(CopyXform::Rotate { angle_deg: 33.0, axis: Some(AxisSpec::Vector([1.0, 2.0, 3.0])), origin: Some([1.0, 0.0, 0.0]), vertices: None, face: None }),
        Some(CopyXform::Scale { factor: 1.5, origin: None }),
        Some(CopyXform::Reflect { normal: [1.0, 2.0, 0.5], origin: Some([0.0, 1.0, 0.0]) }),
    ];
    both(|factory| {
        for (name, op) in &ops {
            for xf in &xforms {
                let mut s = session(factory);
                run(&mut s, op.as_ref()).unwrap_or_else(|e| panic!("{name}: {e:?}"));
                let (v0, a0) = (volumes(&s)[0], total_area(&s));
                let n0 = counts(&s);
                run(&mut s, &copy(vec![vol(1)], None, xf.clone())).unwrap_or_else(|e| panic!("{name} {xf:?}: {e:?}"));
                clean(&s);
                let k = match xf {
                    Some(CopyXform::Scale { factor, .. }) => *factor,
                    _ => 1.0,
                };
                let v = volumes(&s);
                assert_eq!(v.len(), 2, "{name}");
                assert!(near(v[1], v0 * k.powi(3), 1e-11), "{name} {xf:?}: {} vs {}", v[1], v0 * k.powi(3));
                assert!(near(total_area(&s) - a0, a0 * k * k, 1e-11), "{name} {xf:?}: area");
                assert_eq!(counts(&s)[..3].iter().zip(&n0).map(|(a, b)| a - b).collect::<Vec<_>>(), n0[..3].to_vec(), "{name}: the copy has the same cell counts");
                if name != &"sphere" && name != &"hollow sphere" {
                    assert!(worst_outwardness(&s) > 0.0, "{name} {xf:?}");
                }
            }
        }
    });
}

#[test]
fn merged_pair_copies_keep_the_shared_face_shared() {
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = session(factory);
        run(&mut s, &common::MergedBricks {}).expect("merged");
        assert_eq!(counts(&s), vec![12, 20, 11, 2, 2]);
        run(&mut s, &copy(vec![vol(1), vol(2)], None, mv([0.0, 5.0, 0.0]))).expect("copy pair");
        assert_eq!(counts(&s), vec![24, 40, 22, 4, 4], "11 faces per pair, not 12");
        clean(&s);
        assert!(worst_outwardness(&s) > 0.0);
        // One volume of the pair: legal, the copy owns its cells.
        let mut s = session(factory);
        run(&mut s, &common::MergedBricks {}).expect("merged");
        run(&mut s, &copy(vec![vol(2)], None, mv([0.0, 5.0, 0.0]))).expect("copy one");
        assert_eq!(counts(&s), vec![20, 32, 17, 3, 3]);
        clean(&s);
        assert!(worst_outwardness(&s) > 0.0);
        // The same pair as bodies: two bodies, two new bodies.
        let mut s = session(factory);
        run(&mut s, &common::MergedBricks {}).expect("merged");
        run(&mut s, &copy(vec![id(EntityKind::TopoBody, 1), id(EntityKind::TopoBody, 2)], None, mv([0.0, 5.0, 0.0]))).expect("copy bodies");
        assert_eq!(counts(&s), vec![24, 40, 22, 4, 4]);
        // The webcut pair: two volumes of ONE body copy as one body with two volumes.
        let mut s = session(factory);
        run(&mut s, &common::WebcutBrick {}).expect("webcut");
        run(&mut s, &copy(vec![id(EntityKind::TopoBody, 1)], None, mv([5.0, 0.0, 0.0]))).expect("copy webcut");
        assert_eq!(counts(&s), vec![24, 40, 22, 4, 2]);
        clean(&s);
    }
}

#[test]
fn a_meshed_volume_copies_without_mesh_or_attributes() {
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = session(factory);
        run(&mut s, &CreateBrick { x: 1.0, y: None, z: None, center: Some([0.5, 0.5, 0.5]) }).expect("unit cube");
        run(&mut s, &common::MirrorDeck { face: id(EntityKind::TopoFace, 1), normal: [1.0, 0.0, 0.0], offset: 0.0 }).expect("colors and a boundary");
        run(&mut s, &common::FakeMesh { volume: vol(1) }).expect("mesh");
        let layer_colors = model(&s).model().volume_color(vol(1));
        assert!(layer_colors.is_some());
        let (_, d) = run(&mut s, &copy(vec![vol(1)], None, mv([3.0, 0.0, 0.0]))).expect("copy of a meshed volume");
        let new = d.outputs["volumes"][0];
        assert!(s.state().layer::<MeshLayer>().expect("meshes").mesh_of(new).is_none(), "the copy has no mesh");
        assert!(s.state().layer::<MeshLayer>().expect("meshes").mesh_of(vol(1)).is_some(), "the source keeps its mesh");
        assert_eq!(model(&s).model().volume_color(new), None, "no colors");
        assert_eq!(model(&s).model().volume_color(vol(1)), layer_colors);
        for g in model(&s).model().groups() {
            assert!(!format!("{g:?}").contains(&new.to_string()), "no groups");
        }
        clean(&s);
    }
}

#[test]
fn counts_outside_one_to_4096_are_invalid() {
    let mut s = session(cell_complex);
    run(&mut s, &brick10()).expect("brick");
    let before = s.state().hash();
    for count in [0, 4097, u32::MAX] {
        let e = run(&mut s, &copy(vec![vol(1)], Some(count), None)).expect_err("count");
        assert!(format!("{e:?}").contains("Invalid"), "{e:?}");
        assert_eq!(s.state().hash(), before);
    }
    for op in [copy(vec![], None, None), copy(vec![id(EntityKind::TopoFace, 1)], None, None), copy(vec![vol(9)], None, None)] {
        run(&mut s, &op).expect_err("refused");
        assert_eq!(s.state().hash(), before);
    }
    // A bad xform leaves the state alone too.
    let e = run(&mut s, &copy(vec![vol(1)], None, Some(CopyXform::Scale { factor: -1.0, origin: None }))).expect_err("scale");
    assert!(format!("{e:?}").contains("Invalid"));
    let e = run(&mut s, &copy(vec![vol(1)], None, Some(CopyXform::Rotate { angle_deg: 1.0, axis: None, origin: None, vertices: None, face: None }))).expect_err("no form");
    assert!(format!("{e:?}").contains("Invalid"));
    assert_eq!(s.state().hash(), before);
    run(&mut s, &copy(vec![vol(1)], Some(4096), None)).map(|_| ()).expect("4096 copies");
    assert_eq!(counts(&s)[3], 4097);
}

#[test]
fn rotate_copies_about_a_vertex_pair_land_on_the_analytic_positions() {
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &CreateBrick { x: 2.0, y: None, z: None, center: None }).expect("axis cube");
        run(&mut s, &CreateBrick { x: 2.0, y: None, z: None, center: Some([3.0, 0.0, 0.0]) }).expect("subject");
        // Cube 1's vertices 1 and 5 span the vertical edge through (-1, -1).
        let xf = CopyXform::Rotate { angle_deg: 90.0, axis: None, origin: None, vertices: Some([id(EntityKind::TopoVertex, 1), id(EntityKind::TopoVertex, 5)]), face: None };
        run(&mut s, &copy(vec![vol(2)], Some(4), Some(xf))).expect("copies");
        // Offset of the subject from the pivot (-1, -1) is (4, 1); each turn is (x, y) -> (-y, x).
        let want = [[-2.0, 3.0], [-5.0, -2.0], [0.0, -5.0], [3.0, 0.0]];
        for (k, w) in want.iter().enumerate() {
            let c = centroid(&s, vol(3 + k as u64));
            assert!((c[0] - w[0]).abs() < 1e-11 && (c[1] - w[1]).abs() < 1e-11 && c[2].abs() < 1e-11, "copy {}: {c:?} vs {w:?}", k + 1);
        }
        clean(&s);
        assert!(worst_outwardness(&s) > 0.0);
    });
}

#[test]
fn scale_copies_compose_powers_of_the_factor() {
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &CreateSphere { radius: 1.0, center: Some([1.0, 0.0, 0.0]) }).expect("sphere");
        run(&mut s, &copy(vec![vol(1)], Some(3), Some(CopyXform::Scale { factor: 2.0, origin: None }))).expect("copies");
        clean(&s);
        let t = model(&s).topo();
        for k in 1..=3u64 {
            let c = t.volume_moments(vol(1 + k));
            // Sphere faces touching poles integrate fine through the quadrature.
            let v = c.expect("moments").volume;
            let want = 4.0 / 3.0 * PI * 8f64.powi(k as i32);
            assert!(near(v, want, 1e-10), "copy {k}: {v} vs {want}");
        }
    });
}
