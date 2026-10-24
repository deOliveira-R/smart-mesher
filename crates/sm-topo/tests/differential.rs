//! Deterministic differential tests: the sm-testkit topology fixtures, the reference fixture
//! catalogue, pole splits and the decomposition overlay, each run against both implementations
//! with identical ids, lineage, validation results and canonical hashes demanded.

mod common;

use common::{Twin, same_state};
use core::f64::consts::{FRAC_PI_2, PI};
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{Circle, CurveDef, Frame, Interval};
use sm_testkit::{broken, check, determinism, topo};
use sm_topo::TopoStore;
use sm_topo_api::fixtures::{self, FixtureError, with_op};
use sm_topo_api::*;
use std::sync::Arc;

fn fx<T>(r: Result<T, FixtureError>) -> Result<T, TopoError> {
    r.map_err(|e| match e {
        FixtureError::Topo(t) => t,
        FixtureError::Geom(g) => TopoError::Geom(g),
    })
}

fn build(f: impl FnOnce(&mut TopoStore) -> Result<(), FixtureError>) -> TopoStore {
    let mut t = TopoStore::new();
    t.set_paranoid(true);
    f(&mut t).expect("fixture builds");
    t
}

/// Every sm-testkit topology fixture, rebuilt on `TopoStore` with the same op ids, must equal the
/// testkit's reference complex exactly and satisfy the fixture's own expectations.
#[test]
fn testkit_fixtures_are_identical_and_satisfy_their_expectations() {
    let mut cases: Vec<(topo::TopoFixture, TopoStore)> = Vec::new();
    let b = |t: &mut TopoStore, op: u64, min: [f64; 3], size: [f64; 3]| with_op(t, OpId(op), |t| fixtures::brick(t, Point3::new(min[0], min[1], min[2]), size));
    cases.push((topo::cube().unwrap(), build(|t| b(t, 1, [0.0; 3], [1.0; 3]).map(|_| ()))));
    cases.push((topo::brick([-1.0, 2.0, 0.5], [2.0, 3.0, 4.0]).unwrap(), build(|t| b(t, 1, [-1.0, 2.0, 0.5], [2.0, 3.0, 4.0]).map(|_| ()))));
    cases.push((topo::cylinder(1.0, 2.0).unwrap(), build(|t| with_op(t, OpId(1), |t| fixtures::cylinder(t, 1.0, 2.0)).map(|_| ()))));
    cases.push((topo::sphere(1.5).unwrap(), build(|t| with_op(t, OpId(1), |t| fixtures::sphere(t, 1.5)).map(|_| ()))));
    cases.push((
        topo::merged_bricks().unwrap(),
        build(|t| {
            let (a, bb) = with_op(t, OpId(1), fixtures::two_bricks)?;
            with_op(t, OpId(2), |t| fixtures::merge_touching_bricks(t, &a, &bb))
        }),
    ));
    cases.push((
        topo::webcut_cube(0.25).unwrap(),
        build(|t| {
            let br = b(t, 1, [0.0; 3], [1.0; 3])?;
            with_op(t, OpId(2), |t| fixtures::webcut_brick(t, &br, 0.25)).map(|_| ())
        }),
    ));
    cases.push((broken::brick_with_fin().unwrap(), build(|t| with_op(t, OpId(1), fixtures::brick_with_fin).map(|_| ()))));
    cases.push((broken::brick_with_baffle().unwrap(), build(|t| with_op(t, OpId(1), fixtures::brick_with_baffle).map(|_| ()))));
    cases.push((
        broken::two_disjoint_bricks().unwrap(),
        build(|t| {
            with_op(t, OpId(1), |t| {
                fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3])?;
                fixtures::brick(t, Point3::new(5.0, 0.0, 0.0), [1.0; 3])?;
                Ok(())
            })
        }),
    ));
    cases.push((
        broken::brick_with_moved_vertex().unwrap(),
        build(|t| {
            let br = b(t, 1, [0.0; 3], [1.0; 3])?;
            with_op(t, OpId(2), |t| Ok(t.set_vertex_point(br.vertices[0], Point3::new(-0.3, -0.2, -0.1))?))
        }),
    ));
    for (fixture, mine) in &cases {
        same_state(&fixture.complex, mine, fixture.name);
        assert_eq!(Dim::ALL.map(|d| mine.cells(d).len()), fixture.counts, "{} counts", fixture.name);
        assert_eq!(euler_characteristic(mine), fixture.euler, "{} euler", fixture.name);
        // Same verdicts from the testkit's own checkers.
        assert_eq!(check::topo_valid(mine).is_ok(), check::topo_valid(&fixture.complex).is_ok(), "{}", fixture.name);
        assert_eq!(check::manifold_edges(mine).is_ok(), check::manifold_edges(&fixture.complex).is_ok(), "{}", fixture.name);
        assert_eq!(check::check_topo_euler(mine, fixture.euler).is_ok(), check::check_topo_euler(&fixture.complex, fixture.euler).is_ok());
        assert_eq!(check::topo_hash(mine), check::topo_hash(&fixture.complex), "{}", fixture.name);
        // Analytic expectations where the reference can integrate (planar faces, cylinder).
        if fixture.name != "sphere" {
            let vols: Vec<f64> = mine.cells(Dim::Volume).iter().map(|v| mine.volume_moments(*v).map(|m| m.volume).unwrap_or(f64::NAN)).collect();
            let mut want = fixture.volumes.clone();
            want.sort_by(f64::total_cmp);
            let mut got = vols.clone();
            got.sort_by(f64::total_cmp);
            for (g, w) in got.iter().zip(&want) {
                assert!((g - w).abs() < 1e-6 * w.max(1.0), "{}: volume {g} vs {w}", fixture.name);
            }
        }
    }
}

#[test]
fn testkit_topology_checks_pass_on_the_fixtures_and_fail_where_expected() {
    let cube = build(|t| with_op(t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3])).map(|_| ()));
    assert!(check::topo_valid(&cube).is_ok());
    assert!(check::check_topo_euler(&cube, 2).is_ok());
    assert!(check::manifold_edges(&cube).is_ok());
    let fin = build(|t| with_op(t, OpId(1), fixtures::brick_with_fin).map(|_| ()));
    assert!(check::topo_valid(&fin).is_ok());
    assert!(!check::manifold_edges(&fin).is_ok(), "a fin edge has three faces");
    let moved = build(|t| {
        let b = with_op(t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3]))?;
        with_op(t, OpId(2), |t| Ok(t.set_vertex_point(b.vertices[0], Point3::new(-0.3, -0.2, -0.1))?))
    });
    assert!(!check::topo_valid(&moved).is_ok(), "validate_geometry sees the moved vertex");
}

/// The reference fixture catalogue, op by op, in lock step.
#[test]
fn fixture_catalogue_in_lock_step() {
    let mut tw = Twin::new();
    let a = tw.op("brick", 1, |t| fx(fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 2.0, 3.0]))).unwrap();
    let w = tw.op("webcut", 2, |t| fx(fixtures::webcut_brick(t, &a, 0.37)).map(|w| (w.volumes, w.cap))).unwrap();
    assert_eq!(w.0.len(), 2);
    let (p, q) = tw.op("two bricks", 3, |t| fx(fixtures::two_bricks(t)).map(|(p, q)| (p.volume, q.volume))).unwrap();
    let _ = (p, q);
    tw.op("cylinder", 4, |t| fx(fixtures::cylinder(t, 0.7, 2.0)).map(|c| c.volume)).unwrap();
    tw.op("sphere", 5, |t| fx(fixtures::sphere(t, 1.3)).map(|c| c.volume)).unwrap();
    tw.op("baffle", 6, |t| fx(fixtures::brick_with_baffle(t)).map(|c| c.1)).unwrap();
    tw.op("fin", 7, |t| fx(fixtures::brick_with_fin(t)).map(|c| c.1)).unwrap();
}

#[test]
fn merge_and_unmerge_touching_bricks_in_lock_step() {
    for gap in [None, Some(1e-4)] {
        let mut tw = Twin::new();
        let (a, b) = tw.op("two bricks", 1, |t| fx(fixtures::two_bricks(t))).unwrap();
        tw.op("merge", 2, |t| fx(fixtures::merge_touching_bricks_gap(t, &a, &b, gap))).unwrap();
        let shared = a.faces[1];
        tw.op("unmerge", 3, |t| {
            let nf = t.unmerge_face(shared, Side::Right)?;
            let b_faces = |t: &dyn TopologyEdit| -> Vec<EntityId> { t.volume_faces(b.volume).to_vec() };
            for e in t.face_edges(shared).to_vec() {
                let bf = b_faces(t);
                let mut fs: Vec<EntityId> = t.coboundary(e).iter().map(|i| i.cell).filter(|f| bf.contains(f)).collect();
                fs.dedup();
                t.unmerge_edge(e, &fs)?;
            }
            for &v in &[a.vertices[1], a.vertices[3], a.vertices[5], a.vertices[7]] {
                let bf = b_faces(t);
                let es: Vec<EntityId> = t.coboundary(v).iter().map(|i| i.cell).filter(|e| t.coboundary(*e).iter().all(|f| bf.contains(&f.cell))).collect();
                t.unmerge_vertex(v, &es)?;
            }
            Ok(nf)
        })
        .unwrap();
    }
}

#[test]
fn sphere_meridian_cut_in_lock_step() {
    let r = 2.0;
    let mut tw = Twin::new();
    let s = tw.op("sphere", 1, |t| fx(fixtures::sphere(t, r))).unwrap();
    let [pole_s, _seam, pole_n] = s.edges;
    let [vs, vn] = s.vertices;
    let face = s.face;
    // Invalid degenerate splits first (atomic failures).
    assert!(tw.op("bad split", 2, |t| t.split_degenerate_edge(s.edges[1], 0.0)).is_err());
    assert!(tw.op("bad range", 3, |t| t.split_degenerate_edge(pole_s, 7.0)).is_err());
    let out = tw
        .op("meridian", 4, |t| {
            let [h1, _h2] = t.split_degenerate_edge(pole_s, PI)?;
            let [_n1, n2] = t.split_degenerate_edge(pole_n, PI)?;
            let frame = Frame::from_zx(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0), Vector3::new(-1.0, 0.0, 0.0))?;
            let curve = t.shapes_mut()?.add_curve(CurveDef::Circle(Circle::new(frame, r)?))?;
            let chord = t.make_edge(vs, vn, Some(CurveCarrier { curve, range: Interval::new(-FRAC_PI_2, FRAC_PI_2), sense: Sense::Forward }))?;
            let corner = |edge, side| CornerKey { face, incoming: CoedgeKey { face, edge, side } };
            let plain = t.split_face(face, FaceCut::Chord { edge: chord }, InnerLoopAssign::None);
            assert!(matches!(plain, Err(TopoError::Unsupported(_))));
            let wrong = t.split_face(face, FaceCut::ChordAt { edge: chord, start: corner(n2, Side::Right), end: corner(h1, Side::Left) }, InnerLoopAssign::None);
            assert!(wrong.is_err());
            t.split_face(face, FaceCut::ChordAt { edge: chord, start: corner(h1, Side::Left), end: corner(n2, Side::Right) }, InnerLoopAssign::None)
        })
        .unwrap();
    assert_eq!(out.len(), 2);
}

/// Overlay interplay: the same decomposition over `Arc<TopoStore>` and over `Arc<CellComplex>`.
#[test]
fn overlay_over_both_hosts_is_identical() {
    let mut a = CellComplex::new();
    let mut b = TopoStore::new();
    let bi = with_op(&mut a, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 2.0, 3.0])).unwrap();
    with_op(&mut b, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 2.0, 3.0])).unwrap();
    with_op(&mut a, OpId(2), |t| fixtures::cylinder(t, 0.5, 1.0)).unwrap();
    with_op(&mut b, OpId(2), |t| fixtures::cylinder(t, 0.5, 1.0)).unwrap();
    same_state(&a, &b, "hosts");
    let (ha, hb): (Arc<dyn TopologyView>, Arc<dyn TopologyView>) = (Arc::new(a), Arc::new(b));
    let (mut oa, mut ob) = (Overlay::new(ha.clone()), Overlay::new(hb.clone()));
    let cell = |n: u64| EntityId::new(sm_core::EntityKind::PartitionCell, n).unwrap();
    let ca = oa.partition_volume_by_plane(bi.volume, Point3::new(0.0, 0.0, 1.2), Vector3::new(0.0, 0.0, 1.0), [cell(10), cell(11)]);
    let cb = ob.partition_volume_by_plane(bi.volume, Point3::new(0.0, 0.0, 1.2), Vector3::new(0.0, 0.0, 1.0), [cell(10), cell(11)]);
    assert_eq!(format!("{ca:?}"), format!("{cb:?}"));
    let compare = |oa: &Overlay, ob: &Overlay, ctx: &str| {
        assert_eq!(oa.canonical_hash(), ob.canonical_hash(), "{ctx}");
        for d in Dim::ALL {
            assert_eq!(oa.cells(d), ob.cells(d), "{ctx}: {d:?}");
            for &c in oa.cells(d) {
                assert_eq!(oa.boundary(c), ob.boundary(c), "{ctx}: boundary {c:?}");
                assert_eq!(oa.coboundary(c), ob.coboundary(c), "{ctx}: coboundary {c:?}");
                assert_eq!(oa.carrier(c), ob.carrier(c), "{ctx}: carrier {c:?}");
                assert_eq!(oa.radial_uses(c), ob.radial_uses(c), "{ctx}: radial {c:?}");
                assert_eq!(oa.volume_faces(c), ob.volume_faces(c));
                assert_eq!(oa.face_edges(c), ob.face_edges(c));
            }
        }
        assert_eq!(validate(oa), validate(ob), "{ctx}: validate");
    };
    compare(&oa, &ob, "after plane cut");
    // Partition one cylinder edge and a composite of two brick faces.
    let edges = oa.host().cells(Dim::Edge).to_vec();
    let e = edges[edges.len() - 1];
    let (ra, rb) = (oa.partition_edge(e, 0.3), ob.partition_edge(e, 0.3));
    assert_eq!(format!("{ra:?}"), format!("{rb:?}"));
    compare(&oa, &ob, "after partition_edge");
    // The overlay data round-trips identically over both hosts.
    let (da, db) = (oa.data(), ob.data());
    assert_eq!(format!("{da:?}"), format!("{db:?}"));
    let (ra, rb) = (Overlay::from_data(ha, da).unwrap(), Overlay::from_data(hb, db).unwrap());
    compare(&ra, &rb, "replayed");
    // Host untouched by the overlay.
    assert_eq!(oa.host().canonical_hash(), ob.host().canonical_hash());
}

#[test]
fn building_is_deterministic_across_runs_and_threads() {
    fn build_model(threads: usize) -> u64 {
        // A pool of `threads` workers builds the same model; the result must not depend on it.
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| {
            let mut t = TopoStore::new();
            let a = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])).unwrap();
            let (p, q) = with_op(&mut t, OpId(2), fixtures::two_bricks).unwrap();
            with_op(&mut t, OpId(3), |t| fixtures::merge_touching_bricks(t, &p, &q)).unwrap();
            with_op(&mut t, OpId(4), |t| fixtures::webcut_brick(t, &a, 0.4)).unwrap();
            with_op(&mut t, OpId(5), |t| fixtures::cylinder(t, 0.5, 1.0)).unwrap();
            t.canonical_hash() ^ (t.validate_par().len() as u64)
        })
    }
    let h = determinism::check(2, 4, build_model).expect("deterministic");
    let mut r = TopoStore::new();
    let _ = &mut r;
    assert_ne!(h, 0);
}

/// Polyline and NURBS pcurves, weights included, hash to the same words on both stores (the
/// store emits the canonical stream with its own traversal).
#[test]
fn pcurve_variants_emit_identical_canonical_words() {
    use sm_geom_api::{NurbsCurve2, PCurve, Uv};
    let mut tw = Twin::new();
    let b = tw.op("brick", 1, |t| fx(fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 2.0, 3.0]))).unwrap();
    let keys: Vec<CoedgeKey> = tw.a.boundary(b.faces[0]).iter().map(|e| CoedgeKey { face: b.faces[0], edge: e.cell, side: e.side }).collect();
    let poly = PCurve::Polyline2(vec![Uv::new(0.0, 0.0), Uv::new(2.0, 0.5), Uv::new(2.0, 4.0)]);
    let ctrl = vec![Uv::new(0.0, 0.0), Uv::new(1.0, 2.0), Uv::new(3.0, 2.5), Uv::new(4.0, 0.0)];
    let nurbs = NurbsCurve2::new(2, vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0], ctrl.clone(), Some(vec![1.0, 2.0, 0.5, 1.0])).unwrap();
    let plain = NurbsCurve2::new(2, vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0], ctrl, None).unwrap();
    tw.op("pcurves", 2, |t| {
        t.set_pcurve(keys[0], Some(poly.clone()))?;
        t.set_pcurve(keys[1], Some(PCurve::Nurbs2(nurbs.clone())))?;
        t.set_pcurve(keys[2], Some(PCurve::Nurbs2(plain.clone())))?;
        Ok(())
    })
    .unwrap();
}
