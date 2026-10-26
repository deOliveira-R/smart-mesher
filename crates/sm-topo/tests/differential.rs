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
    let b = |t: &mut TopoStore, op: u64, min: [f64; 3], size: [f64; 3]| with_op(t, OpId(op), |t| fixtures::brick(t, Point3::new(min[0], min[1], min[2]), size));
    // One rebuild per fixture of `topo::all_topo_fixtures()`, in its order; the name and count
    // checks below make a new or reordered testkit fixture fail here instead of going untested.
    type Rebuild = Box<dyn Fn() -> TopoStore>;
    let rebuilds: Vec<(&str, Rebuild)> = vec![
        ("cube", Box::new(move || build(|t| b(t, 1, [0.0; 3], [1.0; 3]).map(|_| ())))),
        ("brick", Box::new(move || build(|t| b(t, 1, [-1.0, 2.0, 0.5], [2.0, 3.0, 4.0]).map(|_| ())))),
        ("cylinder", Box::new(|| build(|t| with_op(t, OpId(1), |t| fixtures::cylinder(t, 1.0, 2.0)).map(|_| ())))),
        ("sphere", Box::new(|| build(|t| with_op(t, OpId(1), |t| fixtures::sphere(t, 1.5)).map(|_| ())))),
        (
            "merged_bricks",
            Box::new(|| {
                build(|t| {
                    let (a, bb) = with_op(t, OpId(1), fixtures::two_bricks)?;
                    with_op(t, OpId(2), |t| fixtures::merge_touching_bricks(t, &a, &bb))
                })
            }),
        ),
        (
            "webcut_cube",
            Box::new(move || {
                build(|t| {
                    let br = b(t, 1, [0.0; 3], [1.0; 3])?;
                    with_op(t, OpId(2), |t| fixtures::webcut_brick(t, &br, 0.25)).map(|_| ())
                })
            }),
        ),
        ("torus", Box::new(|| build(|t| with_op(t, OpId(1), |t| fixtures::torus(t, 5.0, 1.0)).map(|_| ())))),
        ("cone", Box::new(|| build(|t| with_op(t, OpId(1), |t| fixtures::cone(t, 2.0, 3.0)).map(|_| ())))),
        (
            "brick_with_void",
            Box::new(|| build(|t| with_op(t, OpId(1), |t| fixtures::brick_with_void(t, Point3::new(0.0, 0.0, 0.0), [10.0; 3], Point3::new(4.0, 4.0, 4.0), [2.0; 3])).map(|_| ()))),
        ),
        ("nested_spheres", Box::new(|| build(|t| with_op(t, OpId(1), |t| fixtures::nested_spheres(t, &[1.0, 2.0, 3.0])).map(|_| ())))),
    ];
    let all = topo::all_topo_fixtures().unwrap();
    assert_eq!(all.len(), rebuilds.len(), "all_topo_fixtures changed: add a rebuild");
    let mut cases: Vec<(topo::TopoFixture, TopoStore)> = Vec::new();
    for (fixture, (name, rebuild)) in all.into_iter().zip(&rebuilds) {
        assert_eq!(fixture.name, *name, "all_topo_fixtures order changed");
        cases.push((fixture, rebuild()));
    }
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
        // Many issues per cell on the broken fixtures: the parallel validator orders them alike.
        assert_eq!(mine.validate_par(), validate(mine), "{} validate_par", fixture.name);
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

/// The topology of the testkit's model fixtures (pin lattice stub, mirror chamber) rebuilt on
/// `TopoStore`: identical state, and the cut's lineage outcome (webcut + kill_cascade in one op)
/// is the same on both.
#[test]
fn testkit_model_fixture_topology_is_identical() {
    let pin = topo::pin_lattice_stub().unwrap();
    let mine = build(|t| {
        with_op(t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [0.4, 0.4, 1.0]))?;
        with_op(t, OpId(2), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 1.0), [0.4, 0.4, 1.0]))?;
        Ok(())
    });
    same_state(&pin.complex, &mine, pin.name);
    assert_eq!(mine.validate_par(), validate(&mine));
    assert!(validate(&mine).is_empty());

    let mirror = topo::mirror_chamber().unwrap();
    let mine = build(|t| {
        let br = with_op(t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3]))?;
        with_op(t, OpId(2), |t| {
            let w = fixtures::webcut_brick(t, &br, 0.5)?;
            t.kill_cascade(w.volumes[1])?;
            Ok(())
        })
    });
    same_state(&mirror.complex, &mine, mirror.name);
    let (want, got) = (mirror.complex.lineage().cut_outcome(OpId(2)), mine.lineage().cut_outcome(OpId(2)));
    assert_eq!(format!("{want:?}"), format!("{got:?}"), "cut_outcome");
    assert_eq!(mine.validate_par(), validate(&mine));
    let vol: f64 = mine.cells(Dim::Volume).iter().map(|v| mine.volume_moments(*v).unwrap().volume).sum();
    assert!((vol - mirror.volume).abs() < 1e-9);
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

/// A DEGENERATE toggle is checked at `end_op` (the edge, its faces and volumes), so a pole edge
/// cannot lose its flag and commit an open shell: the clearing op fails identically on both,
/// rolls back, and later ops (touching the face or volume, or the flag again) all pass.
fn pole_flag_clear_then_touch(prod: bool, touch: u8) {
    let mut tw = if prod { Twin::production() } else { Twin::new() };
    let s = tw.op("sphere", 1, |t| fx(fixtures::sphere(t, 1.0))).unwrap();
    let before = tw.a.canonical_hash();
    let r = tw.op("clear pole flag", 2, |t| t.set_flags(s.edges[0], CellFlags::NONE));
    assert!(matches!(r, Err(TopoError::OpInvalid(_))), "{r:?}");
    assert_eq!(tw.a.canonical_hash(), before, "rolled back");
    assert_eq!(tw.b.canonical_hash(), before, "rolled back (store)");
    assert!(validate(&tw.a).is_empty(), "the committed state stays valid");
    assert_eq!(tw.b.validate_par(), validate(&tw.b));
    tw.op("unrelated", 3, |t| t.make_vertex(Point3::new(9.0, 9.0, 9.0)).map(|_| ())).unwrap();
    tw.op("touch", 4, |t| match touch {
        0 => {
            let c = t.carrier(s.face).clone();
            t.set_carrier(s.face, c)
        }
        1 => t.set_carrier(s.volume, Carrier::None),
        _ => t.set_flags(s.edges[0], CellFlags::DEGENERATE),
    })
    .unwrap();
    assert_eq!(tw.b.validate_par(), validate(&tw.b));
}

#[test]
fn a_rejected_pole_flag_clear_leaves_no_trace_for_later_ops() {
    for prod in [false, true] {
        for touch in 0..3 {
            pole_flag_clear_then_touch(prod, touch);
        }
    }
}

/// Restoring the flag after a rejected clear is a plain no-op edit.
#[test]
fn pole_flag_restored_after_a_rejected_clear_commits() {
    let mut tw = Twin::new();
    let s = tw.op("sphere", 1, |t| fx(fixtures::sphere(t, 1.0))).unwrap();
    assert!(tw.op("clear", 2, |t| t.set_flags(s.edges[0], CellFlags::NONE)).is_err());
    tw.op("restore", 3, |t| t.set_flags(s.edges[0], CellFlags::DEGENERATE)).unwrap();
    assert!(validate(&tw.a).is_empty());
    tw.op("touch", 4, |t| t.set_carrier(s.volume, Carrier::None)).unwrap();
}

/// Adversarial ops, with the long-row paths forced on the sphere's one-entry volume row, agree
/// with the reference whether accepted or rejected, after a rejected flag clear.
#[test]
fn ops_after_a_rejected_flag_clear_agree_with_the_reference() {
    for case in 0..8u8 {
        let mut tw = Twin::new();
        tw.b.set_row_thresholds(1, 1);
        let s = tw.op("sphere", 1, |t| fx(fixtures::sphere(t, 1.0))).unwrap();
        assert!(tw.op("clear pole flag", 2, |t| t.set_flags(s.edges[0], CellFlags::NONE)).is_err());
        let r = tw.op("adversarial", 3, |t| match case {
            0 => t.reverse_face(s.face),
            1 => t.kill_cascade(s.volume).map(|_| ()),
            2 => t.remove_face_use(s.volume, s.face, Side::Left),
            3 => t.set_flags(s.edges[1], CellFlags::NONE),
            4 => t.raise_tolerance(s.edges[0], 1e-6),
            5 => t.set_vertex_point(s.vertices[0], Point3::new(0.0, 0.0, -1.0)),
            6 => t.kill(s.volume),
            _ => {
                let v = t.make_vertex(Point3::new(0.0, 0.0, 5.0))?;
                t.embed(s.volume, v)
            }
        });
        eprintln!("case {case}: {:?}", r.map(|_| ()));
        // The next op on the (possibly changed) state also agrees.
        let r = tw.op("touch volume", 4, |t| {
            let c = t.carrier(s.volume).clone();
            t.set_carrier(s.volume, c)
        });
        eprintln!("case {case} touch: {:?}", r.map(|_| ()));
    }
}

// ---------------------------------------------------------------- directed shell-closure regressions

/// Both settings: the small-threshold paranoid twin and the production twin.
fn twins() -> [(&'static str, Twin); 2] {
    [("paranoid", Twin::new()), ("production", Twin::production())]
}

fn touch_volume(tw: &mut Twin, op: u64, v: EntityId) -> Result<(), TopoError> {
    tw.op("touch volume", op, |t| {
        let c = t.carrier(v).clone();
        t.set_carrier(v, c)
    })
}

fn is_open_shell(r: &Result<(), TopoError>) -> bool {
    matches!(r, Err(TopoError::OpInvalid(_)))
}

/// (a) A rejected clear leaves nothing behind: an aborted touch, further touches and six
/// unrelated commits all behave as on a never-modified sphere.
#[test]
fn rejected_clear_survives_aborts_touches_and_unrelated_commits() {
    for (setting, mut tw) in twins() {
        let s = tw.op("sphere", 1, |t| fx(fixtures::sphere(t, 1.0))).unwrap();
        assert!(is_open_shell(&tw.op("open", 2, |t| t.set_flags(s.edges[0], CellFlags::NONE))), "{setting}");
        // Aborted touch (the edit succeeds, the op is abandoned).
        tw.run(setting, |t| t.begin_op(OpId(3))).unwrap();
        tw.run(setting, |t| {
            let c = t.carrier(s.volume).clone();
            t.set_carrier(s.volume, c)
        })
        .unwrap();
        tw.run(setting, |t| t.abort_op());
        tw.same(setting);
        touch_volume(&mut tw, 4, s.volume).unwrap_or_else(|e| panic!("{setting}: first touch: {e:?}"));
        for n in 0..6u64 {
            tw.op("unrelated", 5 + n, |t| t.make_vertex(Point3::new(9.0 + n as f64, 0.0, 0.0)).map(|_| ())).unwrap();
        }
        touch_volume(&mut tw, 11, s.volume).unwrap_or_else(|e| panic!("{setting}: touch after unrelated commits: {e:?}"));
        assert!(validate(&tw.a).is_empty(), "{setting}: valid");
    }
}

/// (b) Clearing either pole alone, or both in one op, is rejected; restoring flags that are
/// already set commits.
#[test]
fn clearing_either_or_both_poles_is_rejected() {
    for (setting, mut tw) in twins() {
        let s = tw.op("sphere", 1, |t| fx(fixtures::sphere(t, 1.0))).unwrap();
        let (p0, p1) = (s.edges[0], s.edges[2]);
        assert!(is_open_shell(&tw.op("clear 0", 2, |t| t.set_flags(p0, CellFlags::NONE))), "{setting}");
        assert!(is_open_shell(&tw.op("clear 1", 3, |t| t.set_flags(p1, CellFlags::NONE))), "{setting}");
        let both = tw.op("clear both", 4, |t| {
            t.set_flags(p0, CellFlags::NONE)?;
            t.set_flags(p1, CellFlags::NONE)
        });
        assert!(is_open_shell(&both), "{setting}");
        tw.op("restore 0", 5, |t| t.set_flags(p0, CellFlags::DEGENERATE)).unwrap();
        tw.op("restore 1", 6, |t| t.set_flags(p1, CellFlags::DEGENERATE)).unwrap();
        assert!(validate(&tw.a).is_empty(), "{setting}");
        touch_volume(&mut tw, 7, s.volume).unwrap_or_else(|e| panic!("{setting}: touch must pass: {e:?}"));
    }
}

/// (c) A rejected clear on sphere 1 does not disturb edits of sphere 2, which is itself rejected
/// when opened and can be kill_cascaded.
#[test]
fn rejected_clears_on_two_spheres_are_independent() {
    for (setting, mut tw) in twins() {
        let s1 = tw.op("sphere 1", 1, |t| fx(fixtures::sphere(t, 1.0))).unwrap();
        let s2 = tw.op("sphere 2", 2, |t| fx(fixtures::sphere(t, 2.0))).unwrap();
        assert!(is_open_shell(&tw.op("open 1", 3, |t| t.set_flags(s1.edges[0], CellFlags::NONE))), "{setting}");
        touch_volume(&mut tw, 4, s2.volume).unwrap_or_else(|e| panic!("{setting}: sphere 2 touch: {e:?}"));
        touch_volume(&mut tw, 5, s1.volume).unwrap_or_else(|e| panic!("{setting}: sphere 1 touch: {e:?}"));
        assert!(is_open_shell(&tw.op("open 2", 6, |t| t.set_flags(s2.edges[0], CellFlags::NONE))), "{setting}");
        tw.op("kill 2", 7, |t| t.kill_cascade(s2.volume).map(|_| ())).unwrap();
        touch_volume(&mut tw, 8, s1.volume).unwrap_or_else(|e| panic!("{setting}: after killing sphere 2: {e:?}"));
    }
}

/// (d) Clearing a pole and touching the volume in one op fails and rolls back; a DEGENERATE
/// double toggle inside one op commits closed.
#[test]
fn clear_and_touch_in_one_op_fails_and_a_double_toggle_commits_closed() {
    for (setting, mut tw) in twins() {
        let s = tw.op("sphere", 1, |t| fx(fixtures::sphere(t, 1.0))).unwrap();
        let before = tw.a.canonical_hash();
        let r = tw.op("clear+touch", 2, |t| {
            t.set_flags(s.edges[0], CellFlags::NONE)?;
            let c = t.carrier(s.volume).clone();
            t.set_carrier(s.volume, c)
        });
        assert!(is_open_shell(&r), "{setting}: {r:?}");
        assert_eq!(tw.a.canonical_hash(), before, "{setting}: rolled back");
        assert!(validate(&tw.a).is_empty());
        tw.op("double toggle", 3, |t| {
            t.set_flags(s.edges[0], CellFlags::NONE)?;
            t.set_flags(s.edges[0], CellFlags::DEGENERATE)
        })
        .unwrap();
        assert!(validate(&tw.a).is_empty(), "{setting}: closed");
        touch_volume(&mut tw, 4, s.volume).unwrap_or_else(|e| panic!("{setting}: touch after double toggle: {e:?}"));
    }
}
