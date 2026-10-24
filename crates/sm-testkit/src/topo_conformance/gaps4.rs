//! Conformance scenarios: Face and volume geometry corner cases (spherical faces, pcurves, signatures).
#![allow(clippy::too_many_arguments, clippy::type_complexity, clippy::needless_range_loop)]


use core::f64::consts::{FRAC_PI_2, PI, TAU};
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{Circle, CurveDef, Ellipse, Frame, Interval, Line2, PCurve, Uv};
use sm_topo_api::fixtures::{self, with_op};
use super::{Mk, Scenario, Sys};
use sm_topo_api::*;

fn assert_valid(v: &dyn TopologyView) {
    let bad = validate(v);
    assert!(bad.is_empty(), "violations: {bad:#?}");
    let bad = validate_geometry(v);
    assert!(bad.is_empty(), "geometry violations: {bad:#?}");
}

/// An id-free structural signature: cell counts, sorted vertex positions, sorted face
/// (loop sizes, area, centroid), sorted volume (face count, volume, centroid), coincidence
/// statuses and tolerance overrides.
fn signature<E: Sys>(t: &E) -> Vec<String> {
    let q = |x: f64| (x * 1e9).round() as i64;
    let mut out = vec![format!("{:?}", Dim::ALL.map(|d| t.cells(d).len()))];
    let mut vs: Vec<String> = t
        .cells(Dim::Vertex)
        .iter()
        .map(|v| match t.carrier(*v) {
            Carrier::Point(p) => format!("v{:?}", (q(p.x), q(p.y), q(p.z), t.coboundary(*v).len())),
            _ => "v?".into(),
        })
        .collect();
    vs.sort();
    out.extend(vs);
    let mut fs: Vec<String> = t
        .cells(Dim::Face)
        .iter()
        .map(|f| {
            let m = t.face_moments(*f).unwrap();
            let loops: Vec<usize> = t.loops(*f).map(|l| l.len()).collect();
            let c = m.first;
            format!("f{loops:?} {} {:?} up{}", q(m.area), (q(c.x), q(c.y), q(c.z)), t.coboundary(*f).len())
        })
        .collect();
    fs.sort();
    out.extend(fs);
    let mut vols: Vec<String> = t
        .cells(Dim::Volume)
        .iter()
        .map(|v| {
            let m = t.volume_moments(*v).unwrap();
            format!("V{} {} {:?}", t.volume_faces(*v).len(), q(m.volume), (q(m.first.x), q(m.first.y), q(m.first.z)))
        })
        .collect();
    vols.sort();
    out.extend(vols);
    let mut st: Vec<String> = t.coincidences().iter().map(|p| format!("{:?}", p.status)).collect();
    st.sort();
    out.extend(st);
    let tol: Vec<String> = Dim::ALL.iter().flat_map(|d| t.cells(*d).iter()).filter_map(|c| t.tolerance_override(*c)).map(|x| format!("tol{}", q(x))).collect();
    out.extend(tol);
    out
}

pub(super) fn merge_then_unmerge_gives_the_never_merged_bricks<E: Sys>(make: &Mk<E>) {
    // Never merged: two touching bricks, imprinted, not merged.
    let mut plain = make(None);
    with_op(&mut plain, OpId(1), |t| {
        let (a, b) = fixtures::two_bricks(t)?;
        t.record_coincidence(a.faces[1], b.faces[0], Coincidence::ImprintedNotMerged)?;
        Ok(())
    })
    .unwrap();

    let mut t = make(None);
    let (a, b) = with_op(&mut t, OpId(1), fixtures::two_bricks).unwrap();
    with_op(&mut t, OpId(2), |t| fixtures::merge_touching_bricks(t, &a, &b)).unwrap();
    assert_ne!(signature(&t), signature(&plain));
    let shared = a.faces[1];
    let nf = with_op(&mut t, OpId(3), |t| {
        let nf = t.unmerge_face(shared, Side::Right)?;
        // Edges: B's side (the new face and B's other faces) gets a copy.
        let b_faces = |t: &E| -> Vec<EntityId> { t.volume_faces(b.volume).to_vec() };
        for e in t.face_edges(shared).to_vec() {
            let bf = b_faces(t);
            let mut fs: Vec<EntityId> = t.coboundary(e).iter().map(|i| i.cell).filter(|f| bf.contains(f)).collect();
            fs.dedup();
            t.unmerge_edge(e, &fs)?;
        }
        // Vertices: every edge used only by B's faces moves to the copy.
        for &v in &[a.vertices[1], a.vertices[3], a.vertices[5], a.vertices[7]] {
            let bf = b_faces(t);
            let es: Vec<EntityId> = t
                .coboundary(v)
                .iter()
                .map(|i| i.cell)
                .filter(|e| t.coboundary(*e).iter().all(|f| bf.contains(&f.cell)))
                .collect();
            t.unmerge_vertex(v, &es)?;
        }
        Ok(nf)
    })
    .unwrap();
    assert_valid(&t);
    assert_eq!(signature(&t), signature(&plain), "merge + unmerge = never merged, up to ids");
    let pair = t.coincidence(shared, nf).unwrap();
    assert_eq!(pair.status, Coincidence::ImprintedNotMerged);
    assert!(t.lineage().of(nf).any(|r| r.kind == LineageKind::Created));
    // Pcurves and the old face survive; the new face sits on B's side.
    assert!(t.is_alive(shared));
    assert!(t.volume_faces(b.volume).contains(&nf) && !t.volume_faces(b.volume).contains(&shared));
}

pub(super) fn unmerge_operators_check_their_inputs<E: Sys>(make: &Mk<E>) {
    let mut t = make(None);
    let (a, b) = with_op(&mut t, OpId(1), fixtures::two_bricks).unwrap();
    let before = t.canonical_hash();
    t.begin_op(OpId(2)).unwrap();
    assert!(matches!(t.unmerge_face(a.faces[1], Side::Right), Err(TopoError::InvalidParameter(_))), "no volume on the right");
    assert!(matches!(t.unmerge_face(a.faces[1], Side::Both), Err(TopoError::InvalidParameter(_))));
    assert!(matches!(t.unmerge_edge(a.edges[0], &[]), Err(TopoError::InvalidParameter(_))));
    assert!(matches!(t.unmerge_edge(a.edges[0], &[b.faces[0]]), Err(TopoError::InvalidIncidence { .. })));
    assert!(matches!(t.unmerge_vertex(a.vertices[0], &[b.edges[0]]), Err(TopoError::InvalidIncidence { .. })));
    assert!(matches!(t.unmerge_vertex(a.vertices[0], &[a.edges[0], a.edges[0]]), Err(TopoError::InvalidParameter(_))));
    // A lone vertex unmerge leaves broken loops: end_op rolls the whole op back.
    t.unmerge_vertex(a.vertices[0], &[a.edges[0]]).unwrap();
    assert!(matches!(t.end_op(), Err(TopoError::OpInvalid(_))));
    assert_eq!(t.canonical_hash(), before);
}

/// `∮` over the face's pcurve loops of `L(u, v) du` (Green: `∬ -dL/dv du dv`), with `L` chosen
/// so the integrand is the sphere's area element (`k = 0`) or `y dA` (`k = 1`).
fn sphere_integral<E: Sys>(t: &E, f: EntityId, r: f64, k: usize) -> f64 {
    let l = |u: f64, v: f64| match k {
        0 => -r * r * v.sin(),
        _ => -r * r * r * u.sin() * (v / 2.0 + (2.0 * v).sin() / 4.0),
    };
    let mut acc = 0.0;
    for lp in t.loops(f) {
        for key in lp.keys() {
            let pc = t.pcurve(key).unwrap_or_else(|| panic!("no pcurve on {key:?}"));
            let rg = pc.range();
            let n = 2000;
            for i in 0..n {
                let (mut s0, mut s1) = (rg.lo + rg.len() * i as f64 / n as f64, rg.lo + rg.len() * (i + 1) as f64 / n as f64);
                if key.side == Side::Right {
                    (s0, s1) = (rg.hi - (s0 - rg.lo), rg.hi - (s1 - rg.lo));
                }
                let (a, b) = (pc.eval(s0), pc.eval(s1));
                let m = pc.eval(0.5 * (s0 + s1));
                acc += l(m.u, m.v) * (b.u - a.u);
            }
        }
    }
    acc
}

pub(super) fn meridian_cut_of_a_sphere_gives_two_hemispheres<E: Sys>(make: &Mk<E>) {
    let r = 2.0;
    let mut t = make(None);
    let s = with_op(&mut t, OpId(1), |t| fixtures::sphere(t, r)).unwrap();
    let whole = sphere_integral(&t, s.face, r, 0);
    assert!((whole - 4.0 * PI * r * r).abs() < 1e-6, "{whole}");
    let [pole_s, _seam, pole_n] = s.edges;
    let [vs, vn] = s.vertices;
    let (f1, f2) = with_op(&mut t, OpId(2), |t| {
        let [h1, _h2] = t.split_degenerate_edge(pole_s, PI)?;
        let [_n1, n2] = t.split_degenerate_edge(pole_n, PI)?;
        // The meridian at u = pi, from the south pole up.
        let frame = Frame::from_zx(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0), Vector3::new(-1.0, 0.0, 0.0))?;
        let curve = t.shapes_mut()?.add_curve(CurveDef::Circle(Circle::new(frame, r)?))?;
        let chord = t.make_edge(vs, vn, Some(CurveCarrier { curve, range: Interval::new(-FRAC_PI_2, FRAC_PI_2), sense: Sense::Forward }))?;
        // Plain Chord is refused: both poles appear several times in the loop.
        assert!(matches!(t.split_face(s.face, FaceCut::Chord { edge: chord }, InnerLoopAssign::None), Err(TopoError::Unsupported(_))));
        let corner = |edge, side| CornerKey { face: s.face, incoming: CoedgeKey { face: s.face, edge, side } };
        // Wrong vertex for a corner.
        assert!(t.split_face(s.face, FaceCut::ChordAt { edge: chord, start: corner(n2, Side::Right), end: corner(h1, Side::Left) }, InnerLoopAssign::None).is_err());
        let [f1, f2] = t.split_face(s.face, FaceCut::ChordAt { edge: chord, start: corner(h1, Side::Left), end: corner(n2, Side::Right) }, InnerLoopAssign::None)?;
        // Chord pcurves on both children: the line u = pi.
        let line = PCurve::Line2(Line2 { origin: Uv::new(PI, 0.0), dir: Uv::new(0.0, 1.0), range: Interval::new(-FRAC_PI_2, FRAC_PI_2) });
        for f in [f1, f2] {
            let side = t.boundary(f).iter().find(|i| i.cell == chord).map(|i| i.side).unwrap();
            t.set_pcurve(CoedgeKey { face: f, edge: chord, side }, Some(line.clone()))?;
        }
        Ok((f1, f2))
    })
    .unwrap();
    assert_valid(&t);
    assert_eq!(t.lineage().successors(pole_s).len(), 2);
    for e in t.cells(Dim::Edge) {
        if t.flags(*e).contains(CellFlags::DEGENERATE) {
            assert_eq!(t.edge_ends(*e).map(|(a, b)| a == b), Some(true));
        }
    }
    let (a1, a2) = (sphere_integral(&t, f1, r, 0), sphere_integral(&t, f2, r, 0));
    assert!((a1 - 2.0 * PI * r * r).abs() < 1e-6 && (a2 - 2.0 * PI * r * r).abs() < 1e-6, "{a1} {a2}");
    assert!((a1 + a2 - whole).abs() < 1e-9);
    // y moments: +- pi r^3 (hemispheres u in [0, pi] and [pi, 2 pi]), summing to the sphere's 0.
    let (y1, y2) = (sphere_integral(&t, f1, r, 1), sphere_integral(&t, f2, r, 1));
    assert!((y1.abs() - PI * r.powi(3)).abs() < 1e-5 && (y1 + y2).abs() < 1e-9, "{y1} {y2}");
}

pub(super) fn degenerate_split_checks_its_inputs<E: Sys>(make: &Mk<E>) {
    let mut t = make(None);
    let s = with_op(&mut t, OpId(1), |t| fixtures::sphere(t, 1.0)).unwrap();
    t.begin_op(OpId(2)).unwrap();
    assert!(matches!(t.split_degenerate_edge(s.edges[1], 0.0), Err(TopoError::InvalidParameter(_))), "seam is not degenerate");
    assert!(matches!(t.split_degenerate_edge(s.edges[0], 7.0), Err(TopoError::InvalidParameter(_))), "outside the pcurve range");
    assert!(matches!(t.split_edge(s.edges[0], s.vertices[1], 1.0), Err(TopoError::Unsupported(_))));
    let [h1, h2] = t.split_degenerate_edge(s.edges[0], 1.0).unwrap();
    let k = |e| CoedgeKey { face: s.face, edge: e, side: Side::Left };
    assert_eq!(t.pcurve(k(h1)).map(|p| p.range()), Some(Interval::new(0.0, 1.0)));
    assert_eq!(t.pcurve(k(h2)).map(|p| p.range()), Some(Interval::new(1.0, TAU)));
    assert!(t.flags(h1).contains(CellFlags::DEGENERATE) && t.flags(h2).contains(CellFlags::DEGENERATE));
    t.end_op().unwrap();
    assert_valid(&t);
}

pub(super) fn merge_gaps_are_measured_or_refused<E: Sys>(make: &Mk<E>) {
    let mut t = make(None);
    t.begin_op(OpId(1)).unwrap();
    let v0 = t.make_vertex(Point3::new(2.0, 0.0, 0.0)).unwrap();
    let v1 = t.make_vertex(Point3::new(-2.0, 0.0, 0.0)).unwrap();
    let f = Frame::at(Point3::new(0.0, 0.0, 0.0));
    let mut ell = |b: f64| {
        let c = t.shapes_mut().unwrap().add_curve(CurveDef::Ellipse(Ellipse::new(f, 2.0, b).unwrap())).unwrap();
        CurveCarrier { curve: c, range: Interval::new(0.0, PI), sense: Sense::Forward }
    };
    let (c1, c2, c3) = (ell(1.0), ell(1.5), ell(1.0 + 1e-9));
    let e1 = t.make_edge(v0, v1, Some(c1)).unwrap();
    let e2 = t.make_edge(v0, v1, Some(c2)).unwrap();
    let e3 = t.make_edge(v0, v1, None).unwrap();
    let e4 = t.make_edge(v0, v1, Some(c3)).unwrap();
    // Ellipses 0.5 apart at the apex are not merged under the default 1e-6 tolerance.
    assert!(matches!(t.merge_edges(e1, e2, Sense::Forward), Err(TopoError::NotCoincident(_))));
    // A curve against no curve cannot be measured.
    assert!(matches!(t.merge_edges(e1, e3, Sense::Forward), Err(TopoError::Unsupported("gap not measurable"))));
    // Truly coincident ellipses merge.
    t.merge_edges(e1, e4, Sense::Forward).unwrap();
    t.end_op().unwrap();
}

pub(super) fn scale_tolerances_shrinks_merge_overrides<E: Sys>(make: &Mk<E>) {
    let mut t = make(None);
    let (a, b) = with_op(&mut t, OpId(1), fixtures::two_bricks).unwrap();
    // Merge with a gap so the survivors get tolerance overrides.
    with_op(&mut t, OpId(2), |t| {
        t.set_vertex_point(b.vertices[0], Point3::new(1.0 + 1e-4, 0.0, 0.0))?;
        t.merge_vertices(a.vertices[1], b.vertices[0], Some(1e-3))?;
        Ok(())
    })
    .unwrap();
    let v = a.vertices[1];
    let before = t.tolerance_override(v).unwrap();
    assert!(before > t.tolerance().linear);
    t.begin_op(OpId(3)).unwrap();
    assert!(matches!(t.scale_tolerances(&[v], 0.0), Err(TopoError::InvalidParameter(_))));
    assert!(matches!(t.scale_tolerances(&[v], f64::NAN), Err(TopoError::InvalidParameter(_))));
    t.scale_tolerances(&[v, v, a.vertices[0]], 0.5).unwrap();
    assert!((t.tolerance_override(v).unwrap() - 0.5 * before).abs() < 1e-15);
    assert_eq!(t.tolerance_override(a.vertices[0]), None, "no override: unchanged");
    // Clamped to linear_max on the way up, removed at or below the model linear tolerance.
    t.scale_tolerances(&[v], 1e12).unwrap();
    assert_eq!(t.tolerance_override(v), Some(t.tolerance().linear_max));
    t.scale_tolerances(&[v], 1e-12).unwrap();
    assert_eq!(t.tolerance_override(v), None);
    t.end_op().unwrap();
    assert!(t.lineage().of(v).filter(|r| r.kind == LineageKind::Modified(ModKind::Tolerance)).count() >= 3);
}

pub(super) fn scenarios<E: Sys>() -> Vec<Scenario<E>> {
    vec![
        ("merge_then_unmerge_gives_the_never_merged_bricks", merge_then_unmerge_gives_the_never_merged_bricks::<E>),
        ("unmerge_operators_check_their_inputs", unmerge_operators_check_their_inputs::<E>),
        ("meridian_cut_of_a_sphere_gives_two_hemispheres", meridian_cut_of_a_sphere_gives_two_hemispheres::<E>),
        ("degenerate_split_checks_its_inputs", degenerate_split_checks_its_inputs::<E>),
        ("merge_gaps_are_measured_or_refused", merge_gaps_are_measured_or_refused::<E>),
        ("scale_tolerances_shrinks_merge_overrides", scale_tolerances_shrinks_merge_overrides::<E>),
    ]
}
