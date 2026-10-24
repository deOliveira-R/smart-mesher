//! Acceptance tests of the surface `map` scheme (plan T5), through `Driver::mesh` into a
//! dimension-2 builder, with the stub `equal` curve scheme.
#![allow(clippy::needless_range_loop)]

mod common;

use common::*;
use core::f64::consts::{FRAC_PI_2, PI};
use sm_core::Point3;
use sm_mesh_api::{CellIx, CellShape, FaceIx, FaceKind, MeshRead, validate};
use sm_scheme_api::stubs::brick_setup;
use sm_scheme_api::{
    Applicability, MeshInput, MeshIntent, PlanCtx, SchemeError, ScopedThreads, Sequential, Target,
    VertexType, WholeFaces, geom,
};

fn near(a: [f64; 3], b: [f64; 3], tol: f64) -> bool {
    (0..3).all(|k| (a[k] - b[k]).abs() <= tol)
}

/// Every expected point is present exactly once (within `tol`) and nothing else is.
fn assert_same_point_set(got: &[[f64; 3]], want: &[[f64; 3]], tol: f64) {
    assert_eq!(got.len(), want.len(), "point count");
    let mut used = vec![false; got.len()];
    for w in want {
        let hit = (0..got.len()).find(|i| !used[*i] && near(got[*i], *w, tol));
        match hit {
            Some(i) => used[i] = true,
            None => panic!("expected node {w:?} is missing"),
        }
    }
}

#[test]
fn planar_rectangle_5x3_is_the_exact_tfi_lattice_with_area_60() {
    let r = rectangle(10.0, 6.0);
    let intent = counts(&[
        (r.edges[0], 5),
        (r.edges[1], 3),
        (r.edges[2], 5),
        (r.edges[3], 3),
    ]);
    let out = mesh_faces(&r.topo, &r.model, &[r.face], &intent, &Sequential).expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.cell_count(), 15);
    assert_eq!(m.vertex_count(), 24);
    let want: Vec<[f64; 3]> = (0..4)
        .flat_map(|j| (0..6).map(move |i| [2.0 * i as f64, 2.0 * j as f64, 0.0]))
        .collect();
    assert_same_point_set(&points(m), &want, 1e-12);
    assert!((area(m) - 60.0).abs() < 1e-12, "{}", area(m));
    assert!((0..m.cell_count()).all(|c| m.cell_shape(CellIx::new(c)) == CellShape::Quad));
}

#[test]
fn trapezoid_matches_the_bilinear_closed_form() {
    // Counterclockwise: (0,0) (10,0) (8,6) (2,6). Bottom/top 5 intervals, sides 3.
    let c = [
        [0.0, 0.0, 0.0],
        [10.0, 0.0, 0.0],
        [8.0, 6.0, 0.0],
        [2.0, 6.0, 0.0],
    ];
    let t = polygon(&c, [0.0, 0.0, 1.0]);
    let intent = counts(&[
        (t.edges[0], 5),
        (t.edges[1], 3),
        (t.edges[2], 5),
        (t.edges[3], 3),
    ]);
    let out = mesh_faces(&t.topo, &t.model, &[t.face], &intent, &Sequential).expect("mesh");
    let m = &out.finished.mesh;
    let lerp = |a: [f64; 3], b: [f64; 3], s: f64| {
        [a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1]), 0.0]
    };
    let mut want = Vec::new();
    for j in 0..=3 {
        for i in 0..=5 {
            let (s, u) = (i as f64 / 5.0, j as f64 / 3.0);
            want.push(lerp(lerp(c[0], c[1], s), lerp(c[3], c[2], s), u));
        }
    }
    assert_same_point_set(&points(m), &want, 1e-12);
    assert!((area(m) - 48.0).abs() < 1e-12, "{}", area(m));
}

#[test]
fn quarter_cylinder_nodes_lie_on_the_cylinder_on_the_uv_lattice() {
    let (r, h) = (2.0, 3.0);
    let q = quarter_cylinder(r, h);
    let intent = counts(&[
        (q.edges[0], 6),
        (q.edges[1], 4),
        (q.edges[2], 6),
        (q.edges[3], 4),
    ]);
    let out = mesh_faces(&q.topo, &q.model, &[q.face], &intent, &Sequential).expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.cell_count(), 24);
    let want: Vec<[f64; 3]> = (0..=4)
        .flat_map(|j| {
            (0..=6).map(move |i| {
                let a = FRAC_PI_2 * i as f64 / 6.0;
                [r * a.cos(), r * a.sin(), h * j as f64 / 4.0]
            })
        })
        .collect();
    let got = points(m);
    for p in &got {
        assert!(
            (p[0].hypot(p[1]) - r).abs() < 1e-12,
            "{p:?} is off the cylinder"
        );
    }
    assert_same_point_set(&got, &want, 1e-12);
    // The grid is the (u, v) lattice: every node carries lattice surface parameters, lies on
    // the surface at them, and grid neighbours are lattice neighbours. (Which of the two
    // directions is `i` depends on the picked first side.)
    let patch = out.patches.get(Target::Surface(q.face)).expect("patch");
    let grid = patch.grid.as_ref().expect("grid");
    assert_eq!((grid.ni.min(grid.nj), grid.ni.max(grid.nj)), (5, 7));
    let lattice = |i: u32, j: u32| -> (i64, i64) {
        let (p, param) = out
            .patches
            .node(grid.at(i, j).expect("node"))
            .expect("point");
        let sm_mesh_api::Param::Surface([u, v]) = param else {
            // Vertex nodes carry no parameters: recover them from the position.
            let a = p.y.atan2(p.x) / (FRAC_PI_2 / 6.0);
            return (a.round() as i64, (p.z / (h / 4.0)).round() as i64);
        };
        let (a, b) = (u / (FRAC_PI_2 / 6.0), v / (h / 4.0));
        assert!(
            (a - a.round()).abs() < 1e-9 && (b - b.round()).abs() < 1e-9,
            "({i},{j}) = ({a}, {b})"
        );
        let want = Point3::new(r * u.cos(), r * u.sin(), v);
        assert!(
            (p - want).norm() < 1e-12,
            "node ({i},{j}) is off its parameters"
        );
        (a.round() as i64, b.round() as i64)
    };
    for j in 0..grid.nj {
        for i in 0..grid.ni {
            let (a, b) = lattice(i, j);
            if i + 1 < grid.ni {
                let (c, d) = lattice(i + 1, j);
                assert_eq!((c - a).abs() + (d - b).abs(), 1, "i-neighbour of ({i},{j})");
            }
            if j + 1 < grid.nj {
                let (c, d) = lattice(i, j + 1);
                assert_eq!((c - a).abs() + (d - b).abs(), 1, "j-neighbour of ({i},{j})");
            }
        }
    }
}

#[test]
fn full_cylinder_lateral_face_crosses_the_seam_in_parameter_space() {
    let (r, h) = (1.5, 2.0);
    let (topo, ids, model) = cylinder_lateral(r, h);
    let [bot, top, seam] = ids.edges;
    let intent = counts(&[(bot, 8), (top, 8), (seam, 4)]);
    let out = mesh_faces(&topo, &model, &[ids.faces[2]], &intent, &Sequential).expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.cell_count(), 32);
    assert_eq!(m.vertex_count(), 8 * 5, "seam nodes are shared");
    for p in points(m) {
        assert!((p[0].hypot(p[1]) - r).abs() < 1e-12);
    }
    // Inscribed octagon prism: 8 chords of 2 r sin(pi/8), height h.
    let want = 8.0 * 2.0 * r * (PI / 8.0).sin() * h;
    assert!((area(m) - want).abs() < 1e-12, "{} vs {want}", area(m));
}

#[test]
fn a_mid_edge_vertex_is_a_side_corner_and_the_face_meshes_as_four_sides() {
    // Rectangle with an extra vertex in the middle of the bottom edge.
    let p = polygon(
        &[
            [0.0, 0.0, 0.0],
            [4.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            [10.0, 6.0, 0.0],
            [0.0, 6.0, 0.0],
        ],
        [0.0, 0.0, 1.0],
    );
    let intent = counts(&[
        (p.edges[0], 2),
        (p.edges[1], 3),
        (p.edges[2], 3),
        (p.edges[3], 5),
        (p.edges[4], 3),
    ]);
    let out = mesh_faces(&p.topo, &p.model, &[p.face], &intent, &Sequential).expect("mesh");
    let layout = out.plan.layout(p.face).expect("layout");
    assert_eq!(layout.sides.len(), 4);
    let types: Vec<VertexType> = layout.corners.iter().map(|c| c.vertex_type).collect();
    assert_eq!(
        types.iter().filter(|t| **t == VertexType::End).count(),
        4,
        "{types:?}"
    );
    assert_eq!(
        types.iter().filter(|t| **t == VertexType::Side).count(),
        1,
        "{types:?}"
    );
    assert_eq!(
        layout.sides.iter().map(|s| s.coedges.len()).sum::<usize>(),
        5
    );
    let grid = out
        .patches
        .get(Target::Surface(p.face))
        .and_then(|p| p.grid.as_ref())
        .expect("grid");
    assert_eq!((grid.ni.min(grid.nj), grid.ni.max(grid.nj)), (4, 6));
    let m = &out.finished.mesh;
    assert_eq!(m.cell_count(), 15);
    assert!((area(m) - 60.0).abs() < 1e-12);
    // The mid-edge vertex is a node of the mesh (a lattice point of the bottom side).
    assert!(points(m).iter().any(|q| near(*q, [4.0, 0.0, 0.0], 1e-12)));
}

fn applicability(reg: &sm_scheme_api::SchemeRegistry, f: &BareFace) -> Applicability {
    let sel = [f.face];
    let input = MeshInput::new(&f.topo, &f.model, &WholeFaces, &sel);
    let intent = MeshIntent::default();
    let cx = PlanCtx::new(&input, &intent);
    let map = reg
        .get(sm_scheme_api::SchemeDim::Surface, "map")
        .expect("map");
    map.applicability(
        &cx,
        Target::Surface(f.face),
        &sm_scheme_api::Params::default(),
    )
}

#[test]
fn faces_that_are_not_four_sided_or_have_holes_are_not_applicable() {
    let reg = registry();
    let tri = polygon(
        &[[0.0, 0.0, 0.0], [4.0, 0.0, 0.0], [0.0, 3.0, 0.0]],
        [0.0, 0.0, 1.0],
    );
    assert!(matches!(
        applicability(&reg, &tri),
        Applicability::NotApplicable { .. }
    ));
    let annulus = polygon_with_holes(
        &[
            &[
                [0.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
                [10.0, 10.0, 0.0],
                [0.0, 10.0, 0.0],
            ],
            &[
                [3.0, 3.0, 0.0],
                [3.0, 7.0, 0.0],
                [7.0, 7.0, 0.0],
                [7.0, 3.0, 0.0],
            ],
        ],
        [0.0, 0.0, 1.0],
    );
    match applicability(&reg, &annulus) {
        Applicability::NotApplicable { reason, .. } => assert!(reason.contains("loop"), "{reason}"),
        other => panic!("{other:?}"),
    }
    // Through the driver a hard request for `map` on a triangle is a typed error.
    let mut intent = counts(&[]);
    intent.set_scheme(
        tri.face,
        sm_scheme_api::SchemeChoice::new("map", sm_scheme_api::Firmness::Hard),
    );
    let err = mesh_faces(&tri.topo, &tri.model, &[tri.face], &intent, &Sequential)
        .err()
        .expect("error");
    assert!(
        matches!(
            err,
            SchemeError::NotApplicable { .. } | SchemeError::NoApplicableScheme { .. }
        ),
        "{err:?}"
    );
    // The square is exact (score 100); a sphere (two corners) is not applicable.
    assert!(matches!(
        applicability(&reg, &rectangle(2.0, 1.0)),
        Applicability::Applicable { score: 100, .. }
    ));
    let mut c = sm_topo_api::CellComplex::new();
    let s = sm_topo_api::fixtures::with_op(&mut c, sm_topo_api::OpId(1), |t| {
        sm_topo_api::fixtures::sphere(t, 1.0)
    })
    .expect("sphere");
    let sphere = BareFace {
        topo: c,
        model: sm_model_api::Model::new(),
        face: s.face,
        vertices: vec![],
        edges: vec![],
    };
    assert!(matches!(
        applicability(&reg, &sphere),
        Applicability::NotApplicable { .. }
    ));
}

#[test]
fn parameters_are_rejected_as_not_applicable() {
    let reg = registry();
    let r = rectangle(1.0, 1.0);
    let sel = [r.face];
    let input = MeshInput::new(&r.topo, &r.model, &WholeFaces, &sel);
    let intent = MeshIntent::default();
    let cx = PlanCtx::new(&input, &intent);
    let map = reg
        .get(sm_scheme_api::SchemeDim::Surface, "map")
        .expect("map");
    let p = sm_scheme_api::Params(serde_json::json!({ "bias": 2.0 }));
    assert!(matches!(
        map.applicability(&cx, Target::Surface(r.face), &p),
        Applicability::NotApplicable { .. }
    ));
    assert_eq!(
        map.caps(&p),
        sm_scheme_api::Caps::QUAD
            | sm_scheme_api::Caps::STRUCTURED
            | sm_scheme_api::Caps::BLOCK_STRUCTURED
    );
}

#[test]
fn grid_line_walks_match_the_element_strip_walks() {
    let r = rectangle(10.0, 6.0);
    let intent = counts(&[
        (r.edges[0], 5),
        (r.edges[1], 3),
        (r.edges[2], 5),
        (r.edges[3], 3),
    ]);
    let out = mesh_faces(&r.topo, &r.model, &[r.face], &intent, &Sequential).expect("mesh");
    let with_grid = out
        .patches
        .get(Target::Surface(r.face))
        .expect("patch")
        .clone();
    let grid = with_grid.grid.clone().expect("grid");
    let mut without = with_grid.clone();
    without.grid = None;
    let mut walked = 0;
    for j in 0..grid.nj {
        for i in 0..grid.ni {
            let boundary = i == 0 || j == 0 || i == grid.ni - 1 || j == grid.nj - 1;
            if !boundary {
                continue;
            }
            let key = grid.at(i, j).expect("key");
            let line = grid.line_from(key);
            assert_eq!(
                line,
                with_grid.strip_from(key),
                "grid fast path, node ({i},{j})"
            );
            assert_eq!(
                line,
                without.strip_from(key),
                "element walk, node ({i},{j})"
            );
            walked += usize::from(line.is_some());
        }
    }
    // Every non-corner boundary node starts a strip: 2 (5 - 1) + 2 (3 - 1).
    assert_eq!(walked, 2 * 4 + 2 * 2);
}

#[test]
fn the_six_brick_faces_close_to_a_surface_with_euler_characteristic_2() {
    let (topo, ids, model) = brick_setup([1.0, 1.5, 2.0]);
    let intent = counts(&[(ids.edges[0], 2), (ids.edges[4], 3), (ids.edges[8], 4)]);
    let out = mesh_faces(&topo, &model, &ids.faces, &intent, &Sequential).expect("surface mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.cell_count(), 2 * (12 + 8 + 6));
    assert!((0..m.face_count()).all(|f| m.face_kind(FaceIx::new(f)) == FaceKind::Interior));
    assert_eq!(
        m.vertex_count() as i64 - m.face_count() as i64 + m.cell_count() as i64,
        2
    );
    assert!((area(m) - 13.0).abs() < 1e-12);
    // Quads follow their face's normal.
    for c in 0..m.cell_count() {
        let ci = CellIx::new(c);
        let cyc = m.cell_corners(ci).expect("corners");
        let p: Vec<_> = cyc.iter().map(|v| m.point(v.get())).collect();
        let n = (p[2] - p[0]).cross(p[3] - p[1]);
        let face = m.cell_class(ci).entity;
        let want = geom::face_normal(&topo, face, p[0]).expect("normal");
        assert!(n.dot(want) > 0.0, "cell {c} on {face}");
    }
    // Every brick face holds a structured grid with the sides' interval counts.
    for (f, dims) in ids
        .faces
        .iter()
        .zip([(4, 5), (4, 5), (3, 5), (3, 5), (3, 4), (3, 4)])
    {
        let g = out
            .patches
            .get(Target::Surface(*f))
            .and_then(|p| p.grid.as_ref())
            .expect("grid");
        let (a, b) = (g.ni.min(g.nj), g.ni.max(g.nj));
        assert_eq!((a, b), (dims.0 as u32, dims.1 as u32), "face {f}");
    }
}

#[test]
fn faces_without_a_carrier_are_meshed_in_three_dimensions_with_no_parameters() {
    // A face with no surface carrier: build a rectangle, then drop the carrier by making the
    // face with `None`.
    use sm_topo_api::{CoedgeSpec, Sense, TopologyEdit};
    let mut topo = sm_topo_api::CellComplex::new();
    let (face, edges) =
        sm_topo_api::fixtures::with_op(&mut topo, sm_topo_api::OpId(1), |t| {
            let pts = [
                [0.0, 0.0, 0.0],
                [4.0, 0.0, 0.0],
                [4.0, 2.0, 0.0],
                [0.0, 2.0, 0.0],
            ];
            let vs: Vec<_> = pts
                .iter()
                .map(|p| t.make_vertex(Point3::new(p[0], p[1], p[2])))
                .collect::<Result<_, _>>()?;
            let mut es = Vec::new();
            for i in 0..4 {
                let (a, b) = (
                    Point3::new(pts[i][0], pts[i][1], pts[i][2]),
                    Point3::new(
                        pts[(i + 1) % 4][0],
                        pts[(i + 1) % 4][1],
                        pts[(i + 1) % 4][2],
                    ),
                );
                let d = b - a;
                let c = t.shapes_mut()?.add_curve(sm_geom_api::CurveDef::Line(
                    sm_geom_api::Line::new(a, d * (1.0 / d.norm()))?,
                ))?;
                es.push(t.make_edge(
                    vs[i],
                    vs[(i + 1) % 4],
                    Some(sm_topo_api::CurveCarrier {
                        curve: c,
                        range: sm_geom_api::Interval::new(0.0, d.norm()),
                        sense: Sense::Forward,
                    }),
                )?);
            }
            let lp: Vec<CoedgeSpec> = es
                .iter()
                .map(|e| CoedgeSpec {
                    edge: *e,
                    sense: Sense::Forward,
                })
                .collect();
            Ok((t.make_face(&[&lp], None)?, es))
        })
        .expect("face");
    let intent = counts(&[(edges[0], 4), (edges[1], 2), (edges[2], 4), (edges[3], 2)]);
    let out = mesh_faces(
        &topo,
        &sm_model_api::Model::new(),
        &[face],
        &intent,
        &Sequential,
    )
    .expect("mesh");
    assert_eq!(out.finished.mesh.cell_count(), 8);
    assert!((area(&out.finished.mesh) - 8.0).abs() < 1e-12);
}

#[test]
fn output_is_bit_identical_across_executors_runs_and_threads() {
    let q = quarter_cylinder(2.0, 3.0);
    let intent = counts(&[
        (q.edges[0], 7),
        (q.edges[1], 5),
        (q.edges[2], 7),
        (q.edges[3], 5),
    ]);
    let hash = |threads: usize| {
        let out = if threads <= 1 {
            mesh_faces(&q.topo, &q.model, &[q.face], &intent, &Sequential)
        } else {
            mesh_faces(
                &q.topo,
                &q.model,
                &[q.face],
                &intent,
                &ScopedThreads(threads),
            )
        }
        .expect("mesh");
        let m = &out.finished.mesh;
        (
            sm_mesh_api::topology_hash(m),
            sm_mesh_api::geometry_hash(m),
            sm_testkit::check::mesh_hash(m),
        )
    };
    sm_testkit::determinism::check(2, 4, hash).expect("deterministic");
}

#[test]
fn sphere_band_nodes_sit_on_the_parameter_lattice_not_on_projected_chords() {
    // Only the parameter-space blend puts the interior on the (longitude, latitude) lattice;
    // a projected 3-D blend of the same boundary is off by orders of magnitude more than 1e-12.
    let (r, lon, lat0, lat1) = (3.0, FRAC_PI_2, PI / 6.0, PI / 3.0);
    let b = sphere_band(r, lon, lat0, lat1);
    let intent = counts(&[
        (b.edges[0], 5),
        (b.edges[1], 4),
        (b.edges[2], 5),
        (b.edges[3], 4),
    ]);
    let out = mesh_faces(&b.topo, &b.model, &[b.face], &intent, &Sequential).expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.cell_count(), 20);
    let want: Vec<[f64; 3]> = (0..=4)
        .flat_map(|j| {
            (0..=5).map(move |i| {
                let (lam, phi) = (lon * i as f64 / 5.0, lat0 + (lat1 - lat0) * j as f64 / 4.0);
                [
                    r * phi.cos() * lam.cos(),
                    r * phi.cos() * lam.sin(),
                    r * phi.sin(),
                ]
            })
        })
        .collect();
    let got = points(m);
    assert!(
        got.iter()
            .all(|p| ((p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt() - r).abs() < 1e-12)
    );
    assert_same_point_set(&got, &want, 1e-12);
}

#[test]
fn a_five_sided_face_meshes_as_four_logical_sides_once_a_corner_is_demoted() {
    // A regular pentagon has five 108 degree corners. Map needs four Ends, so the picker
    // demotes one to a Side and the face meshes as four logical sides (one holding two edges).
    let pts: Vec<[f64; 3]> = (0..5)
        .map(|k| {
            let a = TAU_5 * k as f64;
            [a.cos(), a.sin(), 0.0]
        })
        .collect();
    let p = polygon(&pts, [0.0, 0.0, 1.0]);
    let mut intent = MeshIntent::default();
    intent.defaults.sizing = Some(sm_scheme_api::SizingSpec::Constant { size: 0.25 });
    let out = mesh_faces(&p.topo, &p.model, &[p.face], &intent, &Sequential).expect("mesh");
    let layout = out.plan.layout(p.face).expect("layout");
    assert_eq!(layout.sides.len(), 4);
    assert_eq!(
        layout.sides.iter().map(|s| s.coedges.len()).sum::<usize>(),
        5
    );
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    // The quads are a planar partition of the pentagon: total area is the pentagon's, to the
    // accuracy of the straight-sided cells (exact: all sides are straight).
    let pentagon = 2.5 * (TAU_5).sin();
    assert!(
        (area(m) - pentagon).abs() < 1e-9,
        "{} vs {pentagon}",
        area(m)
    );
}

const TAU_5: f64 = 2.0 * PI / 5.0;
