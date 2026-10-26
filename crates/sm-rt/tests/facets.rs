//! Acceptances 5 (Cauchy on a facet shell) and 7 (facet input, diagnostics, locate).
mod common;
use common::*;
use sm_core::Point3;
use sm_rt::check_facets;
use sm_rt_api::{BoundarySpec, CellId, FacetInput, RegionId, RtError, Scratch, StepEnd, Tracker};
use sm_testkit::surf3::{icosphere, polyhedron_volume};
use sm_topo_api::Region;
















fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    dot(sub(a, b), sub(a, b)).sqrt()
}

#[test]
fn acc07_tetrahedron_icosphere_and_nested_pair_build() {
    for (name, f) in [("tet", tet_facets()), ("icosphere3", sphere_facets(1.0, 3).0), ("nested", nested_facets())] {
        let t = build(&f, &opts_fail()).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert_eq!(t.info().faces, f.triangles.len());
        assert_eq!(t.info().faceted_faces, f.triangles.len());
        assert!(t.caps().has(sm_rt_api::TrackerCaps::VOLUME_MESH));
        assert_eq!(f.check_watertight().len(), 0);
        assert!(check_facets(&f, &BoundarySpec::default()).is_empty());
    }
    let t = build(&nested_facets(), &opts_fail()).unwrap();
    assert_eq!(t.cell_count(), 2);
    assert!(t.regions().implicit_complement().is_some());
}

#[test]
fn acc07_open_shell_reports_one_located_group_and_build_refuses() {
    let (f, _) = sphere_facets(1.0, 2);
    for removed in [0usize, 7, 100, 319] {
        let g = remove_triangle(&f, removed);
        let issues = check_facets(&g, &BoundarySpec::default());
        assert_eq!(issues.len(), 1, "one group");
        let i = &issues[0];
        assert_eq!(i.code, "facet.open_edge");
        let t = f.triangles[removed];
        let (a, b, c) = (f.vertices[t[0] as usize], f.vertices[t[1] as usize], f.vertices[t[2] as usize]);
        let longest = dist(a, b).max(dist(b, c)).max(dist(c, a));
        assert!((i.gap.unwrap() - longest).abs() < 1e-12, "gap {} vs {longest}", i.gap.unwrap());
        // The location lies on the missing triangle: in its plane and inside it.
        let loc = i.location.unwrap();
        let p = [loc.x, loc.y, loc.z];
        let n = cross(sub(b, a), sub(c, a));
        let nl = dot(n, n).sqrt();
        assert!((dot(n, sub(p, a)) / nl).abs() < 1e-12, "in plane");
        for (u, v) in [(a, b), (b, c), (c, a)] {
            assert!(dot(cross(sub(v, u), sub(p, u)), n) >= 0.0, "inside");
        }
        // The contract's per-edge check reports the three open edges of the same hole.
        assert_eq!(g.check_watertight().iter().filter(|i| i.code == "facet.open_edge").count(), 3);
        match build(&g, &opts_fail()) {
            Err(RtError::Build(issues)) => assert_eq!(issues[0].code, "facet.open_edge"),
            Err(e) => panic!("{e}"),
            Ok(_) => panic!("an open shell must not build"),
        }
    }
}

#[test]
fn acc07_flipped_triangle_is_inconsistent_orientation() {
    let (mut f, _) = sphere_facets(1.0, 2);
    f.triangles[11].swap(1, 2);
    let issues = check_facets(&f, &BoundarySpec::default());
    assert!(issues.iter().any(|i| i.code == "facet.inconsistent_orientation"), "{issues:?}");
    assert!(f.check_watertight().iter().any(|i| i.code == "facet.inconsistent_orientation"));
    assert!(matches!(build(&f, &opts_fail()), Err(RtError::Build(_))));
}

#[test]
fn acc07_hostile_soups_never_panic() {
    let (f, _) = sphere_facets(1.0, 1);
    let mut cases: Vec<FacetInput> = Vec::new();
    let mut g = f.clone();
    g.triangles[0] = [0, 0, 1];
    cases.push(g);
    let mut g = f.clone();
    g.triangles[3] = [0, 1, 9999];
    cases.push(g);
    let mut g = f.clone();
    g.vertices[5] = [f64::NAN, 0.0, 0.0];
    cases.push(g);
    let mut g = f.clone();
    g.front.pop();
    cases.push(g);
    cases.push(FacetInput::default());
    let mut g = f.clone();
    g.back[2] = g.front[2];
    cases.push(g);
    let mut g = f.clone();
    g.front[4] = Region::Exterior;
    cases.push(g);
    for c in &cases {
        let _ = check_facets(c, &BoundarySpec::default());
        assert!(build(c, &opts_fail()).is_err());
    }
}

/// Generalised winding number (van Oosterom-Strackee) of a closed oriented triangle set.
fn winding(pts: &[[f64; 3]], tris: &[[u32; 3]], p: [f64; 3]) -> f64 {
    let mut s = 0.0;
    for t in tris {
        let a = sub(pts[t[0] as usize], p);
        let b = sub(pts[t[1] as usize], p);
        let c = sub(pts[t[2] as usize], p);
        let (la, lb, lc) = (dot(a, a).sqrt(), dot(b, b).sqrt(), dot(c, c).sqrt());
        let num = dot(a, cross(b, c));
        let den = la * lb * lc + dot(a, b) * lc + dot(b, c) * la + dot(c, a) * lb;
        s += 2.0 * num.atan2(den);
    }
    s / (4.0 * std::f64::consts::PI)
}

fn region_of_cell(t: &dyn Tracker, c: CellId) -> RegionId {
    t.cell_region(c).unwrap_or(RegionId::NONE)
}

#[test]
fn acc07_locate_agrees_with_winding_number() {
    let (outer, inner) = (icosphere(2.0, 3), icosphere(1.0, 3));
    let (ot, it) = (outer.oriented_tris(0), inner.oriented_tris(0));
    let mut f = FacetInput::default();
    add_shell(&mut f, &outer.points, &ot, vol(2), Region::Exterior, 1);
    add_shell(&mut f, &inner.points, &it, vol(1), vol(2), 2);
    let t = build(&f, &opts_fail()).unwrap();
    let mut sc = Scratch::new();
    let mut r = rng("acc07_locate");
    let n = if cfg!(debug_assertions) { 20_000 } else { 100_000 };
    let (mut in_inner, mut in_outer, mut out, mut skipped) = (0, 0, 0, 0);
    for _ in 0..n {
        let p = [2.4 * r.unit(), 2.4 * r.unit(), 2.4 * r.unit()];
        let (wi, wo) = (winding(&inner.points, &it, p), winding(&outer.points, &ot, p));
        if (wi - wi.round()).abs() > 0.02 || (wo - wo.round()).abs() > 0.02 {
            skipped += 1;
            continue;
        }
        let want = if wi.round() == 1.0 { Some(0) } else if wo.round() == 1.0 { Some(1) } else { None };
        let loc = t.locate(p3(p), None, None, &mut sc).unwrap();
        match want {
            Some(k) => {
                assert_eq!(loc.cell, CellId::new(k), "point {p:?}");
                assert_eq!(region_of_cell(&*t, loc.cell), RegionId::new(k));
                if k == 0 { in_inner += 1 } else { in_outer += 1 }
            }
            None => {
                assert!(loc.cell == CellId::COMPLEMENT || loc.cell.is_none(), "point {p:?} -> {:?}", loc.cell);
                out += 1;
            }
        }
        assert!(loc.on_face.is_none());
    }
    assert!(in_inner > 100 && in_outer > 100 && out > 100, "{in_inner} {in_outer} {out} (skipped {skipped})");
}

#[test]
fn acc07_nested_shells_chord_sequence_and_complement_entry() {
    let t = build(&nested_facets(), &opts_fail()).unwrap();
    let mut sc = Scratch::new();
    // A ray along +x through the centre, started in the complement just inside the bounds.
    let b = t.bounds();
    let start = Point3::new(b.lo.x, 0.01, 0.01);
    let mut st = t.init_state(start, v3([1.0, 0.0, 0.0]), None, &mut sc).unwrap();
    assert_eq!(st.cell, CellId::COMPLEMENT);
    let mut regions = Vec::new();
    let mut total = [0.0f64; 3];
    loop {
        let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
        regions.push(s.region);
        total[s.region.get().min(2)] += s.dist;
        match s.end {
            StepEnd::Boundary { .. } | StepEnd::Lost(_) => break,
            StepEnd::RegionBoundary { to_cell, .. } if to_cell.is_none() => break,
            _ => {}
        }
    }
    // complement -> outer -> inner -> outer -> complement, then out of the box.
    let r = |i: usize| RegionId::new(i);
    assert_eq!(regions, vec![r(2), r(1), r(0), r(1), r(2)]);
    // The inner shell is a level-2 icosphere: its chord through the vertex axis is ~2.
    assert!((total[0] - 2.0).abs() < 0.1, "inner chord {}", total[0]);
    assert!((total[1] - 2.0).abs() < 0.2, "outer chords {}", total[1]);
}

fn facet_hull_chords(f: &FacetInput, name: &str) -> (f64, f64, usize) {
    let t = build(f, &opts_fail()).unwrap();
    let b = t.bounds();
    let (lo, hi) = ([b.lo.x, b.lo.y, b.lo.z], [b.hi.x, b.hi.y, b.hi.z]);
    let hull_box = Convex::boxed(lo, hi);
    let center = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0];
    let rho = dist(lo, hi) / 2.0 * 1.0001;
    let n = n_lines();
    let chunk = 10_000;
    let parts: Vec<(f64, f64, usize, usize)> = {
        use rayon::prelude::*;
        (0..n.div_ceil(chunk))
            .into_par_iter()
            .map(|ci| {
                let mut r = SplitMix64(seed_of(name) ^ sm_testkit::surf3::mix64(ci as u64 + 1));
                let mut sc = Scratch::new();
                let (mut s, mut s2, mut hits, mut lost) = (0.0, 0.0, 0usize, 0usize);
                for _ in 0..chunk.min(n - ci * chunk) {
                    let (o, d) = iur_line(&mut r, center, rho);
                    let Some((a, bb)) = hull_box.clip(o, d).filter(|(a, b)| b > a) else { continue };
                    // Start on the bounding box (in the complement) and trace through the shell.
                    let p0 = [o[0] + a * d[0], o[1] + a * d[1], o[2] + a * d[2]];
                    let _ = bb;
                    let Ok(mut st) = t.init_state(p3(p0), v3(d), None, &mut sc) else {
                        lost += 1;
                        continue;
                    };
                    let mut chord = 0.0;
                    loop {
                        let step = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
                        if step.cell != CellId::COMPLEMENT {
                            chord += step.dist;
                        }
                        match step.end {
                            StepEnd::Lost(_) => {
                                lost += 1;
                                break;
                            }
                            StepEnd::Boundary { .. } => break,
                            StepEnd::RegionBoundary { to_cell, .. } if to_cell.is_none() => break,
                            _ => {}
                        }
                    }
                    if chord > 0.0 {
                        s += chord;
                        s2 += chord * chord;
                        hits += 1;
                    }
                }
                (s, s2, hits, lost)
            })
            .collect()
    };
    let (mut s, mut s2, mut hits, mut lost) = (0.0, 0.0, 0usize, 0usize);
    for p in parts {
        s += p.0;
        s2 += p.1;
        hits += p.2;
        lost += p.3;
    }
    assert_eq!(lost, 0);
    let mean = s / hits as f64;
    let var = (s2 / hits as f64 - mean * mean).max(0.0);
    (mean, (var / hits as f64).sqrt(), hits)
}

#[test]
fn acc05_cauchy_icosphere_shell() {
    let (f, fx) = sphere_facets(1.0, 4);
    let tris = fx.oriented_tris(0);
    let v = polyhedron_volume(&fx.points, &tris);
    let area: f64 = tris
        .iter()
        .map(|t| {
            let (a, b, c) = (fx.points[t[0] as usize], fx.points[t[1] as usize], fx.points[t[2] as usize]);
            let n = cross(sub(b, a), sub(c, a));
            0.5 * dot(n, n).sqrt()
        })
        .sum();
    let (mean, sigma, hits) = facet_hull_chords(&f, "cauchy_icosphere4");
    let want = 4.0 * v / area;
    eprintln!("Cauchy icosphere(4): mean chord {mean:.6} vs 4V/S {want:.6} ({:.2} sigma, {hits} hits)", (mean - want).abs() / sigma);
    assert!((mean - want).abs() <= 4.0 * sigma, "mean chord {mean} vs 4V/S {want} (sigma {sigma}, {hits} hits)");
}

#[test]
fn acc05_cauchy_hex_block_and_truncated_octahedron() {
    use sm_mesh_api::reference::hex_block;
    use sm_mesh_api::{MeshRead, SerialAlloc};
    let mesh = hex_block([10; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let to = sm_testkit::meshes::truncated_octahedron().unwrap();
    for (name, m, v, s) in [("cauchy_hex_block", &mesh as &dyn MeshRead, 1000.0, 600.0), ("cauchy_truncated_octahedron", &to.mesh as &dyn MeshRead, to.volume, to.area)] {
        let t = build_mesh_tracker(m, &vacuum(), &opts_fail());
        let b = t.bounds();
        let (lo, hi) = ([b.lo.x, b.lo.y, b.lo.z], [b.hi.x, b.hi.y, b.hi.z]);
        let center = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0];
        let rho = dist(lo, hi) / 2.0 * 1.0001;
        // The hull of each case is convex: the box, or the single polyhedral cell.
        let hull = if m.cell_count() == 1 { cell_polytope(m, 0) } else { Convex::boxed(lo, hi) };
        let runs = |o: [f64; 3], d: [f64; 3]| hull.clip(o, d).into_iter().filter(|(a, b)| b > a).collect::<Vec<_>>();
        let st = run_iur(&t, name, n_lines(), center, rho, &runs);
        assert_cauchy(&st, v, s, 4.0);
        assert_eq!(st.lost, 0);
    }
}

#[test]
fn scope4_disjoint_bodies_are_connected_through_the_complement() {
    let mut f = FacetInput::default();
    let pts = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let tris = [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
    add_shell(&mut f, &pts, &tris, vol(1), Region::Exterior, 1);
    let moved = pts.map(|p| [p[0] + 3.0, p[1], p[2]]);
    add_shell(&mut f, &moved, &tris, vol(2), Region::Exterior, 2);
    let t = build(&f, &opts_fail()).unwrap();
    let mut sc = Scratch::new();
    let mut st = t.init_state(Point3::new(0.1, 0.1, 0.1), v3([1.0, 0.0, 0.0]), None, &mut sc).unwrap();
    let mut seen = Vec::new();
    loop {
        let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
        seen.push((s.cell, s.dist));
        match s.end {
            StepEnd::Boundary { .. } | StepEnd::Lost(_) => panic!("history cut short: {:?}", s.end),
            StepEnd::RegionBoundary { to_cell, .. } if to_cell.is_none() => break,
            _ => {}
        }
    }
    // body 1 (0.7), complement (2.2), body 2 (0.8), complement to the box edge.
    assert_eq!(seen.len(), 4, "{seen:?}");
    assert!((seen[0].1 - 0.7).abs() < 1e-14 && seen[0].0 == CellId::new(0));
    assert!((seen[1].1 - 2.2).abs() < 1e-12 && seen[1].0 == CellId::COMPLEMENT);
    assert!((seen[2].1 - 0.8).abs() < 1e-12 && seen[2].0 == CellId::new(1));
    assert_eq!(seen[3].0, CellId::COMPLEMENT);
    // An explicit rule on a face still claims it.
    let spec = BoundarySpec::new(sm_rt_api::BoundaryDef::Respond(sm_rt_api::ResponseKind::Vacuum), vec![(sm_rt_api::BoundarySelector::Entity(ent(sm_core::EntityKind::TopoFace, 1)), sm_rt_api::BoundaryDef::Reflect)]);
    use sm_rt_api::TrackerBuilder;
    let t = sm_rt::MeshTrackerBuilder.build(&sm_rt_api::BuildInput::Facets(&f, &spec), &opts_fail()).unwrap().0;
    let mut st = t.init_state(Point3::new(0.1, 0.1, 0.1), v3([1.0, 0.0, 0.0]), None, &mut sc).unwrap();
    let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    assert!(matches!(s.end, StepEnd::Boundary { action: sm_rt_api::FaceAction::Reflect, .. }), "{:?}", s.end);
}
