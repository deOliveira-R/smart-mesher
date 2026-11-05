//! W3b-RT-BRIDGE acceptance 1-3 and the sign rule: model volumes, prototypes and lattices as CSG
//! cells, tracked exactly.
#[path = "common/csg.rs"]
mod csg;

use csg::{p3, rng, v3};
use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_geom_api::{NurbsSurface, SurfaceDef};
use sm_model_api::{Fill, Lattice, Layout, Model};
use sm_rt::{BridgeError, CheckScope, CsgWorld, CsgWorldOptions, MAX_FLAT_CELLS, check_world_overlaps, csg_world};
use sm_rt_api::{BoundarySpec, BuildOptions, CellId, ComponentKind, Scratch, Tracker};
use sm_testkit::topo::{ModelFixture, pin_assembly};
use sm_topo_api::fixtures::{brick, cone, cylinder, sphere, torus, with_op};
use sm_topo_api::{CellComplex, Dim, OpId, TopologyView};
use std::collections::BTreeMap;

const PITCH: f64 = 1.26;
const RF: f64 = 0.4096;
const RC: f64 = 0.475;
const H: f64 = 100.0;

/// Uniform in `[0, 1)`.
fn u01(r: &mut csg::SplitMix64) -> f64 {
    0.5 * (r.unit() + 1.0)
}

fn world_of(t: &CellComplex) -> CsgWorld {
    csg_world(&Model::new(), t, &CsgWorldOptions::default()).expect("bridges")
}

fn tracker(w: &CsgWorld) -> sm_rt::AnalyticTracker {
    w.analytic_tracker(&BoundarySpec::default(), &BuildOptions::default(), MAX_FLAT_CELLS).expect("builds").0
}

fn inside_cell0(t: &sm_rt::AnalyticTracker, p: Point3) -> bool {
    let mut sc = Scratch::new();
    t.locate(p, Some(v3(0.267, 0.534, 0.802)), None, &mut sc).unwrap().cell == CellId::new(0)
}

// ---- the sign rule, one fixture per surface kind -----------------------------------------------

#[test]
fn a_cylinder_volume_is_one_cell_with_the_right_chords() {
    let (r, h) = (2.0, 3.0);
    let mut c = CellComplex::new();
    with_op(&mut c, OpId(1), |t| cylinder(t, r, h)).unwrap();
    let w = world_of(&c);
    assert!(w.is_single_leaf());
    assert_eq!(w.cells(0).len(), 1);
    let t = tracker(&w);
    assert!(inside_cell0(&t, p3(0.0, 0.0, 1.5)));
    assert!(inside_cell0(&t, p3(1.9, 0.0, 0.1)));
    assert!(!inside_cell0(&t, p3(2.1, 0.0, 1.5)));
    assert!(!inside_cell0(&t, p3(0.0, 0.0, 3.1)));
    assert!(!inside_cell0(&t, p3(0.0, 0.0, -0.1)));
    // A line through the axis along x: chord 2 r.
    let mut sc = Scratch::new();
    let tr = csg::trace(&t, p3(-0.5 * r, 0.0, 1.5), v3(1.0, 0.0, 0.0), f64::INFINITY, &mut sc);
    assert!((tr.inside() - 1.5 * r).abs() < 1e-12, "{}", tr.inside());
    let tr = csg::trace(&t, p3(0.0, 0.0, 0.5), v3(0.0, 0.0, 1.0), f64::INFINITY, &mut sc);
    assert!((tr.inside() - 2.5).abs() < 1e-12);
}

#[test]
fn sphere_torus_brick_and_cone_volumes_hold_the_right_points() {
    let mut c = CellComplex::new();
    with_op(&mut c, OpId(1), |t| sphere(t, 1.5)).unwrap();
    let t = tracker(&world_of(&c));
    assert!(inside_cell0(&t, p3(0.0, 0.0, 0.0)) && inside_cell0(&t, p3(1.4, 0.0, 0.0)) && !inside_cell0(&t, p3(1.6, 0.0, 0.0)));

    let mut c = CellComplex::new();
    with_op(&mut c, OpId(1), |t| torus(t, 4.0, 1.0)).unwrap();
    let t = tracker(&world_of(&c));
    assert!(inside_cell0(&t, p3(4.0, 0.0, 0.0)) && inside_cell0(&t, p3(0.0, -4.5, 0.2)));
    assert!(!inside_cell0(&t, p3(0.0, 0.0, 0.0)) && !inside_cell0(&t, p3(6.0, 0.0, 0.0)));

    let mut c = CellComplex::new();
    with_op(&mut c, OpId(1), |t| brick(t, Point3::new(1.0, 2.0, 3.0), [2.0, 3.0, 4.0])).unwrap();
    let t = tracker(&world_of(&c));
    assert!(inside_cell0(&t, p3(2.0, 3.5, 5.0)) && !inside_cell0(&t, p3(0.5, 3.5, 5.0)) && !inside_cell0(&t, p3(2.0, 3.5, 7.5)));

    // Cone: base disc at z = 0 (radius r), apex at z = h. The far nappe beyond the apex is not in the cell.
    let (r, h) = (2.0, 3.0);
    let mut c = CellComplex::new();
    with_op(&mut c, OpId(1), |t| cone(t, r, h)).unwrap();
    let t = tracker(&world_of(&c));
    assert!(inside_cell0(&t, p3(0.0, 0.0, 0.5)) && inside_cell0(&t, p3(0.9 * r * 0.5, 0.0, 0.5 * h)));
    assert!(!inside_cell0(&t, p3(0.0, 0.0, h + 0.5)), "the nappe beyond the apex is outside");
    assert!(!inside_cell0(&t, p3(0.0, 0.0, h + 1e-3)));
    assert!(!inside_cell0(&t, p3(1.1 * r * 0.5, 0.0, 0.5 * h)) && !inside_cell0(&t, p3(0.0, 0.0, -0.1)));
}

// ---- refusals ------------------------------------------------------------------------------------

#[test]
fn a_nurbs_face_is_unsupported_and_named() {
    let mut c = CellComplex::new();
    let ids = with_op(&mut c, OpId(1), |t| cylinder(t, 2.0, 3.0)).unwrap();
    let patch = NurbsSurface::new(1, 1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0], 2, 2, vec![p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0), p3(0.0, 1.0, 0.0), p3(1.0, 1.0, 0.0)], None).unwrap();
    with_op(&mut c, OpId(2), |t| {
        use sm_topo_api::TopologyEdit;
        t.replace_surface(ids.lateral_surface, SurfaceDef::Nurbs(patch)).map_err(sm_topo_api::fixtures::FixtureError::from)
    })
    .unwrap();
    let face = ids.faces[2];
    let err = csg_world(&Model::new(), &c, &CsgWorldOptions::default()).unwrap_err();
    match &err {
        BridgeError::Unsupported { entity, what } => {
            assert_eq!(*entity, Some(face), "names the face");
            assert!(what.contains("NURBS") && what.contains(&face.to_string()), "{what}");
        }
        other => panic!("{other:?}"),
    }
    // It reaches a caller of the tracker API as a located build issue.
    let e: sm_rt_api::RtError = err.into();
    let sm_rt_api::RtError::Build(v) = e else { panic!() };
    assert_eq!(v[0].code, "csg.unsupported");
    assert_eq!(v[0].entity, Some(face));
}

#[test]
fn hex_lattices_recolors_and_baffles_are_refused_with_the_entity() {
    let fix = pin_assembly(2, PITCH, RF, RC, H).unwrap();
    let mut m = fix.model.clone();
    let proto = *m.instancing().prototypes.keys().next().unwrap();
    let hex = m.add_lattice(Lattice::new(Layout::Hex { rings: 2, pitch: 2.0, orient: sm_model_api::HexOrient::XRows, nz: 1, dz: 1.0 }, Transform::IDENTITY, Fill::Prototype(proto), BTreeMap::new())).unwrap();
    m.add_instance(sm_model_api::InstanceRow::new(Fill::Lattice(hex), Transform::IDENTITY)).unwrap();
    match csg_world(&m, &fix.complex, &CsgWorldOptions::default()).unwrap_err() {
        BridgeError::Unsupported { entity, what } => assert!(entity == Some(hex) && what.contains("hex"), "{what}"),
        other => panic!("{other:?}"),
    }
    // A volume that is not live in the selection is an error, not a silent empty world.
    let ghost = EntityId::new(sm_core::EntityKind::TopoBody, 999).unwrap();
    let e = csg_world(&fix.model, &fix.complex, &CsgWorldOptions { bodies: Some(vec![ghost]), complement: None }).unwrap_err();
    assert!(matches!(e, BridgeError::Invalid(_)));
}

// ---- the pin assembly ---------------------------------------------------------------------------

fn assembly() -> (ModelFixture, CsgWorld) {
    let fix = pin_assembly(17, PITCH, RF, RC, H).unwrap();
    let w = csg_world(&fix.model, &fix.complex, &CsgWorldOptions { bodies: None, complement: Some("void".into()) }).unwrap();
    (fix, w)
}

#[test]
fn region_names_equal_the_model_color_names_and_the_structure_is_the_models() {
    let (fix, w) = assembly();
    // One prototype leaf of two cells; a lattice of it, a cluster of nine lattices, one instance.
    assert_eq!(w.leaf_count(), 1);
    let mut names: Vec<&str> = w.cells(0).iter().map(|c| c.name.as_str()).collect();
    names.sort_unstable();
    assert_eq!(names, ["clad", "fuel"]);
    let colors: Vec<&str> = fix.model.colors().map(|c| c.name.as_str()).collect();
    assert!(names.iter().all(|n| colors.contains(n)));
    assert_eq!(w.flat_cell_count(), 2 * 9 * 17 * 17);
    let lattices = w.spec().components.iter().filter(|c| matches!(c, ComponentKind::Lattice(_))).count();
    let unions = w.spec().components.iter().filter(|c| matches!(c, ComponentKind::Union { .. })).count();
    assert_eq!((lattices, unions), (1, 2), "one lattice, the cluster and the instance root");
    // The flattened tracker's regions carry the same names.
    let flat = w.flatten(MAX_FLAT_CELLS).unwrap();
    assert!(flat.is_single_leaf() && flat.cells(0).len() == 2 * 2601);
    let t = tracker(&w);
    let mut rn: Vec<String> = (0..t.regions().0.len()).map(|i| t.regions().0[i].name.clone()).collect();
    rn.sort();
    assert_eq!(rn, ["clad", "fuel", "void"]);
}

#[test]
fn chords_across_the_assembly_through_the_void() {
    let (_fix, w) = assembly();
    let t = tracker(&w);
    let region_name = |id: sm_rt_api::RegionId| t.regions().get(id).map(|r| r.name.clone()).unwrap_or_default();
    // Pins sit on a 51 x 51 grid of pitch PITCH (the cluster span is n * pitch).
    let centres: Vec<(f64, f64)> = (0..51).flat_map(|j| (0..51).map(move |i| (i as f64 * PITCH, j as f64 * PITCH))).collect();
    let mut r = rng("bridge-chords");
    let mut sc = Scratch::new();
    let (mut worst, mut n_fuel, mut n_clad) = (0.0f64, 0usize, 0usize);
    for k in 0..100 {
        // Start in the gap between four pins, aim anywhere in the plane with a z slope.
        let (a, b) = ((u01(&mut r) * 50.0).floor().min(49.0), (u01(&mut r) * 50.0).floor().min(49.0));
        let o = p3((a + 0.5) * PITCH, (b + 0.5) * PITCH, 30.0 + 40.0 * u01(&mut r));
        let th = std::f64::consts::TAU * u01(&mut r);
        let slope = 0.6 * u01(&mut r) - 0.3;
        let raw = v3(th.cos(), th.sin(), slope);
        let d = raw * (1.0 / raw.norm());
        let rho = d.x.hypot(d.y);
        let (mut exp_fuel, mut exp_clad) = (0.0, 0.0);
        for &(cx, cy) in &centres {
            let (px, py) = (cx - o.x, cy - o.y);
            let along = (px * d.x + py * d.y) / (rho * rho);
            if along <= 0.0 {
                continue;
            }
            let bperp = (px * d.y - py * d.x).abs() / rho;
            if bperp < RC {
                let outer = 2.0 * (RC * RC - bperp * bperp).sqrt() / rho;
                let inner = if bperp < RF { 2.0 * (RF * RF - bperp * bperp).sqrt() / rho } else { 0.0 };
                exp_fuel += inner;
                exp_clad += outer - inner;
            }
        }
        let tr = csg::trace(&t, o, d, f64::INFINITY, &mut sc);
        let (mut got_fuel, mut got_clad) = (0.0, 0.0);
        for s in &tr.segs {
            match region_name(s.region).as_str() {
                "fuel" => got_fuel += s.length,
                "clad" => got_clad += s.length,
                _ => {}
            }
        }
        for (got, exp) in [(got_fuel, exp_fuel), (got_clad, exp_clad)] {
            let rel = (got - exp).abs() / exp.max(1e-3);
            worst = worst.max(rel);
            assert!((got - exp).abs() <= 1e-11 * exp + 1e-12, "ray {k}: got {got} expected {exp}; o {o:?} d {d:?} end {:?} segs {:?}", tr.end, tr.segs.iter().take(6).collect::<Vec<_>>());
        }
        n_fuel += usize::from(exp_fuel > 0.0);
        n_clad += usize::from(exp_clad > 0.0);
    }
    assert!(n_fuel > 30 && n_clad > 50, "the rays really cross pins ({n_fuel}, {n_clad})");
    assert!(worst < 1e-11, "worst relative chord error {worst:e}");
}

#[test]
fn analytic_chords_through_each_cylinder_of_the_assembly_on_1e4_rays() {
    // No complement: each ray starts just inside the cladding of a random pin and runs to the far
    // side of it, so the tracker sees the pin's cylinders only.
    let fix = pin_assembly(17, PITCH, RF, RC, H).unwrap();
    let w = csg_world(&fix.model, &fix.complex, &CsgWorldOptions::default()).unwrap();
    let t = tracker(&w);
    let region_name = |id: sm_rt_api::RegionId| t.regions().get(id).map(|r| r.name.clone()).unwrap_or_default();
    let mut r = rng("bridge-1e4");
    let mut sc = Scratch::new();
    let delta = 1e-3;
    let (mut worst, mut n_fuel) = (0.0f64, 0usize);
    for k in 0..10_000 {
        let (i, j) = ((u01(&mut r) * 51.0).floor().min(50.0), (u01(&mut r) * 51.0).floor().min(50.0));
        let b = (2.0 * u01(&mut r) - 1.0) * 0.999 * RC;
        let th = std::f64::consts::TAU * u01(&mut r);
        let raw = v3(th.cos(), th.sin(), 0.6 * u01(&mut r) - 0.3);
        let d = raw * (1.0 / raw.norm());
        let rho = d.x.hypot(d.y);
        let n = (-d.y / rho, d.x / rho);
        let p0 = p3(i * PITCH + b * n.0, j * PITCH + b * n.1, 30.0 + 40.0 * u01(&mut r));
        let ab = b.abs();
        let half_clad = (RC * RC - ab * ab).sqrt() / rho;
        let half_fuel = if ab < RF { (RF * RF - ab * ab).sqrt() / rho } else { 0.0 };
        let o = p0 + d * (-(half_clad - delta));
        let tr = csg::trace(&t, o, d, f64::INFINITY, &mut sc);
        let (mut got_fuel, mut got_clad) = (0.0, 0.0);
        for s in &tr.segs {
            match region_name(s.region).as_str() {
                "fuel" => got_fuel += s.length,
                "clad" => got_clad += s.length,
                other => panic!("ray {k}: region {other}"),
            }
        }
        let (exp_fuel, exp_clad) = (2.0 * half_fuel, 2.0 * (half_clad - half_fuel) - delta);
        for (got, exp) in [(got_fuel, exp_fuel), (got_clad, exp_clad)] {
            // Near-tangent chords lose digits by conditioning (eps / sqrt(r^2 - b^2)): the relative
            // figure is taken over chords of at least 0.05 cm, all chords obey the bound below.
            if exp >= 0.05 {
                worst = worst.max((got - exp).abs() / exp);
            }
            assert!((got - exp).abs() <= 1e-11 * exp + 1e-12, "ray {k}: got {got} expected {exp} (b {b})");
        }
        n_fuel += usize::from(half_fuel > 0.0);
    }
    assert!(n_fuel > 7000, "{n_fuel}");
    assert!(worst < 1e-11, "worst relative chord error {worst:e}");
}

#[test]
fn the_bridged_assembly_has_no_overlap_and_the_bridge_is_deterministic() {
    let (fix, w) = assembly();
    let (scope, found) = check_world_overlaps(&w, 1500, 30, 0, MAX_FLAT_CELLS).unwrap();
    assert_eq!(scope, CheckScope::Flattened);
    assert!(found.iter().all(|f| f.issue.code != "csg.overlap"), "{:?}", found.iter().find(|f| f.issue.code == "csg.overlap"));
    // Over the cell limit the prototype leaf is checked alone and the scope says so.
    let (scope, found) = check_world_overlaps(&w, 1500, 30, 0, 100).unwrap();
    assert_eq!(scope, CheckScope::Prototypes);
    assert!(found.iter().all(|f| f.issue.code != "csg.overlap"));
    // Same bridge twice: same spec, same flattened tracker bits.
    let w2 = csg_world(&fix.model, &fix.complex, &CsgWorldOptions { bodies: None, complement: Some("void".into()) }).unwrap();
    assert_eq!(w.spec().content_hash(), w2.spec().content_hash());
    assert_eq!(tracker(&w).structure_hash(), tracker(&w2).structure_hash());
    // The flatten cap is a typed error.
    assert!(matches!(w.flatten(10), Err(BridgeError::Invalid(_))));
}

#[test]
fn bodies_select_volumes_in_prototypes_and_improper_placements_flip_signs() {
    let fix = pin_assembly(2, PITCH, RF, RC, H).unwrap();
    let vols: Vec<EntityId> = fix.complex.cells(Dim::Volume).to_vec();
    let fuel = vols.iter().copied().find(|v| fix.model.volume_color(*v).and_then(|c| fix.model.color(c)).is_some_and(|c| c.name == "fuel")).unwrap();
    let w = csg_world(&fix.model, &fix.complex, &CsgWorldOptions { bodies: Some(vec![fuel]), complement: None }).unwrap();
    assert_eq!(w.cells(0).len(), 1);
    assert_eq!(w.cells(0)[0].name, "fuel");
    assert_eq!(w.flat_cell_count(), 9 * 4);

    // A mirrored instance of the same pin: chords equal the unmirrored ones.
    let m = &fix.model;
    let proto = *m.instancing().prototypes.keys().next().unwrap();
    let mirror = Transform { m: [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]], t: Vector3::new(10.0, 0.0, H) };
    // Only the mirrored pin: a model whose sole instance is the mirrored prototype.
    let mut m2 = Model::new();
    for c in fix.model.colors() {
        m2.add_color(c.label, &c.name, c.domain).unwrap();
    }
    for (v, c) in fix.model.volume_coloring() {
        let name = &fix.model.color(c).unwrap().name;
        m2.color_volumes(m2.color_by_name(name).unwrap(), &[v]).unwrap();
    }
    let proto2 = m2.add_prototype(&m.instancing().prototypes[&proto].volumes.iter().copied().collect::<Vec<_>>()).unwrap();
    m2.add_instance(sm_model_api::InstanceRow::new(Fill::Prototype(proto2), mirror)).unwrap();
    let w = csg_world(&m2, &fix.complex, &CsgWorldOptions::default()).unwrap();
    let t = w.analytic_tracker(&BoundarySpec::default(), &BuildOptions::default(), MAX_FLAT_CELLS);
    // A mirrored single leaf is a Union root, flattened; the mirrored pin is at x = 10 and its chords match.
    let t = t.unwrap().0;
    let mut sc = Scratch::new();
    // Both z planes and the cylinder survive the improper map (x and z mirrored: the pin maps onto itself).
    let tr = csg::trace(&t, p3(9.9, 0.0, 50.0), v3(1.0, 0.0, 0.0), f64::INFINITY, &mut sc);
    assert!((tr.inside() - (RC + 0.1)).abs() < 1e-12, "{}", tr.inside());
    let tr = csg::trace(&t, p3(10.1, 0.0, 30.0), v3(0.0, 0.0, 1.0), f64::INFINITY, &mut sc);
    assert!((tr.inside() - (H - 30.0)).abs() < 1e-12, "{}", tr.inside());
}

#[test]
fn two_overlapping_cylinder_volumes_are_found_with_both_cells() {
    let mut c = CellComplex::new();
    let a = with_op(&mut c, OpId(1), |t| cylinder(t, 1.0, 3.0)).unwrap();
    let b = with_op(&mut c, OpId(2), |t| cylinder(t, 2.0, 3.0)).unwrap();
    let w = world_of(&c);
    let (_, found) = check_world_overlaps(&w, 4000, 100, 0, MAX_FLAT_CELLS).unwrap();
    let ov: Vec<_> = found.iter().filter(|f| f.issue.code == "csg.overlap").collect();
    assert!(!ov.is_empty());
    let p = ov[0].issue.location.unwrap();
    assert!(p.x.hypot(p.y) < 1.0 && (0.0..=3.0).contains(&p.z), "{p:?}");
    let mut vols: Vec<EntityId> = ov[0].cells.iter().map(|c| c.volume).collect();
    vols.sort();
    let mut want = vec![a.volume, b.volume];
    want.sort();
    assert_eq!(vols, want);
}
