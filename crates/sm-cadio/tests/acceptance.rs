//! Acceptance 2-9: geometry and topology of the imported fixtures, on both topology hosts.

mod common;

use core::f64::consts::PI;

use common::*;
use sm_cadio::{LengthUnit, NoteCode, ReadOptions, read_step};
use sm_core::Point3;
use sm_topo_api::{CellFlags, Dim, OpId, ShellKind, TopologyView, TopologyViewExt, euler_characteristic, fixtures};

const FACTORIES: [(&str, Factory); 2] = [("CellComplex", cell_complex), ("TopoStore", topo_store)];

/// Import on both hosts, assert the host-independent properties and return the reference host.
fn both(bytes: &[u8]) -> (Box<dyn sm_layer_api::TopoHost>, sm_cadio::Realized) {
    let m = read(bytes);
    let mut out = None;
    let mut hashes = Vec::new();
    for (name, f) in FACTORIES {
        let (h, r) = import_into(f, &m);
        let iss = issues(h.as_view());
        assert!(iss.is_empty(), "{name}: validator issues {iss:?}");
        hashes.push(h.as_view().canonical_hash());
        if out.is_none() {
            out = Some((h, r));
        }
    }
    assert_eq!(hashes[0], hashes[1], "CellComplex and TopoStore give equal canonical hashes");
    out.expect("one host")
}

fn bbox(view: &dyn TopologyView) -> ([f64; 3], [f64; 3]) {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for &v in view.cells(Dim::Vertex) {
        let sm_topo_api::Carrier::Point(p) = view.carrier(v) else { panic!("vertex carrier") };
        for (k, x) in [p.x, p.y, p.z].into_iter().enumerate() {
            lo[k] = lo[k].min(x);
            hi[k] = hi[k].max(x);
        }
    }
    (lo, hi)
}

#[test]
fn brick_2x3x4_counts_volume_area_bbox_and_hash() {
    let (h, r) = both(&data("brick_2x3x4.stp"));
    let v = h.as_view();
    assert_eq!((count(v, Dim::Vertex), count(v, Dim::Edge), count(v, Dim::Face)), (8, 12, 6));
    assert_eq!(euler_characteristic(v), 2);
    assert!(rel(volume(v, r.volumes[0]), 24.0) <= 1e-9);
    assert!(rel(total_area(v), 52.0) <= 1e-9);
    let (lo, hi) = bbox(v);
    assert_eq!((lo, hi), ([1.0, 2.0, 3.0], [3.0, 5.0, 7.0]), "bit-exact bbox");
    // The same cells as the reference fixture built at the same operation id.
    let mut reference = sm_topo_api::CellComplex::new();
    fixtures::with_op(&mut reference, OpId(1), |t| fixtures::brick(t, Point3::new(1.0, 2.0, 3.0), [2.0, 3.0, 4.0])).expect("fixture brick");
    assert_eq!(v.canonical_hash(), reference.canonical_hash(), "imported brick hashes like fixtures::brick");
    let mut store = sm_topo::TopoStore::new();
    fixtures::with_op(&mut store, OpId(1), |t| fixtures::brick(t, Point3::new(1.0, 2.0, 3.0), [2.0, 3.0, 4.0])).expect("fixture brick");
    assert_eq!(v.canonical_hash(), store.canonical_hash());
}

#[test]
fn ap203_and_ap214_bricks_give_the_same_topology_hash() {
    let a = read(&data("brick_2x3x4.stp"));
    let b = read(&data("brick_ap203.stp"));
    assert_ne!(a.schema, b.schema, "only the schema string differs");
    assert!(b.schema.contains("CONFIG_CONTROL_DESIGN"));
    let (ha, _) = import_into(cell_complex, &a);
    let (hb, _) = import_into(cell_complex, &b);
    assert_eq!(ha.as_view().canonical_hash(), hb.as_view().canonical_hash());
}

#[test]
fn inch_brick_converts_to_millimetres_on_request() {
    let bytes = data("brick_inch.stp");
    // Kept in inches: volume 6, and the units note names the inch.
    let m = read(&bytes);
    let (h, r) = import_into(cell_complex, &m);
    assert!(rel(volume(h.as_view(), r.volumes[0]), 6.0) <= 1e-9);
    let note = m.notes.iter().find(|n| n.code == NoteCode::Units).expect("step.units note");
    assert!(note.message.contains("inch"), "{}", note.message);
    assert!(note.message.contains("no conversion"), "{}", note.message);
    // Converted: 6 x 25.4^3.
    let opts = ReadOptions { to_unit: Some(LengthUnit::Mm), ..ReadOptions::default() };
    let mm = read_step(&bytes, &opts).expect("read mm");
    let (h, r) = import_into(cell_complex, &mm);
    assert!(rel(volume(h.as_view(), r.volumes[0]), 98_322.384) <= 1e-9, "{}", volume(h.as_view(), r.volumes[0]));
    assert_eq!(mm.solids[0].units.scale, 25.4);
    // Converted to the unit it is already in: the scale is exactly one.
    let opts = ReadOptions { to_unit: Some(LengthUnit::In), ..ReadOptions::default() };
    assert_eq!(read_step(&bytes, &opts).expect("read in").solids[0].units.scale, 1.0);
}

#[test]
fn occ_cylinder_and_hand_written_frustum() {
    let (h, r) = both(sm_testkit::step::CYLINDER_R2_H5.bytes);
    let v = h.as_view();
    assert_eq!((count(v, Dim::Vertex), count(v, Dim::Edge), count(v, Dim::Face)), (2, 3, 3));
    assert_eq!(euler_characteristic(v), 2);
    assert!(rel(volume(v, r.volumes[0]), 20.0 * PI) <= 1e-9);
    assert!(rel(total_area(v), 28.0 * PI) <= 1e-9);

    let (h, r) = both(&data("frustum_r2_r1_h3.stp"));
    let v = h.as_view();
    assert_eq!((count(v, Dim::Vertex), count(v, Dim::Edge), count(v, Dim::Face)), (2, 3, 3));
    assert!(rel(volume(v, r.volumes[0]), 7.0 * PI) <= 1e-9);
    let slant = 10f64.sqrt();
    let lateral = r.faces.iter().map(|&f| v.face_moments(f).expect("moments").area).fold(0.0, f64::max);
    assert!(rel(lateral, 3.0 * PI * slant) <= 1e-9, "lateral area {lateral}");
    assert!(rel(total_area(v), 3.0 * PI * slant + 5.0 * PI) <= 1e-9);
}

#[test]
fn seam_sphere_proves_the_pole_rule() {
    let (h, r) = both(&data("sphere_seam_r3.stp"));
    let v = h.as_view();
    assert_eq!(count(v, Dim::Face), 1);
    assert_eq!(count(v, Dim::Vertex), 2);
    let degenerate = r.edges.iter().filter(|&&e| v.flags(e).contains(CellFlags::DEGENERATE)).count();
    assert_eq!((count(v, Dim::Edge), degenerate), (3, 2), "one seam plus two DEGENERATE pole edges");
    assert_eq!(r.poles, 2);
    assert_eq!(euler_characteristic(v), 2);
    assert!(rel(volume(v, r.volumes[0]), 36.0 * PI) <= 1e-9);
    assert!(rel(total_area(v), 36.0 * PI) <= 1e-9);
    // The pole pcurves are the only pcurves; each sweeps one full turn of u.
    let f = r.faces[0];
    let mut with_pcurve = 0;
    for lp in v.loops(f) {
        for k in lp.keys() {
            let pc = v.pcurve(k);
            assert_eq!(pc.is_some(), v.flags(k.edge).contains(CellFlags::DEGENERATE), "pcurves only on pole coedges");
            if let Some(pc) = pc {
                with_pcurve += 1;
                assert!((pc.range().len() - 2.0 * PI).abs() < 1e-12);
            }
        }
    }
    assert_eq!(with_pcurve, 2);
}

#[test]
fn occ_torus_matches_the_fixture_numerically() {
    let (h, r) = both(sm_testkit::step::TORUS_R5_R1.bytes);
    let v = h.as_view();
    assert_eq!((count(v, Dim::Vertex), count(v, Dim::Edge), count(v, Dim::Face)), (1, 2, 1));
    assert_eq!(euler_characteristic(v), 0);
    assert!(rel(volume(v, r.volumes[0]), 10.0 * PI * PI) <= 1e-9);
    assert!(rel(total_area(v), 20.0 * PI * PI) <= 1e-9);
    // Against fixtures::torus at the same operation id. The reader leaves pcurves unset on
    // periodic surfaces (the geometry queries unwrap seams by projection) while the fixture sets
    // all four, so the hashes differ by exactly those pcurves: give the imported cells the
    // fixture's pcurves in the same operation and they hash alike.
    let mut reference = sm_topo_api::CellComplex::new();
    let ids = fixtures::with_op(&mut reference, OpId(1), |t| fixtures::torus(t, 5.0, 1.0)).expect("fixture torus");
    assert_ne!(v.canonical_hash(), reference.canonical_hash(), "pcurves are part of the hash");
    let m = read(sm_testkit::step::TORUS_R5_R1.bytes);
    for (name, f) in FACTORIES {
        let mut host = f();
        host.begin_op(OpId(1)).expect("begin");
        let r = sm_cadio::realize(&m, &mut *host, &sm_cadio::RealizeOptions::default()).expect("realize");
        let face = r.faces[0];
        let (eu, ev) = (r.edges[0], r.edges[1]);
        let full = sm_geom_api::Interval::new(0.0, 2.0 * PI);
        let line2 = |o: (f64, f64), d: (f64, f64)| sm_geom_api::PCurve::Line2(sm_geom_api::Line2 { origin: sm_geom_api::Uv::new(o.0, o.1), dir: sm_geom_api::Uv::new(d.0, d.1), range: full });
        let key = |edge, side| sm_topo_api::CoedgeKey { face, edge, side };
        use sm_topo_api::Side::{Left, Right};
        host.set_pcurve(key(eu, Left), Some(line2((0.0, 0.0), (1.0, 0.0)))).expect("pcurve");
        host.set_pcurve(key(ev, Left), Some(line2((2.0 * PI, 0.0), (0.0, 1.0)))).expect("pcurve");
        host.set_pcurve(key(eu, Right), Some(line2((0.0, 2.0 * PI), (1.0, 0.0)))).expect("pcurve");
        host.set_pcurve(key(ev, Right), Some(line2((0.0, 0.0), (0.0, 1.0)))).expect("pcurve");
        host.end_op().expect("end");
        assert_eq!(host.as_view().canonical_hash(), reference.canonical_hash(), "{name}: torus hashes like fixtures::torus once the pcurves match");
    }
    let rv = reference.volume_moments(ids.volume).expect("moments").volume;
    assert!(rel(volume(v, r.volumes[0]), rv) <= 1e-9);
    let rf = reference.face_moments(ids.face).expect("moments").area;
    assert!(rel(total_area(v), rf) <= 1e-9);
}

/// The cylindrical-fillet block of OCC and the hand-written rational NURBS fillet.
#[test]
fn fillet_blocks_cylindrical_and_nurbs() {
    let (h, r) = both(sm_testkit::step::BOX_FILLET1.bytes);
    let v = h.as_view();
    assert_eq!(count(v, Dim::Face), 7);
    assert!(rel(volume(v, r.volumes[0]), 990.0 + 2.5 * PI) <= 1e-9);
    assert!(rel(total_area(v), 578.0 + 5.5 * PI) <= 1e-9);

    let m = read(&data("block_fillet_nurbs.stp"));
    let kinds: Vec<&str> = m.solids[0]
        .surfaces
        .iter()
        .map(|s| match s.def {
            sm_geom_api::SurfaceDef::Plane(_) => "plane",
            sm_geom_api::SurfaceDef::Nurbs(_) => "nurbs",
            _ => "other",
        })
        .collect();
    assert_eq!(kinds.iter().filter(|k| **k == "plane").count(), 6);
    assert_eq!(kinds.iter().filter(|k| **k == "nurbs").count(), 1);
    let (h, r) = both(&data("block_fillet_nurbs.stp"));
    let v = h.as_view();
    assert_eq!(count(v, Dim::Face), 7);
    // The default quadrature of `volume_moments` (5-point Gauss, one panel along a
    // non-periodic parameter) integrates the rational fillet to about 4e-8; see W3_REPORT.md.
    assert!(rel(volume(v, r.volumes[0]), 960.0 + 10.0 * PI) <= 1e-7, "{}", volume(v, r.volumes[0]));
    assert!(rel(total_area(v), 552.0 + 12.0 * PI) <= 1e-8, "{}", total_area(v));
}

#[test]
fn brick_with_void_and_two_bricks() {
    let (h, r) = both(&data("brick_with_void.stp"));
    let v = h.as_view();
    assert_eq!(r.volumes.len(), 1);
    assert_eq!(count(v, Dim::Face), 12);
    assert_eq!(euler_characteristic(v), 4);
    assert_eq!((v.shell_kind(r.volumes[0], 0), v.shell_kind(r.volumes[0], 1)), (Some(ShellKind::Outer), Some(ShellKind::Void)));
    assert!(rel(volume(v, r.volumes[0]), 992.0) <= 1e-9);
    // The inward-plane spelling of the same solid hashes like the stage-0 fixture.
    let (h2, r2) = both(&data("brick_with_void_inward.stp"));
    let mut reference = sm_topo_api::CellComplex::new();
    fixtures::with_op(&mut reference, OpId(1), |t| fixtures::brick_with_void(t, Point3::new(0.0, 0.0, 0.0), [10.0; 3], Point3::new(4.0, 4.0, 4.0), [2.0; 3])).expect("fixture");
    assert_eq!(h2.as_view().canonical_hash(), reference.canonical_hash());
    assert!(rel(volume(h2.as_view(), r2.volumes[0]), 992.0) <= 1e-9);

    let m = read(&data("two_bricks.stp"));
    assert_eq!(m.solids.len(), 2);
    let (h, r) = import_into(cell_complex, &m);
    assert_eq!(r.bodies.len(), 2);
    assert!(r.bodies[0] < r.bodies[1], "ids ascend in file order");
    assert!(rel(volume(h.as_view(), r.volumes[0]), 1.0) <= 1e-9);
    assert!(rel(volume(h.as_view(), r.volumes[1]), 6.0) <= 1e-9);
    assert_eq!(count(h.as_view(), Dim::Face), 12, "no cell is shared across solids");
}

#[test]
fn milestone_part_through_the_reader() {
    let (h, r) = both(sm_testkit::w3a::STEP);
    let v = h.as_view();
    assert_eq!(r.bodies.len(), 3);
    assert_eq!(count(v, Dim::Face), 19);
    for (i, want) in sm_testkit::w3a::VOLUMES.iter().enumerate() {
        assert!(rel(volume(v, r.volumes[i]), *want) <= 1e-12, "volume {i}");
    }
    let total: f64 = r.volumes.iter().map(|&x| volume(v, x)).sum();
    assert!(rel(total, sm_testkit::w3a::TOTAL_VOLUME) <= 1e-12);
    let m = read(sm_testkit::w3a::STEP);
    assert!(m.notes.iter().all(|n| n.code != NoteCode::UnsupportedEntity));
    assert!(m.solids.iter().flat_map(|s| &s.surfaces).all(|s| matches!(s.def, sm_geom_api::SurfaceDef::Plane(_))));
}

/// The OCC loft over a degree-13 B-spline surface, closed but not periodic (ruling C2). The
/// platform measure `volume_moments` must give OCC's 54.454271953 to 1e-6: the reader cuts the
/// closed lateral face into four faces so the seam coedges cannot be confused (see `split.rs`).
#[test]
fn occ_loft_volume_through_volume_moments() {
    let m = read(sm_testkit::step::LOFT_BSPLINE.bytes);
    let s = &m.solids[0];
    // 2 caps + 4 lateral pieces; vertices 2 + 6 cut points; edges 2 x 4 arcs + 3 generators + seam.
    assert_eq!((s.faces.len(), s.vertices.len(), s.edges.len()), (6, 8, 12));
    assert!(m.notes.iter().any(|n| n.code == NoteCode::SplitClosedFace && n.severity == sm_cadio::NoteSeverity::Info), "{:?}", m.notes);
    let (h, r) = both(sm_testkit::step::LOFT_BSPLINE.bytes);
    let v = h.as_view();
    assert!(issues(v).is_empty());
    assert_eq!((count(v, Dim::Vertex), count(v, Dim::Edge), count(v, Dim::Face)), (8, 12, 6));
    assert_eq!(euler_characteristic(v), 2);
    let got = volume(v, r.volumes[0]);
    assert!(rel(got, sm_testkit::step::LOFT_BSPLINE.volume) <= 1e-6, "{got:.12}");
    // Independent cross-check: tensor Gauss over each patch plus the elliptical caps in closed
    // form (the circle r = 2 at z = 0 contributes nothing, the 3 x 1.5 ellipse at z = 4 gives 4 pi 1.5 * 3 / 3).
    let caps = 4.0 * PI * 1.5 * 3.0 / 3.0;
    let patches: f64 = s
        .faces
        .iter()
        .filter(|f| matches!(s.surfaces[f.surface].def, sm_geom_api::SurfaceDef::Nurbs(_)))
        .map(|f| f.sense.sign() * patch_volume_term(s.surfaces[f.surface].def.as_surface().expect("parametric"), 20))
        .sum();
    assert!(rel(patches + caps, 54.454_272_576_4) <= 1e-9, "{:.12}", patches + caps);
}
// ---- Stretch S1: the pole encodings OCC writes

#[test]
fn s1_occ_sphere_vertex_loop_gets_its_seam_and_poles() {
    let (h, r) = both(sm_testkit::step::SPHERE_R3.bytes);
    let v = h.as_view();
    assert_eq!(count(v, Dim::Vertex), 2, "the file's pole plus the synthesized antipode");
    let degenerate = r.edges.iter().filter(|&&e| v.flags(e).contains(CellFlags::DEGENERATE)).count();
    assert_eq!((count(v, Dim::Edge), degenerate), (3, 2), "one seam and two DEGENERATE edges");
    assert_eq!(euler_characteristic(v), 2);
    assert!(rel(volume(v, r.volumes[0]), 36.0 * PI) <= 1e-9);
    assert!(rel(total_area(v), 36.0 * PI) <= 1e-9);
}

#[test]
fn s1_occ_cone_apex_gets_its_pole_edge() {
    let (h, r) = both(sm_testkit::step::CONE_R2_H3.bytes);
    let v = h.as_view();
    assert_eq!((count(v, Dim::Vertex), count(v, Dim::Face), r.poles), (2, 2, 1));
    let degenerate = r.edges.iter().filter(|&&e| v.flags(e).contains(CellFlags::DEGENERATE)).count();
    assert_eq!((count(v, Dim::Edge), degenerate), (3, 1), "circle, generator and the apex pole edge");
    assert!(rel(volume(v, r.volumes[0]), 4.0 * PI) <= 1e-9);
    let lateral = 2.0 * PI * 13f64.sqrt();
    assert!(rel(total_area(v), lateral + 4.0 * PI) <= 1e-9, "{}", total_area(v));
    // The stage-0 fixture has the same cells and moments (its hash differs through the
    // pcurves it sets on every lateral coedge and its frame choices).
    let mut reference = sm_topo_api::CellComplex::new();
    let ids = fixtures::with_op(&mut reference, OpId(1), |t| fixtures::cone(t, 2.0, 3.0)).expect("fixture cone");
    assert_eq!((count(&reference, Dim::Vertex), count(&reference, Dim::Edge), count(&reference, Dim::Face)), (2, 3, 2));
    assert!(rel(volume(v, r.volumes[0]), reference.volume_moments(ids.volume).expect("moments").volume) <= 1e-9);
}

// ---- Stretch S3: assemblies

fn bbox_of_solid(s: &sm_cadio::Solid) -> ([f64; 3], [f64; 3]) {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for v in &s.vertices {
        for (k, x) in [v.point.x, v.point.y, v.point.z].into_iter().enumerate() {
            lo[k] = lo[k].min(x);
            hi[k] = hi[k].max(x);
        }
    }
    (lo, hi)
}

#[test]
fn s3_relationship_placements_are_flattened_into_instances() {
    let m = read(&data("assembly_pins.stp"));
    assert_eq!(m.solids.len(), 2, "one pin placed twice");
    assert_eq!((m.solids[0].id, m.solids[0].instance), (m.solids[1].id, 0));
    assert_eq!(m.solids[1].instance, 1);
    // Instance 0: translated by (10, 0, 0); instance 1: (x, y, z) -> (-y, x, z) + (0, 5, 0).
    assert_eq!(bbox_of_solid(&m.solids[0]), ([10.0, 0.0, 0.0], [11.0, 2.0, 3.0]));
    assert_eq!(bbox_of_solid(&m.solids[1]), ([-2.0, 5.0, 0.0], [0.0, 6.0, 3.0]));
    let note = m.notes.iter().find(|n| n.code == NoteCode::AssemblyFlattened).expect("flattened info");
    assert!(note.message.contains("2 solid instance"), "{}", note.message);
    assert!(m.notes.iter().all(|n| n.code != NoteCode::AssemblyTransformsIgnored));
    // Both instances are valid solids of volume 6 with outward normals (proper rigid maps).
    let (h, r) = both(&data("assembly_pins.stp"));
    for v in &r.volumes {
        assert!(rel(volume(h.as_view(), *v), 6.0) <= 1e-9);
    }
    assert!(rel(total_area(h.as_view()), 2.0 * 22.0) <= 1e-9);
}

#[test]
fn s3_mapped_items_are_flattened_with_their_target_frame() {
    let m = read(&data("assembly_mapped.stp"));
    assert_eq!(m.solids.len(), 1);
    // R p = (x, -z, y) + (0, 0, 20) maps [0,1]x[0,2]x[0,3] to [0,1] x [-3,0] x [20,22].
    assert_eq!(bbox_of_solid(&m.solids[0]), ([0.0, -3.0, 20.0], [1.0, 0.0, 22.0]));
    let (h, r) = both(&data("assembly_mapped.stp"));
    assert!(rel(volume(h.as_view(), r.volumes[0]), 6.0) <= 1e-9);
}

#[test]
fn s3_flattening_can_be_switched_off_and_then_warns() {
    let opts = ReadOptions { flatten_assemblies: false, ..ReadOptions::default() };
    let m = read_step(&data("assembly_pins.stp"), &opts).expect("read");
    assert_eq!(m.solids.len(), 1, "the pin once, in local coordinates");
    assert_eq!(bbox_of_solid(&m.solids[0]), ([0.0, 0.0, 0.0], [1.0, 2.0, 3.0]));
    let w = m.notes.iter().find(|n| n.code == NoteCode::AssemblyTransformsIgnored).expect("the warning");
    assert_eq!(w.severity, sm_cadio::NoteSeverity::Warning);
    assert!(m.notes.iter().all(|n| n.code != NoteCode::AssemblyFlattened));
}

#[test]
fn s3_the_milestone_part_keeps_its_coordinates() {
    // Identity placements: the flattened part equals the unflattened one bit for bit.
    let a = read(sm_testkit::w3a::STEP);
    let b = read_step(sm_testkit::w3a::STEP, &ReadOptions { flatten_assemblies: false, ..ReadOptions::default() }).expect("read");
    assert_eq!(a.solids, b.solids);
}

/// An inward sphere (`same_sense = .F.`) has its carrier flipped back by the orientation check,
/// and the pole sweeps are recomputed for the flipped carrier.
#[test]
fn inward_sphere_and_cone_are_flipped_and_keep_valid_poles() {
    for (name, bytes, vol) in [
        ("sphere", data("sphere_seam_r3.stp"), 36.0 * PI),
        ("occ cone", sm_testkit::step::CONE_R2_H3.bytes.to_vec(), 4.0 * PI),
    ] {
        let t = String::from_utf8(bytes).expect("ascii");
        // Flip the same_sense flag of every ADVANCED_FACE.
        let mut out = String::new();
        for line in t.split_inclusive('\n') {
            if line.contains("ADVANCED_FACE(") && line.trim_end().ends_with(".T.);") {
                out.push_str(&line.replacen(".T.);", ".F.);", 1));
            } else if line.contains("ADVANCED_FACE(") && line.trim_end().ends_with(".F.);") {
                out.push_str(&line.replacen(".F.);", ".T.);", 1));
            } else {
                out.push_str(line);
            }
        }
        // OCC wraps long lines: the flag may sit on a continuation line.
        let out = if out == t { t.replace("),#22,.T.);", "),#22,.F.);") } else { out };
        assert_ne!(out, t, "{name}: the fixture was modified");
        let m = read_step(out.as_bytes(), &ReadOptions::default()).expect("read");
        let (h, r) = import_into(cell_complex, &m);
        assert!(!r.inverted.is_empty(), "{name}: flagged as inverted");
        assert!(rel(volume(h.as_view(), r.volumes[0]), vol) <= 1e-9, "{name}: {}", volume(h.as_view(), r.volumes[0]));
        assert!(issues(h.as_view()).is_empty(), "{name}: {:?}", issues(h.as_view()));
    }
}
