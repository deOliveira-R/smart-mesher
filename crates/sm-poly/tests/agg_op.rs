//! W4-POLY-AGG: `mesh.agglomerate` end to end through a session (coarse mesh, `LevelMap`, face
//! map, per-pin CMFD mode, replay and fork).
#![allow(missing_docs)]

mod common;

use common::{id, rel};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::json;
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_layer_api::{CoreLayers, MeshEntry, MeshLayer, ModelLayer, insert_mesh, model_txn};
use sm_mesh_api::{
    CellIx, FaceIx, GeomTol, MeshCatalog as _, MeshEntityKind, MeshRead, NO_CELL, SerialAlloc, cell_volume, face_area_vector, validate,
    validate_geometry,
};
use sm_model_api::{LatticeFrame, Layout, RegionTag, SlotIx};
use sm_op_api::{ApplyCtx, DerivedCache, Delta, Engine, Hash256, Meta, Op, OpError, OpRecord, QuietControl, Registry, Rev, Services, Session, TimelineError, TypedOp};
use sm_poly::{DualPoints, PolyOptions, PolyPlugin, VertexClassResolver, dualize};
use sm_quality::{Metric, Options, report};
use sm_testkit::meshes::{self, CellSpec};
use std::collections::BTreeMap;

/// Test-only op: inserts a fixture mesh (and, for the pin lattice, the model lattice tags).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct Insert {
    kind: String,
}

const PINS: usize = 3;
/// A pin is 4 x 4 x 2 unit hexes: a 2 x 2 x 2 fuel core (volume `2p+1`) in a moderator ring
/// (volume `2p+2`), `p` the pin number.
fn pin_of(i: usize, j: usize) -> u64 {
    ((j / 4) * PINS + i / 4) as u64
}

fn hex_lattice(nx: usize, ny: usize, nz: usize, vol_of: impl Fn(usize, usize) -> u64, alloc: &mut SerialAlloc) -> sm_mesh_api::reference::ReferenceMesh {
    let mut pts = Vec::new();
    for k in 0..=nz {
        for j in 0..=ny {
            for i in 0..=nx {
                pts.push([i as f64, j as f64, k as f64]);
            }
        }
    }
    let ix = |i: usize, j: usize, k: usize| (k * (ny + 1) + j) * (nx + 1) + i;
    let mut cells = Vec::new();
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                let corners = [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]].map(|o| ix(i + o[0], j + o[1], k + o[2])).to_vec();
                cells.push((CellSpec::Template(sm_mesh_api::CellShape::Hex, corners), vol_of(i, j)));
            }
        }
    }
    meshes::build_mesh_volumes(&pts, &cells, alloc).expect("hex lattice")
}

fn pin_lattice(alloc: &mut SerialAlloc) -> sm_mesh_api::reference::ReferenceMesh {
    let vol = |i: usize, j: usize| {
        let fuel = (1..=2).contains(&(i % 4)) && (1..=2).contains(&(j % 4));
        2 * pin_of(i, j) + if fuel { 1 } else { 2 }
    };
    hex_lattice(4 * PINS, 4 * PINS, 2, vol, alloc)
}

impl TypedOp for Insert {
    const NAME: &'static str = "test.insert";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let quanta = ctx.layer::<ModelLayer>()?.hash_quanta();
        let before = ctx.layer::<MeshLayer>()?.alloc().clone();
        let mut alloc = before.clone();
        let vols = |n: u64| (1..=n).map(|v| id(EntityKind::TopoVolume, v)).collect::<Vec<_>>();
        let entry = match self.kind.as_str() {
            "dual6" => {
                let f = meshes::tet_block([6, 6, 6], [1.0; 3]).expect("tet_block");
                let o = PolyOptions::new().with_all_polyhedra(true).with_dual(DualPoints::Circumcentric);
                let (fin, _) = dualize(&f.mesh, &o, &VertexClassResolver, sm_mesh::MeshBuilder::new(), &mut alloc).map_err(|e| OpError::Failed(e.to_string()))?;
                MeshEntry::new(Box::new(fin.mesh), vols(1), None, quanta)
            }
            "two" => {
                alloc = SerialAlloc::new();
                let m = hex_lattice(4, 2, 2, |i, _| if i < 2 { 1 } else { 2 }, &mut alloc);
                MeshEntry::new(Box::new(m), vols(2), None, quanta)
            }
            "pins" => {
                alloc = SerialAlloc::new();
                let m = pin_lattice(&mut alloc);
                MeshEntry::new(Box::new(m), vols(2 * (PINS * PINS) as u64), None, quanta)
            }
            other if other.starts_with("hex:") => {
                let n: Vec<usize> = other[4..].split(':').map(|x| x.parse().expect("n")).collect();
                alloc = SerialAlloc::new();
                let m = hex_lattice(n[0], n[1], n[2], |_, _| 1, &mut alloc);
                MeshEntry::new(Box::new(m), vols(1), None, quanta)
            }
            other => panic!("unknown kind {other}"),
        };
        if self.kind == "pins" {
            // Frame `core` (3 x 3 pins, pitch 4) and one slot per pin on both of its volumes.
            model_txn(
                ctx,
                |m, _| {
                    let frame = LatticeFrame::new(Layout::Cartesian { n: [PINS as u32, PINS as u32, 1], pitch: Vector3::new(4.0, 4.0, 2.0) }, Point3::new(2.0, 2.0, 1.0), None);
                    m.define_lattice_frame("core", frame).map_err(|e| OpError::Failed(e.to_string()))?;
                    for p in 0..PINS * PINS {
                        for v in [2 * p + 1, 2 * p + 2] {
                            let slots = BTreeMap::from([("core".to_string(), SlotIx::new((p % PINS) as i32, (p / PINS) as i32, 0))]);
                            m.set_region_tag(id(EntityKind::TopoVolume, v as u64), RegionTag::new(slots, None, None)).map_err(|e| OpError::Failed(e.to_string()))?;
                        }
                    }
                    Ok(())
                },
                |_, _| vec![],
            )?;
        }
        insert_mesh(ctx, entry, &before, alloc, "mesh")
    }
}

fn engine() -> Engine {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::default()).expect("layers");
    reg.install(&PolyPlugin).expect("poly");
    reg.register::<Insert>().expect("test op");
    Engine::new(std::sync::Arc::new(reg), Services::new(), std::sync::Arc::new(DerivedCache::default())).expect("engine")
}

fn run(s: &mut Session, op: &dyn Op) -> Result<(Hash256, Delta), TimelineError> {
    let q = QuietControl::new();
    s.apply(op, Meta::new("test", "", ""), &q.control())
}

fn run_json(s: &mut Session, name: &str, params: serde_json::Value) -> Result<(Hash256, Delta), TimelineError> {
    let rec = OpRecord::new(name, 1, params).expect("record");
    let q = QuietControl::new();
    s.apply_record(rec, Meta::new("test", "", ""), &q.control())
}

fn session_with(kind: &str) -> Session {
    let mut s = Session::new(engine());
    run(&mut s, &Insert { kind: kind.into() }).expect("insert");
    s
}

fn m1() -> EntityId {
    id(EntityKind::Mesh, 1)
}
fn m2() -> EntityId {
    id(EntityKind::Mesh, 2)
}

fn mesh_of(s: &Session, m: EntityId) -> std::sync::Arc<dyn sm_layer_api::MeshHost> {
    s.state().layer::<MeshLayer>().unwrap().get(m).unwrap().mesh.clone()
}

fn vol(m: &dyn MeshRead) -> f64 {
    (0..m.cell_count()).map(|c| cell_volume(m, CellIx::new(c))).sum()
}

/// Checks the link, the restriction of volumes and the face map areas.
fn check_link(s: &Session, fine: EntityId, coarse: EntityId) {
    let layer = s.state().layer::<MeshLayer>().unwrap();
    let from = layer.maps_from(fine);
    assert_eq!(from.len(), 1);
    assert_eq!(from[0].0, coarse);
    let map = from[0].1;
    let (f, c) = (layer.mesh(fine).unwrap(), layer.mesh(coarse).unwrap());
    map.check_meshes(f, c).expect("map valid for both meshes");
    let fv: Vec<f64> = (0..f.cell_count()).map(|i| cell_volume(f, CellIx::new(i))).collect();
    let restricted = map.restrict_sum(&fv).expect("restrict");
    for (i, r) in restricted.iter().enumerate() {
        assert!(rel(*r, cell_volume(c, CellIx::new(i))) < 1e-12, "coarse cell {i}: {r} vs {}", cell_volume(c, CellIx::new(i)));
    }
    // each coarse face carries exactly the area of its fine faces
    let fm = map.face_map().expect("face map");
    let mut area = vec![0.0; c.face_count()];
    for (fi, &g) in fm.iter().enumerate() {
        if g != NO_CELL {
            area[g as usize] += face_area_vector(f, FaceIx::new(fi)).norm();
        }
    }
    for (g, a) in area.iter().enumerate() {
        let want = face_area_vector(c, FaceIx::new(g)).norm();
        assert!(rel(*a, want) < 1e-12, "coarse face {g}: fine area {a} vs {want}");
    }
}

fn quality(m: &dyn MeshRead) -> (f64, f64) {
    let r = report(m, &[Metric::ScaledJacobian, Metric::EdgeRatio], &Options::default()).expect("quality");
    let (sj, er) = (r[0].1.min, r[1].1.max);
    eprintln!("quality: cells {} min scaled Jacobian {sj:.12} max edge ratio {er:.6}", m.cell_count());
    (sj, er)
}

#[test]
fn lattice_blocks_give_the_hand_count_and_an_exact_map() {
    let mut s = session_with("dual6");
    let fine = mesh_of(&s, m1());
    assert_eq!(fine.as_read().cell_count(), 343);
    // Bins of width 2 from 0.5 on: per axis the dual cells {0}, {1,2}, {3,4}, {5,6} (widths 0.5, 2, 2, 1.5).
    let (_, d) = run_json(&mut s, "mesh.agglomerate", json!({"mesh": "Mesh#1", "mode": {"blocks": {"size": [2.0, 2.0, 2.0], "origin": [0.5, 0.5, 0.5]}}})).expect("agglomerate");
    assert_eq!(d.outputs["mesh"], vec![m2()]);
    assert_eq!(d.successors[&m1()], vec![m1(), m2()]);
    let coarse = mesh_of(&s, m2());
    let c = coarse.as_read();
    // hand count: 4^3 coarse cells, 3 * 4 * 4 * 5 faces, 5^3 vertices
    assert_eq!(c.cell_count(), 64);
    assert_eq!(c.count(MeshEntityKind::Face), 240);
    assert_eq!(c.count(MeshEntityKind::Vertex), 125);
    assert!(rel(vol(c), 216.0) < 1e-12);
    // 8 interior coarse cubes of volume 8, all coarse cells boxes with the product volumes
    let w = [0.5, 2.0, 2.0, 1.5];
    let mut want: Vec<f64> = (0..64).map(|i| w[i % 4] * w[(i / 4) % 4] * w[i / 16]).collect();
    let mut got: Vec<f64> = (0..64).map(|i| cell_volume(c, CellIx::new(i))).collect();
    want.sort_by(f64::total_cmp);
    got.sort_by(f64::total_cmp);
    for (a, b) in got.iter().zip(&want) {
        assert!(rel(*a, *b) < 1e-12, "{a} vs {b}");
    }
    assert_eq!(got.iter().filter(|v| (**v - 8.0).abs() < 1e-12).count(), 8);
    for i in 0..64 {
        let cv = sm_quality::cell_convexity(c, CellIx::new(i), sm_quality::RT_PLANARITY_TOL);
        assert!(cv.convex && cv.faces == 6, "coarse cell {i}: {cv:?}");
    }
    assert!(validate(c).is_empty() && validate_geometry(c, &GeomTol::default()).is_empty());
    check_link(&s, m1(), m2());
    // children per coarse cell: product of the per-axis counts 1, 2, 2, 2
    let layer = s.state().layer::<MeshLayer>().unwrap();
    let map = layer.maps_from(m1())[0].1;
    let counts = [1usize, 2, 2, 2];
    let mut kids: Vec<usize> = (0..64).map(|i| map.fines_of(CellIx::new(i)).len()).collect();
    let mut wk: Vec<usize> = (0..64).map(|i| counts[i % 4] * counts[(i / 4) % 4] * counts[i / 16]).collect();
    kids.sort_unstable();
    wk.sort_unstable();
    assert_eq!(kids, wk);
    // boundary: the outer skin keeps its area 216 * ... = 6 * 36 and its classification
    let area_on = |m: &dyn MeshRead| -> f64 { (0..m.face_count()).map(FaceIx::new).filter(|f| m.is_boundary_face(*f)).map(|f| face_area_vector(m, f).norm()).sum() };
    assert!(rel(area_on(c), 216.0) < 1e-12);
    assert_eq!(c.count(MeshEntityKind::Face) - (0..c.face_count()).filter(|f| !c.is_boundary_face(FaceIx::new(*f))).count(), 96);
    let (sj, er) = quality(c);
    assert!(sj > 1.0 - 1e-12);
    // edge ratio of the boxes: longest edge 2 over shortest 0.5
    assert!(rel(er, 4.0) < 1e-12, "{er}");
}

#[test]
fn per_pin_mode_gives_one_coarse_cell_per_pin() {
    let mut s = session_with("pins");
    let fine = mesh_of(&s, m1());
    assert_eq!(fine.as_read().cell_count(), 12 * 12 * 2);
    run_json(&mut s, "mesh.agglomerate", json!({"mesh": "Mesh#1", "mode": {"per_pin": {"group": "lattice"}}})).expect("per pin");
    let coarse = mesh_of(&s, m2());
    let c = coarse.as_read();
    assert_eq!(c.cell_count(), PINS * PINS, "exactly one coarse cell per pin");
    assert!(rel(vol(c), 288.0) < 1e-12);
    for i in 0..c.cell_count() {
        assert!(rel(cell_volume(c, CellIx::new(i)), 32.0) < 1e-12, "a pin is 4 x 4 x 2");
    }
    check_link(&s, m1(), m2());
    let layer = s.state().layer::<MeshLayer>().unwrap();
    let map = layer.maps_from(m1())[0].1;
    let f = fine.as_read();
    for fc in 0..f.cell_count() {
        let (x, y, _) = {
            let v = f.cell_corners(CellIx::new(fc)).unwrap();
            let p = f.point(v[0].get());
            (p.x as usize, p.y as usize, p.z as usize)
        };
        assert_eq!(map.coarse_of(CellIx::new(fc)).get() as u64, pin_of(x, y), "fine cell {fc} maps to the coarse cell of its pin");
    }
    for p in 0..PINS * PINS {
        assert_eq!(map.fines_of(CellIx::new(p)).len(), 32);
    }
    // a coarse cell spans both volumes of its pin and is classified on the larger (the moderator)
    let cls = c.cell_class(CellIx::new(0)).entity;
    assert_eq!(cls, id(EntityKind::TopoVolume, 2));
    let (sj, er) = quality(c);
    assert!(sj > 1.0 - 1e-12 && rel(er, 2.0) < 1e-12);
    // pins touch: 12 interior faces between pins in each layer direction... 2 * 3 * 2 = 12 pin interfaces
    assert_eq!(c.count(MeshEntityKind::Face) - (0..c.face_count()).filter(|f| c.is_boundary_face(FaceIx::new(*f))).count(), 12);
}

#[test]
fn per_pin_by_volume_refuses_the_nonconvex_moderator_ring() {
    let mut s = session_with("pins");
    let h = s.state().hash();
    let e = run_json(&mut s, "mesh.agglomerate", json!({"mesh": "Mesh#1", "mode": {"per_pin": {"group": "volume"}}})).expect_err("a ring is not a convex cell");
    assert!(e.to_string().contains("coarse"), "{e}");
    assert_eq!(s.state().hash(), h, "a refusal leaves the state untouched");
}

#[test]
fn per_pin_with_several_cells_per_pin_splits_within_the_pin() {
    let mut s = session_with("pins");
    run_json(&mut s, "mesh.agglomerate", json!({"mesh": "Mesh#1", "mode": {"per_pin": {"group": "lattice", "cells_per_group": 4}}})).expect("per pin x4");
    let coarse = mesh_of(&s, m2());
    let c = coarse.as_read();
    assert!(c.cell_count() >= 4 * PINS * PINS && c.cell_count() < 288);
    assert!(rel(vol(c), 288.0) < 1e-12);
    check_link(&s, m1(), m2());
    // every coarse cell stays inside one pin
    let layer = s.state().layer::<MeshLayer>().unwrap();
    let map = layer.maps_from(m1())[0].1;
    let f = mesh_of(&s, m1());
    for cc in 0..c.cell_count() {
        let pins: std::collections::BTreeSet<u64> = map
            .fines_of(CellIx::new(cc))
            .iter()
            .map(|fc| {
                let p = f.as_read().point(f.as_read().cell_corners(*fc).unwrap()[0].get());
                pin_of(p.x as usize, p.y as usize)
            })
            .collect();
        assert_eq!(pins.len(), 1);
    }
    quality(c);
}

#[test]
fn target_mode_never_merges_across_volumes_and_keeps_the_interface() {
    let mut s = session_with("two");
    run_json(&mut s, "mesh.agglomerate", json!({"mesh": "Mesh#1", "mode": {"target": {"cells": 2}}})).expect("target");
    let coarse = mesh_of(&s, m2());
    let c = coarse.as_read();
    assert_eq!(c.cell_count(), 2, "each 2 x 2 x 2 volume is one box");
    assert_eq!(c.cell_class(CellIx::new(0)).entity, id(EntityKind::TopoVolume, 1));
    assert_eq!(c.cell_class(CellIx::new(1)).entity, id(EntityKind::TopoVolume, 2));
    for i in 0..2 {
        assert!(rel(cell_volume(c, CellIx::new(i)), 8.0) < 1e-12);
    }
    // 2 cells: 11 faces, the single interface face has area 4
    assert_eq!(c.count(MeshEntityKind::Face), 11);
    let iface: Vec<FaceIx> = (0..c.face_count()).map(FaceIx::new).filter(|f| !c.is_boundary_face(*f)).collect();
    assert_eq!(iface.len(), 1);
    assert!(rel(face_area_vector(c, iface[0]).norm(), 4.0) < 1e-12);
    check_link(&s, m1(), m2());
    // a target in between: 3 cells cannot be reached without crossing, so the count is 3..=16
    let mut s = session_with("two");
    run_json(&mut s, "mesh.agglomerate", json!({"mesh": "Mesh#1", "mode": {"target": {"cells": 6}}})).expect("target 6");
    let c6 = mesh_of(&s, m2());
    assert_eq!(c6.as_read().cell_count(), 6);
    assert!(rel(vol(c6.as_read()), 16.0) < 1e-12);
    check_link(&s, m1(), m2());
}

#[test]
fn keep_source_false_retires_the_fine_mesh_and_drops_links() {
    let mut s = session_with("two");
    run_json(&mut s, "mesh.agglomerate", json!({"mesh": "Mesh#1", "mode": {"target": {"cells": 4}}})).expect("first");
    assert_eq!(s.state().layer::<MeshLayer>().unwrap().maps_from(m1()).len(), 1);
    // coarsen the coarse mesh and retire it: the (Mesh#1, Mesh#2) link goes with Mesh#2, and no
    // link is recorded for a retired fine mesh
    let (_, d) = run_json(&mut s, "mesh.agglomerate", json!({"mesh": "Mesh#2", "mode": {"target": {"cells": 2}}, "keep_source": false})).expect("second");
    let layer = s.state().layer::<MeshLayer>().unwrap();
    assert!(layer.get(m2()).is_none() && layer.get(m1()).is_some());
    assert!(layer.maps_from(m1()).is_empty(), "the link of a retired mesh is dropped");
    assert!(layer.maps_to(m2()).is_empty() && layer.maps_from(m2()).is_empty());
    let m3 = d.outputs["mesh"][0];
    assert_eq!(d.successors[&m2()], vec![m3]);
    assert_eq!(layer.get(m3).unwrap().mesh().cell_count(), 2);
}

#[test]
fn refusals_are_atomic() {
    for (kind, params) in [
        ("two", json!({"mesh": "Mesh#1", "mode": {"target": {"cells": 0}}})),
        ("two", json!({"mesh": "Mesh#1", "mode": {"target": {"cells": 16}}})),
        ("two", json!({"mesh": "Mesh#1", "mode": {"blocks": {"size": [0.0, 1.0, 1.0]}}})),
        ("two", json!({"mesh": "Mesh#1", "mode": {"blocks": {"size": [0.5, 0.5, 0.5]}}})),
        ("two", json!({"mesh": "Mesh#1", "mode": {"per_pin": {"group": "volume", "cells_per_group": 0}}})),
        ("two", json!({"mesh": "Mesh#9", "mode": {"target": {"cells": 2}}})),
    ] {
        let mut s = session_with(kind);
        let h = s.state().hash();
        run_json(&mut s, "mesh.agglomerate", params.clone()).expect_err(&params.to_string());
        assert_eq!(s.state().hash(), h, "{params}");
    }
}

#[test]
fn replay_and_fork_give_equal_hashes() {
    let op = json!({"mesh": "Mesh#1", "mode": {"per_pin": {"group": "lattice"}}});
    let mut s = session_with("pins");
    let h_insert = s.position().expect("head");
    run_json(&mut s, "mesh.agglomerate", op.clone()).expect("agglomerate");
    let (h_head, state_hash) = (s.position().expect("head"), s.state().hash());
    // cold replay from the root: no divergence, same state
    let q = QuietControl::new();
    let cold = s.state_at_from_root(h_head, &q.control()).expect("cold replay");
    assert_eq!(cold.hash(), state_hash);
    // a fresh engine replays the timeline to the same state
    let s2 = Session::open(engine(), s.timeline().clone(), s.head().clone(), &q.control()).expect("reopen");
    assert_eq!(s2.state().hash(), state_hash);
    // a second session built by applying the same records
    let mut s3 = session_with("pins");
    run_json(&mut s3, "mesh.agglomerate", op.clone()).expect("agglomerate");
    assert_eq!(s3.state().hash(), state_hash);
    // fork at the insert, apply the same op on the branch: the same hash again
    s.checkout(Rev::Node(h_insert), &q.control()).expect("checkout");
    s.branch("fork").expect("branch");
    run_json(&mut s, "mesh.agglomerate", op).expect("agglomerate on the fork");
    assert_eq!(s.state().hash(), state_hash);
    let layer = s.state().layer::<MeshLayer>().unwrap();
    assert_eq!(layer.maps_from(m1()).len(), 1, "the link survives replay");
}

#[test]
fn registration_shape() {
    let e = engine();
    let info = e.registry().catalog().into_iter().filter_map(Result::ok).find(|i| i.name == "mesh.agglomerate").expect("registered");
    let schema = serde_json::to_value(&info.schema).unwrap();
    assert_eq!(schema["additionalProperties"], false);
    assert_eq!(schema["required"], json!(["mesh", "mode"]));
    assert_eq!(schema["properties"]["mesh"]["x-entity-kind"], json!(["Mesh"]));
    assert_eq!((info.outputs[0].name, info.outputs[0].kinds), ("mesh", &[EntityKind::Mesh][..]));
}

#[test]
fn determinism_one_thread_vs_default() {
    let run_once = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| {
            let mut s = session_with("dual6");
            run_json(&mut s, "mesh.agglomerate", json!({"mesh": "Mesh#1", "mode": {"target": {"cells": 100}}})).expect("agglomerate");
            let c = mesh_of(&s, m2());
            (s.state().hash(), sm_mesh_api::topology_hash(c.as_read()), sm_mesh_api::geometry_hash(c.as_read()), c.as_read().cell_count())
        })
    };
    let a = run_once(1);
    assert_eq!(a, run_once(1));
    assert_eq!(a, run_once(4));
    assert!(a.3 >= 100);
}

proptest::proptest! {
    #![proptest_config(sm_testkit::strategies::config(24))]

    /// Random hex lattices and block sizes: the coarse mesh is valid, convex, conserves volume
    /// and carries a map that restricts volumes exactly.
    #[test]
    fn prop_blocks_conserve_volume_and_link_exactly(
        n in proptest::array::uniform3(2usize..=5),
        size in proptest::array::uniform3(2usize..=4),
        shift in proptest::array::uniform3(0usize..2),
    ) {
        let mut s = session_with(&format!("hex:{}:{}:{}", n[0], n[1], n[2]));
        let sz = size.map(|x| x as f64);
        let origin = shift.map(|x| x as f64);
        run_json(&mut s, "mesh.agglomerate", json!({"mesh": "Mesh#1", "mode": {"blocks": {"size": sz, "origin": origin}}})).expect("agglomerate");
        let coarse = mesh_of(&s, m2());
        let c = coarse.as_read();
        proptest::prop_assert!(c.cell_count() < n[0] * n[1] * n[2]);
        proptest::prop_assert!(rel(vol(c), (n[0] * n[1] * n[2]) as f64) < 1e-12);
        proptest::prop_assert!(validate(c).is_empty() && validate_geometry(c, &GeomTol::default()).is_empty());
        for i in 0..c.cell_count() {
            proptest::prop_assert!(sm_quality::cell_convexity(c, CellIx::new(i), sm_quality::RT_PLANARITY_TOL).convex);
        }
        check_link(&s, m1(), m2());
    }
}
