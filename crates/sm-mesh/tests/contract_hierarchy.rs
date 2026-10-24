//! Coarse CMFD meshes, level maps and the catalog.

mod common;
use common::*;
use sm_core::EntityKind;
use common::shim::{ReferenceCatalog, ReferenceMesh, hex_block};
use sm_mesh_api::*;
use std::collections::BTreeMap;

fn fine_coarse(alloc: &mut SerialAlloc) -> (ReferenceMesh, ReferenceMesh, Vec<CellIx>, Vec<u32>) {
    let fine = hex_block([4, 4, 2], [1.0; 3], alloc).unwrap();
    let coarse = hex_block([2, 2, 1], [2.0; 3], alloc).unwrap();
    let mut coarse_of = Vec::new();
    for k in 0..2 {
        for j in 0..4 {
            for i in 0..4 {
                coarse_of.push(CellIx::new((k / 2 * 2 + j / 2) * 2 + i / 2));
            }
        }
    }
    // Face map by topology: (left coarse, right coarse) for interior faces, (coarse cell, class) for boundary faces.
    let mut interior: BTreeMap<(CellIx, CellIx), u32> = BTreeMap::new();
    let mut bnd: BTreeMap<(CellIx, sm_core::EntityId), u32> = BTreeMap::new();
    for f in 0..coarse.face_count() {
        let f = FaceIx::new(f);
        match coarse.face_right(f) {
            Some(r) => interior.insert((coarse.face_left(f), r), f.0),
            None => bnd.insert((coarse.face_left(f), coarse.face_class(f).entity), f.0),
        };
    }
    let fm: Vec<u32> = (0..fine.face_count())
        .map(|f| {
            let f = FaceIx::new(f);
            let cl = coarse_of[fine.face_left(f).get()];
            match fine.face_right(f) {
                Some(r) => {
                    let cr = coarse_of[r.get()];
                    if cl == cr {
                        NO_CELL
                    } else {
                        interior[&(cl.min(cr), cl.max(cr))]
                    }
                }
                None => bnd[&(cl, fine.face_class(f).entity)],
            }
        })
        .collect();
    (fine, coarse, coarse_of, fm)
}

#[test]
fn coarse_cmfd_mesh_with_fine_to_coarse_map() {
    let mut alloc = SerialAlloc::new();
    let (fine, coarse, coarse_of, fm) = fine_coarse(&mut alloc);
    let map = LevelMap::new(&fine, &coarse, coarse_of, Some(fm.clone())).unwrap();
    assert_eq!((map.fine_len(), map.coarse_len()), (32, 4));
    assert_eq!(map.fine_id(), fine.mesh_id());
    assert_eq!(map.coarse_id(), coarse.mesh_id());
    for c in 0..4 {
        assert_eq!(map.fines_of(CellIx::new(c)).len(), 8);
        assert!(map.fines_of(CellIx::new(c)).windows(2).all(|w| w[0] < w[1]));
        for &f in map.fines_of(CellIx::new(c)) {
            assert_eq!(map.coarse_of(f), CellIx::new(c));
        }
    }
    // restrict_sum conserves extensive quantities; prolong_copy broadcasts intensive ones.
    let fv: Vec<f64> = (0..32)
        .map(|c| cell_volume(&fine, CellIx::new(c)))
        .collect();
    let cv = map.restrict_sum(&fv).unwrap();
    for (c, v) in cv.iter().enumerate() {
        assert!((v - cell_volume(&coarse, CellIx::new(c))).abs() < 1e-12);
    }
    let flux = [1.0, 2.0, 3.0, 4.0];
    let pf = map.prolong_copy(&flux).unwrap();
    assert_eq!(pf.len(), 32);
    assert!(
        pf.iter()
            .enumerate()
            .all(|(f, v)| *v == flux[map.coarse_of(CellIx::new(f)).get()])
    );
    assert!((map.restrict_sum(&pf).unwrap().iter().sum::<f64>() - 8.0 * 10.0).abs() < 1e-12);
    assert!(map.restrict_sum(&[1.0]).is_err() && map.prolong_copy(&[1.0]).is_err());
    // Face map: CMFD surface currents sum fine faces onto coarse faces.
    assert_eq!(map.face_map().unwrap(), fm.as_slice());
    let mut area = vec![0.0; coarse.face_count()];
    for (f, &g) in fm.iter().enumerate() {
        if g != NO_CELL {
            area[g as usize] += face_area_vector(&fine, FaceIx::new(f)).norm();
        }
    }
    for (g, a) in area.iter().enumerate() {
        assert!(
            (a - face_area_vector(&coarse, FaceIx::new(g)).norm()).abs() < 1e-12,
            "coarse face {g}"
        );
    }
    // The map is bound to its meshes.
    map.check_meshes(&fine, &coarse).unwrap();
    assert!(map.check_meshes(&coarse, &fine).is_err());
}

#[test]
fn level_map_rejects_bad_input() {
    let mut alloc = SerialAlloc::new();
    let (fine, coarse, coarse_of, fm) = fine_coarse(&mut alloc);
    assert!(
        LevelMap::new(&fine, &coarse, coarse_of[..31].to_vec(), None).is_err(),
        "wrong length"
    );
    let mut bad = coarse_of.clone();
    bad[0] = CellIx(99);
    assert!(
        LevelMap::new(&fine, &coarse, bad, None).is_err(),
        "out of range"
    );
    let mut orphan = coarse_of.clone();
    for c in &mut orphan {
        if c.get() == 3 {
            *c = CellIx(2);
        }
    }
    assert!(
        LevelMap::new(&fine, &coarse, orphan, None).is_err(),
        "coarse cell without children"
    );
    // Face map checked against both meshes.
    let mut wrong = fm.clone();
    let i = fm.iter().position(|&g| g == NO_CELL).unwrap();
    wrong[i] = 0;
    assert!(
        LevelMap::new(&fine, &coarse, coarse_of.clone(), Some(wrong)).is_err(),
        "face inside a coarse cell must not map"
    );
    let mut wrong = fm.clone();
    let j = fm.iter().position(|&g| g != NO_CELL).unwrap();
    wrong[j] = NO_CELL;
    assert!(
        LevelMap::new(&fine, &coarse, coarse_of.clone(), Some(wrong)).is_err(),
        "coarse face missing"
    );
    assert!(
        LevelMap::new(&fine, &coarse, coarse_of, Some(fm[..5].to_vec())).is_err(),
        "wrong length"
    );
}

#[test]
fn catalog_links_many_levels_and_composes() {
    let mut alloc = SerialAlloc::new();
    let (fine, mut coarse, coarse_of, fm) = fine_coarse(&mut alloc);
    // A CMFD node may sit on a partition cell: the coarse classification need not equal the fine one.
    for c in 0..coarse.cell_count() {
        MeshEdit::set_class(
            &mut coarse,
            MeshEntity::Cell(CellIx::new(c)),
            on(EntityKind::PartitionCell, 1 + c as u64),
        )
        .unwrap();
    }
    let top = hex_block([1, 1, 1], [4.0, 4.0, 2.0], &mut alloc).unwrap();
    let m1 = LevelMap::new(&fine, &coarse, coarse_of.clone(), Some(fm)).unwrap();
    let m2 = LevelMap::new(&coarse, &top, vec![CellIx(0); 4], None).unwrap();
    let m3 = LevelMap::new(&fine, &top, vec![CellIx(0); 32], None).unwrap();
    let composed = m1.compose(&m2).unwrap();
    assert_eq!(
        (composed.fine_id(), composed.coarse_id()),
        (fine.mesh_id(), top.mesh_id())
    );
    assert_eq!(composed.fines_of(CellIx(0)).len(), 32);
    assert!(
        composed.face_map().is_none(),
        "face map survives only if both maps have one"
    );
    assert!(m2.compose(&m1).is_err(), "maps must chain");

    let (fid, cid, tid) = (fine.mesh_id(), coarse.mesh_id(), top.mesh_id());
    let mut cat = ReferenceCatalog::new();
    // Insert out of id order; mesh_ids stays ascending.
    cat.insert_mesh(Box::new(top)).unwrap();
    cat.insert_mesh(Box::new(fine)).unwrap();
    assert!(cat.link(m1.clone()).is_err(), "unknown coarse mesh");
    cat.insert_mesh(Box::new(coarse)).unwrap();
    assert!(cat.mesh_ids().windows(2).all(|w| w[0] < w[1]));
    assert_eq!(cat.mesh_ids().len(), 3);
    cat.link(m1).unwrap();
    cat.link(m2).unwrap();
    cat.link(m3).unwrap();
    // One fine mesh can have several coarse meshes (a CMFD mesh and a multigrid level).
    let from: Vec<EntityId> = cat.maps_from(fid).into_iter().map(|(c, _)| c).collect();
    assert_eq!(from.len(), 2);
    assert!(from.windows(2).all(|w| w[0] < w[1]) && from.contains(&cid) && from.contains(&tid));
    let to: Vec<EntityId> = cat.maps_to(tid).into_iter().map(|(f, _)| f).collect();
    assert_eq!(to.len(), 2);
    assert!(cat.maps_to(fid).is_empty() && cat.maps_from(tid).is_empty());
    assert!(cat.link(composed).is_err(), "the pair is already linked");
    assert!(cat.mesh(cid).is_some());
    // Stale use: a map built for other meshes is refused.
    let (f2, c2, co2, _) = fine_coarse(&mut alloc);
    let other = LevelMap::new(&f2, &c2, co2, None).unwrap();
    assert!(cat.link(other).is_err());
}

use sm_core::EntityId;
