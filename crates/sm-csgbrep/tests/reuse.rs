//! Lattice reuse (draft D4): one prototype per distinct universe, copies by `model.copy`, the
//! result conformal and equal to the flat partition in volumes, blocks and tags.
mod common;
use core::f64::consts::PI;
use common::decks::*;
use common::*;
use sm_csgbrep::OpenmcOptions;
use sm_model_api::Domain;
use sm_topo_api::Dim;
use std::collections::BTreeMap;

fn flat() -> Option<OpenmcOptions> {
    Some(OpenmcOptions { reuse_lattices: Some(false), ..Default::default() })
}

/// A counter of the `import.partition` message: the number before `what`.
fn stat(d: &sm_op_api::Delta, what: &str) -> usize {
    let msg = diag(d, "import.partition")[0];
    let head = msg.split(what).next().unwrap_or_else(|| panic!("{msg}"));
    assert!(msg.contains(what), "{what} in {msg}");
    head.rsplit([' ', ',', ';', ':']).find(|w| !w.is_empty()).and_then(|w| w.parse().ok()).unwrap_or_else(|| panic!("{what}: {msg}"))
}

/// Every face of the deck volumes is used by two volumes, or by one and lies on the vacuum
/// sideset (all the test decks flag their container box): no face is left unmerged.
fn assert_conformal(r: &Rig, d: &sm_op_api::Delta) {
    let topo = r.topo();
    let m = r.model();
    let vac: std::collections::BTreeSet<_> = m.color_by_name("vacuum").map(|c| m.faces_of_color(c)).unwrap_or_default().into_iter().collect();
    let mut faces = std::collections::BTreeSet::new();
    for &v in &d.outputs["deck_volumes"] {
        faces.extend(topo.volume_faces(v).iter().copied());
    }
    let mut open = Vec::new();
    let mut shared = 0;
    for &f in &faces {
        let n = topo.coboundary(f).iter().filter(|i| topo.dim_of(i.cell) == Some(Dim::Volume)).count();
        match n {
            2 => shared += 1,
            1 if vac.contains(&f) => {}
            _ => open.push((f, n)),
        }
    }
    assert!(shared > 0, "no face is shared by two volumes");
    assert!(open.is_empty(), "{} non-conformal faces of {}: {:?}", open.len(), faces.len(), &open[..open.len().min(8)]);
}

type Key = (String, Option<(i32, i32, i32)>);

/// `(block name, tag slot or none) -> sorted volumes`.
fn signature(r: &Rig, d: &sm_op_api::Delta) -> BTreeMap<Key, Vec<f64>> {
    let m = r.model();
    let mut out: BTreeMap<_, Vec<f64>> = BTreeMap::new();
    for c in m.colors().filter(|c| c.domain == Domain::Volume) {
        for v in m.volumes_of_color(c.id) {
            assert!(d.outputs["deck_volumes"].contains(&v));
            let slot = r.tag(v).map(|t| {
                let s = t.slots["lat20"];
                (s.i, s.j, s.k)
            });
            out.entry((c.name.clone(), slot)).or_default().push(r.volume(v));
        }
    }
    for v in out.values_mut() {
        v.sort_by(f64::total_cmp);
    }
    out
}

fn assert_same_as_flat(deck: &str, mats: &str) -> (sm_op_api::Delta, sm_op_api::Delta) {
    let mut a = Rig::new();
    let da = a.import(deck, Some(mats)).unwrap();
    let mut b = Rig::new();
    let db = b.import_with(deck, Some(mats), flat()).unwrap();
    assert_conformal(&a, &da);
    assert_conformal(&b, &db);
    let (sa, sb) = (signature(&a, &da), signature(&b, &db));
    assert_eq!(sa.keys().collect::<Vec<_>>(), sb.keys().collect::<Vec<_>>());
    for (k, va) in &sa {
        let vb = &sb[k];
        assert_eq!(va.len(), vb.len(), "{k:?}");
        for (x, y) in va.iter().zip(vb) {
            assert!((x - y).abs() <= 1e-9 * y.abs().max(1.0), "{k:?}: {x} vs {y}");
        }
    }
    assert_eq!(da.outputs["deck_volumes"].len(), db.outputs["deck_volumes"].len());
    (da, db)
}

#[test]
fn a_3x3_lattice_cuts_one_element_and_copies_it() {
    let (da, db) = assert_same_as_flat(&pin_lattice(3, 1.26, (0.35, 0.37, 0.42), 2.0, false), &materials(4));
    // One prototype: its three circles are the only kernel cuts; eight copies; nothing to imprint.
    assert_eq!(stat(&da, "kernel cuts"), 3);
    assert_eq!(stat(&da, "kernel cuts in prototypes"), 3);
    assert_eq!(stat(&da, "copies"), 8);
    assert_eq!(stat(&da, "imprints"), 0);
    // The flat partition cuts every element: 3 n^2 circles and the interior planes.
    assert_eq!(stat(&db, "kernel cuts"), 3 * 9 + 2 + 6);
}

#[test]
fn a_3x3_lattice_with_outer_imprints_the_ring() {
    let (da, _) = assert_same_as_flat(&pin_lattice(3, 1.0, (0.2, 0.25, 0.3), 1.0, true), &materials(4));
    assert_eq!(stat(&da, "kernel cuts in prototypes"), 3);
    // The four array-box planes cut the container into the ring and the array piece.
    assert_eq!(stat(&da, "kernel cuts"), 3 + 4);
    assert!(stat(&da, "imprints") > 0);
}

#[test]
fn a_mixed_lattice_cuts_one_element_per_distinct_universe() {
    let (ra, rb) = ((0.2, 0.25, 0.3), (0.15, 0.3, 0.4));
    for outer in [false, true] {
        let (da, _) = assert_same_as_flat(&mixed_lattice(3, 1.0, ra, rb, 1.0, outer), &materials(4));
        assert_eq!(stat(&da, "kernel cuts in prototypes"), 6, "outer {outer}");
        assert_eq!(stat(&da, "kernel cuts"), 6 + if outer { 4 } else { 0 });
        // Universe 10 fills 5 slots, 11 fills 4: 4 + 3 copies.
        assert_eq!(stat(&da, "copies"), 7);
    }
}

#[test]
fn a_17x17_lattice_costs_the_cuts_of_one_element() {
    let deck = pin_lattice(17, 1.26, (0.4096, 0.418, 0.475), 2.0, false);
    let mut r = Rig::new();
    let d = r.import(&deck, Some(&materials(4))).unwrap();
    assert_eq!(d.outputs["deck_volumes"].len(), 4 * 289);
    assert_eq!(stat(&d, "kernel cuts"), 3);
    assert_eq!(stat(&d, "copies"), 288);
    assert_conformal(&r, &d);
    // Blocks: 289 pins of each material, analytic volumes.
    let mut by_name: BTreeMap<String, f64> = BTreeMap::new();
    let m = r.model();
    for c in m.colors().filter(|c| c.domain == Domain::Volume) {
        by_name.insert(c.name.clone(), m.volumes_of_color(c.id).iter().map(|&v| r.volume(v)).sum());
    }
    let (r1, r3, p, h) = (0.4096_f64, 0.475_f64, 1.26_f64, 2.0_f64);
    assert!((by_name["m1"] - 289.0 * core::f64::consts::PI * r1 * r1 * h).abs() < 1e-9 * by_name["m1"]);
    assert!((by_name["m3"] - 289.0 * (p * p - core::f64::consts::PI * r3 * r3) * h).abs() < 1e-9 * by_name["m3"]);
}

#[test]
fn a_5x5_lattice_with_outer_stays_conformal() {
    let (da, _) = assert_same_as_flat(&pin_lattice(5, 1.0, (0.2, 0.25, 0.3), 1.0, true), &materials(4));
    assert_eq!(da.outputs["deck_volumes"].len(), 4 * 25 + 1);
    assert_eq!(stat(&da, "kernel cuts"), 3 + 4);
    // The 16 perimeter elements (a corner meets the ring on two sides, one imprint) each have
    // one moderator body that meets the ring.
    assert_eq!(stat(&da, "imprints"), 16);
}

/// The 9 x 9 array with a `p / 2` ring (the lattice `outer`) used to be refused by the kernel
/// (`cannot find a point inside a fragment`); it imports on both paths with analytic volumes.
#[test]
fn a_ring_around_a_large_array_imports_with_analytic_volumes() {
    let deck = pin_lattice(9, 1.0, (0.2, 0.25, 0.3), 1.0, true);
    let (r1, r2, r3, p, h, n2) = (0.2_f64, 0.25_f64, 0.3_f64, 1.0_f64, 1.0_f64, 81.0_f64);
    for opts in [None, flat()] {
        let mut r = Rig::new();
        let d = r.import_with(&deck, Some(&materials(4)), opts).unwrap();
        assert_eq!(d.outputs["deck_volumes"].len(), 4 * 81 + 1);
        let mut by_name: BTreeMap<String, f64> = BTreeMap::new();
        let m = r.model();
        for c in m.colors().filter(|c| c.domain == Domain::Volume) {
            by_name.insert(c.name.clone(), m.volumes_of_color(c.id).iter().map(|&v| r.volume(v)).sum());
        }
        let want = [
            ("m1", n2 * PI * r1 * r1 * h),
            ("m2", n2 * PI * (r3 * r3 - r2 * r2) * h),
            ("m3", n2 * (p * p - PI * r3 * r3) * h),
            // The ring: the 10 x 10 container minus the 9 x 9 array.
            ("m4", (100.0 - 81.0) * h),
        ];
        for (k, w) in want {
            assert!((by_name[k] - w).abs() < 1e-9 * w, "{k}: {} vs {w}", by_name[k]);
        }
        let total: f64 = d.outputs["deck_volumes"].iter().map(|&v| r.volume(v)).sum();
        assert!((total - 100.0 * h).abs() < 1e-9 * 100.0, "total {total}");
    }
}

#[test]
fn a_cell_reaching_into_the_array_is_an_overlap() {
    let deck = pin_lattice(3, 1.26, (0.35, 0.37, 0.42), 2.0, false).replace("<geometry>\n", "<geometry>\n  <cell id=\"800\" material=\"4\" region=\"11 -12 13 -14 15 -16\" universe=\"0\"/>\n");
    for opts in [None, flat()] {
        let e = Rig::new().import_with(&deck, Some(&materials(4)), opts).unwrap_err().to_string();
        assert!(e.contains("csg.overlap"), "{e}");
    }
    // A cell that straddles the array box with its centroid outside: x in [-5, 0], the array
    // starts at x = -1.89. The overlap is found on the part inside the box.
    let straddle = pin_lattice(3, 1.26, (0.35, 0.37, 0.42), 2.0, false).replace("<geometry>\n", "<geometry>\n  <cell id=\"800\" material=\"4\" region=\"501 -502 13 -14 15 -16\" universe=\"0\"/>\n  <surface id=\"501\" type=\"x-plane\" coeffs=\"-5\"/>\n  <surface id=\"502\" type=\"x-plane\" coeffs=\"0\"/>\n");
    // (Not run flat: the plane x = 0 through the pin axes hits a kernel limit, a half cylinder patch.)
    let e = Rig::new().import_with(&straddle, Some(&materials(4)), None).unwrap_err().to_string();
    assert!(e.contains("csg.overlap"), "{e}");
    let mut r = Rig::new();
    let o = OpenmcOptions { overlap: Some(sm_csgbrep::OverlapMode::Report), ..Default::default() };
    let d = r.import_with(&straddle, Some(&materials(4)), Some(o)).unwrap();
    assert!(!diag(&d, "import.overlap").is_empty());
    let total: f64 = d.outputs["deck_volumes"].iter().map(|&v| r.volume(v)).sum();
    // The array once, plus cell 800 outside the array box: x in [-5, -1.89], y in +-1.89, z in +-1.
    let want = 9.0 * 1.26 * 1.26 * 2.0 + (5.0 - 1.89) * 3.78 * 2.0;
    assert!((total - want).abs() < 1e-9 * want, "{total} vs {want}");
    // `report` keeps one owner per piece: the volume of the array, once.
    let mut r = Rig::new();
    let o = OpenmcOptions { overlap: Some(sm_csgbrep::OverlapMode::Report), ..Default::default() };
    let d = r.import_with(&deck, Some(&materials(4)), Some(o)).unwrap();
    assert!(!diag(&d, "import.overlap").is_empty());
    let total: f64 = d.outputs["deck_volumes"].iter().map(|&v| r.volume(v)).sum();
    assert!((total - 9.0 * 1.26 * 1.26 * 2.0).abs() < 1e-9 * total, "{total}");
}

#[test]
fn lattices_that_cannot_reuse_stay_flat() {
    // Every slot its own universe (nothing to reuse), and a rotated lattice.
    for (name, flat_cuts_min) in [("lattice2d", 1), ("lattice_xf", 1)] {
        let mut r = Rig::new();
        let d = r.import(&fixture(name, "geometry.xml"), Some(&fixture(name, "materials.xml"))).unwrap();
        assert_eq!(stat(&d, "copies"), 0, "{name}");
        assert!(stat(&d, "kernel cuts") >= flat_cuts_min, "{name}");
        assert_conformal_or_boundary(&r, &d);
    }
}

/// Every face of the deck volumes has two volumes or only one (a boundary of the model).
fn assert_conformal_or_boundary(r: &Rig, d: &sm_op_api::Delta) {
    let topo = r.topo();
    for &v in &d.outputs["deck_volumes"] {
        for &f in topo.volume_faces(v) {
            let n = topo.coboundary(f).iter().filter(|i| topo.dim_of(i.cell) == Some(Dim::Volume)).count();
            assert!(n == 1 || n == 2, "face {f} has {n} volumes");
        }
    }
}
