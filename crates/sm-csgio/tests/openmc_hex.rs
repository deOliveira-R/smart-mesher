//! Hexagonal lattices (ACR-CSG-HEXIR, ruling D8): the reader's `import_hex` against OpenMC 0.16.0.
//!
//! `tests/fixtures/openmc_hex/{hex_x,hex_y,hex_x3d,hex_y3d}` hold the committed `geometry.xml` and an
//! `oracle.json` that `gen_hex.py` made (it also checked, against OpenMC itself, the ring/position to
//! axial index map and the element centres before writing). Here, without OpenMC:
//! * the `universes` text (OpenMC's row picture) is read into OpenMC's ring order;
//! * the documented `(ring, position) <-> (q, r)` map equals the one OpenMC recorded for every element;
//! * the element centre formulas equal the recorded centres;
//! * an independent brute-force locator over the IR (hexagon test against every element centre) finds
//!   the same universe as OpenMC's `find_cell` at 600 points per model.
use serde_json::Value;
use sm_csgio::ir::{Fill, HexLattice, HexOrientation};
use sm_csgio::{CsgIr, ImportError, ImportOptions, ImportedCsg, openmc};
use std::path::PathBuf;

fn dir(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/openmc_hex").join(name)
}

fn load(name: &str) -> (String, String, Value) {
    let d = dir(name);
    let read = |f: &str| std::fs::read_to_string(d.join(f)).unwrap_or_else(|e| panic!("{name}/{f}: {e}"));
    (read("geometry.xml"), read("materials.xml"), serde_json::from_str(&read("oracle.json")).unwrap())
}

fn f(v: &Value) -> f64 {
    v.as_f64().unwrap()
}

const MODELS: [&str; 4] = ["hex_x", "hex_y", "hex_x3d", "hex_y3d"];

fn ir_of(name: &str) -> (CsgIr, Value) {
    let (g, m, o) = load(name);
    (openmc::import_hex(&g, Some(&m)).unwrap_or_else(|e| panic!("{name}: {e}")), o)
}

#[test]
fn the_reader_fills_the_hex_ir_in_openmc_ring_order() {
    for name in MODELS {
        let (ir, o) = ir_of(name);
        assert!(ir.lattices.is_empty());
        let l = &ir.hex_lattices[&50];
        let three_d = o["n_axial"].as_u64().unwrap() > 1;
        assert_eq!(l.orientation, if o["orientation"] == "x" { HexOrientation::X } else { HexOrientation::Y }, "{name}");
        assert_eq!((l.n_rings, u64::from(l.n_axial)), (3, o["n_axial"].as_u64().unwrap()));
        assert_eq!(l.pitch, f(&o["pitch"]));
        let c = o["center"].as_array().unwrap();
        assert_eq!(&l.center[..2], &[f(&c[0]), f(&c[1])]);
        assert_eq!(l.z_pitch, if three_d { f(&o["z_pitch"]) } else { f64::INFINITY });
        assert_eq!(l.center[2], if three_d { f(&c[2]) } else { 0.0 });
        assert_eq!(l.outer, Some(900));
        // The generator numbers the universes 10, 11, ... in OpenMC's own order (layer, ring,
        // position), so the stored order is exactly that sequence.
        let want: Vec<u32> = (0..19 * l.n_axial).map(|i| 10 + i).collect();
        assert_eq!(l.universes, want, "{name}: the row picture is not read into ring order");
        l.check().unwrap();
    }
}

#[test]
fn the_index_map_equals_the_one_openmc_recorded() {
    for name in MODELS {
        let (ir, o) = ir_of(name);
        let l = &ir.hex_lattices[&50];
        let elements = o["elements"].as_array().unwrap();
        assert_eq!(elements.len(), l.universes.len());
        for e in elements {
            let (ring, pos, q, r, k) = (e["ring"].as_u64().unwrap() as u32, e["position"].as_u64().unwrap() as u32, e["q"].as_i64().unwrap() as i32, e["r"].as_i64().unwrap() as i32, e["layer"].as_u64().unwrap() as u32);
            assert_eq!(HexLattice::ring_axial(l.orientation, 3, ring, pos), Some((q, r)), "{name} ring {ring} pos {pos}");
            assert_eq!(HexLattice::ring_position(l.orientation, 3, q, r), Some((ring, pos)), "{name} ({q},{r})");
            assert_eq!(l.universe_at(q, r, k), Some(e["universe"].as_u64().unwrap() as u32), "{name} layer {k} ({q},{r})");
            let c = l.element_center(q, r, k);
            let oc = e["center"].as_array().unwrap();
            for a in 0..3 {
                assert!((c[a] - f(&oc[a])).abs() < 1e-12, "{name} element ({q},{r},{k}) centre {c:?} vs {oc:?}");
            }
        }
    }
}

/// The element of the lattice that holds `p`, by testing every element's hexagon. `None` outside
/// the array (and `Err` when `p` is within `1e-9` of a face, where the answer is a tie).
fn locate(l: &HexLattice, p: [f64; 3]) -> Result<Option<u32>, ()> {
    let layer = if l.z_pitch.is_finite() {
        let z0 = l.center[2] - 0.5 * l.z_pitch * f64::from(l.n_axial);
        let t = (p[2] - z0) / l.z_pitch;
        if (t - t.round()).abs() * l.z_pitch < 1e-9 {
            return Err(());
        }
        if t < 0.0 || t >= f64::from(l.n_axial) {
            return Ok(None);
        }
        t.floor() as u32
    } else {
        0
    };
    let n = l.n_rings as i32;
    let normals: [f64; 3] = match l.orientation {
        HexOrientation::X => [0.0, 60.0, 120.0],
        HexOrientation::Y => [30.0, 90.0, 150.0],
    };
    let mut hit = None;
    for q in -(n - 1)..n {
        for r in -(n - 1)..n {
            let Some(u) = l.universe_at(q, r, layer) else { continue };
            let c = l.element_center(q, r, layer);
            let (dx, dy) = (p[0] - c[0], p[1] - c[1]);
            let m = normals.iter().map(|a| (dx * a.to_radians().cos() + dy * a.to_radians().sin()).abs() - 0.5 * l.pitch).fold(f64::NEG_INFINITY, f64::max);
            if m.abs() < 1e-9 {
                return Err(());
            }
            if m < 0.0 {
                assert!(hit.is_none(), "two elements hold {p:?}");
                hit = Some(u);
            }
        }
    }
    Ok(hit)
}

#[test]
fn a_brute_force_locator_over_the_ir_finds_what_openmc_found() {
    for name in MODELS {
        let (ir, o) = ir_of(name);
        let l = &ir.hex_lattices[&50];
        let (mut compared, mut inside, mut outside) = (0, 0, 0);
        for pt in o["points"].as_array().unwrap() {
            let a = pt["p"].as_array().unwrap();
            let p = [f(&a[0]), f(&a[1]), f(&a[2])];
            let want = match pt["cell"].as_u64().unwrap() {
                900 => None,
                c => Some(((c - 100) / 10 + 10) as u32),
            };
            // For a lattice without z extent the lattice plane is the slab: z does not matter.
            let Ok(got) = locate(l, p) else { continue };
            assert_eq!(got, want, "{name}: point {p:?} ({})", pt["kind"]);
            compared += 1;
            if want.is_some() { inside += 1 } else { outside += 1 }
        }
        assert!(compared >= 560, "{name}: only {compared} of 600 points compared");
        assert!(inside >= 200 && outside >= 30, "{name}: {inside} inside, {outside} outside: the oracle is vacuous");
    }
}

#[test]
fn the_axial_map_is_a_bijection_and_walks_the_rings() {
    for orientation in [HexOrientation::X, HexOrientation::Y] {
        for n in 1..=9u32 {
            let mut seen = std::collections::BTreeSet::new();
            for ring in 0..n {
                let len = HexLattice::ring_len(n, ring) as u32;
                let radius = (n - 1 - ring) as i32;
                let mut prev = None;
                for pos in 0..=len {
                    let (q, r) = HexLattice::ring_axial(orientation, n, ring, pos % len).unwrap();
                    assert_eq!((q.abs().max(r.abs()).max((q + r).abs())), radius, "{orientation:?} n={n} ring {ring} pos {pos}");
                    if pos < len {
                        assert!(seen.insert((q, r)), "{orientation:?} n={n}: ({q},{r}) twice");
                        assert_eq!(HexLattice::ring_position(orientation, n, q, r), Some((ring, pos)));
                    }
                    if let Some((pq, pr)) = prev {
                        // Consecutive positions (and the last back to the first) are neighbours.
                        let (dq, dr): (i32, i32) = (q - pq, r - pr);
                        assert!(len == 1 || [(1, 0), (-1, 0), (0, 1), (0, -1), (1, -1), (-1, 1)].contains(&(dq, dr)), "{orientation:?} n={n} ring {ring} pos {pos}: step ({dq},{dr})");
                    }
                    prev = Some((q, r));
                }
            }
            assert_eq!(seen.len(), HexLattice::elements_per_layer(n));
            assert_eq!(HexLattice::ring_axial(orientation, n, 0, HexLattice::ring_len(n, 0) as u32), None);
        }
    }
}

#[test]
fn rectangular_decks_emit_no_hex_section_and_hex_decks_do() {
    let (a, _) = ir_of("hex_x");
    let (b, _) = ir_of("hex_y");
    let (c, _) = ir_of("hex_x3d");
    let (ha, hb, hc) = (a.hex_section_hash().unwrap(), b.hex_section_hash().unwrap(), c.hex_section_hash().unwrap());
    assert!(ha != hb && ha != hc && hb != hc);
    assert_eq!(ha, ir_of("hex_x").0.hex_section_hash().unwrap());
    // A deck with only rectangular lattices has no section at all.
    let rect = std::fs::read_to_string(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/openmc/lattice3d/geometry.xml")).unwrap();
    let ir = openmc::import_hex(&rect, None).unwrap();
    assert!(ir.hex_lattices.is_empty() && !ir.lattices.is_empty());
    assert_eq!(ir.hex_section_hash(), None);
    // `import_hex` on a rectangular deck is `import`.
    assert_eq!(ir, openmc::import(&rect, None).unwrap());
    // A changed universe or pitch changes the section.
    let mut d = a.clone();
    d.hex_lattices.get_mut(&50).unwrap().universes.swap(0, 1);
    assert_ne!(d.hex_section_hash(), Some(ha));
    let mut d = a.clone();
    d.hex_lattices.get_mut(&50).unwrap().pitch += 1e-9;
    assert_ne!(d.hex_section_hash(), Some(ha));
}

#[test]
fn consumers_that_cannot_use_hex_lattices_still_refuse_them() {
    let (g, m, _) = load("hex_y3d");
    // The default reader entry (sm-csgbrep op, smcsg, sm-rt-bench) is unchanged.
    match openmc::import(&g, Some(&m)) {
        Err(ImportError::Unsupported { line: Some(l), what }) => {
            assert!(what.contains("hex_lattice"), "{what}");
            assert_eq!(l, 1 + g.lines().position(|t| t.contains("<hex_lattice")).unwrap() as u32);
        }
        other => panic!("{other:?}"),
    }
    // The flattener refuses an IR that has one, with the same typed error (no line in an IR).
    let ir = openmc::import_hex(&g, Some(&m)).unwrap();
    for r in [ImportedCsg::instantiate(&ir, &ImportOptions::default()).map(|_| ()), ImportedCsg::import(&ir, &ImportOptions::default()).map(|_| ())] {
        match r {
            Err(ImportError::Unsupported { line: None, what }) => assert!(what.contains("hex_lattice"), "{what}"),
            other => panic!("{other:?}"),
        }
    }
    // A cell reaches the hex lattice through the ordinary lattice fill.
    assert!(ir.cells.values().any(|c| matches!(c.fill, Fill::Lattice { id: 50, .. })));
}

fn hex_deck(lattice: &str) -> String {
    format!(
        r#"<geometry>
  <cell id="1" universe="0" fill="5" region="-1"/>
  <cell id="2" material="1" universe="7"/>
  <cell id="3" material="1" universe="8"/>
  <cell id="4" fill="8" universe="7"/>
  <surface id="1" type="sphere" coeffs="0 0 0 10" boundary="vacuum"/>
  {lattice}
</geometry>"#
    )
}

#[test]
fn malformed_hex_lattices_are_invalid_with_a_line() {
    let cases: [(&str, &str); 9] = [
        (r#"<hex_lattice id="5" n_rings="2" orientation="z"><pitch>1</pitch><center>0 0</center><universes>7 7 7 7 7 7 7</universes></hex_lattice>"#, "orientation"),
        (r#"<hex_lattice id="5" n_rings="2"><pitch>1</pitch><center>0 0</center><universes>7 7 7 7 7 7</universes></hex_lattice>"#, "need 7"),
        (r#"<hex_lattice id="5" n_rings="0"><pitch>1</pitch><center>0 0</center><universes>7</universes></hex_lattice>"#, "n_rings"),
        (r#"<hex_lattice id="5" n_rings="2" n_axial="2"><pitch>1</pitch><center>0 0</center><universes>7 7 7 7 7 7 7 7 7 7 7 7 7 7</universes></hex_lattice>"#, "pitch needs 2"),
        (r#"<hex_lattice id="5" n_rings="2"><pitch>1 2</pitch><center>0 0</center><universes>7 7 7 7 7 7 7</universes></hex_lattice>"#, "pitch needs 1"),
        (r#"<hex_lattice id="5" n_rings="2"><pitch>-1</pitch><center>0 0</center><universes>7 7 7 7 7 7 7</universes></hex_lattice>"#, "pitch must be positive"),
        (r#"<hex_lattice id="5" n_rings="2"><pitch>1</pitch><center>0 0</center></hex_lattice>"#, "missing universes"),
        (r#"<hex_lattice id="5" n_rings="2"><pitch>1</pitch><center>0 0</center><universes>7 7 7 7 7 7 9</universes></hex_lattice>"#, "line"),
        (r#"<hex_lattice id="5" n_rings="2" n_axial="0"><pitch>1 1</pitch><center>0 0 0</center><universes>7</universes></hex_lattice>"#, "n_axial"),
    ];
    for (lat, needle) in cases {
        let deck = hex_deck(lat);
        match openmc::import_hex(&deck, None) {
            Err(e @ ImportError::Invalid(_)) => {
                let s = e.to_string();
                assert!(s.contains(needle) || needle == "line", "{lat}: {s}");
                assert!(s.contains("line 6") || s.contains("hex_lattice 5"), "{s}");
            }
            // The last pinned case is a valid deck whose universe 9 does not exist: caught at flatten time.
            Ok(ir) if needle == "line" => assert!(ir.hex_lattices[&5].universes.contains(&9)),
            other => panic!("{lat}: {other:?}"),
        }
    }
}

#[test]
fn a_one_ring_lattice_and_a_one_layer_3d_lattice_read() {
    let one = hex_deck(r#"<hex_lattice id="5" n_rings="1"><pitch>2</pitch><center>1 2</center><universes>7</universes></hex_lattice>"#);
    let l = openmc::import_hex(&one, None).unwrap().hex_lattices[&5].clone();
    assert_eq!((l.n_rings, l.n_axial, l.universes.clone(), l.orientation), (1, 1, vec![7], HexOrientation::Y));
    assert_eq!(l.element_center(0, 0, 0), [1.0, 2.0, 0.0]);
    let layered = hex_deck(r#"<hex_lattice id="5" n_rings="1" n_axial="1" orientation="x"><pitch>2 3</pitch><center>1 2 4</center><universes>7</universes></hex_lattice>"#);
    let l = openmc::import_hex(&layered, None).unwrap().hex_lattices[&5].clone();
    assert_eq!((l.z_pitch, l.orientation, l.universes.clone()), (3.0, HexOrientation::X, vec![7]));
    assert_eq!(l.element_center(0, 0, 0), [1.0, 2.0, 4.0]);
    // Two layers: the second is above the first.
    let two = hex_deck(r#"<hex_lattice id="5" n_rings="1" n_axial="2"><pitch>2 3</pitch><center>0 0 0</center><universes>7 8</universes></hex_lattice>"#);
    let l = openmc::import_hex(&two, None).unwrap().hex_lattices[&5].clone();
    assert_eq!((l.universe_at(0, 0, 0), l.universe_at(0, 0, 1), l.universe_at(0, 0, 2)), (Some(7), Some(8), None));
    assert_eq!((l.element_center(0, 0, 0)[2], l.element_center(0, 0, 1)[2]), (-1.5, 1.5));
}

#[test]
fn ids_are_one_space_and_a_lattice_cannot_nest() {
    let both = hex_deck(
        r#"<lattice id="5"><pitch>1 1</pitch><dimension>1 1</dimension><lower_left>0 0</lower_left><universes>7</universes></lattice>
  <hex_lattice id="5" n_rings="1"><pitch>1</pitch><center>0 0</center><universes>7</universes></hex_lattice>"#,
    );
    assert!(matches!(openmc::import_hex(&both, None), Err(ImportError::Invalid(m)) if m.contains("duplicate lattice id")));
    let nested = hex_deck(
        r#"<hex_lattice id="6" n_rings="1"><pitch>1</pitch><center>0 0</center><universes>7</universes></hex_lattice>
  <hex_lattice id="5" n_rings="1"><pitch>1</pitch><center>0 0</center><universes>6</universes></hex_lattice>"#,
    );
    assert!(matches!(openmc::import_hex(&nested, None), Err(ImportError::Unsupported { .. })));
}

#[test]
fn import_is_deterministic() {
    for name in MODELS {
        assert_eq!(ir_of(name).0, ir_of(name).0);
    }
}
