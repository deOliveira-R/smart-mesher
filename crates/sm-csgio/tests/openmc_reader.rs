//! The OpenMC reader on inline decks: Deck O, region grammar, boundaries, materials and names,
//! rejections (each a typed error naming the element and the line), hostile input.
//! Conventions against OpenMC itself are in `openmc_oracle.rs`.
mod common;
use common::*;
use sm_csgio::{ImportError, ImportOptions, ImportedCsg, openmc};
use std::collections::BTreeMap;

const SURFACES_O: &str = r#"
  <surface id="1" type="sphere" coeffs="0 0 0 1"/>
  <surface id="2" type="x-plane" coeffs="-2" boundary="vacuum"/>
  <surface id="3" type="x-plane" coeffs="2"  boundary="vacuum"/>
  <surface id="4" type="y-plane" coeffs="-2" boundary="vacuum"/>
  <surface id="5" type="y-plane" coeffs="2"  boundary="vacuum"/>
  <surface id="6" type="z-plane" coeffs="-2" boundary="vacuum"/>
  <surface id="7" type="z-plane" coeffs="2"  boundary="vacuum"/>
"#;

fn deck(cells: &str, surfaces: &str) -> String {
    format!("<geometry>\n{cells}\n{surfaces}\n</geometry>")
}

/// Deck O with the cell 2 region given.
fn deck_o_xml(region2: &str) -> String {
    deck(&format!(r#"<cell id="1" material="1" region="-1" universe="0"/><cell id="2" material="2" region="{region2}" universe="0"/>"#), SURFACES_O)
}

fn imp(geometry: &str, materials: Option<&str>) -> ImportedCsg {
    let ir = openmc::import(geometry, materials).unwrap_or_else(|e| panic!("{e}"));
    ImportedCsg::import(&ir, &ImportOptions::default()).unwrap_or_else(|e| panic!("{e}"))
}

fn err(geometry: &str) -> ImportError {
    openmc::import(geometry, None).and_then(|ir| ImportedCsg::instantiate(&ir, &ImportOptions::default()).map(|_| ir)).unwrap_err()
}

const MATERIALS: &str = r#"<materials>
  <material id="1" name="fuel"><density value="10.0" units="g/cm3"/><nuclide name="U235" ao="1.0"/></material>
  <material id="2" name="water"><density value="1.0" units="g/cm3"/></material>
</materials>"#;

#[test]
fn deck_o_chords_are_analytic_and_regions_follow_c9() {
    let m = imp(&deck_o_xml("1 2 -3 4 -5 6 -7"), None);
    assert_eq!(m.region_names(), ["material_1", "material_2"]);
    assert!(m.report().warnings.is_empty(), "{:?}", m.report().warnings);
    let t = tracker(&m);
    let c = chords(&t, [-2.0, 0.6, 0.0], [1.0, 0.0, 0.0]);
    assert!((c["material_1"] - 1.6).abs() < 1e-12 && (c["material_2"] - 2.4).abs() < 1e-12, "{c:?}");
    let c = chords(&t, [-2.0, 1.5, 0.0], [1.0, 0.0, 0.0]);
    assert!(!c.contains_key("material_1") && (c["material_2"] - 4.0).abs() < 1e-12, "{c:?}");
    // With materials.xml the names are used.
    let m = imp(&deck_o_xml("1 2 -3 4 -5 6 -7"), Some(MATERIALS));
    assert_eq!(m.region_names(), ["fuel", "water"]);
    let ir = openmc::import(&deck_o_xml("1 2 -3 4 -5 6 -7"), Some(MATERIALS)).unwrap();
    assert!(ir.dropped.iter().any(|d| d.contains("density")), "{:?}", ir.dropped);
}

#[test]
fn child_element_and_attribute_forms_give_the_same_deck() {
    let attrs = deck_o_xml("1 2 -3 4 -5 6 -7");
    let kids = r#"<geometry>
      <cell><id>1</id><material>1</material><region>-1</region><universe>0</universe></cell>
      <cell><id>2</id><material>2</material><region>1 2 -3 4 -5 6 -7</region></cell>
      <surface><id>1</id><type>sphere</type><coeffs>0 0 0 1</coeffs></surface>
      <surface><id>2</id><type>x-plane</type><coeffs>-2</coeffs><boundary>vacuum</boundary></surface>
      <surface><id>3</id><type>x-plane</type><coeffs>2</coeffs><boundary>vacuum</boundary></surface>
      <surface><id>4</id><type>y-plane</type><coeffs>-2</coeffs><boundary>vacuum</boundary></surface>
      <surface><id>5</id><type>y-plane</type><coeffs>2</coeffs><boundary>vacuum</boundary></surface>
      <surface><id>6</id><type>z-plane</type><coeffs>-2</coeffs><boundary>vacuum</boundary></surface>
      <surface><id>7</id><type>z-plane</type><coeffs>2</coeffs><boundary>vacuum</boundary></surface>
    </geometry>"#;
    assert_eq!(openmc::import(&attrs, None).unwrap(), openmc::import(kids, None).unwrap());
}

#[test]
fn region_forms_agree_and_precedence_matters() {
    let hash = |r: &str| imp(&deck_o_xml(r), None).content_hash();
    let h = hash("1 2 -3 4 -5 6 -7");
    assert_eq!(h, hash("~(-1) 2 -3 4 -5 6 -7"));
    assert_eq!(h, hash("+1 +2 -3 +4 -5 +6 -7"));
    assert_eq!(h, hash("~(-1 | -2 | 3 | -4 | 5 | -6 | 7)"));
    assert_eq!(h, hash("(1) (((2 -3))) 4 ~(~(-5 6)) -7"));
    // `|` binds weaker than intersection: `-1 | 2 -3` is -1 or (2 and -3), not (-1 or 2) and -3.
    let cells = r#"<cell id="1" material="1" region="-1 | 2 -3" universe="5"/><cell id="2" material="2" region="~(-1 | 2 -3)" universe="5"/>
      <cell id="3" fill="5" region="-10 11 -12 13 -14 15" universe="0"/>"#;
    let surf = r#"<surface id="1" type="sphere" coeffs="0 0 0 1"/><surface id="2" type="x-plane" coeffs="1.5"/><surface id="3" type="x-plane" coeffs="2"/>
      <surface id="10" type="x-plane" coeffs="3" boundary="vacuum"/><surface id="11" type="x-plane" coeffs="-3" boundary="vacuum"/>
      <surface id="12" type="y-plane" coeffs="3" boundary="vacuum"/><surface id="13" type="y-plane" coeffs="-3" boundary="vacuum"/>
      <surface id="14" type="z-plane" coeffs="3" boundary="vacuum"/><surface id="15" type="z-plane" coeffs="-3" boundary="vacuum"/>"#;
    let t = tracker(&imp(&deck(cells, surf), None));
    // Along +x at y = 2.5: the sphere is missed; material_1 is only the slab 1.5 < x < 2.
    let c = chords(&t, [-3.0, 2.5, 0.0], [1.0, 0.0, 0.0]);
    assert!((c["material_1"] - 0.5).abs() < 1e-12 && (c["material_2"] - 5.5).abs() < 1e-12, "{c:?}");
}

#[test]
fn overlap_is_rejected_with_the_deck_cells_for_one_and_many_threads() {
    let ir = openmc::import(&deck_o_xml("2 -3 4 -5 6 -7"), None).unwrap();
    for threads in [1, 4] {
        let e = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(|| ImportedCsg::import(&ir, &ImportOptions::default()).unwrap_err());
        let ImportError::Overlap { findings } = e else { panic!("expected Overlap, got {e:?}") };
        let f = &findings[0];
        assert!((f.point[0].powi(2) + f.point[1].powi(2) + f.point[2].powi(2)).sqrt() < 1.0);
        let mut cells = [f.a.cell, f.b.cell];
        cells.sort_unstable();
        assert_eq!(cells, [1, 2]);
    }
}

#[test]
fn boundaries_become_rules_and_reflective_planes_reflect() {
    let g = deck_o_xml("1 2 -3 4 -5 6 -7").replace(r#"coeffs="-2" boundary="vacuum""#, r#"coeffs="-2" boundary="reflective""#).replace(r#"id="3" type="x-plane" coeffs="2"  boundary="vacuum""#, r#"id="3" type="x-plane" coeffs="2" boundary="reflective""#);
    let m = imp(&g, None);
    assert_eq!(m.boundaries().rules.len(), 6);
    assert!(m.report().warnings.is_empty());
    let t = tracker(&m);
    let mut sc = sm_rt_api::Scratch::new();
    use sm_rt_api::Tracker;
    let mut st = t.init_state(sm_core::Point3::new(0.0, 0.0, 0.0), sm_core::Vector3::new(1.0, 0.0, 0.0), None, &mut sc).unwrap();
    let mut sphere = 0.0;
    let mut sink = |s: &sm_rt_api::Segment| {
        if t.regions().get(s.region).map(|r| r.name.as_str()) == Some("material_1") {
            sphere += s.length;
        }
        sm_rt_api::Flow::Continue
    };
    t.trace(&mut st, &sm_rt_api::TraceOptions::default().with_max_dist(2.0 + 4.0 * 4.0), &mut sink, &mut sc).unwrap();
    assert!((sphere - 9.0).abs() < 1e-9, "{sphere}");
    // white is accepted, a reflective sphere warns, periodic is refused.
    let w = deck_o_xml("1 2 -3 4 -5 6 -7").replace(r#"coeffs="-2" boundary="vacuum""#, r#"coeffs="-2" boundary="white""#);
    assert!(imp(&w, None).report().warnings.is_empty());
    let s = deck_o_xml("1 2 -3 4 -5 6 -7").replace(r#"coeffs="0 0 0 1""#, r#"coeffs="0 0 0 1" boundary="reflective""#);
    assert!(imp(&s, None).report().warnings.iter().any(|w| w.code == "import.reflective_curved"));
    let p = deck_o_xml("1 2 -3 4 -5 6 -7").replace(r#"coeffs="-2" boundary="vacuum""#, r#"coeffs="-2" boundary="periodic""#);
    assert!(matches!(err(&p), ImportError::Unsupported { line: Some(_), what } if what.contains("periodic")));
}

#[test]
fn materials_names_void_and_unknown_ids() {
    // `material="void"` is the region "void".
    let g = deck_o_xml("1 2 -3 4 -5 6 -7").replace(r#"material="2""#, r#"material="void""#);
    assert_eq!(imp(&g, None).region_names(), ["material_1", "void"]);
    // Duplicate and empty names fall back to material_<id> (C9); the override map renames.
    let dup = r#"<materials><material id="1" name="x"/><material id="2" name="x"/></materials>"#;
    assert_eq!(imp(&deck_o_xml("1 2 -3 4 -5 6 -7"), Some(dup)).region_names(), ["material_1", "material_2"]);
    let ir = openmc::import(&deck_o_xml("1 2 -3 4 -5 6 -7"), Some(MATERIALS)).unwrap();
    let o = ImportOptions { material_names: BTreeMap::from([(1, "uo2".to_string())]), ..ImportOptions::default() };
    assert_eq!(ImportedCsg::instantiate(&ir, &o).unwrap().region_names(), ["uo2", "water"]);
    // A material id materials.xml does not define.
    let only1 = r#"<materials><material id="1" name="fuel"/></materials>"#;
    let ir = openmc::import(&deck_o_xml("1 2 -3 4 -5 6 -7"), Some(only1)).unwrap();
    assert!(matches!(ImportedCsg::instantiate(&ir, &ImportOptions::default()), Err(ImportError::UnknownMaterial { cell: 2, material: 2 })));
}

#[test]
fn refused_features_name_the_element_and_the_line() {
    let cases: [(&str, &str); 7] = [
        (r#"<hex_lattice id="2"><pitch>1</pitch></hex_lattice>"#, "hex_lattice"),
        (r#"<dagmc_universe id="2" filename="x.h5m"/>"#, "dagmc_universe"),
        (r#"<cell id="9" material="1 2" region="-1"/>"#, "material list"),
        (r#"<surface id="9" type="x-torus" coeffs="0 0 0 2 0.5 0.4"/>"#, "unequal"),
        (r#"<surface id="9" type="x-torus" coeffs="0 0 0 0.4 0.5 0.5"/>"#, "ring tori"),
        (r#"<surface id="9" type="y-plane" coeffs="1" boundary="periodic"/>"#, "periodic"),
        (r#"<surface id="9" type="x-hyperboloid" coeffs="1"/>"#, "x-hyperboloid"),
    ];
    for (extra, needle) in cases {
        let g = format!("<geometry>\n<cell id=\"1\" material=\"1\" region=\"-1\"/>\n{extra}\n<surface id=\"1\" type=\"sphere\" coeffs=\"0 0 0 1\"/>\n</geometry>");
        match openmc::import(&g, None) {
            Err(ImportError::Unsupported { line: Some(l), what }) => {
                assert!(what.contains(needle), "{what}");
                assert_eq!(l, 3, "{what}");
            }
            other => panic!("{needle}: expected Unsupported, got {other:?}"),
        }
    }
}

#[test]
fn nested_lattices_and_improper_rotations_are_unsupported() {
    let uni = r#"<cell id="10" material="1" universe="1"/>"#;
    let lat = |inner: &str| {
        format!(
            r#"<geometry>{uni}<cell id="1" fill="21" region="-1"/><surface id="1" type="sphere" coeffs="0 0 0 5" boundary="vacuum"/>
            <lattice id="20"><pitch>1 1</pitch><dimension>1 1</dimension><lower_left>-0.5 -0.5</lower_left><universes>1</universes></lattice>
            <lattice id="21"><pitch>2 2</pitch><dimension>1 1</dimension><lower_left>-1 -1</lower_left><universes>{inner}</universes></lattice></geometry>"#
        )
    };
    assert!(matches!(openmc::import(&lat("20"), None), Err(ImportError::Unsupported { what, .. }) if what.contains("nested")));
    assert!(openmc::import(&lat("1"), None).is_ok());
    let rot = |r: &str| format!(r#"<geometry>{uni}<cell id="1" fill="1" rotation="{r}" region="-1"/><surface id="1" type="sphere" coeffs="0 0 0 5" boundary="vacuum"/></geometry>"#);
    assert!(openmc::import(&rot("1 0 0 0 1 0 0 0 1"), None).is_ok());
    assert!(matches!(openmc::import(&rot("1 0 0 0 1 0 0 0 -1"), None), Err(ImportError::Unsupported { what, .. }) if what.contains("reflection")));
    assert!(matches!(openmc::import(&rot("2 0 0 0 1 0 0 0 1"), None), Err(ImportError::Unsupported { what, .. }) if what.contains("orthonormal")));
    assert!(matches!(openmc::import(&rot("1 2"), None), Err(ImportError::Invalid(_))));
}

#[test]
fn malformed_decks_are_typed_errors() {
    let bad: [(&str, &str); 12] = [
        ("<geometry><cell id=\"1\" material=\"1\" region=\"-1\"", "parse"),
        ("<geometry></materials>", "parse"),
        ("<!DOCTYPE geometry [<!ENTITY a \"b\">]><geometry/>", "parse"),
        ("<other/>", "parse"),
        (r#"<geometry><cell id="1" material="1" region="-1 |"/><surface id="1" type="sphere" coeffs="0 0 0 1"/></geometry>"#, "invalid"),
        (r#"<geometry><cell id="1" material="1" region="(-1"/><surface id="1" type="sphere" coeffs="0 0 0 1"/></geometry>"#, "invalid"),
        (r#"<geometry><cell id="1" material="1" region="-1 )"/><surface id="1" type="sphere" coeffs="0 0 0 1"/></geometry>"#, "invalid"),
        (r#"<geometry><cell id="1" material="1" region="-1 x"/><surface id="1" type="sphere" coeffs="0 0 0 1"/></geometry>"#, "invalid"),
        (r#"<geometry><cell id="1" material="1" fill="2" region="-1"/><surface id="1" type="sphere" coeffs="0 0 0 1"/></geometry>"#, "invalid"),
        (r#"<geometry><cell id="1" material="1" region="-1"/><cell id="1" material="1" region="1"/><surface id="1" type="sphere" coeffs="0 0 0 1"/></geometry>"#, "invalid"),
        (r#"<geometry><cell id="1" material="1" region="-1"/><surface id="1" type="sphere" coeffs="0 0 0"/></geometry>"#, "invalid"),
        (r#"<geometry><cell id="1" material="1" region="-1"/><surface id="1" type="sphere" coeffs="0 0 0 -1"/></geometry>"#, "invalid"),
    ];
    for (g, kind) in bad {
        match (openmc::import(g, None), kind) {
            (Err(ImportError::Parse { line, col, .. }), "parse") => assert!(line >= 1 && col >= 1),
            (Err(ImportError::Invalid(m)), "invalid") => assert!(m.contains("line "), "{m}"),
            (other, _) => panic!("{g}: {other:?}"),
        }
    }
    // Undefined surface and universe ids are named by the core.
    assert!(matches!(err(r#"<geometry><cell id="4" material="1" region="-9"/></geometry>"#), ImportError::UnknownSurface { cell: 4, surface: 9 }));
    assert!(matches!(err(r#"<geometry><cell id="4" fill="7" region="-1"/><surface id="1" type="sphere" coeffs="0 0 0 1"/></geometry>"#), ImportError::UnknownUniverse { .. }));
}

#[test]
fn the_root_universe_is_the_unfilled_one() {
    // Root universe 3 (not 0): found because no cell fills it.
    let g = r#"<geometry><cell id="1" fill="4" region="-1" universe="3"/><cell id="2" material="1" universe="4"/>
      <surface id="1" type="sphere" coeffs="0 0 0 1" boundary="vacuum"/></geometry>"#;
    let ir = openmc::import(g, None).unwrap();
    assert_eq!(ir.root_universe, 3);
    assert_eq!(imp(g, None).region_names(), ["material_1"]);
    // Two roots, or none, are errors.
    let two = r#"<geometry><cell id="1" material="1" region="-1" universe="3"/><cell id="2" material="1" region="1" universe="4"/><surface id="1" type="sphere" coeffs="0 0 0 1"/></geometry>"#;
    assert!(matches!(openmc::import(two, None), Err(ImportError::Invalid(m)) if m.contains("more than one root")));
    let none = r#"<geometry><cell id="1" fill="3" universe="3"/></geometry>"#;
    assert!(matches!(openmc::import(none, None), Err(ImportError::Invalid(m)) if m.contains("no root")));
}

fn pin_lattice_xml(fuel_r: f64, cells_extra: &str) -> String {
    deck(
        &format!(
            r#"<cell id="10" material="1" region="-1" universe="1"/><cell id="11" material="2" region="1" universe="1"/>
      <cell id="12" material="2" universe="2"/>
      <cell id="1" fill="5" region="2 -3 4 -5 6 -7" universe="0"/>{cells_extra}
      <lattice id="5"><pitch>2 2</pitch><dimension>2 1</dimension><lower_left>-1 -1</lower_left><universes>1 2</universes></lattice>"#
        ),
        &format!(
            r#"<surface id="1" type="z-cylinder" coeffs="0 0 {fuel_r}"/>
      <surface id="2" type="x-plane" coeffs="-1" boundary="vacuum"/><surface id="3" type="x-plane" coeffs="3" boundary="vacuum"/>
      <surface id="4" type="y-plane" coeffs="-1" boundary="vacuum"/><surface id="5" type="y-plane" coeffs="1" boundary="vacuum"/>
      <surface id="6" type="z-plane" coeffs="-1" boundary="vacuum"/><surface id="7" type="z-plane" coeffs="1" boundary="vacuum"/>"#
        ),
    )
}

#[test]
fn oversize_pins_are_clipped_and_a_tangent_pin_is_no_overlap() {
    // Lattice 2x1: universe 1's pin of radius 1.2 > 1 spills past the element wall. OpenMC looks
    // a point up inside its own element only, so the spill is clipped and nothing overlaps (the
    // draft's "oversize pin is an overlap" does not hold for OpenMC; see DESIGN.md).
    assert!(ImportedCsg::import(&openmc::import(&pin_lattice_xml(1.2, ""), None).unwrap(), &ImportOptions::default()).is_ok());
    // Tangent pin (r = 1 in a pitch-2 element): chord at the tangency is zero, no overlap.
    let m = imp(&pin_lattice_xml(1.0, ""), None);
    let t = tracker(&m);
    let c = chords(&t, [-1.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    assert!((c["material_1"] - 2.0).abs() < 1e-9, "{c:?}");
    // The same with a pin in both elements: still clipped, no overlap.
    let two = pin_lattice_xml(1.2, "").replace("<universes>1 2</universes>", "<universes>1 1</universes><outer>2</outer>");
    assert!(ImportedCsg::import(&openmc::import(&two, None).unwrap(), &ImportOptions::default()).is_ok());
}

#[test]
fn hostile_input_never_panics() {
    // Deeply nested elements and parentheses.
    let deep = format!("<geometry>{}{}</geometry>", "<a>".repeat(20_000), "</a>".repeat(20_000));
    assert!(openmc::import(&deep, None).is_err());
    let parens = format!(r#"<geometry><cell id="1" material="1" region="{}-1{}"/><surface id="1" type="sphere" coeffs="0 0 0 1"/></geometry>"#, "(".repeat(5000), ")".repeat(5000));
    assert!(matches!(openmc::import(&parens, None), Err(ImportError::Invalid(m)) if m.contains("nesting")));
    let nots = format!(r#"<geometry><cell id="1" material="1" region="{}-1"/><surface id="1" type="sphere" coeffs="0 0 0 1"/></geometry>"#, "~".repeat(5000));
    assert!(openmc::import(&nots, None).is_err());
    // A billion-laughs DTD is refused before expansion.
    let laughs = r#"<!DOCTYPE g [<!ENTITY a "aaaaaaaaaa"><!ENTITY b "&a;&a;&a;&a;&a;&a;&a;&a;&a;&a;">]><geometry a="&b;"/>"#;
    assert!(matches!(openmc::import(laughs, None), Err(ImportError::Parse { .. })));
    // A fixed-seed corpus: 500 truncations and byte mutations of Deck O and of a lattice deck.
    let mut x: u64 = 0x9E37_79B9_7F4A_7C15;
    let mut next = move || {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        x
    };
    let bases = [deck_o_xml("1 2 -3 4 -5 6 -7"), pin_lattice_xml(0.4, "")];
    for i in 0..500 {
        let base = bases[i % 2].as_bytes();
        let mut b = base.to_vec();
        match next() % 3 {
            0 => b.truncate((next() as usize) % b.len()),
            1 => {
                for _ in 0..1 + next() % 4 {
                    let k = (next() as usize) % b.len();
                    b[k] = (next() & 0xff) as u8;
                }
            }
            _ => {
                let k = (next() as usize) % b.len();
                b.insert(k, b"<>&\"'(~|-+ 0"[(next() % 12) as usize]);
            }
        }
        let text = String::from_utf8_lossy(&b).into_owned();
        // Must not panic; instantiation of what parses must not panic either.
        if let Ok(ir) = openmc::import(&text, None) {
            let _ = ImportedCsg::instantiate(&ir, &ImportOptions::default());
        }
    }
}
