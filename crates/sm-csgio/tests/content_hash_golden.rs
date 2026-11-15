//! `ImportedCsg::content_hash` of every committed OpenMC fixture is pinned (ACR-CSG-HEXIR, rule C3:
//! adding hex lattices to the IR must not move a hash of a rectangular deck). The values were
//! measured on the build before the IR gained `HexLattice`. Pinned for macOS arm64 only (the
//! flat model carries rotation matrices computed with libm, which may differ in a last bit on
//! other platforms; the other platforms run the structural checks below).
use sm_csgio::{ImportOptions, ImportedCsg, openmc};
use std::path::PathBuf;

fn fixtures() -> Vec<String> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/openmc");
    let mut v: Vec<String> = std::fs::read_dir(root).unwrap().filter_map(|e| e.ok()).filter(|e| e.path().join("geometry.xml").is_file() && e.file_name() != "outer_structured").map(|e| e.file_name().to_string_lossy().into_owned()).collect();
    v.sort();
    v
}

fn hash_of(name: &str, hex_reader: bool) -> u128 {
    let d = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/openmc").join(name);
    let g = std::fs::read_to_string(d.join("geometry.xml")).unwrap();
    let m = std::fs::read_to_string(d.join("materials.xml")).ok();
    let ir = if hex_reader { openmc::import_hex(&g, m.as_deref()) } else { openmc::import(&g, m.as_deref()) }.unwrap_or_else(|e| panic!("{name}: {e}"));
    assert!(ir.hex_lattices.is_empty(), "{name}");
    assert_eq!(ir.hex_section_hash(), None, "{name}: a rectangular deck emits no hex section");
    ImportedCsg::instantiate(&ir, &ImportOptions::default()).unwrap_or_else(|e| panic!("{name}: {e}")).content_hash().0
}

#[test]
fn the_hex_reader_gives_the_same_hash_as_the_plain_reader_on_every_fixture() {
    let names = fixtures();
    assert!(names.len() >= 20, "{names:?}");
    for n in names {
        assert_eq!(hash_of(&n, false), hash_of(&n, true), "{n}");
    }
}

#[cfg(all(target_os = "macos", target_arch = "aarch64"))]
#[test]
fn content_hashes_of_the_existing_fixtures_are_pinned() {
    let pins: &[(&str, u128)] = &[
        ("deck_o", 0xb240ce707ff6a7d383dc79622befdafa),
        ("disconnected", 0x5616e4dd297a776251c439b8c5531617),
        ("grammar", 0x395932b07c0656d9dcd9d12e84f7edaf),
        ("lattice2d", 0x7d32c320d196aa2b9e8acbe83c4ca57a),
        ("lattice3d", 0xe16b60a8099d6537e254604375a8767d),
        ("lattice_xf", 0x32deb26e2d419cff97599baf9b427587),
        ("nested", 0x02a7cf3f8b8e2ca6bab4691b2c65d881),
        ("pin", 0x34007f986ca0b73dea9384553f850100),
        ("placed_z", 0x2b2cc96843464df1fec8c7c0d4d1eb32),
        ("quadric_cyl", 0x1608a3533c066815a93f482e71ea45dd),
        ("ref_assembly17", 0x6e8649525bb2ae977824bb3e68bb1037),
        ("ref_assembly17_h2", 0x6db4bec22a4832dc848705a84a9ac0a3),
        ("ref_beavrs_assembly", 0x0902512f9a6af377ec99fa0382824ec5),
        ("ref_mini3x3", 0x9e93d6998381d40a630d84c90902a55d),
        ("ref_mini3x3_h2", 0xa3d7e86ad55af8aa4541818bd1c04fcf),
        ("ref_pin", 0xc208a8c53ab150759affb0ebdfd91a1e),
        ("ref_pin_h2", 0xa78d927dd4cde8207dd10d8ef6aa41e8),
        ("rotations", 0x152f0b7aeee397c48e6695985cb430ae),
        ("surfaces", 0x963b76d8ee45628867548e93c48ff27d),
        ("translated", 0x55857f088a948bf0ab5faa75ae1bb9c1),
    ];
    let got: Vec<(String, u128)> = fixtures().into_iter().map(|n| (n.clone(), hash_of(&n, false))).collect();
    assert_eq!(got.len(), pins.len());
    for ((n, h), (pn, ph)) in got.iter().zip(pins) {
        assert_eq!((n.as_str(), *h), (*pn, *ph));
    }
}
