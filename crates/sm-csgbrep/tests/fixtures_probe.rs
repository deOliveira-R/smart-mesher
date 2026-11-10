//! Every committed OpenMC fixture imports or returns a typed `csg.unsupported` naming the element
//! (sphere, cone and torus decks wait for the sm-boolean tools; `outer_structured` is refused by
//! the reader). Anything else, a kernel error or a panic, fails the test.
mod common;
use common::*;

/// `(fixture, imports)`: the expected outcome of each committed fixture today.
const EXPECTED: [(&str, bool); 11] = [
    ("pin", true),
    ("lattice2d", true),
    ("lattice_xf", true),
    ("lattice3d", false),
    ("deck_o", false),
    ("nested", false),
    ("rotations", false),
    ("translated", false),
    ("surfaces", false),
    ("outer_structured", false),
    ("grammar", false),
];

#[test]
fn every_fixture_imports_or_is_refused_typed() {
    let mut report = Vec::new();
    for (name, imports) in EXPECTED {
        let geom = fixture(name, "geometry.xml");
        let mats = std::fs::read_to_string(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../sm-csgio/tests/fixtures/openmc").join(name).join("materials.xml")).ok();
        let mut r = Rig::new();
        match (r.import(&geom, mats.as_deref()), imports) {
            (Ok(d), true) => report.push(format!("{name}: {} volumes", d.outputs["deck_volumes"].len())),
            (Err(e), false) => {
                let e = e.to_string();
                assert!(e.contains("csg.unsupported"), "{name}: not a typed Unsupported: {e}");
                // Names the element: a deck surface id or a deck line.
                assert!(e.contains("surface ") || e.contains("line "), "{name}: {e}");
                report.push(format!("{name}: {e}"));
            }
            (Ok(_), false) => panic!("{name}: imports now; update EXPECTED"),
            (Err(e), true) => panic!("{name}: {e}"),
        }
    }
    println!("\n{}", report.join("\n"));
}
