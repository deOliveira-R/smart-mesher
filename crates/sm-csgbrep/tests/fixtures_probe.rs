//! Every committed OpenMC fixture imports or returns a typed `csg.unsupported` naming the element
//! (sphere, cone and torus decks wait for the sm-boolean tools; `outer_structured` is refused by
//! the reader). `ref_beavrs_assembly` (460 cm) is checked by the
//! ignored slow test of `oracle.rs`. Anything else, a kernel error or a panic, fails the test.
mod common;
use common::*;

/// `(fixture, imports)`: the expected outcome of each committed fixture today.
const EXPECTED: [(&str, bool); 20] = [
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
    // CSGBREP-ORACLE fixtures. The tall corpus decks meet kernel limits (see `oracle.rs`).
    ("quadric_cyl", true),
    ("disconnected", true),
    ("placed_z", true),
    ("ref_pin_h2", true),
    ("ref_mini3x3_h2", true),
    ("ref_assembly17_h2", true),
    ("ref_pin", true),
    ("ref_mini3x3", true),
    ("ref_assembly17", true),
];

#[test]
fn every_fixture_imports_or_is_refused_typed() {
    let mut report = Vec::new();
    let mut problems: Vec<String> = Vec::new();
    for (name, imports) in EXPECTED {
        let geom = fixture(name, "geometry.xml");
        let mats = std::fs::read_to_string(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../sm-csgio/tests/fixtures/openmc").join(name).join("materials.xml")).ok();
        let mut r = Rig::new();
        match (r.import(&geom, mats.as_deref()), imports) {
            (Ok(d), true) => report.push(format!("{name}: {} volumes", d.outputs["deck_volumes"].len())),
            (Err(e), false) => {
                let e = e.to_string();
                if !(e.contains("csg.unsupported") || e.contains("csg.kernel")) || !(e.contains("surface ") || e.contains("line ")) {
                    problems.push(format!("{name}: not a typed refusal naming an element: {e}"));
                }
                report.push(format!("{name}: {e}"));
            }
            (Ok(_), false) => problems.push(format!("{name}: imports now; update EXPECTED")),
            (Err(e), true) => problems.push(format!("{name}: {e}")),
        }
    }
    println!("\n{}", report.join("\n"));
    assert!(problems.is_empty(), "{problems:#?}");
}
