//! CSGBREP-CHECK: exact overlap and gap pieces, measured on the piece (not by subtraction),
//! placed-surface dedup by tolerance, and the reserved-box handling of unowned pieces.
mod common;
use common::decks::*;
use common::*;
use core::f64::consts::PI;
use sm_csgbrep::{OpenmcOptions, OverlapMode};

fn close(what: &str, got: f64, want: f64, tol: f64) {
    assert!((got - want).abs() <= tol * want.abs(), "{what}: got {got:e}, want {want:e}");
}

/// The first `volume <number>` of a diagnostic message.
fn piece_volume(msg: &str) -> f64 {
    let rest = msg.split("volume ").nth(1).unwrap_or_else(|| panic!("{msg}"));
    rest.split_whitespace().next().and_then(|w| w.trim_end_matches([';', ',']).parse().ok()).unwrap_or_else(|| panic!("{msg}"))
}

/// Two boxes side by side along x in a 10 x 10 x 10 container: cell 1 is `[0, a]`, cell 2 is
/// `[b, 10]`. `b < a` overlaps, `b > a` leaves a gap.
fn two_boxes(a: &str, b: &str) -> String {
    format!(
        "<?xml version='1.0'?><geometry>\
         <cell id='1' material='1' region='1 -2 3 -4 5 -6' universe='1'/>\
         <cell id='2' material='2' region='7 -8 3 -4 5 -6' universe='1'/>\
         <surface id='1' type='x-plane' boundary='vacuum' coeffs='0'/><surface id='2' type='x-plane' coeffs='{a}'/>\
         <surface id='7' type='x-plane' coeffs='{b}'/><surface id='8' type='x-plane' boundary='vacuum' coeffs='10'/>\
         <surface id='3' type='y-plane' boundary='vacuum' coeffs='0'/><surface id='4' type='y-plane' boundary='vacuum' coeffs='10'/>\
         <surface id='5' type='z-plane' boundary='vacuum' coeffs='0'/><surface id='6' type='z-plane' boundary='vacuum' coeffs='10'/></geometry>"
    )
    .replace('\'', "\"")
}

fn report() -> Option<OpenmcOptions> {
    Some(OpenmcOptions { overlap: Some(OverlapMode::Report), ..Default::default() })
}

#[test]
fn a_planar_overlap_slab_is_reported_with_its_exact_volume() {
    for (a, b, thick) in [("5.5", "5", 0.5), ("5.1", "5", 0.1)] {
        let deck = two_boxes(a, b);
        // Default policy: the error carries the piece volume.
        let e = Rig::new().import(&deck, Some(&materials(2))).unwrap_err().to_string();
        close("error volume", piece_volume(&e), thick * 100.0, 1e-9);
        // Report: one diagnostic with the same volume; the model has no double-owned space.
        let mut r = Rig::new();
        let d = r.import_with(&deck, Some(&materials(2)), report()).unwrap();
        let o = diag(&d, "import.overlap");
        assert_eq!(o.len(), 1, "{o:?}");
        close("overlap volume", piece_volume(o[0]), thick * 100.0, 1e-9);
        let total: f64 = d.outputs["deck_volumes"].iter().map(|&v| r.volume(v)).sum();
        close("union", total, 1000.0, 1e-9);
    }
}

#[test]
fn a_missing_slab_is_one_gap_piece_with_its_exact_volume() {
    let deck = two_boxes("4.95", "5.05");
    let mut r = Rig::new();
    let d = r.import(&deck, Some(&materials(2))).unwrap();
    let g = diag(&d, "import.gap");
    assert_eq!(g.len(), 1, "{g:?}");
    close("gap volume", piece_volume(g[0]), 0.1 * 100.0, 1e-9);
    assert!(diag(&d, "import.overlap").is_empty());
    assert_eq!(d.outputs["deck_volumes"].len(), 2);
}

#[test]
fn round_off_twin_planes_are_one_surface() {
    // The shared face written twice, 1e-12 apart: no sliver gap, no sliver overlap.
    let deck = two_boxes("5", "5.000000000001");
    let mut r = Rig::new();
    let d = r.import(&deck, Some(&materials(2))).unwrap();
    assert!(diag(&d, "import.gap").is_empty() && diag(&d, "import.overlap").is_empty(), "{:?}", d.diagnostics);
    let v: Vec<f64> = d.outputs["deck_volumes"].iter().map(|&x| r.volume(x)).collect();
    assert_eq!(v.len(), 2);
    close("sum", v.iter().sum(), 1000.0, 1e-12);
    // And swapping the two values does not change the state hash.
    let mut r2 = Rig::new();
    r2.import(&two_boxes("5.000000000001", "5"), Some(&materials(2))).unwrap();
    assert_eq!(r.hash(), r2.hash());
}

#[test]
fn equal_radii_on_two_surface_ids_leave_no_tangent_contact_or_gap() {
    // Fuel inside cylinder 1, clad between cylinder 2 (same radius, other id: a zero gap) and 3.
    let deck = "<?xml version='1.0'?><geometry>\
        <cell id='1' material='1' region='-1 5 -6' universe='1'/>\
        <cell id='2' material='2' region='2 -3 5 -6' universe='1'/>\
        <surface id='1' type='z-cylinder' coeffs='0 0 0.4'/><surface id='2' type='z-cylinder' coeffs='0 0 0.4'/>\
        <surface id='3' type='z-cylinder' boundary='vacuum' coeffs='0 0 1'/>\
        <surface id='5' type='z-plane' boundary='vacuum' coeffs='0'/><surface id='6' type='z-plane' boundary='vacuum' coeffs='3'/></geometry>"
        .replace('\'', "\"");
    let mut r = Rig::new();
    let d = r.import(&deck, Some(&materials(2))).unwrap();
    assert!(diag(&d, "import.gap").is_empty() && diag(&d, "import.overlap").is_empty(), "{:?}", d.diagnostics);
    let mut v: Vec<f64> = d.outputs["deck_volumes"].iter().map(|&x| r.volume(x)).collect();
    v.sort_by(f64::total_cmp);
    close("fuel", v[0], PI * 0.16 * 3.0, 1e-9);
    close("clad", v[1], PI * 0.84 * 3.0, 1e-9);
}

#[test]
fn equal_radii_in_two_universes_placed_coaxially_import() {
    let deck = "<?xml version='1.0'?><geometry>\
        <cell id='1' material='1' region='-1' universe='2'/>\
        <cell id='2' material='2' region='1' universe='2'/>\
        <cell id='3' fill='2' region='-3 5 -6' universe='1'/>\
        <surface id='1' type='z-cylinder' coeffs='0 0 0.4'/>\
        <surface id='3' type='z-cylinder' boundary='vacuum' coeffs='0 0 1'/>\
        <surface id='5' type='z-plane' boundary='vacuum' coeffs='0'/><surface id='6' type='z-plane' boundary='vacuum' coeffs='3'/></geometry>"
        .replace('\'', "\"");
    let mut r = Rig::new();
    let d = r.import(&deck, Some(&materials(2))).unwrap();
    assert!(diag(&d, "import.gap").is_empty() && diag(&d, "import.overlap").is_empty(), "{:?}", d.diagnostics);
    assert_eq!(d.outputs["deck_volumes"].len(), 2);
}

/// The literal draft rows: a gap of 1e-3 and an overlap sliver of 1e-4 in a box of 10 (the
/// kernel cuts a brick by a plane this close to a parallel face since W4-BOOL-TOOLS).
#[test]
fn the_draft_thin_rows_wait_for_the_kernel() {
    let mut r = Rig::new();
    let d = r.import(&two_boxes("4.9995", "5.0005"), Some(&materials(2))).unwrap();
    close("gap volume", piece_volume(diag(&d, "import.gap")[0]), 1e-3 * 100.0, 1e-9);
    let mut r = Rig::new();
    let d = r.import_with(&two_boxes("5.0001", "5"), Some(&materials(2)), report()).unwrap();
    close("sliver volume", piece_volume(diag(&d, "import.overlap")[0]), 1e-4 * 100.0, 1e-9);
}

/// A cut 1e-3 (relative) from a face imports, with exact volumes (once a kernel limit).
#[test]
fn a_cut_very_close_to_a_face_imports_with_exact_volumes() {
    let mut r = Rig::new();
    let d = r.import(&two_boxes("0.01", "0.01"), Some(&materials(2))).unwrap();
    let mut v: Vec<f64> = d.outputs["deck_volumes"].iter().map(|&x| r.volume(x)).collect();
    v.sort_by(f64::total_cmp);
    assert_eq!(v.len(), 2);
    close("thin", v[0], 0.01 * 100.0, 1e-9);
    close("thick", v[1], 9.99 * 100.0, 1e-9);
}

/// Unowned remainder pieces are cut by the array boxes like owned ones: a gap strip that runs
/// across a lattice array is reported without the part under the array (which the lattice
/// fills), and its volume is exact.
#[test]
fn an_unowned_piece_across_an_array_reports_only_the_gap_outside_it() {
    let h = 2.0;
    let extra = "<geometry>\n  <cell id=\"800\" material=\"4\" region=\"505 -506 501 -502 15 -16\" universe=\"0\"/>\n  <cell id=\"801\" material=\"4\" region=\"505 -506 503 -504 15 -16\" universe=\"0\"/>\n  \
<surface id=\"501\" type=\"y-plane\" coeffs=\"3\"/><surface id=\"502\" type=\"y-plane\" coeffs=\"5\"/>\n  \
<surface id=\"503\" type=\"y-plane\" coeffs=\"-5\"/><surface id=\"504\" type=\"y-plane\" coeffs=\"-3\"/>\n  \
<surface id=\"505\" type=\"x-plane\" coeffs=\"-5\"/><surface id=\"506\" type=\"x-plane\" coeffs=\"5\"/>\n";
    let deck = pin_lattice(3, 1.26, (0.35, 0.37, 0.42), h, false).replace("<geometry>\n", extra);
    let mut r = Rig::new();
    let d = r.import(&deck, Some(&materials(4))).unwrap();
    let gaps = diag(&d, "import.gap");
    let total: f64 = gaps.iter().map(|g| piece_volume(g)).sum();
    // The strip |y| < 3 over |x| < 5, minus the array box (3.78 x 3.78), times the height.
    close("gap volume", total, (10.0 * 6.0 - 3.78 * 3.78) * h, 1e-9);
    assert!(gaps.len() >= 2, "{gaps:?}");
}
