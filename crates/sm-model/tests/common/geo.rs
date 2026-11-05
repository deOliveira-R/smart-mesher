//! Measurement helpers for the W3b primitive tests.
#![allow(dead_code)]

use sm_op_api::Session;
use sm_topo_api::{Dim, TopologyViewExt, euler_characteristic};

use super::model;

/// `[V, E, F, volumes, bodies]` counts of the model topology.
pub fn counts(s: &Session) -> Vec<usize> {
    let t = model(s).topo();
    Dim::ALL.iter().map(|&d| t.cells(d).len()).collect()
}

/// Volume of every live volume (ascending id) from `volume_moments`.
pub fn volumes(s: &Session) -> Vec<f64> {
    let t = model(s).topo();
    t.volumes().iter().map(|&v| t.volume_moments(v).expect("volume moments").volume).collect()
}

/// Sum of `face_moments` areas over every live face.
pub fn total_area(s: &Session) -> f64 {
    let t = model(s).topo();
    t.cells(Dim::Face).iter().map(|&f| t.face_moments(f).expect("face moments").area).sum()
}

/// Topological Euler characteristic: `V - E + F` with degenerate edges left out, minus one per
/// extra loop (an annular face has characteristic 0, not 1). It is 2 for a closed solid whose
/// boundary is one sphere, however it is cut.
pub fn euler_with_holes(s: &Session) -> i64 {
    let t = model(s).topo();
    let extra: usize = t.cells(Dim::Face).iter().map(|&f| t.loops(f).count() - 1).sum();
    euler_characteristic(t) - extra as i64
}

/// Relative closeness with a floor of 1 on the scale.
pub fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * b.abs().max(1.0)
}

/// `validate` and `validate_geometry` are empty.
pub fn clean(s: &Session) {
    assert_eq!(sm_topo_api::validate(model(s).topo()), vec![], "validate");
    assert_eq!(sm_topo_api::validate_geometry(model(s).topo()), vec![], "validate_geometry");
}

/// Run `f` against both topology hosts.
pub fn both(f: impl Fn(super::Factory)) {
    f(super::cell_complex);
    f(super::topo_store);
}
