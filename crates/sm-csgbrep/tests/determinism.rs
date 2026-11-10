//! Determinism: two runs, one thread against many, and renumbered or reordered decks give the
//! same state hash.
mod common;
use common::decks::*;
use common::*;

fn in_pool<R: Send>(threads: usize, f: impl FnOnce() -> R + Send) -> R {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(f)
}

fn state(geom: &str, mats: Option<&str>) -> (String, Vec<String>) {
    let mut r = Rig::new();
    let d = r.import(geom, mats).expect("import");
    (format!("{:?}", r.hash()), d.diagnostics.iter().map(|x| format!("{} {}", x.code, x.message)).collect())
}

#[test]
fn runs_threads_and_cold_starts_agree() {
    let mats4 = materials(4);
    let decks: Vec<(String, Option<String>)> = vec![
        (fixture("pin", "geometry.xml"), Some(fixture("pin", "materials.xml"))),
        (fixture("lattice2d", "geometry.xml"), Some(fixture("lattice2d", "materials.xml"))),
        (fixture("lattice_xf", "geometry.xml"), Some(fixture("lattice_xf", "materials.xml"))),
        (pin_lattice(3, 1.26, (0.35, 0.37, 0.42), 2.0, true), Some(mats4)),
    ];
    for (g, m) in &decks {
        let a = state(g, m.as_deref());
        assert_eq!(a, state(g, m.as_deref()));
        assert_eq!(a, in_pool(1, || state(g, m.as_deref())));
        assert_eq!(a, in_pool(4, || state(g, m.as_deref())));
    }
}

#[test]
fn renumbering_and_reordering_the_deck_keeps_the_state_hash() {
    let mats4 = materials(4);
    let plain = state(&pin_lattice(2, 1.26, (0.3, 0.4, 0.5), 1.0, true), Some(&mats4));
    // Surface, cell and universe ids shifted, elements written in the opposite order (the lattice
    // id stays: the lattice frame is named after it, so renumbering it would move the hash).
    let moved = state(&pin_lattice_ex(2, 1.26, (0.3, 0.4, 0.5), 1.0, true, 500, true), Some(&mats4));
    assert_eq!(plain.0, moved.0);
    // Not vacuous: a different radius is a different state.
    let other = state(&pin_lattice(2, 1.26, (0.3, 0.4, 0.51), 1.0, true), Some(&mats4));
    assert_ne!(plain.0, other.0);
}
