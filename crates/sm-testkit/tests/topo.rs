//! B-rep and model fixtures against their analytic properties.

use sm_testkit::check::*;
use sm_testkit::topo::*;
use sm_topo_api::{Dim, TopologyView, TopologyViewExt, euler_characteristic};

fn counts(v: &dyn TopologyView) -> [usize; 5] {
    Dim::ALL.map(|d| v.cells(d).len())
}

#[test]
fn counts_euler_and_validity() {
    for f in all_topo_fixtures().unwrap() {
        assert_eq!(counts(&f.complex), f.counts, "{}", f.name);
        assert_eq!(euler_characteristic(&f.complex), f.euler, "{}", f.name);
        check_topo_euler(&f.complex, f.euler).unwrap();
        topo_valid(&f.complex).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
    }
}

#[test]
fn volumes_and_areas_match_closed_forms() {
    for f in all_topo_fixtures().unwrap() {
        if f.name == "sphere" {
            // Pole coedges contribute dv = 0 (CR-1): the moments exist and match closed forms.
            let v = f.complex.volume_moments(f.complex.volumes()[0]).unwrap().volume;
            assert!((v - f.volumes[0]).abs() <= 1e-6 * f.volumes[0].max(1.0), "sphere: {v}");
            continue;
        }
        let mut vols: Vec<f64> = f.complex.volumes().iter().map(|v| f.complex.volume_moments(*v).unwrap().volume).collect();
        vols.sort_by(f64::total_cmp);
        assert_eq!(vols.len(), f.volumes.len(), "{}", f.name);
        for (got, want) in vols.iter().zip(&f.volumes) {
            assert!((got - want).abs() <= 1e-6 * want.max(1.0), "{}: {got} vs {want}", f.name);
        }
        let area: f64 = f.complex.faces().iter().map(|x| f.complex.face_moments(*x).unwrap().area).sum();
        assert!((area - f.area).abs() <= 1e-6 * f.area, "{}: area {area} vs {}", f.name, f.area);
    }
}

#[test]
fn shared_and_cut_faces() {
    let m = merged_bricks().unwrap();
    // The merged face is shared by both volumes.
    let shared = m.complex.faces().iter().filter(|f| m.complex.face_sides(**f).is_some_and(|s| s.right != sm_topo_api::Region::Exterior)).count();
    assert_eq!(shared, 1);
    let w = webcut_cube(0.5).unwrap();
    assert_eq!(w.volumes, [0.5, 0.5]);
}

#[test]
fn manifold_surfaces_pass_the_edge_check_on_solids_without_t_junctions() {
    for f in [cube().unwrap(), cylinder(1.0, 2.0).unwrap(), sphere(1.0).unwrap()] {
        manifold_edges(&f.complex).unwrap_or_else(|e| panic!("{}: {e:?}", f.name));
    }
}

#[test]
fn pin_lattice_stub_flattens_to_nine_pins() {
    let f = pin_lattice_stub().unwrap();
    topo_valid(&f.complex).unwrap();
    model_valid(&f.model, &f.complex).unwrap();
    let flat = sm_model_api::flatten(&f.model).unwrap();
    assert_eq!(flat.paths.len(), f.flat_paths);
    assert_eq!(flat.entries.len(), f.flat_entries);
    assert!((f.volume - 0.32).abs() < 1e-15);
}

#[test]
fn mirror_chamber_has_one_self_paired_deck_face_and_half_the_volume() {
    let f = mirror_chamber().unwrap();
    topo_valid(&f.complex).unwrap();
    model_valid(&f.model, &f.complex).unwrap();
    assert_eq!(f.deck_faces, 1);
    assert_eq!(f.complex.volumes().len(), 1);
    let v = f.complex.volume_moments(f.complex.volumes()[0]).unwrap().volume;
    assert!((v - f.volume).abs() < 1e-12);
    assert!(sm_model_api::detect_symmetry(&f.model, &f.complex, sm_model_api::Tolerance::default()).is_ok());
    assert!(model_errors(&f).is_empty());
}
