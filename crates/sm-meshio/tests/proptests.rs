//! Property tests on random hex blocks (seeded through `sm-testkit`).

mod common;

use common::assert_matches;
use proptest::prelude::*;
use sm_meshio::{SmshFile, SmshWriter, VtuOptions, vtu_bytes};
use sm_testkit::strategies::{self, hex_block_spec};

proptest! {
    #![proptest_config(strategies::config(24))]

    #[test]
    fn random_blocks_round_trip_and_keep_their_volume(spec in hex_block_spec()) {
        let m = spec.build().unwrap();
        let b = SmshWriter::new(&m).to_bytes().unwrap();
        let f = SmshFile::parse(&b).unwrap();
        f.verify_checksums().unwrap();
        let d = f.decode().unwrap();
        assert_matches(&m, &d);
        prop_assert_eq!(d.cell_ids.len(), spec.cells());
        // Total volume from the stored cell->face signs and coordinates: analytic prod(n h).
        let mut total = 0.0;
        for c in 0..spec.cells() {
            for sf in d.cell_face_list(c) {
                let l = d.face_loop(sf.face().get());
                let p = |v: u32| [d.x[v as usize], d.y[v as usize], d.z[v as usize]];
                // Axis-aligned quad: x-component of the area vector times the face's x coordinate.
                let (a, b2, c2) = (p(l[0]), p(l[1]), p(l[2]));
                let n = [
                    (b2[1] - a[1]) * (c2[2] - a[2]) - (b2[2] - a[2]) * (c2[1] - a[1]),
                    (b2[2] - a[2]) * (c2[0] - a[0]) - (b2[0] - a[0]) * (c2[2] - a[2]),
                    (b2[0] - a[0]) * (c2[1] - a[1]) - (b2[1] - a[1]) * (c2[0] - a[0]),
                ];
                total += sf.sign() * (a[0] * n[0] + a[1] * n[1] + a[2] * n[2]) / 3.0;
            }
        }
        prop_assert!((total - spec.volume()).abs() <= 1e-9 * spec.volume(), "{} vs {}", total, spec.volume());
        // The VTU of the same mesh is stable and has one cell per hex.
        let v = vtu_bytes(&m, &VtuOptions::default()).unwrap();
        let needle = format!("NumberOfCells=\"{}\"", spec.cells()).into_bytes();
        prop_assert!(v.windows(needle.len()).any(|w| w == needle.as_slice()));
        prop_assert_eq!(v, vtu_bytes(&m, &VtuOptions::default()).unwrap());
    }
}
