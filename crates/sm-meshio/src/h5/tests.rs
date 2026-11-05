use super::read::read_file;
use super::*;

fn sample() -> Group {
    let mut root = Group::new();
    let t = root.group("tstt");
    t.put_attr(Attr::u64("max_id", 7));
    t.put_type("elemtypes", Dtype::EnumU8(vec![("Edge".into(), 1), ("Tri".into(), 2)]));
    t.put_type("op", Dtype::Opaque(32));
    t.put_dataset("coords", Dataset::f64s(vec![2, 3], &[0.5, -1.0, 2.0, 3.0, 4.0, 5.0]).with_attr(Attr::i64("start_id", 1)));
    t.put_dataset("empty", Dataset::u64s(vec![0], &[]));
    t.put_dataset("ints", Dataset::i32s(vec![3], &[1, -2, 3]));
    t.put_dataset("pairs", Dataset::raw(Dtype::ArrayU64(2), vec![2], (0..32).collect()));
    t.group("sub").put_attr(Attr::raw("e", Dtype::EnumU8(vec![("Tri".into(), 2)]), vec![2]));
    root
}

#[test]
fn round_trip_and_determinism() {
    let a = write_file(sample()).unwrap();
    assert_eq!(a, write_file(sample()).unwrap());
    let back = read_file(&a).unwrap();
    assert_eq!(back, sample());
}

#[test]
fn lookup3_known_values() {
    // Bob Jenkins' lookup3.c driver values for hashlittle with initval 0.
    assert_eq!(lookup3(b""), 0xdead_beef);
    assert_eq!(lookup3(b"Four score and seven years ago"), 0x1777_0551);
}

#[test]
fn corrupted_input_never_panics() {
    let good = write_file(sample()).unwrap();
    let mut seed = 0x9e37_79b9_7f4a_7c15u64;
    let mut next = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        seed
    };
    for len in 0..good.len().min(200) {
        let _ = read_file(&good[..len]);
    }
    for _ in 0..2000 {
        let mut b = good.clone();
        for _ in 0..1 + next() % 4 {
            let i = (next() % b.len() as u64) as usize;
            b[i] = next() as u8;
        }
        let _ = read_file(&b);
    }
}

/// With `SM_H5_OUT=<path>` the sample file is written there for external readers (h5py).
#[test]
fn dump_sample_for_external_readers() {
    if let Ok(p) = std::env::var("SM_H5_OUT") {
        std::fs::write(p, write_file(sample()).unwrap()).unwrap();
    }
}

#[test]
fn unsupported_inputs_are_typed_errors() {
    let mut g = Group::new();
    g.put_dataset("bad", Dataset::raw(Dtype::U64, vec![3], vec![0; 8]));
    assert!(matches!(write_file(g), Err(H5Error::Unsupported(_))));
    let mut g = Group::new();
    g.put_dataset("a/b", Dataset::u64s(vec![0], &[]));
    assert!(matches!(write_file(g), Err(H5Error::Unsupported(_))));
}
