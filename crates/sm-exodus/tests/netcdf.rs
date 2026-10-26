//! netCDF codec: byte-level layout (acceptance 1), size limits (acceptance 9), round trips.

use sm_exodus::ExodusError;
use sm_exodus::netcdf::{NcData, NcFile, NcFormat, NcType, NcValue};

const FORMATS: [NcFormat; 3] = [NcFormat::Classic, NcFormat::Offset64, NcFormat::Data64];

/// Bytes assembled by hand from the netCDF file-format BNF: dim `x` = 2, global char attribute
/// `t = "ab"`, variable `v` (NC_INT over `x`, no attributes) holding 1, 2.
fn hand_assembled(format: NcFormat) -> Vec<u8> {
    let (cw, ow) = match format {
        NcFormat::Classic => (4, 4),
        NcFormat::Offset64 => (4, 8),
        NcFormat::Data64 => (8, 8),
    };
    let nn = |out: &mut Vec<u8>, v: u64| {
        if cw == 8 {
            out.extend_from_slice(&v.to_be_bytes());
        } else {
            out.extend_from_slice(&(v as u32).to_be_bytes());
        }
    };
    let mut h = Vec::new();
    h.extend_from_slice(b"CDF");
    h.push(match format {
        NcFormat::Classic => 1,
        NcFormat::Offset64 => 2,
        NcFormat::Data64 => 5,
    });
    nn(&mut h, 0); // numrecs
    h.extend_from_slice(&0x0Au32.to_be_bytes()); // NC_DIMENSION
    nn(&mut h, 1);
    nn(&mut h, 1); // name "x"
    h.extend_from_slice(b"x\0\0\0");
    nn(&mut h, 2); // dim_length
    h.extend_from_slice(&0x0Cu32.to_be_bytes()); // NC_ATTRIBUTE (global)
    nn(&mut h, 1);
    nn(&mut h, 1);
    h.extend_from_slice(b"t\0\0\0");
    h.extend_from_slice(&2u32.to_be_bytes()); // NC_CHAR
    nn(&mut h, 2);
    h.extend_from_slice(b"ab\0\0");
    h.extend_from_slice(&0x0Bu32.to_be_bytes()); // NC_VARIABLE
    nn(&mut h, 1);
    nn(&mut h, 1);
    h.extend_from_slice(b"v\0\0\0");
    nn(&mut h, 1); // ndims
    nn(&mut h, 0); // dimid
    h.extend_from_slice(&0u32.to_be_bytes()); // ABSENT attribute list
    nn(&mut h, 0);
    h.extend_from_slice(&4u32.to_be_bytes()); // NC_INT
    nn(&mut h, 8); // vsize
    let begin = h.len() as u64 + ow as u64;
    if ow == 8 {
        h.extend_from_slice(&begin.to_be_bytes());
    } else {
        h.extend_from_slice(&(begin as u32).to_be_bytes());
    }
    h.extend_from_slice(&1i32.to_be_bytes());
    h.extend_from_slice(&2i32.to_be_bytes());
    h
}

fn small(format: NcFormat) -> NcFile<'static> {
    let mut f = NcFile::new(format);
    let x = f.dim("x", Some(2));
    f.global_attr("t", NcValue::Text("ab".into()));
    let v = f.var("v", NcType::Int, &[x]);
    f.set_data(v, NcData::from_i32s(&[1, 2]));
    f
}

#[test]
fn acc1_hand_assembled_bytes_equal_the_writer() {
    for fmt in FORMATS {
        let mut got = Vec::new();
        small(fmt).write(&mut got).unwrap();
        let want = hand_assembled(fmt);
        assert_eq!(got, want, "{fmt:?}");
    }
    // Anchors: the CDF-1 header is 100 bytes, CDF-2 104, CDF-5 134 (wider counts).
    let sizes: Vec<u64> = FORMATS.iter().map(|f| small(*f).layout().unwrap().header_size).collect();
    assert_eq!(sizes[0], 100);
    assert_eq!(sizes[1], 104);
}

#[test]
fn acc1_layout_invariants() {
    for fmt in FORMATS {
        let mut f = NcFile::new(fmt);
        let t = f.dim("t", None);
        let a = f.dim("a", Some(3));
        let b = f.dim("b", Some(5));
        let v1 = f.var("bytes", NcType::Byte, &[a]); // 3 bytes -> padded to 4
        let v2 = f.var("txt", NcType::Char, &[b]); // 5 -> 8
        let v3 = f.var("d", NcType::Double, &[a, b]);
        let _rec = f.var("rec", NcType::Double, &[t]);
        f.set_data(v1, NcData::Bytes(vec![1, 2, 3]));
        f.set_data(v2, NcData::from_text("hello"));
        f.set_data(v3, NcData::from_f64s(&[0.5; 15]));
        let lay = f.layout().unwrap();
        let fixed: Vec<_> = lay.vars.iter().filter(|v| !v.is_record).collect();
        let mut prev = lay.header_size;
        for v in &fixed {
            assert_eq!(v.begin % 4, 0);
            assert_eq!(v.vsize % 4, 0);
            assert_eq!(v.begin, prev);
            prev = v.begin + v.vsize;
        }
        let last = fixed.last().unwrap();
        let mut out = Vec::new();
        let n = f.write(&mut out).unwrap();
        assert_eq!(out.len() as u64, n);
        assert_eq!(n, last.begin + last.vsize);
        assert_eq!(lay.file_size, n);
        // The record variable starts at end of file, zero records written.
        let rec = lay.vars.iter().find(|v| v.is_record).unwrap();
        assert_eq!(rec.begin, n);
        assert_eq!(rec.vsize, 8);
        let magic = match fmt {
            NcFormat::Classic => 1,
            NcFormat::Offset64 => 2,
            NcFormat::Data64 => 5,
        };
        assert_eq!(&out[..4], &[0x43, 0x44, 0x46, magic]);
    }
}

#[test]
fn round_trip_read_write() {
    for fmt in FORMATS {
        let mut f = NcFile::new(fmt);
        let t = f.dim("t", None);
        let a = f.dim("a", Some(2));
        f.global_attr("g", NcValue::Float(vec![8.11]));
        f.global_attr("w", NcValue::Int(vec![-3, 7]));
        let v = f.var("d", NcType::Double, &[a]);
        f.var_attr(v, "name", NcValue::Text("ID".into()));
        f.var_attr(v, "scale", NcValue::Double(vec![0.1]));
        f.set_data(v, NcData::from_f64s(&[1.5, -2.25]));
        f.var("time", NcType::Double, &[t]);
        let mut bytes = Vec::new();
        f.write(&mut bytes).unwrap();
        let back = NcFile::read(&bytes).unwrap();
        assert_eq!(back.format(), fmt);
        assert_eq!(back.dims(), f.dims());
        assert_eq!(back.global_attrs(), f.global_attrs());
        assert_eq!(back.var_by_name("d").unwrap().attrs, f.var_by_name("d").unwrap().attrs);
        assert_eq!(back.var_bytes("d").unwrap(), f.var_bytes("d").unwrap());
        let mut again = Vec::new();
        // The decoded record variable has no data: writing it back reproduces the file.
        back.write(&mut again).unwrap();
        assert_eq!(again, bytes);
    }
}

#[test]
fn lazy_provider_matches_bytes() {
    let mut a = NcFile::new(NcFormat::Offset64);
    let d = a.dim("n", Some(3));
    let v = a.var("v", NcType::Int, &[d]);
    a.set_data(v, NcData::Lazy(Box::new(|| [1i32, 2, 3].iter().flat_map(|x| x.to_be_bytes()).collect())));
    let mut b = NcFile::new(NcFormat::Offset64);
    let d = b.dim("n", Some(3));
    let v = b.var("v", NcType::Int, &[d]);
    b.set_data(v, NcData::from_i32s(&[1, 2, 3]));
    let (mut x, mut y) = (Vec::new(), Vec::new());
    a.write(&mut x).unwrap();
    b.write(&mut y).unwrap();
    assert_eq!(x, y);
}

#[test]
fn acc9_limits_without_allocation() {
    let big_2d = |fmt: NcFormat, a_len: u64, b_len: u64, n_vars: usize, last_big: bool| {
        let mut f = NcFile::new(fmt);
        let a = f.dim("a", Some(a_len));
        let b = f.dim("b", Some(b_len));
        let small_dim = f.dim("s", Some(4));
        for i in 0..n_vars {
            let big = if last_big { i == n_vars - 1 } else { i == 0 };
            let dims = if big { vec![a, b] } else { vec![small_dim] };
            f.var(&format!("v{i}"), NcType::Byte, &dims);
        }
        f
    };
    // 5 GiB = 65536 * 81920 bytes.
    let (a, b) = (1u64 << 16, 81920u64);
    let five_gib = a * b;
    assert_eq!(five_gib, 5 * (1 << 30));

    // Classic: a single variable above u32::MAX is too large; two 3 GB variables exceed 4 GiB in total.
    let f = big_2d(NcFormat::Classic, a, b, 2, true);
    assert!(matches!(f.layout(), Err(ExodusError::TooLarge { .. })));
    let mut two = NcFile::new(NcFormat::Classic);
    let d = two.dim("d", Some(3_000_000_000));
    two.var("p", NcType::Byte, &[d]);
    two.var("q", NcType::Byte, &[d]);
    assert!(matches!(two.layout(), Err(ExodusError::TooLarge { .. })));

    // Offset64: 5 GiB not last -> TooLarge; as the last non-record variable -> vsize clamped.
    let f = big_2d(NcFormat::Offset64, a, b, 2, false);
    assert!(matches!(f.layout(), Err(ExodusError::TooLarge { .. })));
    let f = big_2d(NcFormat::Offset64, a, b, 2, true);
    let lay = f.layout().unwrap();
    assert_eq!(lay.vars[1].vsize, 0xFFFF_FFFF);
    assert_eq!(lay.vars[1].size, five_gib);
    assert_eq!(lay.file_size, lay.vars[1].begin + five_gib);

    // Data64 accepts both, including a 2^33-element dimension.
    let f = big_2d(NcFormat::Data64, a, b, 2, false);
    let lay = f.layout().unwrap();
    assert_eq!(lay.vars[0].vsize, five_gib);
    let f = big_2d(NcFormat::Data64, a, b, 2, true);
    assert_eq!(f.layout().unwrap().vars[1].vsize, five_gib);
    let mut g = NcFile::new(NcFormat::Data64);
    let d = g.dim("huge", Some(1 << 33));
    g.var("v", NcType::Double, &[d]);
    let lay = g.layout().unwrap();
    assert_eq!(lay.vars[0].vsize, 1u64 << 36);
    // A 2^33 dimension is not representable below Data64.
    let mut h = NcFile::new(NcFormat::Offset64);
    let d = h.dim("huge", Some(1 << 33));
    h.var("v", NcType::Byte, &[d]);
    assert!(matches!(h.layout(), Err(ExodusError::TooLarge { .. })));
}

#[test]
fn model_errors_are_typed() {
    let mut f = NcFile::new(NcFormat::Offset64);
    f.dim("zero", Some(0));
    assert!(matches!(f.layout(), Err(ExodusError::Malformed(_))));
    let mut f = NcFile::new(NcFormat::Offset64);
    f.dim("t", None);
    f.dim("u", None);
    assert!(matches!(f.layout(), Err(ExodusError::Malformed(_))));
    let mut f = NcFile::new(NcFormat::Offset64);
    let d = f.dim("d", Some(1));
    f.var("v", NcType::Int64, &[d]);
    assert!(matches!(f.layout(), Err(ExodusError::Unsupported(_))));
    // Missing data / wrong size are errors, not panics.
    let mut f = NcFile::new(NcFormat::Offset64);
    let d = f.dim("d", Some(2));
    let v = f.var("v", NcType::Int, &[d]);
    assert!(matches!(f.write(&mut Vec::new()), Err(ExodusError::Malformed(_))));
    f.set_data(v, NcData::Bytes(vec![0; 7]));
    assert!(matches!(f.write(&mut Vec::new()), Err(ExodusError::Malformed(_))));
}

#[test]
fn read_rejects_bad_input_without_panicking() {
    assert!(matches!(NcFile::read(b"\x89HDF\r\n\x1a\n rest"), Err(ExodusError::Unsupported(m)) if m.contains("HDF5")));
    assert!(matches!(NcFile::read(b""), Err(ExodusError::Malformed(_))));
    assert!(matches!(NcFile::read(b"CDF\x07"), Err(ExodusError::Malformed(_))));
    for fmt in FORMATS {
        let good = hand_assembled(fmt);
        for cut in 0..good.len() {
            // Every truncation is a typed error (the data tail is checked against the end).
            assert!(NcFile::read(&good[..cut]).is_err(), "{fmt:?} cut {cut}");
        }
        // Flip every byte once: must not panic (result may be Ok or Err).
        for i in 0..good.len() {
            let mut bad = good.clone();
            bad[i] ^= 0xFF;
            let _ = NcFile::read(&bad);
        }
    }
}

#[test]
fn cdl_of_a_small_file() {
    let f = small(NcFormat::Offset64);
    let cdl = f.to_cdl();
    let want = "netcdf file {\ndimensions:\n\tx = 2 ;\nvariables:\n\tint v(x) ;\n\n// global attributes:\n\t\t:t = \"ab\" ;\ndata:\n\n v = 1, 2 ;\n}\n";
    assert_eq!(cdl, want);
}
