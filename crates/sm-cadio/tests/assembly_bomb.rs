//! A small assembly file must not make flattening allocate without bound: the number of placed
//! instances is capped by the file's size, and cycles are found at the first revisit.

use sm_cadio::{ReadOptions, StepError, read_step};

mod common;

/// The pin file up to its part representation (`#160`, whose placement item is `#159`).
fn pin_prefix() -> String {
    let text = String::from_utf8(common::data("assembly_pins.stp")).expect("utf8");
    let end = text.find("#161 =").expect("part representation is followed by #161");
    text[..end].to_owned()
}

const FOOTER: &str = "ENDSEC;\nEND-ISO-10303-21;\n";

/// `levels` representations stacked on the part, `fan` relationships between each pair.
fn fan_out_file(fan: usize, levels: usize) -> Vec<u8> {
    let mut s = pin_prefix();
    let (mut id, mut prev_rep, mut prev_frame) = (1000u64, 160u64, 159u64);
    for _ in 0..levels {
        let (p, d, x, f, rep) = (id, id + 1, id + 2, id + 3, id + 4);
        id += 5;
        s += &format!("#{p} = CARTESIAN_POINT('',(1.,0.,0.));\n#{d} = DIRECTION('',(0.,0.,1.));\n#{x} = DIRECTION('',(1.,0.,0.));\n#{f} = AXIS2_PLACEMENT_3D('',#{p},#{d},#{x});\n#{rep} = SHAPE_REPRESENTATION('',(#{f}),#5);\n");
        for _ in 0..fan {
            let (t, r) = (id, id + 1);
            id += 2;
            s += &format!("#{t} = ITEM_DEFINED_TRANSFORMATION('','',#{prev_frame},#{f});\n#{r} = ( REPRESENTATION_RELATIONSHIP('','',#{prev_rep},#{rep}) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#{t}) SHAPE_REPRESENTATION_RELATIONSHIP() );\n");
        }
        (prev_rep, prev_frame) = (rep, f);
    }
    s += &format!("#{id} = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','','',#1,#1,$);\n");
    (s + FOOTER).into_bytes()
}

#[test]
fn a_fan_out_bomb_is_refused_with_a_typed_error() {
    // 4 links per level over 12 levels would be 4^12 = 16.7M solids from about 17 KB.
    let bytes = fan_out_file(4, 12);
    assert!(bytes.len() < 40_000, "{}", bytes.len());
    let e = read_step(&bytes, &ReadOptions { assume_unit: None, ..ReadOptions::default() }).expect_err("bomb");
    assert!(matches!(e, StepError::TooManyInstances { .. }), "{e:?}");
    assert_eq!(e.code(), "step.too_many_instances");
}

#[test]
fn a_modest_nesting_still_flattens() {
    let m = read_step(&fan_out_file(2, 3), &ReadOptions::default()).expect("2^3 instances");
    assert_eq!(m.solids.len(), 8);
}

#[test]
fn a_self_linked_cycle_with_high_fan_out_fails_fast_and_falls_back_to_local_coordinates() {
    // The representation linked to itself `fan` times: the first revisit is a cycle (the old
    // depth counter did fan^16 work first).
    let mut s = pin_prefix();
    s += "#1000 = CARTESIAN_POINT('',(1.,0.,0.));\n#1001 = DIRECTION('',(0.,0.,1.));\n#1002 = DIRECTION('',(1.,0.,0.));\n#1003 = AXIS2_PLACEMENT_3D('',#1000,#1001,#1002);\n";
    for k in 0..20u64 {
        let (t, r) = (1010 + 2 * k, 1011 + 2 * k);
        s += &format!("#{t} = ITEM_DEFINED_TRANSFORMATION('','',#159,#1003);\n#{r} = ( REPRESENTATION_RELATIONSHIP('','',#160,#160) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#{t}) SHAPE_REPRESENTATION_RELATIONSHIP() );\n");
    }
    s += "#1100 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','','',#1,#1,$);\n";
    s += FOOTER;
    let t = std::time::Instant::now();
    let m = read_step(s.as_bytes(), &ReadOptions::default()).expect("a cycle degrades, it does not fail");
    assert!(t.elapsed().as_secs() < 5);
    assert_eq!(m.solids.len(), 1);
    assert!(m.notes.iter().any(|n| n.code.as_str() == "step.assembly_transforms_ignored"), "{:?}", m.notes);
}
