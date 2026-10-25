//! AnglePicker: classification, rule repair, hard requests, sides.
mod common;

use common::{l_face, split_brick};
use sm_core::EntityId;
use sm_scheme_api::geom;
use sm_scheme_api::{
    ConstraintId, CornerPicker, Firm, Firmness, ReferenceCornerPicker, SchemeError, SideDir,
    VertexRule, VertexType,
};
use sm_sizing::{AnglePicker, classify_angle};
use sm_topo_api::{CornerKey, TopologyView};

fn type_at(layout: &sm_scheme_api::FaceLayout, topo: &dyn TopologyView, v: EntityId) -> VertexType {
    let face = layout.face;
    let key = geom::corners(topo, face).expect("corners").into_iter().find(|c| c.vertex == v).expect("corner").key;
    layout.corners.iter().find(|c| c.corner == key).expect("assignment").vertex_type
}

fn key_at(topo: &dyn TopologyView, face: EntityId, v: EntityId) -> CornerKey {
    geom::corners(topo, face).expect("corners").into_iter().find(|c| c.vertex == v).expect("corner").key
}

fn counts(layout: &sm_scheme_api::FaceLayout) -> [usize; 4] {
    let mut c = [0; 4];
    for a in &layout.corners {
        c[a.vertex_type as usize] += 1;
    }
    c
}

#[test]
fn thresholds_follow_cubit_135_225_315() {
    use core::f64::consts::PI;
    let d = |x: f64| x.to_radians();
    assert_eq!(classify_angle(d(90.0)), VertexType::End);
    assert_eq!(classify_angle(3.0 * PI / 4.0), VertexType::End);
    assert_eq!(classify_angle(d(135.0) + 1e-9), VertexType::Side);
    assert_eq!(classify_angle(d(180.0)), VertexType::Side);
    assert_eq!(classify_angle(d(224.9)), VertexType::Side);
    // `VertexType::nearest` ties down at 225; the picker rule is `Side < 225 <= Corner`.
    assert_eq!(classify_angle(d(225.0)), VertexType::Corner);
    assert_eq!(classify_angle(d(270.0)), VertexType::Corner);
    assert_eq!(classify_angle(d(315.0)), VertexType::Reversal);
    assert_eq!(classify_angle(d(359.0)), VertexType::Reversal);
}

#[test]
fn rectangle_with_a_mid_edge_vertex_is_four_ends_and_a_side() {
    let f = split_brick();
    let picker = AnglePicker;
    for face in [f.ids.faces[2], f.ids.faces[4]] {
        let layout = picker.pick(&f.topo, face, VertexRule::Map, &[], &[]).expect("map");
        assert_eq!(counts(&layout), [4, 1, 0, 0]);
        assert_eq!(type_at(&layout, &f.topo, f.mid), VertexType::Side);
        assert_eq!(layout.sides.len(), 4);
        // The side holding the middle vertex has two coedges.
        let mut lens: Vec<usize> = layout.sides.iter().map(|s| s.coedges.len()).collect();
        lens.sort_unstable();
        assert_eq!(lens, vec![1, 1, 1, 2]);
    }
}

#[test]
fn layouts_equal_the_reference_picker_where_the_choice_is_forced() {
    let f = split_brick();
    let (a, r) = (AnglePicker, ReferenceCornerPicker);
    for rule in [VertexRule::Map, VertexRule::Submap, VertexRule::Free, VertexRule::Ends(4)] {
        for face in f.ids.faces {
            let (x, y) = (a.pick(&f.topo, face, rule, &[], &[]), r.pick(&f.topo, face, rule, &[], &[]));
            let (x, y) = (x.expect("ours"), y.expect("reference"));
            let types = |l: &sm_scheme_api::FaceLayout| l.corners.iter().map(|c| (c.corner, c.vertex_type)).collect::<Vec<_>>();
            assert_eq!(types(&x), types(&y), "{rule:?} {face}");
            assert_eq!(x.sides, y.sides, "{rule:?} {face}: sides and directions");
        }
    }
}

#[test]
fn submap_sides_on_the_l_face_equal_the_reference_picker() {
    // Both pickers are forced here (five Ends and one Corner, sum 4), so any difference is in the
    // side cutting and quarter-turn directions with a real Corner turn.
    let f = l_face();
    for rule in [VertexRule::Submap, VertexRule::Free] {
        let ours = AnglePicker.pick(&f.topo, f.face, rule, &[], &[]).expect("ours");
        let theirs = ReferenceCornerPicker.pick(&f.topo, f.face, rule, &[], &[]).expect("reference");
        let types = |l: &sm_scheme_api::FaceLayout| l.corners.iter().map(|c| (c.corner, c.vertex_type)).collect::<Vec<_>>();
        assert_eq!(types(&ours), types(&theirs), "{rule:?}");
        assert_eq!(ours.sides, theirs.sides, "{rule:?}");
    }
}

#[test]
fn l_shaped_face_under_submap_has_one_corner_and_sum_four() {
    let f = l_face();
    let layout = AnglePicker.pick(&f.topo, f.face, VertexRule::Submap, &[], &[]).expect("submap");
    assert_eq!(counts(&layout), [5, 0, 1, 0]);
    let sum: i32 = layout.corners.iter().map(|c| c.vertex_type.submap_weight()).sum();
    assert_eq!(sum, 4);
    assert_eq!(type_at(&layout, &f.topo, f.vertices[3]), VertexType::Corner);
    // Six sides, turning left at Ends and right at the Corner: i-j frame closes up.
    assert_eq!(layout.sides.len(), 6);
    let dirs: Vec<SideDir> = layout.sides.iter().map(|s| s.dir).collect();
    assert_eq!(dirs[0], SideDir::PlusI);
}

#[test]
fn l_shaped_face_under_map_flattens_the_corner_and_demotes_the_lowest_id_end() {
    let f = l_face();
    let layout = AnglePicker.pick(&f.topo, f.face, VertexRule::Map, &[], &[]).expect("map");
    assert_eq!(counts(&layout), [4, 2, 0, 0]);
    assert_eq!(type_at(&layout, &f.topo, f.vertices[3]), VertexType::Side, "the re-entrant vertex");
    // Five Ends tie on angular cost; the lowest vertex id is the corner that is adjusted.
    let mut ends: Vec<EntityId> = f.vertices.iter().copied().filter(|&v| v != f.vertices[3]).collect();
    ends.sort_unstable();
    assert_eq!(type_at(&layout, &f.topo, ends[0]), VertexType::Side);
    for &v in &ends[1..] {
        assert_eq!(type_at(&layout, &f.topo, v), VertexType::End);
    }
}

#[test]
fn hard_requests_win_and_the_rest_adapts() {
    let f = split_brick();
    let face = f.ids.faces[2];
    // The geometric corner at the origin becomes a Side and the mid vertex an End.
    let origin = f.ids.vertices[0];
    let reqs = [
        (key_at(&f.topo, face, origin), Firm::hard(VertexType::Side)),
        (key_at(&f.topo, face, f.mid), Firm::hard(VertexType::End)),
    ];
    let layout = AnglePicker.pick(&f.topo, face, VertexRule::Map, &reqs, &[]).expect("map");
    assert_eq!(type_at(&layout, &f.topo, origin), VertexType::Side);
    assert_eq!(type_at(&layout, &f.topo, f.mid), VertexType::End);
    assert_eq!(counts(&layout), [4, 1, 0, 0]);
    assert!(layout.corners.iter().any(|c| c.firmness == Firmness::Hard));
}

#[test]
fn submap_repair_changes_the_fewest_types() {
    let f = split_brick();
    let face = f.ids.faces[2];
    // A hard End on the mid vertex makes the sum 5; exactly one type must change.
    let reqs = [(key_at(&f.topo, face, f.mid), Firm::hard(VertexType::End))];
    let layout = AnglePicker.pick(&f.topo, face, VertexRule::Submap, &reqs, &[]).expect("submap");
    let sum: i32 = layout.corners.iter().map(|c| c.vertex_type.submap_weight()).sum();
    assert_eq!(sum, 4);
    assert_eq!(counts(&layout), [4, 1, 0, 0]);
    assert_eq!(type_at(&layout, &f.topo, f.mid), VertexType::End, "hard request kept");
}

#[test]
fn infeasible_requests_name_the_request_and_the_constraints() {
    let f = split_brick();
    let face = f.ids.faces[2];
    let ids = [ConstraintId(7), ConstraintId(9)];
    // A hard Corner is not allowed by Map.
    let origin = f.ids.vertices[0];
    let reqs = [(key_at(&f.topo, face, origin), Firm::hard(VertexType::Corner))];
    let err = AnglePicker.pick(&f.topo, face, VertexRule::Map, &reqs, &ids).expect_err("hard corner");
    match err {
        SchemeError::Infeasible(i) => {
            assert!(i.reason.contains("Corner") && i.reason.contains(&origin.to_string()), "{}", i.reason);
            assert_eq!(i.constraints, ids.to_vec());
            assert_eq!(i.entities, vec![face]);
        }
        other => panic!("{other:?}"),
    }
    // Five hard Ends cannot make a Map's four.
    let all: Vec<_> = geom::corners(&f.topo, face).expect("corners").into_iter().map(|c| (c.key, Firm::hard(VertexType::End))).collect();
    let err = AnglePicker.pick(&f.topo, face, VertexRule::Map, &all, &ids).expect_err("five ends");
    match err {
        SchemeError::Infeasible(i) => {
            assert!(i.reason.contains("hard End"), "{}", i.reason);
            assert_eq!(i.constraints, ids.to_vec());
        }
        other => panic!("{other:?}"),
    }
    // Submap with all corners pinned to a sum of 5.
    let err = AnglePicker.pick(&f.topo, face, VertexRule::Submap, &all, &ids).expect_err("sum 5");
    assert!(matches!(err, SchemeError::Infeasible(_)));
}

#[test]
fn soft_requests_beat_default_choices() {
    let f = split_brick();
    let face = f.ids.faces[2];
    // A soft End on the mid vertex: Map still needs four Ends, so one other End gives way; the
    // soft request is kept because changing it costs more.
    let reqs = [(key_at(&f.topo, face, f.mid), Firm::soft(VertexType::End))];
    let layout = AnglePicker.pick(&f.topo, face, VertexRule::Map, &reqs, &[]).expect("map");
    assert_eq!(type_at(&layout, &f.topo, f.mid), VertexType::End);
    assert_eq!(counts(&layout), [4, 1, 0, 0]);
}

#[test]
fn picker_is_deterministic() {
    let f = l_face();
    let a = AnglePicker.pick(&f.topo, f.face, VertexRule::Map, &[], &[]).expect("a");
    let b = AnglePicker.pick(&f.topo, f.face, VertexRule::Map, &[], &[]).expect("b");
    assert_eq!(a, b);
}

#[test]
fn two_hard_requests_with_different_types_on_one_corner_conflict() {
    let f = split_brick();
    let face = f.ids.faces[2];
    let ids = [ConstraintId(3), ConstraintId(4)];
    let origin = f.ids.vertices[0];
    let k = key_at(&f.topo, face, origin);
    let reqs = [(k, Firm::hard(VertexType::Side)), (k, Firm::hard(VertexType::End))];
    for rule in [VertexRule::Map, VertexRule::Free] {
        match AnglePicker.pick(&f.topo, face, rule, &reqs, &ids).expect_err("conflict") {
            SchemeError::Infeasible(i) => {
                assert!(i.reason.contains("conflicting") && i.reason.contains("Side") && i.reason.contains("End"), "{}", i.reason);
                assert!(i.reason.contains(&origin.to_string()), "{}", i.reason);
                assert_eq!(i.constraints, ids.to_vec());
            }
            other => panic!("{other:?}"),
        }
    }
    // The same type twice is not a conflict.
    let same = [(k, Firm::hard(VertexType::Side)), (k, Firm::hard(VertexType::Side))];
    AnglePicker.pick(&f.topo, face, VertexRule::Map, &same, &ids).expect("same type is fine");
}
