//! 2D loop fixtures: areas, orientation, vertex types, turning numbers.

use core::f64::consts::{PI, TAU};
use sm_testkit::loops2d::*;

#[test]
fn closed_form_area_equals_shoelace_for_every_region() {
    for r in all_regions() {
        assert!((r.area - r.shoelace_area()).abs() < 1e-12, "{}: {} vs {}", r.name, r.area, r.shoelace_area());
    }
}

#[test]
fn outer_loops_are_counter_clockwise_and_holes_clockwise() {
    for r in all_regions() {
        assert!(signed_area(&r.outer) > 0.0, "{} outer", r.name);
        for h in &r.holes {
            assert!(signed_area(h) < 0.0, "{} hole", r.name);
        }
    }
}

#[test]
fn known_areas() {
    assert_eq!(square().area, 1.0);
    assert_eq!(rectangle(3.0, 2.0).area, 6.0);
    assert_eq!(rectangle_with_holes().area, 50.0);
    assert_eq!(l_shape().area, 3.0);
    assert_eq!(five_sided_submap().area, 8.0);
    // The discretized circle is smaller than the true disk, by the exact n-gon deficit.
    let c = circle(1.0, 64);
    assert!((c.area - 32.0 * (TAU / 64.0).sin()).abs() < 1e-14);
    assert!(c.area < PI && PI - c.area < 0.006);
    let a = annulus(1.0, 2.0, 48);
    assert!((a.area - (ngon_area(48, 2.0) - ngon_area(48, 1.0))).abs() < 1e-14);
    assert!((a.area - 3.0 * PI).abs() < 0.03);
}

#[test]
fn vertex_types_match_the_documented_ones() {
    for r in all_regions() {
        if let Some(expected) = &r.outer_types {
            assert_eq!(&vertex_types(&r.outer), expected, "{}", r.name);
            assert_eq!(expected.len(), r.outer.len());
        } else {
            assert!(r.name == "circle" || r.name == "annulus");
        }
    }
}

#[test]
fn turning_numbers_of_right_angled_loops() {
    for r in [square(), rectangle(3.0, 2.0), l_shape(), five_sided_submap(), rectangle_with_holes()] {
        let t: i32 = vertex_types(&r.outer).iter().map(|t| t.weight()).sum();
        assert_eq!(t, 4, "{} outer", r.name);
        for h in &r.holes {
            let t: i32 = vertex_types(h).iter().map(|t| t.weight()).sum();
            assert_eq!(t, -4, "{} hole", r.name);
            // Seen from the region the hole corners are 270 degrees.
            assert!(vertex_types(h).iter().all(|t| *t == VertexType::Corner));
        }
    }
}

#[test]
fn smooth_loops_have_side_vertices_only() {
    for r in [circle(1.0, 32), annulus(1.0, 2.0, 32)] {
        assert!(vertex_types(&r.outer).iter().all(|t| *t == VertexType::Side), "{}", r.name);
    }
}

#[test]
fn interior_angles_sum_to_the_polygon_angle_sum() {
    for r in all_regions() {
        let n = r.outer.len() as f64;
        let s: f64 = interior_angles(&r.outer).iter().sum();
        assert!((s - (n - 2.0) * PI).abs() < 1e-9, "{}", r.name);
    }
}

#[test]
fn point_in_region() {
    let r = rectangle_with_holes();
    assert!(r.contains((1.0, 1.0)));
    assert!(!r.contains((3.0, 3.0)), "inside the first hole");
    assert!(!r.contains((11.0, 3.0)));
    assert!(l_shape().contains((0.5, 1.5)) && !l_shape().contains((1.5, 1.5)));
    assert_eq!(five_sided_submap().vertex_count(), 5);
    assert!((perimeter(&square().outer) - 4.0).abs() < 1e-15);
}
