//! Loss reports: the shared type and the VTU report.

mod common;

use common::*;
use sm_meshio::{LossItem, LossReport, LossSeverity, VtuOptions, vtu_loss_report};

#[test]
fn report_stays_sorted_merges_and_serializes() {
    let mut r = LossReport::new();
    assert!(r.is_lossless());
    r.push(LossItem::new("b.code", LossSeverity::Info, 1, "z"));
    r.push(LossItem::new("a.code", LossSeverity::Degraded, 2, "y"));
    r.push(LossItem::new("b.code", LossSeverity::Info, 1, "a"));
    r.push(LossItem::new("a.code", LossSeverity::Dropped, 3, "y"));
    assert!(!r.is_lossless());
    let keys: Vec<(&str, &str)> = r.items.iter().map(|i| (i.code.as_str(), i.detail.as_str())).collect();
    assert_eq!(keys, [("a.code", "y"), ("b.code", "a"), ("b.code", "z")]);
    let a = r.get("a.code").unwrap();
    assert_eq!((a.count, a.severity), (5, LossSeverity::Dropped));
    assert!(r.get("c.code").is_none());
    let json = serde_json::to_string(&r).unwrap();
    assert!(json.contains("\"severity\":\"dropped\""));
    assert_eq!(serde_json::from_str::<LossReport>(&json).unwrap(), r);
}

#[test]
fn vtu_report_for_a_high_order_mesh() {
    let m = ho_hex();
    let r = vtu_loss_report(&m, &VtuOptions::default());
    assert_eq!(r.get("vtu.high_order_dropped").unwrap().count, 1);
    assert_eq!(r.get("vtu.extra_points_unused").unwrap().count, 19);
    assert!(r.get("vtu.edge_table_dropped").is_some_and(|i| i.count == 12));
    assert!(r.get("vtu.face_sets_dropped").is_none());
}

#[test]
fn vtu_report_for_face_sets_and_edge_tables() {
    let m = decorated_block();
    let r = vtu_loss_report(&m, &VtuOptions::default());
    let f = r.get("vtu.face_sets_dropped").unwrap();
    // Faces and Sides sets, in set order; the cell and node sets are written.
    assert_eq!((f.count, f.detail.as_str()), (2, "wall,wall_sides"));
    assert!(r.get("vtu.edge_table_dropped").is_some());
    assert!(r.get("vtu.high_order_dropped").is_none());
    assert!(r.get("vtu.extra_points_unused").is_none());
}

#[test]
fn vtu_report_is_empty_for_plain_meshes() {
    for (name, m) in corpus() {
        if ["ho_hex", "decorated_block"].contains(&name.as_str()) {
            continue;
        }
        let r = vtu_loss_report(&m, &VtuOptions::default());
        assert!(r.is_lossless(), "{name}: {:?}", r.items);
    }
}
