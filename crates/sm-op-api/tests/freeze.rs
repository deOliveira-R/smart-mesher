//! W0g freeze: constructors of the non-exhaustive records and golden serde shapes.

use serde_json::json;
use sm_op_api::*;

#[test]
fn meta_new_and_golden_json() {
    let m = Meta::new("a", "2026-01-01T00:00:00Z", "msg");
    assert_eq!((m.author.as_str(), m.time.as_str(), m.message.as_str()), ("a", "2026-01-01T00:00:00Z", "msg"));
    let v = serde_json::to_value(&m).unwrap();
    assert_eq!(v, json!({"author": "a", "time": "2026-01-01T00:00:00Z", "message": "msg"}));
    let back: Meta = serde_json::from_value(v).unwrap();
    assert_eq!(back, m);
    // Still strict: unknown keys are rejected until a format key exists (DESIGN.md rule 2).
    assert!(serde_json::from_value::<Meta>(json!({"author": "", "time": "", "message": "", "x": 1})).is_err());
}

#[test]
fn check_new_and_golden_json() {
    let h = Hash256::of(b"state");
    let c = Check::new(h);
    assert_eq!(c.state, h);
    assert!(c.layers.is_empty() && c.versions.is_empty());
    let v = serde_json::to_value(&c).unwrap();
    // Empty maps are elided, so a bare check is just the state hash.
    assert_eq!(v, json!({"state": serde_json::to_value(h).unwrap()}));
    let back: Check = serde_json::from_value(v).unwrap();
    assert_eq!(back, c);
}

#[test]
fn rev_shape_is_externally_tagged_lowercase() {
    assert_eq!(serde_json::to_value(Rev::Empty).unwrap(), json!("empty"));
    assert_eq!(serde_json::to_value(Rev::Ref("main".into())).unwrap(), json!({"ref": "main"}));
    let h = Hash256::of(b"n");
    assert_eq!(serde_json::to_value(Rev::Node(h)).unwrap(), json!({"node": serde_json::to_value(h).unwrap()}));
}

#[test]
fn node_hash_depends_only_on_the_three_meta_fields() {
    let body = |m: Meta| NodeBody { parent: None, op: OpRecord::new("model.create_brick", 1, json!({"x": 1.0})).unwrap(), meta: m };
    let a = body(Meta::new("u", "t", "m")).hash().unwrap();
    let b = body(Meta::new("u", "t", "m")).hash().unwrap();
    assert_eq!(a, b);
    assert_ne!(a, body(Meta::new("u", "t", "other")).hash().unwrap());
}
