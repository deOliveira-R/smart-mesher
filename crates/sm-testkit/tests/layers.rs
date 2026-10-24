//! The layer snapshot-consistency check accepts honest layers and rejects a lying one.

use sm_op_api::{CodecError, Digest, LayerType};
use sm_testkit::check::snapshot_support_consistent;

#[derive(Clone, Default)]
struct Honest(Vec<u8>);
impl LayerType for Honest {
    const NAME: &'static str = "honest";
    fn digest(&self, d: &mut Digest) {
        d.u64(self.0.len() as u64);
    }
    fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
    fn supports_snapshot(&self) -> bool {
        true
    }
    fn snapshot(&self) -> Result<Option<Vec<u8>>, CodecError> {
        Ok(Some(self.0.clone()))
    }
}

#[derive(Clone, Default)]
struct Plain(Vec<u8>);
impl LayerType for Plain {
    const NAME: &'static str = "plain";
    fn digest(&self, d: &mut Digest) {
        d.u64(self.0.len() as u64);
    }
    fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

#[derive(Clone, Default)]
struct Liar(Vec<u8>);
impl LayerType for Liar {
    const NAME: &'static str = "liar";
    fn digest(&self, d: &mut Digest) {
        d.u64(self.0.len() as u64);
    }
    fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
    fn supports_snapshot(&self) -> bool {
        true
    }
}

#[test]
fn snapshot_support_is_checked() {
    assert!(snapshot_support_consistent(&Honest(vec![1])).is_ok());
    assert!(snapshot_support_consistent(&Plain(vec![1])).is_ok());
    assert!(snapshot_support_consistent(&Liar(Vec::new())).is_ok(), "empty layers are exempt");
    let e = snapshot_support_consistent(&Liar(vec![1])).unwrap_err();
    assert_eq!(e.check, "snapshot_support_consistent");
}
