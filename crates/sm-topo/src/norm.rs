//! Small numeric helpers for the operators. Row normalization is `sm_topo_api::normalize_row`.

/// `a < b`, false if either is NaN (used by operators).
pub(crate) fn lt(a: f64, b: f64) -> bool {
    a.partial_cmp(&b) == Some(core::cmp::Ordering::Less)
}
