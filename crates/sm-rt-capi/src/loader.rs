//! Open a tracking file for `smrt_open`: plain safe Rust over `sm-rt`.

use crate::status::Fail;
use sm_rt_api::Tracker;
use std::sync::Arc;

/// Open the `.smrt` image at `path` (written by `session.export` format `smrt` or by
/// `sm_rt::write_smrt_file`) and restore the tracker from it through `sm_rt::load_image`.
///
/// A file that cannot be read is `SMRT_ERR_BAD_ARG` (the message names the file); bytes that are
/// not a valid image are `SMRT_ERR_CACHE_CORRUPT` or `SMRT_ERR_CACHE_STALE`. The image is fully
/// validated by the loader.
#[cfg(feature = "sm-rt")]
pub fn open_smrt(path: &std::path::Path) -> Result<Arc<dyn Tracker>, Fail> {
    let bytes = std::fs::read(path).map_err(|e| Fail::bad(format!("smrt_open: cannot read {}: {e}", path.display())))?;
    let image = sm_rt::read_smrt(&bytes)?;
    Ok(Arc::new(sm_rt::load_image(&image)?))
}

/// Without the `sm-rt` cargo feature no loader is linked: `SMRT_ERR_UNSUPPORTED`.
#[cfg(not(feature = "sm-rt"))]
pub fn open_smrt(_path: &std::path::Path) -> Result<Arc<dyn Tracker>, Fail> {
    Err(Fail::Rt(sm_rt_api::RtError::Unsupported("smrt_open: no tracker backend linked (build sm-rt-capi with the `sm-rt` feature)")))
}
