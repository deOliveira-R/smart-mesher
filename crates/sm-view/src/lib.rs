//! View-data buffers for the web viewer (WebGL2 client lives in `web/`, Q8).
//!
//! Skeleton: the VIEW task adds mesh skin, cell-based clip and per-cell scalar overlays.
#![forbid(unsafe_code)]

#[cfg(test)]
mod tests {
    /// Placeholder for the VIEW task's determinism test (run twice, 1 vs N threads, compare hashes).
    #[test]
    fn determinism_placeholder() {
        assert_eq!(1 + 1, 2);
    }
}
