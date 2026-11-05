//! Python bindings for smart-mesh (package `smartmesh`, abi3 >= 3.10, Q7).
//!
//! Skeleton: the PY task fills in `Host`, the typed `Session` API and the mesh tables.
//! Without the `python` feature this crate is an empty rlib.
#![deny(unsafe_code)]

/// Compile-time check that the session host can sit behind a `Mutex` (needs `Send`).
const _: fn() = || {
    fn assert_send<T: Send>() {}
    assert_send::<sm_server::Server>();
};

#[cfg(feature = "python")]
mod module {
    use pyo3::prelude::*;

    /// Version string of the native extension.
    #[pyfunction]
    fn native_version() -> &'static str {
        env!("CARGO_PKG_VERSION")
    }

    /// Entry point of the `smartmesh` extension module.
    // The PyO3 macro expands to `unsafe` FFI glue (CLAUDE.md exception).
    #[allow(unsafe_code)]
    #[pymodule]
    fn smartmesh(m: &Bound<'_, PyModule>) -> PyResult<()> {
        m.add_function(wrap_pyfunction!(native_version, m)?)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    /// Placeholder for the PY task's determinism test (run twice, 1 vs N threads, compare hashes).
    #[test]
    fn determinism_placeholder() {
        assert_eq!(1 + 1, 2);
    }
}
