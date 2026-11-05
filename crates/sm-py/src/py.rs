//! PyO3 glue (`smartmesh._native`). Everything is inside one declarative `#[pymodule]` so the
//! macro-generated FFI `unsafe` sits under the single `#[allow(unsafe_code)]` (CLAUDE.md, Q7).
//! Each entry point releases the GIL and runs through [`run`], which turns a panic into an
//! internal error instead of PyO3's `PanicException` (a `BaseException`).

use pyo3::prelude::*;

// The PyO3 macros expand to `unsafe` FFI glue (CLAUDE.md exception); no hand-written `unsafe`.
#[allow(unsafe_code)]
#[pymodule]
pub mod _native {
    use std::any::Any;
    use std::panic::{AssertUnwindSafe, catch_unwind};
    use std::path::PathBuf;

    use numpy::PyArray1;
    use pyo3::exceptions::PyException;
    use pyo3::prelude::*;
    use pyo3::types::PyDict;
    use sm_core::{EntityId, EntityKind};
    use sm_protocol::{METHODS, RpcError};

    use crate::core::{Column, Host, HostError};
    use crate::errors;

    pyo3::create_exception!(
        _native,
        NativeError,
        PyException,
        "Native failure; args are `(code, kind, message, data_json)`."
    );

    #[pymodule_init]
    fn init(m: &Bound<'_, PyModule>) -> PyResult<()> {
        m.add("NativeError", m.py().get_type::<NativeError>())
    }

    fn native_err(e: &RpcError) -> PyErr {
        let data = if e.data.is_null() { String::new() } else { e.data.to_string() };
        NativeError::new_err((e.code, e.kind.clone(), e.message.clone(), data))
    }

    fn panic_message(p: &(dyn Any + Send)) -> String {
        let text = p.downcast_ref::<&str>().map(|s| (*s).to_owned()).or_else(|| p.downcast_ref::<String>().cloned());
        format!("native panic: {}", text.unwrap_or_else(|| "unknown".into()))
    }

    /// Run `f` without the GIL; a panic or a protocol error becomes a [`NativeError`].
    fn run<T: Send>(py: Python<'_>, f: impl FnOnce() -> Result<T, RpcError> + Send) -> PyResult<T> {
        match py.detach(|| catch_unwind(AssertUnwindSafe(f))) {
            Ok(Ok(v)) => Ok(v),
            Ok(Err(e)) => Err(native_err(&e)),
            Err(p) => Err(native_err(&RpcError::internal(panic_message(p.as_ref())))),
        }
    }

    fn host_err(e: HostError) -> RpcError {
        e.to_rpc()
    }

    /// Version string of the native extension.
    #[pyfunction]
    fn native_version() -> &'static str {
        env!("CARGO_PKG_VERSION")
    }

    /// Protocol method names (sorted).
    #[pyfunction]
    fn methods() -> Vec<&'static str> {
        METHODS.to_vec()
    }

    /// Name of the Python exception class for an error code and kind.
    #[pyfunction]
    fn error_class(code: i64, kind: &str) -> &'static str {
        errors::class_name(code, kind)
    }

    /// The protocol error codes by constant name.
    #[pyfunction]
    fn protocol_codes() -> Vec<(&'static str, i64)> {
        use sm_protocol::{APPLICATION_ERROR, INTERNAL_ERROR, INVALID_PARAMS, INVALID_REQUEST, METHOD_NOT_FOUND, PARSE_ERROR};
        vec![
            ("PARSE_ERROR", PARSE_ERROR),
            ("INVALID_REQUEST", INVALID_REQUEST),
            ("METHOD_NOT_FOUND", METHOD_NOT_FOUND),
            ("INVALID_PARAMS", INVALID_PARAMS),
            ("INTERNAL_ERROR", INTERNAL_ERROR),
            ("APPLICATION_ERROR", APPLICATION_ERROR),
        ]
    }

    /// Every entity kind name, in kind-byte order.
    #[pyfunction]
    fn entity_kinds() -> Vec<(&'static str, u8)> {
        EntityKind::ALL.iter().map(|k| (k.name(), *k as u8)).collect()
    }

    /// Parse `Kind#serial` (canonical or command-language spelling): `(kind, serial, raw)`.
    #[pyfunction]
    fn entity_parse(text: &str) -> Option<(String, u64, u64)> {
        let id: EntityId = text.parse().ok()?;
        Some((id.kind().name().to_owned(), id.serial(), id.to_raw()))
    }

    /// Decode a raw id: `(kind, serial)`, `None` for an unknown kind byte.
    #[pyfunction]
    fn entity_from_raw(raw: u64) -> Option<(String, u64)> {
        let id = EntityId::from_raw(raw)?;
        Some((id.kind().name().to_owned(), id.serial()))
    }

    /// Test hook: panic outside the session lock.
    #[cfg(feature = "test-hooks")]
    #[pyfunction]
    fn _panic(py: Python<'_>) -> PyResult<()> {
        run(py, || -> Result<(), RpcError> { panic!("forced test panic") })
    }

    /// A session host (`Mutex<Server>`); thread-safe, every call releases the GIL.
    #[pyclass(name = "Host", frozen, module = "smartmesh._native")]
    pub struct PyHost {
        inner: Host,
    }

    #[pymethods]
    impl PyHost {
        /// In-memory session over the standard distribution.
        #[staticmethod]
        #[pyo3(signature = (author=None, fixed_time=None))]
        fn memory(py: Python<'_>, author: Option<String>, fixed_time: Option<String>) -> PyResult<Self> {
            let inner = run(py, move || Host::memory_with(author, fixed_time).map_err(host_err))?;
            Ok(Self { inner })
        }

        /// Open (or, with `create`, create) a project directory.
        #[staticmethod]
        #[pyo3(signature = (path, create=None))]
        fn open(py: Python<'_>, path: PathBuf, create: Option<String>) -> PyResult<Self> {
            let inner = run(py, move || Host::open(&path, create.as_deref()).map_err(host_err))?;
            Ok(Self { inner })
        }

        /// Call a protocol method with JSON params text; returns the result as JSON text.
        fn call(&self, py: Python<'_>, method: &str, params_json: &str) -> PyResult<String> {
            run(py, || {
                let params: serde_json::Value =
                    serde_json::from_str(params_json).map_err(|e| RpcError::invalid_params(format!("params are not JSON: {e}")))?;
                self.inner.call(method, &params).map(|v| v.to_string())
            })
        }

        /// Stage bytes in the input store: `(sha256, size)`.
        fn stage(&self, py: Python<'_>, data: &[u8]) -> PyResult<(String, u64)> {
            run(py, || self.inner.stage(data).map(|r| (r.sha256.to_string(), r.bytes)).map_err(host_err))
        }

        /// Ids of the meshes in the current state.
        fn mesh_ids(&self, py: Python<'_>) -> PyResult<Vec<String>> {
            run(py, || Ok(self.inner.mesh_ids().map_err(host_err)?.iter().map(ToString::to_string).collect()))
        }

        /// Snapshot of a mesh's tables as `{name: ndarray}` (owned copies); `mesh=None` means the
        /// only mesh.
        #[pyo3(signature = (mesh=None))]
        fn mesh_tables<'py>(&self, py: Python<'py>, mesh: Option<&str>) -> PyResult<Bound<'py, PyDict>> {
            let id = match mesh {
                None => None,
                Some(t) => Some(t.parse::<EntityId>().map_err(|e| native_err(&RpcError::invalid_params(e.to_string())))?),
            };
            let tables = run(py, || self.inner.mesh_tables(id).map_err(host_err))?;
            let out = PyDict::new(py);
            for (name, col) in tables.columns {
                match col {
                    Column::U8(v) => out.set_item(name, PyArray1::from_vec(py, v))?,
                    Column::U32(v) => out.set_item(name, PyArray1::from_vec(py, v))?,
                    Column::U64(v) => out.set_item(name, PyArray1::from_vec(py, v))?,
                    Column::F64(v) => out.set_item(name, PyArray1::from_vec(py, v))?,
                }
            }
            Ok(out)
        }

        /// Test hook: panic while holding the session lock (poisons it).
        #[cfg(feature = "test-hooks")]
        fn _panic_locked(&self, py: Python<'_>) -> PyResult<()> {
            run(py, || {
                let _ = self.inner.with_server(|_| -> () { panic!("forced test panic under the lock") });
                Ok(())
            })
        }
    }
}
