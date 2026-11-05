//! Python bindings for smart-mesh (package `smartmesh`, abi3 >= 3.10, Q7).
//!
//! | Module | What |
//! |---|---|
//! | [`core`] | [`Host`](core::Host): a `Mutex<Server>` driven through JSON-RPC text, and [`MeshTables`](core::MeshTables), the `.smsh`-named table snapshot of a mesh |
//! | [`errors`] | error code to Python exception class |
//! | `py` (feature `python`) | the `smartmesh._native` extension module |
//!
//! The typed `Session` API, ids, results, the `Mesh` wrapper and the `.smsh` reader are pure
//! Python in `python/smartmesh`. Without the `python` feature this crate is a plain Rust library.
//! The only `unsafe` is what the PyO3 macros expand to, under one `#[allow(unsafe_code)]`.
#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod core;
pub mod errors;
#[cfg(feature = "python")]
mod py;

/// Compile-time check that the session host can sit behind a `Mutex` (needs `Send`).
const _: fn() = || {
    fn assert_send<T: Send>() {}
    assert_send::<sm_server::Server>();
};
