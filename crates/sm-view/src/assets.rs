//! The built web client, embedded for the server to serve.

/// One static file of the web client.
#[derive(Clone, Copy, Debug)]
pub struct Asset {
    /// Path relative to the site root (`index.html`).
    pub path: &'static str,
    /// Media type, with charset for text.
    pub mime: &'static str,
    /// File contents.
    pub bytes: &'static [u8],
}

static ASSETS: [Asset; 3] = [
    Asset { path: "index.html", mime: "text/html; charset=utf-8", bytes: include_bytes!("../web/dist/index.html") },
    Asset { path: "viewer.js", mime: "text/javascript; charset=utf-8", bytes: include_bytes!("../web/dist/viewer.js") },
    Asset { path: "viewer.js.map", mime: "application/json", bytes: include_bytes!("../web/dist/viewer.js.map") },
];

/// The embedded client bundle: `index.html`, `viewer.js`, `viewer.js.map`, in that order.
pub fn assets() -> &'static [Asset] {
    &ASSETS
}
