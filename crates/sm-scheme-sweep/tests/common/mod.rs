//! Shared test helpers: registries built from individual stubs plus the real sweep.
#![allow(dead_code)]

use sm_scheme_api::SchemeRegistry;
use sm_scheme_api::stubs::{
    BadReq, Boom, Equal, Lift, MSweep, Map, MapVol, Noop, Pave, PolyVol, Submap, TetVol, Tri,
};

/// Every stub except the stub `Sweep`, plus the real `"sweep"` scheme. Never call
/// `stubs::registry()` together with `sm_scheme_sweep::register` (duplicate name).
pub fn registry() -> SchemeRegistry {
    let mut r = SchemeRegistry::new();
    r.register(Box::new(Equal)).expect("equal");
    r.register(Box::new(Map)).expect("map");
    r.register(Box::new(Submap)).expect("submap");
    r.register(Box::new(Pave)).expect("pave");
    r.register(Box::new(Tri)).expect("tri");
    sm_scheme_sweep::register(&mut r).expect("sweep");
    r.register(Box::new(MSweep)).expect("msweep");
    r.register(Box::new(MapVol)).expect("mapvol");
    r.register(Box::new(TetVol)).expect("tetvol");
    r.register(Box::new(PolyVol)).expect("polyvol");
    r.register(Box::new(BadReq)).expect("badreq");
    r.register_transform(Box::new(Noop)).expect("noop");
    r.register_transform(Box::new(Lift)).expect("lift");
    r.register_transform(Box::new(Boom)).expect("boom");
    r
}
