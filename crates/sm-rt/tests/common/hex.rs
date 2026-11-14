//! Hexagonal-prism CSG pins for the hex lattice tests.
#![allow(dead_code)]

use super::csg::*;
use sm_core::Aabb;
use sm_geom_api::{Frame, HalfSpace};
use sm_rt_api::ComplementPolicy;

/// `sqrt(3)`.
pub const S3: f64 = 1.732_050_807_568_877_2;

/// Outward unit normals of the six flats: angle `60 k` degrees for rows along x (pointy-top),
/// `30 + 60 k` for rows along y (flat-top).
pub fn flats(x_rows: bool) -> [[f64; 2]; 6] {
    let base = if x_rows { 0.0f64 } else { 30.0 };
    std::array::from_fn(|k| {
        let a = (base + 60.0 * k as f64).to_radians();
        [a.cos(), a.sin()]
    })
}

/// A pin in a hexagonal prism of flat-to-flat `pitch`, z in `[-zh, zh]`: cell 0 `{name}fuel` (the
/// cylinder of `radius`) and cell 1 `{name}mod` (the rest of the prism). The leaf bounds are the
/// box circumscribing the prism.
pub fn hex_pin(x_rows: bool, pitch: f64, radius: f64, zh: f64, name: &str) -> Fx {
    let mut fx = Fx::new(10.0);
    fx.complement = ComplementPolicy::Outside;
    let hb = if x_rows { [0.5 * pitch, pitch / S3] } else { [pitch / S3, 0.5 * pitch] };
    fx.bounds = Some(Aabb { lo: p3(-hb[0], -hb[1], -zh), hi: p3(hb[0], hb[1], zh) });
    let cyl = fx.cylinder(Frame::at(p3(0.0, 0.0, 0.0)), radius);
    let mut sides: Vec<HalfSpace> = flats(x_rows)
        .iter()
        .map(|n| {
            let s = fx.plane(p3(0.5 * pitch * n[0], 0.5 * pitch * n[1], 0.0), v3(n[0], n[1], 0.0));
            hs(s, true)
        })
        .collect();
    let zlo = fx.plane(p3(0.0, 0.0, -zh), v3(0.0, 0.0, -1.0));
    let zhi = fx.plane(p3(0.0, 0.0, zh), v3(0.0, 0.0, 1.0));
    sides.push(hs(zlo, true));
    sides.push(hs(zhi, true));
    let with = |extra: HalfSpace| {
        let mut v = sides.clone();
        v.push(extra);
        v
    };
    fx.cell1(&format!("{name}fuel"), &with(hs(cyl, true)));
    fx.cell1(&format!("{name}mod"), &with(hs(cyl, false)));
    fx
}
