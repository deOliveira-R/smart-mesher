//! OpenMC `<surface>` types to IR surfaces.
//!
//! Sense: every OpenMC surface is a function `f` with the negative side `f < 0`, the IR's
//! `Sign::Negative`. Cones and quadrics are kept as written (a double cone, `f` as given);
//! circular tori need `A > B == C`; the other tori are refused.

use super::{Ctx, invalid, unsupported};
use crate::ImportError;
use crate::ir::Surf;
use sm_core::{Point3, Vector3};
use sm_geom_api::{Cylinder, Frame, ImplicitDef, Quadric, SurfaceDef, Torus};

fn axis_index(c: char) -> usize {
    match c {
        'x' => 0,
        'y' => 1,
        _ => 2,
    }
}

fn dir(v: [f64; 3], cx: &Ctx, what: &str) -> Result<Vector3, ImportError> {
    let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if !(n.is_finite() && n > 0.0) {
        return Err(invalid(cx, format!("{what} has a zero direction")));
    }
    Ok(Vector3::new(v[0] / n, v[1] / n, v[2] / n))
}

fn axis_vec(axis: usize) -> Vector3 {
    let mut v = [0.0; 3];
    v[axis] = 1.0;
    Vector3::new(v[0], v[1], v[2])
}

fn frame(cx: &Ctx, o: [f64; 3], z: Vector3) -> Result<Frame, ImportError> {
    Frame::from_z(Point3::new(o[0], o[1], o[2]), z).map_err(|e| invalid(cx, format!("invalid surface: {e}")))
}

/// Build the surface of `ty` with coefficients `c` (already counted).
pub(super) fn build(cx: &Ctx, ty: &str, c: &[f64]) -> Result<Surf, ImportError> {
    let want = |n: usize| -> Result<(), ImportError> {
        if c.len() == n { Ok(()) } else { Err(invalid(cx, format!("surface type {ty:?} takes {n} coefficients, found {}", c.len()))) }
    };
    let radius = |r: f64| -> Result<f64, ImportError> {
        if r.is_finite() && r > 0.0 { Ok(r) } else { Err(invalid(cx, format!("radius {r} of a {ty} is not positive"))) }
    };
    let wrap = |r: Result<Surf, ImportError>| r.map_err(|e| match e {
        ImportError::Invalid(m) => invalid(cx, m),
        other => other,
    });
    match ty {
        "x-plane" | "y-plane" | "z-plane" => {
            want(1)?;
            Ok(Surf::axis_plane(axis_index(ty.chars().next().unwrap_or('z')), c[0]))
        }
        "plane" => {
            want(4)?;
            // f = A x + B y + C z - D; the point nearest the origin is n D / |n|^2.
            let n2 = c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
            if !(n2.is_finite() && n2 > 0.0) {
                return Err(invalid(cx, "a plane has a zero normal".into()));
            }
            let k = c[3] / n2;
            wrap(Surf::plane([c[0] * k, c[1] * k, c[2] * k], [c[0], c[1], c[2]]))
        }
        "x-cylinder" | "y-cylinder" | "z-cylinder" => {
            want(3)?;
            let ax = axis_index(ty.chars().next().unwrap_or('z'));
            // coefficients: the two transverse centre coordinates in axis order, then r.
            let mut o = [0.0; 3];
            let mut it = (0..3).filter(|i| *i != ax);
            o[it.next().unwrap_or(0)] = c[0];
            o[it.next().unwrap_or(0)] = c[1];
            wrap(Surf::cylinder(ax, o, radius(c[2])?))
        }
        "cylinder" => {
            want(7)?;
            let f = frame(cx, [c[0], c[1], c[2]], dir([c[4], c[5], c[6]], cx, "a cylinder")?)?;
            Cylinder::new(f, radius(c[3])?).map(|d| Surf::new(SurfaceDef::Cylinder(d))).map_err(|e| invalid(cx, format!("invalid surface: {e}")))
        }
        "sphere" => {
            want(4)?;
            wrap(Surf::sphere([c[0], c[1], c[2]], radius(c[3])?))
        }
        "x-cone" | "y-cone" | "z-cone" => {
            want(4)?;
            let f = frame(cx, [c[0], c[1], c[2]], axis_vec(axis_index(ty.chars().next().unwrap_or('z'))))?;
            cone(cx, &f, c[3])
        }
        "cone" => {
            want(7)?;
            let f = frame(cx, [c[0], c[1], c[2]], dir([c[4], c[5], c[6]], cx, "a cone")?)?;
            cone(cx, &f, c[3])
        }
        "quadric" => {
            want(10)?;
            if c.iter().any(|v| !v.is_finite()) {
                return Err(invalid(cx, "quadric coefficients are not finite".into()));
            }
            let mut a = [0.0; 10];
            a.copy_from_slice(c);
            Ok(Surf::new(SurfaceDef::Implicit(ImplicitDef::Quadric(Quadric { coeffs: a }))))
        }
        "x-torus" | "y-torus" | "z-torus" => {
            want(6)?;
            let (a, b, cc) = (c[3], c[4], c[5]);
            if b != cc {
                return Err(unsupported(cx, format!("{ty} with unequal semi-axes B = {b}, C = {cc} (an elliptic torus; only circular tori are supported)")));
            }
            if !(b > 0.0 && a > b && a.is_finite()) {
                return Err(unsupported(cx, format!("{ty} with A = {a}, B = {b}: only ring tori (A > B > 0) are supported")));
            }
            let f = frame(cx, [c[0], c[1], c[2]], axis_vec(axis_index(ty.chars().next().unwrap_or('z'))))?;
            Torus::new(f, a, b).map(|d| Surf::new(SurfaceDef::Torus(d))).map_err(|e| invalid(cx, format!("invalid surface: {e}")))
        }
        other => Err(unsupported(cx, format!("surface type {other:?}"))),
    }
}

/// `f = l_x^2 + l_y^2 - r2 l_z^2` in the frame whose `z` is the axis: both nappes, negative inside.
fn cone(cx: &Ctx, f: &Frame, r2: f64) -> Result<Surf, ImportError> {
    if !(r2.is_finite() && r2 > 0.0) {
        return Err(invalid(cx, format!("cone R^2 = {r2} is not positive")));
    }
    let q = Quadric::from_local(f, [1.0, 1.0, -r2], [0.0; 3], 0.0);
    Ok(Surf::new(SurfaceDef::Implicit(ImplicitDef::Quadric(q))))
}
