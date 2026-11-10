//! Quadric classifier: an OpenMC `quadric` (10 coefficients) to exact parametric surfaces.
//!
//! The 3x3 quadratic part is diagonalised with cyclic Jacobi sweeps (pure arithmetic, fixed
//! iteration order, so deterministic). The linear part is then absorbed by completing squares.
//! Supported: plane, sphere, circular and elliptic cylinder, circular and elliptic cone. Ellipsoids
//! (including spheroids), paraboloids, hyperboloids and degenerate sets are refused with
//! [`ClassifyError::Unsupported`], which names the deck surface and prints the eigenvalues.
//!
//! Zero and equal eigenvalues are decided with a relative tolerance ([`DEFAULT_TOL`]), so a
//! cylinder written with round-off still classifies as a cylinder, not an elliptic one.

use sm_core::{Point3, Vector3};
use sm_geom_api::{Cone, Cylinder, EllipticCone, EllipticCylinder, Frame, GeomError, Plane, Quadric, Sphere, SurfaceDef};

/// Default relative tolerance for zero and equal eigenvalues and for the constant term.
pub const DEFAULT_TOL: f64 = 1e-9;

/// Where a quadric came from in the deck, for error messages.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DeckSurface {
    /// The deck surface id.
    pub id: u32,
    /// The 1-based deck line, when known.
    pub line: Option<u32>,
}

/// Why a quadric was not classified.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum ClassifyError {
    /// A valid quadric of a kind that is not built (ellipsoid, paraboloid, hyperboloid, ...).
    #[error("unsupported quadric, surface {} {}: {what}", .surface.id, line_note(.surface))]
    Unsupported {
        /// The deck surface.
        surface: DeckSurface,
        /// What it is, with the eigenvalues.
        what: String,
    },
    /// The coefficients are not finite, or a surface constructor refused the result.
    #[error("invalid quadric, surface {} {}: {what}", .surface.id, line_note(.surface))]
    Invalid {
        /// The deck surface.
        surface: DeckSurface,
        /// What is wrong.
        what: String,
    },
}

fn line_note(s: &DeckSurface) -> String {
    s.line.map(|l| format!("(line {l})")).unwrap_or_default()
}

/// The result of classifying a quadric.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct Classified {
    /// One surface, or two for a cone: the two nappes (`[0]` widens toward its `+z`, `[1]` is the
    /// mirror image through the apex).
    pub surfaces: Vec<SurfaceDef>,
    /// `+1.0` when the written `f` has the sign of the canonical implicit form of `surfaces[0]`
    /// (positive outside for sphere, cylinders and cones, positive along the normal for a plane),
    /// `-1.0` when it is the opposite. So `sign(f(p)) == sign * sign(canonical(p))`.
    pub sign: f64,
}

type V = Vector3;

/// Eigen-decomposition of the symmetric matrix `m`: eigenvalues and orthonormal eigenvectors,
/// cyclic Jacobi with a fixed sweep order.
fn jacobi(mut m: [[f64; 3]; 3]) -> ([f64; 3], [V; 3]) {
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]; // eigenvectors as columns
    for _ in 0..60 {
        let off = m[0][1] * m[0][1] + m[0][2] * m[0][2] + m[1][2] * m[1][2];
        let diag = m[0][0] * m[0][0] + m[1][1] * m[1][1] + m[2][2] * m[2][2];
        if off == 0.0 || off <= 1e-34 * diag {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if m[p][q] == 0.0 {
                continue;
            }
            let theta = (m[q][q] - m[p][p]) / (2.0 * m[p][q]);
            let sg = if theta >= 0.0 { 1.0 } else { -1.0 };
            let t = sg / (theta.abs() + (theta * theta + 1.0).sqrt());
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for row in m.iter_mut() {
                let (a, b) = (row[p], row[q]);
                row[p] = c * a - s * b;
                row[q] = s * a + c * b;
            }
            let (rp, rq) = (m[p], m[q]);
            m[p] = std::array::from_fn(|k| c * rp[k] - s * rq[k]);
            m[q] = std::array::from_fn(|k| s * rp[k] + c * rq[k]);
            for row in v.iter_mut() {
                let (a, b) = (row[p], row[q]);
                row[p] = c * a - s * b;
                row[q] = s * a + c * b;
            }
        }
    }
    let ev = |j: usize| V::new(v[0][j], v[1][j], v[2][j]);
    ([m[0][0], m[1][1], m[2][2]], [ev(0), ev(1), ev(2)])
}

/// The unit vector `v` or `-v`, whichever has its largest-magnitude component positive.
fn canon_dir(v: V) -> V {
    let a = [v.x.abs(), v.y.abs(), v.z.abs()];
    let k = if a[0] >= a[1] && a[0] >= a[2] {
        0
    } else if a[1] >= a[2] {
        1
    } else {
        2
    };
    if [v.x, v.y, v.z][k] < 0.0 { -v } else { v }
}

/// Right-handed frame with axis `z` and first axis `x` (both unit, orthogonal).
fn frame(origin: Point3, x: V, z: V) -> Frame {
    Frame { origin, x, y: z.cross(x), z }
}

fn pt(v: V) -> Point3 {
    Point3::new(v.x, v.y, v.z)
}

/// Classify with [`DEFAULT_TOL`]. See [`classify_with_tol`].
pub fn classify(q: &Quadric, at: DeckSurface) -> Result<Classified, ClassifyError> {
    classify_with_tol(q, at, DEFAULT_TOL)
}

/// Classify the quadric `q` (written `f = x^T Q x + l.x + k`) into exact parametric surfaces.
///
/// `tol` is relative: an eigenvalue is zero when `|lambda| <= tol * max|lambda|`, two are equal
/// when they differ by at most `tol` times the larger, and the completed-square constant is zero
/// when it is at most `tol` times the size of its terms. A quadric with all quadratic
/// coefficients exactly zero is a plane (no unit-free test exists for "nearly zero").
pub fn classify_with_tol(q: &Quadric, at: DeckSurface, tol: f64) -> Result<Classified, ClassifyError> {
    if q.coeffs.iter().any(|c| !c.is_finite()) {
        return Err(ClassifyError::Invalid { surface: at, what: "coefficients are not finite".into() });
    }
    let [a, b, c, d, e, f, g, h, j, k] = q.coeffs;
    let lin = V::new(g, h, j);
    let (lam, vecs) = jacobi([[a, d / 2.0, f / 2.0], [d / 2.0, b, e / 2.0], [f / 2.0, e / 2.0, c]]);
    let lmax = lam.iter().fold(0.0_f64, |m, l| m.max(l.abs()));
    let eig = || format!("eigenvalues [{:e}, {:e}, {:e}]", lam[0], lam[1], lam[2]);
    let unsupported = |what: &str| ClassifyError::Unsupported { surface: at, what: format!("{what}; {}", eig()) };
    let geom = |e: GeomError| ClassifyError::Invalid { surface: at, what: e.to_string() };

    if lmax == 0.0 {
        let n = lin.norm();
        if n == 0.0 {
            return Err(unsupported("degenerate quadric (all coefficients but the constant are zero)"));
        }
        let z = lin * (1.0 / n);
        let frame = Frame::from_z(pt(z * (-k / n)), z).map_err(geom)?;
        return Ok(Classified { surfaces: vec![SurfaceDef::Plane(Plane { frame })], sign: 1.0 });
    }

    let is_zero = [0, 1, 2].map(|i| lam[i].abs() <= tol * lmax);
    let nz: Vec<usize> = (0..3).filter(|&i| !is_zero[i]).collect();
    let zr: Vec<usize> = (0..3).filter(|&i| is_zero[i]).collect();
    let lp = [vecs[0].dot(lin), vecs[1].dot(lin), vecs[2].dot(lin)]; // linear part in the eigenbasis
    let equal = |x: f64, y: f64| (x - y).abs() <= tol * x.abs().max(y.abs());

    // Centre (zero along null directions) and the constant after centring.
    let mut centre = V::default();
    let mut kp = k;
    let mut kscale = k.abs();
    for &i in &nz {
        centre = centre + vecs[i] * (-lp[i] / (2.0 * lam[i]));
        let t = lp[i] * lp[i] / (4.0 * lam[i]);
        kp -= t;
        kscale = kscale.max(t.abs());
    }
    let k_zero = kp.abs() <= tol * kscale;
    // Length scale of the centred shape, to judge a linear term along a null direction.
    let lscale = nz.iter().map(|&i| (lp[i] / (2.0 * lam[i])).abs().max((kp / lam[i]).abs().sqrt())).fold(0.0_f64, f64::max);
    for &i in &zr {
        if lp[i].abs() > tol * 2.0 * lmax * lscale {
            return Err(unsupported("paraboloid or parabolic cylinder"));
        }
    }

    match nz.len() {
        3 => {
            let (pos, neg): (Vec<usize>, Vec<usize>) = nz.iter().partition(|&&i| lam[i] > 0.0);
            if k_zero {
                // Cone: two eigenvalues of one sign, one of the other.
                let (pair, axis) = match (pos.len(), neg.len()) {
                    (2, 1) => (pos, neg[0]),
                    (1, 2) => (neg, pos[0]),
                    _ => return Err(unsupported("degenerate quadric (a single point)")),
                };
                let apex = pt(centre);
                let z = canon_dir(vecs[axis]);
                let alpha = |i: usize| (-lam[axis] / lam[i]).sqrt();
                let sign = lam[pair[0]].signum();
                if equal(lam[pair[0]], lam[pair[1]]) {
                    let half = ((alpha(pair[0]) + alpha(pair[1])) / 2.0).atan();
                    let x = vecs[pair[0]];
                    let n0 = Cone::new(frame(apex, x, z), 0.0, half).map_err(geom)?;
                    let n1 = Cone::new(frame(apex, x, -z), 0.0, half).map_err(geom)?;
                    return Ok(Classified { surfaces: vec![SurfaceDef::Cone(n0), SurfaceDef::Cone(n1)], sign });
                }
                // Elliptic: x along the larger half-width at unit height. The section at `v = 0`
                // lies one unit above the apex, so `a = alpha`, `b = beta`, `tan(half) = a`.
                let (i, jx) = if alpha(pair[0]) >= alpha(pair[1]) { (pair[0], pair[1]) } else { (pair[1], pair[0]) };
                let (al, be) = (alpha(i), alpha(jx));
                let half = al.atan();
                let x = vecs[i];
                let mk = |zz: V| EllipticCone::new(frame(pt(centre + zz), x, zz), al, be, half).map_err(geom);
                let surfaces = vec![SurfaceDef::EllipticCone(mk(z)?), SurfaceDef::EllipticCone(mk(-z)?)];
                return Ok(Classified { surfaces, sign });
            }
            if pos.len() == 3 || neg.len() == 3 {
                let sign = if pos.len() == 3 { 1.0 } else { -1.0 };
                if kp * sign > 0.0 {
                    return Err(unsupported("empty quadric (no real points)"));
                }
                if equal(lam[0], lam[1]) && equal(lam[1], lam[2]) {
                    let r = (-kp / lam[0]).sqrt();
                    let s = Sphere::new(Frame::at(pt(centre)), r).map_err(geom)?;
                    return Ok(Classified { surfaces: vec![SurfaceDef::Sphere(s)], sign });
                }
                return Err(unsupported("ellipsoid"));
            }
            Err(unsupported("hyperboloid"))
        }
        2 => {
            let (i, jx) = (nz[0], nz[1]);
            let z = canon_dir(vecs[zr[0]]);
            if k_zero {
                return Err(unsupported("degenerate cylinder (a line or a pair of planes)"));
            }
            if lam[i].signum() != lam[jx].signum() {
                return Err(unsupported("hyperbolic cylinder"));
            }
            let sign = lam[i].signum();
            if kp * sign > 0.0 {
                return Err(unsupported("empty quadric (no real points)"));
            }
            let (ri, rj) = ((-kp / lam[i]).sqrt(), (-kp / lam[jx]).sqrt());
            let origin = pt(centre);
            if equal(lam[i], lam[jx]) {
                let fr = Frame::from_z(origin, z).map_err(geom)?;
                let cy = Cylinder::new(fr, (ri + rj) / 2.0).map_err(geom)?;
                return Ok(Classified { surfaces: vec![SurfaceDef::Cylinder(cy)], sign });
            }
            let (xi, ra, rb) = if ri >= rj { (vecs[i], ri, rj) } else { (vecs[jx], rj, ri) };
            let ec = EllipticCylinder::new(frame(origin, xi, z), ra, rb).map_err(geom)?;
            Ok(Classified { surfaces: vec![SurfaceDef::EllipticCylinder(ec)], sign })
        }
        _ => Err(unsupported(if nz.len() == 1 { "parallel planes, parabolic cylinder or empty set" } else { "degenerate quadric" })),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use sm_geom_api::{ImplicitDef, ImplicitSurface};

    const AT: DeckSurface = DeckSurface { id: 7, line: Some(12) };

    fn frame_of(o: [f64; 3], z: [f64; 3], h: [f64; 3]) -> Option<Frame> {
        let hv = V::new(h[0], h[1], h[2]);
        Frame::from_zx(Point3::new(o[0], o[1], o[2]), V::new(z[0], z[1], z[2]), hv).ok().filter(|f| hv.cross(f.z).norm() > 0.2 * hv.norm())
    }

    fn canon(def: &SurfaceDef) -> Quadric {
        match def.as_surface().and_then(|s| s.implicit()) {
            Some(ImplicitDef::Quadric(q)) => q,
            other => panic!("no quadric form: {other:?}"),
        }
    }

    /// The written `f` and the canonical form agree in sign (up to `sign`) at pseudo-random points.
    fn check_signs(f: &Quadric, c: &Classified, centre: Point3) {
        let canon = canon(&c.surfaces[0]);
        let mut s = 0x1234_5678_9abc_def1_u64;
        let mut next = || {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((s >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * 30.0
        };
        let mut checked = 0;
        for _ in 0..100 {
            let p = Point3::new(centre.x + next(), centre.y + next(), centre.z + next());
            let (vf, vc) = (f.value(p), canon.value(p));
            if vf.abs() < 1e-9 || vc.abs() < 1e-9 {
                continue;
            }
            assert_eq!(vf > 0.0, (vc * c.sign) > 0.0, "sign mismatch at {p:?}: f = {vf}, canonical = {vc}, sign = {}", c.sign);
            checked += 1;
        }
        assert!(checked > 50);
    }

    fn rel(a: f64, b: f64) -> f64 {
        (a - b).abs() / b.abs().max(1.0)
    }

    fn mag(o: Point3) -> f64 {
        1.0 + o.x.abs().max(o.y.abs()).max(o.z.abs())
    }

    fn near_pt(a: Point3, b: Point3) -> f64 {
        (a - b).norm() / mag(b)
    }

    /// Distance from `p` to the line through `o` with unit direction `z`, relative.
    fn off_axis(p: Point3, o: Point3, z: V) -> f64 {
        (p - o).cross(z).norm() / mag(o)
    }

    fn frame_strategy() -> impl Strategy<Value = Frame> {
        (prop::array::uniform3(-5.0..5.0_f64), prop::array::uniform3(-1.0..1.0_f64), prop::array::uniform3(-1.0..1.0_f64))
            .prop_filter_map("frame", |(o, z, h)| (V::new(z[0], z[1], z[2]).norm() > 0.3).then(|| frame_of(o, z, h)).flatten())
    }

    fn scale() -> impl Strategy<Value = f64> {
        (0.1..10.0_f64, any::<bool>()).prop_map(|(s, neg)| if neg { -s } else { s })
    }

    fn scaled(q: Quadric, s: f64) -> Quadric {
        Quadric { coeffs: q.coeffs.map(|c| c * s) }
    }

    proptest! {
        #![proptest_config(sm_testkit::strategies::config(64))]

        #[test]
        fn sphere_round_trip(fr in frame_strategy(), r in 0.1..10.0_f64, s in scale()) {
            let q = scaled(Quadric::sphere(&fr, r), s);
            let c = classify(&q, AT).unwrap();
            let [SurfaceDef::Sphere(sp)] = c.surfaces.as_slice() else { panic!("not a sphere") };
            prop_assert!(rel(sp.radius, r) < 1e-12);
            prop_assert!(near_pt(sp.frame.origin, fr.origin) < 1e-12);
            prop_assert_eq!(c.sign, s.signum());
            check_signs(&q, &c, fr.origin);
        }

        #[test]
        fn cylinder_round_trip(fr in frame_strategy(), r in 0.1..10.0_f64, s in scale()) {
            let q = scaled(Quadric::cylinder(&fr, r), s);
            let c = classify(&q, AT).unwrap();
            let [SurfaceDef::Cylinder(cy)] = c.surfaces.as_slice() else { panic!("not a cylinder") };
            prop_assert!(rel(cy.radius, r) < 1e-12);
            prop_assert!(cy.frame.z.cross(fr.z).norm() < 1e-12);
            prop_assert!(off_axis(cy.frame.origin, fr.origin, fr.z) < 1e-12);
            prop_assert_eq!(c.sign, s.signum());
            check_signs(&q, &c, fr.origin);
        }

        #[test]
        fn cone_round_trip(fr in frame_strategy(), half in 0.1..1.4_f64, s in scale()) {
            let q = scaled(Quadric::cone(&fr, half), s);
            let c = classify(&q, AT).unwrap();
            prop_assert_eq!(c.surfaces.len(), 2);
            let mut up = 0;
            for def in &c.surfaces {
                let SurfaceDef::Cone(cn) = def else { panic!("not a cone") };
                prop_assert!(rel(cn.half_angle, half) < 1e-12);
                prop_assert!(near_pt(cn.apex(), fr.origin) < 1e-12);
                prop_assert!(cn.frame.z.cross(fr.z).norm() < 1e-12);
                up += usize::from(cn.frame.z.dot(fr.z) > 0.0);
            }
            prop_assert_eq!(up, 1);
            prop_assert_eq!(c.sign, s.signum());
            check_signs(&q, &c, fr.origin);
        }

        #[test]
        fn elliptic_cylinder_round_trip(fr in frame_strategy(), b in 0.3..5.0_f64, ratio in 1.2..4.0_f64, s in scale()) {
            let a = b * ratio;
            let q = scaled(Quadric::from_local(&fr, [1.0 / (a * a), 1.0 / (b * b), 0.0], [0.0; 3], -1.0), s);
            let c = classify(&q, AT).unwrap();
            let [SurfaceDef::EllipticCylinder(ec)] = c.surfaces.as_slice() else { panic!("not an elliptic cylinder") };
            prop_assert!(rel(ec.a, a) < 1e-12 && rel(ec.b, b) < 1e-12);
            prop_assert!(ec.frame.z.cross(fr.z).norm() < 1e-12);
            prop_assert!(ec.frame.x.cross(fr.x).norm() < 1e-12);
            prop_assert!(off_axis(ec.frame.origin, fr.origin, fr.z) < 1e-12);
            prop_assert_eq!(c.sign, s.signum());
            check_signs(&q, &c, fr.origin);
        }

        #[test]
        fn elliptic_cone_round_trip(fr in frame_strategy(), be in 0.3..2.0_f64, ratio in 1.2..3.0_f64, s in scale()) {
            let al = be * ratio;
            // x^2/al^2 + y^2/be^2 - z^2 = 0: the apex is the frame origin.
            let q = scaled(Quadric::from_local(&fr, [1.0 / (al * al), 1.0 / (be * be), -1.0], [0.0; 3], 0.0), s);
            let c = classify(&q, AT).unwrap();
            prop_assert_eq!(c.surfaces.len(), 2);
            let mut up = 0;
            for def in &c.surfaces {
                let SurfaceDef::EllipticCone(ec) = def else { panic!("not an elliptic cone") };
                prop_assert!(rel(ec.a, al) < 1e-12 && rel(ec.b, be) < 1e-12);
                prop_assert!(rel(ec.half_angle.tan(), al) < 1e-12);
                prop_assert!(near_pt(ec.apex(), fr.origin) < 1e-12);
                prop_assert!(ec.frame.x.cross(fr.x).norm() < 1e-12);
                up += usize::from(ec.frame.z.dot(fr.z) > 0.0);
            }
            prop_assert_eq!(up, 1);
            prop_assert_eq!(c.sign, s.signum());
            check_signs(&q, &c, fr.origin);
        }

        #[test]
        fn plane_written_as_quadric(fr in frame_strategy(), s in 0.1..10.0_f64) {
            let q = scaled(Quadric::plane(&fr), s);
            let c = classify(&q, AT).unwrap();
            let [SurfaceDef::Plane(pl)] = c.surfaces.as_slice() else { panic!("not a plane") };
            prop_assert!(pl.frame.z.cross(fr.z).norm() < 1e-12 && pl.frame.z.dot(fr.z) > 0.0);
            prop_assert!((pl.frame.origin - fr.origin).dot(fr.z).abs() / mag(fr.origin) < 1e-12);
            prop_assert_eq!(c.sign, 1.0);
            check_signs(&q, &c, fr.origin);
        }
    }

    #[test]
    fn round_off_cylinder_stays_circular() {
        let fr = Frame::from_z(Point3::new(1.0, 2.0, 3.0), V::new(1.0, 2.0, 2.0)).unwrap();
        let mut q = Quadric::cylinder(&fr, 2.5);
        for c in &mut q.coeffs {
            *c *= 1.0 + 1e-12;
        }
        q.coeffs[0] *= 1.0 + 3e-12;
        assert!(matches!(classify(&q, AT).unwrap().surfaces.as_slice(), [SurfaceDef::Cylinder(_)]));
    }

    #[test]
    fn unsupported_kinds_name_surface_and_line() {
        let fr = Frame::at(Point3::new(0.0, 0.0, 0.0));
        let cases = [
            ("ellipsoid", Quadric::from_local(&fr, [1.0, 2.0, 3.0], [0.0; 3], -1.0)),
            ("ellipsoid", Quadric::from_local(&fr, [1.0, 1.0, 3.0], [0.0; 3], -1.0)),
            ("hyperboloid", Quadric::from_local(&fr, [1.0, 1.0, -1.0], [0.0; 3], -1.0)),
            ("hyperboloid", Quadric::from_local(&fr, [1.0, 1.0, -1.0], [0.0; 3], 1.0)),
            ("paraboloid", Quadric::from_local(&fr, [1.0, 1.0, 0.0], [0.0, 0.0, -1.0], 0.0)),
            ("hyperbolic cylinder", Quadric::from_local(&fr, [1.0, -1.0, 0.0], [0.0; 3], -1.0)),
            ("empty", Quadric::from_local(&fr, [1.0, 1.0, 1.0], [0.0; 3], 1.0)),
            ("degenerate", Quadric { coeffs: [0.0; 10] }),
        ];
        for (name, q) in cases {
            let e = classify(&q, AT).unwrap_err();
            let msg = e.to_string();
            assert!(matches!(e, ClassifyError::Unsupported { .. }), "{name}: {msg}");
            assert!(msg.contains(name), "{name}: {msg}");
            assert!(msg.contains("surface 7") && msg.contains("line 12") && msg.contains("eigenvalues"), "{name}: {msg}");
        }
    }

    #[test]
    fn non_finite_is_invalid() {
        let mut q = Quadric { coeffs: [0.0; 10] };
        q.coeffs[0] = f64::NAN;
        assert!(matches!(classify(&q, AT), Err(ClassifyError::Invalid { .. })));
    }

    #[test]
    fn analytic_values() {
        // x^2 + y^2 + z^2 - 4: sphere radius 2 at the origin.
        let q = Quadric { coeffs: [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -4.0] };
        let c = classify(&q, AT).unwrap();
        let [SurfaceDef::Sphere(s)] = c.surfaces.as_slice() else { panic!() };
        assert!((s.radius - 2.0).abs() < 1e-14 && near_pt(s.frame.origin, Point3::new(0.0, 0.0, 0.0)) < 1e-14);
        // (x-3)^2 + (z+1)^2 - 4 = x^2 + z^2 - 6x + 2z + 6: y-axis cylinder, radius 2.
        let q = Quadric { coeffs: [1.0, 0.0, 1.0, 0.0, 0.0, 0.0, -6.0, 0.0, 2.0, 6.0] };
        let c = classify(&q, AT).unwrap();
        let [SurfaceDef::Cylinder(cy)] = c.surfaces.as_slice() else { panic!() };
        assert!((cy.radius - 2.0).abs() < 1e-13);
        assert!((cy.frame.origin.x - 3.0).abs() < 1e-13 && (cy.frame.origin.z + 1.0).abs() < 1e-13 && cy.frame.origin.y.abs() < 1e-13);
        assert!(cy.frame.z.y.abs() > 1.0 - 1e-14);
        // 2x - 4: the plane x = 2 with normal +x.
        let q = Quadric { coeffs: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, -4.0] };
        let c = classify(&q, AT).unwrap();
        let [SurfaceDef::Plane(p)] = c.surfaces.as_slice() else { panic!() };
        assert!((p.frame.origin.x - 2.0).abs() < 1e-14 && (p.frame.z.x - 1.0).abs() < 1e-14);
    }

    #[test]
    fn deterministic() {
        let fr = Frame::from_z(Point3::new(0.3, -1.0, 2.0), V::new(0.2, 1.0, -0.7)).unwrap();
        let q = Quadric::from_local(&fr, [0.25, 1.0, 0.0], [0.0; 3], -1.0);
        let a = classify(&q, AT).unwrap();
        assert_eq!(a, classify(&q, AT).unwrap());
        assert_eq!(format!("{a:?}"), format!("{:?}", classify(&q, AT).unwrap()));
    }
}
