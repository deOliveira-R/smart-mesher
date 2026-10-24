//! Root finders against analytic roots.

use sm_math::roots::{RootError, RootOptions, brent, newton_bisect};

fn opts() -> RootOptions {
    RootOptions {
        x_tol: 1e-15,
        max_iter: 200,
    }
}

#[test]
fn brent_sqrt2_and_dottie() {
    let r = brent(|x| x * x - 2.0, 0.0, 2.0, opts()).unwrap();
    assert!((r.x - 2f64.sqrt()).abs() < 4e-16, "{r:?}");
    let r = brent(|x| x.cos() - x, 0.0, 1.0, opts()).unwrap();
    assert!((r.x - 0.739_085_133_215_160_6).abs() < 1e-15, "{r:?}");
    assert!(r.fx.abs() < 1e-15);
}

#[test]
fn brent_handles_flat_and_steep_functions() {
    // Triple root at 1 (sign change, f' = f'' = 0): accuracy limited by flatness, still converges.
    let r = brent(|x| (x - 1.0).powi(3), -1.0, 4.0, opts()).unwrap();
    assert!((r.x - 1.0).abs() < 1e-5, "{r:?}");
    // Double root at 2 plus simple root at 5: bracket the simple one.
    let r = brent(|x| (x - 2.0) * (x - 2.0) * (x - 5.0), 3.0, 9.0, opts()).unwrap();
    assert!((r.x - 5.0).abs() < 1e-14);
    // Steep: exp(100 x) - 1e10 and a nearly vertical atan-like jump.
    let r = brent(|x| (100.0 * x).exp() - 1e10, 0.0, 1.0, opts()).unwrap();
    assert!((r.x - 1e10f64.ln() / 100.0).abs() < 1e-14);
    let r = brent(|x| (1e8 * (x - 0.3)).tanh(), 0.0, 1.0, opts()).unwrap();
    assert!((r.x - 0.3).abs() < 1e-12);
    // Discontinuous sign change: converges to the jump.
    let r = brent(|x| if x < 0.25 { -1.0 } else { 1.0 }, 0.0, 1.0, opts()).unwrap();
    assert!((r.x - 0.25).abs() < 1e-14);
}

#[test]
fn brent_reversed_bracket_and_endpoint_roots() {
    let r = brent(|x| x * x - 2.0, 2.0, 0.0, opts()).unwrap();
    assert!((r.x - 2f64.sqrt()).abs() < 4e-16);
    let r = brent(|x| x - 1.0, 1.0, 3.0, opts()).unwrap();
    assert_eq!((r.x, r.iterations), (1.0, 0));
}

#[test]
fn errors() {
    assert!(matches!(
        brent(|x| x * x + 1.0, -1.0, 1.0, opts()),
        Err(RootError::NotBracketed { .. })
    ));
    assert_eq!(
        brent(|_| f64::NAN, 0.0, 1.0, opts()),
        Err(RootError::NonFinite)
    );
    assert_eq!(
        brent(|x| x, f64::NAN, 1.0, opts()),
        Err(RootError::NonFinite)
    );
    assert_eq!(
        brent(
            |x| x,
            -1.0,
            1.0,
            RootOptions {
                x_tol: -1.0,
                max_iter: 10
            }
        ),
        Err(RootError::NonFinite)
    );
    let e = brent(
        |x| x * x - 2.0,
        0.0,
        2.0,
        RootOptions {
            x_tol: 0.0,
            max_iter: 1,
        },
    );
    assert!(matches!(e, Err(RootError::MaxIterations { .. })));
    assert!(matches!(
        newton_bisect(|x| (x * x + 1.0, 2.0 * x), -1.0, 1.0, None, opts()),
        Err(RootError::NotBracketed { .. })
    ));
}

#[test]
fn newton_bisect_converges_quickly_and_safely() {
    let r = newton_bisect(|x| (x * x - 2.0, 2.0 * x), 0.0, 2.0, Some(1.5), opts()).unwrap();
    assert!((r.x - 2f64.sqrt()).abs() < 4e-16);
    assert!(r.iterations <= 8, "Newton should be quadratic: {r:?}");
    // Zero derivative at the start point and a cubic: must fall back to bisection.
    let r = newton_bisect(
        |x| (x * x * x - 8.0, 3.0 * x * x),
        -1.0,
        5.0,
        Some(0.0),
        opts(),
    )
    .unwrap();
    assert!((r.x - 2.0).abs() < 1e-14, "{r:?}");
    // Newton diverges for atan from far away; bisection safeguard rescues it.
    let r = newton_bisect(
        |x| (x.atan(), 1.0 / (1.0 + x * x)),
        -10.0,
        20.0,
        Some(10.0),
        opts(),
    )
    .unwrap();
    assert!(r.x.abs() < 1e-12, "{r:?}");
    // Wrong derivative (always zero) degrades to pure bisection.
    let r = newton_bisect(|x| (x - 0.3, 0.0), 0.0, 1.0, None, opts()).unwrap();
    assert!((r.x - 0.3).abs() < 1e-14);
    let r = newton_bisect(|x| (x.cos() - x, -x.sin() - 1.0), 0.0, 1.0, None, opts()).unwrap();
    assert!((r.x - 0.739_085_133_215_160_6).abs() < 1e-15);
}
