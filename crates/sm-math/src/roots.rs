//! Bracketed scalar root finding: [`brent`] (derivative-free) and [`newton_bisect`]
//! (Newton with guaranteed bisection fallback).
//!
//! Both require a sign change on the bracket `[a, b]`, never leave it, are fully deterministic
//! (pure `f64` arithmetic, no global state) and have a hard iteration cap. Brent's method
//! converges superlinearly on smooth functions and never needs more than roughly
//! `(log2((b - a) / tol))²` evaluations in the worst case.

/// Termination controls shared by the root finders.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RootOptions {
    /// Absolute tolerance on the root location. The effective tolerance is
    /// `x_tol + 2 * EPSILON * |x|`, so it never asks for more precision than `f64` has.
    pub x_tol: f64,
    /// Maximum number of function evaluations after the two initial ones.
    pub max_iter: u32,
}

impl Default for RootOptions {
    fn default() -> Self {
        Self {
            x_tol: 0.0,
            max_iter: 200,
        }
    }
}

/// A located root.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Root {
    /// The root estimate, within the tolerance of a sign change of `f`.
    pub x: f64,
    /// `f(x)`.
    pub fx: f64,
    /// Iterations used.
    pub iterations: u32,
}

/// Root-finding failure.
#[derive(Clone, Copy, Debug, PartialEq, thiserror::Error)]
pub enum RootError {
    /// `f(a)` and `f(b)` have the same strict sign.
    #[error("root is not bracketed: f(a) = {fa}, f(b) = {fb}")]
    NotBracketed {
        /// `f(a)`.
        fa: f64,
        /// `f(b)`.
        fb: f64,
    },
    /// The function returned NaN/inf, or the bracket/tolerance was invalid.
    #[error("non-finite function value or invalid bracket/options")]
    NonFinite,
    /// The iteration cap was reached; `best` is the best estimate so far.
    #[error("no convergence in {} iterations (best x = {})", .best.iterations, .best.x)]
    MaxIterations {
        /// Best estimate when the cap was hit.
        best: Root,
    },
}

fn valid(a: f64, b: f64, o: &RootOptions) -> bool {
    a.is_finite() && b.is_finite() && o.x_tol.is_finite() && o.x_tol >= 0.0
}

fn same_strict_sign(x: f64, y: f64) -> bool {
    (x > 0.0 && y > 0.0) || (x < 0.0 && y < 0.0)
}

/// Brent's method (inverse quadratic interpolation, secant and bisection) on `[a, b]`.
///
/// # Errors
/// [`RootError::NotBracketed`] if there is no sign change, [`RootError::NonFinite`] if `f`
/// returns a non-finite value or the inputs are invalid, [`RootError::MaxIterations`] if the
/// cap is hit.
pub fn brent(
    mut f: impl FnMut(f64) -> f64,
    a: f64,
    b: f64,
    opts: RootOptions,
) -> Result<Root, RootError> {
    if !valid(a, b, &opts) {
        return Err(RootError::NonFinite);
    }
    let (mut a, mut b) = (a, b);
    let mut fa = f(a);
    let mut fb = f(b);
    if !fa.is_finite() || !fb.is_finite() {
        return Err(RootError::NonFinite);
    }
    if fa == 0.0 {
        return Ok(Root {
            x: a,
            fx: 0.0,
            iterations: 0,
        });
    }
    if fb == 0.0 {
        return Ok(Root {
            x: b,
            fx: 0.0,
            iterations: 0,
        });
    }
    if same_strict_sign(fa, fb) {
        return Err(RootError::NotBracketed { fa, fb });
    }
    let (mut c, mut fc) = (a, fa);
    let mut d = b - a;
    let mut e = d;
    for iter in 1..=opts.max_iter {
        if same_strict_sign(fb, fc) {
            c = a;
            fc = fa;
            d = b - a;
            e = d;
        }
        if fc.abs() < fb.abs() {
            a = b;
            b = c;
            c = a;
            fa = fb;
            fb = fc;
            fc = fa;
        }
        let tol = 2.0 * f64::EPSILON * b.abs() + 0.5 * opts.x_tol;
        let xm = 0.5 * (c - b);
        if xm.abs() <= tol || fb == 0.0 {
            return Ok(Root {
                x: b,
                fx: fb,
                iterations: iter - 1,
            });
        }
        if e.abs() >= tol && fa.abs() > fb.abs() {
            let s = fb / fa;
            let (mut p, mut q);
            if a == c {
                p = 2.0 * xm * s;
                q = 1.0 - s;
            } else {
                let qq = fa / fc;
                let r = fb / fc;
                p = s * (2.0 * xm * qq * (qq - r) - (b - a) * (r - 1.0));
                q = (qq - 1.0) * (r - 1.0) * (s - 1.0);
            }
            if p > 0.0 {
                q = -q;
            }
            p = p.abs();
            let min1 = 3.0 * xm * q - (tol * q).abs();
            let min2 = (e * q).abs();
            if 2.0 * p < min1.min(min2) {
                e = d;
                d = p / q;
            } else {
                d = xm;
                e = d;
            }
        } else {
            d = xm;
            e = d;
        }
        a = b;
        fa = fb;
        b += if d.abs() > tol { d } else { tol.copysign(xm) };
        fb = f(b);
        if !fb.is_finite() {
            return Err(RootError::NonFinite);
        }
    }
    Err(RootError::MaxIterations {
        best: Root {
            x: b,
            fx: fb,
            iterations: opts.max_iter,
        },
    })
}

/// Newton–bisection hybrid on `[a, b]`: takes a Newton step when it stays inside the current
/// bracket and shrinks fast enough, bisects otherwise, so convergence is guaranteed.
///
/// `fdf` returns `(f(x), f'(x))`. `x0` is the first iterate (clamped to the bracket; `None`
/// starts at the midpoint). A zero or non-finite derivative simply forces a bisection step.
///
/// # Errors
/// As for [`brent`].
pub fn newton_bisect(
    mut fdf: impl FnMut(f64) -> (f64, f64),
    a: f64,
    b: f64,
    x0: Option<f64>,
    opts: RootOptions,
) -> Result<Root, RootError> {
    if !valid(a, b, &opts) {
        return Err(RootError::NonFinite);
    }
    let (fa, _) = fdf(a);
    let (fb, _) = fdf(b);
    if !fa.is_finite() || !fb.is_finite() {
        return Err(RootError::NonFinite);
    }
    if fa == 0.0 {
        return Ok(Root {
            x: a,
            fx: 0.0,
            iterations: 0,
        });
    }
    if fb == 0.0 {
        return Ok(Root {
            x: b,
            fx: 0.0,
            iterations: 0,
        });
    }
    if same_strict_sign(fa, fb) {
        return Err(RootError::NotBracketed { fa, fb });
    }
    // Keep `lo` the endpoint with f < 0 and `hi` the one with f > 0.
    let (mut lo, mut hi) = if fa < 0.0 { (a, b) } else { (b, a) };
    let (min_x, max_x) = (a.min(b), a.max(b));
    let mut x = x0
        .filter(|v| v.is_finite())
        .map_or(0.5 * (a + b), |v| v.clamp(min_x, max_x));
    let mut dx_old = (b - a).abs();
    let mut dx = dx_old;
    let (mut fx, mut dfx) = fdf(x);
    for iter in 1..=opts.max_iter {
        if !fx.is_finite() {
            return Err(RootError::NonFinite);
        }
        if fx == 0.0 {
            return Ok(Root {
                x,
                fx,
                iterations: iter - 1,
            });
        }
        // Maintain the bracket.
        if fx < 0.0 {
            lo = x;
        } else {
            hi = x;
        }
        let newton_out = ((x - hi) * dfx - fx) * ((x - lo) * dfx - fx) > 0.0;
        let slow = (2.0 * fx).abs() > (dx_old * dfx).abs();
        let prev = x;
        if !dfx.is_finite() || dfx == 0.0 || newton_out || slow {
            dx_old = dx;
            dx = 0.5 * (hi - lo);
            x = lo + dx;
        } else {
            dx_old = dx;
            dx = fx / dfx;
            x -= dx;
        }
        let tol = opts.x_tol + 2.0 * f64::EPSILON * x.abs();
        if x == prev || dx.abs() <= tol || (hi - lo).abs() <= tol {
            let (fx_end, _) = fdf(x);
            return Ok(Root {
                x,
                fx: fx_end,
                iterations: iter,
            });
        }
        (fx, dfx) = fdf(x);
    }
    Err(RootError::MaxIterations {
        best: Root {
            x,
            fx,
            iterations: opts.max_iter,
        },
    })
}
