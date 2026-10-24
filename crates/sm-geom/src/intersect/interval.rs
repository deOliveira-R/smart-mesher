//! Conservative interval bounds of implicit functions over axis-aligned boxes.

use sm_geom_api::{ImplicitDef, ImplicitTorus, Quadric};

/// `[lo, hi]`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Iv(pub f64, pub f64);

impl Iv {
    fn add(self, o: Iv) -> Iv {
        Iv(self.0 + o.0, self.1 + o.1)
    }
    fn scale(self, k: f64) -> Iv {
        if k >= 0.0 { Iv(self.0 * k, self.1 * k) } else { Iv(self.1 * k, self.0 * k) }
    }
    fn mul(self, o: Iv) -> Iv {
        let p = [self.0 * o.0, self.0 * o.1, self.1 * o.0, self.1 * o.1];
        Iv(p.iter().fold(f64::INFINITY, |a, b| a.min(*b)), p.iter().fold(f64::NEG_INFINITY, |a, b| a.max(*b)))
    }
    fn sq(self) -> Iv {
        let (a, b) = (self.0 * self.0, self.1 * self.1);
        if self.0 <= 0.0 && self.1 >= 0.0 { Iv(0.0, a.max(b)) } else { Iv(a.min(b), a.max(b)) }
    }
    fn shift(self, k: f64) -> Iv {
        Iv(self.0 + k, self.1 + k)
    }
}

fn quadric_range(q: &Quadric, b: &[f64; 6]) -> Iv {
    let [a, bb, c, d, e, f, g, h, j, k] = q.coeffs;
    let (x, y, z) = (Iv(b[0], b[3]), Iv(b[1], b[4]), Iv(b[2], b[5]));
    x.sq().scale(a)
        .add(y.sq().scale(bb))
        .add(z.sq().scale(c))
        .add(x.mul(y).scale(d))
        .add(y.mul(z).scale(e))
        .add(z.mul(x).scale(f))
        .add(x.scale(g))
        .add(y.scale(h))
        .add(z.scale(j))
        .shift(k)
}

fn torus_range(t: &ImplicitTorus, b: &[f64; 6]) -> Iv {
    // Box in the local frame: bound of the transformed corners.
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for i in 0..8 {
        let p = sm_core::Point3::new(if i & 1 == 0 { b[0] } else { b[3] }, if i & 2 == 0 { b[1] } else { b[4] }, if i & 4 == 0 { b[2] } else { b[5] });
        let l = t.frame.to_local(p);
        for (k, v) in [l.x, l.y, l.z].into_iter().enumerate() {
            lo[k] = lo[k].min(v);
            hi[k] = hi[k].max(v);
        }
    }
    let (x, y, z) = (Iv(lo[0], hi[0]), Iv(lo[1], hi[1]), Iv(lo[2], hi[2]));
    let (rr, k) = (t.major * t.major, t.major * t.major - t.minor * t.minor);
    let rho2 = x.sq().add(y.sq());
    let s = rho2.add(z.sq()).shift(k);
    s.sq().add(rho2.scale(-4.0 * rr))
}

/// Range of `f` over the box `[xmin, ymin, zmin, xmax, ymax, zmax]`: `f(p)` for every `p` in the
/// box lies in the returned interval.
pub(crate) fn range(def: &ImplicitDef, b: &[f64; 6]) -> (f64, f64) {
    let iv = match def {
        ImplicitDef::Quadric(q) => quadric_range(q, b),
        ImplicitDef::Torus(t) => torus_range(t, b),
        _ => Iv(f64::NEG_INFINITY, f64::INFINITY),
    };
    (iv.0, iv.1)
}
