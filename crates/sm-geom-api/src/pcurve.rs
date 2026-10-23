//! Curves in a surface's parameter space (pcurves), used by coedges.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A 2D line `(u, v) = origin + t * dir` in surface parameter space. Periodic seams are
/// expressed by pcurves at `u = 0` and `u = period` for the two uses of the seam edge.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Line2 {
    /// Point at `t = 0`.
    pub origin: [f64; 2],
    /// Direction (not necessarily unit; same-parameter with the edge curve).
    pub dir: [f64; 2],
}

/// A coedge's curve in its face's `(u, v)` space, same-parameter with the edge's 3D curve.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[non_exhaustive]
pub enum PCurve {
    /// A straight segment in parameter space.
    Line(Line2),
}

impl PCurve {
    /// `(u, v)` at parameter `t`.
    pub fn eval(&self, t: f64) -> [f64; 2] {
        match self {
            Self::Line(l) => [l.origin[0] + t * l.dir[0], l.origin[1] + t * l.dir[1]],
        }
    }
}
