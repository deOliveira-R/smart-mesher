//! Discretized 2D boundary loops for surface schemes (map, submap, paving, ...).
//!
//! A [`Region2`] is an outer loop (counter-clockwise) plus hole loops (clockwise), so the
//! region's interior is always on the *left* of every loop. Vertex types follow the Cubit
//! submap/map nomenclature (`docs/research/cubit-schemes.md`): the interior angle at a vertex
//! decides how many quadrilaterals meet there.
//!
//! | type | elements | interior angle |
//! |---|---|---|
//! | `End` | 1 | about 90 degrees |
//! | `Side` | 2 | about 180 degrees |
//! | `Corner` | 3 | about 270 degrees |
//! | `Reversal` | 4 | about 360 degrees |
//!
//! A simple outer loop with right angles satisfies `sum(weight) = +4` and a hole loop
//! `sum(weight) = -4`, with weights `End +1, Side 0, Corner -1, Reversal -2` (turning number).

use core::f64::consts::TAU;

/// A point in the plane.
pub type P2 = (f64, f64);

/// Submap vertex type, from the number of quadrilaterals meeting at the vertex.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum VertexType {
    /// One element, interior angle about 90 degrees.
    End,
    /// Two elements, about 180 degrees.
    Side,
    /// Three elements, about 270 degrees.
    Corner,
    /// Four elements, about 360 degrees.
    Reversal,
}

impl VertexType {
    /// Classify an interior angle (radians) to the nearest of 90/180/270/360 degrees.
    pub fn from_interior_angle(angle: f64) -> Self {
        let q = angle / core::f64::consts::FRAC_PI_2;
        if q < 1.5 {
            VertexType::End
        } else if q < 2.5 {
            VertexType::Side
        } else if q < 3.5 {
            VertexType::Corner
        } else {
            VertexType::Reversal
        }
    }

    /// Turning weight: `End +1, Side 0, Corner -1, Reversal -2`.
    pub const fn weight(self) -> i32 {
        match self {
            VertexType::End => 1,
            VertexType::Side => 0,
            VertexType::Corner => -1,
            VertexType::Reversal => -2,
        }
    }
}

/// Signed area of a closed polygon (shoelace); positive for counter-clockwise order.
pub fn signed_area(l: &[P2]) -> f64 {
    let n = l.len();
    let mut s = 0.0;
    for i in 0..n {
        let (a, b) = (l[i], l[(i + 1) % n]);
        s += a.0 * b.1 - b.0 * a.1;
    }
    0.5 * s
}

/// Perimeter of a closed polygon.
pub fn perimeter(l: &[P2]) -> f64 {
    let n = l.len();
    (0..n)
        .map(|i| {
            let (a, b) = (l[i], l[(i + 1) % n]);
            (b.0 - a.0).hypot(b.1 - a.1)
        })
        .sum()
}

/// Interior angle at every vertex of a loop whose interior is on its left (outer loops counter-
/// clockwise, holes clockwise). A left turn of `t` gives `pi - t`.
///
/// **Reversals are ambiguous.** At a slit tip the two segments are antiparallel, the turn is
/// `atan2(+-0, negative) = +-pi` depending on the sign of a zero cross product, and the
/// interior angle comes out as `0` or `2 pi` from rounding alone. Coordinates cannot tell a
/// slit tip from a hairpin, so a caller that needs `VertexType::Reversal` must assign it itself.
pub fn interior_angles(l: &[P2]) -> Vec<f64> {
    let n = l.len();
    (0..n)
        .map(|i| {
            let (p, c, q) = (l[(i + n - 1) % n], l[i], l[(i + 1) % n]);
            let (d0, d1) = ((c.0 - p.0, c.1 - p.1), (q.0 - c.0, q.1 - c.1));
            let turn = (d0.0 * d1.1 - d0.1 * d1.0).atan2(d0.0 * d1.0 + d0.1 * d1.1);
            core::f64::consts::PI - turn
        })
        .collect()
}

/// Vertex types of a loop whose interior is on its left. `Reversal` is never reliable from
/// coordinates (see [`interior_angles`]); assign it explicitly at slit tips.
pub fn vertex_types(l: &[P2]) -> Vec<VertexType> {
    interior_angles(l).into_iter().map(VertexType::from_interior_angle).collect()
}

/// A planar region: a counter-clockwise outer loop and clockwise hole loops.
#[derive(Clone, Debug, PartialEq)]
pub struct Region2 {
    /// Fixture name.
    pub name: &'static str,
    /// Outer loop, counter-clockwise, no repeated first vertex.
    pub outer: Vec<P2>,
    /// Hole loops, clockwise.
    pub holes: Vec<Vec<P2>>,
    /// Exact area of the *discretized* region (outer polygon minus hole polygons), from the
    /// closed form of each fixture, not from the shoelace formula.
    pub area: f64,
    /// Expected types of the outer loop's vertices, where they are meaningful (right-angled
    /// polygons). `None` for loops that discretize a smooth curve.
    pub outer_types: Option<Vec<VertexType>>,
}

impl Region2 {
    /// Area computed by the shoelace formula (outer minus holes).
    pub fn shoelace_area(&self) -> f64 {
        signed_area(&self.outer) + self.holes.iter().map(|h| signed_area(h)).sum::<f64>()
    }

    /// All loops, outer first.
    pub fn loops(&self) -> impl Iterator<Item = &[P2]> {
        core::iter::once(self.outer.as_slice()).chain(self.holes.iter().map(Vec::as_slice))
    }

    /// Total number of boundary vertices.
    pub fn vertex_count(&self) -> usize {
        self.loops().map(<[P2]>::len).sum()
    }

    /// Even-odd point-in-region test over all loops (boundary points unspecified). Empty loops
    /// are ignored.
    pub fn contains(&self, p: P2) -> bool {
        let mut inside = false;
        for l in self.loops() {
            let n = l.len();
            if n == 0 {
                continue;
            }
            let mut j = n - 1;
            for i in 0..n {
                let (a, b) = (l[i], l[j]);
                if (a.1 > p.1) != (b.1 > p.1) && p.0 < (b.0 - a.0) * (p.1 - a.1) / (b.1 - a.1) + a.0 {
                    inside = !inside;
                }
                j = i;
            }
        }
        inside
    }
}

fn rect_loop(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<P2> {
    vec![(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
}

fn reversed(mut l: Vec<P2>) -> Vec<P2> {
    l.reverse();
    l
}

fn ngon(cx: f64, cy: f64, r: f64, n: usize) -> Vec<P2> {
    (0..n)
        .map(|i| {
            let a = TAU * i as f64 / n as f64;
            (cx + r * a.cos(), cy + r * a.sin())
        })
        .collect()
}

/// Area of a regular `n`-gon with circumradius `r`: `(n/2) r^2 sin(2 pi / n)`. This, not
/// `pi r^2`, is the exact area of the discretized circle.
pub fn ngon_area(n: usize, r: f64) -> f64 {
    0.5 * n as f64 * r * r * (TAU / n as f64).sin()
}

/// The unit square `[0, 1]^2`: area 1, types `End x 4`.
pub fn square() -> Region2 {
    rectangle(1.0, 1.0)
}

/// A `w x h` rectangle at the origin: area `w h`, types `End x 4`.
pub fn rectangle(w: f64, h: f64) -> Region2 {
    Region2 { name: "rectangle", outer: rect_loop(0.0, 0.0, w, h), holes: vec![], area: w * h, outer_types: Some(vec![VertexType::End; 4]) }
}

/// A `10 x 6` rectangle with two rectangular holes `[2,4] x [2,4]` and `[6,9] x [1,3]`:
/// area `60 - 4 - 6 = 50`. Holes are clockwise; each hole vertex is a `Corner` seen from the
/// region (270 degrees of interior).
pub fn rectangle_with_holes() -> Region2 {
    Region2 {
        name: "rectangle_with_holes",
        outer: rect_loop(0.0, 0.0, 10.0, 6.0),
        holes: vec![reversed(rect_loop(2.0, 2.0, 4.0, 4.0)), reversed(rect_loop(6.0, 1.0, 9.0, 3.0))],
        area: 60.0 - 4.0 - 6.0,
        outer_types: Some(vec![VertexType::End; 4]),
    }
}

/// An annulus with `n` points per circle, radii `r_in < r_out`, centred at the origin. The
/// exact area of the discretization is `ngon_area(n, r_out) - ngon_area(n, r_in)` (not
/// `pi (r_out^2 - r_in^2)`). Vertex types are not meaningful (all about `Side`), so `None`.
pub fn annulus(r_in: f64, r_out: f64, n: usize) -> Region2 {
    Region2 {
        name: "annulus",
        outer: ngon(0.0, 0.0, r_out, n),
        holes: vec![reversed(ngon(0.0, 0.0, r_in, n))],
        area: ngon_area(n, r_out) - ngon_area(n, r_in),
        outer_types: None,
    }
}

/// A disk of radius `r` discretized with `n` points: area `ngon_area(n, r)`, types not meaningful.
pub fn circle(r: f64, n: usize) -> Region2 {
    Region2 { name: "circle", outer: ngon(0.0, 0.0, r, n), holes: vec![], area: ngon_area(n, r), outer_types: None }
}

/// The L-shape `[0,2]x[0,1] U [0,1]x[0,2]`: area 3, six vertices, five `End` and one `Corner`
/// (the re-entrant vertex at `(1, 1)`). Turning sum `5 - 1 = 4`.
pub fn l_shape() -> Region2 {
    use VertexType::{Corner, End};
    Region2 {
        name: "l_shape",
        outer: vec![(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0), (1.0, 2.0), (0.0, 2.0)],
        holes: vec![],
        area: 3.0,
        outer_types: Some(vec![End, End, End, Corner, End, End]),
    }
}

/// A 5-vertex submap-like region: the rectangle `[0,4] x [0,2]` with an extra vertex at the
/// middle of the bottom side. Area 8; types `End, Side, End, End, End`, so a submap has to pick
/// the four `End` vertices as corners and treat the `Side` vertex as a mid-side interval break.
pub fn five_sided_submap() -> Region2 {
    use VertexType::{End, Side};
    Region2 {
        name: "five_sided_submap",
        outer: vec![(0.0, 0.0), (2.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0)],
        holes: vec![],
        area: 8.0,
        outer_types: Some(vec![End, Side, End, End, End]),
    }
}

/// All region fixtures (circle and annulus with 32 points).
pub fn all_regions() -> Vec<Region2> {
    vec![square(), rectangle(3.0, 2.0), rectangle_with_holes(), annulus(1.0, 2.0, 32), l_shape(), circle(1.5, 32), five_sided_submap()]
}
