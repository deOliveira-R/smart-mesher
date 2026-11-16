//! Wall geometry of a [`HexLattice`] for the consumers that place its elements as prisms
//! (the flattener of this crate and the B-rep import of `sm-csgbrep`). Kept outside `ir.rs`
//! (the frozen core): these are derived formulas, no data.

use crate::ir::{HexLattice, HexOrientation};

impl HexLattice {
    /// Unit normal (in the xy plane) of wall direction `dir` (0, 1, 2; taken modulo 3) of a
    /// hexagonal element: the three directions to a neighbour across a flat (the other three flats
    /// are the opposite normals). `X`: `0, 60, 120` degrees; `Y`: `90, 30, 150` degrees.
    pub fn wall_normal(orientation: HexOrientation, dir: usize) -> [f64; 2] {
        let s = 3f64.sqrt() / 2.0;
        match (orientation, dir % 3) {
            (HexOrientation::X, 0) => [1.0, 0.0],
            (HexOrientation::X, 1) => [0.5, s],
            (HexOrientation::X, _) => [-0.5, s],
            (HexOrientation::Y, 0) => [0.0, 1.0],
            (HexOrientation::Y, 1) => [s, 0.5],
            (HexOrientation::Y, _) => [-s, 0.5],
        }
    }

    /// `m` of the element `(q, r)` along wall direction `dir`: `normal . (centre - lattice centre) =
    /// pitch * m / 2`, an exact integer. The two flats of that direction are at `m - 1` and
    /// `m + 1`, so two neighbours compute the wall they share from the same integer and the planes
    /// are bit-identical.
    pub fn wall_m(orientation: HexOrientation, dir: usize, q: i64, r: i64) -> i64 {
        match (orientation, dir % 3) {
            (HexOrientation::X, 0) => 2 * q + r,
            (HexOrientation::X, 1) => q + 2 * r,
            (HexOrientation::Y, 0) => 2 * r + q,
            (HexOrientation::Y, 1) => 2 * q + r,
            (_, _) => r - q,
        }
    }

    /// Offset `pitch * m / 2` of the wall at integer `m` from the lattice centre along the normal.
    pub fn wall_offset(&self, m: i64) -> f64 {
        self.pitch * m as f64 / 2.0
    }

    /// Normal of the hull planes of the array: the wall normals of the other orientation (the
    /// outline of the array is a hexagon turned by 30 degrees against the element flats).
    pub fn hull_normal(orientation: HexOrientation, dir: usize) -> [f64; 2] {
        Self::wall_normal(
            match orientation {
                HexOrientation::X => HexOrientation::Y,
                HexOrientation::Y => HexOrientation::X,
            },
            dir,
        )
    }

    /// Offset of the hull planes `+-` [`Self::hull_normal`]: `sqrt(3)/2 * n_rings * pitch`, the
    /// plane through the centres of the elements of the first ring outside the array.
    pub fn hull_offset(&self) -> f64 {
        0.5 * 3f64.sqrt() * f64::from(self.n_rings) * self.pitch
    }

    /// Centred axial indices of the `6 n_rings` elements of the first ring outside the array (ring
    /// radius `n_rings`), in ring order.
    ///
    /// The part of space outside the array is exactly these hexagons clipped to the hull (the
    /// notches of the zigzag outline) together with everything outside the hull planes
    /// ([`Self::hull_normal`], [`Self::hull_offset`]): no element of a further ring reaches the
    /// hull. This is how `outer` is placed without one term per element of the array.
    pub fn outside_ring(&self) -> Vec<(i32, i32)> {
        let n = self.n_rings.saturating_add(1);
        (0..HexLattice::ring_len(n, 0) as u32).filter_map(|pos| Self::ring_axial(self.orientation, n, 0, pos)).collect()
    }
}
