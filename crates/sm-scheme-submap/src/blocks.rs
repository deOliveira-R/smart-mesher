//! Decomposition of a lattice polygon into rectangular blocks by parallel cut lines.
//!
//! Every reflex vertex (right turn) extends its incident side that runs along `j` into the
//! interior up to the first boundary contact. The cuts are vertical, so they never cross; each
//! piece is a rectangle. The result is computed as the vertical slab decomposition of the
//! polygon with horizontally adjacent equal slabs merged, which yields exactly the pieces those
//! cuts bound (the tests compare against the literal ray extension).

use crate::lattice::LatticePolygon;
use std::collections::BTreeMap;

/// An axis-aligned rectangle of unit cells `[i0, i1) x [j0, j1)`; its lattice points are
/// `i0..=i1` by `j0..=j1`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Block {
    /// Left lattice line.
    pub i0: i64,
    /// Right lattice line.
    pub i1: i64,
    /// Bottom lattice line.
    pub j0: i64,
    /// Top lattice line.
    pub j1: i64,
}

impl Block {
    /// Width in cells.
    pub fn width(&self) -> i64 {
        self.i1 - self.i0
    }

    /// Height in cells.
    pub fn height(&self) -> i64 {
        self.j1 - self.j0
    }

    /// Area in cells.
    pub fn area(&self) -> u64 {
        (self.width() * self.height()) as u64
    }
}

/// A cut line: the segment `x = const` from `j0` to `j1` between two blocks.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Cut {
    /// Lattice line of the cut.
    pub x: i64,
    /// Lower end (a boundary lattice point).
    pub j0: i64,
    /// Upper end (a boundary lattice point).
    pub j1: i64,
}

/// Decompose `p` into rectangles, sorted by `(j0, i0)`. `p` must come from
/// [`lattice_polygon`](crate::lattice::lattice_polygon) (simple, positive).
pub fn decompose(p: &LatticePolygon) -> Vec<Block> {
    let mut xs: Vec<i64> = p.verts.iter().map(|v| v.0).collect();
    xs.sort_unstable();
    xs.dedup();
    let mut blocks: Vec<Block> = Vec::new();
    let mut open: BTreeMap<(i64, i64), usize> = BTreeMap::new();
    let mut ys: Vec<i64> = Vec::new();
    for w in xs.windows(2) {
        let (x0, x1) = (w[0], w[1]);
        // Sides along `i` that span the whole strip bound its pieces from below and above.
        ys.clear();
        for k in 0..p.len() {
            if !p.dirs[k].along_i() {
                continue;
            }
            let ((a, y), (b, _)) = p.side(k);
            if a.min(b) <= x0 && a.max(b) >= x1 {
                ys.push(y);
            }
        }
        ys.sort_unstable();
        let mut next: BTreeMap<(i64, i64), usize> = BTreeMap::new();
        for pair in ys.as_chunks::<2>().0 {
            let key = (pair[0], pair[1]);
            let at = match open.get(&key) {
                Some(&bi) => {
                    blocks[bi].i1 = x1;
                    bi
                }
                None => {
                    blocks.push(Block {
                        i0: x0,
                        i1: x1,
                        j0: key.0,
                        j1: key.1,
                    });
                    blocks.len() - 1
                }
            };
            next.insert(key, at);
        }
        open = next;
    }
    blocks.sort_by_key(|b| (b.j0, b.i0));
    blocks
}

/// The cut lines between the blocks: for every pair of horizontally adjacent blocks the
/// overlap of their shared line, sorted by `(x, j0)`.
pub fn cuts(blocks: &[Block]) -> Vec<Cut> {
    let mut by_left: BTreeMap<i64, Vec<usize>> = BTreeMap::new();
    for (n, b) in blocks.iter().enumerate() {
        by_left.entry(b.i0).or_default().push(n);
    }
    let mut out = Vec::new();
    for p in blocks {
        for &q in by_left.get(&p.i1).map_or(&[][..], Vec::as_slice) {
            let (lo, hi) = (p.j0.max(blocks[q].j0), p.j1.min(blocks[q].j1));
            if lo < hi {
                out.push(Cut {
                    x: p.i1,
                    j0: lo,
                    j1: hi,
                });
            }
        }
    }
    out.sort_unstable();
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lattice::tests::{Lcg, random_staircase, staircase_sides};
    use crate::lattice::{Location, lattice_polygon};
    use sm_scheme_api::SideDir::{MinusI, MinusJ, PlusI, PlusJ};

    /// The literal rule: extend the `j`-running side of every reflex vertex into the interior
    /// to the first boundary contact.
    fn literal_cuts(p: &LatticePolygon) -> Vec<Cut> {
        let n = p.len();
        let mut out = Vec::new();
        for v in 0..n {
            let (inc, outg) = ((v + n - 1) % n, v);
            let turn = {
                let (a, b) = (
                    crate::lattice::step(p.dirs[inc]),
                    crate::lattice::step(p.dirs[outg]),
                );
                a.0 * b.1 - a.1 * b.0
            };
            if turn >= 0 {
                continue; // convex
            }
            // The side along j and the direction of its extension through the vertex.
            let (x, y) = p.verts[v];
            let dj = if !p.dirs[inc].along_i() {
                crate::lattice::step(p.dirs[inc]).1
            } else {
                -crate::lattice::step(p.dirs[outg]).1
            };
            let mut t = 1i64;
            let end = loop {
                let yy = y + dj * t;
                if p.inside(x, yy) != Location::Inside {
                    break yy;
                }
                t += 1;
            };
            assert_eq!(
                p.inside(x, end),
                Location::Boundary,
                "ray ends on the boundary"
            );
            out.push(Cut {
                x,
                j0: y.min(end),
                j1: y.max(end),
            });
        }
        out.sort_unstable();
        out.dedup();
        out
    }

    fn assert_partition(p: &LatticePolygon, blocks: &[Block]) {
        let (a, _, _) = p.pick_area();
        assert_eq!(blocks.iter().map(Block::area).sum::<u64>(), a, "total area");
        for (n, p1) in blocks.iter().enumerate() {
            assert!(p1.width() > 0 && p1.height() > 0);
            for p2 in &blocks[n + 1..] {
                let overlap = p1.i0 < p2.i1 && p2.i0 < p1.i1 && p1.j0 < p2.j1 && p2.j0 < p1.j1;
                assert!(!overlap, "{p1:?} overlaps {p2:?}");
            }
            // Every block is inside the polygon (corners and centre).
            for (x, y) in [
                (p1.i0, p1.j0),
                (p1.i1, p1.j0),
                (p1.i0, p1.j1),
                (p1.i1, p1.j1),
            ] {
                assert_ne!(p.inside(x, y), Location::Outside);
            }
        }
        assert!(
            blocks
                .windows(2)
                .all(|w| (w[0].j0, w[0].i0) < (w[1].j0, w[1].i0))
        );
    }

    #[test]
    fn the_l_has_two_blocks_and_one_cut() {
        let p = lattice_polygon(&[
            (PlusI, 4),
            (PlusJ, 4),
            (PlusI, 4),
            (PlusJ, 4),
            (MinusI, 8),
            (MinusJ, 8),
        ])
        .expect("L");
        let blocks = decompose(&p);
        assert_eq!(
            blocks,
            vec![
                Block {
                    i0: 0,
                    i1: 4,
                    j0: 0,
                    j1: 8
                },
                Block {
                    i0: 4,
                    i1: 8,
                    j0: 4,
                    j1: 8
                },
            ]
        );
        assert_partition(&p, &blocks);
        assert_eq!(cuts(&blocks), vec![Cut { x: 4, j0: 4, j1: 8 }]);
        assert_eq!(cuts(&blocks), literal_cuts(&p));
    }

    #[test]
    fn a_rectangle_is_one_block_without_cuts() {
        let p = lattice_polygon(&[(PlusI, 6), (PlusJ, 4), (MinusI, 6), (MinusJ, 4)]).expect("rect");
        let blocks = decompose(&p);
        assert_eq!(
            blocks,
            vec![Block {
                i0: 0,
                i1: 6,
                j0: 0,
                j1: 4
            }]
        );
        assert!(cuts(&blocks).is_empty());
    }

    #[test]
    fn staircases_have_k_plus_one_blocks_and_the_literal_cuts() {
        let mut rng = Lcg(77);
        for case in 0..200 {
            let (w, h) = random_staircase(&mut rng);
            let p = lattice_polygon(&staircase_sides(&w, &h)).expect("staircase");
            let blocks = decompose(&p);
            assert_eq!(blocks.len(), w.len(), "case {case}: {w:?} {h:?}");
            assert_partition(&p, &blocks);
            assert_eq!(cuts(&blocks), literal_cuts(&p), "case {case}: {w:?} {h:?}");
        }
    }

    #[test]
    fn plus_h_and_t_shapes_match_the_literal_cuts() {
        // Plus: arms of width 2 and length 2 around a 2 x 2 centre.
        let plus = [
            (PlusI, 2),
            (PlusJ, 2),
            (PlusI, 2),
            (PlusJ, 2),
            (MinusI, 2),
            (PlusJ, 2),
            (MinusI, 2),
            (MinusJ, 2),
            (MinusI, 2),
            (MinusJ, 2),
            (PlusI, 2),
            (MinusJ, 2),
        ];
        // H: two 1-wide posts joined by a bar.
        let h = [
            (PlusI, 1),
            (PlusJ, 2),
            (PlusI, 1),
            (MinusJ, 2),
            (PlusI, 1),
            (PlusJ, 5),
            (MinusI, 1),
            (MinusJ, 2),
            (MinusI, 1),
            (PlusJ, 2),
            (MinusI, 1),
            (MinusJ, 5),
        ];
        // T: a 5 x 1 bar on a 1 x 3 stem.
        let t = [
            (PlusI, 1),
            (PlusJ, 3),
            (PlusI, 2),
            (PlusJ, 1),
            (MinusI, 5),
            (MinusJ, 1),
            (PlusI, 2),
            (MinusJ, 3),
        ];
        for (name, sides, n_blocks) in [("plus", &plus[..], 3), ("H", &h[..], 3), ("T", &t[..], 3)]
        {
            let p = lattice_polygon(sides).unwrap_or_else(|e| panic!("{name}: {e}"));
            let blocks = decompose(&p);
            assert_partition(&p, &blocks);
            assert_eq!(cuts(&blocks), literal_cuts(&p), "{name}");
            assert_eq!(blocks.len(), n_blocks, "{name}: {blocks:?}");
        }
        let _ = (MinusJ, MinusI);
    }
}
