//! Pure 2-D constrained Delaunay kernel with Ruppert/Chew-style refinement and optimization.
//!
//! The kernel knows nothing about topology: it takes a planar straight-line graph
//! ([`Pslg`]), triangulates it with exact predicates (`sm_math::predicates`), recovers the
//! segments by Sloan flips and keeps the inside by a parity flood fill. [`Cdt::refine`] inserts
//! circumcenters of triangles that are bad **in 3-D** (the [`Geometry2`] lifts chart points to
//! the surface); [`Cdt::optimize`] smooths and flips.
//!
//! # Layout
//!
//! Triangles are structure-of-arrays (`Vec<[u32; 3]>` vertices, neighbours and edge codes),
//! always counter-clockwise; `nbr[t][i]` is the neighbour across the edge opposite vertex `i`.
//! During construction the convex hull is closed with *ghost* triangles (a vertex `GHOST`), so
//! insertion needs no special cases at the hull; the ghosts are removed by the flood fill.
//!
//! # Exactness and determinism
//!
//! Every orientation and in-circle decision is an exact predicate; ties are "no flip".
//! Points are inserted in Hilbert order of coordinates quantized on a fixed `2^-40` grid of the
//! bounding box, segments are recovered in the order of the Hilbert ranks of their end points,
//! and every queue and heap tie-break is an integer. The result therefore depends only on the
//! point coordinates, not on their input order, and is bit-identical on every platform.

mod build;
mod geometry;
mod optimize;
mod refine;
mod segments;

pub use geometry::{Geometry2, OptimizeOptions, RefineOptions, RefineReport};

use crate::error::TriError;
use sm_math::Sign;
use sm_math::predicates::orient2d;
use std::collections::BTreeMap;

/// "No neighbour" and the ghost vertex id.
pub(crate) const NONE: u32 = u32::MAX;
const GHOST: u32 = u32::MAX;
const DEAD: u32 = u32::MAX - 1;

/// Edge code: ordinary edge.
const C_NONE: u8 = 0;
/// Edge code: embedded constrained segment (does not toggle inside/outside).
const C_EMB: u8 = 1;
/// Edge code: boundary constrained segment (toggles inside/outside).
const C_BND: u8 = 2;

/// What a segment of a [`Pslg`] means for the inside/outside decision.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SegmentKind {
    /// Part of a boundary loop: crossing it toggles between outside and inside.
    Boundary,
    /// A constrained segment inside the region (an embedded curve): crossing it changes nothing.
    Embedded,
}

/// A constrained segment between two points of a [`Pslg`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Segment {
    /// First end point (index into [`Pslg::points`]).
    pub a: u32,
    /// Second end point.
    pub b: u32,
    /// Boundary or embedded.
    pub kind: SegmentKind,
}

/// A planar straight-line graph: points and constrained segments between them.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Pslg {
    /// The points. Isolated points are mesh vertices too.
    pub points: Vec<[f64; 2]>,
    /// The segments.
    pub segments: Vec<Segment>,
}

/// A constrained Delaunay triangulation of a [`Pslg`].
///
/// Built by [`Cdt::new`], improved by [`Cdt::refine`] and [`Cdt::optimize`]. Input points keep
/// their ids `0..n`; Steiner points get the next ids in insertion order.
#[derive(Clone, Debug)]
pub struct Cdt {
    pts: Vec<[f64; 2]>,
    pts3: Vec<[f64; 3]>,
    n_input: usize,
    tri: Vec<[u32; 3]>,
    nbr: Vec<[u32; 3]>,
    code: Vec<[u8; 3]>,
    tid: Vec<u32>,
    vtri: Vec<u32>,
    free: Vec<u32>,
    next_tid: u32,
    hint: u32,
    ghosts: bool,
    seg_of: BTreeMap<(u32, u32), u32>,
    scratch: build::Scratch,
}

#[inline]
fn key(a: u32, b: u32) -> (u32, u32) {
    if a < b { (a, b) } else { (b, a) }
}

impl Cdt {
    /// All points: the input points first, then the Steiner points in insertion order.
    pub fn points(&self) -> &[[f64; 2]] {
        &self.pts
    }

    /// Number of input points (ids `0..input_count()` are the PSLG points).
    pub fn input_count(&self) -> usize {
        self.n_input
    }

    /// The triangles inside the region, counter-clockwise, in a deterministic order.
    pub fn triangles(&self) -> Vec<[u32; 3]> {
        self.live().map(|t| self.tri[t as usize]).collect()
    }

    /// Neighbours of [`Cdt::triangles`]: entry `[t][i]` is the index (into `triangles()`) of the
    /// triangle across the edge opposite vertex `i`, or [`Cdt::NO_NEIGHBOR`] on the region
    /// boundary.
    pub fn neighbors(&self) -> Vec<[u32; 3]> {
        let mut remap = vec![NONE; self.tri.len()];
        for (k, t) in self.live().enumerate() {
            remap[t as usize] = k as u32;
        }
        self.live()
            .map(|t| {
                let n = self.nbr[t as usize];
                [0, 1, 2].map(|i| {
                    if n[i] == NONE {
                        NONE
                    } else {
                        remap[n[i] as usize]
                    }
                })
            })
            .collect()
    }

    /// The value of [`Cdt::neighbors`] entries with no neighbour.
    pub const NO_NEIGHBOR: u32 = NONE;

    /// True if the edge `a`-`b` is a constrained segment (boundary or embedded).
    pub fn is_constrained(&self, a: u32, b: u32) -> bool {
        self.seg_of.contains_key(&key(a, b))
    }

    /// The constrained edges `(a, b)` with `a < b`, ascending.
    pub fn constrained_edges(&self) -> Vec<(u32, u32)> {
        self.seg_of.keys().copied().collect()
    }

    /// Number of triangles inside the region.
    pub fn triangle_count(&self) -> usize {
        self.live().count()
    }

    /// Live (alive, real) triangle slots in ascending slot order.
    pub(crate) fn live(&self) -> impl Iterator<Item = u32> + '_ {
        (0..self.tri.len() as u32).filter(move |&t| self.is_real(t))
    }

    #[inline]
    pub(crate) fn is_real(&self, t: u32) -> bool {
        let v = self.tri[t as usize];
        v[0] != DEAD && v[2] != GHOST
    }

    #[inline]
    fn alive(&self, t: u32) -> bool {
        self.tri[t as usize][0] != DEAD
    }

    #[inline]
    fn p(&self, v: u32) -> [f64; 2] {
        self.pts[v as usize]
    }

    #[inline]
    fn orient(&self, a: u32, b: u32, c: [f64; 2]) -> Sign {
        orient2d(self.p(a), self.p(b), c)
    }

    /// Index of vertex `v` in triangle `t`.
    #[inline]
    fn index_of(&self, t: u32, v: u32) -> usize {
        let tv = self.tri[t as usize];
        if tv[0] == v {
            0
        } else if tv[1] == v {
            1
        } else {
            2
        }
    }

    /// The triangle `t` with directed edge `u -> v` and the index of its opposite vertex.
    fn find_edge(&self, u: u32, v: u32) -> Option<(u32, usize)> {
        let start = self.vtri[u as usize];
        if start == NONE {
            return None;
        }
        let mut t = start;
        for _ in 0..=self.tri.len() {
            let k = self.index_of(t, u);
            let tv = self.tri[t as usize];
            if tv[(k + 1) % 3] == v {
                return Some((t, (k + 2) % 3));
            }
            let nx = self.nbr[t as usize][(k + 1) % 3];
            if nx == NONE || nx == start {
                break;
            }
            t = nx;
        }
        // Boundary vertex of a ghost-free mesh: walk the other way.
        let mut t = start;
        for _ in 0..=self.tri.len() {
            let k = self.index_of(t, u);
            let tv = self.tri[t as usize];
            if tv[(k + 1) % 3] == v {
                return Some((t, (k + 2) % 3));
            }
            let nx = self.nbr[t as usize][(k + 2) % 3];
            if nx == NONE || nx == start {
                break;
            }
            t = nx;
        }
        None
    }
}
