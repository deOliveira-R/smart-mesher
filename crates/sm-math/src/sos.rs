//! Simulation of simplicity (Edelsbrunner & Mücke, 1990) for the four predicates.
//!
//! [`orient2d_sos`], [`orient3d_sos`], [`incircle_sos`] and [`insphere_sos`] never return
//! [`Sign::Zero`]. They return the exact sign whenever the plain predicate is nonzero (so they
//! can be dropped in anywhere the exact predicate is used), and otherwise the sign the predicate
//! *would* have if every input were perturbed by a fixed, infinitesimal amount.
//!
//! # The perturbation
//! Every predicate is the determinant of the matrix with rows `[x, y, (z), (w), 1]` in argument
//! order, where `w` is the lifted coordinate (`x² + y² (+ z²)`) for the circle/sphere tests.
//! Each point carries a caller-supplied `id` (e.g. its global vertex number). Coordinate `j` of
//! the point with id `p` is replaced by `x_j + ε^(2^g)` where `g` is the rank of `(p, j)` in
//! lexicographic order and `ε > 0` is smaller than any real number we care about. The lifted
//! coordinate is perturbed by its own independent infinitesimal (ordered after `x, y, z`), which
//! corresponds to infinitesimal weights; the resulting triangulation is a regular
//! triangulation of the perturbed points, which is Delaunay whenever the input is in general
//! position and a valid, consistent triangulation of the point set otherwise.
//!
//! Because the determinant is multilinear in the entries, the perturbed determinant expands into
//! monomials indexed by partial permutation matrices `S` (sets of cells sharing no row or
//! column); the coefficient of `S` is the signed minor of the unperturbed matrix with `S`'s rows
//! and columns deleted. The sign is that of the lowest-order monomial with a nonzero
//! coefficient; monomials are ordered by exponent `Σ 2^g`, a binary number, so ordering by `g`
//! relative to the points actually present in the call is sufficient. All minors are evaluated
//! exactly, so the result is exact for the perturbation defined above.
//!
//! The same ids must be used consistently across every predicate call (2D predicates among
//! themselves, 3D among themselves). Two points with equal ids are an error.
//!
//! # Cost
//! The fast path is one exact predicate evaluation. The tie-break path costs a few microseconds
//! (it evaluates a handful of exact sub-determinants), which is acceptable because it is only
//! taken for exactly degenerate inputs.

use crate::exact::Expansion;
use crate::{Sign, incircle, insphere, orient2d, orient3d};
use sm_core::Point3;
use std::sync::OnceLock;

/// A point together with its perturbation priority.
///
/// Ids define the (arbitrary but fixed) order in which points are perturbed; they must be
/// unique within one predicate call and consistent across calls.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tagged<P> {
    /// The point coordinates.
    pub point: P,
    /// Unique, stable id (for example the global vertex index).
    pub id: u64,
}

impl<P> Tagged<P> {
    /// Pair `point` with `id`.
    pub fn new(point: P, id: u64) -> Self {
        Self { point, id }
    }
}

/// Failure of a symbolic-perturbation tie break.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum SosError {
    /// Two arguments of a degenerate call share an id, so no perturbation can separate them.
    #[error("duplicate perturbation id {0}")]
    DuplicateId(u64),
    /// No perturbation monomial had a nonzero coefficient. Not reachable for distinct ids;
    /// reported instead of a wrong sign should an internal invariant ever break.
    #[error("symbolic perturbation could not resolve the sign")]
    Unresolved,
}

/// A partial permutation matrix: up to four `(row, col)` cells with distinct rows and columns.
#[derive(Clone, Copy)]
struct Matching {
    cells: [(u8, u8); 4],
    k: u8,
}

fn gen_matchings(n: usize) -> Vec<Matching> {
    fn rec(r: usize, n: usize, used: u32, cur: &mut Vec<(u8, u8)>, out: &mut Vec<Matching>) {
        if r == n {
            let mut cells = [(0, 0); 4];
            for (i, &c) in cur.iter().enumerate() {
                cells[i] = c;
            }
            out.push(Matching {
                cells,
                k: cur.len() as u8,
            });
            return;
        }
        rec(r + 1, n, used, cur, out);
        for c in 0..n - 1 {
            if used & (1 << c) == 0 {
                cur.push((r as u8, c as u8));
                rec(r + 1, n, used | (1 << c), cur, out);
                cur.pop();
            }
        }
    }
    let mut out = Vec::new();
    rec(0, n, 0, &mut Vec::new(), &mut out);
    out.retain(|m| m.k > 0);
    out
}

fn matchings(n: usize) -> &'static [Matching] {
    static T: [OnceLock<Vec<Matching>>; 3] = [OnceLock::new(), OnceLock::new(), OnceLock::new()];
    T[n - 3].get_or_init(|| gen_matchings(n))
}

/// Matchings sorted by perturbation order for one rank permutation `pos` (cached per
/// permutation: at most 6 + 24 + 120 distinct tables).
fn ordered_matchings(n: usize, pos: &[usize; 5]) -> &'static [Matching] {
    const SLOTS: usize = 6 + 24 + 120;
    static CACHE: [OnceLock<Vec<Matching>>; SLOTS] = [const { OnceLock::new() }; SLOTS];
    // Lehmer code of `pos[..n]` indexes the slot within the block of size n!.
    let mut code = 0usize;
    for i in 0..n {
        let smaller = pos[i + 1..n].iter().filter(|&&p| p < pos[i]).count();
        code = code * (n - i) + smaller;
    }
    let base = match n {
        3 => 0,
        4 => 6,
        _ => 30,
    };
    let pos = *pos;
    CACHE[base + code].get_or_init(|| {
        let cols = n - 1;
        let mut keyed: Vec<(u32, Matching)> = matchings(n)
            .iter()
            .map(|m| {
                let key = m.cells[..m.k as usize]
                    .iter()
                    .map(|&(r, c)| 1u32 << (pos[r as usize] * cols + c as usize))
                    .sum();
                (key, *m)
            })
            .collect();
        keyed.sort_unstable_by_key(|&(key, _)| key);
        keyed.into_iter().map(|(_, m)| m).collect()
    })
}

/// Sign of the determinant of `[entries | 1]` under the perturbation described in the module
/// docs. `entries` is row-major with `n - 1` exact non-constant entries per row.
fn sos_sign(entries: &[Expansion], ids: &[u64]) -> Result<Sign, SosError> {
    let n = ids.len();
    let mut order = [0usize, 1, 2, 3, 4];
    order[..n].sort_by_key(|&r| ids[r]);
    for w in order[..n].windows(2) {
        if ids[w[0]] == ids[w[1]] {
            return Err(SosError::DuplicateId(ids[w[0]]));
        }
    }
    let mut pos = [0usize; 5];
    for (rank, &r) in order[..n].iter().enumerate() {
        pos[r] = rank;
    }
    let cols = n - 1;
    let keyed = ordered_matchings(n, &pos);
    let one = Expansion::from_f64(1.0);
    for m in keyed {
        let cells = &m.cells[..m.k as usize];
        let (mut rows_used, mut cols_used) = (0u32, 0u32);
        let mut parity = 0usize;
        for (i, &(r, c)) in cells.iter().enumerate() {
            rows_used |= 1 << r;
            cols_used |= 1 << c;
            parity += r as usize + c as usize;
            parity += cells[i + 1..]
                .iter()
                .filter(|&&(r2, c2)| r2 > r && c2 < c)
                .count();
        }
        let mut rows = [0usize; 5];
        let mut nr = 0;
        for r in (0..n).filter(|&r| rows_used & (1 << r) == 0) {
            rows[nr] = r;
            nr += 1;
        }
        // Remaining columns; index `cols` is the constant-1 column.
        let mut cs = [0usize; 5];
        let mut nc = 0;
        for c in (0..=cols).filter(|&c| c == cols || cols_used & (1 << c) == 0) {
            cs[nc] = c;
            nc += 1;
        }
        let s = det_minor(entries, cols, &one, &rows[..nr], &cs[..nc]).sign();
        if !s.is_zero() {
            return Ok(if parity.is_multiple_of(2) { s } else { -s });
        }
    }
    // Unreachable for distinct ids: the monomial of a maximal matching has coefficient +-1.
    debug_assert!(false, "SoS fell through with distinct ids");
    Err(SosError::Unresolved)
}

/// Exact determinant of the submatrix of `[entries | 1]` on `rows` x `cols` (column index
/// `stride` is the constant column), by Laplace expansion on the first column.
/// Builds no per-minor matrices; dimensions are at most 5.
fn det_minor(
    entries: &[Expansion],
    stride: usize,
    one: &Expansion,
    rows: &[usize],
    cols: &[usize],
) -> Expansion {
    let get = |r: usize, c: usize| -> &Expansion {
        if c == stride {
            one
        } else {
            &entries[r * stride + c]
        }
    };
    match rows.len() {
        0 => return Expansion::from_f64(1.0),
        1 => return get(rows[0], cols[0]).clone(),
        _ => {}
    }
    let mut acc = Expansion::zero();
    for (i, &r) in rows.iter().enumerate() {
        let e = get(r, cols[0]);
        if e.is_zero() {
            continue;
        }
        let mut sub = [0usize; 5];
        let mut k = 0;
        for (j, &rj) in rows.iter().enumerate() {
            if j != i {
                sub[k] = rj;
                k += 1;
            }
        }
        let term = e.mul(&det_minor(entries, stride, one, &sub[..k], &cols[1..]));
        acc = if i % 2 == 0 {
            acc.add(&term)
        } else {
            acc.sub(&term)
        };
    }
    acc
}

fn ex(x: f64) -> Expansion {
    Expansion::from_f64(x)
}

fn lift2(p: [f64; 2]) -> Expansion {
    ex(p[0]).square().add(&ex(p[1]).square())
}

fn lift3(p: Point3) -> Expansion {
    ex(p.x)
        .square()
        .add(&ex(p.y).square())
        .add(&ex(p.z).square())
}

/// [`orient2d`] that never returns zero: ties are broken by symbolic perturbation.
pub fn orient2d_sos(
    a: Tagged<[f64; 2]>,
    b: Tagged<[f64; 2]>,
    c: Tagged<[f64; 2]>,
) -> Result<Sign, SosError> {
    let s = orient2d(a.point, b.point, c.point);
    if !s.is_zero() {
        return Ok(s);
    }
    let mut v = Vec::with_capacity(6);
    for t in [&a, &b, &c] {
        v.extend([ex(t.point[0]), ex(t.point[1])]);
    }
    sos_sign(&v, &[a.id, b.id, c.id])
}

/// [`orient3d`] that never returns zero: ties are broken by symbolic perturbation.
pub fn orient3d_sos(
    a: Tagged<Point3>,
    b: Tagged<Point3>,
    c: Tagged<Point3>,
    d: Tagged<Point3>,
) -> Result<Sign, SosError> {
    let s = orient3d(a.point, b.point, c.point, d.point);
    if !s.is_zero() {
        return Ok(s);
    }
    let mut v = Vec::with_capacity(12);
    for t in [&a, &b, &c, &d] {
        v.extend([ex(t.point.x), ex(t.point.y), ex(t.point.z)]);
    }
    sos_sign(&v, &[a.id, b.id, c.id, d.id])
}

/// [`incircle`] that never returns zero: ties are broken by symbolic perturbation.
pub fn incircle_sos(
    a: Tagged<[f64; 2]>,
    b: Tagged<[f64; 2]>,
    c: Tagged<[f64; 2]>,
    d: Tagged<[f64; 2]>,
) -> Result<Sign, SosError> {
    let s = incircle(a.point, b.point, c.point, d.point);
    if !s.is_zero() {
        return Ok(s);
    }
    let mut v = Vec::with_capacity(12);
    for t in [&a, &b, &c, &d] {
        v.extend([ex(t.point[0]), ex(t.point[1]), lift2(t.point)]);
    }
    sos_sign(&v, &[a.id, b.id, c.id, d.id])
}

/// [`insphere`] that never returns zero: ties are broken by symbolic perturbation.
pub fn insphere_sos(
    a: Tagged<Point3>,
    b: Tagged<Point3>,
    c: Tagged<Point3>,
    d: Tagged<Point3>,
    e: Tagged<Point3>,
) -> Result<Sign, SosError> {
    let s = insphere(a.point, b.point, c.point, d.point, e.point);
    if !s.is_zero() {
        return Ok(s);
    }
    let mut v = Vec::with_capacity(20);
    for t in [&a, &b, &c, &d, &e] {
        v.extend([ex(t.point.x), ex(t.point.y), ex(t.point.z), lift3(t.point)]);
    }
    sos_sign(&v, &[a.id, b.id, c.id, d.id, e.id])
}
