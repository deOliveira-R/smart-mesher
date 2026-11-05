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

// ---------------------------------------------------------------------------------------------
// Allocation-free orient3d_sos

/// Number of non-empty partial matchings of a 4 x 3 grid (rows x coordinate columns).
const NM3: usize = 72;
/// Number of rank permutations of four points.
const NPERM4: usize = 24;

/// One monomial of the perturbed determinant: up to three `(row, col)` cells, `k` of them used.
#[derive(Clone, Copy)]
struct Mono {
    cells: [(u8, u8); 3],
    k: u8,
}

const EMPTY_MONO: Mono = Mono {
    cells: [(0, 0); 3],
    k: 0,
};

/// All non-empty matchings of the 4 x 3 grid, in enumeration order.
const fn all_monos() -> [Mono; NM3] {
    let mut out = [EMPTY_MONO; NM3];
    let mut n = 0;
    let mut code = 1;
    while code < 256 {
        // Base-4 digit per row: 0 = unmatched, 1..=3 = column + 1.
        let mut cols_used = 0u32;
        let mut ok = true;
        let mut m = EMPTY_MONO;
        let mut r = 0;
        while r < 4 {
            let d = (code >> (2 * r)) & 3;
            if d != 0 {
                let c = d - 1;
                if cols_used & (1 << c) != 0 {
                    ok = false;
                } else {
                    cols_used |= 1 << c;
                    m.cells[m.k as usize] = (r as u8, c as u8);
                    m.k += 1;
                }
            }
            r += 1;
        }
        if ok && m.k > 0 {
            out[n] = m;
            n += 1;
        }
        code += 1;
    }
    out
}

/// Monomials ordered by perturbation exponent for every rank permutation, indexed by Lehmer
/// code (the same code as the allocating version).
const fn build_table() -> [[Mono; NM3]; NPERM4] {
    let monos = all_monos();
    let mut table = [[EMPTY_MONO; NM3]; NPERM4];
    let mut code = 0;
    while code < NPERM4 {
        // Decode the Lehmer code into `pos`: digit i counts later entries smaller than pos[i].
        let mut digits = [0usize; 4];
        let mut c = code;
        let mut i = 4;
        while i > 0 {
            i -= 1;
            let base = 4 - i;
            digits[i] = c % base;
            c /= base;
        }
        let mut used = [false; 4];
        let mut pos = [0usize; 4];
        let mut i = 0;
        while i < 4 {
            let mut skip = digits[i];
            let mut v = 0;
            loop {
                if !used[v] {
                    if skip == 0 {
                        break;
                    }
                    skip -= 1;
                }
                v += 1;
            }
            used[v] = true;
            pos[i] = v;
            i += 1;
        }
        // Key per monomial, then insertion sort by key.
        let mut keyed = [(0u32, 0usize); NM3];
        let mut j = 0;
        while j < NM3 {
            let m = monos[j];
            let mut key = 0u32;
            let mut t = 0;
            while t < m.k as usize {
                let (r, cc) = m.cells[t];
                key += 1u32 << (pos[r as usize] * 3 + cc as usize);
                t += 1;
            }
            keyed[j] = (key, j);
            j += 1;
        }
        let mut a = 1;
        while a < NM3 {
            let cur = keyed[a];
            let mut b = a;
            while b > 0 && keyed[b - 1].0 > cur.0 {
                keyed[b] = keyed[b - 1];
                b -= 1;
            }
            keyed[b] = cur;
            a += 1;
        }
        let mut j = 0;
        while j < NM3 {
            table[code][j] = monos[keyed[j].1];
            j += 1;
        }
        code += 1;
    }
    table
}

/// Fixed perturbation table for 4-point predicates: 24 rank permutations x 72 monomials.
static TABLE4: [[Mono; NM3]; NPERM4] = build_table();

/// Coordinate `c` of a point.
#[inline]
fn coord(p: &Point3, c: usize) -> f64 {
    match c {
        0 => p.x,
        1 => p.y,
        _ => p.z,
    }
}

/// [`orient3d_sos`] without heap allocation: same results, fixed-size perturbation table.
///
/// Every minor of the perturbed determinant is a 3 x 3 determinant of the projected points
/// (an exact `orient2d`), a coordinate difference, or the constant 1, so no expansion
/// arithmetic is needed.
pub fn orient3d_sos_noalloc(
    a: Tagged<Point3>,
    b: Tagged<Point3>,
    c: Tagged<Point3>,
    d: Tagged<Point3>,
) -> Result<Sign, SosError> {
    let s = orient3d(a.point, b.point, c.point, d.point);
    if !s.is_zero() {
        return Ok(s);
    }
    let pts = [a.point, b.point, c.point, d.point];
    let ids = [a.id, b.id, c.id, d.id];
    let mut order = [0usize, 1, 2, 3];
    order.sort_by_key(|&r| ids[r]);
    for w in order.windows(2) {
        if ids[w[0]] == ids[w[1]] {
            return Err(SosError::DuplicateId(ids[w[0]]));
        }
    }
    let mut pos = [0usize; 4];
    for (rank, &r) in order.iter().enumerate() {
        pos[r] = rank;
    }
    let mut code = 0usize;
    for i in 0..4 {
        let smaller = pos[i + 1..].iter().filter(|&&p| p < pos[i]).count();
        code = code * (4 - i) + smaller;
    }
    for m in &TABLE4[code] {
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
        let mut rows = [0usize; 4];
        let mut nr = 0;
        for r in (0..4).filter(|&r| rows_used & (1 << r) == 0) {
            rows[nr] = r;
            nr += 1;
        }
        let mut cs = [0usize; 3];
        for (slot, c) in (0..3).filter(|&c| cols_used & (1 << c) == 0).enumerate() {
            cs[slot] = c;
        }
        let s = match nr {
            3 => {
                let p = |i: usize| [coord(&pts[rows[i]], cs[0]), coord(&pts[rows[i]], cs[1])];
                orient2d(p(0), p(1), p(2))
            }
            2 => Sign::of(coord(&pts[rows[0]], cs[0]) - coord(&pts[rows[1]], cs[0])),
            _ => Sign::Positive,
        };
        if !s.is_zero() {
            return Ok(if parity.is_multiple_of(2) { s } else { -s });
        }
    }
    debug_assert!(false, "SoS fell through with distinct ids");
    Err(SosError::Unresolved)
}
