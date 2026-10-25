//! Orientation of six face grids into one `(i, j, k)` lattice, and 3-D transfinite
//! interpolation of the interior. Pure functions over node keys and positions.
//!
//! Orientation never uses the grids' own `(i, j)` numbering (a copied face keeps its root's
//! numbering): only the vertex keys at the four grid corners and the keys along the grid
//! sides. The frame comes from the topology: the lowest vertex id is the origin and its three
//! neighbours, ordered by id and made right-handed by a triple product, are the axes. The result
//! therefore does not depend on face order, face orientation or grid numbering.

use rayon::prelude::*;
use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh_api::{CellShape, Param};
use sm_scheme_api::{ElemShape, Elements, FailureKind, NodeGrid, NodeKey, Patch, PatchNode, Target};
use std::collections::{BTreeMap, BTreeSet};

/// Above this many interior nodes (or cells) the lattice loops run on rayon. Results do not
/// depend on it: every node and every cell is independent.
const PAR_MIN: usize = 1 << 14;

/// Why a block could not be assembled.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Failure {
    pub kind: FailureKind,
    pub reason: String,
}

fn fail<T>(kind: FailureKind, reason: impl Into<String>) -> Result<T, Failure> {
    Err(Failure { kind, reason: reason.into() })
}

type V3 = [f64; 3];

/// The lattice of one volume: interval counts, keys and positions of every lattice node.
#[derive(Debug)]
pub(crate) struct Block {
    /// Intervals along i, j, k.
    pub n: [usize; 3],
    /// Keys, `i + (ni+1) * (j + (nj+1) * k)`; interior keys are `(owner, 0..)` in k, j, i order.
    pub keys: Vec<NodeKey>,
    /// Positions in the same layout.
    pub pos: Vec<V3>,
}

impl Block {
    fn strides(&self) -> [usize; 3] {
        strides(self.n)
    }
}

fn strides(n: [usize; 3]) -> [usize; 3] {
    [1, n[0] + 1, (n[0] + 1) * (n[1] + 1)]
}

fn is_boundary(c: [usize; 3], n: [usize; 3]) -> bool {
    (0..3).any(|a| c[a] == 0 || c[a] == n[a])
}

/// Lattice faces: (fixed axis, side, u axis, v axis).
const FACES: [(usize, usize, usize, usize); 6] = [(0, 0, 1, 2), (0, 1, 1, 2), (1, 0, 0, 2), (1, 1, 0, 2), (2, 0, 0, 1), (2, 1, 0, 1)];

fn corner_index(bits: [usize; 3]) -> usize {
    bits[0] + 2 * bits[1] + 4 * bits[2]
}

fn is_vertex_key(k: NodeKey) -> bool {
    k.index == 0 && matches!(k.owner.kind(), EntityKind::TopoVertex | EntityKind::PartitionVertex)
}

/// The four corner vertices of a grid, in grid order `(0,0), (ni-1,0), (ni-1,nj-1), (0,nj-1)`.
fn grid_corners(g: &NodeGrid) -> Result<[EntityId; 4], Failure> {
    let (ni, nj) = (g.ni, g.nj);
    if ni < 2 || nj < 2 || g.nodes.len() != ni as usize * nj as usize {
        return fail(FailureKind::Unsupported, format!("a {ni} x {nj} grid is not a structured face"));
    }
    let at = |i, j| g.at(i, j).filter(|k| is_vertex_key(*k)).map(|k| k.owner);
    match (at(0, 0), at(ni - 1, 0), at(ni - 1, nj - 1), at(0, nj - 1)) {
        (Some(a), Some(b), Some(c), Some(d)) if BTreeSet::from([a, b, c, d]).len() == 4 => Ok([a, b, c, d]),
        _ => fail(FailureKind::Unsupported, "a face grid whose corners are not four distinct vertex nodes"),
    }
}

/// Where `a` of a grid lies and which grid axis the step `a -> b` follows.
struct Orient {
    /// Grid corner of the first node: `(i is high, j is high)`.
    hi: (bool, bool),
    /// True if `u` runs along the grid's `i`.
    u_along_i: bool,
}

fn corner_pos(c: &[EntityId; 4], v: EntityId) -> Option<(bool, bool)> {
    let p = c.iter().position(|x| *x == v)?;
    Some([(false, false), (true, false), (true, true), (false, true)][p])
}

fn orient(c: &[EntityId; 4], a: EntityId, b: EntityId, d: EntityId) -> Option<Orient> {
    let (pa, pb, pd) = (corner_pos(c, a)?, corner_pos(c, b)?, corner_pos(c, d)?);
    // b shares j with a (and differs in i): u runs along i; d must then differ in j only.
    if pb.1 == pa.1 && pb.0 != pa.0 && pd.0 == pa.0 && pd.1 != pa.1 {
        Some(Orient { hi: pa, u_along_i: true })
    } else if pb.0 == pa.0 && pb.1 != pa.1 && pd.1 == pa.1 && pd.0 != pa.0 {
        Some(Orient { hi: pa, u_along_i: false })
    } else {
        None
    }
}

/// Node `(u, v)` of a grid seen from `a` with `u` toward `b`.
fn grid_at(g: &NodeGrid, o: &Orient, u: u32, v: u32) -> Option<NodeKey> {
    let (ni, nj) = (g.ni, g.nj);
    let (ru, rv) = if o.u_along_i { (o.hi.0, o.hi.1) } else { (o.hi.1, o.hi.0) };
    let (nu, nv) = if o.u_along_i { (ni, nj) } else { (nj, ni) };
    let uu = if ru { nu - 1 - u } else { u };
    let vv = if rv { nv - 1 - v } else { v };
    if o.u_along_i { g.at(uu, vv) } else { g.at(vv, uu) }
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn det(a: V3, b: V3, c: V3) -> f64 {
    a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])
}

fn pt(p: Point3) -> V3 {
    [p.x, p.y, p.z]
}

/// The six corner-vertex quadruples' common structure: derive the lattice frame from the
/// topology alone. Returns the eight vertices indexed by `i + 2j + 4k`; the handedness is not
/// decided here (see [`assemble`]).
fn frame(corners: &[[EntityId; 4]]) -> Result<[EntityId; 8], Failure> {
    let mut adj: BTreeMap<EntityId, BTreeSet<EntityId>> = BTreeMap::new();
    let mut pair_faces: BTreeMap<(EntityId, EntityId), u32> = BTreeMap::new();
    for c in corners {
        for s in 0..4 {
            let (a, b) = (c[s], c[(s + 1) % 4]);
            adj.entry(a).or_default().insert(b);
            adj.entry(b).or_default().insert(a);
            *pair_faces.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    if adj.len() != 8 || adj.values().any(|n| n.len() != 3) || pair_faces.len() != 12 || pair_faces.values().any(|n| *n != 2) {
        return fail(FailureKind::Unsupported, format!("the six faces do not form a hexahedral block ({} corner vertices)", adj.len()));
    }
    let v0 = *adj.keys().next().ok_or(Failure { kind: FailureKind::Internal, reason: "no vertices".into() })?;
    let nb: Vec<EntityId> = adj[&v0].iter().copied().collect();
    let (a, b, c) = (nb[0], nb[1], nb[2]);
    let common = |x: EntityId, y: EntityId| -> Result<EntityId, Failure> {
        let v: Vec<EntityId> = adj[&x].intersection(&adj[&y]).copied().filter(|v| *v != v0).collect();
        match v.as_slice() {
            [one] => Ok(*one),
            _ => fail(FailureKind::Unsupported, "the six faces do not form a hexahedral block"),
        }
    };
    let (c110, c101, c011) = (common(a, b)?, common(a, c)?, common(b, c)?);
    let far: Vec<EntityId> = adj[&c110].iter().copied().filter(|v| *v != a && *v != b).collect();
    let c111 = match far.as_slice() {
        [one] => *one,
        _ => return fail(FailureKind::Unsupported, "the six faces do not form a hexahedral block"),
    };
    let out = [v0, a, b, c110, c, c101, c011, c111];
    let ok = adj[&c111].contains(&c101) && adj[&c111].contains(&c011) && BTreeSet::from(out).len() == 8;
    if !ok {
        return fail(FailureKind::Unsupported, "the six faces do not form a hexahedral block");
    }
    Ok(out)
}

/// Orient the six face grids into one lattice, check that they agree on the shared boundary and
/// fill the positions: boundary from the grids, interior by transfinite interpolation.
///
/// The frame's axes come from the topology; its handedness from the signed volume enclosed by
/// the boundary nodes, a property of the block as a whole (a concave or warped corner cannot
/// mirror it). A negative volume swaps the `j` and `k` axes.
pub(crate) fn assemble(owner: EntityId, grids: &[&NodeGrid], point: &dyn Fn(NodeKey) -> Option<Point3>) -> Result<Block, Failure> {
    if grids.len() != 6 {
        return fail(FailureKind::Unsupported, format!("{} faces; the volume map needs 6", grids.len()));
    }
    let corners: Vec<[EntityId; 4]> = grids.iter().map(|g| grid_corners(g)).collect::<Result<_, _>>()?;
    let verts = frame(&corners)?;
    let mut b = lattice(owner, grids, &corners, &verts, point)?;
    let vol = signed_volume(&b);
    if !vol.is_finite() || vol == 0.0 {
        return fail(FailureKind::Geometry, "the block boundary encloses no volume");
    }
    if vol < 0.0 {
        let mirrored: [EntityId; 8] = std::array::from_fn(|i| verts[(i & 1) | ((i >> 1) & 1) << 2 | ((i >> 2) & 1) << 1]);
        b = lattice(owner, grids, &corners, &mirrored, point)?;
    }
    tfi(&mut b)?;
    check_orientation(&b)?;
    Ok(b)
}

/// Signed volume enclosed by the boundary nodes (divergence theorem over the quads of the six
/// lattice faces, each split along its first diagonal), positive for a right-handed lattice.
/// Fixed summation order.
fn signed_volume(b: &Block) -> f64 {
    let n = b.n;
    let st = b.strides();
    let o = b.pos[0];
    let at = |c: [usize; 3]| sub(b.pos[c[0] * st[0] + c[1] * st[1] + c[2] * st[2]], o);
    let tri = |p: V3, q: V3, r: V3| det(p, q, r);
    let mut sum = 0.0;
    for (fixed, side, au, av) in FACES {
        // `e_au x e_av` is +axis for fixed 0 and 2, -axis for fixed 1; outward is +axis on side 1.
        let sign = if (fixed == 1) == (side == 1) { -1.0 } else { 1.0 };
        for v in 0..n[av] {
            for u in 0..n[au] {
                let corner = |du: usize, dv: usize| {
                    let mut c = [0; 3];
                    c[fixed] = side * n[fixed];
                    c[au] = u + du;
                    c[av] = v + dv;
                    at(c)
                };
                let (p00, p10, p11, p01) = (corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1));
                sum += sign * (tri(p00, p10, p11) + tri(p00, p11, p01));
            }
        }
    }
    sum / 6.0
}

/// The lattice for the frame `verts`: boundary keys and positions from the grids (checked for
/// agreement on shared nodes), interior keys assigned, interior positions not yet computed.
fn lattice(owner: EntityId, grids: &[&NodeGrid], corners: &[[EntityId; 4]], verts: &[EntityId; 8], point: &dyn Fn(NodeKey) -> Option<Point3>) -> Result<Block, Failure> {
    let by_corners: BTreeMap<[EntityId; 4], usize> = corners
        .iter()
        .enumerate()
        .map(|(i, c)| {
            let mut s = *c;
            s.sort_unstable();
            (s, i)
        })
        .collect();
    if by_corners.len() != 6 {
        return fail(FailureKind::Unsupported, "two faces share all four corners");
    }
    // For each lattice face: its grid, orientation and `(nu, nv)` in lattice terms.
    let mut views: Vec<(usize, Orient, u32, u32)> = Vec::with_capacity(6);
    for (fixed, side, au, av) in FACES {
        let corner = |bu: usize, bv: usize| {
            let mut bits = [0usize; 3];
            bits[fixed] = side;
            bits[au] = bu;
            bits[av] = bv;
            verts[corner_index(bits)]
        };
        let (ca, cb, cd, ce) = (corner(0, 0), corner(1, 0), corner(0, 1), corner(1, 1));
        let mut key = [ca, cb, cd, ce];
        key.sort_unstable();
        let fi = *by_corners.get(&key).ok_or(Failure { kind: FailureKind::Unsupported, reason: "no face over a lattice face".into() })?;
        let o = orient(&corners[fi], ca, cb, cd).ok_or(Failure { kind: FailureKind::Unsupported, reason: "a face grid does not match the block corners".into() })?;
        let g = grids[fi];
        let (nu, nv) = if o.u_along_i { (g.ni, g.nj) } else { (g.nj, g.ni) };
        views.push((fi, o, nu, nv));
    }
    // views: 0,1 are i-faces (u=j, v=k); 2,3 j-faces (u=i, v=k); 4,5 k-faces (u=i, v=j).
    let n = [views[2].2 as usize - 1, views[0].2 as usize - 1, views[0].3 as usize - 1];
    let expect = |a: usize, b: usize| (a as u32 + 1, b as u32 + 1);
    for (idx, (fixed, ..)) in FACES.iter().enumerate() {
        let (au, av) = match fixed {
            0 => (n[1], n[2]),
            1 => (n[0], n[2]),
            _ => (n[0], n[1]),
        };
        if (views[idx].2, views[idx].3) != expect(au, av) {
            return fail(FailureKind::Constraint, format!("faces disagree on the interval counts: lattice {n:?}, a face has {} x {} nodes", views[idx].2, views[idx].3));
        }
    }
    if n.contains(&0) || (n[0] + 1).checked_mul(n[1] + 1).and_then(|x| x.checked_mul(n[2] + 1)).is_none_or(|t| t >= 1 << 31) {
        return fail(FailureKind::Unsupported, format!("lattice {n:?} is out of range"));
    }
    let st = strides(n);
    let total = (n[0] + 1) * (n[1] + 1) * (n[2] + 1);
    let sentinel = NodeKey::new(owner, u32::MAX);
    let mut keys = vec![sentinel; total];
    for (idx, (fixed, side, au, av)) in FACES.iter().enumerate() {
        let (fi, o, nu, nv) = &views[idx];
        let g = grids[*fi];
        for v in 0..*nv {
            for u in 0..*nu {
                let key = grid_at(g, o, u, v).ok_or(Failure { kind: FailureKind::Internal, reason: "grid index out of range".into() })?;
                let slot = side * n[*fixed] * st[*fixed] + u as usize * st[*au] + v as usize * st[*av];
                if keys[slot] != sentinel && keys[slot] != key {
                    return fail(FailureKind::Constraint, format!("faces disagree on a shared node: {:?} against {key:?}", keys[slot]));
                }
                keys[slot] = key;
            }
        }
    }
    // Interior keys, and the check that every boundary slot got a key and no key repeats.
    let mut pos: Vec<V3> = vec![[0.0; 3]; total];
    let (mut next, mut seen) = (0u32, BTreeSet::new());
    for k in 0..=n[2] {
        for j in 0..=n[1] {
            for i in 0..=n[0] {
                let slot = i * st[0] + j * st[1] + k * st[2];
                if is_boundary([i, j, k], n) {
                    let key = keys[slot];
                    if key == sentinel {
                        return fail(FailureKind::Internal, "a boundary node got no key");
                    }
                    if !seen.insert(key) {
                        return fail(FailureKind::Constraint, format!("node {key:?} occupies two lattice positions"));
                    }
                    pos[slot] = pt(point(key).ok_or(Failure { kind: FailureKind::Internal, reason: format!("node {key:?} missing") })?);
                } else {
                    keys[slot] = NodeKey::new(owner, next);
                    next += 1;
                }
            }
        }
    }
    Ok(Block { n, keys, pos })
}

/// Normalized arc-length fractions along the four lattice lines of direction `axis`, averaged.
fn fractions(b: &Block, axis: usize) -> Result<Vec<f64>, Failure> {
    let st = b.strides();
    let (o1, o2) = ((axis + 1) % 3, (axis + 2) % 3);
    let m = b.n[axis];
    let mut avg = vec![0.0; m + 1];
    for (c1, c2) in [(0, 0), (b.n[o1], 0), (0, b.n[o2]), (b.n[o1], b.n[o2])] {
        let base = c1 * st[o1] + c2 * st[o2];
        let mut acc = vec![0.0; m + 1];
        for q in 1..=m {
            let (p, r) = (b.pos[base + (q - 1) * st[axis]], b.pos[base + q * st[axis]]);
            let d = sub(r, p);
            acc[q] = acc[q - 1] + (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        }
        let total = acc[m];
        if !(total.is_finite() && total > 0.0) {
            return fail(FailureKind::Geometry, "a block edge has zero length");
        }
        for q in 0..=m {
            avg[q] += acc[q] / total / 4.0;
        }
    }
    avg[0] = 0.0;
    avg[m] = 1.0;
    Ok(avg)
}

/// Fill the interior by 3-D transfinite interpolation (Gordon-Hall) with tensor parameters.
#[allow(clippy::needless_range_loop)] // lattice indices address several arrays at once
fn tfi(b: &mut Block) -> Result<(), Failure> {
    let n = b.n;
    if n.iter().any(|m| *m < 2) {
        return Ok(());
    }
    let st = b.strides();
    let params = [fractions(b, 0)?, fractions(b, 1)?, fractions(b, 2)?];
    let pos = &b.pos;
    let p = |i: usize, j: usize, k: usize| pos[i * st[0] + j * st[1] + k * st[2]];
    let mut cp = [[0.0; 3]; 8];
    for (c, slot) in cp.iter_mut().enumerate() {
        *slot = p((c & 1) * n[0], ((c >> 1) & 1) * n[1], ((c >> 2) & 1) * n[2]);
    }
    let lin = |w: &[f64], q: &[V3]| -> V3 {
        let mut r = [0.0; 3];
        for (wi, qi) in w.iter().zip(q) {
            for d in 0..3 {
                r[d] += wi * qi[d];
            }
        }
        r
    };
    let slab = |k: usize| -> Vec<V3> {
        let mut out = Vec::with_capacity((n[0] - 1) * (n[1] - 1));
        let (c, z) = (1.0 - params[2][k], params[2][k]);
        for j in 1..n[1] {
            let (bb, y) = (1.0 - params[1][j], params[1][j]);
            for i in 1..n[0] {
                let (a, x) = (1.0 - params[0][i], params[0][i]);
                let u1 = lin(&[a, x], &[p(0, j, k), p(n[0], j, k)]);
                let u2 = lin(&[bb, y], &[p(i, 0, k), p(i, n[1], k)]);
                let u3 = lin(&[c, z], &[p(i, j, 0), p(i, j, n[2])]);
                let u12 = lin(&[a * bb, x * bb, a * y, x * y], &[p(0, 0, k), p(n[0], 0, k), p(0, n[1], k), p(n[0], n[1], k)]);
                let u13 = lin(&[a * c, x * c, a * z, x * z], &[p(0, j, 0), p(n[0], j, 0), p(0, j, n[2]), p(n[0], j, n[2])]);
                let u23 = lin(&[bb * c, y * c, bb * z, y * z], &[p(i, 0, 0), p(i, n[1], 0), p(i, 0, n[2]), p(i, n[1], n[2])]);
                let w = [a * bb * c, x * bb * c, a * y * c, x * y * c, a * bb * z, x * bb * z, a * y * z, x * y * z];
                let u123 = lin(&w, &cp);
                let mut r = [0.0; 3];
                for d in 0..3 {
                    r[d] = u1[d] + u2[d] + u3[d] - u12[d] - u13[d] - u23[d] + u123[d];
                }
                out.push(r);
            }
        }
        out
    };
    let interior = (n[0] - 1) * (n[1] - 1) * (n[2] - 1);
    let slabs: Vec<Vec<V3>> = if interior >= PAR_MIN { (1..n[2]).into_par_iter().map(slab).collect() } else { (1..n[2]).map(slab).collect() };
    for (s, vals) in slabs.into_iter().enumerate() {
        let k = s + 1;
        let mut it = vals.into_iter();
        for j in 1..n[1] {
            for i in 1..n[0] {
                if let Some(v) = it.next() {
                    b.pos[i * st[0] + j * st[1] + k * st[2]] = v;
                }
            }
        }
    }
    Ok(())
}

/// Every hexahedron must have a positive Jacobian determinant at all eight corners.
fn check_orientation(b: &Block) -> Result<(), Failure> {
    let n = b.n;
    let st = b.strides();
    let bad_in_slab = |k: usize| -> Option<[usize; 3]> {
        for j in 0..n[1] {
            for i in 0..n[0] {
                for c in 0..8usize {
                    let bits = [c & 1, (c >> 1) & 1, (c >> 2) & 1];
                    let at = [i + bits[0], j + bits[1], k + bits[2]];
                    let slot = |q: [usize; 3]| q[0] * st[0] + q[1] * st[1] + q[2] * st[2];
                    let p0 = b.pos[slot(at)];
                    let nbr = |a: usize| {
                        let mut q = at;
                        q[a] = if bits[a] == 1 { q[a] - 1 } else { q[a] + 1 };
                        sub(b.pos[slot(q)], p0)
                    };
                    let mut d = det(nbr(0), nbr(1), nbr(2));
                    if (bits[0] + bits[1] + bits[2]) % 2 == 1 {
                        d = -d;
                    }
                    if d.is_nan() || d <= 0.0 {
                        return Some([i, j, k]);
                    }
                }
            }
        }
        None
    };
    let cells = n[0] * n[1] * n[2];
    let first = if cells >= PAR_MIN {
        (0..n[2]).into_par_iter().find_map_first(bad_in_slab)
    } else {
        (0..n[2]).find_map(bad_in_slab)
    };
    match first {
        Some(c) => fail(FailureKind::Quality, format!("the transfinite map inverts or collapses the hexahedron {c:?}")),
        None => Ok(()),
    }
}

/// The patch of a block: new interior nodes, every boundary node as a reference, one hex per
/// lattice cell with the template corner order (bottom ring counter-clockwise, then the top).
pub(crate) fn patch(target: Target, b: &Block) -> Patch {
    let n = b.n;
    let st = b.strides();
    let total = b.keys.len();
    let interior = (n[0] - 1) * (n[1] - 1) * (n[2] - 1);
    let mut local = vec![0u32; total];
    let mut p = Patch::new(target);
    p.nodes.reserve(interior);
    p.refs.reserve(total - interior);
    for k in 0..=n[2] {
        for j in 0..=n[1] {
            for i in 0..=n[0] {
                let slot = i * st[0] + j * st[1] + k * st[2];
                if is_boundary([i, j, k], n) {
                    local[slot] = (interior + p.refs.len()) as u32;
                    p.refs.push(b.keys[slot]);
                } else {
                    local[slot] = p.nodes.len() as u32;
                    let q = b.pos[slot];
                    p.nodes.push(PatchNode::new(b.keys[slot], Point3::new(q[0], q[1], q[2]), Param::None));
                }
            }
        }
    }
    let mut el = Elements::with_capacity(n[0] * n[1] * n[2], 8);
    for k in 0..n[2] {
        for j in 0..n[1] {
            for i in 0..n[0] {
                let at = |di: usize, dj: usize, dk: usize| local[(i + di) * st[0] + (j + dj) * st[1] + (k + dk) * st[2]];
                el.push(
                    ElemShape::Cell(CellShape::Hex),
                    &[at(0, 0, 0), at(1, 0, 0), at(1, 1, 0), at(0, 1, 0), at(0, 0, 1), at(1, 0, 1), at(1, 1, 1), at(0, 1, 1)],
                );
            }
        }
    }
    p.elements = el;
    p
}
