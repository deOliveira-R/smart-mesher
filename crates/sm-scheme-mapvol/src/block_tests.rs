//! Unit tests of the block assembly over synthetic face grids: any rotation or reflection of any
//! grid, any face order and any vertex-id assignment gives the same hexahedra.

use crate::block::{Block, assemble, patch};
use sm_core::{EntityId, EntityKind, Point3};
use sm_scheme_api::{ElemShape, FailureKind, NodeGrid, NodeKey, Target};
use std::collections::{BTreeMap, BTreeSet};

type V3 = [f64; 3];

fn eid(kind: EntityKind, serial: u64) -> EntityId {
    EntityId::new(kind, serial).expect("id")
}

/// A synthetic hex block: lattice `n`, parameters per axis, an arbitrary map from the unit cube.
struct Synth {
    n: [usize; 3],
    params: [Vec<f64>; 3],
    map: Box<dyn Fn(V3) -> V3>,
}

impl Synth {
    fn lattice_pos(&self, c: [usize; 3]) -> V3 {
        (self.map)([self.params[0][c[0]], self.params[1][c[1]], self.params[2][c[2]]])
    }

    /// Key of the lattice node `c`: corners are vertices (ids permuted by `vperm`), edge nodes
    /// belong to the lattice edge (index along +axis), face nodes to the lattice face.
    fn key(&self, c: [usize; 3], vperm: &[u64; 8]) -> NodeKey {
        let ext: Vec<usize> = (0..3).filter(|a| c[*a] == 0 || c[*a] == self.n[*a]).collect();
        match ext.len() {
            3 => {
                let bits = (0..3).map(|a| usize::from(c[a] == self.n[a]) << a).sum::<usize>();
                NodeKey::vertex(eid(EntityKind::TopoVertex, 10 + vperm[bits]))
            }
            2 => {
                let axis = (0..3).find(|a| !ext.contains(a)).expect("axis");
                let (o1, o2) = ((axis + 1) % 3, (axis + 2) % 3);
                let serial = 100 + axis as u64 * 4 + usize::from(c[o1] != 0) as u64 + 2 * usize::from(c[o2] != 0) as u64;
                NodeKey::new(eid(EntityKind::TopoEdge, serial), c[axis] as u32)
            }
            _ => {
                let axis = ext[0];
                let serial = 200 + axis as u64 * 2 + usize::from(c[axis] != 0) as u64;
                let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
                NodeKey::new(eid(EntityKind::TopoFace, serial), (c[u] + (self.n[u] + 1) * c[v]) as u32)
            }
        }
    }

    /// The six face grids; `dihedral[f]` in `0..8` picks the transpose / flip of face `f`, `order`
    /// the face order.
    fn grids(&self, dihedral: [u8; 6], order: [usize; 6], vperm: &[u64; 8]) -> Vec<NodeGrid> {
        let faces = [(0, 0, 1, 2), (0, 1, 1, 2), (1, 0, 2, 0), (1, 1, 2, 0), (2, 0, 0, 1), (2, 1, 0, 1)];
        let mut out = Vec::new();
        for f in order {
            let (fixed, side, a, b) = faces[f];
            let (nu, nv) = (self.n[a] + 1, self.n[b] + 1);
            let d = dihedral[f];
            let (flip_u, flip_v, transpose) = (d & 1 != 0, d & 2 != 0, d & 4 != 0);
            let (ni, nj) = if transpose { (nv, nu) } else { (nu, nv) };
            let mut nodes = Vec::with_capacity(ni * nj);
            for gj in 0..nj {
                for gi in 0..ni {
                    let (mut u, mut v) = if transpose { (gj, gi) } else { (gi, gj) };
                    if flip_u {
                        u = nu - 1 - u;
                    }
                    if flip_v {
                        v = nv - 1 - v;
                    }
                    let mut c = [0; 3];
                    c[fixed] = side * self.n[fixed];
                    c[a] = u;
                    c[b] = v;
                    nodes.push(self.key(c, vperm));
                }
            }
            out.push(NodeGrid::new(ni as u32, nj as u32, nodes));
        }
        out
    }

    fn points(&self, vperm: &[u64; 8]) -> BTreeMap<NodeKey, Point3> {
        let mut m = BTreeMap::new();
        for k in 0..=self.n[2] {
            for j in 0..=self.n[1] {
                for i in 0..=self.n[0] {
                    let boundary = [i, j, k].iter().zip(self.n).any(|(c, n)| *c == 0 || *c == n);
                    if boundary {
                        let p = self.lattice_pos([i, j, k]);
                        m.insert(self.key([i, j, k], vperm), Point3::new(p[0], p[1], p[2]));
                    }
                }
            }
        }
        m
    }
}

fn cells_of(p: &sm_scheme_api::Patch, pos: &BTreeMap<NodeKey, Point3>) -> BTreeSet<Vec<[i64; 3]>> {
    let at = |l: u32| {
        let key = p.key(l).expect("key");
        match p.nodes.get(l as usize) {
            Some(n) => n.point,
            None => pos[&key],
        }
    };
    p.elements
        .iter()
        .map(|(shape, c)| {
            assert_eq!(shape, ElemShape::Cell(sm_mesh_api::CellShape::Hex));
            let mut v: Vec<[i64; 3]> = c.iter().map(|l| {
                let q = at(*l);
                [(q.x * 1e9).round() as i64, (q.y * 1e9).round() as i64, (q.z * 1e9).round() as i64]
            }).collect();
            v.sort_unstable();
            v
        })
        .collect()
}

fn run(s: &Synth, dihedral: [u8; 6], order: [usize; 6], vperm: [u64; 8]) -> (Block, BTreeSet<Vec<[i64; 3]>>) {
    let grids = s.grids(dihedral, order, &vperm);
    let refs: Vec<&NodeGrid> = grids.iter().collect();
    let pos = s.points(&vperm);
    let owner = eid(EntityKind::TopoVolume, 1);
    let b = assemble(owner, &refs, &|k| pos.get(&k).copied()).expect("assemble");
    let p = patch(Target::Volume(owner), &b);
    let cells = cells_of(&p, &pos);
    (b, cells)
}

fn graded(n: usize, ratio: f64) -> Vec<f64> {
    let mut acc = vec![0.0];
    let mut h = 1.0;
    for _ in 0..n {
        acc.push(acc.last().copied().unwrap_or(0.0) + h);
        h *= ratio;
    }
    let t = acc[n];
    acc.iter().map(|a| a / t).collect()
}

fn affine(o: V3, a: V3, b: V3, c: V3) -> Box<dyn Fn(V3) -> V3> {
    Box::new(move |p| {
        let mut r = o;
        for d in 0..3 {
            r[d] += p[0] * a[d] + p[1] * b[d] + p[2] * c[d];
        }
        r
    })
}

fn skew() -> Synth {
    Synth {
        n: [3, 4, 2],
        params: [graded(3, 1.0), graded(4, 1.5), graded(2, 1.0)],
        map: affine([1.0, -2.0, 0.5], [3.0, 0.5, 0.0], [0.4, 2.0, 0.3], [0.1, -0.2, 1.5]),
    }
}

const IDV: [u64; 8] = [0, 1, 2, 3, 4, 5, 6, 7];

#[test]
fn affine_block_with_graded_spacing_is_exact() {
    // Consistent grading on parallel edges and an affine map: the TFI reproduces the affine
    // lattice (position = map of the arc-length parameters).
    let s = skew();
    let (b, cells) = run(&s, [0; 6], [0, 1, 2, 3, 4, 5], IDV);
    assert_eq!(b.n, [3, 4, 2]);
    assert_eq!(cells.len(), 3 * 4 * 2);
    let want: BTreeSet<Vec<[i64; 3]>> = {
        let mut set = BTreeSet::new();
        for k in 0..2 {
            for j in 0..4 {
                for i in 0..3 {
                    let mut v = Vec::new();
                    for c in 0..8usize {
                        let q = s.lattice_pos([i + (c & 1), j + ((c >> 1) & 1), k + ((c >> 2) & 1)]);
                        v.push([(q[0] * 1e9).round() as i64, (q[1] * 1e9).round() as i64, (q[2] * 1e9).round() as i64]);
                    }
                    v.sort_unstable();
                    set.insert(v);
                }
            }
        }
        set
    };
    assert_eq!(cells, want);
}

#[test]
fn every_dihedral_face_order_and_vertex_numbering_gives_the_same_cells() {
    let s = skew();
    let (_, base) = run(&s, [0; 6], [0, 1, 2, 3, 4, 5], IDV);
    let orders = [[5, 4, 3, 2, 1, 0], [2, 0, 4, 1, 5, 3], [3, 5, 1, 4, 0, 2]];
    let perms: [[u64; 8]; 4] = [IDV, [7, 6, 5, 4, 3, 2, 1, 0], [3, 7, 0, 5, 1, 6, 2, 4], [5, 2, 6, 1, 7, 0, 4, 3]];
    let mut seed = 12345u32;
    let mut next = || {
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        (seed >> 24) as u8
    };
    let mut runs = 0;
    for order in orders {
        for perm in perms {
            for _ in 0..4 {
                let d = [next() % 8, next() % 8, next() % 8, next() % 8, next() % 8, next() % 8];
                let (_, cells) = run(&s, d, order, perm);
                assert_eq!(cells, base, "dihedral {d:?}, order {order:?}, vertices {perm:?}");
                runs += 1;
            }
        }
    }
    assert_eq!(runs, 48);
    // All eight dihedral maps of one face.
    for d in 0..8u8 {
        let (_, cells) = run(&s, [d, 0, 0, 0, 0, 0], [0, 1, 2, 3, 4, 5], IDV);
        assert_eq!(cells, base, "dihedral {d} on face 0");
    }
}

#[test]
fn interior_keys_and_counts() {
    let s = skew();
    let (b, _) = run(&s, [0; 6], [0, 1, 2, 3, 4, 5], IDV);
    let owner = eid(EntityKind::TopoVolume, 1);
    let interior: Vec<NodeKey> = b.keys.iter().copied().filter(|k| k.owner == owner).collect();
    assert_eq!(interior.len(), 6);
    let idx: Vec<u32> = interior.iter().map(|k| k.index).collect();
    assert_eq!(idx, (0..6).collect::<Vec<u32>>());
    let p = patch(Target::Volume(owner), &b);
    assert_eq!(p.nodes.len(), 6);
    assert_eq!(p.refs.len(), 4 * 5 * 3 - 6);
}

#[test]
fn a_reflected_block_is_made_right_handed() {
    // A mirror map flips the handedness of the corner at the lowest vertex; the frame swaps two
    // axes so every hex still has a positive Jacobian.
    let s = Synth {
        n: [2, 3, 2],
        params: [graded(2, 1.0), graded(3, 1.0), graded(2, 1.0)],
        map: affine([0.0; 3], [-1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 1.0]),
    };
    let (_, cells) = run(&s, [0; 6], [0, 1, 2, 3, 4, 5], IDV);
    assert_eq!(cells.len(), 12);
}

#[test]
fn faces_that_do_not_close_a_hex_block_are_refused() {
    let s = skew();
    let grids = s.grids([0; 6], [0, 1, 2, 3, 4, 5], &IDV);
    let pos = s.points(&IDV);
    let owner = eid(EntityKind::TopoVolume, 1);
    let point = |k: NodeKey| pos.get(&k).copied();
    // Five faces.
    let five: Vec<&NodeGrid> = grids.iter().take(5).collect();
    let e = assemble(owner, &five, &point).expect_err("five");
    assert_eq!(e.kind, FailureKind::Unsupported);
    // Two faces disagree on the interval count of their shared edge.
    let mut bad = grids.clone();
    let g = &bad[4];
    let (ni, nj) = (g.ni, g.nj);
    let mut nodes = g.nodes.clone();
    nodes.truncate(((ni - 1) * nj) as usize);
    bad[4] = NodeGrid::new(ni - 1, nj, nodes);
    let refs: Vec<&NodeGrid> = bad.iter().collect();
    assert!(assemble(owner, &refs, &point).is_err());
}

#[test]
fn an_inverted_interior_is_a_quality_failure() {
    // Boundary faces of a unit cube whose top face is pushed through the bottom: the TFI folds.
    let s = Synth {
        n: [4, 4, 4],
        params: [graded(4, 1.0), graded(4, 1.0), graded(4, 1.0)],
        map: Box::new(|p| {
            let (x, y, z) = (p[0], p[1], p[2]);
            // z collapses below the bottom in the middle of the top face; boundary stays a graph.
            let bump = 4.0 * x * (1.0 - x) * y * (1.0 - y);
            [x, y, if z > 0.99 { z - 5.0 * bump } else { z }]
        }),
    };
    let grids = s.grids([0; 6], [0, 1, 2, 3, 4, 5], &IDV);
    let refs: Vec<&NodeGrid> = grids.iter().collect();
    let pos = s.points(&IDV);
    let owner = eid(EntityKind::TopoVolume, 1);
    let e = assemble(owner, &refs, &|k| pos.get(&k).copied()).expect_err("folded");
    assert_eq!(e.kind, FailureKind::Quality, "{}", e.reason);
}
