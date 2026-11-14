//! Refinement templates: for every cell shape the new nodes (as the set of parent corners each
//! one spans), the children (as lists of template nodes) and, for every child face, the parent
//! face it lies on.
//!
//! A template node spanning one corner is that corner, two corners an edge midpoint, four a quad
//! face centre (or the cell centre of a quad), eight the cell centre of a hex. The straight
//! position of a node is the mean of the corners it spans.

use std::sync::OnceLock;

use sm_mesh_api::CellShape;

/// One child of a template.
#[derive(Debug)]
pub(crate) struct Child {
    /// Shape of the child.
    pub shape: CellShape,
    /// Template nodes in the template order of `shape`.
    pub nodes: Vec<u8>,
}

/// The refinement template of one shape.
#[derive(Debug)]
pub(crate) struct Template {
    /// Parent corners spanned by each node, ascending; the first `corners` nodes are the corners.
    pub nodes: Vec<Vec<u8>>,
    /// Children per variant (only the tetrahedron has three: one per octahedron diagonal).
    pub variants: Vec<Vec<Child>>,
    /// For each node: the bit set of parent local faces that contain it.
    pub nmask: Vec<u8>,
}

struct NodeSet {
    sets: Vec<Vec<u8>>,
}

impl NodeSet {
    fn corners(n: u8) -> Self {
        Self { sets: (0..n).map(|c| vec![c]).collect() }
    }
    fn intern(&mut self, mut s: Vec<u8>) -> u8 {
        s.sort_unstable();
        s.dedup();
        if let Some(i) = self.sets.iter().position(|x| *x == s) {
            return i as u8;
        }
        self.sets.push(s);
        (self.sets.len() - 1) as u8
    }
}

fn masks(shape: CellShape, nodes: &[Vec<u8>]) -> Vec<u8> {
    nodes
        .iter()
        .map(|s| {
            let mut m = 0u8;
            for (j, lf) in shape.local_faces().iter().enumerate() {
                if s.iter().all(|c| lf.contains(c)) {
                    m |= 1 << j;
                }
            }
            m
        })
        .collect()
}

fn finish(shape: CellShape, ns: NodeSet, variants: Vec<Vec<(CellShape, Vec<u8>)>>) -> Template {
    let nm = masks(shape, &ns.sets);
    let variants = variants
        .into_iter()
        .map(|v| {
            v.into_iter()
                .map(|(cs, nodes)| Child { shape: cs, nodes })
                .collect()
        })
        .collect();
    Template { nodes: ns.sets, variants, nmask: nm }
}

fn tri() -> Template {
    let mut ns = NodeSet::corners(3);
    let (m01, m12, m02) = (ns.intern(vec![0, 1]), ns.intern(vec![1, 2]), ns.intern(vec![0, 2]));
    let ch = [vec![0, m01, m02], vec![m01, 1, m12], vec![m02, m12, 2], vec![m01, m12, m02]];
    finish(CellShape::Tri, ns, vec![ch.into_iter().map(|n| (CellShape::Tri, n)).collect()])
}

fn quad() -> Template {
    let mut ns = NodeSet::corners(4);
    let m: Vec<u8> = (0..4u8).map(|i| ns.intern(vec![i, (i + 1) % 4])).collect();
    let o = ns.intern(vec![0, 1, 2, 3]);
    let ch = [vec![0, m[0], o, m[3]], vec![m[0], 1, m[1], o], vec![o, m[1], 2, m[2]], vec![m[3], o, m[2], 3]];
    finish(CellShape::Quad, ns, vec![ch.into_iter().map(|n| (CellShape::Quad, n)).collect()])
}

fn tet() -> Template {
    let mut ns = NodeSet::corners(4);
    let m01 = ns.intern(vec![0, 1]);
    let m12 = ns.intern(vec![1, 2]);
    let m02 = ns.intern(vec![0, 2]);
    let m03 = ns.intern(vec![0, 3]);
    let m23 = ns.intern(vec![2, 3]);
    let m13 = ns.intern(vec![1, 3]);
    let corner = [vec![0, m01, m02, m03], vec![m01, 1, m12, m13], vec![m02, m12, 2, m23], vec![m03, m13, m23, 3]];
    let diag = [(m01, m23), (m02, m13), (m03, m12)];
    let mut variants = Vec::new();
    for d in 0..3 {
        let (p, q) = diag[d];
        let (o1, o2) = (diag[(d + 1) % 3], diag[(d + 2) % 3]);
        let cyc = [o1.0, o2.0, o1.1, o2.1];
        let mut v: Vec<(CellShape, Vec<u8>)> = corner.iter().map(|n| (CellShape::Tet, n.clone())).collect();
        for i in 0..4 {
            v.push((CellShape::Tet, vec![p, q, cyc[i], cyc[(i + 1) % 4]]));
        }
        variants.push(v);
    }
    finish(CellShape::Tet, ns, variants)
}

/// The three octahedron diagonals of the tetrahedron template, as node pairs (variant `d` cuts
/// along `TET_DIAGONALS[d]`).
pub(crate) fn tet_diagonals() -> [(u8, u8); 3] {
    static D: OnceLock<[(u8, u8); 3]> = OnceLock::new();
    *D.get_or_init(|| {
        let t = template(CellShape::Tet);
        let find = |a: &[u8]| t.nodes.iter().position(|s| s == a).map(|i| i as u8).unwrap_or(0);
        [(find(&[0, 1]), find(&[2, 3])), (find(&[0, 2]), find(&[1, 3])), (find(&[0, 3]), find(&[1, 2]))]
    })
}

fn hex() -> Template {
    let mut ns = NodeSet::corners(8);
    let cid = |x: usize, y: usize, z: usize| -> u8 {
        (match (x, y) {
            (0, 0) => 0,
            (1, 0) => 1,
            (1, 1) => 2,
            _ => 3,
        } + 4 * z) as u8
    };
    let axis = |i: usize| -> Vec<usize> {
        match i {
            0 => vec![0],
            1 => vec![0, 1],
            _ => vec![1],
        }
    };
    let mut grid = [[[0u8; 3]; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                let mut s = Vec::new();
                for &x in &axis(i) {
                    for &y in &axis(j) {
                        for &z in &axis(k) {
                            s.push(cid(x, y, z));
                        }
                    }
                }
                grid[i][j][k] = ns.intern(s);
            }
        }
    }
    let off = |t: usize| -> (usize, usize, usize) {
        let (x, y) = [(0, 0), (1, 0), (1, 1), (0, 1)][t % 4];
        (x, y, t / 4)
    };
    let mut ch = Vec::new();
    for cz in 0..2 {
        for cy in 0..2 {
            for cx in 0..2 {
                let nodes = (0..8)
                    .map(|t| {
                        let (dx, dy, dz) = off(t);
                        grid[cx + dx][cy + dy][cz + dz]
                    })
                    .collect();
                ch.push((CellShape::Hex, nodes));
            }
        }
    }
    finish(CellShape::Hex, ns, vec![ch])
}

fn wedge() -> Template {
    let mut ns = NodeSet::corners(6);
    let tri_span: [Vec<u8>; 6] = [vec![0], vec![1], vec![2], vec![0, 1], vec![1, 2], vec![0, 2]];
    let lvl: [Vec<u8>; 3] = [vec![0], vec![0, 1], vec![1]];
    let mut g = [[0u8; 3]; 6];
    for (t, row) in g.iter_mut().enumerate() {
        for (l, slot) in row.iter_mut().enumerate() {
            let mut s = Vec::new();
            for &c in &tri_span[t] {
                for &z in &lvl[l] {
                    s.push(c + 3 * z);
                }
            }
            *slot = ns.intern(s);
        }
    }
    let tris = [[0usize, 3, 5], [3, 1, 4], [5, 4, 2], [3, 4, 5]];
    let mut ch = Vec::new();
    for l in 0..2 {
        for t in &tris {
            let mut nodes: Vec<u8> = t.iter().map(|&a| g[a][l]).collect();
            nodes.extend(t.iter().map(|&a| g[a][l + 1]));
            ch.push((CellShape::Wedge, nodes));
        }
    }
    finish(CellShape::Wedge, ns, vec![ch])
}

fn pyramid() -> Template {
    let mut ns = NodeSet::corners(5);
    let m: Vec<u8> = (0..4u8).map(|i| ns.intern(vec![i, (i + 1) % 4])).collect();
    let a: Vec<u8> = (0..4u8).map(|i| ns.intern(vec![i, 4])).collect();
    let o = ns.intern(vec![0, 1, 2, 3]);
    let mut ch = Vec::new();
    for i in 0..4usize {
        ch.push((CellShape::Pyramid, vec![i as u8, m[i], o, m[(i + 3) % 4], a[i]]));
    }
    ch.push((CellShape::Pyramid, vec![a[0], a[1], a[2], a[3], 4]));
    ch.push((CellShape::Pyramid, vec![a[0], a[3], a[2], a[1], o]));
    for i in 0..4usize {
        ch.push((CellShape::Tet, vec![a[i], a[(i + 1) % 4], m[i], o]));
    }
    finish(CellShape::Pyramid, ns, vec![ch])
}

/// The template of `shape` (panics for shapes without one; callers check first).
pub(crate) fn template(shape: CellShape) -> &'static Template {
    static T: OnceLock<[Template; 6]> = OnceLock::new();
    let t = T.get_or_init(|| [tet(), hex(), wedge(), pyramid(), tri(), quad()]);
    match shape {
        CellShape::Tet => &t[0],
        CellShape::Hex => &t[1],
        CellShape::Wedge => &t[2],
        CellShape::Pyramid => &t[3],
        CellShape::Tri => &t[4],
        CellShape::Quad => &t[5],
        _ => panic!("no refinement template for {shape:?}"),
    }
}

/// True if `shape` has a template.
pub(crate) fn supported(shape: CellShape) -> bool {
    matches!(
        shape,
        CellShape::Tet | CellShape::Hex | CellShape::Wedge | CellShape::Pyramid | CellShape::Tri | CellShape::Quad
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn child_counts_and_node_counts() {
        let want = [
            (CellShape::Tet, 8, 10),
            (CellShape::Hex, 8, 27),
            (CellShape::Wedge, 8, 18),
            (CellShape::Pyramid, 10, 14),
            (CellShape::Tri, 4, 6),
            (CellShape::Quad, 4, 9),
        ];
        for (s, children, nodes) in want {
            let t = template(s);
            assert_eq!(t.variants[0].len(), children, "{s:?}");
            assert_eq!(t.nodes.len(), nodes, "{s:?}");
        }
        assert_eq!(template(CellShape::Tet).variants.len(), 3);
    }
}
