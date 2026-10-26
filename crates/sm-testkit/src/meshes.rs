//! Small 3D meshes built through `sm-mesh-api`'s [`MeshBuild`] (reference implementor), each with
//! known cell volumes.
//!
//! All meshes are classified like a CAD solid so that they pass `sm_mesh_api::validate`: the
//! cells lie on `TopoVolume#1`; boundary faces are grouped by plane into `TopoFace#k`; a vertex
//! touching one group lies on that face, two groups on a `TopoEdge`, three or more on its own
//! `TopoVertex`, and an interior vertex on the volume. Ids come from a fresh [`SerialAlloc`] per
//! fixture, so every fixture is reproducible.

use core::f64::consts::TAU;
use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh, hex_block};
use sm_mesh_api::{
    CellShape, Classification, FaceIx, MeshBuild, MeshEntityRef, MeshError, MeshRead, Param, Result, SerialAlloc, VertexIx,
};
use std::collections::{BTreeMap, BTreeSet};

/// How to add one cell.
#[derive(Clone, Debug, PartialEq)]
pub enum CellSpec {
    /// A template cell from point indices in the template order of the shape.
    Template(CellShape, Vec<usize>),
    /// A general polyhedron from outward-oriented face loops of point indices.
    Poly(Vec<Vec<usize>>),
}

/// A mesh with its analytic expectations.
#[derive(Clone, Debug)]
pub struct MeshFixture {
    /// Fixture name.
    pub name: &'static str,
    /// The finished mesh.
    pub mesh: ReferenceMesh,
    /// Exact volume of each cell, in cell order.
    pub cell_volumes: Vec<f64>,
    /// Exact total volume.
    pub volume: f64,
    /// Exact area of the boundary surface (the one-cell faces).
    pub area: f64,
    /// Number of faces with one cell.
    pub boundary_faces: usize,
}

fn model(kind: EntityKind, serial: usize) -> Result<Classification> {
    EntityId::new(kind, serial as u64).map(Classification::on).ok_or(MeshError::SerialOverflow)
}

/// Unoriented plane key of a face: unit normal (sign-normalized) and offset, rounded.
fn plane_key(pts: &[[f64; 3]]) -> [i64; 4] {
    let mut n = [0.0; 3];
    for i in 0..pts.len() {
        let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
        n[0] += (a[1] - b[1]) * (a[2] + b[2]);
        n[1] += (a[2] - b[2]) * (a[0] + b[0]);
        n[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len > 0.0 {
        n = [n[0] / len, n[1] / len, n[2] / len];
    }
    if n.iter().find(|c| c.abs() > 1e-9).is_some_and(|c| *c < 0.0) {
        n = [-n[0], -n[1], -n[2]];
    }
    let d = n[0] * pts[0][0] + n[1] * pts[0][1] + n[2] * pts[0][2];
    let q = |x: f64| (x * 1e6).round() as i64;
    [q(n[0]), q(n[1]), q(n[2]), q(d)]
}

/// Build and classify a mesh from points and cell specs (see the module docs for the
/// classification). Points shared by several cells are shared by index.
pub fn build_mesh(points: &[[f64; 3]], cells: &[CellSpec], alloc: &mut SerialAlloc) -> Result<ReferenceMesh> {
    let mut b = ReferenceBuilder::new();
    let vs: Vec<VertexIx> = points.iter().map(|p| b.add_vertex(Point3::new(p[0], p[1], p[2]))).collect::<Result<_>>()?;
    let pick = |ix: &[usize]| -> Result<Vec<VertexIx>> {
        ix.iter().map(|&i| vs.get(i).copied().ok_or_else(|| MeshError::OutOfRange(format!("point {i}")))).collect()
    };
    for spec in cells {
        let added = match spec {
            CellSpec::Template(shape, ix) => b.add_cell_from_vertices(*shape, &pick(ix)?)?,
            CellSpec::Poly(loops) => {
                let l: Vec<Vec<VertexIx>> = loops.iter().map(|l| pick(l)).collect::<Result<_>>()?;
                let r: Vec<&[VertexIx]> = l.iter().map(Vec::as_slice).collect();
                b.add_polyhedron(&r)?
            }
        };
        b.set_class(MeshEntityRef::Cell(added.cell), model(EntityKind::TopoVolume, 1)?)?;
    }
    // Group boundary faces by plane.
    let mut groups: BTreeMap<[i64; 4], usize> = BTreeMap::new();
    let mut vgroups: Vec<BTreeSet<usize>> = vec![BTreeSet::new(); points.len()];
    for f in 0..b.face_count() {
        let f = FaceIx::new(f);
        if b.face_degree(f) != 1 {
            continue;
        }
        let loop_pts: Vec<[f64; 3]> = b
            .face_loop(f)
            .iter()
            .map(|v| {
                let p = b.vertex_point(*v);
                [p.x, p.y, p.z]
            })
            .collect();
        let next = groups.len() + 1;
        let g = *groups.entry(plane_key(&loop_pts)).or_insert(next);
        b.set_class(MeshEntityRef::Face(f), model(EntityKind::TopoFace, g)?)?;
        for v in b.face_loop(f).to_vec() {
            vgroups[v.get()].insert(g);
        }
    }
    let mut edge_ids: BTreeMap<Vec<usize>, usize> = BTreeMap::new();
    for g in &vgroups {
        if g.len() == 2 {
            let next = edge_ids.len() + 1;
            edge_ids.entry(g.iter().copied().collect()).or_insert(next);
        }
    }
    for (i, g) in vgroups.iter().enumerate() {
        let key: Vec<usize> = g.iter().copied().collect();
        let c = match g.len() {
            0 => model(EntityKind::TopoVolume, 1)?,
            1 => model(EntityKind::TopoFace, key[0])?,
            2 => model(EntityKind::TopoEdge, edge_ids[&key])?,
            _ => model(EntityKind::TopoVertex, i + 1)?,
        };
        b.set_class(MeshEntityRef::Vertex(vs[i]), c)?;
    }
    Ok(b.finish(alloc)?.mesh)
}

fn fixture(name: &'static str, points: &[[f64; 3]], cells: &[CellSpec], cell_volumes: Vec<f64>, area: f64) -> Result<MeshFixture> {
    let mesh = build_mesh(points, cells, &mut SerialAlloc::new())?;
    let boundary_faces = (0..mesh.face_count()).filter(|&f| mesh.face_right(FaceIx::new(f)).is_none()).count();
    Ok(MeshFixture { name, mesh, volume: cell_volumes.iter().sum(), cell_volumes, area, boundary_faces })
}

const SQRT2: f64 = core::f64::consts::SQRT_2;

const UNIT_CUBE: [[f64; 3]; 8] =
    [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];

/// One unit hexahedron `[0,1]^3`: volume 1, area 6, 8 vertices, 6 faces.
pub fn single_hex() -> Result<MeshFixture> {
    fixture("single_hex", &UNIT_CUBE, &[CellSpec::Template(CellShape::Hex, (0..8).collect())], vec![1.0], 6.0)
}

/// One tetrahedron `(0,0,0) (1,0,0) (0,1,0) (0,0,1)`: volume 1/6, area `3/2 + sqrt(3)/2` (three
/// right triangles and an equilateral one of side `sqrt 2`), 4 vertices, 4 faces.
pub fn single_tet() -> Result<MeshFixture> {
    let p = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];
    fixture("single_tet", &p, &[CellSpec::Template(CellShape::Tet, (0..4).collect())], vec![1.0 / 6.0], 1.5 + 0.5 * 3.0_f64.sqrt())
}

/// One wedge: the triangle `(0,0) (1,0) (0,1)` extruded over `z in [0,1]`: volume 1/2, area
/// `3 + sqrt 2`, 6 vertices, 5 faces.
pub fn single_wedge() -> Result<MeshFixture> {
    let p = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [0., 1., 1.]];
    fixture("single_wedge", &p, &[CellSpec::Template(CellShape::Wedge, (0..6).collect())], vec![0.5], 3.0 + SQRT2)
}

/// One pyramid: unit square base, apex `(0.5, 0.5, 1)`: volume 1/3, area `1 + sqrt 5` (slant
/// height `sqrt(5)/2`), 5 vertices, 5 faces.
pub fn single_pyramid() -> Result<MeshFixture> {
    let p = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0.5, 0.5, 1.]];
    fixture("single_pyramid", &p, &[CellSpec::Template(CellShape::Pyramid, (0..5).collect())], vec![1.0 / 3.0], 1.0 + 5.0_f64.sqrt())
}

/// Outward, counter-clockwise (seen from outside) loops of the convex polyhedron with the given
/// vertices and bounding planes `(unit normal, offset)`: the vertices with `n.x = d` (within
/// `1e-9`) form each face. Planes with fewer than 3 vertices are skipped.
pub fn convex_loops(points: &[[f64; 3]], planes: &[([f64; 3], f64)]) -> Vec<Vec<usize>> {
    let mut out = Vec::new();
    for (n, d) in planes {
        let mut ix: Vec<usize> =
            (0..points.len()).filter(|&i| (n[0] * points[i][0] + n[1] * points[i][1] + n[2] * points[i][2] - d).abs() < 1e-9).collect();
        if ix.len() < 3 {
            continue;
        }
        let c = ix.iter().fold([0.0; 3], |a, &i| [a[0] + points[i][0], a[1] + points[i][1], a[2] + points[i][2]]);
        let k = 1.0 / ix.len() as f64;
        let c = [c[0] * k, c[1] * k, c[2] * k];
        // In-plane basis (u, v = n x u).
        let hint = if n[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
        let dp = hint[0] * n[0] + hint[1] * n[1] + hint[2] * n[2];
        let mut u = [hint[0] - dp * n[0], hint[1] - dp * n[1], hint[2] - dp * n[2]];
        let ul = (u[0] * u[0] + u[1] * u[1] + u[2] * u[2]).sqrt();
        u = [u[0] / ul, u[1] / ul, u[2] / ul];
        let v = [n[1] * u[2] - n[2] * u[1], n[2] * u[0] - n[0] * u[2], n[0] * u[1] - n[1] * u[0]];
        let ang = |i: usize| {
            let r = [points[i][0] - c[0], points[i][1] - c[1], points[i][2] - c[2]];
            (r[0] * v[0] + r[1] * v[1] + r[2] * v[2]).atan2(r[0] * u[0] + r[1] * u[1] + r[2] * u[2])
        };
        ix.sort_by(|&a, &b| ang(a).total_cmp(&ang(b)));
        out.push(ix);
    }
    out
}

/// A truncated octahedron as one polyhedral cell: the 24 permutations of `(0, +-1, +-2)`, 6
/// square and 8 hexagonal faces (14 faces, 36 edges). Edge length `a = sqrt(2)`; volume
/// `8 sqrt(2) a^3 = 32`; area `(6 + 12 sqrt 3) a^2`.
pub fn truncated_octahedron() -> Result<MeshFixture> {
    let mut pts: Vec<[f64; 3]> = Vec::new();
    for perm in [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]] {
        for s1 in [-1.0, 1.0] {
            for s2 in [-1.0, 1.0] {
                // Coordinates by permutation of the multiset {0, s1 * 1, s2 * 2}.
                let base = [0.0, s1, 2.0 * s2];
                let p = [base[perm[0]], base[perm[1]], base[perm[2]]];
                if !pts.contains(&p) {
                    pts.push(p);
                }
            }
        }
    }
    let mut planes = Vec::new();
    for ax in 0..3 {
        for s in [-1.0, 1.0] {
            let mut n = [0.0; 3];
            n[ax] = s;
            planes.push((n, 2.0));
        }
    }
    let k = 1.0 / 3.0_f64.sqrt();
    for sx in [-1.0, 1.0] {
        for sy in [-1.0, 1.0] {
            for sz in [-1.0, 1.0] {
                planes.push(([sx * k, sy * k, sz * k], 3.0 * k));
            }
        }
    }
    let loops = convex_loops(&pts, &planes);
    fixture("truncated_octahedron", &pts, &[CellSpec::Poly(loops)], vec![32.0], (6.0 + 12.0 * 3.0_f64.sqrt()) * 2.0)
}

fn ring(n: usize, r: f64, center: (f64, f64), phase: f64, z: f64) -> Vec<[f64; 3]> {
    (0..n)
        .map(|i| {
            let a = phase + TAU * i as f64 / n as f64;
            [center.0 + r * a.cos(), center.1 + r * a.sin(), z]
        })
        .collect()
}

/// Outward loops of a prism over the ring points `base[0..n]` (counter-clockwise) at indices
/// `bottom..bottom+n` and `top..top+n`.
fn prism_loops(bottom: &[usize], top: &[usize]) -> Vec<Vec<usize>> {
    let n = bottom.len();
    let mut loops = vec![bottom.iter().rev().copied().collect::<Vec<_>>(), top.to_vec()];
    for i in 0..n {
        let j = (i + 1) % n;
        loops.push(vec![bottom[i], bottom[j], top[j], top[i]]);
    }
    loops
}

/// A regular hexagonal prism of circumradius `r` and height `h` as one polyhedral cell (8
/// faces): volume `3 sqrt(3)/2 r^2 h`, area `3 sqrt(3) r^2 + 6 r h`.
pub fn hexagonal_prism(r: f64, h: f64) -> Result<MeshFixture> {
    let mut pts = ring(6, r, (0.0, 0.0), 0.0, 0.0);
    pts.extend(ring(6, r, (0.0, 0.0), 0.0, h));
    let loops = prism_loops(&(0..6).collect::<Vec<_>>(), &(6..12).collect::<Vec<_>>());
    fixture("hexagonal_prism", &pts, &[CellSpec::Poly(loops)], vec![1.5 * 3.0_f64.sqrt() * r * r * h], 3.0 * 3.0_f64.sqrt() * r * r + 6.0 * r * h)
}

/// The unit cube as a polyhedron whose `-z` face is split into two triangles (7 faces, shape
/// `Polyhedron`): volume 1, area 6.
pub fn split_face_cube() -> Result<MeshFixture> {
    let loops = vec![
        vec![0, 2, 1],
        vec![0, 3, 2],
        vec![4, 5, 6, 7],
        vec![0, 1, 5, 4],
        vec![1, 2, 6, 5],
        vec![2, 3, 7, 6],
        vec![3, 0, 4, 7],
    ];
    fixture("split_face_cube", &UNIT_CUBE, &[CellSpec::Poly(loops)], vec![1.0], 6.0)
}

/// The unit cube split into a `2 x 2 x 2` block of hexahedra (`h = 1/2`): 27 vertices, 8 cells
/// of volume 1/8, 24 boundary faces. Classified like a CAD box by `sm_mesh_api::reference::hex_block`.
pub fn hex_block_2x2x2() -> Result<MeshFixture> {
    let mesh = hex_block([2, 2, 2], [0.5; 3], &mut SerialAlloc::new())?;
    Ok(MeshFixture { name: "hex_block_2x2x2", mesh, cell_volumes: vec![0.125; 8], volume: 1.0, area: 6.0, boundary_faces: 24 })
}

/// Hexahedron, wedge, pyramid and tetrahedron glued by conforming faces:
///
/// - hex `[0,1]^3`;
/// - a wedge on its `+x` face (triangle `(1,0) (2,0) (1,1)` in `xy`, over `z in [0,1]`): volume 1/2;
/// - a pyramid on its `+z` face, apex `(0.5, 0.5, 2)`: volume 1/3;
/// - a tetrahedron on the pyramid's `-y` triangle, extra vertex `(0.5, -1, 1.5)`: volume 5/24.
///
/// Total `1 + 1/2 + 1/3 + 5/24`. Three interior faces: hex|wedge, hex|pyramid, pyramid|tet.
/// Boundary area `6 + sqrt 2 + sqrt 5 + sqrt(35)/4`: four hex faces (4), the wedge's two
/// triangles, `y = 0` quad and slanted quad (`2 + sqrt 2`), three pyramid triangles of slant
/// height `sqrt(5)/2` (`3 sqrt(5)/4`), and the tet's free faces (`sqrt(5)/4 + 2 sqrt(35)/8`).
pub fn mixed_transition() -> Result<MeshFixture> {
    let mut p: Vec<[f64; 3]> = UNIT_CUBE.to_vec();
    p.extend([[2., 0., 0.], [2., 0., 1.]]); // 8, 9
    p.push([0.5, 0.5, 2.0]); // 10 pyramid apex
    p.push([0.5, -1.0, 1.5]); // 11 tet apex
    let cells = [
        CellSpec::Template(CellShape::Hex, (0..8).collect()),
        // wedge: v0=(1,0,0) v1=(2,0,0) v2=(1,1,0) v3=(1,0,1) v4=(2,0,1) v5=(1,1,1)
        CellSpec::Template(CellShape::Wedge, vec![1, 8, 2, 5, 9, 6]),
        // pyramid: base is the hex top (counter-clockwise from above), apex 10
        CellSpec::Template(CellShape::Pyramid, vec![4, 5, 6, 7, 10]),
        // tet on the pyramid face (4, 5, 10): right-handed (4, 5, 10, 11)
        CellSpec::Template(CellShape::Tet, vec![4, 5, 10, 11]),
    ];
    fixture("mixed_transition", &p, &cells, vec![1.0, 0.5, 1.0 / 3.0, 1.25 / 6.0], 6.0 + SQRT2 + 5.0_f64.sqrt() + 35.0_f64.sqrt() / 4.0)
}

/// A polyhedral dual patch: seven regular hexagonal prisms (circumradius 1, height `h = 1`)
/// forming a honeycomb flower (centre cell and its six neighbours), vertices welded so
/// neighbours share hexagon side faces. Each cell has volume `3 sqrt(3)/2`. There are 24
/// distinct points per z level (48 vertices) and 12 interior faces: 6 between the centre and the
/// ring, 6 between ring neighbours. Boundary area `21 sqrt 3 + 18`: 14 hexagon caps and the
/// `42 - 2 * 12` free side quads of area 1.
pub fn dual_patch() -> Result<MeshFixture> {
    let h = 1.0;
    let sqrt3 = 3.0_f64.sqrt();
    let mut centers = vec![(0.0, 0.0)];
    for k in 0..6 {
        let a = TAU * k as f64 / 6.0;
        centers.push((sqrt3 * a.cos(), sqrt3 * a.sin()));
    }
    let q = |x: f64| (x * 1e6).round() as i64;
    let mut index: BTreeMap<(i64, i64, i64), usize> = BTreeMap::new();
    let mut pts: Vec<[f64; 3]> = Vec::new();
    let mut cells = Vec::new();
    for c in &centers {
        let mut levels: Vec<Vec<usize>> = Vec::new();
        for z in [0.0, h] {
            let mut ids = Vec::new();
            for p in ring(6, 1.0, *c, TAU / 12.0, z) {
                let key = (q(p[0]), q(p[1]), q(p[2]));
                let id = *index.entry(key).or_insert_with(|| {
                    pts.push(p);
                    pts.len() - 1
                });
                ids.push(id);
            }
            levels.push(ids);
        }
        cells.push(CellSpec::Poly(prism_loops(&levels[0], &levels[1])));
    }
    let v = 1.5 * sqrt3 * h;
    fixture("dual_patch", &pts, &cells, vec![v; 7], 21.0 * sqrt3 + 18.0 * h)
}

/// Every fixture of this module.
pub fn all_mesh_fixtures() -> Result<Vec<MeshFixture>> {
    Ok(vec![
        single_hex()?,
        single_tet()?,
        single_wedge()?,
        single_pyramid()?,
        truncated_octahedron()?,
        hexagonal_prism(1.0, 2.0)?,
        split_face_cube()?,
        hex_block_2x2x2()?,
        mixed_transition()?,
        dual_patch()?,
    ])
}

// ------------------------------------------------------------------------------ W3 additions

/// Model entity of a box-lattice position, with the numbering of `sm_mesh_api::reference::hex_block`
/// and of `sm_topo_api::fixtures::brick`: per axis either a lattice index (`Some`) or free
/// (`None`, sliding along the entity).
pub(crate) fn box_entity(n: [u32; 3], pos: [Option<usize>; 3]) -> Result<EntityId> {
    let ext = |a: usize, q: usize| q == 0 || q == n[a] as usize;
    let side = |a: usize, q: usize| usize::from(q == n[a] as usize);
    let fixed: Vec<usize> = (0..3).filter(|&a| pos[a].is_some_and(|q| ext(a, q))).collect();
    let sliding: Vec<usize> = (0..3).filter(|a| !fixed.contains(a)).collect();
    let sd = |a: usize| side(a, pos[a].unwrap_or(0));
    let (kind, serial) = match fixed.len() {
        3 => (EntityKind::TopoVertex, 1 + (sd(0) + 2 * sd(1) + 4 * sd(2)) as u64),
        2 => (EntityKind::TopoEdge, 1 + (sliding[0] * 4 + sd(fixed[0]) + 2 * sd(fixed[1])) as u64),
        1 => (EntityKind::TopoFace, 1 + (fixed[0] * 2 + sd(fixed[0])) as u64),
        _ => (EntityKind::TopoVolume, 1),
    };
    EntityId::new(kind, serial).ok_or(MeshError::SerialOverflow)
}

/// The six Kuhn tetrahedra of a lattice cell, as lattice offsets (`0` or `1` per axis) of their
/// four corners: one per axis permutation along the main diagonal, each positively oriented.
pub(crate) fn kuhn_offsets() -> [[[usize; 3]; 4]; 6] {
    const PERMS: [[usize; 3]; 6] = [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]];
    let mut out = [[[0usize; 3]; 4]; 6];
    for (t, p) in PERMS.iter().enumerate() {
        let mut v1 = [0usize; 3];
        v1[p[0]] = 1;
        let mut v2 = v1;
        v2[p[1]] = 1;
        let (a, b, c) = (v1.map(|x| x as i64), v2.map(|x| x as i64), [1i64; 3]);
        let det = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
        let (first, second) = if det > 0 { (v1, v2) } else { (v2, v1) };
        out[t] = [[0; 3], first, second, [1; 3]];
    }
    out
}

/// A box lattice mesh classified like `sm_mesh_api::reference::hex_block`: hexahedra or Kuhn
/// tetrahedra on `(n0+1)(n1+1)(n2+1)` nodes at `point(i, j, k)`; `param(entity, p)` gives the
/// classification parameter of a node.
pub(crate) fn build_box_lattice(
    n: [u32; 3],
    kuhn: bool,
    point: &dyn Fn(usize, usize, usize) -> Point3,
    param: &dyn Fn(EntityId, Point3) -> Param,
    alloc: &mut SerialAlloc,
) -> Result<ReferenceMesh> {
    if n.contains(&0) {
        return Err(MeshError::State("a lattice needs at least one cell per axis".into()));
    }
    let nn = n.map(|x| x as usize);
    let m = [nn[0] + 1, nn[1] + 1, nn[2] + 1];
    let ix = |i: usize, j: usize, k: usize| VertexIx::new((k * m[1] + j) * m[0] + i);
    let mut b = ReferenceBuilder::new();
    for k in 0..m[2] {
        for j in 0..m[1] {
            for i in 0..m[0] {
                let p = point(i, j, k);
                let v = b.add_vertex(p)?;
                let e = box_entity(n, [Some(i), Some(j), Some(k)])?;
                b.set_class(MeshEntityRef::Vertex(v), Classification { entity: e, param: param(e, p) })?;
            }
        }
    }
    let vol = Classification::on(box_entity(n, [None, None, None])?);
    let tets = kuhn_offsets();
    for k in 0..nn[2] {
        for j in 0..nn[1] {
            for i in 0..nn[0] {
                let at = |o: [usize; 3]| ix(i + o[0], j + o[1], k + o[2]);
                if kuhn {
                    for t in &tets {
                        let vs = [at(t[0]), at(t[1]), at(t[2]), at(t[3])];
                        let added = b.add_cell_from_vertices(CellShape::Tet, &vs)?;
                        b.set_class(MeshEntityRef::Cell(added.cell), vol)?;
                    }
                } else {
                    let vs = [
                        at([0, 0, 0]),
                        at([1, 0, 0]),
                        at([1, 1, 0]),
                        at([0, 1, 0]),
                        at([0, 0, 1]),
                        at([1, 0, 1]),
                        at([1, 1, 1]),
                        at([0, 1, 1]),
                    ];
                    let added = b.add_cell_from_vertices(CellShape::Hex, &vs)?;
                    b.set_class(MeshEntityRef::Cell(added.cell), vol)?;
                }
            }
        }
    }
    let pos_of = |v: VertexIx| [v.get() % m[0], (v.get() / m[0]) % m[1], v.get() / (m[0] * m[1])];
    for f in 0..b.face_count() {
        let f = FaceIx::new(f);
        if b.face_degree(f) != 1 {
            continue;
        }
        let ps: Vec<[usize; 3]> = b.face_loop(f).iter().map(|v| pos_of(*v)).collect();
        let on = (0..3).find_map(|a| {
            [0, nn[a]].into_iter().find(|&q| ps.iter().all(|p| p[a] == q)).map(|q| (a, usize::from(q == nn[a])))
        });
        let (a, s) = on.ok_or_else(|| MeshError::State("a boundary face is not on a box side".into()))?;
        let e = EntityId::new(EntityKind::TopoFace, 1 + (2 * a + s) as u64).ok_or(MeshError::SerialOverflow)?;
        b.set_class(MeshEntityRef::Face(f), Classification::on(e))?;
    }
    Ok(b.finish(alloc)?.mesh)
}

fn boundary_area(m: &ReferenceMesh) -> f64 {
    (0..m.face_count())
        .map(FaceIx::new)
        .filter(|f| m.face_right(*f).is_none())
        .map(|f| sm_mesh_api::face_area_vector(m, f).norm())
        .sum()
}

fn boundary_face_count(m: &ReferenceMesh) -> usize {
    (0..m.face_count()).filter(|&f| m.face_right(FaceIx::new(f)).is_none()).count()
}

fn tet_volume_of(a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3]) -> f64 {
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let w = [d[0] - a[0], d[1] - a[1], d[2] - a[2]];
    (u[0] * (v[1] * w[2] - v[2] * w[1]) - u[1] * (v[0] * w[2] - v[2] * w[0]) + u[2] * (v[0] * w[1] - v[1] * w[0])) / 6.0
}

/// Like [`build_mesh`], but each cell names its `TopoVolume` serial, so a mesh can hold several
/// volumes. Classification:
///
/// - boundary faces (one cell) and interface faces (two cells of different volumes) are grouped
///   by plane and volume (or volume pair): one `TopoFace` per group, numbered by first
///   appearance;
/// - a vertex touching one plane lies on that face, two planes on a `TopoEdge` (one per plane
///   pair and set of touching volumes, so a line crossing an interface is split there), three or
///   more planes on its own `TopoVertex`; a vertex touching no such face lies on the volume of a
///   cell that contains it.
pub fn build_mesh_volumes(points: &[[f64; 3]], cells: &[(CellSpec, u64)], alloc: &mut SerialAlloc) -> Result<ReferenceMesh> {
    let mut b = ReferenceBuilder::new();
    let vs: Vec<VertexIx> = points.iter().map(|p| b.add_vertex(Point3::new(p[0], p[1], p[2]))).collect::<Result<_>>()?;
    let pick = |ix: &[usize]| -> Result<Vec<VertexIx>> {
        ix.iter().map(|&i| vs.get(i).copied().ok_or_else(|| MeshError::OutOfRange(format!("point {i}")))).collect()
    };
    let mut cell_vol: Vec<u64> = Vec::with_capacity(cells.len());
    for (spec, vol) in cells {
        let added = match spec {
            CellSpec::Template(shape, ix) => b.add_cell_from_vertices(*shape, &pick(ix)?)?,
            CellSpec::Poly(loops) => {
                let l: Vec<Vec<VertexIx>> = loops.iter().map(|l| pick(l)).collect::<Result<_>>()?;
                let r: Vec<&[VertexIx]> = l.iter().map(Vec::as_slice).collect();
                b.add_polyhedron(&r)?
            }
        };
        let vol_id = EntityId::new(EntityKind::TopoVolume, *vol).ok_or(MeshError::SerialOverflow)?;
        b.set_class(MeshEntityRef::Cell(added.cell), Classification::on(vol_id))?;
        cell_vol.push(*vol);
    }
    type Group = ([i64; 4], u64, u64);
    let mut groups: BTreeMap<Group, usize> = BTreeMap::new();
    let mut vgroups: Vec<BTreeSet<Group>> = vec![BTreeSet::new(); points.len()];
    for f in 0..b.face_count() {
        let f = FaceIx::new(f);
        let l = b.face_cell(f, sm_mesh_api::Side::Left).map(|c| cell_vol[c.get()]);
        let r = b.face_cell(f, sm_mesh_api::Side::Right).map(|c| cell_vol[c.get()]);
        let (lo, hi) = match (l, r) {
            (Some(a), None) | (None, Some(a)) => (a, 0),
            (Some(a), Some(c)) if a != c => (a.min(c), a.max(c)),
            _ => continue,
        };
        let loop_pts: Vec<[f64; 3]> = b
            .face_loop(f)
            .iter()
            .map(|v| {
                let p = b.vertex_point(*v);
                [p.x, p.y, p.z]
            })
            .collect();
        let key = (plane_key(&loop_pts), lo, hi);
        let next = groups.len() + 1;
        let g = *groups.entry(key).or_insert(next);
        b.set_class(MeshEntityRef::Face(f), model(EntityKind::TopoFace, g)?)?;
        for v in b.face_loop(f).to_vec() {
            vgroups[v.get()].insert(key);
        }
    }
    type EdgeKey = (Vec<[i64; 4]>, Vec<u64>);
    let mut edge_ids: BTreeMap<EdgeKey, usize> = BTreeMap::new();
    let mut vertex_serial = 0usize;
    for (i, g) in vgroups.iter().enumerate() {
        let planes: BTreeSet<[i64; 4]> = g.iter().map(|k| k.0).collect();
        let c = match (planes.len(), g.iter().next()) {
            (1, Some(first)) => model(EntityKind::TopoFace, groups[first])?,
            (2, _) => {
                let vols: BTreeSet<u64> = g.iter().flat_map(|k| [k.1, k.2]).filter(|v| *v != 0).collect();
                let key = (planes.into_iter().collect::<Vec<_>>(), vols.into_iter().collect::<Vec<_>>());
                let next = edge_ids.len() + 1;
                model(EntityKind::TopoEdge, *edge_ids.entry(key).or_insert(next))?
            }
            (k, _) if k >= 3 => {
                vertex_serial += 1;
                model(EntityKind::TopoVertex, vertex_serial)?
            }
            _ => {
                // Interior vertex: the volume of any cell that contains it.
                let cell = b.vertex_cell(vs[i]).ok_or_else(|| MeshError::State(format!("point {i} belongs to no cell")))?;
                model(EntityKind::TopoVolume, cell_vol[cell.get()] as usize)?
            }
        };
        b.set_class(MeshEntityRef::Vertex(vs[i]), c)?;
    }
    Ok(b.finish(alloc)?.mesh)
}

/// A Kuhn (Freudenthal) tetrahedral box: every cell of an `n[0] x n[1] x n[2]` lattice with cell
/// sizes `h` is split into 6 tetrahedra along its main diagonal; the same split in every cell
/// makes the mesh conforming. Classified onto a CAD box exactly like
/// `sm_mesh_api::reference::hex_block` (`TopoFace#1..6` are `-x,+x,-y,+y,-z,+z`, edge and vertex
/// numbering as there, one `TopoVolume#1`), so [`crate::check::box_carriers`] applies.
///
/// Invariants: `V = (n0+1)(n1+1)(n2+1)`, `T = 6 n0 n1 n2` tets of volume `h0 h1 h2 / 6`, total
/// volume `n0 n1 n2 h0 h1 h2`, boundary area `2 (L0 L1 + L1 L2 + L0 L2)` with `Li = ni hi`, and
/// `4 (n0 n1 + n1 n2 + n0 n2)` boundary triangles. Use `n >= 2` per axis so every model edge has
/// an interior vertex. Fails with [`MeshError::State`] if any `n[i]` is zero.
pub fn tet_block(n: [u32; 3], h: [f64; 3]) -> Result<MeshFixture> {
    let point = |i: usize, j: usize, k: usize| Point3::new(i as f64 * h[0], j as f64 * h[1], k as f64 * h[2]);
    let mesh = build_box_lattice(n, true, &point, &|_, _| Param::None, &mut SerialAlloc::new())?;
    let nc = n.map(|x| x as usize);
    let ext = [nc[0] as f64 * h[0], nc[1] as f64 * h[1], nc[2] as f64 * h[2]];
    Ok(MeshFixture {
        name: "tet_block",
        mesh,
        cell_volumes: vec![h[0] * h[1] * h[2] / 6.0; 6 * nc[0] * nc[1] * nc[2]],
        volume: ext[0] * ext[1] * ext[2],
        area: 2.0 * (ext[0] * ext[1] + ext[1] * ext[2] + ext[0] * ext[2]),
        boundary_faces: 4 * (nc[0] * nc[1] + nc[1] * nc[2] + nc[0] * nc[2]),
    })
}

type KuhnData = (Vec<[f64; 3]>, Vec<([usize; 4], usize)>);

/// Lattice points (numbered `(k * (n1+1) + j) * (n0+1) + i`) and Kuhn tets (point indices, with
/// the lattice `i` of their cell) of an `n` box with cell sizes `h`.
fn kuhn_block_data(n: [u32; 3], h: [f64; 3]) -> KuhnData {
    let nn = n.map(|x| x as usize);
    let m = [nn[0] + 1, nn[1] + 1, nn[2] + 1];
    let mut pts = Vec::with_capacity(m[0] * m[1] * m[2]);
    for k in 0..m[2] {
        for j in 0..m[1] {
            for i in 0..m[0] {
                pts.push([i as f64 * h[0], j as f64 * h[1], k as f64 * h[2]]);
            }
        }
    }
    let offs = kuhn_offsets();
    let mut tets = Vec::with_capacity(6 * nn[0] * nn[1] * nn[2]);
    for k in 0..nn[2] {
        for j in 0..nn[1] {
            for i in 0..nn[0] {
                for t in &offs {
                    let ix = |o: [usize; 3]| ((k + o[2]) * m[1] + (j + o[1])) * m[0] + i + o[0];
                    tets.push(([ix(t[0]), ix(t[1]), ix(t[2]), ix(t[3])], i));
                }
            }
        }
    }
    (pts, tets)
}

/// [`tet_block`] split at the lattice plane `i = split` into two volumes: cells with `i < split`
/// are `TopoVolume#1`, the others `TopoVolume#2`. The mesh vertices on the interface are shared,
/// and the `2 n1 n2` interface triangles are interior faces between the two volumes, classified
/// on one `TopoFace` (numbering: see [`build_mesh_volumes`]).
///
/// Volume per side: `split n1 n2 h0 h1 h2` and `(n0 - split) n1 n2 h0 h1 h2`. Needs
/// `0 < split < n0`.
pub fn tet_block_two_volumes(n: [u32; 3], h: [f64; 3], split: u32) -> Result<MeshFixture> {
    if n.contains(&0) || split == 0 || split >= n[0] {
        return Err(MeshError::State("tet_block_two_volumes needs n >= 1 and 0 < split < n[0]".into()));
    }
    let (pts, tets) = kuhn_block_data(n, h);
    let cells: Vec<(CellSpec, u64)> =
        tets.iter().map(|(t, i)| (CellSpec::Template(CellShape::Tet, t.to_vec()), if (*i as u32) < split { 1 } else { 2 })).collect();
    let mesh = build_mesh_volumes(&pts, &cells, &mut SerialAlloc::new())?;
    let nc = n.map(|x| x as usize);
    let ext = [nc[0] as f64 * h[0], nc[1] as f64 * h[1], nc[2] as f64 * h[2]];
    let boundary_faces = boundary_face_count(&mesh);
    Ok(MeshFixture {
        name: "tet_block_two_volumes",
        mesh,
        cell_volumes: vec![h[0] * h[1] * h[2] / 6.0; tets.len()],
        volume: ext[0] * ext[1] * ext[2],
        area: 2.0 * (ext[0] * ext[1] + ext[1] * ext[2] + ext[0] * ext[2]),
        boundary_faces,
    })
}

/// [`tet_block`] with its vertices moved by the smooth map
/// `z' = z (1 + amp sin(pi x / Lx) sin(pi y / Ly))` (`Lx = n0 h0`, `Ly = n1 h1`), classification
/// unchanged, so the `+z` face is curved and the other faces keep their planes. Tets stay
/// positive for `|amp| <= 0.2`.
///
/// Cell volumes and area have no closed form here: `cell_volumes` are the tet volumes from the
/// moved points (summed to `volume`) and `area` is the polyhedral area of the boundary
/// triangles. For `amp = 0` the result equals [`tet_block`].
pub fn tet_block_warped(n: [u32; 3], h: [f64; 3], amp: f64) -> Result<MeshFixture> {
    let nc = n.map(|x| x as usize);
    let (lx, ly) = (nc[0] as f64 * h[0], nc[1] as f64 * h[1]);
    let warp = |x: f64, y: f64, z: f64| {
        let w = 1.0 + amp * (core::f64::consts::PI * x / lx).sin() * (core::f64::consts::PI * y / ly).sin();
        [x, y, z * w]
    };
    let point = |i: usize, j: usize, k: usize| {
        let p = warp(i as f64 * h[0], j as f64 * h[1], k as f64 * h[2]);
        Point3::new(p[0], p[1], p[2])
    };
    let mesh = build_box_lattice(n, true, &point, &|_, _| Param::None, &mut SerialAlloc::new())?;
    let (pts, tets) = kuhn_block_data(n, h);
    let moved: Vec<[f64; 3]> = pts.iter().map(|p| warp(p[0], p[1], p[2])).collect();
    let cell_volumes: Vec<f64> = tets.iter().map(|(t, _)| tet_volume_of(moved[t[0]], moved[t[1]], moved[t[2]], moved[t[3]])).collect();
    let boundary_faces = boundary_face_count(&mesh);
    let area = boundary_area(&mesh);
    Ok(MeshFixture { name: "tet_block_warped", volume: cell_volumes.iter().sum(), cell_volumes, area, boundary_faces, mesh })
}

/// An L-shaped prism of Kuhn tetrahedra: three of the four quadrants of a `2m x 2m x m` lattice
/// of cells of size `h` (the quadrant `i >= m, j >= m` is removed), one `TopoVolume`, one reflex
/// `TopoEdge` along `x = y = m h`. Classified by [`build_mesh_volumes`] (plane groups).
///
/// Volume `3 m^3 h^3` (`18 m^3` tets of volume `h^3 / 6`), boundary area `14 m^2 h^2`, `28 m^2`
/// boundary triangles. Needs `m >= 1`.
pub fn tet_lshape(m: u32, h: f64) -> Result<MeshFixture> {
    if m == 0 {
        return Err(MeshError::State("tet_lshape needs m >= 1".into()));
    }
    let mm = m as usize;
    let offs = kuhn_offsets();
    let mut index: BTreeMap<[usize; 3], usize> = BTreeMap::new();
    let mut pts: Vec<[f64; 3]> = Vec::new();
    let mut cells: Vec<(CellSpec, u64)> = Vec::new();
    for k in 0..mm {
        for j in 0..2 * mm {
            for i in 0..2 * mm {
                if i >= mm && j >= mm {
                    continue;
                }
                for t in &offs {
                    let mut ids = Vec::with_capacity(4);
                    for o in t {
                        let key = [i + o[0], j + o[1], k + o[2]];
                        let id = *index.entry(key).or_insert_with(|| {
                            pts.push([key[0] as f64 * h, key[1] as f64 * h, key[2] as f64 * h]);
                            pts.len() - 1
                        });
                        ids.push(id);
                    }
                    cells.push((CellSpec::Template(CellShape::Tet, ids), 1));
                }
            }
        }
    }
    let mesh = build_mesh_volumes(&pts, &cells, &mut SerialAlloc::new())?;
    let mf = mm as f64;
    Ok(MeshFixture {
        name: "tet_lshape",
        mesh,
        cell_volumes: vec![h * h * h / 6.0; cells.len()],
        volume: 3.0 * mf * mf * mf * h * h * h,
        area: 14.0 * mf * mf * h * h,
        boundary_faces: 28 * mm * mm,
    })
}

/// A non-convex polyhedron next to a brick, sharing one face:
///
/// - an L-shaped prism as ONE polyhedral cell (cell 0): the `xy` loop
///   `(0,0) (1,0) (2,0) (2,1) (1,1) (1,2) (0,2) (0,1)` extruded over `z in [0,1]` (volume 3, 10
///   faces, one re-entrant edge along `x = y = 1`). The mid-edge points `(1,0)` and `(0,1)` are
///   there so that the vertex mean of each cap, `(7/8, 7/8)`, lies in the cap's kernel and the
///   cell passes the default geometry rules (side tetrahedra positive);
/// - the brick `[2,3] x [0,1] x [0,1]` (cell 1, hexahedron, volume 1) on the prism's `x = 2` face.
///
/// Total volume 4, boundary area 18 (the union has `xy` area 4 and perimeter 10), 14 boundary
/// faces. The line `x + y = 2.5, z = 0.5` crosses the prism in two disjoint chords,
/// `(0.5,2)-(1,1.5)` and `(1.5,1)-(2,0.5)` (each of length `sqrt(2)/2`), with the notch between.
pub fn nonconvex_poly() -> Result<MeshFixture> {
    let l = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0), (1.0, 2.0), (0.0, 2.0), (0.0, 1.0)];
    let mut pts: Vec<[f64; 3]> = l.iter().map(|p| [p.0, p.1, 0.0]).collect();
    pts.extend(l.iter().map(|p| [p.0, p.1, 1.0]));
    pts.extend([[3., 0., 0.], [3., 1., 0.], [3., 0., 1.], [3., 1., 1.]]); // 16..19
    let prism = CellSpec::Poly(prism_loops(&(0..8).collect::<Vec<_>>(), &(8..16).collect::<Vec<_>>()));
    // Brick: (2,0,0)=2, (3,0,0)=16, (3,1,0)=17, (2,1,0)=3, then the same at z = 1: 10, 18, 19, 11.
    let brick = CellSpec::Template(CellShape::Hex, vec![2, 16, 17, 3, 10, 18, 19, 11]);
    fixture("nonconvex_poly", &pts, &[prism, brick], vec![3.0, 1.0], 18.0)
}
