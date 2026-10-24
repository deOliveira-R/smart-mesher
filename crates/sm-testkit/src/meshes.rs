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
    CellShape, Classification, FaceIx, MeshBuild, MeshEntityRef, MeshError, MeshRead, Result, SerialAlloc, VertexIx,
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
