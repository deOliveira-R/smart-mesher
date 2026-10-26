//! The W3a milestone part and a mixed-shape mesh.
//!
//! # The part
//!
//! Three volumes, derived by hand (all lengths in the STEP file's unit):
//!
//! - **V1**, the L-slab: the polygon `(0,0) (20,0) (20,10) (10,10) (10,20) (0,20)` extruded over
//!   `z in [0, 4]`. Polygon area `20 * 20 - 10 * 10 = 300`, volume `300 * 4 = 1200`. Faces: 8
//!   (two L faces, six sides). Boundary area `2 * 300 + 4 * perimeter`, perimeter
//!   `20 + 10 + 10 + 10 + 10 + 20 = 80`, so `600 + 320 = 920`.
//! - **V2**, the triangular prism: `(10,10) (20,10) (10,20)` over `z in [0, 4]`. Area
//!   `10 * 10 / 2 = 50`, volume `200`. Faces: 5. Boundary area `2 * 50 + 4 * (10 + 10 + 10 sqrt 2)
//!   = 180 + 40 sqrt 2 = 236.568542494923802`.
//! - **V3**, the box `[10, 20]^2 x [4, 10]`: volume `10 * 10 * 6 = 600`, faces 6, boundary area
//!   `2 * 100 + 4 * (10 * 6) = 440`.
//!
//! Total volume `1200 + 200 + 600 = 2000`.
//!
//! V2 sits in the notch of V1: its legs `y = 10` (`x in [10, 20]`) and `x = 10` (`y in [10, 20]`)
//! lie on V1's faces, each `10 x 4 = 40`. V3 sits on V2's top: V2's top triangle (area 50) is
//! part of V3's bottom square `[10,20]^2 at z = 4`; the other triangle of that square touches V1
//! only along an edge. After imprint and merge V3's bottom is split in two faces, so the faces
//! are `8 + 5 + 7 - 3 = 17` where the three shared faces are counted once. Shared face areas:
//! `[40, 40, 50]`. The outer area is the sum of the three boundaries minus both sides of every
//! shared face: `920 + 236.568542... + 440 - 2 * (40 + 40 + 50) = 1336.568542494923802`.

use crate::meshes::{CellSpec, MeshFixture, build_mesh};
use sm_mesh_api::{CellIx, CellShape, FaceIx, MeshEdit, MeshRead, Result, SerialAlloc, SetMembers, Side, SignedFace};

/// Volumes of V1, V2, V3 (the STEP order).
pub const VOLUMES: [f64; 3] = [1200.0, 200.0, 600.0];
/// Total volume of the part.
pub const TOTAL_VOLUME: f64 = 2000.0;
/// Number of faces after imprint and merge.
pub const FACES: usize = 17;
/// Areas of the three shared faces: V1|V2 (two) and V2|V3.
pub const SHARED_FACE_AREAS: [f64; 3] = [40.0, 40.0, 50.0];
/// Boundary area of each volume (the side-set areas).
#[allow(clippy::excessive_precision)]
pub const SIDESET_AREAS: [f64; 3] = [920.0, 236.568542494923802, 440.0];
/// Area of the outer surface of the merged part.
#[allow(clippy::excessive_precision)]
pub const OUTER_AREA: f64 = 1336.568542494923802;
/// The part as a STEP file with three solids (V1, V2, V3), written by Open CASCADE 7.8.
/// sha256 `252eb1e6377ebd4090287def170c3c6442b0f0c9f286308cf75b037606bb463d`.
pub const STEP: &[u8] = include_bytes!("../data/step/w3a_part.step");

/// Names of the cell sets of [`mixed_all_shapes`], in cell order (hex, wedge, pyramid, tet, poly).
pub const CELL_SETS: [&str; 5] = ["hex", "wedge", "pyramid", "tet", "poly"];
/// Name of the side set of [`mixed_all_shapes`]: the boundary sides of the polyhedron cell.
pub const SIDE_SET: &str = "poly_sides";
/// Name of the face set of [`mixed_all_shapes`]: the boundary faces in the plane `z = 0`.
pub const FACE_SET: &str = "bottom";

/// Cell shapes of [`mixed_all_shapes`] in cell order.
pub const SHAPES: [CellShape; 5] = [CellShape::Hex, CellShape::Wedge, CellShape::Pyramid, CellShape::Tet, CellShape::Polyhedron];

/// A classified mesh with one hexahedron, wedge, pyramid, tetrahedron and polyhedron cell.
///
/// The first four are the cells of `meshes::mixed_transition` (hex `[0,1]^3`, wedge on its `+x`
/// face, pyramid on its top, tet on the pyramid), volumes `1, 1/2, 1/3, 5/24`. The polyhedron is
/// the prism over the pentagon `(0,0) (0,1) (-1,1) (-1.5,0.5) (-1,0)` (area `1 + 1/4`) for
/// `z in [0, 1]`, glued to the hex's `-x` face: volume `5/4`, seven faces (two pentagons, five
/// quads). Total volume `1 + 1/2 + 1/3 + 5/24 + 5/4`.
///
/// Boundary area: `mixed_transition`'s `6 + sqrt 2 + sqrt 5 + sqrt(35)/4`, minus the hex's glued
/// face (1), plus the polyhedron's free faces `2 * 5/4 + 1 + 1 + 2 * sqrt(1/2) = 9/2 + sqrt 2`.
///
/// Sets: one cell set per shape ([`CELL_SETS`], one cell each), the side set [`SIDE_SET`] (every
/// boundary side of the polyhedron: 6 sides) and the face set [`FACE_SET`] (boundary faces in
/// `z = 0`: hex bottom, wedge bottom, polyhedron bottom). `cell_volumes` is in cell order.
pub fn mixed_all_shapes() -> Result<MeshFixture> {
    let mut p: Vec<[f64; 3]> = vec![
        [0., 0., 0.],
        [1., 0., 0.],
        [1., 1., 0.],
        [0., 1., 0.],
        [0., 0., 1.],
        [1., 0., 1.],
        [1., 1., 1.],
        [0., 1., 1.],
        [2., 0., 0.],
        [2., 0., 1.],
        [0.5, 0.5, 2.0],
        [0.5, -1.0, 1.5],
    ];
    p.extend([[-1., 0., 0.], [-1., 1., 0.], [-1.5, 0.5, 0.], [-1., 0., 1.], [-1., 1., 1.], [-1.5, 0.5, 1.]]);
    let cells = [
        CellSpec::Template(CellShape::Hex, (0..8).collect()),
        CellSpec::Template(CellShape::Wedge, vec![1, 8, 2, 5, 9, 6]),
        CellSpec::Template(CellShape::Pyramid, vec![4, 5, 6, 7, 10]),
        CellSpec::Template(CellShape::Tet, vec![4, 5, 10, 11]),
        CellSpec::Poly(vec![
            vec![12, 14, 13, 3, 0],
            vec![4, 7, 16, 17, 15],
            vec![0, 3, 7, 4],
            vec![3, 13, 16, 7],
            vec![13, 14, 17, 16],
            vec![14, 12, 15, 17],
            vec![12, 0, 4, 15],
        ]),
    ];
    let mut alloc = SerialAlloc::new();
    let mut mesh = build_mesh(&p, &cells, &mut alloc)?;
    for (i, name) in CELL_SETS.iter().enumerate() {
        mesh.add_set(name, SetMembers::Cells(vec![CellIx::new(i)]), &mut alloc)?;
    }
    let poly = CellIx::new(4);
    let mut sides = Vec::new();
    let mut bottom = Vec::new();
    for f in 0..mesh.face_count() {
        let f = FaceIx::new(f);
        if mesh.face_right(f).is_some() {
            continue;
        }
        if mesh.face_left(f) == poly {
            sides.push(SignedFace::new(f, Side::Left));
        }
        if mesh.face_vertices(f).iter().all(|v| mesh.point(v.get()).z == 0.0) {
            bottom.push(f);
        }
    }
    let boundary_faces = (0..mesh.face_count()).filter(|&f| mesh.face_right(FaceIx::new(f)).is_none()).count();
    mesh.add_set(SIDE_SET, SetMembers::Sides(sides), &mut alloc)?;
    mesh.add_set(FACE_SET, SetMembers::Faces(bottom), &mut alloc)?;
    let cell_volumes = vec![1.0, 0.5, 1.0 / 3.0, 1.25 / 6.0, 1.25];
    let area = 6.0 + core::f64::consts::SQRT_2 + 5.0_f64.sqrt() + 35.0_f64.sqrt() / 4.0 - 1.0 + 4.5 + core::f64::consts::SQRT_2;
    Ok(MeshFixture { name: "mixed_all_shapes", mesh, volume: cell_volumes.iter().sum(), cell_volumes, area, boundary_faces })
}
