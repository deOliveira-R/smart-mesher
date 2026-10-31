//! Checks on the in-process meshes (rows 6, 7, 8, 9, 11, 12): validity, conformity, analytic
//! volumes and areas, quality and the smoothing invariants.

use std::collections::{BTreeMap, BTreeSet};

use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh_api::{
    CellIx, CellShape, FaceIx, FaceKind, GeomTol, MeshRead, VertexIx, cell_volume, face_area_vector, validate,
    validate_geometry,
};

/// Vertex indices of cell `c` (its corners, else the union of its face loops), sorted.
pub fn cell_vertices(m: &dyn MeshRead, c: CellIx) -> Vec<u32> {
    if let Some(cs) = m.cell_corners(c) {
        let mut v: Vec<u32> = cs.iter().map(|x| x.get() as u32).collect();
        v.sort_unstable();
        v.dedup();
        return v;
    }
    let mut v: Vec<u32> = m
        .cell_faces(c)
        .iter()
        .flat_map(|sf| m.face_vertices(sf.face()).iter().map(|x| x.get() as u32))
        .collect();
    v.sort_unstable();
    v.dedup();
    v
}

pub fn face_area(m: &dyn MeshRead, f: FaceIx) -> f64 {
    let a = face_area_vector(m, f);
    (a.x * a.x + a.y * a.y + a.z * a.z).sqrt()
}

/// Cells per shape.
pub fn shapes(m: &dyn MeshRead) -> BTreeMap<String, usize> {
    let mut out = BTreeMap::new();
    for c in 0..m.cell_count() {
        *out.entry(format!("{:?}", m.cell_shape(CellIx::new(c)))).or_insert(0) += 1;
    }
    out
}

/// Volume per classifying model entity, and the total.
pub fn volumes(m: &dyn MeshRead) -> (BTreeMap<EntityId, f64>, f64) {
    let mut per = BTreeMap::new();
    let mut tot = 0.0;
    for c in 0..m.cell_count() {
        let c = CellIx::new(c);
        let v = cell_volume(m, c);
        tot += v;
        *per.entry(m.cell_class(c).entity).or_insert(0.0) += v;
    }
    (per, tot)
}

/// `sm_mesh_api::validate` + `validate_geometry` (default tolerances): the codes found, counted.
pub fn independent_validate(m: &dyn MeshRead) -> BTreeMap<String, usize> {
    let mut out = BTreeMap::new();
    let mut v = validate(m);
    if v.is_empty() {
        v = validate_geometry(m, &GeomTol::default());
    }
    for x in v {
        *out.entry(format!("{:?}", x.code)).or_insert(0) += 1;
    }
    out
}

/// Row 7 facts of the hybrid mesh `m`.
#[derive(Debug, Default)]
pub struct Conformity {
    pub hexes: usize,
    pub wedges: usize,
    pub tets: usize,
    pub other: usize,
    pub source_quads: usize,
    pub source_tris: usize,
    pub interface_tris: usize,
    /// Interface triangles whose two cells are not one wedge and one tet.
    pub bad_interface: Vec<String>,
    /// One-sided faces not on the outer surface, two-sided faces on the outer surface.
    pub bad_sides: Vec<String>,
    pub nonconformal: usize,
    pub boundary_area: f64,
    pub total_volume: f64,
    pub per_volume: BTreeMap<EntityId, f64>,
}

/// `shared` are the three interface model faces; `s1`, `s2` the sweep sources, `t2` the V2|V3
/// triangle.
pub fn conformity(m: &dyn MeshRead, shared: &BTreeSet<EntityId>, s1: EntityId, s2: EntityId, t2: EntityId) -> Conformity {
    let mut r = Conformity::default();
    for c in 0..m.cell_count() {
        match m.cell_shape(CellIx::new(c)) {
            CellShape::Hex => r.hexes += 1,
            CellShape::Wedge => r.wedges += 1,
            CellShape::Tet => r.tets += 1,
            _ => r.other += 1,
        }
    }
    for f in 0..m.face_count() {
        let fi = FaceIx::new(f);
        let class = m.face_class(fi).entity;
        let nv = m.face_vertices(fi).len();
        let (l, rgt) = m.face_cells(fi);
        if m.face_kind(fi) == FaceKind::NonConformal {
            r.nonconformal += 1;
        }
        if class == s1 && nv == 4 {
            r.source_quads += 1;
        }
        if class == s2 && nv == 3 {
            r.source_tris += 1;
        }
        if class == t2 && nv == 3 {
            r.interface_tris += 1;
            let mut s: Vec<String> = std::iter::once(l).chain(rgt).map(|c| format!("{:?}", m.cell_shape(c))).collect();
            s.sort();
            if s != ["Tet", "Wedge"] && r.bad_interface.len() < 5 {
                r.bad_interface.push(format!("face {f}: {s:?}"));
            }
        }
        let on_shared = shared.contains(&class);
        match rgt {
            None => {
                r.boundary_area += face_area(m, fi);
                if on_shared && r.bad_sides.len() < 5 {
                    r.bad_sides.push(format!("face {f} on interface {class} has one cell"));
                }
                if class.kind() != EntityKind::TopoFace && r.bad_sides.len() < 5 {
                    r.bad_sides.push(format!("one-sided face {f} is classified on {class}, not a model face"));
                }
            }
            Some(_) => {
                if class.kind() == EntityKind::TopoFace && !on_shared && r.bad_sides.len() < 5 {
                    r.bad_sides.push(format!("two-sided face {f} is classified on outer face {class}"));
                }
            }
        }
    }
    let (per, tot) = volumes(m);
    r.per_volume = per;
    r.total_volume = tot;
    r
}

/// Minimum scaled Jacobian per cell shape.
pub fn min_sj(m: &dyn MeshRead) -> Result<BTreeMap<String, f64>, String> {
    let col = sm_quality::compute(m, sm_quality::Metric::ScaledJacobian, &sm_quality::Options::default()).map_err(|x| x.to_string())?;
    let mut out: BTreeMap<String, f64> = BTreeMap::new();
    for (c, v) in col.iter().enumerate() {
        if v.is_nan() {
            continue;
        }
        let k = format!("{:?}", m.cell_shape(CellIx::new(c)));
        let e = out.entry(k).or_insert(f64::INFINITY);
        *e = e.min(*v);
    }
    Ok(out)
}

fn p(m: &dyn MeshRead, v: usize) -> Point3 {
    m.point(v)
}

fn sub(a: Point3, b: Point3) -> [f64; 3] {
    [a.x - b.x, a.y - b.y, a.z - b.z]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

/// Smoothing invariants between the mesh before (`pre`) and after (`post`) `mesh.smooth`.
#[derive(Debug, Default)]
pub struct SmoothCheck {
    pub topology_same: bool,
    pub vertex_nodes_moved: usize,
    pub max_curve_dev: f64,
    pub max_surface_dev: f64,
    pub moved: usize,
}

pub fn smoothing(pre: &dyn MeshRead, post: &dyn MeshRead) -> SmoothCheck {
    let mut r = SmoothCheck {
        topology_same: pre.vertex_count() == post.vertex_count()
            && pre.cell_count() == post.cell_count()
            && pre.face_count() == post.face_count()
            && (0..pre.face_count()).all(|f| pre.face_vertices(FaceIx::new(f)) == post.face_vertices(FaceIx::new(f))),
        ..SmoothCheck::default()
    };
    if !r.topology_same {
        return r;
    }
    // Lines and planes from the pre-smoothing positions.
    let mut by_entity: BTreeMap<EntityId, Vec<usize>> = BTreeMap::new();
    for v in 0..pre.vertex_count() {
        by_entity.entry(pre.vertex_class(VertexIx::new(v)).entity).or_default().push(v);
    }
    // Plane normals of the model faces: sum of the area vectors of the mesh faces on them.
    let mut normals: BTreeMap<EntityId, [f64; 3]> = BTreeMap::new();
    for f in 0..pre.face_count() {
        let fi = FaceIx::new(f);
        let class = pre.face_class(fi).entity;
        if class.kind() == EntityKind::TopoFace {
            let a = face_area_vector(pre, fi);
            let n = normals.entry(class).or_insert([0.0; 3]);
            // Orient consistently with the first face.
            let s = if dot(*n, [a.x, a.y, a.z]) < 0.0 { -1.0 } else { 1.0 };
            n[0] += s * a.x;
            n[1] += s * a.y;
            n[2] += s * a.z;
        }
    }
    for (ent, vs) in &by_entity {
        let moved = vs.iter().filter(|&&v| p(pre, v) != p(post, v)).count();
        r.moved += moved;
        match ent.kind() {
            EntityKind::TopoVertex => r.vertex_nodes_moved += moved,
            EntityKind::TopoEdge => {
                // Line through the two pre points farthest apart.
                let a = vs[0];
                let b = *vs.iter().max_by(|&&x, &&y| norm(sub(p(pre, x), p(pre, a))).total_cmp(&norm(sub(p(pre, y), p(pre, a))))).unwrap_or(&a);
                let c = *vs.iter().max_by(|&&x, &&y| norm(sub(p(pre, x), p(pre, b))).total_cmp(&norm(sub(p(pre, y), p(pre, b))))).unwrap_or(&b);
                let d = sub(p(pre, c), p(pre, b));
                let l = norm(d);
                if l == 0.0 {
                    continue;
                }
                for &v in vs {
                    let dev = norm(cross(sub(p(post, v), p(pre, b)), d)) / l;
                    r.max_curve_dev = r.max_curve_dev.max(dev);
                }
            }
            EntityKind::TopoFace => {
                let Some(n) = normals.get(ent) else { continue };
                let l = norm(*n);
                if l == 0.0 {
                    continue;
                }
                let n = [n[0] / l, n[1] / l, n[2] / l];
                // Plane offset: mean of the pre positions.
                let k = vs.len() as f64;
                let off = vs.iter().map(|&v| dot(n, [p(pre, v).x, p(pre, v).y, p(pre, v).z])).sum::<f64>() / k;
                for &v in vs {
                    let q = p(post, v);
                    r.max_surface_dev = r.max_surface_dev.max((dot(n, [q.x, q.y, q.z]) - off).abs());
                }
            }
            _ => {}
        }
    }
    r
}

/// Area of the mesh faces classified on any of `faces`.
pub fn area_on(m: &dyn MeshRead, faces: &BTreeSet<EntityId>) -> f64 {
    (0..m.face_count()).map(FaceIx::new).filter(|&f| faces.contains(&m.face_class(f).entity)).map(|f| face_area(m, f)).sum()
}

/// Number of distinct (vertex, classifying volume) pairs over the cells (row 11).
pub fn vertex_volume_pairs(m: &dyn MeshRead) -> usize {
    let mut pairs = BTreeSet::new();
    for c in 0..m.cell_count() {
        let c = CellIx::new(c);
        let vol = m.cell_class(c).entity;
        for v in cell_vertices(m, c) {
            pairs.insert((v, vol));
        }
    }
    pairs.len()
}
