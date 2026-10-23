//! Validators and the content hash. Both work on any [`MeshRead`] implementor.

use crate::geometry::{cell_volume_centroid, face_geometry};
use crate::index::{CellFace, CellIx, FaceIx, MeshEntity, Side, VertexIx};
use crate::read::MeshRead;
use crate::shape::{CellShape, LoopRelation, loop_relation};
use sm_core::EntityKind;
use std::collections::BTreeMap;

/// Category of a validation finding.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IssueKind {
    /// Stable ids are unordered, duplicated or of the wrong kind.
    Ids,
    /// A face loop is malformed or not canonical.
    Loop,
    /// Left/right ordering or cell->face sides break the orientation rule.
    Orientation,
    /// A cell's faces do not form a closed oriented surface.
    Closedness,
    /// A shape tag or template disagrees with the topology.
    Shape,
    /// Classification is of the wrong kind for the entity.
    Classification,
    /// Geometry disagrees with the declared orientation (normals, volumes).
    Geometry,
}

/// One validation finding.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ValidationIssue {
    /// Category.
    pub kind: IssueKind,
    /// Human-readable detail.
    pub message: String,
}

/// Result of validation. Empty means valid.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ValidationReport {
    /// All findings, in discovery order (deterministic).
    pub issues: Vec<ValidationIssue>,
}

impl ValidationReport {
    /// True if there are no findings.
    pub fn is_ok(&self) -> bool {
        self.issues.is_empty()
    }
    fn push(&mut self, kind: IssueKind, message: String) {
        self.issues.push(ValidationIssue { kind, message });
    }
}

/// Topological validation: ids, loops, orientation rule, closedness, shapes, classification kinds.
pub fn validate<M: MeshRead + ?Sized>(m: &M) -> ValidationReport {
    let mut r = ValidationReport::default();
    let (nv, nf, nc) = (m.vertex_count(), m.face_count(), m.cell_count());

    // Ids: strictly ascending, right kind.
    for (n, kind, get) in [
        (nv, EntityKind::MeshVertex, &(|i| m.vertex_id(VertexIx::new(i))) as &dyn Fn(usize) -> sm_core::EntityId),
        (nf, EntityKind::MeshFace, &|i| m.face_id(FaceIx::new(i))),
        (nc, EntityKind::MeshCell, &|i| m.cell_id(CellIx::new(i))),
    ] {
        for i in 0..n {
            if get(i).kind() != kind || (i > 0 && get(i - 1) >= get(i)) {
                r.push(IssueKind::Ids, format!("{kind:?} id at index {i} is wrong kind or not ascending"));
                break;
            }
        }
    }

    let np = m.coords().len();
    let mut seen_cells: Vec<Vec<(CellIx, Side)>> = vec![Vec::new(); nf];
    for c in 0..nc {
        let c = CellIx::new(c);
        let mut prev: Option<FaceIx> = None;
        for &CellFace { face, side } in m.cell_faces(c) {
            if face.get() >= nf || prev.is_some_and(|p| p >= face) {
                r.push(IssueKind::Orientation, format!("cell {c:?} faces out of range or not ascending"));
                break;
            }
            prev = Some(face);
            let expect = if m.face_left(face) == c { Side::Left } else { Side::Right };
            if side != expect {
                r.push(IssueKind::Orientation, format!("cell {c:?} side for face {face:?} disagrees with face_left"));
            }
            seen_cells[face.get()].push((c, side));
        }
    }

    for (f, seen) in seen_cells.iter().enumerate() {
        let fi = FaceIx::new(f);
        let l = m.face_vertices(fi);
        let mut sorted = l.to_vec();
        sorted.sort_unstable();
        sorted.dedup();
        if l.len() < 3 || sorted.len() != l.len() || l.iter().any(|v| v.get() >= nv) {
            r.push(IssueKind::Loop, format!("face {fi:?} loop invalid"));
            continue;
        }
        if l[0] != sorted[0] {
            r.push(IssueKind::Loop, format!("face {fi:?} loop does not start at its smallest vertex"));
        }
        let left = m.face_left(fi);
        if let Some(rt) = m.face_right(fi)
            && left >= rt {
                r.push(IssueKind::Orientation, format!("face {fi:?} left cell is not lower than right"));
            }
        let expect: Vec<(CellIx, Side)> = std::iter::once((left, Side::Left)).chain(m.face_right(fi).map(|c| (c, Side::Right))).collect();
        if *seen != expect {
            r.push(IssueKind::Orientation, format!("face {fi:?} cell incidence does not match cell_faces"));
        }
    }

    for c in 0..nc {
        let c = CellIx::new(c);
        // Closedness: every directed edge of the outward loops is matched by its reverse.
        let mut count: BTreeMap<(VertexIx, VertexIx), i32> = BTreeMap::new();
        for &CellFace { face, side } in m.cell_faces(c) {
            let l = m.face_vertices(face);
            let n = l.len();
            for i in 0..n {
                let (a, b) = match side {
                    Side::Left => (l[i], l[(i + 1) % n]),
                    Side::Right => (l[(i + 1) % n], l[i]),
                };
                *count.entry((a, b)).or_insert(0) += 1;
            }
        }
        if count.iter().any(|(&(a, b), &k)| k != 1 || count.get(&(b, a)) != Some(&1)) {
            r.push(IssueKind::Closedness, format!("cell {c:?} is not a closed oriented surface"));
        }
        // Shape tag and template.
        let sizes = m.cell_faces(c).iter().map(|cf| m.face_vertices(cf.face).len());
        let detected = CellShape::detect(sizes, m.cell_vertices_sorted(c).len());
        if detected != m.cell_shape(c) {
            r.push(IssueKind::Shape, format!("cell {c:?} tag {:?} but topology is {detected:?}", m.cell_shape(c)));
        }
        if let Some(ov) = m.cell_ordered_vertices(c) {
            let shape = m.cell_shape(c);
            if shape.vertex_count() != Some(ov.len()) {
                r.push(IssueKind::Shape, format!("cell {c:?} ordered vertices do not fit {shape:?}"));
            } else {
                for lf in shape.local_faces() {
                    let want: Vec<VertexIx> = lf.iter().map(|&i| ov[i as usize]).collect();
                    let ok = m.cell_faces(c).iter().any(|cf| {
                        let l = m.face_vertices(cf.face);
                        loop_relation(&want, l)
                            == Some(if cf.side == Side::Left { LoopRelation::Same } else { LoopRelation::Reversed })
                    });
                    if !ok {
                        r.push(IssueKind::Shape, format!("cell {c:?} template face {lf:?} not found with outward orientation"));
                    }
                }
            }
        }
        if let Some(h) = m.high_order_cell(c)
            && (h.degree < 2 || h.nodes.iter().any(|&n| n as usize >= np)) {
                r.push(IssueKind::Shape, format!("cell {c:?} high-order record invalid"));
            }
    }

    // Classification kinds.
    let ok_vol = |k: EntityKind| matches!(k, EntityKind::TopoVolume | EntityKind::PartitionCell);
    for v in 0..nv {
        let v = VertexIx::new(v);
        if m.vertex_class(v).dim().is_none() {
            r.push(IssueKind::Classification, format!("vertex {v:?} classified on a non model/partition entity"));
        }
    }
    for f in 0..nf {
        let fi = FaceIx::new(f);
        let k = m.face_class(fi).entity.kind();
        let ok = if m.is_boundary_face(fi) { k == EntityKind::TopoFace } else { k == EntityKind::TopoFace || ok_vol(k) };
        if !ok {
            r.push(IssueKind::Classification, format!("face {fi:?} classified on {k:?}"));
        }
    }
    for c in 0..nc {
        let c = CellIx::new(c);
        if !ok_vol(m.cell_class(c).entity.kind()) {
            r.push(IssueKind::Classification, format!("cell {c:?} must be classified on a volume or partition cell"));
        }
    }
    r
}

/// Geometric orientation check: every face normal points from its left cell to its right cell
/// (outward on the boundary), and every cell volume is positive.
pub fn validate_geometry<M: MeshRead + ?Sized>(m: &M) -> ValidationReport {
    let mut r = ValidationReport::default();
    let centroids: Vec<_> = (0..m.cell_count()).map(|c| cell_volume_centroid(m, CellIx::new(c))).collect();
    for (c, (vol, _)) in centroids.iter().enumerate() {
        if *vol <= 0.0 {
            r.push(IssueKind::Geometry, format!("cell {c} has non-positive volume {vol}"));
        }
    }
    for f in 0..m.face_count() {
        let fi = FaceIx::new(f);
        let g = face_geometry(m, fi);
        let lc = centroids[m.face_left(fi).get()].1;
        let target = match m.face_right(fi) {
            Some(rc) => centroids[rc.get()].1 - lc,
            None => g.centroid - lc,
        };
        if g.area_vector.dot(target) <= 0.0 {
            r.push(IssueKind::Geometry, format!("face {f} normal does not point left -> right/outward"));
        }
    }
    r
}

/// Deterministic FNV-1a hash of everything a mesh stores (ids, topology, bit-exact coordinates,
/// classification, sets, tags). Equal canonical meshes hash equal on any platform.
pub fn content_hash<M: MeshRead + ?Sized>(m: &M) -> u64 {
    struct H(u64);
    impl H {
        fn u(&mut self, x: u64) {
            for b in x.to_le_bytes() {
                self.0 = (self.0 ^ u64::from(b)).wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
    }
    let mut h = H(0xcbf2_9ce4_8422_2325);
    h.u(m.mesh_id().to_raw());
    let (nv, nf, nc) = (m.vertex_count(), m.face_count(), m.cell_count());
    for n in [nv, nf, nc] {
        h.u(n as u64);
    }
    let co = m.coords();
    h.u(co.len() as u64);
    for i in 0..co.len() {
        h.u(co.x[i].to_bits());
        h.u(co.y[i].to_bits());
        h.u(co.z[i].to_bits());
    }
    for v in 0..nv {
        let v = VertexIx::new(v);
        h.u(m.vertex_id(v).to_raw());
        let c = m.vertex_class(v);
        h.u(c.entity.to_raw());
    }
    for f in 0..nf {
        let f = FaceIx::new(f);
        h.u(m.face_id(f).to_raw());
        h.u(m.face_left(f).0.into());
        h.u(m.face_right(f).map_or(u64::MAX, |c| c.0.into()));
        h.u(m.face_class(f).entity.to_raw());
        for v in m.face_vertices(f) {
            h.u(v.0.into());
        }
    }
    for c in 0..nc {
        let c = CellIx::new(c);
        h.u(m.cell_id(c).to_raw());
        h.u(m.cell_shape(c) as u64);
        h.u(m.cell_class(c).entity.to_raw());
        for cf in m.cell_faces(c) {
            h.u(u64::from(cf.face.0) << 1 | u64::from(cf.side == Side::Right));
        }
        for v in m.cell_ordered_vertices(c).unwrap_or(&[]) {
            h.u(v.0.into());
        }
    }
    for s in m.sets() {
        h.u(s.id.to_raw());
        h.u(s.members.len() as u64);
    }
    h.0
}

/// The `MeshEntity` of every entity of a mesh in canonical order (vertices, faces, cells).
pub fn all_entities<M: MeshRead + ?Sized>(m: &M) -> Vec<MeshEntity> {
    (0..m.vertex_count())
        .map(|i| MeshEntity::Vertex(VertexIx::new(i)))
        .chain((0..m.face_count()).map(|i| MeshEntity::Face(FaceIx::new(i))))
        .chain((0..m.cell_count()).map(|i| MeshEntity::Cell(CellIx::new(i))))
        .collect()
}
