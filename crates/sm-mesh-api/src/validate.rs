//! Validators and content hashes. All of them work on any [`MeshRead`] implementor.
//!
//! [`validate`] checks everything that does not need coordinates; [`validate_geometry`] checks
//! that the stored orientation agrees with the coordinates. Findings carry a stable
//! [`ViolationCode`].

#![allow(clippy::needless_range_loop)] // dense-index loops over parallel columns are the clearest form here

use crate::classify::ClassSet;
use crate::geometry::{cell_volume_centroid, face_geometry};
use crate::highorder::{face_node_count, node_count};
use crate::index::{CellIx, EdgeIx, FaceIx, FaceKind, MeshEntity, MeshEntityKind, Side, VertexIx};
use crate::read::MeshRead;
use crate::sets::TagValues;
use crate::shape::{CellShape, LoopRelation, derive_corners, loop_relation};
use sm_core::{EntityId, EntityKind, Point3};
use std::collections::{BTreeMap, BTreeSet};

/// Stable machine-readable code of a [`Violation`]; [`ViolationCode::as_str`] never changes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub enum ViolationCode {
    /// Ids are unordered, duplicated or of the wrong kind.
    IdsOrder,
    /// Columns or indices are inconsistent in size or out of range.
    IndexRange,
    /// A face loop is malformed.
    LoopInvalid,
    /// A face loop does not start at its smallest vertex.
    LoopNotCanonical,
    /// Faces are not in canonical `(left, right, loop)` order.
    FaceOrder,
    /// `left` is not lower than `right`, or the right cell disagrees with the face kind.
    FaceLeftRight,
    /// A cell's face list is not ascending / in range, or repeats a face.
    CellFacesOrder,
    /// A cell's sign for a face disagrees with `face_left` / `face_right`.
    CellSide,
    /// The face->cell incidence does not match the cell->face lists.
    Incidence,
    /// A cell's faces do not form a closed oriented surface (directed loop edges do not cancel).
    Closedness,
    /// A shape tag disagrees with the topology.
    ShapeTag,
    /// Template corners are missing, or do not realise the template faces.
    Corners,
    /// A high-order cell record is invalid.
    HighOrder,
    /// A face curve record is invalid.
    FaceCurve,
    /// A classification names a non-model entity.
    ClassEntity,
    /// A classification parameter does not fit the classified entity.
    ClassParam,
    /// A face, cell or edge is classified on an entity of the wrong dimension for its role.
    ClassKind,
    /// A sub-entity is classified on a higher-dimensional entity than its parent
    /// (for example an edge on a `TopoFace` with a vertex on a `TopoVolume`).
    ClassDimOrder,
    /// The reverse classification index disagrees with the per-entity columns.
    ClassIndex,
    /// Lineage columns are inconsistent.
    Lineage,
    /// A set is invalid.
    Set,
    /// A tag column is invalid.
    Tag,
    /// The edge table is inconsistent with the faces.
    EdgeTable,
    /// The geometry cache does not match the mesh.
    Cache,
    /// A cell volume is not positive.
    GeomVolume,
    /// A face normal does not point from its left cell towards its right cell (outward at one-cell faces).
    GeomNormal,
    /// A fan side tetrahedron has non-positive volume.
    GeomSideTet,
    /// A face is not planar within the tolerance.
    GeomPlanar,
}

impl ViolationCode {
    /// The stable string form, `"group.detail"`.
    pub const fn as_str(self) -> &'static str {
        use ViolationCode::*;
        match self {
            IdsOrder => "ids.order",
            IndexRange => "index.range",
            LoopInvalid => "loop.invalid",
            LoopNotCanonical => "loop.canonical",
            FaceOrder => "face.order",
            FaceLeftRight => "face.left_right",
            CellFacesOrder => "cell.faces_order",
            CellSide => "cell.side",
            Incidence => "cell.incidence",
            Closedness => "cell.closed",
            ShapeTag => "shape.tag",
            Corners => "shape.corners",
            HighOrder => "highorder.cell",
            FaceCurve => "highorder.face_curve",
            ClassEntity => "class.entity",
            ClassParam => "class.param",
            ClassKind => "class.kind",
            ClassDimOrder => "class.dim_order",
            ClassIndex => "class.index",
            Lineage => "lineage",
            Set => "set",
            Tag => "tag",
            EdgeTable => "edges.table",
            Cache => "cache",
            GeomVolume => "geom.volume",
            GeomNormal => "geom.normal",
            GeomSideTet => "geom.side_tet",
            GeomPlanar => "geom.planar",
        }
    }
}

/// One validation finding.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Violation {
    /// Stable code.
    pub code: ViolationCode,
    /// The offending entity, if one can be named.
    pub entity: Option<MeshEntity>,
    /// Human-readable detail.
    pub detail: String,
}

impl core::fmt::Display for Violation {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "[{}] ", self.code.as_str())?;
        if let Some(e) = self.entity {
            write!(f, "{e:?}: ")?;
        }
        f.write_str(&self.detail)
    }
}

struct Sink(Vec<Violation>);
impl Sink {
    fn add(&mut self, code: ViolationCode, entity: Option<MeshEntity>, detail: impl Into<String>) {
        self.0.push(Violation {
            code,
            entity,
            detail: detail.into(),
        });
    }
}

const fn is_volume(k: EntityKind) -> bool {
    matches!(k, EntityKind::TopoVolume | EntityKind::PartitionCell)
}

/// Topological validation. Empty result means valid. Checks ids and column sizes, loops,
/// canonical order, the orientation rule in both directions, closedness (directed loop edges
/// cancel per cell), shape tags and template corners, high-order and face-curve records,
/// classification (kinds, parameters, dimension order including edges, reverse index), lineage,
/// sets, tags, the edge table and the geometry cache.
pub fn validate(m: &dyn MeshRead) -> Vec<Violation> {
    let mut s = Sink(Vec::new());
    let (nv, nf, nc) = (
        m.count(MeshEntityKind::Vertex),
        m.count(MeshEntityKind::Face),
        m.count(MeshEntityKind::Cell),
    );
    let ne = m.count(MeshEntityKind::Edge);
    let co = m.coords();
    let np = co.len();
    if co.y.len() != np || co.z.len() != np || np < nv {
        s.add(
            ViolationCode::IndexRange,
            None,
            "coordinate arrays have inconsistent lengths or fewer points than vertices",
        );
        return s.0;
    }
    let cols = m.columns();
    if cols.face_vertex_offsets.len() != nf + 1
        || cols.face_left.len() != nf
        || cols.face_right.len() != nf
        || cols.cell_face_offsets.len() != nc + 1
    {
        s.add(
            ViolationCode::IndexRange,
            None,
            "topology column sizes disagree with the entity counts",
        );
        return s.0;
    }

    // ids
    for (k, n) in [
        (MeshEntityKind::Vertex, nv),
        (MeshEntityKind::Edge, ne),
        (MeshEntityKind::Face, nf),
        (MeshEntityKind::Cell, nc),
    ] {
        let want = match k {
            MeshEntityKind::Vertex => EntityKind::MeshVertex,
            MeshEntityKind::Edge => EntityKind::MeshEdge,
            MeshEntityKind::Face => EntityKind::MeshFace,
            MeshEntityKind::Cell => EntityKind::MeshCell,
        };
        for i in 0..n {
            let id = m.id(k, i as u32);
            if id.kind() != want
                || (i > 0 && m.id(k, i as u32 - 1) >= id)
                || m.index_of(id) != Some(i as u32)
            {
                s.add(
                    ViolationCode::IdsOrder,
                    Some(MeshEntity::from_parts(k, i as u32)),
                    format!("{want:?} id is wrong kind, not ascending, or not found by index_of"),
                );
                break;
            }
        }
    }
    if m.mesh_id().kind() != EntityKind::Mesh {
        s.add(ViolationCode::IdsOrder, None, "mesh id is not of kind Mesh");
    }

    // faces
    let mut loops_ok = vec![true; nf];
    for f in 0..nf {
        let fi = FaceIx::new(f);
        let e = Some(MeshEntity::Face(fi));
        let l = m.face_vertices(fi);
        let mut sorted = l.to_vec();
        sorted.sort_unstable();
        sorted.dedup();
        if l.len() < 3 || sorted.len() != l.len() || l.iter().any(|v| v.get() >= nv) {
            s.add(
                ViolationCode::LoopInvalid,
                e,
                "loop has < 3 vertices, repeats a vertex, or names a missing vertex",
            );
            loops_ok[f] = false;
            continue;
        }
        if l[0] != sorted[0] {
            s.add(
                ViolationCode::LoopNotCanonical,
                e,
                "loop does not start at its smallest vertex",
            );
        }
        let (left, right, kind) = (m.face_left(fi), m.face_right(fi), m.face_kind(fi));
        if left.get() >= nc || right.is_some_and(|r| r.get() >= nc) {
            s.add(ViolationCode::IndexRange, e, "face cell out of range");
            loops_ok[f] = false;
            continue;
        }
        match (kind, right) {
            (FaceKind::Interior, None) => s.add(
                ViolationCode::FaceLeftRight,
                e,
                "Interior face has no right cell",
            ),
            (FaceKind::Interior, Some(r)) if left >= r => s.add(
                ViolationCode::FaceLeftRight,
                e,
                "left cell is not lower than right",
            ),
            (FaceKind::Boundary | FaceKind::NonConformal | FaceKind::PartitionCut, Some(_)) => s
                .add(
                    ViolationCode::FaceLeftRight,
                    e,
                    format!("{kind:?} face must have exactly one cell"),
                ),
            _ => {}
        }
    }
    for f in 1..nf {
        if !loops_ok[f] || !loops_ok[f - 1] {
            continue;
        }
        let (a, b) = (FaceIx::new(f - 1), FaceIx::new(f));
        let key = |x: FaceIx| {
            (
                m.face_left(x),
                m.face_right(x).map_or(u32::MAX, |c| c.0),
                m.face_vertices(x),
            )
        };
        if key(a) >= key(b) {
            s.add(
                ViolationCode::FaceOrder,
                Some(MeshEntity::Face(b)),
                "faces are not strictly ascending by (left, right, loop)",
            );
        }
    }

    // cells -> faces, incidence
    let mut seen: Vec<Vec<(CellIx, Side)>> = vec![Vec::new(); nf];
    let mut cell_ok = vec![true; nc];
    for c in 0..nc {
        let ci = CellIx::new(c);
        let mut prev: Option<FaceIx> = None;
        for &sf in m.cell_faces(ci) {
            let face = sf.face();
            if face.get() >= nf || prev.is_some_and(|p| p >= face) {
                s.add(
                    ViolationCode::CellFacesOrder,
                    Some(MeshEntity::Cell(ci)),
                    "faces out of range, repeated or not ascending",
                );
                cell_ok[c] = false;
                break;
            }
            prev = Some(face);
            let expect = if m.face_left(face) == ci {
                Side::Left
            } else {
                Side::Right
            };
            if sf.side() != expect {
                s.add(
                    ViolationCode::CellSide,
                    Some(MeshEntity::Cell(ci)),
                    format!("sign for face {face:?} disagrees with face_left"),
                );
            }
            seen[face.get()].push((ci, sf.side()));
        }
    }
    for (f, got) in seen.iter().enumerate() {
        if !loops_ok[f] {
            continue;
        }
        let fi = FaceIx::new(f);
        let expect: Vec<(CellIx, Side)> = std::iter::once((m.face_left(fi), Side::Left))
            .chain(m.face_right(fi).map(|c| (c, Side::Right)))
            .collect();
        if *got != expect {
            s.add(
                ViolationCode::Incidence,
                Some(MeshEntity::Face(fi)),
                "cell incidence does not match cell_faces",
            );
        }
    }

    // per cell: closedness, shape, corners, high order
    for c in 0..nc {
        if !cell_ok[c]
            || m.cell_faces(CellIx::new(c))
                .iter()
                .any(|sf| !loops_ok[sf.face().get()])
        {
            continue;
        }
        validate_cell(m, CellIx::new(c), np, &mut s);
    }

    // face curves
    for f in 0..nf {
        if !loops_ok[f] {
            continue;
        }
        let fi = FaceIx::new(f);
        if let Some(fc) = m.face_curve(fi) {
            let l = m.face_vertices(fi);
            let e = Some(MeshEntity::Face(fi));
            if face_node_count(l.len(), fc.degree, fc.basis) != Some(fc.nodes.len()) {
                s.add(
                    ViolationCode::FaceCurve,
                    e,
                    "node count does not match (loop length, degree, basis)",
                );
            } else if fc.nodes.iter().any(|n| n.get() >= np) {
                s.add(ViolationCode::FaceCurve, e, "node out of range");
            } else if fc.nodes.iter().zip(l).any(|(n, v)| n.get() != v.get()) {
                s.add(
                    ViolationCode::FaceCurve,
                    e,
                    "curve nodes must start with the loop vertices in loop order",
                );
            }
        }
    }

    validate_classification(m, nv, nf, nc, &loops_ok, &mut s);

    // lineage
    for c in 0..nc {
        let ci = CellIx::new(c);
        if let Some(p) = m.cell_parent(ci) {
            let e = Some(MeshEntity::Cell(ci));
            if p.get() >= c {
                s.add(
                    ViolationCode::Lineage,
                    e,
                    "parent must exist and have a lower index than its child",
                );
            } else if m.cell_active(ci) && m.cell_active(p) {
                s.add(
                    ViolationCode::Lineage,
                    e,
                    "an active cell's parent must be inactive",
                );
            }
        }
    }

    validate_sets_tags_edges(m, nv, nf, nc, &loops_ok, &mut s);

    if let Some(g) = m.geometry_cache() {
        let ok = g.rule == m.geometry_rule()
            && g.face_area.len() == nf
            && g.face_centroid.len() == nf
            && g.face_apex.len() == nf
            && g.cell_volume.len() == nc
            && g.cell_centroid.len() == nc;
        if !ok {
            s.add(
                ViolationCode::Cache,
                None,
                "geometry cache rule or sizes do not match the mesh",
            );
        } else if g.connectivity_hash != connectivity_hash(m) {
            s.add(
                ViolationCode::Cache,
                None,
                "geometry cache was computed for different connectivity",
            );
        } else if g.coordinate_hash != coordinate_hash(m) {
            s.add(
                ViolationCode::Cache,
                None,
                "geometry cache was computed for different coordinates",
            );
        }
    }
    let _ = ne;
    s.0
}

fn validate_cell(m: &dyn MeshRead, ci: CellIx, np: usize, s: &mut Sink) {
    let e = Some(MeshEntity::Cell(ci));
    let faces = m.cell_faces(ci);
    // Outward loops, as slices of directed edges.
    let mut dir: Vec<(u32, u32)> = Vec::new();
    let mut verts: Vec<VertexIx> = Vec::new();
    for sf in faces {
        let l = m.face_vertices(sf.face());
        let n = l.len();
        for i in 0..n {
            let (a, b) = (l[i], l[(i + 1) % n]);
            dir.push(if sf.side() == Side::Left {
                (a.0, b.0)
            } else {
                (b.0, a.0)
            });
        }
        verts.extend_from_slice(l);
    }
    dir.sort_unstable();
    let closed = dir.windows(2).all(|w| w[0] != w[1])
        && dir.iter().all(|&(a, b)| dir.binary_search(&(b, a)).is_ok());
    if !closed {
        s.add(
            ViolationCode::Closedness,
            e,
            "directed loop edges do not cancel: not a closed oriented surface",
        );
    }
    verts.sort_unstable();
    verts.dedup();
    let sizes: Vec<usize> = faces
        .iter()
        .map(|sf| m.face_vertices(sf.face()).len())
        .collect();
    let detected = CellShape::detect(verts.len(), &sizes);
    let shape = m.cell_shape(ci);
    let corners = m.cell_corners(ci);
    match (shape, corners) {
        (CellShape::Polyhedron, None) => {
            if detected != CellShape::Polyhedron && closed {
                let loops: Vec<Vec<VertexIx>> = faces
                    .iter()
                    .map(|sf| {
                        let l = m.face_vertices(sf.face());
                        if sf.side() == Side::Left {
                            l.to_vec()
                        } else {
                            l.iter().rev().copied().collect()
                        }
                    })
                    .collect();
                let refs: Vec<&[VertexIx]> = loops.iter().map(Vec::as_slice).collect();
                if derive_corners(detected, &refs).is_some() {
                    s.add(
                        ViolationCode::ShapeTag,
                        e,
                        format!("tagged Polyhedron but the topology is a {detected:?}"),
                    );
                }
            }
        }
        (CellShape::Polyhedron, Some(_)) => s.add(
            ViolationCode::Corners,
            e,
            "a polyhedron cannot carry template corners",
        ),
        (_, None) => s.add(
            ViolationCode::Corners,
            e,
            format!("{shape:?} cell has no corner list"),
        ),
        (sh, Some(ov)) => {
            if sh != detected {
                s.add(
                    ViolationCode::ShapeTag,
                    e,
                    format!("tag {sh:?} but topology is {detected:?}"),
                );
            }
            if sh.vertex_count() != Some(ov.len())
                || ov.iter().any(|v| verts.binary_search(v).is_err())
            {
                s.add(
                    ViolationCode::Corners,
                    e,
                    "corner list does not fit the shape or names foreign vertices",
                );
            } else {
                for lf in sh.local_faces() {
                    let want: Vec<VertexIx> = lf.iter().map(|&i| ov[i as usize]).collect();
                    let ok = faces.iter().any(|sf| {
                        loop_relation(&want, m.face_vertices(sf.face()))
                            == Some(if sf.side() == Side::Left {
                                LoopRelation::Same
                            } else {
                                LoopRelation::Reversed
                            })
                    });
                    if !ok {
                        s.add(
                            ViolationCode::Corners,
                            e,
                            format!("template face {lf:?} not found with outward orientation"),
                        );
                    }
                }
            }
        }
    }
    if let Some(h) = m.high_order_cell(ci) {
        match (corners, node_count(shape, h.degree, h.basis)) {
            _ if shape == CellShape::Polyhedron => s.add(
                ViolationCode::HighOrder,
                e,
                "polyhedra cannot carry a high-order record (use face curves)",
            ),
            (_, _) if h.degree < 2 => {
                s.add(ViolationCode::HighOrder, e, "stored degree must be >= 2")
            }
            (_, None) => s.add(
                ViolationCode::HighOrder,
                e,
                "unsupported (shape, degree, basis)",
            ),
            (_, Some(n)) if n != h.nodes.len() => s.add(
                ViolationCode::HighOrder,
                e,
                format!("expected {n} nodes, found {}", h.nodes.len()),
            ),
            (Some(ov), Some(_)) => {
                if h.nodes.iter().any(|p| p.get() >= np) {
                    s.add(ViolationCode::HighOrder, e, "node out of range");
                } else if h.nodes.iter().zip(ov).any(|(p, v)| p.get() != v.get()) {
                    s.add(
                        ViolationCode::HighOrder,
                        e,
                        "first nodes must equal the cell corners",
                    );
                }
            }
            (None, Some(_)) => s.add(ViolationCode::HighOrder, e, "cell has no corners"),
        }
    }
}

fn validate_classification(
    m: &dyn MeshRead,
    nv: usize,
    nf: usize,
    nc: usize,
    loops_ok: &[bool],
    s: &mut Sink,
) {
    let check = |k: MeshEntityKind, ix: usize, s: &mut Sink| -> Option<u8> {
        let c = m.classification(k, ix as u32);
        let e = Some(MeshEntity::from_parts(k, ix as u32));
        let Some(d) = c.dim() else {
            s.add(
                ViolationCode::ClassEntity,
                e,
                format!(
                    "classified on {:?}, not a model/partition entity",
                    c.entity.kind()
                ),
            );
            return None;
        };
        if !c.param.fits(c.entity.kind()) {
            s.add(
                ViolationCode::ClassParam,
                e,
                format!("parameter {:?} does not fit {:?}", c.param, c.entity.kind()),
            );
        }
        Some(d)
    };
    let vdim: Vec<Option<u8>> = (0..nv)
        .map(|i| check(MeshEntityKind::Vertex, i, s))
        .collect();
    let fdim: Vec<Option<u8>> = (0..nf).map(|i| check(MeshEntityKind::Face, i, s)).collect();
    for c in 0..nc {
        let cl = m.classification(MeshEntityKind::Cell, c as u32);
        let _ = check(MeshEntityKind::Cell, c, s);
        if cl.dim().is_some() && !is_volume(cl.entity.kind()) {
            s.add(
                ViolationCode::ClassKind,
                Some(MeshEntity::Cell(CellIx::new(c))),
                "cells must be classified on a volume or partition cell",
            );
        }
    }
    for f in 0..nf {
        let fi = FaceIx::new(f);
        let k = m.face_class(fi).entity.kind();
        if m.face_class(fi).dim().is_none() {
            continue;
        }
        let ok = match m.face_kind(fi) {
            FaceKind::Boundary => k == EntityKind::TopoFace,
            FaceKind::Interior | FaceKind::NonConformal | FaceKind::PartitionCut => {
                k == EntityKind::TopoFace || is_volume(k)
            }
        };
        if !ok {
            s.add(
                ViolationCode::ClassKind,
                Some(MeshEntity::Face(fi)),
                format!("{:?} face classified on {k:?}", m.face_kind(fi)),
            );
        }
        if loops_ok[f]
            && let Some(fd) = fdim[f]
            && m.face_vertices(fi)
                .iter()
                .any(|v| vdim[v.get()].is_some_and(|d| d > fd))
        {
            s.add(
                ViolationCode::ClassDimOrder,
                Some(MeshEntity::Face(fi)),
                "a face vertex is classified on a higher-dimensional entity than the face",
            );
        }
    }
    if let Some(t) = m.edges() {
        let mut edim: Vec<Option<u8>> = Vec::with_capacity(t.len());
        for (i, c) in t.class.iter().enumerate() {
            let e = Some(MeshEntity::Edge(EdgeIx::new(i)));
            let d = c.dim();
            if d.is_none() {
                s.add(
                    ViolationCode::ClassEntity,
                    e,
                    "edge classified on a non model/partition entity",
                );
            } else if !c.param.fits(c.entity.kind()) {
                s.add(
                    ViolationCode::ClassParam,
                    e,
                    "edge parameter does not fit the classified entity",
                );
            }
            edim.push(d);
            if let (Some(d), Some([a, b])) = (d, t.vertices.get(i))
                && [a, b].iter().any(|v| {
                    vdim.get(v.get())
                        .copied()
                        .flatten()
                        .is_some_and(|vd| vd > d)
                })
            {
                s.add(
                    ViolationCode::ClassDimOrder,
                    e,
                    "an edge vertex is classified on a higher-dimensional entity than the edge",
                );
            }
        }
        for f in 0..nf.min(t.face_off.len().saturating_sub(1)) {
            if let Some(fd) = fdim[f] {
                for fe in t.of_face(FaceIx::new(f)) {
                    if edim
                        .get(fe.edge.get())
                        .copied()
                        .flatten()
                        .is_some_and(|d| d > fd)
                    {
                        s.add(
                            ViolationCode::ClassDimOrder,
                            Some(MeshEntity::Edge(fe.edge)),
                            format!("edge is classified above its face {f}"),
                        );
                    }
                }
            }
        }
    }
    // Reverse index must equal the recomputed one.
    let mut want: BTreeMap<EntityId, ClassSet> = BTreeMap::new();
    for i in 0..nv {
        want.entry(m.vertex_class(VertexIx::new(i)).entity)
            .or_default()
            .vertices
            .push(VertexIx::new(i));
    }
    if let Some(t) = m.edges() {
        for (i, c) in t.class.iter().enumerate() {
            want.entry(c.entity).or_default().edges.push(EdgeIx::new(i));
        }
    }
    for i in 0..nf {
        want.entry(m.face_class(FaceIx::new(i)).entity)
            .or_default()
            .faces
            .push(FaceIx::new(i));
    }
    for i in 0..nc {
        want.entry(m.cell_class(CellIx::new(i)).entity)
            .or_default()
            .cells
            .push(CellIx::new(i));
    }
    let ents: Vec<EntityId> = want.keys().copied().collect();
    if m.classified_entities() != ents.as_slice()
        || want.iter().any(|(k, v)| m.classified_on(*k) != Some(v))
    {
        s.add(
            ViolationCode::ClassIndex,
            None,
            "reverse classification index disagrees with the per-entity classifications",
        );
    }
}

fn validate_sets_tags_edges(
    m: &dyn MeshRead,
    nv: usize,
    nf: usize,
    nc: usize,
    loops_ok: &[bool],
    s: &mut Sink,
) {
    let sets = m.sets();
    for (i, st) in sets.iter().enumerate() {
        let n = match st.members.kind() {
            MeshEntityKind::Vertex => nv,
            MeshEntityKind::Face => nf,
            MeshEntityKind::Cell => nc,
            MeshEntityKind::Edge => 0,
        };
        let raw = st.members.raw();
        if st.id.kind() != EntityKind::Group
            || (i > 0 && sets[i - 1].id >= st.id)
            || sets[..i].iter().any(|o| o.name == st.name)
            || raw.windows(2).any(|w| w[0] >= w[1])
            || raw.last().is_some_and(|&x| x as usize >= n)
        {
            s.add(ViolationCode::Set, None, format!("set '{}' is invalid (id kind/order, duplicate name, or members not ascending/in range)", st.name));
        }
    }
    let tags = m.tags();
    for (i, t) in tags.iter().enumerate() {
        let n = m.count(t.target);
        if t.values.len() != n
            || (i > 0 && (tags[i - 1].target, &tags[i - 1].name) >= (t.target, &t.name))
        {
            s.add(
                ViolationCode::Tag,
                None,
                format!(
                    "tag '{}' has wrong length, is duplicated, or is out of order",
                    t.name
                ),
            );
        }
        if t.target == MeshEntityKind::Edge && m.edges().is_none() {
            s.add(
                ViolationCode::Tag,
                None,
                format!("edge tag '{}' without an edge table", t.name),
            );
        }
    }
    if let Some(t) = m.edges() {
        let mut want: BTreeSet<[VertexIx; 2]> = BTreeSet::new();
        let mut face_ok = true;
        for f in 0..nf {
            if !loops_ok[f] {
                face_ok = false;
                continue;
            }
            let l = m.face_vertices(FaceIx::new(f));
            for i in 0..l.len() {
                want.insert(crate::edges::edge_key(l[i], l[(i + 1) % l.len()]));
            }
        }
        let have: Vec<[VertexIx; 2]> = t.vertices.clone();
        let consistent = t.ids.len() == have.len()
            && t.class.len() == have.len()
            && t.face_off.len() == nf + 1
            && t.ids.windows(2).all(|w| w[0] < w[1])
            && t.ids.iter().all(|i| i.kind() == EntityKind::MeshEdge)
            && have.windows(2).all(|w| w[0] < w[1])
            && have.iter().all(|[a, b]| a < b);
        if !consistent || (face_ok && have != want.iter().copied().collect::<Vec<_>>()) {
            s.add(
                ViolationCode::EdgeTable,
                None,
                "edge table is not the sorted unique edge set of the faces",
            );
        } else if face_ok {
            for f in 0..nf {
                let l = m.face_vertices(FaceIx::new(f));
                let fe = t.of_face(FaceIx::new(f));
                let ok = fe.len() == l.len()
                    && (0..l.len()).all(|i| {
                        let (a, b) = (l[i], l[(i + 1) % l.len()]);
                        t.vertices.get(fe[i].edge.get()) == Some(&crate::edges::edge_key(a, b))
                            && fe[i].forward == (a < b)
                    });
                if !ok {
                    s.add(
                        ViolationCode::EdgeTable,
                        Some(MeshEntity::Face(FaceIx::new(f))),
                        "face->edge incidence is wrong",
                    );
                    break;
                }
            }
        }
    }
}

/// Tolerances for [`validate_geometry`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeomTol {
    /// Largest allowed distance of a face vertex from the face's best plane, relative to
    /// `sqrt(|area|)`.
    pub planarity: f64,
    /// Cells with volume at most this are reported (default 0: only non-positive volumes).
    pub min_volume: f64,
}

impl Default for GeomTol {
    fn default() -> Self {
        Self {
            planarity: 1e-6,
            min_volume: 0.0,
        }
    }
}

/// Geometric validation: positive cell volumes, positive side tetrahedra, face normals pointing
/// from the left cell to the right cell (outward at one-cell faces), planarity within tolerance.
pub fn validate_geometry(m: &dyn MeshRead, tol: &GeomTol) -> Vec<Violation> {
    let mut s = Sink(Vec::new());
    let (nf, nc) = (m.count(MeshEntityKind::Face), m.count(MeshEntityKind::Cell));
    let cent: Vec<(f64, Point3)> = (0..nc)
        .map(|c| cell_volume_centroid(m, CellIx::new(c)))
        .collect();
    let co = m.coords();
    for (c, (vol, g)) in cent.iter().enumerate() {
        let ci = CellIx::new(c);
        let e = Some(MeshEntity::Cell(ci));
        if *vol <= tol.min_volume {
            s.add(
                ViolationCode::GeomVolume,
                e,
                format!("volume {vol} is not above {}", tol.min_volume),
            );
            continue;
        }
        'cell: for sf in m.cell_faces(ci) {
            let l = m.face_vertices(sf.face());
            let fg = face_geometry(m, sf.face());
            for i in 0..l.len() {
                let (mut a, mut b) = (co.get(l[i].get()), co.get(l[(i + 1) % l.len()].get()));
                if sf.side() == Side::Right {
                    std::mem::swap(&mut a, &mut b);
                }
                if (fg.apex - *g).dot((a - *g).cross(b - *g)) <= 0.0 {
                    s.add(
                        ViolationCode::GeomSideTet,
                        e,
                        format!("side tetrahedron on face {:?} is not positive", sf.face()),
                    );
                    break 'cell;
                }
            }
        }
    }
    for f in 0..nf {
        let fi = FaceIx::new(f);
        let e = Some(MeshEntity::Face(fi));
        let g = face_geometry(m, fi);
        let lc = cent[m.face_left(fi).get()].1;
        let target = match m.face_right(fi) {
            Some(rc) => cent[rc.get()].1 - lc,
            None => g.centroid - lc,
        };
        if g.area_vector.dot(target) <= 0.0 {
            s.add(
                ViolationCode::GeomNormal,
                e,
                "normal does not point left -> right (outward at one-cell faces)",
            );
        }
        let l = m.face_vertices(fi);
        let len = g.area_vector.norm();
        if l.len() > 3 && len > 0.0 {
            let nhat = g.area_vector * (1.0 / len);
            let dev = l
                .iter()
                .map(|v| (co.get(v.get()) - g.apex).dot(nhat).abs())
                .fold(0.0, f64::max);
            if dev / len.sqrt() > tol.planarity {
                s.add(
                    ViolationCode::GeomPlanar,
                    e,
                    format!("deviation {dev:e} from the face plane"),
                );
            }
        }
    }
    s.0
}

/// FNV-1a over little-endian words; bit-exact, platform independent.
struct Hasher(u64);

impl Hasher {
    fn new(domain: &str) -> Self {
        let mut h = Hasher(0xcbf2_9ce4_8422_2325);
        h.str(domain);
        h
    }
    fn u(&mut self, x: u64) {
        for b in x.to_le_bytes() {
            self.0 = (self.0 ^ u64::from(b)).wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    fn str(&mut self, s: &str) {
        self.u(s.len() as u64);
        for b in s.bytes() {
            self.0 = (self.0 ^ u64::from(b)).wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
}

fn counts(m: &dyn MeshRead) -> [usize; 4] {
    [
        m.count(MeshEntityKind::Vertex),
        m.count(MeshEntityKind::Edge),
        m.count(MeshEntityKind::Face),
        m.count(MeshEntityKind::Cell),
    ]
}

fn hash_connectivity(h: &mut Hasher, m: &dyn MeshRead) {
    let [nv, _, nf, nc] = counts(m);
    h.u(m.mesh_id().to_raw());
    for n in [nv, nf, nc, m.coords().len()] {
        h.u(n as u64);
    }
    for i in 0..nv {
        h.u(m.id(MeshEntityKind::Vertex, i as u32).to_raw());
    }
    for f in 0..nf {
        let fi = FaceIx::new(f);
        h.u(m.id(MeshEntityKind::Face, f as u32).to_raw());
        h.u(m.face_left(fi).0.into());
        h.u(m.face_right(fi).map_or(u64::MAX, |c| c.0.into()));
        h.u(m.face_kind(fi) as u64);
        let l = m.face_vertices(fi);
        h.u(l.len() as u64);
        for v in l {
            h.u(v.0.into());
        }
    }
    for c in 0..nc {
        let ci = CellIx::new(c);
        h.u(m.id(MeshEntityKind::Cell, c as u32).to_raw());
        h.u(m.cell_shape(ci) as u64);
        h.u(m.cell_parent(ci).map_or(u64::MAX, |p| p.0.into()));
        h.u(u64::from(m.cell_active(ci)));
        let fs = m.cell_faces(ci);
        h.u(fs.len() as u64);
        for sf in fs {
            h.u(sf.to_raw().into());
        }
        let cs = m.cell_corners(ci).unwrap_or(&[]);
        h.u(cs.len() as u64);
        for v in cs {
            h.u(v.0.into());
        }
    }
}

/// Hash of the connectivity only: ids, counts, face loops, left/right, kinds, signed cell->face
/// lists, shapes, corners and lineage. It does not change when classification, sets, tags,
/// high-order or coordinates change. It keys the geometry cache and hierarchy maps.
pub fn connectivity_hash(m: &dyn MeshRead) -> u64 {
    let mut h = Hasher::new("smartmesh.connectivity.v1");
    hash_connectivity(&mut h, m);
    h.0
}

/// Hash of the point coordinates only (bits of vertices and extra nodes). It keys the geometry
/// cache together with [`connectivity_hash`], so a cache computed before a point moved is
/// rejected by `validate` and `set_geometry_cache`.
pub fn coordinate_hash(m: &dyn MeshRead) -> u64 {
    let mut h = Hasher::new("smartmesh.coordinates.v1");
    let co = m.coords();
    h.u(co.len() as u64);
    for i in 0..co.len() {
        h.u(co.x[i].to_bits());
        h.u(co.y[i].to_bits());
        h.u(co.z[i].to_bits());
    }
    h.0
}

/// Hash of everything stored that is not a real-valued coordinate or parameter: the
/// connectivity, class entities of vertices, edges, faces and cells, the edge table, high-order
/// node indices, face-curve node indices, set members, integer tag values and the names and
/// shapes of real tag columns. Derived caches are excluded.
pub fn topology_hash(m: &dyn MeshRead) -> u64 {
    let mut h = Hasher::new("smartmesh.topology.v1");
    hash_connectivity(&mut h, m);
    let [nv, _, nf, nc] = counts(m);
    for (k, n) in [
        (MeshEntityKind::Vertex, nv),
        (MeshEntityKind::Face, nf),
        (MeshEntityKind::Cell, nc),
    ] {
        for i in 0..n {
            h.u(m.classification(k, i as u32).entity.to_raw());
        }
    }
    match m.edges() {
        None => h.u(0),
        Some(t) => {
            h.u(1 + t.len() as u64);
            for i in 0..t.len() {
                h.u(t.ids[i].to_raw());
                h.u(t.vertices[i][0].0.into());
                h.u(t.vertices[i][1].0.into());
                h.u(t.class[i].entity.to_raw());
            }
            for fe in &t.face_edges {
                h.u(u64::from(fe.edge.0) << 1 | u64::from(fe.forward));
            }
        }
    }
    for c in 0..nc {
        if let Some(ho) = m.high_order_cell(CellIx::new(c)) {
            h.u(c as u64);
            h.u(u64::from(ho.degree));
            h.u(u64::from(ho.basis.code()));
            h.u(ho.nodes.len() as u64);
            for n in &ho.nodes {
                h.u(n.0.into());
            }
        }
    }
    h.u(u64::MAX);
    for f in 0..nf {
        if let Some(fc) = m.face_curve(FaceIx::new(f)) {
            h.u(f as u64);
            h.u(u64::from(fc.degree));
            h.u(u64::from(fc.basis.code()));
            h.u(fc.nodes.len() as u64);
            for n in fc.nodes {
                h.u(n.0.into());
            }
        }
    }
    h.u(u64::MAX);
    h.u(m.sets().len() as u64);
    for st in m.sets() {
        h.u(st.id.to_raw());
        h.str(&st.name);
        h.u(st.members.kind() as u64);
        let raw = st.members.raw();
        h.u(raw.len() as u64);
        for x in raw {
            h.u(x.into());
        }
    }
    h.u(m.tags().len() as u64);
    for t in m.tags() {
        h.str(&t.name);
        h.u(t.target as u64);
        h.u(t.values.len() as u64);
        match &t.values {
            TagValues::I64(v) => {
                h.u(0);
                for x in v {
                    h.u(*x as u64);
                }
            }
            TagValues::F64(_) => h.u(1),
        }
    }
    h.0
}

/// Hash of the real-valued data: coordinate bits (vertices and extra nodes), the parametric
/// coordinates of every classification (vertices, edges, faces, cells), real tag columns and
/// the geometry rule id. Derived caches are excluded.
pub fn geometry_hash(m: &dyn MeshRead) -> u64 {
    let mut h = Hasher::new("smartmesh.geometry.v1");
    h.str(m.geometry_rule());
    let co = m.coords();
    h.u(co.len() as u64);
    for i in 0..co.len() {
        h.u(co.x[i].to_bits());
        h.u(co.y[i].to_bits());
        h.u(co.z[i].to_bits());
    }
    let [nv, _, nf, nc] = counts(m);
    for (k, n) in [
        (MeshEntityKind::Vertex, nv),
        (MeshEntityKind::Face, nf),
        (MeshEntityKind::Cell, nc),
    ] {
        for i in 0..n {
            for w in m.classification(k, i as u32).param_bits() {
                h.u(w);
            }
        }
    }
    if let Some(t) = m.edges() {
        for c in &t.class {
            for w in c.param_bits() {
                h.u(w);
            }
        }
    }
    for t in m.tags() {
        if let TagValues::F64(v) = &t.values {
            h.str(&t.name);
            h.u(t.target as u64);
            for x in v {
                h.u(x.to_bits());
            }
        }
    }
    h.0
}
