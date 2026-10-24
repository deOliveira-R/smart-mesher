//! Validators and content hashes. All of them work on any [`MeshRead`] implementor.
//!
//! [`validate`] checks everything that does not need coordinates; [`validate_geometry`] checks
//! that the stored orientation agrees with the coordinates. Findings carry a stable
//! [`ViolationCode`].

#![allow(clippy::needless_range_loop)] // dense-index loops over parallel columns are the clearest form here

use crate::classify::{ClassSet, Classification};
use crate::geometry::{cell_volume_centroid, face_geometry};
use crate::highorder::{face_node_count, node_count};
use crate::index::{
    CellIx, EdgeIx, FaceIx, FaceKind, MeshDim, MeshEntity, MeshEntityKind, Side, VertexIx,
};
use crate::read::MeshRead;
use crate::sets::{SetMembers, TagValues};
use crate::shape::{CellShape, LoopRelation, derive_corners, loop_relation, polygon_cycle};
use sm_core::{EntityId, EntityKind, Point3, Vector3};
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
    /// A dimension-2 cell's corner cycle is not right-handed about the normal of the face it is
    /// classified on (carrier sense applied), see [`validate_geometry_with`].
    GeomShellOrientation,
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
            GeomShellOrientation => "geom.shell_orientation",
        }
    }
}

/// One validation finding. `#[non_exhaustive]` (W0g r4): build it with [`Violation::new`];
/// fields added later are `#[serde(default)]`.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct Violation {
    /// Stable code.
    pub code: ViolationCode,
    /// The offending entity, if one can be named.
    pub entity: Option<MeshEntity>,
    /// Human-readable detail.
    pub detail: String,
}

impl Violation {
    /// A finding with `code`, the offending `entity` (if one can be named) and a `detail` text.
    pub fn new(code: ViolationCode, entity: Option<MeshEntity>, detail: impl Into<String>) -> Self {
        Self {
            code,
            entity,
            detail: detail.into(),
        }
    }
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
        self.0.push(Violation::new(code, entity, detail));
    }
}

/// Topological validation. Empty result means valid. Checks ids and column sizes, loops,
/// canonical order, the orientation rule in both directions, closedness (directed loop edges
/// cancel per cell), shape tags and template corners, high-order and face-curve records,
/// classification (kinds, parameters, dimension order including edges, reverse index), lineage,
/// sets, tags, the edge table and the geometry cache.
///
/// For a dimension-2 mesh ([`MeshRead::dim`]) facets must have exactly two vertices and are not
/// rotated, closedness means the directed facets of a cell chain into one cycle, every cell
/// carries its vertex cycle as corners, cells lie on face entities and boundary facets on edge
/// entities, and there is no edge table.
pub fn validate(m: &dyn MeshRead) -> Vec<Violation> {
    let mut s = Sink(Vec::new());
    let dim = m.dim();
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
        if !dim.facet_len_ok(l.len()) || sorted.len() != l.len() || l.iter().any(|v| v.get() >= nv)
        {
            s.add(
                ViolationCode::LoopInvalid,
                e,
                "loop has the wrong number of vertices for the mesh dimension (< 3, or not 2 in dimension 2), repeats a vertex, or names a missing vertex",
            );
            loops_ok[f] = false;
            continue;
        }
        if dim == MeshDim::Three && l[0] != sorted[0] {
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

    // A dimension-3 mesh with edge-slot nodes needs a (fully classified) edge table, so every
    // extra node's class follows from its slot owner (`MeshRead::point_class`).
    if dim == MeshDim::Three && m.edges().is_none() {
        let cell = (0..nc).find(|c| m.high_order_cell(CellIx::new(*c)).is_some()).map(|c| MeshEntity::Cell(CellIx::new(c)));
        let face = (0..nf).find(|f| m.face_curve(FaceIx::new(*f)).is_some()).map(|f| MeshEntity::Face(FaceIx::new(f)));
        if let Some(e) = cell.or(face) {
            s.add(ViolationCode::HighOrder, Some(e), "high-order cells and face curves need an edge table (their edge nodes take the class of their edge)");
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

/// Dimension-2 cell: the directed facets chain into one cycle, the tag matches, and the corners
/// are that cycle in boundary order.
fn validate_cell_2d(m: &dyn MeshRead, ci: CellIx, s: &mut Sink) {
    let e = Some(MeshEntity::Cell(ci));
    let faces = m.cell_faces(ci);
    let directed: Vec<[VertexIx; 2]> = faces
        .iter()
        .filter_map(|sf| {
            let l = m.face_vertices(sf.face());
            (l.len() == 2).then(|| {
                if sf.side() == Side::Left {
                    [l[0], l[1]]
                } else {
                    [l[1], l[0]]
                }
            })
        })
        .collect();
    let refs: Vec<&[VertexIx]> = directed.iter().map(|d| d.as_slice()).collect();
    let Some(cyc) = polygon_cycle(&refs) else {
        s.add(
            ViolationCode::Closedness,
            e,
            "directed facets do not chain into one closed cycle",
        );
        return;
    };
    let detected = CellShape::detect(cyc.len(), &vec![2; faces.len()]);
    let shape = m.cell_shape(ci);
    if shape != detected {
        s.add(
            ViolationCode::ShapeTag,
            e,
            format!("tag {shape:?} but topology is {detected:?}"),
        );
    }
    match m.cell_corners(ci) {
        None => s.add(
            ViolationCode::Corners,
            e,
            "a dimension-2 cell must carry its vertex cycle as corners",
        ),
        Some(ov) if loop_relation(ov, &cyc) != Some(LoopRelation::Same) => s.add(
            ViolationCode::Corners,
            e,
            "corners are not the cell's vertex cycle in boundary order",
        ),
        Some(_) => {}
    }
}

fn validate_cell(m: &dyn MeshRead, ci: CellIx, np: usize, s: &mut Sink) {
    if m.dim() == MeshDim::Two {
        validate_cell_2d(m, ci, s);
        validate_high_order(m, ci, np, s);
        return;
    }
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
    validate_high_order(m, ci, np, s);
}

fn validate_high_order(m: &dyn MeshRead, ci: CellIx, np: usize, s: &mut Sink) {
    let e = Some(MeshEntity::Cell(ci));
    let shape = m.cell_shape(ci);
    let corners = m.cell_corners(ci);
    if let Some(h) = m.high_order_cell(ci) {
        match (corners, node_count(shape, h.degree, h.basis)) {
            _ if shape.is_general() => s.add(
                ViolationCode::HighOrder,
                e,
                "polyhedra and polygons cannot carry a high-order record (use face curves)",
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
    let dim = m.dim();
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
        if cl.dim().is_some() && !dim.accepts_cell(cl.entity.kind()) {
            s.add(
                ViolationCode::ClassKind,
                Some(MeshEntity::Cell(CellIx::new(c))),
                format!(
                    "cells of a {dim:?} mesh must be classified on an entity of that dimension, not {:?}",
                    cl.entity.kind()
                ),
            );
        }
    }
    for f in 0..nf {
        let fi = FaceIx::new(f);
        let k = m.face_class(fi).entity.kind();
        if m.face_class(fi).dim().is_none() {
            continue;
        }
        if !dim.accepts_facet(m.face_kind(fi), k) {
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
    let np = m.coords().len();
    for (i, st) in sets.iter().enumerate() {
        let n = match (&st.members, st.members.kind()) {
            (SetMembers::Points(_), _) => np,
            (_, MeshEntityKind::Vertex) => nv,
            (_, MeshEntityKind::Face) => nf,
            (_, MeshEntityKind::Cell) => nc,
            (_, MeshEntityKind::Edge) => 0,
        };
        if matches!(st.members, SetMembers::ShellSides(_)) && m.dim() != MeshDim::Two {
            s.add(ViolationCode::Set, None, format!("shell side set '{}' in a dimension-3 mesh", st.name));
        }
        let raw = st.members.raw();
        if st.id.kind() != EntityKind::MeshSet
            || (i > 0 && sets[i - 1].id >= st.id)
            || sets[..i].iter().any(|o| o.name == st.name)
            || raw.windows(2).any(|w| w[0] >= w[1])
            || st.members.max_index().is_some_and(|x| x as usize >= n)
        {
            s.add(ViolationCode::Set, None, format!("set '{}' is invalid (id kind/order, duplicate name, or members not ascending/in range)", st.name));
        }
        if let SetMembers::Sides(v) = &st.members
            && v.iter().any(|sf| (sf.face().0 as usize) < nf && sf.side() == Side::Right && m.face_right(sf.face()).is_none())
        {
            s.add(ViolationCode::Set, None, format!("side set '{}' names the right side of a one-cell face", st.name));
        }
        if st.label.is_some() && sets[..i].iter().any(|o| o.label == st.label && o.members.kind() == st.members.kind()) {
            s.add(ViolationCode::Set, None, format!("set '{}' repeats label {:?} for its members kind", st.name, st.label));
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
    if m.edges().is_some() && m.dim() == MeshDim::Two {
        s.add(
            ViolationCode::EdgeTable,
            None,
            "a dimension-2 mesh has no edge table (its facets are the edges)",
        );
    } else if let Some(t) = m.edges() {
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

/// Tolerances for [`validate_geometry`]. `#[non_exhaustive]` (W0g r4): build it with
/// [`GeomTol::new`] / `Default` and the `with_*` setters.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
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

impl GeomTol {
    /// The default tolerances (planarity `1e-6`, min volume `0`).
    pub fn new() -> Self {
        Self::default()
    }

    /// Replace the planarity tolerance.
    pub fn with_planarity(mut self, planarity: f64) -> Self {
        self.planarity = planarity;
        self
    }

    /// Replace the minimum cell volume.
    pub fn with_min_volume(mut self, min_volume: f64) -> Self {
        self.min_volume = min_volume;
        self
    }
}

/// Oriented normals of the model faces a dimension-2 mesh is classified on, for the shell
/// orientation rule (W0f gaps 2). Implemented over the topology by the caller (this crate does
/// not depend on it): for a `TopoFace` or `PartitionFace` with a surface carrier, the unit
/// surface normal at the point nearest `near`, times the face's carrier sense (the face normal
/// of `sm-topo-api`); `None` when the entity has no surface carrier (the rule is then not
/// checked for its cells).
pub trait FaceNormals {
    /// The oriented face normal of `entity` near `near`, or `None`.
    fn normal(&self, entity: EntityId, near: Point3) -> Option<Vector3>;
}

/// [`validate_geometry`] plus, for a dimension-2 mesh, the **shell orientation rule** (W0f gaps
/// 2, contract for every mesher): a cell's corner cycle follows the right-hand rule about the
/// normal of the `TopoFace` / `PartitionFace` it is classified on, with the carrier sense
/// applied. The cell's area vector (Newell, over its corner cycle) must have a positive dot
/// product with `normals.normal(entity, centroid)`; cells whose entity has no normal are
/// skipped. This defines the shell normal and its top (side 0 of
/// [`crate::SetMembers::ShellSides`], along the normal) and bottom (side 1).
pub fn validate_geometry_with(m: &dyn MeshRead, tol: &GeomTol, normals: &dyn FaceNormals) -> Vec<Violation> {
    let mut out = validate_geometry(m, tol);
    if m.dim() != MeshDim::Two {
        return out;
    }
    let co = m.coords();
    for c in 0..m.count(MeshEntityKind::Cell) {
        let ci = CellIx::new(c);
        let Some(cyc) = m.cell_corners(ci) else { continue };
        let n = cyc.len();
        if n < 3 {
            continue;
        }
        let entity = m.cell_class(ci).entity;
        let (mut area, mut cen) = (Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0));
        for i in 0..n {
            let (a, b) = (co.get(cyc[i].get()), co.get(cyc[(i + 1) % n].get()));
            area = area + Vector3::new(a.x, a.y, a.z).cross(Vector3::new(b.x, b.y, b.z));
            cen = cen + Vector3::new(a.x, a.y, a.z);
        }
        let cen = cen * (1.0 / n as f64);
        let Some(nrm) = normals.normal(entity, Point3::new(cen.x, cen.y, cen.z)) else { continue };
        if area.dot(nrm) <= 0.0 {
            out.push(Violation::new(
                ViolationCode::GeomShellOrientation,
                Some(MeshEntity::Cell(ci)),
                format!("corner cycle is not right-handed about the normal of {entity}"),
            ));
        }
    }
    out
}

/// Geometric validation: positive cell volumes, positive side tetrahedra, face normals pointing
/// from the left cell to the right cell (outward at one-cell faces), planarity within tolerance.
///
/// For a dimension-2 mesh only the cell areas are checked (`min_volume` applies to the area):
/// an embedded surface has no inside, so facet orientation is purely topological (checked by
/// [`validate`]); the shell orientation rule against the classified faces' normals needs a
/// normal source, see [`validate_geometry_with`].
pub fn validate_geometry(m: &dyn MeshRead, tol: &GeomTol) -> Vec<Violation> {
    let mut s = Sink(Vec::new());
    let (nf, nc) = (m.count(MeshEntityKind::Face), m.count(MeshEntityKind::Cell));
    let cent: Vec<(f64, Point3)> = (0..nc)
        .map(|c| cell_volume_centroid(m, CellIx::new(c)))
        .collect();
    if m.dim() == MeshDim::Two {
        for (c, (area, _)) in cent.iter().enumerate() {
            if *area <= tol.min_volume {
                s.add(
                    ViolationCode::GeomVolume,
                    Some(MeshEntity::Cell(CellIx::new(c))),
                    format!("area {area} is not above {}", tol.min_volume),
                );
            }
        }
        return s.0;
    }
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
    h.u(u64::from(m.dim().get()));
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

/// Hash of the connectivity only: dimension, ids, counts, face loops, left/right, kinds, signed
/// cell->face lists, shapes, corners and lineage. It does not change when classification, sets, tags,
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
        match st.label {
            Some(l) => {
                h.u(1);
                h.u(l as u64);
            }
            None => h.u(0),
        }
        h.u(st.members.kind() as u64);
        h.u(u64::from(st.members.code()));
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

/// Quantize one float with an absolute step: `round(x / quantum)` (half away from zero, the
/// W0c Q2 rule of `sm_geom_api::ModelTolerance::quantize_with`), `-0` folded to `0`,
/// out-of-range and infinite values saturate, NaN (or a non-positive quantum) maps to
/// `i64::MIN`. [`geometry_hash_quantized`] does not hash saturated codes: a value whose code
/// is out of `i64` range is hashed by its raw bits behind a marker instead.
pub fn quantize_word(x: f64, quantum: f64) -> i64 {
    if x.is_nan() || quantum.is_nan() || quantum <= 0.0 {
        return i64::MIN;
    }
    let r = (x / quantum).round();
    if r == 0.0 { 0 } else { r as i64 }
}

/// The quantization steps of [`geometry_hash_quantized`] (W0g r2). Both enter the hash.
///
/// For the state identity use `length = ModelTolerance::linear * 2^-10` and
/// `param = ModelTolerance::parametric * 2^-10` (`quantum()` / `param_quantum()` in
/// `sm-geom-api`, matching topology's length and parameter codes).
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct HashQuanta {
    /// Absolute step for coordinates (vertices and extra nodes).
    pub length: f64,
    /// Absolute step for classification parameters (curve `t`, surface `(u, v)`). Tag values
    /// are scale-free and do not use it.
    pub param: f64,
}

impl HashQuanta {
    /// Construct (the type is `#[non_exhaustive]`).
    pub const fn new(length: f64, param: f64) -> Self {
        Self { length, param }
    }
}

/// Word marking that a quantized code follows.
const QW_CODE: u64 = 0;
/// Word marking that the raw bits follow (non-finite value, code out of `i64` range, or an
/// invalid quantum); NaN is canonicalized first.
const QW_RAW: u64 = 1;

fn raw_words(h: &mut Hasher, x: f64) {
    h.u(QW_RAW);
    h.u(if x.is_nan() { f64::NAN.to_bits() } else { x.to_bits() });
}

/// Absolute-step float: marker plus `round(x / q)` (`-0` folded), or marker plus raw bits.
fn abs_words(h: &mut Hasher, x: f64, q: f64) {
    if x.is_finite() && q.is_finite() && q > 0.0 {
        let r = (x / q).round();
        if r.abs() < 9.2e18 {
            h.u(QW_CODE);
            h.u(if r == 0.0 { 0 } else { r as i64 as u64 });
            return;
        }
    }
    raw_words(h, x);
}

/// Scale-free float (real tag values): subnormals and -0 hash as 0 (no absolute cutoff: user
/// fields have arbitrary units); otherwise the
/// mantissa is rounded to 40 bits (relative step about 1e-12) and packed with exponent and sign,
/// the rule of `sm_geom_api::ModelTolerance::quantize_scale_free` (never out of range for finite
/// input). Non-finite values hash by raw bits.
fn scale_free_words(h: &mut Hasher, x: f64) {
    if !x.is_finite() {
        return raw_words(h, x);
    }
    let a = x.abs();
    let code = if (a.to_bits() >> 52) == 0 {
        0
    } else {
        // Round half up on the low 13 bits; a mantissa carry moves into the exponent.
        let c = ((a.to_bits() + (1u64 << 12)) >> 13) as i64;
        if x < 0.0 { -c } else { c }
    };
    h.u(QW_CODE);
    h.u(code as u64);
}

/// Cross-platform geometry hash (the W0c Q2 implementation for meshes, `v2` since W0g r2): the
/// content of [`geometry_hash`] (rule id, vertex and extra-node coordinates, every
/// classification `Param`, real tag values) with every float quantized instead of hashed by its
/// bits, so it is platform independent:
///
/// - coordinates: absolute step `q.length`;
/// - classification parameters: absolute step `q.param`;
/// - real tag values: scale-free (relative 40-bit mantissa, no absolute cutoff), so a large
///   value never saturates.
///
/// Every float is a marker word then either its code or, when the code would be out of `i64`
/// range (or the value is not finite), its raw bits, so the stream stays prefix-free and no two
/// distinct large values share a saturated code. Both quanta enter the hash. Keep
/// [`geometry_hash`] (exact bits) for cache keys.
pub fn geometry_hash_quantized(m: &dyn MeshRead, q: &HashQuanta) -> u64 {
    let mut h = Hasher::new("smartmesh.geometry-quantized.v2");
    let (ql, qp) = (q.length, q.param);
    h.str(m.geometry_rule());
    h.u(ql.to_bits());
    h.u(qp.to_bits());
    let co = m.coords();
    h.u(co.len() as u64);
    for i in 0..co.len() {
        abs_words(&mut h, co.x[i], ql);
        abs_words(&mut h, co.y[i], ql);
        abs_words(&mut h, co.z[i], ql);
    }
    let param = |h: &mut Hasher, c: Classification| {
        let [tag, a, b, cc] = c.param_bits();
        h.u(tag);
        for (k, bits) in [a, b, cc].into_iter().enumerate() {
            if (k as u64) < tag {
                abs_words(h, f64::from_bits(bits), qp);
            }
        }
    };
    let [nv, _, nf, nc] = counts(m);
    for (k, n) in [
        (MeshEntityKind::Vertex, nv),
        (MeshEntityKind::Face, nf),
        (MeshEntityKind::Cell, nc),
    ] {
        for i in 0..n {
            param(&mut h, m.classification(k, i as u32));
        }
    }
    if let Some(t) = m.edges() {
        for c in &t.class {
            param(&mut h, *c);
        }
    }
    for t in m.tags() {
        if let TagValues::F64(v) = &t.values {
            h.str(&t.name);
            h.u(t.target as u64);
            h.u(v.len() as u64);
            for x in v {
                scale_free_words(&mut h, *x);
            }
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
