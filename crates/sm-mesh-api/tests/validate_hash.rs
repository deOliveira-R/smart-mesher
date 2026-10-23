//! Violation codes (against deliberately corrupted views) and content hashes.

mod common;
use common::*;
use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh, hex_block, hex_block_with_edges};
use sm_mesh_api::*;

/// A `MeshRead` that delegates to a valid mesh except where a field is overridden. It also shows
/// that the trait is object safe and implementable outside the crate.
#[derive(Default)]
struct Broken {
    m: Option<ReferenceMesh>,
    face_kind: Option<(u32, FaceKind)>,
    face_left: Option<(u32, u32)>,
    loop_of: Option<(u32, Vec<VertexIx>)>,
    cfaces: Option<(u32, Vec<SignedFace>)>,
    class: Option<(MeshEntityKind, u32, Classification)>,
    parent: Option<(u32, u32)>,
    shape: Option<(u32, CellShape)>,
    no_corners: Option<u32>,
    edges: Option<EdgeTable>,
    tags: Option<Vec<TagColumn>>,
    sets: Option<Vec<MeshSet>>,
    cache: Option<GeometryCache>,
    ho: Option<(u32, HighOrderCell)>,
    curve: Option<(u32, Vec<PointIx>)>,
    id: Option<(MeshEntityKind, u32, EntityId)>,
}

impl Broken {
    fn of(m: ReferenceMesh) -> Self {
        Broken {
            m: Some(m),
            ..Default::default()
        }
    }
    fn inner(&self) -> &ReferenceMesh {
        self.m.as_ref().unwrap()
    }
}

impl MeshRead for Broken {
    fn mesh_id(&self) -> EntityId {
        self.inner().mesh_id()
    }
    fn count(&self, k: MeshEntityKind) -> usize {
        match (&self.edges, k) {
            (Some(t), MeshEntityKind::Edge) => t.len(),
            _ => self.inner().count(k),
        }
    }
    fn id(&self, k: MeshEntityKind, ix: u32) -> EntityId {
        match self.id {
            Some((kk, i, id)) if kk == k && i == ix => id,
            _ => self.inner().id(k, ix),
        }
    }
    fn index_of(&self, id: EntityId) -> Option<u32> {
        self.inner().index_of(id)
    }
    fn coords(&self) -> Coords<'_> {
        self.inner().coords()
    }
    fn columns(&self) -> TopologyColumns<'_> {
        self.inner().columns()
    }
    fn face_vertices(&self, f: FaceIx) -> &[VertexIx] {
        match &self.loop_of {
            Some((i, l)) if *i == f.0 => l,
            _ => self.inner().face_vertices(f),
        }
    }
    fn face_left(&self, f: FaceIx) -> CellIx {
        match self.face_left {
            Some((i, c)) if i == f.0 => CellIx(c),
            _ => self.inner().face_left(f),
        }
    }
    fn face_right(&self, f: FaceIx) -> Option<CellIx> {
        self.inner().face_right(f)
    }
    fn face_kind(&self, f: FaceIx) -> FaceKind {
        match self.face_kind {
            Some((i, k)) if i == f.0 => k,
            _ => self.inner().face_kind(f),
        }
    }
    fn cell_faces(&self, c: CellIx) -> &[SignedFace] {
        match &self.cfaces {
            Some((i, v)) if *i == c.0 => v,
            _ => self.inner().cell_faces(c),
        }
    }
    fn cell_shape(&self, c: CellIx) -> CellShape {
        match self.shape {
            Some((i, s)) if i == c.0 => s,
            _ => self.inner().cell_shape(c),
        }
    }
    fn cell_corners(&self, c: CellIx) -> Option<&[VertexIx]> {
        if self.no_corners == Some(c.0) {
            None
        } else {
            self.inner().cell_corners(c)
        }
    }
    fn high_order_cell(&self, c: CellIx) -> Option<&HighOrderCell> {
        match &self.ho {
            Some((i, h)) if *i == c.0 => Some(h),
            _ => self.inner().high_order_cell(c),
        }
    }
    fn face_curve(&self, f: FaceIx) -> Option<FaceCurve<'_>> {
        match &self.curve {
            Some((i, n)) if *i == f.0 => Some(FaceCurve {
                degree: 2,
                basis: Basis::LagrangeEquispaced,
                nodes: n,
            }),
            _ => self.inner().face_curve(f),
        }
    }
    fn cell_parent(&self, c: CellIx) -> Option<CellIx> {
        match self.parent {
            Some((i, p)) if i == c.0 => Some(CellIx(p)),
            _ => self.inner().cell_parent(c),
        }
    }
    fn cell_active(&self, c: CellIx) -> bool {
        self.inner().cell_active(c)
    }
    fn classification(&self, k: MeshEntityKind, ix: u32) -> Classification {
        match self.class {
            Some((kk, i, c)) if kk == k && i == ix => c,
            _ => match (&self.edges, k) {
                (Some(t), MeshEntityKind::Edge) => t.class[ix as usize],
                _ => self.inner().classification(k, ix),
            },
        }
    }
    fn classified_on(&self, model: EntityId) -> Option<&ClassSet> {
        self.inner().classified_on(model)
    }
    fn classified_entities(&self) -> &[EntityId] {
        self.inner().classified_entities()
    }
    fn edges(&self) -> Option<&EdgeTable> {
        self.edges.as_ref().or_else(|| self.inner().edges())
    }
    fn sets(&self) -> &[MeshSet] {
        self.sets.as_deref().unwrap_or_else(|| self.inner().sets())
    }
    fn tags(&self) -> &[TagColumn] {
        self.tags.as_deref().unwrap_or_else(|| self.inner().tags())
    }
    fn geometry_cache(&self) -> Option<&GeometryCache> {
        self.cache
            .as_ref()
            .or_else(|| self.inner().geometry_cache())
    }
}

fn has(b: &Broken, code: ViolationCode) -> bool {
    let v = validate(b);
    v.iter().any(|x| x.code == code)
}

fn expect(b: &Broken, code: ViolationCode) {
    let v = validate(b);
    assert!(
        v.iter().any(|x| x.code == code),
        "expected {} in {v:?}",
        code.as_str()
    );
}

fn base() -> ReferenceMesh {
    hex_block([2, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap()
}

#[test]
fn a_valid_mesh_has_no_violations_through_a_foreign_implementation() {
    let b = Broken::of(base());
    assert!(validate(&b).is_empty());
    let dynb: &dyn MeshRead = &b;
    assert!(validate_geometry(dynb, &GeomTol::default()).is_empty());
    let e = Broken::of(hex_block_with_edges([2, 2, 2], [1.0; 3], &mut SerialAlloc::new()).unwrap());
    assert!(validate(&e).is_empty());
}

#[test]
fn violation_codes_are_stable_and_specific() {
    let m = base();
    let bf = (0..m.face_count())
        .map(FaceIx::new)
        .find(|&f| m.face_kind(f) == FaceKind::Boundary)
        .unwrap();
    let inner = interior_faces(&m)[0];

    let mut b = Broken::of(m.clone());
    b.face_kind = Some((bf.0, FaceKind::Interior));
    expect(&b, ViolationCode::FaceLeftRight);

    let mut b = Broken::of(m.clone());
    let mut l = m.face_vertices(bf).to_vec();
    l.rotate_left(1);
    b.loop_of = Some((bf.0, l));
    expect(&b, ViolationCode::LoopNotCanonical);

    let mut b = Broken::of(m.clone());
    let mut l = m.face_vertices(bf).to_vec();
    l[1] = l[0];
    b.loop_of = Some((bf.0, l));
    expect(&b, ViolationCode::LoopInvalid);

    let mut b = Broken::of(m.clone());
    b.face_left = Some((inner.0, 1));
    expect(&b, ViolationCode::FaceLeftRight);

    let mut b = Broken::of(m.clone());
    let mut cf = m.cell_faces(CellIx(0)).to_vec();
    cf[0] = SignedFace::new(cf[0].face(), cf[0].side().flipped());
    b.cfaces = Some((0, cf));
    expect(&b, ViolationCode::CellSide);

    let mut b = Broken::of(m.clone());
    let mut cf = m.cell_faces(CellIx(0)).to_vec();
    cf.pop();
    b.cfaces = Some((0, cf));
    expect(&b, ViolationCode::Closedness);
    expect(&b, ViolationCode::Incidence);

    let mut b = Broken::of(m.clone());
    let mut cf = m.cell_faces(CellIx(0)).to_vec();
    cf.reverse();
    b.cfaces = Some((0, cf));
    expect(&b, ViolationCode::CellFacesOrder);

    let mut b = Broken::of(m.clone());
    b.shape = Some((0, CellShape::Tet));
    expect(&b, ViolationCode::ShapeTag);

    let mut b = Broken::of(m.clone());
    b.no_corners = Some(0);
    expect(&b, ViolationCode::Corners);

    let mut b = Broken::of(m.clone());
    b.ho = Some((
        0,
        HighOrderCell {
            degree: 2,
            basis: Basis::LagrangeEquispaced,
            nodes: vec![PointIx(0); 8],
        },
    ));
    expect(&b, ViolationCode::HighOrder);

    let mut b = Broken::of(m.clone());
    b.curve = Some((0, vec![PointIx(0); 9]));
    expect(&b, ViolationCode::FaceCurve);

    let mut b = Broken::of(m.clone());
    b.class = Some((
        MeshEntityKind::Vertex,
        0,
        Classification::on(m.cell_id(CellIx(0))),
    ));
    expect(&b, ViolationCode::ClassEntity);

    let mut b = Broken::of(m.clone());
    b.class = Some((
        MeshEntityKind::Vertex,
        0,
        Classification {
            entity: id(EntityKind::TopoVertex, 1),
            param: Param::Surface([0.0, 1.0]),
        },
    ));
    expect(&b, ViolationCode::ClassParam);
    assert!(!has(&b, ViolationCode::ClassEntity));

    let mut b = Broken::of(m.clone());
    b.class = Some((MeshEntityKind::Face, bf.0, vol()));
    expect(&b, ViolationCode::ClassKind);

    let mut b = Broken::of(m.clone());
    b.class = Some((MeshEntityKind::Cell, 0, on(EntityKind::TopoFace, 1)));
    expect(&b, ViolationCode::ClassKind);

    let mut b = Broken::of(m.clone());
    b.class = Some((MeshEntityKind::Cell, 0, on(EntityKind::TopoVolume, 99)));
    expect(&b, ViolationCode::ClassIndex);

    let mut b = Broken::of(m.clone());
    b.parent = Some((1, 1));
    expect(&b, ViolationCode::Lineage);

    let mut b = Broken::of(m.clone());
    b.parent = Some((0, 1));
    expect(&b, ViolationCode::Lineage);

    let mut b = Broken::of(m.clone());
    b.sets = Some(vec![MeshSet {
        id: id(EntityKind::Group, 0),
        name: "s".into(),
        members: SetMembers::Cells(vec![CellIx(1), CellIx(0)]),
    }]);
    expect(&b, ViolationCode::Set);

    let mut b = Broken::of(m.clone());
    b.tags = Some(vec![TagColumn {
        name: "t".into(),
        target: MeshEntityKind::Cell,
        values: TagValues::I64(vec![1]),
    }]);
    expect(&b, ViolationCode::Tag);

    let mut b = Broken::of(m.clone());
    b.id = Some((MeshEntityKind::Cell, 1, m.cell_id(CellIx(0))));
    expect(&b, ViolationCode::IdsOrder);

    let mut b = Broken::of(m.clone());
    b.cache = Some(compute_geometry_cache(
        &hex_block([1, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap(),
    ));
    expect(&b, ViolationCode::Cache);
}

#[test]
fn edge_classification_obeys_the_dimension_order() {
    let m = hex_block_with_edges([2, 2, 2], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    assert!(validate(&m).is_empty());
    let t = m.edges().unwrap();
    // An edge inside the x = 0 model face, with a vertex we then (wrongly) put on the volume.
    let e = (0..t.len())
        .find(|&i| t.class[i].entity == id(EntityKind::TopoFace, 1))
        .map(EdgeIx::new)
        .expect("an edge on TopoFace#1");
    let [a, _] = t.vertices[e.get()];
    let mut b = Broken::of(m.clone());
    b.class = Some((MeshEntityKind::Vertex, a.0, vol()));
    let v = validate(&b);
    assert!(
        v.iter().any(
            |x| x.code == ViolationCode::ClassDimOrder && x.entity == Some(MeshEntity::Edge(e))
        ),
        "an edge on a TopoFace with a vertex on a TopoVolume is rejected: {v:?}"
    );
    // And an edge classified above its face is rejected too.
    let mut tt = t.clone();
    tt.class[e.get()] = vol();
    let mut b = Broken::of(m.clone());
    b.edges = Some(tt);
    assert!(has(&b, ViolationCode::ClassDimOrder));
    // Edge classification Params must fit as well.
    let mut tt = t.clone();
    tt.class[e.get()] = Classification {
        entity: id(EntityKind::TopoFace, 1),
        param: Param::Curve(0.1),
    };
    let mut b = Broken::of(m.clone());
    b.edges = Some(tt);
    assert!(has(&b, ViolationCode::ClassParam));
    // A damaged edge table is caught.
    let mut tt = t.clone();
    tt.vertices.swap(0, 1);
    let mut b = Broken::of(m.clone());
    b.edges = Some(tt);
    assert!(has(&b, ViolationCode::EdgeTable));
    // drop_edges removes edge tags; edge classification is part of the reverse index.
    let mut mm = m.clone();
    MeshEdit::set_tag(
        &mut mm,
        "w",
        MeshEntityKind::Edge,
        TagValues::I64(vec![0; t.len()]),
    )
    .unwrap();
    assert!(
        mm.classified_on(id(EntityKind::TopoFace, 1))
            .unwrap()
            .edges
            .contains(&e)
    );
    MeshEdit::drop_edges(&mut mm);
    assert!(mm.tag(MeshEntityKind::Edge, "w").is_none() && mm.edges().is_none());
    assert!(
        mm.classified_on(id(EntityKind::TopoFace, 1))
            .unwrap()
            .edges
            .is_empty()
    );
    assert!(validate(&mm).is_empty());
    // Builder: an unset edge on a model edge cannot be derived and is an error; interior ones are automatic.
    let mut mm = hex_block([1, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    assert!(matches!(
        MeshEdit::ensure_edges(&mut mm, &mut SerialAlloc::new(), &[]),
        Err(MeshError::UnclassifiedEdge(..))
    ));
}

#[test]
fn geometry_checks_report_volume_and_planarity() {
    let mut m = hex_block([1, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    // Lift one top vertex out of its face plane.
    MeshEdit::set_point(&mut m, PointIx(7), Point3::new(0.0, 1.0, 1.3)).unwrap();
    let g = validate_geometry(&m, &GeomTol::default());
    assert!(
        g.iter().any(|v| v.code == ViolationCode::GeomPlanar),
        "{g:?}"
    );
    let loose = validate_geometry(
        &m,
        &GeomTol {
            planarity: 10.0,
            min_volume: 0.0,
        },
    );
    assert!(loose.is_empty(), "{loose:?}");
    // A vertex pulled through the opposite face inverts the cell.
    MeshEdit::set_point(&mut m, PointIx(7), Point3::new(0.0, 1.0, -3.0)).unwrap();
    let g = validate_geometry(&m, &GeomTol::default());
    assert!(
        g.iter()
            .any(|v| v.code == ViolationCode::GeomVolume || v.code == ViolationCode::GeomSideTet),
        "{g:?}"
    );
    assert_eq!(ViolationCode::GeomVolume.as_str(), "geom.volume");
}

#[derive(Clone)]
struct Opts {
    vparam: f64,
    tag_i: i64,
    tag_f: f64,
    set: Vec<u32>,
    vclass: u64,
    x_shift: f64,
    lineage: bool,
    cell_class: u64,
}

impl Default for Opts {
    fn default() -> Self {
        Opts {
            vparam: 0.25,
            tag_i: 7,
            tag_f: 1.5,
            set: vec![0],
            vclass: 2,
            x_shift: 0.0,
            lineage: false,
            cell_class: 1,
        }
    }
}

fn make(o: &Opts) -> ReferenceMesh {
    let mut b = ReferenceBuilder::new();
    let mut vs = Vec::new();
    for k in 0..2 {
        for j in 0..2 {
            for i in 0..3 {
                let v = b
                    .add_vertex(Point3::new(
                        i as f64 + if i == 2 { o.x_shift } else { 0.0 },
                        j as f64,
                        k as f64,
                    ))
                    .unwrap();
                vs.push(v);
            }
        }
    }
    for (n, &v) in vs.iter().enumerate() {
        let c = match n {
            0 => Classification {
                entity: id(EntityKind::TopoEdge, 1),
                param: Param::Curve(o.vparam),
            },
            1 => on(EntityKind::TopoVertex, o.vclass),
            _ => on(EntityKind::TopoVertex, 10 + n as u64),
        };
        b.set_class(MeshEntityRef::Vertex(v), c).unwrap();
    }
    let ix = |i: usize, j: usize, k: usize| vs[(k * 2 + j) * 3 + i];
    let mut cells = Vec::new();
    for i in 0..2 {
        let c = [
            ix(i, 0, 0),
            ix(i + 1, 0, 0),
            ix(i + 1, 1, 0),
            ix(i, 1, 0),
            ix(i, 0, 1),
            ix(i + 1, 0, 1),
            ix(i + 1, 1, 1),
            ix(i, 1, 1),
        ];
        let added = b.add_cell_from_vertices(CellShape::Hex, &c).unwrap();
        b.set_class(
            MeshEntityRef::Cell(added.cell),
            on(
                EntityKind::TopoVolume,
                if i == 0 { o.cell_class } else { 1 },
            ),
        )
        .unwrap();
        for (n, sf) in added.faces.iter().enumerate() {
            if !added.reused[n] {
                b.set_class(
                    MeshEntityRef::Face(sf.face()),
                    on(EntityKind::TopoFace, 1 + n as u64),
                )
                .unwrap();
            }
        }
        cells.push(added.cell);
    }
    // The shared face was classified on the first cell's creation; reclassify it on the volume.
    b.add_tag(
        "mat",
        MeshEntityKind::Cell,
        TagValues::I64(vec![o.tag_i, 0]),
    )
    .unwrap();
    b.add_tag(
        "rho",
        MeshEntityKind::Cell,
        TagValues::F64(vec![o.tag_f, 2.0]),
    )
    .unwrap();
    b.add_set(
        "blk",
        SetMembers::Cells(o.set.iter().map(|&c| CellIx(c)).collect()),
    )
    .unwrap();
    if o.lineage {
        b.set_cell_parent(cells[1], cells[0]).unwrap();
        b.set_cell_active(cells[0], false).unwrap();
    }
    b.finish(&mut SerialAlloc::new()).unwrap().mesh
}

fn hashes(m: &ReferenceMesh) -> (u64, u64, u64) {
    (connectivity_hash(m), topology_hash(m), geometry_hash(m))
}

#[test]
fn content_hashes_cover_all_stored_data() {
    let base = make(&Opts::default());
    assert_valid(&base);
    let h0 = hashes(&base);
    // Identical inputs: identical hashes.
    assert_eq!(h0, hashes(&make(&Opts::default())));

    // (change, expect connectivity, topology, geometry to differ)
    let cases: Vec<(&str, ReferenceMesh, [bool; 3])> = vec![
        (
            "Param only",
            make(&Opts {
                vparam: 0.26,
                ..Opts::default()
            }),
            [false, false, true],
        ),
        (
            "integer tag values only",
            make(&Opts {
                tag_i: 8,
                ..Opts::default()
            }),
            [false, true, false],
        ),
        (
            "real tag values only",
            make(&Opts {
                tag_f: 1.6,
                ..Opts::default()
            }),
            [false, false, true],
        ),
        (
            "set members only",
            make(&Opts {
                set: vec![1],
                ..Opts::default()
            }),
            [false, true, false],
        ),
        (
            "vertex class entity",
            make(&Opts {
                vclass: 3,
                ..Opts::default()
            }),
            [false, true, false],
        ),
        (
            "cell class entity",
            make(&Opts {
                cell_class: 2,
                ..Opts::default()
            }),
            [false, true, false],
        ),
        (
            "a coordinate",
            make(&Opts {
                x_shift: 1e-12,
                ..Opts::default()
            }),
            [false, false, true],
        ),
        (
            "lineage",
            make(&Opts {
                lineage: true,
                ..Opts::default()
            }),
            [true, true, false],
        ),
    ];
    for (name, m, [conn, topo, geom]) in cases {
        let h = hashes(&m);
        assert_eq!(h.0 != h0.0, conn, "{name}: connectivity hash");
        assert_eq!(h.1 != h0.1, topo, "{name}: topology hash");
        assert_eq!(h.2 != h0.2, geom, "{name}: geometry hash");
    }

    // Edits on a finished mesh.
    let edit = |f: &dyn Fn(&mut ReferenceMesh)| {
        let mut m = base.clone();
        f(&mut m);
        hashes(&m)
    };
    let moved = edit(&|m| MeshEdit::set_point(m, PointIx(3), Point3::new(0.0, 1.0, 0.5)).unwrap());
    assert_eq!((moved.1 == h0.1, moved.2 != h0.2), (true, true));
    let class_edit = edit(&|m| {
        MeshEdit::set_class(
            m,
            MeshEntity::Vertex(VertexIx(5)),
            on(EntityKind::TopoVertex, 77),
        )
        .unwrap()
    });
    assert_ne!(class_edit.1, h0.1);
    let param_edit = edit(&|m| {
        MeshEdit::set_class(
            m,
            MeshEntity::Vertex(VertexIx(0)),
            Classification {
                entity: id(EntityKind::TopoEdge, 1),
                param: Param::Curve(0.9),
            },
        )
        .unwrap()
    });
    assert_eq!((param_edit.1 == h0.1, param_edit.2 != h0.2), (true, true));
    let tag_edit = edit(&|m| {
        MeshEdit::set_tag(m, "mat", MeshEntityKind::Cell, TagValues::I64(vec![7, 1])).unwrap()
    });
    assert_ne!(tag_edit.1, h0.1);
    let set_edit = edit(&|m| {
        MeshEdit::add_set(
            m,
            "extra",
            SetMembers::Vertices(vec![VertexIx(0)]),
            &mut SerialAlloc::new(),
        )
        .unwrap();
    });
    assert_ne!(set_edit.1, h0.1);
    let new_tag = edit(&|m| {
        MeshEdit::set_tag(
            m,
            "fresh",
            MeshEntityKind::Vertex,
            TagValues::F64(vec![0.0; 12]),
        )
        .unwrap()
    });
    assert_ne!(
        new_tag.1, h0.1,
        "a new real tag changes the topology hash through its name"
    );
    // Face kind.
    let fk = edit(&|m| {
        let f = (0..m.face_count())
            .map(FaceIx::new)
            .find(|&f| m.is_boundary_face(f))
            .unwrap();
        MeshEdit::set_face_kind(m, f, FaceKind::NonConformal).unwrap();
        MeshEdit::set_class(m, MeshEntity::Face(f), vol()).unwrap();
    });
    assert_ne!(fk.0, h0.0, "face kind is part of the connectivity hash");
    // The geometry cache is derived and excluded from both hashes.
    let cached = edit(&|m| {
        let c = compute_geometry_cache(m);
        MeshEdit::set_geometry_cache(m, Some(c)).unwrap();
    });
    assert_eq!(cached, h0);
    // A cache for different connectivity is refused.
    let mut m = base.clone();
    let wrong =
        compute_geometry_cache(&hex_block([1, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap());
    assert!(MeshEdit::set_geometry_cache(&mut m, Some(wrong)).is_err());
}

#[test]
fn edge_table_is_hashed() {
    let a = hex_block([1, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let b = hex_block_with_edges([1, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    assert_ne!(topology_hash(&a), topology_hash(&b));
    // Edge Params are real-valued stored data, so a present table also moves the geometry hash.
    assert_ne!(geometry_hash(&a), geometry_hash(&b));
    let mut c = b.clone();
    MeshEdit::set_class(
        &mut c,
        MeshEntity::Edge(EdgeIx(0)),
        Classification {
            entity: id(EntityKind::TopoEdge, 1),
            param: Param::Curve(0.5),
        },
    )
    .unwrap();
    assert_ne!(geometry_hash(&c), geometry_hash(&b), "edge Param is hashed");
    let mut c = b.clone();
    MeshEdit::set_class(
        &mut c,
        MeshEntity::Edge(EdgeIx(0)),
        on(EntityKind::TopoEdge, 12),
    )
    .unwrap();
    assert_ne!(
        topology_hash(&c),
        topology_hash(&b),
        "edge classification is hashed"
    );
}

#[test]
fn untested_codes_are_triggered() {
    let m = base();
    let last = m.face_count() as u32 - 1;
    // Face order: the last face claims cell 0, which sorts before its predecessor.
    let mut b = Broken::of(m.clone());
    b.face_left = Some((last, 0));
    expect(&b, ViolationCode::FaceOrder);
    // Index range: a face whose left cell does not exist.
    let mut b = Broken::of(m.clone());
    b.face_left = Some((0, 99));
    expect(&b, ViolationCode::IndexRange);
    // Normal direction: mirror every point; the loops now point the wrong way.
    let mut mm = m.clone();
    for i in 0..mm.coords().len() {
        let p = mm.coords().get(i);
        MeshEdit::set_point(&mut mm, PointIx::new(i), Point3::new(-p.x, p.y, p.z)).unwrap();
    }
    let v = validate_geometry(&mm, &GeomTol::default());
    assert!(
        v.iter().any(|x| x.code == ViolationCode::GeomNormal),
        "{v:?}"
    );
}

#[test]
fn geometry_cache_is_keyed_on_coordinates() {
    let m = base();
    let stale = compute_geometry_cache(&m);
    let mut moved = m.clone();
    MeshEdit::set_point(&mut moved, PointIx(0), Point3::new(-0.25, -0.25, -0.25)).unwrap();
    // Same connectivity, different coordinates.
    assert_eq!(connectivity_hash(&m), connectivity_hash(&moved));
    assert!(MeshEdit::set_geometry_cache(&mut moved, Some(stale.clone())).is_err());
    assert!(moved.geometry_cache().is_none());
    // A cache attached by another implementation is caught by validate.
    let mut b = Broken::of(moved);
    b.cache = Some(stale);
    expect(&b, ViolationCode::Cache);
    // A matching cache is accepted.
    let mut ok = m.clone();
    MeshEdit::set_geometry_cache(&mut ok, Some(compute_geometry_cache(&m))).unwrap();
    assert!(validate(&ok).is_empty());
}
