//! Replay a finished mesh into any `MeshBuild` (the differential harness).
#![allow(dead_code)]

use sm_core::Point3;
use sm_mesh_api::*;

/// How to feed the faces and cells to the builder.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// `add_face` for every source face (provisional index == source index), then `add_cell`.
    Explicit,
    /// `add_polyhedron_with_corners` / `add_cell_from_vertices` (the mesher path).
    Mesher,
}

fn node(nv: usize, p: PointIx) -> NodeRef {
    if p.get() < nv { NodeRef::Vertex(VertexIx(p.0)) } else { NodeRef::Extra((p.get() - nv) as u32) }
}

/// Re-create `src` in `b` and finish it with `alloc`. Returns the finished mesh and the
/// provisional face index of every source face.
pub fn replay<B: MeshBuild>(
    src: &dyn MeshRead,
    mut b: B,
    mode: Mode,
    alloc: &mut SerialAlloc,
) -> Result<(Finished<B::Output>, Vec<FaceIx>)> {
    let (nv, nf, nc) = (src.vertex_count(), src.face_count(), src.cell_count());
    let co = src.coords();
    for i in 0..nv {
        b.add_vertex(co.get(i))?;
    }
    for i in nv..co.len() {
        b.add_extra_node(co.get(i))?;
    }
    let outward = |c: CellIx| -> Vec<Vec<VertexIx>> {
        src.cell_faces(c)
            .iter()
            .map(|sf| {
                let l = src.face_vertices(sf.face());
                if sf.side() == Side::Left { l.to_vec() } else { l.iter().rev().copied().collect() }
            })
            .collect()
    };
    let mut prov: Vec<Option<FaceIx>> = vec![None; nf];
    if mode == Mode::Explicit {
        for f in 0..nf {
            prov[f] = Some(b.add_face(src.face_vertices(FaceIx::new(f)))?);
        }
    }
    for c in 0..nc {
        let ci = CellIx::new(c);
        let corners = src.cell_corners(ci);
        let cell = match mode {
            Mode::Explicit => {
                let sfs: Vec<SignedFace> = src
                    .cell_faces(ci)
                    .iter()
                    .map(|sf| SignedFace::new(prov[sf.face().get()].unwrap_or(sf.face()), sf.side()))
                    .collect();
                b.add_cell(&sfs, corners)?
            }
            Mode::Mesher => {
                let loops = outward(ci);
                let refs: Vec<&[VertexIx]> = loops.iter().map(Vec::as_slice).collect();
                let added = b.add_polyhedron_with_corners(&refs, corners)?;
                for (k, sf) in src.cell_faces(ci).iter().enumerate() {
                    prov[sf.face().get()].get_or_insert(added.faces[k].face());
                }
                added.cell
            }
        };
        assert_eq!(cell, ci);
    }
    let p = |f: FaceIx| prov[f.get()].unwrap_or(f);
    for v in 0..nv {
        b.set_class(MeshEntityRef::Vertex(VertexIx::new(v)), src.vertex_class(VertexIx::new(v)))?;
    }
    for c in 0..nc {
        let ci = CellIx::new(c);
        b.set_class(MeshEntityRef::Cell(ci), src.cell_class(ci))?;
        if let Some(pa) = src.cell_parent(ci) {
            b.set_cell_parent(ci, pa)?;
        }
        if !src.cell_active(ci) {
            b.set_cell_active(ci, false)?;
        }
    }
    for f in 0..nf {
        let fi = FaceIx::new(f);
        b.set_class(MeshEntityRef::Face(p(fi)), src.face_class(fi))?;
        if matches!(src.face_kind(fi), FaceKind::NonConformal | FaceKind::PartitionCut) {
            b.set_face_kind(p(fi), src.face_kind(fi))?;
        }
    }
    if let Some(t) = src.edges() {
        for e in 0..t.len() {
            let [a, c] = t.vertices[e];
            if src.dim() == MeshDim::Three {
                b.set_class(MeshEntityRef::Edge(a, c), t.class[e])?;
            }
        }
    }
    for c in 0..nc {
        if let Some(h) = src.high_order_cell(CellIx::new(c)) {
            let nodes: Vec<NodeRef> = h.nodes.iter().map(|n| node(nv, *n)).collect();
            b.set_high_order(CellIx::new(c), h.degree, h.basis, &nodes)?;
        }
    }
    for f in 0..nf {
        if let Some(fc) = src.face_curve(FaceIx::new(f)) {
            let nodes: Vec<NodeRef> = fc.nodes.iter().map(|n| node(nv, *n)).collect();
            b.set_face_curve(p(FaceIx::new(f)), fc.degree, fc.basis, &nodes)?;
        }
    }
    for s in src.sets() {
        let members = match &s.members {
            SetMembers::Faces(v) => SetMembers::Faces(v.iter().map(|f| p(*f)).collect()),
            SetMembers::Sides(v) => SetMembers::Sides(v.iter().map(|sf| SignedFace::new(p(sf.face()), sf.side())).collect()),
            m => m.clone(),
        };
        b.add_set(&s.name, members)?;
        if let Some(l) = s.label {
            b.set_label(&s.name, Some(l))?;
        }
    }
    for t in src.tags() {
        if t.target == MeshEntityKind::Edge {
            continue;
        }
        let values = if t.target == MeshEntityKind::Face {
            // Face columns are indexed by provisional handle.
            let mut inv = vec![0usize; nf];
            for f in 0..nf {
                inv[p(FaceIx::new(f)).get()] = f;
            }
            match &t.values {
                TagValues::I64(v) => TagValues::I64(inv.iter().map(|&f| v[f]).collect()),
                TagValues::F64(v) => TagValues::F64(inv.iter().map(|&f| v[f]).collect()),
                o => o.clone(),
            }
        } else {
            t.values.clone()
        };
        b.add_tag(&t.name, t.target, values)?;
    }
    let fin = b.finish(alloc)?;
    let prov_vec = (0..nf).map(|f| p(FaceIx::new(f))).collect();
    Ok((fin, prov_vec))
}

/// Edge tags (not buildable) are applied on the finished mesh.
pub fn copy_edge_tags(src: &dyn MeshRead, dst: &mut impl MeshEdit) {
    for t in src.tags().iter().filter(|t| t.target == MeshEntityKind::Edge) {
        dst.set_tag(&t.name, t.target, t.values.clone()).unwrap();
    }
}

/// Everything observable through the trait must agree.
pub fn assert_same(a: &dyn MeshRead, b: &dyn MeshRead, what: &str) {
    assert_eq!(a.dim(), b.dim(), "{what}: dim");
    assert_eq!(a.mesh_id(), b.mesh_id(), "{what}: mesh id");
    for k in [MeshEntityKind::Vertex, MeshEntityKind::Edge, MeshEntityKind::Face, MeshEntityKind::Cell] {
        assert_eq!(a.count(k), b.count(k), "{what}: count {k:?}");
        for i in 0..a.count(k) {
            let id = a.id(k, i as u32);
            assert_eq!(id, b.id(k, i as u32), "{what}: id {k:?} {i}");
            assert_eq!(b.index_of(id), Some(i as u32), "{what}: index_of {k:?} {i}");
            assert_eq!(a.classification(k, i as u32), b.classification(k, i as u32), "{what}: class {k:?} {i}");
        }
    }
    let (ca, cb) = (a.coords(), b.coords());
    assert_eq!(ca.len(), cb.len(), "{what}: points");
    for i in 0..ca.len() {
        let (p, q) = (ca.get(i), cb.get(i));
        assert_eq!((p.x.to_bits(), p.y.to_bits(), p.z.to_bits()), (q.x.to_bits(), q.y.to_bits(), q.z.to_bits()), "{what}: point {i}");
    }
    let (ta, tb) = (a.columns(), b.columns());
    assert_eq!(ta.face_vertex_offsets, tb.face_vertex_offsets, "{what}: face offsets");
    assert_eq!(ta.face_vertices, tb.face_vertices, "{what}: face vertices");
    assert_eq!(ta.face_left, tb.face_left, "{what}: face_left");
    assert_eq!(ta.face_right, tb.face_right, "{what}: face_right");
    assert_eq!(ta.cell_face_offsets, tb.cell_face_offsets, "{what}: cell offsets");
    assert_eq!(ta.cell_faces, tb.cell_faces, "{what}: cell faces");
    for f in 0..a.face_count() {
        let f = FaceIx::new(f);
        assert_eq!(a.face_kind(f), b.face_kind(f), "{what}: face kind {f:?}");
        assert_eq!(a.face_vertices(f), b.face_vertices(f));
        assert_eq!(a.face_curve(f), b.face_curve(f), "{what}: face curve {f:?}");
    }
    for c in 0..a.cell_count() {
        let c = CellIx::new(c);
        assert_eq!(a.cell_shape(c), b.cell_shape(c), "{what}: shape {c:?}");
        assert_eq!(a.cell_corners(c), b.cell_corners(c), "{what}: corners {c:?}");
        assert_eq!(a.high_order_cell(c), b.high_order_cell(c), "{what}: ho {c:?}");
        assert_eq!(a.cell_parent(c), b.cell_parent(c), "{what}: parent {c:?}");
        assert_eq!(a.cell_active(c), b.cell_active(c), "{what}: active {c:?}");
    }
    assert_eq!(a.classified_entities(), b.classified_entities(), "{what}: class entities");
    for e in a.classified_entities() {
        assert_eq!(a.classified_on(*e), b.classified_on(*e), "{what}: classified_on {e:?}");
    }
    assert_eq!(a.edges(), b.edges(), "{what}: edge table");
    assert_eq!(a.sets(), b.sets(), "{what}: sets");
    assert_eq!(a.tags(), b.tags(), "{what}: tags");
    assert_eq!(a.geometry_cache(), b.geometry_cache(), "{what}: cache");
    assert_eq!(connectivity_hash(a), connectivity_hash(b), "{what}: connectivity hash");
    assert_eq!(topology_hash(a), topology_hash(b), "{what}: topology hash");
    assert_eq!(geometry_hash(a), geometry_hash(b), "{what}: geometry hash");
    assert_eq!(geometry_hash_quantized(a, &HashQuanta::new(1e-9, 1e-9)), geometry_hash_quantized(b, &HashQuanta::new(1e-9, 1e-9)));
    assert!(validate(b).is_empty(), "{what}: {:?}", validate(b));
    let _ = Point3::new(0.0, 0.0, 0.0);
}
