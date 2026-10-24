//! `.smsh` writer / reader: round trips, container layout, corruption, metadata.

mod common;

use common::*;
use sm_core::{EntityKind, Point3, Vector3};
use sm_meshio::smsh::{ALIGN, DeckParams, MAGIC, SmshFile};
use sm_meshio::{DeckRecord, MeshioError, SmshWriter};
use sm_mesh_api::reference::{ReferenceBuilder, hex_block};
use sm_mesh_api::*;
use sm_model_api::{Axis, Deck, Plane, Turns};

fn bytes(m: &dyn MeshRead) -> Vec<u8> {
    SmshWriter::new(m).to_bytes().unwrap()
}

#[test]
fn every_corpus_mesh_round_trips_exactly() {
    for (name, m) in corpus() {
        assert!(validate(&m).is_empty(), "{name}: {:?}", validate(&m));
        let b = bytes(&m);
        let f = SmshFile::parse(&b).unwrap_or_else(|e| panic!("{name}: {e}"));
        f.verify_checksums().unwrap();
        let d = f.decode().unwrap_or_else(|e| panic!("{name}: {e}"));
        assert_matches(&m, &d);
    }
}

#[test]
fn corpus_covers_the_interesting_features() {
    let c = corpus();
    let get = |n: &str| &c.iter().find(|(k, _)| k == n).unwrap().1;
    assert!(get("decorated_block").edges().is_some() && get("decorated_block").geometry_cache().is_some());
    assert!(get("ho_hex").high_order_cell(CellIx(0)).is_some() && get("ho_hex").coords().len() > 8);
    assert_eq!(get("surface_strip").dim(), MeshDim::Two);
    assert!(!get("lineage_pair").cell_active(CellIx(0)));
    assert!((0..get("truncated_octahedron").cell_count()).any(|i| get("truncated_octahedron").cell_shape(CellIx::new(i)) == CellShape::Polyhedron));
    let d = bytes(get("decorated_block"));
    let d = SmshFile::parse(&d).unwrap().decode().unwrap();
    assert!(d.vertex_class.iter().any(|c| matches!(c.param, Param::Surface([u, v]) if u == 0.25 && v == 0.5)));
}

#[test]
fn container_layout_follows_the_spec() {
    let (_, m) = corpus().into_iter().find(|(n, _)| n == "decorated_block").unwrap();
    let b = bytes(&m);
    assert_eq!(b[..8], MAGIC);
    let hl = u64::from_le_bytes(b[8..16].try_into().unwrap());
    assert_eq!((16 + hl) % ALIGN, 0, "data region is 64-byte aligned");
    let json = &b[16..16 + hl as usize];
    assert_eq!(json[0], b'{');
    assert_eq!(*json.trim_ascii_end().last().unwrap(), b'}', "header padded with spaces only");
    let f = SmshFile::parse(&b).unwrap();
    let mut end = 0;
    for a in &f.header().arrays {
        assert_eq!(a.offset % ALIGN, 0, "{}", a.name);
        assert!(a.offset >= end, "{} overlaps its predecessor", a.name);
        end = a.offset + a.nbytes;
        assert_eq!(f.raw(&a.name).unwrap().len() as u64, a.nbytes);
    }
    assert_eq!(b.len() as u64, 16 + hl + end, "no trailing bytes");
    // Spot-check against the mesh: x is a plain f64 array.
    let x = f.f64s("x").unwrap();
    assert_eq!(x.len(), m.coords().len());
    assert_eq!(x[1].to_bits(), m.coords().x[1].to_bits());
    // Names are unique and the sets/tags point at existing arrays.
    for s in &f.header().sets {
        assert!(f.header().array(&s.array).is_some());
    }
}

#[test]
fn corruption_is_detected() {
    let (_, m) = corpus().into_iter().find(|(n, _)| n == "decorated_block").unwrap();
    let good = bytes(&m);
    // Flip a payload byte: parse succeeds, checksums fail.
    let f = SmshFile::parse(&good).unwrap();
    let x = f.header().array("x").unwrap().clone();
    let hl = u64::from_le_bytes(good[8..16].try_into().unwrap());
    let mut bad = good.clone();
    bad[(16 + hl + x.offset) as usize + 3] ^= 0x40;
    let f = SmshFile::parse(&bad).unwrap();
    assert!(matches!(f.verify_checksums(), Err(MeshioError::Malformed(_))));
    // Truncation: an array runs past the end.
    assert!(matches!(SmshFile::parse(&good[..good.len() - 8]), Err(MeshioError::Malformed(_))));
    // Bad magic and empty input.
    let mut m2 = good.clone();
    m2[0] = b'X';
    assert!(matches!(SmshFile::parse(&m2), Err(MeshioError::Malformed(_))));
    assert!(SmshFile::parse(&[]).is_err());
    // A newer major version is refused with a typed error.
    let text = String::from_utf8(good[16..16 + hl as usize].to_vec()).unwrap();
    let patched = text.replacen("\"version\":{\"major\":1,", "\"version\":{\"major\":2,", 1);
    assert_eq!(patched.len(), text.len());
    let mut v2 = good.clone();
    v2[16..16 + hl as usize].copy_from_slice(patched.as_bytes());
    assert!(matches!(SmshFile::parse(&v2), Err(MeshioError::UnsupportedVersion { major: 2, .. })));
}

#[test]
fn unaligned_input_still_decodes() {
    // Shift the whole file by one byte inside a larger buffer so typed views cannot borrow.
    let (_, m) = corpus().into_iter().find(|(n, _)| n == "hex_block").unwrap();
    let b = bytes(&m);
    let mut shifted = vec![0u8];
    shifted.extend_from_slice(&b);
    let d = SmshFile::parse(&shifted[1..]).unwrap().decode().unwrap();
    assert_matches(&m, &d);
}

#[test]
fn provenance_and_decks_are_recorded() {
    let mut alloc = SerialAlloc::new();
    let mut m = hex_block([2, 2, 1], [1.0; 3], &mut alloc).unwrap();
    let wall: Vec<FaceIx> = (0..m.face_count()).map(FaceIx::new).filter(|f| m.is_boundary_face(*f) && m.face_class(*f).entity.serial() == 1).collect();
    MeshEdit::add_set(&mut m, "sym_x", SetMembers::Faces(wall), &mut alloc).unwrap();
    MeshEdit::add_set(&mut m, "mate", SetMembers::Faces(vec![]), &mut alloc).unwrap();
    let mirror = Deck::Mirror { plane: Plane::new(Vector3::new(1.0, 0.0, 0.0), 0.0).unwrap() };
    let rot = Deck::rotation(Axis::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap(), Turns::new(1, 4).unwrap());
    let d1 = DeckRecord::from_deck("sym", &mirror, Some("sym_x".into()), None).unwrap();
    let d2 = DeckRecord::from_deck("rot", &rot, Some("sym_x".into()), Some("mate".into())).unwrap();
    assert!(d1.improper && d1.order == Some(2));
    assert_eq!(d2.order, Some(4));
    assert!(matches!(d2.params, DeckParams::Rotation { turns: [1, 4], .. }));
    // Rotation by a quarter turn about z: matrix [[0,-1,0],[1,0,0],[0,0,1]] within rounding.
    assert!((d2.matrix[1] + 1.0).abs() < 1e-12 && (d2.matrix[3] - 1.0).abs() < 1e-12 && d2.matrix[8] == 1.0);
    let b = SmshWriter::new(&m).provenance("00ff00ff00ff00ff").deck(d1.clone()).deck(d2.clone()).to_bytes().unwrap();
    let f = SmshFile::parse(&b).unwrap();
    assert_eq!(f.header().provenance.as_ref().unwrap().timeline_hash, "00ff00ff00ff00ff");
    assert_eq!(f.header().decks, vec![d1.clone(), d2]);
    assert_eq!(f.header().sets[0].label, None);
    // A deck naming a set that does not exist is rejected.
    let mut bad = d1.clone();
    bad.face_set = Some("nope".into());
    assert!(matches!(SmshWriter::new(&m).deck(bad).to_bytes(), Err(MeshioError::Inconsistent(_))));
    // Non-finite numbers cannot enter the JSON header.
    let mut nan = d1;
    nan.matrix[0] = f64::NAN;
    assert!(matches!(SmshWriter::new(&m).deck(nan).to_bytes(), Err(MeshioError::Inconsistent(_))));
}

fn centre(m: &dyn MeshRead, c: CellIx) -> Point3 {
    cell_volume_centroid(m, c).1
}

#[test]
fn hierarchy_maps_are_stored_with_the_mesh() {
    let mut alloc = SerialAlloc::new();
    let fine = hex_block([4, 4, 2], [1.0; 3], &mut alloc).unwrap();
    let coarse = hex_block([2, 2, 1], [2.0; 3], &mut alloc).unwrap();
    let coarse_of: Vec<CellIx> = (0..fine.cell_count())
        .map(|c| {
            let p = centre(&fine, CellIx::new(c));
            CellIx::new((p.x / 2.0).floor() as usize + 2 * (p.y / 2.0).floor() as usize)
        })
        .collect();
    let lm = LevelMap::new(&fine, &coarse, coarse_of.clone(), None).unwrap();
    // Extrusion: strip base times 3 layers.
    let base = {
        let mut b = ReferenceBuilder::surface();
        let pts = [[0., 0.], [1., 0.], [2., 0.], [0., 1.], [1., 1.], [2., 1.]];
        let vs: Vec<VertexIx> = pts
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let v = b.add_vertex(Point3::new(p[0], p[1], 0.0)).unwrap();
                b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoVertex, 1 + i as u64)).unwrap();
                v
            })
            .collect();
        for q in [[0, 1, 4, 3], [1, 2, 5, 4]] {
            let c = b.add_polygon(&q.map(|i| vs[i])).unwrap();
            b.set_class(MeshEntityRef::Cell(c.cell), on(EntityKind::TopoFace, 1)).unwrap();
        }
        for f in 0..b.face_count() {
            let fi = FaceIx::new(f);
            let c = if b.face_degree(fi) == 1 { on(EntityKind::TopoEdge, 1 + f as u64) } else { on(EntityKind::TopoFace, 1) };
            b.set_class(MeshEntityRef::Face(fi), c).unwrap();
        }
        b.finish(&mut alloc).unwrap().mesh
    };
    let ext = hex_block([2, 1, 3], [1.0; 3], &mut alloc).unwrap();
    let cell_of: Vec<(CellIx, u32)> = (0..ext.cell_count())
        .map(|c| {
            let p = centre(&ext, CellIx::new(c));
            (CellIx::new(p.x.floor() as usize), p.z.floor() as u32)
        })
        .collect();
    let xm = ExtrusionMap::new(&base, &ext, vec![0.0, 1.0, 2.0, 3.0], cell_of.clone()).unwrap();

    let b = SmshWriter::new(&fine).level_map(&lm).to_bytes().unwrap();
    let d = SmshFile::parse(&b).unwrap().decode().unwrap();
    assert_eq!(d.level_maps.len(), 1);
    let l = &d.level_maps[0];
    assert_eq!((l.fine, l.coarse), (fine.mesh_id(), coarse.mesh_id()));
    assert_eq!((l.fine_hash, l.coarse_hash), (connectivity_hash(&fine), connectivity_hash(&coarse)));
    assert_eq!(l.coarse_of, coarse_of.iter().map(|c| c.0).collect::<Vec<_>>());
    assert!(l.face_map.is_none());
    // The coarse mesh's file carries the same map.
    let d = SmshFile::parse(&SmshWriter::new(&coarse).level_map(&lm).to_bytes().unwrap()).unwrap().decode().unwrap();
    assert_eq!(d.level_maps[0].coarse_of.len(), fine.cell_count());
    // A mesh outside the pair is refused.
    assert!(matches!(SmshWriter::new(&base).level_map(&lm).to_bytes(), Err(MeshioError::Inconsistent(_))));

    for (m, n_ext) in [(&ext as &dyn MeshRead, ext.cell_count()), (&base, ext.cell_count())] {
        let d = SmshFile::parse(&SmshWriter::new(m).extrusion(&xm).to_bytes().unwrap()).unwrap().decode().unwrap();
        let x = &d.extrusions[0];
        assert_eq!((x.base, x.extruded), (base.mesh_id(), ext.mesh_id()));
        assert_eq!((x.base_hash, x.extruded_hash), (connectivity_hash(&base), connectivity_hash(&ext)));
        assert_eq!(x.z, vec![0.0, 1.0, 2.0, 3.0]);
        assert_eq!(x.cell_base.len(), n_ext);
        for (c, (bc, layer)) in cell_of.iter().enumerate() {
            assert_eq!((x.cell_base[c], x.cell_layer[c]), (bc.0, *layer));
        }
    }
    assert!(matches!(SmshWriter::new(&fine).extrusion(&xm).to_bytes(), Err(MeshioError::Inconsistent(_))));
}

#[test]
fn output_is_deterministic_across_runs_and_thread_counts() {
    use sm_testkit::determinism;
    for (name, m) in corpus() {
        let hash = |threads: usize| -> u64 {
            let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
            let b = pool.install(|| bytes(&m));
            // FNV-1a over the bytes, so the harness compares hashes.
            b.iter().fold(0xcbf2_9ce4_8422_2325u64, |h, x| (h ^ u64::from(*x)).wrapping_mul(0x0100_0000_01b3))
        };
        determinism::check(2, 4, hash).unwrap_or_else(|e| panic!("{name}: {e}"));
    }
}

#[test]
fn large_block_exercises_parallel_chunks() {
    // More than one CHUNK / BLOCK of cells so the parallel fill really splits.
    let m = hex_block([40, 30, 20], [0.1, 0.2, 0.3], &mut SerialAlloc::new()).unwrap();
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().unwrap().install(|| bytes(&m));
    let many = rayon::ThreadPoolBuilder::new().num_threads(7).build().unwrap().install(|| bytes(&m));
    assert!(one == many, "1-thread and 7-thread output differ");
    let d = SmshFile::parse(&one).unwrap().decode().unwrap();
    assert_matches(&m, &d);
}

/// Volume of cell `c` from the decoded arrays alone: `V = 1/3 sum_f s_f (p_f . A_f)` with the
/// Newell area vector of each loop (exact for planar faces), `s_f` the stored cell sign.
fn volume_from_file(d: &sm_meshio::SmshMesh, c: usize) -> f64 {
    let p = |v: u32| [d.x[v as usize], d.y[v as usize], d.z[v as usize]];
    let mut vol = 0.0;
    for sf in d.cell_face_list(c) {
        let l = d.face_loop(sf.face().get());
        let (mut a, o) = ([0.0f64; 3], p(l[0]));
        for i in 0..l.len() {
            let (u, w) = (p(l[i]), p(l[(i + 1) % l.len()]));
            a[0] += (u[1] - o[1]) * (w[2] - o[2]) - (u[2] - o[2]) * (w[1] - o[1]);
            a[1] += (u[2] - o[2]) * (w[0] - o[0]) - (u[0] - o[0]) * (w[2] - o[2]);
            a[2] += (u[0] - o[0]) * (w[1] - o[1]) - (u[1] - o[1]) * (w[0] - o[0]);
        }
        vol += sf.sign() * (o[0] * a[0] + o[1] * a[1] + o[2] * a[2]) / 6.0;
    }
    vol
}

#[test]
fn volumes_recomputed_from_the_file_match_the_analytic_fixtures() {
    for f in sm_testkit::meshes::all_mesh_fixtures().unwrap() {
        let d = SmshFile::parse(&bytes(&f.mesh)).unwrap().decode().unwrap();
        for (c, want) in f.cell_volumes.iter().enumerate() {
            let got = volume_from_file(&d, c);
            assert!((got - want).abs() <= 1e-12 * want.abs().max(1.0), "{} cell {c}: {got} vs {want}", f.name);
        }
    }
}

#[cfg(feature = "mmap")]
#[test]
fn mapped_file_matches_the_in_memory_reader() {
    let (_, m) = corpus().into_iter().find(|(n, _)| n == "decorated_block").unwrap();
    let path = common::py::tmp("mapped.smsh");
    SmshWriter::new(&m).write_file(&path).unwrap();
    let map = sm_meshio::smsh::SmshMap::open(&path).unwrap();
    let f = map.file().unwrap();
    f.verify_checksums().unwrap();
    assert_matches(&m, &f.decode().unwrap());
    // The mapped data region is 64-aligned from a page boundary, so typed views borrow.
    assert!(matches!(f.f64s("x").unwrap(), std::borrow::Cow::Borrowed(_)));
}

#[test]
fn hashes_can_be_skipped() {
    let (_, m) = corpus().into_iter().find(|(n, _)| n == "hex_block").unwrap();
    let b = SmshWriter::new(&m).without_hashes().to_bytes().unwrap();
    let f = SmshFile::parse(&b).unwrap();
    assert!(f.header().mesh.hashes.is_none());
    f.decode().unwrap();
    assert!(b.len() < bytes(&m).len());
}
