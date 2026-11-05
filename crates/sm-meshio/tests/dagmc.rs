//! DAGMC `.h5m` export: analytic checks through the file (read back with the in-crate reader of
//! the written HDF5 subset), senses by offset points and the divergence theorem, closed
//! volumes, quads and polygons, the graveyard, facets, determinism and property tests.

mod common;

use common::wrap::Wrap;
use proptest::prelude::*;
use sm_core::{EntityId, EntityKind};
use sm_mesh_api::*;
use sm_meshio::dagmc::*;
use sm_meshio::h5::read::read_file;
use sm_rt_api::FacetInput;
use sm_testkit::meshes::{MeshFixture, SetSpec, hex_block_two_volumes, hexagonal_prism, tet_block_two_volumes, with_sets};
use sm_testkit::strategies;
use sm_testkit::surf3::icosphere;
use sm_topo_api::Region;
use std::collections::BTreeMap;

type V3 = [f64; 3];

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}
fn area(t: [V3; 3]) -> f64 {
    norm(cross(sub(t[1], t[0]), sub(t[2], t[0]))) / 2.0
}

fn export(mesh: &dyn MeshRead, o: &DagmcOptions) -> (Vec<u8>, DagmcReport) {
    let mut out = Vec::new();
    let r = export_h5m(mesh, o, &mut out).unwrap();
    (out, r)
}

/// One surface read back from the file.
struct Surf {
    gid: i32,
    tris: Vec<usize>,
    fwd: Option<i32>,
    bwd: Option<i32>,
}

/// An `.h5m` read back: geometry, senses as volume `GLOBAL_ID`s, groups as (name, volume gids).
struct H5m {
    coords: Vec<V3>,
    tris: Vec<[usize; 3]>,
    surfaces: Vec<Surf>,
    volumes: Vec<i32>,
    groups: Vec<(String, Vec<i32>)>,
}

fn start_id(d: &sm_meshio::h5::Dataset) -> i64 {
    let a = d.attrs.iter().find(|a| a.name == "start_id").expect("start_id");
    i64::from_le_bytes(a.data[..8].try_into().unwrap())
}

fn cstr(b: &[u8]) -> String {
    let z = b.iter().position(|x| *x == 0).unwrap_or(b.len());
    String::from_utf8(b[..z].to_vec()).unwrap()
}

fn read_h5m(bytes: &[u8]) -> H5m {
    let g = read_file(bytes).unwrap();
    let coords: Vec<V3> = g.dataset("tstt/nodes/coordinates").unwrap().as_f64().chunks(3).map(|c| [c[0], c[1], c[2]]).collect();
    let conn = g.dataset("tstt/elements/Tri3/connectivity").unwrap();
    let tri0 = start_id(conn) as u64;
    let tris: Vec<[usize; 3]> = conn.as_u64().chunks(3).map(|c| [c[0] as usize - 1, c[1] as usize - 1, c[2] as usize - 1]).collect();
    assert_eq!(tri0 as usize, coords.len() + 1);
    let list = g.dataset("tstt/sets/list").unwrap();
    let set0 = start_id(list) as u64;
    assert_eq!(set0 as usize, coords.len() + tris.len() + 1);
    let rows: Vec<[i64; 4]> = list.as_i64().chunks(4).map(|c| [c[0], c[1], c[2], c[3]]).collect();
    let get = |n: &str| g.dataset(&format!("tstt/sets/{n}")).map(|d| d.as_u64()).unwrap_or_default();
    let (contents, children, parents) = (get("contents"), get("children"), get("parents"));
    let range = |tbl: &[u64], rows: &[[i64; 4]], col: usize, k: usize| -> Vec<u64> {
        let lo = if k == 0 { 0 } else { (rows[k - 1][col] + 1) as usize };
        let hi = (rows[k][col] + 1) as usize;
        tbl[lo..hi].to_vec()
    };
    assert_eq!(*rows.last().unwrap().first().unwrap() + 1, contents.len() as i64);
    let tag = |n: &str| (g.dataset(&format!("tstt/tags/{n}/id_list")).map(|d| d.as_u64()).unwrap_or_default(), g.dataset(&format!("tstt/tags/{n}/values")));
    let (dim_ids, dim_vals) = tag("GEOM_DIMENSION");
    let dims = dim_vals.unwrap().as_i32();
    let (cat_ids, cat_vals) = tag("CATEGORY");
    let cats: Vec<String> = cat_vals.unwrap().data.chunks(32).map(cstr).collect();
    let (sense_ids, sense_vals) = tag("GEOM_SENSE_2");
    let sense = sense_vals.unwrap().as_u64();
    let gid = g.dataset("tstt/sets/tags/GLOBAL_ID").unwrap().as_i32();
    let gid_of = |h: u64| gid[(h - set0) as usize];
    assert_eq!(dim_ids.len(), dims.len());
    let mut surfaces = Vec::new();
    let mut volumes = Vec::new();
    let mut groups = Vec::new();
    let names: BTreeMap<u64, String> = {
        let (ids, vals) = tag("NAME");
        match vals {
            Some(v) => ids.into_iter().zip(v.data.chunks(32).map(cstr)).collect(),
            None => BTreeMap::new(),
        }
    };
    for (k, row) in rows.iter().enumerate() {
        let h = set0 + k as u64;
        let Some(ci) = cat_ids.iter().position(|x| *x == h) else { continue };
        match cats[ci].as_str() {
            "Surface" => {
                let di = dim_ids.iter().position(|x| *x == h).unwrap();
                assert_eq!(dims[di], 2);
                let si = sense_ids.iter().position(|x| *x == h).unwrap();
                let tri_handles = range(&contents, &rows, 0, k);
                assert!(tri_handles.iter().all(|t| *t >= tri0 && *t < set0));
                let to = |v: u64| (v != 0).then(|| gid_of(v));
                // parents must be the volumes of the sense tag
                let par = range(&parents, &rows, 2, k);
                let mut want: Vec<u64> = [sense[2 * si], sense[2 * si + 1]].into_iter().filter(|v| *v != 0).collect();
                want.sort_unstable();
                let mut got = par.clone();
                got.sort_unstable();
                assert_eq!(want, got, "surface parents are its sense volumes");
                surfaces.push(Surf { gid: gid_of(h), tris: tri_handles.iter().map(|t| (t - tri0) as usize).collect(), fwd: to(sense[2 * si]), bwd: to(sense[2 * si + 1]) });
                assert_eq!(row[3], 2);
            }
            "Volume" => {
                let di = dim_ids.iter().position(|x| *x == h).unwrap();
                assert_eq!(dims[di], 3);
                volumes.push(gid_of(h));
                let ch = range(&children, &rows, 1, k);
                assert!(!ch.is_empty());
            }
            "Group" => {
                let n = names.get(&h).expect("group has a NAME").clone();
                groups.push((n, range(&contents, &rows, 0, k).into_iter().map(gid_of).collect()));
            }
            c => panic!("category {c}"),
        }
    }
    H5m { coords, tris, surfaces, volumes, groups }
}

impl H5m {
    fn tri(&self, i: usize) -> [V3; 3] {
        self.tris[i].map(|v| self.coords[v])
    }
    fn surface_area(&self, s: &Surf) -> f64 {
        s.tris.iter().map(|&t| area(self.tri(t))).sum()
    }
    fn total_area(&self) -> f64 {
        self.surfaces.iter().map(|s| self.surface_area(s)).sum()
    }
    /// Volume by the divergence theorem over the triangles oriented out of volume `gid`.
    fn volume(&self, gid: i32) -> f64 {
        let mut v = 0.0;
        for s in &self.surfaces {
            let sign = if s.fwd == Some(gid) { 1.0 } else if s.bwd == Some(gid) { -1.0 } else { continue };
            for &t in &s.tris {
                let [a, b, c] = self.tri(t);
                v += sign * dot(a, cross(b, c)) / 6.0;
            }
        }
        v
    }
    /// Every edge of volume `gid` is used twice, once in each direction.
    fn volume_is_closed(&self, gid: i32) -> bool {
        let mut uses: BTreeMap<(usize, usize), (u32, u32)> = BTreeMap::new();
        for s in &self.surfaces {
            let rev = if s.fwd == Some(gid) { false } else if s.bwd == Some(gid) { true } else { continue };
            for &t in &s.tris {
                let tri = self.tris[t];
                for k in 0..3 {
                    let (a, b) = if rev { (tri[(k + 1) % 3], tri[k]) } else { (tri[k], tri[(k + 1) % 3]) };
                    let e = uses.entry((a.min(b), a.max(b))).or_default();
                    if a < b { e.0 += 1 } else { e.1 += 1 }
                }
            }
        }
        uses.values().all(|&(f, b)| f == 1 && b == 1)
    }
}

fn cells_of(mesh: &dyn MeshRead, volume: u64) -> Vec<CellIx> {
    (0..mesh.cell_count()).map(CellIx::new).filter(|c| mesh.cell_class(*c).entity.serial() == volume).collect()
}

fn two_volume_tet() -> MeshFixture {
    tet_block_two_volumes([4, 4, 4], [0.25; 3], 2).unwrap()
}

const BOXES: [([f64; 3], [f64; 3]); 2] = [([0.0, 0.0, 0.0], [0.5, 1.0, 1.0]), ([0.5, 0.0, 0.0], [1.0, 1.0, 1.0])];

fn inside(b: ([f64; 3], [f64; 3]), p: V3) -> bool {
    (0..3).all(|k| p[k] >= b.0[k] && p[k] <= b.1[k])
}

#[test]
fn two_volume_box_reads_back_with_analytic_areas() {
    let f = two_volume_tet();
    let (bytes, r) = export(&f.mesh, &DagmcOptions::default());
    let h = read_h5m(&bytes);
    // 4(n0 n1 + n1 n2 + n0 n2) skin triangles plus 2 n1 n2 interface triangles; skin + interface vertices.
    assert_eq!(h.tris.len(), 4 * (16 * 3) + 2 * 16);
    assert_eq!(h.coords.len(), 98 + 9);
    assert_eq!(h.volumes, vec![1, 2]);
    assert_eq!(h.surfaces.len(), 11);
    assert_eq!((r.triangles, r.surfaces, r.volumes, r.groups), (224, 11, 2, 0));
    assert_eq!(r.bytes as usize, bytes.len());
    // Analytic areas: interface 1, the two x end caps 1, eight half faces 0.5.
    let mut got: Vec<f64> = h.surfaces.iter().map(|s| h.surface_area(s)).collect();
    got.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut want = vec![0.5; 8];
    want.extend([1.0; 3]);
    for (g, w) in got.iter().zip(&want) {
        assert!((g - w).abs() <= 1e-12 * w, "{g} vs {w}");
    }
    // Senses: the interface is the only surface with two volumes; outer surfaces are [vol, none].
    let two: Vec<&Surf> = h.surfaces.iter().filter(|s| s.bwd.is_some()).collect();
    assert_eq!(two.len(), 1);
    let mut both = [two[0].fwd.unwrap(), two[0].bwd.unwrap()];
    both.sort_unstable();
    assert_eq!(both, [1, 2]);
    assert!(h.surfaces.iter().all(|s| s.fwd.is_some()));
    assert_eq!(h.surfaces.iter().map(|s| s.gid).collect::<Vec<_>>(), (1..=11).collect::<Vec<_>>());
}

#[test]
fn senses_by_offset_points_and_divergence_volume() {
    let f = two_volume_tet();
    let (bytes, _) = export(&f.mesh, &DagmcOptions::default());
    let h = read_h5m(&bytes);
    for s in &h.surfaces {
        for &t in &s.tris {
            let [a, b, c] = h.tri(t);
            let n = cross(sub(b, a), sub(c, a));
            let len = norm(n);
            let cen = [(a[0] + b[0] + c[0]) / 3.0, (a[1] + b[1] + c[1]) / 3.0, (a[2] + b[2] + c[2]) / 3.0];
            let p = [cen[0] + 1e-6 * n[0] / len, cen[1] + 1e-6 * n[1] / len, cen[2] + 1e-6 * n[2] / len];
            let fwd = BOXES[s.fwd.unwrap() as usize - 1];
            assert!(!inside(fwd, p), "normal points out of the forward volume");
            match s.bwd {
                Some(v) => assert!(inside(BOXES[v as usize - 1], p), "normal points into the backward volume"),
                None => assert!(!inside(BOXES[0], p) && !inside(BOXES[1], p), "outer normals leave the model"),
            }
        }
    }
    for (gid, b) in [(1, BOXES[0]), (2, BOXES[1])] {
        let vol = (b.1[0] - b.0[0]) * (b.1[1] - b.0[1]) * (b.1[2] - b.0[2]);
        assert!((h.volume(gid) - vol).abs() <= 1e-12 * vol, "volume {gid}: {}", h.volume(gid));
        assert!(h.volume_is_closed(gid));
    }
}

#[test]
fn mixed_triangle_senses_of_one_surface_are_normalised() {
    // One sphere triangle is given the other way round (front and back swapped, winding
    // reversed): the same physical surface. The exporter must give the surface one sense.
    let names: BTreeMap<EntityId, String> = BTreeMap::new();
    let plain = model_from_facets(&sphere_in_box(true), &names, &DagmcOptions::default()).unwrap().0;
    let mut f = sphere_in_box(true);
    for i in [5usize, 17, 30] {
        f.triangles[i].swap(1, 2);
        let (a, b) = (f.front[i], f.back[i]);
        f.front[i] = b;
        f.back[i] = a;
    }
    let (m, _) = model_from_facets(&f, &names, &DagmcOptions::default()).unwrap();
    assert_eq!(m.surfaces, plain.surfaces);
    assert!((model_volume(&m, 0) - model_volume(&plain, 0)).abs() <= 1e-12);
    // Flipping the first triangle instead flips the whole surface sense (the first triangle
    // names the forward volume) and the closed shell is unchanged.
    let mut g = sphere_in_box(true);
    g.triangles[0].swap(1, 2);
    let (a, b) = (g.front[0], g.back[0]);
    g.front[0] = b;
    g.back[0] = a;
    let (m0, _) = model_from_facets(&g, &names, &DagmcOptions::default()).unwrap();
    assert_eq!((m0.surfaces[0].forward, m0.surfaces[0].backward), (Some(1), Some(0)));
    assert!((model_volume(&m0, 0) - model_volume(&plain, 0)).abs() <= 1e-12);
    // The reversed triangles are flipped back: same triangles as a set.
    let norm_set = |m: &DagmcModel| {
        let mut v: Vec<[u32; 3]> = m.triangles.iter().map(|t| { let k = (0..3).min_by_key(|&k| t[k]).unwrap(); [t[k], t[(k + 1) % 3], t[(k + 2) % 3]] }).collect();
        v.sort_unstable();
        v
    };
    assert_eq!(norm_set(&m), norm_set(&plain));
}

#[test]
fn deleted_skin_face_is_an_open_surface() {
    let f = two_volume_tet();
    let m = &f.mesh;
    let i = (0..m.face_count()).find(|&i| m.face_right(FaceIx::new(i)).is_none()).unwrap();
    let pts: Vec<V3> = m.face_vertices(FaceIx::new(i)).iter().map(|v| m.point(v.get())).map(|p| [p.x, p.y, p.z]).collect();
    let w = Wrap::new(m).without_face(i);
    match model_from_mesh(&w, &DagmcOptions::default()) {
        Err(DagmcError::OpenSurface { edge, at, .. }) => {
            // The open edge lies on the deleted triangle.
            let on_face = (0..3).any(|k| {
                let (a, b) = (pts[k], pts[(k + 1) % 3]);
                let mid = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0];
                norm(sub(mid, at)) < 1e-12
            });
            assert!(on_face, "edge {edge:?} at {at:?} is not an edge of the deleted face");
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn quads_split_to_two_triangles_and_polygons_fan() {
    let f = hex_block_two_volumes([3, 2, 2], [0.5, 1.0, 2.0], 1).unwrap();
    let m = &f.mesh;
    let surfaced = (0..m.face_count()).filter(|&i| m.face_right(FaceIx::new(i)).is_none() || m.cell_class(m.face_left(FaceIx::new(i))).entity != m.cell_class(m.face_right(FaceIx::new(i)).unwrap()).entity).count();
    let (bytes, _) = export(m, &DagmcOptions::default());
    let h = read_h5m(&bytes);
    assert_eq!(h.tris.len(), 2 * surfaced);
    let interface = 2.0 * 1.0 * 2.0 * 2.0; // n1 n2 quads of h1 x h2
    let want = f.area + interface;
    assert!((h.total_area() - want).abs() <= 1e-12 * want, "{} vs {want}", h.total_area());
    assert!(h.volume_is_closed(1) && h.volume_is_closed(2));

    let p = hexagonal_prism(1.0, 1.0).unwrap();
    let (bytes, _) = export(&p.mesh, &DagmcOptions::default());
    let h = read_h5m(&bytes);
    assert_eq!(h.tris.len(), 2 * 6 + 6 * 2);
    assert_eq!(h.coords.len(), 12 + 2, "two hexagon centroids");
    assert!((h.total_area() - p.area).abs() <= 1e-12 * p.area);
    assert!(h.volume_is_closed(1));
    assert!((h.volume(1) - p.volume).abs() <= 1e-12 * p.volume);
}

fn bbox(pts: &[V3]) -> (V3, V3) {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in pts {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    (lo, hi)
}

#[test]
fn graveyard_boxes_and_group() {
    let f = two_volume_tet();
    let opts = DagmcOptions::with_graveyard(Graveyard::default());
    let (bytes, r) = export(&f.mesh, &opts);
    let h = read_h5m(&bytes);
    assert_eq!(r.volumes, 3);
    assert_eq!(h.surfaces.len(), 11 + 12);
    assert_eq!(h.tris.len(), 224 + 24);
    let (lo, hi) = bbox(&h.coords[..107]);
    // Inner box: the six surfaces after the model's own.
    let inner: Vec<V3> = h.surfaces[11..17].iter().flat_map(|s| s.tris.iter().flat_map(|&t| h.tri(t))).collect();
    let (ilo, ihi) = bbox(&inner);
    for k in 0..3 {
        let c = (lo[k] + hi[k]) / 2.0;
        let half = 1.1 * (hi[k] - lo[k]) / 2.0;
        assert!((ilo[k] - (c - half)).abs() <= 1e-12 && (ihi[k] - (c + half)).abs() <= 1e-12);
    }
    // No graveyard vertex lies inside the model's bounding box; outer box is the thicker one.
    let outer: Vec<V3> = h.surfaces[17..].iter().flat_map(|s| s.tris.iter().flat_map(|&t| h.tri(t))).collect();
    let (olo, ohi) = bbox(&outer);
    for k in 0..3 {
        assert!(ilo[k] < lo[k] && ihi[k] > hi[k]);
        assert!(olo[k] < ilo[k] && ohi[k] > ihi[k]);
        assert!((ohi[k] - ihi[k] - 0.05 * 1.1 * (hi[k] - lo[k]).max(1.0)).abs() <= 1e-12 * 2.0, "wall thickness is 5% of the longest inner edge");
    }
    // The group `mat:Graveyard` holds only the graveyard volume; all volumes closed.
    let (name, vols) = h.groups.iter().find(|(n, _)| n == "mat:Graveyard").unwrap();
    assert_eq!((name.as_str(), vols.as_slice()), ("mat:Graveyard", &[3][..]));
    assert_eq!(h.groups.len(), 1);
    for gid in 1..=3 {
        assert!(h.volume_is_closed(gid));
    }
    // Normals leave the graveyard: its volume by the divergence theorem is outer box minus inner box.
    let (iv, ov) = (ihi.iter().zip(&ilo).map(|(a, b)| a - b).product::<f64>(), ohi.iter().zip(&olo).map(|(a, b)| a - b).product::<f64>());
    assert!((h.volume(3) - (ov - iv)).abs() <= 1e-10 * ov);
    // A model that already has a Graveyard group gets none and says so.
    let g = with_sets(two_volume_tet(), &[SetSpec::new("Graveyard", SetMembers::Cells(cells_of(&f.mesh, 2)))]).unwrap();
    let (_, loss) = model_from_mesh(&g.mesh, &opts).unwrap();
    assert!(loss.get("h5m.graveyard_exists").is_some());
    let bad = DagmcOptions::with_graveyard(Graveyard::new(0.5, 0.05));
    assert!(matches!(model_from_mesh(&f.mesh, &bad), Err(DagmcError::Invalid(_))));
}

#[test]
fn groups_are_mat_prefixed_and_losses_are_reported() {
    let base = two_volume_tet();
    let c1 = cells_of(&base.mesh, 1);
    let c2 = cells_of(&base.mesh, 2);
    let half: Vec<CellIx> = c2.iter().copied().take(3).collect();
    let f = with_sets(
        two_volume_tet(),
        &[
            SetSpec::new("fuel", SetMembers::Cells(c1.clone())),
            SetSpec::new("part", SetMembers::Cells(half)),
            SetSpec::new("skin", SetMembers::Faces(vec![FaceIx::new(0)])),
        ],
    )
    .unwrap();
    let (bytes, r) = export(&f.mesh, &DagmcOptions::default());
    let h = read_h5m(&bytes);
    assert_eq!(h.groups, vec![("mat:fuel".to_string(), vec![1]), ("mat:part".to_string(), vec![2])]);
    assert_eq!(r.groups, 2);
    assert_eq!(r.loss.get("h5m.set_cuts_volume").unwrap().count, 1);
    assert!(r.loss.get("h5m.sets_dropped").is_some());
    assert!(r.loss.get("h5m.volume_without_material").is_none());
    // No sets at all: both volumes are reported.
    let (_, r) = export(&base.mesh, &DagmcOptions::default());
    assert_eq!(r.loss.get("h5m.volume_without_material").unwrap().count, 2);
    // A volume in two groups (the upstream reference file does this) is Info.
    let f = with_sets(two_volume_tet(), &[SetSpec::new("a", SetMembers::Cells(c1.clone())), SetSpec::new("b", SetMembers::Cells(c1))]).unwrap();
    let (_, r) = export(&f.mesh, &DagmcOptions::default());
    assert_eq!(r.loss.get("h5m.volume_in_multiple_groups").unwrap().count, 1);
    // Names never truncate: 31 bytes with the prefix fit, 32 do not.
    let long = |n: usize| with_sets(two_volume_tet(), &[SetSpec::new(&"x".repeat(n), SetMembers::Cells(cells_of(&base.mesh, 1)))]).unwrap();
    export(&long(27).mesh, &DagmcOptions::default());
    let err = export_h5m(&long(28).mesh, &DagmcOptions::default(), &mut Vec::new()).unwrap_err();
    assert!(matches!(err, DagmcError::NameTooLong { .. }), "{err:?}");
}

#[test]
fn faceting_tol_is_a_tag_on_a_file_set() {
    let f = two_volume_tet();
    let o = DagmcOptions::default().with_faceting_tol(1e-3);
    let (bytes, _) = export(&f.mesh, &o);
    let g = read_file(&bytes).unwrap();
    assert_eq!(g.dataset("tstt/tags/FACETING_TOL/values").unwrap().as_f64(), vec![1e-3]);
    let list = g.dataset("tstt/sets/list").unwrap();
    let rows = list.as_i64();
    assert_eq!(rows.len(), 4 * 14);
    assert_eq!(&rows[..4], &[1, -1, -1, 8], "the file set holds one range");
    let h = read_h5m(&bytes);
    assert_eq!(h.surfaces.len(), 11);
    let bad = DagmcOptions::default().with_faceting_tol(-1.0);
    assert!(matches!(export_h5m(&f.mesh, &bad, &mut Vec::new()), Err(DagmcError::Invalid(_))));
}

fn ent(kind: EntityKind, n: u64) -> EntityId {
    EntityId::new(kind, n).unwrap()
}

fn vol(n: u64) -> Region {
    Region::Volume(ent(EntityKind::TopoVolume, n))
}

/// A closed shell: `front` inside (normals point out of it), `back` outside.
fn add_shell(f: &mut FacetInput, pts: &[V3], tris: &[[u32; 3]], front: Region, back: Region, face: u64) {
    let base = f.vertices.len() as u32;
    f.vertices.extend_from_slice(pts);
    for t in tris {
        f.triangles.push([t[0] + base, t[1] + base, t[2] + base]);
        f.front.push(front);
        f.back.push(back);
        f.face.push(ent(EntityKind::TopoFace, face));
    }
}

/// The cube `[-3,3]^3`: 6 faces of two triangles, normals out; one `TopoFace` per cube face.
fn cube_facets(f: &mut FacetInput, outward_front: bool) {
    let h = 3.0;
    let corner = |c: usize| [if c & 1 == 1 { h } else { -h }, if c & 2 == 2 { h } else { -h }, if c & 4 == 4 { h } else { -h }];
    let pts: Vec<V3> = (0..8).map(corner).collect();
    let base = f.vertices.len() as u32;
    f.vertices.extend_from_slice(&pts);
    let quads: [[u32; 4]; 6] = [[0, 2, 6, 4], [1, 3, 7, 5], [0, 1, 5, 4], [2, 3, 7, 6], [0, 1, 3, 2], [4, 5, 7, 6]];
    for (i, q) in quads.iter().enumerate() {
        let mut tris = [[q[0], q[1], q[2]], [q[0], q[2], q[3]]];
        let n = cross(sub(pts[q[1] as usize], pts[q[0] as usize]), sub(pts[q[2] as usize], pts[q[0] as usize]));
        let want = if i % 2 == 1 { 1.0 } else { -1.0 };
        if n[i / 2] * want < 0.0 {
            tris.iter_mut().for_each(|t| t.swap(1, 2));
        }
        for mut t in tris {
            // Inward orientation (front exterior) reverses the winding.
            if !outward_front {
                t.swap(1, 2);
            }
            f.triangles.push([t[0] + base, t[1] + base, t[2] + base]);
            let (fr, bk) = if outward_front { (vol(2), Region::Exterior) } else { (Region::Exterior, vol(2)) };
            f.front.push(fr);
            f.back.push(bk);
            f.face.push(ent(EntityKind::TopoFace, 2 + i as u64));
        }
    }
}

fn sphere_in_box(outward_front: bool) -> FacetInput {
    let s = icosphere(1.0, 2);
    let mut f = FacetInput::default();
    add_shell(&mut f, &s.points, &s.oriented_tris(0), vol(1), vol(2), 1);
    cube_facets(&mut f, outward_front);
    f
}

fn model_volume(m: &DagmcModel, vol: usize) -> f64 {
    let mut v = 0.0;
    for (ti, t) in m.triangles.iter().enumerate() {
        let s = &m.surfaces[m.tri_surface[ti] as usize];
        let sign = if s.forward == Some(vol as u32) { 1.0 } else if s.backward == Some(vol as u32) { -1.0 } else { continue };
        let [a, b, c] = t.map(|i| m.vertices[i as usize]);
        v += sign * dot(a, cross(b, c)) / 6.0;
    }
    v
}

#[test]
fn facets_give_closed_volumes_and_senses_from_front_back() {
    let sphere = icosphere(1.0, 2);
    let names: BTreeMap<EntityId, String> = [(ent(EntityKind::TopoVolume, 1), "ball".to_string()), (ent(EntityKind::TopoVolume, 2), "water".to_string())].into();
    for outward in [true, false] {
        let f = sphere_in_box(outward);
        let (m, loss) = model_from_facets(&f, &names, &DagmcOptions::default()).unwrap();
        assert_eq!(m.volumes.len(), 2);
        assert_eq!(m.surfaces.len(), 7);
        assert!(loss.get("h5m.volume_without_material").is_none());
        // Sphere surface (id 1): forward ball (index 0), backward water; cube faces: [water, none].
        assert_eq!((m.surfaces[0].forward, m.surfaces[0].backward), (Some(0), Some(1)));
        assert!(m.surfaces[1..].iter().all(|s| s.forward == Some(1) && s.backward.is_none()));
        let ball = model_volume(&m, 0);
        assert!((ball - sphere.volumes[0].enclosed_volume).abs() <= 1e-12 * ball, "{ball}");
        assert!((model_volume(&m, 1) - (216.0 - sphere.volumes[0].enclosed_volume)).abs() <= 1e-10);
        assert_eq!(m.groups.iter().map(|g| (g.name.as_str(), g.volumes.clone())).collect::<Vec<_>>(), vec![("ball", vec![0]), ("water", vec![1])]);
        let mut out = Vec::new();
        write_h5m(&m, &DagmcOptions::default(), &mut out).unwrap();
        let h = read_h5m(&out);
        assert!(h.volume_is_closed(1) && h.volume_is_closed(2));
        assert_eq!(h.groups.iter().map(|g| g.0.as_str()).collect::<Vec<_>>(), vec!["mat:ball", "mat:water"]);
    }
    // A missing triangle leaves an open volume.
    let mut f = sphere_in_box(true);
    let n = f.triangles.len() - 1;
    f.triangles.truncate(n);
    f.front.truncate(n);
    f.back.truncate(n);
    f.face.truncate(n);
    assert!(matches!(model_from_facets(&f, &names, &DagmcOptions::default()), Err(DagmcError::OpenSurface { .. })));
    // A name for a volume without facets, and inconsistent triangles of one surface.
    let mut bad = names.clone();
    bad.insert(ent(EntityKind::TopoVolume, 9), "ghost".into());
    assert!(matches!(model_from_facets(&sphere_in_box(true), &bad, &DagmcOptions::default()), Err(DagmcError::Invalid(_))));
    let mut f = sphere_in_box(true);
    f.back[0] = vol(2);
    f.front[0] = vol(3);
    assert!(model_from_facets(&f, &names, &DagmcOptions::default()).is_err());
}

#[test]
fn partition_and_two_dimensional_inputs_are_typed_errors() {
    let f = two_volume_tet();
    let w = Wrap::new(&f.mesh).dim2();
    assert!(matches!(model_from_mesh(&w, &DagmcOptions::default()), Err(DagmcError::Unsupported(_))));
    let w = Wrap::new(&f.mesh).inactive(&[0]);
    assert!(matches!(model_from_mesh(&w, &DagmcOptions::default()), Err(DagmcError::Unsupported(_))));
    let mut m = DagmcModel::default();
    assert!(matches!(write_h5m(&m, &DagmcOptions::default(), &mut Vec::new()), Err(DagmcError::Invalid(_))));
    let (good, _) = model_from_mesh(&f.mesh, &DagmcOptions::default()).unwrap();
    m = good.clone();
    m.triangles[0][0] = 1 << 30;
    assert!(matches!(write_h5m(&m, &DagmcOptions::default(), &mut Vec::new()), Err(DagmcError::Invalid(_))));
    m = good;
    m.groups.push(DagmcGroup { name: "x\0y".into(), volumes: vec![0] });
    assert!(matches!(write_h5m(&m, &DagmcOptions::default(), &mut Vec::new()), Err(DagmcError::Invalid(_))));
}

fn in_pool<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(f)
}

#[test]
fn bytes_are_identical_across_runs_and_thread_counts() {
    let f = with_sets(two_volume_tet(), &[SetSpec::new("fuel", SetMembers::Cells(cells_of(&two_volume_tet().mesh, 1)))]).unwrap();
    let o = DagmcOptions::with_graveyard(Graveyard::default()).with_faceting_tol(1e-3);
    sm_testkit::determinism::check(2, 4, |t| in_pool(t, || export(&f.mesh, &o).1.hash)).unwrap();
    let a = export(&f.mesh, &o).0;
    assert_eq!(a, export(&f.mesh, &o).0);
    let h = hexagonal_prism(1.0, 2.0).unwrap();
    sm_testkit::determinism::check(2, 4, |t| in_pool(t, || export(&h.mesh, &DagmcOptions::default()).1.hash)).unwrap();
    // No timestamps: the file contains neither OHDR timestamp flag bits nor a history dataset.
    let g = read_file(&a).unwrap();
    assert!(g.find("tstt/history").is_none());
}

proptest! {
    #![proptest_config(strategies::config(16))]

    #[test]
    fn random_blocks_export_closed_volumes_with_exact_area(n0 in 2u32..5, n1 in 1u32..4, n2 in 1u32..4, split in 1u32..4, h in 0.1f64..2.0) {
        let split = 1 + split % (n0 - 1);
        let hs = [h, h * 0.75, h * 1.5];
        for fixture in [tet_block_two_volumes([n0, n1, n2], hs, split).unwrap(), hex_block_two_volumes([n0, n1, n2], hs, split).unwrap()] {
            let (bytes, _) = export(&fixture.mesh, &DagmcOptions::default());
            let hm = read_h5m(&bytes);
            prop_assert!(hm.volume_is_closed(1) && hm.volume_is_closed(2));
            let interface = n1 as f64 * hs[1] * n2 as f64 * hs[2];
            let want = fixture.area + interface;
            prop_assert!((hm.total_area() - want).abs() <= 1e-12 * want);
            prop_assert!((hm.volume(1) + hm.volume(2) - fixture.volume).abs() <= 1e-11 * fixture.volume);
        }
    }
}
