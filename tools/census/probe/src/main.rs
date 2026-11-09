//! Prints `group|key|os-arch-or-any|value` lines for values that no test prints.
//!
//! usage: census-probe OS_ARCH OPENMC_FIXTURES_DIR
use sm_core::{EntityId, EntityKind};
use sm_csgio::{ImportOptions, ImportedCsg, openmc};
use sm_mesh::MeshBuilder;
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{Classification, MeshRead, SerialAlloc, geometry_hash, topology_hash};
use sm_meshio::dagmc::{DagmcOptions, Graveyard, export_h5m};
use sm_poly::{DualReport, EdgeResolver, FaceMode, PolyOptions, VertexClassResolver, dualize};
use sm_testkit::meshes;
use std::path::PathBuf;

/// Edge ids from the face pair, for the single-tet fixture.
struct PairResolver;
impl EdgeResolver for PairResolver {
    fn resolve(&self, a: EntityId, b: EntityId, _e: [Classification; 2]) -> Option<EntityId> {
        EntityId::new(EntityKind::TopoEdge, a.serial() * 1000 + b.serial())
    }
}

fn row(group: &str, key: &str, plat: &str, value: impl std::fmt::Display) {
    println!("{group}|{key}|{plat}|{value}");
}

fn mode_name(m: FaceMode) -> &'static str {
    match m {
        FaceMode::Planar => "planar",
        _ => "keep",
    }
}

fn poly(os: &str) {
    // The 3x3x3 tet block, Keep mode, reference builder: counts of the 375-face dual.
    let f = meshes::tet_block([3, 3, 3], [1.0; 3]).expect("tet_block");
    let o = PolyOptions::new().with_faces(FaceMode::Keep);
    let mut a = SerialAlloc::new();
    let (b, rep): (_, DualReport) = dualize(&f.mesh, &o, &VertexClassResolver, ReferenceBuilder::new(), &mut a).expect("dual");
    let m = b.mesh;
    row("poly", "block333.keep.cells", "any", m.cell_count());
    row("poly", "block333.keep.faces", "any", m.face_count());
    row("poly", "block333.keep.points", "any", rep.points);
    row("poly", "block333.keep.edge_faces", "any", rep.edge_faces);
    row("poly", "block333.keep.boundary_faces", "any", rep.boundary_faces);
    // The determinism fixtures: production builder on a 1-thread pool.
    let fixtures = [
        meshes::tet_block([4, 3, 3], [1.0, 0.7, 1.3]).expect("f"),
        meshes::tet_block_two_volumes([4, 2, 2], [1.0; 3], 2).expect("f"),
        meshes::tet_block_warped([3, 3, 3], [1.0; 3], 0.1).expect("f"),
        meshes::tet_lshape(2, 1.0).expect("f"),
    ];
    let pool = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool");
    for fx in &fixtures {
        for mode in [FaceMode::Planar, FaceMode::Keep] {
            let (t, g) = pool.install(|| {
                let o = PolyOptions::new().with_faces(mode).with_edges(Some(true));
                let mut a = SerialAlloc::new();
                let (b, _) = dualize(&fx.mesh, &o, &VertexClassResolver, MeshBuilder::new(), &mut a).expect("dual");
                (topology_hash(&b.mesh), geometry_hash(&b.mesh))
            });
            let k = format!("acc7.{}.{}", fx.name, mode_name(mode));
            row("poly", &format!("{k}.topology_hash"), "any", format_args!("{t:016x}"));
            row("poly", &format!("{k}.geometry_hash"), os, format_args!("{g:016x}"));
        }
    }
    let one = meshes::single_tet().expect("tet");
    let mut a = SerialAlloc::new();
    let (b, _) = dualize(&one.mesh, &PolyOptions::new(), &PairResolver, ReferenceBuilder::new(), &mut a).expect("dual");
    row("poly", "single_tet.faces", "any", b.mesh.face_count());
}

fn dagmc(os: &str) {
    let grave = DagmcOptions::with_graveyard(Graveyard::default());
    let cases = [
        ("tet_two_volumes", meshes::tet_block_two_volumes([4, 4, 4], [0.25; 3], 2).expect("f")),
        ("hex_two_volumes", meshes::hex_block_two_volumes([3, 2, 2], [0.5, 1.0, 2.0], 1).expect("f")),
        ("hexagonal_prism", meshes::hexagonal_prism(1.0, 1.0).expect("f")),
    ];
    for (name, f) in &cases {
        for (tag, o) in [("default", DagmcOptions::default()), ("graveyard", grave.clone())] {
            let mut out = Vec::new();
            let r = export_h5m(&f.mesh, &o, &mut out).expect("export");
            let k = format!("{name}.{tag}");
            row("dagmc", &format!("{k}.counts"), "any", format_args!("tris={} surfaces={} volumes={} groups={}", r.triangles, r.surfaces, r.volumes, r.groups));
            row("dagmc", &format!("{k}.bytes"), os, r.bytes);
            row("dagmc", &format!("{k}.fnv1a64"), os, format_args!("{:016x}", r.hash));
        }
    }
}

fn csgio(os: &str, dir: &str) {
    let mut names: Vec<String> = std::fs::read_dir(dir)
        .expect("fixtures dir")
        .map(|e| e.expect("entry"))
        .filter(|e| e.path().is_dir())
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    for n in names {
        let p = PathBuf::from(dir).join(&n);
        let g = std::fs::read_to_string(p.join("geometry.xml")).expect("geometry.xml");
        let m = std::fs::read_to_string(p.join("materials.xml")).expect("materials.xml");
        // A deck the importer refuses is itself a measured value: the error text.
        let hash = openmc::import(&g, Some(&m))
            .map_err(|e| e.to_string())
            .and_then(|ir| ImportedCsg::instantiate(&ir, &ImportOptions::default()).map_err(|e| e.to_string()))
            .map(|c| format!("{:032x}", c.content_hash().0));
        match hash {
            Ok(h) => row("csgio", &format!("content_hash.{n}"), os, h),
            Err(e) => row("csgio", &format!("content_hash.{n}"), "any", format_args!("error: {e}")),
        }
    }
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let (os, dir) = (a.get(1).expect("OS_ARCH"), a.get(2).expect("fixtures dir"));
    poly(os);
    dagmc(os);
    csgio(os, dir);
}
