//! The benchmark cases and their deterministic generator.
//!
//! | Case | What |
//! |---|---|
//! | `C1`, `C2`, `C3` | W3a legacy (L1): icosphere shell, nested icospheres, box with void; bytes equal the W3a files |
//! | `P1` | pin cell: fuel r 0.4096, clad r 0.475, square pitch 1.26, height 100; water, vacuum outside |
//! | `P2` | `pin_assembly(n, 1.26, 0.4096, 0.475, 100)`: `9 n^2` pins (n = 17: 2601 copies of one prototype); facet twin is flat |
//! | `H1` | 200 cm water cube with 8 inclusions (4 boxes, 4 spheres) and a Kuhn tet mesh |
//! | `F1` | synthetic fusion-like nested torus shells (R 600) |
//!
//! Every curved case carries a facet twin from `sm_tess` at the recorded `sag`.

use crate::error::{BenchError, Result};
use crate::formats::{Facets, Manifest, Rays, RegionSpec, Tets, bbox_of, platform_tag};
use crate::legacy::{self, Legacy};
use crate::rng::{SplitMix64, sha256_hex};
use crate::truth::{Scene, Shape, Solid};
use crate::twin;
use std::collections::BTreeMap;
use std::path::Path;

/// A benchmark case.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Case {
    /// Legacy C1 (L1).
    C1,
    /// Legacy C2 (L1).
    C2,
    /// Legacy C3 (L1).
    C3,
    /// Pin cell.
    P1,
    /// Pin assembly.
    P2,
    /// Homogeneous block with inclusions and tets.
    H1,
    /// Synthetic fusion-like torus shells.
    F1,
}

impl Case {
    /// All cases in report order.
    pub const ALL: [Case; 7] = [Case::C1, Case::C2, Case::C3, Case::P1, Case::P2, Case::H1, Case::F1];

    /// Case name (also the ray seed).
    pub fn name(self) -> &'static str {
        match self {
            Case::C1 => "C1",
            Case::C2 => "C2",
            Case::C3 => "C3",
            Case::P1 => "P1",
            Case::P2 => "P2",
            Case::H1 => "H1",
            Case::F1 => "F1",
        }
    }

    /// Parse a name.
    pub fn parse(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|c| c.name() == s)
    }

    fn legacy(self) -> Option<Legacy> {
        match self {
            Case::C1 => Some(Legacy::C1),
            Case::C2 => Some(Legacy::C2),
            Case::C3 => Some(Legacy::C3),
            _ => None,
        }
    }
}

/// Size knobs of a generated case set. `full` is the benchmark size; `reduced` and `tiny` keep
/// the same geometry families small enough for tests and smoke runs.
#[derive(Clone, Debug, PartialEq)]
pub struct Scale {
    /// Name written to the manifest and used in `expected_sha256.txt`.
    pub name: &'static str,
    /// Rays per case (F1 uses `f1_rays`).
    pub rays: usize,
    /// Rays for F1.
    pub f1_rays: usize,
    /// Facet sag of the pin cases, cm.
    pub sag_pin: f64,
    /// Pins per lattice side of P2 (`pin_assembly(n, ..)`).
    pub lattice_n: usize,
    /// Cell edge of the H1 tet mesh, cm (the box is 200 cm).
    pub h1_cell: f64,
    /// Facet sag of F1, cm.
    pub sag_f1: f64,
    /// Number of torus shells of F1.
    pub f1_shells: usize,
}

impl Scale {
    /// Benchmark size (P2 has about 4e6 triangles).
    pub fn full() -> Self {
        Self { name: "full", rays: 200_000, f1_rays: 10_000, sag_pin: 1e-4, lattice_n: 17, h1_cell: 4.0, sag_f1: 0.05, f1_shells: 18 }
    }
    /// Smoke size.
    pub fn reduced() -> Self {
        Self { name: "reduced", rays: 10_000, f1_rays: 10_000, sag_pin: 1e-3, lattice_n: 2, h1_cell: 20.0, sag_f1: 0.5, f1_shells: 18 }
    }
    /// Unit-test size.
    pub fn tiny() -> Self {
        Self { name: "tiny", rays: 200, f1_rays: 200, sag_pin: 1e-2, lattice_n: 1, h1_cell: 25.0, sag_f1: 1.0, f1_shells: 6 }
    }
    /// Parse `full`, `reduced` or `tiny`.
    pub fn parse(s: &str) -> Option<Self> {
        [Self::full(), Self::reduced(), Self::tiny()].into_iter().find(|x| x.name == s)
    }
}

/// A generated case in memory.
#[derive(Clone, Debug)]
pub struct CaseData {
    /// Which case.
    pub case: String,
    /// Facet twin.
    pub facets: Facets,
    /// Seeded rays.
    pub rays: Rays,
    /// Tet mesh (H1 only).
    pub tets: Option<Tets>,
    /// Analytic truth, when the case has one the scene can express.
    pub scene: Option<Scene>,
    /// The manifest (hashes of the three files included).
    pub manifest: Manifest,
}

impl CaseData {
    /// The files of the case: `(name, bytes)` in a fixed order, `smcase.json` last.
    pub fn files(&self) -> Result<Vec<(String, Vec<u8>)>> {
        let mut v = vec![("facets.bin".to_string(), self.facets.to_bytes()), ("rays.bin".to_string(), self.rays.to_bytes())];
        if let Some(t) = &self.tets {
            v.push(("tets.bin".to_string(), t.to_bytes()));
        }
        let mut json = serde_json::to_vec_pretty(&self.manifest)?;
        json.push(b'\n');
        v.push(("smcase.json".to_string(), json));
        Ok(v)
    }

    /// Write the case into `<dir>/<case>/`.
    pub fn write(&self, dir: &Path) -> Result<()> {
        let d = dir.join(&self.case);
        std::fs::create_dir_all(&d).map_err(|e| BenchError::io(&d, e))?;
        for (name, bytes) in self.files()? {
            let p = d.join(name);
            std::fs::write(&p, bytes).map_err(|e| BenchError::io(&p, e))?;
        }
        Ok(())
    }

    /// The regions of the manifest by id.
    pub fn region(&self, id: i32) -> Option<&RegionSpec> {
        self.manifest.regions.iter().find(|r| r.id == id)
    }
}

/// Agreement tolerance for chords: `max(1e-9 diag, 8 sag)`. A ray near grazing sees a larger
/// error; the checker treats those through its grazing rule, not through this constant.
pub fn chord_tolerance(diag: f64, sag: f64) -> f64 {
    (1e-9 * diag).max(8.0 * sag)
}

/// Workload constants of region `k`: the benchmark's own transport problem (never sm-rt input).
fn workload_region(id: i32, name: &str) -> RegionSpec {
    let sigma_t = 0.5 + 0.25 * f64::from(id.rem_euclid(4));
    RegionSpec { id, name: name.to_string(), sigma_t, sigma_a: 0.3 * sigma_t }
}

fn regions(names: &[&str]) -> Vec<RegionSpec> {
    names.iter().enumerate().map(|(i, n)| workload_region(i as i32, n)).collect()
}

/// Generate rays: uniform starts in `bbox`, isotropic directions, `classify` gives the start
/// region or `None` to reject. Seeded from the case name only.
pub fn gen_rays(name: &str, bbox: ([f64; 3], [f64; 3]), n: usize, classify: &dyn Fn([f64; 3]) -> Option<i32>) -> Result<Rays> {
    if n > u32::MAX as usize {
        return Err(BenchError::Usage("too many rays".into()));
    }
    let mut rng = SplitMix64::from_name(name);
    let (lo, hi) = bbox;
    let mut out = Rays::default();
    let mut tries = 0u64;
    while out.len() < n {
        tries += 1;
        if tries > 1000 * (n as u64 + 10) {
            return Err(BenchError::Geometry(format!("{name}: start rejection rate too high")));
        }
        let mut p = [0.0; 3];
        for a in 0..3 {
            p[a] = lo[a] + (hi[a] - lo[a]) * rng.unit();
        }
        let d = rng.direction();
        if let Some(r) = classify(p) {
            out.push(p, d, r);
        }
    }
    Ok(out)
}

fn diag(lo: [f64; 3], hi: [f64; 3]) -> f64 {
    ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt()
}

#[allow(clippy::too_many_arguments)]
fn finish(
    case: &str,
    scale: &str,
    facets: Facets,
    rays: Rays,
    tets: Option<Tets>,
    scene: Option<Scene>,
    regions: Vec<RegionSpec>,
    sag: f64,
    prototypes: usize,
    copies: usize,
    notes: Vec<String>,
) -> Result<CaseData> {
    let (lo, hi) = facets.bbox().ok_or_else(|| BenchError::Geometry("empty facet twin".into()))?;
    let d = diag(lo, hi);
    let mut files = BTreeMap::new();
    files.insert("facets.bin".to_string(), sha256_hex(&facets.to_bytes()));
    files.insert("rays.bin".to_string(), sha256_hex(&rays.to_bytes()));
    if let Some(t) = &tets {
        files.insert("tets.bin".to_string(), sha256_hex(&t.to_bytes()));
    }
    let manifest = Manifest {
        schema: "smcase v1".into(),
        case: case.into(),
        scale: scale.into(),
        regions,
        sag,
        chord_tol: chord_tolerance(d, sag),
        bbox_lo: lo,
        bbox_hi: hi,
        diag: d,
        vertices: facets.verts.len(),
        triangles: facets.tris.len(),
        tets: tets.as_ref().map_or(0, |t| t.tets.len()),
        rays: rays.len(),
        prototypes,
        copies,
        platform: platform_tag(),
        notes,
        files,
    };
    Ok(CaseData { case: case.into(), facets, rays, tets, scene, manifest })
}

/// Build a case at a scale.
pub fn build(case: Case, scale: &Scale) -> Result<CaseData> {
    match case {
        Case::C1 | Case::C2 | Case::C3 => build_legacy(case, scale),
        Case::P1 => build_p1(scale),
        Case::P2 => build_p2(scale),
        Case::H1 => build_h1(scale),
        Case::F1 => build_f1(scale),
    }
}

fn build_legacy(case: Case, scale: &Scale) -> Result<CaseData> {
    let lc = case.legacy().ok_or_else(|| BenchError::Usage("not a legacy case".into()))?;
    let g = legacy::build(lc)?;
    let rays = legacy::rays(lc, &g, scale.rays)?;
    let h = legacy::HALF;
    let mut solids = vec![];
    let names: Vec<&str>;
    let mut notes = vec!["W3a legacy case regenerated natively; bytes equal sm-rt-capi/bench/gen_case.py output".to_string()];
    match lc {
        Legacy::C1 => {
            solids.push(Solid { shape: Shape::Sphere { c: [0.0; 3], r: 1.0 }, region: 0, depth: 1 });
            solids.push(Solid { shape: Shape::Box { lo: [-h; 3], hi: [h; 3] }, region: 1, depth: 0 });
            names = vec!["ball", "shell"];
        }
        Legacy::C2 => {
            for (k, r) in g.radii.iter().enumerate() {
                solids.push(Solid { shape: Shape::Sphere { c: [0.0; 3], r: *r }, region: k as i32, depth: (g.radii.len() - k) as u32 });
            }
            solids.push(Solid { shape: Shape::Box { lo: [-h; 3], hi: [h; 3] }, region: 5, depth: 0 });
            names = vec!["s0", "s1", "s2", "s3", "s4", "box"];
        }
        Legacy::C3 => {
            solids.push(Solid { shape: Shape::Box { lo: [-h; 3], hi: [h; 3] }, region: 0, depth: 0 });
            solids.push(Solid { shape: Shape::Sphere { c: [0.0; 3], r: 1.0 }, region: -1, depth: 1 });
            names = vec!["solid"];
            notes.push("the spherical void is the implicit complement".into());
        }
    }
    let sag = g.radii.iter().zip(&g.inradii).map(|(r, i)| r - i).fold(0.0, f64::max);
    finish(case.name(), scale.name, g.facets, rays, None, Some(Scene::new(solids)), regions(&names), sag, 1, 1, notes)
}

/// Pin cell geometry constants (cm).
/// Fuel radius of the pin cases, cm.
pub const R_FUEL: f64 = 0.4096;
/// Cladding outer radius of the pin cases, cm.
pub const R_CLAD: f64 = 0.475;
/// Square pin pitch, cm.
pub const PITCH: f64 = 1.26;
/// Pin height, cm.
pub const PIN_H: f64 = 100.0;

fn pin_solids(centers: &[[f64; 2]], with_water: bool) -> Vec<Solid> {
    let mut s = Vec::new();
    if with_water {
        s.push(Solid { shape: Shape::Box { lo: [-0.5 * PITCH, -0.5 * PITCH, 0.0], hi: [0.5 * PITCH, 0.5 * PITCH, PIN_H] }, region: 2, depth: 0 });
    }
    for c in centers {
        s.push(Solid { shape: Shape::Cyl { c: *c, r: R_CLAD, z0: 0.0, z1: PIN_H }, region: 1, depth: 1 });
        s.push(Solid { shape: Shape::Cyl { c: *c, r: R_FUEL, z0: 0.0, z1: PIN_H }, region: 0, depth: 2 });
    }
    s
}

/// Band rule for curved surfaces: `(inner, outer)` widths around each curved solid inside which
/// the faceted and the analytic labels may differ. Starts in the band are rejected.
fn in_curved_band(solids: &[Solid], bands: &[(f64, f64)], p: [f64; 3]) -> bool {
    solids.iter().zip(bands).any(|(s, &(bi, bo))| {
        let signed = match s.shape {
            Shape::Cyl { c, r, z0, z1 } => {
                if p[2] < z0 - bo || p[2] > z1 + bo {
                    return false;
                }
                ((p[0] - c[0]).powi(2) + (p[1] - c[1]).powi(2)).sqrt() - r
            }
            Shape::Sphere { c, r } => ((p[0] - c[0]).powi(2) + (p[1] - c[1]).powi(2) + (p[2] - c[2]).powi(2)).sqrt() - r,
            Shape::Box { .. } => return false,
        };
        -bi < signed && signed <= bo
    })
}

fn scene_rays(name: &str, facets: &Facets, scene: &Scene, bands: &[(f64, f64)], n: usize) -> Result<Rays> {
    let bb = facets.bbox().ok_or_else(|| BenchError::Geometry("empty facets".into()))?;
    gen_rays(name, bb, n, &|p| {
        let r = scene.region_at(p);
        (r >= 0 && !in_curved_band(&scene.solids, bands, p)).then_some(r)
    })
}

fn build_p1(scale: &Scale) -> Result<CaseData> {
    let fix = sm_testkit::topo::pin_assembly(1, PITCH, R_FUEL, R_CLAD, PIN_H).map_err(|e| BenchError::Geometry(e.to_string()))?;
    let mut tw = twin::pin_twin(&fix, scale.sag_pin, R_CLAD, PIN_H)?;
    twin::add_moderator(&mut tw, 0.5 * PITCH, PIN_H, 2)?;
    let facets = tw.facets;
    let scene = Scene::new(pin_solids(&[[0.0, 0.0]], true));
    let bands: Vec<(f64, f64)> = scene.solids.iter().map(|s| if matches!(s.shape, Shape::Cyl { .. }) { (scale.sag_pin, 0.0) } else { (0.0, 0.0) }).collect();
    let rays = scene_rays("P1", &facets, &scene, &bands, scale.rays)?;
    let notes = vec![
        "gap merged: fuel, cladding, water; vacuum outside; no reflective faces (DAGMC has none)".into(),
        "the moderator is a hand-built planar strip triangulation around the cladding rings".into(),
    ];
    finish("P1", scale.name, facets, rays, None, Some(scene), regions(&["fuel", "clad", "water"]), scale.sag_pin, 1, 1, notes)
}

fn build_p2(scale: &Scale) -> Result<CaseData> {
    let n = scale.lattice_n;
    let fix = sm_testkit::topo::pin_assembly(n, PITCH, R_FUEL, R_CLAD, PIN_H).map_err(|e| BenchError::Geometry(e.to_string()))?;
    let tw = twin::pin_twin(&fix, scale.sag_pin, R_CLAD, PIN_H)?;
    let mut base = tw.facets;
    // In P2 the outer cladding borders the complement (pins in vacuum, as in the fixture).
    for s in &mut base.sides {
        if *s == [1, 2] {
            *s = [1, -1];
        }
    }
    let flat = sm_model_api::flatten(&fix.model).map_err(|e| BenchError::Geometry(e.to_string()))?;
    let facets = twin::replicate(&base, &flat.placements)?;
    let centers: Vec<[f64; 2]> = flat
        .placements
        .iter()
        .map(|p| {
            let c = p.transform.apply_point(sm_core::Point3::new(0.0, 0.0, 0.0));
            [c.x, c.y]
        })
        .collect();
    let scene = Scene::new(pin_solids(&centers, false));
    let bands: Vec<(f64, f64)> = scene.solids.iter().map(|_| (scale.sag_pin, 0.0)).collect();
    let rays = scene_rays("P2", &facets, &scene, &bands, scale.rays)?;
    let notes = vec![
        format!("pin_assembly({n}, ..) is a 3x3 cluster of {n}x{n} lattices: {} copies of one pin prototype, no guide tubes and no plates", flat.paths.len()),
        "triangle count is exactly copies * triangles of one pin".into(),
        "pins stand in vacuum (fuel and cladding regions, no moderator)".into(),
    ];
    let copies = flat.paths.len();
    finish("P2", scale.name, facets, rays, None, Some(scene), regions(&["fuel", "clad"]), scale.sag_pin, 1, copies, notes)
}

const H1_HALF: f64 = 100.0;

struct H1Inclusion {
    shape: Shape,
}

/// The eight inclusion blocks: grid-aligned boxes (tet-conforming) and spheres (stair-stepped in
/// the tet mesh) on the octant corners of the cube.
fn h1_inclusions(n: usize, h: f64) -> Vec<H1Inclusion> {
    let s = (n / 10).max(1);
    let lo_ix = n / 4;
    let hi_ix = 3 * n / 4 - s;
    let at = |bit: usize| if bit == 0 { lo_ix } else { hi_ix };
    let coord = |ix: usize| -H1_HALF + h * ix as f64;
    (0..8usize)
        .map(|m| {
            let (bx, by, bz) = (m & 1, (m >> 1) & 1, (m >> 2) & 1);
            let (ix, iy, iz) = (at(bx), at(by), at(bz));
            let lo = [coord(ix), coord(iy), coord(iz)];
            let size = h * s as f64;
            if (bx ^ by ^ bz) == 1 {
                H1Inclusion { shape: Shape::Box { lo, hi: [lo[0] + size, lo[1] + size, lo[2] + size] } }
            } else {
                H1Inclusion { shape: Shape::Sphere { c: [lo[0] + 0.5 * size, lo[1] + 0.5 * size, lo[2] + 0.5 * size], r: 0.45 * size } }
            }
        })
        .collect()
}

/// Kuhn subdivision of the cube `[-half, half]^3` into `n^3` cells of 6 tets each.
fn kuhn_tets(n: usize, h: f64, region_of: &dyn Fn([f64; 3]) -> i32) -> Result<Tets> {
    let np = n + 1;
    let node = |i: usize, j: usize, k: usize| (i + np * (j + np * k)) as u32;
    let mut t = Tets::default();
    for k in 0..np {
        for j in 0..np {
            for i in 0..np {
                t.nodes.push([-H1_HALF + h * i as f64, -H1_HALF + h * j as f64, -H1_HALF + h * k as f64]);
            }
        }
    }
    if t.nodes.len() > u32::MAX as usize {
        return Err(BenchError::Geometry("tet mesh too large".into()));
    }
    // Six tets around the main diagonal 0 -> 7, one per axis permutation.
    const PERMS: [[usize; 3]; 6] = [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]];
    for k in 0..n {
        for j in 0..n {
            for i in 0..n {
                for (pi, perm) in PERMS.iter().enumerate() {
                    let mut c = [i, j, k];
                    let mut ids = [node(c[0], c[1], c[2]); 4];
                    for (s, &ax) in perm.iter().enumerate() {
                        c[ax] += 1;
                        ids[s + 1] = node(c[0], c[1], c[2]);
                    }
                    // Even permutations are positively oriented; swap two nodes for the others.
                    if pi == 1 || pi == 2 || pi == 5 {
                        ids.swap(2, 3);
                    }
                    let cen: [f64; 3] = core::array::from_fn(|a| ids.iter().map(|&v| t.nodes[v as usize][a]).sum::<f64>() / 4.0);
                    t.tets.push(ids);
                    t.region.push(region_of(cen));
                }
            }
        }
    }
    Ok(t)
}

fn build_h1(scale: &Scale) -> Result<CaseData> {
    let n = (2.0 * H1_HALF / scale.h1_cell).round() as usize;
    if n < 4 {
        return Err(BenchError::Usage("h1_cell too large (need at least 4 cells per side)".into()));
    }
    let h = 2.0 * H1_HALF / n as f64;
    let inc = h1_inclusions(n, h);
    let water = inc.len() as i32; // region 8
    let mut solids = vec![Solid { shape: Shape::Box { lo: [-H1_HALF; 3], hi: [H1_HALF; 3] }, region: water, depth: 0 }];
    for (k, i) in inc.iter().enumerate() {
        solids.push(Solid { shape: i.shape, region: k as i32, depth: 1 });
    }
    let scene = Scene::new(solids);
    // Facet twin: cube, boxes and icospheres.
    let mut facets = Facets::default();
    let (v, t) = legacy::boxed([-H1_HALF; 3], [H1_HALF; 3]);
    facets.add(&v, &t, water, -1, false)?;
    let mut sag = 0.0f64;
    for (k, i) in inc.iter().enumerate() {
        match i.shape {
            Shape::Box { lo, hi } => {
                let (v, t) = legacy::boxed(lo, hi);
                facets.add(&v, &t, k as i32, water, false)?;
            }
            Shape::Sphere { c, r } => {
                let (v, t) = legacy::icosphere(r, 3);
                let v: Vec<[f64; 3]> = v.iter().map(|p| [p[0] + c[0], p[1] + c[1], p[2] + c[2]]).collect();
                sag = sag.max(r - legacy::inscribed_radius(&legacy::icosphere(r, 3).0, &t));
                facets.add(&v, &t, k as i32, water, false)?;
            }
            Shape::Cyl { .. } => {}
        }
    }
    // Tet region by centroid; spheres are stair-stepped, so starts keep a band of one cell
    // diagonal around them (the facet sag is far smaller).
    let cell_diag = h * 3.0f64.sqrt();
    let region_of = |p: [f64; 3]| scene.region_at(p);
    let tets = kuhn_tets(n, h, &region_of)?;
    let bands: Vec<(f64, f64)> = scene.solids.iter().map(|s| if matches!(s.shape, Shape::Sphere { .. }) { (cell_diag, cell_diag) } else { (0.0, 0.0) }).collect();
    let rays = scene_rays("H1", &facets, &scene, &bands, scale.rays)?;
    let notes = vec![
        format!("tet mesh: structured Kuhn subdivision, {n}^3 cells of edge {h} cm (sm-scheme-tet cannot mesh conforming inclusions in this wave)"),
        "box inclusions are grid aligned and conform exactly; sphere inclusions are stair-stepped by tet centroid (not conforming)".into(),
        format!("starts keep {cell_diag:.6} cm away from sphere surfaces; tolerance for sphere chords in tet walks is one cell diagonal"),
        "regions 0..7 are the inclusions (boxes odd bit parity), region 8 is water".into(),
    ];
    let names: Vec<String> = (0..inc.len()).map(|k| format!("inclusion{k}")).chain(core::iter::once("water".to_string())).collect();
    let name_refs: Vec<&str> = names.iter().map(String::as_str).collect();
    let mut data = finish("H1", scale.name, facets, rays, Some(tets), Some(scene), regions(&name_refs), sag, 1, 1, notes)?;
    data.manifest.chord_tol = data.manifest.chord_tol.max(cell_diag);
    Ok(data)
}

/// F1 geometry constants (cm).
const F1_R: f64 = 600.0;
const F1_R_MIN: f64 = 5.0;
const F1_R_MAX: f64 = 200.0;

/// Minor radii of the F1 shells, ascending.
pub fn f1_minors(shells: usize) -> Vec<f64> {
    let k = shells.max(2);
    (0..k).map(|i| F1_R_MIN + (F1_R_MAX - F1_R_MIN) * i as f64 / (k - 1) as f64).collect()
}

fn build_f1(scale: &Scale) -> Result<CaseData> {
    let minors = f1_minors(scale.f1_shells);
    let facets = twin::nested_tori(F1_R, &minors, scale.sag_f1)?;
    let region_at = |p: [f64; 3]| -> i32 {
        let rho = (p[0] * p[0] + p[1] * p[1]).sqrt();
        let d = ((rho - F1_R).powi(2) + p[2] * p[2]).sqrt();
        minors.iter().position(|&r| d <= r).map_or(-1, |k| k as i32)
    };
    let sag = scale.sag_f1;
    let bb = facets.bbox().ok_or_else(|| BenchError::Geometry("empty facets".into()))?;
    let rays = gen_rays("F1", bb, scale.f1_rays, &|p| {
        let rho = (p[0] * p[0] + p[1] * p[1]).sqrt();
        let d = ((rho - F1_R).powi(2) + p[2] * p[2]).sqrt();
        if minors.iter().any(|&r| r - 2.0 * sag < d && d <= r + 2.0 * sag) {
            return None;
        }
        let r = region_at(p);
        (r >= 0).then_some(r)
    })?;
    let names: Vec<String> = (0..minors.len()).map(|k| format!("shell{k}")).collect();
    let name_refs: Vec<&str> = names.iter().map(String::as_str).collect();
    let notes = vec![
        "synthetic on purpose (no licence issue): nested full torus shells about z, R = 600 cm".into(),
        "ports and the cut plane of the fusion-like draft need booleans and are not generated; no analytic truth scene (torus quartic not in the truth module)".into(),
        "starts keep two sag away from every shell".into(),
    ];
    finish("F1", scale.name, facets, rays, None, None, regions(&name_refs), sag, 1, 1, notes)
}

/// A case from a local STEP model (`--model`). Never committed; start regions come from a
/// parity test on the facets.
pub fn build_model(bytes: &[u8], sag: f64, n_rays: usize) -> Result<CaseData> {
    let facets = twin::facets_of_step(bytes, sag)?;
    let bb = facets.bbox().ok_or_else(|| BenchError::Geometry("empty facet twin".into()))?;
    let classify = |p: [f64; 3]| -> Option<i32> { (crate::parity::point_in_facets(&facets, p) != 0).then_some(0) };
    let rays = gen_rays("model", bb, n_rays, &classify)?;
    let notes = vec!["local model: the file and everything derived from it stay out of the repository".to_string(), format!("model sha256 {}", sha256_hex(bytes))];
    finish("model", "model", facets, rays, None, None, regions(&["solid"]), sag, 1, 1, notes)
}

/// One line of `expected_sha256.txt` per generated file: `<sha>  <scale>/<case>/<file>`, with an
/// `@<os>-<arch>` suffix for platform-blessed files (everything `sm_tess` produced).
pub fn expected_lines(scale: &Scale, data: &CaseData) -> Result<Vec<String>> {
    let mut v = Vec::new();
    for (name, bytes) in data.files()? {
        let suffix = if platform_keyed(&data.case, &name) { format!("@{}", platform_tag()) } else { String::new() };
        v.push(format!("{}  {}/{}/{}{}", sha256_hex(&bytes), scale.name, data.case, name, suffix));
    }
    Ok(v)
}

/// Files that depend on `sm_tess` output (or carry the platform tag) are blessed per platform.
fn platform_keyed(case: &str, file: &str) -> bool {
    file == "smcase.json" || matches!(case, "P1" | "P2" | "F1")
}

/// Outcome of comparing generated files with the recorded hashes.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct CheckReport {
    /// Files that matched.
    pub matched: usize,
    /// Platform-blessed files with no entry for this platform (not a failure of the generator).
    pub unblessed: Vec<String>,
    /// Files whose hash differs from the recorded one.
    pub mismatched: Vec<String>,
}

impl CheckReport {
    /// True when nothing mismatched.
    pub fn ok(&self) -> bool {
        self.mismatched.is_empty()
    }
}

/// Compare `data` with the recorded hashes in `expected` (the text of `expected_sha256.txt`).
pub fn check_against(expected: &str, scale: &Scale, data: &CaseData) -> Result<CheckReport> {
    let table: BTreeMap<&str, &str> = expected.lines().filter_map(|l| l.split_once("  ")).map(|(h, k)| (k.trim(), h.trim())).collect();
    let plat = platform_tag();
    let mut rep = CheckReport::default();
    for (name, bytes) in data.files()? {
        let key = format!("{}/{}/{}", scale.name, data.case, name);
        let sha = sha256_hex(&bytes);
        let want = table.get(format!("{key}@{plat}").as_str()).or_else(|| table.get(key.as_str()));
        match want {
            Some(w) if *w == sha => rep.matched += 1,
            Some(_) => rep.mismatched.push(key),
            None => rep.unblessed.push(key),
        }
    }
    Ok(rep)
}

/// Bounding box of facet vertices.
pub fn facets_bbox(f: &Facets) -> Option<([f64; 3], [f64; 3])> {
    bbox_of(f.verts.iter())
}
