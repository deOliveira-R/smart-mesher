//! Row 11: an exported OpenFOAM case, parsed back with `sm_meshio::foam`, checked by
//! `check_polymesh` and by an independent re-check of checkMesh's topological rules.

use std::path::Path;

use sm_meshio::foam::{FoamFile, PolyMesh, check_polymesh, parse_polymesh};

/// The `constant/polyMesh` files of a case directory, sorted by name.
pub fn read_case(case: &Path) -> Result<Vec<FoamFile>, String> {
    let dir = case.join("constant/polyMesh");
    let mut names: Vec<String> = std::fs::read_dir(&dir)
        .map_err(|x| format!("{}: {x}", dir.display()))?
        .filter_map(Result::ok)
        .filter(|d| d.path().is_file())
        .map(|d| d.file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    names
        .into_iter()
        .map(|n| {
            let bytes = std::fs::read(dir.join(&n)).map_err(|x| format!("{n}: {x}"))?;
            Ok(FoamFile { name: n, bytes })
        })
        .collect()
}

fn newell(m: &PolyMesh, f: usize) -> ([f64; 3], [f64; 3]) {
    let l = m.face(f);
    let mut a = [0.0; 3];
    let mut c = [0.0; 3];
    for (i, &p) in l.iter().enumerate() {
        let p = m.points[p as usize];
        let q = m.points[l[(i + 1) % l.len()] as usize];
        a[0] += (p[1] - q[1]) * (p[2] + q[2]);
        a[1] += (p[2] - q[2]) * (p[0] + q[0]);
        a[2] += (p[0] - q[0]) * (p[1] + q[1]);
        for k in 0..3 {
            c[k] += p[k];
        }
    }
    let n = l.len().max(1) as f64;
    ([a[0] / 2.0, a[1] / 2.0, a[2] / 2.0], [c[0] / n, c[1] / n, c[2] / n])
}

fn norm(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

/// Row 11 for one case: problems found (empty = pass) and the parsed mesh.
pub struct FoamCheck {
    pub problems: Vec<String>,
    pub mesh: Option<PolyMesh>,
    /// `foam.upper_triangular_tie` warnings (faces sharing an owner / neighbour pair).
    pub ties: usize,
}

pub fn check_case(case: &Path) -> FoamCheck {
    let mut problems = Vec::new();
    let files = match read_case(case) {
        Ok(f) => f,
        Err(x) => return FoamCheck { problems: vec![x], mesh: None, ties: 0 },
    };
    let m = match parse_polymesh(&files) {
        Ok(m) => m,
        Err(x) => return FoamCheck { problems: vec![format!("parse_polymesh: {x}")], mesh: None, ties: 0 },
    };
    // Parsed back == written: re-serializing the parsed mesh gives the same bytes.
    match m.files() {
        Ok(again) => {
            let names = |v: &[FoamFile]| v.iter().map(|f| f.name.clone()).collect::<std::collections::BTreeSet<_>>();
            if names(&again) != names(&files) {
                problems.push(format!("re-serialized file set {:?} != written {:?}", names(&again), names(&files)));
            }
            for f in &again {
                if let Some(w) = files.iter().find(|w| w.name == f.name)
                    && w.bytes != f.bytes
                {
                    problems.push(format!("parsed-back {} does not re-serialize to the written bytes", f.name));
                }
            }
        }
        Err(x) => problems.push(format!("files(): {x}")),
    }
    let all = check_polymesh(&m, 1e-9);
    // `foam.upper_triangular_tie` is documented by sm-meshio as a warning: several faces between
    // the same two cells (the planar pieces of a dual face). OpenFOAM's upper-triangular check
    // accepts them; checkMesh in tools/conformance.sh is the judge. Counted and reported.
    let ties = all.iter().filter(|i| i.code == "foam.upper_triangular_tie").count();
    let issues: Vec<_> = all.into_iter().filter(|i| i.code != "foam.upper_triangular_tie").collect();
    if !issues.is_empty() {
        let mut codes: std::collections::BTreeMap<&str, usize> = std::collections::BTreeMap::new();
        for i in &issues {
            *codes.entry(i.code).or_insert(0) += 1;
        }
        problems.push(format!("check_polymesh: {codes:?}; first: {:?}", issues.first()));
    }
    // Independent re-check.
    let (nf, ni, nc) = (m.n_faces(), m.n_internal(), m.n_cells as usize);
    let mut prev = (0u32, 0u32);
    for f in 0..ni {
        let (o, n) = (m.owner[f], m.neighbour[f]);
        if n <= o {
            problems.push(format!("internal face {f}: neighbour {n} <= owner {o}"));
            break;
        }
        if f > 0 && (o, n) < prev {
            problems.push(format!("internal face {f}: (owner, neighbour) ({o}, {n}) not ascending after {prev:?}"));
            break;
        }
        prev = (o, n);
    }
    let mut at = ni as u32;
    for p in &m.patches {
        if p.start_face != at {
            problems.push(format!("patch {} starts at {} instead of {at}", p.name, p.start_face));
        }
        at = p.start_face + p.n_faces;
    }
    if at as usize != nf {
        problems.push(format!("patches end at {at}, faces {nf}"));
    }
    // Closed cells and owner -> neighbour normals.
    let mut sum = vec![[0.0f64; 3]; nc];
    let mut mag = vec![0.0f64; nc];
    let mut cen = vec![[0.0f64; 3]; nc];
    let mut faces: Vec<([f64; 3], [f64; 3])> = Vec::with_capacity(nf);
    for f in 0..nf {
        let (a, c) = newell(&m, f);
        let w = norm(a);
        let mut add = |cell: usize, s: f64| {
            for k in 0..3 {
                sum[cell][k] += s * a[k];
                cen[cell][k] += w * c[k];
            }
            mag[cell] += w;
        };
        add(m.owner[f] as usize, 1.0);
        if f < ni {
            add(m.neighbour[f] as usize, -1.0);
        }
        faces.push((a, c));
    }
    let open = (0..nc).filter(|&c| norm(sum[c]) > 1e-9 * mag[c].max(1e-300)).count();
    if open > 0 {
        problems.push(format!("{open} cells are not closed (sum of outward area vectors)"));
    }
    for (c, w) in cen.iter_mut().zip(&mag) {
        for x in c.iter_mut() {
            *x /= w.max(1e-300);
        }
    }
    let wrong = (0..ni)
        .filter(|&f| {
            let (a, _) = faces[f];
            let (o, n) = (cen[m.owner[f] as usize], cen[m.neighbour[f] as usize]);
            a[0] * (n[0] - o[0]) + a[1] * (n[1] - o[1]) + a[2] * (n[2] - o[2]) <= 0.0
        })
        .count();
    if wrong > 0 {
        problems.push(format!("{wrong} internal faces do not point from owner to neighbour"));
    }
    FoamCheck { problems, mesh: Some(m), ties }
}

/// What the exporter's INT-8 small-determinant split did to one case.
#[derive(Debug, Default)]
pub struct SplitFacts {
    /// Internal cells split (cells appearing more than once in `cell_map`).
    pub parents: usize,
    /// Exported sub-cells of those parents.
    pub sub_cells: usize,
    /// Internal faces added by the split (one per parent edge).
    pub added_faces: usize,
    /// Split parents by internal shape.
    pub by_shape: std::collections::BTreeMap<String, usize>,
}

/// Row 11 counts: the exported case against the internal mesh as INT-8 defines the export.
/// Without a cell map the counts are equal. With one: `cell_map.len() == n_cells`, the map is
/// onto `0..cell_count`, every split parent has one sub-cell per face, points = vertices + one
/// centroid per split parent, boundary faces unchanged, internal faces = ours + one per parent
/// edge (a tet adds 6).
pub fn compare_counts(pm: &PolyMesh, mesh: &dyn sm_mesh_api::MeshRead) -> (Vec<String>, Option<SplitFacts>) {
    use sm_mesh_api::{CellIx, FaceIx};
    let mut p = Vec::new();
    let (nv, nc, nf) = (mesh.vertex_count(), mesh.cell_count(), mesh.face_count());
    let our_internal = (0..nf).filter(|&f| mesh.face_right(FaceIx::new(f)).is_some()).count();
    let (fb, fi) = (pm.n_faces() - pm.n_internal(), pm.n_internal());
    if pm.cell_map.is_empty() {
        for (what, a, b) in [("points", pm.points.len(), nv), ("cells", pm.n_cells as usize, nc), ("faces", pm.n_faces(), nf), ("internal faces", fi, our_internal)] {
            if a != b {
                p.push(format!("{what}: {a} != {b}"));
            }
        }
        return (p, None);
    }
    if pm.cell_map.len() != pm.n_cells as usize {
        p.push(format!("cell_map has {} entries for {} cells", pm.cell_map.len(), pm.n_cells));
        return (p, None);
    }
    let mut mult = vec![0usize; nc];
    for &c in &pm.cell_map {
        match mult.get_mut(c as usize) {
            Some(m) => *m += 1,
            None => {
                p.push(format!("cell_map entry {c} >= internal cell count {nc}"));
                return (p, None);
            }
        }
    }
    let missing = mult.iter().filter(|&&m| m == 0).count();
    if missing > 0 {
        p.push(format!("cell_map is not onto the internal cells: {missing} internal cells have no exported cell"));
    }
    let mut sf = SplitFacts::default();
    let mut bad_mult = Vec::new();
    for (c, &m) in mult.iter().enumerate() {
        if m <= 1 {
            continue;
        }
        let cf = mesh.cell_faces(CellIx::new(c));
        let corners: usize = cf.iter().map(|s| mesh.face_vertices(s.face()).len()).sum();
        if m != cf.len() {
            bad_mult.push((c, m, cf.len()));
        }
        sf.parents += 1;
        sf.sub_cells += m;
        sf.added_faces += corners / 2;
        *sf.by_shape.entry(format!("{:?}", mesh.cell_shape(CellIx::new(c)))).or_insert(0) += 1;
    }
    if !bad_mult.is_empty() {
        p.push(format!("{} split parents do not have one sub-cell per face (cell, sub-cells, faces): {:?}", bad_mult.len(), &bad_mult[..bad_mult.len().min(5)]));
    }
    let want_cells = nc + sf.sub_cells - sf.parents;
    for (what, a, b) in [
        ("cells (internal + sub-cells - split parents)", pm.n_cells as usize, want_cells),
        ("points (vertices + one centroid per split parent)", pm.points.len(), nv + sf.parents),
        ("boundary faces (unchanged by the split)", fb, nf - our_internal),
        ("internal faces (ours + one per split parent edge)", fi, our_internal + sf.added_faces),
    ] {
        if a != b {
            p.push(format!("{what}: {a} != {b}"));
        }
    }
    (p, Some(sf))
}
