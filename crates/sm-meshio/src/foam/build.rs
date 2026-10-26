//! Face-first IR -> [`PolyMesh`]: reorder faces, assign patches and zones, collect the report.

use super::{
    FoamBuild, FoamError, FoamOptions, FoamPatchType, FoamReport, PatchOrigin, PatchSource, PatchSummary, PolyFaceZone, PolyMesh, PolyPatch, PolyZone, ShapeCounts,
    ZoneSelect, sanitize,
};
use crate::error::{MeshioError, Result};
use crate::report::{LossItem, LossReport, LossSeverity};
use crate::smsh::{DeckParams, check_columns};
use rayon::prelude::*;
use sm_core::EntityKind;
use sm_mesh_api::{CellIx, CellShape, FaceIx, FaceKind, MeshDim, MeshRead, NO_CELL, SetMembers, Side};
use std::collections::BTreeSet;

const NONE: u32 = u32::MAX;
/// Faces per parallel task when filling the face CSR.
const CHUNK: usize = 4096;

fn fe(e: FoamError) -> MeshioError {
    MeshioError::Foam(e)
}

/// A patch under construction: source faces ascending.
struct PatchBuild {
    name: String,
    patch_type: FoamPatchType,
    origin: PatchOrigin,
    faces: Vec<u32>,
}

/// Build the OpenFOAM polyMesh of `mesh` under `opts`.
///
/// Cells keep their indices; faces are ordered `[internal][patch 0][patch 1]...` with internal
/// faces sorted by `(owner, neighbour, source index)` (taken as is when already sorted, which is
/// the case for every mesh in canonical face order). Loops are copied as stored: they are outward
/// from `left`, which is the owner. See `FOAM_FORMAT.md` for the patch rules.
///
/// Errors: [`FoamError::NotThreeDimensional`], [`FoamError::InactiveCells`],
/// [`FoamError::NonFinite`], [`FoamError::Orientation`] (an interior face with `left >= right`),
/// [`FoamError::UnknownSet`], [`FoamError::PatchOverlap`] and the name errors.
pub fn build_polymesh(mesh: &dyn MeshRead, opts: &FoamOptions) -> Result<FoamBuild> {
    if mesh.dim() != MeshDim::Three {
        return Err(fe(FoamError::NotThreeDimensional));
    }
    check_columns(mesh)?;
    let (nv, nf, nc) = (mesh.vertex_count(), mesh.face_count(), mesh.cell_count());
    let co = mesh.coords();
    if co.y.len() != co.x.len() || co.z.len() != co.x.len() || co.len() < nv {
        return Err(MeshioError::Inconsistent("coordinate columns disagree with the vertex count".into()));
    }
    let inactive = (0..nc).into_par_iter().filter(|&c| !mesh.cell_active(CellIx::new(c))).count();
    if inactive > 0 {
        return Err(fe(FoamError::InactiveCells { count: inactive }));
    }
    let cols = mesh.columns();
    let (left, right) = (cols.face_left, cols.face_right);
    let mut loss = LossReport::new();
    let mut note = |code: &str, sev: LossSeverity, count: usize, detail: String| loss.push(LossItem::new(code, sev, count as u64, detail));

    // ---- face kinds and the internal faces ----------------------------------------------------
    let kinds: Vec<u8> = (0..nf).into_par_iter().map(|f| mesh.face_kind(FaceIx::new(f)).code()).collect();
    let (k_int, k_bnd, k_nc, k_pc) = (FaceKind::Interior.code(), FaceKind::Boundary.code(), FaceKind::NonConformal.code(), FaceKind::PartitionCut.code());
    if let Some(f) = (0..nf).into_par_iter().find_first(|&f| (kinds[f] == k_int) != (right[f] != NO_CELL)) {
        return Err(MeshioError::Inconsistent(format!("face {f}: kind and right cell disagree (only interior faces have a right cell)")));
    }
    let mut internal: Vec<u32> = (0..nf).into_par_iter().filter(|&f| kinds[f] == k_int).map(|f| f as u32).collect();
    if let Some(&f) = internal.par_iter().find_first(|&&f| left[f as usize].0 >= right[f as usize]) {
        return Err(fe(FoamError::Orientation { face: f }));
    }
    let key = |f: u32| (left[f as usize].0, right[f as usize]);
    if !internal.par_windows(2).all(|w| key(w[0]) <= key(w[1])) {
        internal.par_sort_unstable_by_key(|&f| (key(f), f));
    }
    let n_int = internal.len();

    // ---- patch assignment ----------------------------------------------------------------------
    let mut assigned = vec![NONE; nf];
    let mut patches: Vec<PatchBuild> = Vec::new();
    let eligible = |members: &SetMembers| -> Option<Vec<u32>> {
        match members {
            SetMembers::Faces(v) => v.iter().all(|f| kinds[f.get()] == k_bnd).then(|| v.iter().map(|f| f.0).collect()),
            SetMembers::Sides(v) => v.iter().all(|s| kinds[s.face().get()] == k_bnd && s.side() == Side::Left).then(|| v.iter().map(|s| s.face().0).collect()),
            _ => None,
        }
    };
    let is_face_set = |m: &SetMembers| matches!(m, SetMembers::Faces(_) | SetMembers::Sides(_));
    match &opts.patches {
        PatchSource::Sets => {
            for s in mesh.sets().iter().filter(|s| is_face_set(&s.members)) {
                let Some(faces) = eligible(&s.members) else {
                    note("foam.set_skipped", LossSeverity::Degraded, s.members.len(), format!("set '{}' holds faces that are not model-boundary faces (or right-side entries)", s.name));
                    continue;
                };
                let idx = patches.len() as u32;
                let mut mine = Vec::with_capacity(faces.len());
                let mut overlap = 0usize;
                for f in faces {
                    if assigned[f as usize] == NONE {
                        assigned[f as usize] = idx;
                        mine.push(f);
                    } else {
                        overlap += 1;
                    }
                }
                if overlap > 0 {
                    note("foam.patch_overlap", LossSeverity::Degraded, overlap, format!("set '{}': faces already claimed by an earlier set", s.name));
                }
                patches.push(PatchBuild { name: s.name.clone(), patch_type: FoamPatchType::Patch, origin: PatchOrigin::Set(s.name.clone()), faces: mine });
            }
        }
        PatchSource::Explicit(specs) => {
            let mut owner_of: Vec<&str> = Vec::new(); // patch index -> set name
            for spec in specs {
                let set = mesh.set_by_name(&spec.set).ok_or_else(|| fe(FoamError::UnknownSet(spec.set.clone())))?;
                if !is_face_set(&set.members) {
                    return Err(fe(FoamError::IneligibleSet { set: spec.set.clone(), detail: "not a face or side set".into() }));
                }
                let Some(faces) = eligible(&set.members) else {
                    return Err(fe(FoamError::IneligibleSet {
                        set: spec.set.clone(),
                        detail: "holds faces that are not model-boundary faces (or right-side entries)".into(),
                    }));
                };
                let idx = patches.len() as u32;
                for &f in &faces {
                    let prev = assigned[f as usize];
                    if prev != NONE {
                        return Err(fe(FoamError::PatchOverlap { face: f, a: owner_of[prev as usize].to_string(), b: spec.set.clone() }));
                    }
                    assigned[f as usize] = idx;
                }
                owner_of.push(&spec.set);
                patches.push(PatchBuild {
                    name: spec.name.clone().unwrap_or_else(|| spec.set.clone()),
                    patch_type: spec.patch_type,
                    origin: PatchOrigin::Set(spec.set.clone()),
                    faces,
                });
            }
        }
        PatchSource::Classification => {
            let mut pairs: Vec<(u64, u32)> = (0..nf)
                .into_par_iter()
                .filter(|&f| kinds[f] == k_bnd)
                .filter_map(|f| {
                    let c = mesh.face_class(FaceIx::new(f));
                    (c.entity.kind() == EntityKind::TopoFace).then(|| (c.entity.serial(), f as u32))
                })
                .collect();
            pairs.par_sort_unstable();
            let mut i = 0;
            while i < pairs.len() {
                let serial = pairs[i].0;
                let j = i + pairs[i..].iter().take_while(|p| p.0 == serial).count();
                let idx = patches.len() as u32;
                let faces: Vec<u32> = pairs[i..j].iter().map(|p| p.1).collect();
                for &f in &faces {
                    assigned[f as usize] = idx;
                }
                let id = mesh.face_class(FaceIx(faces[0])).entity;
                patches.push(PatchBuild { name: format!("topo_face_{serial}"), patch_type: FoamPatchType::Patch, origin: PatchOrigin::Classification(id), faces });
                i = j;
            }
        }
    }
    let collect = |kind: u8, only_unassigned: bool| -> Vec<u32> {
        (0..nf).into_par_iter().filter(|&f| kinds[f] == kind && (!only_unassigned || assigned[f] == NONE)).map(|f| f as u32).collect()
    };
    let default_faces = collect(k_bnd, true);
    let nc_faces = collect(k_nc, false);
    let pc_faces = collect(k_pc, false);
    if !default_faces.is_empty() {
        note("foam.default_patch", LossSeverity::Info, default_faces.len(), format!("boundary faces no patch claims go to '{}'", opts.default_patch));
        patches.push(PatchBuild { name: opts.default_patch.clone(), patch_type: opts.default_patch_type, origin: PatchOrigin::Default, faces: default_faces });
    }
    if !nc_faces.is_empty() {
        note("foam.nonconformal_as_patch", LossSeverity::Degraded, nc_faces.len(), "non-conformal faces are written as the plain patch 'nonConformal' (no AMI/NCC)".into());
        patches.push(PatchBuild { name: "nonConformal".into(), patch_type: FoamPatchType::Patch, origin: PatchOrigin::NonConformal, faces: nc_faces });
    }
    if !pc_faces.is_empty() {
        note("foam.partition_cut_as_patch", LossSeverity::Degraded, pc_faces.len(), "partition cuts are written as the plain patch 'partitionCut'".into());
        patches.push(PatchBuild { name: "partitionCut".into(), patch_type: FoamPatchType::Patch, origin: PatchOrigin::PartitionCut, faces: pc_faces });
    }
    patches.retain(|p| !p.faces.is_empty());

    // Decks: a mirror's face set becomes a symmetryPlane when exactly planar; translation and
    // rotation pairs stay plain patches (no cyclic output).
    let diag = bbox_diagonal(mesh);
    for p in patches.iter_mut() {
        let PatchOrigin::Set(set) = &p.origin else { continue };
        for d in &opts.decks {
            let on_a = d.face_set.as_deref() == Some(set.as_str());
            let on_b = d.mate_set.as_deref() == Some(set.as_str());
            match &d.params {
                DeckParams::Mirror { .. } if on_a && p.patch_type == FoamPatchType::Patch => {
                    if is_planar(mesh, &p.faces, diag) {
                        p.patch_type = FoamPatchType::SymmetryPlane;
                    } else {
                        note("foam.symmetry_not_planar", LossSeverity::Degraded, p.faces.len(), format!("mirror deck '{}': patch '{}' is not planar, written as a plain patch", d.name, p.name));
                    }
                }
                DeckParams::Translation { .. } | DeckParams::Rotation { .. } if on_a || on_b => {
                    note("foam.deck_cyclic_not_emitted", LossSeverity::Degraded, p.faces.len(), format!("deck '{}': patch '{}' is a plain patch, no cyclic pair", d.name, p.name));
                }
                _ => {}
            }
        }
    }

    // Names.
    let mut seen = BTreeSet::new();
    for p in patches.iter_mut() {
        let clean = sanitize(&p.name).map_err(fe)?;
        if clean != p.name {
            note("foam.name_sanitized", LossSeverity::Info, 1, format!("patch '{}' -> '{clean}'", p.name));
            p.name = clean;
        }
        if !seen.insert(p.name.clone()) {
            return Err(fe(FoamError::DuplicateName(p.name.clone())));
        }
    }

    // ---- foam face order and point renumbering ---------------------------------------------------
    let mut face_of_foam: Vec<u32> = Vec::with_capacity(nf);
    face_of_foam.extend_from_slice(&internal);
    let mut poly_patches = Vec::with_capacity(patches.len());
    for p in &patches {
        poly_patches.push(PolyPatch { name: p.name.clone(), patch_type: p.patch_type, start_face: face_of_foam.len() as u32, n_faces: p.faces.len() as u32, origin: p.origin.clone() });
        face_of_foam.extend_from_slice(&p.faces);
    }
    debug_assert_eq!(face_of_foam.len(), nf);

    let mut used = vec![false; nv];
    for v in cols.face_vertices {
        used[v.get()] = true;
    }
    let n_used = used.iter().filter(|u| **u).count();
    let all_used = n_used == nv;
    let (point_of_foam, new_of_old): (Vec<u32>, Vec<u32>) = if all_used {
        ((0..nv as u32).collect(), Vec::new())
    } else {
        let mut map = vec![NONE; nv];
        let mut kept = Vec::with_capacity(n_used);
        for (i, u) in used.iter().enumerate() {
            if *u {
                map[i] = kept.len() as u32;
                kept.push(i as u32);
            }
        }
        note("foam.unused_points_dropped", LossSeverity::Info, nv - n_used, "vertices no face references are not written".into());
        (kept, map)
    };
    let points: Vec<[f64; 3]> = point_of_foam.par_iter().map(|&v| [co.x[v as usize], co.y[v as usize], co.z[v as usize]]).collect();
    if let Some(i) = points.par_iter().position_first(|p| !(p[0].is_finite() && p[1].is_finite() && p[2].is_finite())) {
        return Err(fe(FoamError::NonFinite { point: point_of_foam[i] }));
    }

    let offs = cols.face_vertex_offsets;
    let mut face_offsets: Vec<u64> = Vec::with_capacity(nf + 1);
    face_offsets.push(0);
    let mut acc = 0u64;
    for &src in &face_of_foam {
        acc += offs[src as usize + 1] - offs[src as usize];
        face_offsets.push(acc);
    }
    let mut face_points = vec![0u32; acc as usize];
    {
        let src_pts: &[u32] = bytemuck::cast_slice(cols.face_vertices);
        let mut windows: Vec<(usize, &mut [u32])> = Vec::with_capacity(nf.div_ceil(CHUNK));
        let mut rest = face_points.as_mut_slice();
        for k in 0..nf.div_ceil(CHUNK) {
            let (lo, hi) = (k * CHUNK, ((k + 1) * CHUNK).min(nf));
            let (head, tail) = rest.split_at_mut((face_offsets[hi] - face_offsets[lo]) as usize);
            windows.push((k, head));
            rest = tail;
        }
        windows.into_par_iter().for_each(|(k, win)| {
            let (lo, hi) = (k * CHUNK, ((k + 1) * CHUNK).min(nf));
            let mut at = 0usize;
            for &src in &face_of_foam[lo..hi] {
                let src = src as usize;
                let s = &src_pts[offs[src] as usize..offs[src + 1] as usize];
                let dst = &mut win[at..at + s.len()];
                if all_used {
                    dst.copy_from_slice(s);
                } else {
                    for (d, v) in dst.iter_mut().zip(s) {
                        *d = new_of_old[*v as usize];
                    }
                }
                at += s.len();
            }
        });
    }
    let owner: Vec<u32> = face_of_foam.par_iter().map(|&f| left[f as usize].0).collect();
    let neighbour: Vec<u32> = internal.par_iter().map(|&f| right[f as usize]).collect();

    // ---- zones ---------------------------------------------------------------------------------
    let find_cell_set = |name: &str| mesh.sets().iter().any(|s| s.name == name && matches!(s.members, SetMembers::Cells(_)));
    let mut cell_zones: Vec<PolyZone> = Vec::new();
    let wanted: Option<BTreeSet<&str>> = match &opts.cell_zones {
        ZoneSelect::All => None,
        ZoneSelect::None => Some(BTreeSet::new()),
        ZoneSelect::Names(v) => {
            if let Some(bad) = v.iter().find(|n| !find_cell_set(n)) {
                return Err(fe(FoamError::UnknownSet(bad.clone())));
            }
            Some(v.iter().map(String::as_str).collect())
        }
    };
    for s in mesh.sets() {
        let SetMembers::Cells(c) = &s.members else { continue };
        if c.is_empty() || wanted.as_ref().is_some_and(|w| !w.contains(s.name.as_str())) {
            continue;
        }
        let clean = sanitize(&s.name).map_err(fe)?;
        if clean != s.name {
            note("foam.name_sanitized", LossSeverity::Info, 1, format!("cellZone '{}' -> '{clean}'", s.name));
        }
        cell_zones.push(PolyZone { name: clean, labels: c.iter().map(|c| c.0).collect() });
    }
    unique(cell_zones.iter().map(|z| z.name.as_str()))?;

    let mut face_zones: Vec<PolyFaceZone> = Vec::new();
    if !opts.face_zones.is_empty() {
        for n in &opts.face_zones {
            match mesh.set_by_name(n) {
                None => return Err(fe(FoamError::UnknownSet(n.clone()))),
                Some(s) if !is_face_set(&s.members) => return Err(fe(FoamError::IneligibleSet { set: n.clone(), detail: "not a face or side set".into() })),
                Some(_) => {}
            }
        }
        let want: BTreeSet<&str> = opts.face_zones.iter().map(String::as_str).collect();
        let mut foam_of = vec![NONE; nf];
        for (i, &f) in face_of_foam.iter().enumerate() {
            foam_of[f as usize] = i as u32;
        }
        for s in mesh.sets() {
            if !want.contains(s.name.as_str()) {
                continue;
            }
            let mut pairs: Vec<(u32, bool)> = Vec::with_capacity(s.members.len());
            match &s.members {
                SetMembers::Faces(v) => pairs.extend(v.iter().map(|f| (foam_of[f.get()], false))),
                SetMembers::Sides(v) => {
                    let mut both = 0usize;
                    let mut i = 0;
                    while i < v.len() {
                        let f = v[i].face();
                        let two = v.get(i + 1).is_some_and(|n| n.face() == f);
                        let flip = if two {
                            both += 1;
                            false
                        } else {
                            v[i].side() == Side::Right
                        };
                        if flip && right[f.get()] == NO_CELL {
                            return Err(fe(FoamError::Malformed(format!("set '{}': right-side entry on one-cell face {}", s.name, f.0))));
                        }
                        pairs.push((foam_of[f.get()], flip));
                        i += if two { 2 } else { 1 };
                    }
                    if both > 0 {
                        note("foam.zone_both_sides", LossSeverity::Degraded, both, format!("faceZone '{}': faces listed on both sides keep the left entry", s.name));
                    }
                }
                _ => continue,
            }
            if pairs.is_empty() {
                continue;
            }
            pairs.sort_unstable();
            let clean = sanitize(&s.name).map_err(fe)?;
            if clean != s.name {
                note("foam.name_sanitized", LossSeverity::Info, 1, format!("faceZone '{}' -> '{clean}'", s.name));
            }
            face_zones.push(PolyFaceZone { name: clean, labels: pairs.iter().map(|p| p.0).collect(), flips: pairs.iter().map(|p| p.1).collect() });
        }
        unique(face_zones.iter().map(|z| z.name.as_str()))?;
    }

    let ho = (0..nc).into_par_iter().filter(|&c| mesh.high_order_cell(CellIx::new(c)).is_some()).count();
    if ho > 0 {
        note("foam.high_order_dropped", LossSeverity::Degraded, ho, "high-order cells are written as their linear corners; extra nodes are not written".into());
    }

    // ---- report ----------------------------------------------------------------------------------
    let counts = (0..nc)
        .into_par_iter()
        .fold(
            || [0u64; 5],
            |mut a, c| {
                match mesh.cell_shape(CellIx::new(c)) {
                    CellShape::Hex => a[0] += 1,
                    CellShape::Wedge => a[1] += 1,
                    CellShape::Pyramid => a[2] += 1,
                    CellShape::Tet => a[3] += 1,
                    _ => a[4] += 1,
                }
                a
            },
        )
        .reduce(|| [0u64; 5], |mut a, b| {
            for i in 0..5 {
                a[i] += b[i];
            }
            a
        });
    let report = FoamReport {
        n_points: points.len() as u64,
        n_cells: nc as u64,
        n_faces: nf as u64,
        n_internal_faces: n_int as u64,
        shape_counts: ShapeCounts { hexahedra: counts[0], prisms: counts[1], pyramids: counts[2], tetrahedra: counts[3], polyhedra: counts[4] },
        patches: poly_patches
            .iter()
            .map(|p| PatchSummary { name: p.name.clone(), patch_type: p.patch_type, start_face: p.start_face, n_faces: p.n_faces, origin: p.origin.clone() })
            .collect(),
        loss,
    };
    let poly = PolyMesh { points, face_offsets, face_points, owner, neighbour, n_cells: nc as u32, patches: poly_patches, cell_zones, face_zones };
    Ok(FoamBuild { mesh: poly, face_of_foam, point_of_foam, report })
}

/// `DuplicateName` for the first repeated name.
fn unique<'a>(names: impl Iterator<Item = &'a str>) -> Result<()> {
    let mut seen = BTreeSet::new();
    for n in names {
        if !seen.insert(n) {
            return Err(fe(FoamError::DuplicateName(n.to_string())));
        }
    }
    Ok(())
}

/// Diagonal of the bounding box of the vertices (0 for an empty mesh).
fn bbox_diagonal(m: &dyn MeshRead) -> f64 {
    let co = m.coords();
    let n = m.vertex_count();
    if n == 0 {
        return 0.0;
    }
    let ext = |a: &[f64]| a[..n].iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| (lo.min(*v), hi.max(*v)));
    let (x, y, z) = (ext(co.x), ext(co.y), ext(co.z));
    ((x.1 - x.0).powi(2) + (y.1 - y.0).powi(2) + (z.1 - z.0).powi(2)).sqrt()
}

/// True if every vertex of every listed face lies in the plane of the first non-degenerate face,
/// within `1e-9 * diag`.
fn is_planar(m: &dyn MeshRead, faces: &[u32], diag: f64) -> bool {
    let co = m.coords();
    let p = |v: sm_mesh_api::VertexIx| [co.x[v.get()], co.y[v.get()], co.z[v.get()]];
    let mut plane: Option<([f64; 3], [f64; 3])> = None;
    for &f in faces {
        let l = m.face_vertices(FaceIx(f));
        let mut n = [0.0; 3];
        for i in 0..l.len() {
            let (a, b) = (p(l[i]), p(l[(i + 1) % l.len()]));
            n[0] += (a[1] - b[1]) * (a[2] + b[2]);
            n[1] += (a[2] - b[2]) * (a[0] + b[0]);
            n[2] += (a[0] - b[0]) * (a[1] + b[1]);
        }
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        if len > 0.0 {
            plane = Some(([n[0] / len, n[1] / len, n[2] / len], p(l[0])));
            break;
        }
    }
    let Some((n, p0)) = plane else { return false };
    let tol = 1e-9 * diag;
    faces.par_iter().all(|&f| {
        m.face_vertices(FaceIx(f)).iter().all(|&v| {
            let q = p(v);
            (n[0] * (q[0] - p0[0]) + n[1] * (q[1] - p0[1]) + n[2] * (q[2] - p0[2])).abs() <= tol
        })
    })
}
