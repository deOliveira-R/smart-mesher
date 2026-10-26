//! Conversion of a mesh or a facet soup into the flat [`Core`], plus region, surface and
//! boundary tables.

use crate::core::{Core, DeriveIssues, NONE};
use crate::facets::check_facets;
use rayon::prelude::*;
use sm_core::{EntityId, Point3};
use sm_mesh_api::{CellIx, FaceIx, FaceKind, MeshDim, SetMembers, cell_volume};
use sm_rt_api::{
    BoundaryDef, BoundarySelector, BoundarySpec, BuildIssue, BuildOptions, BuildReport, ComplementPolicy, FaceAction, FacetInput, MeshInput, RegionId,
    RegionInfo, RegionRule, RegionTable, ResponseKind, RtError, SurfaceInfo, SurfaceTable, TrackerKind,
};
use sm_topo_api::Region;
use std::collections::BTreeMap;

/// A finished build, before it is wrapped into a tracker.
pub(crate) struct Built {
    pub core: Core,
    pub regions: RegionTable,
    pub surfaces: SurfaceTable,
    /// Region id of the implicit complement, when the tracker has one.
    pub complement: Option<u32>,
    pub kind: TrackerKind,
    pub report: BuildReport,
}

const MAX_ISSUES: usize = 16;

fn push_issue(v: &mut Vec<BuildIssue>, i: BuildIssue) {
    if v.len() < MAX_ISSUES {
        v.push(i);
    }
}

fn to_u32(x: u64) -> Result<u32, RtError> {
    u32::try_from(x).map_err(|_| RtError::Unsupported("more than u32::MAX faces, cells or face-vertex entries"))
}

/// Build from a polyhedral mesh.
pub(crate) fn build_mesh(inp: &MeshInput<'_>, opts: &BuildOptions) -> Result<Built, RtError> {
    let m = inp.mesh;
    if m.dim() != MeshDim::Three {
        return Err(RtError::Unsupported("two-dimensional mesh"));
    }
    let (nv, nf, nc) = (m.vertex_count(), m.face_count(), m.cell_count());
    if nc == 0 {
        return Err(RtError::Build(vec![BuildIssue::new("mesh.empty", "the mesh has no cells")]));
    }
    if (0..nc).any(|c| !m.cell_active(CellIx::new(c))) {
        return Err(RtError::Unsupported("mesh with inactive (refined) cells"));
    }
    let cols = m.columns();
    let co = m.coords();
    let mut fatal: Vec<BuildIssue> = Vec::new();
    if cols.face_vertex_offsets.len() != nf + 1 || cols.cell_face_offsets.len() != nc + 1 || cols.face_left.len() != nf || cols.face_right.len() != nf {
        return Err(RtError::Build(vec![BuildIssue::new("mesh.columns", "inconsistent topology column lengths")]));
    }
    let mut core = Core {
        pos: (0..nv).map(|i| [co.x[i], co.y[i], co.z[i]]).collect(),
        fv_off: cols.face_vertex_offsets.iter().map(|&o| to_u32(o)).collect::<Result<_, _>>()?,
        fv: cols.face_vertices.iter().map(|v| v.0).collect(),
        f_left: cols.face_left.iter().map(|c| c.0).collect(),
        f_right: cols.face_right.to_vec(),
        c_off: cols.cell_face_offsets.iter().map(|&o| to_u32(o)).collect::<Result<_, _>>()?,
        c_face: cols.cell_faces.iter().map(|s| s.face().0).collect(),
        ..Core::default()
    };
    for (i, p) in core.pos.iter().enumerate() {
        if !p.iter().all(|c| c.is_finite()) {
            push_issue(&mut fatal, BuildIssue::new("mesh.non_finite_vertex", format!("vertex {i} is not finite")));
        }
    }
    if core.fv.iter().any(|&v| v as usize >= nv) || core.f_left.iter().any(|&c| c as usize >= nc) || core.f_right.iter().any(|&c| c != NONE && c as usize >= nc) || core.c_face.iter().any(|&f| f as usize >= nf) {
        return Err(RtError::Build(vec![BuildIssue::new("mesh.index_range", "a topology column indexes a missing entity")]));
    }
    for c in 0..nc {
        let fs = core.cell_faces(c);
        if fs.len() < 4 {
            push_issue(&mut fatal, BuildIssue::new("mesh.cell_faces", format!("cell {c} has {} faces", fs.len())).with_entity(m.cell_id(CellIx::new(c))));
        }
        for &f in fs {
            if core.f_left[f as usize] as usize != c && core.f_right[f as usize] as usize != c {
                push_issue(&mut fatal, BuildIssue::new("mesh.inconsistent_adjacency", format!("cell {c} lists face {f} that does not list it")).with_entity(m.face_id(FaceIx(f))));
            }
        }
    }
    for f in 0..nf {
        if core.f_right[f] == NONE && m.face_kind(FaceIx::new(f)) != FaceKind::Boundary {
            push_issue(&mut fatal, BuildIssue::new("mesh.nonconformal_face", format!("face {f} is a one-cell face that is not a model boundary")).with_entity(m.face_id(FaceIx::new(f))).suggest("tracking needs a conformal mesh in W3a"));
        }
    }
    if !fatal.is_empty() {
        return Err(RtError::Build(fatal));
    }

    // Regions.
    let mut regions: Vec<RegionInfo> = Vec::new();
    let mut c_region: Vec<u32> = vec![NONE; nc];
    match inp.regions {
        RegionRule::ByVolumeClassification => {
            let ents: Vec<EntityId> = (0..nc).map(|c| m.cell_class(CellIx::new(c)).entity).collect();
            let mut uniq: Vec<EntityId> = ents.clone();
            uniq.sort();
            uniq.dedup();
            for e in &uniq {
                regions.push(RegionInfo::new(e.to_string()).with_entity(*e));
            }
            for (c, e) in ents.iter().enumerate() {
                c_region[c] = uniq.binary_search(e).map_or(NONE, |i| i as u32);
            }
        }
        RegionRule::ByCellSets(names) => {
            for (ri, name) in names.iter().enumerate() {
                regions.push(RegionInfo::new(name.clone()));
                let Some(set) = m.set_by_name(name) else {
                    push_issue(&mut fatal, BuildIssue::new("mesh.region_set_missing", format!("cell set `{name}` does not exist")));
                    continue;
                };
                let SetMembers::Cells(cells) = &set.members else {
                    push_issue(&mut fatal, BuildIssue::new("mesh.region_set_kind", format!("set `{name}` is not a cell set")));
                    continue;
                };
                for c in cells {
                    if c_region[c.get()] != NONE {
                        push_issue(&mut fatal, BuildIssue::new("mesh.region_overlap", format!("cell {} is in more than one region set", c.0)).with_entity(m.cell_id(*c)));
                    }
                    c_region[c.get()] = ri as u32;
                }
            }
            if let Some(c) = c_region.iter().position(|&r| r == NONE) {
                push_issue(&mut fatal, BuildIssue::new("mesh.region_uncovered", format!("cell {c} is in no region set")).with_entity(m.cell_id(CellIx::new(c))));
            }
        }
        _ => return Err(RtError::Unsupported("region rule")),
    }
    if !fatal.is_empty() {
        return Err(RtError::Build(fatal));
    }
    let n_real = regions.len() as u32;
    let complement = match inp.complement {
        ComplementPolicy::Outside => None,
        ComplementPolicy::Region(name) => {
            regions.push(RegionInfo::new(name.clone()).as_implicit_complement());
            Some(n_real)
        }
        _ => return Err(RtError::Unsupported("complement policy")),
    };
    core.c_region = c_region;

    // Surfaces: faces between different regions or on the model boundary.
    let keys: Vec<Option<(u64, u32, u32)>> = (0..nf)
        .into_par_iter()
        .map(|f| {
            let lr = core.c_region[core.f_left[f] as usize];
            let rr = if core.f_right[f] == NONE { NONE } else { core.c_region[core.f_right[f] as usize] };
            if lr == rr {
                return None;
            }
            let cl = m.face_class(FaceIx::new(f));
            let ent = if cl.dim() == Some(2) { cl.entity.to_raw() } else { u64::MAX };
            Some((ent, lr, rr))
        })
        .collect();
    let mut uniq: Vec<(u64, u32, u32)> = keys.iter().flatten().copied().collect();
    uniq.sort_unstable();
    uniq.dedup();
    core.f_surface = keys.iter().map(|k| k.map_or(NONE, |k| uniq.binary_search(&k).map_or(NONE, |i| i as u32))).collect();
    let mut surfaces: Vec<SurfaceInfo> = uniq
        .iter()
        .map(|&(ent, lr, rr)| {
            let mut s = SurfaceInfo::new(RegionId(lr), if rr == NONE { RegionId::NONE } else { RegionId(rr) });
            if let Some(e) = EntityId::from_raw(ent).filter(|_| ent != u64::MAX) {
                s = s.with_entity(e);
            }
            s
        })
        .collect();

    core.f_action = vec![NONE; nf];
    let volumes: Vec<f64> = (0..nc).into_par_iter().map(|c| cell_volume(m, CellIx::new(c))).collect();
    let DeriveIssues { nonplanar, degenerate } = core.derive(opts.planarity_tol, opts.bvh_leaf_size.max(1) as usize, Some(volumes));
    let mut report = BuildReport::default();
    if !degenerate.is_empty() {
        let issues = degenerate.iter().take(MAX_ISSUES).map(|&f| BuildIssue::new("mesh.degenerate_face", format!("face {f} has zero area")).with_entity(m.face_id(FaceIx(f)))).collect();
        return Err(RtError::Build(issues));
    }
    for &f in nonplanar.iter().take(MAX_ISSUES) {
        let p = core.f_apex[f as usize];
        report.issues.push(
            BuildIssue::new("mesh.nonplanar_face", format!("face {f} is not planar within {:e} of its diameter; tracked through its fan triangles", opts.planarity_tol))
                .with_entity(m.face_id(FaceIx(f)))
                .at(Point3::new(p[0], p[1], p[2])),
        );
    }

    // Boundary conditions.
    let rule_faces: Vec<Vec<u32>> = inp
        .boundaries
        .rules
        .iter()
        .map(|(sel, _)| match sel {
            BoundarySelector::Entity(e) => m.classified_on(*e).map(|s| s.faces.iter().map(|f| f.0).collect()).unwrap_or_default(),
            BoundarySelector::FaceSet(name) => match m.set_by_name(name).map(|s| &s.members) {
                Some(SetMembers::Faces(v)) => v.iter().map(|f| f.0).collect(),
                Some(SetMembers::Sides(v)) => {
                    let mut f: Vec<u32> = v.iter().map(|s| s.face().0).collect();
                    f.dedup();
                    f
                }
                _ => Vec::new(),
            },
            _ => Vec::new(),
        })
        .collect();
    let mate_faces = |e: EntityId| -> Vec<u32> { m.classified_on(e).map(|s| s.faces.iter().map(|f| f.0).collect()).unwrap_or_default() };
    let mut issues = Vec::new();
    assign_actions(&mut core, inp.boundaries, &rule_faces, &mate_faces, opts, &mut issues, complement.is_some(), |f| m.face_id(FaceIx(f)));
    if !issues.is_empty() {
        return Err(RtError::Build(issues));
    }
    fill_surface_boundaries(&core, &mut surfaces);
    Ok(Built { core, regions: RegionTable(regions), surfaces: SurfaceTable(surfaces), complement, kind: TrackerKind::MeshWalk, report })
}

/// Build from a watertight facet soup: one polyhedral cell per region, one face per triangle.
pub(crate) fn build_facets(f: &FacetInput, spec: &BoundarySpec, opts: &BuildOptions) -> Result<Built, RtError> {
    f.validate()?;
    let issues = check_facets(f, spec);
    if !issues.is_empty() {
        return Err(RtError::Build(issues));
    }
    let nt = f.triangles.len();
    if nt == 0 {
        return Err(RtError::Build(vec![BuildIssue::new("facet.empty", "no triangles")]));
    }
    let mut vols: Vec<EntityId> = f.front.iter().chain(&f.back).filter_map(|r| if let Region::Volume(v) = r { Some(*v) } else { None }).collect();
    vols.sort();
    vols.dedup();
    let nr = vols.len();
    let cell_of = |r: &Region| -> u32 {
        match r {
            Region::Volume(v) => vols.binary_search(v).map_or(NONE, |i| i as u32),
            Region::Exterior => NONE,
        }
    };
    let mut core = Core {
        pos: f.vertices.clone(),
        fv_off: (0..=nt as u32).map(|i| i * 3).collect(),
        fv: Vec::with_capacity(nt * 3),
        f_left: Vec::with_capacity(nt),
        f_right: Vec::with_capacity(nt),
        ..Core::default()
    };
    let mut fatal = Vec::new();
    for i in 0..nt {
        let (mut l, mut r) = (cell_of(&f.front[i]), cell_of(&f.back[i]));
        let mut t = f.triangles[i];
        if l == NONE && r == NONE {
            push_issue(&mut fatal, BuildIssue::new("facet.no_volume", format!("triangle {i} has no volume on either side")).with_entity(f.face[i]));
        } else if l == NONE {
            // Normal points out of the exterior: flip so the left cell is a volume.
            core::mem::swap(&mut l, &mut r);
            t.swap(1, 2);
        } else if l == r {
            push_issue(&mut fatal, BuildIssue::new("facet.same_region", format!("triangle {i} has the same volume on both sides")).with_entity(f.face[i]));
        }
        core.fv.extend_from_slice(&t);
        core.f_left.push(l);
        core.f_right.push(r);
    }
    if !fatal.is_empty() {
        return Err(RtError::Build(fatal));
    }
    // Cell CSR (ascending face index per cell).
    let mut counts = vec![0u32; nr + 1];
    for i in 0..nt {
        counts[core.f_left[i] as usize + 1] += 1;
        if core.f_right[i] != NONE {
            counts[core.f_right[i] as usize + 1] += 1;
        }
    }
    for c in 0..nr {
        counts[c + 1] += counts[c];
    }
    core.c_off = counts.clone();
    core.c_face = vec![0; counts[nr] as usize];
    let mut cursor = counts;
    for i in 0..nt {
        for c in [core.f_left[i], core.f_right[i]] {
            if c != NONE {
                core.c_face[cursor[c as usize] as usize] = i as u32;
                cursor[c as usize] += 1;
            }
        }
    }
    core.c_region = (0..nr as u32).collect();
    let mut regions: Vec<RegionInfo> = vols.iter().map(|v| RegionInfo::new(v.to_string()).with_entity(*v)).collect();
    regions.push(RegionInfo::new("complement").as_implicit_complement());

    let keys: Vec<(u64, u32, u32)> = (0..nt).map(|i| (f.face[i].to_raw(), core.f_left[i], core.f_right[i])).collect();
    let mut uniq = keys.clone();
    uniq.sort_unstable();
    uniq.dedup();
    core.f_surface = keys.iter().map(|k| uniq.binary_search(k).map_or(NONE, |i| i as u32)).collect();
    let mut surfaces: Vec<SurfaceInfo> = uniq
        .iter()
        .map(|&(ent, lr, rr)| {
            let mut s = SurfaceInfo::new(RegionId(lr), if rr == NONE { RegionId::NONE } else { RegionId(rr) });
            if let Some(e) = EntityId::from_raw(ent) {
                s = s.with_entity(e);
            }
            s
        })
        .collect();
    core.f_action = vec![NONE; nt];
    let DeriveIssues { degenerate, .. } = core.derive(opts.planarity_tol, opts.bvh_leaf_size.max(1) as usize, None);
    if !degenerate.is_empty() {
        let issues = degenerate.iter().take(MAX_ISSUES).map(|&t| BuildIssue::new("facet.zero_area", format!("triangle {t} has zero area")).with_entity(f.face[t as usize])).collect();
        return Err(RtError::Build(issues));
    }
    let mut by_entity: BTreeMap<u64, Vec<u32>> = BTreeMap::new();
    for i in 0..nt {
        by_entity.entry(f.face[i].to_raw()).or_default().push(i as u32);
    }
    let rule_faces: Vec<Vec<u32>> = spec
        .rules
        .iter()
        .map(|(sel, _)| match sel {
            BoundarySelector::Entity(e) => by_entity.get(&e.to_raw()).cloned().unwrap_or_default(),
            _ => Vec::new(),
        })
        .collect();
    let mate_faces = |e: EntityId| -> Vec<u32> { by_entity.get(&e.to_raw()).cloned().unwrap_or_default() };
    let mut issues = Vec::new();
    assign_actions(&mut core, spec, &rule_faces, &mate_faces, opts, &mut issues, true, |t| f.face[t as usize]);
    if !issues.is_empty() {
        return Err(RtError::Build(issues));
    }
    fill_surface_boundaries(&core, &mut surfaces);
    let report = BuildReport::default();
    Ok(Built { core, regions: RegionTable(regions), surfaces: SurfaceTable(surfaces), complement: Some(nr as u32), kind: TrackerKind::Facets, report })
}

fn fill_surface_boundaries(core: &Core, surfaces: &mut [SurfaceInfo]) {
    for f in 0..core.n_faces() {
        let (s, a) = (core.f_surface[f], core.f_action[f]);
        if s != NONE && a != NONE && surfaces[s as usize].boundary.is_none() {
            surfaces[s as usize].boundary = Some(core.actions[a as usize]);
        }
    }
}

/// Exact-bits key of an action, for interning in O(log n).
fn action_key(a: &FaceAction) -> [u64; 16] {
    let mut k = [0u64; 16];
    match a {
        FaceAction::Reflect => k[0] = 1,
        FaceAction::Wrap { transform, mate } => {
            k[0] = 2;
            for (i, v) in transform.m.iter().flatten().enumerate() {
                k[1 + i] = v.to_bits();
            }
            k[10] = transform.t.x.to_bits();
            k[11] = transform.t.y.to_bits();
            k[12] = transform.t.z.to_bits();
            k[13] = u64::from(mate.0);
        }
        FaceAction::Respond(r) => {
            k[0] = 3;
            k[1] = u64::from(r.code());
            k[2] = if let ResponseKind::Albedo(x) = r { x.to_bits() } else { 0 };
        }
        _ => {}
    }
    k
}

/// Resolve the boundary action of every one-cell face (first matching rule wins, then the
/// default), including the periodic mate matching by transformed face centre.
#[allow(clippy::too_many_arguments)] // one internal call site per input kind
fn assign_actions(
    core: &mut Core,
    spec: &BoundarySpec,
    rule_faces: &[Vec<u32>],
    mate_faces: &dyn Fn(EntityId) -> Vec<u32>,
    opts: &BuildOptions,
    issues: &mut Vec<BuildIssue>,
    has_complement: bool,
    entity_of: impl Fn(u32) -> EntityId,
) {
    let nf = core.n_faces();
    let mut face_rule = vec![NONE; nf];
    for (i, faces) in rule_faces.iter().enumerate().rev() {
        for &f in faces {
            if (f as usize) < nf && core.f_right[f as usize] == NONE {
                face_rule[f as usize] = i as u32;
            }
        }
    }
    // Candidate mate faces per mate entity: boundary faces sorted by centre x.
    let mut mates: BTreeMap<u64, Vec<([f64; 3], u32)>> = BTreeMap::new();
    let mut table: Vec<FaceAction> = Vec::new();
    let mut seen: BTreeMap<[u64; 16], u32> = BTreeMap::new();
    let mut intern = |a: FaceAction, table: &mut Vec<FaceAction>| -> u32 {
        *seen.entry(action_key(&a)).or_insert_with(|| {
            table.push(a);
            (table.len() - 1) as u32
        })
    };
    for (f, &rule) in face_rule.iter().enumerate() {
        if core.f_right[f] != NONE {
            continue;
        }
        // With a complement, a face no rule claims is crossed into it (no action).
        if rule == NONE && has_complement {
            continue;
        }
        let def = if rule == NONE { spec.default } else { spec.rules[rule as usize].1 };
        let action = match def {
            BoundaryDef::Reflect => FaceAction::Reflect,
            BoundaryDef::Respond(r) => {
                if !r.is_valid() {
                    push_issue(issues, BuildIssue::new("boundary.bad_albedo", "albedo must be in [0, 1]").with_entity(entity_of(f as u32)));
                    continue;
                }
                FaceAction::Respond(r)
            }
            BoundaryDef::Wrap { transform, mate } => {
                let cands = mates.entry(mate.to_raw()).or_insert_with(|| {
                    let mut v: Vec<([f64; 3], u32)> = mate_faces(mate).into_iter().filter(|&g| (g as usize) < nf && core.f_right[g as usize] == NONE).map(|g| (core.f_apex[g as usize], g)).collect();
                    v.sort_by(|a, b| a.0[0].total_cmp(&b.0[0]).then(a.1.cmp(&b.1)));
                    v
                });
                let c = core.f_apex[f];
                let img = transform.apply_point(Point3::new(c[0], c[1], c[2]));
                let img = [img.x, img.y, img.z];
                let tol = opts.tol.linear;
                let lo = cands.partition_point(|x| x.0[0] < img[0] - tol);
                let mut best: Option<(f64, u32)> = None;
                for (p, g) in &cands[lo..] {
                    if p[0] > img[0] + tol {
                        break;
                    }
                    let d = ((p[1] - img[1]).powi(2) + (p[2] - img[2]).powi(2) + (p[0] - img[0]).powi(2)).sqrt();
                    if d <= tol && best.is_none_or(|(bd, bg)| d < bd || (d == bd && *g < bg)) {
                        best = Some((d, *g));
                    }
                }
                match best {
                    Some((_, g)) => FaceAction::Wrap { transform, mate: sm_rt_api::FaceId(g) },
                    None => {
                        push_issue(
                            issues,
                            BuildIssue::new("deck.mismatch", "no mate face under the deck transform (non-matching periodic faces are not supported)")
                                .with_entity(entity_of(f as u32))
                                .at(Point3::new(img[0], img[1], img[2]))
                                .suggest("make the periodic boundary meshes match one-to-one"),
                        );
                        continue;
                    }
                }
            }
            _ => FaceAction::Respond(ResponseKind::Vacuum),
        };
        core.f_action[f] = intern(action, &mut table);
    }
    core.actions = table;
}
