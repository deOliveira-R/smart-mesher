//! Building a [`DagmcModel`]: from a classified mesh, from facets, the sense normalisation, the
//! closed-volume check and the graveyard.

use super::{DagmcError, DagmcGroup, DagmcModel, DagmcOptions, DagmcSurface, DagmcVolume, Graveyard};
use crate::report::{LossItem, LossReport, LossSeverity};
use sm_core::{EntityId, EntityKind};
use sm_mesh_api::{CellIx, FaceIx, MeshDim, MeshRead, SetMembers};
use sm_rt_api::FacetInput;
use sm_topo_api::Region;
use std::collections::{BTreeMap, BTreeSet};

/// One triangle before normalisation: vertex indices, surface id and the volume ids (serials)
/// its normal points out of and into.
struct RawTri {
    v: [u32; 3],
    surf: u64,
    front: Option<u64>,
    back: Option<u64>,
}

fn unsupported<T>(m: impl Into<String>) -> Result<T, DagmcError> {
    Err(DagmcError::Unsupported(m.into()))
}

/// Normalise senses (forward = `Some`, reversed windings flipped), group triangles by surface
/// and check that every volume closes.
fn assemble(vertices: Vec<[f64; 3]>, raw: Vec<RawTri>, extra_volumes: &[u64]) -> Result<DagmcModel, DagmcError> {
    let mut senses: BTreeMap<u64, (Option<u64>, Option<u64>)> = BTreeMap::new();
    let mut tris: Vec<([u32; 3], u64)> = Vec::with_capacity(raw.len());
    for t in raw {
        let (mut f, mut b, mut v) = (t.front, t.back, t.v);
        if f.is_none() {
            if b.is_none() {
                return Err(DagmcError::Invalid(format!("surface {}: a triangle has no volume on either side", t.surf)));
            }
            std::mem::swap(&mut f, &mut b);
            v.swap(1, 2);
        }
        if f == b {
            return Err(DagmcError::Invalid(format!("surface {}: a triangle has the same volume on both sides", t.surf)));
        }
        match senses.get(&t.surf) {
            None => {
                senses.insert(t.surf, (f, b));
            }
            Some(&(sf, sb)) => {
                if (f, b) == (sf, sb) {
                } else if sb.is_some() && f == sb && b == sf {
                    v.swap(1, 2);
                } else {
                    return Err(DagmcError::InconsistentSense { surface: t.surf });
                }
            }
        }
        tris.push((v, t.surf));
    }
    let mut vols: BTreeSet<u64> = extra_volumes.iter().copied().collect();
    for (f, b) in senses.values() {
        vols.extend(f.iter().chain(b.iter()));
    }
    let volumes: Vec<DagmcVolume> = vols.iter().map(|&id| DagmcVolume { id }).collect();
    let vix = |id: u64| volumes.binary_search_by_key(&id, |v| v.id).ok().map(|i| i as u32);
    let surfaces: Vec<DagmcSurface> = senses
        .iter()
        .map(|(&id, &(f, b))| DagmcSurface { id, forward: f.and_then(vix), backward: b.and_then(vix) })
        .collect();
    let six = |id: u64| surfaces.binary_search_by_key(&id, |s| s.id).unwrap_or(0) as u32;
    let mut order: Vec<usize> = (0..tris.len()).collect();
    order.sort_by_key(|&i| tris[i].1);
    let triangles = order.iter().map(|&i| tris[i].0).collect();
    let tri_surface = order.iter().map(|&i| six(tris[i].1)).collect();
    let model = DagmcModel { vertices, triangles, tri_surface, surfaces, volumes, groups: Vec::new() };
    check_closed(&model)?;
    Ok(model)
}

/// Every edge of every volume must be used exactly twice, in opposite directions, by the
/// triangles oriented out of that volume.
pub(super) fn check_closed(m: &DagmcModel) -> Result<(), DagmcError> {
    // (volume, lo, hi, direction bit (1: the outward edge runs lo -> hi), triangle)
    let mut uses: Vec<(u32, u32, u32, u8, u32)> = Vec::with_capacity(m.triangles.len() * 6);
    for (ti, t) in m.triangles.iter().enumerate() {
        let Some(s) = m.tri_surface.get(ti).and_then(|&s| m.surfaces.get(s as usize)) else {
            return Err(DagmcError::Invalid(format!("triangle {ti} has no surface")));
        };
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            let (lo, hi) = (a.min(b), a.max(b));
            if let Some(v) = s.forward {
                uses.push((v, lo, hi, u8::from(a < b), ti as u32));
            }
            if let Some(v) = s.backward {
                uses.push((v, lo, hi, u8::from(b < a), ti as u32));
            }
        }
    }
    uses.sort_unstable();
    let mut i = 0;
    while i < uses.len() {
        let key = (uses[i].0, uses[i].1, uses[i].2);
        let mut j = i;
        while j < uses.len() && (uses[j].0, uses[j].1, uses[j].2) == key {
            j += 1;
        }
        let g = &uses[i..j];
        let closed = g.len() == 2 && g[0].3 != g[1].3;
        if !closed {
            let edge = [key.1, key.2];
            if g.len() > 2 {
                let surface = m.surfaces[m.tri_surface[g[0].4 as usize] as usize].id;
                return Err(DagmcError::NonManifoldSurface { surface, edge });
            }
            let p = |v: u32| m.vertices.get(v as usize).copied().unwrap_or([f64::NAN; 3]);
            let (a, b) = (p(key.1), p(key.2));
            let at = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0];
            return Err(DagmcError::OpenSurface { volume: m.volumes[key.0 as usize].id, edge, at });
        }
        i = j;
    }
    Ok(())
}

/// Keep only the vertices `raw` uses (ascending original index), then append `extra`.
fn compact(coords: &[[f64; 3]], raw: &mut [RawTri], extra: &[[f64; 3]]) -> Vec<[f64; 3]> {
    let n = coords.len();
    let mut used = vec![false; n];
    for t in raw.iter() {
        for &v in &t.v {
            if (v as usize) < n {
                used[v as usize] = true;
            }
        }
    }
    let mut map = vec![0u32; n];
    let mut out = Vec::new();
    for (i, u) in used.iter().enumerate() {
        if *u {
            map[i] = out.len() as u32;
            out.push(coords[i]);
        }
    }
    let base = out.len() as u32;
    out.extend_from_slice(extra);
    for t in raw.iter_mut() {
        for v in &mut t.v {
            *v = if (*v as usize) < n { map[*v as usize] } else { base + (*v - n as u32) };
        }
    }
    out
}

fn d2(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|k| (a[k] - b[k]) * (a[k] - b[k])).sum()
}

fn loss_names(r: &mut LossReport, code: &str, sev: LossSeverity, names: &[String], what: &str) {
    if !names.is_empty() {
        r.push(LossItem::new(code, sev, names.len() as u64, format!("{what}: {}", names.join(","))));
    }
}

/// The model of a classified volume mesh: surfaces are the faces on a `TopoFace` that bound a
/// cell and separate two volumes (or a volume and the outside); volumes are the `TopoVolume`s of
/// the cells; each cell set becomes the group `mat:<set name>`.
///
/// The forward volume of a surface is the left volume of its lowest face; triangles of faces
/// whose left volume is the other one are reversed, so one surface has one sense. Quads split
/// on the shorter diagonal (ties: the diagonal through the lowest vertex index); polygons of
/// five or more vertices fan from a new face-centroid vertex. Loss codes: `h5m.high_order_dropped`,
/// `h5m.face_curves_dropped`, `h5m.sets_dropped` (face, vertex and side sets),
/// `h5m.set_cuts_volume` (a cell set holds part of a volume: the whole volume is in the group),
/// `h5m.volume_without_material`, `h5m.volume_in_multiple_groups`, `h5m.graveyard_exists`.
pub fn model_from_mesh(mesh: &dyn MeshRead, o: &DagmcOptions) -> Result<(DagmcModel, LossReport), DagmcError> {
    if mesh.dim() != MeshDim::Three {
        return unsupported("DAGMC export needs a dimension-3 mesh");
    }
    let nc = mesh.cell_count();
    if nc == 0 {
        return unsupported("the mesh has no cells");
    }
    let mut vol_of: Vec<u64> = Vec::with_capacity(nc);
    for c in 0..nc {
        let ci = CellIx::new(c);
        if !mesh.cell_active(ci) {
            return unsupported("inactive (refined parent) cells: extract the leaf mesh first");
        }
        let cl = mesh.cell_class(ci).entity;
        match cl.kind() {
            EntityKind::TopoVolume => vol_of.push(cl.serial()),
            EntityKind::PartitionCell => return unsupported(format!("cell {c} is classified on a partition cell; map partitions to their TopoVolume first")),
            k => return unsupported(format!("cell {c} is classified on {k:?}, not a TopoVolume")),
        }
    }
    let nv = mesh.vertex_count();
    let co = mesh.coords();
    if co.len() < nv {
        return unsupported("fewer coordinates than vertices");
    }
    let coords: Vec<[f64; 3]> = (0..nv).map(|i| [co.x[i], co.y[i], co.z[i]]).collect();
    if coords.iter().flatten().any(|c| !c.is_finite()) {
        return unsupported("a vertex coordinate is not finite");
    }
    let mut raw: Vec<RawTri> = Vec::new();
    let mut centroids: Vec<[f64; 3]> = Vec::new();
    let mut curves = 0u64;
    for f in 0..mesh.face_count() {
        let fi = FaceIx::new(f);
        let l = mesh.face_left(fi);
        let r = mesh.face_right(fi);
        let Some(&lv) = vol_of.get(l.get()) else { return unsupported(format!("face {f} has a left cell out of range")) };
        let rv = match r {
            Some(c) => match vol_of.get(c.get()) {
                Some(&v) => Some(v),
                None => return unsupported(format!("face {f} has a right cell out of range")),
            },
            None => None,
        };
        if rv == Some(lv) {
            continue;
        }
        let fc = mesh.face_class(fi).entity;
        let surf = match fc.kind() {
            EntityKind::TopoFace => fc.serial(),
            EntityKind::PartitionFace => return unsupported(format!("face {f} is classified on a partition face; map partitions to their TopoFace first")),
            _ => return Err(DagmcError::Unclassified { face: f as u32 }),
        };
        if mesh.face_curve(fi).is_some() {
            curves += 1;
        }
        let vs = mesh.face_vertices(fi);
        let n = vs.len();
        if n < 3 {
            return unsupported(format!("face {f} has {n} vertices"));
        }
        if vs.iter().any(|v| v.get() >= nv) {
            return unsupported(format!("face {f} indexes a missing vertex"));
        }
        let v: Vec<u32> = vs.iter().map(|v| v.0).collect();
        let mut push = |t: [u32; 3]| raw.push(RawTri { v: t, surf, front: Some(lv), back: rv });
        match n {
            3 => push([v[0], v[1], v[2]]),
            4 => {
                let p = |k: usize| coords[v[k] as usize];
                let (d02, d13) = (d2(p(0), p(2)), d2(p(1), p(3)));
                let first = d02 < d13 || (d02 == d13 && v[0].min(v[2]) < v[1].min(v[3]));
                if first {
                    push([v[0], v[1], v[2]]);
                    push([v[0], v[2], v[3]]);
                } else {
                    push([v[1], v[2], v[3]]);
                    push([v[1], v[3], v[0]]);
                }
            }
            _ => {
                let mut c = [0.0; 3];
                for &x in &v {
                    for (k, ck) in c.iter_mut().enumerate() {
                        *ck += coords[x as usize][k] / n as f64;
                    }
                }
                let cid = (nv + centroids.len()) as u32;
                centroids.push(c);
                for k in 0..n {
                    push([v[k], v[(k + 1) % n], cid]);
                }
            }
        }
    }
    // Vertex ids >= nv are centroids: compaction maps them behind the used mesh vertices.
    let vertices = compact(&coords, &mut raw, &centroids);
    let mut vol_ids: Vec<u64> = vol_of.clone();
    vol_ids.sort_unstable();
    vol_ids.dedup();
    let mut model = assemble(vertices, raw, &vol_ids)?;

    let mut loss = LossReport::new();
    let ho = (0..nc).filter(|&c| mesh.high_order_cell(CellIx::new(c)).is_some()).count();
    if ho > 0 {
        loss.push(LossItem::new("h5m.high_order_dropped", LossSeverity::Degraded, ho as u64, "cells with high-order geometry are faceted on their linear corners"));
    }
    if curves > 0 {
        loss.push(LossItem::new("h5m.face_curves_dropped", LossSeverity::Dropped, curves, "curved-face records are not written"));
    }
    let mut cells_in_volume: BTreeMap<u64, u64> = BTreeMap::new();
    for v in &vol_of {
        *cells_in_volume.entry(*v).or_default() += 1;
    }
    let mut groups: BTreeMap<String, BTreeSet<u32>> = BTreeMap::new();
    let (mut dropped, mut cuts) = (Vec::new(), Vec::new());
    for s in mesh.sets() {
        match &s.members {
            SetMembers::Cells(cs) => {
                if cs.is_empty() {
                    continue;
                }
                let mut per: BTreeMap<u64, u64> = BTreeMap::new();
                for c in cs {
                    let Some(&v) = vol_of.get(c.get()) else { return unsupported(format!("set {} has a cell out of range", s.name)) };
                    *per.entry(v).or_default() += 1;
                }
                let g = groups.entry(s.name.clone()).or_default();
                for (v, n) in per {
                    if n < cells_in_volume[&v] {
                        cuts.push(format!("{}:{v}", s.name));
                    }
                    if let Ok(i) = model.volumes.binary_search_by_key(&v, |x| x.id) {
                        g.insert(i as u32);
                    }
                }
            }
            _ => dropped.push(s.name.clone()),
        }
    }
    loss_names(&mut loss, "h5m.sets_dropped", LossSeverity::Dropped, &dropped, "only cell sets become material groups; face, vertex and side sets");
    loss_names(&mut loss, "h5m.set_cuts_volume", LossSeverity::Degraded, &cuts, "a cell set holds part of a volume, the whole volume is in the group (set:volume)");
    model.groups = groups.into_iter().map(|(name, v)| DagmcGroup { name, volumes: v.into_iter().collect() }).collect();
    finish(&mut model, o, &mut loss)?;
    Ok((model, loss))
}

/// The model of sm-tess style facets: surfaces are the distinct `face` ids, `front` and `back`
/// map to forward and backward (`Region::Exterior` is no volume), and `names` maps a volume
/// (`TopoVolume` id) to its material set name (group `mat:<name>`; volumes sharing a name share
/// the group). The same sense normalisation, closed-volume check and loss codes as
/// [`model_from_mesh`] apply. A name for a volume that has no facet is [`DagmcError::Invalid`].
pub fn model_from_facets(f: &FacetInput, names: &BTreeMap<EntityId, String>, o: &DagmcOptions) -> Result<(DagmcModel, LossReport), DagmcError> {
    f.validate().map_err(|e| DagmcError::Invalid(e.to_string()))?;
    let region = |r: Region| -> Result<Option<u64>, DagmcError> {
        match r {
            Region::Exterior => Ok(None),
            Region::Volume(id) if id.kind() == EntityKind::TopoVolume => Ok(Some(id.serial())),
            Region::Volume(id) => Err(DagmcError::Invalid(format!("region {id:?} is not a TopoVolume"))),
        }
    };
    let mut raw = Vec::with_capacity(f.triangles.len());
    for (i, t) in f.triangles.iter().enumerate() {
        let id = f.face[i];
        if id.kind() != EntityKind::TopoFace {
            return Err(DagmcError::Invalid(format!("triangle {i}: face {id:?} is not a TopoFace")));
        }
        raw.push(RawTri { v: *t, surf: id.serial(), front: region(f.front[i])?, back: region(f.back[i])? });
    }
    let vertices = compact(&f.vertices, &mut raw, &[]);
    let mut model = assemble(vertices, raw, &[])?;
    let mut groups: BTreeMap<String, BTreeSet<u32>> = BTreeMap::new();
    for (id, name) in names {
        let Ok(i) = model.volumes.binary_search_by_key(&id.serial(), |x| x.id) else {
            return Err(DagmcError::Invalid(format!("name {name:?} is given for volume {id:?}, which has no facet")));
        };
        groups.entry(name.clone()).or_default().insert(i as u32);
    }
    model.groups = groups.into_iter().map(|(name, v)| DagmcGroup { name, volumes: v.into_iter().collect() }).collect();
    let mut loss = LossReport::new();
    finish(&mut model, o, &mut loss)?;
    Ok((model, loss))
}

/// Material-coverage loss items and the graveyard.
fn finish(model: &mut DagmcModel, o: &DagmcOptions, loss: &mut LossReport) -> Result<(), DagmcError> {
    let mut count = vec![0u32; model.volumes.len()];
    for g in &model.groups {
        for &v in &g.volumes {
            count[v as usize] += 1;
        }
    }
    let none: Vec<String> = model.volumes.iter().zip(&count).filter(|(_, c)| **c == 0).map(|(v, _)| v.id.to_string()).collect();
    loss_names(loss, "h5m.volume_without_material", LossSeverity::Degraded, &none, "volumes in no material group (ids)");
    let multi: Vec<String> = model.volumes.iter().zip(&count).filter(|(_, c)| **c > 1).map(|(v, _)| v.id.to_string()).collect();
    loss_names(loss, "h5m.volume_in_multiple_groups", LossSeverity::Info, &multi, "volumes in more than one material group (ids)");
    if let Some(g) = &o.graveyard {
        if model.groups.iter().any(|x| x.name == "Graveyard") {
            loss.push(LossItem::new("h5m.graveyard_exists", LossSeverity::Info, 1, "the model already has a group named Graveyard; none added"));
        } else {
            add_graveyard(model, g)?;
        }
    }
    Ok(())
}

/// Corner cycles of the six box faces: `-x +x -y +y -z +z` (corner bits: x = 1, y = 2, z = 4).
const BOX_FACES: [[usize; 4]; 6] = [[0, 2, 6, 4], [1, 3, 7, 5], [0, 1, 5, 4], [2, 3, 7, 6], [0, 1, 3, 2], [4, 5, 7, 6]];

fn add_graveyard(m: &mut DagmcModel, g: &Graveyard) -> Result<(), DagmcError> {
    if !(g.scale.is_finite() && g.scale > 1.0 && g.thickness_frac.is_finite() && g.thickness_frac > 0.0) {
        return Err(DagmcError::Invalid("graveyard needs scale > 1 and thickness_frac > 0".into()));
    }
    if m.vertices.is_empty() {
        return Err(DagmcError::Invalid("graveyard around an empty model".into()));
    }
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for p in &m.vertices {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let ext = (0..3).map(|k| hi[k] - lo[k]).fold(0.0, f64::max);
    if ext <= 0.0 {
        return Err(DagmcError::Invalid("graveyard around a model of zero extent".into()));
    }
    let centre: Vec<f64> = (0..3).map(|k| (lo[k] + hi[k]) / 2.0).collect();
    // A flat axis still gets the thickness of the longest one.
    let half: Vec<f64> = (0..3).map(|k| g.scale * ((hi[k] - lo[k]) / 2.0).max(ext * 1e-3)).collect();
    let thick = g.thickness_frac * 2.0 * half.iter().copied().fold(0.0, f64::max);
    let vol = m.volumes.len() as u32;
    m.volumes.push(DagmcVolume { id: m.volumes.last().map_or(1, |v| v.id + 1) });
    let mut next_surface = m.surfaces.last().map_or(1, |s| s.id + 1);
    for (outer, grow) in [(false, 0.0), (true, thick)] {
        let base = m.vertices.len() as u32;
        for c in 0..8 {
            let p: Vec<f64> = (0..3).map(|k| centre[k] + if c >> k & 1 == 1 { half[k] + grow } else { -(half[k] + grow) }).collect();
            m.vertices.push([p[0], p[1], p[2]]);
        }
        for (fi, quad) in BOX_FACES.iter().enumerate() {
            let sidx = m.surfaces.len() as u32;
            m.surfaces.push(DagmcSurface { id: next_surface, forward: Some(vol), backward: None });
            next_surface += 1;
            // Outward normal of this box face; the inner box faces the other way.
            let axis = fi / 2;
            let sign = if fi % 2 == 1 { 1.0 } else { -1.0 };
            let q: Vec<u32> = quad.iter().map(|&c| base + c as u32).collect();
            let mut tris = [[q[0], q[1], q[2]], [q[0], q[2], q[3]]];
            let (a, b, c) = (m.vertices[tris[0][0] as usize], m.vertices[tris[0][1] as usize], m.vertices[tris[0][2] as usize]);
            let (u, w) = ([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
            let n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]];
            let want = if outer { sign } else { -sign };
            if n[axis] * want < 0.0 {
                for t in &mut tris {
                    t.swap(1, 2);
                }
            }
            for t in tris {
                m.triangles.push(t);
                m.tri_surface.push(sidx);
            }
        }
    }
    m.groups.push(DagmcGroup { name: "Graveyard".into(), volumes: vec![vol] });
    m.groups.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(())
}
