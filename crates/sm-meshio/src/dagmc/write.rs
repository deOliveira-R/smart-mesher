//! The `.h5m` byte writer: a [`DagmcModel`] to the MOAB HDF5 layout of `H5M_FORMAT.md`.

use super::{DagmcError, DagmcModel, DagmcOptions, DagmcReport, model::model_from_mesh};
use crate::h5::{Attr, Dataset, Dtype, Group, write_file};
use crate::report::LossReport;
use sm_mesh_api::MeshRead;
use std::io::Write;

/// MOAB element type names and codes (`mhdf` `EntityType` order of `elemtypes`).
const ELEM_TYPES: [(&str, u8); 10] = [("Edge", 1), ("Tri", 2), ("Quad", 3), ("Polygon", 4), ("Tet", 5), ("Pyramid", 6), ("Prism", 7), ("Knife", 8), ("Hex", 9), ("Polyhedron", 10)];

/// Set flags: unordered set semantics, and the range-encoded contents bit.
const SET: i64 = 2;
const RANGE: i64 = 8;

/// Width of a MOAB opaque `NAME` / `CATEGORY` tag value.
const OPAQUE: usize = 32;

fn opaque(s: &str) -> Result<[u8; OPAQUE], DagmcError> {
    if s.len() >= OPAQUE {
        return Err(DagmcError::NameTooLong { name: s.to_string() });
    }
    if s.contains('\0') {
        return Err(DagmcError::Invalid(format!("name {s:?} contains a NUL byte")));
    }
    let mut b = [0u8; OPAQUE];
    b[..s.len()].copy_from_slice(s.as_bytes());
    Ok(b)
}

fn fnv(bytes: &[u8]) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for b in bytes {
        h = (h ^ u64::from(*b)).wrapping_mul(0x0100_0000_01b3);
    }
    h
}

fn invalid<T>(m: impl Into<String>) -> Result<T, DagmcError> {
    Err(DagmcError::Invalid(m.into()))
}

fn validate(m: &DagmcModel) -> Result<(), DagmcError> {
    if m.triangles.is_empty() {
        return invalid("the model has no triangles");
    }
    if m.tri_surface.len() != m.triangles.len() {
        return invalid("tri_surface and triangles differ in length");
    }
    if m.vertices.iter().flatten().any(|c| !c.is_finite()) {
        return invalid("a vertex coordinate is not finite");
    }
    if m.triangles.iter().flatten().any(|&v| v as usize >= m.vertices.len()) {
        return invalid("a triangle indexes a missing vertex");
    }
    if m.tri_surface.iter().any(|&s| s as usize >= m.surfaces.len()) {
        return invalid("a triangle indexes a missing surface");
    }
    let nv = m.volumes.len();
    if m.surfaces.iter().any(|s| s.forward.is_some_and(|v| v as usize >= nv) || s.backward.is_some_and(|v| v as usize >= nv)) {
        return invalid("a surface indexes a missing volume");
    }
    if m.groups.iter().any(|g| g.volumes.iter().any(|&v| v as usize >= nv)) {
        return invalid("a group indexes a missing volume");
    }
    if m.surfaces.windows(2).any(|w| w[0].id >= w[1].id) || m.volumes.windows(2).any(|w| w[0].id >= w[1].id) {
        return invalid("surface and volume ids must be strictly ascending");
    }
    Ok(())
}

struct SetTable {
    rows: Vec<[i64; 4]>,
    contents: Vec<u64>,
    children: Vec<u64>,
    parents: Vec<u64>,
}

impl SetTable {
    fn push(&mut self, contents: &[u64], children: &[u64], parents: &[u64], flags: i64) {
        self.contents.extend_from_slice(contents);
        self.children.extend_from_slice(children);
        self.parents.extend_from_slice(parents);
        self.rows.push([self.contents.len() as i64 - 1, self.children.len() as i64 - 1, self.parents.len() as i64 - 1, flags]);
    }
}

/// A tag definition group: class, optional extra attributes, the committed `type`, and for a
/// sparse tag (`ids`, `values`) the `id_list` and `values` datasets.
fn tag(tags: &mut Group, name: &str, class: i32, dtype: Dtype, attrs: Vec<Attr>, sparse: Option<(Vec<u64>, Dataset)>) {
    let g = tags.group(name);
    g.put_attr(Attr::i32("class", class));
    for a in attrs {
        g.put_attr(a);
    }
    g.put_type("type", dtype);
    if let Some((ids, values)) = sparse {
        g.put_dataset("id_list", Dataset::u64s(vec![ids.len() as u64], &ids));
        g.put_dataset("values", values);
    }
}

fn opaque_values(vals: &[[u8; OPAQUE]]) -> Dataset {
    Dataset::raw(Dtype::Opaque(OPAQUE as u32), vec![vals.len() as u64], vals.iter().flatten().copied().collect())
}

fn build_tree(m: &DagmcModel, o: &DagmcOptions) -> Result<(Group, [u64; 4]), DagmcError> {
    let (n_nodes, n_tris) = (m.vertices.len() as u64, m.triangles.len() as u64);
    let tri0 = n_nodes + 1;
    let set0 = tri0 + n_tris;
    let with_file_set = o.faceting_tol.is_some();
    if let Some(t) = o.faceting_tol
        && !(t.is_finite() && t > 0.0)
    {
        return invalid("faceting_tol must be finite and positive");
    }
    let file_sets = u64::from(with_file_set);
    let surf_h = |i: usize| set0 + file_sets + i as u64;
    let vol_h = |i: usize| set0 + file_sets + m.surfaces.len() as u64 + i as u64;
    let group_h = |i: usize| set0 + file_sets + (m.surfaces.len() + m.volumes.len()) as u64 + i as u64;
    let n_sets = file_sets + (m.surfaces.len() + m.volumes.len() + m.groups.len()) as u64;
    let max_id = set0 + n_sets - 1;

    // Triangle handles per surface (ascending).
    let mut by_surface: Vec<Vec<u64>> = vec![Vec::new(); m.surfaces.len()];
    for (i, &s) in m.tri_surface.iter().enumerate() {
        by_surface[s as usize].push(tri0 + i as u64);
    }
    let mut surfaces_of: Vec<Vec<u64>> = vec![Vec::new(); m.volumes.len()];
    for (i, s) in m.surfaces.iter().enumerate() {
        for v in [s.forward, s.backward].into_iter().flatten() {
            surfaces_of[v as usize].push(surf_h(i));
        }
    }

    let mut t = SetTable { rows: Vec::new(), contents: Vec::new(), children: Vec::new(), parents: Vec::new() };
    if with_file_set {
        t.push(&[1, max_id], &[], &[], RANGE);
    }
    for (i, s) in m.surfaces.iter().enumerate() {
        let parents: Vec<u64> = [s.forward, s.backward].into_iter().flatten().map(|v| vol_h(v as usize)).collect();
        t.push(&by_surface[i], &[], &parents, SET);
    }
    for sl in &surfaces_of {
        t.push(&[], sl, &[], SET);
    }
    let mut names = Vec::with_capacity(m.groups.len());
    for g in &m.groups {
        names.push(opaque(&format!("mat:{}", g.name))?);
        let vols: Vec<u64> = g.volumes.iter().map(|&v| vol_h(v as usize)).collect();
        t.push(&vols, &[], &[], SET);
    }

    let mut root = Group::new();
    let tstt = root.group("tstt");
    tstt.put_attr(Attr::u64("max_id", max_id));
    tstt.put_type("elemtypes", Dtype::EnumU8(ELEM_TYPES.iter().map(|(n, v)| (n.to_string(), *v)).collect()));

    let flat: Vec<f64> = m.vertices.iter().flatten().copied().collect();
    tstt.group("nodes").put_dataset("coordinates", Dataset::f64s(vec![n_nodes, 3], &flat).with_attr(Attr::i64("start_id", 1)));

    let conn: Vec<u64> = m.triangles.iter().flatten().map(|&v| u64::from(v) + 1).collect();
    let tri = tstt.group("elements").group("Tri3");
    tri.put_attr(Attr::raw("element_type", Dtype::EnumU8(ELEM_TYPES.iter().map(|(n, v)| (n.to_string(), *v)).collect()), vec![2]));
    tri.put_dataset("connectivity", Dataset::u64s(vec![n_tris, 3], &conn).with_attr(Attr::i64("start_id", tri0 as i64)));

    let sets = tstt.group("sets");
    let list: Vec<i64> = t.rows.iter().flatten().copied().collect();
    sets.put_dataset("list", Dataset::i64s(vec![n_sets, 4], &list).with_attr(Attr::i64("start_id", set0 as i64)));
    for (name, v) in [("contents", &t.contents), ("children", &t.children), ("parents", &t.parents)] {
        if !v.is_empty() {
            sets.put_dataset(name, Dataset::u64s(vec![v.len() as u64], v));
        }
    }
    // GLOBAL_ID is dense on sets: 1-based per category (the file set, if any, is 0).
    let mut gid: Vec<i32> = Vec::with_capacity(n_sets as usize);
    if with_file_set {
        gid.push(0);
    }
    gid.extend((1..=m.surfaces.len() as i32).chain(1..=m.volumes.len() as i32).chain(1..=m.groups.len() as i32));
    sets.group("tags").put_dataset("GLOBAL_ID", Dataset::i32s(vec![n_sets], &gid));

    let tags = tstt.group("tags");
    tag(tags, "GLOBAL_ID", 2, Dtype::I32, vec![Attr::i32("default", 0), Attr::i32("global", 0)], None);
    let geom_ids: Vec<u64> = (0..m.surfaces.len()).map(surf_h).chain((0..m.volumes.len()).map(vol_h)).collect();
    let dims: Vec<i32> = m.surfaces.iter().map(|_| 2).chain(m.volumes.iter().map(|_| 3)).collect();
    let dim_ds = Dataset::i32s(vec![dims.len() as u64], &dims);
    tag(tags, "GEOM_DIMENSION", 1, Dtype::I32, vec![Attr::i32("default", -1), Attr::i32("global", -1)], Some((geom_ids.clone(), dim_ds)));
    let cat = |s: &str| opaque(s);
    let mut cats = Vec::new();
    for _ in &m.surfaces {
        cats.push(cat("Surface")?);
    }
    for _ in &m.volumes {
        cats.push(cat("Volume")?);
    }
    for _ in &m.groups {
        cats.push(cat("Group")?);
    }
    let cat_ids: Vec<u64> = geom_ids.iter().copied().chain((0..m.groups.len()).map(group_h)).collect();
    tag(tags, "CATEGORY", 1, Dtype::Opaque(OPAQUE as u32), vec![], Some((cat_ids, opaque_values(&cats))));
    if !m.groups.is_empty() {
        let ids: Vec<u64> = (0..m.groups.len()).map(group_h).collect();
        tag(tags, "NAME", 1, Dtype::Opaque(OPAQUE as u32), vec![], Some((ids, opaque_values(&names))));
    }
    let sense: Vec<u8> = m
        .surfaces
        .iter()
        .flat_map(|s| {
            let h = |v: Option<u32>| v.map_or(0, |v| vol_h(v as usize));
            [h(s.forward), h(s.backward)]
        })
        .flat_map(u64::to_le_bytes)
        .collect();
    let sense_ids: Vec<u64> = (0..m.surfaces.len()).map(surf_h).collect();
    tag(
        tags,
        "GEOM_SENSE_2",
        1,
        Dtype::ArrayU64(2),
        vec![Attr::raw("default", Dtype::ArrayU64(2), vec![0; 16]), Attr::i32("is_handle", 1)],
        Some((sense_ids, Dataset::raw(Dtype::ArrayU64(2), vec![m.surfaces.len() as u64], sense))),
    );
    if let Some(tol) = o.faceting_tol {
        tag(tags, "FACETING_TOL", 1, Dtype::F64, vec![], Some((vec![set0], Dataset::f64s(vec![1], &[tol]))));
    }
    Ok((root, [n_tris, m.surfaces.len() as u64, m.volumes.len() as u64, m.groups.len() as u64]))
}

/// Write `m` as a DAGMC `.h5m` file. The bytes are a pure function of `m` and `o`.
///
/// Handles ascend: nodes, then `Tri3`, then sets (an optional file set carrying
/// `FACETING_TOL`, the surfaces, the volumes, the `mat:` groups). The model is validated for
/// index ranges only; use [`model_from_mesh`] for the closed-volume check. Group names longer
/// than 31 bytes (with the `mat:` prefix) are [`DagmcError::NameTooLong`], never truncated.
pub fn write_h5m(m: &DagmcModel, o: &DagmcOptions, w: &mut dyn Write) -> Result<DagmcReport, DagmcError> {
    validate(m)?;
    let (root, [triangles, surfaces, volumes, groups]) = build_tree(m, o)?;
    let bytes = write_file(root)?;
    w.write_all(&bytes)?;
    Ok(DagmcReport { triangles, surfaces, volumes, groups, bytes: bytes.len() as u64, hash: fnv(&bytes), loss: LossReport::new() })
}

/// [`model_from_mesh`] followed by [`write_h5m`]; the report's `loss` is the model's.
pub fn export_h5m(mesh: &dyn MeshRead, o: &DagmcOptions, w: &mut dyn Write) -> Result<DagmcReport, DagmcError> {
    let (model, loss) = model_from_mesh(mesh, o)?;
    let mut r = write_h5m(&model, o, w)?;
    r.loss = loss;
    Ok(r)
}
