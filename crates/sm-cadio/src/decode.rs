//! [`read_step`]: STEP bytes to a [`BrepModel`], independent of any topology store.

use std::collections::{BTreeMap, BTreeSet};

use rayon::prelude::*;
use sm_core::{Point3, Transform};
use sm_topo_api::{Sense, ShellKind};

use crate::args::{Args, Ents};
use crate::assembly;
use crate::edge::{fit_edge, with_id};
use crate::error::{OpenEdge, StepError};
use crate::gcodec::Geo;
use crate::model::*;
use crate::part21::{self, Entity, StepFile};
use crate::units::{self, ContextUnits, FileLength};
use crate::weld::{weld_edges, weld_vertices};

/// Smallest cap on placed solid instances of a file.
const INSTANCE_FLOOR: usize = 1024;
/// Placed instances allowed per entity of the file.
const INSTANCES_PER_ENTITY: usize = 8;
/// Smallest cap on the cells copied by flattening (about 6 MB).
const CLONE_CELLS_FLOOR: usize = 1 << 16;
/// Copied cells allowed per entity of the file.
const CLONE_CELLS_PER_ENTITY: usize = 4;

/// Read a STEP file into plain B-rep data.
///
/// Pure: no clock, no environment, no files, no topology store. Per-face geometry is decoded
/// under rayon and collected by index, so the result is identical for any thread count.
///
/// Supported: `MANIFOLD_SOLID_BREP` and `BREP_WITH_VOIDS` (also through
/// `ORIENTED_CLOSED_SHELL`) with `ADVANCED_FACE`/`FACE_SURFACE` faces bounded by `EDGE_LOOP`s;
/// `LINE`, `CIRCLE`, `ELLIPSE`, `B_SPLINE_CURVE_WITH_KNOTS` (also rational) curves, possibly
/// wrapped in `SURFACE_CURVE`/`SEAM_CURVE` (the pcurves are ignored); `PLANE`,
/// `CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`, `SPHERICAL_SURFACE`, `TOROIDAL_SURFACE` and
/// `B_SPLINE_SURFACE_WITH_KNOTS` (also rational). Sheets are skipped with a note.
///
/// # Errors
/// Any [`StepError`]; see the variants. A file without a length unit is an error unless
/// [`ReadOptions::assume_unit`] is set.
pub fn read_step(bytes: &[u8], opts: &ReadOptions) -> Result<BrepModel, StepError> {
    let file = part21::parse(bytes)?;
    read_file(&file, opts)
}

/// Solid entities of the file in ascending id order.
pub(crate) fn solid_entities(file: &StepFile) -> Vec<(u64, SolidKind)> {
    file.entities
        .iter()
        .filter_map(|(id, e)| {
            if e.is("BREP_WITH_VOIDS") {
                Some((*id, SolidKind::WithVoids))
            } else if e.is("MANIFOLD_SOLID_BREP") {
                Some((*id, SolidKind::Manifold))
            } else if e.is("FACETED_BREP") {
                Some((*id, SolidKind::Faceted))
            } else {
                None
            }
        })
        .collect()
}

/// `SHELL_BASED_SURFACE_MODEL` entities in ascending id order.
pub(crate) fn sheet_entities(file: &StepFile) -> Vec<u64> {
    file.entities.iter().filter(|(_, e)| e.is("SHELL_BASED_SURFACE_MODEL")).map(|(id, _)| *id).collect()
}

/// Counts of entity keywords that are not imported, by category.
pub(crate) struct Skipped {
    pub sheets: BTreeMap<String, usize>,
    pub unsupported: BTreeMap<String, usize>,
    pub attributes: BTreeMap<&'static str, usize>,
    pub assemblies: bool,
}

pub(crate) fn scan_skipped(file: &StepFile) -> Skipped {
    let mut s = Skipped { sheets: BTreeMap::new(), unsupported: BTreeMap::new(), attributes: BTreeMap::new(), assemblies: false };
    for e in file.entities.values() {
        let names: Vec<&str> = match e {
            Entity::Simple(r) => vec![r.name.as_str()],
            Entity::Complex(v) => v.iter().map(|r| r.name.as_str()).collect(),
        };
        for n in names {
            match n {
                "SHELL_BASED_SURFACE_MODEL" | "OPEN_SHELL" | "MANIFOLD_SURFACE_SHAPE_REPRESENTATION" | "GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION" => *s.sheets.entry(n.to_owned()).or_default() += 1,
                "TESSELLATED_SOLID" | "TRIANGULATED_FACE" => *s.unsupported.entry(n.to_owned()).or_default() += 1,
                "MAPPED_ITEM" | "NEXT_ASSEMBLY_USAGE_OCCURRENCE" => s.assemblies = true,
                "COLOUR_RGB" | "DRAUGHTING_PRE_DEFINED_COLOUR" => *s.attributes.entry("colors").or_default() += 1,
                "STYLED_ITEM" | "OVER_RIDING_STYLED_ITEM" => *s.attributes.entry("styled items").or_default() += 1,
                "PRESENTATION_LAYER_ASSIGNMENT" => *s.attributes.entry("layers").or_default() += 1,
                _ if n.starts_with("TESSELLATED_") => *s.attributes.entry("tessellation").or_default() += 1,
                _ if n.contains("TOLERANCE") && n != "UNCERTAINTY_MEASURE_WITH_UNIT" || n.starts_with("DIMENSIONAL_") || n.starts_with("DATUM") => *s.attributes.entry("PMI").or_default() += 1,
                _ => {}
            }
        }
    }
    s
}

pub(crate) fn read_file(file: &StepFile, opts: &ReadOptions) -> Result<BrepModel, StepError> {
    if !(opts.weld_tol.is_finite() && opts.weld_tol >= 0.0) {
        return Err(StepError::BadOption(format!("weld tolerance {} must be finite and non-negative", opts.weld_tol)));
    }
    let all = solid_entities(file);
    let selected: Vec<(u64, SolidKind)> = match &opts.solids {
        None => all.clone(),
        Some(ords) => {
            let mut sel = Vec::new();
            for &o in ords {
                let Some(s) = all.get(o as usize) else {
                    return Err(StepError::BadOption(format!("solid ordinal {o} out of range: the file has {} solids", all.len())));
                };
                sel.push(*s);
            }
            sel.sort_unstable_by_key(|s| s.0);
            sel.dedup_by_key(|s| s.0);
            sel
        }
    };
    let mut selected = selected;
    if opts.sheets {
        selected.extend(sheet_entities(file).into_iter().map(|id| (id, SolidKind::Sheet)));
    }
    let mut notes: Vec<Note> = Vec::new();
    let skipped = scan_skipped(file);
    if !skipped.sheets.is_empty() && !opts.sheets {
        let list: Vec<String> = skipped.sheets.iter().map(|(n, c)| format!("{c} {n}")).collect();
        notes.push(Note::info(NoteCode::SheetsSkipped, format!("sheets not imported: {}", list.join(", ")), Vec::new()));
    }
    if !skipped.unsupported.is_empty() {
        let list: Vec<String> = skipped.unsupported.iter().map(|(n, c)| format!("{c} {n}")).collect();
        notes.push(Note::warn(NoteCode::UnsupportedEntity, format!("not imported (unsupported): {}", list.join(", ")), Vec::new()));
    }
    if !skipped.attributes.is_empty() {
        let list: Vec<String> = skipped.attributes.iter().map(|(n, c)| format!("{c} {n}")).collect();
        notes.push(Note::info(NoteCode::Attributes, format!("attribute entities not imported: {}", list.join(", ")), Vec::new()));
    }
    let ctx_of = units::contexts_by_item(file);
    let default_ctx = ctx_of.values().next().copied();
    let ents = Ents::new(file);
    // Assembly graph: only when the file has assembly entities.
    let asm = (skipped.assemblies && opts.flatten_assemblies).then(|| assembly::scan(file));
    let mut solid_rep: BTreeMap<u64, u64> = BTreeMap::new();
    let mut rep_um: BTreeMap<u64, f64> = BTreeMap::new();
    if let Some(a) = &asm {
        for (rep, (items, ctx)) in &a.reps {
            for it in items {
                solid_rep.entry(*it).or_insert(*rep);
            }
            if let Some(c) = ctx
                && let Ok(cu) = units::context_units(file, *c)
                && let Some(l) = cu.length
            {
                rep_um.insert(*rep, l.um);
            }
        }
    }
    // Units are resolved first (cheap, sequential, deterministic first error).
    let mut preps: Vec<(u64, SolidKind, UnitReport, Vec<Transform>)> = Vec::with_capacity(selected.len());
    let mut placement_failures = 0usize;
    // Flattening copies a solid per placement: bound the count by the file's size, so a few
    // links per level cannot multiply into millions of solids (and, below, the cloned cells).
    let instance_limit = INSTANCE_FLOOR.max(INSTANCES_PER_ENTITY * file.entities.len());
    let clone_limit = CLONE_CELLS_FLOOR.max(CLONE_CELLS_PER_ENTITY * file.entities.len());
    let mut instance_total = 0usize;
    let mut unit_groups: Vec<(String, Vec<u64>)> = Vec::new();
    for (id, kind) in &selected {
        let ctx = ctx_of.get(id).copied().or(default_ctx);
        let cu = match ctx {
            Some(c) => units::context_units(file, c)?,
            None => ContextUnits { length: None, angle: units::context_units_default(), uncertainty: None },
        };
        let (length, declared) = match (&cu.length, opts.assume_unit) {
            (Some(l), _) => (l.clone(), true),
            (None, Some(u)) => (FileLength { name: format!("assumed {}", u.symbol()), um: u.micrometres() }, false),
            (None, None) => return Err(StepError::Units(format!("solid #{id}: the file declares no length unit (pass assume_unit to read it anyway)"))),
        };
        let target_um = opts.to_unit.map_or(length.um, |u| u.micrometres());
        let scale = if target_um == length.um { 1.0 } else { length.um / target_um };
        let report = UnitReport {
            length_name: length.name.clone(),
            length_um: length.um,
            declared,
            angle_name: cu.angle.name.clone(),
            angle_rad: cu.angle.rad,
            uncertainty: cu.uncertainty,
            scale,
        };
        let key = format!(
            "length unit {}{}{}; plane angle unit {}; {}",
            units::describe_length(&length),
            if declared { "" } else { " (not declared in the file)" },
            cu.uncertainty.map_or(String::new(), |u| format!("; uncertainty {u:e}")),
            cu.angle.name,
            match (opts.to_unit, scale == 1.0) {
                (Some(u), false) => format!("converted to {} (x{scale})", u.symbol()),
                (Some(u), true) => format!("already in {}", u.symbol()),
                (None, _) => "no conversion".to_owned(),
            }
        );
        match unit_groups.iter_mut().find(|(k, _)| *k == key) {
            Some((_, v)) => v.push(*id),
            None => unit_groups.push((key, vec![*id])),
        }
        let placements = match (&asm, solid_rep.get(id)) {
            (Some(a), Some(rep)) => {
                let own = length.um;
                let scale_of = |r: u64| rep_um.get(&r).copied().unwrap_or(own) / target_um;
                let mut walk = assembly::Walk::new(instance_limit.saturating_sub(instance_total));
                match assembly::instances(file, a, *rep, &scale_of, &mut walk) {
                    Ok(v) if !v.is_empty() => {
                        instance_total += v.len();
                        v
                    }
                    Err(e @ StepError::TooManyInstances { .. }) => return Err(e),
                    _ => {
                        placement_failures += 1;
                        vec![Transform::IDENTITY]
                    }
                }
            }
            _ => vec![Transform::IDENTITY],
        };
        preps.push((*id, *kind, report, placements));
    }
    // Solids that share their body (same entity arguments after the name, same units) are
    // decoded once; later ones are charged to one file-wide clone budget and cloned. This keeps
    // a fan of tiny solid records over one shell from multiplying the decoded shell.
    let mut group_of: Vec<usize> = Vec::with_capacity(preps.len());
    let mut firsts: Vec<usize> = Vec::new();
    {
        let mut seen: BTreeMap<String, usize> = BTreeMap::new();
        for (i, (id, kind, report, _)) in preps.iter().enumerate() {
            let body = match file.entities.get(id) {
                Some(Entity::Simple(r)) => format!("{:?}", &r.params[r.params.len().min(1)..]),
                Some(Entity::Complex(v)) => format!("{:?}", v.iter().map(|r| (&r.name, &r.params[r.params.len().min(1)..])).collect::<Vec<_>>()),
                None => format!("missing {id}"),
            };
            let key = format!("{kind:?}|{body}|{report:?}");
            let g = *seen.entry(key).or_insert_with(|| {
                firsts.push(i);
                firsts.len() - 1
            });
            group_of.push(g);
        }
    }
    type BaseResult = Result<(Solid, Vec<Note>, (usize, usize)), StepError>;
    let decode_base = |i: &usize| -> BaseResult {
        let (id, kind, report, _) = &preps[*i];
        let mut local = Vec::new();
        let (solid, stats) = read_solid(ents, *id, *kind, report.clone(), opts, &mut local)?;
        Ok((solid, local, stats))
    };
    let bases: Vec<BaseResult> = if firsts.len() >= 8 { firsts.par_iter().map(decode_base).collect() } else { firsts.iter().map(decode_base).collect() };
    let mut bases: Vec<(Solid, Vec<Note>, (usize, usize))> = bases.into_iter().collect::<Result<_, _>>()?;
    let mut solids = Vec::with_capacity(preps.len());
    let (mut welded_v, mut welded_e) = (0usize, 0usize);
    let mut instances = 0usize;
    let mut clone_total = 0usize;
    for (_, base_notes, stats) in bases.iter_mut() {
        notes.append(base_notes);
        welded_v += stats.0;
        welded_e += stats.1;
    }
    let mut used = vec![false; bases.len()];
    for (i, (id, _, _, placements)) in preps.iter().enumerate() {
        let g = group_of[i];
        let base = &bases[g].0;
        let cells = base.vertices.len() + base.curves.len() + base.surfaces.len() + base.edges.len() + base.faces.len() + 1;
        // The first placement of a group's first solid is input-proportional; the rest are copies.
        let copies = placements.len() - usize::from(!used[g]);
        used[g] = true;
        clone_total = clone_total.saturating_add(copies.saturating_mul(cells));
        if clone_total > clone_limit {
            return Err(StepError::TooManyInstances { id: *id, limit: clone_limit / cells });
        }
        let name = file.entities.get(id).and_then(|e| e.rec("MANIFOLD_SOLID_BREP").or_else(|| e.first())).and_then(|r| match r.params.first() {
            Some(part21::Param::Str(s)) => Some(s.clone()),
            _ => None,
        });
        let stamp = |inst: &mut Solid, k: usize| {
            inst.id = *id;
            if let Some(n) = &name {
                inst.name = n.clone();
            }
            inst.instance = k as u32;
        };
        for (k, t) in placements.iter().enumerate() {
            let mut inst = base.clone();
            stamp(&mut inst, k);
            if *t != Transform::IDENTITY && assembly::transform_solid(&mut inst, t).is_err() {
                placement_failures += 1;
                inst = base.clone();
                stamp(&mut inst, k);
            }
            solids.push(inst);
            instances += 1;
        }
    }
    let mut unit_notes = Vec::new();
    if selected.is_empty() {
        // Still report the file's units when it holds no solid.
        if let Some(c) = default_ctx {
            let cu = units::context_units(file, c)?;
            if let Some(l) = cu.length {
                unit_notes.push(Note::info(NoteCode::Units, format!("length unit {}; plane angle unit {}", units::describe_length(&l), cu.angle.name), Vec::new()));
            }
        }
    }
    for (msg, ids) in unit_groups {
        unit_notes.push(Note::info(NoteCode::Units, msg, ids));
    }
    notes.splice(0..0, unit_notes);
    if welded_v + welded_e > 0 {
        notes.push(Note::info(NoteCode::Welded, format!("welded {welded_v} vertices and {welded_e} duplicate edges"), Vec::new()));
    }
    if skipped.assemblies && !solids.is_empty() {
        match &asm {
            Some(a) if !a.links.is_empty() && a.unsupported.is_empty() && placement_failures == 0 => {
                notes.push(Note::info(NoteCode::AssemblyFlattened, format!("assembly flattened: {instances} solid instance(s) of {} solid(s) placed in the root representation", selected.len()), Vec::new()));
            }
            _ => {
                let why = match &asm {
                    None => "assembly flattening is off".to_owned(),
                    Some(a) if a.links.is_empty() && a.unsupported.is_empty() => "the file has assembly entities but no placement this reader understands".to_owned(),
                    Some(a) if !a.unsupported.is_empty() => format!("{} (#{})", a.unsupported[0].1, a.unsupported[0].0),
                    Some(_) => format!("{placement_failures} placement(s) could not be applied (mirrored or inconsistent)"),
                };
                notes.push(Note::warn(NoteCode::AssemblyTransformsIgnored, format!("the file is an assembly: solids are imported in their local coordinates ({why})"), Vec::new()));
            }
        }
    }
    Ok(BrepModel { schema: file.header.schemas.first().cloned().unwrap_or_default(), solids, notes })
}

// ---------------------------------------------------------------------------------------------
// One solid

struct RawBound {
    outer: bool,
    orientation: bool,
    /// (EDGE_CURVE id, oriented edge orientation)
    edges: Vec<(u64, bool)>,
    /// `VERTEX_LOOP`: (loop id, vertex id). Such a bound has no edges.
    vertex_loop: Option<(u64, u64)>,
}

struct RawFace {
    id: u64,
    shell: usize,
    reversed: bool,
    surface: u64,
    same_sense: bool,
    bounds: Vec<RawBound>,
}

struct RawEdge {
    start: u64,
    end: u64,
    curve: u64,
    same_sense: bool,
}

/// The closed shell behind `id` and whether an `ORIENTED_CLOSED_SHELL` reverses it.
fn shell_of(ents: Ents<'_>, id: u64, by: u64) -> Result<(u64, bool), StepError> {
    let mut cur = id;
    let mut by = by;
    let mut reversed = false;
    for _ in 0..8 {
        let e = ents.get(cur, by)?;
        if let Some(rec) = e.rec("ORIENTED_CLOSED_SHELL") {
            let a = Args { id: cur, rec };
            reversed ^= !a.boolean(3)?;
            by = cur;
            cur = a.reference(2)?;
            continue;
        }
        if e.is("CLOSED_SHELL") {
            return Ok((cur, reversed));
        }
        return Err(StepError::WrongType { id: cur, expected: "CLOSED_SHELL".into(), found: e.display_name() });
    }
    Err(StepError::Cycle { id })
}

fn read_solid(ents: Ents<'_>, id: u64, kind: SolidKind, units: UnitReport, opts: &ReadOptions, notes: &mut Vec<Note>) -> Result<(Solid, (usize, usize)), StepError> {
    if kind == SolidKind::Faceted {
        return read_faceted(ents, id, units, opts);
    }
    let entity = ents.get(id, id)?;
    // Name and shells: outer shell and voids for a solid (simple or complex spelling), the
    // shells of a surface model for a sheet.
    let (name, shell_srcs): (String, Vec<(u64, bool)>) = if kind == SolidKind::Sheet {
        let a = ents.rec(id, id, "SHELL_BASED_SURFACE_MODEL")?;
        (a.string(0), a.refs(1)?.into_iter().map(|s| (s, false)).collect())
    } else {
        let (name, outer_ref, void_refs) = match entity {
            Entity::Simple(r) => {
                let a = Args { id, rec: r };
                let voids = if kind == SolidKind::WithVoids { a.refs(2)? } else { Vec::new() };
                (a.string(0), a.reference(1)?, voids)
            }
            Entity::Complex(_) => {
                let m = entity.rec("MANIFOLD_SOLID_BREP").ok_or(StepError::Unsupported { id, entity: entity.display_name() })?;
                let a = Args { id, rec: m };
                let voids = match entity.rec("BREP_WITH_VOIDS") {
                    Some(r) => Args { id, rec: r }.refs(0)?,
                    None => Vec::new(),
                };
                (a.string(0), a.reference(1)?, voids)
            }
        };
        // ---- shells and faces (topology only; geometry is decoded afterwards)
        let mut srcs: Vec<(u64, bool)> = vec![shell_of(ents, outer_ref, id)?];
        for v in void_refs {
            srcs.push(shell_of(ents, v, id)?);
        }
        (name, srcs)
    };
    if shell_srcs.is_empty() {
        return Err(StepError::BadParam { id, entity: "SHELL_BASED_SURFACE_MODEL".into(), index: 1, msg: "a surface model needs at least one shell".into() });
    }
    let shell_names: &[&str] = if kind == SolidKind::Sheet { &["CLOSED_SHELL", "OPEN_SHELL"] } else { &["CLOSED_SHELL"] };
    let mut face_ids: BTreeMap<u64, (usize, bool)> = BTreeMap::new();
    for (si, (shell_id, reversed)) in shell_srcs.iter().enumerate() {
        let a = ents.one_of(*shell_id, id, shell_names)?;
        for f in a.refs(1)? {
            face_ids.entry(f).or_insert((si, *reversed));
        }
    }
    let mut raw_faces: Vec<RawFace> = Vec::with_capacity(face_ids.len());
    let mut raw_edges: BTreeMap<u64, RawEdge> = BTreeMap::new();
    let mut vertex_points: BTreeMap<u64, u64> = BTreeMap::new();
    for (&fid, &(shell, reversed)) in &face_ids {
        let a = ents.one_of(fid, id, &["ADVANCED_FACE", "FACE_SURFACE"])?;
        let bound_ids = a.refs(1)?;
        if bound_ids.is_empty() {
            return Err(a.bad(1, "a face needs at least one bound"));
        }
        let surface = a.reference(2)?;
        let same_sense = a.boolean(3)?;
        let mut bounds = Vec::with_capacity(bound_ids.len());
        for b in bound_ids {
            let ba = ents.one_of(b, fid, &["FACE_OUTER_BOUND", "FACE_BOUND"])?;
            let outer = ba.rec.name == "FACE_OUTER_BOUND";
            let orientation = ba.boolean(2)?;
            let loop_id = ba.reference(1)?;
            let le = ents.get(loop_id, b)?;
            let Some(lrec) = le.rec("EDGE_LOOP") else {
                if let Some(vl) = le.rec("VERTEX_LOOP") {
                    let v = Args { id: loop_id, rec: vl }.reference(1)?;
                    if let std::collections::btree_map::Entry::Vacant(s) = vertex_points.entry(v) {
                        let va = ents.rec(v, loop_id, "VERTEX_POINT")?;
                        s.insert(va.reference(1)?);
                    }
                    bounds.push(RawBound { outer, orientation, edges: Vec::new(), vertex_loop: Some((loop_id, v)) });
                    continue;
                }
                return Err(match le.first() {
                    Some(r) if r.name == "POLY_LOOP" => StepError::Unsupported { id: loop_id, entity: r.name.clone() },
                    _ => StepError::WrongType { id: loop_id, expected: "EDGE_LOOP".into(), found: le.display_name() },
                });
            };
            let la = Args { id: loop_id, rec: lrec };
            let mut edges = Vec::new();
            for oe in la.refs(1)? {
                let (edge_id, orient) = oriented_edge(ents, oe, loop_id)?;
                if let std::collections::btree_map::Entry::Vacant(slot) = raw_edges.entry(edge_id) {
                    let ea = ents.rec(edge_id, oe, "EDGE_CURVE")?;
                    let (v0, v1) = (ea.reference(1)?, ea.reference(2)?);
                    for v in [v0, v1] {
                        if let std::collections::btree_map::Entry::Vacant(s) = vertex_points.entry(v) {
                            let va = ents.rec(v, edge_id, "VERTEX_POINT")?;
                            s.insert(va.reference(1)?);
                        }
                    }
                    slot.insert(RawEdge { start: v0, end: v1, curve: ea.reference(3)?, same_sense: ea.boolean(4)? });
                }
                edges.push((edge_id, orient));
            }
            if edges.is_empty() {
                return Err(la.bad(1, "an edge loop needs at least one edge"));
            }
            bounds.push(RawBound { outer, orientation, edges, vertex_loop: None });
        }
        raw_faces.push(RawFace { id: fid, shell, reversed, surface, same_sense, bounds });
    }

    // ---- geometry
    let geo = Geo { ents, scale: units.scale, angle: units.angle_rad };
    let v_ids: Vec<u64> = vertex_points.keys().copied().collect();
    let v_pts: Vec<Point3> = v_ids.iter().map(|v| geo_point(&geo, vertex_points[v], *v)).collect::<Result<_, _>>()?;
    // Underlying curve entity of each edge, ascending.
    let mut edge_curve: BTreeMap<u64, u64> = BTreeMap::new();
    for (eid, e) in &raw_edges {
        edge_curve.insert(*eid, geo.curve_ref(e.curve, *eid)?);
    }
    let curve_ids: Vec<u64> = edge_curve.values().copied().collect::<BTreeSet<_>>().into_iter().collect();
    let surface_ids: Vec<u64> = raw_faces.iter().map(|f| f.surface).collect::<BTreeSet<_>>().into_iter().collect();
    let curve_defs = par_decode(&curve_ids, |c| geo.curve(c, c))?;
    let surface_defs = par_decode(&surface_ids, |s| geo.surface(s, s))?;
    let mut curves: Vec<CurveRec> = curve_ids.iter().zip(curve_defs).map(|(id, def)| CurveRec { id: *id, def }).collect();
    let surfaces: Vec<SurfaceRec> = surface_ids.iter().zip(surface_defs).map(|(id, def)| SurfaceRec { id: *id, def }).collect();
    let curve_ix: BTreeMap<u64, usize> = curve_ids.iter().enumerate().map(|(i, c)| (*c, i)).collect();
    let surface_ix: BTreeMap<u64, usize> = surface_ids.iter().enumerate().map(|(i, c)| (*c, i)).collect();

    // ---- vertex weld
    let rep = weld_vertices(&v_pts, opts.weld_tol);
    let mut compact = vec![usize::MAX; v_ids.len()];
    let mut vertices: Vec<Vertex> = Vec::new();
    for (i, r) in rep.iter().enumerate() {
        if *r == i {
            compact[i] = vertices.len();
            vertices.push(Vertex { id: v_ids[i], point: v_pts[i] });
        }
    }
    let v_ix: BTreeMap<u64, usize> = v_ids.iter().enumerate().map(|(i, v)| (*v, compact[rep[i]])).collect();
    let welded_vertices = v_ids.len() - vertices.len();

    // ---- edges
    let unc = units.uncertainty.map_or(0.0, |u| u * units.scale);
    let limit = opts.weld_tol.max(10.0 * unc);
    let mut edges: Vec<Edge> = Vec::with_capacity(raw_edges.len());
    let mut edge_ids: Vec<u64> = Vec::with_capacity(raw_edges.len());
    for (eid, e) in &raw_edges {
        let (s, t) = (v_ix[&e.start], v_ix[&e.end]);
        let ci = curve_ix[&edge_curve[eid]];
        let fit = fit_edge(&curves[ci].def, vertices[s].point, vertices[t].point, e.same_sense, s == t, limit).map_err(|er| with_id(er, *eid))?;
        if fit.gap > opts.weld_tol {
            notes.push(Note::warn(NoteCode::EdgeGap, format!("edge #{eid}: a vertex lies {:e} from the edge curve (weld tolerance {:e})", fit.gap, opts.weld_tol), vec![*eid]));
        }
        if fit.repaired {
            notes.push(Note::warn(NoteCode::EdgeSense, format!("edge #{eid}: same_sense contradicts the vertices; the sense was taken from the geometry"), vec![*eid]));
        }
        edges.push(Edge { id: *eid, start: s, end: t, curve: ci, range: fit.range, sense: fit.sense, merged: Vec::new() });
        edge_ids.push(*eid);
    }
    let merge = weld_edges(&edges, &curves, opts.weld_tol);
    let mut new_ix = vec![usize::MAX; edges.len()];
    let mut kept: Vec<Edge> = Vec::new();
    for (i, e) in edges.iter().enumerate() {
        if merge[i].0 == i {
            new_ix[i] = kept.len();
            kept.push(e.clone());
        }
    }
    let mut welded_edges = 0;
    for (i, (k, _)) in merge.iter().enumerate() {
        if *k != i {
            new_ix[i] = new_ix[*k];
            kept[new_ix[*k]].merged.push(edge_ids[i]);
            welded_edges += 1;
        }
    }
    // (kept edge index, relative direction) by STEP edge id
    let edge_map: BTreeMap<u64, (usize, Sense)> = edge_ids.iter().enumerate().map(|(i, id)| (*id, (new_ix[i], merge[i].1))).collect();

    // ---- faces
    let mut faces: Vec<Face> = Vec::with_capacity(raw_faces.len());
    for rf in &raw_faces {
        let mut order: Vec<usize> = (0..rf.bounds.len()).collect();
        // The outer bound first (stable: file order among the others).
        if let Some(p) = rf.bounds.iter().position(|b| b.outer) {
            order.remove(p);
            order.insert(0, p);
        }
        let mut loops = Vec::with_capacity(order.len());
        for &bi in &order {
            let b = &rf.bounds[bi];
            if let Some((loop_id, v)) = b.vertex_loop {
                let lp = pole_loop(&surfaces[surface_ix[&rf.surface]].def, v_ix[&v], loop_id, opts.weld_tol, &mut vertices, &mut curves, &mut kept)?;
                loops.push(lp);
                continue;
            }
            // `.F.` on the bound or a reversed shell reverses the loop.
            let flip = !b.orientation ^ rf.reversed;
            let mut lp: Vec<Coedge> = b
                .edges
                .iter()
                .map(|(eid, orient)| {
                    let (ei, rel) = edge_map[eid];
                    let forward = *orient == (rel == Sense::Forward);
                    Coedge { edge: ei, sense: if forward { Sense::Forward } else { Sense::Reversed } }
                })
                .collect();
            if flip {
                lp.reverse();
                for c in &mut lp {
                    c.sense = c.sense.flip();
                }
            }
            loops.push(lp);
        }
        let sense = if rf.same_sense ^ rf.reversed { Sense::Forward } else { Sense::Reversed };
        faces.push(Face { id: rf.id, surface: surface_ix[&rf.surface], sense, loops, shell: rf.shell });
    }
    // ---- shells and closure
    let mut shells: Vec<Shell> = shell_srcs
        .iter()
        .enumerate()
        .map(|(i, (sid, _))| Shell { id: *sid, kind: if i == 0 || kind == SolidKind::Sheet { ShellKind::Outer } else { ShellKind::Void }, faces: Vec::new() })
        .collect();
    for (fi, f) in faces.iter().enumerate() {
        shells[f.shell].faces.push(fi);
    }
    if kind != SolidKind::Sheet {
        check_closed(id, &shells, &faces, &kept)?;
    }
    let mut solid = Solid { id, instance: 0, name, kind, units, vertices, curves, surfaces, edges: kept, faces, shells };
    crate::split::split_closed_nurbs(&mut solid, opts.weld_tol, notes);
    Ok((solid, (welded_vertices, welded_edges)))
}

/// A `VERTEX_LOOP` on a sphere at one of its poles bounds the whole sphere: synthesize the
/// opposite pole vertex, a meridian seam edge (south to north, at `u = 0`) and the seam-pair loop
/// the pole rule then closes with two `DEGENERATE` edges.
fn pole_loop(surface: &sm_geom_api::SurfaceDef, pole: usize, loop_id: u64, tol: f64, vertices: &mut Vec<Vertex>, curves: &mut Vec<CurveRec>, edges: &mut Vec<Edge>) -> Result<Vec<Coedge>, StepError> {
    use core::f64::consts::FRAC_PI_2;
    let sm_geom_api::SurfaceDef::Sphere(sp) = surface else {
        return Err(StepError::Unsupported { id: loop_id, entity: "VERTEX_LOOP".into() });
    };
    let local = sp.frame.to_local(vertices[pole].point);
    let r = sp.radius;
    if local.x.hypot(local.y) > tol.max(1e-9 * r) {
        return Err(StepError::Unsupported { id: loop_id, entity: "VERTEX_LOOP away from a pole of the sphere".into() });
    }
    let north = local.z > 0.0;
    let other = vertices.len();
    vertices.push(Vertex { id: loop_id, point: sp.frame.origin + sp.frame.z * if north { -r } else { r } });
    let (south, north_i) = if north { (other, pole) } else { (pole, other) };
    let frame = sm_geom_api::Frame::from_zx(sp.frame.origin, sp.frame.y * -1.0, sp.frame.x).map_err(|e| StepError::GeometryMismatch { id: loop_id, msg: e.to_string() })?;
    let circle = sm_geom_api::Circle::new(frame, r).map_err(|e| StepError::GeometryMismatch { id: loop_id, msg: e.to_string() })?;
    let curve = curves.len();
    curves.push(CurveRec { id: loop_id, def: sm_geom_api::CurveDef::Circle(circle) });
    let edge = edges.len();
    edges.push(Edge { id: loop_id, start: south, end: north_i, curve, range: sm_geom_api::Interval::new(-FRAC_PI_2, FRAC_PI_2), sense: Sense::Forward, merged: Vec::new() });
    Ok(vec![Coedge { edge, sense: Sense::Forward }, Coedge { edge, sense: Sense::Reversed }])
}

/// Every edge must cancel (forward uses minus reverse uses) in every shell.
pub(crate) fn check_closed(solid: u64, shells: &[Shell], faces: &[Face], edges: &[Edge]) -> Result<(), StepError> {
    for sh in shells {
        let mut net: BTreeMap<usize, i64> = BTreeMap::new();
        for &fi in &sh.faces {
            for lp in &faces[fi].loops {
                for c in lp {
                    *net.entry(c.edge).or_default() += if c.sense == Sense::Forward { 1 } else { -1 };
                }
            }
        }
        let mut issues: Vec<OpenEdge> = net.iter().filter(|(_, n)| **n != 0).map(|(e, n)| OpenEdge { edge: edges[*e].id, net_uses: *n }).collect();
        if !issues.is_empty() {
            issues.sort_by_key(|i| i.edge);
            return Err(StepError::OpenShell { solid, issues });
        }
    }
    Ok(())
}

/// A `FACETED_BREP`: planar facets bounded by `POLY_LOOP`s. Vertices are the points (welded),
/// edges the straight segments between consecutive points (one per unordered vertex pair, id =
/// the `POLY_LOOP` that first used it), each face gets a plane through its outer loop (Newell
/// normal, so the loop is counter-clockwise seen from outside as in a `FACE_OUTER_BOUND`).
fn read_faceted(ents: Ents<'_>, id: u64, units: UnitReport, opts: &ReadOptions) -> Result<(Solid, (usize, usize)), StepError> {
    use sm_geom_api::{CurveDef, Frame, Interval, Line, Plane, SurfaceDef};
    let a = ents.rec(id, id, "FACETED_BREP")?;
    let (shell_id, shell_reversed) = shell_of(ents, a.reference(1)?, id)?;
    let cs = ents.rec(shell_id, id, "CLOSED_SHELL")?;
    let face_ids: BTreeSet<u64> = cs.refs(1)?.into_iter().collect();
    struct Poly {
        loop_id: u64,
        outer: bool,
        /// Point entity ids, in the order of the face (bound orientation applied).
        points: Vec<u64>,
    }
    let mut polys: Vec<(u64, Vec<Poly>)> = Vec::with_capacity(face_ids.len());
    let mut point_ids: BTreeSet<u64> = BTreeSet::new();
    for &fid in &face_ids {
        let fa = ents.one_of(fid, id, &["FACE", "FACE_SURFACE", "ADVANCED_FACE"])?;
        let mut bounds = Vec::new();
        for b in fa.refs(1)? {
            let ba = ents.one_of(b, fid, &["FACE_OUTER_BOUND", "FACE_BOUND"])?;
            let orientation = ba.boolean(2)?;
            let loop_id = ba.reference(1)?;
            let la = ents.rec(loop_id, b, "POLY_LOOP")?;
            let mut pts = la.refs(1)?;
            if pts.len() < 3 {
                return Err(la.bad(1, "a polygon loop needs at least three points"));
            }
            if orientation == shell_reversed {
                pts.reverse();
            }
            point_ids.extend(pts.iter().copied());
            bounds.push(Poly { loop_id, outer: ba.rec.name == "FACE_OUTER_BOUND", points: pts });
        }
        if bounds.is_empty() {
            return Err(fa.bad(1, "a face needs at least one bound"));
        }
        if let Some(p) = bounds.iter().position(|b| b.outer) {
            let o = bounds.remove(p);
            bounds.insert(0, o);
        }
        polys.push((fid, bounds));
    }
    let pids: Vec<u64> = point_ids.into_iter().collect();
    let geo = Geo { ents, scale: units.scale, angle: units.angle_rad };
    let pts: Vec<Point3> = pids.iter().map(|p| geo_point(&geo, *p, id)).collect::<Result<_, _>>()?;
    let rep = weld_vertices(&pts, opts.weld_tol);
    let mut compact = vec![usize::MAX; pids.len()];
    let mut vertices: Vec<Vertex> = Vec::new();
    for (i, r) in rep.iter().enumerate() {
        if *r == i {
            compact[i] = vertices.len();
            vertices.push(Vertex { id: pids[i], point: pts[i] });
        }
    }
    let welded = pids.len() - vertices.len();
    let v_of: BTreeMap<u64, usize> = pids.iter().enumerate().map(|(i, p)| (*p, compact[rep[i]])).collect();
    let mut edges: Vec<Edge> = Vec::new();
    let mut curves: Vec<CurveRec> = Vec::new();
    let mut edge_ix: BTreeMap<(usize, usize), usize> = BTreeMap::new();
    let mut surfaces: Vec<SurfaceRec> = Vec::new();
    let mut faces: Vec<Face> = Vec::with_capacity(polys.len());
    for (fid, bounds) in &polys {
        let mut loops = Vec::with_capacity(bounds.len());
        let mut plane: Option<Plane> = None;
        for (bi, b) in bounds.iter().enumerate() {
            let vs: Vec<usize> = b.points.iter().map(|p| v_of[p]).collect();
            let mut lp = Vec::with_capacity(vs.len());
            for i in 0..vs.len() {
                let (va, vb) = (vs[i], vs[(i + 1) % vs.len()]);
                if va == vb {
                    continue;
                }
                let key = (va.min(vb), va.max(vb));
                let ei = match edge_ix.get(&key) {
                    Some(e) => *e,
                    None => {
                        let (p0, p1) = (vertices[key.0].point, vertices[key.1].point);
                        let d = p1 - p0;
                        let line = Line::new(p0, d).map_err(|e| StepError::GeometryMismatch { id: b.loop_id, msg: e.to_string() })?;
                        let curve = curves.len();
                        curves.push(CurveRec { id: b.loop_id, def: CurveDef::Line(line) });
                        let ei = edges.len();
                        edges.push(Edge { id: b.loop_id, start: key.0, end: key.1, curve, range: Interval::new(0.0, d.norm()), sense: Sense::Forward, merged: Vec::new() });
                        edge_ix.insert(key, ei);
                        ei
                    }
                };
                lp.push(Coedge { edge: ei, sense: if (va, vb) == key { Sense::Forward } else { Sense::Reversed } });
            }
            if lp.len() < 3 {
                return Err(StepError::GeometryMismatch { id: b.loop_id, msg: "a polygon loop collapses after welding".into() });
            }
            if bi == 0 {
                // Newell normal of the outer loop.
                let mut n = [0.0f64; 3];
                for i in 0..vs.len() {
                    let (p, q) = (vertices[vs[i]].point, vertices[vs[(i + 1) % vs.len()]].point);
                    n[0] += (p.y - q.y) * (p.z + q.z);
                    n[1] += (p.z - q.z) * (p.x + q.x);
                    n[2] += (p.x - q.x) * (p.y + q.y);
                }
                let frame = Frame::from_z(vertices[vs[0]].point, sm_core::Vector3::new(n[0], n[1], n[2])).map_err(|_| StepError::GeometryMismatch { id: *fid, msg: "the outer polygon of the face has no area".into() })?;
                plane = Some(Plane { frame });
            }
            loops.push(lp);
        }
        let Some(plane) = plane else { continue };
        let surface = surfaces.len();
        surfaces.push(SurfaceRec { id: *fid, def: SurfaceDef::Plane(plane) });
        faces.push(Face { id: *fid, surface, sense: Sense::Forward, loops, shell: 0 });
    }
    let shells = vec![Shell { id: shell_id, kind: ShellKind::Outer, faces: (0..faces.len()).collect() }];
    check_closed(id, &shells, &faces, &edges)?;
    let name = a.string(0);
    Ok((Solid { id, instance: 0, name, kind: SolidKind::Faceted, units, vertices, curves, surfaces, edges, faces, shells }, (welded, 0)))
}

fn geo_point(geo: &Geo<'_>, point_id: u64, by: u64) -> Result<Point3, StepError> {
    let a = geo.ents.rec(point_id, by, "CARTESIAN_POINT")?;
    let c = a.reals(1)?;
    if c.len() != 3 || c.iter().any(|v| !v.is_finite()) {
        return Err(a.bad(1, "expected three finite coordinates"));
    }
    Ok(Point3::new(c[0] * geo.scale, c[1] * geo.scale, c[2] * geo.scale))
}

/// The `EDGE_CURVE` id and effective orientation of an `ORIENTED_EDGE` (nested ones flatten).
fn oriented_edge(ents: Ents<'_>, id: u64, by: u64) -> Result<(u64, bool), StepError> {
    let mut cur = id;
    let mut by = by;
    let mut orient = true;
    for _ in 0..4 {
        let a = ents.rec(cur, by, "ORIENTED_EDGE")?;
        orient ^= !a.boolean(4)?;
        let el = a.reference(3)?;
        if ents.get(el, cur)?.is("ORIENTED_EDGE") {
            by = cur;
            cur = el;
            continue;
        }
        return Ok((el, orient));
    }
    Err(StepError::Cycle { id })
}

/// Decode `ids` in parallel, collected by index; the first error by index wins.
fn par_decode<T: Send>(ids: &[u64], f: impl Fn(u64) -> Result<T, StepError> + Sync) -> Result<Vec<T>, StepError> {
    if ids.len() < 128 {
        return ids.iter().map(|&i| f(i)).collect();
    }
    let results: Vec<Result<T, StepError>> = ids.par_iter().map(|&i| f(i)).collect();
    results.into_iter().collect()
}

