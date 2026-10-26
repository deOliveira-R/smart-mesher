//! Inspection of a STEP file without decoding geometry (the `cad.step_info` query).

use std::collections::{BTreeMap, BTreeSet};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::args::{Args, Ents};
use crate::decode::{scan_skipped, solid_entities};
use crate::error::StepError;
use crate::gcodec::Geo;
use crate::model::SolidKind;
use crate::part21::{Entity, StepFile};
use crate::units;

/// Per-solid summary.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct SolidInfo {
    /// STEP id of the solid entity.
    pub step_id: u64,
    /// Its name attribute.
    pub name: String,
    /// `manifold` or `with_voids`.
    pub kind: String,
    /// Number of shells (1 plus the voids).
    pub shells: usize,
    /// Number of faces.
    pub faces: usize,
    /// Number of distinct `EDGE_CURVE`s.
    pub edges: usize,
    /// Number of distinct `VERTEX_POINT`s.
    pub vertices: usize,
    /// Faces per surface kind (`plane`, `cylinder`, `cone`, `sphere`, `torus`, `nurbs`, or the
    /// entity name of an unsupported kind).
    pub surface_kinds: BTreeMap<String, usize>,
}

/// An entity kind the reader cannot import.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct UnsupportedInfo {
    /// Entity name.
    pub entity: String,
    /// How many times a solid uses (or the file contains) it.
    pub count: usize,
}

/// The `cad.step_info` reply.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct StepInfo {
    /// `AP203`, `AP214`, `AP242` or `other`.
    pub schema: String,
    /// The first `FILE_SCHEMA` identifier as written.
    pub schema_id: String,
    /// Lowercase hex sha256 of the bytes.
    pub sha256: String,
    /// Size in bytes.
    pub bytes: u64,
    /// Name of the length unit the first solid's context declares, if any.
    pub length_unit: Option<String>,
    /// Micrometres per length unit.
    pub length_unit_um: Option<f64>,
    /// Name of the plane angle unit.
    pub angle_unit: String,
    /// Radians per angle unit.
    pub angle_unit_rad: f64,
    /// Declared distance uncertainty in file length units.
    pub uncertainty: Option<f64>,
    /// The solids in ascending STEP id order.
    pub solids: Vec<SolidInfo>,
    /// Entity kinds that cannot be imported.
    pub unsupported: Vec<UnsupportedInfo>,
    /// `MAPPED_ITEM` or `NEXT_ASSEMBLY_USAGE_OCCURRENCE` is present.
    pub assemblies: bool,
    /// Number of sheet entities (`SHELL_BASED_SURFACE_MODEL`, `OPEN_SHELL`, ...).
    pub sheets: usize,
}

/// Classify the schema identifier.
pub(crate) fn schema_kind(id: &str) -> &'static str {
    let u = id.to_uppercase();
    if u.contains("MANAGED_MODEL_BASED_3D_ENGINEERING") || u.contains("AP242") {
        "AP242"
    } else if u.contains("CONFIG_CONTROL_DESIGN") || u.contains("CONFIGURATION_CONTROL") || u.contains("AP203") {
        "AP203"
    } else if u.contains("AUTOMOTIVE_DESIGN") || u.contains("AP214") {
        "AP214"
    } else {
        "other"
    }
}

fn surface_kind(e: &Entity) -> String {
    let name = e.first().map(|r| r.name.as_str()).unwrap_or("");
    match name {
        "PLANE" => "plane".into(),
        "CYLINDRICAL_SURFACE" => "cylinder".into(),
        "CONICAL_SURFACE" => "cone".into(),
        "SPHERICAL_SURFACE" => "sphere".into(),
        "TOROIDAL_SURFACE" => "torus".into(),
        _ if e.is("B_SPLINE_SURFACE_WITH_KNOTS") => "nurbs".into(),
        _ => e.display_name(),
    }
}

fn is_supported_curve(e: &Entity) -> bool {
    matches!(e.first().map(|r| r.name.as_str()), Some("LINE" | "CIRCLE" | "ELLIPSE" | "TRIMMED_CURVE")) || e.is("B_SPLINE_CURVE_WITH_KNOTS")
}

/// A `FACETED_BREP`: faces are planar polygons; vertices and edges are counted by distinct
/// point entities and distinct unordered point pairs (before any welding).
fn inspect_faceted(ents: Ents<'_>, id: u64) -> Result<SolidInfo, StepError> {
    let a = ents.rec(id, id, "FACETED_BREP")?;
    let mut shell = a.reference(1)?;
    for _ in 0..4 {
        match ents.get(shell, id)?.rec("ORIENTED_CLOSED_SHELL") {
            Some(r) => shell = Args { id: shell, rec: r }.reference(2)?,
            None => break,
        }
    }
    let faces: BTreeSet<u64> = ents.rec(shell, id, "CLOSED_SHELL")?.refs(1)?.into_iter().collect();
    let mut points: BTreeSet<u64> = BTreeSet::new();
    let mut pairs: BTreeSet<(u64, u64)> = BTreeSet::new();
    for &f in &faces {
        let fa = ents.one_of(f, id, &["FACE", "FACE_SURFACE", "ADVANCED_FACE"])?;
        for b in fa.refs(1)? {
            let ba = ents.one_of(b, f, &["FACE_OUTER_BOUND", "FACE_BOUND"])?;
            let la = ents.rec(ba.reference(1)?, b, "POLY_LOOP")?;
            let pts = la.refs(1)?;
            points.extend(pts.iter().copied());
            for i in 0..pts.len() {
                let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
                pairs.insert((p.min(q), p.max(q)));
            }
        }
    }
    Ok(SolidInfo {
        step_id: id,
        name: a.string(0),
        kind: "faceted".to_owned(),
        shells: 1,
        faces: faces.len(),
        edges: pairs.len(),
        vertices: points.len(),
        surface_kinds: BTreeMap::from([("plane".to_owned(), faces.len())]),
    })
}

/// Summarize a parsed file.
pub(crate) fn inspect(file: &StepFile, sha256: String, bytes: u64) -> Result<StepInfo, StepError> {
    let ents = Ents::new(file);
    let schema_id = file.header.schemas.first().cloned().unwrap_or_default();
    let mut solids = Vec::new();
    let mut unsupported: BTreeMap<String, usize> = BTreeMap::new();
    let ctx_of = units::contexts_by_item(file);
    let mut first_ctx: Option<u64> = None;
    for (id, kind) in solid_entities(file) {
        first_ctx = first_ctx.or_else(|| ctx_of.get(&id).copied());
        let e = ents.get(id, id)?;
        if kind == SolidKind::Faceted {
            solids.push(inspect_faceted(ents, id)?);
            continue;
        }
        let (name, outer, voids) = match e {
            Entity::Simple(r) => {
                let a = Args { id, rec: r };
                (a.string(0), a.reference(1)?, if kind == SolidKind::WithVoids { a.refs(2)? } else { Vec::new() })
            }
            Entity::Complex(_) => {
                let m = e.rec("MANIFOLD_SOLID_BREP").ok_or(StepError::Unsupported { id, entity: e.display_name() })?;
                let a = Args { id, rec: m };
                let voids = match e.rec("BREP_WITH_VOIDS") {
                    Some(r) => Args { id, rec: r }.refs(0)?,
                    None => Vec::new(),
                };
                (a.string(0), a.reference(1)?, voids)
            }
        };
        let mut shell_ids = vec![outer];
        shell_ids.extend(voids);
        let geo = Geo { ents, scale: 1.0, angle: 1.0 };
        let mut faces: BTreeSet<u64> = BTreeSet::new();
        for s in &shell_ids {
            // ORIENTED_CLOSED_SHELL wraps a CLOSED_SHELL.
            let mut cur = *s;
            for _ in 0..4 {
                let se = ents.get(cur, id)?;
                match se.rec("ORIENTED_CLOSED_SHELL") {
                    Some(r) => cur = Args { id: cur, rec: r }.reference(2)?,
                    None => break,
                }
            }
            let a = ents.rec(cur, id, "CLOSED_SHELL")?;
            faces.extend(a.refs(1)?);
        }
        let mut edges: BTreeSet<u64> = BTreeSet::new();
        let mut vertices: BTreeSet<u64> = BTreeSet::new();
        let mut kinds: BTreeMap<String, usize> = BTreeMap::new();
        let mut seen_curves: BTreeSet<u64> = BTreeSet::new();
        let mut seen_surfaces: BTreeSet<u64> = BTreeSet::new();
        for f in &faces {
            let fa = ents.one_of(*f, id, &["ADVANCED_FACE", "FACE_SURFACE"])?;
            let surf = fa.reference(2)?;
            let se = ents.get(surf, *f)?;
            let kind = surface_kind(se);
            if seen_surfaces.insert(surf) && !matches!(kind.as_str(), "plane" | "cylinder" | "cone" | "sphere" | "torus" | "nurbs") {
                *unsupported.entry(kind.clone()).or_default() += 1;
            }
            *kinds.entry(kind).or_default() += 1;
            for b in fa.refs(1)? {
                let ba = ents.one_of(b, *f, &["FACE_OUTER_BOUND", "FACE_BOUND"])?;
                let lp = ents.get(ba.reference(1)?, b)?;
                let Some(lrec) = lp.rec("EDGE_LOOP") else {
                    match lp.rec("VERTEX_LOOP") {
                        Some(vl) => {
                            vertices.insert(Args { id: ba.reference(1)?, rec: vl }.reference(1)?);
                        }
                        None => *unsupported.entry(lp.display_name()).or_default() += 1,
                    }
                    continue;
                };
                let la = Args { id: ba.reference(1)?, rec: lrec };
                for oe in la.refs(1)? {
                    let mut cur = oe;
                    let mut guard = 0;
                    let edge_id = loop {
                        let oa = ents.rec(cur, la.id, "ORIENTED_EDGE")?;
                        let el = oa.reference(3)?;
                        guard += 1;
                        if guard < 4 && ents.get(el, cur)?.is("ORIENTED_EDGE") {
                            cur = el;
                        } else {
                            break el;
                        }
                    };
                    if edges.insert(edge_id) {
                        let ea = ents.rec(edge_id, oe, "EDGE_CURVE")?;
                        vertices.insert(ea.reference(1)?);
                        vertices.insert(ea.reference(2)?);
                        let c = geo.curve_ref(ea.reference(3)?, edge_id)?;
                        if seen_curves.insert(c) {
                            let ce = ents.get(c, edge_id)?;
                            if !is_supported_curve(ce) {
                                *unsupported.entry(ce.display_name()).or_default() += 1;
                            }
                        }
                    }
                }
            }
        }
        solids.push(SolidInfo {
            step_id: id,
            name,
            kind: match kind {
                SolidKind::WithVoids => "with_voids",
                SolidKind::Faceted => "faceted",
                SolidKind::Sheet => "sheet",
                SolidKind::Manifold => "manifold",
            }
            .to_owned(),
            shells: shell_ids.len(),
            faces: faces.len(),
            edges: edges.len(),
            vertices: vertices.len(),
            surface_kinds: kinds,
        });
    }
    let skipped = scan_skipped(file);
    for (n, c) in &skipped.unsupported {
        *unsupported.entry(n.clone()).or_default() += c;
    }
    let ctx = first_ctx.or_else(|| ctx_of.values().next().copied());
    let cu = match ctx {
        Some(c) => Some(units::context_units(file, c)?),
        None => None,
    };
    Ok(StepInfo {
        schema: schema_kind(&schema_id).to_owned(),
        schema_id,
        sha256,
        bytes,
        length_unit: cu.as_ref().and_then(|c| c.length.as_ref().map(|l| l.name.clone())),
        length_unit_um: cu.as_ref().and_then(|c| c.length.as_ref().map(|l| l.um)),
        angle_unit: cu.as_ref().map_or_else(|| "radian".to_owned(), |c| c.angle.name.clone()),
        angle_unit_rad: cu.as_ref().map_or(1.0, |c| c.angle.rad),
        uncertainty: cu.as_ref().and_then(|c| c.uncertainty),
        solids,
        unsupported: unsupported.into_iter().map(|(entity, count)| UnsupportedInfo { entity, count }).collect(),
        assemblies: skipped.assemblies,
        sheets: skipped.sheets.values().sum(),
    })
}
