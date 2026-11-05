//! STEP names and colors of solids (`model.import_step {attributes: true}`).
//!
//! Names: a solid's own `name` attribute, else the `PRODUCT` name reached through
//! `SHAPE_DEFINITION_REPRESENTATION`. Colors: only `STYLED_ITEM`s whose item is the solid itself
//! (Q14); styled faces and other items are counted and ignored.

use std::collections::BTreeMap;

use crate::part21::{Entity, Param, Rec, StepFile};

/// A color found on a solid.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct StepColor {
    /// The `COLOUR_RGB` name (lower-cased predefined name for `DRAUGHTING_PRE_DEFINED_COLOUR`),
    /// empty when the file gives none.
    pub name: String,
    /// Red, green, blue in `[0, 1]`.
    pub rgb: [f64; 3],
}

impl StepColor {
    /// The model color name: the file's name, else `rgb_rrggbb`.
    pub fn model_name(&self) -> String {
        if !self.name.is_empty() {
            return self.name.clone();
        }
        let [r, g, b] = self.rgb.map(|c| (c.clamp(0.0, 1.0) * 255.0).round() as u8);
        format!("rgb_{r:02x}{g:02x}{b:02x}")
    }
}

/// Names and colors per solid STEP id, plus what was ignored.
#[derive(Debug, Default)]
pub(crate) struct Attributes {
    /// Group name per solid (solids without any name are absent).
    pub names: BTreeMap<u64, String>,
    /// Color per solid (lowest styled-item id wins).
    pub colors: BTreeMap<u64, StepColor>,
    /// Styled items with a color on a face: ignored.
    pub face_only: usize,
    /// Styled items on anything else (curves, mapped items, ...): ignored.
    pub other: usize,
}

fn recs(e: &Entity) -> Vec<&Rec> {
    match e {
        Entity::Simple(r) => vec![r],
        Entity::Complex(v) => v.iter().collect(),
    }
}

fn refs(p: Option<&Param>) -> Vec<u64> {
    match p {
        Some(Param::Ref(r)) => vec![*r],
        Some(Param::List(l)) => l.iter().filter_map(|x| if let Param::Ref(r) = x { Some(*r) } else { None }).collect(),
        _ => Vec::new(),
    }
}

fn string(r: &Rec, i: usize) -> String {
    match r.params.get(i) {
        Some(Param::Str(s)) => s.clone(),
        _ => String::new(),
    }
}

fn predefined(name: &str) -> Option<[f64; 3]> {
    Some(match name {
        "red" => [1.0, 0.0, 0.0],
        "green" => [0.0, 1.0, 0.0],
        "blue" => [0.0, 0.0, 1.0],
        "yellow" => [1.0, 1.0, 0.0],
        "cyan" => [0.0, 1.0, 1.0],
        "magenta" => [1.0, 0.0, 1.0],
        "black" => [0.0; 3],
        "white" => [1.0; 3],
        _ => return None,
    })
}

fn colour(file: &StepFile, id: u64) -> Option<StepColor> {
    let e = file.entities.get(&id)?;
    if let Some(r) = e.rec("COLOUR_RGB") {
        let c = |i: usize| match r.params.get(i) {
            Some(Param::Real(v)) => Some(*v),
            Some(Param::Int(v)) => Some(*v as f64),
            _ => None,
        };
        let rgb = [c(1)?, c(2)?, c(3)?];
        return rgb.iter().all(|v| v.is_finite()).then(|| StepColor { name: string(r, 0), rgb });
    }
    let r = e.rec("DRAUGHTING_PRE_DEFINED_COLOUR")?;
    let name = string(r, 0).to_lowercase();
    predefined(&name).map(|rgb| StepColor { name, rgb })
}

/// Entities a surface style is followed through to reach its color.
const STYLE_CHAIN: &[&str] = &[
    "PRESENTATION_STYLE_ASSIGNMENT",
    "PRESENTATION_STYLE_BY_CONTEXT",
    "SURFACE_STYLE_USAGE",
    "SURFACE_SIDE_STYLE",
    "SURFACE_STYLE_FILL_AREA",
    "FILL_AREA_STYLE",
    "FILL_AREA_STYLE_COLOUR",
    "SURFACE_STYLE_RENDERING",
    "SURFACE_STYLE_RENDERING_WITH_PROPERTIES",
];

/// The first surface color under `id` (depth-first in parameter order, depth capped).
fn style_color(file: &StepFile, id: u64, depth: u32) -> Option<StepColor> {
    if depth > 10 {
        return None;
    }
    if let Some(c) = colour(file, id) {
        return Some(c);
    }
    let e = file.entities.get(&id)?;
    let r = recs(e).into_iter().find(|r| STYLE_CHAIN.contains(&r.name.as_str()))?;
    r.params.iter().flat_map(|p| refs(Some(p))).find_map(|c| style_color(file, c, depth + 1))
}

/// Read names and colors. `solids` lists `(STEP id, own name)` of the imported solids and sheets.
pub(crate) fn read_attributes(file: &StepFile, solids: &[(u64, String)]) -> Attributes {
    let mut out = Attributes::default();
    // Product names through SHAPE_DEFINITION_REPRESENTATION, lowest SDR id first.
    let mut product_of: BTreeMap<u64, String> = BTreeMap::new();
    for e in file.entities.values() {
        let Some(sdr) = e.rec("SHAPE_DEFINITION_REPRESENTATION") else { continue };
        let Some(pds) = refs(sdr.params.first()).first().copied() else { continue };
        let Some(rep) = refs(sdr.params.get(1)).first().copied() else { continue };
        let chain = |id: u64, kw: &str| file.entities.get(&id).and_then(|e| e.rec(kw)).and_then(|r| refs(r.params.get(2)).first().copied());
        let Some(prod) = chain(pds, "PRODUCT_DEFINITION_SHAPE").and_then(|pd| chain(pd, "PRODUCT_DEFINITION")).and_then(|f| chain(f, "PRODUCT_DEFINITION_FORMATION")) else { continue };
        let Some(p) = file.entities.get(&prod).and_then(|e| e.rec("PRODUCT")) else { continue };
        let name = if string(p, 1).is_empty() { string(p, 0) } else { string(p, 1) };
        if name.is_empty() {
            continue;
        }
        let Some(items) = file.entities.get(&rep).and_then(|e| e.first()).map(|r| refs(r.params.get(1))) else { continue };
        for it in items {
            product_of.entry(it).or_insert_with(|| name.clone());
        }
    }
    for (id, own) in solids {
        let name = if own.is_empty() { product_of.get(id).cloned().unwrap_or_default() } else { own.clone() };
        if !name.is_empty() {
            out.names.insert(*id, name);
        }
    }
    for e in file.entities.values() {
        let Some(si) = e.rec("STYLED_ITEM").or_else(|| e.rec("OVER_RIDING_STYLED_ITEM")) else { continue };
        let Some(item) = refs(si.params.get(2)).first().copied() else { continue };
        let Some(c) = refs(si.params.get(1)).into_iter().find_map(|s| style_color(file, s, 0)) else { continue };
        let target = file.entities.get(&item);
        if solids.iter().any(|(id, _)| *id == item) {
            out.colors.entry(item).or_insert(c);
        } else if target.is_some_and(|t| t.is("ADVANCED_FACE") || t.is("FACE_SURFACE")) {
            out.face_only += 1;
        } else if !target.is_some_and(|t| t.is("MANIFOLD_SOLID_BREP") || t.is("BREP_WITH_VOIDS") || t.is("FACETED_BREP") || t.is("SHELL_BASED_SURFACE_MODEL")) {
            out.other += 1;
        }
    }
    out
}
