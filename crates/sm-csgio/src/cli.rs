//! The `smcsg` tool as a library function: argument parsing, the two commands and the exit codes.
//! The binary supplies only the loader that turns a deck file into a [`CsgIr`], so tests can run
//! the whole tool on hand-built decks.
//!
//! ```text
//! smcsg check <deck> [--format openmc] [--materials PATH] [--samples N] [--rays N] [--seed S]
//!                    [--names 1=fuel,2=water] [--strict-gaps] [--json]
//! smcsg info  <deck> [--format openmc] [--materials PATH] [--names ...] [--json]
//! ```
//! Exit code 0: clean. 1: an overlap was found, or a gap with `--strict-gaps`. 2: the deck cannot
//! be read or used (parse error, unsupported feature, bad usage). Warnings go to stderr as
//! `warning:` lines; stdout carries the result only.

use crate::{CsgIr, ImportError, ImportOptions, ImportedCsg, Warning};
use serde_json::{Value, json};
use sm_geom_api::SurfaceDef;
use std::collections::{BTreeMap, BTreeSet};
use std::io::Write;
use std::path::Path;

/// Deck formats `--format` accepts.
pub const FORMATS: [&str; 1] = ["openmc"];

/// What the loader is asked to read.
#[derive(Clone, Copy, Debug)]
pub struct LoadRequest<'a> {
    /// The format name (one of [`FORMATS`]).
    pub format: &'a str,
    /// The deck file.
    pub deck: &'a Path,
    /// The materials file, when `--materials` named one.
    pub materials: Option<&'a Path>,
}

/// Reads a deck into the IR.
pub type Loader<'a> = &'a dyn Fn(&LoadRequest<'_>) -> Result<CsgIr, ImportError>;

struct Args {
    cmd: String,
    deck: String,
    format: String,
    materials: Option<String>,
    opts: ImportOptions,
    strict_gaps: bool,
    json: bool,
}

const USAGE: &str = "usage: smcsg check <deck> [--format openmc] [--materials PATH] [--samples N] [--rays N] [--seed S] [--names 1=fuel,2=water] [--strict-gaps] [--json]\n       smcsg info <deck> [--format openmc] [--materials PATH] [--names ...] [--json]";

fn parse_args(args: &[String]) -> Result<Args, String> {
    let mut it = args.iter();
    let cmd = it.next().ok_or("missing command")?.clone();
    if cmd != "check" && cmd != "info" {
        return Err(format!("unknown command {cmd:?}"));
    }
    let mut a = Args { cmd, deck: String::new(), format: "openmc".into(), materials: None, opts: ImportOptions::default(), strict_gaps: false, json: false };
    let mut deck = None;
    while let Some(x) = it.next() {
        let mut value = |name: &str| it.next().cloned().ok_or(format!("{name} needs a value"));
        match x.as_str() {
            "--format" => a.format = value("--format")?,
            "--materials" => a.materials = Some(value("--materials")?),
            "--samples" => a.opts.check.samples = value("--samples")?.parse().map_err(|_| "--samples needs an integer".to_string())?,
            "--rays" => a.opts.check.rays = value("--rays")?.parse().map_err(|_| "--rays needs an integer".to_string())?,
            "--seed" => a.opts.check.seed = value("--seed")?.parse().map_err(|_| "--seed needs an integer".to_string())?,
            "--names" => {
                for pair in value("--names")?.split(',').filter(|p| !p.is_empty()) {
                    let (id, name) = pair.split_once('=').ok_or(format!("--names entry {pair:?} is not id=name"))?;
                    let id: u32 = id.trim().parse().map_err(|_| format!("--names id {id:?} is not an integer"))?;
                    a.opts.material_names.insert(id, name.to_string());
                }
            }
            "--strict-gaps" => a.strict_gaps = true,
            "--json" => a.json = true,
            s if s.starts_with("--") => return Err(format!("unknown option {s}")),
            s => {
                if deck.replace(s.to_string()).is_some() {
                    return Err("more than one deck given".into());
                }
            }
        }
    }
    a.deck = deck.ok_or("missing deck path")?;
    if !FORMATS.contains(&a.format.as_str()) {
        return Err(format!("unknown format {:?} (supported: {})", a.format, FORMATS.join(", ")));
    }
    Ok(a)
}

fn point_json(p: Option<[f64; 3]>) -> Value {
    p.map_or(Value::Null, |p| json!(p))
}

fn warning_json(w: &Warning) -> Value {
    json!({"code": w.code, "message": w.message, "point": point_json(w.point)})
}

fn warning_line(w: &Warning) -> String {
    match w.point {
        Some([x, y, z]) => format!("warning: {}: {} (at {x}, {y}, {z})", w.code, w.message),
        None => format!("warning: {}: {}", w.code, w.message),
    }
}

fn kind_name(d: &SurfaceDef) -> &'static str {
    match d {
        SurfaceDef::Plane(_) => "plane",
        SurfaceDef::Cylinder(_) => "cylinder",
        SurfaceDef::Cone(_) => "cone",
        SurfaceDef::Sphere(_) => "sphere",
        SurfaceDef::Torus(_) => "torus",
        SurfaceDef::Implicit(_) => "quadric",
        SurfaceDef::EllipticCylinder(_) => "elliptic_cylinder",
        SurfaceDef::EllipticCone(_) => "elliptic_cone",
        _ => "other",
    }
}

fn put(out: &mut dyn Write, s: &str) {
    // A closed pipe is not worth a different exit code.
    let _ = writeln!(out, "{s}");
}

fn finish_error(a: &Args, e: &ImportError, out: &mut dyn Write, err: &mut dyn Write) -> i32 {
    put(err, &format!("error: {e}"));
    if a.json {
        put(out, &serde_json::to_string_pretty(&json!({"status": "error", "error": e.to_string()})).unwrap_or_default());
    }
    2
}

/// Run `smcsg` with `args` (without the program name). Returns the exit code.
pub fn run(args: &[String], load: Loader<'_>, out: &mut dyn Write, err: &mut dyn Write) -> i32 {
    let a = match parse_args(args) {
        Ok(a) => a,
        Err(m) => {
            put(err, &format!("error: {m}"));
            put(err, USAGE);
            return 2;
        }
    };
    let req = LoadRequest { format: &a.format, deck: Path::new(&a.deck), materials: a.materials.as_deref().map(Path::new) };
    let ir = match load(&req) {
        Ok(ir) => ir,
        Err(e) => return finish_error(&a, &e, out, err),
    };
    if a.cmd == "info" { info(&a, &ir, out, err) } else { check(&a, &ir, out, err) }
}

fn check(a: &Args, ir: &CsgIr, out: &mut dyn Write, err: &mut dyn Write) -> i32 {
    match ImportedCsg::import(ir, &a.opts) {
        Ok(m) => {
            let r = m.report();
            for w in &r.warnings {
                put(err, &warning_line(w));
            }
            let gaps = r.warnings.iter().any(|w| w.code == "import.gap");
            let code = i32::from(gaps && a.strict_gaps);
            let c = r.checked.unwrap_or(a.opts.check);
            if a.json {
                let v = json!({
                    "status": if code == 1 { "gaps" } else { "ok" },
                    "cells": m.cell_count(), "surfaces": m.surface_count(), "regions": m.region_names(),
                    "warnings": r.warnings.iter().map(warning_json).collect::<Vec<_>>(),
                    "overlaps": [], "dropped": r.dropped,
                    "check": {"samples": c.samples, "rays": c.rays, "seed": c.seed},
                });
                put(out, &serde_json::to_string_pretty(&v).unwrap_or_default());
            } else {
                put(out, &format!("{}: {} cells, {} regions ({}); no overlap found in {} points and {} chords at seed {}", if code == 1 { "gaps" } else { "ok" }, m.cell_count(), m.region_names().len(), m.region_names().join(", "), c.samples, c.rays, c.seed));
            }
            code
        }
        Err(ImportError::Overlap { findings }) => {
            let c = a.opts.check;
            if a.json {
                let f: Vec<Value> = findings.iter().map(|f| json!({"point": f.point, "a": f.a.to_string(), "b": f.b.to_string()})).collect();
                let v = json!({"status": "overlap", "overlaps": f, "check": {"samples": c.samples, "rays": c.rays, "seed": c.seed}});
                put(out, &serde_json::to_string_pretty(&v).unwrap_or_default());
            } else {
                put(out, &format!("overlap: {} finding(s)", findings.len()));
                for f in &findings {
                    put(out, &format!("  cells {} and {} at ({}, {}, {})", f.a, f.b, f.point[0], f.point[1], f.point[2]));
                }
            }
            put(err, "error: the deck has overlapping cells");
            1
        }
        Err(e) => finish_error(a, &e, out, err),
    }
}

fn info(a: &Args, ir: &CsgIr, out: &mut dyn Write, err: &mut dyn Write) -> i32 {
    let m = match ImportedCsg::instantiate(ir, &a.opts) {
        Ok(m) => m,
        Err(e) => return finish_error(a, &e, out, err),
    };
    let mut kinds: BTreeMap<&str, usize> = BTreeMap::new();
    for s in ir.surfaces.values() {
        *kinds.entry(kind_name(&s.def)).or_default() += 1;
    }
    let universes: BTreeSet<u32> = ir.cells.values().map(|c| c.universe).collect();
    for w in &m.report().warnings {
        put(err, &warning_line(w));
    }
    if a.json {
        let v = json!({
            "surfaces": kinds, "cells": ir.cells.len(), "materials": ir.materials.len(), "universes": universes.len(), "lattices": ir.lattices.len(),
            "flat_cells": m.cell_count(), "flat_surfaces": m.surface_count(), "regions": m.region_names(),
            "warnings": m.report().warnings.iter().map(warning_json).collect::<Vec<_>>(), "dropped": ir.dropped,
        });
        put(out, &serde_json::to_string_pretty(&v).unwrap_or_default());
    } else {
        let k: Vec<String> = kinds.iter().map(|(k, n)| format!("{k} {n}")).collect();
        put(out, &format!("surfaces: {} ({})", ir.surfaces.len(), k.join(", ")));
        put(out, &format!("cells: {}, materials: {}, universes: {}, lattices: {}", ir.cells.len(), ir.materials.len(), universes.len(), ir.lattices.len()));
        put(out, &format!("flat: {} cells, {} surfaces, regions: {}", m.cell_count(), m.surface_count(), m.region_names().join(", ")));
        if !ir.dropped.is_empty() {
            put(out, &format!("dropped: {}", ir.dropped.join("; ")));
        }
    }
    0
}
