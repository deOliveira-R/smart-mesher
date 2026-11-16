//! OpenMC `geometry.xml` / `materials.xml` reader.
//!
//! Supported (see `DESIGN.md` for the table and for how each convention was confirmed against
//! OpenMC itself): `<surface>` of the types plane, x/y/z-plane, x/y/z-cylinder, sphere,
//! x/y/z-cone, quadric and circular x/y/z-torus, with `boundary` transmission, vacuum,
//! reflective or white; `<cell>` with `material` (an id or `void`), `fill`, `universe`, `region`,
//! `translation` and `rotation`; rectangular `<lattice>` and `<hex_lattice>` (both orientations, 2-D and layered) with a position-independent `outer`; `materials.xml` for names.
//! Refused with [`ImportError::Unsupported`]: a lattice `outer` universe whose cells have regions,
//! the general `cylinder` and `cone` types (OpenMC's C++ layer rejects them), `dagmc_universe`, periodic boundaries,
//! a `material` list, elliptic tori, lattices nested directly in a lattice, and rotations that are
//! not proper orthonormal matrices.
//!
//! Attributes and child elements are both accepted for every field. Errors carry the 1-based line
//! of the element.

mod region;
mod surfaces;
mod xform;

use crate::ir::{BoundaryFlag, Cell, CsgIr, Fill, HexLattice, HexOrientation, Lattice, Material};
use crate::ImportError;
use roxmltree::{Document, Node, ParsingOptions};
use std::collections::{BTreeMap, BTreeSet};

/// Inputs above this many bytes are refused (the `smcsg` binary applies the same limit on files).
pub const MAX_INPUT_BYTES: usize = 256 << 20;
/// XML nodes beyond this number are refused.
const MAX_NODES: u32 = 8_000_000;

/// Where in the deck an element is, for error messages.
pub(crate) struct Ctx {
    line: u32,
    what: String,
}

pub(crate) fn invalid(cx: &Ctx, msg: String) -> ImportError {
    ImportError::Invalid(format!("line {}: {}: {msg}", cx.line, cx.what))
}

pub(crate) fn unsupported(cx: &Ctx, what: String) -> ImportError {
    ImportError::Unsupported { line: Some(cx.line), what: format!("{}: {what}", cx.what) }
}

fn parse_xml(text: &str) -> Result<Document<'_>, ImportError> {
    if text.len() > MAX_INPUT_BYTES {
        return Err(ImportError::Invalid(format!("input of {} bytes is over the {MAX_INPUT_BYTES} byte limit", text.len())));
    }
    check_depth(text)?;
    let opts = ParsingOptions { allow_dtd: false, nodes_limit: MAX_NODES, ..ParsingOptions::default() };
    Document::parse_with_options(text, opts).map_err(|e| {
        let p = e.pos();
        ImportError::Parse { line: p.row, col: p.col, msg: e.to_string() }
    })
}

/// Deepest element nesting accepted. Real decks are 3 deep; a hostile one must not reach the
/// parser's recursion.
const MAX_XML_DEPTH: usize = 64;

/// A conservative scan of the element depth before the parser sees the text.
fn check_depth(text: &str) -> Result<(), ImportError> {
    let b = text.as_bytes();
    let (mut depth, mut i, mut line) = (0usize, 0usize, 1u32);
    while i < b.len() {
        if b[i] == b'\n' {
            line += 1;
        } else if b[i] == b'<' {
            let rest = &b[i..];
            if rest.starts_with(b"<!--") {
                i += rest.windows(3).position(|w| w == b"-->").map_or(rest.len(), |p| p + 2);
            } else if rest.starts_with(b"<![CDATA[") {
                i += rest.windows(3).position(|w| w == b"]]>").map_or(rest.len(), |p| p + 2);
            } else if rest.starts_with(b"<?") || rest.starts_with(b"<!") {
                i += rest.iter().position(|c| *c == b'>').unwrap_or(rest.len());
            } else if rest.starts_with(b"</") {
                depth = depth.saturating_sub(1);
            } else {
                // An opening tag; self-closing when the first unquoted `>` follows a `/`.
                let (mut quote, mut j, mut self_closing) = (0u8, 1usize, false);
                while j < rest.len() {
                    let c = rest[j];
                    if quote != 0 {
                        if c == quote {
                            quote = 0;
                        }
                    } else if c == b'"' || c == b'\'' {
                        quote = c;
                    } else if c == b'>' {
                        self_closing = rest[j - 1] == b'/';
                        break;
                    }
                    j += 1;
                }
                if !self_closing {
                    depth += 1;
                    if depth > MAX_XML_DEPTH {
                        return Err(ImportError::Parse { line, col: 1, msg: format!("elements are nested deeper than {MAX_XML_DEPTH}") });
                    }
                }
                i += j.saturating_sub(1);
            }
        }
        i += 1;
    }
    Ok(())
}

fn ctx(doc: &Document<'_>, n: Node<'_, '_>, what: String) -> Ctx {
    Ctx { line: doc.text_pos_at(n.range().start).row, what }
}

/// The value of `name` as an attribute, else as the text of a child element.
fn field<'a>(n: Node<'a, '_>, name: &str) -> Option<&'a str> {
    if let Some(v) = n.attribute(name) {
        return Some(v.trim());
    }
    n.children().find(|c| c.is_element() && c.tag_name().name() == name).map(|c| c.text().unwrap_or("").trim())
}

fn id_of(cx: &Ctx, n: Node<'_, '_>) -> Result<u32, ImportError> {
    let s = field(n, "id").ok_or_else(|| invalid(cx, "missing id".into()))?;
    s.parse::<u32>().map_err(|_| invalid(cx, format!("bad id {s:?}")))
}

fn numbers(cx: &Ctx, what: &str, s: &str) -> Result<Vec<f64>, ImportError> {
    s.split_whitespace()
        .map(|t| t.parse::<f64>().ok().filter(|v| v.is_finite()).ok_or_else(|| invalid(cx, format!("{what}: {t:?} is not a finite number"))))
        .collect()
}

fn count_label(n: usize, what: &str) -> String {
    format!("{what} ({n})")
}

/// Import an OpenMC geometry from the text of its `geometry.xml` and, when there is one, of its
/// `materials.xml`.
///
/// Without `materials.xml` every material id a cell uses becomes an unnamed material; with it,
/// a cell that uses an id the file does not define is an [`ImportError::UnknownMaterial`] when the
/// model is instantiated. The root universe is the one universe that no cell, lattice element or
/// `outer` fills.
///
/// # Errors
/// [`ImportError::Parse`] for malformed XML, [`ImportError::Unsupported`] for features listed in
/// the module documentation, [`ImportError::Invalid`] for inconsistent data (duplicate ids, wrong
/// coefficient counts, bad regions, a lattice whose `universes` count does not match, no unique
/// root universe).
///
/// `<hex_lattice>` (both orientations, `n_axial >= 1`) goes to [`CsgIr::hex_lattices`]. Its
/// `universes` text is OpenMC's row picture: for `orientation="x"` the rows from the highest `y`
/// down, each left to right; for `"y"` (the default) the lines of the picture from the highest `y`
/// down (a line holds the elements whose `2r + q` is equal), each left to right. Layers follow from
/// the lowest z up. The reader stores them in OpenMC's ring order ([`crate::ir::HexLattice`]). A
/// wrong `n_rings`, `n_axial`, `orientation`, `pitch`/`center` count or `universes` count is
/// [`ImportError::Invalid`].
pub fn import(geometry_xml: &str, materials_xml: Option<&str>) -> Result<CsgIr, ImportError> {
    import_impl(geometry_xml, materials_xml)
}

/// The same as [`import`]; kept as the entry the hex-lattice IR change introduced (ACR-CSG-HEXIR),
/// before [`import`] itself read `<hex_lattice>`.
///
/// # Errors
/// As [`import`].
pub fn import_hex(geometry_xml: &str, materials_xml: Option<&str>) -> Result<CsgIr, ImportError> {
    import(geometry_xml, materials_xml)
}

fn import_impl(geometry_xml: &str, materials_xml: Option<&str>) -> Result<CsgIr, ImportError> {
    let gdoc = parse_xml(geometry_xml)?;
    let root = gdoc.root_element();
    if root.tag_name().name() != "geometry" {
        return Err(ImportError::Parse { line: 1, col: 1, msg: format!("the root element is <{}>, expected <geometry>", root.tag_name().name()) });
    }
    let mut ir = CsgIr::new(0);
    let mut dropped: BTreeMap<&'static str, usize> = BTreeMap::new();
    let mut lattice_ids = BTreeSet::new();
    // Pass 1: lattice ids, so a cell's `fill` can be told apart.
    for n in root.children().filter(|n| n.is_element()) {
        let t = n.tag_name().name();
        if t == "lattice" || t == "hex_lattice" {
            let cx = ctx(&gdoc, n, t.into());
            lattice_ids.insert(id_of(&cx, n)?);
        }
    }
    let mut lattice_lines: BTreeMap<u32, u32> = BTreeMap::new();
    let mut filled: BTreeSet<u32> = BTreeSet::new();
    let mut universes: BTreeSet<u32> = BTreeSet::new();
    let mut used_materials: BTreeSet<u32> = BTreeSet::new();
    for n in root.children().filter(|n| n.is_element()) {
        let tag = n.tag_name().name();
        match tag {
            "surface" => surface(&gdoc, n, &mut ir)?,
            "cell" => cell(&gdoc, n, &mut ir, &lattice_ids, &mut filled, &mut universes, &mut used_materials, &mut dropped)?,
            "lattice" => {
                lattice(&gdoc, n, &mut ir, &lattice_ids, &mut filled)?;
                lattice_lines.insert(id_of(&ctx(&gdoc, n, "lattice".into()), n)?, ctx(&gdoc, n, String::new()).line);
            }
            "hex_lattice" => {
                hex_lattice(&gdoc, n, &mut ir, &lattice_ids, &mut filled)?;
                lattice_lines.insert(id_of(&ctx(&gdoc, n, "hex_lattice".into()), n)?, ctx(&gdoc, n, String::new()).line);
            }
            "dagmc_universe" => {
                let cx = ctx(&gdoc, n, tag.into());
                return Err(unsupported(&cx, format!("<{tag}> is not supported (no DAGMC universes)")));
            }
            other => {
                let cx = ctx(&gdoc, n, other.into());
                return Err(unsupported(&cx, format!("element <{other}> in geometry.xml")));
            }
        }
    }
    // A lattice id used as a cell universe would make the id space ambiguous.
    for id in &lattice_ids {
        if universes.contains(id) {
            return Err(ImportError::Invalid(format!("id {id} is both a lattice and the universe of a cell")));
        }
    }
    // OpenMC looks `outer` up per virtual element (tiled at the lattice pitch); the IR places it
    // once. Only a position-independent `outer` is therefore the same thing in both.
    let outers = ir.lattices.iter().map(|(id, l)| (*id, l.outer)).chain(ir.hex_lattices.iter().map(|(id, l)| (*id, l.outer)));
    for (id, outer) in outers {
        let id = &id;
        if let Some(u) = outer
            && !homogeneous(&ir, u, &mut BTreeSet::new())
        {
            let cx = Ctx { line: lattice_lines.get(id).copied().unwrap_or(0), what: format!("lattice {id}") };
            return Err(unsupported(&cx, format!("outer universe {u} depends on position (OpenMC tiles `outer` at the lattice pitch around the array, this reader places it once); only an `outer` whose cells have no region, recursively, is supported")));
        }
    }
    let roots: Vec<u32> = universes.iter().copied().filter(|u| !filled.contains(u)).collect();
    ir.root_universe = match roots.as_slice() {
        [r] => *r,
        [] => return Err(ImportError::Invalid("every universe is filled by some cell or lattice: no root universe".into())),
        many => return Err(ImportError::Invalid(format!("more than one root universe: {many:?} (no cell or lattice fills them)"))),
    };
    materials(materials_xml, &used_materials, &mut ir, &mut dropped)?;
    for (k, n) in dropped {
        ir.dropped.push(count_label(n, k));
    }
    Ok(ir)
}

/// True when every cell of universe `u` has a region that is all of space and any universe it fills
/// is homogeneous too (a lattice fill is positional).
fn homogeneous(ir: &CsgIr, u: u32, seen: &mut BTreeSet<u32>) -> bool {
    fn all_space(e: &crate::ir::Expr) -> bool {
        use crate::ir::Expr;
        match e {
            Expr::And(v) => v.iter().all(all_space),
            Expr::Or(v) => v.iter().any(all_space),
            _ => false,
        }
    }
    if !seen.insert(u) {
        return false;
    }
    ir.cells.values().filter(|c| c.universe == u).all(|c| {
        all_space(&c.region)
            && match &c.fill {
                Fill::Universe { id, .. } => homogeneous(ir, *id, seen),
                Fill::Lattice { .. } => false,
                _ => true,
            }
    })
}

fn surface(doc: &Document<'_>, n: Node<'_, '_>, ir: &mut CsgIr) -> Result<(), ImportError> {
    let mut cx = ctx(doc, n, "surface".into());
    let id = id_of(&cx, n)?;
    cx.what = format!("surface {id}");
    if id == 0 {
        return Err(invalid(&cx, "surface id 0 cannot be written in a region".into()));
    }
    let ty = field(n, "type").ok_or_else(|| invalid(&cx, "missing type".into()))?;
    let coeffs = numbers(&cx, "coeffs", field(n, "coeffs").ok_or_else(|| invalid(&cx, "missing coeffs".into()))?)?;
    let flag = match field(n, "boundary").unwrap_or("transmission") {
        "transmission" => BoundaryFlag::None,
        "vacuum" => BoundaryFlag::Vacuum,
        "reflective" | "reflect" => BoundaryFlag::Reflective,
        "white" => BoundaryFlag::White,
        "periodic" => return Err(unsupported(&cx, "periodic boundary".into())),
        other => return Err(invalid(&cx, format!("unknown boundary {other:?}"))),
    };
    let s = surfaces::build(&cx, ty, &coeffs)?.with_flag(flag);
    if ir.surfaces.insert(id, s).is_some() {
        return Err(invalid(&cx, "duplicate surface id".into()));
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn cell(
    doc: &Document<'_>,
    n: Node<'_, '_>,
    ir: &mut CsgIr,
    lattice_ids: &BTreeSet<u32>,
    filled: &mut BTreeSet<u32>,
    universes: &mut BTreeSet<u32>,
    used_materials: &mut BTreeSet<u32>,
    dropped: &mut BTreeMap<&'static str, usize>,
) -> Result<(), ImportError> {
    let mut cx = ctx(doc, n, "cell".into());
    let id = id_of(&cx, n)?;
    cx.what = format!("cell {id}");
    let uni = match field(n, "universe") {
        None => 0,
        Some(s) => s.parse::<u32>().map_err(|_| invalid(&cx, format!("bad universe {s:?}")))?,
    };
    let region = region::parse(&cx, field(n, "region").unwrap_or(""))?;
    let fill = field(n, "fill");
    let material = field(n, "material");
    for (name, key) in [("temperature", "cell temperature"), ("volume", "cell volume")] {
        if field(n, name).is_some() {
            *dropped.entry(key).or_default() += 1;
        }
    }
    let tr = field(n, "translation");
    let rot = field(n, "rotation");
    let c = match (fill, material) {
        (Some(_), Some(_)) => return Err(invalid(&cx, "has both fill and material".into())),
        (None, None) => return Err(invalid(&cx, "has neither fill nor material".into())),
        (Some(f), None) => {
            let fid = f.parse::<u32>().map_err(|_| invalid(&cx, format!("bad fill {f:?}")))?;
            let xf = xform::fill_transform(&cx, tr, rot)?;
            filled.insert(fid);
            let fill = if lattice_ids.contains(&fid) { Fill::Lattice { id: fid, xf } } else { Fill::Universe { id: fid, xf } };
            Cell::filled(uni, region, fill)
        }
        (None, Some(m)) => {
            if tr.is_some() || rot.is_some() {
                return Err(invalid(&cx, "translation and rotation apply to filled cells only".into()));
            }
            let mat = if m == "void" {
                None
            } else if m.split_whitespace().count() > 1 {
                return Err(unsupported(&cx, format!("a material list ({m:?}); one material per cell")));
            } else {
                let v = m.parse::<u32>().map_err(|_| invalid(&cx, format!("bad material {m:?}")))?;
                used_materials.insert(v);
                Some(v)
            };
            Cell::new(uni, region, mat)
        }
    };
    universes.insert(uni);
    if ir.cells.insert(id, c).is_some() {
        return Err(invalid(&cx, "duplicate cell id".into()));
    }
    Ok(())
}

fn lattice(doc: &Document<'_>, n: Node<'_, '_>, ir: &mut CsgIr, lattice_ids: &BTreeSet<u32>, filled: &mut BTreeSet<u32>) -> Result<(), ImportError> {
    let mut cx = ctx(doc, n, "lattice".into());
    let id = id_of(&cx, n)?;
    cx.what = format!("lattice {id}");
    let req = |k: &str| field(n, k).ok_or_else(|| invalid(&cx, format!("missing {k}")));
    let dim = numbers(&cx, "dimension", req("dimension")?)?;
    let ll = numbers(&cx, "lower_left", req("lower_left")?)?;
    let pitch = numbers(&cx, "pitch", req("pitch")?)?;
    if !(dim.len() == 2 || dim.len() == 3) || ll.len() != dim.len() || pitch.len() != dim.len() {
        return Err(invalid(&cx, "dimension, lower_left and pitch must each have 2 (or all 3) values".into()));
    }
    let mut dims = [1u32; 3];
    for (i, d) in dim.iter().enumerate() {
        if !(d.fract() == 0.0 && *d >= 1.0 && *d <= 1e6) {
            return Err(invalid(&cx, format!("dimension {d} is not a positive integer")));
        }
        dims[i] = *d as u32;
    }
    let total = dims.iter().try_fold(1usize, |a, d| a.checked_mul(*d as usize)).filter(|t| *t <= 1 << 24).ok_or_else(|| invalid(&cx, "too many lattice elements".into()))?;
    if pitch.iter().any(|p| *p <= 0.0) {
        return Err(invalid(&cx, "pitch must be positive".into()));
    }
    let mut lower_left = [0.0; 3];
    let mut p3 = [f64::INFINITY; 3];
    lower_left[..dim.len()].copy_from_slice(&ll);
    p3[..dim.len()].copy_from_slice(&pitch);
    // A 2-D lattice has no z extent: infinite pitch, universes keep the cell's z.
    let ids: Vec<&str> = req("universes")?.split_whitespace().collect();
    if ids.len() != total {
        return Err(invalid(&cx, format!("universes has {} entries, dimension needs {total}", ids.len())));
    }
    let mut uni = Vec::with_capacity(total);
    for t in &ids {
        let u = t.parse::<u32>().map_err(|_| invalid(&cx, format!("bad universe id {t:?}")))?;
        if lattice_ids.contains(&u) {
            return Err(unsupported(&cx, format!("lattice {u} used as an element of a lattice (nested lattices)")));
        }
        filled.insert(u);
        uni.push(u);
    }
    // Text order: layers k = 0.. upward; within a layer rows from the top (highest y) down; each
    // row left to right. The IR wants i fastest, then j upward, then k.
    let [nx, ny, nz] = dims.map(|d| d as usize);
    let mut ordered = vec![0u32; total];
    let mut it = uni.into_iter();
    for k in 0..nz {
        for j in (0..ny).rev() {
            for i in 0..nx {
                ordered[i + nx * (j + ny * k)] = it.next().unwrap_or(0);
            }
        }
    }
    let outer = match field(n, "outer") {
        None => None,
        Some(s) => {
            let u = s.parse::<u32>().map_err(|_| invalid(&cx, format!("bad outer {s:?}")))?;
            if lattice_ids.contains(&u) {
                return Err(unsupported(&cx, format!("lattice {u} used as the outer of a lattice")));
            }
            filled.insert(u);
            Some(u)
        }
    };
    let l = Lattice { lower_left, pitch: p3, dims, universes: ordered, outer };
    if ir.hex_lattices.contains_key(&id) || ir.lattices.insert(id, l).is_some() {
        return Err(invalid(&cx, "duplicate lattice id".into()));
    }
    Ok(())
}

/// Axial `(q, r)` of the entries of one layer in the order of OpenMC's `geometry.xml` text: for
/// `x` the rows from the highest `y` down (`r` descending), each left to right (`q` ascending); for
/// `y` the picture lines from the highest `y` down (`2r + q` descending), each left to right.
fn hex_text_order(orientation: HexOrientation, n_rings: u32) -> Vec<(i32, i32)> {
    let n = n_rings as i32;
    let mut v: Vec<(i32, i32)> = Vec::new();
    for q in -(n - 1)..n {
        for r in -(n - 1)..n {
            if (q + r).abs() < n {
                v.push((q, r));
            }
        }
    }
    match orientation {
        HexOrientation::X => v.sort_by_key(|&(q, r)| (-r, q)),
        HexOrientation::Y => v.sort_by_key(|&(q, r)| (-(2 * r + q), q)),
    }
    v
}

fn hex_lattice(doc: &Document<'_>, n: Node<'_, '_>, ir: &mut CsgIr, lattice_ids: &BTreeSet<u32>, filled: &mut BTreeSet<u32>) -> Result<(), ImportError> {
    let mut cx = ctx(doc, n, "hex_lattice".into());
    let id = id_of(&cx, n)?;
    cx.what = format!("hex_lattice {id}");
    let req = |k: &str| field(n, k).ok_or_else(|| invalid(&cx, format!("missing {k}")));
    let count = |k: &str, v: &str, lo: f64| -> Result<u32, ImportError> {
        let x: f64 = v.parse().ok().filter(|x: &f64| x.fract() == 0.0 && *x >= lo && *x <= 1e4).ok_or_else(|| invalid(&cx, format!("{k} {v:?} is not an integer of at least {lo}")))?;
        Ok(x as u32)
    };
    let n_rings = count("n_rings", req("n_rings")?, 1.0)?;
    // As in OpenMC, the presence of `n_axial` makes the lattice three-dimensional (2 pitches, 3 centre values).
    let (n_axial, is_3d) = match field(n, "n_axial") {
        Some(v) => (count("n_axial", v, 1.0)?, true),
        None => (1, false),
    };
    let orientation = match field(n, "orientation").unwrap_or("y") {
        "x" => HexOrientation::X,
        "y" => HexOrientation::Y,
        other => return Err(invalid(&cx, format!("unknown orientation {other:?}"))),
    };
    let pitch = numbers(&cx, "pitch", req("pitch")?)?;
    let center = numbers(&cx, "center", req("center")?)?;
    let want = if is_3d { 3 } else { 2 };
    if pitch.len() != want - 1 || center.len() != want {
        return Err(invalid(&cx, format!("pitch needs {} and center {want} values{}", want - 1, if is_3d { " (n_axial is given)" } else { "" })));
    }
    if pitch.iter().any(|p| *p <= 0.0) {
        return Err(invalid(&cx, "pitch must be positive".into()));
    }
    let per = HexLattice::elements_per_layer(n_rings);
    let total = per.checked_mul(n_axial as usize).filter(|t| *t <= 1 << 24).ok_or_else(|| invalid(&cx, "too many lattice elements".into()))?;
    let ids: Vec<&str> = req("universes")?.split_whitespace().collect();
    if ids.len() != total {
        return Err(invalid(&cx, format!("universes has {} entries, {n_rings} rings and {n_axial} layers need {total}", ids.len())));
    }
    let text = hex_text_order(orientation, n_rings);
    debug_assert_eq!(text.len(), per);
    let mut ordered = vec![0u32; total];
    for k in 0..n_axial as usize {
        for (t, &(q, r)) in text.iter().enumerate() {
            let tok = ids[k * per + t];
            let u = tok.parse::<u32>().map_err(|_| invalid(&cx, format!("bad universe id {tok:?}")))?;
            if lattice_ids.contains(&u) {
                return Err(unsupported(&cx, format!("lattice {u} used as an element of a lattice (nested lattices)")));
            }
            filled.insert(u);
            let (ring, pos) = HexLattice::ring_position(orientation, n_rings, q, r).ok_or_else(|| invalid(&cx, "internal: element outside the rings".into()))?;
            let at = k * per + HexLattice::ring_offset(n_rings, ring).unwrap_or(0) + pos as usize;
            ordered[at] = u;
        }
    }
    let outer = match field(n, "outer") {
        None => None,
        Some(s) => {
            let u = s.parse::<u32>().map_err(|_| invalid(&cx, format!("bad outer {s:?}")))?;
            if lattice_ids.contains(&u) {
                return Err(unsupported(&cx, format!("lattice {u} used as the outer of a lattice")));
            }
            filled.insert(u);
            Some(u)
        }
    };
    let l = HexLattice {
        center: if is_3d { [center[0], center[1], center[2]] } else { [center[0], center[1], 0.0] },
        pitch: pitch[0],
        z_pitch: if is_3d { pitch[1] } else { f64::INFINITY },
        orientation,
        n_rings,
        n_axial,
        universes: ordered,
        outer,
    };
    l.check().map_err(|e| invalid(&cx, e))?;
    if ir.lattices.contains_key(&id) || ir.hex_lattices.insert(id, l).is_some() {
        return Err(invalid(&cx, "duplicate lattice id".into()));
    }
    Ok(())
}

fn materials(xml: Option<&str>, used: &BTreeSet<u32>, ir: &mut CsgIr, dropped: &mut BTreeMap<&'static str, usize>) -> Result<(), ImportError> {
    let Some(text) = xml else {
        for id in used {
            ir.materials.insert(*id, Material::default());
        }
        return Ok(());
    };
    let doc = parse_xml(text)?;
    let root = doc.root_element();
    if root.tag_name().name() != "materials" {
        return Err(ImportError::Parse { line: 1, col: 1, msg: format!("the root element is <{}>, expected <materials>", root.tag_name().name()) });
    }
    for n in root.children().filter(|n| n.is_element() && n.tag_name().name() == "material") {
        let mut cx = ctx(&doc, n, "material".into());
        let id = id_of(&cx, n)?;
        cx.what = format!("material {id}");
        let name = field(n, "name").map(str::to_owned);
        if ir.materials.insert(id, Material { name }).is_some() {
            return Err(invalid(&cx, "duplicate material id".into()));
        }
        if n.children().any(|c| c.is_element()) || n.attributes().any(|a| a.name() != "id" && a.name() != "name") {
            *dropped.entry("material composition, density and options").or_default() += 1;
        }
    }
    Ok(())
}
