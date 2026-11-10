//! The deck placed in the global frame: canonical surfaces and cell regions as expressions over
//! them, with the origin of every cell (deck cell, lattice elements) for diagnostics and tags.
//!
//! This mirrors sm-csgio's flattening (`instantiate.rs`) but keeps the regions as expressions (no
//! DNF, so no term cap) and keeps the kernel surfaces: only planes and cylinders are cut with
//! (sphere, cone and torus decks are `Unsupported` until the sm-boolean tools land). A quadric
//! that classifies as a plane or a cylinder is replaced by the exact surface.

use std::collections::{BTreeMap, BTreeSet};

use crate::error::CsgbrepError;
use crate::quadric::{DeckSurface, classify};
use sm_core::{Point3, Transform, Vector3};
use sm_csgio::ir::{BoundaryFlag, CsgIr, Expr, Fill, Lattice, Sign};
use sm_csgio::resolve_region_names;
use sm_geom_api::{Cylinder, Frame, ImplicitDef, Plane, Sense, ShapeValue, SurfaceDef};

/// Maximum nesting of universes.
const MAX_DEPTH: usize = 64;

/// One level of an [`Origin`] path.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Hop {
    /// The deck cell whose fill was entered.
    pub cell: u32,
    /// The lattice and the element `[i, j, k]` entered, when the fill is a lattice.
    pub element: Option<(u32, [i32; 3])>,
}

/// Where a placed cell comes from: the deck cell and the fills it was reached through.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Origin {
    /// The deck cell that carries the material.
    pub cell: u32,
    /// The filled cells (and lattice elements), outermost first.
    pub path: Vec<Hop>,
}

impl core::fmt::Display for Origin {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for h in &self.path {
            match h.element {
                Some((_, [i, j, k])) => write!(f, "{}[{i},{j},{k}]/", h.cell)?,
                None => write!(f, "{}/", h.cell)?,
            }
        }
        write!(f, "{}", self.cell)
    }
}

/// A placed surface: the kernel definition (a plane or a cylinder), its flag and the deck surface
/// it first came from.
#[derive(Clone, Debug)]
pub struct PSurf {
    /// Plane or cylinder; the positive side is the side of the plane normal, or the outside.
    pub def: SurfaceDef,
    /// Boundary condition.
    pub flag: BoundaryFlag,
    /// The deck surface id this one was placed from first (for messages).
    pub deck_id: u32,
}

/// A placed cell.
#[derive(Clone, Debug)]
pub struct PCell {
    /// The region over placed surface ids.
    pub expr: Expr,
    /// The region name (ruling C9).
    pub name: String,
    /// Where it comes from.
    pub origin: Origin,
}

/// The deck data of a lattice that a frame needs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LatticeGeom {
    /// Elements per axis.
    pub dims: [u32; 3],
    /// Element size (infinite on an unbounded one-element axis).
    pub pitch: [f64; 3],
    /// Lower-left corner of element `[0, 0, 0]` in the lattice frame.
    pub lower_left: [f64; 3],
}

/// Where a lattice was first placed.
#[derive(Clone, Debug)]
pub struct LatticePlacement {
    /// Deck data of the lattice.
    pub geometry: LatticeGeom,
    /// The deck lattice id.
    pub id: u32,
    /// Lattice frame to global frame.
    pub xf: Transform,
    /// The enclosing lattice (by deck id), if any.
    pub parent: Option<u32>,
    /// More than one placement of this lattice exists.
    pub placements: usize,
    /// The placements differ in their parent lattice.
    pub parents_differ: bool,
    /// The placement carries a rotation (or other non-translation).
    pub rotated: bool,
    /// The universe of each element, `[i + nx * (j + ny * k)]` (deck order).
    pub universes: Vec<u32>,
    /// The parent clips (region parts of the enclosing cells) of the first placement.
    pub clips: Vec<Expr>,
}

/// The placed deck.
#[derive(Debug)]
pub struct Placed {
    /// Surfaces in canonical order (planes first, then by definition bytes).
    pub surfaces: Vec<PSurf>,
    /// Cells in canonical order.
    pub cells: Vec<PCell>,
    /// Lattices in ascending deck id.
    pub lattices: Vec<LatticePlacement>,
}

fn axis_vec(a: usize) -> Vector3 {
    let mut v = [0.0; 3];
    v[a] = 1.0;
    Vector3::new(v[0], v[1], v[2])
}

fn axis_plane(axis: usize, c: f64) -> SurfaceDef {
    let mut o = [0.0; 3];
    o[axis] = c;
    // `from_z` cannot fail for a unit axis.
    let frame = Frame::from_z(Point3::new(o[0], o[1], o[2]), axis_vec(axis)).unwrap_or_else(|_| Frame::at(Point3::new(o[0], o[1], o[2])));
    SurfaceDef::Plane(Plane { frame })
}

/// Byte key of a definition (exact bits, `-0.0` as `0.0`), prefixed by its kind.
fn def_key(def: &SurfaceDef) -> Vec<u8> {
    let f = |x: f64| (if x == 0.0 { 0.0 } else { x }).to_le_bytes();
    let mut k = vec![match def {
        SurfaceDef::Plane(_) => 0u8,
        _ => 1u8,
    }];
    def.for_each_value(|v| match v {
        ShapeValue::Int(x) => {
            k.push(1);
            k.extend(x.to_le_bytes());
        }
        ShapeValue::Length(x) => {
            k.push(2);
            k.extend(f(x));
        }
        ShapeValue::Angle(x) => {
            k.push(3);
            k.extend(f(x));
        }
        ShapeValue::Unitless(x) => {
            k.push(4);
            k.extend(f(x));
        }
        ShapeValue::Param(x) => {
            k.push(5);
            k.extend(f(x));
        }
        other => k.extend(format!("{other:?}").bytes()),
    });
    k
}

fn invalid(e: impl core::fmt::Display) -> CsgbrepError {
    CsgbrepError::Invalid(e.to_string())
}

/// Plane in canonical form: unit normal with its first nonzero component positive, frame at the
/// point nearest the origin. Returns the sense flip.
fn canonical_plane(p: &Plane) -> Result<(SurfaceDef, bool), CsgbrepError> {
    let (mut n, o) = (p.frame.z, p.frame.origin);
    let mut d = n.x * o.x + n.y * o.y + n.z * o.z;
    let first = [n.x, n.y, n.z].into_iter().find(|c| *c != 0.0).unwrap_or(1.0);
    let flip = first < 0.0;
    if flip {
        n = Vector3::new(-n.x, -n.y, -n.z);
        d = -d;
    }
    let frame = Frame::from_z(Point3::new(n.x * d, n.y * d, n.z * d), n).map_err(invalid)?;
    Ok((SurfaceDef::Plane(Plane { frame }), flip))
}

/// Cylinder in canonical form: axis direction with its first nonzero component positive, origin
/// the axis point nearest the world origin (so a cylinder shifted along its own axis is the same
/// surface), default frame.
fn canonical_cylinder(c: &Cylinder) -> Result<SurfaceDef, CsgbrepError> {
    let mut z = c.frame.z;
    let first = [z.x, z.y, z.z].into_iter().find(|v| *v != 0.0).unwrap_or(1.0);
    if first < 0.0 {
        z = Vector3::new(-z.x, -z.y, -z.z);
    }
    let o = c.frame.origin;
    let t = o.x * z.x + o.y * z.y + o.z * z.z;
    let p = Point3::new(o.x - t * z.x, o.y - t * z.y, o.z - t * z.z);
    let frame = Frame::from_z(p, z).map_err(invalid)?;
    Ok(SurfaceDef::Cylinder(Cylinder::new(frame, c.radius).map_err(invalid)?))
}

struct Placer<'a> {
    ir: &'a CsgIr,
    names: BTreeMap<u32, String>,
    max_cells: usize,
    by_universe: BTreeMap<u32, Vec<u32>>,
    defs: Vec<SurfaceDef>,
    keys: Vec<Vec<u8>>,
    flags: Vec<BoundaryFlag>,
    deck_ids: Vec<u32>,
    index: BTreeMap<Vec<u8>, usize>,
    cells: Vec<(Expr, Option<u32>, Origin)>,
    stack: Vec<u32>,
    path: Vec<Hop>,
    lattice_stack: Vec<u32>,
    lattices: BTreeMap<u32, LatticePlacement>,
}

fn unsupported(id: u32, what: String) -> CsgbrepError {
    CsgbrepError::Unsupported { line: None, what: format!("surface {id}: {what}") }
}

impl Placer<'_> {
    /// Store id of `def` placed by `t`, and whether the sense is reversed.
    fn store(&mut self, deck_id: u32, def: &SurfaceDef, t: &Transform, flag: BoundaryFlag) -> Result<(usize, bool), CsgbrepError> {
        let (mut moved, sense) = if *t == Transform::IDENTITY { (def.clone(), Sense::Forward) } else { def.transformed(t).map_err(|e| CsgbrepError::Invalid(format!("surface {deck_id}: cannot place: {e}")))? };
        let mut flip = sense == Sense::Reversed;
        if let SurfaceDef::Implicit(ImplicitDef::Quadric(q)) = &moved {
            let c = classify(q, DeckSurface { id: deck_id, line: None }).map_err(|e| CsgbrepError::Unsupported { line: None, what: e.to_string() })?;
            match c.surfaces.as_slice() {
                [s @ (SurfaceDef::Plane(_) | SurfaceDef::Cylinder(_))] => {
                    moved = s.clone();
                    flip ^= c.sign < 0.0;
                }
                [SurfaceDef::Sphere(_)] => return Err(unsupported(deck_id, "a quadric that is a sphere needs the sphere cut tool (W4-BOOL-TOOLS)".into())),
                [SurfaceDef::EllipticCylinder(_)] => return Err(unsupported(deck_id, "a quadric that is an elliptic cylinder needs the elliptic cut tool (W4-BOOL-TOOLS)".into())),
                _ => return Err(unsupported(deck_id, "a quadric that is a cone needs the cone cut tool (W4-BOOL-TOOLS)".into())),
            }
        }
        match &moved {
            SurfaceDef::Plane(p) => {
                let (d, f) = canonical_plane(p)?;
                moved = d;
                flip ^= f;
            }
            SurfaceDef::Cylinder(c) => moved = canonical_cylinder(c)?,
            SurfaceDef::Sphere(_) => return Err(unsupported(deck_id, "a sphere needs the sphere cut tool (W4-BOOL-TOOLS)".into())),
            SurfaceDef::Cone(_) | SurfaceDef::EllipticCone(_) => return Err(unsupported(deck_id, "a cone needs the cone cut tool (W4-BOOL-TOOLS)".into())),
            SurfaceDef::Torus(_) => return Err(unsupported(deck_id, "a torus needs the torus cut tool (W4-BOOL-TOOLS)".into())),
            other => return Err(unsupported(deck_id, format!("surface kind {other:?} is not supported by the import"))),
        }
        let key = def_key(&moved);
        let id = match self.index.get(&key) {
            Some(i) => *i,
            None => {
                let i = self.defs.len();
                self.defs.push(moved);
                self.keys.push(key.clone());
                self.flags.push(BoundaryFlag::None);
                self.deck_ids.push(deck_id);
                self.index.insert(key, i);
                i
            }
        };
        if flag != BoundaryFlag::None {
            let cur = &mut self.flags[id];
            if *cur != BoundaryFlag::None && *cur != flag {
                return Err(CsgbrepError::Invalid(format!("surface {deck_id}: one surface carries two different boundary conditions")));
            }
            *cur = flag;
        }
        Ok((id, flip))
    }

    fn map_expr(&mut self, e: &Expr, t: &Transform, memo: &mut BTreeMap<u32, (usize, bool)>) -> Result<Expr, CsgbrepError> {
        Ok(match e {
            Expr::Half(s, sign) => {
                let (id, flip) = match memo.get(s) {
                    Some(x) => *x,
                    None => {
                        let surf = self.ir.surfaces.get(s).ok_or_else(|| CsgbrepError::Invalid(format!("undefined surface {s}")))?;
                        let x = self.store(*s, &surf.def.clone(), t, surf.flag)?;
                        memo.insert(*s, x);
                        x
                    }
                };
                let sign = match (sign, flip) {
                    (Sign::Negative, false) | (Sign::Positive, true) => Sign::Negative,
                    _ => Sign::Positive,
                };
                Expr::Half(id as u32, sign)
            }
            Expr::Not(x) => Expr::Not(Box::new(self.map_expr(x, t, memo)?)),
            Expr::And(v) => Expr::And(v.iter().map(|x| self.map_expr(x, t, memo)).collect::<Result<_, _>>()?),
            Expr::Or(v) => Expr::Or(v.iter().map(|x| self.map_expr(x, t, memo)).collect::<Result<_, _>>()?),
        })
    }

    /// The half-space `axis >= c` (`upper == false`) or `axis <= c` (`upper`) in the frame of `t`.
    fn axis_half(&mut self, axis: usize, c: f64, upper: bool, t: &Transform) -> Result<Expr, CsgbrepError> {
        let (id, flip) = self.store(0, &axis_plane(axis, c), t, BoundaryFlag::None)?;
        let want_positive = !upper;
        Ok(Expr::Half(id as u32, if want_positive != flip { Sign::Positive } else { Sign::Negative }))
    }

    fn push_cell(&mut self, parts: Vec<Expr>, material: Option<u32>, cell: u32) -> Result<(), CsgbrepError> {
        if self.cells.len() >= self.max_cells {
            return Err(CsgbrepError::Invalid(format!("the flat model has more than {} cells", self.max_cells)));
        }
        self.cells.push((Expr::And(parts), material, Origin { cell, path: self.path.clone() }));
        Ok(())
    }

    fn place_universe(&mut self, uid: u32, t: &Transform, clips: &[Expr]) -> Result<(), CsgbrepError> {
        if self.stack.contains(&uid) {
            return Err(CsgbrepError::Invalid(format!("universe {uid} contains itself")));
        }
        if self.stack.len() >= MAX_DEPTH {
            return Err(CsgbrepError::Invalid(format!("universes are nested deeper than {MAX_DEPTH}")));
        }
        self.stack.push(uid);
        let ir = self.ir;
        let mut memo = BTreeMap::new();
        let ids = self.by_universe.get(&uid).cloned().unwrap_or_default();
        for cid in ids {
            let cell = &ir.cells[&cid];
            let region = self.map_expr(&cell.region, t, &mut memo)?;
            let mut parts: Vec<Expr> = clips.to_vec();
            parts.push(region);
            match &cell.fill {
                Fill::None => self.push_cell(parts, cell.material, cid)?,
                Fill::Universe { id, xf } => {
                    self.path.push(Hop { cell: cid, element: None });
                    let r = self.place_universe(*id, &t.compose(xf), &parts);
                    self.path.pop();
                    r?;
                }
                Fill::Lattice { id, xf } => {
                    let lat = &ir.lattices[id];
                    self.place_lattice(cid, *id, lat, &t.compose(xf), &parts)?;
                }
            }
        }
        self.stack.pop();
        Ok(())
    }

    fn note_lattice(&mut self, id: u32, lat: &Lattice, t: &Transform, clips: &[Expr]) {
        let parent = self.lattice_stack.last().copied();
        let rotated = !is_translation(t);
        match self.lattices.get_mut(&id) {
            Some(p) => {
                p.placements += 1;
                if p.parent != parent {
                    p.parents_differ = true;
                }
            }
            None => {
                let geometry = LatticeGeom { dims: lat.dims, pitch: lat.pitch, lower_left: lat.lower_left };
                self.lattices.insert(id, LatticePlacement { geometry, id, xf: *t, parent, placements: 1, parents_differ: false, rotated, universes: lat.universes.clone(), clips: clips.to_vec() });
            }
        }
    }

    fn place_lattice(&mut self, cid: u32, lid: u32, lat: &Lattice, t: &Transform, clips: &[Expr]) -> Result<(), CsgbrepError> {
        if self.lattice_stack.contains(&lid) {
            return Err(CsgbrepError::Invalid(format!("lattice {lid} contains itself")));
        }
        self.note_lattice(lid, lat, t, clips);
        self.lattice_stack.push(lid);
        let [nx, ny, nz] = lat.dims;
        let bound = |a: usize| lat.pitch[a].is_finite();
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let idx = [i as usize, j as usize, k as usize];
                    let uid = lat.universes[(k as usize * ny as usize + j as usize) * nx as usize + i as usize];
                    let mut parts: Vec<Expr> = clips.to_vec();
                    let mut centre = [0.0; 3];
                    for a in 0..3 {
                        if bound(a) {
                            let lo = lat.lower_left[a] + idx[a] as f64 * lat.pitch[a];
                            let hi = lat.lower_left[a] + (idx[a] + 1) as f64 * lat.pitch[a];
                            centre[a] = 0.5 * (lo + hi);
                            parts.push(self.axis_half(a, lo, false, t)?);
                            parts.push(self.axis_half(a, hi, true, t)?);
                        }
                    }
                    let el = t.compose(&Transform::translation(Vector3::new(centre[0], centre[1], centre[2])));
                    self.path.push(Hop { cell: cid, element: Some((lid, [i as i32, j as i32, k as i32])) });
                    let r = self.place_universe(uid, &el, &parts);
                    self.path.pop();
                    r?;
                }
            }
        }
        if let Some(outer) = lat.outer {
            let mut bx = Vec::new();
            for a in 0..3 {
                if bound(a) {
                    let lo = lat.lower_left[a];
                    let hi = lat.lower_left[a] + f64::from(lat.dims[a]) * lat.pitch[a];
                    bx.push(self.axis_half(a, lo, false, t)?);
                    bx.push(self.axis_half(a, hi, true, t)?);
                }
            }
            let mut parts: Vec<Expr> = clips.to_vec();
            parts.push(Expr::Not(Box::new(Expr::And(bx))));
            self.path.push(Hop { cell: cid, element: None });
            let r = self.place_universe(outer, t, &parts);
            self.path.pop();
            r?;
        }
        self.lattice_stack.pop();
        Ok(())
    }
}

/// True when `t` is a pure translation (linear part the identity).
fn is_translation(t: &Transform) -> bool {
    let o = t.apply_point(Point3::new(0.0, 0.0, 0.0));
    [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)].iter().all(|&(x, y, z)| {
        let p = t.apply_point(Point3::new(x, y, z));
        (p.x - o.x - x).abs() < 1e-12 && (p.y - o.y - y).abs() < 1e-12 && (p.z - o.z - z).abs() < 1e-12
    })
}

fn expr_key(e: &Expr, out: &mut String) {
    match e {
        Expr::Half(s, sg) => out.push_str(&format!("{}{s}", if *sg == Sign::Positive { '+' } else { '-' })),
        Expr::Not(x) => {
            out.push('!');
            expr_key(x, out);
        }
        Expr::And(v) | Expr::Or(v) => {
            out.push(if matches!(e, Expr::And(_)) { '(' } else { '[' });
            for x in v {
                expr_key(x, out);
                out.push(',');
            }
            out.push(')');
        }
    }
}

fn remap(e: &Expr, renum: &[u32]) -> Expr {
    match e {
        Expr::Half(s, sg) => Expr::Half(renum[*s as usize], *sg),
        Expr::Not(x) => Expr::Not(Box::new(remap(x, renum))),
        Expr::And(v) => Expr::And(sorted(v.iter().map(|x| remap(x, renum)).collect())),
        Expr::Or(v) => Expr::Or(sorted(v.iter().map(|x| remap(x, renum)).collect())),
    }
}

fn sorted(mut v: Vec<Expr>) -> Vec<Expr> {
    v.sort_by_cached_key(|e| {
        let mut s = String::new();
        expr_key(e, &mut s);
        s
    });
    v.dedup();
    v
}

fn used(e: &Expr, into: &mut BTreeSet<u32>) {
    match e {
        Expr::Half(s, _) => {
            into.insert(*s);
        }
        Expr::Not(x) => used(x, into),
        Expr::And(v) | Expr::Or(v) => v.iter().for_each(|x| used(x, into)),
    }
}

/// Place the deck: universes and lattices flattened into cells over canonical global surfaces.
///
/// # Errors
/// [`CsgbrepError::Unsupported`] for a surface the cutter does not handle (naming the deck
/// surface), [`CsgbrepError::Invalid`] for a structurally bad deck (unknown ids, cycles) or more
/// than `max_cells` placed cells.
pub fn place(ir: &CsgIr, material_names: &BTreeMap<u32, String>, max_cells: usize) -> Result<Placed, CsgbrepError> {
    let names = resolve_region_names(&ir.materials, material_names).map_err(invalid)?;
    let mut by_universe: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for (id, c) in &ir.cells {
        by_universe.entry(c.universe).or_default().push(*id);
    }
    if !by_universe.contains_key(&ir.root_universe) {
        return Err(CsgbrepError::Invalid(format!("the root universe {} has no cells", ir.root_universe)));
    }
    for (id, c) in &ir.cells {
        if let Some(m) = c.material
            && !ir.materials.contains_key(&m)
        {
            return Err(CsgbrepError::Invalid(format!("cell {id} uses undefined material {m}")));
        }
        match &c.fill {
            Fill::Universe { id: u, .. } if !by_universe.contains_key(u) => return Err(CsgbrepError::Invalid(format!("cell {id} fills with undefined universe {u}"))),
            Fill::Lattice { id: l, .. } if !ir.lattices.contains_key(l) => return Err(CsgbrepError::Invalid(format!("cell {id} fills with undefined lattice {l}"))),
            _ => {}
        }
    }
    for (id, l) in &ir.lattices {
        let n: u64 = l.dims.iter().map(|d| u64::from(*d)).product();
        if l.dims.contains(&0) || l.universes.len() as u64 != n {
            return Err(CsgbrepError::Invalid(format!("lattice {id} has dimensions {:?} and {} universes", l.dims, l.universes.len())));
        }
        for a in 0..3 {
            let p = l.pitch[a];
            if !((l.dims[a] == 1 && p == f64::INFINITY) || (p.is_finite() && p > 0.0 && l.lower_left[a].is_finite())) {
                return Err(CsgbrepError::Invalid(format!("lattice {id} has pitch {:?} and lower left {:?}", l.pitch, l.lower_left)));
            }
        }
        if let Some(u) = l.universes.iter().chain(l.outer.iter()).find(|u| !by_universe.contains_key(u)) {
            return Err(CsgbrepError::Invalid(format!("lattice {id} uses undefined universe {u}")));
        }
    }
    let mut p = Placer {
        ir,
        names,
        max_cells,
        by_universe,
        defs: Vec::new(),
        keys: Vec::new(),
        flags: Vec::new(),
        deck_ids: Vec::new(),
        index: BTreeMap::new(),
        cells: Vec::new(),
        stack: Vec::new(),
        path: Vec::new(),
        lattice_stack: Vec::new(),
        lattices: BTreeMap::new(),
    };
    p.place_universe(ir.root_universe, &Transform::IDENTITY, &[])?;

    // Canonical surface order: referenced surfaces only, planes first, then by definition bytes.
    let mut referenced = BTreeSet::new();
    for (e, _, _) in &p.cells {
        used(e, &mut referenced);
    }
    let mut order: Vec<u32> = referenced.into_iter().collect();
    order.sort_by(|a, b| p.keys[*a as usize].cmp(&p.keys[*b as usize]));
    let mut renum = vec![u32::MAX; p.defs.len()];
    for (new, old) in order.iter().enumerate() {
        renum[*old as usize] = new as u32;
    }
    let surfaces = order.iter().map(|o| PSurf { def: p.defs[*o as usize].clone(), flag: p.flags[*o as usize], deck_id: p.deck_ids[*o as usize] }).collect();
    let mut cells: Vec<(String, String, PCell)> = p
        .cells
        .iter()
        .map(|(e, m, o)| {
            let expr = remap(e, &renum);
            let mut key = String::new();
            expr_key(&expr, &mut key);
            let name = m.and_then(|m| p.names.get(&m).cloned()).unwrap_or_else(|| sm_csgio::VOID_REGION.to_string());
            (name.clone(), key, PCell { expr, name, origin: o.clone() })
        })
        .collect();
    cells.sort_by(|a, b| a.0.cmp(&b.0).then_with(|| a.1.cmp(&b.1)).then_with(|| a.2.origin.cmp(&b.2.origin)));
    let mut lattices: Vec<LatticePlacement> = p.lattices.into_values().collect();
    for l in &mut lattices {
        l.clips = l.clips.iter().map(|e| remap(e, &renum)).collect();
    }
    Ok(Placed { surfaces, cells: cells.into_iter().map(|c| c.2).collect(), lattices })
}

/// Axis-aligned bounds of a region with the provenance of each side.
#[derive(Clone, Copy, Debug)]
pub struct Bounds {
    /// Lower corner.
    pub lo: [f64; 3],
    /// Upper corner.
    pub hi: [f64; 3],
    /// The lower side on that axis is an axis-aligned deck plane.
    pub lo_plane: [bool; 3],
    /// The upper side on that axis is an axis-aligned deck plane.
    pub hi_plane: [bool; 3],
}

impl Bounds {
    const ALL: Self = Self { lo: [f64::NEG_INFINITY; 3], hi: [f64::INFINITY; 3], lo_plane: [false; 3], hi_plane: [false; 3] };
    const NONE: Self = Self { lo: [f64::INFINITY; 3], hi: [f64::NEG_INFINITY; 3], lo_plane: [false; 3], hi_plane: [false; 3] };

    fn meet(&mut self, o: &Self) {
        for a in 0..3 {
            if o.lo[a] > self.lo[a] {
                self.lo[a] = o.lo[a];
                self.lo_plane[a] = o.lo_plane[a];
            }
            if o.hi[a] < self.hi[a] {
                self.hi[a] = o.hi[a];
                self.hi_plane[a] = o.hi_plane[a];
            }
        }
    }

    fn join(&mut self, o: &Self) {
        for a in 0..3 {
            if o.lo[a] < self.lo[a] {
                self.lo[a] = o.lo[a];
                self.lo_plane[a] = o.lo_plane[a];
            }
            if o.hi[a] > self.hi[a] {
                self.hi[a] = o.hi[a];
                self.hi_plane[a] = o.hi_plane[a];
            }
        }
    }

    /// True when every side is finite.
    pub fn finite(&self) -> bool {
        (0..3).all(|a| self.lo[a].is_finite() && self.hi[a].is_finite() && self.lo[a] < self.hi[a])
    }
}

fn unit_axis(v: Vector3) -> Option<usize> {
    let a = [v.x.abs(), v.y.abs(), v.z.abs()];
    (0..3).find(|&i| (a[i] - 1.0).abs() < 1e-12 && a[(i + 1) % 3] < 1e-12 && a[(i + 2) % 3] < 1e-12)
}

/// Bounds of the region `e` (infinite where the expression does not bound an axis).
pub fn bounds_of(e: &Expr, surfaces: &[PSurf]) -> Bounds {
    match e {
        Expr::Half(s, sign) => {
            let mut b = Bounds::ALL;
            match &surfaces[*s as usize].def {
                SurfaceDef::Plane(p) => {
                    if let Some(a) = unit_axis(p.frame.z) {
                        let c = [p.frame.origin.x, p.frame.origin.y, p.frame.origin.z][a];
                        let up = p.frame.z.x + p.frame.z.y + p.frame.z.z > 0.0;
                        // The normal is +axis (canonical) or -axis; f < 0 is below along the normal.
                        if (*sign == Sign::Negative) == up {
                            b.hi[a] = c;
                            b.hi_plane[a] = true;
                        } else {
                            b.lo[a] = c;
                            b.lo_plane[a] = true;
                        }
                    }
                }
                SurfaceDef::Cylinder(cy) if *sign == Sign::Negative => {
                    if let Some(ax) = unit_axis(cy.frame.z) {
                        let o = [cy.frame.origin.x, cy.frame.origin.y, cy.frame.origin.z];
                        for a in (0..3).filter(|a| *a != ax) {
                            b.lo[a] = o[a] - cy.radius;
                            b.hi[a] = o[a] + cy.radius;
                        }
                    }
                }
                _ => {}
            }
            b
        }
        Expr::And(v) => {
            let mut b = Bounds::ALL;
            for x in v {
                b.meet(&bounds_of(x, surfaces));
            }
            b
        }
        Expr::Or(v) => {
            let mut b = Bounds::NONE;
            for x in v {
                b.join(&bounds_of(x, surfaces));
            }
            b
        }
        Expr::Not(_) => Bounds::ALL,
    }
}

impl Placed {
    /// The hull of every cell's bounds.
    pub fn bounds(&self) -> Bounds {
        let ids: Vec<usize> = (0..self.cells.len()).collect();
        self.bounds_of_cells(&ids)
    }

    /// The hull of the bounds of the cells `ids`.
    pub fn bounds_of_cells(&self, ids: &[usize]) -> Bounds {
        let mut b = Bounds::NONE;
        for &i in ids {
            b.join(&bounds_of(&self.cells[i].expr, &self.surfaces));
        }
        b
    }
}
