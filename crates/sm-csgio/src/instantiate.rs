//! Instantiation: the universe tree and lattices of a [`CsgIr`] become one flat leaf.
//!
//! Every placed copy of a material cell becomes one flat cell. Its region is the cell's region
//! ANDed with the regions of every cell it was reached through and with the box of the lattice
//! element it sits in, so universes that are infinite are clipped where they are used. Surfaces
//! are moved with `SurfaceDef::transformed`, shared when identical, and then put in canonical
//! order: surfaces sorted by definition bytes and renumbered (unreferenced ones dropped),
//! half-spaces within a term and terms within a region sorted. The result does not depend on
//! deck ids or on the order cells were written.

use crate::dnf::{Term, to_dnf};
use crate::error::{CellSource, PathStep};
use crate::ir::{BoundaryFlag, CsgIr, Expr, Fill, Lattice, Sign, axis_plane_def};
use crate::ImportError;
use sm_core::{Point3, Transform, Vector3};
use sm_geom_api::{Frame, Plane, Sense, ShapeValue, SurfaceDef};
use std::collections::{BTreeMap, BTreeSet};

/// A placed material cell in canonical surface numbering (cells not yet in canonical order).
pub(crate) struct RawCell {
    pub terms: Vec<Term>,
    pub material: Option<u32>,
    pub source: CellSource,
}

pub(crate) struct Instantiated {
    /// Surfaces in canonical order.
    pub defs: Vec<SurfaceDef>,
    /// Boundary flag per surface.
    pub flags: Vec<BoundaryFlag>,
    pub cells: Vec<RawCell>,
}

/// Maximum nesting of universes.
const MAX_DEPTH: usize = 64;

/// Byte key of a definition: its value stream at exact bits, `-0.0` as `0.0`.
fn def_key(def: &SurfaceDef) -> Vec<u8> {
    fn put(k: &mut Vec<u8>, tag: u8, bytes: [u8; 8]) {
        k.push(tag);
        k.extend(bytes);
    }
    let f = |x: f64| (if x == 0.0 { 0.0 } else { x }).to_le_bytes();
    let mut k = Vec::new();
    def.for_each_value(|v| match v {
        ShapeValue::Int(x) => put(&mut k, 1, x.to_le_bytes()),
        ShapeValue::Length(x) => put(&mut k, 2, f(x)),
        ShapeValue::Angle(x) => put(&mut k, 3, f(x)),
        ShapeValue::Unitless(x) => put(&mut k, 4, f(x)),
        ShapeValue::Param(x) => put(&mut k, 5, f(x)),
        // A future kind of value: key on its text rather than drop it.
        other => k.extend(format!("{other:?}").bytes()),
    });
    k
}

/// A plane in canonical form: normal with its first nonzero component positive, frame
/// `Frame::from_z` at the point nearest the origin. Returns the sense flip.
fn canonical_plane(p: &Plane) -> Result<(SurfaceDef, bool), ImportError> {
    let (mut n, o) = (p.frame.z, p.frame.origin);
    let mut d = n.x * o.x + n.y * o.y + n.z * o.z;
    let first = [n.x, n.y, n.z].into_iter().find(|c| *c != 0.0).unwrap_or(1.0);
    let flip = first < 0.0;
    if flip {
        n = Vector3::new(-n.x, -n.y, -n.z);
        d = -d;
    }
    let frame = Frame::from_z(Point3::new(n.x * d, n.y * d, n.z * d), n).map_err(|e| ImportError::Invalid(format!("invalid plane: {e}")))?;
    Ok((SurfaceDef::Plane(Plane { frame }), flip))
}

struct Placer<'a> {
    ir: &'a CsgIr,
    max_cells: usize,
    by_universe: BTreeMap<u32, Vec<u32>>,
    defs: Vec<SurfaceDef>,
    keys: Vec<Vec<u8>>,
    flags: Vec<BoundaryFlag>,
    index: BTreeMap<Vec<u8>, u32>,
    cells: Vec<(Vec<Term>, Option<u32>, CellSource)>,
    stack: Vec<u32>,
    path: Vec<PathStep>,
}

fn walk_surfaces(e: &Expr, f: &mut impl FnMut(u32)) {
    match e {
        Expr::Half(s, _) => f(*s),
        Expr::Not(x) => walk_surfaces(x, f),
        Expr::And(v) | Expr::Or(v) => v.iter().for_each(|x| walk_surfaces(x, f)),
    }
}

fn validate(ir: &CsgIr, by_universe: &BTreeMap<u32, Vec<u32>>, max_cells: usize) -> Result<(), ImportError> {
    if !by_universe.contains_key(&ir.root_universe) {
        return Err(ImportError::UnknownUniverse { cell: None, universe: ir.root_universe });
    }
    for (id, c) in &ir.cells {
        let mut bad = None;
        walk_surfaces(&c.region, &mut |s| {
            if bad.is_none() && !ir.surfaces.contains_key(&s) {
                bad = Some(s);
            }
        });
        if let Some(surface) = bad {
            return Err(ImportError::UnknownSurface { cell: *id, surface });
        }
        if let Some(m) = c.material
            && !ir.materials.contains_key(&m)
        {
            return Err(ImportError::UnknownMaterial { cell: *id, material: m });
        }
        match &c.fill {
            Fill::None => {}
            Fill::Universe { id: u, .. } => {
                if !by_universe.contains_key(u) {
                    return Err(ImportError::UnknownUniverse { cell: Some(*id), universe: *u });
                }
            }
            Fill::Lattice { id: l, .. } => {
                if !ir.lattices.contains_key(l) {
                    return Err(ImportError::UnknownLattice { cell: *id, lattice: *l });
                }
            }
        }
    }
    for (id, l) in &ir.lattices {
        let n: u64 = l.dims.iter().map(|d| u64::from(*d)).product();
        if l.dims.contains(&0) || n > (1 << 32) {
            return Err(ImportError::Invalid(format!("lattice {id} has dimensions {:?}", l.dims)));
        }
        if l.universes.len() as u64 != n {
            return Err(ImportError::Invalid(format!("lattice {id} lists {} universes for {:?} elements", l.universes.len(), l.dims)));
        }
        for a in 0..3 {
            let p = l.pitch[a];
            let infinite_ok = l.dims[a] == 1 && p == f64::INFINITY;
            if !(infinite_ok || (p.is_finite() && p > 0.0)) || !(l.lower_left[a].is_finite() || infinite_ok) {
                return Err(ImportError::Invalid(format!("lattice {id} has pitch {:?} and lower left {:?}", l.pitch, l.lower_left)));
            }
        }
        if let Some(u) = l.universes.iter().chain(l.outer.iter()).find(|u| !by_universe.contains_key(u)) {
            return Err(ImportError::Invalid(format!("lattice {id} uses undefined universe {u}")));
        }
        if n > (max_cells as u64).saturating_mul(1024) {
            return Err(ImportError::TooManyCells { limit: max_cells });
        }
    }
    Ok(())
}

impl Placer<'_> {
    /// Store id of `def` placed by `t`, and whether the sense is reversed.
    fn store(&mut self, def: &SurfaceDef, t: &Transform, flag: BoundaryFlag) -> Result<(u32, bool), ImportError> {
        let (mut moved, sense) = if *t == Transform::IDENTITY {
            (def.clone(), Sense::Forward)
        } else {
            def.transformed(t).map_err(|e| ImportError::Invalid(format!("cannot place a surface: {e}")))?
        };
        let mut flip = sense == Sense::Reversed;
        if let SurfaceDef::Plane(p) = &moved {
            let (d, f) = canonical_plane(p)?;
            moved = d;
            flip ^= f;
        }
        let key = def_key(&moved);
        let id = match self.index.get(&key) {
            Some(i) => *i,
            None => {
                let i = self.defs.len() as u32;
                self.defs.push(moved);
                self.keys.push(key.clone());
                self.flags.push(BoundaryFlag::None);
                self.index.insert(key, i);
                i
            }
        };
        if flag != BoundaryFlag::None {
            let cur = &mut self.flags[id as usize];
            if *cur != BoundaryFlag::None && *cur != flag {
                return Err(ImportError::Invalid("one surface carries two different boundary conditions".into()));
            }
            *cur = flag;
        }
        Ok((id, flip))
    }

    fn map_expr(&mut self, e: &Expr, t: &Transform, memo: &mut BTreeMap<u32, (u32, bool)>) -> Result<Expr, ImportError> {
        Ok(match e {
            Expr::Half(s, sign) => {
                let (id, flip) = match memo.get(s) {
                    Some(x) => *x,
                    None => {
                        let surf = self.ir.surfaces.get(s).ok_or_else(|| ImportError::Invalid(format!("undefined surface {s}")))?;
                        let x = self.store(&surf.def.clone(), t, surf.flag)?;
                        memo.insert(*s, x);
                        x
                    }
                };
                let sign = match (sign, flip) {
                    (Sign::Negative, false) | (Sign::Positive, true) => Sign::Negative,
                    _ => Sign::Positive,
                };
                Expr::Half(id, sign)
            }
            Expr::Not(x) => Expr::Not(Box::new(self.map_expr(x, t, memo)?)),
            Expr::And(v) => Expr::And(v.iter().map(|x| self.map_expr(x, t, memo)).collect::<Result<_, _>>()?),
            Expr::Or(v) => Expr::Or(v.iter().map(|x| self.map_expr(x, t, memo)).collect::<Result<_, _>>()?),
        })
    }

    /// The half-space `axis >= c` (`upper == false`) or `axis <= c` (`upper`) in the frame of `t`.
    fn axis_half(&mut self, axis: usize, c: f64, upper: bool, t: &Transform) -> Result<Expr, ImportError> {
        let (id, flip) = self.store(&axis_plane_def(axis, c), t, BoundaryFlag::None)?;
        // Positive side of the placed plane is `axis > c` unless the sense is reversed.
        let want_positive = !upper;
        Ok(Expr::Half(id, if want_positive != flip { Sign::Positive } else { Sign::Negative }))
    }

    fn push_cell(&mut self, cell: u32, parts: Vec<Expr>, material: Option<u32>) -> Result<(), ImportError> {
        let terms = to_dnf(&Expr::And(parts)).map_err(|e| ImportError::TooManyTerms { cell, terms: e.0 })?;
        if terms.is_empty() {
            return Ok(());
        }
        if self.cells.len() >= self.max_cells {
            return Err(ImportError::TooManyCells { limit: self.max_cells });
        }
        self.cells.push((terms, material, CellSource { cell, path: self.path.clone() }));
        Ok(())
    }

    fn place_universe(&mut self, uid: u32, t: &Transform, clips: &[Expr]) -> Result<(), ImportError> {
        if self.stack.contains(&uid) {
            return Err(ImportError::UniverseCycle { universe: uid });
        }
        if self.stack.len() >= MAX_DEPTH {
            return Err(ImportError::Invalid(format!("universes are nested deeper than {MAX_DEPTH}")));
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
                Fill::None => self.push_cell(cid, parts, cell.material)?,
                Fill::Universe { id, xf } => {
                    self.path.push(PathStep { cell: cid, element: None });
                    let r = self.place_universe(*id, &t.compose(xf), &parts);
                    self.path.pop();
                    r?;
                }
                Fill::Lattice { id, xf } => {
                    let lat = &ir.lattices[id];
                    self.place_lattice(cid, lat, &t.compose(xf), &parts)?;
                }
            }
        }
        self.stack.pop();
        Ok(())
    }

    fn place_lattice(&mut self, cid: u32, lat: &Lattice, t: &Transform, clips: &[Expr]) -> Result<(), ImportError> {
        let [nx, ny, nz] = lat.dims;
        let bound = |lat: &Lattice, a: usize| lat.pitch[a].is_finite();
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let idx = [i as usize, j as usize, k as usize];
                    let uid = lat.universes[(k as usize * ny as usize + j as usize) * nx as usize + i as usize];
                    let mut parts: Vec<Expr> = clips.to_vec();
                    let mut centre = [0.0; 3];
                    let mut walls = Vec::new();
                    for a in 0..3 {
                        if bound(lat, a) {
                            // Adjacent elements compute the shared wall from the same expression.
                            let lo = lat.lower_left[a] + idx[a] as f64 * lat.pitch[a];
                            let hi = lat.lower_left[a] + (idx[a] + 1) as f64 * lat.pitch[a];
                            centre[a] = 0.5 * (lo + hi);
                            walls.push(self.axis_half(a, lo, false, t)?);
                            walls.push(self.axis_half(a, hi, true, t)?);
                        }
                    }
                    parts.extend(walls);
                    let el = t.compose(&Transform::translation(Vector3::new(centre[0], centre[1], centre[2])));
                    self.path.push(PathStep { cell: cid, element: Some([i as i32, j as i32, k as i32]) });
                    let r = self.place_universe(uid, &el, &parts);
                    self.path.pop();
                    r?;
                }
            }
        }
        if let Some(outer) = lat.outer {
            let mut bx = Vec::new();
            for a in 0..3 {
                if bound(lat, a) {
                    let lo = lat.lower_left[a];
                    let hi = lat.lower_left[a] + f64::from(lat.dims[a]) * lat.pitch[a];
                    bx.push(self.axis_half(a, lo, false, t)?);
                    bx.push(self.axis_half(a, hi, true, t)?);
                }
            }
            let mut parts: Vec<Expr> = clips.to_vec();
            parts.push(Expr::Not(Box::new(Expr::And(bx))));
            self.path.push(PathStep { cell: cid, element: None });
            let r = self.place_universe(outer, t, &parts);
            self.path.pop();
            r?;
        }
        Ok(())
    }
}

/// Flatten `ir` and put the result in canonical order.
pub(crate) fn instantiate(ir: &CsgIr, max_cells: usize) -> Result<Instantiated, ImportError> {
    let mut by_universe: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for (id, c) in &ir.cells {
        by_universe.entry(c.universe).or_default().push(*id);
    }
    validate(ir, &by_universe, max_cells)?;
    let mut p = Placer { ir, max_cells, by_universe, defs: Vec::new(), keys: Vec::new(), flags: Vec::new(), index: BTreeMap::new(), cells: Vec::new(), stack: Vec::new(), path: Vec::new() };
    p.place_universe(ir.root_universe, &Transform::IDENTITY, &[])?;

    // Canonical surface order: referenced surfaces only, sorted by definition bytes.
    let used: BTreeSet<u32> = p.cells.iter().flat_map(|(t, _, _)| t.iter().flatten().map(|l| l.0)).collect();
    let mut order: Vec<u32> = used.into_iter().collect();
    order.sort_by(|a, b| p.keys[*a as usize].cmp(&p.keys[*b as usize]));
    let mut renum = vec![u32::MAX; p.defs.len()];
    for (new, old) in order.iter().enumerate() {
        renum[*old as usize] = new as u32;
    }
    let defs = order.iter().map(|o| p.defs[*o as usize].clone()).collect();
    let flags = order.iter().map(|o| p.flags[*o as usize]).collect();
    let cells = p
        .cells
        .into_iter()
        .map(|(terms, material, source)| {
            let mut terms: Vec<Term> = terms
                .into_iter()
                .map(|t| {
                    let mut t: Term = t.into_iter().map(|(s, sg)| (renum[s as usize], sg)).collect();
                    t.sort();
                    t
                })
                .collect();
            terms.sort();
            terms.dedup();
            RawCell { terms, material, source }
        })
        .collect();
    Ok(Instantiated { defs, flags, cells })
}
