//! Reuse plan (draft D4): which lattices are built once per distinct universe and copied.
//!
//! A lattice is *reusable* when it is a leaf (no lattice inside its elements), placed once,
//! translation-only, every element is a whole box inside the cells that enclose it, and the
//! elements of one universe place the same cells. Everything else keeps the flat path. The plan
//! is pure data (no kernel calls): per lattice the prototype of each distinct universe (the
//! element at the first slot holding it), the other slots, the array box, and the cells that stay
//! in the remainder.

use std::collections::{BTreeMap, BTreeSet};

use crate::deck::{Origin, PSurf, Placed};
use crate::partition::{Reserved, Signs, Tri, eval, surfaces_in};
use sm_csgio::ir::Expr;
use sm_geom_api::SurfaceDef;

/// One distinct universe of a lattice.
#[derive(Clone, Debug)]
pub struct ProtoPlan {
    /// The deck universe.
    pub universe: u32,
    /// The slot whose element is built (the first one, in k, j, i order).
    pub ref_slot: [i32; 3],
    /// The placed cells of the element at `ref_slot` (indices of `Placed::cells`).
    pub cells: Vec<usize>,
    /// The other slots holding the universe, ascending in k, j, i order.
    pub slots: Vec<[i32; 3]>,
}

/// One reusable lattice.
#[derive(Clone, Debug)]
pub struct GroupPlan {
    /// The deck lattice id.
    pub lattice: u32,
    /// The array box (hull of the element cells).
    pub hull: Reserved,
    /// Element size per axis (0 on an unbounded axis).
    pub pitch: [f64; 3],
    /// One prototype per distinct universe, ascending by universe id.
    pub protos: Vec<ProtoPlan>,
}

/// The reuse plan of a deck.
#[derive(Clone, Debug, Default)]
pub struct ReusePlan {
    /// The reusable lattices.
    pub groups: Vec<GroupPlan>,
    /// The cells that are built flat (everything not inside a reusable lattice).
    pub rest: Vec<usize>,
}

impl ReusePlan {
    /// The array boxes of the groups.
    pub fn reserved(&self) -> Vec<Reserved> {
        self.groups.iter().map(|g| g.hull).collect()
    }
}

/// `(position in the path, lattice, slot)` of the innermost lattice element of an origin.
fn last_element(o: &Origin) -> Option<(usize, u32, [i32; 3])> {
    o.path.iter().enumerate().rev().find_map(|(p, h)| h.element.map(|(l, ix)| (p, l, ix)))
}

/// The origin of the same cell at another slot of its innermost lattice.
pub fn at_slot(o: &Origin, slot: [i32; 3]) -> Option<Origin> {
    let (p, l, _) = last_element(o)?;
    let mut n = o.clone();
    n.path[p].element = Some((l, slot));
    Some(n)
}

/// The signed value of a placed surface at `p` (plane: along the normal; cylinder: distance to
/// the axis minus the radius). `None` for other kinds.
fn signed(def: &SurfaceDef, p: [f64; 3]) -> Option<f64> {
    match def {
        SurfaceDef::Plane(pl) => {
            let (o, n) = (pl.frame.origin, pl.frame.z);
            Some((p[0] - o.x) * n.x + (p[1] - o.y) * n.y + (p[2] - o.z) * n.z)
        }
        SurfaceDef::Cylinder(c) => {
            let (o, z) = (c.frame.origin, c.frame.z);
            let d = [p[0] - o.x, p[1] - o.y, p[2] - o.z];
            let t = d[0] * z.x + d[1] * z.y + d[2] * z.z;
            let r = [d[0] - t * z.x, d[1] - t * z.y, d[2] - t * z.z];
            Some((r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt() - c.radius)
        }
        _ => None,
    }
}

/// True when every expression of `clips` holds over the whole closed box `hull`. Only provable
/// signs count: a plane with the box on one closed side, a cylinder with all corners inside.
fn clips_hold(clips: &[Expr], surfaces: &[PSurf], hull: Reserved) -> bool {
    let mut ids = BTreeSet::new();
    for c in clips {
        surfaces_in(c, &mut ids);
    }
    let corners: Vec<[f64; 3]> = (0..8).map(|m| [0, 1, 2].map(|a| if m >> a & 1 == 1 { hull.1[a] } else { hull.0[a] })).collect();
    let mut signs = Signs::new();
    for s in ids {
        let def = &surfaces[s as usize].def;
        let Some(v) = corners.iter().map(|&c| signed(def, c)).collect::<Option<Vec<f64>>>() else { continue };
        let tol = 1e-9;
        let (mn, mx) = v.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &x| (a.min(x), b.max(x)));
        match def {
            SurfaceDef::Plane(_) if mn >= -tol => {
                signs.insert(s, true);
            }
            SurfaceDef::Plane(_) if mx <= tol => {
                signs.insert(s, false);
            }
            SurfaceDef::Cylinder(_) if mx <= tol => {
                signs.insert(s, false);
            }
            _ => {}
        }
    }
    clips.iter().all(|c| eval(c, &signs) == Tri::T)
}

/// Plan the reuse of `placed`.
pub fn plan(placed: &Placed) -> ReusePlan {
    // lattice -> slot -> cells (by innermost element hop); lattices that contain a lattice.
    let mut by_lattice: BTreeMap<u32, BTreeMap<[i32; 3], Vec<usize>>> = BTreeMap::new();
    let mut nested: BTreeSet<u32> = BTreeSet::new();
    for (ci, c) in placed.cells.iter().enumerate() {
        let elems: Vec<(u32, [i32; 3])> = c.origin.path.iter().filter_map(|h| h.element).collect();
        if let Some(&(l, ix)) = elems.last() {
            by_lattice.entry(l).or_default().entry(ix).or_default().push(ci);
            nested.extend(elems[..elems.len() - 1].iter().map(|e| e.0));
        }
    }
    let mut out = ReusePlan::default();
    let mut taken: BTreeSet<usize> = BTreeSet::new();
    for lp in &placed.lattices {
        if lp.rotated || lp.placements != 1 || nested.contains(&lp.id) {
            continue;
        }
        let Some(slots) = by_lattice.get(&lp.id) else { continue };
        let g = &lp.geometry;
        let n_slots: usize = g.dims.iter().map(|d| *d as usize).product();
        if slots.len() != n_slots || lp.universes.len() != n_slots {
            continue;
        }
        let ids: Vec<usize> = slots.values().flatten().copied().collect();
        let hb = placed.bounds_of_cells(&ids);
        if !hb.finite() {
            continue;
        }
        let hull = (hb.lo, hb.hi);
        if !clips_hold(&lp.clips, &placed.surfaces, hull) {
            continue;
        }
        let [nx, ny, _] = g.dims;
        // Slots by universe, in k, j, i order.
        let mut by_uni: BTreeMap<u32, Vec<[i32; 3]>> = BTreeMap::new();
        for ix in 0..n_slots {
            let (i, j, k) = (ix % nx as usize, (ix / nx as usize) % ny as usize, ix / (nx as usize * ny as usize));
            by_uni.entry(lp.universes[ix]).or_default().push([i as i32, j as i32, k as i32]);
        }
        // Same placed cells at every slot of a universe (the origins match apart from the slot).
        let shape = |slot: [i32; 3]| -> Option<Vec<Origin>> {
            let mut v: Vec<Origin> = slots.get(&slot)?.iter().map(|&c| at_slot(&placed.cells[c].origin, [0, 0, 0])).collect::<Option<_>>()?;
            v.sort();
            Some(v)
        };
        let mut protos = Vec::new();
        let mut ok = true;
        for (u, list) in &by_uni {
            let r = list[0];
            let Some(want) = shape(r) else {
                ok = false;
                break;
            };
            if list.iter().any(|s| shape(*s).as_ref() != Some(&want)) {
                ok = false;
                break;
            }
            protos.push(ProtoPlan { universe: *u, ref_slot: r, cells: slots[&r].clone(), slots: list[1..].to_vec() });
        }
        // Reuse pays only when some universe fills more than one slot.
        if !ok || protos.len() == n_slots {
            continue;
        }
        let pitch = g.pitch.map(|x| if x.is_finite() { x } else { 0.0 });
        taken.extend(ids);
        out.groups.push(GroupPlan { lattice: lp.id, hull, pitch, protos });
    }
    out.rest = (0..placed.cells.len()).filter(|c| !taken.contains(c)).collect();
    out
}
