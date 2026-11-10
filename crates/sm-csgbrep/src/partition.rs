//! The space-partition builder (draft D2): start from one box, cut the pieces by the placed
//! surfaces, classify each piece by evaluating the cell regions on its sign vector (no point
//! sampling), then unite the pieces of one cell that share a face.
//!
//! Every cut is `sm_boolean::webcut` with `merge: true`, so the cut faces are shared and the
//! result is conformal by construction. A piece is cut by a surface only while some cell that can
//! still own it depends on that surface (three-valued evaluation), which keeps the fragment
//! count near the number of deck cells.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use crate::deck::{Origin, PSurf, Placed};
use crate::error::CsgbrepError;
use sm_boolean::{BoolOptions, BoolRule, BooleanError, CutTool, WebcutOptions, boolean, webcut};
use sm_core::{EntityId, Point3};
use sm_csgio::ir::{BoundaryFlag, Expr, Sign};
use sm_geom_api::{Cylinder, Frame, SurfaceDef};
use sm_layer_api::TopoHost;
use sm_topo_api::{Carrier, Dim, fixtures::brick};

/// What to do with a piece that two or more cells own.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum OverlapPolicy {
    /// Fail naming the cells (default; the tracker import does the same).
    #[default]
    Error,
    /// Give the piece to the first cell in canonical order.
    FirstWins,
    /// As `FirstWins`, and report each overlap as a warning.
    Report,
}

/// Options of [`build`].
#[derive(Clone, Copy, Debug, Default)]
pub struct BuildOptions {
    /// Overlap policy.
    pub overlap: OverlapPolicy,
    /// Bounds `[lo, hi]` of the start box; `None` derives them from the cells.
    pub bounds: Option<([f64; 3], [f64; 3])>,
}

/// A finding that is not an error.
#[derive(Clone, Debug, PartialEq)]
pub struct Note {
    /// Stable code (`import.gap`, `import.overlap`).
    pub code: &'static str,
    /// Human text.
    pub message: String,
}

/// Counters of the build (work counters, never wall clock).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    /// `webcut` calls.
    pub cut_calls: usize,
    /// Calls that split a piece in two (the kernel cuts).
    pub cuts: usize,
    /// Pieces classified.
    pub pieces: usize,
    /// Unions of same-cell pieces.
    pub unions: usize,
}

/// One final volume of the build.
#[derive(Clone, Debug)]
pub struct Vol {
    /// The volume.
    pub volume: EntityId,
    /// Its body.
    pub body: EntityId,
    /// Index of the owning cell in [`Placed::cells`].
    pub cell: usize,
}

/// The result of [`build`]. Everything alive at the end, ascending by body id.
#[derive(Debug, Default)]
pub struct Built {
    /// The final volumes.
    pub vols: Vec<Vol>,
    /// Exterior faces on boundary-flagged surfaces: vacuum, reflective and white (ascending).
    pub boundary_faces: [Vec<EntityId>; 3],
    /// Findings.
    pub notes: Vec<Note>,
    /// Counters.
    pub stats: Stats,
    /// The start box `[lo, hi]` the model was cut from.
    pub start: ([f64; 3], [f64; 3]),
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Tri {
    T,
    F,
    U,
}

fn not(t: Tri) -> Tri {
    match t {
        Tri::T => Tri::F,
        Tri::F => Tri::T,
        Tri::U => Tri::U,
    }
}

type Signs = BTreeMap<u32, bool>;

fn eval(e: &Expr, s: &Signs) -> Tri {
    match e {
        Expr::Half(id, sign) => match s.get(id) {
            Some(&pos) => {
                if pos == (*sign == Sign::Positive) {
                    Tri::T
                } else {
                    Tri::F
                }
            }
            None => Tri::U,
        },
        Expr::Not(x) => not(eval(x, s)),
        Expr::And(v) => {
            let mut all = true;
            for x in v {
                match eval(x, s) {
                    Tri::F => return Tri::F,
                    Tri::U => all = false,
                    Tri::T => {}
                }
            }
            if all { Tri::T } else { Tri::U }
        }
        Expr::Or(v) => {
            let mut none = true;
            for x in v {
                match eval(x, s) {
                    Tri::T => return Tri::T,
                    Tri::U => none = false,
                    Tri::F => {}
                }
            }
            if none { Tri::F } else { Tri::U }
        }
    }
}

fn surfaces_in(e: &Expr, out: &mut BTreeSet<u32>) {
    match e {
        Expr::Half(s, _) => {
            out.insert(*s);
        }
        Expr::Not(x) => surfaces_in(x, out),
        Expr::And(v) | Expr::Or(v) => v.iter().for_each(|x| surfaces_in(x, out)),
    }
}

/// Which sides of each surface the cells use: `(positive used, negative used)`.
fn sides_used(e: &Expr, flip: bool, out: &mut BTreeMap<u32, (bool, bool)>) {
    match e {
        Expr::Half(s, sign) => {
            let pos = (*sign == Sign::Positive) != flip;
            let u = out.entry(*s).or_default();
            if pos {
                u.0 = true;
            } else {
                u.1 = true;
            }
        }
        Expr::Not(x) => sides_used(x, !flip, out),
        Expr::And(v) | Expr::Or(v) => v.iter().for_each(|x| sides_used(x, flip, out)),
    }
}

struct Piece {
    body: EntityId,
    signs: Signs,
}

fn kernel(what: &str, e: BooleanError) -> CsgbrepError {
    CsgbrepError::Kernel(format!("{what}: {e}"))
}

fn topo_err(e: impl core::fmt::Display) -> CsgbrepError {
    CsgbrepError::Kernel(e.to_string())
}

fn volume_of_body(topo: &dyn TopoHost, body: EntityId) -> Option<EntityId> {
    topo.boundary(body).iter().map(|i| i.cell).find(|c| topo.dim_of(*c) == Some(Dim::Volume))
}

/// The start box `[lo, hi]`: the hull of the cell bounds, enlarged by 10% on every side that is
/// not an axis-aligned deck plane (a curved extreme would otherwise be tangent to a box face).
fn start_box(placed: &Placed, opts: &BuildOptions) -> Result<([f64; 3], [f64; 3]), CsgbrepError> {
    if let Some(b) = opts.bounds {
        return Ok(b);
    }
    let b = placed.bounds();
    if !b.finite() {
        return Err(CsgbrepError::Unbounded(format!("the bounds derived from the cells are {:?} to {:?}; give `options.bounds`", b.lo, b.hi)));
    }
    let ext = (0..3).map(|a| b.hi[a] - b.lo[a]).fold(0.0_f64, f64::max);
    let m = 0.1 * ext;
    let lo = [0, 1, 2].map(|a| if b.lo_plane[a] { b.lo[a] } else { b.lo[a] - m });
    let hi = [0, 1, 2].map(|a| if b.hi_plane[a] { b.hi[a] } else { b.hi[a] + m });
    Ok((lo, hi))
}

fn same_surface(a: &SurfaceDef, b: &SurfaceDef, tol: f64) -> bool {
    match (a, b) {
        (SurfaceDef::Plane(p), SurfaceDef::Plane(q)) => {
            let (n, m) = (p.frame.z, q.frame.z);
            n.dot(m).abs() >= 1.0 - 1e-12 && (p.frame.origin - q.frame.origin).dot(n).abs() <= tol
        }
        (SurfaceDef::Cylinder(p), SurfaceDef::Cylinder(q)) => {
            let (z, w) = (p.frame.z, q.frame.z);
            z.dot(w).abs() >= 1.0 - 1e-12 && (p.radius - q.radius).abs() <= tol && (q.frame.origin - p.frame.origin).cross(z).norm() <= tol
        }
        _ => false,
    }
}

/// Seam reference turn (radians) of a cylinder cut tool. With the default frame, the kernel fails
/// on some concentric cuts of a pin (the W4-REGIONS-CUT finding); the same surface with its x
/// reference turned avoids it. The surface is unchanged.
const SEAM_TURN: f64 = 1.0;

/// The kernel tool for a placed surface: planes as they are, cylinders with the seam turned.
fn tool_def(def: &SurfaceDef) -> Result<SurfaceDef, CsgbrepError> {
    match def {
        SurfaceDef::Cylinder(c) => {
            let x = c.frame.x * SEAM_TURN.cos() + c.frame.y * SEAM_TURN.sin();
            let frame = Frame::from_zx(c.frame.origin, c.frame.z, x).map_err(topo_err)?;
            Ok(SurfaceDef::Cylinder(Cylinder::new(frame, c.radius).map_err(topo_err)?))
        }
        other => Ok(other.clone()),
    }
}

fn flag_slot(f: BoundaryFlag) -> Option<usize> {
    match f {
        BoundaryFlag::Vacuum => Some(0),
        BoundaryFlag::Reflective => Some(1),
        BoundaryFlag::White => Some(2),
        BoundaryFlag::None => None,
    }
}

/// Build the B-rep volumes of `placed` inside the open operation of `topo`.
///
/// # Errors
/// `Unbounded` when no finite box exists, `Overlap` under the default policy, `Kernel` when a cut
/// or union is refused (the message names the deck surface).
pub fn build(topo: &mut dyn TopoHost, placed: &Placed, opts: &BuildOptions) -> Result<Built, CsgbrepError> {
    let (lo, hi) = start_box(placed, opts)?;
    let size = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
    let b = brick(topo, Point3::new(lo[0], lo[1], lo[2]), size).map_err(topo_err)?;
    let mut out = Built { start: (lo, hi), ..Built::default() };

    // One-sided flagged surfaces: the unused side is outside the model.
    let mut used = BTreeMap::new();
    for c in &placed.cells {
        sides_used(&c.expr, false, &mut used);
    }
    let outside: Vec<(u32, bool)> = used
        .iter()
        .filter(|(s, (p, n))| p != n && placed.surfaces[**s as usize].flag != BoundaryFlag::None)
        .map(|(s, (p, _))| (*s, !*p))
        .collect();

    // ---- cut ----
    let mut queue: VecDeque<Piece> = VecDeque::new();
    queue.push_back(Piece { body: b.body, signs: Signs::new() });
    let mut done: Vec<(Piece, Vec<usize>)> = Vec::new();
    while let Some(piece) = queue.pop_front() {
        let tri: Vec<Tri> = placed.cells.iter().map(|c| eval(&c.expr, &piece.signs)).collect();
        let owners: Vec<usize> = (0..tri.len()).filter(|&i| tri[i] == Tri::T).collect();
        let unknown: Vec<usize> = (0..tri.len()).filter(|&i| tri[i] == Tri::U).collect();
        if unknown.is_empty() {
            done.push((piece, owners));
            continue;
        }
        let mut open = BTreeSet::new();
        for &i in &unknown {
            surfaces_in(&placed.cells[i].expr, &mut open);
        }
        open.retain(|s| !piece.signs.contains_key(s));
        // The first surface whose side changes some candidate's value; else the first open one.
        let mut pick = None;
        for &s in &open {
            let (mut a, mut c) = (piece.signs.clone(), piece.signs.clone());
            a.insert(s, true);
            c.insert(s, false);
            if unknown.iter().any(|&i| eval(&placed.cells[i].expr, &a) != eval(&placed.cells[i].expr, &c)) {
                pick = Some(s);
                break;
            }
        }
        let Some(s) = pick.or_else(|| open.iter().next().copied()) else {
            return Err(CsgbrepError::Invalid("a cell stays undecided with every surface assigned".into()));
        };
        let surf: &PSurf = &placed.surfaces[s as usize];
        out.stats.cut_calls += 1;
        let tool = CutTool::Surface { surface: tool_def(&surf.def)?, window: None };
        let r = webcut(topo, &[piece.body], &tool, &WebcutOptions::default()).map_err(|e| kernel(&format!("cutting by deck surface {}", surf.deck_id), e))?;
        let split = !((r.negative.is_empty() && r.positive == [piece.body]) || (r.positive.is_empty() && r.negative == [piece.body]));
        if split {
            out.stats.cuts += 1;
        }
        for (bodies, pos) in [(&r.negative, false), (&r.positive, true)] {
            for &body in bodies {
                let mut signs = piece.signs.clone();
                signs.insert(s, pos);
                queue.push_back(Piece { body, signs });
            }
        }
    }
    out.stats.pieces = done.len();

    // ---- classify ----
    let mut keep: BTreeMap<usize, Vec<EntityId>> = BTreeMap::new(); // cell -> bodies
    for (piece, owners) in done {
        let vol = volume_of_body(topo, piece.body).ok_or_else(|| CsgbrepError::Invalid("a piece has no volume".into()))?;
        let moments = topo.volume_moments(vol).map_err(topo_err)?;
        match owners.as_slice() {
            [c] => keep.entry(*c).or_default().push(piece.body),
            [] => {
                let beyond = outside.iter().any(|(s, side)| piece.signs.get(s) == Some(side));
                let centre = moments.centroid();
                let in_box = centre.is_none_or(|c| (0..3).all(|a| [c.x, c.y, c.z][a] > lo[a] && [c.x, c.y, c.z][a] < hi[a]));
                if !beyond && in_box {
                    out.notes.push(Note { code: "import.gap", message: format!("no cell contains a piece of volume {:e} near {:?}; the space between cells is not part of the model", moments.volume, centre.map(|c| [c.x, c.y, c.z])) });
                }
                topo.kill_cascade(piece.body).map_err(topo_err)?;
            }
            [a, rest @ ..] => {
                let (na, nb) = (placed.cells[*a].origin.to_string(), placed.cells[rest[0]].origin.to_string());
                match opts.overlap {
                    OverlapPolicy::Error => return Err(CsgbrepError::Overlap { a: na, b: nb, volume: moments.volume }),
                    OverlapPolicy::Report => out.notes.push(Note { code: "import.overlap", message: format!("cells {na} and {nb} overlap in a piece of volume {:e}; kept by the first", moments.volume) }),
                    OverlapPolicy::FirstWins => {}
                }
                keep.entry(*a).or_default().push(piece.body);
            }
        }
    }

    // ---- unite the pieces of one cell that share a face ----
    let mut finals: Vec<(EntityId, usize)> = Vec::new();
    for (cell, mut bodies) in keep {
        bodies.sort_unstable();
        let vols: Vec<EntityId> = bodies.iter().map(|&b| volume_of_body(topo, b).ok_or_else(|| CsgbrepError::Invalid("a piece has no volume".into()))).collect::<Result<_, _>>()?;
        let faces: Vec<BTreeSet<EntityId>> = vols.iter().map(|&v| topo.volume_faces(v).iter().copied().collect()).collect();
        let mut seen = vec![false; bodies.len()];
        for start in 0..bodies.len() {
            if seen[start] {
                continue;
            }
            seen[start] = true;
            // Breadth-first over shared faces, in ascending piece order.
            let mut order = vec![start];
            let mut head = 0;
            while head < order.len() {
                let cur = order[head];
                head += 1;
                for j in 0..bodies.len() {
                    if !seen[j] && !faces[cur].is_disjoint(&faces[j]) {
                        seen[j] = true;
                        order.push(j);
                    }
                }
            }
            let mut cur = bodies[order[0]];
            for &j in &order[1..] {
                let r = boolean(topo, BoolRule::Unite, &[cur], &[bodies[j]], &BoolOptions::default()).map_err(|e| kernel(&format!("uniting the pieces of cell {}", placed.cells[cell].origin), e))?;
                match r.bodies.as_slice() {
                    [one] => cur = *one,
                    other => return Err(CsgbrepError::Kernel(format!("uniting the pieces of cell {} gave {} bodies", placed.cells[cell].origin, other.len()))),
                }
                out.stats.unions += 1;
            }
            finals.push((cur, cell));
        }
    }
    finals.sort_unstable();
    for (body, cell) in finals {
        let volume = volume_of_body(topo, body).ok_or_else(|| CsgbrepError::Invalid("a result has no volume".into()))?;
        out.vols.push(Vol { volume, body, cell });
    }

    // ---- exterior faces on flagged surfaces ----
    let tol = 10.0 * topo.tolerance().linear;
    let flagged: Vec<(usize, &SurfaceDef)> = placed.surfaces.iter().filter_map(|s| flag_slot(s.flag).map(|k| (k, &s.def))).collect();
    let mut seen_faces = BTreeSet::new();
    for v in &out.vols {
        for &f in topo.volume_faces(v.volume) {
            if !seen_faces.insert(f) {
                continue;
            }
            let n_vol = topo.coboundary(f).iter().filter(|i| topo.dim_of(i.cell) == Some(Dim::Volume)).count();
            if n_vol != 1 {
                continue;
            }
            let Carrier::Surface(sc) = topo.carrier(f) else { continue };
            let Some(def) = topo.shapes().surface_def(sc.surface) else { continue };
            if let Some((k, _)) = flagged.iter().find(|(_, d)| same_surface(def, d, tol)) {
                out.boundary_faces[*k].push(f);
            }
        }
    }
    for l in &mut out.boundary_faces {
        l.sort_unstable();
    }
    Ok(out)
}

/// Deck origin of a volume, for messages.
pub fn origin_of<'a>(placed: &'a Placed, v: &Vol) -> &'a Origin {
    &placed.cells[v.cell].origin
}
