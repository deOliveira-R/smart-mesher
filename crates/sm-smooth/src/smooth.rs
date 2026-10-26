//! The smoothing driver: colouring, sweeps, the Jacobi repair loop, statistics and the outcome.

use rayon::prelude::*;
use sm_mesh_api::{CellIx, Classification, MeshEdit, MeshEntity, MeshRead, Param, PointIx, VertexIx};
use sm_core::Point3;
use sm_quality::{Metric, Options, SurfaceNormals, compute, summarize, validity};

use crate::kernel::{Kernel, Prop};
use crate::moved::Moved;
use crate::normals::CarrierNormals;
use crate::prep::{Mode, Prep, V};
use crate::types::{
    Method, QualityStats, Schedule, SmoothError, SmoothInput, SmoothOutcome, SmoothReport, SmoothRequest,
};

/// Repair passes of the Jacobi schedule before the whole sweep is reverted.
const REPAIR_PASSES: usize = 20;
/// Smallest chunk handed to a rayon task.
const MIN_LEN: usize = 32;

/// Running totals of a run.
#[derive(Default)]
struct Counts {
    accepted: u64,
    rej_guard: u64,
    rej_geom: u64,
    max_move: f64,
}

impl Counts {
    fn tally(&mut self, p: Prop, v: usize, pos: &mut [V], hint: &mut [[f64; 2]], rel: &dyn Fn(usize, V, V) -> f64) {
        match p {
            Prop::Stay => {}
            Prop::Move { p, h } => {
                self.max_move = self.max_move.max(rel(v, pos[v], p));
                pos[v] = p;
                hint[v] = h;
                self.accepted += 1;
            }
            Prop::RejGuard => self.rej_guard += 1,
            Prop::RejGeom => self.rej_geom += 1,
        }
    }
}

/// Greedy first-fit colouring of the movable nodes in ascending index; two nodes conflict when
/// they share a template cell. Returns the nodes of each colour, ascending.
pub(crate) fn colour(prep: &Prep<'_>) -> Vec<Vec<u32>> {
    let mut of = vec![u32::MAX; prep.n];
    let mut seen: Vec<u32> = Vec::new();
    let mut out: Vec<Vec<u32>> = Vec::new();
    for &v in &prep.movable {
        let stamp = v;
        for &c in prep.cells_of(v as usize) {
            for &w in prep.cell_corners(c as usize) {
                let k = of[w as usize];
                if w != v && k != u32::MAX {
                    seen[k as usize] = stamp;
                }
            }
        }
        let k = (0..seen.len()).find(|&k| seen[k] != stamp).unwrap_or(seen.len());
        if k == seen.len() {
            seen.push(u32::MAX);
            out.push(Vec::new());
        }
        of[v as usize] = k as u32;
        out[k].push(v);
    }
    out
}

fn quality(mesh: &dyn MeshRead, topo: &dyn sm_topo_api::TopologyView) -> Result<(QualityStats, Vec<f64>, Vec<CellIx>), SmoothError> {
    let cn = CarrierNormals { topo };
    let opts = Options::default().with_normals(SurfaceNormals::Field(&cn));
    let err = |e: sm_quality::QualityError| SmoothError::BadParams(format!("quality evaluation failed: {e}"));
    let shape = compute(mesh, Metric::Shape, &opts).ok();
    let cond = compute(mesh, Metric::Condition, &opts).ok();
    let val = validity(mesh, &opts).map_err(err)?;
    let fin = |x: f64| if x.is_finite() { x } else { 0.0 };
    let s = shape.as_deref().map(summarize);
    let c = cond.as_deref().map(summarize);
    let stats = QualityStats {
        min_shape: s.map_or(0.0, |s| fin(s.min)),
        mean_shape: s.map_or(0.0, |s| fin(s.mean)),
        max_condition: c.map_or(0.0, |c| fin(c.max)),
        n_inverted: val.n_inverted,
        n_degenerate: val.n_degenerate,
    };
    let inverted: Vec<CellIx> = val.inverted().into_iter().take(10).collect();
    Ok((stats, shape.unwrap_or_default(), inverted))
}

fn worst(shape: &[f64]) -> Vec<CellIx> {
    let mut v: Vec<(f64, usize)> = shape.iter().enumerate().filter(|(_, s)| s.is_finite()).map(|(i, s)| (*s, i)).collect();
    let cmp = |a: &(f64, usize), b: &(f64, usize)| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1));
    if v.len() > 10 {
        v.select_nth_unstable_by(10, cmp);
        v.truncate(10);
    }
    v.sort_by(cmp);
    v.into_iter().map(|(_, i)| CellIx::new(i)).collect()
}

/// Run the smoother. Pure: nothing is written; see [`apply_outcome`].
pub fn smooth(input: &SmoothInput<'_>, req: &SmoothRequest) -> Result<SmoothOutcome, SmoothError> {
    let prep = Prep::build(input, req)?;
    if prep.movable.is_empty() {
        return Err(SmoothError::NothingToSmooth);
    }
    let (before, _, inverted_before) = quality(input.mesh, input.topo)?;

    let co = input.mesh.coords();
    let p0: Vec<V> = (0..prep.n).map(|i| [co.x[i], co.y[i], co.z[i]]).collect();
    let mut pos = p0.clone();
    let mut hint = prep.hint0.clone();
    let kernel = Kernel { prep: &prep, normals: CarrierNormals { topo: input.topo }, floor: req.guard_floor };
    let colours = if req.schedule == Schedule::Colored { colour(&prep) } else { Vec::new() };

    let mut cnt = Counts::default();
    let (mut iterations_run, mut converged, mut max_move_last) = (0u32, false, 0.0f64);
    let rel = |v: usize, a: V, b: V| {
        let d = ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
        if prep.ell[v] > 0.0 { d / prep.ell[v] } else { d }
    };
    let method: &Method = &req.method;
    for _ in 0..req.iterations {
        cnt.max_move = 0.0;
        match req.schedule {
            Schedule::Colored => {
                for col in &colours {
                    let props: Vec<Prop> = col
                        .par_iter()
                        .with_min_len(MIN_LEN)
                        .map(|&v| kernel.propose(v as usize, &pos, &hint, method))
                        .collect();
                    for (&v, p) in col.iter().zip(props) {
                        cnt.tally(p, v as usize, &mut pos, &mut hint, &rel);
                    }
                }
            }
            Schedule::Jacobi => {
                let props: Vec<Prop> = prep
                    .movable
                    .par_iter()
                    .with_min_len(MIN_LEN)
                    .map(|&v| kernel.propose(v as usize, &pos, &hint, method))
                    .collect();
                let old = pos.clone();
                let old_hint = hint.clone();
                let mut moved: Vec<u32> = Vec::new();
                for (&v, p) in prep.movable.iter().zip(props) {
                    if let Prop::Move { p: q, h } = p {
                        pos[v as usize] = q;
                        hint[v as usize] = h;
                        moved.push(v);
                    } else {
                        cnt.tally(p, v as usize, &mut pos, &mut hint, &rel);
                    }
                }
                let reverted = repair(&prep, &kernel, &old, &mut pos, &mut hint, &old_hint, moved.clone());
                cnt.rej_guard += reverted.len() as u64;
                for &v in &moved {
                    if reverted.binary_search(&v).is_err() {
                        cnt.max_move = cnt.max_move.max(rel(v as usize, old[v as usize], pos[v as usize]));
                        cnt.accepted += 1;
                    }
                }
            }
        }
        iterations_run += 1;
        max_move_last = cnt.max_move;
        if cnt.max_move < req.tolerance || cnt.max_move == 0.0 {
            converged = true;
            break;
        }
    }
    let Counts { accepted, rej_guard, rej_geom, .. } = cnt;

    // Outcome.
    let mut classes = Vec::new();
    for &v in &prep.movable {
        let i = v as usize;
        if pos[i] == p0[i] {
            continue;
        }
        let param = match prep.mode[i] {
            Mode::Curve => Param::Curve(hint[i][0]),
            Mode::Surf => Param::Surface(hint[i]),
            _ => continue,
        };
        classes.push((VertexIx::new(i), Classification { entity: prep.vcls[i].entity, param }));
    }
    let after_mesh = Moved::new(input.mesh, &pos);
    let (after, shape_after, _) = quality(&after_mesh, input.topo)?;
    let report = SmoothReport {
        iterations_run,
        converged,
        max_move_last,
        movable: prep.movable.len(),
        pinned: prep.pinned.clone(),
        pinned_entities: prep.pinned_entities.clone(),
        accepted,
        rejected_guard: rej_guard,
        rejected_geometry: rej_geom,
        before,
        after,
        worst_after: worst(&shape_after),
        inverted_before,
    };
    Ok(SmoothOutcome { points: pos, classes, report })
}

/// Jacobi repair: revert the moved nodes of every cell that violates the guard against the
/// pre-sweep state, up to [`REPAIR_PASSES`] times, then the whole sweep if violations remain.
/// Returns the reverted nodes, ascending.
fn repair(
    prep: &Prep<'_>,
    kernel: &Kernel<'_, '_>,
    old: &[V],
    pos: &mut [V],
    hint: &mut [[f64; 2]],
    old_hint: &[[f64; 2]],
    moved: Vec<u32>,
) -> Vec<u32> {
    let mut live = vec![false; prep.n];
    for &v in &moved {
        live[v as usize] = true;
    }
    let mut reverted: Vec<u32> = Vec::new();
    let mut stamp = vec![false; prep.shape.len()];
    for pass in 0..=REPAIR_PASSES {
        // Cells touched by a still-moved node, ascending.
        let mut cells: Vec<u32> = Vec::new();
        for &v in &moved {
            if live[v as usize] {
                for &c in prep.cells_of(v as usize) {
                    if !stamp[c as usize] {
                        stamp[c as usize] = true;
                        cells.push(c);
                    }
                }
            }
        }
        for &c in &cells {
            stamp[c as usize] = false;
        }
        cells.sort_unstable();
        let bad: Vec<u32> = cells
            .par_iter()
            .with_min_len(MIN_LEN)
            .copied()
            .filter(|&c| kernel.cell_violates(c as usize, old, pos))
            .collect();
        if bad.is_empty() {
            break;
        }
        let mut undo: Vec<u32> = Vec::new();
        if pass == REPAIR_PASSES {
            undo.extend(moved.iter().copied().filter(|&v| live[v as usize]));
        } else {
            for &c in &bad {
                for &w in prep.cell_corners(c as usize) {
                    if live[w as usize] {
                        live[w as usize] = false;
                        undo.push(w);
                    }
                }
            }
        }
        for &w in &undo {
            live[w as usize] = false;
            pos[w as usize] = old[w as usize];
            hint[w as usize] = old_hint[w as usize];
        }
        reverted.extend(undo);
    }
    reverted.sort_unstable();
    reverted.dedup();
    reverted
}

/// Write an outcome into an editable mesh: `set_point` for each moved vertex, then one `set_classes`.
pub fn apply_outcome(m: &mut dyn MeshEdit, o: &SmoothOutcome) -> Result<(), SmoothError> {
    if o.points.len() != m.vertex_count() {
        return Err(SmoothError::BadParams(format!(
            "outcome has {} points, the mesh {} vertices",
            o.points.len(),
            m.vertex_count()
        )));
    }
    for (i, p) in o.points.iter().enumerate() {
        let c = m.coords().get(i);
        if c.x.to_bits() != p[0].to_bits() || c.y.to_bits() != p[1].to_bits() || c.z.to_bits() != p[2].to_bits() {
            m.set_point(PointIx::new(i), Point3::new(p[0], p[1], p[2]))?;
        }
    }
    if !o.classes.is_empty() {
        let items: Vec<(MeshEntity, Classification)> =
            o.classes.iter().map(|(v, c)| (MeshEntity::Vertex(*v), *c)).collect();
        m.set_classes(&items)?;
    }
    Ok(())
}
