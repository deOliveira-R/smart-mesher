//! [`ImportedCsg`]: a flattened, canonical CSG model that tracks through `AnalyticTracker`.

use crate::error::{CellSource, OverlapFinding};
use crate::instantiate::{Instantiated, instantiate};
use crate::ir::{BoundaryFlag, CsgIr, Sign};
use crate::names::{VOID_REGION, resolve_region_names};
use crate::ImportError;
use sm_core::{Aabb, EntityId};
use sm_geom_api::{CsgCell, GeomAppend, HalfSpace, MemoryStore, SurfaceDef};
use sm_rt::{AnalyticTracker, AnalyticTrackerBuilder, MAX_FLAT_CELLS, check_overlaps};
use sm_rt_api::{
    BoundaryDef, BoundarySelector, BoundarySpec, BuildInput, BuildOptions, BuildReport, ComplementPolicy, ComponentId, ComponentKind, ContentHash, ContentHasher, CsgCellDef, CsgLeafInput, CsgRegion, HierInput,
    InstancePolicy, LeafInput, ResponseKind, Tracker, WorldSpec,
};
use std::collections::{BTreeMap, BTreeSet};

/// Sampling effort of the overlap check. The defaults are higher than the `rt.check_overlaps`
/// op's (10 000 points, 500 chords), and no field may go below that op's defaults.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CheckOptions {
    /// Stratified sample points (at least [`MIN_SAMPLES`], at most [`MAX_SAMPLES`]).
    pub samples: u64,
    /// Random chords of 8 points each (at least [`MIN_RAYS`], at most [`MAX_RAYS`]).
    pub rays: u64,
    /// RNG seed. Never derived from time or threads.
    pub seed: u64,
}

/// Fewest sample points the check accepts (the op's default).
pub const MIN_SAMPLES: u64 = 10_000;
/// Fewest chords the check accepts (the op's default).
pub const MIN_RAYS: u64 = 500;
/// Most sample points.
pub const MAX_SAMPLES: u64 = 10_000_000;
/// Most chords.
pub const MAX_RAYS: u64 = 1_000_000;

impl Default for CheckOptions {
    fn default() -> Self {
        Self { samples: 100_000, rays: 2_000, seed: 0 }
    }
}

impl CheckOptions {
    fn validate(&self) -> Result<(), ImportError> {
        if !(MIN_SAMPLES..=MAX_SAMPLES).contains(&self.samples) || !(MIN_RAYS..=MAX_RAYS).contains(&self.rays) {
            return Err(ImportError::Invalid(format!(
                "check effort out of range: samples {} (allowed {MIN_SAMPLES}..={MAX_SAMPLES}), rays {} (allowed {MIN_RAYS}..={MAX_RAYS})",
                self.samples, self.rays
            )));
        }
        Ok(())
    }
}

/// Options of [`ImportedCsg::import`] and [`ImportedCsg::instantiate`].
#[derive(Clone, Debug)]
pub struct ImportOptions {
    /// Check effort.
    pub check: CheckOptions,
    /// Region names that replace the derived ones, by material id (see [`resolve_region_names`]).
    pub material_names: BTreeMap<u32, String>,
    /// Cap on flat cells.
    pub max_cells: usize,
    /// Bounds of the model; when `None` the tracker derives them from the cells (and the import
    /// warns when some cell is not provably bounded).
    pub bounds: Option<Aabb>,
}

impl Default for ImportOptions {
    fn default() -> Self {
        Self { check: CheckOptions::default(), material_names: BTreeMap::new(), max_cells: MAX_FLAT_CELLS, bounds: None }
    }
}

/// A non-fatal finding of an import. A clean import has none (ruling C9).
#[derive(Clone, Debug, PartialEq)]
pub struct Warning {
    /// Stable code: `import.gap`, `import.unbounded`, `import.reflective_curved`.
    pub code: &'static str,
    /// Human text.
    pub message: String,
    /// Where, when the finding has a place.
    pub point: Option<[f64; 3]>,
}

/// What an import found besides the model.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct ImportReport {
    /// Warnings, in the order found.
    pub warnings: Vec<Warning>,
    /// Data the deck had and the IR dropped.
    pub dropped: Vec<String>,
    /// The check that ran, when one did.
    pub checked: Option<CheckOptions>,
}

/// Result of the sampled check of an instantiated model.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct CheckOutcome {
    /// Overlapping cells found (at most 16).
    pub overlaps: Vec<OverlapFinding>,
    /// Gaps and unbounded-leaf notes.
    pub warnings: Vec<Warning>,
}

/// A flattened CSG model: one leaf over one store, canonical order, with names and boundaries.
pub struct ImportedCsg {
    store: MemoryStore,
    cells: Vec<CsgCellDef>,
    sources: Vec<CellSource>,
    boundaries: BoundarySpec,
    bounds: Option<Aabb>,
    complement: ComplementPolicy,
    /// Store handles of the surfaces that carry a boundary flag (for the check's `<outside>`).
    flagged: Vec<EntityId>,
    report: ImportReport,
}

impl core::fmt::Debug for ImportedCsg {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ImportedCsg").field("cells", &self.cells.len()).finish()
    }
}

/// Name of the check-only cell beyond the flagged boundaries.
const OUTSIDE_CELL: &str = "<outside>";

fn cell_name(names: &BTreeMap<u32, String>, material: Option<u32>) -> String {
    material.and_then(|m| names.get(&m).cloned()).unwrap_or_else(|| VOID_REGION.to_string())
}

impl ImportedCsg {
    /// Flatten `ir` (no sampling). Cells are named by [`resolve_region_names`] and put in
    /// canonical order, so the result does not depend on deck ids or text order.
    ///
    /// # Errors
    /// The structural [`ImportError`]s (unknown ids, cycles, `TooManyTerms`, `TooManyCells`),
    /// invalid options, and `Invalid` for a deck with no cell left.
    pub fn instantiate(ir: &CsgIr, opts: &ImportOptions) -> Result<Self, ImportError> {
        let names = resolve_region_names(&ir.materials, &opts.material_names)?;
        let Instantiated { defs, flags, cells } = instantiate(ir, opts.max_cells)?;
        if cells.is_empty() {
            return Err(ImportError::Invalid("the deck has no cell with a region".into()));
        }
        let mut store = MemoryStore::new();
        let mut ids = Vec::with_capacity(defs.len());
        for d in &defs {
            ids.push(store.add_surface(d.clone()).map_err(|e| ImportError::Invalid(format!("surface: {e}")))?);
        }
        let mut keyed: Vec<(String, _, CellSource)> = cells.into_iter().map(|c| (cell_name(&names, c.material), c.terms, c.source)).collect();
        keyed.sort_by(|a, b| a.0.cmp(&b.0).then_with(|| a.1.cmp(&b.1)).then_with(|| a.2.cmp(&b.2)));
        let mut out = Vec::with_capacity(keyed.len());
        let mut sources = Vec::with_capacity(keyed.len());
        for (name, terms, source) in keyed {
            let terms: Vec<CsgCell> =
                terms.iter().map(|t| CsgCell::new(t.iter().map(|(s, pos)| HalfSpace { surface: ids[*s as usize], sign: if *pos { Sign::Positive } else { Sign::Negative } }))).collect();
            out.push(CsgCellDef::new(CsgRegion::new(terms), name));
            sources.push(source);
        }
        let mut rules = Vec::new();
        let mut flagged = Vec::new();
        let mut warnings = Vec::new();
        for (i, f) in flags.iter().enumerate() {
            let def = match f {
                BoundaryFlag::None => continue,
                BoundaryFlag::Vacuum => BoundaryDef::Respond(ResponseKind::Vacuum),
                BoundaryFlag::White => BoundaryDef::Respond(ResponseKind::White),
                BoundaryFlag::Reflective => {
                    if !matches!(defs[i], SurfaceDef::Plane(_)) {
                        warnings.push(Warning { code: "import.reflective_curved", message: format!("surface {i} of the flat model is reflective but not planar; reflection is exact on planes only"), point: None });
                    }
                    BoundaryDef::Reflect
                }
            };
            flagged.push(ids[i]);
            rules.push((BoundarySelector::Entity(ids[i]), def));
        }
        Ok(Self {
            store,
            cells: out,
            sources,
            boundaries: BoundarySpec::new(BoundaryDef::Respond(ResponseKind::Vacuum), rules),
            bounds: opts.bounds,
            complement: ComplementPolicy::Outside,
            flagged,
            report: ImportReport { warnings, dropped: ir.dropped.clone(), checked: None },
        })
    }

    /// Flatten `ir`, run the sampled check, and reject overlapping decks. Gaps and unbounded
    /// cells are warnings in the [`ImportReport`].
    ///
    /// # Errors
    /// Everything [`Self::instantiate`] returns, [`ImportError::Overlap`] when the check finds
    /// overlapping cells, and `Build` when the tracker layer refuses the model.
    pub fn import(ir: &CsgIr, opts: &ImportOptions) -> Result<Self, ImportError> {
        let mut m = Self::instantiate(ir, opts)?;
        let out = m.check(&opts.check)?;
        if !out.overlaps.is_empty() {
            return Err(ImportError::Overlap { findings: out.overlaps });
        }
        m.report.warnings.extend(out.warnings);
        m.report.checked = Some(opts.check);
        Ok(m)
    }

    /// The leaf input over the flat cells.
    fn leaf(&self) -> CsgLeafInput<'_> {
        CsgLeafInput::new(&self.store, &self.cells, self.bounds, &self.complement)
    }

    fn with_hier<R>(&self, f: impl FnOnce(&HierInput<'_>) -> R) -> R {
        self.with_hier_of(self.leaf(), f)
    }

    fn with_hier_of<R>(&self, leaf: CsgLeafInput<'_>, f: impl FnOnce(&HierInput<'_>) -> R) -> R {
        let spec = WorldSpec::new(vec![ComponentKind::Leaf { input: 0 }], ComponentId(0), InstancePolicy::Flatten);
        let leaves = [LeafInput::Csg(leaf)];
        let h = HierInput::new(&spec, &leaves, &self.boundaries);
        f(&h)
    }

    /// The `<outside>` region of the check: the union of the outer half-spaces of the
    /// boundary-flagged surfaces that every flat cell uses on one side only (one single-literal
    /// term per surface). Space beyond a flagged surface is not a gap, whatever its shape (a
    /// vacuum cylinder or sphere leaves the corners of the bounding box outside the model).
    /// `None` when no surface qualifies.
    fn outside_region(&self) -> Option<CsgRegion> {
        // (used on the positive side, used on the negative side)
        let mut used: BTreeMap<EntityId, (bool, bool)> = self.flagged.iter().map(|h| (*h, (false, false))).collect();
        for c in &self.cells {
            for t in c.region.terms() {
                for hs in &t.halfspaces {
                    if let Some(u) = used.get_mut(&hs.surface) {
                        match hs.sign {
                            Sign::Positive => u.0 = true,
                            Sign::Negative => u.1 = true,
                        }
                    }
                }
            }
        }
        // One-sided use only; the outer half-space is the side no cell uses.
        let terms: Vec<CsgCell> = used
            .into_iter()
            .filter(|(_, (pos, neg))| pos != neg)
            .map(|(surface, (pos, _))| CsgCell::new([HalfSpace { surface, sign: if pos { Sign::Negative } else { Sign::Positive } }]))
            .collect();
        (!terms.is_empty()).then(|| CsgRegion::new(terms))
    }

    /// Sample the flat model for overlapping cells and gaps (`sm_rt::check_overlaps`). Findings
    /// are the same for any thread count. An overlap names the deck cells (and lattice elements)
    /// of both cells.
    ///
    /// The sampled leaf is a check-only copy of the flat cells plus one extra `<outside>` cell
    /// (`outside_region`), over the bounds of the tracking leaf; the tracking leaf and
    /// [`Self::content_hash`] do not see it. A sample in `<outside>` is neither a gap nor an
    /// overlap: gaps are taken from this leaf only, and overlaps from the plain tracking leaf
    /// (so `<outside>` pairs cannot crowd real pairs out of sm-rt's capped report).
    ///
    /// # Errors
    /// `Invalid` for out-of-range effort, `Build` when the model cannot be compiled.
    pub fn check(&self, opts: &CheckOptions) -> Result<CheckOutcome, ImportError> {
        opts.validate()?;
        let run = |h: &HierInput<'_>| check_overlaps(h, opts.samples as usize, opts.rays as usize, opts.seed);
        let n = self.cells.len();
        // Overlaps (and everything but gaps) come from the plain tracking leaf: sm-rt reports at
        // most MAX_REPORT distinct pairs, so `<outside>` pairs must never compete for those slots.
        let mut issues = self.with_hier(run);
        if let Some((outside, bounds)) = self.outside_region().zip(self.check_bounds()) {
            // Gaps come from the check leaf with `<outside>`, where space beyond a flagged
            // boundary is covered and so not a gap.
            issues.retain(|i| i.code != "csg.gap");
            let mut cells = self.cells.clone();
            cells.push(CsgCellDef::new(outside, OUTSIDE_CELL));
            let with_outside = self.with_hier_of(CsgLeafInput::new(&self.store, &cells, Some(bounds), &self.complement), run);
            issues.extend(with_outside.into_iter().filter(|i| i.code == "csg.gap"));
        }
        let mut out = CheckOutcome::default();
        let mut build = Vec::new();
        for i in issues {
            let point = i.location.map(|p| [p.x, p.y, p.z]);
            match i.code {
                "csg.overlap" => {
                    let cells = parse_cells(&i.message);
                    if cells.is_some_and(|(a, b)| a >= n || b >= n) {
                        continue;
                    }
                    let found = point.zip(cells).and_then(|(point, (a, b))| Some(OverlapFinding { point, a: self.sources.get(a)?.clone(), b: self.sources.get(b)?.clone() }));
                    match found {
                        Some(f) => out.overlaps.push(f),
                        None => build.push(i),
                    }
                }
                "csg.gap" => out.warnings.push(Warning { code: "import.gap", message: format!("no cell contains the point; the space between cells is not part of the model{}", seed_note(&i.message)), point }),
                "csg.unbounded" => out.warnings.push(Warning { code: "import.unbounded", message: i.message, point }),
                _ => build.push(i),
            }
        }
        if build.is_empty() { Ok(out) } else { Err(ImportError::Build { issues: build }) }
    }

    /// Finite bounds to sample: the given ones, else those of the tracking leaf. `None` when the
    /// leaf is not provably bounded (the plain check then reports `import.unbounded`).
    fn check_bounds(&self) -> Option<Aabb> {
        let b = match self.bounds {
            Some(b) => b,
            None => self.tracker(&BuildOptions::default()).ok()?.0.bounds(),
        };
        let (lo, hi) = ([b.lo.x, b.lo.y, b.lo.z], [b.hi.x, b.hi.y, b.hi.z]);
        (0..3).all(|k| lo[k].is_finite() && hi[k].is_finite() && hi[k] - lo[k] < 1e29).then_some(b)
    }

    /// Build the analytic tracker of the model.
    ///
    /// # Errors
    /// Whatever the analytic builder reports.
    pub fn tracker(&self, opts: &BuildOptions) -> Result<(AnalyticTracker, BuildReport), sm_rt_api::RtError> {
        self.with_hier(|h| AnalyticTrackerBuilder.build_concrete(&BuildInput::Hier(*h), opts))
    }

    /// Hash of the flat model by value: surface definitions, cell regions and names, bounds and
    /// boundaries. Invariant under deck ids and text order (canonical form).
    pub fn content_hash(&self) -> ContentHash {
        let mut h = ContentHasher::new();
        h.write_hash(self.leaf().content_hash());
        h.write_hash(self.boundaries.content_hash());
        h.finish()
    }

    /// The region names, sorted and unique.
    pub fn region_names(&self) -> Vec<String> {
        self.cells.iter().map(|c| c.name.clone()).collect::<BTreeSet<_>>().into_iter().collect()
    }

    /// Number of flat cells.
    pub fn cell_count(&self) -> usize {
        self.cells.len()
    }

    /// Number of surfaces in the flat store.
    pub fn surface_count(&self) -> usize {
        use sm_geom_api::GeomStore;
        self.store.surface_count()
    }

    /// The flat cells in canonical order.
    pub fn cells(&self) -> &[CsgCellDef] {
        &self.cells
    }

    /// Where each flat cell comes from (same order as [`Self::cells`]).
    pub fn sources(&self) -> &[CellSource] {
        &self.sources
    }

    /// Boundary conditions of the flat surfaces.
    pub fn boundaries(&self) -> &BoundarySpec {
        &self.boundaries
    }

    /// What the import found.
    pub fn report(&self) -> &ImportReport {
        &self.report
    }
}

fn seed_note(msg: &str) -> String {
    msg.find("(seed").map(|i| format!(" {}", msg[i..].split(')').next().unwrap_or("").to_owned() + ")")).unwrap_or_default()
}

/// The two cell indices of a `csg.overlap` message: `... cells {a} and {b} both contain ...`.
/// The check reports no structured cell indices, so this couples to its message format; a test
/// pins it, and an unparsable message becomes a `Build` issue rather than a lost finding.
fn parse_cells(msg: &str) -> Option<(usize, usize)> {
    let rest = &msg[msg.find("cells ")? + 6..];
    let (a, rest) = rest.split_once(" and ")?;
    let b = rest.split_whitespace().next()?;
    Some((a.trim().parse().ok()?, b.parse().ok()?))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn overlap_message_format_is_pinned() {
        // The text sm-rt's analytic check emits today.
        let m = "leaf 0: cells 3 and 17 both contain the point (seed 0); cells must be disjoint";
        assert_eq!(parse_cells(m), Some((3, 17)));
        assert_eq!(parse_cells("something else"), None);
    }
}
