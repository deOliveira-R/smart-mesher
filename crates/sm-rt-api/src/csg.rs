//! CSG leaf input: cells that are unions of intersections of signed half-spaces.
//!
//! A [`CsgRegion`] is a union of [`CsgCell`]s (each an intersection of half-spaces over
//! surfaces of a `GeomStore`). A leaf is a list of [`CsgCellDef`]s over one store.
//!
//! # Cells must be disjoint
//! There is no priority and no "first cell wins" rule: a point is never resolved by cell order.
//! Overlap cannot be proven absent for general CSG, so it is reported where it is found: at
//! build time for what a builder can see (`csg.overlap`, `world.overlap`), by a Check-stage
//! sampling op, and at track time as [`crate::LostCause::Overlap`].

use crate::cache::{ContentHash, ContentHasher};
use crate::input::ComplementPolicy;
use sm_core::Aabb;
use sm_geom_api::{CsgCell, GeomStore, Sign};

/// A region of space as a union of intersections of half-spaces (a DNF over signed surfaces).
#[derive(Clone, Debug, PartialEq, Default)]
pub struct CsgRegion {
    terms: Vec<CsgCell>,
}

impl CsgRegion {
    /// The union of `terms`. No terms is the empty region.
    pub fn new(terms: Vec<CsgCell>) -> Self {
        Self { terms }
    }
    /// All of space: one term with no half-space.
    pub fn everything() -> Self {
        Self { terms: vec![CsgCell::default()] }
    }
    /// A single intersection.
    pub fn single(cell: CsgCell) -> Self {
        Self { terms: vec![cell] }
    }
    /// The terms, in input order.
    pub fn terms(&self) -> &[CsgCell] {
        &self.terms
    }
    /// Hash by *value*: term order and half-space order as given, each surface by its
    /// definition in `geom` (not by its handle), so renumbering handles does not move it.
    /// A handle the store does not know hashes as a distinct marker.
    pub fn content_hash(&self, geom: &dyn GeomStore) -> ContentHash {
        let mut h = ContentHasher::new();
        h.write_u64(self.terms.len() as u64);
        for t in &self.terms {
            h.write_u64(t.halfspaces.len() as u64);
            for hs in &t.halfspaces {
                h.write_u64(match hs.sign {
                    Sign::Negative => 0,
                    Sign::Positive => 1,
                });
                match geom.surface_def(hs.surface) {
                    Some(def) => {
                        h.write_u64(1);
                        // `Debug` prints floats with their shortest round-trip text: by value.
                        let text = match def.implicit_def() {
                            Some(i) => format!("{i:?}"),
                            None => format!("{def:?}"),
                        };
                        h.write_bytes(text.as_bytes());
                    }
                    None => h.write_u64(0),
                }
            }
        }
        h.finish()
    }
}

/// One CSG cell of a leaf.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct CsgCellDef {
    /// The cell's region. Cells of a leaf must be pairwise disjoint.
    pub region: CsgRegion,
    /// Region (material) name the caller pairs with its cross sections.
    pub name: String,
    /// Exact volume when the producer knows it; `None` otherwise (never estimated).
    pub volume: Option<f64>,
}

impl CsgCellDef {
    /// A named cell without a known volume.
    pub fn new(region: CsgRegion, name: impl Into<String>) -> Self {
        Self { region, name: name.into(), volume: None }
    }
    /// Attach the exact volume.
    pub fn with_volume(mut self, volume: f64) -> Self {
        self.volume = Some(volume);
        self
    }
}

/// A leaf of CSG cells over one geometry store.
#[derive(Clone, Copy)]
#[non_exhaustive]
pub struct CsgLeafInput<'a> {
    /// The store holding the surfaces the half-spaces name.
    pub geom: &'a dyn GeomStore,
    /// The cells, dense by local cell id.
    pub cells: &'a [CsgCellDef],
    /// Bounds of the leaf (required when some cell is unbounded).
    pub bounds: Option<Aabb>,
    /// What lies outside the cells.
    pub complement: &'a ComplementPolicy,
}

impl<'a> CsgLeafInput<'a> {
    /// Construct (the type is `#[non_exhaustive]`).
    pub fn new(geom: &'a dyn GeomStore, cells: &'a [CsgCellDef], bounds: Option<Aabb>, complement: &'a ComplementPolicy) -> Self {
        Self { geom, cells, bounds, complement }
    }
    /// Hash of the leaf by value: every cell's region hash, name, volume bits and the bounds.
    pub fn content_hash(&self) -> ContentHash {
        let mut h = ContentHasher::new();
        h.write_u64(self.cells.len() as u64);
        for c in self.cells {
            h.write_hash(c.region.content_hash(self.geom));
            h.write_bytes(c.name.as_bytes());
            h.write_u64(u64::from(c.volume.is_some()));
            h.write_f64(c.volume.unwrap_or(0.0));
        }
        match self.bounds {
            Some(b) => {
                h.write_u64(1);
                for p in [b.lo, b.hi] {
                    for v in [p.x, p.y, p.z] {
                        h.write_f64(v);
                    }
                }
            }
            None => h.write_u64(0),
        }
        match self.complement {
            ComplementPolicy::Region(name) => {
                h.write_u64(1);
                h.write_bytes(name.as_bytes());
            }
            _ => h.write_u64(0),
        }
        h.finish()
    }
}
